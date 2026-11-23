/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * A380X FMGC managed guidance: LNAV path, leg sequencing, VNAV descent path and managed speed.
 * See a380x_fmgc_guidance.h for the scope, the FBW provenance and the port models.
 */
#include "a380x_fmgc_guidance.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define D2R (A380X_NAV_PI / 180.0)
#define FT_PER_NM 6076.115
#define G0 9.80665
#define KT_MS 0.514444

/* FMS phases (a380x_fms.h A380XFmsPhase; not included to keep this module FMS-core independent). */
enum { PH_PREFLIGHT = 0, PH_TAKEOFF, PH_CLIMB, PH_CRUISE, PH_DESCENT, PH_APPROACH, PH_GOAROUND, PH_DONE };

static int fin2(double a, double b) { return isfinite(a) && isfinite(b); }
static double n360(double x) { return a380x_geo_norm360(x); }
/* to - from, -180..180 (positive: `to` is to the right of `from`). */
static double adiff(double from, double to) { return a380x_geo_angle_diff(from, to); }
static double clampd(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
static int is_leg(const A380XFplnLeg *l) { return l && !(l->flags & A380X_LEGF_DISCONTINUITY); }
static int is_disco(const A380XFplnLeg *l) { return l && (l->flags & A380X_LEGF_DISCONTINUITY); }
static int has_fix(const A380XFplnLeg *l) { return is_leg(l) && fin2(l->fix.lat, l->fix.lon); }

static double crs_true(const A380XFplnLeg *l)
{
    if (!isfinite(l->course_deg)) return NAN;
    return n360(l->course_true ? l->course_deg : l->course_deg + (isfinite(l->magvar_deg) ? l->magvar_deg : 0.0));
}

/* Local east/north NM around (lat0, lon0); adequate for turn and hold geometry (< ~30 NM). */
static void enu(double lat0, double lon0, double lat, double lon, double *e, double *n)
{
    double dlon = lon - lon0;
    while (dlon > 180.0) dlon -= 360.0;
    while (dlon < -180.0) dlon += 360.0;
    *n = (lat - lat0) * 60.0;
    *e = dlon * 60.0 * cos(lat0 * D2R);
}

static void enu_inv(double lat0, double lon0, double e, double n, double *lat, double *lon)
{
    *lat = lat0 + n / 60.0;
    *lon = lon0 + e / (60.0 * cos(lat0 * D2R));
}

double a380x_fmgc_turn_radius_nm(double gs_kt, double bank_deg)
{
    double v = (gs_kt < 100.0 ? 100.0 : gs_kt) * KT_MS;
    return v * v / (G0 * tan(bank_deg * D2R)) / 1852.0;
}

double a380x_fmgc_anticipation_nm(double gs_kt, double track_change_deg)
{
    double d = fabs(track_change_deg);
    if (d < 1.0) return 0.0;
    if (d > 135.0) d = 135.0;
    return a380x_fmgc_turn_radius_nm(gs_kt, A380X_FMGC_DESIGN_BANK_DEG) * tan(d * D2R / 2.0);
}

/* Nominal idle descent gradient of the port model (still air, managed descent speed): 3.0 deg.
 * Not an FBW performance computation; the profile scales it (wind, learned idle factor). */
double a380x_fmgc_idle_gradient_ft_nm(double alt_ft, double speed_limit_alt_ft)
{
    (void)alt_ft; (void)speed_limit_alt_ft;
    return tan(3.0 * D2R) * FT_PER_NM;
}

/* ============================================================================================ */
/* Managed profile (moved from a380x_fms_adapter.inc; FAI.ts:1459-1603, spec 07 5.7 / 6.3)       */
/* ============================================================================================ */

static double nanv(void) { return (double)NAN; }

/* Upper altitude of a CLB constraint ('@', '-', 'B' alt1), NAN if none. */
static double climb_cstr_alt(const A380XFplnLeg *l)
{
    A380XFplnAltCstr a = a380x_fpln_eff_alt(l);
    if (l->cstr_type != A380X_CSTR_CLB) return nanv();
    if ((a.desc == '@' || a.desc == '-' || a.desc == 'B') && isfinite(a.alt1_ft)) return a.alt1_ft;
    return nanv();
}

/* Lower altitude of a DES constraint ('@', '+' alt1, 'B' alt2), NAN if none. */
static double descent_cstr_alt(const A380XFplnLeg *l)
{
    A380XFplnAltCstr a = a380x_fpln_eff_alt(l);
    if (l->cstr_type != A380X_CSTR_DES) return nanv();
    if ((a.desc == '@' || a.desc == '+') && isfinite(a.alt1_ft)) return a.alt1_ft;
    if (a.desc == 'B' && isfinite(a.alt2_ft)) return a.alt2_ft;
    return nanv();
}

static double cstr_speed(const A380XFplnLeg *l, A380XFplnCstrType type)
{
    A380XFplnSpdCstr s = a380x_fpln_eff_spd(l);
    if (l->cstr_type != type || !s.desc || !isfinite(s.speed) || s.speed < 2.0f) return nanv();
    return s.speed;
}

double a380x_fmgc_lowest_climb_constraint(const A380XFlightPlans *fp, int plan)
{
    const A380XFplnPlan *pl = fp && plan >= 0 && plan < A380X_PLAN_COUNT ? a380x_fpln_plan(fp, plan) : NULL;
    double lowest = nanv();
    int i, end;
    if (!pl) return lowest;
    end = pl->pri.first_missed_appr_index < pl->pri.count ? pl->pri.first_missed_appr_index : pl->pri.count;
    for (i = 0; i < end; ++i) {
        double a = climb_cstr_alt(&pl->pri.el[i]);
        if (isfinite(a) && (!isfinite(lowest) || a < lowest)) lowest = a;
    }
    return lowest;
}

typedef struct { double climb_speed, climb_alt, descent_speed, prev_descent_speed, descent_alt; } Profile;

static int managed_profile(const A380XFlightPlans *fp, Profile *out)
{
    const A380XFplnPlan *pl = fp ? a380x_fpln_plan(fp, A380X_PLAN_ACTIVE) : NULL;
    int i, end, active;
    double run;
    out->climb_speed = out->climb_alt = out->descent_speed = out->prev_descent_speed = out->descent_alt = nanv();
    if (!pl) return 0;
    active = pl->active_leg_index;
    end = pl->pri.first_missed_appr_index < pl->pri.count ? pl->pri.first_missed_appr_index : pl->pri.count;
    if (active < 0 || active >= end) return 0;
    /* Backward pass: running minimum of CLB speed / altitude constraints from the end down to the
     * active leg; running maximum of DES altitude constraints. */
    for (i = end - 1; i >= active; --i) {
        const A380XFplnLeg *l = &pl->pri.el[i];
        double s = cstr_speed(l, A380X_CSTR_CLB), a = climb_cstr_alt(l), d = descent_cstr_alt(l);
        if (isfinite(s) && (!isfinite(out->climb_speed) || s < out->climb_speed)) out->climb_speed = s;
        if (isfinite(a) && (!isfinite(out->climb_alt) || a < out->climb_alt)) out->climb_alt = a;
        if (isfinite(d) && (!isfinite(out->descent_alt) || d > out->descent_alt)) out->descent_alt = d;
    }
    if (isfinite(out->descent_alt) && isfinite(pl->pri.dest_elev_ft) && out->descent_alt < pl->pri.dest_elev_ft)
        out->descent_alt = pl->pri.dest_elev_ft;
    /* Forward pass: running minimum of DES speed constraints up to the active leg. */
    run = nanv();
    for (i = 0; i <= active; ++i) {
        double s = cstr_speed(&pl->pri.el[i], A380X_CSTR_DES);
        if (i == active) out->prev_descent_speed = run;
        if (isfinite(s) && (!isfinite(run) || s < run)) run = s;
    }
    out->descent_speed = run;
    return 1;
}

double a380x_fmgc_speed_constraint(const A380XFlightPlans *fp, const A380XFplnAircraft *ac, int phase)
{
    Profile pr;
    if (!managed_profile(fp, &pr)) return nanv();
    if (phase == PH_CLIMB || phase == PH_GOAROUND) return pr.climb_speed;
    if (phase == PH_DESCENT) {
        float dtg = ac ? a380x_fpln_dist_to_active(fp, A380X_PLAN_ACTIVE, ac) : NAN;
        double prev = pr.prev_descent_speed, decel;
        if (!isfinite(pr.descent_speed)) return nanv();
        if (!isfinite(prev)) return pr.descent_speed;
        decel = (prev - pr.descent_speed) * 0.15;
        decel = decel < 3.0 ? 3.0 : decel > 20.0 ? 20.0 : decel;
        return isfinite(dtg) && dtg < decel ? pr.descent_speed : prev;
    }
    return nanv();
}

int a380x_fmgc_constraint_altitudes(const A380XFlightPlans *fp, double *climb_alt_ft, double *descent_alt_ft)
{
    Profile pr;
    if (!managed_profile(fp, &pr)) return 0;
    *climb_alt_ft = pr.climb_alt;
    *descent_alt_ft = pr.descent_alt;
    return 1;
}

/* ============================================================================================ */
/* Lateral guidance elements                                                                    */
/* ============================================================================================ */

typedef struct {
    A380XLnavKind kind;
    /* LINE: great circle a -> b; ARC: centre c, radius, direction, start bearing, sweep */
    double alat, alon, blat, blon;
    double clat, clon, radius, start_brg, sweep;
    int dir;
    double course;              /* TRACK / HEADING: true course / heading; DIRECT: fix lat/lon in b */
    /* results */
    double xtk_right, desired, along, length, progress, dtg;
} Elem;

static void elem_line(Elem *e, double alat, double alon, double blat, double blon)
{
    memset(e, 0, sizeof *e);
    e->kind = A380X_LNAV_LINE;
    e->alat = alat; e->alon = alon; e->blat = blat; e->blon = blon;
}

static void elem_arc(Elem *e, double clat, double clon, double radius, int dir, double start_brg, double sweep)
{
    memset(e, 0, sizeof *e);
    e->kind = A380X_LNAV_ARC;
    e->clat = clat; e->clon = clon; e->radius = radius; e->dir = dir;
    e->start_brg = start_brg; e->sweep = sweep;
}

/* Evaluates xtk / desired track / along-track progress of an element at the aircraft position. */
static void elem_eval(Elem *e, const A380XFmgcInputs *in)
{
    switch (e->kind) {
    case A380X_LNAV_LINE: {
        double along, cross, len = a380x_geo_distance_nm(e->alat, e->alon, e->blat, e->blon);
        a380x_geo_along_cross(e->alat, e->alon, e->blat, e->blon, in->lat_deg, in->lon_deg, &along, &cross);
        e->along = along; e->length = len; e->xtk_right = cross; e->dtg = len - along;
        if (len - along > 1.0) {
            double qlat, qlon;
            a380x_geo_destination(e->alat, e->alon, a380x_geo_bearing_deg(e->alat, e->alon, e->blat, e->blon),
                                  along > 0 ? along : 0.0, &qlat, &qlon);
            e->desired = a380x_geo_bearing_deg(qlat, qlon, e->blat, e->blon);
        } else {
            e->desired = n360(a380x_geo_bearing_deg(e->blat, e->blon, e->alat, e->alon) + 180.0);
        }
        break;
    }
    case A380X_LNAV_ARC: {
        double x, y, rho, brg, prog;
        enu(e->clat, e->clon, in->lat_deg, in->lon_deg, &x, &y);
        rho = hypot(x, y);
        brg = n360(atan2(x, y) / D2R);
        e->desired = n360(brg + 90.0 * e->dir);
        e->xtk_right = -e->dir * (rho - e->radius);
        prog = n360(e->dir * (brg - e->start_brg));
        if (prog > 300.0) prog -= 360.0;   /* slightly before the start */
        e->progress = prog;
        e->dtg = (e->sweep - prog) * D2R * e->radius;
        break;
    }
    case A380X_LNAV_DIRECT:
        e->desired = a380x_geo_bearing_deg(in->lat_deg, in->lon_deg, e->blat, e->blon);
        e->dtg = a380x_geo_distance_nm(in->lat_deg, in->lon_deg, e->blat, e->blon);
        e->xtk_right = 0.0;
        break;
    case A380X_LNAV_TRACK: case A380X_LNAV_HEADING:
        e->desired = e->course;
        e->xtk_right = 0.0;
        break;
    default:
        break;
    }
}

/* ============================================================================================ */
/* Leg geometry                                                                                 */
/* ============================================================================================ */

static double hold_leg_length(const A380XFplnLeg *l, const A380XFmgcInputs *in)
{
    double t;
    if (isfinite(l->length_nm) && l->length_nm > 0.5) return l->length_nm;
    t = isfinite(l->length_min) && l->length_min > 0.2 ? l->length_min : (in->alt_ft <= 14000.0 ? 1.0 : 1.5);
    return fmax(3.0, (in->tas_kt > 100.0 ? in->tas_kt : fmax(in->gs_kt, 180.0)) * t / 60.0);
}

/* Initial true course of a leg's path from its start (fly-by anticipation of the previous leg). */
static double leg_initial_course(const A380XFplnLeg *l, const A380XFplnLeg *prev)
{
    switch (l->type) {
    case A380X_LEG_TF: case A380X_LEG_PI:
        if (fin2(l->start_lat, l->start_lon) && has_fix(l))
            return a380x_geo_bearing_deg(l->start_lat, l->start_lon, l->fix.lat, l->fix.lon);
        return NAN;
    case A380X_LEG_CF: case A380X_LEG_FA: case A380X_LEG_FC: case A380X_LEG_FD: case A380X_LEG_FM:
        return crs_true(l);
    case A380X_LEG_DF:
        if (has_fix(prev) && has_fix(l)) return a380x_geo_bearing_deg(prev->fix.lat, prev->fix.lon, l->fix.lat, l->fix.lon);
        return NAN;
    default:
        return NAN;
    }
}

/* Final true course of a line leg at its termination, NAN if the leg does not end on a line. */
static double leg_final_course(const A380XFplnLeg *l)
{
    switch (l->type) {
    case A380X_LEG_TF: case A380X_LEG_DF: case A380X_LEG_PI: case A380X_LEG_IF:
        if (fin2(l->start_lat, l->start_lon) && has_fix(l)
            && a380x_geo_distance_nm(l->start_lat, l->start_lon, l->fix.lat, l->fix.lon) > 0.1)
            return n360(a380x_geo_bearing_deg(l->fix.lat, l->fix.lon, l->start_lat, l->start_lon) + 180.0);
        return NAN;
    case A380X_LEG_CF:
        return crs_true(l);
    default:
        return NAN;
    }
}

/* Final course of the active leg as flown: a DF / IF (and TF without a start) is flown direct from
 * the aircraft (or along its frozen line), not from the previous termination. */
static double active_final_course(const A380XFmgc *g, const A380XFplnLeg *l, const A380XFmgcInputs *in)
{
    int direct = l->type == A380X_LEG_DF || l->type == A380X_LEG_IF
        || ((l->type == A380X_LEG_TF || l->type == A380X_LEG_PI) && !fin2(l->start_lat, l->start_lon));
    if (direct && has_fix(l)) {
        double slat = g->df_frozen ? g->df_lat : in->lat_deg, slon = g->df_frozen ? g->df_lon : in->lon_deg;
        if (a380x_geo_distance_nm(slat, slon, l->fix.lat, l->fix.lon) > 0.1)
            return n360(a380x_geo_bearing_deg(l->fix.lat, l->fix.lon, slat, slon) + 180.0);
        return in->true_track_deg;
    }
    return leg_final_course(l);
}

/* Track change and anticipation at the end of the active leg i into leg i+1 (0 = sequence at the fix). */
static double leg_anticipation(const A380XFmgc *g, const A380XFplnRoute *r, int i, const A380XFmgcInputs *in, double *change)
{
    const A380XFplnLeg *l = &r->el[i], *nx = i + 1 < r->count ? &r->el[i + 1] : NULL;
    double cin, cout;
    *change = 0.0;
    if (!is_leg(nx) || (l->flags & A380X_LEGF_OVERFLY)) return 0.0;
    if (a380x_leg_is_hx(nx->type)) return 0.0;              /* hold entry over the fix */
    if (r->destination_leg_index >= 0 && i >= r->destination_leg_index) return 0.0;
    cin = active_final_course(g, l, in);
    cout = leg_initial_course(nx, l);
    if (!fin2(cin, cout)) return 0.0;
    *change = adiff(cin, cout);
    return a380x_fmgc_anticipation_nm(in->gs_kt, *change);
}

/* DF / IF (and TF without a start): pursuit of the fix, or the frozen line once established. */
static int direct_element(const A380XFmgc *g, const A380XFplnLeg *l, Elem *e)
{
    if (!has_fix(l)) return 0;
    if (g->df_frozen) { elem_line(e, g->df_lat, g->df_lon, l->fix.lat, l->fix.lon); return 1; }
    memset(e, 0, sizeof *e);
    e->kind = A380X_LNAV_DIRECT; e->blat = l->fix.lat; e->blon = l->fix.lon;
    return 1;
}

/* Builds the guidance element of the active leg (state-dependent for DF and holds). */
static int leg_element(A380XFmgc *g, const A380XFplnRoute *r, int a, const A380XFmgcInputs *in, Elem *e)
{
    const A380XFplnLeg *l = &r->el[a];
    double crs = crs_true(l);
    (void)in;
    memset(e, 0, sizeof *e);
    switch (l->type) {
    case A380X_LEG_TF: case A380X_LEG_PI:
        if (!has_fix(l)) return 0;
        if (fin2(l->start_lat, l->start_lon) && a380x_geo_distance_nm(l->start_lat, l->start_lon, l->fix.lat, l->fix.lon) > 0.05) {
            elem_line(e, l->start_lat, l->start_lon, l->fix.lat, l->fix.lon);
            return 1;
        }
        return direct_element(g, l, e);   /* no start point: direct to the fix */
    case A380X_LEG_IF: case A380X_LEG_DF:
        return direct_element(g, l, e);
    case A380X_LEG_CF:
        if (!has_fix(l)) return 0;
        if (isfinite(crs)) {
            double alat, alon, back = fmax(30.0, isfinite(l->leg_dist_nm) ? l->leg_dist_nm + 5.0 : 30.0);
            a380x_geo_destination(l->fix.lat, l->fix.lon, n360(crs + 180.0), back, &alat, &alon);
            elem_line(e, alat, alon, l->fix.lat, l->fix.lon);
            return 1;
        }
        if (fin2(l->start_lat, l->start_lon)) { elem_line(e, l->start_lat, l->start_lon, l->fix.lat, l->fix.lon); return 1; }
        return 0;
    case A380X_LEG_RF: case A380X_LEG_AF: {
        const A380XNavRef *c = l->type == A380X_LEG_RF ? &l->arc_center : &l->navaid;
        double radius, sb, eb, sweep;
        int dir = l->turn == A380X_TURN_LEFT ? -1 : 1;
        if (!has_fix(l) || !fin2(c->lat, c->lon)) {
            if (has_fix(l) && fin2(l->start_lat, l->start_lon)) { elem_line(e, l->start_lat, l->start_lon, l->fix.lat, l->fix.lon); return 1; }
            return 0;
        }
        radius = l->type == A380X_LEG_RF ? (isfinite(l->arc_radius_nm) ? l->arc_radius_nm : a380x_geo_distance_nm(c->lat, c->lon, l->fix.lat, l->fix.lon))
                                         : (isfinite(l->rho_nm) ? l->rho_nm : a380x_geo_distance_nm(c->lat, c->lon, l->fix.lat, l->fix.lon));
        eb = a380x_geo_bearing_deg(c->lat, c->lon, l->fix.lat, l->fix.lon);
        sb = fin2(l->start_lat, l->start_lon) ? a380x_geo_bearing_deg(c->lat, c->lon, l->start_lat, l->start_lon)
                                              : (l->type == A380X_LEG_AF && isfinite(l->theta_deg) ? n360(l->theta_deg + l->magvar_deg) : eb);
        sweep = n360(dir * (eb - sb));
        elem_arc(e, c->lat, c->lon, radius, dir, sb, sweep);
        return 1;
    }
    case A380X_LEG_CA: case A380X_LEG_CD: case A380X_LEG_CI: case A380X_LEG_CR:
        if (!isfinite(crs)) return 0;
        e->kind = A380X_LNAV_TRACK; e->course = crs;
        return 1;
    case A380X_LEG_VA: case A380X_LEG_VD: case A380X_LEG_VI: case A380X_LEG_VM: case A380X_LEG_VR:
        if (!isfinite(crs)) return 0;
        e->kind = A380X_LNAV_HEADING; e->course = crs;
        return 1;
    case A380X_LEG_FA: case A380X_LEG_FC: case A380X_LEG_FD: case A380X_LEG_FM: {
        double blat, blon, len = isfinite(l->leg_dist_nm) && l->leg_dist_nm > 1.0 ? l->leg_dist_nm + 30.0 : 60.0;
        if (!has_fix(l) || !isfinite(crs)) return 0;
        a380x_geo_destination(l->fix.lat, l->fix.lon, crs, len, &blat, &blon);
        elem_line(e, l->fix.lat, l->fix.lon, blat, blon);
        return 1;
    }
    case A380X_LEG_HA: case A380X_LEG_HF: case A380X_LEG_HM: {
        double c = crs, R, L, ue, un, ne, nn, s;
        if (!has_fix(l)) return 0;
        if (!isfinite(c)) c = 0.0;
        if (g->hold_phase == 4 || g->hold_phase == 9 || !g->hold_entered) {
            /* arrival, immediate exit, end of a parallel entry: direct to the fix */
            e->kind = A380X_LNAV_DIRECT; e->blat = l->fix.lat; e->blon = l->fix.lon;
            return 1;
        }
        R = g->hold_radius_nm; L = g->hold_leg_nm;
        ue = sin(c * D2R); un = cos(c * D2R);          /* inbound direction */
        ne = cos(c * D2R); nn = -sin(c * D2R);         /* right normal */
        s = l->turn == A380X_TURN_LEFT ? -1.0 : 1.0;
        switch (g->hold_phase) {
        case 5: case 6: {   /* teardrop: outbound offset alpha towards the holding side, then the turn
                             * of 180 + alpha rolling out on the inbound course */
            double trk = n360(c + 180.0 - s * g->hold_teardrop_deg), plat, plon, clat, clon;
            a380x_geo_destination(l->fix.lat, l->fix.lon, trk, L, &plat, &plon);
            if (g->hold_phase == 5) { elem_line(e, l->fix.lat, l->fix.lon, plat, plon); return 1; }
            a380x_geo_destination(plat, plon, n360(trk + 90.0 * s), R, &clat, &clon);
            elem_arc(e, clat, clon, R, (int)s, n360(trk - 90.0 * s), 180.0 + g->hold_teardrop_deg);
            return 1;
        }
        case 7: case 8: {   /* parallel: outbound on the non-holding side along the inbound course line,
                             * a 225 deg turn towards the holding side, then direct to the fix (9) */
            double plat, plon, clat, clon;
            a380x_geo_destination(l->fix.lat, l->fix.lon, n360(c + 180.0), L, &plat, &plon);
            if (g->hold_phase == 7) { elem_line(e, l->fix.lat, l->fix.lon, plat, plon); return 1; }
            a380x_geo_destination(plat, plon, n360(c + 90.0 * s), R, &clat, &clon);
            elem_arc(e, clat, clon, R, (int)-s, n360(c - 90.0 * s), 225.0);
            return 1;
        }
        case 0: {   /* inbound I0 -> F */
            double ilat, ilon;
            enu_inv(l->fix.lat, l->fix.lon, -L * ue, -L * un, &ilat, &ilon);
            elem_line(e, ilat, ilon, l->fix.lat, l->fix.lon);
            return 1;
        }
        case 1: {   /* turn at the fix around C1 */
            double clat, clon;
            enu_inv(l->fix.lat, l->fix.lon, s * R * ne, s * R * nn, &clat, &clon);
            elem_arc(e, clat, clon, R, (int)s, n360(c - 90.0 * s), 180.0);
            return 1;
        }
        case 2: {   /* outbound O1 -> O2 */
            double o1lat, o1lon, o2lat, o2lon;
            enu_inv(l->fix.lat, l->fix.lon, 2 * s * R * ne, 2 * s * R * nn, &o1lat, &o1lon);
            enu_inv(l->fix.lat, l->fix.lon, 2 * s * R * ne - L * ue, 2 * s * R * nn - L * un, &o2lat, &o2lon);
            elem_line(e, o1lat, o1lon, o2lat, o2lon);
            return 1;
        }
        default: {  /* turn back around C2 */
            double clat, clon;
            enu_inv(l->fix.lat, l->fix.lon, s * R * ne - L * ue, s * R * nn - L * un, &clat, &clon);
            elem_arc(e, clat, clon, R, (int)s, n360(c + 90.0 * s), 180.0);
            return 1;
        }
        }
    }
    default:
        return 0;
    }
}

/* Element of the fly-by transition arc into the active leg. */
static void transition_element(const A380XFmgc *g, Elem *e)
{
    elem_arc(e, g->arc_clat, g->arc_clon, g->arc_radius_nm, g->arc_dir, g->arc_start_brg, g->arc_sweep_deg);
}

/* ============================================================================================ */
/* Sequencing                                                                                   */
/* ============================================================================================ */

static const A380XNavRef *dme_ref(const A380XFplnLeg *l)
{
    return l->navaid.ident[0] && fin2(l->navaid.lat, l->navaid.lon) ? &l->navaid : &l->fix;
}

A380XHoldEntry a380x_fmgc_hold_entry(double inbound_true, double arrival_track_true, int right_turns)
{
    /* d: arrival track relative to the inbound course, mirrored for left-hand patterns */
    double d = adiff(inbound_true, arrival_track_true) * (right_turns ? 1.0 : -1.0);
    if (d >= -115.0 && d <= 75.0) return A380X_HOLD_ENTRY_DIRECT;      /* -110..+70, +-5 towards direct */
    if (d > 75.0 && d <= 145.0) return A380X_HOLD_ENTRY_TEARDROP;      /* +70..+140, +5 towards teardrop */
    return A380X_HOLD_ENTRY_PARALLEL;
}

/* ISA CAS of a Mach number at a pressure altitude (holding speed above FL340). */
static double isa_mach_to_cas(double mach, double alt_ft)
{
    double h = alt_ft * 0.3048, t = h < 11000.0 ? 288.15 - 0.0065 * h : 216.65;
    double p = h < 11000.0 ? 101325.0 * pow(t / 288.15, 5.25588) : 22632.06 * exp(-(h - 11000.0) / 6341.62);
    double qc = p * (pow(1.0 + 0.2 * mach * mach, 3.5) - 1.0);
    return 661.4788 * sqrt(5.0 * (pow(qc / 101325.0 + 1.0, 2.0 / 7.0) - 1.0));
}

double a380x_fmgc_holding_speed(const A380XFmgcInputs *in, double cstr_speed_kt, double cstr_alt_ft)
{
    double alt = fmax(in->fcu_alt_ft, isfinite(cstr_alt_ft) ? cstr_alt_ft : -1e9), k;
    k = in->green_dot_kt > 80.0 ? in->green_dot_kt : 220.0;
    if (in->engine_out) k += 25.0;
    if (in->phase == PH_APPROACH) {   /* getAppManagedSpeed: GD / S / F / VAPP by the flap lever */
        double conf = in->flap_lever == 0 ? in->green_dot_kt : in->flap_lever == 1 ? in->s_speed_kt
                    : in->flap_lever == 2 ? in->f_speed_kt
                    : in->flap_lever == 3 ? (in->landing_conf3 ? in->vapp_kt : in->f_speed_kt) : in->vapp_kt;
        if (conf > 80.0) k = conf;
    }
    if (isfinite(cstr_speed_kt) && cstr_speed_kt > 100.0) k = fmin(k, cstr_speed_kt);
    if (alt < 14000.0) k = fmin(230.0, k);          /* ICAO maximum holding speeds */
    else if (alt < 20000.0) k = fmin(240.0, k);
    else if (alt < 34000.0) k = fmin(265.0, k);
    else k = fmin(k, isa_mach_to_cas(0.83, alt));
    if (in->phase <= PH_CRUISE && in->clb_limit_kt > 0 && alt <= in->clb_limit_alt_ft) k = fmin(k, in->clb_limit_kt);
    else if (in->phase > PH_CRUISE && in->phase < PH_GOAROUND && in->des_limit_kt > 0 && alt <= in->des_limit_alt_ft)
        k = fmin(k, in->des_limit_kt);
    if (in->vls_kt > 80.0) k = fmax(k, in->vls_kt);
    return ceil(k);
}

/* Enters the hold at the fix with the entry for the arrival track. */
static void hold_begin_entry(A380XFmgc *g, const A380XFplnLeg *l, const A380XFmgcInputs *in)
{
    double c = crs_true(l), R = g->hold_radius_nm, L = g->hold_leg_nm;
    if (!isfinite(c)) c = 0.0;
    g->hold_entered = 1;
    g->hold_entry = a380x_fmgc_hold_entry(c, in->true_track_deg, l->turn != A380X_TURN_LEFT);
    /* teardrop offset so that the 180 + alpha turn rolls out on the inbound course:
     * tan(alpha / 2) = R / L (clamped 20..60 deg) */
    g->hold_teardrop_deg = clampd(2.0 * atan(R / fmax(L, 0.5)) / D2R, 20.0, 60.0);
    g->hold_phase = g->hold_entry == A380X_HOLD_ENTRY_TEARDROP ? 5 : g->hold_entry == A380X_HOLD_ENTRY_PARALLEL ? 7 : 1;
}

/* Leg-specific state at activation (crossing directions, holds, DF). */
static void leg_activate(A380XFmgc *g, const A380XFplnRoute *r, int a, const A380XFmgcInputs *in)
{
    const A380XFplnLeg *l = &r->el[a];
    g->leg = a;
    g->df_frozen = 0;
    g->cross_sign = 0;
    g->hold_phase = 0; g->hold_entered = 0; g->hold_circuits = 0; g->hold_entry = A380X_HOLD_ENTRY_NONE;
    if (!in->position_valid) return;
    switch (l->type) {
    case A380X_LEG_CD: case A380X_LEG_FD: case A380X_LEG_VD: {
        const A380XNavRef *c = dme_ref(l);
        if (fin2(c->lat, c->lon) && isfinite(l->length_nm))
            g->cross_sign = a380x_geo_distance_nm(c->lat, c->lon, in->lat_deg, in->lon_deg) < l->length_nm ? 1 : -1;
        break;
    }
    case A380X_LEG_CR: case A380X_LEG_VR:
        if (fin2(l->navaid.lat, l->navaid.lon) && isfinite(l->theta_deg)) {
            double rad = n360(l->theta_deg + l->magvar_deg);
            double b = a380x_geo_bearing_deg(l->navaid.lat, l->navaid.lon, in->lat_deg, in->lon_deg);
            g->cross_sign = adiff(rad, b) >= 0 ? 1 : -1;
        }
        break;
    case A380X_LEG_HA: case A380X_LEG_HF: case A380X_LEG_HM:
        g->hold_radius_nm = a380x_fmgc_turn_radius_nm(in->tas_kt > 100 ? in->tas_kt : in->gs_kt, A380X_FMGC_DESIGN_BANK_DEG);
        g->hold_leg_nm = hold_leg_length(l, in);
        if (has_fix(l) && a380x_geo_distance_nm(in->lat_deg, in->lon_deg, l->fix.lat, l->fix.lon) < 2.0)
            hold_begin_entry(g, l, in);   /* sequenced over the fix */
        break;
    default:
        break;
    }
}

/* 1 when the hold leg is complete; advances the racetrack phases. */
static int hold_step(A380XFmgc *g, const A380XFplnLeg *l, const A380XFmgcInputs *in, Elem *e)
{
    if (!g->hold_entered) {
        if (e->kind == A380X_LNAV_DIRECT
            && (e->dtg < fmax(0.5, a380x_fmgc_turn_radius_nm(in->gs_kt, A380X_FMGC_DESIGN_BANK_DEG) * 0.15)
                || (e->dtg < 1.5 && fabs(adiff(in->true_track_deg, e->desired)) > 90.0)))
            hold_begin_entry(g, l, in);
        return 0;
    }
    if ((l->flags & A380X_LEGF_HOLD_IMM_EXIT) && g->hold_phase != 0 && g->hold_phase != 4) g->hold_phase = 4;
    switch (g->hold_phase) {
    case 5: if (e->along >= e->length) g->hold_phase = 6; break;          /* teardrop leg flown */
    case 6: if (e->progress >= e->sweep - 1.0) g->hold_phase = 0; break;  /* rolled out inbound */
    case 7: if (e->along >= e->length) g->hold_phase = 8; break;          /* parallel leg flown */
    case 8: if (e->progress >= e->sweep - 1.0) g->hold_phase = 9; break;
    case 9:                                                               /* back over the fix */
        if (e->dtg < 0.6 || (e->dtg < 1.5 && fabs(adiff(in->true_track_deg, e->desired)) > 90.0)) {
            g->hold_circuits++;
            if (l->type == A380X_LEG_HF) return 1;
            if (l->type == A380X_LEG_HA && isfinite(l->alt_db.alt1_ft) && in->alt_ft >= l->alt_db.alt1_ft - 50.0) return 1;
            g->hold_phase = 1;
        }
        break;
    case 1: if (e->progress >= 178.0) g->hold_phase = 2; break;
    case 2: if (e->along >= e->length) g->hold_phase = 3; break;
    case 3: if (e->progress >= 178.0) g->hold_phase = 0; break;
    case 4: return e->dtg < 0.6;
    default:   /* inbound: passing the fix completes a circuit */
        if (e->along >= e->length) {
            g->hold_circuits++;
            if (l->type == A380X_LEG_HF) return 1;
            if (l->type == A380X_LEG_HA && isfinite(l->alt_db.alt1_ft) && in->alt_ft >= l->alt_db.alt1_ft - 50.0) return 1;
            if (l->flags & A380X_LEGF_HOLD_IMM_EXIT) return 1;
            g->hold_phase = 1;
        }
        break;
    }
    return 0;
}

/* Next-leg line used by CI/VI intercept sequencing (NULL element kind NONE when not a line). */
static int next_line(A380XFmgc *g, const A380XFplnRoute *r, int a, const A380XFmgcInputs *in, Elem *e)
{
    A380XFmgc tmp = *g;
    if (a + 1 >= r->count || !is_leg(&r->el[a + 1])) return 0;
    tmp.df_frozen = 0; tmp.hold_entered = 0; tmp.hold_phase = 0;
    if (!leg_element(&tmp, r, a + 1, in, e)) return 0;
    if (e->kind != A380X_LNAV_LINE && e->kind != A380X_LNAV_ARC) return 0;
    elem_eval(e, in);
    return 1;
}

/* 1 when the active leg (evaluated element e) is complete. *antic_used = anticipation applied. */
static int leg_complete(A380XFmgc *g, const A380XFplnRoute *r, int a, const A380XFmgcInputs *in, Elem *e,
                        int allow_anticipation, double *antic_out)
{
    const A380XFplnLeg *l = &r->el[a];
    double change, antic = 0.0;
    *antic_out = 0.0;
    switch (l->type) {
    case A380X_LEG_CA: case A380X_LEG_FA: case A380X_LEG_VA:
        return isfinite(l->alt_db.alt1_ft) && in->alt_ft >= l->alt_db.alt1_ft - 50.0;
    case A380X_LEG_FM: case A380X_LEG_VM:
        return 0;
    case A380X_LEG_FC:
        return isfinite(l->length_nm) && e->kind == A380X_LNAV_LINE && e->along >= l->length_nm;
    case A380X_LEG_CD: case A380X_LEG_FD: case A380X_LEG_VD: {
        const A380XNavRef *c = dme_ref(l);
        double d;
        if (!fin2(c->lat, c->lon) || !isfinite(l->length_nm) || !g->cross_sign) return 0;
        d = a380x_geo_distance_nm(c->lat, c->lon, in->lat_deg, in->lon_deg);
        return g->cross_sign > 0 ? d >= l->length_nm : d <= l->length_nm;
    }
    case A380X_LEG_CR: case A380X_LEG_VR: {
        double rad, b, d;
        if (!fin2(l->navaid.lat, l->navaid.lon) || !isfinite(l->theta_deg) || !g->cross_sign) return 0;
        rad = n360(l->theta_deg + l->magvar_deg);
        b = a380x_geo_bearing_deg(l->navaid.lat, l->navaid.lon, in->lat_deg, in->lon_deg);
        d = adiff(rad, b);
        return (d >= 0 ? 1 : -1) != g->cross_sign && fabs(d) < 90.0;
    }
    case A380X_LEG_CI: case A380X_LEG_VI: {
        Elem nx;
        double dlt, cap, R;
        if (!next_line(g, r, a, in, &nx)) return 0;
        dlt = fabs(adiff(in->true_track_deg, nx.desired));
        if (fabs(nx.xtk_right) < 0.1) return 1;
        R = a380x_fmgc_turn_radius_nm(in->gs_kt, A380X_FMGC_DESIGN_BANK_DEG);
        cap = R * (1.0 - cos(fmin(dlt, 150.0) * D2R)) + 0.1;
        return nx.xtk_right * adiff(nx.desired, in->true_track_deg) < 0 && fabs(nx.xtk_right) <= cap;
    }
    case A380X_LEG_HA: case A380X_LEG_HF: case A380X_LEG_HM:
        return hold_step(g, l, in, e);
    case A380X_LEG_RF: case A380X_LEG_AF:
        if (e->kind == A380X_LNAV_ARC) return e->progress >= e->sweep - 0.2;
        break;
    default:
        break;
    }
    /* XF legs ending at a fix */
    if (allow_anticipation) antic = leg_anticipation(g, r, a, in, &change);
    *antic_out = antic;
    if (e->kind == A380X_LNAV_LINE) {
        if (!allow_anticipation && fabs(e->xtk_right) > 7.0) return 0;
        return e->dtg <= antic + 0.05;
    }
    if (e->kind == A380X_LNAV_DIRECT) {
        double rel = fabs(adiff(in->true_track_deg, e->desired));
        return e->dtg <= antic + 0.05 || (e->dtg < 1.5 && rel > 90.0);
    }
    return 0;
}

/* Fly-by transition arc from the previous leg's line into the new leg. */
static void setup_transition(A380XFmgc *g, const A380XFplnRoute *r, int prev, const A380XFmgcInputs *in)
{
    const A380XFplnLeg *l = &r->el[prev], *nx = &r->el[prev + 1];
    double change, antic = leg_anticipation(g, r, prev, in, &change), cin, R, t1lat, t1lon;
    g->arc_active = 0;
    g->entry_antic_nm = g->entry_arc_nm = 0.0;
    if (antic <= 0.0 || !has_fix(l)) return;
    if (!(nx->type == A380X_LEG_TF || nx->type == A380X_LEG_CF || nx->type == A380X_LEG_FA || nx->type == A380X_LEG_FC
          || nx->type == A380X_LEG_FD || nx->type == A380X_LEG_FM || nx->type == A380X_LEG_PI)) return;
    cin = active_final_course(g, l, in);
    if (!isfinite(cin)) return;
    R = a380x_fmgc_turn_radius_nm(in->gs_kt, A380X_FMGC_DESIGN_BANK_DEG);
    g->arc_dir = change >= 0 ? 1 : -1;
    a380x_geo_destination(l->fix.lat, l->fix.lon, n360(cin + 180.0), antic, &t1lat, &t1lon);
    a380x_geo_destination(t1lat, t1lon, n360(cin + 90.0 * g->arc_dir), R, &g->arc_clat, &g->arc_clon);
    g->arc_radius_nm = R;
    g->arc_start_brg = n360(cin - 90.0 * g->arc_dir);
    g->arc_sweep_deg = fmin(fabs(change), 135.0);
    g->arc_active = 1;
    g->entry_antic_nm = antic;
    g->entry_arc_nm = R * g->arc_sweep_deg * D2R;
}

/* ============================================================================================ */
/* VNAV descent profile                                                                         */
/* ============================================================================================ */

/* Effective altitude window of a descent constraint: lo..hi (NAN = open). */
static int cstr_window(const A380XFplnLeg *l, double *lo, double *hi)
{
    A380XFplnAltCstr a = a380x_fpln_eff_alt(l);
    *lo = *hi = NAN;
    if (!a.desc || !isfinite(a.alt1_ft)) return 0;
    switch (a.desc) {
    case '@': case 'I': case 'X': case 'G': *lo = *hi = a.alt1_ft; return 1;
    case '+': case 'H': case 'J': case 'V': *lo = a.alt1_ft; return 1;
    case '-': case 'Y': *hi = a.alt1_ft; return 1;
    case 'B': *hi = a.alt1_ft; *lo = isfinite(a.alt2_ft) ? a.alt2_ft : NAN; return 1;
    case 'C': if (isfinite(a.alt2_ft)) { *lo = a.alt2_ft; return 1; } return 0;
    default: return 0;
    }
}

static double leg_length(const A380XFplnRoute *r, int k, double *last_lat, double *last_lon)
{
    const A380XFplnLeg *l = &r->el[k];
    double d = NAN;
    if (a380x_leg_is_hx(l->type)) d = 0.0;
    else if (k > 0 && is_disco(&r->el[k - 1]) && fin2(*last_lat, *last_lon) && fin2(l->term_lat, l->term_lon))
        d = a380x_geo_distance_nm(*last_lat, *last_lon, l->term_lat, l->term_lon);
    else if (isfinite(l->leg_dist_nm)) d = l->leg_dist_nm;
    else if (fin2(*last_lat, *last_lon) && fin2(l->term_lat, l->term_lon))
        d = a380x_geo_distance_nm(*last_lat, *last_lon, l->term_lat, l->term_lon);
    if (fin2(l->term_lat, l->term_lon)) { *last_lat = l->term_lat; *last_lon = l->term_lon; }
    return d;
}

typedef struct {
    double crz, lim_alt, decel_nm, decel_left, scale;
    int in_decel, decel_done;
    double tod, speed_limit_start;
} Prop;

/* Propagates the path backwards (upwards) over `len` NM from altitude h at distance d. Decelerating
 * segments (speed limit, speed constraints) use a third of the idle gradient. */
static double propagate(Prop *p, double h, double d, double len, double fpa_grad, int *segment)
{
    double step, done = 0.0;
    while (done < len - 1e-9) {
        double g, h_next;
        step = fmin(0.25, len - done);
        if (fpa_grad > 0) { g = fpa_grad; }
        else {
            g = a380x_fmgc_idle_gradient_ft_nm(h, p->lim_alt) * p->scale;
            if (!p->decel_done && p->lim_alt > 0 && h >= p->lim_alt) {
                p->decel_done = 1;
                if (p->decel_nm > p->decel_left) p->decel_left = p->decel_nm;
                p->speed_limit_start = d + done + p->decel_nm;   /* where the deceleration begins */
            }
            if (p->decel_left > 0) {
                g /= 3.0;
                if (segment && *segment == 0) *segment = 3;
                p->decel_left -= step;
            }
        }
        h_next = h + g * step;
        if (!isfinite(p->tod) && h < p->crz && h_next >= p->crz) p->tod = d + done + step * (p->crz - h) / (h_next - h);
        h = h_next;
        done += step;
    }
    return h;
}

/* Fly-by transition at the end of leg k: anticipation distance and arc length (flown path). The
 * active leg uses its flown final course; later legs their coded geometry. */
static void transition_at(const A380XFmgc *g, const A380XFplnRoute *r, int k, int active, const A380XFmgcInputs *in,
                          double *antic, double *arc)
{
    const A380XFplnLeg *l = &r->el[k], *nx = k + 1 < r->count ? &r->el[k + 1] : NULL;
    double change = 0.0, cin, cout;
    *antic = *arc = 0.0;
    if (k == active) *antic = leg_anticipation(g, r, k, in, &change);
    else {
        if (!is_leg(nx) || (l->flags & A380X_LEGF_OVERFLY) || a380x_leg_is_hx(nx->type)) return;
        if (r->destination_leg_index >= 0 && k >= r->destination_leg_index) return;
        cin = leg_final_course(l); cout = leg_initial_course(nx, l);
        if (!fin2(cin, cout)) return;
        change = adiff(cin, cout);
        *antic = a380x_fmgc_anticipation_nm(in->gs_kt, change);
    }
    if (*antic > 0.0)
        *arc = a380x_fmgc_turn_radius_nm(in->gs_kt, A380X_FMGC_DESIGN_BANK_DEG) * fmin(fabs(change), 135.0) * D2R;
}

static void compute_profile(A380XFmgc *g, const A380XFplnRoute *r, int a, const A380XFmgcInputs *in, double rem_active)
{
    A380XFmgcOutputs *o = &g->out;
    int dleg = r->destination_leg_index, k, n = 0;
    double dist[A380X_FPLN_MAX_ELEMENTS], llat = NAN, llon = NAN, h, d;
    double lens[A380X_FPLN_MAX_ELEMENTS], vpre[A380X_FPLN_MAX_ELEMENTS];
    Prop p;
    const A380XFplnLeg *dl;
    o->vnav_valid = 0;
    o->profile_count = 0;
    if (dleg < 0 || a > dleg || !isfinite(rem_active)) return;
    /* leg lengths a+1..dleg, distance of each termination before the destination */
    for (k = 0; k <= a && k < r->count; ++k)
        if (is_leg(&r->el[k]) && fin2(r->el[k].term_lat, r->el[k].term_lon)) { llat = r->el[k].term_lat; llon = r->el[k].term_lon; }
    {   /* flown-path lengths: each fly-by transition replaces 2 x anticipation by its arc */
        double ant_prev, arc_prev, ant, arc;
        transition_at(g, r, a, a, in, &ant_prev, &arc_prev);
        if (!in->nav_engaged) ant_prev = arc_prev = 0.0;
        for (k = a + 1; k <= dleg; ++k) {
            if (!is_leg(&r->el[k])) { lens[k] = 0.0; ant_prev = arc_prev = 0.0; continue; }
            lens[k] = leg_length(r, k, &llat, &llon);
            if (!isfinite(lens[k])) return;   /* manual / undefined leg: no profile */
            transition_at(g, r, k, a, in, &ant, &arc);
            lens[k] = fmax(0.0, lens[k] - ant_prev - ant + 0.5 * (arc_prev + arc));
            ant_prev = ant; arc_prev = arc;
        }
    }
    dist[dleg] = 0.0;
    for (k = dleg; k > a; --k) dist[k - 1] = dist[k] + lens[k];
    o->dist_to_dest_nm = dist[a] + rem_active;
    /* end point */
    dl = &r->el[dleg];
    {
        double lo, hi;
        double elev = isfinite(r->dest_elev_ft) ? r->dest_elev_ft : 0.0;
        h = elev + 50.0;
        if (dl->wp_desc == A380X_WPDESC_RUNWAY && cstr_window(dl, &lo, &hi) && isfinite(lo) && lo > elev && lo < elev + 500) h = lo;
    }
    memset(&p, 0, sizeof p);
    p.crz = in->cruise_alt_ft > 0 ? in->cruise_alt_ft : fmax(in->alt_ft, in->fcu_alt_ft);
    p.lim_alt = in->des_limit_alt_ft > 0 ? in->des_limit_alt_ft : 0.0;
    {
        double vdes = in->descent_speed_kt > 0 ? in->descent_speed_kt : 300.0;
        double vlim = in->des_limit_kt > 0 ? in->des_limit_kt : 250.0;
        p.decel_nm = vdes > vlim ? clampd((vdes - vlim) * 0.15, 3.0, 20.0) : 0.0;
        if (p.decel_nm <= 0) p.decel_done = 1;
    }
    p.tod = NAN; p.speed_limit_start = NAN;
    if (p.lim_alt <= 0) p.decel_done = 1;
    /* ground gradient: still-air gradient x TAS/GS (current wind), x learned idle factor */
    p.scale = (g->idle_factor > 0 ? g->idle_factor : 1.0) *
              (in->gs_kt > 100 && in->tas_kt > 100 ? clampd(in->tas_kt / in->gs_kt, 0.75, 1.35) : 1.0);
    /* forward speed before each fix: descent speed, then the running minimum of the DES speed
     * constraints from the start of the plan (those behind the aircraft stay in force) */
    {
        double v = in->descent_speed_kt > 0 ? in->descent_speed_kt : 300.0;
        for (k = 0; k <= dleg; ++k) {
            A380XFplnSpdCstr sc;
            vpre[k] = v;
            if (!is_leg(&r->el[k])) continue;
            sc = a380x_fpln_eff_spd(&r->el[k]);
            if (r->el[k].cstr_type == A380X_CSTR_DES && (sc.desc == '-' || sc.desc == '@') && isfinite(sc.speed) && sc.speed > 2.0 && sc.speed < v)
                v = sc.speed;
        }
    }
    o->profile[n].dist_to_dest_nm = 0.0; o->profile[n].alt_ft = h; o->profile[n].leg_index = dleg; o->profile[n].segment = 1;
    n++;
    /* backward pass: segment of leg k runs from termination k-1 (or PPOS) to termination k */
    for (k = dleg; k >= a; --k) {
        const A380XFplnLeg *l = &r->el[k];
        double len = k == a ? rem_active : lens[k], fpa = 0.0, h_raw, lo, hi;
        int seg = 0;
        d = dist[k];
        if (!is_leg(l)) continue;
        if (k == a && len < 0) {
            /* the flown path already passed the termination (large turn onto a short leg) */
            double slope = n >= 2 ? (o->profile[n - 1].alt_ft - o->profile[n - 2].alt_ft) /
                                    fmax(0.05, o->profile[n - 1].dist_to_dest_nm - o->profile[n - 2].dist_to_dest_nm) : 0.0;
            h += slope * len;
            seg = o->profile[n - 1].segment;
            goto stored;
        }
        if (isfinite(l->vertical_angle_deg) && l->vertical_angle_deg < -0.5 && l->segment >= A380X_SEG_APPROACH)
            fpa = tan(fabs(l->vertical_angle_deg) * D2R) * FT_PER_NM;
        if (fpa > 0) seg = 1;
        {   /* deceleration before a DES speed constraint at this termination */
            A380XFplnSpdCstr sc = a380x_fpln_eff_spd(l);
            if (l->cstr_type == A380X_CSTR_DES && (sc.desc == '-' || sc.desc == '@') && isfinite(sc.speed)
                && sc.speed > 2.0 && sc.speed < vpre[k] - 1.0) {
                double dd = clampd((vpre[k] - sc.speed) * 0.15, 3.0, 20.0);
                if (dd > p.decel_left) p.decel_left = dd;
            }
        }
        if (k == a && a > 0 && is_leg(&r->el[a - 1]) && r->el[a - 1].cstr_type != A380X_CSTR_CLB
            && isfinite(l->leg_dist_nm) && l->leg_dist_nm > 0.05 && cstr_window(&r->el[a - 1], &lo, &hi)) {
            /* the segment of the active leg is shaped by the constraint behind the aircraft: fly the
             * straight (geometric) line between the two points; before the leg start (fly-by
             * anticipation) continue from that constraint point with the idle gradient */
            Prop tmp = p;
            double ant_end, arc_end, len_a, h_full, h_prev;
            transition_at(g, r, a, a, in, &ant_end, &arc_end);
            if (!in->nav_engaged) ant_end = arc_end = 0.0;
            /* flown-path length of the active leg, same convention as rem_active */
            len_a = fmax(0.1, l->leg_dist_nm - g->entry_antic_nm - ant_end + g->entry_arc_nm + 0.5 * arc_end);
            h_full = propagate(&tmp, h, d, len_a, fpa, NULL); h_prev = h_full;
            if (isfinite(hi) && h_prev > hi) h_prev = hi;
            if (isfinite(lo) && h_prev < lo) h_prev = lo;
            if (fabs(h_prev - h_full) > 1.0) {
                /* the constraint point is the middle of the entry arc, Lc before the termination */
                double lc = fmax(0.1, len_a - 0.5 * g->entry_arc_nm);
                double behind = a380x_fmgc_idle_gradient_ft_nm(h_prev, p.lim_alt) * p.scale;
                A380XFplnSpdCstr sb = a380x_fpln_eff_spd(&r->el[a - 1]);
                if (r->el[a - 1].cstr_type == A380X_CSTR_DES && (sb.desc == '-' || sb.desc == '@') && isfinite(sb.speed)
                    && sb.speed > 2.0 && sb.speed < vpre[a - 1] - 1.0) behind /= 3.0;   /* decelerating segment behind */
                h_raw = rem_active <= lc ? h + (h_prev - h) * rem_active / lc : h_prev + behind * (rem_active - lc);
                seg = h_prev < h_full ? 1 : 2;
                if (!isfinite(p.tod) && h < p.crz && h_raw >= p.crz) p.tod = d + rem_active * (p.crz - h) / (h_raw - h);
                h = h_raw;
                goto stored;
            }
        }
        h_raw = propagate(&p, h, d, len, fpa, &seg);
        h = h_raw;
        if (k > a) {
            const A380XFplnLeg *pv = &r->el[k - 1];
            int prev_k = k - 1;
            while (prev_k > a && !is_leg(&r->el[prev_k])) pv = &r->el[--prev_k];
            if (is_leg(pv) && pv->cstr_type != A380X_CSTR_CLB && cstr_window(pv, &lo, &hi)) {
                if (isfinite(hi) && h > hi) { h = hi; if (seg == 0 || seg == 3) seg = 1; }
                if (isfinite(lo) && h < lo) { h = lo; seg = 2; }
            }
        }
    stored:
        if (n < A380X_FMGC_MAX_PROFILE) {
            /* store the segment kind on the point that ends it (termination k) */
            o->profile[n - 1].segment = seg;
            o->profile[n].dist_to_dest_nm = k > a ? dist[k - 1] : o->dist_to_dest_nm;
            o->profile[n].alt_ft = h;
            o->profile[n].leg_index = k > a ? k - 1 : -1;
            o->profile[n].segment = 0;
            n++;
        }
        if (k == a) {
            o->path_segment = seg;
            /* local gradient at PPOS: from the last two profile points */
            if (n >= 2) {
                double dd = o->profile[n - 1].dist_to_dest_nm - o->profile[n - 2].dist_to_dest_nm;
                double grad = dd > 0.05 ? (o->profile[n - 1].alt_ft - o->profile[n - 2].alt_ft) / dd
                                        : a380x_fmgc_idle_gradient_ft_nm(h, p.lim_alt);
                if ((seg == 0 || seg == 3) && fpa <= 0)
                    grad = a380x_fmgc_idle_gradient_ft_nm(h, p.lim_alt) * p.scale / (p.decel_left > 0 ? 3.0 : 1.0);
                o->path_vs_fpm = -grad * fmax(in->gs_kt, 60.0) / 60.0;
            }
        }
    }
    o->profile_count = n;
    o->path_alt_ft = h;
    o->vdev_ft = in->alt_ft - h;
    o->tod_dist_nm = p.tod;
    o->before_tod = isfinite(p.tod) ? o->dist_to_dest_nm > p.tod + 0.05 : 0;
    o->speed_limit_dist_nm = p.speed_limit_start;
    if (o->before_tod) {   /* cruise part: the path is the idle path extended above the cruise level */
        o->path_segment = 0;
        o->path_vs_fpm = -a380x_fmgc_idle_gradient_ft_nm(h, p.lim_alt) * p.scale * fmax(in->gs_kt, 60.0) / 60.0;
    }
    /* DECEL: 1000 ft above the landing elevation on the final path plus the deceleration
     * from the speed-limit speed to VAPP (port model: (Vlim - VAPP)/10 + 2 NM, 4..15 NM). */
    {
        double elev = isfinite(r->dest_elev_ft) ? r->dest_elev_ft : o->profile[0].alt_ft - 50.0;
        double d1000 = NAN, vlim = in->des_limit_kt > 0 ? in->des_limit_kt : 250.0, vapp = in->vapp_kt > 0 ? in->vapp_kt : 140.0;
        for (k = 1; k < n; ++k) {
            if (o->profile[k].alt_ft >= elev + 1000.0) {
                double a0 = o->profile[k - 1].alt_ft, a1 = o->profile[k].alt_ft;
                double f = a1 > a0 ? (elev + 1000.0 - a0) / (a1 - a0) : 0.0;
                d1000 = o->profile[k - 1].dist_to_dest_nm + f * (o->profile[k].dist_to_dest_nm - o->profile[k - 1].dist_to_dest_nm);
                break;
            }
        }
        if (!isfinite(d1000)) d1000 = 1000.0 / a380x_fmgc_idle_gradient_ft_nm(0, 0);
        o->decel_dist_nm = d1000 + clampd((vlim - vapp) / 10.0 + 2.0, 4.0, 15.0);
    }
    o->vnav_valid = 1;
}

/* Position at a distance before the destination along the ACTIVE plan (pseudo waypoints). */
static int point_at_dist(const A380XFmgcOutputs *o, const A380XFplnRoute *r, double dist, const A380XFmgcInputs *in,
                         double *lat, double *lon, int *leg)
{
    int k;
    *lat = *lon = NAN; *leg = -1;
    if (!isfinite(dist) || o->profile_count < 2) return 0;
    for (k = 1; k < o->profile_count; ++k) {
        const A380XFmgcProfilePoint *a0 = &o->profile[k - 1], *a1 = &o->profile[k];
        if (dist >= a0->dist_to_dest_nm && dist <= a1->dist_to_dest_nm) {
            const A380XFplnLeg *l = a0->leg_index >= 0 ? &r->el[a0->leg_index] : NULL;
            double back = dist - a0->dist_to_dest_nm, slat, slon, brg;
            *leg = a0->leg_index;
            if (!l || !fin2(l->term_lat, l->term_lon)) return 0;
            if (a1->leg_index >= 0 && fin2(r->el[a1->leg_index].term_lat, r->el[a1->leg_index].term_lon)) {
                slat = r->el[a1->leg_index].term_lat; slon = r->el[a1->leg_index].term_lon;
            } else if (a1->leg_index < 0) { slat = in->lat_deg; slon = in->lon_deg; }
            else if (fin2(l->start_lat, l->start_lon)) { slat = l->start_lat; slon = l->start_lon; }
            else return 0;
            brg = a380x_geo_bearing_deg(l->term_lat, l->term_lon, slat, slon);
            a380x_geo_destination(l->term_lat, l->term_lon, brg, back, lat, lon);
            return 1;
        }
    }
    return 0;
}

/* ============================================================================================ */
/* Update                                                                                       */
/* ============================================================================================ */

void a380x_fmgc_init(A380XFmgc *g)
{
    memset(g, 0, sizeof *g);
    g->leg = -1;
    g->out.active_leg = -1;
    g->idle_factor = 1.0;
    g->vs_filt_fpm = NAN;
}

const A380XFmgcOutputs *a380x_fmgc_outputs(const A380XFmgc *g) { return &g->out; }

static void outputs_reset(A380XFmgcOutputs *o)
{
    memset(o, 0, sizeof *o);
    o->active_leg = -1;
    o->desired_track_true_deg = o->managed_heading_mag_deg = o->dtg_active_nm = NAN;
    o->dist_to_dest_nm = o->path_alt_ft = o->vdev_ft = o->tod_dist_nm = o->decel_dist_nm = NAN;
    o->speed_limit_dist_nm = o->managed_altitude_ft = o->toc_dist_from_ppos_nm = NAN;
    o->tod_lat = o->tod_lon = o->decel_lat = o->decel_lon = o->toc_lat = o->toc_lon = NAN;
    o->tod_leg = o->decel_leg = o->toc_leg = -1;
    o->hold_speed_kt = NAN;
    o->hold_leg_index = -1;
}

static void leg_identity_store(A380XFmgc *g, const A380XFplnRoute *r, int a)
{
    const A380XFplnLeg *l = &r->el[a];
    g->leg = a;
    g->leg_type = (int)l->type;
    snprintf(g->leg_ident, sizeof g->leg_ident, "%s", l->ident);
    g->leg_lat = l->fix.lat; g->leg_lon = l->fix.lon;
}

/* The TO leg changed by something other than our sequencing (DIR TO, F-PLN edit, new plan). */
static int leg_identity_changed(const A380XFmgc *g, const A380XFplnPlan *p)
{
    const A380XFplnLeg *l;
    if (g->leg != p->active_leg_index) return 1;
    l = &p->pri.el[p->active_leg_index];
    if (g->leg_type != (int)l->type || strcmp(g->leg_ident, l->ident) != 0) return 1;
    if (isfinite(l->fix.lat) != isfinite(g->leg_lat)) return 1;
    return isfinite(l->fix.lat) && (fabs(l->fix.lat - g->leg_lat) > 1e-7 || fabs(l->fix.lon - g->leg_lon) > 1e-7);
}

/* Descent speed constraints ahead (beyond the active leg too): decelerate decelDist before the fix
 * (FBW decelDist = clamp((from - to) * 0.15, 3, 20) NM). */
static double lookahead_speed(const A380XFplnRoute *r, int a, double dtg, double target)
{
    double cum = isfinite(dtg) ? dtg : 0.0, llat = NAN, llon = NAN;
    int k, end = r->first_missed_appr_index < r->count ? r->first_missed_appr_index : r->count;
    for (k = a; k < end && cum < 40.0; ++k) {
        const A380XFplnLeg *l = &r->el[k];
        A380XFplnSpdCstr sc;
        if (k > a) {
            double len = leg_length(r, k, &llat, &llon);
            if (!isfinite(len)) break;
            cum += len;
        } else if (fin2(l->term_lat, l->term_lon)) { llat = l->term_lat; llon = l->term_lon; }
        if (!is_leg(l) || l->cstr_type != A380X_CSTR_DES) continue;
        sc = a380x_fpln_eff_spd(l);
        if ((sc.desc == '-' || sc.desc == '@') && isfinite(sc.speed) && sc.speed > 100.0 && sc.speed < target
            && cum <= clampd((target - sc.speed) * 0.15, 3.0, 20.0))
            target = sc.speed;
    }
    return target;
}

/* FBW updateHoldingSpeed (spec 07 section 5.8): the holding speed of the active hold leg, or of the
 * next leg when it is a hold - the managed target from the deceleration distance before its fix. */
static void hold_speed_update(A380XFmgc *g, const A380XFplnRoute *r, int a, const A380XFmgcInputs *in)
{
    A380XFmgcOutputs *o = &g->out;
    const A380XFplnLeg *l;
    int k = -1;
    if (a < 0 || a >= r->count || in->on_ground) return;
    if (is_leg(&r->el[a]) && a380x_leg_is_hx(r->el[a].type)) k = a;
    else if (a + 1 < r->count && is_leg(&r->el[a + 1]) && a380x_leg_is_hx(r->el[a + 1].type)) k = a + 1;
    if (k < 0) return;
    l = &r->el[k];
    {
        A380XFplnSpdCstr sc = a380x_fpln_eff_spd(l);
        A380XFplnAltCstr ac = a380x_fpln_eff_alt(l);
        double cs = (sc.desc == '-' || sc.desc == '@') && isfinite(sc.speed) && sc.speed > 100.0 ? sc.speed : NAN;
        o->hold_speed_kt = a380x_fmgc_holding_speed(in, cs, ac.desc && isfinite(ac.alt1_ft) ? ac.alt1_ft : NAN);
    }
    o->hold_leg_index = k;
    if (k == a) {
        o->hold_decel_reached = 1;
        o->hold_entry = g->hold_entry;
    } else {
        double v = in->cas_kt > 60.0 ? in->cas_kt : o->hold_speed_kt;
        double dd = clampd((v - o->hold_speed_kt) * 0.15, 3.0, 20.0);
        o->hold_decel_reached = isfinite(o->dtg_active_nm) && o->dtg_active_nm <= dd;
    }
}

/* ---- pseudo waypoints (ND symbols + MFD F-PLN lines) ---------------------------------------- */

/* Point `ahead` NM along the ACTIVE plan from PPOS: the rest of the active leg (rem_active), then
 * the leg lengths of the profile (holds 0), placed on the straight line back from each termination.
 * 0 beyond a discontinuity, a manual leg, the missed approach or the end of the plan. */
static int point_ahead(const A380XFplnRoute *r, int a, const A380XFmgcInputs *in, double rem_active, double ahead,
                       double *lat, double *lon, int *leg, double *before_term)
{
    double cum = 0.0, plat = in->lat_deg, plon = in->lon_deg, llat = in->lat_deg, llon = in->lon_deg;
    int k, end = r->first_missed_appr_index < r->count ? r->first_missed_appr_index : r->count;
    *lat = *lon = *before_term = NAN; *leg = -1;
    if (!isfinite(ahead) || ahead < 0.0) return 0;
    for (k = a; k < end; ++k) {
        const A380XFplnLeg *l = &r->el[k];
        double len, geo;
        if (!is_leg(l) || !fin2(l->term_lat, l->term_lon)) return 0;
        geo = a380x_geo_distance_nm(plat, plon, l->term_lat, l->term_lon);
        if (k == a) { len = isfinite(rem_active) && rem_active > 0.05 ? rem_active : geo; llat = l->term_lat; llon = l->term_lon; }
        else { len = leg_length(r, k, &llat, &llon); if (!isfinite(len)) return 0; }
        if (len > 1e-3 && cum + len >= ahead) {
            double back = cum + len - ahead;
            a380x_geo_destination(l->term_lat, l->term_lon, a380x_geo_bearing_deg(l->term_lat, l->term_lon, plat, plon),
                                  geo > 1e-3 ? back * geo / len : 0.0, lat, lon);
            *leg = k; *before_term = back;
            return 1;
        }
        cum += len; plat = l->term_lat; plon = l->term_lon;
    }
    return 0;
}

/* Distance along the plan from PPOS to the termination of leg k (same convention), NAN. */
static double ahead_of_term(const A380XFplnRoute *r, int a, const A380XFmgcInputs *in, double rem_active, int k)
{
    double cum = 0.0, llat = in->lat_deg, llon = in->lon_deg;
    int j;
    for (j = a; j <= k && j < r->count; ++j) {
        const A380XFplnLeg *l = &r->el[j];
        double len;
        if (!is_leg(l) || !fin2(l->term_lat, l->term_lon)) return NAN;
        if (j == a) {
            len = isfinite(rem_active) && rem_active > 0.05 ? rem_active : a380x_geo_distance_nm(llat, llon, l->term_lat, l->term_lon);
            llat = l->term_lat; llon = l->term_lon;
        } else len = leg_length(r, j, &llat, &llon);
        if (!isfinite(len)) return NAN;
        cum += len;
    }
    return cum;
}

static A380XFmgcNdPwp *pwp_new(A380XFmgcOutputs *o, const char *ident, unsigned symbol, A380XPwpColour colour, int mfd)
{
    A380XFmgcNdPwp *w;
    if (o->nd_pwp_count >= A380X_FMGC_MAX_ND_PWP) return NULL;
    w = &o->nd_pwp[o->nd_pwp_count++];
    memset(w, 0, sizeof *w);
    snprintf(w->ident, sizeof w->ident, "%s", ident);
    w->symbol = symbol; w->colour = colour; w->mfd = mfd;
    w->lat = w->lon = w->alt_ft = w->speed_kt = w->before_term_nm = NAN;
    w->leg_index = -1;
    return w;
}

/* Places a pseudo waypoint `ahead` NM from PPOS: on the path in NAV, along the track in HDG/TRK
 * when the symbol has a selected-mode meaning (track_ok: level-off / intercept arrows). */
static void pwp_place(A380XFmgcNdPwp *w, const A380XFplnRoute *r, int a, const A380XFmgcInputs *in, double rem,
                      double ahead, int track_ok)
{
    int on_path;
    if (!w) return;
    w->dist_ahead_nm = ahead;
    on_path = point_ahead(r, a, in, rem, ahead, &w->lat, &w->lon, &w->leg_index, &w->before_term_nm);
    w->nd = in->nav_engaged ? (on_path ? A380X_PWP_ND_PATH : A380X_PWP_ND_NONE)
                            : (track_ok ? A380X_PWP_ND_TRACK : A380X_PWP_ND_NONE);
}

/* First constraint leg at/after the active one of the given kind whose limit (climb: upper, descent:
 * lower) is within 150 ft of alt_ft, -1. */
static int constraint_leg(const A380XFplnRoute *r, int a, A380XFplnCstrType kind, double alt_ft)
{
    int k, end = r->first_missed_appr_index < r->count ? r->first_missed_appr_index : r->count;
    for (k = a; k < end; ++k) {
        double lo, hi, lim;
        if (!is_leg(&r->el[k])) return -1;
        if (r->el[k].cstr_type != kind || !cstr_window(&r->el[k], &lo, &hi)) continue;
        lim = kind == A380X_CSTR_CLB ? hi : lo;
        if (isfinite(lim) && fabs(lim - alt_ft) <= 150.0) return k;
    }
    return -1;
}

/* Distance to climb from alt_ft to target_ft at the present V/S, the rate falling linearly with
 * altitude to zero at the A380 ceiling (classical absolute-ceiling approximation; the FBW FMS
 * integrates its performance model instead): t = (C - h0) / VS0 * ln((C - h0) / (C - h1)). */
#define FMGC_CLIMB_CEILING_FT 43000.0
double a380x_fmgc_climb_distance_nm(double alt_ft, double target_ft, double vs_fpm, double gs_kt)
{
    const double c = FMGC_CLIMB_CEILING_FT;
    double t_min;
    if (!(vs_fpm > 1.0) || !(target_ft > alt_ft)) return 0.0;
    if (alt_ft >= c - 1000.0 || target_ft >= c - 500.0) t_min = (target_ft - alt_ft) / vs_fpm;
    else t_min = (c - alt_ft) / vs_fpm * log((c - alt_ft) / (c - target_ft));
    return t_min * fmax(gs_kt, 60.0) / 60.0;
}

/* Distance before the destination where the descent profile first reaches alt_ft ahead, NAN. */
static double profile_reaches(const A380XFmgcOutputs *o, double alt_ft)
{
    int j;
    for (j = o->profile_count - 1; j >= 1; --j) {
        const A380XFmgcProfilePoint *hi = &o->profile[j], *lo = &o->profile[j - 1];
        if (hi->alt_ft >= alt_ft && lo->alt_ft < alt_ft) {
            double f = hi->alt_ft > lo->alt_ft ? (hi->alt_ft - alt_ft) / (hi->alt_ft - lo->alt_ft) : 0.0;
            return hi->dist_to_dest_nm + f * (lo->dist_to_dest_nm - hi->dist_to_dest_nm);
        }
    }
    return NAN;
}

/* FBW PseudoWaypoints / VNAV checkpoints, port subset: climb (T/C or level-off at the FCU / an
 * altitude constraint, start of climb at that constraint, climb speed limit), T/D, descent speed
 * limit, DECEL, descent level-off, start of descent after an ALT CST level-off, profile intercept.
 * Colours: managed vertical mode magenta, selected cyan; T/D white; DECEL magenta with managed speed,
 * else white; start of climb / descent cyan when CLB / DES is armed, else white. */
static void pseudo_waypoints(A380XFmgc *g, const A380XFplnRoute *r, int a, const A380XFmgcInputs *in, double rem)
{
    A380XFmgcOutputs *o = &g->out;
    A380XFmgcNdPwp *w;
    const double vs = g->vs_filt_fpm, gs = fmax(in->gs_kt, 60.0), crz = in->cruise_alt_ft;
    const int vm = in->fg_vertical_mode;
    const int clb = vm == A380X_FMGC_VM_CLB, des = vm == A380X_FMGC_VM_DES;
    const int climbing = clb || vm == A380X_FMGC_VM_OP_CLB || vm == A380X_FMGC_VM_SRS || (vm == A380X_FMGC_VM_VS_FPA && vs > 100.0);
    const int descending = des || vm == A380X_FMGC_VM_OP_DES || (vm == A380X_FMGC_VM_VS_FPA && vs < -100.0);
    int i, j;
    if (in->on_ground || !in->position_valid || a < 0) return;
    /* ---- climb ---- */
    if (climbing && vs > 300.0 && in->phase <= PH_CRUISE) {
        int cstr = clb && o->managed_altitude_is_constraint;
        double target = clb && isfinite(o->managed_altitude_ft) ? o->managed_altitude_ft : in->fcu_alt_ft;
        int toc;
        if (clb && crz > 0) target = fmin(target, crz);   /* managed climb levels at the cruise level (ALT CRZ) */
        toc = !cstr && crz > 0 && target >= crz - 50.0;
        if (target > in->alt_ft + 100.0) {
            w = pwp_new(o, toc ? "(T/C)" : "LEVEL", A380X_PWP_CLIMB_LEVEL_OFF, clb ? A380X_PWP_MAGENTA : A380X_PWP_CYAN, toc);
            pwp_place(w, r, a, in, rem, a380x_fmgc_climb_distance_nm(in->alt_ft, target, vs, gs), 1);
            if (w) w->alt_ft = toc ? crz : target;
        }
        if (!toc && crz > in->alt_ft + 100.0 && (w = pwp_new(o, "(T/C)", A380X_PWP_CLIMB_LEVEL_OFF, A380X_PWP_MAGENTA, 1))) {
            pwp_place(w, r, a, in, rem, a380x_fmgc_climb_distance_nm(in->alt_ft, crz, vs, gs), 0);
            w->nd = A380X_PWP_ND_NONE;   /* the level-off below the cruise level is the ND symbol */
            w->alt_ft = crz;
        }
        if (cstr) {   /* the climb resumes at the constraint waypoint */
            int k = constraint_leg(r, a, A380X_CSTR_CLB, target);
            if (k >= 0 && (w = pwp_new(o, "START", A380X_PWP_START_OF_CLIMB, A380X_PWP_CYAN, 0))) {
                pwp_place(w, r, a, in, rem, ahead_of_term(r, a, in, rem, k), 0);
                w->alt_ft = target;
            }
        }
        if (in->clb_limit_kt > 0 && in->clb_limit_alt_ft > in->alt_ft + 100.0 && target > in->clb_limit_alt_ft
            && (w = pwp_new(o, "(SPDLIM)", A380X_PWP_SPEED_CHANGE, A380X_PWP_MAGENTA, 1))) {
            pwp_place(w, r, a, in, rem, a380x_fmgc_climb_distance_nm(in->alt_ft, in->clb_limit_alt_ft, vs, gs), 0);
            if (!in->speed_managed) w->nd = A380X_PWP_ND_NONE;
            w->alt_ft = in->clb_limit_alt_ft; w->speed_kt = in->clb_limit_kt;
        }
    } else if (vm == A380X_FMGC_VM_ALT && in->alt_constraint_mode && in->clb_armed && in->phase <= PH_CLIMB) {
        int k = constraint_leg(r, a, A380X_CSTR_CLB, in->alt_ft);
        if (k >= 0 && (w = pwp_new(o, "START", A380X_PWP_START_OF_CLIMB, A380X_PWP_CYAN, 0))) {
            pwp_place(w, r, a, in, rem, ahead_of_term(r, a, in, rem, k), 0);
            w->alt_ft = in->alt_ft;
        }
    }
    /* ---- descent profile ---- */
    if (o->vnav_valid && isfinite(o->dist_to_dest_nm) && in->phase >= PH_CLIMB && in->phase <= PH_DESCENT) {
        if (o->before_tod && isfinite(o->tod_dist_nm)
            && (w = pwp_new(o, "(T/D)", A380X_PWP_START_OF_DESCENT, A380X_PWP_WHITE, 1))) {
            pwp_place(w, r, a, in, rem, o->dist_to_dest_nm - o->tod_dist_nm, 0);
            w->alt_ft = crz > 0 ? crz : in->alt_ft;
        }
        if (in->des_limit_kt > 0 && in->alt_ft > in->des_limit_alt_ft + 100.0 && isfinite(o->speed_limit_dist_nm)
            && o->speed_limit_dist_nm < o->dist_to_dest_nm
            && (w = pwp_new(o, "(SPDLIM)", A380X_PWP_SPEED_CHANGE, A380X_PWP_MAGENTA, 1))) {
            pwp_place(w, r, a, in, rem, o->dist_to_dest_nm - o->speed_limit_dist_nm, 0);
            if (!in->speed_managed) w->nd = A380X_PWP_ND_NONE;
            w->speed_kt = in->des_limit_kt;
        }
        if (!o->decel_reached && isfinite(o->decel_dist_nm) && o->decel_dist_nm < o->dist_to_dest_nm
            && (w = pwp_new(o, "(DECEL)", A380X_PWP_DECEL, in->speed_managed ? A380X_PWP_MAGENTA : A380X_PWP_WHITE, 1)))
            pwp_place(w, r, a, in, rem, o->dist_to_dest_nm - o->decel_dist_nm, 0);
    }
    /* ---- descent level-off / start of descent / intercept ---- */
    if (descending && vs < -300.0 && in->phase >= PH_CRUISE && in->phase <= PH_APPROACH) {
        double target = des && isfinite(o->managed_altitude_ft) ? o->managed_altitude_ft : in->fcu_alt_ft;
        if (target < in->alt_ft - 100.0) {
            double ahead = NAN;
            if (des && in->des_submode == 0 && o->vnav_valid) {
                double d2d = profile_reaches(o, target);
                if (isfinite(d2d)) ahead = o->dist_to_dest_nm - d2d;
            }
            if (!isfinite(ahead) || ahead < 0.0) ahead = (in->alt_ft - target) / -vs * gs / 60.0;
            if ((w = pwp_new(o, "LEVEL", A380X_PWP_DESCENT_LEVEL_OFF, des ? A380X_PWP_MAGENTA : A380X_PWP_CYAN, 0))) {
                pwp_place(w, r, a, in, rem, ahead, 1);
                w->alt_ft = target;
            }
        }
    } else if (vm == A380X_FMGC_VM_ALT && in->alt_constraint_mode && in->des_armed && in->phase >= PH_DESCENT) {
        int k = constraint_leg(r, a, A380X_CSTR_DES, in->alt_ft);
        if (k >= 0 && (w = pwp_new(o, "START", A380X_PWP_START_OF_DESCENT, A380X_PWP_CYAN, 0))) {
            pwp_place(w, r, a, in, rem, ahead_of_term(r, a, in, rem, k), 0);
            w->alt_ft = in->alt_ft;
        }
    }
    if (in->phase == PH_DESCENT && o->vnav_valid && !o->before_tod && (des || in->des_armed) && fabs(o->vdev_ft) > 250.0) {
        double closure = vs - o->path_vs_fpm, t = fabs(closure) > 50.0 ? -o->vdev_ft / closure : -1.0;
        if (t > 0.0 && t < 30.0 && (w = pwp_new(o, "INTCPT", A380X_PWP_INTERCEPT_PROFILE, A380X_PWP_CYAN, 0)))
            pwp_place(w, r, a, in, rem, t * gs / 60.0, 1);
    }
    /* nearest first, nothing behind the aircraft */
    for (i = 0, j = 0; i < o->nd_pwp_count; ++i)
        if (isfinite(o->nd_pwp[i].dist_ahead_nm) && o->nd_pwp[i].dist_ahead_nm > 0.0) o->nd_pwp[j++] = o->nd_pwp[i];
    o->nd_pwp_count = j;
    for (i = 1; i < o->nd_pwp_count; ++i) {
        A380XFmgcNdPwp t = o->nd_pwp[i];
        for (j = i; j > 0 && o->nd_pwp[j - 1].dist_ahead_nm > t.dist_ahead_nm; --j) o->nd_pwp[j] = o->nd_pwp[j - 1];
        o->nd_pwp[j] = t;
    }
}

static void managed_targets(A380XFmgc *g, const A380XFlightPlans *fp, const A380XFmgcInputs *in)
{
    A380XFmgcOutputs *o = &g->out;
    double clb = NAN, des = NAN;
    a380x_fmgc_constraint_altitudes(fp, &clb, &des);
    o->managed_altitude_ft = in->fcu_alt_ft;
    o->managed_altitude_is_constraint = 0;
    if (in->phase < PH_CRUISE || in->phase == PH_GOAROUND) {
        if (isfinite(clb) && clb < in->fcu_alt_ft) { o->managed_altitude_ft = clb; o->managed_altitude_is_constraint = 1; }
    } else if (in->phase > PH_CRUISE && in->phase < PH_GOAROUND) {
        if (isfinite(des) && des > in->fcu_alt_ft) { o->managed_altitude_ft = des; o->managed_altitude_is_constraint = 1; }
    }
    /* managed speed (CAS) */
    o->speed_valid = 0;
    if (in->phase == PH_APPROACH || (in->phase == PH_DESCENT && g->decel_latched)) {
        double conf = in->green_dot_kt;
        switch (in->flap_lever) {
        case 0: conf = in->green_dot_kt; break;
        case 1: conf = in->s_speed_kt; break;
        case 2: conf = in->f_speed_kt; break;
        case 3: conf = in->landing_conf3 ? in->vapp_kt : in->f_speed_kt; break;
        default: conf = in->vapp_kt; break;
        }
        if (in->vapp_kt > 80.0) {
            o->speed_valid = 1;
            o->speed_kt = fmax(in->vapp_kt, conf > 80.0 ? conf : 0.0);
            o->speed_is_mach = 0;
        }
    } else if (in->phase == PH_DESCENT && in->managed_speed_kt > 80.0) {
        o->speed_valid = 1;
        o->speed_kt = in->managed_speed_kt;
        o->speed_is_mach = in->managed_is_mach;
        o->speed_mach = in->managed_mach;
        /* reach the speed limit at its altitude (decelerating segment of the profile) */
        if (in->des_limit_kt > 0 && in->alt_ft > in->des_limit_alt_ft && isfinite(o->speed_limit_dist_nm)
            && isfinite(o->dist_to_dest_nm) && o->dist_to_dest_nm <= o->speed_limit_dist_nm && o->speed_kt > in->des_limit_kt) {
            o->speed_kt = in->des_limit_kt;
            o->speed_is_mach = 0;
        }
        {
            const A380XFplnPlan *p = fp ? a380x_fpln_plan(fp, A380X_PLAN_ACTIVE) : NULL;
            if (p && o->active_leg >= 0 && o->active_leg < p->pri.count) {
                double cas_target = o->speed_kt, la = lookahead_speed(&p->pri, o->active_leg, o->dtg_active_nm, cas_target);
                if (la < cas_target) { o->speed_kt = la; o->speed_is_mach = 0; }
            }
        }
    }
    /* holding speed (active hold, or decelerating for the next one) outside the approach speeds */
    if (o->hold_decel_reached && isfinite(o->hold_speed_kt) && in->phase >= PH_CLIMB && in->phase <= PH_DESCENT
        && !(in->phase == PH_DESCENT && g->decel_latched)) {
        o->speed_valid = 1;
        o->speed_kt = o->hold_speed_kt;
        o->speed_is_mach = 0;
    }
}

void a380x_fmgc_update(A380XFmgc *g, A380XFlightPlans *fp, const A380XFmgcInputs *in, double dt)
{
    A380XFmgcOutputs *o = &g->out;
    const A380XFplnPlan *p;
    const A380XFplnRoute *r;
    Elem e;
    int a, guard, have = 0;
    double antic = 0.0, rem_active = NAN;
    outputs_reset(o);
    if (!g || !in) return;
    g->timer += dt > 0 ? dt : 0;
    /* V/S for the level-off predictions: 4 s first-order filter */
    if (!isfinite(in->vs_fpm)) g->vs_filt_fpm = 0.0;
    else if (!isfinite(g->vs_filt_fpm)) g->vs_filt_fpm = in->vs_fpm;
    else g->vs_filt_fpm += (in->vs_fpm - g->vs_filt_fpm) * (dt > 0 ? fmin(1.0, dt / 4.0) : 0.0);
    if (in->phase != PH_DESCENT && in->phase != PH_APPROACH) g->decel_latched = 0;
    p = fp ? a380x_fpln_plan(fp, A380X_PLAN_ACTIVE) : NULL;
    if (!p || !in->position_valid || p->active_leg_index < 0 || p->active_leg_index >= p->pri.count) {
        g->leg = -1;
        managed_targets(g, fp, in);
        return;
    }
    r = &p->pri;
    if (leg_identity_changed(g, p)) {
        g->arc_active = 0;
        g->entry_antic_nm = g->entry_arc_nm = 0.0;
        leg_activate(g, r, p->active_leg_index, in);
        leg_identity_store(g, r, p->active_leg_index);
    }
    for (guard = 0; guard < 4; ++guard) {
        int done, at_dest;
        a = p->active_leg_index;
        if (!is_leg(&r->el[a])) break;
        have = leg_element(g, r, a, in, &e);
        if (!have) break;
        elem_eval(&e, in);
        /* DF: freeze the line once established on the bearing in NAV */
        if (e.kind == A380X_LNAV_DIRECT && (r->el[a].type == A380X_LEG_DF || r->el[a].type == A380X_LEG_IF)
            && in->nav_engaged && !g->df_frozen && e.dtg > 2.0 && fabs(adiff(in->true_track_deg, e.desired)) < 3.0) {
            g->df_frozen = 1; g->df_lat = in->lat_deg; g->df_lon = in->lon_deg;
            leg_element(g, r, a, in, &e); elem_eval(&e, in);
        }
        if (in->on_ground) break;
        /* the new leg is not sequenced before its fly-by transition is complete (short legs after
         * large turns: the arc may end beyond the leg's termination, which then sequences) */
        if (g->arc_active && in->nav_engaged) {
            Elem t;
            transition_element(g, &t);
            elem_eval(&t, in);
            if (t.progress < t.sweep - 0.5 && fabs(t.xtk_right) <= 2.0 * g->arc_radius_nm) break;
        }
        done = leg_complete(g, r, a, in, &e, in->nav_engaged, &antic);
        if (!done) break;
        at_dest = r->destination_leg_index >= 0 && a >= r->destination_leg_index && a < r->first_missed_appr_index
                  && in->phase != PH_GOAROUND;
        if (at_dest) break;
        if (a + 1 >= r->count || is_disco(&r->el[a + 1])) {
            /* the TO waypoint is followed by a discontinuity: NAV is lost once it is passed
             * (anticipation is 0 before a discontinuity, so `done` means passed) */
            o->discontinuity = 1;
            break;
        }
        if (in->nav_engaged) setup_transition(g, r, a, in);
        else { g->arc_active = 0; g->entry_antic_nm = g->entry_arc_nm = 0.0; }
        if (!a380x_fpln_sequence_to(fp, a + 1)) break;
        p = a380x_fpln_plan(fp, A380X_PLAN_ACTIVE);
        r = &p->pri;
        leg_activate(g, r, p->active_leg_index, in);
        leg_identity_store(g, r, p->active_leg_index);
        o->sequenced = 1;
    }
    a = p->active_leg_index;
    o->active_leg = a;
    o->turn_radius_nm = a380x_fmgc_turn_radius_nm(in->gs_kt, A380X_FMGC_DESIGN_BANK_DEG);
    o->manual_leg = is_leg(&r->el[a]) && a380x_leg_is_vectors(r->el[a].type);
    if (have && !o->discontinuity) {
        Elem use = e;
        if (g->arc_active) {
            Elem t;
            transition_element(g, &t);
            elem_eval(&t, in);
            if (t.progress >= t.sweep - 0.5 || fabs(t.xtk_right) > 2.0 * g->arc_radius_nm) g->arc_active = 0;
            else use = t;
        }
        o->lnav_valid = 1;
        o->kind = use.kind;
        o->in_transition = g->arc_active;
        o->xtk_right_nm = use.xtk_right;
        o->fbw_xtk_nm = -use.xtk_right;
        if (use.kind == A380X_LNAV_HEADING) {
            o->fbw_tae_deg = adiff(in->true_heading_deg, use.desired);
            o->desired_track_true_deg = NAN;
            o->managed_heading_mag_deg = n360(use.desired - in->magvar_deg);
        } else {
            o->fbw_tae_deg = adiff(in->true_track_deg, use.desired);
            o->desired_track_true_deg = use.desired;
            o->managed_heading_mag_deg = n360(use.desired - in->magvar_deg);
        }
        if (use.kind == A380X_LNAV_ARC) {
            double v = fmax(in->gs_kt, 60.0) * KT_MS;
            o->bank_ff_deg = use.dir * atan(v * v / (G0 * use.radius * 1852.0)) / D2R;
            if (fabs(o->bank_ff_deg) > 25.0) o->bank_ff_deg = 25.0 * use.dir;
        }
        o->dtg_active_nm = e.kind == A380X_LNAV_TRACK || e.kind == A380X_LNAV_HEADING
            ? (fin2(r->el[a].term_lat, r->el[a].term_lon) ? a380x_geo_distance_nm(in->lat_deg, in->lon_deg, r->el[a].term_lat, r->el[a].term_lon) : NAN)
            : e.dtg;
        /* NAV capture zone */
        if (use.kind == A380X_LNAV_TRACK || use.kind == A380X_LNAV_HEADING || use.kind == A380X_LNAV_DIRECT) o->nav_capture = 1;
        else {
            double dlt = fabs(adiff(in->true_track_deg, use.desired));
            double cap = o->turn_radius_nm * (1.0 - cos(fmin(dlt, 150.0) * D2R)) + 0.15;
            int converging = use.xtk_right * adiff(use.desired, in->true_track_deg) < 0;
            o->nav_capture = (fabs(use.xtk_right) < 0.15 && dlt < 30.0) || (converging && dlt <= 150.0 && fabs(use.xtk_right) <= cap);
        }
        rem_active = e.dtg;
        if (!isfinite(rem_active) && fin2(r->el[a].term_lat, r->el[a].term_lon))
            rem_active = a380x_geo_distance_nm(in->lat_deg, in->lon_deg, r->el[a].term_lat, r->el[a].term_lon);
        if (isfinite(rem_active)) {
            /* along the flown path: the rest of the entry arc, the line up to the exit transition and
             * half of the exit arc (the profile's leg lengths use the same convention) */
            double ant_end = 0.0, arc_end = 0.0, line = rem_active;
            if (in->nav_engaged) transition_at(g, r, a, a, in, &ant_end, &arc_end);
            if (g->arc_active && use.kind == A380X_LNAV_ARC && e.kind == A380X_LNAV_LINE)
                line = use.dtg + (isfinite(r->el[a].leg_dist_nm) ? r->el[a].leg_dist_nm : e.length) - g->entry_antic_nm;
            rem_active = line - ant_end + 0.5 * arc_end;   /* < 0 when an arc ends beyond the termination */
        }
    } else if (is_leg(&r->el[a]) && fin2(r->el[a].term_lat, r->el[a].term_lon)) {
        rem_active = a380x_geo_distance_nm(in->lat_deg, in->lon_deg, r->el[a].term_lat, r->el[a].term_lon);
    }
    if (is_leg(&r->el[a]) && a380x_leg_is_hx(r->el[a].type) && has_fix(&r->el[a]))   /* holding: exit at the fix */
        rem_active = a380x_geo_distance_nm(in->lat_deg, in->lon_deg, r->el[a].fix.lat, r->el[a].fix.lon);
    /* ---- VNAV ---- */
    if (in->phase == PH_DESCENT && in->idle_speed_descent && in->vs_fpm < -300.0 && in->tas_kt > 150.0 && dt > 0) {
        /* idle descent performance: still-air gradient flown at idle with the speed on target */
        double meas = -in->vs_fpm * 60.0 / in->tas_kt, nom = a380x_fmgc_idle_gradient_ft_nm(in->alt_ft, in->des_limit_alt_ft);
        double ratio = clampd(meas / nom, 0.6, 1.4);
        g->idle_factor = clampd(g->idle_factor + (ratio - g->idle_factor) * fmin(1.0, dt / 120.0), 0.7, 1.3);
    }
    compute_profile(g, r, a, in, rem_active);
    o->more_drag = in->phase >= PH_DESCENT && in->idle_speed_descent && o->vnav_valid && o->vdev_ft > 500.0;
    if (o->vnav_valid) {
        if (in->phase == PH_DESCENT && in->nav_engaged && o->dist_to_dest_nm <= o->decel_dist_nm) g->decel_latched = 1;
        point_at_dist(o, r, o->tod_dist_nm, in, &o->tod_lat, &o->tod_lon, &o->tod_leg);
        point_at_dist(o, r, o->decel_dist_nm, in, &o->decel_lat, &o->decel_lon, &o->decel_leg);
    }
    o->decel_reached = g->decel_latched;
    /* T/C from the current climb rate (port estimate) */
    if (in->phase <= PH_CLIMB && in->cruise_alt_ft > in->alt_ft + 100.0 && in->vs_fpm > 300.0 && in->gs_kt > 60.0)
        o->toc_dist_from_ppos_nm = (in->cruise_alt_ft - in->alt_ft) / in->vs_fpm * in->gs_kt / 60.0;
    hold_speed_update(g, r, a, in);
    managed_targets(g, fp, in);
    pseudo_waypoints(g, r, a, in, rem_active);
}

size_t a380x_fmgc_nd_pwp_json(const A380XFmgcNdPwp *list, int count, char *out, size_t cap)
{
    static const int colour_flag[4] = {0, 131072, 32768, 65536};   /* white, magenta, cyan, amber */
    size_t n = 0;
    int i, first = 1, w;
    if (!out || cap < 3) return 0;
#define PWP_PUT(...) do { w = snprintf(out + n, cap - n, __VA_ARGS__); if (w < 0 || (size_t)w >= cap - n) goto overflow; n += (size_t)w; } while (0)
    PWP_PUT("[");
    for (i = 0; list && i < count && i < A380X_FMGC_MAX_ND_PWP; ++i) {
        const A380XFmgcNdPwp *p = &list[i];
        char ident[sizeof p->ident];
        size_t k, m = 0;
        if (p->nd == A380X_PWP_ND_NONE || !isfinite(p->dist_ahead_nm)) continue;
        if (p->nd == A380X_PWP_ND_PATH && !fin2(p->lat, p->lon)) continue;
        for (k = 0; k < sizeof p->ident && p->ident[k]; ++k)   /* labels are fixed: keep JSON-safe characters only */
            if ((p->ident[k] >= 'A' && p->ident[k] <= 'Z') || (p->ident[k] >= '0' && p->ident[k] <= '9') || strchr("()/ -", p->ident[k]))
                ident[m++] = p->ident[k];
        ident[m < sizeof ident ? m : sizeof ident - 1] = 0;
        PWP_PUT("%s{\"ident\":\"%s\",\"pwp\":%u,\"type\":%d", first ? "" : ",", ident, p->symbol, colour_flag[(unsigned)p->colour & 3u]);
        if (p->nd == A380X_PWP_ND_PATH) PWP_PUT(",\"lat\":%.5f,\"lon\":%.5f", p->lat, p->lon);
        PWP_PUT(",\"dist\":%.1f", p->dist_ahead_nm);
        if (isfinite(p->alt_ft)) PWP_PUT(",\"alt\":%.0f", p->alt_ft);
        PWP_PUT("}");
        first = 0;
    }
    PWP_PUT("]");
#undef PWP_PUT
    return n;
overflow:
    snprintf(out, cap, "[]");
    return 0;
}

/* ============================================================================================ */
/* MFD predictions                                                                              */
/* ============================================================================================ */

void a380x_fmgc_predictions(const A380XFmgc *g, const A380XFlightPlans *fp, const A380XFmgcInputs *in,
                            A380XFplnPredFeed *feed, A380XFplnWptPred *legs)
{
    const A380XFmgcOutputs *o = &g->out;
    const A380XFplnPlan *p = fp ? a380x_fpln_plan(fp, A380X_PLAN_ACTIVE) : NULL;
    const A380XFplnRoute *r;
    double cum = 0.0, llat, llon, gs = in->gs_kt > 120.0 ? in->gs_kt : 250.0, crz;
    int k, a, dleg;
    memset(feed, 0, sizeof *feed);
    feed->hold_speed_kt = NAN;
    feed->hold_leg_index = -1;
    if (!p || !in->position_valid || p->active_leg_index < 0) return;
    r = &p->pri;
    a = p->active_leg_index;
    dleg = r->destination_leg_index;
    crz = in->cruise_alt_ft > 0 ? in->cruise_alt_ft : in->alt_ft;
    memset(legs, 0, sizeof(*legs) * A380X_FPLN_MAX_ELEMENTS);
    llat = in->lat_deg; llon = in->lon_deg;
    for (k = 0; k < r->count; ++k) {
        legs[k].altitude_ft = legs[k].speed = legs[k].efob_kg = NAN;
        legs[k].dist_from_aircraft_nm = NAN;
        legs[k].alt_cstr_met = legs[k].spd_cstr_met = 1;
    }
    for (k = a; k < r->count && k < A380X_FPLN_MAX_ELEMENTS; ++k) {
        const A380XFplnLeg *l = &r->el[k];
        double len, alt = NAN, lo, hi, d2d;
        if (!is_leg(l)) continue;
        if (k >= r->first_missed_appr_index) break;
        if (k == a) len = isfinite(o->dtg_active_nm) ? o->dtg_active_nm : (fin2(l->term_lat, l->term_lon) ? a380x_geo_distance_nm(llat, llon, l->term_lat, l->term_lon) : NAN);
        else len = leg_length(r, k, &llat, &llon);
        if (k == a && fin2(l->term_lat, l->term_lon)) { llat = l->term_lat; llon = l->term_lon; }
        if (!isfinite(len)) break;
        cum += len;
        d2d = isfinite(o->dist_to_dest_nm) ? o->dist_to_dest_nm - cum : NAN;
        if (o->vnav_valid && isfinite(d2d) && isfinite(o->tod_dist_nm) && d2d <= o->tod_dist_nm && k <= dleg) {
            int j;
            for (j = 1; j < o->profile_count; ++j)
                if (o->profile[j].leg_index == k) { alt = o->profile[j].alt_ft; break; }
            if (k == dleg) alt = o->profile[0].alt_ft;
        } else if (in->phase <= PH_CLIMB && in->vs_fpm > 300.0) {
            alt = fmin(crz, in->alt_ft + in->vs_fpm * cum / gs * 60.0);
        } else {
            alt = crz;
        }
        legs[k].valid = 1;
        legs[k].seconds_from_present = (float)(cum / gs * 3600.0);
        legs[k].altitude_ft = (float)alt;
        legs[k].dist_from_aircraft_nm = (float)cum;
        if (isfinite(alt) && cstr_window(l, &lo, &hi))
            legs[k].alt_cstr_met = !((isfinite(lo) && alt < lo - 250.0) || (isfinite(hi) && alt > hi + 250.0));
    }
    feed->leg_pred = legs;
    feed->leg_pred_count = r->count;
    /* pseudo waypoints: the FMGC list (nearest first, same distances as the ND symbols), placed by
     * leg and distance in the plan's cumulative distance reference */
    for (k = 0; k < o->nd_pwp_count && feed->pwp_count < A380X_FPLN_MAX_PWP; ++k) {
        const A380XFmgcNdPwp *w = &o->nd_pwp[k];
        A380XFplnPwp *pp;
        if (!w->mfd || w->leg_index < a || w->leg_index >= r->count || !isfinite(r->el[w->leg_index].cum_dist_nm)) continue;
        pp = &feed->pwp[feed->pwp_count++];
        memset(pp, 0, sizeof *pp);
        snprintf(pp->ident, sizeof pp->ident, "%s", w->ident);
        pp->along_leg_index = w->leg_index;
        pp->dist_from_start_nm = (float)(r->el[w->leg_index].cum_dist_nm - w->before_term_nm);
        pp->pred.valid = 1;
        pp->pred.seconds_from_present = (float)(w->dist_ahead_nm / gs * 3600.0);
        pp->pred.altitude_ft = (float)w->alt_ft;
        pp->pred.speed = (float)w->speed_kt;
        pp->pred.efob_kg = NAN;
        pp->pred.dist_from_aircraft_nm = (float)w->dist_ahead_nm;
        pp->pred.alt_cstr_met = pp->pred.spd_cstr_met = 1;
    }
    feed->hold_speed_kt = (float)o->hold_speed_kt;
    feed->hold_leg_index = o->hold_leg_index;
    feed->hold_decel_reached = o->hold_decel_reached;
}
