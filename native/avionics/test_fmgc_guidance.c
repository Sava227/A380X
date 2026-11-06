/* SPDX-License-Identifier: GPL-3.0-or-later
 * FMGC managed guidance unit tests: leg geometry, sequencing, transitions, holds, descent profile,
 * managed altitude/speed. Synthetic routes go through the real flight-plan geometry
 * (a380x_fpln_recompute) with an empty navigation database (zero magnetic variation).
 * Lateral dynamics: kinematic point mass with an FBW-HPATH-like bank law (bounded, rate limited);
 * the real FBW law and aircraft response are exercised by test_fmgc_flight.c.
 */
#include "a380x_fmgc_guidance.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures, checks;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
#define D2R (3.14159265358979323846 / 180.0)

static A380XNavdata *nav;
static A380XFlightPlans *fp;
static A380XFplnRoute *R;

static int zero_magvar(void *ctx, double lat, double lon, double *out) { (void)ctx; (void)lat; (void)lon; *out = 0; return 1; }

static void plan_begin(void)
{
    A380XFplnPlan *p;
    a380x_fpln_reset(fp);
    p = &fp->plan[A380X_PLAN_ACTIVE];
    memset(p, 0, sizeof *p);
    p->exists = 1;
    R = &p->pri;
    R->count = 0;
}

static A380XFplnLeg *add(A380XLegType t, const char *id, double lat, double lon)
{
    A380XFplnLeg *l = &R->el[R->count++];
    memset(l, 0, sizeof *l);
    l->type = t;
    l->segment = A380X_SEG_ENROUTE;
    snprintf(l->fix.ident, sizeof l->fix.ident, "%s", id ? id : "");
    snprintf(l->ident, sizeof l->ident, "%s", id ? id : "");
    l->fix.lat = id ? lat : NAN; l->fix.lon = id ? lon : NAN;
    l->navaid.lat = l->navaid.lon = l->arc_center.lat = l->arc_center.lon = NAN;
    l->course_deg = l->theta_deg = l->rho_nm = l->arc_radius_nm = l->rnp_nm = NAN;
    l->length_nm = l->length_min = l->vertical_angle_deg = l->trans_alt_ft = NAN;
    l->alt_db.alt1_ft = l->alt_db.alt2_ft = l->alt_pilot.alt1_ft = l->alt_pilot.alt2_ft = NAN;
    l->spd_db.speed = l->spd_pilot.speed = NAN;
    return l;
}

static void disco(void)
{
    A380XFplnLeg *l = &R->el[R->count++];
    memset(l, 0, sizeof *l);
    l->flags = A380X_LEGF_DISCONTINUITY;
}

static void plan_end(int active)
{
    a380x_fpln_recompute(fp, A380X_PLAN_ACTIVE);
    fp->plan[A380X_PLAN_ACTIVE].active_leg_index = active;
    R->dest_elev_ft = 0;
}

/* ---- kinematic aircraft ---- */
typedef struct { double lat, lon, trk, gs, bank, alt, vs; } Ac;

static void inputs(A380XFmgcInputs *in, const Ac *a, int nav_engaged)
{
    memset(in, 0, sizeof *in);
    in->position_valid = 1;
    in->lat_deg = a->lat; in->lon_deg = a->lon;
    in->true_track_deg = in->true_heading_deg = a->trk;
    in->gs_kt = in->tas_kt = a->gs; in->cas_kt = a->gs;
    in->alt_ft = a->alt; in->vs_fpm = a->vs;
    in->fms_valid = 1; in->phase = 3;
    in->cruise_alt_ft = 30000; in->fcu_alt_ft = a->alt;
    in->nav_engaged = nav_engaged;
    in->des_limit_kt = 250; in->des_limit_alt_ft = 10000; in->descent_speed_kt = 300;
}

/* FBW-HPATH-like law: phi = ff + Kv*tae + K*xtk (same structure and gains as A380FgOuterLoops case 3). */
static void fly(Ac *a, const A380XFmgcOutputs *o, double dt)
{
    double k2v = 0.004451 * a->gs, phi = 0, rate;
    double xt = o->fbw_xtk_nm * 6839.0 / a->gs;
    if (xt > 45) xt = 45;
    if (xt < -45) xt = -45;
    if (o->lnav_valid) phi = o->bank_ff_deg + (o->fbw_tae_deg + xt) * k2v;
    if (phi > 25) phi = 25;
    if (phi < -25) phi = -25;
    rate = (phi - a->bank) / 1.5;
    if (rate > 5) rate = 5;
    if (rate < -5) rate = -5;
    a->bank += rate * dt;
    a->trk = fmod(a->trk + 9.80665 * tan(a->bank * D2R) / (a->gs * 0.514444) / D2R * dt + 360.0, 360.0);
    a380x_geo_destination(a->lat, a->lon, a->trk, a->gs * dt / 3600.0, &a->lat, &a->lon);
    a->alt += a->vs * dt / 60.0;
}

static void test_radius(void)
{
    double r = a380x_fmgc_turn_radius_nm(250, 20);
    CHECK(fabs(r - 2.50) < 0.03);
    CHECK(fabs(a380x_fmgc_anticipation_nm(250, 90) - a380x_fmgc_turn_radius_nm(250, A380X_FMGC_DESIGN_BANK_DEG)) < 1e-9);
    CHECK(a380x_fmgc_anticipation_nm(250, 0.5) == 0.0);
    CHECK(fabs(a380x_fmgc_idle_gradient_ft_nm(5000, 10000) - tan(3.0 * D2R) * 6076.115) < 1e-6);
}

/* TF -> TF 90 deg fly-by: sequencing at R*tan(45), transition arc, path tracking. */
static void test_flyby(void)
{
    A380XFmgc g; A380XFmgcInputs in; Ac a = {0.0, 0.2, 90.0, 250.0, 0, 20000, 0};
    double max_xtk = 0, seq_dtg = NAN, max_after = 0;
    int t, seq = 0, in_arc = 0;
    plan_begin();
    add(A380X_LEG_IF, "AAA", 0.0, 0.0);
    add(A380X_LEG_TF, "BBB", 0.0, 1.0);
    add(A380X_LEG_TF, "CCC", 1.0, 1.0);
    plan_end(1);
    a380x_fmgc_init(&g);
    for (t = 0; t < 4000; ++t) {
        double before;
        inputs(&in, &a, 1);
        before = a380x_geo_distance_nm(a.lat, a.lon, 0.0, 1.0);
        a380x_fmgc_update(&g, fp, &in, 0.5);
        if (g.out.sequenced && !seq) { seq = 1; seq_dtg = before; }
        if (g.out.in_transition) in_arc = 1;
        CHECK(g.out.lnav_valid);
        if (fabs(g.out.xtk_right_nm) > max_xtk) max_xtk = fabs(g.out.xtk_right_nm);
        if (seq && !g.out.in_transition && t % 10 == 0 && a.lat > 0.1 && fabs(g.out.xtk_right_nm) > max_after) max_after = fabs(g.out.xtk_right_nm);
        fly(&a, &g.out, 0.5);
        if (a.lat > 0.8) break;
    }
    printf("fly-by 90deg: sequenced %.2f NM before the fix (anticipation %.2f), max |xtk| %.3f NM, after turn %.3f NM\n",
           seq_dtg, a380x_fmgc_anticipation_nm(250, 90), max_xtk, max_after);
    CHECK(seq && in_arc);
    CHECK(fabs(seq_dtg - a380x_fmgc_anticipation_nm(250, 90)) < 0.15);
    CHECK(max_xtk < 0.35);
    CHECK(max_after < 0.05);
    CHECK(fp->plan[0].active_leg_index == 2);
}

/* Discontinuity: NAV is lost once the TO waypoint before it is passed; no sequencing past it. */
static void test_discontinuity(void)
{
    A380XFmgc g; A380XFmgcInputs in; Ac a = {0.0, 0.5, 90.0, 250.0, 0, 20000, 0};
    int t, lost_at = -1;
    plan_begin();
    add(A380X_LEG_IF, "AAA", 0.0, 0.0);
    add(A380X_LEG_TF, "BBB", 0.0, 1.0);
    disco();
    add(A380X_LEG_IF, "CCC", 1.0, 2.0);
    plan_end(1);
    a380x_fmgc_init(&g);
    for (t = 0; t < 2000; ++t) {
        inputs(&in, &a, 1);
        a380x_fmgc_update(&g, fp, &in, 0.5);
        if (!g.out.lnav_valid && lost_at < 0) lost_at = t;
        if (lost_at >= 0) break;
        fly(&a, &g.out, 0.5);
    }
    CHECK(lost_at > 0 && g.out.discontinuity && !g.out.nav_capture);
    CHECK(fp->plan[0].active_leg_index == 1);
    CHECK(a.lon >= 0.999);   /* the waypoint itself was reached (no anticipation before a discontinuity) */
}

/* Course / heading legs: track and heading targets, altitude / DME / radial / intercept terminations. */
static void test_course_and_heading_legs(void)
{
    A380XFmgc g; A380XFmgcInputs in; Ac a = {0.0, 0.0, 80.0, 200.0, 0, 500, 2000};
    A380XFplnLeg *l;
    int t;
    plan_begin();
    add(A380X_LEG_IF, "RW09", 0.0, 0.0);
    l = add(A380X_LEG_VA, NULL, 0, 0); l->course_deg = 100; l->alt_db.desc = '+'; l->alt_db.alt1_ft = 1500; l->cstr_type = A380X_CSTR_CLB;
    l = add(A380X_LEG_CD, NULL, 0, 0); l->course_deg = 90; l->length_nm = 10;
    l->navaid.lat = 0.0; l->navaid.lon = 0.0; snprintf(l->navaid.ident, sizeof l->navaid.ident, "DME");
    l = add(A380X_LEG_CR, NULL, 0, 0); l->course_deg = 90; l->theta_deg = 60;
    l->navaid.lat = -0.5; l->navaid.lon = 0.0; snprintf(l->navaid.ident, sizeof l->navaid.ident, "VOR");
    l = add(A380X_LEG_CI, NULL, 0, 0); l->course_deg = 45;
    l = add(A380X_LEG_CF, "FIX", 0.6, 1.2); l->course_deg = 0;
    plan_end(1);
    a380x_fmgc_init(&g);
    inputs(&in, &a, 1);
    a380x_fmgc_update(&g, fp, &in, 0.1);
    CHECK(g.out.kind == A380X_LNAV_HEADING && fabs(g.out.fbw_tae_deg - 20.0) < 1e-6 && g.out.fbw_xtk_nm == 0.0);
    CHECK(fabs(g.out.managed_heading_mag_deg - 100.0) < 1e-6 && g.out.nav_capture);
    for (t = 0; t < 30000 && fp->plan[0].active_leg_index < 5; ++t) {
        int before = fp->plan[0].active_leg_index;
        inputs(&in, &a, 1);
        a380x_fmgc_update(&g, fp, &in, 0.2);
        if (fp->plan[0].active_leg_index != before) {
            double dme = a380x_geo_distance_nm(0, 0, a.lat, a.lon);
            double radial = a380x_geo_bearing_deg(-0.5, 0.0, a.lat, a.lon);
            if (before == 1) CHECK(a.alt >= 1450);                        /* VA to 1500 ft */
            if (before == 2) CHECK(fabs(dme - 10.0) < 0.2);               /* CD 10 NM */
            if (before == 3) CHECK(fabs(a380x_geo_angle_diff(60.0, radial)) < 0.5); /* CR radial 060 */
            printf("  leg %d -> %d at alt %.0f dme %.2f radial(VOR) %.1f track %.1f\n", before, fp->plan[0].active_leg_index, a.alt, dme, radial, a.trk);
        }
        if (fp->plan[0].active_leg_index == 2) CHECK(g.out.kind == A380X_LNAV_TRACK);
        fly(&a, &g.out, 0.2);
        if (a.alt > 3000) a.vs = 0;
    }
    CHECK(fp->plan[0].active_leg_index == 5);
    /* CI sequenced inside the CF capture zone: the CF course line is then intercepted */
    for (t = 0; t < 3000; ++t) { inputs(&in, &a, 1); a380x_fmgc_update(&g, fp, &in, 0.2); fly(&a, &g.out, 0.2); }
    CHECK(fabs(g.out.xtk_right_nm) < 0.1 || fp->plan[0].active_leg_index > 5);
}

/* RF arc: radius / side / feed-forward. */
static void test_rf_arc(void)
{
    A380XFmgc g; A380XFmgcInputs in;
    Ac a;
    A380XFplnLeg *l;
    double max_xtk = 0;
    int t;
    plan_begin();
    add(A380X_LEG_IF, "S", 0.0, 0.0);
    /* quarter circle of 5 NM radius around (0, 5/60): from (0,0) clockwise (right turn) to (5/60, 5/60) */
    l = add(A380X_LEG_RF, "E", 5.0 / 60.0, 5.0 / 60.0);
    l->arc_center.lat = 0.0; l->arc_center.lon = 5.0 / 60.0; l->arc_radius_nm = 5.0; l->turn = A380X_TURN_RIGHT;
    snprintf(l->arc_center.ident, sizeof l->arc_center.ident, "CTR");
    add(A380X_LEG_TF, "N", 1.0, 5.0 / 60.0);
    plan_end(1);
    a.lat = 0.0; a.lon = 0.0; a.trk = 0.0; a.gs = 220; a.bank = 0; a.alt = 3000; a.vs = 0;
    a380x_fmgc_init(&g);
    for (t = 0; t < 2000 && fp->plan[0].active_leg_index == 1; ++t) {
        inputs(&in, &a, 1);
        a380x_fmgc_update(&g, fp, &in, 0.25);
        /* required bank for 5 NM at 220 kt: atan(V^2 / (g R)) = 8.0 deg */
        if (t == 2) CHECK(g.out.kind == A380X_LNAV_ARC && fabs(g.out.bank_ff_deg - 8.0) < 0.3);
        if (getenv("FMGC_DEBUG") && t % 20 == 0) printf("    rf t=%d xtk %.3f tae %.2f bank %.1f trk %.1f\n", t, g.out.xtk_right_nm, g.out.fbw_tae_deg, a.bank, a.trk);
        if (fabs(g.out.xtk_right_nm) > max_xtk) max_xtk = fabs(g.out.xtk_right_nm);
        fly(&a, &g.out, 0.25);
    }
    printf("RF 5 NM arc at 220 kt: max |xtk| %.3f NM, sequenced at track %.1f\n", max_xtk, a.trk);
    CHECK(fp->plan[0].active_leg_index == 2 && max_xtk < 0.15 && fabs(a380x_geo_angle_diff(a.trk, 90.0)) < 5.0);
}

/* HM hold: direct entry at the fix, racetrack tracking, IMMEDIATE EXIT. */
static void test_hold(void)
{
    A380XFmgc g; A380XFmgcInputs in; Ac a = {-0.3, 0.0, 0.0, 230.0, 0, 8000, 0};
    A380XFplnLeg *l;
    int t, phases[5] = {0}, entered = 0, circuits = 0;
    double max_xtk = 0;
    plan_begin();
    add(A380X_LEG_IF, "IAF", -0.5, 0.0);
    add(A380X_LEG_TF, "HLD", 0.0, 0.0);
    l = add(A380X_LEG_HM, "HLD", 0.0, 0.0);
    l->course_deg = 0; l->turn = A380X_TURN_RIGHT; l->length_min = 1.0;
    add(A380X_LEG_TF, "OUT", 0.5, 0.5);
    plan_end(1);
    a380x_fmgc_init(&g);
    for (t = 0; t < 12000; ++t) {
        inputs(&in, &a, 1);
        a380x_fmgc_update(&g, fp, &in, 0.25);
        if (fp->plan[0].active_leg_index == 2) {
            entered = 1;
            phases[g.hold_phase < 5 ? g.hold_phase : 0]++;
            if (g.hold_circuits >= 1 && fabs(g.out.xtk_right_nm) > max_xtk) max_xtk = fabs(g.out.xtk_right_nm);
            if (g.hold_circuits == 3) fp->plan[0].pri.el[2].flags |= A380X_LEGF_HOLD_IMM_EXIT;
            if (g.hold_circuits > circuits) circuits = g.hold_circuits;
        }
        if (fp->plan[0].active_leg_index == 3) break;
        fly(&a, &g.out, 0.25);
    }
    printf("HM 1 min right hold at 230 kt: radius %.2f NM leg %.2f NM, circuits %d, max |xtk| after first circuit %.3f NM\n",
           g.hold_radius_nm, g.hold_leg_nm, circuits, max_xtk);
    CHECK(entered && phases[0] && phases[1] && phases[2] && phases[3]);
    CHECK(fp->plan[0].active_leg_index == 3 && circuits >= 3);
    CHECK(max_xtk < 0.5);
}

/* Hold entry sectors (right turns, d = arrival track - inbound course): direct -110..+70, teardrop
 * +70..+140, parallel +140..-110; +-5 deg: direct kept at its boundaries, teardrop at +140. */
static void test_hold_entry_sectors(void)
{
    static const struct { double inbound, arrival; int right; A380XHoldEntry e; } c[] = {
        {0, 0, 1, A380X_HOLD_ENTRY_DIRECT}, {0, 70, 1, A380X_HOLD_ENTRY_DIRECT}, {0, 74, 1, A380X_HOLD_ENTRY_DIRECT},
        {0, 76, 1, A380X_HOLD_ENTRY_TEARDROP}, {0, 110, 1, A380X_HOLD_ENTRY_TEARDROP}, {0, 140, 1, A380X_HOLD_ENTRY_TEARDROP},
        {0, 144, 1, A380X_HOLD_ENTRY_TEARDROP}, {0, 146, 1, A380X_HOLD_ENTRY_PARALLEL}, {0, 180, 1, A380X_HOLD_ENTRY_PARALLEL},
        {0, 200, 1, A380X_HOLD_ENTRY_PARALLEL}, {0, 244, 1, A380X_HOLD_ENTRY_PARALLEL}, {0, 246, 1, A380X_HOLD_ENTRY_DIRECT},
        {0, 250, 1, A380X_HOLD_ENTRY_DIRECT}, {0, 270, 1, A380X_HOLD_ENTRY_DIRECT},
        /* left turns: mirrored */
        {0, 286, 0, A380X_HOLD_ENTRY_DIRECT}, {0, 284, 0, A380X_HOLD_ENTRY_TEARDROP}, {0, 250, 0, A380X_HOLD_ENTRY_TEARDROP},
        {0, 216, 0, A380X_HOLD_ENTRY_TEARDROP}, {0, 214, 0, A380X_HOLD_ENTRY_PARALLEL}, {0, 160, 0, A380X_HOLD_ENTRY_PARALLEL},
        {0, 116, 0, A380X_HOLD_ENTRY_PARALLEL}, {0, 114, 0, A380X_HOLD_ENTRY_DIRECT}, {0, 90, 0, A380X_HOLD_ENTRY_DIRECT},
        /* other inbound courses */
        {90, 200, 1, A380X_HOLD_ENTRY_TEARDROP}, {90, 290, 1, A380X_HOLD_ENTRY_PARALLEL}, {90, 0, 1, A380X_HOLD_ENTRY_DIRECT},
        {270, 20, 1, A380X_HOLD_ENTRY_TEARDROP}, {270, 20, 0, A380X_HOLD_ENTRY_DIRECT}, {315, 100, 0, A380X_HOLD_ENTRY_PARALLEL},
    };
    int i, ok = 0;
    for (i = 0; i < (int)(sizeof c / sizeof c[0]); ++i) {
        A380XHoldEntry e = a380x_fmgc_hold_entry(c[i].inbound, c[i].arrival, c[i].right);
        CHECK(e == c[i].e);
        if (e != c[i].e) printf("  entry inbound %.0f arrival %.0f %s: got %d expected %d\n", c[i].inbound, c[i].arrival,
                                c[i].right ? "R" : "L", (int)e, (int)c[i].e);
        else ++ok;
    }
    printf("hold entry sectors: %d/%d cases (incl. +-5 deg boundaries, left turns)\n", ok, (int)(sizeof c / sizeof c[0]));
}

/* Flown entry into an HM (inbound 360 at HLD, 1 min legs, 230 kt) from an arrival track: entry type,
 * phases, excursions, established inbound after the entry, IMM EXIT after two circuits. */
static void fly_hold_entry(double arrival, int right, A380XHoldEntry expect, const char *name)
{
    A380XFmgc g; A380XFmgcInputs in; Ac a;
    A380XFplnLeg *l;
    double slat, slon, s = right ? 1.0 : -1.0, max_nonhold = 0, max_far = 0, max_xtk_in = 0, max_hold_side = 0;
    int t, seen[10] = {0}, circuits = 0, exited = 0;
    A380XHoldEntry entry = A380X_HOLD_ENTRY_NONE;
    a380x_geo_destination(0.0, 0.0, fmod(arrival + 180.0, 360.0), 25.0, &slat, &slon);
    plan_begin();
    add(A380X_LEG_IF, "STA", slat, slon);
    add(A380X_LEG_TF, "HLD", 0.0, 0.0);
    l = add(A380X_LEG_HM, "HLD", 0.0, 0.0);
    l->course_deg = 0; l->course_true = 1; l->turn = right ? A380X_TURN_RIGHT : A380X_TURN_LEFT; l->length_min = 1.0;
    add(A380X_LEG_TF, "OUT", 0.4, 0.0);
    plan_end(1);
    a.lat = slat; a.lon = slon; a.trk = arrival; a.gs = 230; a.bank = 0; a.alt = 8000; a.vs = 0;
    a380x_fmgc_init(&g);
    for (t = 0; t < 16000; ++t) {
        inputs(&in, &a, 1);
        a380x_fmgc_update(&g, fp, &in, 0.25);
        if (fp->plan[0].active_leg_index == 2) {
            double e = a.lon * 60.0, n = a.lat * 60.0, side = s * e;   /* NM about the fix, + holding side */
            if (g.hold_entry != A380X_HOLD_ENTRY_NONE && entry == A380X_HOLD_ENTRY_NONE) entry = g.hold_entry;
            seen[g.hold_phase]++;
            if (-side > max_nonhold) max_nonhold = -side;
            if (side > max_hold_side) max_hold_side = side;
            if (hypot(e, n) > max_far) max_far = hypot(e, n);
            if (g.hold_circuits >= 1 && g.hold_phase == 0 && n < -0.5 && fabs(g.out.xtk_right_nm) > max_xtk_in)
                max_xtk_in = fabs(g.out.xtk_right_nm);
            if (g.hold_circuits > circuits) circuits = g.hold_circuits;
            if (g.hold_circuits == 2) fp->plan[0].pri.el[2].flags |= A380X_LEGF_HOLD_IMM_EXIT;
        }
        if (fp->plan[0].active_leg_index == 3) { exited = 1; break; }
        fly(&a, &g.out, 0.25);
    }
    printf("  %-22s arrival %03.0f %s: entry %d, phases d%d/%d/%d/%d t%d/%d p%d/%d/%d, circuits %d, non-holding side %.2f NM,"
           " holding side %.2f NM, max %.1f NM from the fix, inbound |xtk| %.3f NM, exit %d\n",
           name, arrival, right ? "R" : "L", (int)entry, seen[1], seen[2], seen[3], seen[0], seen[5], seen[6], seen[7], seen[8],
           seen[9], circuits, max_nonhold, max_hold_side, max_far, max_xtk_in, exited);
    CHECK(entry == expect);
    if (expect == A380X_HOLD_ENTRY_TEARDROP) CHECK(seen[5] && seen[6] && !seen[7] && !seen[9]);
    if (expect == A380X_HOLD_ENTRY_PARALLEL) CHECK(seen[7] && seen[8] && seen[9] && !seen[5]);
    if (expect == A380X_HOLD_ENTRY_DIRECT) CHECK(!seen[5] && !seen[7] && !seen[9]);
    CHECK(seen[0] && seen[1] && seen[2] && seen[3]);
    CHECK(exited && circuits >= 2);
    CHECK(max_xtk_in < 0.25);                                  /* established inbound after the entry */
    CHECK(max_far < g.hold_leg_nm + 2.0 * g.hold_radius_nm + 2.5);
    CHECK(max_hold_side < 2.0 * g.hold_radius_nm + 1.5);
    /* direct / teardrop stay on the holding side (within the turn overshoot); the parallel entry is
     * flown on the non-holding side, up to about one turn diameter */
    CHECK(max_nonhold < (expect == A380X_HOLD_ENTRY_PARALLEL ? 2.0 * g.hold_radius_nm + 0.5 : g.hold_radius_nm + 0.6));
}

static void test_hold_entries_flown(void)
{
    puts("flown hold entries (HM HLD inbound 360, 1 min, 230 kt):");
    fly_hold_entry(0.0, 1, A380X_HOLD_ENTRY_DIRECT, "direct (straight in)");
    fly_hold_entry(270.0, 1, A380X_HOLD_ENTRY_DIRECT, "direct (abeam)");
    fly_hold_entry(110.0, 1, A380X_HOLD_ENTRY_TEARDROP, "teardrop");
    fly_hold_entry(200.0, 1, A380X_HOLD_ENTRY_PARALLEL, "parallel");
    fly_hold_entry(180.0, 1, A380X_HOLD_ENTRY_PARALLEL, "parallel (reciprocal)");
    fly_hold_entry(250.0, 0, A380X_HOLD_ENTRY_TEARDROP, "teardrop (left)");
    fly_hold_entry(160.0, 0, A380X_HOLD_ENTRY_PARALLEL, "parallel (left)");
}

/* FBW getHoldingSpeed: green dot (+25 engine out), approach speeds, constraint, ICAO caps, Mach
 * 0.83 above FL340, speed limits, VLS floor. */
static void test_holding_speed(void)
{
    A380XFmgcInputs in;
    memset(&in, 0, sizeof in);
    in.phase = 3; in.green_dot_kt = 210; in.vls_kt = 150; in.clb_limit_kt = 250; in.clb_limit_alt_ft = 10000;
    in.des_limit_kt = 250; in.des_limit_alt_ft = 10000;
    in.fcu_alt_ft = 10000;
    CHECK(a380x_fmgc_holding_speed(&in, NAN, NAN) == 210);
    in.green_dot_kt = 245;
    CHECK(a380x_fmgc_holding_speed(&in, NAN, NAN) == 230);                     /* below 14000: 230 */
    CHECK(a380x_fmgc_holding_speed(&in, 200, NAN) == 200);                     /* speed constraint */
    in.fcu_alt_ft = 15000;
    CHECK(a380x_fmgc_holding_speed(&in, NAN, NAN) == 240);
    CHECK(a380x_fmgc_holding_speed(&in, NAN, 21000) == 245);                   /* constraint altitude above the FCU */
    in.fcu_alt_ft = 25000; in.engine_out = 1;
    CHECK(a380x_fmgc_holding_speed(&in, NAN, NAN) == 265);                     /* 245 + 25, cap 265 */
    in.engine_out = 0; in.green_dot_kt = 280; in.fcu_alt_ft = 37000;
    {
        double v = a380x_fmgc_holding_speed(&in, NAN, NAN);                    /* M0.83 at FL370 ~ 271 KCAS */
        CHECK(v >= 269 && v <= 273);
    }
    in.fcu_alt_ft = 8000; in.green_dot_kt = 240; in.phase = 4; in.des_limit_kt = 220;
    CHECK(a380x_fmgc_holding_speed(&in, NAN, NAN) == 220);                     /* descent speed limit */
    in.vls_kt = 225;
    CHECK(a380x_fmgc_holding_speed(&in, NAN, NAN) == 225);                     /* VLS floor */
    in.vls_kt = 130; in.phase = 5; in.flap_lever = 1; in.s_speed_kt = 185; in.vapp_kt = 140;
    CHECK(a380x_fmgc_holding_speed(&in, NAN, NAN) == 185);                     /* approach: S speed in CONF 1 */
}

/* Holding speed as the managed target: decelerating for the next hold from decelDist before its fix,
 * the hold speed in the hold, the MFD HOLD line feed. */
static void test_hold_speed_target(void)
{
    A380XFmgc g; A380XFmgcInputs in; Ac a = {0.0, 0.75, 90.0, 300.0, 0, 10000, 0};
    A380XFplnPredFeed feed; static A380XFplnWptPred legs[A380X_FPLN_MAX_ELEMENTS];
    A380XFplnLeg *l;
    plan_begin();
    add(A380X_LEG_IF, "AAA", 0.0, 0.0);
    add(A380X_LEG_TF, "HLD", 0.0, 1.0);
    l = add(A380X_LEG_HM, "HLD", 0.0, 1.0);
    l->course_deg = 90; l->course_true = 1; l->turn = A380X_TURN_RIGHT; l->length_min = 1.0;
    add(A380X_LEG_TF, "OUT", 0.0, 2.0);
    plan_end(1);
    a380x_fmgc_init(&g);
    inputs(&in, &a, 1); in.phase = 3; in.green_dot_kt = 210; in.fcu_alt_ft = 10000; in.clb_limit_kt = 250; in.clb_limit_alt_ft = 10000;
    a380x_fmgc_update(&g, fp, &in, 0.1);       /* 15 NM before the fix, decelDist (300 - 210) * 0.15 = 13.5 NM */
    CHECK(g.out.hold_leg_index == 2 && g.out.hold_speed_kt == 210 && !g.out.hold_decel_reached && !g.out.speed_valid);
    a.lon = 0.8; inputs(&in, &a, 1); in.phase = 3; in.green_dot_kt = 210; in.fcu_alt_ft = 10000;
    a380x_fmgc_update(&g, fp, &in, 0.1);       /* 12 NM */
    CHECK(g.out.hold_decel_reached && g.out.speed_valid && g.out.speed_kt == 210 && !g.out.speed_is_mach);
    a380x_fmgc_predictions(&g, fp, &in, &feed, legs);
    CHECK(feed.hold_leg_index == 2 && feed.hold_speed_kt == 210 && feed.hold_decel_reached);
    in.phase = 5; in.vapp_kt = 140; in.flap_lever = 2; in.f_speed_kt = 170;
    a380x_fmgc_update(&g, fp, &in, 0.1);       /* approach phase: approach speeds, not the hold speed */
    CHECK(g.out.speed_valid && g.out.speed_kt == 170);
    /* in the hold */
    fp->plan[0].active_leg_index = 2; a.lon = 1.0;
    inputs(&in, &a, 1); in.phase = 4; in.green_dot_kt = 210; in.fcu_alt_ft = 10000;
    a380x_fmgc_update(&g, fp, &in, 0.1);
    CHECK(g.out.hold_leg_index == 2 && g.out.hold_decel_reached && g.out.speed_valid && g.out.speed_kt == 210);
    CHECK(g.out.hold_entry == A380X_HOLD_ENTRY_DIRECT);
}

static const A380XFmgcNdPwp *find_pwp(const A380XFmgcOutputs *o, const char *ident, unsigned symbol)
{
    int i;
    for (i = 0; i < o->nd_pwp_count; ++i)
        if (!strcmp(o->nd_pwp[i].ident, ident) && o->nd_pwp[i].symbol == symbol) return &o->nd_pwp[i];
    return NULL;
}

static const A380XFplnPwp *find_feed(const A380XFplnPredFeed *f, const char *ident)
{
    int i;
    for (i = 0; i < f->pwp_count; ++i) if (!strcmp(f->pwp[i].ident, ident)) return &f->pwp[i];
    return NULL;
}

static void pwp_dump(const char *what, const A380XFmgcOutputs *o)
{
    int i;
    printf("  %s:", what);
    for (i = 0; i < o->nd_pwp_count; ++i)
        printf(" %s[sym %u col %d nd %d mfd %d %.1f NM]", o->nd_pwp[i].ident, o->nd_pwp[i].symbol, (int)o->nd_pwp[i].colour,
               (int)o->nd_pwp[i].nd, o->nd_pwp[i].mfd, o->nd_pwp[i].dist_ahead_nm);
    printf("\n");
}

/* Climb pseudo waypoints: T/C, level-off at the FCU / a constraint (managed magenta, selected cyan,
 * along the track in HDG), start of climb, climb speed limit; MFD lines at the same distances. */
static void test_pwp_climb(void)
{
    A380XFmgc g; A380XFmgcInputs in; Ac a = {0.0, 0.05, 90.0, 300.0, 0, 5000, 2000};
    A380XFplnPredFeed feed; static A380XFplnWptPred legs[A380X_FPLN_MAX_ELEMENTS];
    const A380XFmgcNdPwp *w, *w2;
    const A380XFplnPwp *f;
    A380XFplnLeg *l;
    const double toc5k = a380x_fmgc_climb_distance_nm(5000, 35000, 2000, 300);
    plan_begin();
    add(A380X_LEG_IF, "ORG", 0.0, 0.0);
    add(A380X_LEG_TF, "AAA", 0.0, 1.0);
    add(A380X_LEG_TF, "BBB", 0.0, 2.0);
    add(A380X_LEG_TF, "CCC", 0.0, 3.0);
    l = add(A380X_LEG_TF, "RW09", 0.0, 6.0); l->flags |= A380X_LEGF_DESTINATION; l->wp_desc = A380X_WPDESC_RUNWAY;
    l->segment = A380X_SEG_APPROACH; l->cstr_type = A380X_CSTR_DES;
    plan_end(1);
#define CLIMB_IN(vm_, nav_, fcu_) do { inputs(&in, &a, nav_); in.phase = 2; in.cruise_alt_ft = 35000; in.fcu_alt_ft = fcu_; \
        in.fg_vertical_mode = vm_; in.speed_managed = 1; in.clb_limit_kt = 250; in.clb_limit_alt_ft = 10000; } while (0)
    a380x_fmgc_init(&g);
    CLIMB_IN(A380X_FMGC_VM_CLB, 1, 39000);
    a380x_fmgc_update(&g, fp, &in, 0.1);
    pwp_dump("CLB, FCU above CRZ", &g.out);
    w = find_pwp(&g.out, "(T/C)", A380X_PWP_CLIMB_LEVEL_OFF);
    w2 = find_pwp(&g.out, "(SPDLIM)", A380X_PWP_SPEED_CHANGE);
    /* climb rate falling to zero at 43000 ft: 5000 -> FL350 at 2000 ft/min (from 2000 at 5000 to 421 at FL350) */
    CHECK(fabs(toc5k - 38000.0 / 2000.0 * log(38000.0 / 8000.0) * 5.0) < 1e-6 && toc5k > 75.0);
    CHECK(fabs(a380x_fmgc_climb_distance_nm(30000, 30100, 1500, 400) - 100.0 / 1500.0 * 400.0 / 60.0) < 0.01);
    CHECK(w && w->colour == A380X_PWP_MAGENTA && w->nd == A380X_PWP_ND_PATH && w->mfd && fabs(w->dist_ahead_nm - toc5k) < 0.01);
    CHECK(w && fabs(w->lat) < 0.01 && fabs(w->lon - (0.05 + toc5k / 60.0)) < 0.01 && w->leg_index == 3);
    CHECK(w2 && w2->colour == A380X_PWP_MAGENTA && w2->nd == A380X_PWP_ND_PATH
          && fabs(w2->dist_ahead_nm - a380x_fmgc_climb_distance_nm(5000, 10000, 2000, 300)) < 0.01 && w2->dist_ahead_nm > 12.5);
    CHECK(g.out.nd_pwp_count >= 2 && g.out.nd_pwp[0].dist_ahead_nm <= g.out.nd_pwp[1].dist_ahead_nm);
    a380x_fmgc_predictions(&g, fp, &in, &feed, legs);
    f = find_feed(&feed, "(T/C)");
    CHECK(f && f->along_leg_index == 3 && w && fabs(f->pred.dist_from_aircraft_nm - w->dist_ahead_nm) < 0.01
          && fabs(f->pred.altitude_ft - 35000) < 1);
    CHECK(feed.pwp_count >= 2 && !strcmp(feed.pwp[0].ident, "(SPDLIM)") && fabs(feed.pwp[0].pred.speed - 250) < 0.01);
    /* the MFD distance reference: cumulative distance of the leg minus the distance before its termination */
    CHECK(f && fabs(f->dist_from_start_nm - (fp->plan[0].pri.el[3].cum_dist_nm - (180.0 - (toc5k + 3.0)))) < 0.3);
    /* OP CLB to FCU FL200 in HDG: cyan level-off along the track, T/C and SPDLIM only on the MFD */
    CLIMB_IN(A380X_FMGC_VM_OP_CLB, 0, 20000);
    a380x_fmgc_update(&g, fp, &in, 0.1);
    pwp_dump("OP CLB FL200, HDG", &g.out);
    w = find_pwp(&g.out, "LEVEL", A380X_PWP_CLIMB_LEVEL_OFF);
    CHECK(w && w->colour == A380X_PWP_CYAN && w->nd == A380X_PWP_ND_TRACK && !w->mfd
          && fabs(w->dist_ahead_nm - a380x_fmgc_climb_distance_nm(5000, 20000, 2000, 300)) < 0.01);
    w = find_pwp(&g.out, "(T/C)", A380X_PWP_CLIMB_LEVEL_OFF);
    CHECK(w && w->nd == A380X_PWP_ND_NONE && w->mfd);
    w = find_pwp(&g.out, "(SPDLIM)", A380X_PWP_SPEED_CHANGE);
    CHECK(w && w->nd == A380X_PWP_ND_NONE && w->mfd);
    /* managed climb to an altitude constraint: magenta level-off, cyan start of climb at the waypoint */
    fp->plan[0].pri.el[1].cstr_type = A380X_CSTR_CLB; fp->plan[0].pri.el[1].alt_db.desc = '-'; fp->plan[0].pri.el[1].alt_db.alt1_ft = 8000;
    CLIMB_IN(A380X_FMGC_VM_CLB, 1, 39000);
    a380x_fmgc_update(&g, fp, &in, 0.1);
    pwp_dump("CLB to AAA at or below 8000", &g.out);
    w = find_pwp(&g.out, "LEVEL", A380X_PWP_CLIMB_LEVEL_OFF);
    CHECK(g.out.managed_altitude_is_constraint && w && w->colour == A380X_PWP_MAGENTA
          && fabs(w->dist_ahead_nm - a380x_fmgc_climb_distance_nm(5000, 8000, 2000, 300)) < 0.01);
    w = find_pwp(&g.out, "START", A380X_PWP_START_OF_CLIMB);
    CHECK(w && w->colour == A380X_PWP_CYAN && w->nd == A380X_PWP_ND_PATH && fabs(w->lon - 1.0) < 0.005 && fabs(w->dist_ahead_nm - 57.0) < 0.1);
    CHECK(find_pwp(&g.out, "(T/C)", A380X_PWP_CLIMB_LEVEL_OFF) && !find_pwp(&g.out, "(SPDLIM)", A380X_PWP_SPEED_CHANGE));
    /* levelled in ALT CST, CLB armed: start of climb at the constraint waypoint */
    a.alt = 8000; a.vs = 0;
    CLIMB_IN(A380X_FMGC_VM_ALT, 1, 39000); in.alt_constraint_mode = 1; in.clb_armed = 1;
    a380x_fmgc_update(&g, fp, &in, 10.0);
    pwp_dump("ALT CST 8000, CLB armed", &g.out);
    w = find_pwp(&g.out, "START", A380X_PWP_START_OF_CLIMB);
    CHECK(w && w->colour == A380X_PWP_CYAN && fabs(w->lon - 1.0) < 0.005 && !find_pwp(&g.out, "LEVEL", A380X_PWP_CLIMB_LEVEL_OFF));
    /* on the ground: none */
    a.alt = 0; CLIMB_IN(A380X_FMGC_VM_SRS, 1, 39000); in.on_ground = 1;
    a380x_fmgc_update(&g, fp, &in, 0.1);
    CHECK(g.out.nd_pwp_count == 0);
#undef CLIMB_IN
}

/* Cruise / descent pseudo waypoints: T/D, descent speed limit, DECEL (managed / selected speed),
 * descent level-off (OP DES), profile intercept from below (DES); the MFD lines at the same distances. */
static void test_pwp_descent(void)
{
    A380XFmgc g; A380XFmgcInputs in; Ac a = {0.0, 0.0, 90.0, 450.0, 0, 30000, 0};
    A380XFplnPredFeed feed; static A380XFplnWptPred legs[A380X_FPLN_MAX_ELEMENTS];
    const A380XFmgcNdPwp *td, *dc, *sl, *w;
    A380XFplnLeg *l;
    plan_begin();
    add(A380X_LEG_IF, "ORG", 0.0, -1.0);
    add(A380X_LEG_TF, "CRZ", 0.0, 0.5);
    l = add(A380X_LEG_TF, "STR", 0.0, 2.0); l->segment = A380X_SEG_STAR; l->cstr_type = A380X_CSTR_DES;
    l->alt_db.desc = '-'; l->alt_db.alt1_ft = 15000;
    l = add(A380X_LEG_TF, "IAF", 0.0, 2.5); l->segment = A380X_SEG_STAR; l->cstr_type = A380X_CSTR_DES;
    l->alt_db.desc = '@'; l->alt_db.alt1_ft = 6000;
    l = add(A380X_LEG_TF, "RW09", 0.0, 2.7 + 3000.0 / (tan(3 * D2R) * 6076.115) / 60.0);
    l->segment = A380X_SEG_APPROACH; l->cstr_type = A380X_CSTR_DES; l->vertical_angle_deg = -3.0;
    l->flags |= A380X_LEGF_DESTINATION; l->wp_desc = A380X_WPDESC_RUNWAY; l->alt_db.desc = '@'; l->alt_db.alt1_ft = 50;
    plan_end(1);
    a380x_fmgc_init(&g);
    inputs(&in, &a, 1); in.phase = 3; in.fg_vertical_mode = A380X_FMGC_VM_ALT; in.speed_managed = 1; in.vapp_kt = 140;
    a380x_fmgc_update(&g, fp, &in, 0.1);
    pwp_dump("cruise FL300, managed speed", &g.out);
    td = find_pwp(&g.out, "(T/D)", A380X_PWP_START_OF_DESCENT);
    dc = find_pwp(&g.out, "(DECEL)", A380X_PWP_DECEL);
    sl = find_pwp(&g.out, "(SPDLIM)", A380X_PWP_SPEED_CHANGE);
    CHECK(td && td->colour == A380X_PWP_WHITE && td->nd == A380X_PWP_ND_PATH
          && fabs(td->dist_ahead_nm - (g.out.dist_to_dest_nm - g.out.tod_dist_nm)) < 1e-6);
    CHECK(td && fabs(td->lat) < 0.01 && fabs(td->lon - td->dist_ahead_nm / 60.0) < 0.02);
    CHECK(dc && dc->colour == A380X_PWP_MAGENTA && fabs(dc->dist_ahead_nm - (g.out.dist_to_dest_nm - g.out.decel_dist_nm)) < 1e-6);
    CHECK(sl && sl->colour == A380X_PWP_MAGENTA && sl->nd == A380X_PWP_ND_PATH && fabs(sl->speed_kt - 250) < 1e-6);
    CHECK(td && sl && dc && td->dist_ahead_nm < sl->dist_ahead_nm && sl->dist_ahead_nm < dc->dist_ahead_nm);
    a380x_fmgc_predictions(&g, fp, &in, &feed, legs);
    CHECK(feed.pwp_count == 3 && !strcmp(feed.pwp[0].ident, "(T/D)") && !strcmp(feed.pwp[1].ident, "(SPDLIM)")
          && !strcmp(feed.pwp[2].ident, "(DECEL)"));
    CHECK(td && feed.pwp_count == 3 && fabs(feed.pwp[0].pred.dist_from_aircraft_nm - td->dist_ahead_nm) < 0.01
          && fabs(feed.pwp[2].pred.dist_from_aircraft_nm - dc->dist_ahead_nm) < 0.01);
    /* selected speed: DECEL white, no speed-change dot on the ND */
    in.speed_managed = 0;
    a380x_fmgc_update(&g, fp, &in, 0.1);
    dc = find_pwp(&g.out, "(DECEL)", A380X_PWP_DECEL); sl = find_pwp(&g.out, "(SPDLIM)", A380X_PWP_SPEED_CHANGE);
    CHECK(dc && dc->colour == A380X_PWP_WHITE && sl && sl->nd == A380X_PWP_ND_NONE && sl->mfd);
    /* HDG: T/D and DECEL are not drawn on the ND (NAV only), still on the MFD */
    inputs(&in, &a, 0); in.phase = 3; in.fg_vertical_mode = A380X_FMGC_VM_ALT; in.speed_managed = 1;
    a380x_fmgc_update(&g, fp, &in, 0.1);
    td = find_pwp(&g.out, "(T/D)", A380X_PWP_START_OF_DESCENT);
    CHECK(td && td->nd == A380X_PWP_ND_NONE && td->mfd);
    /* after the T/D (STR leg, 75 NM before the runway): OP DES to FL100 at -2000 ft/min, 300 kt: cyan
     * level-off 25 NM ahead */
    fp->plan[0].active_leg_index = 2;
    a.lon = 1.6; a.alt = 20000; a.vs = -2000; a.gs = 300;
    inputs(&in, &a, 1); in.phase = 4; in.fg_vertical_mode = A380X_FMGC_VM_OP_DES; in.fcu_alt_ft = 10000; in.speed_managed = 1;
    in.vapp_kt = 140;
    a380x_fmgc_update(&g, fp, &in, 10.0);
    pwp_dump("OP DES FL100", &g.out);
    w = find_pwp(&g.out, "LEVEL", A380X_PWP_DESCENT_LEVEL_OFF);
    CHECK(w && w->colour == A380X_PWP_CYAN && w->nd == A380X_PWP_ND_PATH && fabs(w->dist_ahead_nm - 25.0) < 0.01);
    CHECK(!find_pwp(&g.out, "(T/D)", A380X_PWP_START_OF_DESCENT));
    /* DES below the path at -1000 ft/min: intercept point where the climb-back rate closes the gap */
    {
        double path, vs_path, ahead;
        a.vs = -1000; a.alt = g.out.path_alt_ft - 2000.0;
        inputs(&in, &a, 1); in.phase = 4; in.fg_vertical_mode = A380X_FMGC_VM_DES; in.des_submode = 2; in.fcu_alt_ft = 3000;
        in.vapp_kt = 140;
        a380x_fmgc_update(&g, fp, &in, 10.0);
        path = g.out.path_alt_ft; vs_path = g.out.path_vs_fpm;
        pwp_dump("DES below path", &g.out);
        w = find_pwp(&g.out, "INTCPT", A380X_PWP_INTERCEPT_PROFILE);
        ahead = (path - a.alt) / (-1000.0 - vs_path) * 300.0 / 60.0;
        printf("    path %.0f ft (vdev %.0f) path V/S %.0f ft/min: intercept %.1f NM ahead (expected %.1f)\n", path,
               g.out.vdev_ft, vs_path, w ? w->dist_ahead_nm : NAN, ahead);
        CHECK(g.out.vdev_ft < -250 && w && w->colour == A380X_PWP_CYAN && fabs(w->dist_ahead_nm - ahead) < 0.05);
        w = find_pwp(&g.out, "LEVEL", A380X_PWP_DESCENT_LEVEL_OFF);
        CHECK(w && w->colour == A380X_PWP_MAGENTA);
    }
}

/* DIR TO (new DF leg replacing the TO leg): pursuit, then frozen great circle. */
static void test_direct_to(void)
{
    A380XFmgc g; A380XFmgcInputs in; Ac a = {0.0, 0.0, 180.0, 300.0, 0, 20000, 0};
    int t;
    plan_begin();
    add(A380X_LEG_IF, "T-P", 0.0, 0.0);
    add(A380X_LEG_DF, "TGT", 1.0, 1.0);
    add(A380X_LEG_TF, "NXT", 2.0, 1.0);
    plan_end(1);
    a380x_fmgc_init(&g);
    inputs(&in, &a, 1);
    a380x_fmgc_update(&g, fp, &in, 0.5);
    CHECK(g.out.kind == A380X_LNAV_DIRECT && g.out.nav_capture);
    CHECK(fabs(g.out.fbw_tae_deg - a380x_geo_angle_diff(180.0, a380x_geo_bearing_deg(0, 0, 1, 1))) < 1e-6);
    for (t = 0; t < 600; ++t) { inputs(&in, &a, 1); a380x_fmgc_update(&g, fp, &in, 0.5); fly(&a, &g.out, 0.5); }
    CHECK(g.df_frozen && g.out.kind == A380X_LNAV_LINE && fabs(g.out.xtk_right_nm) < 0.1);
    /* a DIR TO replacing the TO leg resets the leg state */
    fp->plan[0].pri.el[1].fix.lat = 1.2;
    snprintf(fp->plan[0].pri.el[1].ident, sizeof fp->plan[0].pri.el[1].ident, "TG2");
    inputs(&in, &a, 1);
    a380x_fmgc_update(&g, fp, &in, 0.5);
    CHECK(!g.df_frozen && g.out.kind == A380X_LNAV_DIRECT);
}

/* NAV capture zone from HDG. */
static void test_capture(void)
{
    A380XFmgc g; A380XFmgcInputs in; Ac a = {-0.1, 0.3, 60.0, 250.0, 0, 20000, 0};   /* 6 NM south, 30 deg intercept */
    int t, cap = -1;
    double xtk_at = NAN;
    plan_begin();
    add(A380X_LEG_IF, "AAA", 0.0, 0.0);
    add(A380X_LEG_TF, "BBB", 0.0, 2.0);
    plan_end(1);
    a380x_fmgc_init(&g);
    for (t = 0; t < 2000; ++t) {
        inputs(&in, &a, 0);
        a380x_fmgc_update(&g, fp, &in, 0.5);
        if (g.out.nav_capture) { cap = t; xtk_at = g.out.xtk_right_nm; break; }
        a380x_geo_destination(a.lat, a.lon, a.trk, a.gs * 0.5 / 3600.0, &a.lat, &a.lon);
    }
    printf("NAV capture on a 30 deg intercept at 250 kt: |xtk| %.2f NM (radius %.2f NM)\n", fabs(xtk_at), g.out.turn_radius_nm);
    CHECK(cap > 0 && fabs(xtk_at) < g.out.turn_radius_nm * (1 - cos(30 * D2R)) + 0.2 && fabs(xtk_at) > 0.15);
    /* diverging aircraft: no capture */
    a.lat = -0.1; a.trk = 120.0;
    inputs(&in, &a, 0);
    a380x_fmgc_update(&g, fp, &in, 0.5);
    CHECK(!g.out.nav_capture);
}

/* Descent profile: idle path from the approach, constraints, speed-limit deceleration, T/D. */
static void test_profile(void)
{
    A380XFmgc g; A380XFmgcInputs in;
    Ac a = {0.0, 0.0, 90.0, 450.0, 0, 30000, 0};
    A380XFplnLeg *l;
    double gr = a380x_fmgc_idle_gradient_ft_nm(15000, 10000), gr_lo = a380x_fmgc_idle_gradient_ft_nm(5000, 10000);
    int k;
    plan_begin();
    add(A380X_LEG_IF, "ORG", 0.0, -1.0);
    add(A380X_LEG_TF, "CRZ", 0.0, 0.5);
    l = add(A380X_LEG_TF, "STR", 0.0, 2.0); l->segment = A380X_SEG_STAR; l->cstr_type = A380X_CSTR_DES;
    l->alt_db.desc = '-'; l->alt_db.alt1_ft = 15000;                       /* at or below FL150 */
    l = add(A380X_LEG_TF, "IAF", 0.0, 2.5); l->segment = A380X_SEG_STAR; l->cstr_type = A380X_CSTR_DES;
    l->alt_db.desc = '@'; l->alt_db.alt1_ft = 6000;
    l = add(A380X_LEG_TF, "FAF", 0.0, 2.7); l->segment = A380X_SEG_APPROACH; l->cstr_type = A380X_CSTR_DES;
    l->alt_db.desc = '+'; l->alt_db.alt1_ft = 3000;
    l = add(A380X_LEG_TF, "RW09", 0.0, 2.7 + 3000.0 / (tan(3 * D2R) * 6076.115) / 60.0);
    l->segment = A380X_SEG_APPROACH; l->cstr_type = A380X_CSTR_DES; l->vertical_angle_deg = -3.0;
    l->flags |= A380X_LEGF_DESTINATION; l->wp_desc = A380X_WPDESC_RUNWAY;
    l->alt_db.desc = '@'; l->alt_db.alt1_ft = 50;
    plan_end(1);
    a380x_fmgc_init(&g);
    inputs(&in, &a, 1);
    a380x_fmgc_update(&g, fp, &in, 0.5);
    CHECK(g.out.vnav_valid && g.out.profile_count >= 5);
    for (k = 0; k < g.out.profile_count; ++k)
        printf("  profile %d: %6.1f NM %7.0f ft leg %d seg %d\n", k, g.out.profile[k].dist_to_dest_nm, g.out.profile[k].alt_ft,
               g.out.profile[k].leg_index, g.out.profile[k].segment);
    printf("  T/D %.1f NM before destination, speed-limit decel from %.1f NM, DECEL %.1f NM, PPOS %.1f NM path %.0f ft\n",
           g.out.tod_dist_nm, g.out.speed_limit_dist_nm, g.out.decel_dist_nm, g.out.dist_to_dest_nm, g.out.path_alt_ft);
    /* runway 50 ft, 3 deg to FAF at 3000 */
    CHECK(fabs(g.out.profile[0].alt_ft - 50.0) < 1e-6);
    CHECK(fabs(g.out.profile[1].alt_ft - 3050.0) < 20.0 && g.out.profile[1].leg_index == 4);
    CHECK(fabs(g.out.profile[2].alt_ft - 6000.0) < 1e-6);   /* '@' 6000: the FAF->IAF climb-back is too steep -> clamped */
    CHECK(g.out.profile[3].alt_ft <= 15000.0 + 1e-6);         /* at or below FL150 respected */
    CHECK(isfinite(g.out.tod_dist_nm) && g.out.before_tod);
    CHECK(isfinite(g.out.speed_limit_dist_nm));
    /* T/D roughly where the idle path from the FL150 point (with the 10000 ft deceleration below it) reaches FL300 */
    {
        double d150 = g.out.profile[3].dist_to_dest_nm, expect = d150 + (30000.0 - g.out.profile[3].alt_ft) / gr;
        CHECK(fabs(g.out.tod_dist_nm - expect) < 3.0);
    }
    (void)gr_lo;
    CHECK(g.out.path_alt_ft > 30000.0 && g.out.vdev_ft < 0);   /* cruise part: path extended above the cruise level */
    CHECK(g.out.path_vs_fpm < -1000);
    /* managed descent speed: limit applied from the decel point down to 10000 ft */
    in.phase = 4; in.managed_speed_kt = 300; in.alt_ft = 11000;
    {
        /* 1 NM after the speed-limit deceleration point, on the STR -> IAF leg (IAF 21.4 NM out) */
        double target = g.out.speed_limit_dist_nm - 1.0, iaf = g.out.profile[2].dist_to_dest_nm;
        fp->plan[0].active_leg_index = 3;
        a.lat = 0.0; a.lon = 2.5 - (target - iaf) / 60.0; a.alt = 11000;
        inputs(&in, &a, 1); in.phase = 4; in.managed_speed_kt = 300;
        a380x_fmgc_update(&g, fp, &in, 0.0);
        CHECK(fabs(g.out.dist_to_dest_nm - target) < 0.2);
        if (getenv("FMGC_DEBUG")) printf("    limit: valid %d spd %.1f mach %d d2d %.2f lim %.2f\n", g.out.speed_valid, g.out.speed_kt, g.out.speed_is_mach, g.out.dist_to_dest_nm, g.out.speed_limit_dist_nm);
        CHECK(g.out.speed_valid && fabs(g.out.speed_kt - 250.0) < 1e-6 && !g.out.speed_is_mach);
    }
}

static void test_managed_altitude_and_speed(void)
{
    A380XFmgc g; A380XFmgcInputs in; Ac a = {0.0, -0.9, 90.0, 250.0, 0, 3000, 1500};
    A380XFplnLeg *l;
    plan_begin();
    add(A380X_LEG_IF, "ORG", 0.0, -1.0);
    l = add(A380X_LEG_TF, "C1", 0.0, -0.5); l->cstr_type = A380X_CSTR_CLB; l->alt_db.desc = '-'; l->alt_db.alt1_ft = 6000;
    l = add(A380X_LEG_TF, "C2", 0.0, 0.0); l->cstr_type = A380X_CSTR_CLB; l->alt_db.desc = '@'; l->alt_db.alt1_ft = 8000;
    l->spd_db.desc = '-'; l->spd_db.speed = 220;
    l = add(A380X_LEG_TF, "D1", 0.0, 1.0); l->cstr_type = A380X_CSTR_DES; l->alt_db.desc = '+'; l->alt_db.alt1_ft = 7000;
    l = add(A380X_LEG_TF, "D2", 0.0, 1.5); l->cstr_type = A380X_CSTR_DES; l->alt_db.desc = 'B'; l->alt_db.alt1_ft = 5000; l->alt_db.alt2_ft = 4000;
    plan_end(1);
    a380x_fmgc_init(&g);
    inputs(&in, &a, 1); in.phase = 2; in.fcu_alt_ft = 20000;
    a380x_fmgc_update(&g, fp, &in, 0.1);
    CHECK(g.out.managed_altitude_is_constraint && g.out.managed_altitude_ft == 6000);
    CHECK(fabs(a380x_fmgc_speed_constraint(fp, NULL, 2) - 220.0) < 1e-6);
    CHECK(a380x_fmgc_lowest_climb_constraint(fp, 0) == 6000);
    in.fcu_alt_ft = 5000;
    a380x_fmgc_update(&g, fp, &in, 0.1);
    CHECK(!g.out.managed_altitude_is_constraint && g.out.managed_altitude_ft == 5000);
    fp->plan[0].active_leg_index = 3;
    inputs(&in, &a, 1); in.phase = 4; in.fcu_alt_ft = 2000;
    a380x_fmgc_update(&g, fp, &in, 0.1);
    CHECK(g.out.managed_altitude_is_constraint && g.out.managed_altitude_ft == 7000);
    /* approach managed speed: max(VAPP, maneuvering speed of the lever) */
    in.phase = 5; in.vapp_kt = 140; in.green_dot_kt = 210; in.s_speed_kt = 180; in.f_speed_kt = 160; in.landing_conf3 = 0;
    in.flap_lever = 0; a380x_fmgc_update(&g, fp, &in, 0.1); CHECK(g.out.speed_valid && g.out.speed_kt == 210);
    in.flap_lever = 1; a380x_fmgc_update(&g, fp, &in, 0.1); CHECK(g.out.speed_kt == 180);
    in.flap_lever = 2; a380x_fmgc_update(&g, fp, &in, 0.1); CHECK(g.out.speed_kt == 160);
    in.flap_lever = 3; a380x_fmgc_update(&g, fp, &in, 0.1); CHECK(g.out.speed_kt == 160);
    in.landing_conf3 = 1; a380x_fmgc_update(&g, fp, &in, 0.1); CHECK(g.out.speed_kt == 140);
    in.flap_lever = 4; a380x_fmgc_update(&g, fp, &in, 0.1); CHECK(g.out.speed_kt == 140);
    in.phase = 3; a380x_fmgc_update(&g, fp, &in, 0.1); CHECK(!g.out.speed_valid);
}

/* Predictions feed for the MFD (pseudo waypoints and per-leg values). */
static void test_predictions(void)
{
    A380XFmgc g; A380XFmgcInputs in; Ac a = {0.0, 0.0, 90.0, 450.0, 0, 30000, 0};
    A380XFplnPredFeed feed; static A380XFplnWptPred legs[A380X_FPLN_MAX_ELEMENTS];
    A380XFplnLeg *l;
    int k, has_td = 0;
    plan_begin();
    add(A380X_LEG_IF, "ORG", 0.0, -1.0);
    add(A380X_LEG_TF, "AAA", 0.0, 1.0);
    add(A380X_LEG_TF, "BBB", 0.0, 3.0);
    l = add(A380X_LEG_TF, "RW09", 0.0, 4.0); l->flags |= A380X_LEGF_DESTINATION; l->wp_desc = A380X_WPDESC_RUNWAY;
    l->segment = A380X_SEG_APPROACH; l->cstr_type = A380X_CSTR_DES;
    plan_end(1);
    a380x_fmgc_init(&g);
    inputs(&in, &a, 1);
    a380x_fmgc_update(&g, fp, &in, 0.1);
    a380x_fmgc_predictions(&g, fp, &in, &feed, legs);
    for (k = 0; k < feed.pwp_count; ++k) if (!strcmp(feed.pwp[k].ident, "(T/D)")) has_td = 1;
    CHECK(has_td && feed.leg_pred == legs && legs[1].valid && legs[3].valid);
    CHECK(legs[3].altitude_ft < 100 && legs[2].seconds_from_present > legs[1].seconds_from_present);
}

int main(void)
{
    A380XNavdataConfig cfg;
    A380XNavdataHost host;
    memset(&cfg, 0, sizeof cfg);
    cfg.xplane_root = "/nonexistent/x-plane";
    memset(&host, 0, sizeof host);
    host.magvar = zero_magvar;
    nav = a380x_nav_create(&cfg, &host);
    fp = a380x_fpln_create(nav);
    if (!nav || !fp) { printf("FAIL: setup\n"); return 1; }
    test_radius();
    test_flyby();
    test_discontinuity();
    test_course_and_heading_legs();
    test_rf_arc();
    test_hold();
    test_hold_entry_sectors();
    test_hold_entries_flown();
    test_holding_speed();
    test_hold_speed_target();
    test_direct_to();
    test_capture();
    test_profile();
    test_managed_altitude_and_speed();
    test_predictions();
    test_pwp_climb();
    test_pwp_descent();
    a380x_fpln_destroy(fp);
    a380x_nav_destroy(nav);
    if (failures) { printf("FMGC guidance: %d of %d checks FAILED\n", failures, checks); return 1; }
    printf("FMGC guidance: %d checks passed (legs, sequencing, transitions, holds + entries, hold speed, DIR TO, capture, profile, speeds, pseudo waypoints)\n", checks);
    return 0;
}
