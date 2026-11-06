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
    double k2v = 0.004451 * a->gs, k1k2 = 30.44, phi = 0, rate;
    double xt = o->fbw_xtk_nm * 6839.0 / a->gs;
    if (xt > 45) xt = 45;
    if (xt < -45) xt = -45;
    if (o->lnav_valid) phi = o->bank_ff_deg + (o->fbw_tae_deg + xt) * k2v;
    (void)k1k2;
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
        Ac hold = a;
        inputs(&in, &a, 0);
        a380x_fmgc_update(&g, fp, &in, 0.5);
        if (g.out.nav_capture) { cap = t; xtk_at = g.out.xtk_right_nm; break; }
        hold.bank = 0;
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
    for (k = 0; k < feed.pwp_count; ++k) if (!strcmp(feed.pwp[k].ident, "T/D")) has_td = 1;
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
    test_direct_to();
    test_capture();
    test_profile();
    test_managed_altitude_and_speed();
    test_predictions();
    a380x_fpln_destroy(fp);
    a380x_nav_destroy(nav);
    if (failures) { printf("FMGC guidance: %d of %d checks FAILED\n", failures, checks); return 1; }
    printf("FMGC guidance: %d checks passed (legs, sequencing, transitions, holds, DIR TO, capture, profile, speeds)\n", checks);
    return 0;
}
