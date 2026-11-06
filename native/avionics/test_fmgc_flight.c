/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Offline closed-loop flight of the managed guidance chain on real X-Plane CIFP data:
 *   navdata (X-Plane Custom Data / default data, read only) -> flight plan (SID, STAR, approach)
 *   -> native FMS (phases, speeds, constraints) -> FMGC (LNAV / VNAV / managed speed)
 *   -> FCU guidance state machine -> unmodified FBW A380 outer loops -> port actuator servos
 *   -> a simple point-mass aircraft (below) -> sensors -> back.
 *
 * The aircraft model is NOT the X-Plane A380 flight model: point mass with lift/drag polars per
 * configuration, four engines with spool lag, ISA atmosphere and a constant-direction wind, the
 * pitch/roll actuator response of test_guidance.c. It is physically reasonable, so the run proves
 * mode sequencing, path geometry, constraint logic and loop stability, not X-Plane gains.
 *
 * Usage: test_fmgc_flight <X-Plane root> [csv trace]. Without navigation data it prints SKIPPED.
 */
#include "a380x_avionics_core.h"
#include "a380x_fmgc_guidance.h"
#include "a380x_fms.h"
#include "a380x_guidance.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define D2R (3.14159265358979323846 / 180.0)
#define FT 0.3048
#define KT 0.514444
#define G0 9.80665

static int failures;
#define CHECK(c, ...) do { if (!(c)) { ++failures; printf("FAIL %s:%d: %s -- ", __FILE__, __LINE__, #c); printf(__VA_ARGS__); printf("\n"); } } while (0)

static A380XNavdata *nav;
static A380XFlightPlans *fp;
static A380XFms fms;
static A380XFmgc fmgc;
static A380XGuidanceState gs;
static A380XFplnAircraft fac;

/* ---------------------------------------------------------------------------------------------- */
/* FMS plan hooks (the X-Plane adapter's, a380x_fms_adapter.inc, without XPLM)                      */
/* ---------------------------------------------------------------------------------------------- */

static const A380XFplnPlan *hk_plan(struct A380XFlightPlans *p, int plan)
{
    return p && plan >= 0 && plan < A380X_PLAN_COUNT ? a380x_fpln_plan(p, plan) : NULL;
}
static int hk_exists(struct A380XFlightPlans *p, int plan) { return hk_plan(p, plan) != NULL; }
static unsigned hk_version(struct A380XFlightPlans *p, int plan) { const A380XFplnPlan *pl = hk_plan(p, plan); return pl ? pl->version : 0u; }
static int hk_city_pair(struct A380XFlightPlans *p, int plan, char o[8], char d[8], char a[8])
{
    const A380XFplnPlan *pl = hk_plan(p, plan);
    o[0] = d[0] = a[0] = 0;
    if (!pl) return 0;
    snprintf(o, 8, "%.7s", pl->pri.origin_icao); snprintf(d, 8, "%.7s", pl->pri.dest_icao); snprintf(a, 8, "%.7s", pl->alt.dest_icao);
    return 1;
}
static int rwy_info(struct A380XFlightPlans *p, const char *icao, const char *rwy, A380XFmsRunwayInfo *out)
{
    const A380XNavAirport *apt; const A380XNavRunway *r;
    memset(out, 0, sizeof *out);
    out->threshold_elev_ft = out->length_m = out->magnetic_bearing_deg = out->threshold_lat_deg = out->threshold_lon_deg = NAN;
    if (!p || !icao[0] || !rwy[0] || !(apt = a380x_nav_airport(p->nav, icao)) || !(r = a380x_nav_runway(apt, rwy))) return 0;
    out->valid = 1;
    snprintf(out->ident, sizeof out->ident, "%.7s", r->designator);
    out->threshold_elev_ft = r->thr_elev_ft; out->length_m = r->length_ft * 0.3048; out->magnetic_bearing_deg = r->mag_brg;
    out->threshold_lat_deg = r->thr_lat; out->threshold_lon_deg = r->thr_lon;
    return 1;
}
static int hk_orig_rwy(struct A380XFlightPlans *p, int plan, A380XFmsRunwayInfo *o)
{ const A380XFplnPlan *pl = hk_plan(p, plan); if (!pl) { memset(o, 0, sizeof *o); return 0; } return rwy_info(p, pl->pri.origin_icao, pl->pri.origin_rwy, o); }
static int hk_dest_rwy(struct A380XFlightPlans *p, int plan, A380XFmsRunwayInfo *o)
{ const A380XFplnPlan *pl = hk_plan(p, plan); if (!pl) { memset(o, 0, sizeof *o); return 0; } return rwy_info(p, pl->pri.dest_icao, pl->pri.dest_rwy, o); }
static int hk_approach(struct A380XFlightPlans *p, int plan, A380XFmsApproachInfo *o)
{
    const A380XFplnPlan *pl = hk_plan(p, plan);
    memset(o, 0, sizeof *o);
    if (!pl || !pl->pri.approach[0]) return 0;
    o->valid = 1; o->type = A380X_FMS_APPR_ILS;
    a380x_fpln_approach_name(p, &pl->pri, o->name, sizeof o->name);
    return 1;
}
static double hk_dist_dest(struct A380XFlightPlans *p, int plan) { float d = a380x_fpln_dist_to_dest(p, plan, &fac); return isfinite(d) ? d : NAN; }
static double hk_lowest_clb(struct A380XFlightPlans *p, int plan) { return a380x_fmgc_lowest_climb_constraint(p, plan); }
static double hk_spd_cstr(struct A380XFlightPlans *p, int phase) { return a380x_fmgc_speed_constraint(p, &fac, phase); }
static int hk_cstr_alt(struct A380XFlightPlans *p, double *c, double *d) { return a380x_fmgc_constraint_altitudes(p, c, d); }
static int hk_new_city_pair(struct A380XFlightPlans *p, int plan, const char *f, const char *t, const char *a)
{ return a380x_fpln_new_city_pair(p, plan, f, t, a) == A380X_FPLN_OK; }
static int hk_tmpy_insert(struct A380XFlightPlans *p) { return a380x_fpln_tmpy_insert(p, &fac) == A380X_FPLN_OK; }
static int hk_tmpy_erase(struct A380XFlightPlans *p) { return a380x_fpln_tmpy_erase(p) == A380X_FPLN_OK; }
static int hk_airport(void *ctx, const char *icao, A380XFmsAirportInfo *out)
{
    A380XNavAirportInfo info;
    (void)ctx;
    memset(out, 0, sizeof *out);
    memset(&info, 0, sizeof info);
    if (!a380x_nav_airport_info(nav, icao, &info)) return 0;
    out->found = 1; snprintf(out->ident, sizeof out->ident, "%s", icao);
    out->elevation_ft = info.elev_ft; out->lat_deg = info.lat; out->lon_deg = info.lon;
    out->trans_alt_ft = isfinite(info.trans_alt_ft) ? info.trans_alt_ft : NAN;
    out->trans_level_fl = isfinite(info.trans_level_ft) ? info.trans_level_ft / 100.0 : NAN;
    return 1;
}

/* ---------------------------------------------------------------------------------------------- */
/* Aircraft model                                                                                  */
/* ---------------------------------------------------------------------------------------------- */

typedef struct {
    double lat, lon, h_ft, v_tas, gamma, psi;   /* psi: true heading, deg */
    double theta, q, phi, p;                    /* deg, deg/s */
    double mass, thrust_n[4];
    int on_ground, flaps, gear, speedbrake;
    double trk, gs_kt, vs_fpm, alpha, nz;
    double wind_dir, wind_kt;                   /* wind FROM, true */
} Aircraft;

static void isa(double h_ft, double *rho, double *p, double *t, double *a)
{
    double h = h_ft * FT;
    if (h < 11000.0) { *t = 288.15 - 0.0065 * h; *p = 101325.0 * pow(*t / 288.15, 5.25588); }
    else { *t = 216.65; *p = 22632.06 * exp(-(h - 11000.0) / 6341.62); }
    *rho = *p / (287.053 * *t);
    *a = sqrt(1.4 * 287.053 * *t);
}

static double cas_kt(double tas, double h_ft, double *mach)
{
    double rho, p, t, a, qc;
    isa(h_ft, &rho, &p, &t, &a);
    *mach = tas / a;
    qc = p * (pow(1.0 + 0.2 * *mach * *mach, 3.5) - 1.0);
    return 340.294 * sqrt(5.0 * (pow(qc / 101325.0 + 1.0, 2.0 / 7.0) - 1.0)) / KT;
}

/* Clean CD0 0.021 / k 0.045 gives an idle descent near 3 deg at 250-300 kt CAS, the A380 idle path
 * (about 2000 ft/min at FL150 / 300 kt) the FMGC profile assumes. */
static const double CL0[5] = {0.20, 0.45, 0.65, 0.85, 1.05}, CD0[5] = {0.021, 0.030, 0.042, 0.059, 0.082};

static double max_thrust_n(double h_ft, double mach)
{
    double rho, p, t, a;
    isa(h_ft, &rho, &p, &t, &a);
    return 310000.0 * pow(rho / 1.225, 0.75) * (1.0 - 0.35 * mach);
}

static void aircraft_step(Aircraft *ac, double throttle[4], double pitch_servo, double roll_servo,
                          int pilot_pitch, double pitch_target, double dt)
{
    double rho, p, t, a, qbar, cl, cd, lift, drag, thr = 0, mach, w, e, n, tmax;
    int k;
    isa(ac->h_ft, &rho, &p, &t, &a);
    mach = ac->v_tas / a;
    tmax = max_thrust_n(ac->h_ft, mach);
    for (k = 0; k < 4; ++k) {
        double target = tmax * (0.025 + 0.975 * fmax(0.0, fmin(1.0, throttle[k])));
        double tau = target > ac->thrust_n[k] ? 3.0 : 2.0;
        ac->thrust_n[k] += (target - ac->thrust_n[k]) * fmin(1.0, dt / tau);
        thr += ac->thrust_n[k];
    }
    /* attitude */
    if (pilot_pitch) {
        double rate = fmax(-3.0, fmin(3.0, (pitch_target - ac->theta) * 1.0));
        ac->q = rate; ac->theta += rate * dt;
    } else {
        ac->q += (3.0 * pitch_servo - 0.7 * ac->q) * dt;
        ac->theta += ac->q * dt;
    }
    if (ac->on_ground) { ac->p = 0; ac->phi = 0; }
    else { ac->p += (10.0 * roll_servo - ac->p) * dt; ac->phi += ac->p * dt; }
    ac->alpha = ac->theta - ac->gamma / D2R;
    qbar = 0.5 * rho * ac->v_tas * ac->v_tas;
    cl = CL0[ac->flaps] + 5.0 * ac->alpha * D2R;
    if (cl > 2.6) cl = 2.6;
    cd = CD0[ac->flaps] + (ac->gear ? 0.015 : 0.0) + (ac->speedbrake ? 0.008 : 0.0) + 0.045 * cl * cl;
    lift = qbar * 845.0 * cl; drag = qbar * 845.0 * cd;
    w = ac->mass * G0;
    if (ac->on_ground) {
        double fric = 0.02 * fmax(0.0, w - lift);
        ac->v_tas += ((thr - drag - fric) / ac->mass) * dt;
        ac->gamma = 0;
        if (lift > w * 1.02 && ac->theta > 5.0) ac->on_ground = 0;
        ac->nz = 1.0;
    } else {
        double gd;
        ac->v_tas += ((thr * cos(ac->alpha * D2R) - drag) / ac->mass - G0 * sin(ac->gamma)) * dt;
        gd = ((lift + thr * sin(ac->alpha * D2R)) * cos(ac->phi * D2R) - w * cos(ac->gamma)) / (ac->mass * ac->v_tas);
        ac->gamma += gd * dt;
        ac->psi = fmod(ac->psi + (lift * sin(ac->phi * D2R)) / (ac->mass * ac->v_tas * cos(ac->gamma)) / D2R * dt + 360.0, 360.0);
        ac->nz = lift / w;
    }
    /* kinematics with wind (wind speed scales with altitude up to the given value at FL200) */
    {
        double wk = ac->wind_kt * fmin(1.0, 0.3 + ac->h_ft / 20000.0) * KT, wto = (ac->wind_dir + 180.0) * D2R;
        double vh = ac->v_tas * cos(ac->gamma);
        e = vh * sin(ac->psi * D2R) + wk * sin(wto);
        n = vh * cos(ac->psi * D2R) + wk * cos(wto);
        if (ac->on_ground) { e = vh * sin(ac->psi * D2R); n = vh * cos(ac->psi * D2R); }
        ac->gs_kt = hypot(e, n) / KT;
        ac->trk = fmod(atan2(e, n) / D2R + 360.0, 360.0);
        a380x_geo_destination(ac->lat, ac->lon, ac->trk, ac->gs_kt * dt / 3600.0, &ac->lat, &ac->lon);
        ac->vs_fpm = ac->v_tas * sin(ac->gamma) / FT * 60.0;
        ac->h_ft += ac->vs_fpm * dt / 60.0;
    }
}

/* ---------------------------------------------------------------------------------------------- */
/* Adapter mapping (a380x_fms_adapter.inc / a380x_fcu_adapter.inc, without XPLM)                   */
/* ---------------------------------------------------------------------------------------------- */

static A380XFmsVerticalMode vm_of(int takeoff_power, int on_ground, int phase)
{
    if (takeoff_power && (on_ground || phase <= A380X_FMS_PHASE_TAKEOFF)) return A380X_FMS_VM_SRS;
    switch (gs.vertical_mode) {
    case A380X_B1_GS: case A380X_B1_GS_STAR: return A380X_FMS_VM_GS;
    case A380X_B1_SRS: return A380X_FMS_VM_SRS;
    case A380X_B1_DES: return A380X_FMS_VM_DES;
    case A380X_B1_OP_DES: return A380X_FMS_VM_OP_DES;
    case A380X_B1_CLB: return A380X_FMS_VM_CLB;
    case A380X_B1_OP_CLB: return A380X_FMS_VM_OP_CLB;
    case A380X_B1_ALT: return A380X_FMS_VM_ALT;
    case A380X_B1_ALT_STAR: return A380X_FMS_VM_ALT_STAR;
    case A380X_B1_FPA: return A380X_FMS_VM_FPA;
    case A380X_B1_VS: return A380X_FMS_VM_VS;
    default: return A380X_FMS_VM_NONE;
    }
}

static double perf(int field)
{
    A380XFmsValue v = a380x_fms_get(&fms, A380X_FMS_PLAN_ACTIVE, (A380XFmsField)field);
    return v.has && isfinite(v.v) ? v.v : 0.0;
}

static double magvar_at(double lat, double lon) { return a380x_nav_magvar(nav, lat, lon); }

/* ---------------------------------------------------------------------------------------------- */
/* Flight                                                                                          */
/* ---------------------------------------------------------------------------------------------- */

typedef struct { int leg; char ident[16]; char desc; double lo, hi, alt, cas, spd; int spd_desc; double lat, lon, dmin, seq_alt, seq_cas; int seen, climb; } Cstr;

static int find_disco(const A380XFplnRoute *r)
{
    int i;
    for (i = 1; i < r->count && i < r->first_missed_appr_index; ++i)
        if (r->el[i].flags & A380X_LEGF_DISCONTINUITY) return i;
    return -1;
}

static int build_plan(void)
{
    MfdFmsError err;
    int k, d;
    if (!a380x_fms_enter_text(&fms, A380X_FMS_PLAN_ACTIVE, A380X_FMS_T_FROM, "EGLL", &err)) return 0;
    if (!a380x_fms_enter_text(&fms, A380X_FMS_PLAN_ACTIVE, A380X_FMS_T_TO, "EGCC", &err)) return 0;
    if (!hk_plan(fp, A380X_PLAN_ACTIVE)) return 0;
#define TMP(call) do { if ((call) != A380X_FPLN_OK) { printf("plan op failed: %s\n", #call); return 0; } \
                       if (a380x_fpln_has_tmpy(fp)) { a380x_fms_tmpy_begin(&fms); a380x_fms_tmpy_insert(&fms); } } while (0)
    TMP(a380x_fpln_set_origin_runway(fp, A380X_PLAN_ACTIVE, 0, "27R"));
    TMP(a380x_fpln_set_sid(fp, A380X_PLAN_ACTIVE, 0, "BPK7F"));
    TMP(a380x_fpln_set_approach(fp, A380X_PLAN_ACTIVE, 0, "I23R"));
    TMP(a380x_fpln_set_star(fp, A380X_PLAN_ACTIVE, 0, "LEST1M"));
    for (k = 0; k < 6 && (d = find_disco(&hk_plan(fp, A380X_PLAN_ACTIVE)->pri)) > 0; ++k)
        TMP(a380x_fpln_delete_element(fp, A380X_PLAN_ACTIVE, 0, d));
#undef TMP
    return 1;
}

static void print_plan(void)
{
    const A380XFplnRoute *r = &hk_plan(fp, A380X_PLAN_ACTIVE)->pri;
    int i;
    printf("ACTIVE F-PLN EGLL 27R BPK7F LEST1M I23R EGCC (%d elements, destination leg %d, missed from %d):\n",
           r->count, r->destination_leg_index, r->first_missed_appr_index);
    for (i = 0; i < r->count && i < r->first_missed_appr_index; ++i) {
        const A380XFplnLeg *l = &r->el[i];
        A380XFplnAltCstr a = a380x_fpln_eff_alt(l);
        A380XFplnSpdCstr s = a380x_fpln_eff_spd(l);
        if (l->flags & A380X_LEGF_DISCONTINUITY) { printf("  %2d ---- DISCONTINUITY\n", i); continue; }
        printf("  %2d %-2s %-8s %5.1f NM  crs %5.1f", i, a380x_leg_type_name(l->type), l->ident, l->leg_dist_nm, l->path_true_crs);
        if (a.desc) { printf("  alt %c%.0f", a.desc, a.alt1_ft); if (isfinite(a.alt2_ft)) printf("/%.0f", a.alt2_ft); }
        if (s.desc) printf("  spd %c%.0f", s.desc, s.speed);
        if (isfinite(l->vertical_angle_deg)) printf("  fpa %.1f", l->vertical_angle_deg);
        printf("  %s\n", l->cstr_type == A380X_CSTR_CLB ? "[CLB]" : l->cstr_type == A380X_CSTR_DES ? "[DES]" : "");
    }
}

static const char *b1_name(int b)
{
    static const char *n[] = {"", "G/S", "F-G/S", "G/S*", "F-G/S*", "SRS", "TCAS", "APP DES", "DES", "OP DES", "CLB",
                              "OP CLB", "ALT", "ALT*", "ALT CST", "ALT CST*", "ALT CRZ", "ALT CRZ*", "FPA", "V/S"};
    return b >= 0 && b < 20 ? n[b] : "?";
}
static const char *c1_name(int c)
{
    static const char *n[] = {"", "GA TRK", "LOC B/C*", "LOC*", "F-LOC*", "HDG", "RWY", "RWY TRK", "TRACK", "LOC B/C", "LOC", "F-LOC", "NAV"};
    return c >= 0 && c < 13 ? n[c] : "?";
}
static const char *a_name(int a)
{
    static const char *n[] = {"", "MAN TOGA", "MAN GA SOFT", "MAN FLX", "MAN MCT", "MAN THR", "SPEED", "MACH", "THR MCT", "THR CLB",
                              "THR LVR", "THR IDLE", "A.FLOOR", "TOGA LK"};
    return a >= 0 && a < 14 ? n[a] : "?";
}

int main(int argc, char **argv)
{
    A380XNavdataConfig cfg;
    A380XFmsNavHooks nh;
    A380XFmsPlanHooks ph;
    A380XFmsInputs fin;
    A380XFmgcInputs gin;
    A380XGuidanceInputs i;
    Aircraft ac;
    const A380XNavAirport *apt;
    const A380XNavRunway *rw;
    double thr_lat, thr_lon, rwy_brg, origin_elev, dest_elev, t = 0, dt = 0.05;
    double max_xtk_line = 0, max_xtk_turn = 0, max_xtk_recover = 0, t_seq = -1000, sum_xtk = 0, max_vdev_path = 0, tod_pred = NAN, des_start_d2d = NAN;
    double liftoff_t = -1, levers_cl = 0, fcu_des_set = 0, final_xtk = NAN, final_dist = NAN;
    long n_xtk = 0, steps = 0;
    int ap_engaged = 0, prev_b1 = -1, prev_c1 = -1, prev_a = -1, prev_arm = -1, prev_phase = -1, prev_leg = -1;
    int ncstr = 0, k, done = 0, stop_reason = 0, n_spdbrk = 0;
    int seen[32] = {0}, des_sub_seen[3] = {0}, thr_seen[16] = {0};
    double nav_engage_ra = NAN;
    double spdbrk_time = 0;
    Cstr cs[A380X_FPLN_MAX_ELEMENTS];
    FILE *csv = argc > 2 ? fopen(argv[2], "w") : NULL;
    double next_speed_log_alt = 2000;
    int climbing_log = 1;
    double max_alt = 0, crz_level_err = NAN, lvl_cst_err[8]; int n_lvl = 0;

    if (argc < 2) { printf("usage: test_fmgc_flight <X-Plane root> [trace.csv]\n"); return 2; }
    memset(&cfg, 0, sizeof cfg);
    cfg.xplane_root = argv[1];
    nav = a380x_nav_create(&cfg, NULL);
    if (!nav || a380x_nav_load_global(nav) != 0 || !(apt = a380x_nav_airport(nav, "EGLL")) || !a380x_nav_airport(nav, "EGCC")) {
        printf("FMGC closed-loop flight: SKIPPED (no X-Plane navigation data / CIFP at %s)\n", argv[1]);
        return 0;
    }
    apt = a380x_nav_airport(nav, "EGLL");
    rw = a380x_nav_runway(apt, "27R");
    if (!rw) { printf("FMGC closed-loop flight: SKIPPED (EGLL 27R not in the CIFP)\n"); return 0; }
    thr_lat = rw->thr_lat; thr_lon = rw->thr_lon; rwy_brg = rw->true_brg; origin_elev = rw->thr_elev_ft;
    rw = a380x_nav_runway(a380x_nav_airport(nav, "EGCC"), "23R");
    dest_elev = rw ? rw->thr_elev_ft : 250.0;
    fp = a380x_fpln_create(nav);
    memset(&nh, 0, sizeof nh); nh.airport_lookup = hk_airport;
    memset(&ph, 0, sizeof ph);
    ph.plans = fp; ph.plan_exists = hk_exists; ph.plan_version = hk_version; ph.city_pair = hk_city_pair;
    ph.origin_runway = hk_orig_rwy; ph.destination_runway = hk_dest_rwy; ph.approach = hk_approach;
    ph.distance_to_destination_nm = hk_dist_dest; ph.lowest_climb_constraint_ft = hk_lowest_clb;
    ph.speed_constraint_kt = hk_spd_cstr; ph.constraint_altitudes = hk_cstr_alt; ph.new_city_pair = hk_new_city_pair;
    ph.tmpy_insert = hk_tmpy_insert; ph.tmpy_erase = hk_tmpy_erase;
    a380x_fms_init(&fms, NULL, &nh, &ph);
    a380x_fmgc_init(&fmgc);

    /* aircraft on the 27R threshold */
    memset(&ac, 0, sizeof ac);
    ac.lat = thr_lat; ac.lon = thr_lon; ac.h_ft = origin_elev; ac.psi = ac.trk = rwy_brg; ac.mass = 420000.0;
    ac.on_ground = 1; ac.flaps = 2; ac.gear = 1; ac.wind_dir = 250.0; ac.wind_kt = 35.0;
    memset(&fac, 0, sizeof fac);
    fac.valid = 1; fac.lat = ac.lat; fac.lon = ac.lon; fac.alt_ft = (float)ac.h_ft; fac.true_track_deg = (float)ac.trk; fac.on_ground = 1;

    if (!build_plan()) { printf("FAIL: flight plan\n"); return 1; }
    print_plan();
    {
        MfdFmsError err;
        a380x_fms_enter(&fms, 0, A380X_FMS_F_CRZ_FL, 240, &err);
        a380x_fms_enter(&fms, 0, A380X_FMS_F_CI, 60, &err);
        a380x_fms_enter(&fms, 0, A380X_FMS_F_ZFW, 330000, &err);
        a380x_fms_enter(&fms, 0, A380X_FMS_F_ZFWCG, 32, &err);
        a380x_fms_enter(&fms, 0, A380X_FMS_F_BLOCK, 90000, &err);
        a380x_fms_enter(&fms, 0, A380X_FMS_F_V1, 145, &err);
        a380x_fms_enter(&fms, 0, A380X_FMS_F_VR, 152, &err);
        a380x_fms_enter(&fms, 0, A380X_FMS_F_V2, 160, &err);
    }
    /* collect the constraints of the plan for compliance */
    {
        const A380XFplnRoute *r = &hk_plan(fp, A380X_PLAN_ACTIVE)->pri;
        for (k = 0; k < r->count && k < r->first_missed_appr_index; ++k) {
            const A380XFplnLeg *l = &r->el[k];
            A380XFplnAltCstr a = a380x_fpln_eff_alt(l);
            A380XFplnSpdCstr s = a380x_fpln_eff_spd(l);
            if (l->flags & A380X_LEGF_DISCONTINUITY) continue;
            if (!(a.desc && isfinite(a.alt1_ft)) && !(s.desc && isfinite(s.speed))) continue;
            if (a380x_leg_is_heading(l->type) || l->type == A380X_LEG_CA || l->type == A380X_LEG_CI) continue;
            memset(&cs[ncstr], 0, sizeof cs[ncstr]);
            cs[ncstr].leg = k; snprintf(cs[ncstr].ident, sizeof cs[ncstr].ident, "%s", l->ident);
            cs[ncstr].desc = a.desc; cs[ncstr].lo = cs[ncstr].hi = NAN;
            if (a.desc == '@' || a.desc == 'I' || a.desc == 'X' || a.desc == 'G') cs[ncstr].lo = cs[ncstr].hi = a.alt1_ft;
            else if (a.desc == '+' || a.desc == 'H' || a.desc == 'J' || a.desc == 'V') cs[ncstr].lo = a.alt1_ft;
            else if (a.desc == '-' || a.desc == 'Y') cs[ncstr].hi = a.alt1_ft;
            else if (a.desc == 'B') { cs[ncstr].hi = a.alt1_ft; cs[ncstr].lo = a.alt2_ft; }
            cs[ncstr].spd = s.desc ? s.speed : NAN; cs[ncstr].spd_desc = s.desc;
            cs[ncstr].lat = l->fix.lat; cs[ncstr].lon = l->fix.lon; cs[ncstr].dmin = 1e9; cs[ncstr].climb = l->cstr_type == A380X_CSTR_CLB;
            ncstr++;
        }
    }

    a380x_guidance_init(&gs, NULL);
    {
        A380XGuidanceInputs z; memset(&z, 0, sizeof z); z.powered = 1; z.on_ground = 1;
        a380x_guidance_destroy(&gs); a380x_guidance_init(&gs, &z);
    }
    gs.selected_altitude_ft = 24000;
    if (csv) fprintf(csv, "t,lat,lon,alt,cas,target,mach,vs,trk,bank,pitch,thrust,leg,xtk,vdev,path,d2d,phase,fma_a,fma_b,fma_c,des_sub\n");

    for (steps = 0; steps < 200000 && !done; ++steps, t += dt) {
        double rho, p, tt, a, mach, cas, var = magvar_at(ac.lat, ac.lon), throttle[4], lever, ra;
        const A380XFmsOutputs *o;
        const A380XFmgcOutputs *g;
        int takeoff_power, pilot_pitch = 0, fa, fb, fc, arm;
        double pitch_target = 0;
        isa(ac.h_ft, &rho, &p, &tt, &a);
        cas = cas_kt(ac.v_tas, ac.h_ft, &mach);
        ra = ac.h_ft - (a380x_geo_distance_nm(ac.lat, ac.lon, thr_lat, thr_lon) < 60 ? origin_elev : dest_elev);
        if (ac.on_ground) ra = 0;
        /* ---- simulated crew ---- */
        lever = levers_cl > 0 ? 25.0 / 45.0 : 1.0;
        if (!ac.on_ground && liftoff_t < 0) liftoff_t = t;
        o = a380x_fms_outputs(&fms);
        if (levers_cl == 0 && !ac.on_ground && o->thr_red_alt_valid && isfinite(o->thr_red_alt_ft) && ac.h_ft >= o->thr_red_alt_ft) levers_cl = t;
        takeoff_power = levers_cl == 0;
        /* ---- FMS ---- */
        memset(&fin, 0, sizeof fin);
        fin.utc_seconds = fmod(36000.0 + t, 86400.0);
        fin.fmc_powered[0] = fin.fmc_powered[1] = fin.fmc_powered[2] = 1;
        fin.switching_knob = 1;
        fin.position_valid = 1; fin.lat_deg = ac.lat; fin.lon_deg = ac.lon; fin.gps_primary = 1; fin.irs_aligned = 1;
        fin.ground_speed_kt = ac.gs_kt; fin.true_track_deg = ac.trk; fin.magnetic_variation_deg = var;
        fin.adr_valid = 1; fin.cas_kt = cas; fin.mach = mach; fin.tas_kt = ac.v_tas / KT;
        fin.baro_alt_ft = fin.pressure_alt_ft = ac.h_ft; fin.vertical_speed_fpm = ac.vs_fpm;
        fin.sat_valid = 1; fin.sat_c = tt - 273.15; fin.static_pressure_valid = 1; fin.static_pressure_hpa = p / 100.0;
        fin.radio_alt_valid = ra < 2500; fin.radio_alt_ft = ra;
        fin.wind_valid = 1; fin.wind_dir_true_deg = ac.wind_dir; fin.wind_speed_kt = ac.wind_kt;
        fin.fqms_gw_valid = 1; fin.fqms_gw_kg = ac.mass; fin.fqms_fob_valid = 1; fin.fqms_fob_kg = 90000;
        for (k = 0; k < 4; ++k) { fin.n1_pct[k] = takeoff_power ? 90 : 60; fin.n2_pct[k] = 90; }
        fin.on_ground = ac.on_ground;
        fin.flap_lever = ac.flaps; fin.spoiler_ratio = ac.speedbrake ? 0.5 : 0.0;
        fin.fcu_alt_valid = 1; fin.fcu_selected_alt_ft = gs.selected_altitude_ft;
        fin.fcu_speed_is_mach = gs.mach_mode; fin.fcu_selected_speed_kt = gs.selected_speed_knots; fin.fcu_selected_mach = gs.selected_mach;
        fin.speed_managed = gs.speed_managed;
        fin.vertical_mode = vm_of(takeoff_power, ac.on_ground, a380x_fms_phase(&fms));
        fin.lateral_mode = gs.lateral_mode == A380X_C1_NAV ? A380X_FMS_LM_NAV : gs.lateral_mode == A380X_C1_HDG ? A380X_FMS_LM_HDG : A380X_FMS_LM_NONE;
        fin.fcu_vs_fpm = gs.selected_vs_fpm; fin.fcu_fpa_deg = gs.selected_fpa_deg;
        fin.vnav_decel_reached = fmgc.out.decel_reached;
        fin.cold_and_dark_spawn = 0;
        a380x_fms_update(&fms, &fin, dt);
        o = a380x_fms_outputs(&fms);
        /* ---- FMGC ---- */
        fac.valid = 1; fac.lat = ac.lat; fac.lon = ac.lon; fac.alt_ft = (float)ac.h_ft; fac.true_track_deg = (float)ac.trk;
        fac.gs_kt = (float)ac.gs_kt; fac.on_ground = ac.on_ground; fac.nav_mode = gs.lateral_mode == A380X_C1_NAV;
        memset(&gin, 0, sizeof gin);
        gin.position_valid = 1; gin.lat_deg = ac.lat; gin.lon_deg = ac.lon; gin.alt_ft = ac.h_ft; gin.vs_fpm = ac.vs_fpm;
        gin.gs_kt = ac.gs_kt; gin.tas_kt = ac.v_tas / KT; gin.cas_kt = cas; gin.mach = mach;
        gin.true_track_deg = ac.trk; gin.true_heading_deg = ac.psi; gin.magvar_deg = var; gin.on_ground = ac.on_ground; gin.radio_alt_ft = ra;
        gin.fms_valid = o->valid; gin.phase = a380x_fms_phase(&fms); gin.cruise_alt_ft = o->cruise_altitude_ft;
        gin.fcu_alt_ft = gs.selected_altitude_ft;
        gin.managed_speed_kt = o->managed_speed_valid ? o->managed_speed_knots : 0; gin.managed_is_mach = o->managed_speed_is_mach;
        gin.managed_mach = o->managed_mach;
        gin.descent_speed_kt = perf(A380X_FMS_F_DES_MANAGED_SPD); gin.descent_mach = perf(A380X_FMS_F_DES_MANAGED_MACH);
        gin.des_limit_kt = perf(A380X_FMS_F_DES_SPD_LIM); gin.des_limit_alt_ft = perf(A380X_FMS_F_DES_SPD_LIM_ALT);
        gin.green_dot_kt = o->green_dot_knots; gin.s_speed_kt = o->s_speed_knots; gin.f_speed_kt = o->f_speed_knots;
        gin.vapp_kt = o->approach_vapp_knots; gin.vls_kt = o->vls_knots; gin.flap_lever = ac.flaps; gin.landing_conf3 = o->landing_conf3;
        gin.static_pressure_hpa = p / 100.0;
        gin.nav_engaged = gs.lateral_mode == A380X_C1_NAV; gin.nav_armed = gs.nav_armed; gin.vertical_managed = gs.altitude_managed;
        gin.idle_speed_descent = gs.thrust_mode == A380X_A1A2_THR_IDLE && gs.athr_active
            && (gs.vertical_mode == A380X_B1_OP_DES || (gs.vertical_mode == A380X_B1_DES && gs.des_submode == 1))
            && fabs(cas - gs.speed_target_knots) < 5.0;
        a380x_fmgc_update(&fmgc, fp, &gin, dt);
        g = a380x_fmgc_outputs(&fmgc);
        /* ---- guidance inputs (FCU adapter mapping) ---- */
        memset(&i, 0, sizeof i);
        i.powered = 1; i.on_ground = ac.on_ground; i.attitude_valid = i.airdata_valid = 1; i.athr_speed_valid = 1;
        i.ias_knots = cas; i.tas_knots = ac.v_tas / KT; i.mach = mach; i.ground_speed_knots = ac.gs_kt;
        i.altitude_ft = i.pressure_altitude_ft = ac.h_ft; i.radio_height_ft = ra; i.vertical_speed_fpm = ac.vs_fpm;
        i.pitch_deg = ac.theta; i.roll_deg = ac.phi;
        i.true_heading_deg = ac.psi; i.true_track_deg = ac.trk;
        i.heading_deg = fmod(ac.psi - var + 360.0, 360.0); i.track_deg = fmod(ac.trk - var + 360.0, 360.0);
        i.pitch_rate_deg_s = ac.q; i.roll_rate_deg_s = ac.p; i.yaw_rate_deg_s = 0;
        i.alpha_deg = ac.alpha; i.accel_z_m_s2 = (ac.nz - 1.0) * G0;
        i.weight_kg = ac.mass; i.vls_knots = o->vls_knots > 0 && !ac.on_ground ? o->vls_knots : 0; i.vmax_knots = 340;
        i.throttle_levers_valid = 1;
        for (k = 0; k < 4; ++k) i.throttle_lever_ratio[k] = lever;
        i.idle_detent_ratio = .01; i.cl_detent_ratio = 25.0 / 45.0; i.mct_detent_ratio = 35.0 / 45.0; i.toga_detent_ratio = 1; i.detent_tolerance_ratio = .02;
        i.throttle_ratio = gs.athr_active ? gs.throttle_command : fmin(1.0, lever / i.cl_detent_ratio * 0.88);
        if (o->valid && o->managed_speed_valid) {
            i.managed_speed_valid = 1; i.managed_speed_knots = o->managed_speed_knots;
            i.managed_speed_is_mach = o->managed_speed_is_mach; i.managed_mach = o->managed_mach;
        }
        if (g->speed_valid) { i.managed_speed_valid = 1; i.managed_speed_knots = g->speed_kt; i.managed_speed_is_mach = g->speed_is_mach; i.managed_mach = g->speed_mach; }
        if (o->valid && g->lnav_valid) {
            i.managed_route_valid = 1; i.managed_nav_capture = g->nav_capture;
            i.route_xtk_nm = g->fbw_xtk_nm; i.route_track_error_deg = g->fbw_tae_deg; i.route_bank_deg = g->bank_ff_deg;
            i.managed_heading_deg = g->managed_heading_mag_deg;
            if (isfinite(g->managed_altitude_ft)) {
                i.managed_vertical_valid = 1; i.managed_altitude_ft = g->managed_altitude_ft;
                i.managed_altitude_is_constraint = g->managed_altitude_is_constraint;
                i.managed_cruise_altitude_ft = o->cruise_altitude_ft > 0 ? o->cruise_altitude_ft : 0;
                i.managed_path_valid = g->vnav_valid && isfinite(g->path_alt_ft);
                i.managed_path_altitude_ft = g->path_alt_ft; i.managed_path_vs_fpm = g->path_vs_fpm; i.managed_vdev_ft = g->vdev_ft;
                i.managed_path_idle = g->path_segment == 0 || g->path_segment == 3; i.managed_before_tod = g->before_tod;
            }
        }
        i.fms_phase_valid = o->valid; i.fms_phase = a380x_fms_phase(&fms); i.fms_v2_knots = o->valid ? o->v2_knots : 0;
        /* ---- crew actions on the FCU ---- */
        if (!ac.on_ground && !ap_engaged && t - liftoff_t > 6 && ra > 100) {
            a380x_guidance_command(&gs, &i, A380X_FCU_AP1, 0); ap_engaged = gs.ap1;
            if (ap_engaged) printf("t=%6.1f AP1 engaged at %.0f ft\n", t, ac.h_ft);
        }
        if (ap_engaged && !gs.ap1) { printf("t=%6.1f AP1 DISCONNECTED at %.0f ft (cas %.0f pitch %.1f bank %.1f)\n", t, ac.h_ft, cas, ac.theta, ac.phi); stop_reason = 3; break; }
        if (levers_cl > 0 && levers_cl == t) printf("t=%6.1f THR RED %.0f ft: levers CL\n", t, ac.h_ft);
        if (fcu_des_set == 0 && a380x_fms_phase(&fms) == A380X_FMS_PHASE_CRUISE && isfinite(g->tod_dist_nm) && isfinite(g->dist_to_dest_nm)
            && g->dist_to_dest_nm <= g->tod_dist_nm + 2.0) {
            /* crew: descent clearance 3000 ft 2 NM before T/D, ALT push (early descent) */
            tod_pred = g->tod_dist_nm; des_start_d2d = g->dist_to_dest_nm;
            gs.selected_altitude_ft = 3000; a380x_fms_on_fcu_alt_turn(&fms);
            a380x_guidance_command(&gs, &i, A380X_FCU_ALT_PUSH, 0); a380x_fms_on_fcu_alt_push_pull(&fms);
            fcu_des_set = t;
            printf("t=%6.1f FCU 3000, ALT push %.1f NM before T/D (T/D %.1f NM from destination)\n", t, g->dist_to_dest_nm - g->tod_dist_nm, g->tod_dist_nm);
        }
        /* speed brakes on MORE DRAG (above the path at idle) or too fast at idle on the path; FCU 1000 ft
         * on the final (stands for the missed-approach / final-descent FCU setting) */
        {
            int fast = gs.vertical_mode == A380X_B1_DES && gs.thrust_mode == A380X_A1A2_THR_IDLE && cas > gs.speed_target_knots + 8;
            if (!ac.speedbrake && (g->more_drag || fast) && ac.flaps < 3) { ac.speedbrake = 1; n_spdbrk++; }
            if (ac.speedbrake && ((!g->more_drag && !fast && g->vdev_ft < 150 && cas < gs.speed_target_knots + 3) || ac.flaps >= 3
                                  || gs.vertical_mode != A380X_B1_DES)) ac.speedbrake = 0;
            if (ac.speedbrake) spdbrk_time += dt;
        }
        if (a380x_fms_phase(&fms) == A380X_FMS_PHASE_APPROACH && gs.selected_altitude_ft > 1000 && fcu_des_set > 0) {
            gs.selected_altitude_ft = 1000; a380x_fms_on_fcu_alt_turn(&fms);
            printf("t=%6.1f FCU 1000 ft (approach phase)\n", t);
        }
        /* flaps on schedule in the approach phase (selected at the target speed of the current configuration) */
        /* (the effective target includes the VLS floor, raised by extended speed brakes) */
        /* normal schedule: CONF 1 at the decel, CONF 2 inside 12 NM, gear + CONF 3 inside 8 NM, FULL inside 6 NM */
        if (a380x_fms_phase(&fms) == A380X_FMS_PHASE_APPROACH && !ac.on_ground && ac.flaps < 4 && g->speed_valid
            && cas < gs.speed_target_knots + 4 && cas > gs.speed_target_knots - 6
            && (ac.flaps == 0 || (ac.flaps == 1 && g->dist_to_dest_nm < 12) || (ac.flaps == 2 && g->dist_to_dest_nm < 8)
                || (ac.flaps == 3 && g->dist_to_dest_nm < 6))) {
            static double last_flap_t = -100;
            if (t - last_flap_t > 20) { ac.flaps++; last_flap_t = t; if (ac.flaps >= 3) ac.gear = 1;
                printf("t=%6.1f flaps %d at %.0f kt %.0f ft (managed target %.0f, effective %.0f)\n", t, ac.flaps, cas, ac.h_ft, g->speed_kt, gs.speed_target_knots); }
        }
        if (!ac.on_ground && ac.flaps > 0 && a380x_fms_phase(&fms) <= A380X_FMS_PHASE_CLIMB && cas > (ac.flaps == 2 ? 200 : 225)) {
            ac.flaps--; if (ac.flaps == 0) ac.gear = 0;
        }
        if (!ac.on_ground && ac.gear && ra > 300 && a380x_fms_phase(&fms) <= A380X_FMS_PHASE_CLIMB && ac.flaps < 2) ac.gear = 0;
        /* ---- guidance ---- */
        a380x_guidance_update(&gs, &i, dt);
        if (getenv("FMGC_DEBUG_T") && fabs(t - atof(getenv("FMGC_DEBUG_T"))) < 40 && steps % 40 == 0)
            printf("  dbg t=%.1f vert %d sub %d law %d Htgt %.0f VStgt %.0f vpath %d %.0f/%.0f spd_tgt %.0f ias %.0f pcmd %.2f pitch %.2f fd %.2f sb %d\n", t,
                   gs.vertical_mode, gs.des_submode, gs.vertical_law_override, gs.altitude_target_ft, gs.vs_target_fpm, gs.vpath_valid,
                   gs.vpath_altitude_ft, gs.vpath_vs_fpm, gs.speed_target_knots, cas, gs.pitch_command_deg, ac.theta, gs.fd_pitch_error_deg, ac.speedbrake);
        /* ---- actuators ---- */
        for (k = 0; k < 4; ++k) throttle[k] = gs.athr_active ? fmin(gs.throttle_command, lever / i.cl_detent_ratio * 0.88 + (lever > 0.99 ? 0.12 : 0)) : (lever > 0.99 ? 1.0 : lever / i.cl_detent_ratio * 0.88);
        if (ac.on_ground) {
            pilot_pitch = 1;
            pitch_target = cas >= 152 ? 12.5 : 0.0;
        } else if (!gs.ap1) {
            pilot_pitch = 1;
            pitch_target = gs.guidance_valid ? ac.theta + gs.fd_pitch_error_deg : 12.5;
            pitch_target = fmax(-5.0, fmin(15.0, pitch_target));
        }
        aircraft_step(&ac, throttle, gs.pitch_servo, gs.ap1 ? gs.roll_servo : 0.0, pilot_pitch, pitch_target, dt);
        if (!gs.ap1 && !ac.on_ground) { ac.phi += (gs.fd_roll_error_deg * 0.5 - 0) * dt; ac.phi = fmax(-25, fmin(25, ac.phi)); }
        if (ac.h_ft > max_alt) max_alt = ac.h_ft;
        /* ---- logging ---- */
        fa = gs.thrust_mode; fb = a380x_guidance_fma_vertical(&gs); fc = gs.lateral_mode;
        if (fb >= 0 && fb < 32) seen[fb] = 1;
        if (fa >= 0 && fa < 16 && gs.athr_active) thr_seen[fa] = 1;
        if (gs.vertical_mode == A380X_B1_DES && gs.des_submode >= 0 && gs.des_submode < 3) des_sub_seen[gs.des_submode] = 1;
        if (fc == A380X_C1_NAV && !isfinite(nav_engage_ra)) nav_engage_ra = ra;
        arm = gs.nav_armed | gs.clb_armed << 1 | gs.des_armed << 2 | gs.altitude_armed << 3 | gs.alt_constraint << 4;
        if (fa != prev_a || fb != prev_b1 || fc != prev_c1 || arm != prev_arm) {
            printf("t=%6.1f %6.0f ft %3.0f kt  FMA %-9s| %-8s| %-5s  armed:%s%s%s%s%s  phase %d leg %d\n", t, ac.h_ft, cas,
                   a_name(fa), b1_name(fb), c1_name(fc), gs.nav_armed ? " NAV" : "", gs.clb_armed ? " CLB" : "", gs.des_armed ? " DES" : "",
                   gs.altitude_armed ? (gs.alt_constraint ? " ALT(cst)" : " ALT") : "", gs.vertical_mode == A380X_B1_DES ? (gs.des_submode == 0 ? " [on path]" : gs.des_submode == 1 ? " [above/idle]" : " [below -1000]") : "",
                   a380x_fms_phase(&fms), g->active_leg);
            if ((fb == A380X_B1_ALT_CST || fb == A380X_B1_ALT || fb == A380X_B1_ALT_CRZ) && prev_b1 != fb && n_lvl < 8)
                lvl_cst_err[n_lvl++] = ac.h_ft - gs.held_altitude_ft;
            if (fb == A380X_B1_ALT_CRZ && prev_b1 != fb) crz_level_err = ac.h_ft - 24000;
            prev_a = fa; prev_b1 = fb; prev_c1 = fc; prev_arm = arm;
        }
        if (a380x_fms_phase(&fms) != prev_phase) { printf("t=%6.1f FMS phase %d at %.0f ft, %.1f NM to destination\n", t, a380x_fms_phase(&fms), ac.h_ft, g->dist_to_dest_nm); prev_phase = a380x_fms_phase(&fms); }
        if (g->active_leg != prev_leg) {
            const A380XFplnRoute *r = &hk_plan(fp, A380X_PLAN_ACTIVE)->pri;
            /* constraint compliance at the termination of the leg just completed */
            for (k = 0; k < ncstr; ++k) if (cs[k].leg == prev_leg && prev_leg >= 0) { cs[k].seen = 1; cs[k].seq_alt = ac.h_ft; cs[k].seq_cas = cas; }
            t_seq = t;
            if (g->active_leg >= 0) printf("t=%6.1f TO %-8s (%s) at %.0f ft %.0f kt\n", t, r->el[g->active_leg].ident, a380x_leg_type_name(r->el[g->active_leg].type), ac.h_ft, cas);
            prev_leg = g->active_leg;
        }
        /* constraint compliance: altitude / speed at the closest point to the fix */
        for (k = 0; k < ncstr; ++k) {
            double dfix = a380x_geo_distance_nm(ac.lat, ac.lon, cs[k].lat, cs[k].lon);
            if (dfix < cs[k].dmin && dfix < 5.0) { cs[k].dmin = dfix; cs[k].alt = ac.h_ft; cs[k].cas = cas; }
        }
        /* speed profile */
        if (climbing_log && ac.h_ft >= next_speed_log_alt) {
            printf("    climb %5.0f ft: CAS %.0f (target %.0f) M%.3f VS %.0f\n", ac.h_ft, cas, gs.speed_target_knots, mach, ac.vs_fpm);
            next_speed_log_alt += 4000;
            if (next_speed_log_alt > 24000) climbing_log = 0;
        }
        if (!climbing_log && fcu_des_set > 0 && ac.h_ft <= next_speed_log_alt && next_speed_log_alt > 1500) {
            printf("    descent %5.0f ft: CAS %.0f (target %.0f) M%.3f VS %.0f vdev %+.0f %s\n", ac.h_ft, cas, gs.speed_target_knots, mach, ac.vs_fpm,
                   g->vdev_ft, gs.thrust_mode == A380X_A1A2_THR_IDLE ? "THR IDLE" : a_name(gs.thrust_mode));
            next_speed_log_alt -= 2000;
        }
        if (!climbing_log && fcu_des_set == 0) next_speed_log_alt = 22000;
        /* path tracking statistics while NAV is engaged */
        if (gs.lateral_mode == A380X_C1_NAV && g->lnav_valid && g->kind != A380X_LNAV_TRACK && g->kind != A380X_LNAV_HEADING && g->kind != A380X_LNAV_DIRECT) {
            double x = fabs(g->xtk_right_nm);
            if (g->in_transition || g->kind == A380X_LNAV_ARC) { if (x > max_xtk_turn) max_xtk_turn = x; }
            else if (t - t_seq < 60.0) { if (x > max_xtk_recover) max_xtk_recover = x; }
            else if (x > max_xtk_line) { max_xtk_line = x; }
            sum_xtk += x; n_xtk++;
        }
        if (gs.vertical_mode == A380X_B1_DES && gs.des_submode == 0 && fabs(g->vdev_ft) > max_vdev_path && t - fcu_des_set > 5) max_vdev_path = fabs(g->vdev_ft);
        if (csv && steps % 20 == 0)
            fprintf(csv, "%.1f,%.6f,%.6f,%.0f,%.1f,%.1f,%.3f,%.0f,%.1f,%.1f,%.2f,%.0f,%d,%.3f,%.0f,%.0f,%.2f,%d,%d,%d,%d,%d\n", t, ac.lat, ac.lon,
                    ac.h_ft, cas, gs.speed_target_knots, mach, ac.vs_fpm, ac.trk, ac.phi, ac.theta, (ac.thrust_n[0] + ac.thrust_n[1] + ac.thrust_n[2] + ac.thrust_n[3]) / 4000.0,
                    g->active_leg, g->xtk_right_nm, g->vdev_ft, g->path_alt_ft, g->dist_to_dest_nm, a380x_fms_phase(&fms), fa, fb, fc, gs.des_submode);
        /* stop at 1500 ft above the destination on the final approach */
        if (fcu_des_set > 0 && ac.h_ft < dest_elev + 1500 && a380x_fms_phase(&fms) == A380X_FMS_PHASE_APPROACH) {
            final_xtk = g->xtk_right_nm; final_dist = g->dist_to_dest_nm; done = 1; stop_reason = 1;
        }
        if (ac.h_ft < dest_elev + 1000 && !ac.on_ground && t > 300) { stop_reason = 2; break; }
        if (!isfinite(ac.h_ft) || !isfinite(ac.v_tas) || !isfinite(ac.lat)) { stop_reason = 4; break; }
    }
    if (csv) fclose(csv);
    printf("\n==== FMGC closed-loop flight EGLL 27R BPK7F - LEST1M - ILS 23R EGCC (point-mass model, wind %03.0f/%.0f kt) ====\n", ac.wind_dir, ac.wind_kt);
    printf("end: %s after %.0f s at %.0f ft, %.1f NM to the threshold, final XTK %.3f NM, max altitude %.0f ft\n",
           stop_reason == 1 ? "1500 ft above the destination on the final approach" : stop_reason == 2 ? "TOO LOW" :
           stop_reason == 3 ? "AP disconnected" : stop_reason == 4 ? "numerical failure" : "time limit", t, ac.h_ft, final_dist, final_xtk, max_alt);
    printf("lateral: max |XTK| established on straight legs %.3f NM, within 60 s after sequencing %.3f NM, in transitions/arcs %.3f NM, mean %.3f NM (NAV engaged)\n",
           max_xtk_line, max_xtk_recover, max_xtk_turn, n_xtk ? sum_xtk / n_xtk : 0.0);
    printf("vertical: T/D predicted %.1f NM from destination, descent initiated at %.1f NM; max |vdev| on path %.0f ft;"
           " speed brakes %d times, %.0f s (MORE DRAG); learned idle factor %.2f\n", tod_pred, des_start_d2d, max_vdev_path,
           n_spdbrk, spdbrk_time, fmgc.idle_factor);
    printf("level-offs (altitude - held at ALT/ALT CST/ALT CRZ entry):");
    for (k = 0; k < n_lvl; ++k) printf(" %+.0f", lvl_cst_err[k]);
    printf(" ft; ALT CRZ FL240 entry error %+.0f ft\n", crz_level_err);
    printf("constraints at the fix (altitude window / speed):\n");
    printf("  (climb constraints are evaluated where the fix is sequenced, descent constraints at the closest point;"
           " fly-by fixes are not overflown)\n");
    for (k = 0; k < ncstr; ++k) {
        int ok = 1, sok = 1;
        if (!cs[k].seen) { printf("  %-8s not sequenced\n", cs[k].ident); continue; }
        if (cs[k].climb) { cs[k].alt = cs[k].seq_alt; cs[k].cas = cs[k].seq_cas; }
        if (isfinite(cs[k].lo) && cs[k].alt < cs[k].lo - 250) ok = 0;
        if (isfinite(cs[k].hi) && cs[k].alt > cs[k].hi + 250) ok = 0;
        if (isfinite(cs[k].spd) && cs[k].spd > 2 && cs[k].spd_desc == '-' && cs[k].cas > cs[k].spd + 10) sok = 0;
        printf("  %-8s (closest %.2f NM) alt %c%5.0f..%-5.0f actual %5.0f ft %s   spd %c%3.0f actual %3.0f kt %s\n", cs[k].ident, cs[k].dmin, cs[k].desc ? cs[k].desc : ' ',
               isfinite(cs[k].lo) ? cs[k].lo : -1, isfinite(cs[k].hi) ? cs[k].hi : -1, cs[k].alt, ok ? "OK" : "MISSED",
               cs[k].spd_desc ? cs[k].spd_desc : ' ', isfinite(cs[k].spd) ? cs[k].spd : 0, cs[k].cas, sok ? "OK" : "MISSED");
        CHECK(ok, "%s altitude constraint missed (%.0f ft)", cs[k].ident, cs[k].alt);
        CHECK(sok, "%s speed constraint missed (%.0f kt)", cs[k].ident, cs[k].cas);
    }
    printf("modes seen: SRS %d CLB %d ALT CST* %d ALT CST %d ALT CRZ %d DES %d (on path %d, above %d, below %d); "
           "A/THR THR CLB %d THR IDLE %d SPEED %d MACH %d; NAV engaged at %.0f ft RA\n",
           seen[A380X_B1_SRS], seen[A380X_B1_CLB], seen[A380X_B1_ALT_CST_STAR], seen[A380X_B1_ALT_CST], seen[A380X_B1_ALT_CRZ],
           seen[A380X_B1_DES], des_sub_seen[0], des_sub_seen[1], des_sub_seen[2], thr_seen[A380X_A1A2_THR_CLB],
           thr_seen[A380X_A1A2_THR_IDLE], thr_seen[A380X_A1A2_SPEED], thr_seen[A380X_A1A2_MACH], nav_engage_ra);
    CHECK(seen[A380X_B1_SRS] && seen[A380X_B1_CLB] && seen[A380X_B1_ALT_CST] && seen[A380X_B1_ALT_CRZ] && seen[A380X_B1_DES],
          "managed vertical mode sequence incomplete");
    CHECK(des_sub_seen[0] && des_sub_seen[1] && des_sub_seen[2], "DES path submodes incomplete");
    CHECK(thr_seen[A380X_A1A2_THR_CLB] && thr_seen[A380X_A1A2_THR_IDLE] && thr_seen[A380X_A1A2_SPEED], "A/THR mode coupling incomplete");
    CHECK(isfinite(nav_engage_ra) && nav_engage_ra >= 30 && nav_engage_ra < 60, "NAV engagement after takeoff at %.0f ft RA", nav_engage_ra);
    CHECK(stop_reason == 1, "flight did not reach 1500 ft on the final approach (reason %d)", stop_reason);
    CHECK(max_xtk_line < 0.3, "straight-leg XTK %.3f NM", max_xtk_line);
    CHECK(max_xtk_recover < 1.0, "XTK after sequencing %.3f NM", max_xtk_recover);
    CHECK(max_xtk_turn < 1.5, "transition XTK %.3f NM", max_xtk_turn);
    CHECK(fabs(final_xtk) < 0.3, "final approach XTK %.3f NM", final_xtk);
    CHECK(isfinite(crz_level_err) && fabs(crz_level_err) < 100, "ALT CRZ level-off %.0f ft", crz_level_err);
    CHECK(max_alt < 24200, "overshoot of FL240: %.0f ft", max_alt);
    a380x_guidance_destroy(&gs);
    a380x_fpln_destroy(fp);
    a380x_nav_destroy(nav);
    if (failures) { printf("FMGC closed-loop flight: %d check(s) FAILED\n", failures); return 1; }
    printf("FMGC closed-loop flight passed: SID/enroute/STAR/approach to 1500 ft, constraints met, FMA sequence above\n");
    return 0;
}
