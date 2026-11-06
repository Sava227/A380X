/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * X-Plane boundary of the native A380X FMS (WP-J, docs/mfd-spec/07-fmc-logic.md sections 9.4 and
 * 10, docs/mfd-spec/09-port-integration.md sections 1.2 and 1.11, spec 12 section 7):
 *   - navigation database (X-Plane files, Custom Data first) and the flight-plan store, with the
 *     X-Plane navaid / magnetic-variation host callbacks;
 *   - the FMS <-> flight-plan bridge (A380XFmsPlanHooks / A380XFmsNavHooks);
 *   - A380XFmsInputs from raw simulator data and the port's own systems (ADIRS, guidance, PFD
 *     configuration, EFIS). Stock X-Plane FMS / autopilot mode datarefs are never consumed;
 *   - FMS outputs to the FWS phase V1, the PFD V1/VR/V2, the managed speed target of the native
 *     guidance (fms_feed), DH for the PFD, EWD FLEX and the a380x/xp12/fms/ datarefs;
 *   - the A380XMfdSystemsHost used by the MFD pages (DATA/STATUS, DATA/AIRPORT, SURV, FCU BKUP,
 *     POSITION).
 *
 * Unity-build include of a380x_avionics.c, after a380x_mfd_adapter.inc. Statics are prefixed
 * fmsa_. Raw simulator inputs are listed with FMSA_RAW("...") and the published datarefs with
 * FMSR(...): build.py scans both for its DataRefs.txt and provider contracts.
 */
#include "a380x_elec_power.h"
#include "a380x_fmgc_guidance.h"

/* ---- Raw simulator inputs (spec 07 section 9.4, spec 06 section 12) ------------------------ */

enum {
    FMSA_IN_ZULU, FMSA_IN_LAT, FMSA_IN_LON, FMSA_IN_MAGVAR, FMSA_IN_TAS, FMSA_IN_SAT, FMSA_IN_BARO_PAS,
    FMSA_IN_WIND_DIR, FMSA_IN_WIND_SPD, FMSA_IN_M_TOTAL, FMSA_IN_M_FUEL, FMSA_IN_N2, FMSA_IN_STARTER,
    FMSA_IN_GPS_FAIL, FMSA_IN_GPS2_FAIL, FMSA_IN_GPS_POWER, FMSA_IN_GPS2_POWER, FMSA_IN_TRUE_PSI,
    FMSA_IN_COUNT
};
#define FMSA_RAW(name) name
static const char *const fmsa_raw_names[FMSA_IN_COUNT] = {
    FMSA_RAW("sim/time/zulu_time_sec"),
    FMSA_RAW("sim/flightmodel/position/latitude"),
    FMSA_RAW("sim/flightmodel/position/longitude"),
    FMSA_RAW("sim/flightmodel/position/magnetic_variation"),
    FMSA_RAW("sim/flightmodel/position/true_airspeed"),
    FMSA_RAW("sim/weather/aircraft/temperature_ambient_deg_c"),
    FMSA_RAW("sim/weather/aircraft/barometer_current_pas"),
    FMSA_RAW("sim/weather/aircraft/wind_now_direction_degt"),
    FMSA_RAW("sim/weather/aircraft/wind_now_speed_msc"),
    FMSA_RAW("sim/flightmodel/weight/m_total"),
    FMSA_RAW("sim/flightmodel/weight/m_fuel_total"),
    FMSA_RAW("sim/cockpit2/engine/indicators/N2_percent"),
    FMSA_RAW("sim/flightmodel2/engines/starter_is_running"),
    FMSA_RAW("sim/operation/failures/rel_gps"),
    FMSA_RAW("sim/operation/failures/rel_gps2"),
    FMSA_RAW("sim/cockpit2/radios/actuators/gps_power"),
    FMSA_RAW("sim/cockpit2/radios/actuators/gps2_power"),
    FMSA_RAW("sim/flightmodel/position/true_psi"),
};
#undef FMSA_RAW
static XPLMDataRef fmsa_raw[FMSA_IN_COUNT];

/* ---- State ---------------------------------------------------------------------------------- */

static A380XFms fmsa_fms;
static A380XNavdata *fmsa_nav;
static A380XFlightPlans *fmsa_fpln;
static A380XFplnAircraft fmsa_aircraft;
static A380XFmsInputs fmsa_in;
static A380XMfdSystemsHost fmsa_host;
static int fmsa_initialized, fmsa_nav_loaded;
static unsigned fmsa_nav_seq;
static double fmsa_fpln_timer;
static int fmsa_cold_and_dark = 1;
static char fmsa_xplane_root[512];
/* FMGC managed guidance (LNAV/VNAV/managed speed, a380x_fmgc_guidance.c) and its MFD predictions. */
static A380XFmgc fmsa_fmgc;
static A380XFmgcInputs fmsa_fmgc_in;
static A380XFplnPredFeed fmsa_pred_feed;
static A380XFplnWptPred fmsa_leg_pred[A380X_FPLN_MAX_ELEMENTS];
static int fmsa_pred_valid;

/* Writable controls (spec 07 section 10.3). */
typedef struct {
    int switching_knob;
    int fmc_failed[3], fmc_reset[3];
} FmsaControls;
static FmsaControls fmsa_ctl = {1, {0, 0, 0}, {0, 0, 0}};

static double fmsa_nan(void) { return (double)NAN; }

static float fmsa_getf(int i) { return fmsa_raw[i] ? XPLMGetDataf(fmsa_raw[i]) : NAN; }
static double fmsa_getd(int i) { return fmsa_raw[i] ? XPLMGetDatad(fmsa_raw[i]) : NAN; }
static int fmsa_geti(int i) { return fmsa_raw[i] ? XPLMGetDatai(fmsa_raw[i]) : 0; }

/* ---- Navdata host (XPLMFindNavAid / XPLMGetNavAidInfo / XPLMGetMagneticVariation) ---------- */

static int fmsa_nav_airport_info(void *ctx, const char *icao, A380XNavAirportInfo *out)
{
    XPLMNavRef ref;
    float lat = 0.0f, lon = 0.0f, height_m = 0.0f, heading = 0.0f;
    int freq = 0;
    XPLMNavType type = xplm_Nav_Unknown;
    char id[32] = "", name[256] = "";
    (void)ctx;
    if (!icao || !*icao || !out) return 0;
    ref = XPLMFindNavAid(NULL, icao, NULL, NULL, NULL, xplm_Nav_Airport);
    if (ref == XPLM_NAV_NOT_FOUND) return 0;
    XPLMGetNavAidInfo(ref, &type, &lat, &lon, &height_m, &freq, &heading, id, name, NULL);
    if (type != xplm_Nav_Airport || strcmp(id, icao) != 0) return 0; /* the ident is a fragment match */
    out->lat = lat; out->lon = lon;
    out->elev_ft = height_m * 3.2808399f; /* metres in XPLMGetNavAidInfo */
    snprintf(out->name, sizeof(out->name), "%s", name);
    return 1;
}

static int fmsa_nav_magvar(void *ctx, double lat, double lon, double *out_deg)
{
    float v;
    (void)ctx;
    v = XPLMGetMagneticVariation(lat, lon);
    if (!isfinite(v)) return 0;
    *out_deg = v;
    return 1;
}

static void fmsa_nav_log(void *ctx, const char *message)
{
    (void)ctx;
    XPLMDebugString("A380X navdata: ");
    XPLMDebugString(message);
    XPLMDebugString("\n");
}

/* ---- FMS nav hook ----------------------------------------------------------------------------- */

static int fmsa_fms_airport(void *ctx, const char *icao, A380XFmsAirportInfo *out)
{
    A380XNavAirportInfo info;
    (void)ctx;
    memset(out, 0, sizeof(*out));
    if (!fmsa_nav || !icao || !*icao) return 0;
    memset(&info, 0, sizeof(info));
    if (!a380x_nav_airport_info(fmsa_nav, icao, &info)) return 0;
    out->found = 1;
    snprintf(out->ident, sizeof(out->ident), "%s", info.icao[0] ? info.icao : icao);
    out->elevation_ft = isfinite(info.elev_ft) ? (double)info.elev_ft : fmsa_nan();
    out->lat_deg = info.lat; out->lon_deg = info.lon;
    out->trans_alt_ft = isfinite(info.trans_alt_ft) ? (double)info.trans_alt_ft : fmsa_nan();
    out->trans_level_fl = isfinite(info.trans_level_ft) ? (double)info.trans_level_ft / 100.0 : fmsa_nan();
    return 1;
}

/* ---- FMS <-> flight-plan bridge (A380XFmsPlanHooks) ---------------------------------------- */

static const A380XFplnPlan *fmsa_plan(struct A380XFlightPlans *p, int plan)
{
    return p && plan >= 0 && plan < A380X_PLAN_COUNT ? a380x_fpln_plan(p, plan) : NULL;
}

static int fmsa_hk_plan_exists(struct A380XFlightPlans *p, int plan) { return fmsa_plan(p, plan) != NULL; }

static unsigned fmsa_hk_plan_version(struct A380XFlightPlans *p, int plan)
{
    const A380XFplnPlan *pl = fmsa_plan(p, plan);
    return pl ? (unsigned)pl->version : 0u;
}

static int fmsa_hk_city_pair(struct A380XFlightPlans *p, int plan, char origin[8], char dest[8], char altn[8])
{
    const A380XFplnPlan *pl = fmsa_plan(p, plan);
    origin[0] = dest[0] = altn[0] = '\0';
    if (!pl) return 0;
    snprintf(origin, 8, "%.7s", pl->pri.origin_icao);
    snprintf(dest, 8, "%.7s", pl->pri.dest_icao);
    snprintf(altn, 8, "%.7s", pl->alt.dest_icao);
    return 1;
}

static int fmsa_runway_info(struct A380XFlightPlans *p, const char *icao, const char *rwy, A380XFmsRunwayInfo *out)
{
    const A380XNavAirport *apt;
    const A380XNavRunway *r;
    memset(out, 0, sizeof(*out));
    out->threshold_elev_ft = out->length_m = out->magnetic_bearing_deg = fmsa_nan();
    out->threshold_lat_deg = out->threshold_lon_deg = fmsa_nan();
    if (!p || !p->nav || !icao[0] || !rwy[0]) return 0;
    apt = a380x_nav_airport(p->nav, icao);
    if (!apt) return 0;
    r = a380x_nav_runway(apt, rwy);
    if (!r) return 0;
    out->valid = 1;
    snprintf(out->ident, sizeof(out->ident), "%.7s", r->designator);
    if (isfinite(r->thr_elev_ft)) out->threshold_elev_ft = r->thr_elev_ft;
    if (isfinite(r->length_ft)) out->length_m = (double)r->length_ft * 0.3048;
    if (isfinite(r->mag_brg)) out->magnetic_bearing_deg = r->mag_brg;
    out->threshold_lat_deg = r->thr_lat; out->threshold_lon_deg = r->thr_lon;
    return 1;
}

static int fmsa_hk_origin_runway(struct A380XFlightPlans *p, int plan, A380XFmsRunwayInfo *out)
{
    const A380XFplnPlan *pl = fmsa_plan(p, plan);
    if (!pl) { memset(out, 0, sizeof(*out)); return 0; }
    return fmsa_runway_info(p, pl->pri.origin_icao, pl->pri.origin_rwy, out);
}

static int fmsa_hk_destination_runway(struct A380XFlightPlans *p, int plan, A380XFmsRunwayInfo *out)
{
    const A380XFplnPlan *pl = fmsa_plan(p, plan);
    if (!pl) { memset(out, 0, sizeof(*out)); return 0; }
    return fmsa_runway_info(p, pl->pri.dest_icao, pl->pri.dest_rwy, out);
}

static A380XFmsApproachType fmsa_approach_type(A380XApproachType t)
{
    switch (t) {
    case A380X_APPR_ILS: return A380X_FMS_APPR_ILS;
    case A380X_APPR_GLS: return A380X_FMS_APPR_GLS;
    case A380X_APPR_IGS: return A380X_FMS_APPR_IGS;
    case A380X_APPR_LOC: return A380X_FMS_APPR_LOC;
    case A380X_APPR_LOC_BACKCOURSE: return A380X_FMS_APPR_LOC_BC;
    case A380X_APPR_LDA: return A380X_FMS_APPR_LDA;
    case A380X_APPR_SDF: return A380X_FMS_APPR_SDF;
    case A380X_APPR_GPS: return A380X_FMS_APPR_GPS;
    case A380X_APPR_RNAV: return A380X_FMS_APPR_RNAV;
    case A380X_APPR_VOR: case A380X_APPR_VORDME: case A380X_APPR_VORTAC: case A380X_APPR_TACAN:
        return A380X_FMS_APPR_VOR;
    case A380X_APPR_NDB: case A380X_APPR_NDBDME: return A380X_FMS_APPR_NDB;
    case A380X_APPR_FMS: return A380X_FMS_APPR_FMS;
    default: return A380X_FMS_APPR_NONE;
    }
}

static int fmsa_hk_approach(struct A380XFlightPlans *p, int plan, A380XFmsApproachInfo *out)
{
    const A380XFplnPlan *pl = fmsa_plan(p, plan);
    const A380XNavAirport *apt;
    const A380XNavProcedure *pr;
    memset(out, 0, sizeof(*out));
    if (!pl || !pl->pri.approach[0] || !pl->pri.dest_icao[0]) return 0;
    out->valid = 1;
    a380x_fpln_approach_name(p, &pl->pri, out->name, sizeof(out->name));
    apt = a380x_nav_airport(p->nav, pl->pri.dest_icao);
    pr = apt ? a380x_nav_procedure(apt, A380X_PROC_APPROACH, pl->pri.approach) : NULL;
    out->type = pr ? fmsa_approach_type(pr->appr_type) : A380X_FMS_APPR_NONE;
    return 1;
}

static double fmsa_hk_distance_to_destination(struct A380XFlightPlans *p, int plan)
{
    float d;
    if (!fmsa_plan(p, plan)) return fmsa_nan();
    d = a380x_fpln_dist_to_dest(p, plan, &fmsa_aircraft);
    return isfinite(d) ? (double)d : fmsa_nan();
}

/* Managed profile queries (spec 07 sections 5.7 / 6.3): shared with the FMGC guidance and its
 * offline tests in a380x_fmgc_guidance.c (moved there unchanged from this adapter). */
static double fmsa_hk_lowest_climb_constraint(struct A380XFlightPlans *p, int plan)
{
    return a380x_fmgc_lowest_climb_constraint(p, plan);
}

static double fmsa_hk_speed_constraint(struct A380XFlightPlans *p, int phase)
{
    return a380x_fmgc_speed_constraint(p, &fmsa_aircraft, phase);
}

static int fmsa_hk_constraint_altitudes(struct A380XFlightPlans *p, double *climb_alt_ft, double *descent_alt_ft)
{
    return a380x_fmgc_constraint_altitudes(p, climb_alt_ft, descent_alt_ft);
}

static int fmsa_hk_active_legs_compatible(struct A380XFlightPlans *p, int sec_plan)
{
    const A380XFplnPlan *a = fmsa_plan(p, A380X_PLAN_ACTIVE), *s = fmsa_plan(p, sec_plan);
    int ia, is;
    if (!s) return 0;
    ia = a ? a->active_leg_index : -1;
    is = s->active_leg_index;
    if (ia < 0 && is < 0) return 1;
    if (ia != is || ia < 0 || ia >= a->pri.count || is >= s->pri.count) return 0;
    return strcmp(a->pri.el[ia].ident, s->pri.el[is].ident) == 0;
}

static int fmsa_hk_new_city_pair(struct A380XFlightPlans *p, int plan, const char *from, const char *to, const char *altn)
{
    return a380x_fpln_new_city_pair(p, plan, from, to, altn) == A380X_FPLN_OK;
}

static int fmsa_hk_set_alternate(struct A380XFlightPlans *p, int plan, const char *altn)
{
    return a380x_fpln_set_alternate(p, plan, altn) == A380X_FPLN_OK;
}

static int fmsa_hk_tmpy_insert(struct A380XFlightPlans *p) { return a380x_fpln_tmpy_insert(p, &fmsa_aircraft) == A380X_FPLN_OK; }
static int fmsa_hk_tmpy_erase(struct A380XFlightPlans *p) { return a380x_fpln_tmpy_erase(p) == A380X_FPLN_OK; }

static int fmsa_hk_sec_copy_from_active(struct A380XFlightPlans *p, int sec_plan)
{
    return a380x_fpln_sec_copy_from_active(p, sec_plan - A380X_FMS_PLAN_SEC1 + 1) == A380X_FPLN_OK;
}

static int fmsa_hk_sec_copy_from_sec(struct A380XFlightPlans *p, int src_plan, int dst_plan)
{
    return a380x_fpln_sec_copy(p, src_plan, dst_plan - A380X_FMS_PLAN_SEC1 + 1) == A380X_FPLN_OK;
}

static int fmsa_hk_sec_delete(struct A380XFlightPlans *p, int sec_plan)
{
    return a380x_fpln_sec_delete(p, sec_plan - A380X_FMS_PLAN_SEC1 + 1) == A380X_FPLN_OK;
}

static int fmsa_hk_sec_swap(struct A380XFlightPlans *p, int sec_plan)
{
    return a380x_fpln_sec_swap(p, sec_plan - A380X_FMS_PLAN_SEC1 + 1) == A380X_FPLN_OK;
}

static int fmsa_hk_delete_cruise_steps(struct A380XFlightPlans *p)
{
    const A380XFplnPlan *pl = fmsa_plan(p, A380X_PLAN_ACTIVE);
    int i, deleted = 0, start;
    if (!pl) return 0;
    start = pl->active_leg_index < 0 ? 0 : pl->active_leg_index;
    for (i = pl->pri.count - 1; i >= start; --i) {
        pl = fmsa_plan(p, A380X_PLAN_ACTIVE);
        if (!pl || i >= pl->pri.count) continue;
        if (pl->pri.el[i].cruise_step.valid && a380x_fpln_remove_cruise_step(p, A380X_PLAN_ACTIVE, i) == A380X_FPLN_OK)
            ++deleted;
    }
    return deleted;
}

static void fmsa_hk_reset_all(struct A380XFlightPlans *p) { a380x_fpln_reset(p); }

static void fmsa_plan_hooks(A380XFmsPlanHooks *h)
{
    memset(h, 0, sizeof(*h));
    if (!fmsa_fpln) return;
    h->plans = fmsa_fpln;
    h->plan_exists = fmsa_hk_plan_exists;
    h->plan_version = fmsa_hk_plan_version;
    h->city_pair = fmsa_hk_city_pair;
    h->origin_runway = fmsa_hk_origin_runway;
    h->destination_runway = fmsa_hk_destination_runway;
    h->approach = fmsa_hk_approach;
    h->distance_to_destination_nm = fmsa_hk_distance_to_destination;
    h->lowest_climb_constraint_ft = fmsa_hk_lowest_climb_constraint;
    h->speed_constraint_kt = fmsa_hk_speed_constraint;
    h->constraint_altitudes = fmsa_hk_constraint_altitudes;
    h->active_legs_compatible = fmsa_hk_active_legs_compatible;
    h->new_city_pair = fmsa_hk_new_city_pair;
    h->set_alternate = fmsa_hk_set_alternate;
    h->tmpy_insert = fmsa_hk_tmpy_insert;
    h->tmpy_erase = fmsa_hk_tmpy_erase;
    h->sec_copy_from_active = fmsa_hk_sec_copy_from_active;
    h->sec_copy_from_sec = fmsa_hk_sec_copy_from_sec;
    h->sec_init = NULL; /* FBW creates only the data store; the lateral SEC is created by its first edit */
    h->sec_delete = fmsa_hk_sec_delete;
    h->sec_swap = fmsa_hk_sec_swap;
    h->delete_cruise_steps = fmsa_hk_delete_cruise_steps;
    h->reset_all = fmsa_hk_reset_all;
}

/* ---- Published FMS datarefs (spec 07 section 10.3) ------------------------------------------ */

enum { FMSA_T_DOUBLE = 0, FMSA_T_INT, FMSA_T_UNSIGNED, FMSA_T_CTL };

typedef struct {
    const char *name;
    int type;
    size_t offset;   /* into A380XFmsOutputs (or fmsa_ctl for FMSA_T_CTL) */
    int lo, hi;      /* writable range (FMSA_T_CTL) */
    XPLMDataRef handle;
} FmsaRef;

#define FMSR(field) {"a380x/xp12/fms/" #field, FMSA_T_DOUBLE, offsetof(A380XFmsOutputs, field), 0, 0, NULL}
#define FMSR_I(field) {"a380x/xp12/fms/" #field, FMSA_T_INT, offsetof(A380XFmsOutputs, field), 0, 0, NULL}
#define FMSR_AS(name, type, member) {"a380x/xp12/fms/" name, type, offsetof(A380XFmsOutputs, member), 0, 0, NULL}
#define FMSR_CTL(name, member, lo, hi) {"a380x/xp12/fms/" name, FMSA_T_CTL, offsetof(FmsaControls, member), lo, hi, NULL}
static FmsaRef fmsa_refs[] = {
    FMSR_I(flight_phase), FMSR(v1_knots), FMSR(vr_knots), FMSR(v2_knots), FMSR(to_flex_temp_c),
    FMSR_I(to_power_setting), FMSR_I(to_flaps), FMSR(to_ths_for_pct), FMSR_I(to_speeds_not_inserted),
    FMSR_I(vspeeds_disagree), FMSR_I(to_speeds_too_low),
    FMSR(thr_red_alt_ft), FMSR(acc_alt_ft), FMSR(eo_acc_alt_ft),
    FMSR_I(thr_red_alt_valid), FMSR_I(acc_alt_valid), FMSR_I(eo_acc_alt_valid),
    FMSR(trans_alt_ft), FMSR(trans_level_fl), FMSR_I(trans_alt_valid), FMSR_I(trans_level_valid),
    FMSR(cruise_altitude_ft), FMSR(fg_altitude_constraint_ft),
    FMSR(managed_speed_knots), FMSR_I(managed_speed_is_mach), FMSR(managed_mach), FMSR_I(managed_speed_valid),
    FMSR(presel_speed_knots), FMSR(presel_mach),
    FMSR(green_dot_knots), FMSR(f_speed_knots), FMSR(s_speed_knots), FMSR(vls_knots),
    FMSR(approach_vapp_knots), FMSR(approach_vls_knots), FMSR(approach_vref_knots),
    FMSR(approach_f_knots), FMSR(approach_s_knots), FMSR(approach_gd_knots),
    FMSR_I(landing_conf3), FMSR(approach_headwind_knots), FMSR_I(approach_headwind_valid),
    FMSR(dh_ft), FMSR(mda_ft), FMSR_I(minimums_in_range),
    FMSR(landing_elevation_ft), FMSR(dest_lat_deg), FMSR(dest_lon_deg), FMSR_I(landing_elevation_valid),
    FMSR_I(dest_pos_valid),
    FMSR(gross_weight_kg), FMSR(zfw_kg), FMSR(zfw_cg_pct),
    FMSR_I(dest_efob_below_min), FMSR_I(msg_check_speed_mode), FMSR_I(msg_set_hold_speed), FMSR_I(msg_td_reached),
    FMSR_AS("nd_fm_message_flags_l", FMSA_T_UNSIGNED, nd_fm_message_flags[0]),
    FMSR_AS("nd_fm_message_flags_r", FMSA_T_UNSIGNED, nd_fm_message_flags[1]),
    FMSR_AS("fmc_a_healthy", FMSA_T_INT, fmc_healthy[0]),
    FMSR_AS("fmc_b_healthy", FMSA_T_INT, fmc_healthy[1]),
    FMSR_AS("fmc_c_healthy", FMSA_T_INT, fmc_healthy[2]),
    FMSR_AS("fms_l_failed", FMSA_T_INT, fms_failed[0]),
    FMSR_AS("fms_r_failed", FMSA_T_INT, fms_failed[1]),
    FMSR_CTL("switching_knob", switching_knob, 0, 2),
    FMSR_CTL("fmc_a_failed", fmc_failed[0], 0, 1), FMSR_CTL("fmc_b_failed", fmc_failed[1], 0, 1),
    FMSR_CTL("fmc_c_failed", fmc_failed[2], 0, 1),
    FMSR_CTL("fmc_a_reset", fmc_reset[0], 0, 1), FMSR_CTL("fmc_b_reset", fmc_reset[1], 0, 1),
    FMSR_CTL("fmc_c_reset", fmc_reset[2], 0, 1),
};
#undef FMSR
#undef FMSR_I
#undef FMSR_AS
#undef FMSR_CTL

static double fmsa_ref_value(const FmsaRef *r)
{
    const char *base = r->type == FMSA_T_CTL ? (const char *)&fmsa_ctl : (const char *)&fmsa_fms.out;
    const char *p = base + r->offset;
    switch (r->type) {
    case FMSA_T_DOUBLE: return *(const double *)p;
    case FMSA_T_UNSIGNED: return (double)*(const unsigned *)p;
    default: return (double)*(const int *)p;
    }
}

static float fmsa_ref_getf(void *refcon) { return (float)fmsa_ref_value((const FmsaRef *)refcon); }

static int fmsa_ref_geti(void *refcon)
{
    double v = fmsa_ref_value((const FmsaRef *)refcon);
    return isfinite(v) ? (int)lround(v) : 0;
}

static void fmsa_ref_seti(void *refcon, int v)
{
    FmsaRef *r = (FmsaRef *)refcon;
    if (!r || r->type != FMSA_T_CTL) return;
    *(int *)((char *)&fmsa_ctl + r->offset) = v < r->lo ? r->lo : v > r->hi ? r->hi : v;
}

static void fmsa_ref_setf(void *refcon, float v) { fmsa_ref_seti(refcon, (int)lroundf(v)); }

static void fmsa_unregister_refs(void)
{
    int i;
    for (i = 0; i < ARRAY_LEN(fmsa_refs); ++i)
        if (fmsa_refs[i].handle) { XPLMUnregisterDataAccessor(fmsa_refs[i].handle); fmsa_refs[i].handle = NULL; }
}

static int fmsa_register_refs(void)
{
    int i;
    for (i = 0; i < ARRAY_LEN(fmsa_refs); ++i) {
        if (XPLMFindDataRef(fmsa_refs[i].name)) {
            XPLMDebugString("A380X FMS: custom dataref collision ");
            XPLMDebugString(fmsa_refs[i].name);
            XPLMDebugString("\n");
            return 0;
        }
    }
    for (i = 0; i < ARRAY_LEN(fmsa_refs); ++i) {
        FmsaRef *r = &fmsa_refs[i];
        int w = r->type == FMSA_T_CTL;
        r->handle = XPLMRegisterDataAccessor(r->name, xplmType_Int | xplmType_Float, w,
            fmsa_ref_geti, w ? fmsa_ref_seti : NULL, fmsa_ref_getf, w ? fmsa_ref_setf : NULL,
            NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, r, w ? r : NULL);
        if (!r->handle) { fmsa_unregister_refs(); return 0; }
    }
    return 1;
}

/* ---- Inputs (spec 07 section 9.4) ------------------------------------------------------------ */

static A380XFmsVerticalMode fmsa_vertical_mode(int takeoff_power, int on_ground, int phase)
{
    if (takeoff_power) { /* [PORT] no SRS in the FCU port: lever predicate (spec 07 section 2.1) */
        if (on_ground) return A380X_FMS_VM_SRS;
        if (phase == A380X_FMS_PHASE_APPROACH || phase == A380X_FMS_PHASE_GOAROUND) return A380X_FMS_VM_SRS_GA;
        if (phase <= A380X_FMS_PHASE_TAKEOFF) return A380X_FMS_VM_SRS;
    }
    switch (guidance.vertical_mode) {
    case A380X_B1_GS: case A380X_B1_F_GS: case A380X_B1_GS_STAR: case A380X_B1_F_GS_STAR: return A380X_FMS_VM_GS;
    case A380X_B1_SRS: return on_ground || phase <= A380X_FMS_PHASE_TAKEOFF ? A380X_FMS_VM_SRS : A380X_FMS_VM_SRS_GA;
    case A380X_B1_TCAS: return A380X_FMS_VM_TCAS;
    case A380X_B1_APP_DES: return A380X_FMS_VM_FINAL;
    case A380X_B1_DES: return A380X_FMS_VM_DES;
    case A380X_B1_OP_DES: return A380X_FMS_VM_OP_DES;
    case A380X_B1_CLB: return A380X_FMS_VM_CLB;
    case A380X_B1_OP_CLB: return A380X_FMS_VM_OP_CLB;
    case A380X_B1_ALT: return A380X_FMS_VM_ALT;
    case A380X_B1_ALT_STAR: return A380X_FMS_VM_ALT_STAR;
    case A380X_B1_ALT_CST: return A380X_FMS_VM_ALT_CST;
    case A380X_B1_ALT_CST_STAR: return A380X_FMS_VM_ALT_CST_STAR;
    case A380X_B1_ALT_CRZ: return A380X_FMS_VM_ALT_CRZ;
    case A380X_B1_ALT_CRZ_STAR: return A380X_FMS_VM_ALT_CRZ_STAR;
    case A380X_B1_FPA: return A380X_FMS_VM_FPA;
    case A380X_B1_VS: return A380X_FMS_VM_VS;
    default: return A380X_FMS_VM_NONE;
    }
}

static A380XFmsLateralMode fmsa_lateral_mode(void)
{
    switch (guidance.lateral_mode) {
    case A380X_C1_GA_TRK: return A380X_FMS_LM_GA_TRK;
    case A380X_C1_LOC_BC_STAR: case A380X_C1_LOC_STAR: case A380X_C1_F_LOC_STAR: return A380X_FMS_LM_LOC_CPT;
    case A380X_C1_HDG: return A380X_FMS_LM_HDG;
    case A380X_C1_RWY: return A380X_FMS_LM_RWY;
    case A380X_C1_RWY_TRK: return A380X_FMS_LM_RWY_TRK;
    case A380X_C1_TRACK: return A380X_FMS_LM_TRK;
    case A380X_C1_LOC_BC: case A380X_C1_LOC: case A380X_C1_F_LOC: return A380X_FMS_LM_LOC_TRACK;
    case A380X_C1_NAV: return A380X_FMS_LM_NAV;
    default: return A380X_FMS_LM_NONE;
    }
}

static int fmsa_mfd_on_init_fuel_perf(void)
{
    int s;
    for (s = 0; s < MFD_SIDE_COUNT; ++s) {
        const char *u = a380x_mfd_active_uri(&mfda_mfd, (MfdSide)s);
        if (!strncmp(u, "fms/active/init", 15) || !strncmp(u, "fms/active/fuel-load", 20)
            || !strncmp(u, "fms/active/perf", 15)) return 1;
    }
    return 0;
}

static void fmsa_read_inputs(int powered)
{
    A380XFmsInputs *in = &fmsa_in;
    const Side *s = &refs.side[0];
    int i, takeoff = 0, irs_nav = 0, gps1, gps2;
    float n2[4] = {0, 0, 0, 0};
    int starting[4] = {0, 0, 0, 0};
    double zulu, sat, pas, wdir, wspd, gw, fob;
    memset(in, 0, sizeof(*in));
    zulu = fmsa_getf(FMSA_IN_ZULU);
    in->utc_seconds = isfinite(zulu) ? fmod(fmod(zulu, 86400.0) + 86400.0, 86400.0) : fmsa_nan();
    for (i = 0; i < 3; ++i) {
        in->fmc_powered[i] = powered;
        in->fmc_failed[i] = fmsa_ctl.fmc_failed[i];
        in->fmc_reset[i] = fmsa_ctl.fmc_reset[i];
    }
    in->switching_knob = fmsa_ctl.switching_knob;
    for (i = 0; i < A380X_ADIRS_UNITS; ++i) if (adirs.unit[i].nav_valid) irs_nav = 1;
    gps1 = fmsa_geti(FMSA_IN_GPS_POWER) && fmsa_geti(FMSA_IN_GPS_FAIL) < 6;
    gps2 = fmsa_geti(FMSA_IN_GPS2_POWER) && fmsa_geti(FMSA_IN_GPS2_FAIL) < 6;
    in->lat_deg = fmsa_getd(FMSA_IN_LAT);
    in->lon_deg = fmsa_getd(FMSA_IN_LON);
    in->position_valid = powered && (irs_nav || gps1 || gps2) && isfinite(in->lat_deg) && isfinite(in->lon_deg);
    in->gps_primary = powered && (gps1 || gps2) && in->position_valid;
    in->irs_aligned = irs_nav;
    in->ground_speed_kt = ground_speed_knots;
    in->true_track_deg = XPLMGetDataf(refs.ground_track); /* hpath, true */
    in->magnetic_variation_deg = fmsa_getf(FMSA_IN_MAGVAR);
    in->adr_valid = s->air_valid && s->altitude_valid;
    in->cas_kt = in->adr_valid ? s->ias_value : 0.0;
    in->mach = s->mach_value;
    in->tas_kt = fmsa_getf(FMSA_IN_TAS) * 1.94384449;
    in->baro_alt_ft = s->altitude_value;
    in->pressure_alt_ft = guidance_inputs.pressure_altitude_ft;
    in->vertical_speed_fpm = s->vertical_speed_value;
    sat = fmsa_getf(FMSA_IN_SAT);
    in->sat_valid = isfinite(sat);
    in->sat_c = in->sat_valid ? sat : 15.0;
    pas = fmsa_getf(FMSA_IN_BARO_PAS);
    in->static_pressure_valid = isfinite(pas) && pas > 1000.0;
    in->static_pressure_hpa = in->static_pressure_valid ? pas / 100.0 : 1013.25;
    in->radio_alt_valid = s->radio_altitude_valid;
    in->radio_alt_ft = s->radio_altitude_value;
    wdir = fmsa_getf(FMSA_IN_WIND_DIR);
    wspd = fmsa_getf(FMSA_IN_WIND_SPD);
    in->wind_valid = isfinite(wdir) && isfinite(wspd);
    in->wind_dir_true_deg = wdir;
    in->wind_speed_kt = wspd * 1.94384449;
    gw = fmsa_getf(FMSA_IN_M_TOTAL);
    fob = fmsa_getf(FMSA_IN_M_FUEL);
    in->fqms_gw_valid = isfinite(gw) && gw > 0.0;
    in->fqms_gw_kg = gw;
    in->fqms_fob_valid = isfinite(fob) && fob >= 0.0;
    in->fqms_fob_kg = fob;
    in->fqms_cg_valid = 0; /* no W&B CG source yet (spec 07 section 4.1) */
    if (fmsa_raw[FMSA_IN_N2]) XPLMGetDatavf(fmsa_raw[FMSA_IN_N2], n2, 0, 4);
    if (fmsa_raw[FMSA_IN_STARTER]) XPLMGetDatavi(fmsa_raw[FMSA_IN_STARTER], starting, 0, 4);
    for (i = 0; i < 4; ++i) {
        in->n1_pct[i] = n1_values[i];
        in->n2_pct[i] = n2[i];
        in->engine_starting[i] = starting[i] != 0;
        if (throttle_values[i] >= 0.85f) ++takeoff;
    }
    in->on_ground = XPLMGetDatai(refs.on_ground) != 0;
    in->flap_lever = pfd_aircraft.flap_handle_index;
    in->spoiler_ratio = pfd_aircraft.spoiler_ratio;
    in->fcu_alt_valid = guidance.initialized;
    in->fcu_selected_alt_ft = guidance.selected_altitude_ft;
    in->fcu_speed_is_mach = guidance.mach_mode;
    in->fcu_selected_speed_kt = guidance.selected_speed_knots;
    in->fcu_selected_mach = guidance.selected_mach;
    in->speed_managed = guidance.speed_managed;
    in->vertical_mode = fmsa_vertical_mode(takeoff >= 2, in->on_ground, a380x_fms_phase(&fmsa_fms));
    in->lateral_mode = fmsa_lateral_mode();
    in->fcu_vs_fpm = guidance.selected_vs_fpm;
    in->fcu_fpa_deg = guidance.selected_fpa_deg;
    in->vnav_decel_reached = fmsa_fmgc.out.decel_reached; /* FMGC DECEL point (previous frame) */
    in->cold_and_dark_spawn = fmsa_cold_and_dark;
    in->mfd_on_active_init_fuel_perf = fmsa_mfd_on_init_fuel_perf();
    in->efis_baro_inhg[0] = efis[0].baro_inhg;
    in->efis_baro_inhg[1] = efis[1].baro_inhg;

    memset(&fmsa_aircraft, 0, sizeof(fmsa_aircraft));
    fmsa_aircraft.valid = in->position_valid;
    fmsa_aircraft.lat = in->lat_deg;
    fmsa_aircraft.lon = in->lon_deg;
    fmsa_aircraft.alt_ft = (float)in->baro_alt_ft;
    fmsa_aircraft.true_track_deg = isfinite(in->true_track_deg) ? (float)in->true_track_deg : NAN;
    fmsa_aircraft.gs_kt = (float)in->ground_speed_kt;
    fmsa_aircraft.on_ground = in->on_ground;
    fmsa_aircraft.nav_mode = in->lateral_mode == A380X_FMS_LM_NAV;
}

/* ---- Systems host for the MFD pages ---------------------------------------------------------- */

static void fmsa_host_read_status(void *ctx, A380XMfdStatusData *out)
{
    const A380XNavCycle *c = fmsa_nav ? a380x_nav_cycle(fmsa_nav) : NULL;
    A380XFmsValue pen;
    (void)ctx;
    memset(out, 0, sizeof(*out));
    snprintf(out->airframe, sizeof(out->airframe), "A380-800");
    snprintf(out->engine, sizeof(out->engine), "TRENT 972");
    if (c && c->valid && c->airac[0]) {
        mfd_navdb_serial(c->provider[0] ? c->provider : "XP", c->airac, out->navdb_serial, sizeof(out->navdb_serial));
        if (!mfd_airac_cycle_dates(c->airac, out->navdb_active, sizeof(out->navdb_active)))
            out->navdb_active[0] = '\0';
        snprintf(out->navdb_second, sizeof(out->navdb_second), "%s", out->navdb_active);
    }
    pen = a380x_fms_get(&fmsa_fms, A380X_FMS_PLAN_ACTIVE, A380X_FMS_F_FUEL_PENALTY);
    out->fuel_penalty_has = pen.has;
    out->fuel_penalty_pilot = pen.has && pen.v > 0.0;
    out->fuel_penalty_pct = pen.has ? pen.v : 0.0;
}

static int fmsa_host_set_fuel_penalty(void *ctx, int has_value, double pct, MfdFmsError *err)
{
    (void)ctx;
    return has_value ? a380x_fms_enter(&fmsa_fms, A380X_FMS_PLAN_ACTIVE, A380X_FMS_F_FUEL_PENALTY, pct, err)
                     : a380x_fms_clear(&fmsa_fms, A380X_FMS_PLAN_ACTIVE, A380X_FMS_F_FUEL_PENALTY, err);
}

static void fmsa_host_swap_nav_database(void *ctx)
{
    (void)ctx;
    a380x_fms_reset(&fmsa_fms); /* SWAP * CONFIRM = FMS reset (spec 03 section 5.3) */
}

static void fmsa_host_delete_all_stored(void *ctx) { (void)ctx; /* no pilot-stored data in the port */ }

static int fmsa_host_airport_lookup(void *ctx, const char *icao, A380XMfdAirport *out)
{
    const A380XNavAirport *apt;
    int i;
    (void)ctx;
    memset(out, 0, sizeof(*out));
    if (!fmsa_nav || !icao || !*icao) return 0;
    apt = a380x_nav_airport(fmsa_nav, icao);
    if (!apt) return 0;
    snprintf(out->icao, sizeof(out->icao), "%.5s", apt->info.icao[0] ? apt->info.icao : icao);
    snprintf(out->name, sizeof(out->name), "%s", apt->info.name);
    out->lat = apt->info.lat; out->lon = apt->info.lon;
    for (i = 0; i < apt->runway_count && out->runway_count < A380X_MFD_MAX_RUNWAYS; ++i) {
        const A380XNavRunway *r = &apt->runways[i];
        A380XMfdRunway *o = &out->rwy[out->runway_count++];
        snprintf(o->ident, sizeof(o->ident), "%.5s", r->designator);
        o->lat = r->thr_lat; o->lon = r->thr_lon;
        o->elevation_ft = r->thr_elev_ft;
        o->length_m = isfinite(r->length_ft) ? r->length_ft * 0.3048f : NAN;
        o->course_deg = r->true_brg;
        snprintf(o->ls_ident, sizeof(o->ls_ident), "%.5s", r->ls_ident);
    }
    return 1;
}

#include "a380x_mfd_systems_host.inc"

static void fmsa_host_init(void)
{
    memset(&fmsa_host, 0, sizeof(fmsa_host));
    fmsa_host.ctx = NULL;
    fmsa_host.read_status = fmsa_host_read_status;
    fmsa_host.set_fuel_penalty = fmsa_host_set_fuel_penalty;
    fmsa_host.swap_nav_database = fmsa_host_swap_nav_database;
    fmsa_host.delete_all_stored = fmsa_host_delete_all_stored;
    fmsa_host.airport_lookup = fmsa_host_airport_lookup;
    fmsa_host.command = fmsa_sys_command;
    fmsa_host.read_value = fmsa_sys_read_value;
}

/* ---- FCU knob events into the FMS (spec 07 section 2.1) ------------------------------------- */

static void fmsa_on_fcu_command(A380XFcuCommand command)
{
    if (!fmsa_initialized) return;
    switch (command) {
    case A380X_FCU_ALT_PUSH: case A380X_FCU_ALT_PULL: a380x_fms_on_fcu_alt_push_pull(&fmsa_fms); break;
    case A380X_FCU_ALT_ROTATE: a380x_fms_on_fcu_alt_turn(&fmsa_fms); break;
    case A380X_FCU_VS_ROTATE: case A380X_FCU_VS_PUSH: case A380X_FCU_VS_PULL: a380x_fms_on_fcu_vs_knob(&fmsa_fms); break;
    default: break;
    }
}

/* ---- Lifecycle -------------------------------------------------------------------------------- */

static int fmsa_engines_running(void)
{
    float n2[4] = {0, 0, 0, 0};
    int i;
    if (fmsa_raw[FMSA_IN_N2]) XPLMGetDatavf(fmsa_raw[FMSA_IN_N2], n2, 0, 4);
    for (i = 0; i < 4; ++i) if (n2[i] > 50.0f) return 1;
    return 0;
}

static void fmsa_shutdown(void)
{
    fcu_command_observer = NULL;
    sys_shutdown();
    fmsa_unregister_refs();
    if (fmsa_fpln) { a380x_fpln_destroy(fmsa_fpln); fmsa_fpln = NULL; }
    if (fmsa_nav) { a380x_nav_destroy(fmsa_nav); fmsa_nav = NULL; }
    fmsa_nav_loaded = 0;
    memset(&fms_feed, 0, sizeof(fms_feed));
    fmsa_initialized = 0;
}

/* XPluginStart, before mfda_init (the MFD gets the FMS, its queue and the systems host). */
static int fmsa_init(void)
{
    A380XNavdataConfig cfg;
    A380XNavdataHost host;
    A380XFmsNavHooks nav_hooks;
    A380XFmsPlanHooks plan_hooks;
    int i;
    if (fmsa_initialized) return 1;
    for (i = 0; i < FMSA_IN_COUNT; ++i) fmsa_raw[i] = XPLMFindDataRef(fmsa_raw_names[i]);
    fmsa_ctl.switching_knob = 1;
    for (i = 0; i < 3; ++i) fmsa_ctl.fmc_failed[i] = fmsa_ctl.fmc_reset[i] = 0;
    if (!fmsa_register_refs()) return 0;
    XPLMGetSystemPath(fmsa_xplane_root);
    memset(&cfg, 0, sizeof(cfg));
    cfg.xplane_root = fmsa_xplane_root;
    memset(&host, 0, sizeof(host));
    host.airport_info = fmsa_nav_airport_info;
    host.magvar = fmsa_nav_magvar;
    host.log = fmsa_nav_log;
    fmsa_nav = a380x_nav_create(&cfg, &host);
    fmsa_fpln = fmsa_nav ? a380x_fpln_create(fmsa_nav) : NULL;
    if (!fmsa_nav || !fmsa_fpln) XPLMDebugString("A380X FMS: navigation database / flight plan store unavailable\n");
    memset(&nav_hooks, 0, sizeof(nav_hooks));
    nav_hooks.airport_lookup = fmsa_nav ? fmsa_fms_airport : NULL;
    fmsa_plan_hooks(&plan_hooks);
    a380x_fms_init(&fmsa_fms, NULL, &nav_hooks, &plan_hooks);
    /* Spawn type: the A380 network decides it once per load (spec 7.2); legacy check otherwise. */
    fmsa_cold_and_dark = a380x_power.available ? a380x_power.spawn_mode == A380X_ELN_SPAWN_COLD_DARK
                                               : !fmsa_engines_running();
    fmsa_host_init();
    if (!sys_init()) {
        fmsa_unregister_refs();
        if (fmsa_fpln) { a380x_fpln_destroy(fmsa_fpln); fmsa_fpln = NULL; }
        if (fmsa_nav) { a380x_nav_destroy(fmsa_nav); fmsa_nav = NULL; }
        return 0;
    }
    memset(&fms_feed, 0, sizeof(fms_feed));
    fmsa_nav_seq = 0;
    fmsa_fpln_timer = 0.0;
    a380x_fmgc_init(&fmsa_fmgc);
    fmsa_pred_valid = 0;
    fcu_command_observer = fmsa_on_fcu_command;
    fmsa_initialized = 1;
    return 1;
}

/* XPluginEnable: load the global navigation files once (~0.6 s, during aircraft load). */
static void fmsa_enable(void)
{
    char line[160];
    float t0;
    if (!fmsa_initialized || !fmsa_nav || fmsa_nav_loaded) return;
    t0 = XPLMGetElapsedTime();
    if (a380x_nav_load_global(fmsa_nav) == 0) {
        A380XNavStats st;
        const A380XNavCycle *c = a380x_nav_cycle(fmsa_nav);
        a380x_nav_stats(fmsa_nav, &st);
        snprintf(line, sizeof(line), "A380X FMS: navdata AIRAC %s loaded (%d fixes, %d navaids, %.0f ms)\n",
                 c && c->valid ? c->airac : "?", st.waypoints, st.navaids, st.global_load_ms);
        XPLMDebugString(line);
    } else {
        XPLMDebugString("A380X FMS: no X-Plane navigation data found (earth_fix.dat / earth_nav.dat)\n");
    }
    (void)t0;
    fmsa_nav_loaded = 1;
}

/* XPLM_MSG_PLANE_LOADED: fresh FMS (power-up state), spawn type re-evaluated. */
static void fmsa_plane_loaded(void)
{
    if (!fmsa_initialized) return;
    if (fmsa_fpln) a380x_fpln_reset(fmsa_fpln);
    /* Spawn type: the A380 network decides it once per load (spec 7.2); legacy check otherwise. */
    fmsa_cold_and_dark = a380x_power.available ? a380x_power.spawn_mode == A380X_ELN_SPAWN_COLD_DARK
                                               : !fmsa_engines_running();
    a380x_fms_reset(&fmsa_fms);
    a380x_fmgc_init(&fmsa_fmgc);
    fmsa_pred_valid = 0;
    fmsa_nav_seq = fmsa_fms.out.nav_request_seq;
    sys_fms_reset();
    survh_defaults(fmsa_cold_and_dark);
}

/* ---- FMGC managed guidance (a380x_fmgc_guidance.c) ------------------------------------------ */

static double fmsa_perf(int field)
{
    A380XFmsValue v = a380x_fms_get(&fmsa_fms, A380X_FMS_PLAN_ACTIVE, (A380XFmsField)field);
    return v.has && isfinite(v.v) ? v.v : 0.0;
}

static void fmsa_fmgc_inputs(A380XFmgcInputs *g, const A380XFmsOutputs *o)
{
    const A380XFmsInputs *in = &fmsa_in;
    double psi = fmsa_getf(FMSA_IN_TRUE_PSI);
    memset(g, 0, sizeof(*g));
    g->position_valid = in->position_valid && isfinite(in->true_track_deg);
    g->lat_deg = in->lat_deg; g->lon_deg = in->lon_deg;
    g->alt_ft = in->baro_alt_ft; g->vs_fpm = in->vertical_speed_fpm;
    g->gs_kt = in->ground_speed_kt; g->tas_kt = in->tas_kt; g->cas_kt = in->cas_kt; g->mach = in->mach;
    g->true_track_deg = in->true_track_deg;
    g->true_heading_deg = isfinite(psi) ? psi : in->true_track_deg;
    g->magvar_deg = isfinite(in->magnetic_variation_deg) ? in->magnetic_variation_deg : 0.0;
    g->on_ground = in->on_ground;
    g->radio_alt_ft = in->radio_alt_valid ? in->radio_alt_ft : 0.0;
    g->fms_valid = o->valid;
    g->phase = a380x_fms_phase(&fmsa_fms);
    g->cruise_alt_ft = o->cruise_altitude_ft;
    g->fcu_alt_ft = guidance.selected_altitude_ft;
    g->managed_speed_kt = o->managed_speed_valid ? o->managed_speed_knots : 0.0;
    g->managed_is_mach = o->managed_speed_is_mach;
    g->managed_mach = o->managed_mach;
    g->descent_speed_kt = fmsa_perf(A380X_FMS_F_DES_MANAGED_SPD);
    g->descent_mach = fmsa_perf(A380X_FMS_F_DES_MANAGED_MACH);
    g->des_limit_kt = fmsa_perf(A380X_FMS_F_DES_SPD_LIM);
    g->des_limit_alt_ft = fmsa_perf(A380X_FMS_F_DES_SPD_LIM_ALT);
    g->green_dot_kt = o->green_dot_knots; g->s_speed_kt = o->s_speed_knots; g->f_speed_kt = o->f_speed_knots;
    g->vapp_kt = o->approach_vapp_knots; g->vls_kt = o->vls_knots;
    g->flap_lever = in->flap_lever;
    g->landing_conf3 = o->landing_conf3;
    g->static_pressure_hpa = in->static_pressure_valid ? in->static_pressure_hpa : 0.0;
    g->nav_engaged = guidance.lateral_mode == A380X_C1_NAV;
    g->nav_armed = guidance.nav_armed;
    g->vertical_managed = guidance.altitude_managed;
    g->idle_speed_descent = guidance.thrust_mode == A380X_A1A2_THR_IDLE && guidance.athr_active
        && (guidance.vertical_mode == A380X_B1_OP_DES || (guidance.vertical_mode == A380X_B1_DES && guidance.des_submode == 1))
        && fabs(in->cas_kt - guidance.speed_target_knots) < 5.0;
}

/* FMGC outputs into the guidance feed; the managed speed of DESCENT / APPROACH replaces the FMS one. */
static void fmsa_fmgc_feed(A380XFmsFeed *f, const A380XFmsOutputs *o)
{
    const A380XFmgcOutputs *g = a380x_fmgc_outputs(&fmsa_fmgc);
    f->phase_valid = 1;
    f->phase = a380x_fms_phase(&fmsa_fms);
    f->cruise_altitude_ft = o->cruise_altitude_ft > 0 ? o->cruise_altitude_ft : 0.0;
    if (g->lnav_valid && fmsa_fmgc_in.position_valid) {
        f->route_valid = 1;
        f->nav_capture = g->nav_capture;
        f->route_xtk_nm = g->fbw_xtk_nm;
        f->route_track_error_deg = g->fbw_tae_deg;
        f->route_bank_deg = g->bank_ff_deg;
        f->managed_heading_deg = g->managed_heading_mag_deg;
        f->vertical_valid = isfinite(g->managed_altitude_ft);
        f->managed_altitude_ft = g->managed_altitude_ft;
        f->altitude_is_constraint = g->managed_altitude_is_constraint;
        f->path_valid = g->vnav_valid && isfinite(g->path_alt_ft);
        f->path_altitude_ft = g->path_alt_ft;
        f->path_vs_fpm = g->path_vs_fpm;
        f->vdev_ft = g->vdev_ft;
        f->path_idle = g->path_segment == 0 || g->path_segment == 3;
        f->before_tod = g->before_tod;
    }
    if (g->speed_valid && isfinite(g->speed_kt) && g->speed_kt > 80.0) {
        f->managed_speed_valid = 1;
        f->managed_speed_knots = g->speed_kt;
        f->managed_speed_is_mach = g->speed_is_mach;
        f->managed_mach = g->speed_is_mach ? g->speed_mach : NAN;
    }
}

/* Flight loop, after adirs_update and before update_guidance (spec 07 section 1.4). */
static void fmsa_update(float dt, int powered)
{
    const A380XFmsOutputs *o;
    int s;
    if (!fmsa_initialized) return;
    fmsa_read_inputs(powered);
    a380x_fms_update(&fmsa_fms, &fmsa_in, dt);
    o = a380x_fms_outputs(&fmsa_fms);
    /* FMGC every frame: LNAV path + leg sequencing (replaces a380x_fpln_update), VNAV, speeds. */
    fmsa_fmgc_inputs(&fmsa_fmgc_in, o);
    a380x_fmgc_update(&fmsa_fmgc, o->valid ? fmsa_fpln : NULL, &fmsa_fmgc_in, dt);
    fmsa_fpln_timer += dt;
    if (fmsa_fpln_timer >= 0.5) {   /* MFD F-PLN predictions at 2 Hz */
        fmsa_fpln_timer = 0.0;
        a380x_fmgc_predictions(&fmsa_fmgc, fmsa_fpln, &fmsa_fmgc_in, &fmsa_pred_feed, fmsa_leg_pred);
        fmsa_pred_valid = o->valid && fmsa_fmgc.out.lnav_valid;
    }

    /* PFD / guidance feed (fms_feed is declared in a380x_avionics.c before the FCU adapter). */
    memset(&fms_feed, 0, sizeof(fms_feed));
    fms_feed.valid = o->valid;
    if (o->valid) {
        fms_feed.v1_knots = o->v1_knots;
        fms_feed.vr_knots = o->vr_knots;
        fms_feed.v2_knots = o->v2_knots;
        fms_feed.managed_speed_valid = o->managed_speed_valid;
        fms_feed.managed_speed_knots = o->managed_speed_knots;
        fms_feed.managed_speed_is_mach = o->managed_speed_is_mach;
        fms_feed.managed_mach = o->managed_mach;
        fms_feed.f_speed_knots = o->f_speed_knots;
        fms_feed.s_speed_knots = o->s_speed_knots;
        fmsa_fmgc_feed(&fms_feed, o);
        /* FMA D3 (FBW D3Cell priority NO DH, RADIO, BARO; a380x_pfd_render.inc / xp-kit FM words). */
        fms_feed.minimum_mode = (o->no_dh || o->dh_ft == 0.0) ? 3 : o->dh_ft > 0.0 ? 1 : o->mda_ft >= 0.0 ? 2 : 0;
        fms_feed.minimum_ft = fms_feed.minimum_mode == 1 ? o->dh_ft : fms_feed.minimum_mode == 2 ? o->mda_ft : 0.0;
        /* DH from the FMS (spec 07 section 10.2): radio minimum in range replaces the stock RA bug;
         * NO DH or a BARO (MDA) minimum hides it. Without FMS minimums the RA bug stays in use. */
        if (o->dh_ft > 0.0 || o->dh_ft == 0.0 || o->mda_ft >= 0.0) {
            float dh = (o->dh_ft > 0.0 && o->minimums_in_range && !(o->mda_ft >= 0.0)) ? (float)o->dh_ft : -1.0f;
            refs.side[0].decision_height_value = dh;
            refs.side[1].decision_height_value = dh;
        }
    }

    /* FMC-initiated MFD navigation applies to both sides ([PORT] quirk 15). */
    if (o->nav_request_seq != fmsa_nav_seq) {
        fmsa_nav_seq = o->nav_request_seq;
        if (o->nav_request == A380X_FMS_NAV_DATA_STATUS) sys_fms_reset(); /* FMS reset / DONE */
        for (s = 0; s < MFD_SIDE_COUNT; ++s) {
            if (o->nav_request == A380X_FMS_NAV_DATA_STATUS) a380x_mfd_navigate(&mfda_mfd, (MfdSide)s, "fms/data/status");
            else if (o->nav_request == A380X_FMS_NAV_PERF) a380x_mfd_navigate(&mfda_mfd, (MfdSide)s, "fms/active/perf");
        }
    }

    /* MFD header / title-bar inputs. */
    for (s = 0; s < MFD_SIDE_COUNT; ++s) mfda_inputs.fms_failed[s] = (unsigned char)(a380x_fms_side_failed(&fmsa_fms, s) != 0);
    mfda_inputs.fms_knob = fmsa_ctl.switching_knob;
    mfda_inputs.fmc_a_healthy = (unsigned char)(o->fmc_healthy[0] != 0);
    mfda_inputs.fmc_b_healthy = (unsigned char)(o->fmc_healthy[1] != 0);
    a380x_fms_get_text(&fmsa_fms, A380X_FMS_PLAN_ACTIVE, A380X_FMS_T_CALLSIGN, mfda_inputs.atc_callsign,
                       sizeof(mfda_inputs.atc_callsign));
    mfda_inputs.engine_out = (unsigned char)(a380x_fms_engine_out(&fmsa_fms) != 0);
    mfda_inputs.tmpy_exists = (unsigned char)(a380x_fms_tmpy_exists(&fmsa_fms) != 0);
    {
        A380XFmsValue pen = a380x_fms_get(&fmsa_fms, A380X_FMS_PLAN_ACTIVE, A380X_FMS_F_FUEL_PENALTY);
        mfda_inputs.fuel_penalty_active = (unsigned char)(pen.has && pen.v > 0.0);
    }
    mfda_inputs.metric_units = (unsigned char)(fmsa_fms.cfg.metric_units != 0);
    mfda_inputs.flight_phase = a380x_fms_phase(&fmsa_fms);
    if (fmsa_fpln && a380x_fpln_has_tmpy(fmsa_fpln)) mfda_inputs.tmpy_exists = 1;
    mfda_mfd.atc_msgs = &sys_atc_queue;
    sys_update(dt, powered);
}
