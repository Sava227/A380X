/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Aircraft-systems side of the MFD pages (WP-J, spec 12 section 7, spec 06 sections 1-4, 9.4,
 * 10-13): the generic A380XMfdSystemsHost read_value / command channel used by SURV, FCU BKUP,
 * POSITION and ATCCOM (ids are the page modules' (page id << 16) | n enumerations), the SURV
 * states (a380x/xp12/surv/<name>) with the transponder mode law, the FM position EPU / RNP model,
 * the POSITION navaid tuner host and the D-ATIS provider synthesised from X-Plane weather.
 *
 * Included by a380x_fms_adapter.inc (unity build). Statics are prefixed fmsa_ / survh_.
 */
#include "XPLMWeather.h"
#include "a380x_mfd_page_surv.h"
#include "a380x_mfd_page_fcubkup.h"
#include "a380x_mfd_page_position.h"
#include "a380x_mfd_page_atccom.h"
#include "a380x_mfd_page_fpln.h"

/* ---- Raw simulator data of the systems host (scanned by build.py: FMSA_RAW) --------------- */

enum {
    SYS_IN_XPDR_CODE, SYS_IN_XPDR_MODE, SYS_IN_XPDR_FAIL, SYS_IN_TCAS_FILTER, SYS_IN_NAV_FREQ,
    SYS_IN_NAV_TYPE, SYS_IN_NAV_HAS_DME, SYS_IN_NAV1_ID, SYS_IN_NAV2_ID, SYS_IN_OBS_PILOT,
    SYS_IN_OBS_COPILOT, SYS_IN_NAV1_CRS_PILOT, SYS_IN_NAV2_CRS_COPILOT, SYS_IN_BARO_PILOT,
    SYS_IN_BARO_COPILOT, SYS_IN_WEATHER_SOURCE,
    SYS_IN_COUNT
};
#define FMSA_RAW(name) name
static const char *const sys_raw_names[SYS_IN_COUNT] = {
    FMSA_RAW("sim/cockpit2/radios/actuators/transponder_code"),
    FMSA_RAW("sim/cockpit2/radios/actuators/transponder_mode"),
    FMSA_RAW("sim/operation/failures/rel_xpndr"),
    FMSA_RAW("sim/cockpit2/radios/actuators/tcas_filter"),
    FMSA_RAW("sim/cockpit2/radios/actuators/nav_frequency_hz"),
    FMSA_RAW("sim/cockpit2/radios/indicators/nav_type"),
    FMSA_RAW("sim/cockpit2/radios/indicators/nav_has_dme"),
    FMSA_RAW("sim/cockpit2/radios/indicators/nav1_nav_id"),
    FMSA_RAW("sim/cockpit2/radios/indicators/nav2_nav_id"),
    FMSA_RAW("sim/cockpit2/radios/actuators/nav_obs_deg_mag_pilot"),
    FMSA_RAW("sim/cockpit2/radios/actuators/nav_obs_deg_mag_copilot"),
    FMSA_RAW("sim/cockpit2/radios/actuators/nav1_course_deg_mag_pilot"),
    FMSA_RAW("sim/cockpit2/radios/actuators/nav2_course_deg_mag_copilot"),
    FMSA_RAW("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot"),
    FMSA_RAW("sim/cockpit2/gauges/actuators/barometer_setting_in_hg_copilot"),
    FMSA_RAW("sim/weather/region/weather_source"),
};
#undef FMSA_RAW
static XPLMDataRef sys_raw[SYS_IN_COUNT];
static XPLMCommandRef sys_ident_cmd;

/* ---- SURV states (spec 06 section 4.2), published a380x/xp12/surv/<name> ------------------- */

enum {
    SURV_XPDR_AUTO, SURV_ALT_RPTG, SURV_TCAS_LEVEL, SURV_TCAS_ALT_SELECT, SURV_TERR_OFF, SURV_GPWS_OFF,
    SURV_GS_OFF, SURV_FLAP_OFF, SURV_WXR_TAWS_SYS, SURV_XPDR_TCAS_SYS, SURV_STATE_COUNT
};
typedef struct { const char *name; int lo, hi; XPLMDataRef handle; } SurvRef;
#define SURVR(name, lo, hi) {"a380x/xp12/surv/" name, lo, hi, NULL}
static SurvRef survh_refs[SURV_STATE_COUNT] = {
    SURVR("xpdr_auto", 0, 1), SURVR("alt_rptg_request", 0, 1), SURVR("tcas_alert_level", 0, 2),
    SURVR("tcas_alt_select", 0, 2), SURVR("terr_sys_off", 0, 1), SURVR("gpws_sys_off", 0, 1),
    SURVR("gs_mode_off", 0, 1), SURVR("flap_mode_off", 0, 1), SURVR("wxr_taws_sys", 0, 2),
    SURVR("xpdr_tcas_sys", 1, 2),
};
#undef SURVR
static int survh_state[SURV_STATE_COUNT];
static int survh_last_mode = -1, survh_last_filter = -1;

static void survh_defaults(int cold_and_dark)
{
    memset(survh_state, 0, sizeof(survh_state));
    survh_state[SURV_XPDR_AUTO] = !cold_and_dark;
    survh_state[SURV_ALT_RPTG] = 1;
    survh_state[SURV_TCAS_LEVEL] = cold_and_dark ? 0 : 2;
    survh_state[SURV_WXR_TAWS_SYS] = 1;
    survh_state[SURV_XPDR_TCAS_SYS] = 1;
    survh_last_mode = survh_last_filter = -1;
}

static void survh_set(int i, int v)
{
    if (i < 0 || i >= SURV_STATE_COUNT) return;
    survh_state[i] = v < survh_refs[i].lo ? survh_refs[i].lo : v > survh_refs[i].hi ? survh_refs[i].hi : v;
}

static int survh_ref_geti(void *refcon) { return survh_state[(SurvRef *)refcon - survh_refs]; }
static void survh_ref_seti(void *refcon, int v) { survh_set((int)((SurvRef *)refcon - survh_refs), v); }
static float survh_ref_getf(void *refcon) { return (float)survh_ref_geti(refcon); }
static void survh_ref_setf(void *refcon, float v) { survh_ref_seti(refcon, (int)lroundf(v)); }

static void survh_unregister(void)
{
    int i;
    for (i = 0; i < SURV_STATE_COUNT; ++i)
        if (survh_refs[i].handle) { XPLMUnregisterDataAccessor(survh_refs[i].handle); survh_refs[i].handle = NULL; }
}

static int survh_register(void)
{
    int i;
    for (i = 0; i < SURV_STATE_COUNT; ++i) if (XPLMFindDataRef(survh_refs[i].name)) return 0;
    for (i = 0; i < SURV_STATE_COUNT; ++i) {
        survh_refs[i].handle = XPLMRegisterDataAccessor(survh_refs[i].name, xplmType_Int | xplmType_Float, 1,
            survh_ref_geti, survh_ref_seti, survh_ref_getf, survh_ref_setf, NULL, NULL, NULL, NULL,
            NULL, NULL, NULL, NULL, &survh_refs[i], &survh_refs[i]);
        if (!survh_refs[i].handle) { survh_unregister(); return 0; }
    }
    return 1;
}

static int sys_xpdr_failed(void) { return sys_raw[SYS_IN_XPDR_FAIL] && XPLMGetDatai(sys_raw[SYS_IN_XPDR_FAIL]) >= 6; }

/* Transponder mode law (FBW Transponder.ts:90-106 on X-Plane's enum), written on a change only so
 * other writers (EFB, hardware) are not fought every frame. */
static void survh_update(int powered, int on_ground)
{
    int mode, filter;
    if (!powered || sys_xpdr_failed()) mode = 0;
    else if (!survh_state[SURV_XPDR_AUTO]) mode = 1;
    else if (on_ground) mode = 5;
    else if (!survh_state[SURV_ALT_RPTG]) mode = 2;
    else mode = survh_state[SURV_TCAS_LEVEL] == 0 ? 3 : survh_state[SURV_TCAS_LEVEL] == 1 ? 6 : 7;
    if (mode != survh_last_mode && sys_raw[SYS_IN_XPDR_MODE]) XPLMSetDatai(sys_raw[SYS_IN_XPDR_MODE], mode);
    survh_last_mode = mode;
    filter = survh_state[SURV_TCAS_ALT_SELECT] == 1 ? 2 : survh_state[SURV_TCAS_ALT_SELECT] == 2 ? 3 : 0;
    if (filter != survh_last_filter && sys_raw[SYS_IN_TCAS_FILTER]) XPLMSetDatai(sys_raw[SYS_IN_TCAS_FILTER], filter);
    survh_last_filter = filter;
}

/* ---- FM position EPU / RNP model (spec 06 section 1.4, port model) ---------------------------- */

static double fmsa_epu_at_loss = 0.05, fmsa_irs_only_s;
static double fmsa_rnp_pilot = NAN;

static double fmsa_epu(void)
{
    if (!fmsa_in.position_valid) return INFINITY;
    if (fmsa_in.gps_primary) return 0.05;
    return fmin(20.0, fmsa_epu_at_loss + 2.0 * fmsa_irs_only_s / 3600.0);
}

static void fmsa_epu_update(double dt)
{
    if (fmsa_in.gps_primary || !fmsa_in.position_valid) { fmsa_irs_only_s = 0.0; fmsa_epu_at_loss = 0.05; }
    else fmsa_irs_only_s += dt;
}

static double fmsa_rnp_default(void)
{
    const A380XFplnPlan *pl = fmsa_fpln ? a380x_fpln_plan(fmsa_fpln, A380X_PLAN_ACTIVE) : NULL;
    if (pl && pl->active_leg_index >= 0 && pl->active_leg_index < pl->pri.count) {
        const A380XFplnLeg *l = &pl->pri.el[pl->active_leg_index];
        A380XNavAirportInfo info;
        if (isfinite(l->rnp_nm) && l->rnp_nm > 0.0f) return l->rnp_nm;
        if (l->segment == A380X_SEG_APPROACH || l->segment == A380X_SEG_MISSED) return 0.30;
        if (fmsa_in.position_valid && fmsa_in.baro_alt_ft < 15000.0 && fmsa_nav) {
            const char *apts[2] = {pl->pri.origin_icao, pl->pri.dest_icao};
            int k;
            for (k = 0; k < 2; ++k) {
                memset(&info, 0, sizeof(info));
                if (apts[k][0] && a380x_nav_airport_info(fmsa_nav, apts[k], &info) && isfinite(info.lat)
                    && a380x_fms_great_circle_nm(fmsa_in.lat_deg, fmsa_in.lon_deg, info.lat, info.lon) <= 30.0)
                    return 1.00;
            }
        }
    }
    return 2.00;
}

/* ---- FCU / EFIS commands by name (the same handlers as the 3D cockpit) ---------------------- */

static const char *const sys_fcu_names[] = {
    "speed_up", "speed_down", "speed_push", "speed_pull", "heading_up", "heading_down", "heading_push",
    "heading_pull", "altitude_up", "altitude_down", "altitude_push", "altitude_pull",
    "vertical_speed_up", "vertical_speed_down", "vertical_speed_push", "vertical_speed_pull",
    "altitude_step_toggle", "ap1_toggle", "ap2_toggle", "fd_toggle", "athr_toggle", "speed_mach_toggle",
    "trk_fpa_toggle", "altitude_hold", "loc_toggle", "approach_toggle", "true_mag_toggle", "metric_toggle",
};
#define SYS_FCU_COUNT ((int)(sizeof(sys_fcu_names) / sizeof(sys_fcu_names[0])))
static XPLMCommandRef sys_fcu_cmd[SYS_FCU_COUNT];
static const char *const sys_efis_names[] = {
    "cstr_toggle", "wpt_toggle", "vord_toggle", "ndb_toggle", "arpt_toggle", "vor1_toggle", "vor2_toggle",
    "terr_toggle", "traf_toggle", "wx_toggle", "taxi_toggle", "ls_toggle", "vv_toggle", "mode_up",
    "mode_down", "range_up", "range_down", "baro_up", "baro_down", "baro_std", "baro_qnh", "baro_unit_toggle",
};
#define SYS_EFIS_COUNT ((int)(sizeof(sys_efis_names) / sizeof(sys_efis_names[0])))
static XPLMCommandRef sys_efis_cmd[MFD_SIDE_COUNT][SYS_EFIS_COUNT];
/* FCU window state read back from the FCU module's own datarefs (no duplicate state). */
static const char *const sys_fcu_key_names[] = {
    NULL, "selected_speed", "selected_mach", "selected_heading", "selected_altitude",
    "selected_vertical_speed", "selected_fpa", "speed_managed", "heading_managed", "altitude_managed",
    "mach_mode", "trk_fpa", "true_heading", "metric_altitude", "ap1_active", "ap2_active", "fd_active",
    NULL, "loc_active", "approach_active", "altitude_step_ft", "altitude_hold_active",
};
#define SYS_FCU_KEY_COUNT ((int)(sizeof(sys_fcu_key_names) / sizeof(sys_fcu_key_names[0])))
static XPLMDataRef sys_fcu_ref[SYS_FCU_KEY_COUNT];
static XPLMDataRef sys_fcu_athr_armed, sys_fcu_athr_active;

static void sys_bind(void)
{
    int i, s;
    char name[128];
    for (i = 0; i < SYS_IN_COUNT; ++i) sys_raw[i] = XPLMFindDataRef(sys_raw_names[i]);
    sys_ident_cmd = XPLMFindCommand("sim/transponder/transponder_ident");
    for (i = 0; i < SYS_FCU_COUNT; ++i) {
        snprintf(name, sizeof(name), "a380x/xp12/fcu/%s", sys_fcu_names[i]);
        sys_fcu_cmd[i] = XPLMFindCommand(name);
    }
    for (s = 0; s < MFD_SIDE_COUNT; ++s)
        for (i = 0; i < SYS_EFIS_COUNT; ++i) {
            snprintf(name, sizeof(name), "a380x/xp12/efis/%s/%s", s ? "first_officer" : "captain", sys_efis_names[i]);
            sys_efis_cmd[s][i] = XPLMFindCommand(name);
        }
    for (i = 0; i < SYS_FCU_KEY_COUNT; ++i) {
        sys_fcu_ref[i] = NULL;
        if (!sys_fcu_key_names[i]) continue;
        snprintf(name, sizeof(name), "a380x/xp12/fcu/%s", sys_fcu_key_names[i]);
        sys_fcu_ref[i] = XPLMFindDataRef(name);
    }
    sys_fcu_athr_armed = XPLMFindDataRef("a380x/xp12/fcu/athr_armed");
    sys_fcu_athr_active = XPLMFindDataRef("a380x/xp12/fcu/athr_active");
}

static void sys_run(XPLMCommandRef c, double action)
{
    if (!c) return;
    if (action == A380X_MFD_ACT_BEGIN) XPLMCommandBegin(c);
    else if (action == A380X_MFD_ACT_END) XPLMCommandEnd(c);
    else XPLMCommandOnce(c);
}

/* Typed FCU target: apply amount 0 first (collapses dashes like a first detent), then the
 * difference to the current selection in detents (spec 06 section 13). */
static void sys_fcu_apply(A380XFcuCommand cmd, double amount)
{
    if (!guidance_inputs.powered) return;
    a380x_guidance_command(&guidance, &guidance_inputs, cmd, amount);
    if (fcu_command_observer) fcu_command_observer(cmd);
}

static void sys_fcu_set(unsigned int cmd, double target)
{
    double d;
    if (!isfinite(target)) return;
    switch (cmd) {
    case A380X_FCUB_C_SET_SPEED:
        sys_fcu_apply(A380X_FCU_SPD_ROTATE, 0);
        sys_fcu_apply(A380X_FCU_SPD_ROTATE, round(target - guidance.selected_speed_knots));
        break;
    case A380X_FCUB_C_SET_MACH:
        sys_fcu_apply(A380X_FCU_SPD_ROTATE, 0);
        sys_fcu_apply(A380X_FCU_SPD_ROTATE, round((target - guidance.selected_mach) * 100.0));
        break;
    case A380X_FCUB_C_SET_HEADING:
        sys_fcu_apply(A380X_FCU_HDG_ROTATE, 0);
        d = fmod(target - guidance.selected_heading_deg + 540.0, 360.0) - 180.0;
        sys_fcu_apply(A380X_FCU_HDG_ROTATE, round(d));
        break;
    case A380X_FCUB_C_SET_ALTITUDE:
        d = (target - guidance.selected_altitude_ft) / (guidance.altitude_increment_1000 ? 1000.0 : 100.0);
        sys_fcu_apply(A380X_FCU_ALT_ROTATE, d);
        break;
    case A380X_FCUB_C_SET_VS:
        sys_fcu_apply(A380X_FCU_VS_ROTATE, 0);
        sys_fcu_apply(A380X_FCU_VS_ROTATE, round((target - guidance.selected_vs_fpm) / 50.0));
        break;
    case A380X_FCUB_C_SET_FPA:
        sys_fcu_apply(A380X_FCU_VS_ROTATE, 0);
        sys_fcu_apply(A380X_FCU_VS_ROTATE, round((target - guidance.selected_fpa_deg) / 0.1));
        break;
    default:
        break;
    }
}

static double sys_efis_field(int side, int k)
{
    const EfisSide *e = &efis[side ? 1 : 0];
    switch (k) {
    case 0: return e->cstr; case 1: return e->wpt; case 2: return e->vord; case 3: return e->ndb;
    case 4: return e->arpt; case 5: return e->vor1; case 6: return e->vor2; case 7: return e->terr;
    case 8: return e->traf; case 9: return e->ls; case 10: return e->vv; case 11: return e->wx_requested;
    case 12: return e->taxi_requested; case 13: return e->mode; case 14: return e->range_index;
    case 15: return e->zoom; case 16: return e->baro_std; case 17: return e->baro_inhg; case 18: return e->powered;
    case 19: return e->range_nm; case 20: return e->qnh_hpa; case 21: return e->baro_preset_hpa;
    default: return fmsa_nan();
    }
}

static double sys_ir_field(int u, int f)
{
    const A380XAdirsUnitState *s;
    if (u < 0 || u >= A380X_ADIRS_UNITS) return fmsa_nan();
    s = &adirs.unit[u];
    switch (f) {
    case A380X_POS_IR_MODE: return s->mode;
    case A380X_POS_IR_ALIGNING: return s->aligning;
    case A380X_POS_IR_ALIGN_REMAIN_S: return s->alignment_remaining;
    case A380X_POS_IR_NAV_VALID: return s->nav_valid;
    case A380X_POS_IR_HDG_VALID: return s->heading_valid;
    case A380X_POS_IR_FAULT: return s->ir_fault_lit;
    case A380X_POS_IR_HDG_REQUIRED: return s->att_heading_required;
    case A380X_POS_IR_NAV_LOCKED: return s->nav_locked;
    case A380X_POS_IR_LAT: return s->nav_valid ? fmsa_in.lat_deg : fmsa_nan();
    case A380X_POS_IR_LON: return s->nav_valid ? fmsa_in.lon_deg : fmsa_nan();
    case A380X_POS_IR_GS_KT: return s->nav_valid ? fmsa_in.ground_speed_kt : fmsa_nan();
    case A380X_POS_IR_HDG_MAG: return s->heading_valid ? s->heading_deg : fmsa_nan();
    default: return fmsa_nan();
    }
}

static int sys_gps_valid(int n)
{
    int fail = fmsa_geti(n ? FMSA_IN_GPS2_FAIL : FMSA_IN_GPS_FAIL);
    return mfda_powered && fmsa_geti(n ? FMSA_IN_GPS2_POWER : FMSA_IN_GPS_POWER) && fail < 6;
}

static double fmsa_sys_read_value(void *ctx, MfdSide side, unsigned int key)
{
    (void)ctx;
    /* SURV */
    if (key >= A380X_SURV_XPDR_AUTO && key <= A380X_SURV_XPDR_TCAS_SYS) return survh_state[key - A380X_SURV_XPDR_AUTO];
    switch (key) {
    case A380X_SURV_K_XPDR_CODE: return sys_raw[SYS_IN_XPDR_CODE] ? XPLMGetDatai(sys_raw[SYS_IN_XPDR_CODE]) : fmsa_nan();
    case A380X_SURV_K_XPDR_MODE: return sys_raw[SYS_IN_XPDR_MODE] ? XPLMGetDatai(sys_raw[SYS_IN_XPDR_MODE]) : fmsa_nan();
    case A380X_SURV_K_XPDR_FAILED: return sys_xpdr_failed();
    case A380X_SURV_K_TCAS_FAULT: return !mfda_powered || sys_xpdr_failed();
    case A380X_SURV_K_ON_GROUND: return fmsa_in.on_ground;
    case A380X_SURV_K_TERR1_FAILED: case A380X_SURV_K_GPWS1_FAILED:
        return survh_state[SURV_WXR_TAWS_SYS] == 0;
    case A380X_SURV_K_TERR2_FAILED: case A380X_SURV_K_GPWS2_FAILED:
        return survh_state[SURV_WXR_TAWS_SYS] == 0;
    /* FCU BKUP */
    case A380X_FCUB_K_POWERED: return guidance_inputs.powered;
    case A380X_FCUB_K_ATHR:
        return (sys_fcu_athr_armed && XPLMGetDatai(sys_fcu_athr_armed)) || (sys_fcu_athr_active && XPLMGetDatai(sys_fcu_athr_active));
    case A380X_FCUB_K_SPEED_DASHES: return guidance.speed_dashes;
    case A380X_FCUB_K_HEADING_DASHES: return guidance.heading_dashes;
    case A380X_FCUB_K_VS_DASHES: return guidance.vs_dashes;
    /* POSITION */
    case A380X_POS_K_FM_VALID: return fmsa_in.position_valid;
    case A380X_POS_K_FM_LAT: return fmsa_in.position_valid ? fmsa_in.lat_deg : fmsa_nan();
    case A380X_POS_K_FM_LON: return fmsa_in.position_valid ? fmsa_in.lon_deg : fmsa_nan();
    case A380X_POS_K_GPS_PRIMARY: return fmsa_in.gps_primary;
    case A380X_POS_K_EPU_NM: return fmsa_epu();
    case A380X_POS_K_RNP_NM: return isfinite(fmsa_rnp_pilot) ? fmsa_rnp_pilot : fmsa_rnp_default();
    case A380X_POS_K_RNP_PILOT: return isfinite(fmsa_rnp_pilot);
    case A380X_POS_K_GPS1_VALID: return sys_gps_valid(0);
    case A380X_POS_K_GPS2_VALID: return sys_gps_valid(1);
    case A380X_POS_K_TRUE_TRACK: return fmsa_in.true_track_deg;
    case A380X_POS_K_TRUE_HDG: return fmsa_getf(FMSA_IN_TRUE_PSI);
    case A380X_POS_K_MAGVAR: return fmsa_in.magnetic_variation_deg;
    case A380X_POS_K_WIND_DIR: return fmsa_in.wind_valid ? fmsa_in.wind_dir_true_deg : fmsa_nan();
    case A380X_POS_K_WIND_KT: return fmsa_in.wind_valid ? fmsa_in.wind_speed_kt : fmsa_nan();
    case A380X_POS_K_ON_GROUND: return fmsa_in.on_ground;
    case A380X_POS_K_UTC_S: case A380X_ATC_K_UTC_S: return fmsa_in.utc_seconds;
    default: break;
    }
    if (key >= A380X_FCUB_K_SELECTED_SPEED && key <= A380X_FCUB_K_ALT_HOLD) {
        int k = (int)(key - A380X_FCUB_K_POWERED);
        XPLMDataRef r = k < SYS_FCU_KEY_COUNT ? sys_fcu_ref[k] : NULL;
        return r ? (double)XPLMGetDataf(r) : fmsa_nan();
    }
    if (key >= A380X_EFIS_K_CSTR && key <= A380X_EFIS_K_BARO_PRESET_HPA) return sys_efis_field(side, (int)(key - A380X_EFIS_K_CSTR));
    if (key >= A380X_POS_K_IR_BASE && key < A380X_POS_K_IR_BASE + 48u) {
        unsigned o = key - A380X_POS_K_IR_BASE;
        return sys_ir_field((int)(o / 16u), (int)(o % 16u));
    }
    return fmsa_nan();
}

static void fmsa_sys_command(void *ctx, MfdSide side, unsigned int cmd, double value)
{
    (void)ctx;
    if (cmd >= A380X_SURV_XPDR_AUTO && cmd <= A380X_SURV_XPDR_TCAS_SYS) {
        if (isfinite(value)) survh_set((int)(cmd - A380X_SURV_XPDR_AUTO), (int)lround(value));
        return;
    }
    if (cmd == A380X_SURV_C_XPDR_CODE) {
        int code = isfinite(value) ? (int)lround(value) : -1;
        if (code >= 0 && code <= 7777 && sys_raw[SYS_IN_XPDR_CODE]) XPLMSetDatai(sys_raw[SYS_IN_XPDR_CODE], code);
        return;
    }
    if (cmd == A380X_SURV_C_XPDR_IDENT) { if (sys_ident_cmd) XPLMCommandOnce(sys_ident_cmd); return; }
    if (cmd >= A380X_FCUB_C_SPEED_UP && cmd <= A380X_FCUB_C_METRIC) {
        sys_run(sys_fcu_cmd[cmd - A380X_FCUB_C_SPEED_UP], value);
        return;
    }
    if (cmd >= A380X_FCUB_C_SET_SPEED && cmd <= A380X_FCUB_C_SET_FPA) { sys_fcu_set(cmd, value); return; }
    if (cmd >= A380X_EFIS_C_CSTR && cmd <= A380X_EFIS_C_BARO_UNIT) {
        sys_run(sys_efis_cmd[side ? 1 : 0][cmd - A380X_EFIS_C_CSTR], value);
        return;
    }
    if (cmd == A380X_EFIS_C_SET_BARO_HPA) {
        XPLMDataRef r = sys_raw[side ? SYS_IN_BARO_COPILOT : SYS_IN_BARO_PILOT];
        if (r && isfinite(value) && value > 700.0 && value < 1200.0) XPLMSetDataf(r, (float)(value / 33.8638867));
        return;
    }
    if (cmd == A380X_POS_C_SET_RNP) { fmsa_rnp_pilot = isfinite(value) && value > 0.0 ? value : fmsa_nan(); return; }
}

/* ---- POSITION host: navaid tuner on the X-Plane receivers ------------------------------------- *
 * Receiver allocation [PORT interim, spec 06 section 3.1]: VOR1 = NAV1, VOR2 = NAV2. The PFD/ILS
 * inputs of the port still read NAV1/NAV2, so LS tuning writes the same frequency and course to
 * NAV1 and NAV2 (the MMR pair); the VOR slots then show the LS facility. */

static int sys_ls_freq(int f10k) { return f10k >= 10810 && f10k <= 11195 && (f10k / 10) % 2 == 1; }

static int sys_nav_freq(int i)
{
    int f[2] = {0, 0};
    if (!sys_raw[SYS_IN_NAV_FREQ]) return 0;
    XPLMGetDatavi(sys_raw[SYS_IN_NAV_FREQ], f, 0, 2);
    return f[i & 1];
}

static void sys_slot(int i, MfdSide side, A380XMfdNavaidSlot *slot)
{
    int type[2] = {0, 0}, dme[2] = {0, 0}, f = sys_nav_freq(i);
    float obs[2] = {NAN, NAN};
    char id[160] = "";
    XPLMDataRef idref = sys_raw[i ? SYS_IN_NAV2_ID : SYS_IN_NAV1_ID];
    memset(slot, 0, sizeof(*slot));
    slot->freq_mhz = f > 0 ? f / 100.0 : fmsa_nan();
    slot->course_deg = fmsa_nan();
    slot->slope_deg = fmsa_nan();
    if (idref) { int n = XPLMGetDatab(idref, id, 0, (int)sizeof(id) - 1); id[n > 0 ? n : 0] = '\0'; }
    snprintf(slot->ident, sizeof(slot->ident), "%.7s", id);
    if (sys_raw[SYS_IN_NAV_TYPE]) XPLMGetDatavi(sys_raw[SYS_IN_NAV_TYPE], type, 0, 2);
    if (sys_raw[SYS_IN_NAV_HAS_DME]) XPLMGetDatavi(sys_raw[SYS_IN_NAV_HAS_DME], dme, 0, 2);
    if (sys_raw[side ? SYS_IN_OBS_COPILOT : SYS_IN_OBS_PILOT]) XPLMGetDatavf(sys_raw[side ? SYS_IN_OBS_COPILOT : SYS_IN_OBS_PILOT], obs, 0, 2);
    slot->course_deg = isfinite(obs[i & 1]) ? obs[i & 1] : fmsa_nan();
    if (slot->ident[0]) {
        int t = type[i & 1];
        if (sys_ls_freq(f)) snprintf(slot->cls, sizeof(slot->cls), dme[i & 1] ? "ILS/DME" : "ILS");
        else if (t == xplm_Nav_DME) snprintf(slot->cls, sizeof(slot->cls), "DME");
        else snprintf(slot->cls, sizeof(slot->cls), dme[i & 1] ? "VOR/DME" : "VOR");
    }
}

static int sys_ls_locked(void)
{
    switch (guidance.lateral_mode) {
    case A380X_C1_LOC_STAR: case A380X_C1_F_LOC_STAR: case A380X_C1_LOC: case A380X_C1_F_LOC:
    case A380X_C1_LOC_BC: case A380X_C1_LOC_BC_STAR: return 1;
    default: return guidance.vertical_mode == A380X_B1_GS || guidance.vertical_mode == A380X_B1_GS_STAR
                    || guidance.vertical_mode == A380X_B1_F_GS || guidance.vertical_mode == A380X_B1_F_GS_STAR;
    }
}

static void pos_read_navaids(void *ctx, MfdSide side, A380XMfdNavaidData *out)
{
    (void)ctx;
    memset(out, 0, sizeof(*out));
    sys_slot(0, side, &out->vor[0]);
    sys_slot(1, side, &out->vor[1]);
    if (sys_ls_freq(sys_nav_freq(0))) {
        float crs = sys_raw[SYS_IN_NAV1_CRS_PILOT] ? XPLMGetDataf(sys_raw[SYS_IN_NAV1_CRS_PILOT]) : NAN;
        out->ls = out->vor[0];
        out->ls.course_deg = isfinite(crs) ? crs : fmsa_nan();
        if (fmsa_nav && out->ls.ident[0]) {
            A380XNavFix fx;
            if (a380x_nav_find(fmsa_nav, out->ls.ident, A380X_NAVMASK(A380X_NAV_LOC), &fx, 1) > 0 && isfinite(fx.gs_angle_deg))
                out->ls.slope_deg = fx.gs_angle_deg;
        }
    } else {
        out->ls.freq_mhz = out->ls.course_deg = out->ls.slope_deg = fmsa_nan();
    }
    out->ls_locked = sys_ls_locked();
}

static void sys_tune(int receiver, int f10k)
{
    int f[2];
    if (!sys_raw[SYS_IN_NAV_FREQ]) return;
    if (receiver == A380X_POS_RCV_LS) { f[0] = f[1] = f10k; XPLMSetDatavi(sys_raw[SYS_IN_NAV_FREQ], f, 0, 2); }
    else XPLMSetDatavi(sys_raw[SYS_IN_NAV_FREQ], &f10k, receiver == A380X_POS_RCV_VOR2 ? 1 : 0, 1);
}

static int pos_tune_freq(void *ctx, int receiver, double mhz)
{
    int f10k;
    (void)ctx;
    if (receiver == A380X_POS_RCV_LS && sys_ls_locked()) return A380X_POS_TUNE_NOT_ALLOWED;
    if (!isfinite(mhz)) return A380X_POS_TUNE_OK; /* back to auto: keep the current tuning */
    f10k = (int)lround(mhz * 100.0);
    sys_tune(receiver, f10k);
    return A380X_POS_TUNE_OK;
}

static int pos_tune_ident(void *ctx, int receiver, const char *ident)
{
    A380XNavFix fx;
    unsigned mask;
    int i;
    (void)ctx;
    if (!ident || !*ident) return A380X_POS_TUNE_OK;
    if (receiver == A380X_POS_RCV_LS && sys_ls_locked()) return A380X_POS_TUNE_NOT_ALLOWED;
    for (i = 0; i < 6; ++i) {
        const char *d = a380x_mfd_position_deselected(i);
        if (d && d[0] && !strcmp(d, ident)) return A380X_POS_TUNE_DESELECTED;
    }
    if (!fmsa_nav) return A380X_POS_TUNE_NOT_IN_DATABASE;
    mask = receiver == A380X_POS_RCV_LS ? A380X_NAVMASK(A380X_NAV_LOC)
                                        : (A380X_NAVMASK(A380X_NAV_VOR) | A380X_NAVMASK(A380X_NAV_DME));
    if (a380x_nav_find_near(fmsa_nav, ident, mask, fmsa_in.lat_deg, fmsa_in.lon_deg, &fx, 1) <= 0 || !(fx.freq > 0.0f))
        return A380X_POS_TUNE_NOT_IN_DATABASE;
    sys_tune(receiver, (int)lround(fx.freq * 100.0));
    if (receiver == A380X_POS_RCV_LS && isfinite(fx.loc_mag_brg)) {
        if (sys_raw[SYS_IN_NAV1_CRS_PILOT]) XPLMSetDataf(sys_raw[SYS_IN_NAV1_CRS_PILOT], fx.loc_mag_brg);
        if (sys_raw[SYS_IN_NAV2_CRS_COPILOT]) XPLMSetDataf(sys_raw[SYS_IN_NAV2_CRS_COPILOT], fx.loc_mag_brg);
    }
    return A380X_POS_TUNE_OK;
}

static int pos_set_course(void *ctx, int receiver, double crs)
{
    float c;
    (void)ctx;
    if (!isfinite(crs)) return A380X_POS_TUNE_OK;
    c = (float)crs;
    if (receiver == A380X_POS_RCV_LS) {
        if (sys_ls_locked()) return A380X_POS_TUNE_NOT_ALLOWED;
        if (sys_raw[SYS_IN_NAV1_CRS_PILOT]) XPLMSetDataf(sys_raw[SYS_IN_NAV1_CRS_PILOT], c);
        if (sys_raw[SYS_IN_NAV2_CRS_COPILOT]) XPLMSetDataf(sys_raw[SYS_IN_NAV2_CRS_COPILOT], c);
    } else {
        int i = receiver == A380X_POS_RCV_VOR2 ? 1 : 0;
        if (sys_raw[SYS_IN_OBS_PILOT]) XPLMSetDatavf(sys_raw[SYS_IN_OBS_PILOT], &c, i, 1);
        if (sys_raw[SYS_IN_OBS_COPILOT]) XPLMSetDatavf(sys_raw[SYS_IN_OBS_COPILOT], &c, i, 1);
    }
    return A380X_POS_TUNE_OK;
}

static int pos_navaid_exists(void *ctx, const char *ident)
{
    A380XNavFix fx;
    (void)ctx;
    return fmsa_nav && ident && *ident && a380x_nav_find(fmsa_nav, ident, A380X_NAVMASK_NAVAIDS, &fx, 1) > 0;
}

static int pos_fix_lookup(void *ctx, const char *ident, double ref_lat, double ref_lon, double *lat, double *lon)
{
    A380XNavFix fx;
    (void)ctx;
    if (!fmsa_nav || !ident || !*ident) return 0;
    if (a380x_nav_find_near(fmsa_nav, ident, A380X_NAVMASK_PLACE, ref_lat, ref_lon, &fx, 1) <= 0) return 0;
    *lat = fx.lat; *lon = fx.lon;
    return 1;
}

static int pos_adirs_enter_heading(void *ctx, int unit, double hdg)
{
    (void)ctx;
    return a380x_adirs_enter_heading(&adirs, &adirs_inputs, unit, hdg);
}

/* ---- D-ATIS provider: synthesised from X-Plane weather (spec 06 section 9.4) ----------------- */

typedef int (*SysPfnWeather)(double, double, double, XPLMWeatherInfo_t *);
static SysPfnWeather sys_weather_at;

#define ATIS_PENDING 4
static struct { int used; uint32_t id; char icao[5]; int type; double due; } atis_queue[ATIS_PENDING];
static struct { char icao[5]; int letter, rwy_hdg; char rwy[6]; double wind_dir, wind_kt, qnh; int hour; } atis_mem[8];

static void atis_request(void *ctx, const char *icao, int type, int overwrite, uint32_t id)
{
    int i;
    (void)ctx; (void)overwrite;
    for (i = 0; i < ATIS_PENDING; ++i) if (!atis_queue[i].used) break;
    if (i == ATIS_PENDING) i = 0;
    atis_queue[i].used = 1;
    atis_queue[i].id = id;
    snprintf(atis_queue[i].icao, sizeof(atis_queue[i].icao), "%.4s", icao ? icao : "");
    atis_queue[i].type = type;
    atis_queue[i].due = (double)XPLMGetElapsedTime() + 3.0; /* link delay */
}

static void atis_city_pair(void *ctx, char origin[8], char dest[8], char altn[8])
{
    (void)ctx;
    origin[0] = dest[0] = altn[0] = '\0';
    if (fmsa_fpln) fmsa_hk_city_pair(fmsa_fpln, A380X_PLAN_ACTIVE, origin, dest, altn);
}

static void atis_append(char *buf, size_t n, const char *s)
{
    size_t len = strlen(buf);
    if (len + 1 < n) snprintf(buf + len, n - len, "%s", s);
}

static void atis_build(uint32_t id, const char *icao, int type)
{
    A380XDatisAnswer a;
    XPLMWeatherInfo_t w;
    const A380XNavAirport *apt;
    A380XNavAirportInfo info;
    char part[160];
    double wdir, wkt, best = -1e9, qnh, vis, elev_m, utc;
    int i, k, slot = -1, letter, hh, mm, best_i = -1;
    memset(&a, 0, sizeof(a));
    a.request_id = id;
    snprintf(a.icao, sizeof(a.icao), "%.4s", icao);
    memset(&info, 0, sizeof(info));
    if (!fmsa_nav || !a380x_nav_airport_info(fmsa_nav, icao, &info) || !isfinite(info.lat) || !sys_weather_at) {
        a.status = A380X_DATIS_NO_REPLY;
        a380x_mfd_atccom_deliver(&a);
        return;
    }
    elev_m = isfinite(info.elev_ft) ? info.elev_ft * 0.3048 : 0.0;
    memset(&w, 0, sizeof(w));
    w.structSize = (int)sizeof(w);
    sys_weather_at(info.lat, info.lon, elev_m + 10.0, &w);
    wdir = w.wind_dir_alt; wkt = w.wind_spd_alt * 1.943844;
    qnh = floor(w.pressure_sl / 100.0);
    vis = w.visibility;
    /* runway end with the largest headwind component */
    apt = a380x_nav_airport(fmsa_nav, icao);
    for (i = 0; apt && i < apt->runway_count; ++i) {
        double hw = isfinite(apt->runways[i].true_brg) ? wkt * cos((wdir - apt->runways[i].true_brg) * M_PI / 180.0) : -1e9;
        if (wkt < 5.0 && isfinite(apt->runways[i].length_ft)) hw = apt->runways[i].length_ft; /* calm: longest */
        if (hw > best) { best = hw; best_i = i; }
    }
    for (k = 0; k < 8; ++k) if (!strcmp(atis_mem[k].icao, a.icao)) { slot = k; break; }
    if (slot < 0) {
        for (k = 0; k < 8; ++k) if (!atis_mem[k].icao[0]) { slot = k; break; }
        if (slot < 0) slot = 0;
        memset(&atis_mem[slot], 0, sizeof(atis_mem[slot]));
        snprintf(atis_mem[slot].icao, sizeof(atis_mem[slot].icao), "%s", a.icao);
        utc = isfinite(fmsa_in.utc_seconds) ? fmsa_in.utc_seconds : 0.0;
        atis_mem[slot].letter = ((int)(utc / 3600.0) + icao[0] + icao[1] + icao[2] + icao[3]) % 26;
        atis_mem[slot].hour = -1;
        atis_mem[slot].qnh = qnh; atis_mem[slot].wind_dir = wdir; atis_mem[slot].wind_kt = wkt;
    }
    utc = isfinite(fmsa_in.utc_seconds) ? fmsa_in.utc_seconds : 0.0;
    hh = (int)(utc / 3600.0) % 24; mm = (int)(utc / 60.0) % 60;
    {   /* letter change on significant changes or each HH:50 observation */
        int obs_hour = mm >= 50 ? hh : (hh + 23) % 24;
        int changed = atis_mem[slot].hour >= 0 && (obs_hour != atis_mem[slot].hour
            || fabs(fmod(wdir - atis_mem[slot].wind_dir + 540.0, 360.0) - 180.0) >= 30.0
            || fabs(wkt - atis_mem[slot].wind_kt) >= 10.0 || fabs(qnh - atis_mem[slot].qnh) >= 1.0
            || (best_i >= 0 && strcmp(atis_mem[slot].rwy, apt->runways[best_i].designator) != 0));
        if (changed) atis_mem[slot].letter = (atis_mem[slot].letter + 1) % 26;
        atis_mem[slot].hour = obs_hour;
        atis_mem[slot].wind_dir = wdir; atis_mem[slot].wind_kt = wkt; atis_mem[slot].qnh = qnh;
        if (best_i >= 0) snprintf(atis_mem[slot].rwy, sizeof(atis_mem[slot].rwy), "%s", apt->runways[best_i].designator);
        hh = obs_hour;
    }
    letter = 'A' + atis_mem[slot].letter;
    a.status = A380X_DATIS_NEW;
    a.letter = (char)letter;
    snprintf(a.text, sizeof(a.text), "%s %s ATIS INFO %c %02d50Z. ", a.icao, type == A380X_ATIS_ARR ? "ARR" : "DEP", letter, hh);
    if (best_i >= 0) {
        const A380XNavRunway *r = &apt->runways[best_i];
        if (type == A380X_ATIS_ARR) {
            snprintf(part, sizeof(part), "EXPECT %s APPROACH RWY %s. ", r->ls_ident[0] ? "ILS" : "RNAV", r->designator);
            atis_append(a.text, sizeof(a.text), part);
        }
        snprintf(part, sizeof(part), "%s RWY %s. ", type == A380X_ATIS_ARR ? "ARR" : "DEP", r->designator);
        atis_append(a.text, sizeof(a.text), part);
    }
    if (isfinite(info.trans_level_ft)) {
        snprintf(part, sizeof(part), "TRANSITION LEVEL FL%03d. ", (int)lround(info.trans_level_ft / 100.0));
        atis_append(a.text, sizeof(a.text), part);
    }
    if (wkt < 1.0) snprintf(part, sizeof(part), "WIND CALM. ");
    else snprintf(part, sizeof(part), "WIND %03d DEG %d KT. ", ((int)lround(wdir) + 359) % 360 + 1, (int)lround(wkt));
    atis_append(a.text, sizeof(a.text), part);
    if (vis >= 9999.0) snprintf(part, sizeof(part), "VIS 10 KM OR MORE. ");
    else snprintf(part, sizeof(part), "VIS %d M. ", (int)(vis / 100.0) * 100);
    atis_append(a.text, sizeof(a.text), part);
    if (w.precip_rate > 0.1f) {
        snprintf(part, sizeof(part), "%s%s. ", w.precip_rate > 0.6f ? "+" : "", w.temperature_alt <= 0.0f ? "SN" : "RA");
        atis_append(a.text, sizeof(a.text), part);
    }
    {
        int clouds = 0;
        for (i = 0; i < XPLM_NUM_CLOUD_LAYERS; ++i) {
            const XPLMWeatherInfoClouds_t *c = &w.cloud_layers[i];
            const char *cov;
            int base;
            if (!(c->coverage > 0.05f) || !(c->alt_base > 0.0f)) continue;
            cov = c->coverage < 0.25f ? "FEW" : c->coverage < 0.5f ? "SCT" : c->coverage < 0.875f ? "BKN" : "OVC";
            base = (int)((c->alt_base - elev_m) * 3.2808399 / 100.0) * 100;
            if (base < 0) base = 0;
            snprintf(part, sizeof(part), "%s%s %d FT", clouds ? " " : "CLOUD ", cov, base);
            atis_append(a.text, sizeof(a.text), part);
            ++clouds;
        }
        atis_append(a.text, sizeof(a.text), clouds ? ". " : (vis >= 9999.0 ? "CAVOK. " : "NSC. "));
    }
    snprintf(part, sizeof(part), "TEMP %d DEWPOINT %d. ", (int)lround(w.temperature_alt), (int)lround(w.dewpoint_alt));
    atis_append(a.text, sizeof(a.text), part);
    if (strchr("KPCTM", icao[0])) snprintf(part, sizeof(part), "ALTIMETER %04d. ", (int)lround(w.pressure_sl / 3386.38867 * 100.0));
    else snprintf(part, sizeof(part), "QNH %04d HPA. ", (int)qnh);
    atis_append(a.text, sizeof(a.text), part);
    snprintf(part, sizeof(part), "ACKNOWLEDGE INFO %c ON INITIAL CONTACT.", letter);
    atis_append(a.text, sizeof(a.text), part);
    a380x_mfd_atccom_deliver(&a);
}

static void atis_update(void)
{
    int i;
    double now = (double)XPLMGetElapsedTime();
    for (i = 0; i < ATIS_PENDING; ++i) {
        if (!atis_queue[i].used || now < atis_queue[i].due) continue;
        atis_queue[i].used = 0;
        atis_build(atis_queue[i].id, atis_queue[i].icao, atis_queue[i].type);
    }
}

/* ---- Lifecycle and per-frame work ------------------------------------------------------------ */

static MfdMsgQueue sys_atc_queue;
static A380XMfdFplnEnv sys_fpln_env;

static int sys_init(void)
{
    A380XMfdPositionHost ph;
    A380XMfdAtccomHost ah;
    sys_bind();
    sys_weather_at = (SysPfnWeather)XPLMFindSymbol("XPLMGetWeatherAtLocation");
    survh_defaults(fmsa_cold_and_dark);
    if (!survh_register()) return 0;
    memset(&ph, 0, sizeof(ph));
    ph.read_navaids = pos_read_navaids;
    ph.tune_ident = pos_tune_ident;
    ph.tune_freq = pos_tune_freq;
    ph.set_course = pos_set_course;
    ph.navaid_exists = pos_navaid_exists;
    ph.fix_lookup = pos_fix_lookup;
    ph.adirs_enter_heading = pos_adirs_enter_heading;
    a380x_mfd_page_position_set_host(&ph);
    memset(&ah, 0, sizeof(ah));
    ah.atis_request = atis_request;
    ah.city_pair = atis_city_pair;
    a380x_mfd_atccom_set_host(&ah);
    memset(atis_queue, 0, sizeof(atis_queue));
    memset(atis_mem, 0, sizeof(atis_mem));
    mfd_msg_init(&sys_atc_queue);
    fmsa_rnp_pilot = fmsa_nan();
    memset(&sys_fpln_env, 0, sizeof(sys_fpln_env));
    sys_fpln_env.plans = fmsa_fpln;
    a380x_mfd_fpln_set_env(&sys_fpln_env);
    return 1;
}

static void sys_shutdown(void)
{
    a380x_mfd_page_position_set_host(NULL);
    a380x_mfd_atccom_set_host(NULL);
    a380x_mfd_fpln_set_env(NULL);
    survh_unregister();
}

/* FMS reset (DATA/STATUS SWAP, DONE, aircraft reload): FMS-owned page data. */
static void sys_fms_reset(void)
{
    a380x_mfd_page_position_reset();
    a380x_mfd_atccom_reset();
    fmsa_rnp_pilot = fmsa_nan();
}

static void sys_update(double dt, int powered)
{
    survh_update(powered, fmsa_in.on_ground);
    fmsa_epu_update(dt);
    /* F-PLN pages: the plan store and the aircraft state for DIR TO / TMPY INSERT. */
    sys_fpln_env.plans = fmsa_fpln;
    sys_fpln_env.aircraft_valid = 1;
    sys_fpln_env.aircraft = fmsa_aircraft;
    sys_fpln_env.feed = fmsa_pred_valid ? &fmsa_pred_feed : NULL;   /* FMGC basic predictions */
    a380x_mfd_fpln_set_env(&sys_fpln_env);
    atis_update();
    a380x_mfd_atccom_update((double)XPLMGetElapsedTime(), fmsa_in.utc_seconds, &sys_atc_queue);
}
