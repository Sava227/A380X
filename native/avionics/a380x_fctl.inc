/* SPDX-License-Identifier: GPL-3.0-or-later
 * X-Plane adapter of the A380 flight-control laws and steering (core: a380x_fctl.h,
 * docs/FLIGHT_CONTROLS.md).
 *
 * Flight laws. The pilot's yoke ratios (sim/joystick/yoke_*_ratio: joystick, mouse or keyboard, as
 * X-Plane reads them) are the sidestick. The law's surface order reaches X-Plane's flight model
 * through the artificial-stability channel: sim/operation/override/override_artstab with
 * sim/joystick/artstab_*_ratio = order − pilot input, so X-Plane's FCTLC_* = yoke + artstab = the law's
 * order and no pilot input path is taken away. The AP keeps its own path (override_joystick_pitch/
 * roll, a380x_fcu_adapter.inc): while it flies, the corrections are zero and the law tracks it for a
 * bumpless disconnect. Auto-trim moves sim/cockpit2/controls/elevator_trim within the installed ACF's
 * stabiliser range; A380XTrim's own auto-trim covers the AP (a380x/xp12/fcu/servo_active) and its
 * manual trim follows a380x/xp12/fbw/normal_law.
 *
 * Steering. sim/operation/override/override_wheel_steer with sim/flightmodel2/gear/tire_steer_actual_deg
 * on the nose gear (most forward gear) and the body gears (inboard main gears, behind the wing gears).
 * The tiller: a hardware axis assigned to X-Plane's "Nosewheel tiller" (joy_mapped_axis_value[37]) or
 * the held commands a380x/xp12/steering/tiller_left / tiller_right. Stands down while BetterPushback
 * is connected (bp/connected, bp/started) or another plugin owns the steering override.
 *
 * A.FLOOR: the law's α floor reaches the A/THR through fctl_link (a380x_fcu_thrust.inc); the speed
 * brake retraction uses the high-AoA protection (SPB_ALPHA_PROTECTION).
 *
 * Unity-build include of a380x_avionics.c after a380x_fcu_adapter.inc, a380x_pfd_inputs.inc and
 * a380x_flaps.inc.
 */
#include "a380x_fctl.h"

#define FCTL_NAME(suffix) "a380x/xp12/fbw/" suffix
#define STEER_NAME(suffix) "a380x/xp12/steering/" suffix
#define FCTL_JOY_TILLER 37   /* X-Plane joy_use_tiller */

enum { FCTLR_YOKE_P, FCTLR_YOKE_R, FCTLR_YOKE_H, FCTLR_ART_P, FCTLR_ART_R, FCTLR_ART_H, FCTLR_OVR_ART, FCTLR_OVR_STEER, FCTLR_STEER,
       FCTLR_JOY_AVAIL, FCTLR_JOY_VALUE, FCTLR_IAS, FCTLR_TAS, FCTLR_GS, FCTLR_MACH, FCTLR_VS, FCTLR_THETA, FCTLR_PHI, FCTLR_ALPHA,
       FCTLR_Q, FCTLR_P, FCTLR_R, FCTLR_NZ, FCTLR_ON_GROUND, FCTLR_PAUSED, FCTLR_REPLAY, FCTLR_TRIM, FCTLR_TRIM_UP, FCTLR_TRIM_DN,
       FCTLR_GEAR_X, FCTLR_GEAR_Z, FCTLR_GEAR_TYPE, FCTLR_VNE, FCTLR_MMO, FCTLR_COUNT };
static const char *const fctl_raw_names[FCTLR_COUNT] = {
    "sim/joystick/yoke_pitch_ratio", "sim/joystick/yoke_roll_ratio", "sim/joystick/yoke_heading_ratio",
    "sim/joystick/artstab_pitch_ratio", "sim/joystick/artstab_roll_ratio", "sim/joystick/artstab_heading_ratio",
    "sim/operation/override/override_artstab", "sim/operation/override/override_wheel_steer",
    "sim/flightmodel2/gear/tire_steer_actual_deg",
    "sim/joystick/joy_mapped_axis_avail", "sim/joystick/joy_mapped_axis_value",
    "sim/flightmodel/position/indicated_airspeed", "sim/flightmodel/position/true_airspeed",
    "sim/flightmodel/position/groundspeed", "sim/flightmodel/misc/machno", "sim/flightmodel/position/vh_ind_fpm",
    "sim/flightmodel/position/theta", "sim/flightmodel/position/phi", "sim/flightmodel/position/alpha",
    "sim/flightmodel/position/Q", "sim/flightmodel/position/P", "sim/flightmodel/position/R",
    "sim/flightmodel/forces/g_nrml", "sim/flightmodel/failures/onground_any",
    "sim/time/paused", "sim/time/is_in_replay", "sim/cockpit2/controls/elevator_trim",
    "sim/aircraft/controls/acf_hstb_trim_up", "sim/aircraft/controls/acf_hstb_trim_dn",
    "sim/aircraft/parts/acf_gear_xnodef", "sim/aircraft/parts/acf_gear_znodef", "sim/aircraft/parts/acf_gear_type",
    "sim/aircraft/view/acf_Vne", "sim/aircraft/view/acf_Mmo",
};

enum { FCTLC_TOGGLE, FCTLC_STEER_TOGGLE, FCTLC_TILLER_LEFT, FCTLC_TILLER_RIGHT, FCTLC_COUNT };
static const char *const fctl_command_names[FCTLC_COUNT] = {
    FCTL_NAME("toggle"), STEER_NAME("toggle"), STEER_NAME("tiller_left"), STEER_NAME("tiller_right"),
};
static const char *const fctl_command_descriptions[FCTLC_COUNT] = {
    "Fly-by-wire laws on/off (off: X-Plane's direct controls)",
    "A380 nosewheel / body-wheel steering on/off (off: X-Plane's own steering)",
    "Nosewheel tiller left (hold)", "Nosewheel tiller right (hold)",
};

enum { FCTLD_ENABLED, FCTLD_NORMAL, FCTLD_LAW, FCTLD_MODE, FCTLD_PROT, FCTLD_TRIM, FCTLD_STEER_ACTIVE,   /* ints first */
       FCTLD_FIRST_FLOAT, FCTLD_NZ = FCTLD_FIRST_FLOAT, FCTLD_Q, FCTLD_P, FCTLD_ALPHA_PROT, FCTLD_ALPHA_MAX, FCTLD_NOSE, FCTLD_BODY, FCTLD_TILLER,
       FCTLD_COUNT };
static const char *const fctl_dataref_names[FCTLD_COUNT] = {
    FCTL_NAME("enabled"), FCTL_NAME("normal_law"), FCTL_NAME("law"), FCTL_NAME("mode"), FCTL_NAME("protections"),
    FCTL_NAME("auto_trim"), STEER_NAME("active"),
    FCTL_NAME("nz_command"), FCTL_NAME("pitch_rate_command"), FCTL_NAME("roll_rate_command"),
    FCTL_NAME("alpha_prot"), FCTL_NAME("alpha_max"), STEER_NAME("nose_deg"), STEER_NAME("body_deg"),
    STEER_NAME("tiller"),
};

static struct {
    int ready, laws_on, steer_on, own_artstab, own_steer, gears_known, nose, body[4], body_count;
    XPLMDataRef raw[FCTLR_COUNT], bp_started, bp_connected;
    XPLMDataRef ref[FCTLD_COUNT];
    XPLMCommandRef command[FCTLC_COUNT];
    int tiller_key[2];
    double tiller_key_value;
    A380XFctlState core;
    A380XFctlInputs in;
    A380XFctlOutputs out;
    int last_law, last_mode, last_prot;
} fctl;

static float fctl_rawf(int k) { return fctl.raw[k] ? XPLMGetDataf(fctl.raw[k]) : 0.0f; }
static int fctl_rawi(int k) { return fctl.raw[k] ? XPLMGetDatai(fctl.raw[k]) : 0; }

static int fctl_get_int(void *refcon)
{
    switch ((int)(intptr_t)refcon) {
    case FCTLD_ENABLED: return fctl.ready;
    case FCTLD_NORMAL: return fctl.out.law == A380X_FCTL_LAW_NORMAL;
    case FCTLD_LAW: return fctl.out.law;
    case FCTLD_MODE: return fctl.out.mode;
    case FCTLD_PROT: return fctl.out.protections;
    case FCTLD_TRIM: return fctl.out.auto_trim;
    case FCTLD_STEER_ACTIVE: return fctl.own_steer;
    default: return 0;
    }
}
static float fctl_get_float(void *refcon)
{
    switch ((int)(intptr_t)refcon) {
    case FCTLD_NZ: return (float)fctl.out.nz_cmd;
    case FCTLD_Q: return (float)fctl.out.q_cmd;
    case FCTLD_P: return (float)fctl.out.p_cmd;
    case FCTLD_ALPHA_PROT: return (float)fctl.out.alpha_prot;
    case FCTLD_ALPHA_MAX: return (float)fctl.out.alpha_max;
    case FCTLD_NOSE: return (float)fctl.out.nose_deg;
    case FCTLD_BODY: return (float)fctl.out.body_deg;
    case FCTLD_TILLER: return (float)fctl.in.tiller;
    default: return 0.0f;
    }
}
static double fctl_get_double(void *refcon) { return fctl_get_float(refcon); }

static void fctl_release_artstab(void)
{
    if (!fctl.own_artstab) return;
    if (fctl.raw[FCTLR_ART_P]) XPLMSetDataf(fctl.raw[FCTLR_ART_P], 0.0f);
    if (fctl.raw[FCTLR_ART_R]) XPLMSetDataf(fctl.raw[FCTLR_ART_R], 0.0f);
    if (fctl.raw[FCTLR_ART_H]) XPLMSetDataf(fctl.raw[FCTLR_ART_H], 0.0f);
    if (fctl.raw[FCTLR_OVR_ART]) XPLMSetDatai(fctl.raw[FCTLR_OVR_ART], 0);
    fctl.own_artstab = 0;
}
static void fctl_release_steering(void)
{
    if (!fctl.own_steer) return;
    if (fctl.raw[FCTLR_OVR_STEER]) XPLMSetDatai(fctl.raw[FCTLR_OVR_STEER], 0);
    fctl.own_steer = 0;
}
/* Plugin disable, aircraft reload, shutdown: X-Plane's own controls and steering. */
static void fctl_release_controls(void)
{
    fctl_release_artstab();
    fctl_release_steering();
    fctl.tiller_key[0] = fctl.tiller_key[1] = 0; fctl.tiller_key_value = 0;
    a380x_fctl_init(&fctl.core);
    memset(&fctl.out, 0, sizeof(fctl.out));
    fctl_link.alpha_floor = fctl_link.alpha_prot = 0;
}

static int fctl_command_handler(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    int k = (int)(intptr_t)refcon;
    (void)command;
    if (!fctl.ready) return 1;
    if (k == FCTLC_TILLER_LEFT || k == FCTLC_TILLER_RIGHT) {
        fctl.tiller_key[k - FCTLC_TILLER_LEFT] = phase != xplm_CommandEnd;
        return 0;
    }
    if (phase != xplm_CommandBegin) return 0;
    if (k == FCTLC_TOGGLE) {
        fctl.laws_on = !fctl.laws_on;
        if (!fctl.laws_on) fctl_release_artstab();
        afs_log("FBW laws %s by the crew", fctl.laws_on ? "ON" : "OFF (X-Plane direct controls)");
    } else if (k == FCTLC_STEER_TOGGLE) {
        fctl.steer_on = !fctl.steer_on;
        if (!fctl.steer_on) fctl_release_steering();
        afs_log("A380 steering %s", fctl.steer_on ? "ON" : "OFF (X-Plane's own steering)");
    }
    return 0;
}

/* Nose gear = the most forward gear; body gears = the main gears inboard of the wing gears. */
static void fctl_find_gears(void)
{
    float x[10] = {0}, z[10] = {0};
    int type[10] = {0}, n = 0, i;
    double zmin = 1e9, xmax = 0;
    fctl.gears_known = 0; fctl.nose = -1; fctl.body_count = 0;
    if (!fctl.raw[FCTLR_GEAR_X] || !fctl.raw[FCTLR_GEAR_Z] || !fctl.raw[FCTLR_GEAR_TYPE]) return;
    XPLMGetDatavf(fctl.raw[FCTLR_GEAR_X], x, 0, 10);
    XPLMGetDatavf(fctl.raw[FCTLR_GEAR_Z], z, 0, 10);
    n = XPLMGetDatavi(fctl.raw[FCTLR_GEAR_TYPE], type, 0, 10);
    for (i = 0; i < n && i < 10; ++i) if (type[i] != 0) {
        if (z[i] < zmin) { zmin = z[i]; fctl.nose = i; }
        if (fabs(x[i]) > xmax) xmax = fabs(x[i]);
    }
    if (fctl.nose < 0) return;
    for (i = 0; i < n && i < 10; ++i)
        if (type[i] != 0 && i != fctl.nose && z[i] > 0 && fabs(x[i]) < 0.7 * xmax && fctl.body_count < 4)
            fctl.body[fctl.body_count++] = i;
    fctl.gears_known = 1;
}

static void fctl_plane_loaded(void)
{
    fctl_release_controls();
    fctl_find_gears();
}

static void fctl_log_changes(void)
{
    static const char *const law[] = {"OFF", "NORMAL", "DIRECT"}, *const mode[] = {"ground", "flight", "flare"};
    int changed = fctl.out.protections & ~fctl.last_prot;
    if (fctl.out.law != fctl.last_law || (fctl.out.law && fctl.out.mode != fctl.last_mode))
        afs_log("FBW %s law, %s mode", law[fctl.out.law % 3], mode[fctl.out.mode % 3]);
    if (changed & A380X_FCTL_PROT_ALPHA) afs_log("FBW high angle-of-attack protection (alpha %.1f, prot %.1f)",
                                                 fctl.in.alpha_deg, fctl.out.alpha_prot);
    if (changed & A380X_FCTL_PROT_HIGH_SPEED) afs_log("FBW high speed protection (IAS %.0f, M %.3f)", fctl.in.ias_kt, fctl.in.mach);
    if (changed & A380X_FCTL_PROT_ALPHA_FLOOR) afs_log("FBW ALPHA FLOOR request");
    fctl.last_law = fctl.out.law; fctl.last_mode = fctl.out.mode; fctl.last_prot = fctl.out.protections;
}

/* After update_guidance (the AP decided and wrote its yoke orders) and before the next physics step. */
static void fctl_update(float dt)
{
    A380XFctlInputs *in = &fctl.in;
    float value[81] = {0};
    int avail[81] = {0};
    int replay, paused, bp, foreign_art, foreign_steer, k;
    if (!fctl.ready || !plugin_enabled) return;
    if (!fctl.gears_known) fctl_find_gears();
    replay = fctl_rawi(FCTLR_REPLAY) != 0; paused = fctl_rawi(FCTLR_PAUSED) != 0;
    if (replay) { fctl_release_artstab(); fctl_release_steering(); return; }
    if (!fctl.bp_started) fctl.bp_started = XPLMFindDataRef("bp/started");
    if (!fctl.bp_connected) fctl.bp_connected = XPLMFindDataRef("bp/connected");
    bp = (fctl.bp_started && XPLMGetDatai(fctl.bp_started)) || (fctl.bp_connected && XPLMGetDatai(fctl.bp_connected));

    memset(in, 0, sizeof(*in));
    in->dt = paused || !isfinite(dt) ? 0.0 : fmin(fmax(dt, 0.0f), 0.25f);
    in->enabled = fctl.laws_on;
    in->powered = guidance_inputs.powered;
    in->adr_valid = refs.side[0].air_valid || refs.side[1].air_valid;
    in->ir_valid = refs.side[0].attitude_valid || refs.side[1].attitude_valid;
    in->on_ground = fctl_rawi(FCTLR_ON_GROUND) != 0;
    in->ap_pitch_roll = (guidance.ap1 || guidance.ap2) && (fcu.own_pitch || fcu.own_roll);
    in->ap_yaw = fcu.own_yaw;
    in->ap_pitch = guidance.pitch_servo; in->ap_roll = guidance.roll_servo;
    in->stick_pitch = fctl_rawf(FCTLR_YOKE_P); in->stick_roll = fctl_rawf(FCTLR_YOKE_R); in->pedals = fctl_rawf(FCTLR_YOKE_H);
    if (fctl.raw[FCTLR_JOY_AVAIL] && fctl.raw[FCTLR_JOY_VALUE]) {   /* int[81] / float[81], indexed by joy use */
        XPLMGetDatavi(fctl.raw[FCTLR_JOY_AVAIL], avail, 0, 81);
        XPLMGetDatavf(fctl.raw[FCTLR_JOY_VALUE], value, 0, 81);
    }
    in->tiller_hardware = avail[FCTL_JOY_TILLER] != 0 && isfinite(value[FCTL_JOY_TILLER]);
    {   /* keyboard tiller: moves while held, springs back to centre */
        double target = (double)fctl.tiller_key[1] - (double)fctl.tiller_key[0];
        double rate = target != 0.0 ? 1.5 : 3.0;
        fctl.tiller_key_value = a380x_fctl_rate(fctl.tiller_key_value, target, rate, in->dt);
    }
    in->tiller = a380x_fctl_clamp((in->tiller_hardware ? value[FCTL_JOY_TILLER] : 0.0) + fctl.tiller_key_value, -1.0, 1.0);
    in->ias_kt = fctl_rawf(FCTLR_IAS); in->tas_kt = fctl_rawf(FCTLR_TAS) * 1.9438444924;
    in->gs_kt = fctl_rawf(FCTLR_GS) * 1.9438444924; in->mach = fctl_rawf(FCTLR_MACH); in->vs_fpm = fctl_rawf(FCTLR_VS);
    in->pitch_deg = fctl_rawf(FCTLR_THETA); in->roll_deg = fctl_rawf(FCTLR_PHI); in->alpha_deg = fctl_rawf(FCTLR_ALPHA);
    in->q_dps = fctl_rawf(FCTLR_Q); in->p_dps = fctl_rawf(FCTLR_P); in->r_dps = fctl_rawf(FCTLR_R); in->nz = fctl_rawf(FCTLR_NZ);
    in->ra_valid = refs.side[0].radio_altitude_valid; in->ra_ft = refs.side[0].radio_altitude_value;
    in->flap_index = pfd_aircraft.flap_handle_index;
    in->vmo_kt = fctl_rawf(FCTLR_VNE); in->mmo = fctl_rawf(FCTLR_MMO);
    a380x_fctl_step(&fctl.core, in, &fctl.out);
    fctl_link.alpha_floor = fctl.out.alpha_floor;
    fctl_link.alpha_prot = (fctl.out.protections & A380X_FCTL_PROT_ALPHA) != 0;
    fctl_link.ready = 1;

    /* ---- flight laws through the artificial-stability channel ---- */
    foreign_art = fctl.raw[FCTLR_OVR_ART] && XPLMGetDatai(fctl.raw[FCTLR_OVR_ART]) && !fctl.own_artstab;
    if (fctl.out.active && !foreign_art) {
        if (!fctl.own_artstab) { XPLMSetDatai(fctl.raw[FCTLR_OVR_ART], 1); fctl.own_artstab = 1; }
        XPLMSetDataf(fctl.raw[FCTLR_ART_P], (float)fctl.out.artstab_pitch);
        XPLMSetDataf(fctl.raw[FCTLR_ART_R], (float)fctl.out.artstab_roll);
        XPLMSetDataf(fctl.raw[FCTLR_ART_H], (float)fctl.out.artstab_yaw);
    } else fctl_release_artstab();
    /* Auto-trim within the ACF stabiliser range (+ nose up = trim up limit). */
    if (fctl.out.auto_trim && fctl.raw[FCTLR_TRIM] && in->dt > 0) {
        double up = fctl_rawf(FCTLR_TRIM_UP), dn = fctl_rawf(FCTLR_TRIM_DN), trim = fctl_rawf(FCTLR_TRIM);
        double range = fctl.out.trim_rate_deg_s >= 0 ? up : dn;
        if (!(range > 0.5)) range = 12.0;
        trim = a380x_fctl_clamp(trim + fctl.out.trim_rate_deg_s * in->dt / range, -1.0, 1.0);
        XPLMSetDataf(fctl.raw[FCTLR_TRIM], (float)trim);
    }

    /* ---- steering ---- */
    foreign_steer = fctl.raw[FCTLR_OVR_STEER] && XPLMGetDatai(fctl.raw[FCTLR_OVR_STEER]) && !fctl.own_steer;
    if (fctl.steer_on && fctl.out.steering && fctl.gears_known && !bp && !foreign_steer && fctl.raw[FCTLR_STEER]) {
        float steer[10] = {0};
        int n = XPLMGetDatavf(fctl.raw[FCTLR_STEER], steer, 0, 10);
        if (!fctl.own_steer) { XPLMSetDatai(fctl.raw[FCTLR_OVR_STEER], 1); fctl.own_steer = 1; }
        for (k = 0; k < n && k < 10; ++k) steer[k] = 0.0f;
        if (fctl.nose >= 0 && fctl.nose < 10) steer[fctl.nose] = (float)fctl.out.nose_deg;
        for (k = 0; k < fctl.body_count; ++k) steer[fctl.body[k]] = (float)fctl.out.body_deg;
        XPLMSetDatavf(fctl.raw[FCTLR_STEER], steer, 0, n > 0 && n <= 10 ? n : 10);
    } else fctl_release_steering();
    fctl_log_changes();
}

static void fctl_shutdown(void)
{
    int i;
    if (fctl.ready) fctl_release_controls();
    for (i = 0; i < FCTLC_COUNT; ++i) if (fctl.command[i]) {
        XPLMUnregisterCommandHandler(fctl.command[i], fctl_command_handler, 1, (void *)(intptr_t)i);
        fctl.command[i] = NULL;
    }
    for (i = 0; i < FCTLD_COUNT; ++i) if (fctl.ref[i]) { XPLMUnregisterDataAccessor(fctl.ref[i]); fctl.ref[i] = NULL; }
    fctl.ready = 0;
    memset(&fctl_link, 0, sizeof(fctl_link));
}

static int fctl_init(void)
{
    int i;
    memset(&fctl, 0, sizeof(fctl));
    /* The experimental FlyWithLua FBW bridge publishes the same names: never run both. */
    for (i = 0; i < FCTLD_COUNT; ++i) if (XPLMFindDataRef(fctl_dataref_names[i])) {
        XPLMDebugString("A380X FBW: another flight-control module is loaded (a380x/xp12/fbw/*); native laws not started\n");
        return 0;
    }
    for (i = 0; i < FCTLR_COUNT; ++i) {
        fctl.raw[i] = XPLMFindDataRef(fctl_raw_names[i]);
        if (!fctl.raw[i] && i < FCTLR_GEAR_X) { XPLMDebugString("A380X FBW: missing X-Plane dataref; laws not started\n"); return 0; }
    }
    for (i = 0; i < FCTLD_COUNT; ++i) {
        void *refcon = (void *)(intptr_t)i;
        fctl.ref[i] = i < FCTLD_FIRST_FLOAT
            ? XPLMRegisterDataAccessor(fctl_dataref_names[i], xplmType_Int, 0, fctl_get_int, NULL, NULL, NULL,
                  NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, refcon, NULL)
            : XPLMRegisterDataAccessor(fctl_dataref_names[i], xplmType_Float | xplmType_Double, 0, NULL, NULL,
                  fctl_get_float, NULL, fctl_get_double, NULL, NULL, NULL, NULL, NULL, NULL, NULL, refcon, NULL);
        if (!fctl.ref[i]) { fctl_shutdown(); return 0; }
    }
    for (i = 0; i < FCTLC_COUNT; ++i) {
        fctl.command[i] = XPLMCreateCommand(fctl_command_names[i], fctl_command_descriptions[i]);
        if (!fctl.command[i]) { fctl_shutdown(); return 0; }
        XPLMRegisterCommandHandler(fctl.command[i], fctl_command_handler, 1, (void *)(intptr_t)i);
    }
    a380x_fctl_init(&fctl.core);
    fctl.laws_on = fctl.steer_on = 1;
    fctl.ready = 1;
    fctl_find_gears();
    XPLMDebugString("A380X FBW: native normal law (pitch nz demand + auto-trim, roll rate, protections) and "
                    "nosewheel/body-wheel steering ready\n");
    return 1;
}
