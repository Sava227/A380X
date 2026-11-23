/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 thrust levers / FADEC: X-Plane adapter of the core in a380x_thrust.h (docs/THRUST_LEVERS.md).
 *
 * Levers. The OBJ click-hold-drag manipulators (ATTR_manip_drag_rotate along each lever's authored
 * arc) write a380x/xp12/thrust/lever_N_input (TLA 0..45) and reverse_N_input (0..1); the core adds
 * the magnetic CL / FLX-MCT detents, the end stops and the reverse interlocks. The resulting lever
 * is kept in X-Plane's own pilot handle, sim/cockpit2/engine/actuators/throttle_ratio (calibrated
 * ratio, fcu-calibration.ini) and prop_mode (3 = reverse), so the FCU, EWD, FWS and any hardware
 * axis keep one source of truth. Foreign changes of those actuators (hardware axes, commands) are
 * adopted as lever positions and never written back unless refused (reverse interlocks).
 *
 * Engines. The FBW FADEC N1 command of each lever (rating, FLEX, low-speed rules) is applied to
 * X-Plane through sim/operation/override/override_throttles + ENGN_thro_use while A/THR is not
 * driving the engines; the FCU adapter (fcu_thrust_link) treats this lease as its own, takes over
 * when A/THR becomes active, and uses the same FADEC schedule as the A/THR thrust ceiling.
 *
 * Instinctive disconnect. The two red buttons (athr_disconnect_left/right) animate and hold
 * a380x/xp12/fcu/athr_disconnect (FCU disconnect, FWS A/THR OFF); that command is tapped for the
 * held time: >= 15 s simulated disables A/THR, alpha floor included, until FMGC power-up on ground.
 *
 * Unity-build include of a380x_avionics.c after a380x_fcu_adapter.inc and a380x_ewd_data.inc.
 */
#include "a380x_thrust.h"
#include "a380x_ewd_fadec.h"
#include "a380x_cockpit_lever.inc"

#define THR_NAME(suffix) "a380x/xp12/thrust/" suffix

enum { THR_RAW_RATIO, THR_RAW_PROP_MODE, THR_RAW_OVERRIDE, THR_RAW_THRO_USE, THR_RAW_RUNNING, THR_RAW_N1,
       THR_RAW_PAUSED, THR_RAW_REPLAY, THR_RAW_SPEED, THR_RAW_ON_GROUND, THR_RAW_IAS, THR_RAW_OAT,
       THR_RAW_PRESSURE, THR_RAW_PRESSURE_ALT, THR_RAW_MACH, THR_RAW_TAT, THR_RAW_PACK_L, THR_RAW_PACK_R,
       THR_RAW_ENGINE_AI, THR_RAW_WING_AI, THR_RAW_COUNT };
static const char *const thr_raw_names[THR_RAW_COUNT] = {
    "sim/cockpit2/engine/actuators/throttle_ratio", "sim/cockpit2/engine/actuators/prop_mode",
    "sim/operation/override/override_throttles", "sim/flightmodel/engine/ENGN_thro_use",
    "sim/flightmodel/engine/ENGN_running", "sim/cockpit2/engine/indicators/N1_percent",
    "sim/time/paused", "sim/time/is_in_replay", "sim/time/sim_speed_actual",
    "sim/flightmodel/failures/onground_any", "sim/cockpit2/gauges/indicators/airspeed_kts_pilot",
    "sim/weather/aircraft/temperature_ambient_deg_c", "sim/weather/aircraft/barometer_current_pas",
    "sim/flightmodel2/position/pressure_altitude", "sim/flightmodel/misc/machno",
    "sim/cockpit2/temperature/outside_air_LE_temp_degc", "sim/cockpit2/bleedair/actuators/pack_left",
    "sim/cockpit2/bleedair/actuators/pack_right", "sim/cockpit2/ice/cowling_thermal_anti_ice_per_engine",
    "sim/cockpit2/ice/ice_surface_hot_bleed_air_on",
};
/* Provider datarefs read lazily (they may register after this module). */
static const char *const thr_fms_names[4] = {
    "a380x/xp12/fms/to_flex_temp_c", "a380x/xp12/fms/fmc_a_healthy",
    "a380x/xp12/fms/fmc_b_healthy", "a380x/xp12/fms/fmc_c_healthy",
};
#define THR_FCU_DISCONNECT "a380x/xp12/fcu/athr_disconnect"

enum { THR_C_DISC_LEFT, THR_C_DISC_RIGHT, THR_C_LEVERS_IDLE, THR_C_LEVERS_CL, THR_C_LEVERS_FLX,
       THR_C_LEVERS_TOGA, THR_C_LEVERS_UP, THR_C_LEVERS_DOWN, THR_C_COUNT };
static const char *const thr_command_names[THR_C_COUNT] = {
    THR_NAME("athr_disconnect_left"), THR_NAME("athr_disconnect_right"),
    THR_NAME("levers_idle"), THR_NAME("levers_cl"), THR_NAME("levers_flx_mct"), THR_NAME("levers_toga"),
    THR_NAME("levers_detent_up"), THR_NAME("levers_detent_down"),
};
static const char *const thr_command_descriptions[THR_C_COUNT] = {
    "A/THR instinctive disconnect (left of thrust lever 1): hold > 15 s disables A/THR",
    "A/THR instinctive disconnect (right of thrust lever 4): hold > 15 s disables A/THR",
    "Thrust levers: all to IDLE", "Thrust levers: all to CL (A/THR active range top)",
    "Thrust levers: all to FLX/MCT", "Thrust levers: all to TOGA",
    "Thrust levers: all to the next detent up (IDLE, CL, FLX/MCT, TOGA)",
    "Thrust levers: all to the next detent down (TOGA, FLX/MCT, CL, IDLE)",
};
/* X-Plane's own throttle step commands (keyboard / joystick buttons): tapped for the detents. */
#define THR_KEY_COUNT 10
static const char *const thr_key_names[THR_KEY_COUNT] = {
    "sim/engines/throttle_up", "sim/engines/throttle_down",
    "sim/engines/throttle_up_1", "sim/engines/throttle_up_2", "sim/engines/throttle_up_3", "sim/engines/throttle_up_4",
    "sim/engines/throttle_down_1", "sim/engines/throttle_down_2", "sim/engines/throttle_down_3",
    "sim/engines/throttle_down_4",
};

/* Manipulator inputs (writable, CockpitLeverInput) first, then fadec_mode (writable int), the
 * integer state, then the real-valued outputs. Order = tools/avionics_provider.py THRUST_DATAREFS. */
enum { THR_IN_LEVER_1, THR_IN_LEVER_2, THR_IN_LEVER_3, THR_IN_LEVER_4, THR_IN_REVERSE_2, THR_IN_REVERSE_3,
       THR_IN_COUNT };
static const char *const thr_input_names[THR_IN_COUNT] = {
    THR_NAME("lever_1_input"), THR_NAME("lever_2_input"), THR_NAME("lever_3_input"), THR_NAME("lever_4_input"),
    THR_NAME("reverse_2_input"), THR_NAME("reverse_3_input"),
};
enum { THR_R_FADEC_MODE, THR_R_LIMIT_TYPE, THR_R_FLEX_RATING, THR_R_FLEX_LATCH, THR_R_LIMITS_VALID,
       THR_R_ATHR_DISABLED, THR_R_ENGINE_OVERRIDE,
       THR_R_LEVER_1_ANIM, THR_R_LEVER_2_ANIM, THR_R_LEVER_3_ANIM, THR_R_LEVER_4_ANIM,
       THR_R_REVERSE_2_ANIM, THR_R_REVERSE_3_ANIM, THR_R_DISC_LEFT_ANIM, THR_R_DISC_RIGHT_ANIM,
       THR_R_TLA_1, THR_R_TLA_2, THR_R_TLA_3, THR_R_TLA_4,
       THR_R_N1_COMMAND_1, THR_R_N1_COMMAND_2, THR_R_N1_COMMAND_3, THR_R_N1_COMMAND_4,
       THR_R_ENGINE_COMMAND_1, THR_R_ENGINE_COMMAND_2, THR_R_ENGINE_COMMAND_3, THR_R_ENGINE_COMMAND_4,
       THR_R_LIMIT_N1, THR_R_FLEX_N1, THR_R_FLEX_TEMP, THR_R_DISC_HELD, THR_R_COUNT };
#define THR_FIRST_REAL THR_R_LEVER_1_ANIM
static const char *const thr_ref_names[THR_R_COUNT] = {
    THR_NAME("fadec_mode"), THR_NAME("thrust_limit_type"), THR_NAME("flex_rating"), THR_NAME("flex_latch"),
    THR_NAME("limits_valid"), THR_NAME("athr_disabled"), THR_NAME("engine_override"),
    THR_NAME("lever_1_anim"), THR_NAME("lever_2_anim"), THR_NAME("lever_3_anim"), THR_NAME("lever_4_anim"),
    THR_NAME("reverse_2_anim"), THR_NAME("reverse_3_anim"), THR_NAME("disc_left_anim"), THR_NAME("disc_right_anim"),
    THR_NAME("tla_1"), THR_NAME("tla_2"), THR_NAME("tla_3"), THR_NAME("tla_4"),
    THR_NAME("n1_command_1"), THR_NAME("n1_command_2"), THR_NAME("n1_command_3"), THR_NAME("n1_command_4"),
    THR_NAME("engine_command_1"), THR_NAME("engine_command_2"), THR_NAME("engine_command_3"),
    THR_NAME("engine_command_4"), THR_NAME("thrust_limit_n1"), THR_NAME("flex_n1"), THR_NAME("flex_temp_c"),
    THR_NAME("athr_disc_held_s"),
};

/* SOUND_LEVER_THROTTLE (glTF 0.00016, 2.50721, 32.52765) relative to the captain's eye (acf _pe
 * -0.5304/3.2004/3.0785 m), aircraft axes (acf = (-x, y, 35.085 - z), as a380x_elec_panel.inc);
 * per lever the handle's lateral position (glTF x +0.0381, +0.0124, -0.0120, -0.0367). */
static const float thr_click_from_eye[4][3] = {
    {0.4923f, -0.6932f, -0.5211f}, {0.5180f, -0.6932f, -0.5211f},
    {0.5424f, -0.6932f, -0.5211f}, {0.5671f, -0.6932f, -0.5211f},
};
#define THR_CLICK_FILE "thrust_detent_click.wav"
#define THR_CLICK_VOLUME 0.8f

static struct {
    A380XThrust core;
    A380XThrustInputs in;
    CockpitLeverInput input[THR_IN_COUNT];
    XPLMDataRef raw[THR_RAW_COUNT];
    XPLMDataRef fms[4];
    XPLMDataRef ref[THR_R_COUNT];
    XPLMCommandRef command[THR_C_COUNT];
    XPLMCommandRef fcu_disconnect;
    int disc_tapped, disc_held, disc_forwarded[2];
    double engine_command[A380X_THRUST_ENGINES];
    int lease, ready, replay;
    XPLMCommandRef key[THR_KEY_COUNT];
    int key_held, key_grace;
    double key_hold[A380X_THRUST_ENGINES];   /* detent ratio a held key command stopped at, < 0 none */
    float bind_timer;
    CockpitSound click;
    unsigned clicks_played;
} thr;

/* ---------------------------------------------------------------- dataref accessors */
static double thr_input_get(void *refcon)
{
    int k = (int)(intptr_t)refcon;
    if (k < 4) return a380x_detent_lever_input(&thr.core.lever[k]);
    return a380x_detent_lever_input(&thr.core.reverse[k == THR_IN_REVERSE_2 ? 1 : 2]);
}
static void thr_input_set(void *refcon, double value)
{
    int k = (int)(intptr_t)refcon;
    if (!thr.ready || thr.replay) return;
    if (k < 4) a380x_thrust_drag_lever(&thr.core, k, value);
    else a380x_thrust_drag_reverse(&thr.core, k == THR_IN_REVERSE_2 ? 1 : 2, value);
}

static double thr_value(int r)
{
    const A380XThrust *t = &thr.core;
    switch (r) {
    case THR_R_FADEC_MODE: return t->fadec_mode;
    case THR_R_LIMIT_TYPE: return t->limit_type;
    case THR_R_FLEX_RATING: return t->flex_rating[0];
    case THR_R_FLEX_LATCH: return t->flex_latch[0];
    case THR_R_LIMITS_VALID: return thr.in.limits.valid;
    case THR_R_ATHR_DISABLED: return t->athr_disabled;
    case THR_R_ENGINE_OVERRIDE: return thr.lease;
    case THR_R_LEVER_1_ANIM: case THR_R_LEVER_2_ANIM: case THR_R_LEVER_3_ANIM: case THR_R_LEVER_4_ANIM:
        return a380x_thrust_lever_anim_percent(t->lever[r - THR_R_LEVER_1_ANIM].value);
    case THR_R_REVERSE_2_ANIM: return t->reverse[1].value;
    case THR_R_REVERSE_3_ANIM: return t->reverse[2].value;
    case THR_R_DISC_LEFT_ANIM: return t->disc_anim[0];
    case THR_R_DISC_RIGHT_ANIM: return t->disc_anim[1];
    case THR_R_TLA_1: case THR_R_TLA_2: case THR_R_TLA_3: case THR_R_TLA_4: return t->tla[r - THR_R_TLA_1];
    case THR_R_N1_COMMAND_1: case THR_R_N1_COMMAND_2: case THR_R_N1_COMMAND_3: case THR_R_N1_COMMAND_4:
        return t->n1_command[r - THR_R_N1_COMMAND_1];
    case THR_R_ENGINE_COMMAND_1: case THR_R_ENGINE_COMMAND_2: case THR_R_ENGINE_COMMAND_3: case THR_R_ENGINE_COMMAND_4:
        return thr.engine_command[r - THR_R_ENGINE_COMMAND_1];
    case THR_R_LIMIT_N1: return t->limit_n1;
    case THR_R_FLEX_N1: return thr.in.limits.flex_valid ? thr.in.limits.flex : 0.0;
    case THR_R_FLEX_TEMP:
        return fabs(t->latched_flex_temp - 0.1) < 1e-6 ? 0.0 : t->latched_flex_temp;
    case THR_R_DISC_HELD: return t->disc_held_s;
    default: return 0.0;
    }
}
static int thr_get_int(void *refcon) { return (int)lround(thr_value((int)(intptr_t)refcon)); }
static float thr_get_float(void *refcon) { return (float)thr_value((int)(intptr_t)refcon); }
static double thr_get_double(void *refcon) { return thr_value((int)(intptr_t)refcon); }
static void thr_set_mode(void *refcon, int value)
{
    (void)refcon;
    if (value >= A380X_FADEC_RAW && value <= A380X_FADEC_N1_LOOP) thr.core.fadec_mode = value;
}

/* ---------------------------------------------------------------- raw reads */
static int thr_raw_int(int r) { return thr.raw[r] ? XPLMGetDatai(thr.raw[r]) : 0; }
static int thr_raw_float(int r, double low, double high, double *out)
{
    double v;
    XPLMDataTypeID type;
    if (!thr.raw[r]) return 0;
    type = XPLMGetDataRefTypes(thr.raw[r]);
    v = (type & xplmType_Double) ? XPLMGetDatad(thr.raw[r]) : (double)XPLMGetDataf(thr.raw[r]);
    if (!isfinite(v) || v < low || v > high) return 0;
    *out = v;
    return 1;
}
static int thr_ref_int(XPLMDataRef ref, int *out)
{
    if (!ref || !(XPLMGetDataRefTypes(ref) & xplmType_Int)) return 0;
    *out = XPLMGetDatai(ref);
    return 1;
}

static void thr_bind_late(void)
{
    int i;
    for (i = 0; i < 4; ++i) if (!thr.fms[i]) thr.fms[i] = XPLMFindDataRef(thr_fms_names[i]);
    cockpit_sound_bind();
}

/* FBW FADEC limits at the current conditions (same inputs as the EWD's native fallback). */
static void thr_limits(A380XThrustLimits *l, double flex_temp)
{
    A380xEwdFadecLimits n;
    double oat, pressure, altitude, mach;
    int i, ai[4] = {0}, engine_ai = 0;
    memset(l, 0, sizeof(*l));
    if (!thr_raw_float(THR_RAW_OAT, -100, 100, &oat) || !thr_raw_float(THR_RAW_PRESSURE, 1, 120000, &pressure)
        || !thr_raw_float(THR_RAW_PRESSURE_ALT, -2000, 60000, &altitude) || !thr_raw_float(THR_RAW_MACH, 0, 1, &mach))
        return;
    if (thr.raw[THR_RAW_ENGINE_AI] && XPLMGetDatavi(thr.raw[THR_RAW_ENGINE_AI], ai, 0, 4) == 4)
        for (i = 0; i < 4; ++i) engine_ai = engine_ai || ai[i] != 0;
    {
        int packs = (thr_raw_int(THR_RAW_PACK_L) != 0) + (thr_raw_int(THR_RAW_PACK_R) != 0);
        int wing = thr_raw_int(THR_RAW_WING_AI) != 0;
        if (!a380x_ewd_fadec_limits(oat, pressure * 0.01, altitude, mach, packs, engine_ai, wing, &n)) return;
        l->valid = 1;
        l->idle = n.idle_n1; l->clb = n.climb_n1; l->mct = n.mct_n1; l->toga = n.toga_n1; l->rev = n.reverse_n1;
        if (flex_temp > 0.0)
            l->flex_valid = a380x_ewd_fadec_flex_n1(oat, pressure * 0.01, altitude, mach, packs, engine_ai, wing,
                                                    flex_temp, &l->flex);
    }
}

static void thr_read_inputs(void)
{
    A380XThrustInputs *in = &thr.in;
    float n1[4] = {0};
    int running[4] = {0}, i, healthy = 0, value;
    double flex = 0.0, tat = 0.0, ias = 0.0;
    in->on_ground = thr_raw_int(THR_RAW_ON_GROUND) != 0;
    if (thr.raw[THR_RAW_RUNNING]) XPLMGetDatavi(thr.raw[THR_RAW_RUNNING], running, 0, 4);
    if (thr.raw[THR_RAW_N1]) XPLMGetDatavf(thr.raw[THR_RAW_N1], n1, 0, 4);
    for (i = 0; i < 4; ++i) { in->engine_running[i] = running[i] != 0; in->n1[i] = n1[i]; }
    in->ias_knots = thr_raw_float(THR_RAW_IAS, -50, 1000, &ias) ? ias : 0.0;
    in->tat_valid = thr_raw_float(THR_RAW_TAT, -100, 100, &tat);
    in->tat_c = tat;
    for (i = 1; i < 4; ++i) if (thr_ref_int(thr.fms[i], &value) && value == 1) healthy = 1;
    in->fmgc_powered = healthy;
    in->flex_temp_valid = 0;
    if (healthy && thr.fms[0] && (XPLMGetDataRefTypes(thr.fms[0]) & xplmType_Float)) {
        flex = XPLMGetDataf(thr.fms[0]);
        in->flex_temp_valid = isfinite(flex) && flex != 0.0 && flex > -100.0 && flex <= 99.0;
    }
    in->flex_temp_raw = in->flex_temp_valid ? flex : 0.0;
    in->athr_disconnect_held = thr.disc_held > 0;
}

static void thr_publish_link(void)
{
    int i;
    fcu_thrust_link.ready = thr.ready;
    fcu_thrust_link.flex_rating = thr.core.flex_rating[0];
    fcu_thrust_link.athr_inhibited = thr.core.athr_disabled;
    fcu_thrust_link.lease = thr.lease;
    fcu_thrust_link.ceiling_valid = thr.ready && thr.in.limits.valid && thr.core.fadec_mode != A380X_FADEC_RAW;
    for (i = 0; i < A380X_THRUST_ENGINES; ++i)
        fcu_thrust_link.ceiling[i] = fcu_thrust_link.ceiling_valid
            ? a380x_thrust_athr_ceiling(&thr.core, i, &thr.in.limits) : 0.0;
}

/* Writes the engines whose lever state must be restored or set in X-Plane. */
static void thr_write_levers(void)
{
    int i;
    for (i = 0; i < A380X_THRUST_ENGINES; ++i) if (thr.core.dirty[i]) {
        A380XXpLever x = a380x_thrust_xp_lever(&thr.core, i);
        float ratio = (float)x.ratio;
        int mode = x.reverse ? 3 : 1, current = 1;
        if (thr.raw[THR_RAW_PROP_MODE]) {
            XPLMGetDatavi(thr.raw[THR_RAW_PROP_MODE], &current, i, 1);
            if ((current == 3) != (mode == 3)) XPLMSetDatavi(thr.raw[THR_RAW_PROP_MODE], &mode, i, 1);
        }
        if (thr.raw[THR_RAW_RATIO]) XPLMSetDatavf(thr.raw[THR_RAW_RATIO], &ratio, i, 1);
        x.ratio = ratio;
        a380x_thrust_mark_written(&thr.core, i, x);
    }
}

static void thr_play_clicks(void)
{
    unsigned clicks = a380x_thrust_take_clicks(&thr.core);
    int i;
    if (!clicks) return;
    for (i = 0; i < A380X_THRUST_ENGINES; ++i) if ((clicks >> (8 * i)) & 0xffu) break;
    /* One click per frame: four levers moved together by a hardware axis sound as one detent. */
    if (i < A380X_THRUST_ENGINES && cockpit_sound_play(&thr.click, thr_click_from_eye[i], THR_CLICK_VOLUME))
        ++thr.clicks_played;
}

/* Before the FCU / guidance: levers, interlocks, FADEC rating, FLEX and the A/THR link. */
static void thrust_levers_update(float wall_dt)
{
    A380XXpLever read[A380X_THRUST_ENGINES];
    float ratio[4] = {0};
    int mode[4] = {0}, i, paused;
    double speed = 1.0, sim_dt = 0.0;
    if (!thr.ready || !plugin_enabled) return;
    if (!isfinite(wall_dt) || wall_dt < 0.0f || wall_dt > 5.0f) wall_dt = 0.0f;
    thr.bind_timer += wall_dt;
    if (thr.bind_timer >= 1.0f || !thr.fms[0]) { thr.bind_timer = 0.0f; thr_bind_late(); }
    paused = thr_raw_int(THR_RAW_PAUSED) != 0;
    thr.replay = thr_raw_int(THR_RAW_REPLAY) != 0;
    if (!thr_raw_float(THR_RAW_SPEED, 0.0, 100.0, &speed)) speed = 1.0;
    if (!paused && !thr.replay) sim_dt = (double)wall_dt * speed;
    if (fcu_thrust.calibrated) {
        A380XThrustCalibration c = { fcu_thrust.idle, fcu_thrust.cl, fcu_thrust.mct, fcu_thrust.toga, fcu_thrust.tolerance };
        thr.core.cal = c;
    } else thr.core.cal = a380x_thrust_default_calibration();
    a380x_thrust_tick(&thr.core, wall_dt);
    if (!thr.replay && thr.raw[THR_RAW_RATIO] && thr.raw[THR_RAW_PROP_MODE]
        && XPLMGetDatavf(thr.raw[THR_RAW_RATIO], ratio, 0, 4) == 4
        && XPLMGetDatavi(thr.raw[THR_RAW_PROP_MODE], mode, 0, 4) == 4) {
        int snapped[A380X_THRUST_ENGINES] = {0}, keys = thr.key_held > 0 || thr.key_grace > 0;
        if (thr.key_held <= 0 && thr.key_grace > 0) --thr.key_grace;
        for (i = 0; i < A380X_THRUST_ENGINES; ++i) {
            read[i].reverse = mode[i] == 3; read[i].ratio = ratio[i];
            if (!keys) thr.key_hold[i] = -1.0;
            else if (thr.core.written_valid && !read[i].reverse && !thr.core.written[i].reverse && isfinite(read[i].ratio)) {
                double adopted = a380x_thrust_key_detent(&thr.core.cal, thr.core.written[i].ratio, read[i].ratio,
                                                         &thr.key_hold[i]);
                if (fabs(adopted - read[i].ratio) > 1e-5) { read[i].ratio = adopted; snapped[i] = 1; }
            }
        }
        a380x_thrust_sync_external(&thr.core, read);
        for (i = 0; i < A380X_THRUST_ENGINES; ++i) if (snapped[i]) thr.core.dirty[i] = 1;  /* X-Plane back to the detent */
    }
    thr_read_inputs();
    thr_limits(&thr.in.limits, a380x_thrust_latch_flex_temp(&thr.core, &thr.in));
    a380x_thrust_update(&thr.core, &thr.in, sim_dt);
    if (!thr.replay) thr_write_levers();
    thr_play_clicks();
    a380x_thrust_disc_buttons(&thr.core, wall_dt);
    thr_publish_link();
}

static void thr_release_lease(void)
{
    if (thr.lease && thr.raw[THR_RAW_OVERRIDE] && !fcu.own_throttle) XPLMSetDatai(thr.raw[THR_RAW_OVERRIDE], 0);
    thr.lease = 0;
    fcu_thrust_link.lease = 0;
}

/* After the FCU / guidance: the FADEC N1 command to X-Plane's engines while A/THR is not active. */
static void thrust_engines_update(float wall_dt)
{
    float use[A380X_THRUST_ENGINES];
    double speed = 1.0, sim_dt = 0.0;
    int i;
    if (!thr.ready || !plugin_enabled) return;
    if (fcu.own_throttle) { thr.lease = 0; fcu_thrust_link.lease = 0; return; }   /* A/THR drives */
    if (thr.replay || thr.core.fadec_mode == A380X_FADEC_RAW || !thr.in.limits.valid || !thr.raw[THR_RAW_OVERRIDE]
        || !thr.raw[THR_RAW_THRO_USE]) {
        thr_release_lease();
        for (i = 0; i < A380X_THRUST_ENGINES; ++i) thr.engine_command[i] = a380x_thrust_xp_lever(&thr.core, i).ratio;
        return;
    }
    if (!thr.lease) {
        if (XPLMGetDatai(thr.raw[THR_RAW_OVERRIDE])) return;   /* another controller owns the throttles */
        XPLMSetDatai(thr.raw[THR_RAW_OVERRIDE], 1);
        thr.lease = 1;
    } else if (!XPLMGetDatai(thr.raw[THR_RAW_OVERRIDE])) {
        XPLMSetDatai(thr.raw[THR_RAW_OVERRIDE], 1);
    }
    fcu_thrust_link.lease = 1;
    if (!isfinite(wall_dt) || wall_dt < 0.0f || wall_dt > 5.0f) wall_dt = 0.0f;
    if (!thr_raw_float(THR_RAW_SPEED, 0.0, 100.0, &speed)) speed = 1.0;
    if (thr_raw_int(THR_RAW_PAUSED)) return;
    sim_dt = (double)wall_dt * speed;
    for (i = 0; i < A380X_THRUST_ENGINES; ++i) {
        /* A stopped engine sees the raw handle, so X-Plane's start and shutdown are unchanged. */
        thr.engine_command[i] = thr.in.engine_running[i]
            ? a380x_thrust_engine_command(&thr.core, i, &thr.in, sim_dt)
            : a380x_thrust_xp_lever(&thr.core, i).ratio;
        use[i] = (float)thr.engine_command[i];
    }
    XPLMSetDatavf(thr.raw[THR_RAW_THRO_USE], use, 0, A380X_THRUST_ENGINES);
}

/* Engines at take-off power for the legacy flight phase: FLX/MCT detent or beyond (FBW
 * FwsFlightPhases counts FLX with FLEX, >= MCT and full power). */
static int thrust_takeoff_power(int engine)
{
    return thr.ready && engine >= 0 && engine < A380X_THRUST_ENGINES && thr.core.tla[engine] >= A380X_TLA_FLX;
}

/* EWD rating source (a380x_ewd_data.inc ewd_rating_source): engine 1's FADEC, as FBW shows it. */
static int thr_ewd_rating(EwdRatingSource *out)
{
    memset(out, 0, sizeof(*out));
    if (!thr.ready || !thr.in.limits.valid || thr.core.limit_type < A380X_LIMIT_CLB) return 0;
    out->valid = 1;
    out->mode = thr.core.limit_type;
    out->rating_n1 = thr.core.limit_n1;
    out->flex_n1 = thr.in.limits.flex_valid ? thr.in.limits.flex : 0.0;
    return 1;
}

/* ---------------------------------------------------------------- commands */
static int thr_disconnect_tap(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    (void)command; (void)refcon;
    if (phase == xplm_CommandBegin) ++thr.disc_held;
    else if (phase == xplm_CommandEnd && thr.disc_held > 0) --thr.disc_held;
    return 1;   /* never consume: the FCU disconnects and the FWS counts the press */
}

/* Keyboard throttle steps: X-Plane moves its handle; thrust_levers_update applies the detents. */
static int thr_key_tap(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    (void)command; (void)refcon;
    if (phase == xplm_CommandBegin) ++thr.key_held;
    else if (phase == xplm_CommandEnd && thr.key_held > 0 && --thr.key_held == 0) thr.key_grace = 3;
    return 1;   /* X-Plane still moves the handle */
}

/* levers_idle / cl / flx_mct / toga / detent_up / detent_down: all four levers to that detent. */
static void thr_levers_to_detent(int k)
{
    static const double detent[4] = {A380X_TLA_IDLE, A380X_TLA_CL, A380X_TLA_FLX, A380X_TLA_TOGA};
    double now = -1.0, target;
    int i, d;
    for (i = 0; i < A380X_THRUST_ENGINES; ++i)
        now = fmax(now, thr.core.reverse[i].value > 0.0 ? -1.0 : thr.core.lever[i].value);
    if (k >= THR_C_LEVERS_IDLE && k <= THR_C_LEVERS_TOGA) target = detent[k - THR_C_LEVERS_IDLE];
    else if (k == THR_C_LEVERS_UP) {
        target = A380X_TLA_TOGA;
        for (d = 0; d < 4; ++d) if (detent[d] > now + 0.5) { target = detent[d]; break; }
    } else {
        target = A380X_TLA_IDLE;
        for (d = 3; d >= 0; --d) if (detent[d] < now - 0.5) { target = detent[d]; break; }
    }
    for (i = 0; i < A380X_THRUST_ENGINES; ++i) {
        if (thr.core.reverse[i].value > 0.0) a380x_detent_lever_set(&thr.core.reverse[i], 0.0);
        a380x_detent_lever_set(&thr.core.lever[i], target);
        thr.core.dirty[i] = 1;
    }
    if (cockpit_sound_play(&thr.click, thr_click_from_eye[0], THR_CLICK_VOLUME)) ++thr.clicks_played;
}

static int thr_command_handler(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    int k = (int)(intptr_t)refcon;
    (void)command;
    if (k >= THR_C_LEVERS_IDLE && k < THR_C_COUNT) {
        if (phase == xplm_CommandBegin && plugin_enabled && thr.ready && !thr.replay) thr_levers_to_detent(k);
        return 0;
    }
    if (k < 0 || k > 1) return 1;
    if (phase == xplm_CommandBegin) {
        if (!plugin_enabled || !thr.ready) return 1;
        thr.core.disc_pressed[k] = 1;
        if (thr.fcu_disconnect && !thr.disc_forwarded[k]) {
            thr.disc_forwarded[k] = 1;
            XPLMCommandBegin(thr.fcu_disconnect);
        }
    } else if (phase == xplm_CommandEnd) {
        thr.core.disc_pressed[k] = 0;
        if (thr.disc_forwarded[k]) {
            thr.disc_forwarded[k] = 0;
            XPLMCommandEnd(thr.fcu_disconnect);
        }
    }
    return 1;
}

/* ---------------------------------------------------------------- lifecycle */
/* Plugin disable: the engines follow X-Plane's handles again; held buttons are released. */
static void thrust_release_controls(void)
{
    int k;
    if (!thr.ready) return;
    for (k = 0; k < 2; ++k) {
        thr.core.disc_pressed[k] = 0;
        if (thr.disc_forwarded[k]) { thr.disc_forwarded[k] = 0; XPLMCommandEnd(thr.fcu_disconnect); }
    }
    thr_release_lease();
}

/* XPLM_MSG_PLANE_LOADED: levers re-read from X-Plane next frame, FADEC and A/THR disable reset
 * (a fresh FMGC). */
static void thrust_plane_loaded(void)
{
    int mode;
    if (!thr.ready) return;
    thrust_release_controls();
    mode = thr.core.fadec_mode;
    a380x_thrust_init(&thr.core, &thr.core.cal);
    thr.core.fadec_mode = mode;
    thr.disc_held = 0;
    memset(thr.engine_command, 0, sizeof(thr.engine_command));
    thr_publish_link();
}

static void thrust_shutdown(void)
{
    int i;
    thrust_release_controls();
    thr.ready = 0;
    if (thr.disc_tapped && thr.fcu_disconnect) XPLMUnregisterCommandHandler(thr.fcu_disconnect, thr_disconnect_tap, 1, NULL);
    thr.disc_tapped = 0;
    for (i = 0; i < THR_KEY_COUNT; ++i) if (thr.key[i]) {
        XPLMUnregisterCommandHandler(thr.key[i], thr_key_tap, 1, NULL);
        thr.key[i] = NULL;
    }
    thr.key_held = thr.key_grace = 0;
    for (i = 0; i < THR_C_COUNT; ++i) if (thr.command[i]) {
        XPLMUnregisterCommandHandler(thr.command[i], thr_command_handler, 1, (void *)(intptr_t)i);
        thr.command[i] = NULL;
    }
    for (i = 0; i < THR_IN_COUNT; ++i) cockpit_lever_input_unregister(&thr.input[i]);
    for (i = 0; i < THR_R_COUNT; ++i) if (thr.ref[i]) { XPLMUnregisterDataAccessor(thr.ref[i]); thr.ref[i] = NULL; }
    cockpit_sound_free(&thr.click);
    memset(&fcu_thrust_link, 0, sizeof(fcu_thrust_link));
    ewd_rating_source = NULL;
}

/* After fcu_init (needs a380x/xp12/fcu/athr_disconnect) and before the aircraft OBJs load. */
static int thrust_init(void)
{
    int i;
    memset(&thr, 0, sizeof(thr));
    a380x_thrust_init(&thr.core, NULL);
    for (i = 0; i < THR_RAW_COUNT; ++i)
        if (!(thr.raw[i] = XPLMFindDataRef(thr_raw_names[i]))) return 0;
    for (i = 0; i < THR_IN_COUNT; ++i) if (XPLMFindDataRef(thr_input_names[i])) return 0;
    for (i = 0; i < THR_R_COUNT; ++i) if (XPLMFindDataRef(thr_ref_names[i])) return 0;
    for (i = 0; i < THR_IN_COUNT; ++i) {
        CockpitLeverInput *in = &thr.input[i];
        in->name = thr_input_names[i];
        in->get = thr_input_get;
        in->set = thr_input_set;
        in->refcon = (void *)(intptr_t)i;
        if (!cockpit_lever_input_register(in)) { thrust_shutdown(); return 0; }
    }
    for (i = 0; i < THR_R_COUNT; ++i) {
        void *refcon = (void *)(intptr_t)i;
        if (i == THR_R_FADEC_MODE)
            thr.ref[i] = XPLMRegisterDataAccessor(thr_ref_names[i], xplmType_Int, 1, thr_get_int, thr_set_mode,
                NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, refcon, refcon);
        else if (i >= THR_FIRST_REAL)
            thr.ref[i] = XPLMRegisterDataAccessor(thr_ref_names[i], xplmType_Float | xplmType_Double, 0, NULL, NULL,
                thr_get_float, NULL, thr_get_double, NULL, NULL, NULL, NULL, NULL, NULL, NULL, refcon, NULL);
        else
            thr.ref[i] = XPLMRegisterDataAccessor(thr_ref_names[i], xplmType_Int, 0, thr_get_int, NULL, NULL, NULL,
                NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, refcon, NULL);
        if (!thr.ref[i]) { thrust_shutdown(); return 0; }
    }
    for (i = 0; i < THR_C_COUNT; ++i) {
        thr.command[i] = XPLMCreateCommand(thr_command_names[i], thr_command_descriptions[i]);
        if (!thr.command[i]) { thrust_shutdown(); return 0; }
        XPLMRegisterCommandHandler(thr.command[i], thr_command_handler, 1, (void *)(intptr_t)i);
    }
    for (i = 0; i < A380X_THRUST_ENGINES; ++i) thr.key_hold[i] = -1.0;
    for (i = 0; i < THR_KEY_COUNT; ++i)
        if ((thr.key[i] = XPLMFindCommand(thr_key_names[i]))) XPLMRegisterCommandHandler(thr.key[i], thr_key_tap, 1, NULL);
    thr.fcu_disconnect = XPLMFindCommand(THR_FCU_DISCONNECT);
    if (thr.fcu_disconnect) {
        XPLMRegisterCommandHandler(thr.fcu_disconnect, thr_disconnect_tap, 1, NULL);
        thr.disc_tapped = 1;
    }
    thr_bind_late();
    cockpit_sound_load(&thr.click, THR_CLICK_FILE);
    thr.ready = 1;
    ewd_rating_source = thr_ewd_rating;
    thr_publish_link();
    return 1;
}
