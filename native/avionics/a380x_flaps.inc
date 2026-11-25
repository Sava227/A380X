/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 FLAPS lever, SFCC and flap/slat drives: X-Plane adapter of the core in a380x_flaps.h
 * (docs/FLAPS_LEVER.md).
 *
 * Lever. The cockpit OBJ carries X-Plane's lift-detent click-hold-drag (ATTR_manip_drag_rotate with a lift
 * and ATTR_axis_detent_range pits at 0..4, tools/flaps_lever_cockpit.py) on a380x/xp12/flaps/lever_input
 * (lever units 0..4) and lift_input (0..1). The core adds the gates at 1 and 3, the pull-out rule and the
 * spring into a position; the lever and its pull-out sleeve are shown by lever_anim / lock_anim.
 * a380x/xp12/flaps/lever_down / lever_up and X-Plane's sim/flight_controls/flaps_* commands (intercepted)
 * move the lever one position (or to 0 / FULL / a detent) as complete actions.
 *
 * Surfaces. The FBW SFCC configuration (1 vs 1+F, relief, baulk, alpha lock) is written into X-Plane's
 * flap handle, sim/cockpit2/controls/flap_handle_request_ratio = FlapsConf / 6, on an ACF whose seven
 * detents are the seven configurations (xp_layout_ok); X-Plane's flight model then flies those flaps and
 * slats. The visible surfaces (flap_anim_ratio / slat_anim_ratio, the exterior OBJs), the PFD, the FWS and
 * the xp-kit SFCC words follow the FBW FPPU drives of the core (hydraulic green / yellow from X-Plane's
 * hydraulic_pressure_1 / _2, 3000 psi nominal scaled to FBW's 5250 psi). A change of X-Plane's handle the
 * plugin did not write (hardware flap axis, another plugin, a flyPad preset) is adopted as a lever position
 * (ratio x 4) and the configuration is written back.
 *
 * Unity-build include of a380x_avionics.c after a380x_thrust.inc (flaps_feed is declared by
 * a380x_pfd_inputs.inc, which the PFD and the FWS adapter read).
 */
#include "a380x_flaps.h"
#include "a380x_hyd_view.h"

#define FLP_NAME(suffix) "a380x/xp12/flaps/" suffix
#define FLP_HYD_SCALE (A380X_FLAPS_TARGET_PSI / 3000.0)   /* ACF engine pumps 3000 psi -> FBW 5250 psi */

enum { FLP_RAW_HANDLE, FLP_RAW_DETENTS, FLP_RAW_FLAP_DN, FLP_RAW_IAS, FLP_RAW_PRESSURE_ALT, FLP_RAW_ALPHA,
       FLP_RAW_ON_GROUND, FLP_RAW_HYD_1, FLP_RAW_HYD_2, FLP_RAW_PAUSED, FLP_RAW_REPLAY, FLP_RAW_SPEED,
       FLP_RAW_COUNT };
static const char *const flp_raw_names[FLP_RAW_COUNT] = {
    "sim/cockpit2/controls/flap_handle_request_ratio", "sim/aircraft/controls/acf_flap_detents",
    "sim/aircraft/controls/acf_flap_dn", "sim/cockpit2/gauges/indicators/airspeed_kts_pilot",
    "sim/flightmodel2/position/pressure_altitude", "sim/flightmodel/position/alpha",
    "sim/flightmodel/failures/onground_any", "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1",
    "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_2", "sim/time/paused", "sim/time/is_in_replay",
    "sim/time/sim_speed_actual",
};

/* X-Plane's own flap commands, taken over (consumed) while the plugin runs. */
enum { FLP_STOCK_DOWN, FLP_STOCK_UP, FLP_STOCK_DOWN_FULL, FLP_STOCK_UP_FULL, FLP_STOCK_DETENT_1, FLP_STOCK_COUNT
       = FLP_STOCK_DETENT_1 + 8 };
static const char *const flp_stock_names[FLP_STOCK_COUNT] = {
    "sim/flight_controls/flaps_down", "sim/flight_controls/flaps_up", "sim/flight_controls/flaps_down_full",
    "sim/flight_controls/flaps_up_full", "sim/flight_controls/flaps_detent_1", "sim/flight_controls/flaps_detent_2",
    "sim/flight_controls/flaps_detent_3", "sim/flight_controls/flaps_detent_4", "sim/flight_controls/flaps_detent_5",
    "sim/flight_controls/flaps_detent_6", "sim/flight_controls/flaps_detent_7", "sim/flight_controls/flaps_detent_8",
};

enum { FLP_C_DOWN, FLP_C_UP, FLP_C_COUNT };
static const char *const flp_command_names[FLP_C_COUNT] = { FLP_NAME("lever_down"), FLP_NAME("lever_up") };
static const char *const flp_command_descriptions[FLP_C_COUNT] = {
    "FLAPS lever one position down (extend: 0 -> 1 -> 2 -> 3 -> FULL)",
    "FLAPS lever one position up (retract: FULL -> 3 -> 2 -> 1 -> 0)",
};

/* Manipulator inputs (writable), integer state, then the real-valued outputs. Order =
 * tools/avionics_provider.py FLAPS_DATAREFS. */
enum { FLP_IN_LEVER, FLP_IN_LIFT, FLP_IN_COUNT };
static const char *const flp_input_names[FLP_IN_COUNT] = { FLP_NAME("lever_input"), FLP_NAME("lift_input") };
enum { FLP_R_HANDLE, FLP_R_LATCHED, FLP_R_CONF, FLP_R_DISPLAY, FLP_R_RELIEF, FLP_R_ALPHA_LOCK, FLP_R_BAULK,
       FLP_R_LAYOUT, FLP_R_LEVER_ANIM, FLP_R_LOCK_ANIM, FLP_R_FLAP_ANIM, FLP_R_SLAT_ANIM, FLP_R_FLAP_DEG,
       FLP_R_SLAT_DEG, FLP_R_FLAP_FPPU, FLP_R_SLAT_FPPU, FLP_R_FLAP_TARGET, FLP_R_SLAT_TARGET, FLP_R_DROOP,
       FLP_R_VFE, FLP_R_VFE_NEXT, FLP_R_COUNT };
#define FLP_FIRST_REAL FLP_R_LEVER_ANIM
static const char *const flp_ref_names[FLP_R_COUNT] = {
    FLP_NAME("handle_index"), FLP_NAME("lever_latched"), FLP_NAME("conf_index"), FLP_NAME("config_display"),
    FLP_NAME("flap_relief"), FLP_NAME("alpha_lock"), FLP_NAME("cruise_baulk"), FLP_NAME("xp_layout_ok"),
    FLP_NAME("lever_anim"), FLP_NAME("lock_anim"), FLP_NAME("flap_anim_ratio"), FLP_NAME("slat_anim_ratio"),
    FLP_NAME("flap_angle_deg"), FLP_NAME("slat_angle_deg"), FLP_NAME("flap_fppu_deg"), FLP_NAME("slat_fppu_deg"),
    FLP_NAME("flap_target_fppu_deg"), FLP_NAME("slat_target_fppu_deg"), FLP_NAME("aileron_droop_deg"),
    FLP_NAME("vfe_knots"), FLP_NAME("vfe_next_knots"),
};

static struct {
    A380XFlaps core;
    XPLMDataRef raw[FLP_RAW_COUNT];
    XPLMDataRef input[FLP_IN_COUNT];
    XPLMDataRef ref[FLP_R_COUNT];
    XPLMCommandRef command[FLP_C_COUNT];
    XPLMCommandRef stock[FLP_STOCK_COUNT];
    int ready, synced, replay, layout_ok, detents, dn_count;
    float flap_dn[10];
    float written;          /* X-Plane flap handle value last written (or adopted) */
    unsigned adoptions;     /* foreign handle changes adopted as lever positions */
} flp;

/* ---------------------------------------------------------------- dataref accessors */
static int flp_accepts_input(void) { return plugin_enabled && flp.ready && !flp.replay; }
static double flp_input_value(int k)
{
    return k == FLP_IN_LEVER ? a380x_flaps_lever_input(&flp.core.lever) : a380x_flaps_lever_lift_input(&flp.core.lever);
}
static float flp_input_get_f(void *refcon) { return (float)flp_input_value((int)(intptr_t)refcon); }
static double flp_input_get_d(void *refcon) { return flp_input_value((int)(intptr_t)refcon); }
static void flp_input_set_d(void *refcon, double value)
{
    if (!flp_accepts_input() || !isfinite(value)) return;
    if ((int)(intptr_t)refcon == FLP_IN_LEVER) a380x_flaps_lever_drag(&flp.core.lever, value);
    else a380x_flaps_lever_lift(&flp.core.lever, value);
}
static void flp_input_set_f(void *refcon, float value) { flp_input_set_d(refcon, (double)value); }

static double flp_value(int r)
{
    const A380XFlaps *f = &flp.core;
    switch (r) {
    case FLP_R_HANDLE: return f->sfcc.handle;
    case FLP_R_LATCHED: return f->lever.latched;
    case FLP_R_CONF: return f->sfcc.conf;
    case FLP_R_DISPLAY: return a380x_flaps_display_config(&f->sfcc);
    case FLP_R_RELIEF: return f->sfcc.relief;
    case FLP_R_ALPHA_LOCK: return f->sfcc.alpha_lock;
    case FLP_R_BAULK: return f->sfcc.cruise_baulk;
    case FLP_R_LAYOUT: return flp.layout_ok;
    case FLP_R_LEVER_ANIM: return f->lever.anim;
    case FLP_R_LOCK_ANIM: return f->lever.lock_anim;
    case FLP_R_FLAP_ANIM: return f->flaps.fppu / A380X_FLAPS_MAX_FPPU;
    case FLP_R_SLAT_ANIM: return f->slats.fppu / A380X_SLATS_MAX_FPPU;
    case FLP_R_FLAP_DEG: return a380x_flap_deg_of_fppu(f->flaps.fppu);
    case FLP_R_SLAT_DEG: return a380x_slat_deg_of_fppu(f->slats.fppu);
    case FLP_R_FLAP_FPPU: return f->flaps.fppu;
    case FLP_R_SLAT_FPPU: return f->slats.fppu;
    case FLP_R_FLAP_TARGET: return a380x_flaps_demand_fppu[f->sfcc.conf];
    case FLP_R_SLAT_TARGET: return a380x_slats_demand_fppu[f->sfcc.conf];
    case FLP_R_DROOP: return f->droop_deg;
    case FLP_R_VFE: return a380x_flaps_vfe(f);
    case FLP_R_VFE_NEXT: return a380x_flaps_vfe_next(f);
    default: return 0.0;
    }
}
static int flp_get_int(void *refcon) { return (int)lround(flp_value((int)(intptr_t)refcon)); }
static float flp_get_float(void *refcon) { return (float)flp_value((int)(intptr_t)refcon); }
static double flp_get_double(void *refcon) { return flp_value((int)(intptr_t)refcon); }

/* ---------------------------------------------------------------- raw reads */
static int flp_raw_int(int r) { return flp.raw[r] ? XPLMGetDatai(flp.raw[r]) : 0; }
static int flp_raw_number(int r, double low, double high, double *out)
{
    double v;
    if (!flp.raw[r]) return 0;
    v = (XPLMGetDataRefTypes(flp.raw[r]) & xplmType_Double) ? XPLMGetDatad(flp.raw[r]) : (double)XPLMGetDataf(flp.raw[r]);
    if (!isfinite(v) || v < low || v > high) return 0;
    *out = v;
    return 1;
}

static void flp_read_layout(void)
{
    flp.detents = flp_raw_int(FLP_RAW_DETENTS);
    flp.dn_count = flp.raw[FLP_RAW_FLAP_DN] ? XPLMGetDatavf(flp.raw[FLP_RAW_FLAP_DN], flp.flap_dn, 0, 10) : 0;
    if (flp.dn_count < 0 || flp.dn_count > 10) flp.dn_count = 0;
    flp.layout_ok = a380x_flaps_xp_layout_ok(flp.detents, flp.flap_dn, flp.dn_count);
}

/* FBW hydraulic speed factor of one X-Plane hydraulic system (missing dataref: pressurised). */
static double flp_side_factor(int r)
{
    double psi;
    /* A380 hydraulic model (a380x_hyd.inc): GREEN / YELLOW system pressure in psi */
    if (a380x_hyd_view.available) return a380x_flaps_pressure_factor(a380x_hyd_view.psi[r == FLP_RAW_HYD_1 ? 0 : 1]);
    if (!flp.raw[r]) return 1.0;
    if (!flp_raw_number(r, -100.0, 100000.0, &psi)) return 0.0;
    return a380x_flaps_pressure_factor(psi * FLP_HYD_SCALE);
}

static void flp_publish_feed(void)
{
    if (!flp.ready) { memset(&flaps_feed, 0, sizeof(flaps_feed)); return; }
    a380x_flaps_feed(&flp.core, &flaps_feed);
}

/* ---------------------------------------------------------------- frame */
/* Before the PFD / FWS / guidance: lever, SFCC, drives, X-Plane's flap handle. */
static void flaps_update(float wall_dt)
{
    A380XSfccInputs in;
    double current = 0.0, desired, speed = 1.0, sim_dt, green, yellow, value;
    if (!flp.ready || !plugin_enabled) return;
    if (!isfinite(wall_dt) || wall_dt < 0.0f || wall_dt > 5.0f) wall_dt = 0.0f;
    flp.replay = flp_raw_int(FLP_RAW_REPLAY) != 0;
    if (flp.replay) {   /* X-Plane replays its own flaps; re-synchronise afterwards */
        if (flp.core.lever.dragging) a380x_flaps_lever_end_drag(&flp.core.lever);
        flp.synced = 0;
        flp_publish_feed();
        return;
    }
    flp_read_layout();
    if (!flp_raw_number(FLP_RAW_HANDLE, -1.0, 2.0, &current)) current = 0.0;
    if (!flp.synced) {
        int conf = a380x_flaps_conf_of_request(current, flp.layout_ok);
        a380x_flaps_init(&flp.core, a380x_flaps_lever_of_conf[conf], conf);
        flp.written = (float)current;
        flp.synced = 1;
    } else if (fabs(current - flp.written) > 1e-4) {
        /* Hardware axis / another writer: a complete lever action, never written back as such. */
        a380x_flaps_lever_set(&flp.core.lever, a380x_flaps_hardware_position(current, flp.core.lever.detent));
        ++flp.adoptions;
    }
    a380x_flaps_lever_tick(&flp.core.lever, wall_dt);
    memset(&in, 0, sizeof(in));
    in.on_ground = flp_raw_int(FLP_RAW_ON_GROUND) != 0;
    in.ias_kt = flp_raw_number(FLP_RAW_IAS, -50.0, 1000.0, &value) ? value : 0.0;
    in.pressure_alt_ft = flp_raw_number(FLP_RAW_PRESSURE_ALT, -5000.0, 80000.0, &value) ? value : 0.0;
    in.aoa_valid = flp_raw_number(FLP_RAW_ALPHA, -90.0, 90.0, &value);
    in.aoa_deg = in.aoa_valid ? value : 0.0;
    if (!flp_raw_number(FLP_RAW_SPEED, 0.0, 100.0, &speed)) speed = 1.0;
    sim_dt = flp_raw_int(FLP_RAW_PAUSED) ? 0.0 : (double)wall_dt * speed;
    green = flp_side_factor(FLP_RAW_HYD_1);
    yellow = flp_side_factor(FLP_RAW_HYD_2);
    a380x_flaps_update(&flp.core, &in, 0.5 * green + 0.5 * yellow, green, sim_dt);
    desired = a380x_flaps_xp_request(flp.core.sfcc.conf, flp.layout_ok, flp.detents, flp.flap_dn, flp.dn_count);
    if (fabs(current - desired) > 1e-6) XPLMSetDataf(flp.raw[FLP_RAW_HANDLE], (float)desired);
    flp.written = (float)desired;
    flp_publish_feed();
}

/* ---------------------------------------------------------------- commands */
static int flp_command_handler(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    (void)command;
    if (phase != xplm_CommandBegin || !flp_accepts_input()) return 1;
    a380x_flaps_lever_step(&flp.core.lever, (int)(intptr_t)refcon == FLP_C_DOWN ? 1 : -1);
    return 1;
}

/* "Before" handler of X-Plane's flap commands: consumed while the plugin drives the flaps. */
static int flp_stock_handler(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    int k = (int)(intptr_t)refcon;
    A380XFlapsLever *l = &flp.core.lever;
    (void)command;
    if (!flp_accepts_input() || k < 0 || k >= FLP_STOCK_COUNT) return 1;
    if (phase == xplm_CommandBegin) {
        if (k == FLP_STOCK_DOWN) a380x_flaps_lever_step(l, 1);
        else if (k == FLP_STOCK_UP) a380x_flaps_lever_step(l, -1);
        else if (k == FLP_STOCK_DOWN_FULL) a380x_flaps_lever_set(l, A380X_FLAPS_FULL);
        else if (k == FLP_STOCK_UP_FULL) a380x_flaps_lever_set(l, 0);
        else a380x_flaps_lever_set(l, k - FLP_STOCK_DETENT_1 + 1 > A380X_FLAPS_FULL ? A380X_FLAPS_FULL
                                                                                      : k - FLP_STOCK_DETENT_1 + 1);
    }
    return 0;
}

/* ---------------------------------------------------------------- lifecycle */
/* Plugin disable: a drag ends; X-Plane keeps its handle (the configuration); re-sync on enable. */
static void flaps_release_controls(void)
{
    if (!flp.ready) return;
    if (flp.core.lever.dragging) a380x_flaps_lever_end_drag(&flp.core.lever);
    flp.synced = 0;
}

/* XPLM_MSG_PLANE_LOADED: the lever and the surfaces are re-read from X-Plane's handle next frame. */
static void flaps_plane_loaded(void)
{
    if (!flp.ready) return;
    flaps_release_controls();
    flp.adoptions = 0;
}

static void flaps_shutdown(void)
{
    int i;
    for (i = 0; i < FLP_STOCK_COUNT; ++i) if (flp.stock[i]) {
        XPLMUnregisterCommandHandler(flp.stock[i], flp_stock_handler, 1, (void *)(intptr_t)i);
        flp.stock[i] = NULL;
    }
    for (i = 0; i < FLP_C_COUNT; ++i) if (flp.command[i]) {
        XPLMUnregisterCommandHandler(flp.command[i], flp_command_handler, 1, (void *)(intptr_t)i);
        flp.command[i] = NULL;
    }
    for (i = 0; i < FLP_IN_COUNT; ++i) if (flp.input[i]) { XPLMUnregisterDataAccessor(flp.input[i]); flp.input[i] = NULL; }
    for (i = 0; i < FLP_R_COUNT; ++i) if (flp.ref[i]) { XPLMUnregisterDataAccessor(flp.ref[i]); flp.ref[i] = NULL; }
    flp.ready = 0;
    memset(&flaps_feed, 0, sizeof(flaps_feed));
}

/* Before the aircraft OBJs load (they bind the manipulator and animation datarefs). */
static int flaps_init(void)
{
    int i;
    memset(&flp, 0, sizeof(flp));
    memset(&flaps_feed, 0, sizeof(flaps_feed));
    a380x_flaps_init(&flp.core, 0, A380X_CONF0);
    for (i = 0; i < FLP_RAW_COUNT; ++i)
        if (!(flp.raw[i] = XPLMFindDataRef(flp_raw_names[i]))) return 0;
    for (i = 0; i < FLP_IN_COUNT; ++i) if (XPLMFindDataRef(flp_input_names[i])) return 0;
    for (i = 0; i < FLP_R_COUNT; ++i) if (XPLMFindDataRef(flp_ref_names[i])) return 0;
    for (i = 0; i < FLP_IN_COUNT; ++i) {
        void *refcon = (void *)(intptr_t)i;
        flp.input[i] = XPLMRegisterDataAccessor(flp_input_names[i], xplmType_Float | xplmType_Double, 1, NULL, NULL,
            flp_input_get_f, flp_input_set_f, flp_input_get_d, flp_input_set_d, NULL, NULL, NULL, NULL, NULL, NULL,
            refcon, refcon);
        if (!flp.input[i]) { flaps_shutdown(); return 0; }
    }
    for (i = 0; i < FLP_R_COUNT; ++i) {
        void *refcon = (void *)(intptr_t)i;
        if (i >= FLP_FIRST_REAL)
            flp.ref[i] = XPLMRegisterDataAccessor(flp_ref_names[i], xplmType_Float | xplmType_Double, 0, NULL, NULL,
                flp_get_float, NULL, flp_get_double, NULL, NULL, NULL, NULL, NULL, NULL, NULL, refcon, NULL);
        else
            flp.ref[i] = XPLMRegisterDataAccessor(flp_ref_names[i], xplmType_Int, 0, flp_get_int, NULL, NULL, NULL,
                NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, refcon, NULL);
        if (!flp.ref[i]) { flaps_shutdown(); return 0; }
    }
    for (i = 0; i < FLP_C_COUNT; ++i) {
        flp.command[i] = XPLMCreateCommand(flp_command_names[i], flp_command_descriptions[i]);
        if (!flp.command[i]) { flaps_shutdown(); return 0; }
        XPLMRegisterCommandHandler(flp.command[i], flp_command_handler, 1, (void *)(intptr_t)i);
    }
    for (i = 0; i < FLP_STOCK_COUNT; ++i) {
        flp.stock[i] = XPLMFindCommand(flp_stock_names[i]);
        if (flp.stock[i]) XPLMRegisterCommandHandler(flp.stock[i], flp_stock_handler, 1, (void *)(intptr_t)i);
    }
    flp.ready = 1;
    flp.synced = 0;
    return 1;
}
