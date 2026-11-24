/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 SPEED BRAKE lever, speed brake and ground spoilers: X-Plane adapter of the core in a380x_speedbrake.h
 * (docs/SPEED_BRAKE.md).
 *
 * Lever. The cockpit OBJ carries one X-Plane click-hold-drag along the lever arc (ATTR_manip_drag_rotate,
 * tools/speedbrake_lever_cockpit.py) on a380x/xp12/speedbrake/lever_input: handle units 0 (RET) .. 1 (FULL),
 * continued to -0.25 beyond RET toward ARM. The core turns the hand into the lever: held = pushed down,
 * the 1/2 notch, RET seating, the pull-up into ARM at RET; lever_anim / lift_anim show it. The plugin
 * commands and X-Plane's sim/flight_controls/speed_brakes_* commands (intercepted and consumed) are complete
 * lever actions; a value of X-Plane's handle the plugin did not write (hardware axis, another plugin) is
 * adopted as a lever position (X-Plane's ARMED -0.5 as ARM).
 *
 * Surfaces. FBW's PRIM speed brake and ground spoiler orders (a380x_speedbrake.h) reach X-Plane's flight
 * model through its own speedbrake handle: sim/cockpit2/controls/speedbrake_ratio = the mean FBW panel order
 * over X-Plane's maximum deflection (45 deg in flight, 50 on the ground), never X-Plane's ARMED -0.5, so
 * X-Plane's own touchdown auto-deployment never acts; it is also switched off while the plugin drives
 * (sim/aircraft/specialcontrols/acf_autosbrkEQ, restored on disable). X-Plane's surfaces, the FlyWithLua
 * exterior driver and the SD then follow FBW's command. Wheel speeds: X-Plane tire rotation x radius of the
 * four main gears (left/right by acf_gear_xnodef sign, body = inboard, wing = outboard). TLA: the thrust
 * module (a380x/xp12/thrust/tla_N), else X-Plane's throttle (x 45 deg; reverse -6..-20). High-AoA
 * protection: A.FLOOR latched (guidance), the port has no PRIM protection flag. Flap index and the 1+F bit:
 * the FLAPS module feed.
 *
 * Unity-build include of a380x_avionics.c after a380x_flaps.inc (speedbrake_feed is declared by
 * a380x_pfd_inputs.inc, which the PFD, FWS, xp-kit and the flyPad presets read).
 */
#include "a380x_speedbrake.h"
#include "a380x_cockpit_lever.inc"

#define SPB_NAME(suffix) "a380x/xp12/speedbrake/" suffix
#ifndef SPB_ALPHA_PROTECTION
#define SPB_ALPHA_PROTECTION() (guidance.alpha_floor_latched != 0)
#endif
#define SPB_KT_PER_MS 1.9438444924406046
#define SPB_CLICK_FILE "thrust_detent_click.wav"   /* the thrust detent click, reused (no new sound) */
#define SPB_CLICK_VOLUME 0.7f

enum { SPB_RAW_HANDLE, SPB_RAW_ON_GROUND, SPB_RAW_TIRE_SPEED, SPB_RAW_TIRE_RADIUS, SPB_RAW_GEAR_X,
       SPB_RAW_THROTTLE, SPB_RAW_PROP_MODE, SPB_RAW_AUTO_SBRK, SPB_RAW_PAUSED, SPB_RAW_REPLAY, SPB_RAW_SPEED,
       SPB_RAW_COUNT };
static const char *const spb_raw_names[SPB_RAW_COUNT] = {
    "sim/cockpit2/controls/speedbrake_ratio", "sim/flightmodel/failures/onground_any",
    "sim/flightmodel2/gear/tire_rotation_speed_rad_sec", "sim/flightmodel2/gear/tire_radius_mtrs",
    "sim/aircraft/parts/acf_gear_xnodef", "sim/cockpit2/engine/actuators/throttle_ratio",
    "sim/cockpit2/engine/actuators/prop_mode", "sim/aircraft/specialcontrols/acf_autosbrkEQ", "sim/time/paused",
    "sim/time/is_in_replay", "sim/time/sim_speed_actual",
};
static const char *const spb_tla_names[4] = {
    "a380x/xp12/thrust/tla_1", "a380x/xp12/thrust/tla_2", "a380x/xp12/thrust/tla_3", "a380x/xp12/thrust/tla_4",
};

/* X-Plane's own speed brake commands, taken over (consumed) while the plugin runs. */
enum { SPB_STOCK_DOWN_ONE, SPB_STOCK_UP_ONE, SPB_STOCK_DOWN_ALL, SPB_STOCK_UP_ALL, SPB_STOCK_TOGGLE,
       SPB_STOCK_ARM_TOGGLE, SPB_STOCK_COUNT };
static const char *const spb_stock_names[SPB_STOCK_COUNT] = {
    "sim/flight_controls/speed_brakes_down_one", "sim/flight_controls/speed_brakes_up_one",
    "sim/flight_controls/speed_brakes_down_all", "sim/flight_controls/speed_brakes_up_all",
    "sim/flight_controls/speed_brakes_toggle", "sim/flight_controls/speed_brakes_arm_toggle",
};

enum { SPB_C_EXTEND, SPB_C_RETRACT, SPB_C_ARM_TOGGLE, SPB_C_ARM, SPB_C_RET, SPB_C_COUNT };
static const char *const spb_command_names[SPB_C_COUNT] = {
    SPB_NAME("extend_one"), SPB_NAME("retract_one"), SPB_NAME("arm_toggle"), SPB_NAME("arm"), SPB_NAME("retract"),
};
static const char *const spb_command_descriptions[SPB_C_COUNT] = {
    "SPEED BRAKE lever one position toward FULL (ARM -> RET -> 1/2 -> FULL)",
    "SPEED BRAKE lever one position toward RET (FULL -> 1/2 -> RET -> ARM)",
    "SPEED BRAKE lever: ground spoilers ARM / disarm (lever pulled up at RET / pushed down)",
    "SPEED BRAKE lever to RET and pulled up: ground spoilers ARMED",
    "SPEED BRAKE lever to RET, ground spoilers disarmed",
};

/* Manipulator input (writable), integer state, then the real-valued outputs. Order =
 * tools/avionics_provider.py SPEEDBRAKE_DATAREFS. */
#define SPB_INPUT_NAME SPB_NAME("lever_input")
enum { SPB_R_ARMED, SPB_R_POSITION, SPB_R_HELD, SPB_R_INHIBITED, SPB_R_GS_OUT, SPB_R_PHASED, SPB_R_LIFT_ACTIVE,
       SPB_R_WHEEL_LATCH, SPB_R_HANDLE, SPB_R_LEVER_ANIM, SPB_R_LIFT_ANIM, SPB_R_SB_DEG, SPB_R_GS_DEG, SPB_R_LIFT_DEG,
       SPB_R_PANEL_1, SPB_R_PANEL_8 = SPB_R_PANEL_1 + 7, SPB_R_EXTENSION, SPB_R_XP_RATIO, SPB_R_COUNT };
#define SPB_FIRST_REAL SPB_R_HANDLE
static const char *const spb_ref_names[SPB_R_COUNT] = {
    SPB_NAME("ground_spoilers_armed"), SPB_NAME("lever_position"), SPB_NAME("lever_held"),
    SPB_NAME("speed_brake_inhibited"), SPB_NAME("ground_spoilers_out"), SPB_NAME("phased_lift_dumping"),
    SPB_NAME("spoiler_lift_active"), SPB_NAME("wheel_latch"),
    SPB_NAME("handle_position"), SPB_NAME("lever_anim"), SPB_NAME("lift_anim"), SPB_NAME("speed_brake_command_deg"),
    SPB_NAME("ground_spoiler_command_deg"), SPB_NAME("spoiler_lift_deg"),
    SPB_NAME("spoiler_1_deg"), SPB_NAME("spoiler_2_deg"), SPB_NAME("spoiler_3_deg"), SPB_NAME("spoiler_4_deg"),
    SPB_NAME("spoiler_5_deg"), SPB_NAME("spoiler_6_deg"), SPB_NAME("spoiler_7_deg"), SPB_NAME("spoiler_8_deg"),
    SPB_NAME("extension_ratio"), SPB_NAME("xp_handle_ratio"),
};

/* LEVER_SPEEDBRAKE handle (glTF 0.107, 2.4321, 32.2012 at RET) relative to the captain's eye (acf _pe
 * -0.5304/3.2004/3.0785 m), aircraft axes (acf = (-x, y, 35.085 - z), as a380x_thrust.inc). */
static const float spb_click_from_eye[3] = {0.4234f, -0.7683f, -0.1947f};

static struct {
    A380XSpeedbrake core;
    A380XSpoilerInputs in;
    CockpitLeverInput input;
    XPLMDataRef raw[SPB_RAW_COUNT];
    XPLMDataRef tla[4];
    XPLMDataRef ref[SPB_R_COUNT];
    XPLMCommandRef command[SPB_C_COUNT];
    XPLMCommandRef stock[SPB_STOCK_COUNT];
    int ready, synced, replay, after_replay, on_ground, auto_saved, auto_original, wheel[4];
    float written;          /* X-Plane speedbrake handle value last written (or adopted) */
    double xp_ratio;
    unsigned adoptions;     /* foreign handle values adopted as lever actions */
    unsigned clicks_played;
    CockpitSound click;
} spb;

#ifdef A380X_HAVE_CENTER_RIGHT
/* For the autobrake (a380x_center_right.inc, hooked at init): PRIM ground spoilers extended. */
static int speedbrake_ground_spoilers_out(void)
{
    return spb.ready && spb.core.computer.ground_spoilers_out != 0;
}
#endif

/* ---------------------------------------------------------------- dataref accessors */
static int spb_raw_int(int r);
static int spb_accepts_input(void) { return plugin_enabled && spb.ready && !spb.replay && !spb_raw_int(SPB_RAW_REPLAY); }
static double spb_input_get(void *refcon) { (void)refcon; return a380x_sb_lever_input(&spb.core.lever); }
static void spb_input_set(void *refcon, double value)
{
    (void)refcon;
    if (spb_accepts_input()) a380x_sb_lever_drag(&spb.core.lever, value);
}

static double spb_value(int r)
{
    const A380XSpeedbrakeLever *l = &spb.core.lever;
    const A380XSpoilerComputer *c = &spb.core.computer;
    if (r >= SPB_R_PANEL_1 && r <= SPB_R_PANEL_8) return c->panel_deg[r - SPB_R_PANEL_1];
    switch (r) {
    case SPB_R_ARMED: return l->armed;
    case SPB_R_POSITION: return a380x_sb_lever_position(l);
    case SPB_R_HELD: return l->dragging;
    case SPB_R_INHIBITED: return c->inhibited;
    case SPB_R_GS_OUT: return c->ground_spoilers_out;
    case SPB_R_PHASED: return c->phased;
    case SPB_R_LIFT_ACTIVE: return c->lift_active;
    case SPB_R_WHEEL_LATCH: return c->wheel_latch;
    case SPB_R_HANDLE: return a380x_sb_lever_handle(l);
    case SPB_R_LEVER_ANIM: return l->lever_anim;
    case SPB_R_LIFT_ANIM: return l->lift_anim;
    case SPB_R_SB_DEG: return c->speed_brake_deg;
    case SPB_R_GS_DEG: return c->ground_spoiler_deg;
    case SPB_R_LIFT_DEG: return c->lift_deg;
    case SPB_R_EXTENSION: return a380x_spoilers_extension(c);
    case SPB_R_XP_RATIO: return spb.xp_ratio;
    default: return 0.0;
    }
}
static int spb_get_int(void *refcon) { return (int)lround(spb_value((int)(intptr_t)refcon)); }
static float spb_get_float(void *refcon) { return (float)spb_value((int)(intptr_t)refcon); }
static double spb_get_double(void *refcon) { return spb_value((int)(intptr_t)refcon); }

/* ---------------------------------------------------------------- raw reads */
static int spb_raw_int(int r) { return spb.raw[r] ? XPLMGetDatai(spb.raw[r]) : 0; }
static int spb_raw_number(int r, double low, double high, double *out)
{
    double v;
    if (!spb.raw[r]) return 0;
    v = (XPLMGetDataRefTypes(spb.raw[r]) & xplmType_Double) ? XPLMGetDatad(spb.raw[r]) : (double)XPLMGetDataf(spb.raw[r]);
    if (!isfinite(v) || v < low || v > high) return 0;
    *out = v;
    return 1;
}

/* Main gears from their lateral attach points: left x < 0, body = inboard, wing = outboard (this ACF:
 * gear 1/2 wing at -/+6.2 m, 3/4 body at -/+2.6 m). Falls back to 3, 1, 4, 2. */
static void spb_classify_gears(void)
{
    float x[10] = {0};
    int i, n = spb.raw[SPB_RAW_GEAR_X] ? XPLMGetDatavf(spb.raw[SPB_RAW_GEAR_X], x, 0, 10) : 0;
    int lb = -1, lw = -1, rb = -1, rw = -1;
    for (i = 0; i < n && i < 10; ++i) {
        if (!isfinite(x[i]) || fabsf(x[i]) < 0.5f) continue;   /* nose (and centre-line) gear */
        if (x[i] < 0.0f) {
            if (lb < 0 || x[i] > x[lb]) lb = i;
            if (lw < 0 || x[i] < x[lw]) lw = i;
        } else {
            if (rb < 0 || x[i] < x[rb]) rb = i;
            if (rw < 0 || x[i] > x[rw]) rw = i;
        }
    }
    if (lb < 0 || rb < 0) { lb = 3; lw = 1; rb = 4; rw = 2; }
    spb.wheel[A380X_SB_WHEEL_LEFT_BODY] = lb;
    spb.wheel[A380X_SB_WHEEL_LEFT_WING] = lw;
    spb.wheel[A380X_SB_WHEEL_RIGHT_BODY] = rb;
    spb.wheel[A380X_SB_WHEEL_RIGHT_WING] = rw;
}

static void spb_read_inputs(A380XSpoilerInputs *in)
{
    float speed[10] = {0}, radius[10] = {0}, ratio[4] = {0};
    int mode[4] = {0}, i, n_speed, n_radius;
    memset(in, 0, sizeof(*in));
    in->on_ground = spb_raw_int(SPB_RAW_ON_GROUND) != 0;
    n_speed = spb.raw[SPB_RAW_TIRE_SPEED] ? XPLMGetDatavf(spb.raw[SPB_RAW_TIRE_SPEED], speed, 0, 10) : 0;
    n_radius = spb.raw[SPB_RAW_TIRE_RADIUS] ? XPLMGetDatavf(spb.raw[SPB_RAW_TIRE_RADIUS], radius, 0, 10) : 0;
    for (i = 0; i < 4; ++i) {
        int g = spb.wheel[i];
        double kt = g >= 0 && g < n_speed && g < n_radius ? (double)speed[g] * (double)radius[g] * SPB_KT_PER_MS : 0.0;
        in->wheel_kt[i] = isfinite(kt) && kt > 0.0 ? kt : 0.0;
    }
    /* TLA: the thrust module, else X-Plane's throttle (forward 0..45 deg, reverse -6..-20 deg). */
    for (i = 0; i < 4; ++i) if (!spb.tla[i]) spb.tla[i] = XPLMFindDataRef(spb_tla_names[i]);
    if (spb.tla[0] && spb.tla[1] && spb.tla[2] && spb.tla[3]) {
        for (i = 0; i < 4; ++i) {
            double v = XPLMGetDataf(spb.tla[i]);
            in->tla[i] = isfinite(v) ? v : 0.0;
        }
    } else {
        if (spb.raw[SPB_RAW_THROTTLE]) XPLMGetDatavf(spb.raw[SPB_RAW_THROTTLE], ratio, 0, 4);
        if (spb.raw[SPB_RAW_PROP_MODE]) XPLMGetDatavi(spb.raw[SPB_RAW_PROP_MODE], mode, 0, 4);
        for (i = 0; i < 4; ++i) {
            double r = isfinite(ratio[i]) ? a380x_sb_clamp(ratio[i], 0.0, 1.0) : 0.0;
            in->tla[i] = mode[i] == 3 ? -6.0 - 14.0 * r : 45.0 * r;
        }
    }
    in->alpha_prot = SPB_ALPHA_PROTECTION();
    if (flaps_feed.ready) {
        in->flap_index = flaps_feed.display_config;
        in->sfcc_valid = 1;
        in->flaps_1f = (flaps_feed.actual_position_word & (1u << 19)) != 0;   /* bit 20: flaps >= 1+F */
    }
}

static void spb_publish_feed(void)
{
    if (!spb.ready || !spb.synced) { memset(&speedbrake_feed, 0, sizeof(speedbrake_feed)); return; }
    a380x_speedbrake_feed(&spb.core, &speedbrake_feed);
}

/* X-Plane's own touchdown auto-deployment off while the plugin drives (it acts on X-Plane's ARMED). */
static void spb_auto_speedbrake(int plugin_owns)
{
    if (!spb.raw[SPB_RAW_AUTO_SBRK]) return;
    if (plugin_owns && !spb.auto_saved) {
        spb.auto_original = XPLMGetDatai(spb.raw[SPB_RAW_AUTO_SBRK]);
        spb.auto_saved = 1;
        if (spb.auto_original) XPLMSetDatai(spb.raw[SPB_RAW_AUTO_SBRK], 0);
    } else if (!plugin_owns && spb.auto_saved) {
        if (spb.auto_original) XPLMSetDatai(spb.raw[SPB_RAW_AUTO_SBRK], spb.auto_original);
        spb.auto_saved = 0;
    }
}

static void spb_play_clicks(void)
{
    unsigned clicks = a380x_sb_lever_take_clicks(&spb.core.lever);
    if (clicks && cockpit_sound_play(&spb.click, spb_click_from_eye, SPB_CLICK_VOLUME)) ++spb.clicks_played;
}

/* ---------------------------------------------------------------- frame */
/* After the thrust levers and the FLAPS lever, before the PFD / FWS / guidance. */
static void speedbrake_update(float wall_dt)
{
    double current = 0.0, speed = 1.0, sim_dt;
    if (!spb.ready || !plugin_enabled) return;
    if (!isfinite(wall_dt) || wall_dt < 0.0f || wall_dt > 5.0f) wall_dt = 0.0f;
    spb.replay = spb_raw_int(SPB_RAW_REPLAY) != 0;
    if (spb.replay) {   /* X-Plane replays its own handle; the lever and the computer hold */
        if (spb.core.lever.dragging) a380x_sb_lever_end_drag(&spb.core.lever);
        spb.after_replay = 1;
        spb_publish_feed();
        return;
    }
    if (!spb_raw_number(SPB_RAW_HANDLE, -1.0, 2.0, &current)) current = 0.0;
    if (spb.after_replay) {   /* the replayed handle is X-Plane's record of our own writes: not a lever action */
        spb.written = (float)current;
        spb.after_replay = 0;
    }
    if (!spb.synced) {
        /* Aircraft load: X-Plane's handle is the lever (ARMED -0.5 = ARM), the computer starts settled. */
        A380XSpoilerInputs in;
        spb_classify_gears();
        spb_read_inputs(&in);
        a380x_speedbrake_init(&spb.core, current < -0.25 ? 0.0 : a380x_sb_clamp(current, 0.0, 1.0),
                              current < -0.25, &in);
        spb.written = (float)current;
        spb.synced = 1;
    } else if (fabs(current - spb.written) > 1e-4) {
        /* Hardware axis / another writer: a lever action, never written back as such. */
        a380x_sb_lever_hardware(&spb.core.lever, current);
        ++spb.adoptions;
    }
    spb_auto_speedbrake(1);
    a380x_sb_lever_tick(&spb.core.lever, wall_dt);
    spb_read_inputs(&spb.in);
    spb.on_ground = spb.in.on_ground;
    if (!spb_raw_number(SPB_RAW_SPEED, 0.0, 100.0, &speed)) speed = 1.0;
    sim_dt = spb_raw_int(SPB_RAW_PAUSED) ? 0.0 : (double)wall_dt * speed;
    a380x_speedbrake_update(&spb.core, &spb.in, sim_dt);
    spb.xp_ratio = a380x_sb_xp_ratio(&spb.core.computer, spb.in.on_ground);
    if (fabs(current - spb.xp_ratio) > 1e-6) XPLMSetDataf(spb.raw[SPB_RAW_HANDLE], (float)spb.xp_ratio);
    spb.written = (float)spb.xp_ratio;
    spb_play_clicks();
    spb_publish_feed();
}

/* ---------------------------------------------------------------- commands */
static void spb_action(int k)
{
    A380XSpeedbrakeLever *l = &spb.core.lever;
    switch (k) {
    case SPB_C_EXTEND: a380x_sb_lever_step(l, 1); break;
    case SPB_C_RETRACT: a380x_sb_lever_step(l, -1); break;
    case SPB_C_ARM_TOGGLE: a380x_sb_lever_arm(l, !l->armed); break;
    case SPB_C_ARM: a380x_sb_lever_arm(l, 1); break;
    case SPB_C_RET: a380x_sb_lever_arm(l, 0); a380x_sb_lever_set(l, 0.0); break;
    default: break;
    }
}

static int spb_command_handler(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    (void)command;
    if (phase == xplm_CommandBegin && spb_accepts_input() && spb.synced) spb_action((int)(intptr_t)refcon);
    return 1;
}

/* "Before" handler of X-Plane's speed brake commands: consumed while the plugin drives the speed brakes. */
static int spb_stock_handler(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    int k = (int)(intptr_t)refcon;
    A380XSpeedbrakeLever *l = &spb.core.lever;
    (void)command;
    if (!spb_accepts_input() || !spb.synced || k < 0 || k >= SPB_STOCK_COUNT) return 1;
    if (phase == xplm_CommandBegin) {
        switch (k) {
        case SPB_STOCK_DOWN_ONE: a380x_sb_lever_step(l, 1); break;            /* "extend one" */
        case SPB_STOCK_UP_ONE: a380x_sb_lever_step(l, -1); break;             /* "retract one" (RET -> ARM) */
        case SPB_STOCK_DOWN_ALL: a380x_sb_lever_arm(l, 0); a380x_sb_lever_set(l, 1.0); break;   /* SPOILERS_ON */
        case SPB_STOCK_UP_ALL: a380x_sb_lever_arm(l, 0); a380x_sb_lever_set(l, 0.0); break;     /* SPOILERS_OFF */
        case SPB_STOCK_TOGGLE: a380x_sb_lever_toggle(l); break;
        case SPB_STOCK_ARM_TOGGLE: a380x_sb_lever_arm(l, !l->armed); break;
        default: break;
        }
    }
    return 0;
}

/* ---------------------------------------------------------------- lifecycle */
/* Plugin disable: a drag ends, X-Plane's auto speedbrake setting is restored; X-Plane keeps its handle. */
static void speedbrake_release_controls(void)
{
    if (!spb.ready) return;
    if (spb.core.lever.dragging) a380x_sb_lever_end_drag(&spb.core.lever);
    spb_auto_speedbrake(0);
}

/* XPLM_MSG_PLANE_LOADED: the lever is re-read from X-Plane's handle next frame. */
static void speedbrake_plane_loaded(void)
{
    if (!spb.ready) return;
    speedbrake_release_controls();
    spb.synced = 0;
    spb.adoptions = 0;
    memset(spb.tla, 0, sizeof(spb.tla));
    memset(&speedbrake_feed, 0, sizeof(speedbrake_feed));
}

static void speedbrake_shutdown(void)
{
    int i;
#ifdef A380X_HAVE_CENTER_RIGHT
    center_right_ground_spoilers = NULL;
#endif
    speedbrake_release_controls();
    for (i = 0; i < SPB_STOCK_COUNT; ++i) if (spb.stock[i]) {
        XPLMUnregisterCommandHandler(spb.stock[i], spb_stock_handler, 1, (void *)(intptr_t)i);
        spb.stock[i] = NULL;
    }
    for (i = 0; i < SPB_C_COUNT; ++i) if (spb.command[i]) {
        XPLMUnregisterCommandHandler(spb.command[i], spb_command_handler, 1, (void *)(intptr_t)i);
        spb.command[i] = NULL;
    }
    cockpit_lever_input_unregister(&spb.input);
    for (i = 0; i < SPB_R_COUNT; ++i) if (spb.ref[i]) { XPLMUnregisterDataAccessor(spb.ref[i]); spb.ref[i] = NULL; }
    cockpit_sound_free(&spb.click);
    spb.ready = spb.synced = 0;
    memset(&speedbrake_feed, 0, sizeof(speedbrake_feed));
}

/* Before the aircraft OBJs load (they bind the manipulator and animation datarefs). */
static int speedbrake_init(void)
{
    int i;
    A380XSpoilerInputs in;
#ifdef A380X_HAVE_CENTER_RIGHT
    center_right_ground_spoilers = speedbrake_ground_spoilers_out;
#endif
    memset(&spb, 0, sizeof(spb));
    memset(&speedbrake_feed, 0, sizeof(speedbrake_feed));
    memset(&in, 0, sizeof(in));
    a380x_speedbrake_init(&spb.core, 0.0, 0, &in);
    for (i = 0; i < SPB_RAW_COUNT; ++i)
        if (!(spb.raw[i] = XPLMFindDataRef(spb_raw_names[i]))) return 0;
    if (XPLMFindDataRef(SPB_INPUT_NAME)) return 0;
    for (i = 0; i < SPB_R_COUNT; ++i) if (XPLMFindDataRef(spb_ref_names[i])) return 0;
    spb.input.name = SPB_INPUT_NAME;
    spb.input.get = spb_input_get;
    spb.input.set = spb_input_set;
    spb.input.refcon = NULL;
    if (!cockpit_lever_input_register(&spb.input)) { speedbrake_shutdown(); return 0; }
    for (i = 0; i < SPB_R_COUNT; ++i) {
        void *refcon = (void *)(intptr_t)i;
        if (i >= SPB_FIRST_REAL)
            spb.ref[i] = XPLMRegisterDataAccessor(spb_ref_names[i], xplmType_Float | xplmType_Double, 0, NULL, NULL,
                spb_get_float, NULL, spb_get_double, NULL, NULL, NULL, NULL, NULL, NULL, NULL, refcon, NULL);
        else
            spb.ref[i] = XPLMRegisterDataAccessor(spb_ref_names[i], xplmType_Int, 0, spb_get_int, NULL, NULL, NULL,
                NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, refcon, NULL);
        if (!spb.ref[i]) { speedbrake_shutdown(); return 0; }
    }
    for (i = 0; i < SPB_C_COUNT; ++i) {
        spb.command[i] = XPLMCreateCommand(spb_command_names[i], spb_command_descriptions[i]);
        if (!spb.command[i]) { speedbrake_shutdown(); return 0; }
        XPLMRegisterCommandHandler(spb.command[i], spb_command_handler, 1, (void *)(intptr_t)i);
    }
    for (i = 0; i < SPB_STOCK_COUNT; ++i) {
        spb.stock[i] = XPLMFindCommand(spb_stock_names[i]);
        if (spb.stock[i]) XPLMRegisterCommandHandler(spb.stock[i], spb_stock_handler, 1, (void *)(intptr_t)i);
    }
    cockpit_sound_bind();
    cockpit_sound_load(&spb.click, SPB_CLICK_FILE);
    spb.ready = 1;
    spb.synced = 0;
    return 1;
}
