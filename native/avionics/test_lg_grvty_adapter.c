/* SPDX-License-Identifier: GPL-3.0-or-later
 * Production L/G GRVTY adapter (a380x_lg_grvty.inc) with the production center-right adapter
 * against a mock XPLM: registry contract, cold & dark, native interlocks on the cockpit
 * commands, simulated-time handle (pause / sim speed / replay), X-Plane gear-handle ownership,
 * stock lever-command interception, emergency override ordering, disable, reload and shutdown.
 * No simulator process is used.
 *
 *   ./test_lg_grvty_adapter              run the tests
 *   ./test_lg_grvty_adapter --contracts  C commands, D datarefs, R raw datarefs, X stock commands
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#define ARRAY_LEN(a) ((int)(sizeof(a)/sizeof((a)[0])))

typedef struct {
    char name[160];
    int registered, type, writable;
    float value, array[20];
    XPLMGetDatai_f get_i;
    XPLMGetDataf_f get_f;
    XPLMGetDatad_f get_d;
    void *refcon;
} MockRef;
typedef struct {
    char name[160];
    XPLMCommandCallback_f callback;
    void *refcon;
} MockCommand;
static MockRef mock_refs[512];
static MockCommand mock_commands[64];
static int ref_count, command_count;
static int emer_on_calls, emer_off_calls, handle_at_emer_off = -1, handle_at_emer_on = -1;

static MockRef *new_ref(const char *name)
{
    MockRef *ref;
    assert(ref_count < ARRAY_LEN(mock_refs));
    ref = &mock_refs[ref_count++];
    memset(ref, 0, sizeof(*ref));
    snprintf(ref->name, sizeof(ref->name), "%s", name);
    ref->registered = 1;
    return ref;
}
XPLMDataRef XPLMFindDataRef(const char *name)
{
    int i;
    for (i = 0; i < ref_count; ++i)
        if (mock_refs[i].registered && !strcmp(mock_refs[i].name, name)) return &mock_refs[i];
    return !strncmp(name, "sim/", 4) ? new_ref(name) : NULL;
}
static MockRef *ref(const char *name) { MockRef *r = XPLMFindDataRef(name); assert(r); return r; }
int XPLMGetDatai(XPLMDataRef handle)
{
    MockRef *r = handle;
    assert(r && r->registered);
    return r->get_i ? r->get_i(r->refcon) : (int)r->value;
}
float XPLMGetDataf(XPLMDataRef handle)
{
    MockRef *r = handle;
    assert(r && r->registered);
    return r->get_f ? r->get_f(r->refcon) : r->value;
}
double XPLMGetDatad(XPLMDataRef handle)
{
    MockRef *r = handle;
    assert(r && r->registered);
    return r->get_d ? r->get_d(r->refcon) : r->value;
}
void XPLMSetDatai(XPLMDataRef handle, int value)
{
    MockRef *r = handle;
    assert(r && r->registered && !r->get_i && !r->get_f); /* plugin datarefs are read-only */
    r->value = (float)value;
}
int XPLMGetDatavf(XPLMDataRef handle, float *values, int offset, int count)
{
    MockRef *r = handle;
    assert(r && r->registered && offset >= 0 && offset + count <= 20);
    memcpy(values, r->array + offset, (size_t)count * sizeof(*values));
    return count;
}
void XPLMSetDatavf(XPLMDataRef handle, float *values, int offset, int count)
{
    MockRef *r = handle;
    assert(r && r->registered && offset >= 0 && offset + count <= 20);
    memcpy(r->array + offset, values, (size_t)count * sizeof(*values));
}
int XPLMGetDatavi(XPLMDataRef handle, int *values, int offset, int count)
{
    MockRef *r = handle;
    int i;
    assert(r && r->registered && offset >= 0 && offset + count <= 20);
    for (i = 0; i < count; ++i) values[i] = (int)r->array[offset + i];
    return count;
}
void XPLMSetDataf(XPLMDataRef handle, float value)
{
    MockRef *r = handle;
    assert(r && r->registered && !r->get_i && !r->get_f);
    r->value = value;
}
XPLMDataRef XPLMRegisterDataAccessor(const char *name, XPLMDataTypeID type, int writable,
    XPLMGetDatai_f read_i, XPLMSetDatai_f write_i, XPLMGetDataf_f read_f, XPLMSetDataf_f write_f,
    XPLMGetDatad_f read_d, XPLMSetDatad_f write_d, XPLMGetDatavi_f read_vi, XPLMSetDatavi_f write_vi,
    XPLMGetDatavf_f read_vf, XPLMSetDatavf_f write_vf, XPLMGetDatab_f read_b, XPLMSetDatab_f write_b,
    void *read_refcon, void *write_refcon)
{
    MockRef *r;
    assert(!writable && !write_i && !write_f && !write_d && !write_refcon);
    assert(!read_vi && !write_vi && !read_vf && !write_vf && !read_b && !write_b);
    assert(type == xplmType_Int ? (read_i && !read_f && !read_d)
                                : (type == (xplmType_Float | xplmType_Double) && !read_i && read_f && read_d));
    assert(!XPLMFindDataRef(name));
    r = new_ref(name);
    r->type = type;
    r->get_i = read_i; r->get_f = read_f; r->get_d = read_d;
    r->refcon = read_refcon;
    return r;
}
void XPLMUnregisterDataAccessor(XPLMDataRef handle)
{
    MockRef *r = handle;
    assert(r && r->registered);
    r->registered = 0;
    r->get_i = NULL; r->get_f = NULL; r->get_d = NULL; r->refcon = NULL;
}
XPLMCommandRef XPLMFindCommand(const char *name)
{
    int i;
    for (i = 0; i < command_count; ++i)
        if (!strcmp(mock_commands[i].name, name)) return &mock_commands[i];
    return NULL;
}
XPLMCommandRef XPLMCreateCommand(const char *name, const char *description)
{
    MockCommand *command = XPLMFindCommand(name);
    assert(description && *description);
    if (command) return command;
    assert(command_count < ARRAY_LEN(mock_commands));
    command = &mock_commands[command_count++];
    snprintf(command->name, sizeof(command->name), "%s", name);
    return command;
}
void XPLMRegisterCommandHandler(XPLMCommandRef handle, XPLMCommandCallback_f callback, int before, void *refcon)
{
    MockCommand *command = handle;
    assert(command && before && callback && !command->callback);
    command->callback = callback;
    command->refcon = refcon;
}
void XPLMUnregisterCommandHandler(XPLMCommandRef handle, XPLMCommandCallback_f callback, int before, void *refcon)
{
    MockCommand *command = handle;
    assert(command && before && command->callback == callback && command->refcon == refcon);
    command->callback = NULL;
    command->refcon = NULL;
}
/* X-Plane's own behaviour after the plugin "before" handlers. */
static void native_command(MockCommand *command)
{
    MockRef *handle = ref("sim/cockpit2/controls/gear_handle_down");
    if (!strcmp(command->name, "sim/flight_controls/landing_gear_toggle")) handle->value = !(int)handle->value;
    else if (!strcmp(command->name, "sim/flight_controls/landing_gear_up")) handle->value = 0;
    else if (!strcmp(command->name, "sim/flight_controls/landing_gear_down")) handle->value = 1;
    else if (!strcmp(command->name, "sim/flight_controls/landing_gear_emer_on")) {
        ++emer_on_calls; handle_at_emer_on = (int)handle->value;
    } else if (!strcmp(command->name, "sim/flight_controls/landing_gear_emer_off")) {
        ++emer_off_calls; handle_at_emer_off = (int)handle->value;
    }
}
void XPLMCommandOnce(XPLMCommandRef handle)
{
    MockCommand *command = handle;
    int pass = 1;
    assert(command);
    if (command->callback) {
        pass = command->callback(command, xplm_CommandBegin, command->refcon);
        pass = command->callback(command, xplm_CommandEnd, command->refcon) && pass;
    }
    if (pass) native_command(command);
}
static MockCommand *cmd(const char *name) { return (MockCommand *)XPLMFindCommand(name); }
static void command(const char *name) { MockCommand *c = cmd(name); assert(c); XPLMCommandOnce(c); }

/* Minimal unity-build environment of a380x_avionics.c for the two adapters. */
static int plugin_enabled = 1;
static float gear_values[5] = {0, 0, 0, 0, 0}, flap_value;
static struct { float flash_time; } glare;
static struct {
    XPLMDataRef on_ground, avionics_power;
    struct {
        int radio_altitude_valid, attitude_valid, heading_valid;
        float radio_altitude_value;
    } side[2];
} refs;
#include "a380x_center_right.inc"
#include "a380x_lg_grvty.inc"

#define GEAR(name) "a380x/xp12/gear/" name
static int pub_i(const char *suffix)
{
    char name[160];
    snprintf(name, sizeof(name), "a380x/xp12/gear/%s", suffix);
    return XPLMGetDatai(ref(name));
}
static float pub_f(const char *suffix)
{
    char name[160];
    snprintf(name, sizeof(name), "a380x/xp12/gear/%s", suffix);
    return XPLMGetDataf(ref(name));
}
static int sim_handle(void) { return (int)ref("sim/cockpit2/controls/gear_handle_down")->value; }
/* Flight-loop order of a380x_avionics.c: L/G GRVTY first, then the center-right panel. */
static void frame(float dt) { lgg_update(dt); center_right_update(dt); }
static void run(float seconds) { int i, n = (int)lroundf(seconds / 0.02f); for (i = 0; i < n; ++i) frame(0.02f); }

static void setup_sim(void)
{
    static const char *const stock[] = {
        "sim/flight_controls/landing_gear_toggle", "sim/flight_controls/landing_gear_up",
        "sim/flight_controls/landing_gear_down", "sim/flight_controls/landing_gear_off",
        "sim/flight_controls/landing_gear_emer_on", "sim/flight_controls/landing_gear_emer_off",
    };
    int i;
    for (i = 0; i < ARRAY_LEN(stock); ++i) XPLMCreateCommand(stock[i], "stock");
    ref("sim/time/sim_speed_actual")->value = 1.0f;
    ref("sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1")->value = 3000.0f;
    ref("sim/cockpit2/hydraulics/indicators/hydraulic_pressure_2")->value = 3000.0f;
    refs.on_ground = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    refs.avionics_power = XPLMFindDataRef("sim/cockpit2/switches/avionics_power_on");
}

static void test_registry_and_cold_state(void)
{
    static const char *const ints[] = {"grvty_master_guard", "grvty_guard_1", "grvty_guard_2", "grvty_switch_pos",
        "grvty_handle_turns", "grvty_hyd_isolated", "grvty_doors_released", "grvty_gears_released",
        "grvty_lever_disconnected", "grvty_emer_override"};
    static const char *const reals[] = {"grvty_handle_angle_deg", "grvty_master_guard_anim", "grvty_guard_1_anim",
        "grvty_guard_2_anim", "grvty_switch_anim"};
    int i;
    char name[160];
    for (i = 0; i < ARRAY_LEN(ints); ++i) {
        snprintf(name, sizeof(name), GEAR("%s"), ints[i]);
        assert(ref(name)->type == xplmType_Int && !ref(name)->writable);
    }
    for (i = 0; i < ARRAY_LEN(reals); ++i) {
        snprintf(name, sizeof(name), GEAR("%s"), reals[i]);
        assert(ref(name)->type == (xplmType_Float | xplmType_Double) && !ref(name)->writable);
    }
    for (i = 0; i < LGG_C_COUNT; ++i) assert(cmd(lgg_command_names[i])->callback == lgg_command_handler);
    for (i = 0; i < LGG_STOCK_LEVERS; ++i) assert(cmd(lgg_stock_names[i])->callback == lgg_stock_lever_handler);
    assert(!cmd("sim/flight_controls/landing_gear_emer_on")->callback); /* commanded, never intercepted */
    /* Cold & dark: OFF, covers closed, handle at rest, lever drives the gear. */
    assert(pub_i("grvty_master_guard") == 0 && pub_i("grvty_guard_1") == 0 && pub_i("grvty_guard_2") == 0);
    assert(pub_i("grvty_switch_pos") == 1 && pub_f("grvty_switch_anim") == 1.0f && pub_f("grvty_master_guard_anim") == 0.0f);
    assert(pub_i("grvty_handle_turns") == 0 && pub_f("grvty_handle_angle_deg") == 0.0f);
    assert(!pub_i("grvty_hyd_isolated") && !pub_i("grvty_lever_disconnected") && !pub_i("grvty_emer_override"));
    assert(lgg_switch_position() == 1);
}

static void test_interlocks_on_commands(void)
{
    unsigned refused = lgg.refused;
    command(GEAR("grvty_switch_down"));
    command(GEAR("grvty_switch_up"));
    command(GEAR("grvty_guard_1_toggle"));
    run(0.5f);
    assert(lgg.refused == refused + 3 && pub_i("grvty_switch_pos") == 1 && pub_i("grvty_guard_1") == 0);
    assert(pub_f("grvty_switch_anim") == 1.0f && pub_f("grvty_guard_1_anim") == 0.0f); /* refused: no motion */
    command(GEAR("grvty_master_guard_toggle"));
    run(0.1f);
    assert(pub_i("grvty_master_guard") == 1 && pub_f("grvty_master_guard_anim") > 0.3f && pub_f("grvty_master_guard_anim") < 0.6f);
    run(0.2f);
    assert(pub_f("grvty_master_guard_anim") == 1.0f);
    command(GEAR("grvty_switch_down"));
    assert(pub_i("grvty_switch_pos") == 1 && lgg.refused == refused + 4);       /* guards closed */
    command(GEAR("grvty_guard_1_toggle"));
    command(GEAR("grvty_switch_down"));
    assert(pub_i("grvty_switch_pos") == 1 && lgg.refused == refused + 5);       /* guard 2 closed */
    command(GEAR("grvty_guard_2_toggle"));
    run(0.3f);
    assert(pub_i("grvty_guard_1") && pub_i("grvty_guard_2") && pub_f("grvty_guard_2_anim") == 1.0f);
}

static void test_gravity_extension_cycle(void)
{
    MockCommand *toggle = cmd("sim/flight_controls/landing_gear_toggle");
    float angle;
    /* FBW procedure 320700001: L/G LEVER UP (already), L/G GRVTY DOWN. */
    assert(sim_handle() == 0 && !center_right.gear_lever);
    command(GEAR("grvty_switch_down"));
    assert(pub_i("grvty_switch_pos") == 2 && lgg_switch_position() == 2);
    run(0.1f);
    assert(fabsf(pub_f("grvty_switch_anim") - 2.0f) < 1e-5f);
    run(7.0f);
    assert(pub_i("grvty_handle_turns") == 0 && !pub_i("grvty_lever_disconnected"));
    /* Pause: the handle stops (simulated time), the clicks still animate. */
    ref("sim/time/paused")->value = 1;
    angle = pub_f("grvty_handle_angle_deg");
    run(3.0f);
    assert(pub_f("grvty_handle_angle_deg") == angle);
    ref("sim/time/paused")->value = 0;
    run(0.2f);
    assert(pub_i("grvty_handle_turns") == 1 && pub_i("grvty_hyd_isolated") && pub_i("grvty_lever_disconnected"));
    assert(center_right.lever_owned && !pub_i("grvty_doors_released"));
    /* Lever lockout: every lever path moves only the crew lever. */
    command("sim/flight_controls/landing_gear_toggle");
    frame(0.0f);
    assert(sim_handle() == 0 && center_right.gear_lever == 1);
    command("a380x/xp12/center_right/gear_toggle");
    frame(0.0f);
    assert(sim_handle() == 0 && center_right.gear_lever == 0);
    command("sim/flight_controls/landing_gear_down");
    frame(0.0f);
    assert(sim_handle() == 0 && center_right.gear_lever == 1);
    command("sim/flight_controls/landing_gear_up");
    command("sim/flight_controls/landing_gear_off");
    frame(0.0f);
    assert(sim_handle() == 0 && center_right.gear_lever == 0);
    assert(XPLMGetDatai(ref("a380x/xp12/center_right/gear_lever")) == 0);
    ref("sim/cockpit2/controls/gear_handle_down")->value = 1;   /* a hardware dataref writer */
    frame(0.0f);
    assert(sim_handle() == 0 && center_right.gear_lever == 1);
    command("a380x/xp12/center_right/gear_toggle");             /* back to UP per the procedure */
    frame(0.0f);
    assert(center_right.gear_lever == 0);
    /* Double speed: 2 turns 7.2 s later in simulated time = 3.6 s wall. */
    ref("sim/time/sim_speed_actual")->value = 2.0f;
    run(3.7f);
    assert(pub_i("grvty_handle_turns") == 2 && pub_i("grvty_doors_released") && !pub_i("grvty_gears_released"));
    assert(sim_handle() == 0 && emer_on_calls == 0);
    ref("sim/time/sim_speed_actual")->value = 1.0f;
    run(7.3f);
    assert(pub_i("grvty_handle_turns") == 3 && pub_i("grvty_gears_released"));
    assert(sim_handle() == 1 && emer_on_calls == 1 && handle_at_emer_on == 1 && pub_i("grvty_emer_override"));
    assert(center_right.gear_lever == 0);                       /* lever still UP: gear down by gravity */
    run(5.0f);
    assert(emer_on_calls == 1 && pub_f("grvty_handle_angle_deg") == 1080.0f);
    /* WHEN L/G LOCKED DOWN: L/G LEVER DOWN. */
    gear_values[0] = gear_values[1] = gear_values[2] = gear_values[3] = gear_values[4] = 1.0f;
    command("a380x/xp12/center_right/gear_toggle");
    frame(0.0f);
    assert(center_right.gear_lever == 1 && center_right.gear_nose_green && sim_handle() == 1);
    /* RESET (two clicks: DOWN -> OFF -> RESET; master cover still open). */
    command(GEAR("grvty_switch_up"));
    command(GEAR("grvty_switch_up"));
    assert(pub_i("grvty_switch_pos") == 0);
    run(3.5f);
    assert(pub_i("grvty_lever_disconnected") && emer_off_calls == 0);
    run(0.2f);
    assert(!pub_i("grvty_lever_disconnected") && !center_right.lever_owned);
    assert(emer_off_calls == 1 && handle_at_emer_off == 1 && sim_handle() == 1);  /* lever DOWN: no retraction */
    run(2.0f);
    assert(pub_f("grvty_handle_angle_deg") == 0.0f);
    command(GEAR("grvty_switch_down"));                          /* RESET -> OFF */
    run(1.0f);
    assert(pub_f("grvty_handle_angle_deg") == 0.0f && pub_i("grvty_switch_pos") == 1);
    /* Normal mode again: X-Plane's handle is the lever. */
    XPLMCommandOnce(toggle);
    frame(0.0f);
    assert(sim_handle() == 0 && center_right.gear_lever == 0);
    command("a380x/xp12/center_right/gear_toggle");
    frame(0.0f);
    assert(sim_handle() == 1 && center_right.gear_lever == 1);
}

static void test_replay_disable_and_reload(void)
{
    int on = emer_on_calls, off = emer_off_calls;
    command(GEAR("grvty_switch_down"));
    run(22.0f);
    assert(pub_i("grvty_gears_released") && emer_on_calls == on + 1 && sim_handle() == 1);
    command("sim/flight_controls/landing_gear_up");              /* crew lever UP */
    frame(0.0f);
    assert(center_right.gear_lever == 0 && sim_handle() == 1);
    ref("sim/time/is_in_replay")->value = 1;
    ref("sim/cockpit2/controls/gear_handle_down")->value = 0;    /* replayed state */
    run(1.0f);
    assert(sim_handle() == 0 && pub_f("grvty_handle_angle_deg") == 1080.0f); /* replay: untouched */
    ref("sim/time/is_in_replay")->value = 0;
    ref("sim/cockpit2/controls/gear_handle_down")->value = 1;
    frame(0.0f);
    /* Plugin disable: X-Plane's handle at the crew lever (UP), then the override released. */
    lgg_release_controls();
    assert(sim_handle() == 0 && emer_off_calls == off + 1 && handle_at_emer_off == 0 && !center_right.lever_owned);
    plugin_enabled = 0;
    command("sim/flight_controls/landing_gear_toggle");
    assert(sim_handle() == 1);
    command(GEAR("grvty_switch_up"));
    assert(pub_i("grvty_switch_pos") == 2);                      /* disabled: no clicks */
    plugin_enabled = 1;
    frame(0.0f);
    assert(center_right.lever_owned && pub_i("grvty_emer_override") && emer_on_calls == on + 2);
    /* Aircraft reload: cold & dark again, override released, X-Plane's handle untouched. */
    ref("sim/cockpit2/controls/gear_handle_down")->value = 0;
    lgg_plane_loaded();
    assert(emer_off_calls == off + 2 && sim_handle() == 0 && !center_right.lever_owned);
    assert(pub_i("grvty_switch_pos") == 1 && !pub_i("grvty_master_guard") && !pub_i("grvty_guard_1") &&
           !pub_i("grvty_guard_2") && pub_f("grvty_handle_angle_deg") == 0.0f);
    run(1.0f);
    assert(pub_f("grvty_master_guard_anim") == 0.0f && pub_f("grvty_switch_anim") == 1.0f);
}

static void test_shutdown_and_reinit(void)
{
    int i;
    lgg_shutdown();
    for (i = 0; i < LGG_R_COUNT; ++i) assert(!XPLMFindDataRef(lgg_ref_names[i]));
    for (i = 0; i < LGG_STOCK_LEVERS; ++i) assert(!cmd(lgg_stock_names[i])->callback);
    for (i = 0; i < LGG_C_COUNT; ++i) assert(cmd(lgg_command_names[i]) && !cmd(lgg_command_names[i])->callback);
    assert(!lgg.ready && lgg_switch_position() == 1);
    command("sim/flight_controls/landing_gear_toggle");          /* stock behaviour without the module */
    assert(sim_handle() == 1);
    assert(lgg_init());                                          /* reload: persistent commands reused */
    assert(pub_i("grvty_switch_pos") == 1);
    lgg_shutdown();
}

int main(int argc, char **argv)
{
    int i;
    setup_sim();
    assert(center_right_init());
    assert(lgg_init());
    if (argc > 1 && !strcmp(argv[1], "--contracts")) {
        for (i = 0; i < LGG_C_COUNT; ++i) printf("C %s\n", lgg_command_names[i]);
        for (i = 0; i < LGG_R_COUNT; ++i) printf("D %s\n", lgg_ref_names[i]);
        for (i = 0; i < LGG_RAW_COUNT; ++i) printf("R %s\n", lgg_raw_names[i]);
        for (i = 0; i < LGG_STOCK_COUNT; ++i) printf("X %s\n", lgg_stock_names[i]);
        lgg_shutdown();
        center_right_shutdown();
        return 0;
    }
    test_registry_and_cold_state();
    test_interlocks_on_commands();
    test_gravity_extension_cycle();
    test_replay_disable_and_reload();
    test_shutdown_and_reinit();
    center_right_release_controls();
    center_right_shutdown();
    puts("L/G GRVTY adapter: registry, cold & dark, interlocks, handle timing, lever lockout, override ordering, replay, disable, reload and shutdown passed");
    return 0;
}
