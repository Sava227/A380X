/* Link the real plugin translation unit to these SDK mocks, not a fake header.
 * No simulator is launched. Unmocked SDK calls fail the test executable link: in particular no
 * XPLMSetData* is provided, so the plugin cannot write any simulator (flight-control) dataref.
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

int XPluginStart(char *, char *, char *);
void XPluginStop(void);
int XPluginEnable(void);
void XPluginDisable(void);
void XPluginReceiveMessage(XPLMPluginID, int, void *);

enum { REF_COUNT = 22, RATIO_COUNT = 14, OWNED = 23, FLAG = 22, SIM_COUNT = 16 };
static const char *names[OWNED] = {
    "a380x/xp12/visual/aileron_l_inboard_ratio",
    "a380x/xp12/visual/aileron_l_midboard_ratio",
    "a380x/xp12/visual/aileron_l_outboard_ratio",
    "a380x/xp12/visual/aileron_r_inboard_ratio",
    "a380x/xp12/visual/aileron_r_midboard_ratio",
    "a380x/xp12/visual/aileron_r_outboard_ratio",
    "a380x/xp12/hydraulics/aileron_l_inboard_ratio",
    "a380x/xp12/hydraulics/aileron_l_midboard_ratio",
    "a380x/xp12/hydraulics/aileron_l_outboard_ratio",
    "a380x/xp12/hydraulics/aileron_r_inboard_ratio",
    "a380x/xp12/hydraulics/aileron_r_midboard_ratio",
    "a380x/xp12/hydraulics/aileron_r_outboard_ratio",
    "a380x/xp12/hydraulics/elevator_ratio",
    "a380x/xp12/hydraulics/rudder_ratio",
    "a380x/xp12/hydraulics/spoiler_l_inboard_deg",
    "a380x/xp12/hydraulics/spoiler_r_inboard_deg",
    "a380x/xp12/hydraulics/spoiler_l_inner_deg",
    "a380x/xp12/hydraulics/spoiler_r_inner_deg",
    "a380x/xp12/hydraulics/spoiler_l_middle_deg",
    "a380x/xp12/hydraulics/spoiler_r_middle_deg",
    "a380x/xp12/hydraulics/spoiler_l_outer_deg",
    "a380x/xp12/hydraulics/spoiler_r_outer_deg",
    "a380x/xp12/hydraulics/native_driver",
};
/* The only simulator datarefs the plugin may look up (all read only). */
static const char *sim_names[SIM_COUNT] = {
    "sim/flightmodel2/controls/roll_ratio", "sim/flightmodel2/controls/pitch_ratio",
    "sim/flightmodel2/controls/heading_ratio", "sim/flightmodel/failures/onground_any", "sim/time/paused",
    "sim/operation/misc/frame_rate_period", "sim/flightmodel/position/psi",
    "sim/weather/aircraft/wind_now_direction_degt", "sim/weather/aircraft/wind_now_speed_msc",
    "sim/flightmodel/position/true_airspeed", "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1",
    "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_2",
    "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_3",
    "sim/flightmodel2/wing/speedbrake1_deg", "sim/flightmodel2/wing/spoiler1_deg",
    "sim/flightmodel2/wing/spoiler2_deg"};
enum { S_ROLL, S_PITCH, S_YAW, S_GROUND, S_PAUSED, S_DT, S_PSI, S_WDIR, S_WSPD, S_TAS, S_P1, S_P2, S_P3,
       S_SB1, S_SP1, S_SP2 };
typedef struct {
    int active;
    XPLMDataTypeID type;
    int writable;
    XPLMGetDataf_f read;
    XPLMSetDataf_f write;
    XPLMGetDatai_f read_int;
    void *read_refcon;
    void *write_refcon;
} MockRef;
static MockRef registered[OWNED];
static int find_calls, register_calls, unregister_calls, debug_calls, sim_find_calls;
static int collision_index = -1, fail_index = -1, missing_sim = -1, wrong_type_sim = -1;
static char name[256], signature[256], description[256];
static int foreign_handle;
static int sim_handle[SIM_COUNT];
static float sim_value[SIM_COUNT];
static float sim_arrays[3][48];
static XPLMFlightLoop_f loop_fn;
static int loop_alive, loop_creates, loop_schedules;
static char last_debug[512];

XPLMDataRef XPLMFindDataRef(const char *data_name)
{
    int i;
    for (i = 0; i < SIM_COUNT; ++i) {
        if (strcmp(data_name, sim_names[i]) == 0) {
            ++sim_find_calls;
            return i == missing_sim ? NULL : &sim_handle[i];
        }
    }
    ++find_calls;
    for (i = 0; i < OWNED; ++i) {
        if (strcmp(data_name, names[i]) == 0) {
            if (i == collision_index)
                return &foreign_handle;
            return registered[i].active ? &registered[i] : NULL;
        }
    }
    assert(!"plugin looked up an unexpected dataref");
    return NULL;
}

static int sim_of(XPLMDataRef ref)
{
    int i;
    for (i = 0; i < SIM_COUNT; ++i) if (ref == &sim_handle[i]) return i;
    assert(!"read of a dataref that is not a simulator input");
    return -1;
}

XPLMDataTypeID XPLMGetDataRefTypes(XPLMDataRef ref)
{
    int i = sim_of(ref);
    XPLMDataTypeID type = i >= S_SB1 ? xplmType_FloatArray : (i == S_GROUND || i == S_PAUSED) ? xplmType_Int
                          : (i == S_PSI ? (xplmType_Float | xplmType_Double) : xplmType_Float);
    return i == wrong_type_sim ? xplmType_Data : type;
}

float XPLMGetDataf(XPLMDataRef ref)
{
    int i = sim_of(ref);
    assert(i != S_GROUND && i != S_PAUSED && i < S_SB1);
    return sim_value[i];
}

int XPLMGetDatai(XPLMDataRef ref)
{
    int i = sim_of(ref);
    assert(i == S_GROUND || i == S_PAUSED);
    return (int)sim_value[i];
}

int XPLMGetDatavf(XPLMDataRef ref, float *values, int offset, int count)
{
    int i = sim_of(ref) - S_SB1;
    assert(i >= 0 && values != NULL && offset >= 0 && count > 0 && offset + count <= 48);
    memcpy(values, &sim_arrays[i][offset], sizeof(float) * (size_t)count);
    return count;
}

XPLMDataRef XPLMRegisterDataAccessor(
    const char *data_name, XPLMDataTypeID type, int writable,
    XPLMGetDatai_f read_int, XPLMSetDatai_f write_int,
    XPLMGetDataf_f read_float, XPLMSetDataf_f write_float,
    XPLMGetDatad_f read_double, XPLMSetDatad_f write_double,
    XPLMGetDatavi_f read_ints, XPLMSetDatavi_f write_ints,
    XPLMGetDatavf_f read_floats, XPLMSetDatavf_f write_floats,
    XPLMGetDatab_f read_bytes, XPLMSetDatab_f write_bytes,
    void *read_refcon, void *write_refcon)
{
    int i = register_calls++;
    assert(find_calls == OWNED); /* All collision checks preceded first register. */
    assert(sim_find_calls == 0); /* Registration happens before any simulator lookup. */
    assert(i < OWNED && strcmp(data_name, names[i]) == 0);
    assert(read_double == NULL && write_double == NULL);
    assert(read_ints == NULL && write_ints == NULL);
    assert(read_floats == NULL && write_floats == NULL);
    assert(read_bytes == NULL && write_bytes == NULL);
    if (i == FLAG) {
        /* The native-driver flag: a read-only Int. */
        assert(type == xplmType_Int && writable == 0 && read_int != NULL && write_int == NULL);
        assert(read_float == NULL && write_float == NULL);
    } else {
        assert(type == xplmType_Float && writable == 1);
        assert(read_float != NULL && write_float != NULL);
        assert(read_refcon != NULL && write_refcon == read_refcon);
        assert(read_int == NULL && write_int == NULL);
    }
    if (i == fail_index)
        return NULL;
    registered[i] = (MockRef){1, type, writable, read_float, write_float, read_int, read_refcon, write_refcon};
    if (i == FLAG) assert(read_int(read_refcon) == 0);
    else assert(read_float(read_refcon) == 0.0f);
    return &registered[i];
}

void XPLMUnregisterDataAccessor(XPLMDataRef ref)
{
    int i;
    /* Reject unregistering a foreign, failed or already-unregistered ref. */
    for (i = 0; i < OWNED; ++i) {
        if (ref == &registered[i]) {
            assert(registered[i].active);
            registered[i].active = 0;
            ++unregister_calls;
            return;
        }
    }
    assert(!"unregistered a dataref not owned by this plugin");
}

void XPLMDebugString(const char *message)
{
    assert(message != NULL);
    ++debug_calls;
    snprintf(last_debug, sizeof(last_debug), "%s", message);
}

XPLMFlightLoopID XPLMCreateFlightLoop(XPLMCreateFlightLoop_t *params)
{
    assert(params != NULL && params->structSize == (int)sizeof(XPLMCreateFlightLoop_t));
    assert(params->phase == xplm_FlightLoop_Phase_AfterFlightModel && params->callbackFunc != NULL);
    assert(!loop_alive);
    loop_fn = params->callbackFunc;
    loop_alive = 1;
    ++loop_creates;
    return (XPLMFlightLoopID)&loop_alive;
}

void XPLMDestroyFlightLoop(XPLMFlightLoopID id)
{
    assert(id == (XPLMFlightLoopID)&loop_alive && loop_alive);
    loop_alive = 0;
}

void XPLMScheduleFlightLoop(XPLMFlightLoopID id, float interval, int relative)
{
    assert(id == (XPLMFlightLoopID)&loop_alive && loop_alive && interval == -1.0f && relative == 1);
    ++loop_schedules;
}

static void set_sim_defaults(void)
{
    memset(sim_value, 0, sizeof(sim_value));
    memset(sim_arrays, 0, sizeof(sim_arrays));
    sim_value[S_GROUND] = 1;
    sim_value[S_DT] = 1.0f / 60.0f;
    sim_value[S_P1] = sim_value[S_P2] = 3000;
}

static void reset_mock(void)
{
    int i;
    XPluginStop();
    for (i = 0; i < OWNED; ++i)
        assert(!registered[i].active);
    assert(!loop_alive);
    memset(registered, 0, sizeof(registered));
    find_calls = register_calls = unregister_calls = debug_calls = sim_find_calls = 0;
    loop_creates = loop_schedules = 0;
    collision_index = fail_index = missing_sim = wrong_type_sim = -1;
    memset(name, 0, sizeof(name));
    memset(signature, 0, sizeof(signature));
    memset(description, 0, sizeof(description));
    set_sim_defaults();
}

static void start(void)
{
    assert(XPluginStart(name, signature, description) == 1);
    assert(register_calls == OWNED && find_calls == OWNED && sim_find_calls == 0);
}

static float get(int i)
{
    assert(i < REF_COUNT && registered[i].active);
    return registered[i].read(registered[i].read_refcon);
}

static void set(int i, float value)
{
    assert(i < REF_COUNT && registered[i].active);
    registered[i].write(registered[i].write_refcon, value);
}

static int flag(void)
{
    assert(registered[FLAG].active);
    return registered[FLAG].read_int(registered[FLAG].read_refcon);
}

static void frames(int count)
{
    int i;
    assert(loop_alive);
    for (i = 0; i < count; ++i) assert(loop_fn(sim_value[S_DT], sim_value[S_DT], i, NULL) == -1.0f);
}

static void test_early_registration(void)
{
    int i;
    reset_mock();
    assert(XPluginEnable() == 0 && find_calls == 0 && register_calls == 0 && !loop_alive);
    start();
    /* Simulate attached OBJ resolution immediately after Start, before Enable. */
    for (i = 0; i < REF_COUNT; ++i) {
        assert(XPLMFindDataRef(names[i]) == &registered[i]);
        assert(get(i) == 0.0f);
    }
    assert(flag() == 0 && !loop_alive);
    assert(XPluginEnable() == 1 && register_calls == OWNED);
    assert(strcmp(signature, "org.a380x.xp12.aileron-visual-datarefs.v1") == 0);
    assert(strlen(name) < 256 && strlen(description) < 256);
}

static void test_independent_cached_values(void)
{
    int i;
    reset_mock(); start();
    for (i = 0; i < REF_COUNT; ++i)
        set(i, (float)(i + 1) / (float)(REF_COUNT + 1));
    for (i = 0; i < REF_COUNT; ++i)
        assert(get(i) == (float)(i + 1) / (float)(REF_COUNT + 1));
    assert(find_calls == OWNED && register_calls == OWNED && unregister_calls == 0);
}

static void test_clamping(void)
{
    int i;
    reset_mock(); start();
    for (i = 0; i < REF_COUNT; ++i) {
        float minimum = i < RATIO_COUNT ? -1.0f : 0.0f;
        float maximum = i < RATIO_COUNT ? 1.0f : 65.0f;
        set(i, -100.0f); assert(get(i) == minimum);
        set(i, 100.0f); assert(get(i) == maximum);
        set(i, minimum); assert(get(i) == minimum);
        set(i, maximum); assert(get(i) == maximum);
        if (i >= RATIO_COUNT) {
            set(i, 32.5f); assert(get(i) == 32.5f);
        }
        set(i, 0.125f); assert(get(i) == 0.125f);
    }
}

static void test_nonfinite_rejected(void)
{
    int i;
    reset_mock(); start();
    for (i = 0; i < REF_COUNT; ++i) {
        set(i, 0.25f);
        set(i, NAN); assert(get(i) == 0.25f);
        set(i, INFINITY); assert(get(i) == 0.25f);
        set(i, -INFINITY); assert(get(i) == 0.25f);
    }
}

static void test_every_collision_prevents_registration(void)
{
    int i;
    for (i = 0; i < OWNED; ++i) {
        reset_mock(); collision_index = i;
        assert(XPluginStart(name, signature, description) == 0);
        assert(find_calls == OWNED && register_calls == 0 && unregister_calls == 0);
        assert(XPluginEnable() == 0 && !loop_alive);
        XPluginStop(); assert(unregister_calls == 0);
    }
}

static void test_every_partial_failure_rolls_back(void)
{
    int i, j;
    for (i = 0; i < OWNED; ++i) {
        reset_mock(); fail_index = i;
        assert(XPluginStart(name, signature, description) == 0);
        assert(register_calls == i + 1 && unregister_calls == i);
        for (j = 0; j < OWNED; ++j)
            assert(!registered[j].active);
        assert(XPluginEnable() == 0 && !loop_alive);
        XPluginStop(); assert(unregister_calls == i);
    }
}

static void test_stop_is_idempotent(void)
{
    reset_mock(); start(); assert(XPluginEnable() == 1 && loop_alive);
    XPluginStop(); assert(unregister_calls == OWNED && XPluginEnable() == 0 && !loop_alive);
    XPluginStop(); assert(unregister_calls == OWNED);
}

static void test_disable_keeps_refs_neutral(void)
{
    int i;
    reset_mock(); start(); assert(XPluginEnable() == 1);
    sim_value[S_ROLL] = 1; frames(120);
    for (i = 0; i < REF_COUNT; ++i)
        set(i, 0.75f);
    XPluginDisable();
    for (i = 0; i < REF_COUNT; ++i)
        assert(get(i) == 0.0f);
    assert(flag() == 0 && !loop_alive);
    assert(register_calls == OWNED && unregister_calls == 0 && XPluginEnable() == 1 && flag() == 1);
}

static void test_duplicate_start_does_not_touch_owned_refs(void)
{
    reset_mock(); start(); set(0, 0.5f);
    assert(XPluginStart(name, signature, description) == 0);
    assert(register_calls == OWNED && find_calls == OWNED && unregister_calls == 0);
    assert(get(0) == 0.5f);
}

static void test_restart_and_messages(void)
{
    reset_mock(); start(); set(3, 0.75f);
    XPluginReceiveMessage(0, 102, NULL);
    assert(get(3) == 0.75f && register_calls == OWNED && find_calls == OWNED);
    reset_mock(); start();
    assert(get(3) == 0.0f);
}

/* ---- the native driver ---- */
static void test_native_driver_flag_and_flight_loop(void)
{
    reset_mock(); start();
    assert(flag() == 0);
    assert(XPluginEnable() == 1);
    assert(flag() == 1 && loop_alive && loop_creates == 1 && loop_schedules == 1 && sim_find_calls == SIM_COUNT);
    XPluginDisable(); assert(flag() == 0 && !loop_alive);
    assert(XPluginEnable() == 1 && flag() == 1 && loop_creates == 2);
    XPluginStop(); assert(!loop_alive);
}

static void test_missing_or_mistyped_input_keeps_driver_off(void)
{
    int i;
    for (i = 0; i < SIM_COUNT; ++i) {
        reset_mock(); start(); missing_sim = i;
        assert(XPluginEnable() == 1 && flag() == 0 && !loop_alive);
        reset_mock(); start(); wrong_type_sim = i;
        assert(XPluginEnable() == 1 && flag() == 0 && !loop_alive);
    }
}

static void test_surfaces_follow_controls(void)
{
    reset_mock(); start(); assert(XPluginEnable() == 1);
    frames(1);   /* script load */
    sim_value[S_ROLL] = 1.0f;    /* right roll: left aileron trailing edge up, right down */
    sim_value[S_PITCH] = -1.0f;
    sim_value[S_YAW] = 0.5f;
    sim_arrays[0][0] = sim_arrays[0][1] = 40.0f;   /* speed brakes */
    sim_arrays[2][5] = 80.0f;                      /* beyond the 65 degree visual limit */
    frames(240);
    /* On the ground (no PRIM delay): -30 deg command; left trailing edge up to its 20 deg stop, right
     * at its -30 deg stop (ratio = side * degrees / 30, a380x_aileron_dance.lua). */
    assert(fabsf(get(0) - 20.0f / 30.0f) < 1e-6f && fabsf(get(3) - 1.0f) < 1e-6f);
    assert(fabsf(get(6) - get(0)) < 1e-6f && fabsf(get(9) - get(3)) < 1e-6f);   /* hydraulics follow */
    assert(get(12) == -1.0f && get(13) == 0.5f);
    assert(get(14) == 40.0f && get(15) == 40.0f && get(21) == 65.0f);
    assert(strstr(last_debug, "first nonzero aileron publication") || debug_calls > 0);
}

static void test_pause_holds_and_reload_reseeds(void)
{
    float before[REF_COUNT];
    int i;
    reset_mock(); start(); assert(XPluginEnable() == 1);
    sim_value[S_ROLL] = 0.5f; frames(120);
    for (i = 0; i < REF_COUNT; ++i) before[i] = get(i);
    sim_value[S_PAUSED] = 1; sim_value[S_ROLL] = -1.0f; frames(200);
    for (i = 0; i < REF_COUNT; ++i) assert(get(i) == before[i]);
    sim_value[S_PAUSED] = 0;
    /* A user-aircraft reload seeds the cores from the current input on the next frame (no sweep). */
    XPluginReceiveMessage(0, XPLM_MSG_PLANE_LOADED, (void *)(intptr_t)0);
    frames(1);
    assert(fabsf(get(0) + 1.0f) < 1e-6f || fabsf(get(0) - before[0]) > 0.1f);
    /* Another aircraft's load message (index 1) is ignored. */
    XPluginReceiveMessage(0, XPLM_MSG_PLANE_LOADED, (void *)(intptr_t)1);
    frames(1);
    assert(flag() == 1);
}

static void test_cold_ground_droop_and_no_unpowered_spoiler(void)
{
    reset_mock(); start(); assert(XPluginEnable() == 1);
    sim_value[S_P1] = sim_value[S_P2] = 0;
    sim_arrays[0][0] = 50.0f;
    frames(1);
    assert(get(14) == 0.0f);   /* a cold load starts neutral */
    frames(60 * 120);
    assert(get(14) == 0.0f && get(6) > 0.6f && get(9) < -0.6f && get(12) < -0.6f);
}

int main(void)
{
    test_early_registration();
    test_independent_cached_values();
    test_clamping();
    test_nonfinite_rejected();
    test_every_collision_prevents_registration();
    test_every_partial_failure_rolls_back();
    test_stop_is_idempotent();
    test_disable_keeps_refs_neutral();
    test_duplicate_start_does_not_touch_owned_refs();
    test_restart_and_messages();
    test_native_driver_flag_and_flight_loop();
    test_missing_or_mistyped_input_keeps_driver_off();
    test_surfaces_follow_controls();
    test_pause_holds_and_reload_reseeds();
    test_cold_ground_droop_and_no_unpowered_spoiler();
    XPluginStop();
    puts("15 tests passed (all 23 collision and rollback positions, both ranges, native-driver flag, "
         "flight loop, missing/mistyped inputs, motion, pause, reload, cold droop; no simulator writes linked).");
    return 0;
}
