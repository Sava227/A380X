/* SPDX-License-Identifier: GPL-3.0-or-later
 * Production thrust-lever adapter (a380x_thrust.inc + a380x_cockpit_lever.inc) against a mock XPLM:
 * registry contract, cockpit drags into X-Plane's handles with sticky detents, the input dataref's
 * hand/lever reporting, click sound (paused / replay / disabled silence, cockpit placement), hardware
 * adoption without write-back, reverse interlocks, FBW FLEX rating to the EWD and the engines
 * (override lease, FCU A/THR hand-over, foreign owner, raw mode), the two instinctive-disconnect
 * buttons (FCU command forwarding, 15 s inhibit, FMGC power-up reset), aircraft reload, disable and
 * shutdown. Linked with the vendored FBW FADEC tables (a380x_ewd_fadec.cpp). No simulator is used.
 *
 *   ./test_thrust_adapter              run the tests
 *   ./test_thrust_adapter --contracts  C commands, D datarefs, R raw datarefs
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include "XPLMUtilities.h"
#include "XPLMSound.h"
#define ARRAY_LEN(a) ((int)(sizeof(a)/sizeof((a)[0])))
#ifndef THRUST_TEST_ROOT
#define THRUST_TEST_ROOT "."
#endif

typedef struct {
    char name[160];
    int registered, type, writable;
    double value;
    float array_f[16];
    int array_i[16];
    XPLMGetDatai_f get_i; XPLMSetDatai_f set_i;
    XPLMGetDataf_f get_f; XPLMSetDataf_f set_f;
    XPLMGetDatad_f get_d; XPLMSetDatad_f set_d;
    void *read_refcon, *write_refcon;
} MockRef;
typedef struct {
    char name[160];
    XPLMCommandCallback_f callback[4];
    void *refcon[4];
    int handlers;
} MockCommand;
static MockRef mock_refs[256];
static MockCommand mock_commands[32];
static int ref_count, command_count, plays;
static FMOD_VECTOR last_position;
static float last_volume;

static MockRef *new_ref(const char *name, int type)
{
    MockRef *r;
    assert(ref_count < ARRAY_LEN(mock_refs));
    r = &mock_refs[ref_count++];
    memset(r, 0, sizeof(*r));
    snprintf(r->name, sizeof(r->name), "%s", name);
    r->registered = 1;
    r->type = type;
    return r;
}
static int sim_type(const char *name)
{
    static const char *const float_arrays[] = {"throttle_ratio", "ENGN_thro_use", "N1_percent"};
    static const char *const int_arrays[] = {"prop_mode", "ENGN_running", "cowling_thermal_anti_ice_per_engine"};
    int i;
    for (i = 0; i < 3; ++i) if (strstr(name, float_arrays[i])) return xplmType_FloatArray;
    for (i = 0; i < 3; ++i) if (strstr(name, int_arrays[i])) return xplmType_IntArray;
    if (strstr(name, "local_") || strstr(name, "pressure_altitude")) return xplmType_Double;
    if (strstr(name, "paused") || strstr(name, "replay") || strstr(name, "onground") || strstr(name, "pack_")
        || strstr(name, "override") || strstr(name, "hot_bleed") || strstr(name, "external")) return xplmType_Int;
    return xplmType_Float;
}
XPLMDataRef XPLMFindDataRef(const char *name)
{
    int i;
    for (i = 0; i < ref_count; ++i)
        if (mock_refs[i].registered && !strcmp(mock_refs[i].name, name)) return &mock_refs[i];
    return !strncmp(name, "sim/", 4) ? new_ref(name, sim_type(name)) : NULL;
}
static MockRef *ref(const char *name) { MockRef *r = XPLMFindDataRef(name); assert(r); return r; }
XPLMDataTypeID XPLMGetDataRefTypes(XPLMDataRef handle) { return ((MockRef *)handle)->type; }
int XPLMGetDatai(XPLMDataRef handle)
{
    MockRef *r = handle;
    assert(r && r->registered);
    return r->get_i ? r->get_i(r->read_refcon) : (int)r->value;
}
float XPLMGetDataf(XPLMDataRef handle)
{
    MockRef *r = handle;
    assert(r && r->registered);
    return r->get_f ? r->get_f(r->read_refcon) : (float)r->value;
}
double XPLMGetDatad(XPLMDataRef handle)
{
    MockRef *r = handle;
    assert(r && r->registered);
    return r->get_d ? r->get_d(r->read_refcon) : r->value;
}
void XPLMSetDatai(XPLMDataRef handle, int value)
{
    MockRef *r = handle;
    assert(r && r->registered);
    if (r->set_i) r->set_i(r->write_refcon, value);
    else { assert(!r->get_i); r->value = value; }
}
void XPLMSetDataf(XPLMDataRef handle, float value)
{
    MockRef *r = handle;
    assert(r && r->registered);
    if (r->set_f) r->set_f(r->write_refcon, value);
    else { assert(!r->get_f); r->value = value; }
}
int XPLMGetDatavf(XPLMDataRef handle, float *values, int offset, int count)
{
    MockRef *r = handle;
    assert(r && r->registered && r->type == xplmType_FloatArray && offset >= 0 && offset + count <= 16);
    memcpy(values, r->array_f + offset, (size_t)count * sizeof(*values));
    return count;
}
void XPLMSetDatavf(XPLMDataRef handle, float *values, int offset, int count)
{
    MockRef *r = handle;
    assert(r && r->registered && r->type == xplmType_FloatArray && offset >= 0 && offset + count <= 16);
    memcpy(r->array_f + offset, values, (size_t)count * sizeof(*values));
}
int XPLMGetDatavi(XPLMDataRef handle, int *values, int offset, int count)
{
    MockRef *r = handle;
    assert(r && r->registered && r->type == xplmType_IntArray && offset >= 0 && offset + count <= 16);
    memcpy(values, r->array_i + offset, (size_t)count * sizeof(*values));
    return count;
}
void XPLMSetDatavi(XPLMDataRef handle, int *values, int offset, int count)
{
    MockRef *r = handle;
    assert(r && r->registered && r->type == xplmType_IntArray && offset >= 0 && offset + count <= 16);
    memcpy(r->array_i + offset, values, (size_t)count * sizeof(*values));
}
XPLMDataRef XPLMRegisterDataAccessor(const char *name, XPLMDataTypeID type, int writable,
    XPLMGetDatai_f read_i, XPLMSetDatai_f write_i, XPLMGetDataf_f read_f, XPLMSetDataf_f write_f,
    XPLMGetDatad_f read_d, XPLMSetDatad_f write_d, XPLMGetDatavi_f read_vi, XPLMSetDatavi_f write_vi,
    XPLMGetDatavf_f read_vf, XPLMSetDatavf_f write_vf, XPLMGetDatab_f read_b, XPLMSetDatab_f write_b,
    void *read_refcon, void *write_refcon)
{
    MockRef *r;
    assert(!read_vi && !write_vi && !read_vf && !write_vf && !read_b && !write_b);
    assert(type == xplmType_Int ? (read_i && !read_f && !read_d)
                                : (type == (xplmType_Float | xplmType_Double) && !read_i && read_f && read_d));
    assert(writable ? (write_i || (write_f && write_d)) : (!write_i && !write_f && !write_d && !write_refcon));
    assert(!XPLMFindDataRef(name));
    r = new_ref(name, type);
    r->writable = writable;
    r->get_i = read_i; r->set_i = write_i; r->get_f = read_f; r->set_f = write_f;
    r->get_d = read_d; r->set_d = write_d;
    r->read_refcon = read_refcon; r->write_refcon = write_refcon;
    return r;
}
void XPLMUnregisterDataAccessor(XPLMDataRef handle)
{
    MockRef *r = handle;
    assert(r && r->registered);
    r->registered = 0;
}
XPLMCommandRef XPLMFindCommand(const char *name)
{
    int i;
    for (i = 0; i < command_count; ++i) if (!strcmp(mock_commands[i].name, name)) return &mock_commands[i];
    return NULL;
}
XPLMCommandRef XPLMCreateCommand(const char *name, const char *description)
{
    MockCommand *c = XPLMFindCommand(name);
    assert(description && *description);
    if (c) return c;
    assert(command_count < ARRAY_LEN(mock_commands));
    c = &mock_commands[command_count++];
    memset(c, 0, sizeof(*c));
    snprintf(c->name, sizeof(c->name), "%s", name);
    return c;
}
void XPLMRegisterCommandHandler(XPLMCommandRef handle, XPLMCommandCallback_f callback, int before, void *refcon)
{
    MockCommand *c = handle;
    assert(c && before && callback && c->handlers < 4);
    c->callback[c->handlers] = callback;
    c->refcon[c->handlers++] = refcon;
}
void XPLMUnregisterCommandHandler(XPLMCommandRef handle, XPLMCommandCallback_f callback, int before, void *refcon)
{
    MockCommand *c = handle;
    int i;
    assert(c && before);
    for (i = 0; i < c->handlers; ++i) if (c->callback[i] == callback && c->refcon[i] == refcon) {
        memmove(c->callback + i, c->callback + i + 1, sizeof(c->callback[0]) * (size_t)(c->handlers - i - 1));
        memmove(c->refcon + i, c->refcon + i + 1, sizeof(c->refcon[0]) * (size_t)(c->handlers - i - 1));
        --c->handlers;
        return;
    }
    assert(!"handler not registered");
}
static void phase(MockCommand *c, XPLMCommandPhase p)
{
    int i;
    for (i = 0; i < c->handlers; ++i) if (!c->callback[i](c, p, c->refcon[i])) break;
}
void XPLMCommandBegin(XPLMCommandRef c) { phase(c, xplm_CommandBegin); }
void XPLMCommandEnd(XPLMCommandRef c) { phase(c, xplm_CommandEnd); }
void XPLMCommandOnce(XPLMCommandRef c) { phase(c, xplm_CommandBegin); phase(c, xplm_CommandEnd); }
XPLMPluginID XPLMGetMyID(void) { return 7; }
void XPLMGetPluginInfo(XPLMPluginID id, char *name, char *path, char *signature, char *description)
{
    (void)id; (void)name; (void)signature; (void)description;
    if (path) snprintf(path, 256, "%s", THRUST_TEST_ROOT "/mac_x64/A380XAvionics.xpl");
}
FMOD_CHANNEL *XPLMPlayPCMOnBus(void *data, uint32_t size, FMOD_SOUND_FORMAT format, int rate, int channels,
                               int loop, XPLMAudioBus bus, XPLMPCMComplete_f callback, void *refcon)
{
    static int channel;
    assert(data && size > 1000 && format == FMOD_SOUND_FORMAT_PCM16 && rate == 44100 && channels == 1);
    assert(!loop && bus == xplm_AudioInterior && !callback && !refcon);
    ++plays;
    return (FMOD_CHANNEL *)&channel;
}
FMOD_RESULT XPLMSetAudioPosition(FMOD_CHANNEL *c, FMOD_VECTOR *position, FMOD_VECTOR *velocity)
{
    (void)c; (void)velocity; last_position = *position; return FMOD_OK;
}
FMOD_RESULT XPLMSetAudioFadeDistance(FMOD_CHANNEL *c, float min, float max) { (void)c; (void)min; (void)max; return FMOD_OK; }
FMOD_RESULT XPLMSetAudioVolume(FMOD_CHANNEL *c, float volume) { (void)c; last_volume = volume; return FMOD_OK; }

/* Minimal unity-build environment of a380x_avionics.c the thrust adapter links to. These mirror
 * the production declarations in a380x_fcu_adapter.inc / a380x_fcu_thrust.inc / a380x_ewd_data.inc. */
static int plugin_enabled = 1;
static struct { int own_throttle; } fcu;
static struct { double idle, cl, mct, toga, tolerance; int calibrated; } fcu_thrust;
static struct { int ready, lease, flex_rating, athr_inhibited, ceiling_valid; double ceiling[4]; } fcu_thrust_link;
typedef struct { int valid, mode; double rating_n1, flex_n1; } EwdRatingSource;
static int (*ewd_rating_source)(EwdRatingSource *out);
#include "a380x_thrust.inc"

/* The FCU's own athr_disconnect handler (registered first, as fcu_init does). */
static int fcu_begin, fcu_end;
static int fcu_disconnect(XPLMCommandRef c, XPLMCommandPhase p, void *refcon)
{
    (void)c; (void)refcon;
    if (p == xplm_CommandBegin) ++fcu_begin;
    if (p == xplm_CommandEnd) ++fcu_end;
    return 1;
}

#define T(name) "a380x/xp12/thrust/" name
static double pub(const char *name) { return XPLMGetDatad(ref(name)); }
static int pub_i(const char *name) { return XPLMGetDatai(ref(name)); }
static void write_input(const char *name, double value) { MockRef *r = ref(name); r->set_d(r->write_refcon, value); }
static float *ratio(void) { return ref("sim/cockpit2/engine/actuators/throttle_ratio")->array_f; }
static int *prop_mode(void) { return ref("sim/cockpit2/engine/actuators/prop_mode")->array_i; }
static float *thro_use(void) { return ref("sim/flightmodel/engine/ENGN_thro_use")->array_f; }
static MockRef *override_ref(void) { return ref("sim/operation/override/override_throttles"); }
static void frame(float dt) { thrust_levers_update(dt); thrust_engines_update(dt); }
static void run(float seconds) { int i, n = (int)lroundf(seconds / 0.05f); for (i = 0; i < n; ++i) frame(0.05f); }
static void drag(const char *name, double from, double to)
{
    double h, s = to >= from ? 0.2 : -0.2;
    for (h = from; s > 0 ? h <= to + 1e-9 : h >= to - 1e-9; h += s) { write_input(name, h); frame(0.02f); }
}

static void setup_sim(void)
{
    MockRef *r;
    int i;
    ref("sim/time/sim_speed_actual")->value = 1.0;
    ref("sim/flightmodel/failures/onground_any")->value = 1;
    ref("sim/weather/aircraft/temperature_ambient_deg_c")->value = 15.0;
    ref("sim/weather/aircraft/barometer_current_pas")->value = 101325.0;
    ref("sim/flightmodel2/position/pressure_altitude")->value = 0.0;
    ref("sim/flightmodel/misc/machno")->value = 0.0;
    ref("sim/cockpit2/temperature/outside_air_LE_temp_degc")->value = 15.0;
    ref("sim/cockpit2/gauges/indicators/airspeed_kts_pilot")->value = 0.0;
    ref("sim/aircraft/view/acf_peX")->value = -0.5304;
    ref("sim/aircraft/view/acf_peY")->value = 3.2004;
    ref("sim/aircraft/view/acf_peZ")->value = 3.0785;
    ref("sim/graphics/view/view_x")->value = -0.53;
    ref("sim/graphics/view/view_y")->value = 3.2;
    ref("sim/graphics/view/view_z")->value = 3.08;
    for (i = 0; i < 4; ++i) {
        ref("sim/flightmodel/engine/ENGN_running")->array_i[i] = 1;
        ref("sim/cockpit2/engine/indicators/N1_percent")->array_f[i] = 20.0f;
        prop_mode()[i] = 1;
    }
    r = new_ref("a380x/xp12/fms/to_flex_temp_c", xplmType_Float);
    r->value = 0.0;
    r = new_ref("a380x/xp12/fms/fmc_a_healthy", xplmType_Int);
    r->value = 1;
    new_ref("a380x/xp12/fms/fmc_b_healthy", xplmType_Int);
    new_ref("a380x/xp12/fms/fmc_c_healthy", xplmType_Int);
    XPLMRegisterCommandHandler(XPLMCreateCommand(THR_FCU_DISCONNECT, "FCU A/THR disconnect"), fcu_disconnect, 1, NULL);
    XPLMCreateCommand("sim/engines/throttle_up", "Throttle up a bit");
    XPLMCreateCommand("sim/engines/throttle_down", "Throttle down a bit");
    fcu_thrust.calibrated = 1;
    fcu_thrust.idle = 0.01; fcu_thrust.cl = 25.0 / 45.0; fcu_thrust.mct = 35.0 / 45.0; fcu_thrust.toga = 1.0;
    fcu_thrust.tolerance = 0.02;
}

static void test_registry(void)
{
    int i;
    for (i = 0; i < THR_IN_COUNT; ++i)
        assert(ref(thr_input_names[i])->type == (xplmType_Float | xplmType_Double) && ref(thr_input_names[i])->writable);
    assert(ref(T("fadec_mode"))->type == xplmType_Int && ref(T("fadec_mode"))->writable);
    for (i = 1; i < THR_FIRST_REAL; ++i) assert(ref(thr_ref_names[i])->type == xplmType_Int && !ref(thr_ref_names[i])->writable);
    for (i = THR_FIRST_REAL; i < THR_R_COUNT; ++i)
        assert(ref(thr_ref_names[i])->type == (xplmType_Float | xplmType_Double) && !ref(thr_ref_names[i])->writable);
    for (i = 0; i < THR_C_COUNT; ++i) assert(XPLMFindCommand(thr_command_names[i]));
    assert(thr.click.data && thr.click.rate == 44100);   /* sounds/thrust_detent_click.wav loaded */
    assert(fcu_thrust_link.ready && ewd_rating_source == thr_ewd_rating && pub_i(T("fadec_mode")) == 1);
}

static void test_cockpit_drag_and_click(void)
{
    int before;
    frame(0.05f);                                   /* first sync: levers at idle, no click */
    assert(plays == 0 && pub(T("lever_1_anim")) == 0.0);
    drag(T("lever_1_input"), 0.0, 24.4);
    assert(pub(T("tla_1")) == 25.0 && fabsf(ratio()[0] - (float)(25.0 / 45.0)) < 1e-6f);
    assert(pub(T("lever_1_anim")) == 55.0 && plays == 1 && last_volume == THR_CLICK_VOLUME);
    assert(fabs(last_position.y - (3.2004 - 0.6932)) < 1e-3 && fabs(last_position.x - (-0.5304 + 0.4923)) < 1e-3);
    write_input(T("lever_1_input"), 27.5); frame(0.02f);
    assert(pub(T("tla_1")) == 25.0 && pub(T("lever_1_input")) == 27.5);   /* sticky; the input shows the hand */
    run(0.3f);
    assert(pub(T("lever_1_input")) == 25.0);       /* drag over: the input shows the lever again */
    assert(ratio()[1] == 0.0f && plays == 1);      /* other levers untouched */
    /* Paused: the lever still moves, silently. */
    ref("sim/time/paused")->value = 1;
    before = plays;
    drag(T("lever_1_input"), 25.0, 35.0);
    assert(pub(T("tla_1")) == 35.0 && plays == before);
    ref("sim/time/paused")->value = 0;
    drag(T("lever_1_input"), 35.0, 0.0);
    assert(pub(T("tla_1")) == 0.0 && ratio()[0] == 0.0f && plays == before + 2);   /* CL and IDLE */
    run(0.3f);
}

static void test_hardware_adoption(void)
{
    int i, before = plays;
    for (i = 0; i < 4; ++i) ratio()[i] = 0.565f;   /* a hardware axis near CL */
    frame(0.05f);
    for (i = 0; i < 4; ++i) assert(pub(i == 0 ? T("tla_1") : i == 1 ? T("tla_2") : i == 2 ? T("tla_3") : T("tla_4")) == 25.0);
    assert(ratio()[2] == 0.565f);                  /* never written back: the axis is not fought */
    assert(plays == before + 1);                   /* four levers into CL together: one click */
    for (i = 0; i < 4; ++i) ratio()[i] = 0.0f;
    frame(0.05f);
    run(0.1f);
}

/* Keyboard throttle steps stop in CL while the key is held; the detent commands set all levers. */
static void test_keyboard_detents(void)
{
    MockCommand *up = (MockCommand *)XPLMFindCommand("sim/engines/throttle_up");
    const float cl = (float)(25.0 / 45.0);
    int i, step;
    assert(up && up->handlers == 1 && XPLMFindCommand("sim/engines/throttle_down"));
    for (i = 0; i < 4; ++i) ratio()[i] = 0.40f;
    frame(0.05f);                                   /* adopted below CL, no key */
    XPLMCommandBegin(up);
    for (step = 0; step < 10; ++step) {             /* X-Plane moves its handle while the key is held */
        for (i = 0; i < 4; ++i) ratio()[i] += 0.03f;
        frame(0.05f);
    }
    for (i = 0; i < 4; ++i) assert(fabsf(ratio()[i] - cl) < 1e-6f);   /* stopped in CL, written back */
    assert(pub(T("tla_1")) == 25.0 && pub(T("tla_4")) == 25.0);
    XPLMCommandEnd(up);
    run(0.2f);
    XPLMCommandBegin(up);
    for (i = 0; i < 4; ++i) ratio()[i] += 0.03f;    /* the next press leaves CL */
    frame(0.05f);
    XPLMCommandEnd(up);
    run(0.2f);
    assert(pub(T("tla_1")) > 25.5 && pub(T("tla_1")) < 35.0);
    XPLMCommandOnce(XPLMFindCommand(T("levers_cl"))); frame(0.05f);
    for (i = 0; i < 4; ++i) assert(fabsf(ratio()[i] - cl) < 1e-6f);
    XPLMCommandOnce(XPLMFindCommand(T("levers_detent_up"))); frame(0.05f);
    assert(pub(T("tla_2")) == 35.0);
    XPLMCommandOnce(XPLMFindCommand(T("levers_detent_down"))); frame(0.05f);
    assert(pub(T("tla_3")) == 25.0);
    XPLMCommandOnce(XPLMFindCommand(T("levers_toga"))); frame(0.05f);
    assert(pub(T("tla_1")) == 45.0 && ratio()[0] == 1.0f);
    XPLMCommandOnce(XPLMFindCommand(T("levers_idle"))); frame(0.05f);
    for (i = 0; i < 4; ++i) assert(ratio()[i] == 0.0f && pub(T("tla_1")) == 0.0);   /* IDLE writes a true 0 */
    for (i = 0; i < 4; ++i) ratio()[i] = 0.0f;
    frame(0.05f);
    run(0.1f);
}

static void test_reverse(void)
{
    int before;
    drag(T("reverse_2_input"), 0.0, 1.0);
    assert(prop_mode()[1] == 3 && ratio()[1] == 1.0f && pub(T("tla_2")) == -20.0 && pub(T("reverse_2_anim")) == 1.0);
    write_input(T("lever_2_input"), 20.0); frame(0.02f);
    assert(pub(T("tla_2")) == -20.0);             /* thrust lever locked while reversed */
    run(0.3f);
    drag(T("reverse_2_input"), 1.0, 0.0);
    assert(prop_mode()[1] == 1 && ratio()[1] == 0.0f && pub(T("reverse_2_anim")) == 0.0);
    run(0.3f);
    /* X-Plane's reverse toggle on an outer engine is refused and restored. */
    prop_mode()[0] = 3;
    frame(0.05f);
    assert(prop_mode()[0] == 1);
    /* Reverse lever refused with the thrust lever forward. */
    drag(T("lever_3_input"), 0.0, 10.0);
    run(0.3f);
    before = prop_mode()[2];
    drag(T("reverse_3_input"), 0.0, 1.0);
    assert(prop_mode()[2] == before && pub(T("reverse_3_anim")) == 0.0);
    run(0.3f);
    drag(T("lever_3_input"), 10.0, 0.0);
    run(0.3f);
}

static void test_flex_rating_and_engines(void)
{
    EwdRatingSource source;
    double flex_n1, toga_n1, idle_n1, flex_low;
    A380xEwdFadecLimits limits;
    int i;
    assert(a380x_ewd_fadec_limits(15.0, 1013.25, 0.0, 0.0, 0, 0, 0, &limits));
    assert(a380x_ewd_fadec_flex_n1(15.0, 1013.25, 0.0, 0.0, 0, 0, 0, 60.0, &flex_n1));
    assert(a380x_ewd_fadec_flex_n1(15.0, 1013.25, 0.0, 0.0, 0, 0, 0, 45.0, &flex_low));
    assert(flex_n1 < flex_low && flex_low < limits.toga_n1 + 1e-9);          /* higher FLEX temp: less N1 */
    assert(!a380x_ewd_fadec_flex_n1(15.0, 1013.25, 0.0, 0.0, 0, 0, 0, -5.0, &flex_low) && flex_low == 0.0);
    assert(override_ref()->value == 1 && pub_i(T("engine_override")) == 1 && fcu_thrust_link.lease);
    ref("a380x/xp12/fms/to_flex_temp_c")->value = 60.0;
    for (i = 0; i < 4; ++i) {
        static const char *const inputs[] = {T("lever_1_input"), T("lever_2_input"), T("lever_3_input"), T("lever_4_input")};
        drag(inputs[i], 0.0, 34.4);
    }
    run(0.3f);
    assert(pub_i(T("thrust_limit_type")) == A380X_LIMIT_FLX && pub_i(T("flex_rating")) && pub_i(T("flex_latch")));
    assert(fcu_thrust_link.flex_rating && fabs(pub(T("flex_temp_c")) - 60.0) < 1e-9);
    assert(ewd_rating_source(&source) && source.mode == A380X_LIMIT_FLX && fabs(source.rating_n1 - flex_n1) < 1e-6);
    toga_n1 = limits.toga_n1; idle_n1 = limits.idle_n1;
    /* IAS 0 kt: the FBW low-speed cap (76.5 %) applies first. */
    assert(fabs(pub(T("n1_command_1")) - fmin(flex_n1, 76.5)) < 1e-6);
    ref("sim/cockpit2/gauges/indicators/airspeed_kts_pilot")->value = 80.0;
    run(0.1f);
    assert(fabs(pub(T("n1_command_1")) - flex_n1) < 1e-6);
    assert(fabsf(thro_use()[0] - (float)((flex_n1 - idle_n1) / (toga_n1 - idle_n1))) < 1e-5f && thro_use()[0] < 1.0f);
    assert(fcu_thrust_link.ceiling_valid && fabs(fcu_thrust_link.ceiling[0] - (flex_n1 - idle_n1) / (toga_n1 - idle_n1)) < 1e-6);
    /* TAT above the FLEX temperature: FLX/MCT gives MCT (TOGA rating on the ground, as FBW). */
    ref("sim/cockpit2/temperature/outside_air_LE_temp_degc")->value = 61.0;
    run(0.1f);
    assert(pub_i(T("thrust_limit_type")) == A380X_LIMIT_TOGA && fabs(pub(T("n1_command_1")) - limits.mct_n1) < 1e-6);
    ref("sim/cockpit2/temperature/outside_air_LE_temp_degc")->value = 15.0;
    run(0.1f);
    /* A/THR becomes active (FCU own_throttle): the lease is handed over without touching the override. */
    fcu.own_throttle = 1;
    thro_use()[0] = 0.3f;
    frame(0.05f);
    assert(!fcu_thrust_link.lease && override_ref()->value == 1 && thro_use()[0] == 0.3f);
    /* A/THR releases (the FCU clears the override): the FADEC takes the engines again. */
    fcu.own_throttle = 0; override_ref()->value = 0;
    frame(0.05f);
    assert(override_ref()->value == 1 && fcu_thrust_link.lease && thro_use()[0] < 1.0f && thro_use()[0] > 0.5f);
    /* Raw mode: X-Plane's handles drive the engines directly. */
    XPLMSetDatai(ref(T("fadec_mode")), 0);
    frame(0.05f);
    assert(override_ref()->value == 0 && !fcu_thrust_link.lease && !fcu_thrust_link.ceiling_valid);
    /* Another controller owns the throttles: not taken over. */
    override_ref()->value = 1;
    XPLMSetDatai(ref(T("fadec_mode")), 1);
    frame(0.05f);
    assert(!fcu_thrust_link.lease && pub_i(T("engine_override")) == 0);
    override_ref()->value = 0;
    frame(0.05f);
    assert(fcu_thrust_link.lease);
    /* Engines at take-off power for the legacy flight phase: FLX detent counts. */
    assert(thrust_takeoff_power(0) && thrust_takeoff_power(3) && !thrust_takeoff_power(4));
}

static void test_instinctive_disconnect(void)
{
    MockCommand *left = (MockCommand *)XPLMFindCommand(T("athr_disconnect_left"));
    MockCommand *right = (MockCommand *)XPLMFindCommand(T("athr_disconnect_right"));
    int begin = fcu_begin, end = fcu_end;
    XPLMCommandBegin(left);
    assert(fcu_begin == begin + 1 && thr.disc_held == 1);
    run(0.1f);
    assert(pub(T("disc_left_anim")) == 1.0 && pub(T("disc_right_anim")) == 0.0);
    ref("sim/time/paused")->value = 1;
    run(5.0f);                                     /* paused: no simulated time */
    ref("sim/time/paused")->value = 0;
    run(14.8f);
    assert(!pub_i(T("athr_disabled")) && pub(T("athr_disc_held_s")) < 15.0);
    run(0.3f);
    assert(pub_i(T("athr_disabled")) && fcu_thrust_link.athr_inhibited);
    XPLMCommandEnd(left);
    assert(fcu_end == end + 1 && thr.disc_held == 0);
    run(0.2f);
    assert(pub(T("disc_left_anim")) == 0.0 && pub_i(T("athr_disabled")));
    XPLMCommandOnce(right);                        /* a short press: no hold */
    assert(fcu_begin == begin + 2 && fcu_end == end + 2);
    /* FMGC power-up on the ground (FMC healthy 0 -> 1) re-enables A/THR. */
    ref("a380x/xp12/fms/fmc_a_healthy")->value = 0;
    run(0.1f);
    ref("a380x/xp12/fms/fmc_a_healthy")->value = 1;
    run(0.1f);
    assert(!pub_i(T("athr_disabled")) && !fcu_thrust_link.athr_inhibited);
    /* Disable while a button is held: the FCU command is ended, the override released. */
    XPLMCommandBegin(right);
    plugin_enabled = 0;
    thrust_release_controls();
    assert(fcu_end == end + 3 && override_ref()->value == 0 && !fcu_thrust_link.lease);
    XPLMCommandEnd(right);
    assert(fcu_end == end + 3);                    /* not ended twice */
    write_input(T("lever_1_input"), 45.0);         /* disabled: manipulator writes ignored */
    plugin_enabled = 1;
    frame(0.05f);
    assert(pub(T("tla_1")) == 35.0);
}

static void test_reload_and_shutdown(void)
{
    int i;
    thrust_plane_loaded();
    assert(override_ref()->value == 0 && pub(T("tla_1")) == 0.0 && !pub_i(T("athr_disabled")));
    frame(0.05f);                                  /* re-sync from X-Plane's handles (still at FLX) */
    assert(pub(T("tla_1")) == 35.0 && fcu_thrust_link.lease);
    thrust_shutdown();
    assert(override_ref()->value == 0 && !fcu_thrust_link.ready && !ewd_rating_source);
    for (i = 0; i < THR_IN_COUNT; ++i) assert(!XPLMFindDataRef(thr_input_names[i]));
    for (i = 0; i < THR_R_COUNT; ++i) assert(!XPLMFindDataRef(thr_ref_names[i]));
    for (i = 0; i < THR_C_COUNT; ++i) assert(((MockCommand *)XPLMFindCommand(thr_command_names[i]))->handlers == 0);
    assert(((MockCommand *)XPLMFindCommand(THR_FCU_DISCONNECT))->handlers == 1);   /* only the FCU's own */
    assert(thrust_init());                         /* reload: persistent commands reused */
    frame(0.05f);
    assert(pub(T("tla_1")) == 35.0);
    thrust_shutdown();
}

int main(int argc, char **argv)
{
    int i;
    setup_sim();
    assert(thrust_init());
    if (argc > 1 && !strcmp(argv[1], "--contracts")) {
        for (i = 0; i < THR_C_COUNT; ++i) printf("C %s\n", thr_command_names[i]);
        for (i = 0; i < THR_IN_COUNT; ++i) printf("D %s\n", thr_input_names[i]);
        for (i = 0; i < THR_R_COUNT; ++i) printf("D %s\n", thr_ref_names[i]);
        for (i = 0; i < THR_RAW_COUNT; ++i) printf("R %s\n", thr_raw_names[i]);
        for (i = 0; i < CS_RAW_COUNT; ++i) printf("R %s\n", cockpit_sound_raw_names[i]);
        thrust_shutdown();
        return 0;
    }
    test_registry();
    test_cockpit_drag_and_click();
    test_hardware_adoption();
    test_keyboard_detents();
    test_reverse();
    test_flex_rating_and_engines();
    test_instinctive_disconnect();
    test_reload_and_shutdown();
    puts("Thrust-lever adapter: registry, sticky cockpit drags, click sound, hardware adoption, reverse interlocks, "
         "keyboard detents, FLEX rating and engine lease, instinctive disconnect, reload and shutdown passed");
    return 0;
}
