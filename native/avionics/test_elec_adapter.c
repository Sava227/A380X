/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 electrical network adapter (a380x_elec_adapter.inc) with the production ELEC panel
 * adapter against a mock XPLM (spec 11.2): registry contract, spawn decision, the manual
 * cold & dark power-up, stock slaving (battery / charge pin / GPU / generators / avionics),
 * hardware command interception, failure mapping to FAULT lamps, pause, APU ECB, presets,
 * reload and shutdown.
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMSound.h"
#include "XPLMUtilities.h"
#include "a380x_adirs.h"
#ifndef ELEC_TEST_ROOT
#define ELEC_TEST_ROOT "."
#endif

typedef struct {
    char name[128];
    int registered, writable, type, vi[16];
    double value;
    float vf[16];
    XPLMGetDatai_f get_i; XPLMSetDatai_f set_i;
    XPLMGetDataf_f get_f; XPLMSetDataf_f set_f;
    XPLMGetDatad_f get_d; XPLMSetDatad_f set_d;
    XPLMGetDatavi_f get_vi; XPLMGetDatavf_f get_vf; XPLMSetDatavf_f set_vf; XPLMGetDatab_f get_b;
    void *read_context, *write_context;
} MockRef;
typedef struct { char name[128]; XPLMCommandCallback_f callback; void *context; int begun; } MockCommand;
static MockRef mock_refs[2048];
static MockCommand mock_commands[256];
static int ref_count, command_count, plays, mouse_y = 500, plugin_enabled = 1, fail_registration_after = -1;
static float clock_now;

static MockRef *new_ref(const char *name)
{
    MockRef *r;
    assert(ref_count < (int)(sizeof(mock_refs) / sizeof(mock_refs[0])));
    r = &mock_refs[ref_count++];
    memset(r, 0, sizeof(*r));
    snprintf(r->name, sizeof(r->name), "%s", name);
    r->registered = 1;
    return r;
}
XPLMDataRef XPLMFindDataRef(const char *name)
{
    int n;
    for (n = 0; n < ref_count; ++n) if (mock_refs[n].registered && !strcmp(name, mock_refs[n].name)) return &mock_refs[n];
    return !strncmp(name, "sim/", 4) ? new_ref(name) : NULL;
}
static MockRef *raw(const char *name) { return (MockRef *)XPLMFindDataRef(name); }
int XPLMGetDatai(XPLMDataRef h) { MockRef *r = h; assert(r && r->registered); return r->get_i ? r->get_i(r->read_context) : (int)r->value; }
float XPLMGetDataf(XPLMDataRef h) { MockRef *r = h; assert(r && r->registered); return r->get_f ? r->get_f(r->read_context) : (float)r->value; }
double XPLMGetDatad(XPLMDataRef h) { MockRef *r = h; assert(r && r->registered); return r->get_d ? r->get_d(r->read_context) : r->value; }
void XPLMSetDatai(XPLMDataRef h, int v)
{
    MockRef *r = h; assert(r && r->registered);
    if (r->set_i) { assert(r->writable); r->set_i(r->write_context, v); } else { assert(!r->get_i); r->value = v; }
}
void XPLMSetDataf(XPLMDataRef h, float v)
{
    MockRef *r = h; assert(r && r->registered);
    if (r->set_f) { assert(r->writable); r->set_f(r->write_context, v); } else { assert(!r->get_f); r->value = v; }
}
int XPLMGetDatavi(XPLMDataRef h, int *values, int offset, int count)
{
    MockRef *r = h; assert(r && r->registered && offset >= 0 && offset + count <= 16);
    if (r->get_vi) return r->get_vi(r->read_context, values, offset, count);
    memcpy(values, r->vi + offset, (size_t)count * sizeof(int)); return count;
}
int XPLMGetDatavf(XPLMDataRef h, float *values, int offset, int count)
{
    MockRef *r = h; assert(r && r->registered && offset >= 0 && offset + count <= 16);
    if (r->get_vf) return r->get_vf(r->read_context, values, offset, count);
    memcpy(values, r->vf + offset, (size_t)count * sizeof(float)); return count;
}
void XPLMSetDatavi(XPLMDataRef h, int *values, int offset, int count)
{
    MockRef *r = h; assert(r && r->registered && offset >= 0 && offset + count <= 16 && !r->get_vi);
    memcpy(r->vi + offset, values, (size_t)count * sizeof(int));
}
void XPLMSetDatavf(XPLMDataRef h, float *values, int offset, int count)
{
    MockRef *r = h; assert(r && r->registered && offset >= 0 && offset + count <= 16);
    if (r->set_vf) { r->set_vf(r->write_context, values, offset, count); return; }
    assert(!r->get_vf);
    memcpy(r->vf + offset, values, (size_t)count * sizeof(float));
}
XPLMDataRef XPLMRegisterDataAccessor(const char *name, XPLMDataTypeID type, int writable,
    XPLMGetDatai_f get_i, XPLMSetDatai_f set_i, XPLMGetDataf_f get_f, XPLMSetDataf_f set_f,
    XPLMGetDatad_f get_d, XPLMSetDatad_f set_d, XPLMGetDatavi_f get_vi, XPLMSetDatavi_f set_vi,
    XPLMGetDatavf_f get_vf, XPLMSetDatavf_f set_vf, XPLMGetDatab_f get_b, XPLMSetDatab_f set_b,
    void *read_context, void *write_context)
{
    MockRef *r;
    assert(!XPLMFindDataRef(name) && !set_vi && !set_b);
    if (type == xplmType_Int) assert(get_i && !get_f && !get_d);
    else if (type == (xplmType_Float | xplmType_Double)) assert(get_f && get_d);
    else if (type == xplmType_IntArray) assert(get_vi && !writable);
    else if (type == xplmType_FloatArray) assert(get_vf);
    else assert(type == xplmType_Data && get_b && !writable);
    assert(writable ? write_context != NULL : (!set_i && !set_f && !set_d && !set_vf && !write_context));
    if (fail_registration_after == 0) return NULL;
    if (fail_registration_after > 0) --fail_registration_after;
    r = new_ref(name);
    r->type = type; r->writable = writable; r->get_i = get_i; r->set_i = set_i; r->get_f = get_f; r->set_f = set_f;
    r->get_d = get_d; r->set_d = set_d; r->get_vi = get_vi; r->get_vf = get_vf; r->set_vf = set_vf; r->get_b = get_b;
    r->read_context = read_context; r->write_context = write_context;
    return r;
}
int XPLMGetDatab(XPLMDataRef h, void *values, int offset, int max)
{
    MockRef *r = h; assert(r && r->registered && r->get_b); return r->get_b(r->read_context, values, offset, max);
}
void XPLMUnregisterDataAccessor(XPLMDataRef h) { MockRef *r = h; assert(r && r->registered); r->registered = 0; }
XPLMCommandRef XPLMFindCommand(const char *name)
{
    int n;
    for (n = 0; n < command_count; ++n) if (!strcmp(name, mock_commands[n].name)) return &mock_commands[n];
    return NULL;
}
XPLMCommandRef XPLMCreateCommand(const char *name, const char *description)
{
    MockCommand *c = XPLMFindCommand(name);
    (void)description;
    if (c) return c;
    assert(command_count < (int)(sizeof(mock_commands) / sizeof(mock_commands[0])));
    c = &mock_commands[command_count++];
    memset(c, 0, sizeof(*c));
    snprintf(c->name, sizeof(c->name), "%s", name);
    return c;
}
void XPLMRegisterCommandHandler(XPLMCommandRef h, XPLMCommandCallback_f callback, int before, void *context)
{
    MockCommand *c = h; assert(c && !c->callback && callback && before); c->callback = callback; c->context = context;
}
void XPLMUnregisterCommandHandler(XPLMCommandRef h, XPLMCommandCallback_f callback, int before, void *context)
{
    MockCommand *c = h; assert(c && c->callback == callback && c->context == context && before);
    c->callback = NULL; c->context = NULL;
}
static int own_gpu_requests;
void XPLMCommandOnce(XPLMCommandRef h)
{
    MockCommand *c = h; assert(c);
    if (!strcmp(c->name, "sim/ground_ops/toggle_gpu_request")) ++own_gpu_requests;
    if (c->callback) c->callback(c, xplm_CommandBegin, c->context), c->callback(c, xplm_CommandEnd, c->context);
}
void XPLMCommandBegin(XPLMCommandRef h) { MockCommand *c = h; assert(c && !c->begun); c->begun = 1; }
void XPLMCommandEnd(XPLMCommandRef h) { MockCommand *c = h; assert(c && c->begun); c->begun = 0; }
void XPLMDebugString(const char *text) { (void)text; }
float XPLMGetElapsedTime(void) { return clock_now; }
void XPLMGetMouseLocationGlobal(int *x, int *y) { if (x) *x = 100; if (y) *y = mouse_y; }
XPLMPluginID XPLMGetMyID(void) { return 7; }
void XPLMGetPluginInfo(XPLMPluginID id, char *name, char *path, char *signature, char *description)
{
    (void)id; (void)name; (void)signature; (void)description;
    if (path) snprintf(path, 256, "%s/mac_x64/A380XAvionics.xpl", ELEC_TEST_ROOT);
}
static int fake_channel;
FMOD_CHANNEL *XPLMPlayPCMOnBus(void *buffer, uint32_t size, FMOD_SOUND_FORMAT format, int rate, int channels,
                               int loop, XPLMAudioBus bus, XPLMPCMComplete_f callback, void *refcon)
{
    (void)buffer; (void)size; (void)format; (void)rate; (void)channels; (void)loop; (void)bus; (void)callback; (void)refcon;
    ++plays; return (FMOD_CHANNEL *)&fake_channel;
}
FMOD_RESULT XPLMSetAudioPosition(FMOD_CHANNEL *channel, FMOD_VECTOR *position, FMOD_VECTOR *velocity)
{ (void)channel; (void)position; (void)velocity; return FMOD_OK; }
FMOD_RESULT XPLMSetAudioFadeDistance(FMOD_CHANNEL *channel, float min_distance, float max_distance)
{ (void)channel; (void)min_distance; (void)max_distance; return FMOD_OK; }
FMOD_RESULT XPLMSetAudioVolume(FMOD_CHANNEL *channel, float volume) { (void)channel; (void)volume; return FMOD_OK; }

/* ---- the unity-build neighbours the adapter talks to ---- */
static struct { XPLMDataRef n1; } refs;
static A380XAdirsState adirs;
static struct { int locked; } cockpit_door;
static struct { int rto_arm; } center_right;
static int spawn_changes, align_now_calls;
static int adirs_set_mode(int unit, int mode) { adirs.unit[unit].mode = mode; adirs.unit[unit].adr_valid = mode != 0; return 1; }
static int adirs_align_now(void) { int i; ++align_now_calls; for (i = 0; i < 3; ++i) adirs.unit[i].aligned = 1; return 1; }
#include "a380x_elec_panel.inc"
#include "a380x_elec_adapter.inc"
static void elec_spawn_changed(void) { ++spawn_changes; }
static int cds_overlay(int du, float x, float y, float w, float h) { (void)du; (void)x; (void)y; (void)w; (void)h; return 0; }

static MockCommand *cmd(const char *name) { MockCommand *c = XPLMFindCommand(name); assert(c && c->callback); return c; }
static int fire(const char *name)
{
    MockCommand *c = cmd(name);
    int r = c->callback(c, xplm_CommandBegin, c->context);
    c->callback(c, xplm_CommandEnd, c->context);
    return r;
}
static void click(const char *suffix) { char n[128]; snprintf(n, sizeof(n), "a380x/xp12/elec/%s", suffix); fire(n); }
static MockRef *pub(const char *name)
{
    char full[128]; MockRef *r;
    snprintf(full, sizeof(full), strncmp(name, "a380x/", 6) ? "a380x/xp12/elec/%s" : "%s", name);
    r = (MockRef *)XPLMFindDataRef(full); assert(r && r->registered); return r;
}
static int geti(const char *name) { return XPLMGetDatai(pub(name)); }
static float getf(const char *name) { return XPLMGetDataf(pub(name)); }
static float brightness[A380X_ELN_DU_COUNT] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
static void frames(float seconds)
{
    while (seconds > 0.0f) {
        clock_now += 0.05f; seconds -= 0.05f;
        elec_update(0.05f);
        eln_update(0.05f, brightness);
    }
}
static void engines(float n1, float n2, int running)
{
    int i;
    for (i = 0; i < 4; ++i) {
        raw("sim/cockpit2/engine/indicators/N1_percent")->vf[i] = n1;
        raw("sim/flightmodel/engine/ENGN_N2_")->vf[i] = n2;
        raw("sim/flightmodel/engine/ENGN_running")->vi[i] = running;
    }
}
static void stock_world(void)
{
    static const char *const commands[] = {
        "sim/ground_ops/toggle_gpu_request", "sim/electrical/battery_1_toggle", "sim/electrical/battery_1_on",
        "sim/electrical/battery_1_off", "sim/electrical/battery_2_toggle", "sim/electrical/generator_1_off",
        "sim/electrical/GPU_on", "sim/electrical/GPU_off", "sim/electrical/APU_start", "sim/systems/avionics_on",
        "sim/starters/engage_starter_1", "sim/starters/engage_starter_2", "sim/starters/engage_starter_3",
        "sim/starters/engage_starter_4", "sim/flight_controls/flaps_down", "a380x/xp12/cockpit_door/lock",
        "a380x/xp12/center_right/rto_toggle"
    };
    int i;
    raw("sim/aircraft/electrical/num_batteries")->value = 1;
    raw("sim/aircraft/electrical/battery_watt_hr_max")->value = 1272.0;
    raw("sim/flightmodel/failures/onground_any")->value = 1;
    raw("sim/time/sim_speed_actual")->value = 1.0;
    raw("sim/cockpit2/electrical/battery_on")->vi[0] = 1;   /* X-Plane cold: battery switch left on */
    raw("sim/cockpit2/electrical/generator_on")->vi[0] = 1;
    for (i = 0; i < (int)(sizeof(commands) / sizeof(commands[0])); ++i) XPLMCreateCommand(commands[i], "");
    refs.n1 = raw("sim/cockpit2/engine/indicators/N1_percent");
}

static void test_cold_dark_power_up(void)
{
    MockRef *battery = raw("sim/cockpit2/electrical/battery_on"), *charge = raw("sim/cockpit/electrical/battery_charge_watt_hr");
    MockRef *gpu = raw("sim/cockpit2/electrical/GPU_generator_on"), *avionics = raw("sim/cockpit2/switches/avionics_power_on");
    MockRef *apu = raw("sim/cockpit2/electrical/APU_starter_switch");
    assert(elec_init() && eln_init());
    assert(geti("network_active") && geti("external_logic") && geti("spawn_mode") == A380X_ELN_SPAWN_COLD_DARK);
    frames(0.5f);
    /* 0. cold & dark: everything dark, stock battery off, only the hot buses live */
    assert(!geti("pb/bat1") && !geti("pb/bat_ess") && !geti("pb/bat2") && !geti("pb/bat_apu") && geti("pb/gen1"));
    assert(battery->vi[0] == 0 && avionics->value == 0 && geti("bus/hot1_powered") && !geti("bus/dc_ess_powered"));
    assert(geti("du/ewd_state") == A380X_ELN_DU_OFF && geti("du/capt_pfd_state") == A380X_ELN_DU_OFF);
    assert(getf("lamp/bat1_lower") == 0.0f && getf("lamp/gen1_upper") == 0.0f && a380x_power.available);
    assert(!a380x_power_bus(A380X_ELN_DC_ESS, 1) && a380x_power_bus(A380X_ELN_HOT1, 0) && !a380x_power_isis(1));
    /* 1. BAT 1: nothing new (ELC needs ESS) */
    click("bat1_toggle"); frames(0.5f);
    assert(geti("pb/bat1") && !geti("bus/dc_ess_powered") && battery->vi[0] == 0);
    /* 2. ESS: DC ESS on batteries, AC EMER via the static inverter, EWD + CAPT MFD boot, ISIS */
    click("bat_ess_toggle"); frames(0.6f);
    assert(geti("bus/dc_ess_powered") && geti("bus/ac_emer_powered") && !geti("bus/dc_ess_inflt_powered"));
    assert(geti("contactor/7XB") && geti("contactor/3XB_2") && getf("stat_inv/potential") > 100.0f);
    assert(battery->vi[0] == 1 && avionics->value == 1);                 /* stock slaved to native DC */
    assert(geti("du/ewd_state") == A380X_ELN_DU_SELFTEST && geti("du/capt_mfd_state") == A380X_ELN_DU_SELFTEST);
    assert(geti("du/capt_pfd_state") == A380X_ELN_DU_OFF && geti("du/sd_state") == A380X_ELN_DU_OFF);
    assert(a380x_power_isis(0) && !a380x_power_fcu(1));
    assert(getf("lamp/gen1_upper") == 1.0f && getf("lamp/gen3_upper") == 0.0f);   /* GEN 1/2 on DC ESS, 3/4 on DC 2 */
    assert(getf("lamp/bat2_lower") == 0.0f && getf("lamp/ac_ess_feed_upper") == 0.0f);
    /* stock battery charge pinned while native DC is live */
    charge->vf[0] = 100.0f; frames(0.1f); assert(charge->vf[0] == 1272.0f);
    /* 3./4. BAT 2 and APU BAT: DC APU from the APU battery */
    click("bat2_toggle"); click("bat_apu_toggle"); frames(0.2f);
    assert(!geti("bus/dc2_powered") && geti("bus/dc_apu_powered"));
    frames(15.5f);
    assert(geti("du/ewd_state") == A380X_ELN_DU_ON && geti("du/capt_mfd_state") == A380X_ELN_DU_ON);
    /* 5. ground power: EXT 1-4 AVAIL; 6. EXT 2 ON: AC 1/2, 108PH, CAPT PFD + F/O side boot */
    click("gpu_connect_toggle"); frames(0.1f);
    assert(geti("ext/connected") && getf("lamp/ext1_upper") == 1.0f && getf("lamp/ext4_upper") == 1.0f && gpu->value == 0);
    click("ext2_press"); frames(0.2f);
    assert(geti("bus/ac1_powered") && geti("bus/ac2_powered") && !geti("bus/ac3_powered") && gpu->value == 1);
    assert(geti("bus/dc_ess_inflt_powered") && geti("bus/dc2_powered") && geti("contactor/980PC"));
    assert(getf("lamp/ext2_lower") == 1.0f && getf("lamp/ext2_upper") == 0.0f && a380x_power_fcu(0));
    assert(geti("du/capt_pfd_state") == A380X_ELN_DU_BOOT || geti("du/capt_pfd_state") == A380X_ELN_DU_SELFTEST);
    assert(geti("du/sd_state") != A380X_ELN_DU_OFF && geti("ac_main_powered"));
    /* 7. EXT 3: right side, TR 2, TR APU */
    click("ext3_press"); frames(0.5f);
    assert(geti("bus/ac4_powered") && !geti("contactor/980PC") && geti("contactor/7PU"));
    frames(16.0f);
    assert(geti("du/capt_pfd_state") == A380X_ELN_DU_ON && geti("du/fo_nd_state") == A380X_ELN_DU_ON && a380x_power_du_on(A380X_ELN_DU_SD, 0));
    /* 10.-11. APU: MASTER (stock switch, cockpit manipulator) -> flap -> START -> starter switch 2 */
    apu->value = 1; frames(0.5f);
    assert(geti("a380x/xp12/apu/ecb_powered") && getf("a380x/xp12/apu/flap_open_ratio") > 0.0f);
    assert(fire("sim/electrical/APU_start") == 0 && geti("a380x/xp12/apu/start_on"));   /* intercepted */
    frames(8.5f); assert(apu->value == 2 && geti("contactor/10KA_5KA"));
    /* FBW PW980 from the start contactor: N 55 % (starter cut-out) at 34.2 s, AVAIL 2 s after 95 % (42.3 s) */
    frames(32.0f); assert(geti("contactor/10KA_5KA") && getf("a380x/xp12/apu/n_percent") < 55.0f);
    frames(2.0f);
    assert(apu->value == 2 && !geti("contactor/10KA_5KA"));                     /* stock start held */
    raw("sim/cockpit2/electrical/APU_running")->value = 1; frames(0.1f); assert(apu->value == 1);
    frames(5.0f); assert(!geti("a380x/xp12/apu/avail"));
    frames(3.0f);
    assert(geti("a380x/xp12/apu/avail") && !geti("a380x/xp12/apu/start_on"));
    assert(getf("a380x/xp12/apu/generator_volts") > 100.0f);
    /* EXT 2 + 3 + APU (spec 3.2): AC 1 APU A, AC 2 EXT 2, AC 3 EXT 3, AC 4 APU B */
    assert(geti("contactor/990XG2") && geti("contactor/990XG3") && geti("contactor/990XS1") && geti("contactor/990XS2"));
}

static void test_hardware_failures_pause(void)
{
    MockRef *battery = raw("sim/cockpit2/electrical/battery_on"), *gen = raw("sim/cockpit2/electrical/generator_on");
    int before = plays;
    /* hardware bindings act on the native pbs and are consumed */
    assert(fire("sim/electrical/battery_2_toggle") == 0 && !geti("pb/bat2") && plays > before);
    assert(fire("sim/electrical/battery_2_toggle") == 0 && geti("pb/bat2"));
    assert(fire("sim/electrical/generator_1_off") == 0 && !geti("pb/gen1"));
    frames(0.1f); assert(gen->vi[0] == 0);
    click("gen1_on"); frames(0.1f); assert(gen->vi[0] == 1);
    assert(fire("sim/systems/avionics_on") == 0);
    assert(fire("sim/electrical/GPU_off") == 0 && !geti("pb/ext2") && !geti("pb/ext3"));
    frames(0.2f); assert(raw("sim/cockpit2/electrical/GPU_generator_on")->value == 0);
    assert(geti("contactor/990XS1") && geti("bus/ac1_powered"));                       /* APU takes over */
    assert(fire("sim/electrical/GPU_on") == 0 && geti("pb/ext1") && geti("pb/ext4"));
    frames(0.2f);
    /* stock failure inputs map to FAULT lamps; native-only failures are writable */
    raw("sim/operation/failures/rel_genera1")->value = 6; frames(0.1f);
    assert(getf("lamp/gen2_upper") == 1.0f);                                           /* lit anyway: engine off */
    raw("sim/operation/failures/rel_batter0")->value = 6; frames(0.1f);
    assert(getf("lamp/bat1_upper") == 1.0f && !geti("contactor/990PB1"));
    raw("sim/operation/failures/rel_batter0")->value = 0; raw("sim/operation/failures/rel_genera1")->value = 0;
    XPLMSetDatai(pub("fail/tr_ess"), 1); frames(6.0f);
    assert(geti("bus/dc_ess_powered") && geti("contactor/6PC2") && !geti("contactor/6PE"));
    XPLMSetDatai(pub("fail/tr_ess"), 0);
    XPLMSetDatai(pub("fail/drive1"), 1); frames(0.1f); assert(getf("lamp/drive1_upper") == 1.0f);
    XPLMSetDatai(pub("fail/drive1"), 0);
    XPLMSetDatai(pub("ann_lt"), 0); frames(0.1f); assert(getf("lamp/bus_tie_lower") == 1.0f);
    XPLMSetDatai(pub("ann_lt"), 1);
    /* pause freezes the network timers: AC lost while paused does not shed 108PH */
    assert(fire("sim/electrical/GPU_off") == 0);
    raw("sim/cockpit2/electrical/APU_starter_switch")->value = 0;
    eln.apu.state = A380X_PW980_SHUTDOWN; eln.apu.n = eln.apu.n2 = 0.0;   /* past FBW's 60 s cooldown */
    raw("sim/time/paused")->value = 1; frames(30.0f);
    assert(geti("bus/dc_ess_inflt_powered") && !geti("ac_main_powered"));
    raw("sim/time/paused")->value = 0; frames(10.5f);
    assert(!geti("bus/dc_ess_inflt_powered") && geti("bus/dc_ess_powered"));
    /* BAT selector / voltmeter */
    XPLMSetDatai(pub("bat/selector"), 3); assert(getf("bat/display_volts") > 20.0f);
    XPLMSetDatai(pub("bat/selector"), 2); assert(getf("bat/display_volts") < 0.0f);
    /* NOSE switch OFF / TAXI / T.O.: TAXI lights the taxi light, T.O. also the T.O. lights (generic light 0);
     * a taxi light switched elsewhere moves the switch */
    {
        MockRef *taxi = raw("sim/cockpit2/switches/taxi_light_on"), *gen = raw("sim/cockpit2/switches/generic_lights_switch");
        click("lights/nose_up"); frames(0.1f);
        assert(geti("lights/nose_switch") == 1 && (int)taxi->value == 1 && gen->vf[A380X_ELN_TO_LIGHT] == 0.0f);
        click("lights/nose_up"); frames(0.1f);
        assert(geti("lights/nose_switch") == 2 && (int)taxi->value == 1 && gen->vf[A380X_ELN_TO_LIGHT] == 1.0f);
        click("lights/nose_up"); frames(0.1f); assert(geti("lights/nose_switch") == 2);
        click("lights/nose_down"); click("lights/nose_down"); frames(0.1f);
        assert(geti("lights/nose_switch") == 0 && (int)taxi->value == 0 && gen->vf[A380X_ELN_TO_LIGHT] == 0.0f);
        taxi->value = 1; frames(0.1f); assert(geti("lights/nose_switch") == 1);
        taxi->value = 0; frames(0.1f); assert(geti("lights/nose_switch") == 0);
    }
    (void)battery;
}

static void test_presets_and_reload(void)
{
    MockRef *strobe = raw("sim/cockpit2/switches/strobe_lights_on"), *pumps = raw("sim/cockpit2/engine/actuators/fuel_pump_on");
    char text[64];
    int frames_run;
    /* reload with engines stopped: cold & dark again (panel reset + network spawn) */
    elec_plane_loaded(); eln_plane_loaded(); frames(0.2f);
    assert(geti("spawn_mode") == A380X_ELN_SPAWN_COLD_DARK && !geti("pb/bat1") && !geti("ext/connected"));
    assert(geti("external_logic") && !geti("bus/dc_ess_powered"));
    /* flyPad TURNAROUND, expedited: batteries, GPU, EXT, APU, ADIRS, lights, fuel pumps */
    XPLMSetDatai(pub("preset/expedited"), 1);
    click("preset/2");
    assert(geti("preset/active") == 2 && geti("preset/step_count") > 20);
    for (frames_run = 0; frames_run < 400 && geti("preset/active"); ++frames_run) {
        frames(0.05f);
        if (raw("sim/cockpit2/electrical/APU_starter_switch")->value == 2) raw("sim/cockpit2/electrical/APU_N1_percent")->value = 100.0;
    }
    assert(!geti("preset/active") && geti("preset/done") == 2);
    assert(geti("pb/bat1") && geti("pb/bat_apu") && geti("ext/connected") && geti("pb/ext1") && geti("pb/ext4"));
    assert(adirs.unit[0].mode == A380X_ADIRS_NAV && adirs.unit[2].mode == A380X_ADIRS_NAV);
    assert(pumps->vi[0] == 1 && pumps->vi[3] == 1 && strobe->vi[0] == 0 && geti("a380x/xp12/apu/avail"));
    assert(raw("sim/cockpit2/bleedair/actuators/apu_bleed")->value == 1);
    assert(XPLMGetDatab(pub("preset/step_name"), text, 0, sizeof(text)) > 0 && strstr(text, "DONE"));
    /* presets are refused in flight */
    raw("sim/flightmodel/failures/onground_any")->value = 0; click("preset/1"); assert(!geti("preset/active"));
    raw("sim/flightmodel/failures/onground_any")->value = 1;
    /* reload with the engines running: READY spawn (late detection after the load message) */
    engines(0.0f, 0.0f, 0);
    elec_plane_loaded(); eln_plane_loaded();
    assert(geti("spawn_mode") == A380X_ELN_SPAWN_COLD_DARK);
    engines(25.0f, 70.0f, 1); frames(0.2f);
    assert(geti("spawn_mode") == A380X_ELN_SPAWN_READY && spawn_changes == 1);
    /* every engine reads its own element (READY FOR TAXI waits on ENG 2-4 too) */
    raw("sim/flightmodel/engine/ENGN_running")->vi[2] = 0;
    assert(eln_eng_running(0) && eln_eng_running(1) && !eln_eng_running(2) && eln_eng_running(3));
    raw("sim/flightmodel/engine/ENGN_running")->vi[2] = 1;
    assert(geti("pb/bat1") && geti("pb/bat2") && geti("bus/ac1_powered") && geti("contactor/990XU1"));
    assert(geti("du/capt_pfd_state") == A380X_ELN_DU_ON && geti("du/ewd_state") == A380X_ELN_DU_ON);
    assert(!getf("lamp/gen1_upper") && raw("sim/cockpit2/electrical/generator_on")->vi[3] == 1);
    /* ready spawn directly from the load message */
    elec_plane_loaded(); eln_plane_loaded();
    assert(geti("spawn_mode") == A380X_ELN_SPAWN_READY && a380x_power_du_on(A380X_ELN_DU_FO_PFD, 0));
    /* new flight: plugins load before X-Plane puts the aircraft on the ground -> READY at load, then cold &
     * dark (batteries OFF) once the spawn checks end on the ground with the engines stopped */
    engines(0.0f, 0.0f, 0);
    raw("sim/flightmodel/failures/onground_any")->value = 0;
    spawn_changes = 0;
    elec_plane_loaded(); eln_plane_loaded();
    assert(geti("spawn_mode") == A380X_ELN_SPAWN_READY && geti("pb/bat1"));
    raw("sim/flightmodel/failures/onground_any")->value = 1;
    frames(1.0f);
    assert(geti("spawn_mode") == A380X_ELN_SPAWN_READY && !spawn_changes);
    frames(4.0f);
    assert(geti("spawn_mode") == A380X_ELN_SPAWN_COLD_DARK && spawn_changes == 1);
    assert(!geti("pb/bat1") && !geti("pb/bat2") && !geti("pb/bat_ess") && !geti("pb/bat_apu") && !geti("ext/connected"));
    /* disable stops a running preset; the CDS overlay hook is the renderer's (stubbed here) */
    click("preset/3"); assert(geti("preset/active") == 3); eln_disable(); assert(!geti("preset/active"));
    assert(!cds_overlay(A380X_ELN_DU_EWD, 0, 0, 768, 1024));
    /* shutdown releases everything and hands the panel back */
    eln_shutdown();
    assert(!a380x_power.available && !geti("external_logic") && !XPLMFindDataRef("a380x/xp12/elec/bus/ac1_powered"));
    assert(!((MockCommand *)XPLMFindCommand("sim/electrical/battery_1_toggle"))->callback);
    elec_shutdown();
}

int main(int argc, char **argv)
{
    int n;
    stock_world();
    if (argc > 1 && !strcmp(argv[1], "--contracts")) {
        assert(elec_init() && eln_init());
        for (n = 0; n < command_count; ++n)
            if (!strncmp(mock_commands[n].name, "a380x/", 6) && mock_commands[n].callback && mock_commands[n].callback == eln_command_handler)
                printf("C %s\n", mock_commands[n].name);
        for (n = 0; n < eln_ref_count; ++n) printf("D %s\n", eln_refs[n].name);
        for (n = 0; n < ENR_COUNT; ++n) printf("R %s\n", eln_raw_names[n]);
        for (n = 0; n < (int)(sizeof(eln_intercepts) / sizeof(eln_intercepts[0])); ++n) printf("I %s\n", eln_intercepts[n].name);
        eln_shutdown(); elec_shutdown();
        return 0;
    }
    test_cold_dark_power_up();
    test_hardware_failures_pause();
    test_presets_and_reload();
    /* registration failure: the network stays off, the panel keeps working */
    assert(elec_init());
    fail_registration_after = 5; assert(!eln_init()); fail_registration_after = -1;
    assert(!a380x_power.available && !geti("external_logic"));
    elec_shutdown();
    puts("A380 ELEC network adapter: spawn, cold & dark power-up, stock slaving, interception, failures, pause, APU, presets and reload passed");
    return 0;
}
