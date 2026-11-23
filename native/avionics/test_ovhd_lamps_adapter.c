/* SPDX-License-Identifier: GPL-3.0-or-later
 * Overhead annunciator lamp adapter (a380x_ovhd_lamps.inc) with the production ELEC panel and
 * A380 network adapters against a mock XPLM: registry contract, cold & dark battery power-up
 * per FBW lamp bus, APU MASTER / START / AVAIL, ground power and AC legends, ANN LT shared with
 * the ELEC panel (TEST / BRT / DIM, both datarefs and the switch commands), ANN LT TEST on the
 * ADIRS legends, the simplified supply setting (ELEC legends included), fault inputs, plane
 * reload, disable and registration failure.
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

/* ---- the unity-build neighbours the adapters talk to ---- */
static struct { XPLMDataRef n1; } refs;
static A380XAdirsState adirs;
static A380XAdirsInputs adirs_inputs;
static struct { int locked; } cockpit_door;
static struct { int rto_arm; } center_right;
static int spawn_changes, align_now_calls;
static int adirs_set_mode(int unit, int mode) { adirs.unit[unit].mode = mode; adirs.unit[unit].adr_valid = mode != 0; return 1; }
static int adirs_align_now(void) { int i; ++align_now_calls; for (i = 0; i < 3; ++i) adirs.unit[i].aligned = 1; return 1; }
#include "a380x_elec_panel.inc"
#include "a380x_elec_adapter.inc"
#include "a380x_ovhd_lamps.inc"
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
    snprintf(full, sizeof(full), strncmp(name, "a380x/", 6) ? "a380x/xp12/ovhd/%s" : "%s", name);
    r = (MockRef *)XPLMFindDataRef(full); assert(r && r->registered); return r;
}
static int geti(const char *name) { return XPLMGetDatai(pub(name)); }
static float lamp(const char *name) { char n[128]; snprintf(n, sizeof(n), "lamp/%s", name); return XPLMGetDataf(pub(n)); }
static float elamp(const char *name) { char n[128]; snprintf(n, sizeof(n), "a380x/xp12/elec/lamp/%s", name); return XPLMGetDataf(pub(n)); }
static void seti(const char *name, int v) { XPLMSetDatai(pub(name), v); }
static float brightness[A380X_ELN_DU_COUNT] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
static void frames(float seconds)
{
    while (seconds > 0.0f) {
        clock_now += 0.05f; seconds -= 0.05f;
        elec_update(0.05f);
        eln_update(0.05f, brightness);
        ovhd_update(0.05f);
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
    for (i = 0; i < (int)(sizeof(commands) / sizeof(commands[0])); ++i) XPLMCreateCommand(commands[i], "");
    refs.n1 = raw("sim/cockpit2/engine/indicators/N1_percent");
}

static void test_power_up(void)
{
    MockRef *apu = raw("sim/cockpit2/electrical/APU_starter_switch"), *bleed = raw("sim/cockpit2/bleedair/actuators/engine_bleed_sov");
    MockRef *pump = raw("sim/cockpit2/engine/actuators/fuel_pump_on");
    assert(elec_init() && eln_init() && ovhd_init());
    /* Default is the simplified supply (user decision); these checks exercise FBW supply. */
    assert(geti("ann_lt") == A380X_OVHD_ANN_BRT && geti("settings/lamp_supply") == A380X_OVHD_SUPPLY_SIMPLE);
    seti("settings/lamp_supply", A380X_OVHD_SUPPLY_FBW);
    frames(0.5f);
    /* cold & dark (engine bleeds and feed pumps OFF in the mock): every overhead legend dark,
     * even in ANN LT TEST, because no lamp bus is powered */
    assert(geti("lamp_count") == 0 && lamp("eng2_bleed_lower") == 0.0f && lamp("feed1_main_lower") == 0.0f);
    seti("ann_lt", 0); frames(0.1f);
    assert(geti("lamp_count") == 0 && elamp("bat1_lower") == 0.0f && geti("a380x/xp12/elec/ann_lt") == 0);
    seti("ann_lt", 1); frames(0.1f);
    /* BAT 1 + BAT 2 (no ESS): DC ESS stays dead (FBW ELC needs BAT 1 AND ESS) -> still dark */
    click("bat1_toggle"); click("bat2_toggle"); frames(0.5f);
    assert(!geti("a380x/xp12/elec/bus/dc_ess_powered") && geti("lamp_count") == 0 && elamp("bat_ess_lower") == 0.0f);
    /* BAT 1 + ESS: DC ESS -> DC ESS legends (ENG 2/3 BLEED OFF, ELEC GEN 1/2 FAULT, ESS) light */
    click("bat2_toggle"); click("bat_ess_toggle"); frames(0.6f);
    assert(geti("a380x/xp12/elec/bus/dc_ess_powered") && !geti("a380x/xp12/elec/bus/dc2_powered"));
    assert(lamp("eng2_bleed_lower") == 1.0f && lamp("eng3_bleed_lower") == 1.0f);
    assert(lamp("eng1_bleed_lower") == 0.0f && lamp("eng4_bleed_lower") == 0.0f);     /* DC 2 */
    assert(lamp("feed1_main_lower") == 0.0f && elamp("bat2_lower") == 0.0f);           /* AC ESS / DC 2 */
    assert(elamp("gen1_upper") == 1.0f && geti("lamp_count") == 2);
    bleed->vi[1] = 1; frames(0.1f); assert(lamp("eng2_bleed_lower") == 0.0f);
    /* APU MASTER on DC APU: dark until APU BAT */
    apu->value = 1; frames(0.2f);
    assert(lamp("apu_master_lower") == 0.0f);
    click("bat_apu_toggle"); frames(0.2f);
    assert(geti("a380x/xp12/elec/bus/dc_apu_powered") && lamp("apu_master_lower") == 1.0f);
    /* ANN LT TEST on batteries: DC ESS / DC APU legends only */
    assert(fire("a380x/xp12/ovhd/ann_lt_up") == 1); frames(0.2f);
    assert(geti("ann_lt") == 0 && geti("a380x/xp12/elec/ann_lt") == 0 && XPLMGetDataf(pub("ann_lt_switch")) == 0.0f);
    assert(lamp("rcdr_gnd_ctl_lower") == 1.0f && lamp("apu_bleed_upper") == 1.0f && lamp("apu_start_upper") == 1.0f);
    assert(lamp("feed1_main_lower") == 0.0f && lamp("pack2_upper") == 0.0f && lamp("eng1_anti_ice_lower") == 0.0f);
    assert(elamp("bat1_upper") == 1.0f && elamp("gen3_upper") == 0.0f);
    /* ADIRS legends of a powered ADIRU light in TEST */
    adirs_inputs.unit[0].ir_powered = 1; frames(0.05f);
    assert(adirs.unit[0].ir_off_lit && adirs.unit[0].ir_fault_lit && !adirs.unit[1].ir_off_lit);
    adirs_inputs.unit[0].ir_powered = 0;
    /* the ELEC alias drives the same state: elec/ann_lt = 2 (DIM) */
    XPLMSetDatai(pub("a380x/xp12/elec/ann_lt"), 2); frames(0.2f);
    assert(geti("ann_lt") == 2 && fabsf(lamp("apu_master_lower") - A380X_OVHD_DIM_LEVEL) < 1e-6f);
    assert(fabsf(XPLMGetDataf(pub("ann_lt_switch")) - 1.0f) < 1e-6f);
    assert(fire("a380x/xp12/ovhd/ann_lt_down") == 1 && geti("ann_lt") == 2);
    assert(fire("a380x/xp12/ovhd/ann_lt_up") == 1 && geti("ann_lt") == 1 && geti("a380x/xp12/elec/ann_lt") == 1);
    frames(0.1f); assert(lamp("apu_master_lower") == 1.0f && lamp("rcdr_gnd_ctl_lower") == 0.0f);
    /* fault input */
    seti("fail/apu_bleed", 1); frames(0.05f); assert(lamp("apu_bleed_upper") == 1.0f);
    seti("fail/apu_bleed", 0); frames(0.05f); assert(lamp("apu_bleed_upper") == 0.0f);
    /* ground power + EXT 2: AC 1/2 + AC ESS + DC 1/2 -> AC legends */
    click("gpu_connect_toggle"); click("ext2_press"); frames(0.5f);
    assert(geti("a380x/xp12/elec/bus/ac_ess_powered") && geti("a380x/xp12/elec/bus/ac2_powered"));
    assert(lamp("feed1_main_lower") == 1.0f && lamp("feed2_main_lower") == 1.0f);
    pump->vi[0] = 1; frames(0.1f);
    assert(lamp("feed1_main_lower") == 0.0f && lamp("eng1_bleed_lower") == 1.0f && lamp("pack1_lower") == 1.0f);
    assert(lamp("crew_oxy_lower") == 1.0f);
    raw("sim/cockpit2/oxygen/actuators/o2_valve_on")->value = 1; frames(0.1f); assert(lamp("crew_oxy_lower") == 0.0f);
    raw("sim/cockpit2/ice/ice_inlet_heat_on_per_engine")->vi[1] = 1; frames(0.1f);
    assert(lamp("eng2_anti_ice_lower") == 1.0f && lamp("eng1_anti_ice_lower") == 0.0f);
    raw("sim/operation/failures/rel_ice_inlet_heat2")->value = 6; frames(0.1f);
    assert(lamp("eng2_anti_ice_upper") == 1.0f);
    raw("sim/operation/failures/rel_ice_inlet_heat2")->value = 0;
    /* APU START (native ECB): ON while starting, AVAIL at 95 % N + 2 s */
    assert(fire("sim/electrical/APU_start") == 0);
    frames(0.2f); assert(lamp("apu_start_lower") == 1.0f && lamp("apu_start_upper") == 0.0f);
    frames(8.5f);
    raw("sim/cockpit2/electrical/APU_N1_percent")->value = 100.0;
    raw("sim/cockpit2/electrical/APU_running")->value = 1; frames(2.5f);
    assert(lamp("apu_start_upper") == 1.0f && lamp("apu_start_lower") == 0.0f);
    raw("sim/cockpit2/bleedair/actuators/apu_bleed")->value = 1; frames(0.1f);
    assert(lamp("apu_bleed_lower") == 1.0f);
}

static void test_simple_supply_and_lifecycle(void)
{
    /* simplified supply: BAT 1 + BAT 2 only light every OFF legend, ELEC panel included */
    click("ext2_press"); click("gpu_connect_toggle");
    raw("sim/cockpit2/electrical/APU_starter_switch")->value = 0;
    raw("sim/cockpit2/electrical/APU_N1_percent")->value = 0.0; raw("sim/cockpit2/electrical/APU_running")->value = 0;
    click("bat_ess_toggle"); click("bat_apu_toggle"); click("bat2_toggle"); frames(12.0f);
    assert(geti("a380x/xp12/elec/pb/bat1") && geti("a380x/xp12/elec/pb/bat2") && !geti("a380x/xp12/elec/pb/bat_ess"));
    assert(!geti("a380x/xp12/elec/bus/dc_ess_powered") && elamp("bat_ess_lower") == 0.0f);
    raw("sim/cockpit2/engine/actuators/fuel_pump_on")->vi[0] = 0;
    assert(lamp("feed1_main_lower") == 0.0f);
    seti("settings/lamp_supply", 1); frames(0.1f);
    assert(geti("settings/lamp_supply") == 1 && eln.lamp_supply_simple);
    assert(lamp("feed1_main_lower") == 1.0f && elamp("bat_ess_lower") == 1.0f && elamp("bat_apu_lower") == 1.0f);
    click("bat1_toggle"); click("bat2_toggle"); frames(0.2f);                  /* all BAT OFF */
    assert(lamp("feed1_main_lower") == 0.0f && elamp("bat1_lower") == 0.0f);
    seti("settings/lamp_supply", 0); frames(0.1f); assert(!eln.lamp_supply_simple);
    /* reload: ANN LT back to BRT, faults cleared, setting kept */
    seti("fail/pack1", 1); seti("ann_lt", 0);
    elec_plane_loaded(); eln_plane_loaded(); ovhd_plane_loaded();
    assert(geti("ann_lt") == 1 && geti("fail/pack1") == 0 && geti("a380x/xp12/elec/ann_lt") == 1);
    /* disabled plugin: every lamp reads dark */
    click("bat1_toggle"); click("bat_ess_toggle"); frames(0.6f);
    seti("ann_lt", 0); frames(0.1f); assert(lamp("rcdr_gnd_ctl_lower") == 1.0f);
    eln_disable(); plugin_enabled = 0; assert(lamp("rcdr_gnd_ctl_lower") == 0.0f); plugin_enabled = 1;
    seti("ann_lt", 1);
}

int main(int argc, char **argv)
{
    int n;
    stock_world();
    if (argc > 1 && !strcmp(argv[1], "--contracts")) {
        assert(elec_init() && eln_init() && ovhd_init());
        for (n = 0; n < command_count; ++n)
            if (mock_commands[n].callback == ovhd_command_handler || mock_commands[n].callback == ovhd_pb_handler)
                printf("C %s\n", mock_commands[n].name);
        for (n = 0; n < ovhd_ref_count; ++n) printf("D %s\n", ovhd_refs[n].name);
        for (n = 0; n < OVR_COUNT; ++n) printf("R %s\n", ovhd_raw_names[n]);
        ovhd_shutdown(); eln_shutdown(); elec_shutdown();
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "--cold-dark")) {
        /* Report: the lit overhead / ELEC legends after loading the aircraft with only the batteries
         * on (default lamp supply), then with HOT AIR 1 / CARGO HEATER pushed to OFF. */
        static const char *const steps[] = {"plane loaded", "BAT 1 + BAT 2 + ESS + APU BAT AUTO",
                                            "HOT AIR 1 and CARGO HEATER pushed (to OFF)"};
        assert(elec_init() && eln_init() && ovhd_init());
        elec_plane_loaded(); eln_plane_loaded(); ovhd_plane_loaded();
        for (int step = 0; step < 3; ++step) {
            if (step == 1) { click("bat1_toggle"); click("bat2_toggle"); click("bat_ess_toggle"); click("bat_apu_toggle"); }
            if (step == 2) { fire("a380x/xp12/ovhd/pb/hot_air1"); fire("a380x/xp12/ovhd/pb/cargo_heater"); }
            frames(1.0f);
            printf("== %s:", steps[step]);
            for (n = 0; n < ref_count; ++n)
                if (mock_refs[n].registered && strstr(mock_refs[n].name, "/lamp/") &&
                    XPLMGetDataf((XPLMDataRef)&mock_refs[n]) > 0.0f)
                    printf(" %s", strstr(mock_refs[n].name, "/lamp/") + 6);
            printf("\n");
        }
        ovhd_shutdown(); eln_shutdown(); elec_shutdown();
        return 0;
    }
    (void)cds_overlay;
    test_power_up();
    test_simple_supply_and_lifecycle();
    ovhd_shutdown();
    for (n = 0; n < ovhd_ref_count; ++n) assert(!ovhd_refs[n].handle);
    /* re-registration after shutdown, then a registration failure leaves nothing registered */
    assert(ovhd_init()); ovhd_shutdown();
    fail_registration_after = 3; assert(!ovhd_init()); fail_registration_after = -1;
    assert(!XPLMFindDataRef("a380x/xp12/ovhd/lamp/apu_master_upper"));
    eln_shutdown(); elec_shutdown();
    puts("A380 overhead lamp adapter: registry, battery power-up per FBW bus, APU, ground power, ANN LT, ADIRS test, simplified supply, reload and failure passed");
    return 0;
}
