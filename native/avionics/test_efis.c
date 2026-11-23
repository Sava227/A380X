/* SPDX-License-Identifier: GPL-3.0-or-later
 * Exercise the production EFIS adapter with the actual command handlers and
 * registered outputs. SDK commands persist between unload/reload cycles.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMNavigation.h"
#include "a380x_guidance.h"
#define ARRAY_LEN(a) ((int)(sizeof(a)/sizeof((a)[0])))
typedef struct {
    char name[160];
    int registered;
    float value;
    int array[8];
    XPLMGetDatai_f read_i;
    XPLMGetDataf_f read_f;
    void *refcon;
} MockRef;
typedef struct {
    char name[160];
    XPLMCommandCallback_f callback;
    void *refcon;
} MockCommand;
static MockRef mock_refs[600];
static MockCommand mock_commands[60];
static int ref_count, command_count, plan_entry, plan_count = 3;
static float mock_time = 1.0f;
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
int XPLMGetDatai(XPLMDataRef handle)
{
    MockRef *ref = handle;
    assert(ref && ref->registered);
    return ref->read_i ? ref->read_i(ref->refcon) : (int)ref->value;
}
float XPLMGetDataf(XPLMDataRef handle)
{
    MockRef *ref = handle;
    assert(ref && ref->registered);
    return ref->read_f ? ref->read_f(ref->refcon) : ref->value;
}
void XPLMSetDatai(XPLMDataRef handle, int value)
{
    MockRef *ref = handle;
    assert(ref && ref->registered && !ref->read_i);
    ref->value = (float)value;
}
void XPLMSetDataf(XPLMDataRef handle, float value)
{
    MockRef *ref = handle;
    assert(ref && ref->registered && !ref->read_f);
    ref->value = value;
}
void XPLMSetDatavi(XPLMDataRef handle, int *values, int offset, int count)
{
    MockRef *ref = handle;
    assert(ref && ref->registered && offset >= 0 && offset + count <= 8);
    memcpy(ref->array + offset, values, (size_t)count * sizeof(*values));
}
XPLMDataRef XPLMRegisterDataAccessor(const char *name, XPLMDataTypeID type, int writable,
    XPLMGetDatai_f read_i, XPLMSetDatai_f write_i, XPLMGetDataf_f read_f, XPLMSetDataf_f write_f,
    XPLMGetDatad_f read_d, XPLMSetDatad_f write_d, XPLMGetDatavi_f read_vi, XPLMSetDatavi_f write_vi,
    XPLMGetDatavf_f read_vf, XPLMSetDatavf_f write_vf, XPLMGetDatab_f read_b, XPLMSetDatab_f write_b,
    void *read_refcon, void *write_refcon)
{
    MockRef *ref;
    assert(type == (xplmType_Int|xplmType_Float) && !writable && read_i && read_f);
    assert(!write_i && !write_f && !read_d && !write_d && !read_vi && !write_vi);
    assert(!read_vf && !write_vf && !read_b && !write_b && !write_refcon);
    assert(!XPLMFindDataRef(name));
    ref = new_ref(name);
    ref->read_i = read_i; ref->read_f = read_f; ref->refcon = read_refcon;
    return ref;
}
void XPLMUnregisterDataAccessor(XPLMDataRef handle)
{
    MockRef *ref = handle;
    assert(ref && ref->registered);
    ref->registered = 0;
    ref->read_i = NULL; ref->read_f = NULL; ref->refcon = NULL;
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
    (void)description;
    if (command) return command;
    assert(command_count < ARRAY_LEN(mock_commands));
    command = &mock_commands[command_count++];
    snprintf(command->name, sizeof(command->name), "%s", name);
    return command;
}
void XPLMRegisterCommandHandler(XPLMCommandRef handle, XPLMCommandCallback_f callback,
                               int before, void *refcon)
{
    MockCommand *command = handle;
    assert(command && before && callback && !command->callback);
    command->callback = callback; command->refcon = refcon;
}
void XPLMUnregisterCommandHandler(XPLMCommandRef handle, XPLMCommandCallback_f callback,
                                 int before, void *refcon)
{
    MockCommand *command = handle;
    assert(command && before && command->callback == callback && command->refcon == refcon);
    command->callback = NULL; command->refcon = NULL;
}
float XPLMGetElapsedTime(void) { return mock_time; }
int XPLMCountFMSEntries(void) { return plan_count; }
int XPLMGetDisplayedFMSEntry(void) { return plan_entry; }
void XPLMSetDisplayedFMSEntry(int value) { assert(value >= 0 && value < plan_count); plan_entry = value; }
static int plugin_enabled = 1;
typedef struct {
    XPLMDataRef baro, baro_standard;
    int nd_mode, baro_std, ls_visible, display_valid;
    float nd_range_nm, baro_hpa_value;
} Side;
static struct { Side side[2]; } refs;
static A380XGuidanceState guidance;
static A380XGuidanceInputs guidance_inputs;
static XPLMDataRef find_required(const char *name, int *ok)
{
    XPLMDataRef ref = XPLMFindDataRef(name);
    if (!ref) *ok = 0;
    return ref;
}
void a380x_guidance_command(A380XGuidanceState *s, const A380XGuidanceInputs *inputs,
                           A380XFcuCommand command, double amount)
{
    (void)inputs; (void)amount;
    if (command == A380X_FCU_LS1) s->ls1 = !s->ls1;
    else if (command == A380X_FCU_LS2) s->ls2 = !s->ls2;
    else assert(0);
}
#include "a380x_efis.inc"
static void command(int side, const char *action)
{
    char name[160];
    MockCommand *c;
    snprintf(name, sizeof(name), "a380x/xp12/efis/%s/%s", side ? "first_officer" : "captain", action);
    c = XPLMFindCommand(name);
    assert(c && c->callback);
    c->callback(c, xplm_CommandBegin, c->refcon);
    c->callback(c, xplm_CommandEnd, c->refcon);
}
static void near(float actual, float expected) { assert(fabsf(actual-expected)<0.005f); }
static void setup(void)
{
    int side;
    for (side = 0; side < 2; ++side) {
        char name[128];
        snprintf(name, sizeof(name), "sim/cockpit2/gauges/actuators/barometer_setting_in_hg_%s", side ? "copilot" : "pilot");
        refs.side[side].baro = XPLMFindDataRef(name);
        XPLMSetDataf(refs.side[side].baro, 1013.25f / efis_hpa_per_inhg);
        snprintf(name, sizeof(name), "sim/cockpit2/gauges/actuators/barometer_setting_is_std_%s", side ? "copilot" : "pilot");
        refs.side[side].baro_standard = XPLMFindDataRef(name);
        XPLMSetDatai(refs.side[side].baro_standard, 0);
        refs.side[side].display_valid = 1;
    }
    XPLMSetDatai(XPLMFindDataRef("sim/cockpit2/EFIS/map_mode"), 99);
    XPLMSetDatai(XPLMFindDataRef("sim/cockpit2/EFIS/map_range"), 99);
    XPLMSetDatai(XPLMFindDataRef("sim/cockpit2/EFIS/map_mode_copilot"), 2);
    /* The old table used index 3 for 80 NM; the FBW table uses index 4. */
    XPLMSetDatai(XPLMFindDataRef("sim/cockpit2/EFIS/map_range_copilot"), 3);
    XPLMSetDataf(XPLMFindDataRef("sim/cockpit2/EFIS/map_range_nm"), 80);
    XPLMSetDataf(XPLMFindDataRef("sim/cockpit2/EFIS/map_range_nm_copilot"), 80);
    memset(&guidance, 0, sizeof(guidance));
    guidance_inputs.powered = 1;
    plugin_enabled = 1;
}
static void exercise(void)
{
    float preset;
    int i;
    assert(efis[0].mode == 2 && efis[0].range_index == 4 && refs.side[0].nd_range_nm == 80);
    assert(efis[1].mode == 2 && efis[1].range_index == 4);
    {
        MockRef *steps = XPLMFindDataRef("sim/cockpit2/EFIS/map_range_steps");
        assert(steps->array[0] == 1 && steps->array[4] == 80 && steps->array[7] == 640);
    }
    near(XPLMGetDataf(XPLMFindDataRef("a380x/xp12/efis/captain/range_nm")), 80);
    assert(XPLMGetDatai(XPLMFindDataRef("a380x/xp12/efis/captain/powered")) == 1);
    command(0, "wpt_toggle");
    assert(efis[0].wpt && !efis[1].wpt && XPLMGetDatai(efis_boundary[0].wpt));
    command(1, "vord_toggle");
    assert(efis[1].vord && !efis[0].vord);
    command(0, "mode_up");
    assert(efis[0].mode == 3 && efis[0].range_nm == 40);
    command(0, "mode_up");
    assert(efis[0].mode == 4 && efis[0].range_nm == 80);
    command(0, "mode_down");
    assert(efis[0].mode == 3 && efis[0].range_nm == 40);
    assert(efis[1].mode == 2 && efis[1].range_nm == 80);
    for (i = 0; i < 10; ++i) command(0, "range_down");
    assert(efis[0].range_index == 0 && efis[0].zoom && efis[0].range_nm == 1);
    for (i = 0; i < 10; ++i) command(0, "range_up");
    assert(efis[0].range_index == 7 && !efis[0].zoom && efis[0].range_nm == 640);
    XPLMSetDatai(efis_boundary[0].mode, -1);
    efis_update(.1f);
    assert(efis[0].mode == 2 && efis[0].range_nm == 80);
    XPLMSetDatai(efis_boundary[0].range, 8);
    efis_update(.1f);
    assert(efis[0].mode == 2 && efis[0].range_nm == 80);
    command(0, "baro_up"); near(efis[0].qnh_hpa, 1014);
    command(0, "baro_std"); near(refs.side[0].baro_hpa_value, 1013.25f);
    command(0, "baro_up");
    near(efis[0].baro_preset_hpa, 1015); near(efis[0].qnh_hpa, 1014);
    efis_update(.1f); near(refs.side[0].baro_hpa_value, 1013.25f);
    command(0, "baro_qnh"); near(refs.side[0].baro_hpa_value, 1015);
    near(efis[1].qnh_hpa, 1013.25f);
    command(0, "baro_unit_toggle"); assert(efis[0].baro_inhg && !efis[1].baro_inhg);
    preset = roundf(1015 / efis_hpa_per_inhg * 100) * .01f + .01f;
    command(0, "baro_up"); near(efis[0].qnh_hpa / efis_hpa_per_inhg, preset);
    for (i = 0; i < 400; ++i) command(0, "baro_up");
    near(efis[0].qnh_hpa, 1100);
    for (i = 0; i < 1100; ++i) command(0, "baro_down");
    near(efis[0].qnh_hpa, 745);
    XPLMSetDataf(efis_boundary[0].baro, 1001 / efis_hpa_per_inhg);
    efis_update(.1f); near(efis[0].baro_preset_hpa, 1001);
    XPLMSetDatai(efis_boundary[0].standard, 1);
    XPLMSetDataf(efis_boundary[0].baro, 1013.25f / efis_hpa_per_inhg);
    efis_update(.1f); near(efis[0].baro_preset_hpa, 1001);
    XPLMSetDataf(efis_boundary[0].baro, 1008 / efis_hpa_per_inhg);
    efis_update(.1f); near(efis[0].baro_preset_hpa, 1008); near(refs.side[0].baro_hpa_value, 1013.25f);
    command(0, "baro_qnh"); near(refs.side[0].baro_hpa_value, 1008);
    command(0, "ls_toggle"); assert(efis[0].ls && !efis[1].ls && guidance.ls1);
    command(0, "ls_toggle"); assert(!efis[0].ls);
    guidance.loc_armed = 1; efis_update(.1f); assert(efis[0].ls && efis[1].ls);
    command(0, "ls_toggle"); efis_update(.1f); assert(!efis[0].ls && efis[1].ls);
    guidance.loc_armed = 0; efis_update(.1f);
    guidance.loc_armed = 1; efis_update(.1f); assert(efis[0].ls);
    command(1, "vv_toggle"); assert(efis[1].vv && !efis[0].vv);
    command(0, "wx_toggle"); command(0, "taxi_toggle");
    assert(efis[0].wx_requested && efis[0].taxi_requested && !XPLMGetDatai(efis_boundary[0].wx));
    assert(!efis[1].wx_requested && !efis[1].taxi_requested);
    /* WX and TERR: one ND surveillance overlay (FBW pSurvFilter NONE / WX / TERR). */
    command(0, "terr_toggle");
    assert(efis[0].terr && !efis[0].wx_requested && XPLMGetDatai(efis_boundary[0].terr) && !efis[1].terr);
    command(0, "wx_toggle");
    assert(efis[0].wx_requested && !efis[0].terr && !XPLMGetDatai(efis_boundary[0].terr));
    command(0, "wx_toggle"); assert(!efis[0].wx_requested && !efis[0].terr);
    command(0, "terr_toggle"); command(0, "terr_toggle"); assert(!efis[0].terr && !efis[0].wx_requested);
    command(0, "wx_toggle"); XPLMSetDatai(efis_boundary[0].terr, 1); efis_update(.1f);
    assert(efis[0].terr && !efis[0].wx_requested);   /* stock EFIS_terrain_on replaces WX too */
    efis_update(.1f); assert(efis[0].terr);
    command(0, "terr_toggle"); assert(!efis[0].terr && !XPLMGetDatai(efis_boundary[0].terr));
    command(0, "vor1_toggle"); command(1, "vor2_toggle");
    assert(XPLMGetDatai(efis_boundary[0].vor1) == 2 && XPLMGetDatai(efis_boundary[1].vor1) == 1);
    assert(XPLMGetDatai(efis_boundary[1].vor2) == 2 && XPLMGetDatai(efis_boundary[0].vor2) == 1);
    plan_entry = 0;
    command(0, "plan_previous"); assert(plan_entry == 0);
    for (i = 0; i < 10; ++i) command(0, "plan_next");
    assert(plan_entry == 2);
    command(1, "plan_previous"); assert(plan_entry == 1);
    plan_count = 0; command(0, "plan_next"); assert(plan_entry == 1); plan_count = 3;
    guidance_inputs.powered = 0; efis_update(.1f); assert(!efis[0].powered && !efis[1].powered);
    plugin_enabled = 0; command(0, "vv_toggle"); assert(!efis[0].vv); plugin_enabled = 1;
}
int main(int argc, char **argv)
{
    int cycle, i, first_count = 0;
    if (argc > 1 && !strcmp(argv[1], "--contracts")) {
        setup();
        assert(efis_init());
        for (i = 0; i < command_count; ++i) printf("C %s\n", mock_commands[i].name);
        for (i = 0; i < ref_count; ++i) {
            if (!strncmp(mock_refs[i].name, "a380x/", 6)) printf("D %s\n", mock_refs[i].name);
            else if (!strncmp(mock_refs[i].name, "sim/", 4)) printf("R %s\n", mock_refs[i].name);
        }
        efis_shutdown();
        return 0;
    }
    for (cycle = 0; cycle < 3; ++cycle) {
        setup();
        assert(efis_init());
        if (!cycle) first_count = command_count;
        else assert(command_count == first_count);
        /* Each reload preserves X-Plane's selections, so clear tested layer
         * switches before checking independent button transitions again. */
        for (i = 0; i < 2; ++i) {
            XPLMSetDatai(efis_boundary[i].wpt, 0); XPLMSetDatai(efis_boundary[i].vord, 0);
            XPLMSetDatai(efis_boundary[i].vor1, 1); XPLMSetDatai(efis_boundary[i].vor2, 1);
        }
        efis_update(0);
        exercise();
        efis_shutdown();
        for (i = 0; i < command_count; ++i) assert(!mock_commands[i].callback);
        for (i = 0; i < ref_count; ++i)
            if (!strncmp(mock_refs[i].name, "a380x/", 6)) assert(!mock_refs[i].registered);
    }
    setup();
    XPLMSetDatai(XPLMFindDataRef("sim/cockpit2/EFIS/map_mode_copilot"), 4);
    assert(efis_init());
    assert(efis[1].mode == 4 && efis[1].range_index == 4 && efis[1].range_nm == 80);
    efis_shutdown();
    setup();
    XPLMSetDatai(XPLMFindDataRef("sim/cockpit2/EFIS/map_mode_copilot"), 4);
    XPLMSetDataf(XPLMFindDataRef("sim/cockpit2/EFIS/map_range_nm_copilot"), 5);
    assert(efis_init());
    assert(efis[1].mode == 2 && efis[1].range_index == 4 && efis[1].range_nm == 80);
    efis_shutdown();
    /* Momentary key travel: rises while held, springs back after End; baro shaft push/pull. */
    setup();
    assert(efis_init());
    {
        MockCommand *c = XPLMFindCommand("a380x/xp12/efis/first_officer/vv_toggle");
        int step;
        assert(c && c->callback);
        c->callback(c, xplm_CommandBegin, c->refcon);
        for (step = 0; step < 10; ++step) { mock_time += 0.01f; efis_update(0.01f); }
        assert(efis[1].pressed[EFIS_VV] == 1.0f && efis[0].pressed[EFIS_VV] == 0.0f);
        c->callback(c, xplm_CommandEnd, c->refcon);
        for (step = 0; step < 10; ++step) { mock_time += 0.01f; efis_update(0.01f); }
        assert(efis[1].pressed[EFIS_VV] == 0.0f && efis[1].vv);
        c = XPLMFindCommand("a380x/xp12/efis/captain/baro_qnh");
        c->callback(c, xplm_CommandBegin, c->refcon);
        for (step = 0; step < 10; ++step) { mock_time += 0.01f; efis_update(0.01f); }
        assert(efis[0].pressed[EFIS_PRESS_BARO] == -1.0f);
        c->callback(c, xplm_CommandEnd, c->refcon);
        c = XPLMFindCommand("a380x/xp12/efis/captain/baro_std");
        c->callback(c, xplm_CommandBegin, c->refcon);
        c->callback(c, xplm_CommandEnd, c->refcon);   /* click: full 80 ms press */
        for (step = 0; step < 8; ++step) { mock_time += 0.01f; efis_update(0.01f); }
        assert(efis[0].pressed[EFIS_PRESS_BARO] > 0.95f);
        for (step = 0; step < 12; ++step) { mock_time += 0.01f; efis_update(0.01f); }
        assert(efis[0].pressed[EFIS_PRESS_BARO] == 0.0f);
    }
    efis_shutdown();
    puts("EFIS: momentary key travel, baro push/pull shaft, independent sides, mode/range compensation, STD preset/baro units/limits, LS arming, unsupported selections, PLAN scrolling, reload cleanup passed");
    return 0;
}
