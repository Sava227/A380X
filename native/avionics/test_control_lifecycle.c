/* SPDX-License-Identifier: GPL-3.0-or-later
 * Exercise production control startup/shutdown with persistent XPLM commands,
 * as happens when X-Plane reloads an aircraft. No simulator process is used.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#define ARRAY_LEN(a) ((int)(sizeof(a)/sizeof((a)[0])))

typedef struct {
    char name[160];
    int registered;
    float value, array[20];
    XPLMGetDatai_f read;
    void *refcon;
} MockRef;
typedef struct {
    char name[160];
    XPLMCommandCallback_f callback;
    void *refcon;
} MockCommand;
static MockRef mock_refs[512];
static MockCommand mock_commands[64];
static int ref_count, command_count, native_door_calls;

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
        if (mock_refs[i].registered && !strcmp(mock_refs[i].name, name))
            return &mock_refs[i];
    return !strncmp(name, "sim/", 4) ? new_ref(name) : NULL;
}
int XPLMGetDatai(XPLMDataRef handle)
{
    MockRef *ref = handle;
    assert(ref && ref->registered);
    return ref->read ? ref->read(ref->refcon) : (int)ref->value;
}
float XPLMGetDataf(XPLMDataRef handle)
{
    MockRef *ref = handle;
    assert(ref && ref->registered);
    return ref->value;
}
void XPLMSetDatai(XPLMDataRef handle, int value)
{
    MockRef *ref = handle;
    assert(ref && ref->registered);
    ref->value = (float)value;
}
int XPLMGetDatavf(XPLMDataRef handle, float *values, int offset, int count)
{
    MockRef *ref = handle;
    assert(ref && ref->registered && offset >= 0 && offset + count <= 20);
    memcpy(values, ref->array + offset, (size_t)count * sizeof(*values));
    return count;
}
void XPLMSetDatavf(XPLMDataRef handle, float *values, int offset, int count)
{
    MockRef *ref = handle;
    assert(ref && ref->registered && offset >= 0 && offset + count <= 20);
    memcpy(ref->array + offset, values, (size_t)count * sizeof(*values));
}
int XPLMGetDatavi(XPLMDataRef handle, int *values, int offset, int count)
{
    MockRef *ref = handle;
    int i;
    assert(ref && ref->registered && offset >= 0 && offset + count <= 20);
    for (i = 0; i < count; ++i) values[i] = (int)ref->array[offset + i];
    return count;
}
void XPLMSetDataf(XPLMDataRef handle, float value)
{
    MockRef *ref = handle;
    assert(ref && ref->registered);
    ref->value = value;
}
XPLMDataRef XPLMRegisterDataAccessor(const char *name, XPLMDataTypeID type, int writable,
    XPLMGetDatai_f read_i, XPLMSetDatai_f write_i, XPLMGetDataf_f read_f, XPLMSetDataf_f write_f,
    XPLMGetDatad_f read_d, XPLMSetDatad_f write_d, XPLMGetDatavi_f read_vi, XPLMSetDatavi_f write_vi,
    XPLMGetDatavf_f read_vf, XPLMSetDatavf_f write_vf, XPLMGetDatab_f read_b, XPLMSetDatab_f write_b,
    void *read_refcon, void *write_refcon)
{
    MockRef *ref;
    assert(type == xplmType_Int && !writable && read_i);
    assert(!write_i && !read_f && !write_f && !read_d && !write_d && !read_vi && !write_vi);
    assert(!read_vf && !write_vf && !read_b && !write_b && !write_refcon);
    assert(!XPLMFindDataRef(name));
    ref = new_ref(name);
    ref->read = read_i;
    ref->refcon = read_refcon;
    return ref;
}
void XPLMUnregisterDataAccessor(XPLMDataRef handle)
{
    MockRef *ref = handle;
    assert(ref && ref->registered);
    ref->registered = 0;
    ref->read = NULL;
    ref->refcon = NULL;
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
    command->callback = callback;
    command->refcon = refcon;
}
void XPLMUnregisterCommandHandler(XPLMCommandRef handle, XPLMCommandCallback_f callback,
                                 int before, void *refcon)
{
    MockCommand *command = handle;
    assert(command && before && command->callback == callback && command->refcon == refcon);
    /* The command name survives removal of its plugin's handler. */
    command->callback = NULL;
    command->refcon = NULL;
}
void XPLMCommandOnce(XPLMCommandRef handle)
{
    MockCommand *command = handle;
    assert(command);
    if (!strcmp(command->name, "sim/flight_controls/door_toggle_17")) {
        MockRef *doors = XPLMFindDataRef("sim/flightmodel2/misc/door_open_ratio");
        doors->array[16] = 1.0f - doors->array[16];
        ++native_door_calls;
    } else {
        assert(command->callback);
        command->callback(command, xplm_CommandBegin, command->refcon);
        command->callback(command, xplm_CommandEnd, command->refcon);
    }
}

static int plugin_enabled = 1;
static float gear_values[5] = {1, 1, 1, 1, 1}, flap_value;
static struct { float flash_time; } glare;
static struct {
    XPLMDataRef on_ground, avionics_power;
    struct {
        int radio_altitude_valid, attitude_valid, heading_valid;
        float radio_altitude_value;
    } side[2];
} refs;
#include "a380x_clock.inc"
#include "a380x_center_right.inc"
static int mock_ground_spoilers;
static int mock_ground_spoilers_out(void) { return mock_ground_spoilers; }
#include "a380x_cockpit_door.inc"

static void exercise_clock(void)
{
    MockCommand *button;
    MockRef *battery=XPLMFindDataRef("sim/cockpit2/electrical/battery_voltage_actual_volts");
    MockRef *bus=XPLMFindDataRef("sim/cockpit2/electrical/bus_volts");
    memset(battery->array,0,sizeof(battery->array));
    memset(bus->array,0,sizeof(bus->array));
    battery->array[0]=28;
    XPLMSetDatai(clock_state.sim_speed,1);
    XPLMSetDatai(clock_state.paused,0);
    XPLMSetDatai(clock_state.replay,0);
    XPLMSetDatai(clock_state.zulu_seconds,45296);
    XPLMSetDatai(clock_state.zulu_days,0);
    clock_update(0);
    assert(clock_state.powered && clock_state.hour==12 && clock_state.minute==34
           && clock_state.second==56 && clock_state.month==1 && clock_state.day==1);
    assert(!clock_state.chr_display && !clock_state.et_display);
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/clock/mode_up"));
    assert(XPLMGetDatai(XPLMFindDataRef("a380x/xp12/clock/mode")) == 1);
    button=XPLMFindCommand("a380x/xp12/clock/chr_toggle");
    button->callback(button,xplm_CommandBegin,button->refcon);
    assert(clock_state.chr_held && clock_state.chr_display && clock_state.chr_running);
    button->callback(button,xplm_CommandEnd,button->refcon);
    assert(!clock_state.chr_held);
    clock_update(1.0f);
    assert(XPLMGetDatai(XPLMFindDataRef("a380x/xp12/clock/chr_seconds")) == 1);
    assert(clock_state.second==57);
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/clock/chr_reset"));
    assert(clock_state.chr_seconds==0 && clock_state.chr_running && !clock_state.rst_held);
    clock_update(2.0f);
    XPLMCommandOnce(button);
    clock_update(2.0f);
    assert(clock_state.chr_seconds==2 && !clock_state.chr_running && clock_state.chr_display);
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/clock/chr_reset"));
    assert(clock_state.chr_seconds==2); /* User spec: reset operates while running. */

    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/clock/et_up"));
    clock_update(2.0f);
    assert(clock_state.et_seconds==2 && clock_state.et_display);
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/clock/et_down"));
    clock_update(2.0f);
    assert(clock_state.et_seconds==2 && clock_state.et_selector==1);
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/clock/et_up"));
    clock_update(1.0f);
    assert(clock_state.et_seconds==3);
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/clock/et_down"));
    button=XPLMFindCommand("a380x/xp12/clock/et_down");
    button->callback(button,xplm_CommandBegin,button->refcon);
    assert(clock_state.et_selector==2 && !clock_state.et_display && clock_state.et_seconds==0);
    button->callback(button,xplm_CommandEnd,button->refcon);
    assert(clock_state.et_selector==1 && !clock_state.et_display);

    button=XPLMFindCommand("a380x/xp12/clock/date");
    button->callback(button,xplm_CommandBegin,button->refcon);
    assert(clock_state.date_held);
    button->callback(button,xplm_CommandEnd,button->refcon);
    assert(!clock_state.date_held);
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/clock/mode_up"));
    assert(clock_state.mode==2);
    clock_state.year=2024; clock_state.month=2; clock_state.day=29;
    clock_state.set_field=5;
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/clock/set_up"));
    assert(clock_state.year==2025 && clock_state.day==28);
    clock_state.set_field=2; clock_state.second=59;
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/clock/set_up"));
    assert(clock_state.second==0);
    XPLMCommandOnce(button);
    assert(clock_state.set_field==3);
    clock_state.hour=23; clock_state.minute=59; clock_state.second=59;
    clock_update(2.0f);
    assert(clock_state.second==59); /* SET freezes UTC, but not independent counters. */
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/clock/mode_down"));
    clock_update(1.0f);
    assert(clock_state.month==3 && clock_state.day==1 && clock_state.hour==0);

    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/clock/chr_toggle"));
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/clock/et_up"));
    clock_state.chr_seconds=5999; clock_state.et_seconds=359940;
    clock_update(1.0f);
    assert(clock_state.chr_seconds==0 && clock_state.et_seconds==359940);
    XPLMSetDatai(clock_state.paused,1);
    clock_update(2.0f);
    assert(clock_state.chr_seconds==0);
    XPLMSetDatai(clock_state.paused,0);
    XPLMSetDatai(clock_state.replay,1);
    clock_update(2.0f);
    assert(clock_state.chr_seconds==0);
    XPLMSetDatai(clock_state.replay,0);
    XPLMSetDatai(clock_state.sim_speed,2);
    clock_update(1.0f);
    assert(clock_state.chr_seconds==2);
    clock_update(NAN);
    assert(clock_state.chr_seconds==2 && isfinite(clock_state.chr_fraction));
    XPLMSetDatai(clock_state.sim_speed,1);

    battery->array[0]=0; bus->array[1]=28;
    clock_update(1.0f);
    assert(clock_state.powered && clock_state.chr_seconds==3); /* Redundant supply. */
    bus->array[1]=0;
    clock_update(1.0f);
    assert(!clock_state.powered && !clock_state.chr_display && !clock_state.et_display);
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/clock/chr_toggle"));
    assert(!clock_state.chr_running);
    battery->array[0]=28;
    clock_update(0.0f);
    assert(clock_state.powered);
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/clock/mode_down"));
    assert(clock_state.hour==12 && clock_state.minute==34 && clock_state.second==56);
    clock_state.chr_held=clock_state.rst_held=clock_state.date_held=1;
    clock_state.et_selector=2;
    clock_release_controls();
    assert(!clock_state.chr_held && !clock_state.rst_held && !clock_state.date_held);
    assert(clock_state.et_selector==1);
}
static void exercise_center_right(void)
{
    int i;
    MockCommand *rto=XPLMFindCommand("a380x/xp12/center_right/rto_toggle");
    MockCommand *reinflate=XPLMFindCommand("a380x/xp12/center_right/accus_reinflate");
    ((MockRef *)center_right.hydraulic_ref[0])->value=3000.0f;
    ((MockRef *)center_right.hydraulic_ref[1])->value=3000.0f;
    ((MockRef *)center_right.pedal_ref[0])->value=0.0f;
    ((MockRef *)center_right.pedal_ref[1])->value=0.0f;
    ((MockRef *)center_right.spoiler_ref)->value=0.0f;
    ((MockRef *)center_right.brake_failure_ref[0])->value=0.0f;
    ((MockRef *)center_right.brake_failure_ref[1])->value=0.0f;
    XPLMSetDatai(refs.avionics_power,1);
    refs.side[0].attitude_valid=refs.side[0].heading_valid=1;
    assert(fabsf(((MockRef *)center_right.decel_ref)->array[0]-2.0f/9.80665f)<0.0001f);
    assert(fabsf(((MockRef *)center_right.decel_ref)->array[3]-3.5f/9.80665f)<0.0001f);
    XPLMSetDatai(center_right.lever_ref, 1);
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/gear_toggle"));
    center_right_update(0.0f);
    assert(XPLMGetDatai(XPLMFindDataRef("a380x/xp12/center_right/gear_lever")) == 0);
    assert(center_right.gear_wing == 2 && center_right.gear_body == 2);
    for (i=0;i<4;++i) center_right_update(5.0f);
    assert(center_right.gear_not_locked_up_warning);
    assert(!center_right.gear_not_locked_down_warning);
    refs.side[0].radio_altitude_valid=1;
    refs.side[0].radio_altitude_value=500.0f;
    flap_value=0.75f;
    gear_values[2]=0.5f;
    glare.flash_time=0.25f;
    center_right_update(0.0f);
    assert(center_right.gear_not_down_warning && center_right.gear_arrow);
    /* The landing arrow does not depend on the selected lever position. */
    XPLMSetDatai(center_right.lever_ref,1);
    center_right_update(0.0f);
    assert(center_right.gear_not_down_warning && center_right.gear_arrow);
    flap_value=0.0f;
    gear_values[2]=0.5f;
    for (i=0;i<4;++i) center_right_update(5.0f);
    assert(center_right.gear_wing == 2 && center_right.gear_not_locked_down_warning);
    assert(center_right.gear_wing_left_green && !center_right.gear_wing_left_red);
    assert(center_right.gear_wing_right_red && !center_right.gear_wing_right_green);
    gear_values[2]=1.0f;
    center_right_update(0.0f);
    assert(!center_right.gear_not_locked_down_warning && center_right.gear_wing==1);
    ((MockRef *)center_right.accumulator_ref)->value=0.75f;
    center_right_update(0.0f);
    assert(center_right.body_accu_psi==3750 && center_right.body_accu_zone==0);
    ((MockRef *)center_right.accumulator_ref)->value=3840.0f/5000.0f;
    center_right_update(0.0f);
    assert(center_right.body_accu_psi==3840 && center_right.body_accu_zone==0);
    ((MockRef *)center_right.accumulator_ref)->value=4000.0f/5000.0f;
    center_right_update(0.0f);
    assert(center_right.body_accu_zone==1);
    ((MockRef *)center_right.accumulator_ref)->value=4872.0f/5000.0f;
    center_right_update(0.0f);
    assert(center_right.body_accu_psi==4872 && center_right.body_accu_zone==2);
    ((MockRef *)center_right.accumulator_ref)->value=0.98f;
    center_right_update(0.0f);
    assert(center_right.body_accu_psi==4900 && center_right.body_accu_zone==2);
    /* BCU autobrake (a380x_autobrake.h; its logic is tested in test_autobrake.c): the adapter wiring. */
    center_right_ground_spoilers=mock_ground_spoilers_out;
    for (i=0;i<5;++i) ((MockRef *)center_right.gear_ground_ref)->array[i]=1.0f;
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_up"));
    /* BTV: no OANS exit in the port, so not armed; the selector springs back after 1 s. */
    assert(center_right.autobrake_selected==1 && center_right.autobrake_armed==0 && !center_right.btv_reverted);
    assert(XPLMGetDatai(center_right.autobrake_ref)==1);
    center_right_update(0.6f); center_right_update(0.6f);
    assert(center_right.autobrake_selected==0);
    /* X-Plane's own autobrake switch is read as the selector and returned to OFF. */
    XPLMSetDatai(center_right.autobrake_ref,3);
    center_right_update(0.0f);
    assert(center_right.autobrake_selected==3 && center_right.autobrake_armed==3 && XPLMGetDatai(center_right.autobrake_ref)==1);
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_down"));
    assert(center_right.autobrake_selected==2 && center_right.autobrake_armed==2);
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_down"));
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_down"));
    assert(center_right.autobrake_selected==0 && center_right.autobrake_armed==0);
    XPLMSetDatai(center_right.autobrake_ref,0);           /* X-Plane RTO switch = RTO pushbutton */
    center_right_update(0.0f);
    assert(center_right.rto_arm && center_right.autobrake_armed==6 && XPLMGetDatai(center_right.autobrake_ref)==1);
    XPLMCommandOnce(rto);
    assert(!center_right.rto_arm);
    XPLMCommandOnce(rto);
    assert(center_right.rto_arm);
    /* RTO is rejected 10 s after lift-off. */
    for (i=0;i<5;++i) ((MockRef *)center_right.gear_ground_ref)->array[i]=0.0f;
    for (i=0;i<19;++i) center_right_update(0.5f);
    assert(center_right.rto_arm);
    center_right_update(0.5f); center_right_update(0.5f);
    assert(!center_right.rto_arm);
    /* In flight (more than 10 s): landing modes arm, RTO does not. */
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_up"));
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_up"));
    assert(center_right.autobrake_armed==2);
    XPLMCommandOnce(rto);
    assert(!center_right.rto_arm && center_right.autobrake_armed==2);
    for (i=0;i<5;++i) ((MockRef *)center_right.gear_ground_ref)->array[i]=1.0f;
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_down"));
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_down"));
    assert(center_right.autobrake_armed==0);
    /* Held RTO animation follows command phase, independent of arm state. */
    rto->callback(rto,xplm_CommandBegin,rto->refcon);
    assert(center_right.rto_pressed && center_right.rto_arm);
    rto->callback(rto,xplm_CommandEnd,rto->refcon);
    assert(!center_right.rto_pressed && center_right.rto_arm);
    XPLMCommandOnce(rto);
    assert(!center_right.rto_arm);
    /* Anti-skid off: no arming (the selector springs back), no RTO. */
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_up"));
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_up"));
    assert(center_right.autobrake_armed==2);
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/antiskid_toggle"));
    assert(XPLMGetDatai(center_right.abs_ref) == 0);
    assert(center_right.emergency_braking && !center_right.autobrake_available);
    assert(!center_right.antiskid_available && !center_right.autobrake_armed);
    center_right_update(0.6f); center_right_update(0.6f);
    assert(!center_right.autobrake_selected);
    XPLMCommandOnce(rto);
    assert(!center_right.rto_arm);
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/antiskid_toggle"));
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_up"));
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_up"));
    assert(center_right.autobrake_armed==2);
    /* Loss of both hydraulic feeds springs the selector back to DISARM. */
    ((MockRef *)center_right.hydraulic_ref[0])->value=1200.0f;
    ((MockRef *)center_right.hydraulic_ref[1])->value=1200.0f;
    center_right_update(0.0f);
    assert(!center_right.normal_brakes_available && !center_right.autobrake_armed);
    center_right_update(0.6f); center_right_update(0.6f);
    assert(!center_right.autobrake_selected);
    ((MockRef *)center_right.hydraulic_ref[0])->value=1800.0f;
    center_right_update(0.0f);
    assert(!center_right.normal_brakes_available);
    ((MockRef *)center_right.hydraulic_ref[0])->value=3000.0f;
    center_right_update(0.0f);
    assert(center_right.normal_brakes_available);
    XPLMCommandOnce(rto);
    XPLMSetDatai(center_right.brake_failure_ref[1],6);
    center_right_update(0.0f);
    assert(!center_right.rto_arm && !center_right.autobrake_available);
    XPLMSetDatai(center_right.brake_failure_ref[1],0);
    center_right_update(0.0f);
    XPLMCommandOnce(rto);
    refs.side[0].attitude_valid=0;
    center_right_update(0.0f);
    assert(!center_right.rto_arm && !center_right.autobrake_available);
    refs.side[0].attitude_valid=1;
    center_right_update(0.0f);
    XPLMCommandOnce(rto);
    XPLMSetDatai(refs.avionics_power,0);
    center_right_update(0.0f);
    assert(!center_right.rto_arm && !center_right.autobrake_available);
    XPLMSetDatai(refs.avionics_power,1);
    /* Reinflation operates the native pump until observed charge completes,
     * never changes the pressure sample and preserves a pre-existing ON. */
    ((MockRef *)center_right.accumulator_ref)->value=0.4f;
    XPLMSetDatai(center_right.electric_pump_ref,0);
    reinflate->callback(reinflate,xplm_CommandBegin,reinflate->refcon);
    assert(center_right.reinflate_pressed && center_right.reinflate_active);
    assert(XPLMGetDatai(center_right.electric_pump_ref)==1 && center_right.body_accu_psi==2000);
    reinflate->callback(reinflate,xplm_CommandEnd,reinflate->refcon);
    assert(!center_right.reinflate_pressed && center_right.reinflate_active);
    ((MockRef *)center_right.accumulator_ref)->value=1.0f;
    center_right_update(0.0f);
    assert(!center_right.reinflate_active && !XPLMGetDatai(center_right.electric_pump_ref));
    XPLMSetDatai(center_right.electric_pump_ref,1);
    ((MockRef *)center_right.accumulator_ref)->value=0.4f;
    XPLMCommandOnce(reinflate);
    ((MockRef *)center_right.accumulator_ref)->value=1.0f;
    center_right_update(0.0f);
    assert(!center_right.reinflate_active && XPLMGetDatai(center_right.electric_pump_ref));
    XPLMSetDatai(center_right.electric_pump_ref,0);
    /* Body-wheel brake needles follow X-Plane's applied braking. An engaged landing mode brakes through
     * l/r_brake_add and disarms on pedal override or ground spoiler retraction. */
    ((MockRef *)center_right.wheel_brake_ref)->array[3]=0.4f;
    ((MockRef *)center_right.wheel_brake_ref)->array[4]=0.7f;
    center_right_update(0.0f);
    assert(center_right.brake_left_ratio_milli==400 && center_right.brake_right_ratio_milli==700);
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_up"));
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_up"));
    assert(center_right.autobrake_armed==2 && !center_right.autobrake_active);
    mock_ground_spoilers=1;
    for (i=0;i<5;++i) center_right_update(0.05f);
    assert(center_right.autobrake_active && XPLMGetDataf(center_right.brake_add_ref[0])>0.0f);
    assert(XPLMGetDataf(center_right.brake_add_ref[1])==XPLMGetDataf(center_right.brake_add_ref[0]));
    ((MockRef *)center_right.pedal_ref[0])->value=0.6f;
    center_right_update(0.02f);
    assert(!center_right.autobrake_active && !center_right.autobrake_armed);
    assert(XPLMGetDataf(center_right.brake_add_ref[0])==0.0f && XPLMGetDataf(center_right.brake_add_ref[1])==0.0f);
    ((MockRef *)center_right.pedal_ref[0])->value=0.0f;
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_down"));
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/center_right/autobrake_up"));
    for (i=0;i<5;++i) center_right_update(0.05f);
    assert(center_right.autobrake_active);
    mock_ground_spoilers=0;
    center_right_update(0.02f);
    assert(!center_right.autobrake_active && !center_right.autobrake_armed && XPLMGetDataf(center_right.brake_add_ref[0])==0.0f);
    /* RTO keeps its higher pedal override threshold. */
    XPLMCommandOnce(rto);
    mock_ground_spoilers=1;
    center_right_update(0.05f);
    assert(center_right.autobrake_active && center_right.rto_arm);
    ((MockRef *)center_right.pedal_ref[0])->value=0.6f;
    center_right_update(0.02f);
    assert(center_right.rto_arm);
    ((MockRef *)center_right.pedal_ref[0])->value=0.8f;
    center_right_update(0.02f);
    assert(!center_right.rto_arm && XPLMGetDataf(center_right.brake_add_ref[0])==0.0f);
    ((MockRef *)center_right.pedal_ref[0])->value=0.0f;
    mock_ground_spoilers=0;
    ((MockRef *)center_right.spoiler_ref)->value=0.0f;
    memset(((MockRef *)center_right.wheel_brake_ref)->array,0,sizeof(((MockRef *)center_right.wheel_brake_ref)->array));
    ((MockRef *)center_right.accumulator_ref)->value=0.4f;
    reinflate->callback(reinflate,xplm_CommandBegin,reinflate->refcon);
    assert(center_right.reinflate_active && center_right.reinflate_pressed);
    center_right_release_controls();
    assert(!center_right.reinflate_active && !center_right.reinflate_pressed);
    assert(!XPLMGetDatai(center_right.electric_pump_ref));
    /* Leave a request active for shutdown to exercise owned-pump cleanup. */
    XPLMCommandOnce(reinflate);
}
static void exercise_cockpit_door(void)
{
    MockRef *doors = cockpit_door.doors;
    int before = native_door_calls;
    doors->array[16] = 0;
    XPLMCommandOnce(XPLMFindCommand("a380x/xp12/cockpit_door/unlock"));
    assert(native_door_calls == before + 1 && doors->array[16] == 1.0f);
    assert(XPLMGetDatai(cockpit_door.position_ref) == 0);
    cockpit_door_update();
    assert(XPLMGetDatai(cockpit_door.locked_ref) == 0);
}
static void test_reload(int (*init)(void), void (*shutdown)(void), void (*exercise)(void))
{
    int cycle, i, first_command_count = 0;
    for (cycle = 0; cycle < 3; ++cycle) {
        assert(init());
        if (cycle == 0) first_command_count = command_count;
        else assert(command_count == first_command_count);
        exercise();
        shutdown();
        for (i = 0; i < command_count; ++i) assert(!mock_commands[i].callback);
        for (i = 0; i < ref_count; ++i)
            if (!strncmp(mock_refs[i].name, "a380x/", 6)) assert(!mock_refs[i].registered);
        assert(XPLMGetDatai(XPLMFindDataRef("sim/aircraft/gear/acf_has_abs")) == 1);
        if (init==center_right_init) {
            assert(!XPLMGetDatai(center_right.electric_pump_ref));
            for (i=0;i<4;++i)
                assert(((MockRef *)center_right.decel_ref)->array[i]==center_right.original_decels[i]);
        }
    }
}
int main(int argc, char **argv)
{
    refs.avionics_power = XPLMFindDataRef("sim/cockpit2/switches/avionics_power_on");
    XPLMSetDatai(refs.avionics_power,1);
    refs.side[0].attitude_valid=refs.side[0].heading_valid=1;
    ((MockRef *)XPLMFindDataRef("sim/cockpit2/electrical/battery_voltage_actual_volts"))->array[0]=28.0f;
    ((MockRef *)XPLMFindDataRef("sim/time/sim_speed_actual"))->value=1.0f;
    refs.on_ground = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    XPLMSetDatai(refs.on_ground, 1);
    XPLMSetDatai(XPLMFindDataRef("sim/aircraft/gear/acf_has_abs"), 1);
    XPLMCreateCommand("sim/flight_controls/door_toggle_17", "Native door toggle");
    if (argc < 2 || !strcmp(argv[1], "clock"))
        test_reload(clock_init, clock_shutdown, exercise_clock);
    if (argc < 2 || !strcmp(argv[1], "center_right"))
        test_reload(center_right_init, center_right_shutdown, exercise_center_right);
    if (argc < 2 || !strcmp(argv[1], "cockpit_door"))
        test_reload(cockpit_door_init, cockpit_door_shutdown, exercise_cockpit_door);
    puts("Controls: clock, center-right and cockpit-door reloads preserve commands, reattach one handler and clean up datarefs");
    return 0;
}
