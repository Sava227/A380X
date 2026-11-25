/* SPDX-License-Identifier: GPL-3.0-or-later
 * Center-right gear/brake controls backed by X-Plane's landing-gear and anti-skid systems.
 * Autobrake: the A380 BCU autobrake core (a380x_autobrake.h, FBW A380AutobrakeController) brakes through
 * sim/flightmodel/controls/l_brake_add / r_brake_add, so the crew's pedals stay readable for disarming; X-Plane's
 * own autobrake is kept off (its switch, written by hardware or X-Plane commands, is read as the selector / RTO
 * pushbutton).
 */
#include "a380x_elec_power.h"
#include "a380x_hyd_view.h"
#include "a380x_autobrake.h"
typedef struct {
    int gear_lever, gear_nose, gear_wing, gear_body, gear_arrow;
    int gear_not_down_warning;
    int gear_not_locked_down_warning, gear_not_locked_up_warning;
    int gear_nose_green, gear_nose_red, gear_wing_green, gear_wing_red;
    int gear_body_green, gear_body_red;
    int gear_wing_left_green, gear_wing_left_red;
    int gear_wing_right_green, gear_wing_right_red;
    int gear_body_left_green, gear_body_left_red;
    int gear_body_right_green, gear_body_right_red;
    int body_accu_psi, body_accu_zone;
    int autobrake_selected, autobrake_armed, rto_arm, antiskid_on;
    int rto_pressed, btv_reverted;
    int normal_brakes_available, autobrake_available, antiskid_available;
    int emergency_braking, autobrake_active;
    int reinflate_pressed, reinflate_active, reinflate_owns_pump;
    int brake_left_ratio_milli, brake_right_ratio_milli;
    int autobrake_was_active, decel_owned;
    float original_decels[4];
    int abs_initial;
    int previous_gear_lever;
    /* L/G GRVTY (a380x_lg_grvty.inc): while gravity extension isolates the gear hydraulics the
     * plugin holds X-Plane's gear handle and the physical lever is owned_lever_down. */
    int lever_owned, owned_lever_down;
    float gear_selection_elapsed;
    XPLMDataRef lever_ref, autobrake_ref, abs_ref, accumulator_ref;
    XPLMDataRef hydraulic_ref[2], pedal_ref[2], brake_failure_ref[2];
    XPLMDataRef electric_pump_ref, wheel_brake_ref, spoiler_ref, park_ref, decel_ref;
    /* BCU autobrake */
    A380XAutobrake ab;
    int autobrake_decel;
    int ab_knob_moved, ab_rto_edge, ab_athr_edge, brake_add_owned;
    XPLMDataRef brake_add_ref[2], gear_ground_ref, local_v_ref[2], local_a_ref[2];
    XPLMCommandRef athr_cmd;
} CenterRightState;
static CenterRightState center_right;

typedef struct { const char *name; int *value; XPLMDataRef handle; } CenterRightRef;
#define CR_REF(name, field) {"a380x/xp12/center_right/" name, &center_right.field, NULL}
static CenterRightRef center_right_refs[] = {
    CR_REF("gear_lever",gear_lever), CR_REF("gear_nose",gear_nose),
    CR_REF("gear_wing",gear_wing), CR_REF("gear_body",gear_body),
    CR_REF("gear_arrow",gear_arrow), CR_REF("body_accu_psi",body_accu_psi),
    CR_REF("gear_not_down_warning",gear_not_down_warning),
    CR_REF("gear_not_locked_down_warning",gear_not_locked_down_warning),
    CR_REF("gear_not_locked_up_warning",gear_not_locked_up_warning),
    CR_REF("gear_nose_green",gear_nose_green), CR_REF("gear_nose_red",gear_nose_red),
    CR_REF("gear_wing_green",gear_wing_green), CR_REF("gear_wing_red",gear_wing_red),
    CR_REF("gear_body_green",gear_body_green), CR_REF("gear_body_red",gear_body_red),
    CR_REF("gear_wing_left_green",gear_wing_left_green),
    CR_REF("gear_wing_left_red",gear_wing_left_red),
    CR_REF("gear_wing_right_green",gear_wing_right_green),
    CR_REF("gear_wing_right_red",gear_wing_right_red),
    CR_REF("gear_body_left_green",gear_body_left_green),
    CR_REF("gear_body_left_red",gear_body_left_red),
    CR_REF("gear_body_right_green",gear_body_right_green),
    CR_REF("gear_body_right_red",gear_body_right_red),
    CR_REF("body_accu_zone",body_accu_zone),
    CR_REF("autobrake_selected",autobrake_selected),
    CR_REF("autobrake_armed",autobrake_armed), CR_REF("rto_arm",rto_arm),
    CR_REF("antiskid_on",antiskid_on),
    CR_REF("rto_pressed",rto_pressed), CR_REF("btv_reverted",btv_reverted),
    CR_REF("normal_brakes_available",normal_brakes_available),
    CR_REF("autobrake_available",autobrake_available),
    CR_REF("antiskid_available",antiskid_available),
    CR_REF("emergency_braking",emergency_braking),
    CR_REF("autobrake_active",autobrake_active),
    CR_REF("autobrake_decel",autobrake_decel),
    CR_REF("brake_left_ratio_milli",brake_left_ratio_milli),
    CR_REF("brake_right_ratio_milli",brake_right_ratio_milli),
    CR_REF("reinflate_pressed",reinflate_pressed),
    CR_REF("reinflate_active",reinflate_active),
};
#undef CR_REF
static int center_right_get(void *refcon)
{
    return *(((CenterRightRef *)refcon)->value);
}

typedef struct { const char *name; int action; XPLMCommandRef handle; } CenterRightCommand;
enum { CR_GEAR, CR_AUTOBRAKE_UP, CR_AUTOBRAKE_DOWN, CR_RTO, CR_ANTISKID, CR_REINFLATE };
#define CR_CMD(name, action) {"a380x/xp12/center_right/" name, action, NULL}
static CenterRightCommand center_right_commands[] = {
    CR_CMD("gear_toggle",CR_GEAR),
    CR_CMD("autobrake_up",CR_AUTOBRAKE_UP),
    CR_CMD("autobrake_down",CR_AUTOBRAKE_DOWN),
    CR_CMD("rto_toggle",CR_RTO),
    CR_CMD("antiskid_toggle",CR_ANTISKID),
    CR_CMD("accus_reinflate",CR_REINFLATE),
};
#undef CR_CMD

static float center_right_ratio(float value)
{
    return isfinite(value) ? fmaxf(0.0f,fminf(1.0f,value)) : 0.0f;
}
static void center_right_update(float dt);
/* PRIM ground spoiler extension: plugged in by a380x_speedbrake.inc at init (NULL: none, e.g. in unit tests). */
#define A380X_HAVE_CENTER_RIGHT 1
static int (*center_right_ground_spoilers)(void);
/* Longitudinal acceleration along the ground track (m/s^2, negative when slowing). */
static double center_right_long_accel(void)
{
    float vx=XPLMGetDataf(center_right.local_v_ref[0]), vz=XPLMGetDataf(center_right.local_v_ref[1]);
    float ax=XPLMGetDataf(center_right.local_a_ref[0]), az=XPLMGetDataf(center_right.local_a_ref[1]);
    double speed=sqrt((double)vx*vx+(double)vz*vz);
    if (!isfinite(speed) || speed<0.5 || !isfinite(ax) || !isfinite(az)) return 0.0;
    return ((double)vx*ax+(double)vz*az)/speed;
}
static void center_right_brake_add(float value)
{
    XPLMSetDataf(center_right.brake_add_ref[0],value);
    XPLMSetDataf(center_right.brake_add_ref[1],value);
}
/* One BCU autobrake step (dt 0 from a click: edges only). */
static void center_right_autobrake_step(float dt, const float *pedal)
{
    A380XAutobrakeInputs in;
    int ground[5]={0,0,0,0,0}, i, main_gear=0;
    memset(&in,0,sizeof(in));
    XPLMGetDatavi(center_right.gear_ground_ref,ground,0,5);
    for (i=1;i<5;++i) main_gear=main_gear||ground[i]!=0;
    in.knob=center_right.autobrake_selected;
    in.knob_moved=center_right.ab_knob_moved;
    in.rto_pressed=center_right.ab_rto_edge;
    in.arming_allowed=center_right.autobrake_available;
    in.ground_spoilers_out=center_right_ground_spoilers ? center_right_ground_spoilers() : 0;
    in.nose_compressed=ground[0]!=0;
    in.main_compressed=main_gear;
    in.pedal_left=pedal[0];
    in.pedal_right=pedal[1];
    in.athr_disconnect=center_right.ab_athr_edge;
    in.long_accel=center_right_long_accel();
    in.btv_armable=0;      /* no OANS runway-exit selection in the port yet */
    a380x_ab_step(&center_right.ab,&in,isfinite(dt) && dt>0.0f ? (double)fminf(dt,0.5f) : 0.0);
    center_right.ab_knob_moved=center_right.ab_rto_edge=center_right.ab_athr_edge=0;
    if (center_right.ab.knob_return) center_right.autobrake_selected=0;   /* selector solenoid */
    center_right.autobrake_armed=center_right.ab.mode;
    center_right.rto_arm=center_right.ab.mode==A380X_AB_RTO;
    center_right.autobrake_active=a380x_ab_active(&center_right.ab);
    center_right.autobrake_decel=center_right.ab.decel_light;
    center_right.btv_reverted=0;
    if (center_right.ab.brake>0.0) {
        center_right_brake_add((float)center_right.ab.brake);
        center_right.brake_add_owned=1;
    } else if (center_right.brake_add_owned) {
        center_right_brake_add(0.0f);
        center_right.brake_add_owned=0;
    }
}
/* A/THR instinctive disconnect pushbutton: also disarms a braking autobrake (not RTO). Pass-through. */
static int center_right_athr_handler(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    (void)command; (void)refcon;
    if (phase==xplm_CommandBegin) center_right.ab_athr_edge=1;
    return 1;
}
static int center_right_command_handler(XPLMCommandRef command,
                                         XPLMCommandPhase phase, void *refcon)
{
    CenterRightCommand *item=(CenterRightCommand *)refcon;
    (void)command;
    if (phase==xplm_CommandEnd) {
        if (item->action==CR_RTO) center_right.rto_pressed=0;
        if (item->action==CR_REINFLATE) center_right.reinflate_pressed=0;
        return 1;
    }
    if (!plugin_enabled || phase!=xplm_CommandBegin) return 1;
    /* Refresh conditions at the click, rather than accepting a stale armed
     * state between flight-loop callbacks. */
    center_right_update(0.0f);
    switch (item->action) {
    case CR_GEAR:
        /* Hydraulics isolated by L/G GRVTY: the lever moves without commanding the gear. */
        if (center_right.lever_owned) center_right.owned_lever_down=!center_right.owned_lever_down;
        else XPLMSetDatai(center_right.lever_ref,!XPLMGetDatai(center_right.lever_ref));
        break;
    case CR_AUTOBRAKE_UP: case CR_AUTOBRAKE_DOWN:
        /* The selector always turns; the BCU decides (an unarmable mode springs back to DISARM after 1 s). */
        center_right.autobrake_selected+=item->action==CR_AUTOBRAKE_UP ? 1 : -1;
        if (center_right.autobrake_selected<0) center_right.autobrake_selected=0;
        if (center_right.autobrake_selected>5) center_right.autobrake_selected=5;
        center_right.ab_knob_moved=1;
        break;
    case CR_RTO:
        center_right.rto_pressed=1;
        center_right.ab_rto_edge=1;   /* arms RTO, or disarms it when armed */
        break;
    case CR_ANTISKID:
        XPLMSetDatai(center_right.abs_ref,!XPLMGetDatai(center_right.abs_ref));
        break;
    case CR_REINFLATE:
        center_right.reinflate_pressed=1;
        /* The port has one native accumulator. Request its configured
         * electric pump and let X-Plane charge it; never write fake pressure.
         * A pump that was already on is left on at completion. */
        if (center_right.body_accu_psi<4872 && !center_right.reinflate_active) {
            center_right.reinflate_owns_pump=!XPLMGetDatai(center_right.electric_pump_ref);
            XPLMSetDatai(center_right.electric_pump_ref,1);
            center_right.reinflate_active=1;
        }
        break;
    default: break;
    }
    center_right_update(0.0f);
    return 1;
}
static int center_right_gear_state(int first, int last)
{
    int i, down=1, up=1;
    for (i=first;i<=last;++i) {
        down=down && gear_values[i]>=0.99f;
        up=up && gear_values[i]<=0.01f;
    }
    if (center_right.gear_lever && down) return 1;
    if (!center_right.gear_lever && up) return 0;
    return 2;
}
static void center_right_update(float dt)
{
    float pressure=XPLMGetDataf(center_right.accumulator_ref);
    float hydraulic_pressure=0.0f, pedal[2], wheel_brakes[5]={0};
    int native_mode, inertial_available;
    int all_down=1, all_up=1, below_750, i;
    center_right.gear_lever=center_right.lever_owned ? center_right.owned_lever_down!=0
                                                     : XPLMGetDatai(center_right.lever_ref)!=0;
    if (center_right.gear_lever!=center_right.previous_gear_lever) {
        center_right.previous_gear_lever=center_right.gear_lever;
        center_right.gear_selection_elapsed=0.0f;
    } else if (isfinite(dt) && dt>0.0f) {
        center_right.gear_selection_elapsed+=fminf(dt,5.0f);
    }
    for (i=0;i<5;++i) {
        if (!isfinite(gear_values[i]) || gear_values[i]<0.99f) all_down=0;
        if (!isfinite(gear_values[i]) || gear_values[i]>0.01f) all_up=0;
    }
    center_right.gear_nose=center_right_gear_state(0,0);
    center_right.gear_wing=center_right_gear_state(1,2);
    center_right.gear_body=center_right_gear_state(3,4);
    center_right.gear_nose_green=center_right.gear_nose==1;
    center_right.gear_nose_red=center_right.gear_nose==2;
    center_right.gear_wing_green=center_right.gear_wing==1;
    center_right.gear_wing_red=center_right.gear_wing==2;
    center_right.gear_body_green=center_right.gear_body==1;
    center_right.gear_body_red=center_right.gear_body==2;
    center_right.gear_wing_left_green=center_right_gear_state(1,1)==1;
    center_right.gear_wing_left_red=center_right_gear_state(1,1)==2;
    center_right.gear_wing_right_green=center_right_gear_state(2,2)==1;
    center_right.gear_wing_right_red=center_right_gear_state(2,2)==2;
    center_right.gear_body_left_green=center_right_gear_state(3,3)==1;
    center_right.gear_body_left_red=center_right_gear_state(3,3)==2;
    center_right.gear_body_right_green=center_right_gear_state(4,4)==1;
    center_right.gear_body_right_red=center_right_gear_state(4,4)==2;
    /* The package ACF specifies a 20-second gear cycle for all five gear.
     * Red UNLK is immediate; the ECAM lock alert follows that normal delay. */
    center_right.gear_not_locked_down_warning=center_right.gear_lever &&
        !all_down && center_right.gear_selection_elapsed>=20.0f;
    center_right.gear_not_locked_up_warning=!center_right.gear_lever &&
        !all_up && center_right.gear_selection_elapsed>=20.0f;
    below_750=(refs.side[0].radio_altitude_valid &&
        refs.side[0].radio_altitude_value<750.0f) ||
        (refs.side[1].radio_altitude_valid &&
        refs.side[1].radio_altitude_value<750.0f);
    center_right.gear_not_down_warning=flap_value>=0.5f && below_750 && !all_down;
    center_right.gear_arrow=center_right.gear_not_down_warning &&
        fmodf(glare.flash_time,1.0f)<0.5f;
    if (!isfinite(pressure)) pressure=0.0f;
    center_right.body_accu_psi=(int)lroundf(fmaxf(0.0f,fminf(1.0f,pressure))*5000.0f);
    center_right.body_accu_zone=center_right.body_accu_psi>=4872 ? 2 :
        center_right.body_accu_psi<=3840 ? 0 : 1;
    if (center_right.reinflate_active &&
        (center_right.body_accu_psi>=4872 || !XPLMGetDatai(center_right.electric_pump_ref))) {
        if (center_right.reinflate_owns_pump)
            XPLMSetDatai(center_right.electric_pump_ref,0);
        center_right.reinflate_active=0;
        center_right.reinflate_owns_pump=0;
    }
    for (i=0;i<2;++i) {
        float value=XPLMGetDataf(center_right.hydraulic_ref[i]);
        if (isfinite(value)) hydraulic_pressure=fmaxf(hydraulic_pressure,value);
        pedal[i]=center_right_ratio(XPLMGetDataf(center_right.pedal_ref[i]));
    }
    /* A380 hydraulic model (a380x_hyd.inc): system pressures in psi, FBW's units. */
    if (a380x_hyd_view.available)
        hydraulic_pressure=(float)fmax(a380x_hyd_view.psi[0],a380x_hyd_view.psi[1]);
    /* FBW A380 BCU normal-brake hysteresis (hydraulic/mod.rs). The native
     * scaffold uses 3000-PSI engine systems 1/2, so its pressure units already
     * match these thresholds. The full three-ADIRS alignment computer is not
     * ported: a powered, valid raw attitude/heading channel is used here. */
    if (hydraulic_pressure>2176.0f && pedal[0]<0.2f && pedal[1]<0.2f)
        center_right.normal_brakes_available=1;
    else if (hydraulic_pressure<1305.0f)
        center_right.normal_brakes_available=0;
    if (XPLMGetDatai(center_right.brake_failure_ref[0])>=6 ||
        XPLMGetDatai(center_right.brake_failure_ref[1])>=6)
        center_right.normal_brakes_available=0;
    center_right.antiskid_on=XPLMGetDatai(center_right.abs_ref)!=0;
    center_right.emergency_braking=!center_right.antiskid_on;
    center_right.antiskid_available=center_right.antiskid_on &&
        (center_right.normal_brakes_available || center_right.body_accu_psi>1305);
    /* Autobrake panel: AC ESS 400XP (spec 6.4) with ADIRS attitude/heading. */
    inertial_available=a380x_power_bus(A380X_ELN_AC_ESS, XPLMGetDatai(refs.avionics_power)) &&
        ((refs.side[0].attitude_valid && refs.side[0].heading_valid) ||
         (refs.side[1].attitude_valid && refs.side[1].heading_valid));
    center_right.autobrake_available=center_right.antiskid_on &&
        center_right.normal_brakes_available && inertial_available;
    /* X-Plane's own autobrake switch (hardware, X-Plane commands, situations) is read as the AUTO BRK selector /
     * RTO pushbutton and returned to OFF: the BCU core below does the braking. */
    native_mode=XPLMGetDatai(center_right.autobrake_ref);
    if (native_mode!=1) {
        if (native_mode==0) center_right.ab_rto_edge=1;
        else if (native_mode>=2 && native_mode<=5) {
            center_right.autobrake_selected=native_mode;
            center_right.ab_knob_moved=1;
        }
        XPLMSetDatai(center_right.autobrake_ref,1);
    }
    center_right_autobrake_step(dt,pedal);
    XPLMGetDatavf(center_right.wheel_brake_ref,wheel_brakes,0,5);
    center_right.brake_left_ratio_milli=(int)lroundf(center_right_ratio(wheel_brakes[3])*1000.0f);
    center_right.brake_right_ratio_milli=(int)lroundf(center_right_ratio(wheel_brakes[4])*1000.0f);
}
static void center_right_release_controls(void)
{
    if (center_right.reinflate_active && center_right.reinflate_owns_pump && center_right.electric_pump_ref)
        XPLMSetDatai(center_right.electric_pump_ref,0);
    center_right.reinflate_active=0;
    center_right.reinflate_owns_pump=0;
    center_right.rto_pressed=0;
    center_right.reinflate_pressed=0;
    if (center_right.brake_add_owned && center_right.brake_add_ref[0] && center_right.brake_add_ref[1])
        center_right_brake_add(0.0f);
    center_right.brake_add_owned=0;
}
static void center_right_shutdown(void)
{
    int i;
    for (i=0;i<ARRAY_LEN(center_right_commands);++i)
        if (center_right_commands[i].handle) {
            XPLMUnregisterCommandHandler(center_right_commands[i].handle,
                center_right_command_handler,1,&center_right_commands[i]);
            center_right_commands[i].handle=NULL;
        }
    if (center_right.athr_cmd) {
        XPLMUnregisterCommandHandler(center_right.athr_cmd,center_right_athr_handler,1,NULL);
        center_right.athr_cmd=NULL;
    }
    for (i=0;i<ARRAY_LEN(center_right_refs);++i)
        if (center_right_refs[i].handle) {
            XPLMUnregisterDataAccessor(center_right_refs[i].handle);
            center_right_refs[i].handle=NULL;
        }
    center_right_release_controls();
    if (center_right.autobrake_ref) XPLMSetDatai(center_right.autobrake_ref,1);
    if (center_right.decel_owned && center_right.decel_ref)
        XPLMSetDatavf(center_right.decel_ref,center_right.original_decels,0,4);
    center_right.decel_owned=0;
    if (center_right.abs_ref) XPLMSetDatai(center_right.abs_ref,center_right.abs_initial);
}
static int center_right_init(void)
{
    int i;
    memset(&center_right,0,sizeof(center_right));
    center_right.lever_ref=XPLMFindDataRef("sim/cockpit2/controls/gear_handle_down");
    center_right.autobrake_ref=XPLMFindDataRef("sim/cockpit2/switches/auto_brake_level");
    center_right.abs_ref=XPLMFindDataRef("sim/aircraft/gear/acf_has_abs");
    center_right.accumulator_ref=XPLMFindDataRef(
        "sim/cockpit2/hydraulics/indicators/brake_accumulator_pressure_ratio");
    center_right.hydraulic_ref[0]=XPLMFindDataRef("sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1");
    center_right.hydraulic_ref[1]=XPLMFindDataRef("sim/cockpit2/hydraulics/indicators/hydraulic_pressure_2");
    center_right.pedal_ref[0]=XPLMFindDataRef("sim/cockpit2/controls/left_brake_ratio");
    center_right.pedal_ref[1]=XPLMFindDataRef("sim/cockpit2/controls/right_brake_ratio");
    center_right.brake_failure_ref[0]=XPLMFindDataRef("sim/operation/failures/rel_lbrakes");
    center_right.brake_failure_ref[1]=XPLMFindDataRef("sim/operation/failures/rel_rbrakes");
    center_right.electric_pump_ref=XPLMFindDataRef("sim/cockpit2/hydraulics/actuators/electric_hydraulic_pump_on");
    center_right.wheel_brake_ref=XPLMFindDataRef("sim/flightmodel2/gear/tire_part_brake");
    center_right.spoiler_ref=XPLMFindDataRef("sim/flightmodel/controls/sbrkrat");
    center_right.park_ref=XPLMFindDataRef("sim/cockpit2/controls/wheel_brake_ratio");
    center_right.decel_ref=XPLMFindDataRef("sim/aircraft/specialcontrols/acf_autofbrk_decels");
    center_right.brake_add_ref[0]=XPLMFindDataRef("sim/flightmodel/controls/l_brake_add");
    center_right.brake_add_ref[1]=XPLMFindDataRef("sim/flightmodel/controls/r_brake_add");
    center_right.gear_ground_ref=XPLMFindDataRef("sim/flightmodel2/gear/on_ground");
    center_right.local_v_ref[0]=XPLMFindDataRef("sim/flightmodel/position/local_vx");
    center_right.local_v_ref[1]=XPLMFindDataRef("sim/flightmodel/position/local_vz");
    center_right.local_a_ref[0]=XPLMFindDataRef("sim/flightmodel/position/local_ax");
    center_right.local_a_ref[1]=XPLMFindDataRef("sim/flightmodel/position/local_az");
    if (!center_right.lever_ref || !center_right.autobrake_ref ||
        !center_right.abs_ref || !center_right.accumulator_ref ||
        !center_right.electric_pump_ref || !center_right.wheel_brake_ref ||
        !center_right.spoiler_ref || !center_right.park_ref || !center_right.decel_ref ||
        !center_right.gear_ground_ref) return 0;
    for (i=0;i<2;++i)
        if (!center_right.hydraulic_ref[i] || !center_right.pedal_ref[i] ||
            !center_right.brake_failure_ref[i] || !center_right.brake_add_ref[i] ||
            !center_right.local_v_ref[i] || !center_right.local_a_ref[i]) return 0;
    center_right.abs_initial=XPLMGetDatai(center_right.abs_ref);
    if (XPLMGetDatavf(center_right.decel_ref,center_right.original_decels,0,4)!=4) return 0;
    {
        /* Final landing targets from A380AutobrakeController in the pinned
         * FBW hydraulic/autobrakes.rs. X-Plane consumes positive g units.
         * The native controller supplies engagement/ramp timing and RTO. */
        float decels[4]={2.0f/9.80665f,2.5f/9.80665f,3.0f/9.80665f,3.5f/9.80665f};
        XPLMSetDatavf(center_right.decel_ref,decels,0,4);
        center_right.decel_owned=1;
    }
    center_right.previous_gear_lever=XPLMGetDatai(center_right.lever_ref)!=0;
    /* BCU start: on the ground, or in flight (no 10 s disarm pulse, RTO refused). X-Plane's own autobrake switch
     * at load (situation, .acf) is taken over by the first update as a selection. */
    a380x_ab_init(&center_right.ab,!XPLMGetDatai(refs.on_ground));
    center_right.athr_cmd=XPLMFindCommand("a380x/xp12/fcu/athr_disconnect");
    if (center_right.athr_cmd)
        XPLMRegisterCommandHandler(center_right.athr_cmd,center_right_athr_handler,1,NULL);
    for (i=0;i<ARRAY_LEN(center_right_refs);++i) {
        CenterRightRef *ref=&center_right_refs[i];
        if (XPLMFindDataRef(ref->name)) { center_right_shutdown(); return 0; }
        ref->handle=XPLMRegisterDataAccessor(ref->name,xplmType_Int,0,
            center_right_get,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,ref,NULL);
        if (!ref->handle) { center_right_shutdown(); return 0; }
    }
    for (i=0;i<ARRAY_LEN(center_right_commands);++i) {
        CenterRightCommand *item=&center_right_commands[i];
        /* XPLM retains commands across aircraft reloads; CreateCommand reuses them. */
        item->handle=XPLMCreateCommand(item->name,"A380 center-right control");
        if (!item->handle) { center_right_shutdown(); return 0; }
        XPLMRegisterCommandHandler(item->handle,center_right_command_handler,1,item);
    }
    center_right_update(0.0f);
    return 1;
}
