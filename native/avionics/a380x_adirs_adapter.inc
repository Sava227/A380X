/* SPDX-License-Identifier: GPL-3.0-or-later
 * Three independent ADIRUs at the X-Plane sensor/electrical boundary.
 * The native aircraft has a shared electrical bus and three pitot/static
 * channels. This adapter does not claim to reproduce Airbus bus contactors.
 */
#include "a380x_adirs.h"
#include "a380x_elec_power.h"
static A380XAdirsState adirs;
static A380XAdirsInputs adirs_inputs;
typedef struct {
    double knob, ir_press, adr_press;
    double ias, altitude, mach, alpha, sat, tat, pitch, roll, vertical_speed;
    double heading, track, ground_speed, latitude, longitude, flight_path;
    int ir_failure, adr_failure, overspeed;
    XPLMDataRef ias_ref, altitude_ref, baro_ref, alpha_ref, sat_ref, tat_ref;
    XPLMDataRef pitot_fault, static_fault, adc_fault, attitude_fault;
} AdirsSample;
static AdirsSample adirs_sample[3];
enum {
    AR_BUS, AR_BATTERY, AR_BATTERY_ON, AR_GEN_ON, AR_GEN_VOLTS, AR_GEN_AMPS,
    AR_APU_ON, AR_APU_AMPS, AR_GPU_ON, AR_GPU_AMPS, AR_PAUSED, AR_REPLAY,
    AR_SPEED, AR_LAT, AR_LON, AR_PITCH, AR_ROLL, AR_HEADING, AR_MAGVAR,
    AR_VY, AR_MACH, AR_VPATH, AR_VMO, AR_MMO, AR_COUNT
};
static const char *adirs_raw_names[AR_COUNT]={
    "sim/cockpit2/electrical/bus_volts",
    "sim/cockpit2/electrical/battery_voltage_actual_volts",
    "sim/cockpit2/electrical/battery_on",
    "sim/cockpit2/electrical/generator_on",
    "sim/cockpit2/electrical/generator_volts",
    "sim/cockpit2/electrical/generator_amps",
    "sim/cockpit2/electrical/APU_generator_on",
    "sim/cockpit2/electrical/APU_generator_amps",
    "sim/cockpit/electrical/gpu_on", "sim/cockpit/electrical/gpu_amps",
    "sim/time/paused", "sim/operation/prefs/replay_mode", "sim/time/sim_speed_actual",
    "sim/flightmodel/position/latitude", "sim/flightmodel/position/longitude",
    "sim/flightmodel/position/theta", "sim/flightmodel/position/phi",
    "sim/flightmodel/position/psi", "sim/flightmodel/position/magnetic_variation",
    "sim/flightmodel/position/local_vy", "sim/flightmodel/misc/machno",
    "sim/flightmodel/position/vpath", "sim/aircraft/view/acf_Vne", "sim/aircraft/view/acf_Mmo"
};
static XPLMDataRef adirs_raw[AR_COUNT];
static double adirs_wrap(double value)
{
    value=fmod(value,360.0); return value<0 ? value+360.0 : value;
}
static void adirs_read_inputs(void)
{
    int i, external=0, battery_available=0;
    float bus[6]={0}, battery[8]={0}, volts[8]={0}, amps[8]={0};
    int battery_on[8]={0}, gen_on[8]={0};
    double heading, latitude, longitude, pitch, roll, gs, track, vertical;
    XPLMGetDatavf(adirs_raw[AR_BUS],bus,0,6);
    XPLMGetDatavf(adirs_raw[AR_BATTERY],battery,0,8);
    XPLMGetDatavi(adirs_raw[AR_BATTERY_ON],battery_on,0,8);
    XPLMGetDatavi(adirs_raw[AR_GEN_ON],gen_on,0,8);
    XPLMGetDatavf(adirs_raw[AR_GEN_VOLTS],volts,0,8);
    XPLMGetDatavf(adirs_raw[AR_GEN_AMPS],amps,0,8);
    for(i=0;i<8;++i) {
        battery_available |= battery_on[i] && isfinite(battery[i]) && battery[i]>=18;
        external |= gen_on[i] && isfinite(volts[i]) && volts[i]>=18 && amps[i]>0.1f;
    }
    external |= XPLMGetDatai(adirs_raw[AR_APU_ON]) && XPLMGetDataf(adirs_raw[AR_APU_AMPS])>0.1f;
    external |= XPLMGetDatai(adirs_raw[AR_GPU_ON]) && XPLMGetDataf(adirs_raw[AR_GPU_AMPS])>0.1f;
    adirs_inputs.on_ground=XPLMGetDatai(refs.on_ground)!=0;
    adirs_inputs.paused=XPLMGetDatai(adirs_raw[AR_PAUSED]) || XPLMGetDatai(adirs_raw[AR_REPLAY]);
    latitude=XPLMGetDatad(adirs_raw[AR_LAT]); longitude=XPLMGetDatad(adirs_raw[AR_LON]);
    adirs_inputs.latitude_deg=latitude;
    gs=XPLMGetDataf(refs.ground_speed)*1.94384449;
    adirs_inputs.ground_speed_knots=gs;
    heading=adirs_wrap(XPLMGetDataf(adirs_raw[AR_HEADING])-XPLMGetDataf(adirs_raw[AR_MAGVAR]));
    track=adirs_wrap(XPLMGetDataf(refs.ground_track)-XPLMGetDataf(adirs_raw[AR_MAGVAR]));
    /* FBW adirs.rs: below MINIMUM_GROUND_SPEED_FOR_TRACK_KNOTS (50 kt) the IR track output is the
     * heading. X-Plane's ground track from a near-zero velocity wanders, which made the ND track
     * diamond jump around the rose while parked. */
    if(!(isfinite(gs) && gs>=50.0)) track=heading;
    pitch=XPLMGetDataf(adirs_raw[AR_PITCH]); roll=XPLMGetDataf(adirs_raw[AR_ROLL]);
    vertical=XPLMGetDataf(adirs_raw[AR_VY])*196.850394;
    for(i=0;i<3;++i) {
        AdirsSample *s=&adirs_sample[i];
        A380XAdirsUnitInputs *in=&adirs_inputs.unit[i];
        int powered=isfinite(bus[0]) && bus[0]>=18; /* legacy: one common stock bus */
        if(a380x_power.available) {
            /* A380 network (spec 6.4 proposal): ADIRU 1 AC EMER, 2 AC 2, 3 AC 1 / AC ESS, each
             * backed by its hot bus; ON BAT = running from the hot-bus backup only. */
            int primary=a380x_power.adiru_primary[i], backup=a380x_power.adiru_backup[i];
            in->adr_powered=in->ir_powered=primary || backup;
            in->on_battery=!primary && backup;
        } else {
            in->adr_powered=powered; in->ir_powered=powered;
            in->on_battery=powered && battery_available && !external;
        }
        s->ias=XPLMGetDataf(s->ias_ref); s->altitude=XPLMGetDataf(s->altitude_ref);
        s->alpha=XPLMGetDataf(s->alpha_ref); s->sat=XPLMGetDataf(s->sat_ref); s->tat=XPLMGetDataf(s->tat_ref);
        s->mach=i<2 ? XPLMGetDataf(refs.side[i].mach) : XPLMGetDataf(adirs_raw[AR_MACH]);
        s->pitch=pitch; s->roll=roll; s->vertical_speed=vertical;
        s->track=track; s->ground_speed=gs; s->latitude=latitude; s->longitude=longitude;
        s->flight_path=XPLMGetDataf(adirs_raw[AR_VPATH]);
        in->adr_fault=s->adr_failure || failed(s->adc_fault) || failed(s->pitot_fault) || failed(s->static_fault);
        in->ir_fault=s->ir_failure || failed(s->attitude_fault);
        if(i<2) {
            in->adr_fault |= failed(refs.side[i].fail_ias) || failed(refs.side[i].fail_altitude);
            in->ir_fault |= failed(refs.side[i].fail_attitude) || failed(refs.side[i].fail_heading);
        }
        in->adr_sensor_valid=isfinite(s->ias) && isfinite(s->altitude) && isfinite(s->mach) &&
            isfinite(s->alpha) && isfinite(s->sat) && isfinite(s->tat);
        in->attitude_sensor_valid=isfinite(pitch) && isfinite(roll);
        in->heading_sensor_valid=isfinite(heading); in->heading_deg=heading;
        in->navigation_sensor_valid=isfinite(latitude) && fabs(latitude)<=90 &&
            isfinite(longitude) && fabs(longitude)<=180 && isfinite(gs) && isfinite(track) && isfinite(vertical);
        s->overspeed=0;
    }
}
static void adirs_route(void)
{
    int i;
    for(i=0;i<3;++i) {
        AdirsSample *s=&adirs_sample[i]; const A380XAdirsUnitState *u=&adirs.unit[i];
        double vmo=XPLMGetDataf(adirs_raw[AR_VMO]), mmo=XPLMGetDataf(adirs_raw[AR_MMO]);
        s->knob=u->knob_ratio*2; s->ir_press=u->ir_press_ratio; s->adr_press=u->adr_press_ratio;
        s->heading=u->heading_deg;
        s->overspeed=u->adr_valid && ((isfinite(vmo) && vmo>0 && s->ias>vmo) ||
            (isfinite(mmo) && mmo>0 && s->mach>mmo));
    }
    for(i=0;i<2;++i) {
        Side *side=&refs.side[i];
        int ai=adirs.adr_source[i], ii=adirs.ir_source[i];
        const AdirsSample *a=&adirs_sample[ai], *r=&adirs_sample[ii];
        const A380XAdirsUnitState *au=&adirs.unit[ai], *iu=&adirs.unit[ii];
        side->ias_value=(float)a->ias; side->mach_value=(float)a->mach;
        /* Keep the receiving PFD's barometric setting after source switching. */
        side->altitude_value=(float)(a->altitude+(XPLMGetDataf(adirs_sample[i].baro_ref)-XPLMGetDataf(adirs_sample[ai].baro_ref))*1000.0);
        side->pitch_value=(float)r->pitch; side->roll_value=(float)r->roll;
        side->heading_value=(float)iu->heading_deg; side->vertical_speed_value=(float)r->vertical_speed;
        side->air_valid=au->adr_valid; side->altitude_valid=au->adr_valid && isfinite(side->altitude_value);
        side->attitude_valid=iu->attitude_valid; side->heading_valid=iu->heading_valid;
        side->vertical_speed_valid=iu->nav_valid && !failed(side->fail_vertical_speed);
        side->navigation_valid=iu->nav_valid;
        if(!iu->nav_valid) { side->ias_trend_visible=0; side->trend_initialized=0; }
    }
}
static void adirs_update(float dt)
{
    double speed, knob[3], ir_press[3], adr_press[3]; int i;
    double anim=isfinite(dt) && dt>0 ? 1-exp(-18*fmin(dt,1)) : 0;
    for(i=0;i<3;++i) {
        knob[i]=adirs.unit[i].knob_ratio;
        ir_press[i]=adirs.unit[i].ir_press_ratio; adr_press[i]=adirs.unit[i].adr_press_ratio;
    }
    adirs_read_inputs(); speed=XPLMGetDataf(adirs_raw[AR_SPEED]);
    if(!isfinite(dt) || dt<0 || !isfinite(speed) || speed<0) dt=0;
    a380x_adirs_update(&adirs,&adirs_inputs,dt*fmin(speed,100));
    /* Physical controls keep moving while paused; system timers use simulator time. */
    for(i=0;i<3;++i) {
        A380XAdirsUnitState *u=&adirs.unit[i];
        u->knob_ratio=knob[i]+(.5*u->mode-knob[i])*anim;
        u->ir_press_ratio=ir_press[i]+(u->ir_pressed-ir_press[i])*anim;
        u->adr_press_ratio=adr_press[i]+(u->adr_pressed-adr_press[i])*anim;
    }
    adirs_route();
}
static int adirs_set_mode(int unit,int mode)
{
    int result;
    if(!plugin_enabled) return 0;
    adirs_read_inputs();
    result=a380x_adirs_set_mode(&adirs,&adirs_inputs,unit,mode);
    adirs_update(0); return result;
}
static int adirs_set_button(int unit,int is_ir,int pressed)
{
    int was_pressed;
    if(unit<0 || unit>=3 || (pressed && !plugin_enabled)) return 0;
    was_pressed=is_ir ? adirs.unit[unit].ir_pressed : adirs.unit[unit].adr_pressed;
    a380x_adirs_set_pressed(&adirs,unit,is_ir,pressed);
    if(pressed && !was_pressed) {
        if(is_ir) a380x_adirs_toggle_ir(&adirs,unit);
        else a380x_adirs_toggle_adr(&adirs,unit);
    }
    adirs_update(0); return 1;
}
static int adirs_enter_heading(int unit,float heading)
{
    int result; if(!plugin_enabled) return 0; adirs_read_inputs();
    result=a380x_adirs_enter_heading(&adirs,&adirs_inputs,unit,heading);
    adirs_update(0); return result;
}
static int adirs_set_source(int side,int is_ir,int use_third)
{
    if(!plugin_enabled) return 0;
    int result=a380x_adirs_set_source(&adirs,side,is_ir,use_third);
    adirs_update(0); return result;
}
static int adirs_align_now(void)
{
    int result; if(!plugin_enabled) return 0; adirs_read_inputs();
    result=a380x_adirs_align_now(&adirs,&adirs_inputs);
    adirs_update(0); return result;
}
/* Datarefs remain numeric; each value has an explicit validity flag. */
typedef struct { const char *name; void *value; int integer, write, unit; XPLMDataRef handle; } AdirsRef;
#define AD_I(name,field) {"a380x/xp12/adirs/" name,&field,1,0,0,NULL}
#define AD_D(name,field) {"a380x/xp12/adirs/" name,&field,0,0,0,NULL}
#define AD_W(name,field,u) {"a380x/xp12/adirs/" name,&field,1,1,u,NULL}
#define AD_UNIT(n,i) \
    AD_I("ir" n "_mode",adirs.unit[i].mode), AD_D("ir" n "_knob",adirs_sample[i].knob), \
    AD_D("ir" n "_pressed",adirs_sample[i].ir_press), AD_D("adr" n "_pressed",adirs_sample[i].adr_press), \
    AD_I("ir" n "_off",adirs.unit[i].ir_off_lit), AD_I("adr" n "_off",adirs.unit[i].adr_off_lit), \
    AD_I("ir" n "_fault",adirs.unit[i].ir_fault_lit), AD_I("adr" n "_fault",adirs.unit[i].adr_fault_lit), \
    AD_W("ir" n "_failure",adirs_sample[i].ir_failure,i), AD_W("adr" n "_failure",adirs_sample[i].adr_failure,i), \
    AD_I("ir" n "_nav_valid",adirs.unit[i].nav_valid), AD_I("adr" n "_valid",adirs.unit[i].adr_valid), \
    AD_I("ir" n "_attitude_valid",adirs.unit[i].attitude_valid), AD_I("ir" n "_heading_valid",adirs.unit[i].heading_valid), \
    AD_I("ir" n "_nav_locked",adirs.unit[i].nav_locked), AD_I("ir" n "_aligning",adirs.unit[i].aligning), \
    AD_D("ir" n "_alignment_remaining",adirs.unit[i].alignment_remaining), \
    AD_I("ir" n "_heading_required",adirs.unit[i].att_heading_required), \
    AD_I("ir" n "_heading_reminder",adirs.unit[i].att_heading_reminder), \
    AD_D("ir" n "_heading_age",adirs.unit[i].heading_age_seconds), \
    AD_D("ir" n "_heading_deg",adirs_sample[i].heading), AD_D("ir" n "_pitch_deg",adirs_sample[i].pitch), \
    AD_D("ir" n "_roll_deg",adirs_sample[i].roll), AD_D("ir" n "_vertical_speed_fpm",adirs_sample[i].vertical_speed), \
    AD_D("ir" n "_track_deg",adirs_sample[i].track), AD_D("ir" n "_ground_speed_knots",adirs_sample[i].ground_speed), \
    AD_D("ir" n "_latitude_deg",adirs_sample[i].latitude), AD_D("ir" n "_longitude_deg",adirs_sample[i].longitude), \
    AD_D("ir" n "_flight_path_deg",adirs_sample[i].flight_path), \
    AD_D("adr" n "_ias_knots",adirs_sample[i].ias), AD_D("adr" n "_altitude_ft",adirs_sample[i].altitude), \
    AD_D("adr" n "_mach",adirs_sample[i].mach), AD_D("adr" n "_aoa_deg",adirs_sample[i].alpha), \
    AD_D("adr" n "_sat_c",adirs_sample[i].sat), AD_D("adr" n "_tat_c",adirs_sample[i].tat), \
    AD_I("adr" n "_overspeed",adirs_sample[i].overspeed)
static AdirsRef adirs_refs[]={
    AD_UNIT("1",0), AD_UNIT("2",1), AD_UNIT("3",2), AD_I("on_batt",adirs.on_batt),
    AD_I("captain_ir_source",adirs.ir_source[0]), AD_I("captain_adr_source",adirs.adr_source[0]),
    AD_I("first_officer_ir_source",adirs.ir_source[1]), AD_I("first_officer_adr_source",adirs.adr_source[1])
};
#undef AD_UNIT
#undef AD_I
#undef AD_D
#undef AD_W
static int adirs_get_int(void *context)
{
    AdirsRef *r=context; return r->integer ? *(int *)r->value : (int)*(double *)r->value;
}
static double adirs_get_double(void *context)
{
    AdirsRef *r=context; return r->integer ? *(int *)r->value : *(double *)r->value;
}
static float adirs_get_float(void *context) { return (float)adirs_get_double(context); }
static void adirs_set_int(void *context,int value)
{
    AdirsRef *r=context; if(plugin_enabled && r->write) { *(int *)r->value=value!=0; adirs_update(0); }
}
enum { AD_MODE_UP, AD_MODE_DOWN, AD_IR, AD_ADR, AD_SOURCE_IR, AD_SOURCE_ADR, AD_ALIGN };
typedef struct { const char *name; int action,unit; XPLMCommandRef handle; } AdirsCommand;
#define AD_CMD(name,action,unit) {"a380x/xp12/adirs/" name,action,unit,NULL}
#define AD_CMDS(n,i) AD_CMD("ir" n "_mode_up",AD_MODE_UP,i), AD_CMD("ir" n "_mode_down",AD_MODE_DOWN,i), \
    AD_CMD("ir" n "_toggle",AD_IR,i), AD_CMD("adr" n "_toggle",AD_ADR,i)
static AdirsCommand adirs_commands[]={
    AD_CMDS("1",0), AD_CMDS("2",1), AD_CMDS("3",2),
    AD_CMD("captain_ir_toggle",AD_SOURCE_IR,0), AD_CMD("captain_adr_toggle",AD_SOURCE_ADR,0),
    AD_CMD("first_officer_ir_toggle",AD_SOURCE_IR,1), AD_CMD("first_officer_adr_toggle",AD_SOURCE_ADR,1),
    AD_CMD("align_now",AD_ALIGN,0)
};
#undef AD_CMDS
#undef AD_CMD
static int adirs_command_handler(XPLMCommandRef command,XPLMCommandPhase phase,void *context)
{
    AdirsCommand *c=context; int i=c->unit; (void)command;
    if(phase==xplm_CommandEnd) {
        if(c->action==AD_IR || c->action==AD_ADR) adirs_set_button(i,c->action==AD_IR,0);
        return 1;
    }
    if(!plugin_enabled || phase!=xplm_CommandBegin) return 1;
    adirs_read_inputs();
    switch(c->action) {
    case AD_MODE_UP: case AD_MODE_DOWN:
        adirs_set_mode(i,adirs.unit[i].mode+(c->action==AD_MODE_UP?1:-1)); break;
    case AD_IR: adirs_set_button(i,1,1); break;
    case AD_ADR: adirs_set_button(i,0,1); break;
    case AD_SOURCE_IR: adirs_set_source(i,1,adirs.ir_source[i]!=2); break;
    case AD_SOURCE_ADR: adirs_set_source(i,0,adirs.adr_source[i]!=2); break;
    case AD_ALIGN: adirs_align_now(); break;
    default: break;
    }
    adirs_update(0); return 1;
}
static void adirs_release_controls(void)
{
    int i; for(i=0;i<3;++i) {
        a380x_adirs_set_pressed(&adirs,i,0,0); a380x_adirs_set_pressed(&adirs,i,1,0);
        adirs.unit[i].ir_press_ratio=adirs.unit[i].adr_press_ratio=0;
        adirs_sample[i].ir_press=adirs_sample[i].adr_press=0;
    }
}
static void adirs_reset(void)
{
    int i,hot=!XPLMGetDatai(refs.on_ground); float engine[4]={0};
    adirs_read_inputs(); XPLMGetDatavf(refs.n1,engine,0,4);
    for(i=0;i<4;++i) if(isfinite(engine[i]) && engine[i]>15) hot=1;
    /* READY / TURNAROUND spawn of the A380 network: ADIRS aligned in NAV (spec 7.2). */
    if(a380x_power.available && a380x_power.spawn_mode!=A380X_ELN_SPAWN_COLD_DARK) hot=1;
    a380x_adirs_init(&adirs,&adirs_inputs,hot); adirs_update(0);
}
static void adirs_shutdown(void)
{
    int i; adirs_release_controls();
    for(i=0;i<ARRAY_LEN(adirs_commands);++i) if(adirs_commands[i].handle) {
        XPLMUnregisterCommandHandler(adirs_commands[i].handle,adirs_command_handler,1,&adirs_commands[i]);
        adirs_commands[i].handle=NULL;
    }
    for(i=0;i<ARRAY_LEN(adirs_refs);++i) if(adirs_refs[i].handle) {
        XPLMUnregisterDataAccessor(adirs_refs[i].handle); adirs_refs[i].handle=NULL;
    }
}
static int adirs_init(void)
{
    int i,ok=1; char name[160];
    static const char *suffix[]={"pilot","copilot","stby"};
    static const char *pitot[]={"rel_pitot","rel_pitot2","rel_pitot_stby"};
    static const char *stat[]={"rel_static","rel_static2","rel_static_stby"};
    memset(&adirs,0,sizeof(adirs)); memset(&adirs_inputs,0,sizeof(adirs_inputs));
    memset(adirs_sample,0,sizeof(adirs_sample));
    for(i=0;i<AR_COUNT;++i) adirs_raw[i]=find_required(adirs_raw_names[i],&ok);
    for(i=0;i<3;++i) {
        AdirsSample *s=&adirs_sample[i];
#define AD_SENSOR(field,stem) snprintf(name,sizeof(name),"sim/cockpit2/gauges/indicators/" stem "_%s",suffix[i]); s->field=find_required(name,&ok)
        AD_SENSOR(ias_ref,"airspeed_kts"); AD_SENSOR(altitude_ref,"altitude_ft");
        AD_SENSOR(alpha_ref,"AoA"); AD_SENSOR(sat_ref,"SAT"); AD_SENSOR(tat_ref,"TAT");
#undef AD_SENSOR
        snprintf(name,sizeof(name),"sim/cockpit2/gauges/actuators/barometer_setting_in_hg_%s",suffix[i]);
        s->baro_ref=find_required(name,&ok);
        snprintf(name,sizeof(name),"sim/operation/failures/%s",pitot[i]); s->pitot_fault=find_required(name,&ok);
        snprintf(name,sizeof(name),"sim/operation/failures/%s",stat[i]); s->static_fault=find_required(name,&ok);
        if(i<2) {
            s->adc_fault=find_required(i?"sim/operation/failures/rel_adc_comp_2":"sim/operation/failures/rel_adc_comp",&ok);
            s->attitude_fault=find_required(i?"sim/operation/failures/rel_g_arthorz_2":"sim/operation/failures/rel_g_arthorz",&ok);
        }
    }
    if(!ok) return 0;
    for(i=0;i<ARRAY_LEN(adirs_refs);++i) if(XPLMFindDataRef(adirs_refs[i].name)) return 0;
    for(i=0;i<ARRAY_LEN(adirs_refs);++i) {
        AdirsRef *r=&adirs_refs[i];
        r->handle=XPLMRegisterDataAccessor(r->name,r->integer?xplmType_Int:xplmType_Float|xplmType_Double,r->write,
            r->integer?adirs_get_int:NULL,r->write?adirs_set_int:NULL,
            r->integer?NULL:adirs_get_float,NULL,r->integer?NULL:adirs_get_double,NULL,
            NULL,NULL,NULL,NULL,NULL,NULL,r,r->write?r:NULL);
        if(!r->handle) { adirs_shutdown(); return 0; }
    }
    for(i=0;i<ARRAY_LEN(adirs_commands);++i) {
        AdirsCommand *c=&adirs_commands[i]; c->handle=XPLMCreateCommand(c->name,c->name);
        if(!c->handle) { adirs_shutdown(); return 0; }
        XPLMRegisterCommandHandler(c->handle,adirs_command_handler,1,c);
    }
    adirs_reset(); return 1;
}
