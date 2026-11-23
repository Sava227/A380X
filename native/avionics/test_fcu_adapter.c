/* SPDX-License-Identifier: GPL-3.0-or-later
 * Executes the production FCU adapter and guidance core against in-memory XPLM.
 * No simulator process, network or aircraft file is read or written.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "a380x_guidance.h"
#include "a380x_pfd_data.h"
#define ARRAY_LEN(a) ((int)(sizeof(a)/sizeof((a)[0])))

typedef struct {
    char name[180]; double value; int array_i[500],count,registered,writable,writes;
    float array_f[500]; char bytes[150];
    XPLMDataTypeID types;
    XPLMGetDatai_f read_i; XPLMGetDataf_f read_f; void *read_refcon;
} MockRef;
typedef struct { char name[180]; XPLMCommandCallback_f callback; void *refcon; int active; } MockCommand;
static MockRef mock_refs[200];
static MockCommand mock_commands[100];
static int ref_count,command_count,registrations,registration_fail=-1,command_once_count;
static float mock_clock;
static const char *missing_raw;
static char mock_aircraft_root[1024],mock_aircraft_path[1200],mock_debug[32768];
static const char *const mock_calibration="version=1\nidle_detent_ratio=.01\ncl_detent_ratio=.5555555555555556\nmct_detent_ratio=.7777777777777778\ntoga_detent_ratio=1\ndetent_tolerance_ratio=.02\n";
static MockRef *r(XPLMDataRef ref) { assert(ref);return (MockRef *)ref; }
static MockRef *new_ref(const char *name) {
    MockRef *p;int i;assert(ref_count<ARRAY_LEN(mock_refs));p=&mock_refs[ref_count++];
    memset(p,0,sizeof(*p));snprintf(p->name,sizeof(p->name),"%s",name);p->count=500;
    p->types=xplmType_Int|xplmType_Float;
    if(strstr(name,"/throttle_ratio"))p->types=xplmType_FloatArray;
    if(strstr(name,"/prop_mode"))p->types=xplmType_IntArray;
    for(i=0;i<500;++i) p->array_f[i]=strstr(name,"maximum")?1:strstr(name,"minimum")?0:.5f;
    if(strstr(name,"gear/deploy_ratio"))for(i=0;i<500;++i)p->array_f[i]=0;
    if(strstr(name,"acf_flap_detents"))p->value=6;
    if(strstr(name,"acf_flap_dn")){
        const float schedule[]={0,1,5,10,20,25,40};
        memset(p->array_f,0,sizeof(p->array_f));memcpy(p->array_f,schedule,sizeof(schedule));
    }
    if(strstr(name,"avionics_power"))p->value=1;
    if(strstr(name,"true_airspeed"))p->value=150;
    if(strstr(name,"acf_Vne"))p->value=340;
    if(strstr(name,"m_total"))p->value=400000;
    if(strstr(name,"pressure_altitude"))p->value=10020.125;
    if(strstr(name,"g_nrml"))p->value=1;
    if(strstr(name,"barometer_setting_in_hg"))p->value=29.92;
    if(strstr(name,"_slope_degt"))p->value=3.2;
    if(strstr(name,"nav1_type")||strstr(name,"nav2_type"))p->value=4;
    if(strstr(name,"display_horizontal")||strstr(name,"display_vertical")||strstr(name,"has_dme"))p->value=1;
    if(strstr(name,"course_deg"))p->value=90;
    if(strstr(name,"frequency_hz"))p->value=11030;
    if(strstr(name,"dme_distance"))p->value=12.5;
    if(strstr(name,"nav_id"))strcpy(p->bytes,"ITEST");
    return p;
}
XPLMDataRef XPLMFindDataRef(const char *name) {
    int i;if(missing_raw&&!strcmp(name,missing_raw))return NULL;
    for(i=0;i<ref_count;++i)if(!strcmp(mock_refs[i].name,name))return &mock_refs[i];
    return !strncmp(name,"sim/",4)?new_ref(name):NULL;
}
int XPLMGetDatai(XPLMDataRef ref) { MockRef*p=r(ref);return p->read_i?p->read_i(p->read_refcon):(int)p->value; }
XPLMDataTypeID XPLMGetDataRefTypes(XPLMDataRef ref) {return r(ref)->types;}
float XPLMGetDataf(XPLMDataRef ref) { MockRef*p=r(ref);assert(!strstr(p->name,"pressure_altitude"));return p->read_f?p->read_f(p->read_refcon):(float)p->value; }
double XPLMGetDatad(XPLMDataRef ref) { return r(ref)->value; }
void XPLMSetDatai(XPLMDataRef ref,int value) { r(ref)->value=value;r(ref)->writes++; }
void XPLMSetDataf(XPLMDataRef ref,float value) { assert(isfinite(value));r(ref)->value=value;r(ref)->writes++; }
int XPLMGetDatavi(XPLMDataRef ref,int *values,int offset,int count) {
    MockRef*p=r(ref);if(count>p->count-offset)count=p->count-offset;if(count<0)return 0;
    memcpy(values,p->array_i+offset,(size_t)count*sizeof(*values));return count;
}
int XPLMGetDatavf(XPLMDataRef ref,float *values,int offset,int count) {
    MockRef*p=r(ref);if(count>p->count-offset)count=p->count-offset;if(count<0)return 0;
    memcpy(values,p->array_f+offset,(size_t)count*sizeof(*values));return count;
}
void XPLMSetDatavf(XPLMDataRef ref,float *values,int offset,int count) {
    int i;MockRef*p=r(ref);for(i=0;i<count;++i)assert(isfinite(values[i]));
    memcpy(p->array_f+offset,values,(size_t)count*sizeof(*values));p->writes++;
}
int XPLMGetDatab(XPLMDataRef ref,void *out,int offset,int count) {
    MockRef*p=r(ref);if(count>150-offset)count=150-offset;memcpy(out,p->bytes+offset,(size_t)count);return count;
}
XPLMDataRef XPLMRegisterDataAccessor(const char *name,XPLMDataTypeID type,int writable,
    XPLMGetDatai_f read_i,XPLMSetDatai_f write_i,XPLMGetDataf_f read_f,XPLMSetDataf_f write_f,
    XPLMGetDatad_f read_d,XPLMSetDatad_f write_d,XPLMGetDatavi_f read_vi,XPLMSetDatavi_f write_vi,
    XPLMGetDatavf_f read_vf,XPLMSetDatavf_f write_vf,XPLMGetDatab_f read_b,XPLMSetDatab_f write_b,
    void *read_refcon,void *write_refcon) {
    MockRef*p;(void)type;(void)read_d;(void)write_d;(void)read_vi;(void)write_vi;(void)read_vf;(void)write_vf;
    (void)read_b;(void)write_b;(void)write_refcon;
    assert(!writable&&!write_i&&!write_f);
    if(registrations++==registration_fail)return NULL;
    assert(!XPLMFindDataRef(name));p=new_ref(name);p->registered=1;p->writable=writable;
    p->types=type;p->read_i=read_i;p->read_f=read_f;p->read_refcon=read_refcon;return p;
}
void XPLMUnregisterDataAccessor(XPLMDataRef ref) { r(ref)->registered=0;r(ref)->name[0]=0; }
XPLMCommandRef XPLMFindCommand(const char *name) {
    int i;for(i=0;i<command_count;++i)if(!strcmp(mock_commands[i].name,name))return &mock_commands[i];return NULL;
}
XPLMCommandRef XPLMCreateCommand(const char *name,const char *description) {
    MockCommand*p=(MockCommand*)XPLMFindCommand(name);(void)description;if(p)return p;
    assert(command_count<ARRAY_LEN(mock_commands));p=&mock_commands[command_count++];memset(p,0,sizeof(*p));
    snprintf(p->name,sizeof(p->name),"%s",name);return p;
}
void XPLMRegisterCommandHandler(XPLMCommandRef ref,XPLMCommandCallback_f handler,int before,void *refcon) {
    MockCommand*p=(MockCommand*)ref;assert(before);p->callback=handler;p->refcon=refcon;p->active=1;
}
void XPLMUnregisterCommandHandler(XPLMCommandRef ref,XPLMCommandCallback_f handler,int before,void *refcon) {
    MockCommand*p=(MockCommand*)ref;assert(before&&p->callback==handler&&p->refcon==refcon);p->active=0;
}
void XPLMCommandOnce(XPLMCommandRef ref) {
    MockCommand*p=(MockCommand*)ref;int pass=1;assert(p);command_once_count++;
    if(p->active)pass=p->callback(p,xplm_CommandBegin,p->refcon);
    if(pass&&!strcmp(p->name,"sim/autopilot/servos_off_any")) {
        r(XPLMFindDataRef("sim/cockpit2/autopilot/servos_on"))->value=0;
        r(XPLMFindDataRef("sim/cockpit2/autopilot/servos2_on"))->value=0;
        r(XPLMFindDataRef("sim/cockpit2/autopilot/servos3_on"))->value=0;
    }
    if(pass&&!strcmp(p->name,"sim/autopilot/servos_on"))r(XPLMFindDataRef("sim/cockpit2/autopilot/servos_on"))->value=1;
    if(pass&&!strcmp(p->name,"sim/autopilot/servos2_on"))r(XPLMFindDataRef("sim/cockpit2/autopilot/servos2_on"))->value=1;
    if(pass&&(!strcmp(p->name,"sim/autopilot/autothrottle_hard_off")||!strcmp(p->name,"sim/autopilot/autothrottle_off")))
        r(XPLMFindDataRef("sim/cockpit2/autopilot/autothrottle_enabled"))->value=-1;
    if(pass&&!strcmp(p->name,"sim/autopilot/autothrottle_on"))r(XPLMFindDataRef("sim/cockpit2/autopilot/autothrottle_enabled"))->value=1;
    if(p->active)p->callback(p,xplm_CommandEnd,p->refcon);
}
float XPLMGetElapsedTime(void) { return mock_clock; }
void XPLMDebugString(const char *message) {size_t n=strlen(mock_debug);snprintf(mock_debug+n,sizeof(mock_debug)-n,"%s",message);}
static char mock_spoken[512];
void XPLMGetSystemPath(char *out) {out[0]=0;}
void XPLMSpeakString(const char *text) {snprintf(mock_spoken,sizeof(mock_spoken),"%s",text);}
void XPLMGetNthAircraftModel(int index,char *filename,char *path) {
    assert(index==0);strcpy(filename,"User Selected Aircraft.acf");strcpy(path,mock_aircraft_path);
}

typedef struct {
    XPLMDataRef baro,baro_standard,nav_type,nav_horizontal,nav_vertical,nav_loc,nav_gs,nav_course,nav_distance;
    XPLMDataRef nav_frequency,nav_identifier,nav_has_dme;char nav_ident[16];
    float nav_frequency_mhz,nav_dme_nm;int nav_dme_valid;
    float ias_trend_value,previous_ias_value,baro_hpa_value,loc_dots,gs_dots;
    int ias_trend_visible,trend_initialized,baro_std,loc_valid,gs_valid,ls_visible;
    float ias_value,altitude_value,vertical_speed_value,pitch_value,roll_value,heading_value,radio_altitude_value,mach_value;
    int navigation_valid;
    int air_valid,altitude_valid,vertical_speed_valid,attitude_valid,heading_valid,radio_altitude_valid,display_valid;
} Side;
static struct {XPLMDataRef avionics_power,on_ground,ground_track;Side side[2];} refs;
static struct {int fma_a1a2,fma_a3,fma_b1,fma_c1,fma_d1d2;} published;
static A380XGuidanceState guidance;
static A380XGuidanceInputs guidance_inputs;
static A380XPfdGuidance pfd_guidance;
static A380XFmsFeed fms_feed; /* native FMS outputs (MFD adapter); zero = none */
static int plugin_enabled;
static float throttle_values[4],ground_speed_knots,ground_track_degrees;
static XPLMDataRef find_required(const char *name,int *ok) {XPLMDataRef p=XPLMFindDataRef(name);if(!p)*ok=0;return p;}
static void read_side(Side *side,int powered) {side->display_valid=powered;}
#include "a380x_fcu_adapter.inc"
#include "a380x_lighting.inc"
#include "a380x_du.inc"
static void setup(void);

static void test_display_unit_controls(void) {
    int i,j;
    XPLMDataRef handles[8];
    setup();assert(du_init());
    for(i=0;i<ARRAY_LEN(du_channels);++i) {
        DuChannel *channel=&du_channels[i];
        assert(channel->handle&&channel->up&&channel->down);
        handles[i]=channel->handle;
        assert(XPLMGetDatai(channel->handle)==100);
        for(j=0;j<25;++j)XPLMCommandOnce(channel->down);
        assert(XPLMGetDatai(channel->handle)==0);
        if(i==0)assert(XPLMGetDatai(du_channels[4].handle)==100);
        XPLMCommandOnce(channel->up);
        assert(XPLMGetDatai(channel->handle)==5);
        for(j=0;j<25;++j)XPLMCommandOnce(channel->up);
        assert(XPLMGetDatai(channel->handle)==100);
    }
    du_shutdown();
    for(i=0;i<ARRAY_LEN(du_channels);++i)assert(!r(handles[i])->registered);
}

static void test_glareshield_lighting(void) {
    int i,j;
    XPLMDataRef panel;
    XPLMDataRef handles[4];
    setup();
    assert(lighting_init());
    panel=XPLMFindDataRef("sim/cockpit2/switches/panel_brightness_ratio");
    assert(fabsf(r(panel)->array_f[1]-1.0f)<.001f);
    assert(fabsf(r(panel)->array_f[2]-1.0f)<.001f);
    for(i=0;i<ARRAY_LEN(lighting);++i) {
        LightingChannel *channel=&lighting[i];
        assert(channel->handle&&channel->up&&channel->down);
        handles[i]=channel->handle;
        for(j=0;j<25;++j) XPLMCommandOnce(channel->down);
        assert(XPLMGetDataf(channel->handle)==0.0f);
        XPLMCommandOnce(channel->up);
        assert(fabsf(XPLMGetDataf(channel->handle)-.05f)<.001f);
        assert(XPLMGetDatai(channel->handle)==5);
        for(j=0;j<25;++j) XPLMCommandOnce(channel->up);
        assert(XPLMGetDataf(channel->handle)==1.0f);
    }
    XPLMCommandOnce(lighting[0].down);
    assert(fabsf(r(panel)->array_f[1]-.95f)<.001f);
    assert(fabsf(r(panel)->array_f[2]-1.0f)<.001f);
    XPLMCommandOnce(lighting[3].down);
    assert(fabsf(r(panel)->array_f[2]-.95f)<.001f);
    lighting_shutdown();
    for(i=0;i<ARRAY_LEN(lighting);++i) assert(!r(handles[i])->registered);
}

static void normal_side(Side*s) {
    memset(s,0,sizeof(*s));s->ias_value=250;s->mach_value=.55f;s->altitude_value=10000;
    s->heading_value=90;s->radio_altitude_value=8000;s->pitch_value=2;
    s->navigation_valid=1;
    s->air_valid=s->altitude_valid=s->vertical_speed_valid=s->attitude_valid=s->heading_valid=s->radio_altitude_valid=s->display_valid=1;
}
static void cleanup_aircraft(void) {
    char path[1200];
    if(!mock_aircraft_root[0])return;
    snprintf(path,sizeof(path),"%s/fcu-calibration.ini",mock_aircraft_root);unlink(path);
    unlink(mock_aircraft_path);rmdir(mock_aircraft_root);
}
static void prepare_aircraft(void) {
    char path[1200];FILE *file;
    if(!mock_aircraft_root[0]) {
        const char *task_tmp=getenv("TMPDIR");
        snprintf(mock_aircraft_root,sizeof(mock_aircraft_root),"%s/a380x-fcu-test-XXXXXX",task_tmp?task_tmp:".");
        assert(mkdtemp(mock_aircraft_root));
        snprintf(mock_aircraft_path,sizeof(mock_aircraft_path),"%s/User Selected Aircraft.acf",mock_aircraft_root);
        file=fopen(mock_aircraft_path,"wb");assert(file);fclose(file);atexit(cleanup_aircraft);
    }
    snprintf(path,sizeof(path),"%s/fcu-calibration.ini",mock_aircraft_root);
    file=fopen(path,"wb");assert(file);assert(fputs(mock_calibration,file)>=0);assert(!fclose(file));
}
static void setup(void) {
    int i;memset(mock_refs,0,sizeof(mock_refs));memset(mock_commands,0,sizeof(mock_commands));
    memset(&fcu,0,sizeof(fcu));memset(&guidance,0,sizeof(guidance));
    ref_count=command_count=registrations=command_once_count=0;registration_fail=-1;missing_raw=NULL;mock_clock=0;
    mock_debug[0]=0;prepare_aircraft();
    refs.avionics_power=XPLMFindDataRef("sim/cockpit2/switches/avionics_power_on");
    refs.on_ground=XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    refs.ground_track=XPLMFindDataRef("sim/flightmodel/position/hpath");r(refs.ground_track)->value=90;
    normal_side(&refs.side[0]);normal_side(&refs.side[1]);ground_speed_knots=280;
    for(i=0;i<4;++i)throttle_values[i]=.5f;
    XPLMCreateCommand("sim/autopilot/servos_off_any","native disconnect");
    for(i=0;i<ARRAY_LEN(fcu_authority_commands);++i)XPLMCreateCommand(fcu_authority_commands[i].name,"stock command");
    plugin_enabled=1;
}
static void tick(int count) {int i;for(i=0;i<count;++i){mock_clock+=.1f;update_guidance(.1f);}}
static void command(const char *suffix) {
    char name[180];MockCommand*c;snprintf(name,sizeof(name),"a380x/xp12/fcu/%s",suffix);
    c=XPLMFindCommand(name);assert(c&&c->active);c->callback(c,xplm_CommandBegin,c->refcon);
}
static void engage(void) {tick(70);command("ap1_toggle");tick(1);assert(guidance.ap1&&fcu.own_pitch&&fcu.own_roll);}
static void test_registry_and_partial_init(void) {
    int i,j;setup();assert(fcu_init());assert(ARRAY_LEN(fcu_commands)==32&&ARRAY_LEN(fcu_outputs)==32);
    for(i=0;i<ARRAY_LEN(fcu_commands);++i){assert(fcu_commands[i].handle);for(j=0;j<i;++j)assert(strcmp(fcu_commands[i].name,fcu_commands[j].name));}
    for(i=0;i<ARRAY_LEN(fcu_outputs);++i)assert(r(fcu_outputs[i].handle)->registered&&!r(fcu_outputs[i].handle)->writable);
    command("speed_up");assert(guidance.selected_speed_knots==251);
    {MockCommand*c=fcu_commands[0].handle;c->callback(c,xplm_CommandContinue,c->refcon);assert(guidance.selected_speed_knots==251);
     mock_clock=.41f;c->callback(c,xplm_CommandContinue,c->refcon);assert(guidance.selected_speed_knots==252);}
    fcu_shutdown();for(i=0;i<command_count;++i)assert(!mock_commands[i].active);
    for(i=0;i<ref_count;++i)assert(!mock_refs[i].registered);
    setup();registration_fail=5;assert(!fcu_init());for(i=0;i<ref_count;++i)assert(!mock_refs[i].registered);
    setup();registration_fail=27;assert(!fcu_init());for(i=0;i<ref_count;++i)assert(!mock_refs[i].registered);
    setup();missing_raw="sim/flightmodel2/position/pressure_altitude";assert(!fcu_init());assert(!guidance.initialized);
}
static void test_pfd_aircraft_inputs(void) {
    int i;MockRef *envelope,*vls,*prot,*limit;
    setup();
    envelope=new_ref("a380x/xp12/systems/pfd/envelope_valid");
    vls=new_ref("a380x/xp12/systems/pfd/vls_knots");vls->value=170;
    prot=new_ref("a380x/xp12/systems/pfd/alpha_prot_knots");prot->value=155;
    limit=new_ref("a380x/xp12/systems/pfd/alpha_limit_knots");limit->value=145;
    assert(fcu_init());tick(1);
    assert(pfd_guidance.flap_valid&&pfd_guidance.slat_valid&&pfd_guidance.speedbrake_valid);
    assert(pfd_guidance.flap_config==0&&pfd_guidance.vmax_knots==340);
    assert(pfd_guidance.vfe_next_knots==263&&pfd_guidance.clean_performance_valid);
    assert(fabs(pfd_guidance.vls_knots-189.0161)<.001&&!pfd_guidance.envelope_valid);
    assert(fabs(pfd_green_dot(400000,10000)-205)<.001);
    assert(fabs(pfd_green_dot(425000,15000)-219.25)<.001);
    assert(pfd_green_dot(NAN,10000)==0&&pfd_green_dot(200000,10000)==0);
    for(i=0;i<ARRAY_LEN(pfd_outputs);++i)
        assert(r(pfd_outputs[i].handle)->registered&&!r(pfd_outputs[i].handle)->writable);
    r(pfd_input_refs[PI_FLAP_HANDLE])->value=.5;r(pfd_input_refs[PI_FLAP_ACTUAL])->value=.25;
    r(pfd_input_refs[PI_SLAT_ACTUAL])->value=1;tick(1);
    assert(pfd_guidance.flap_handle_index==2&&pfd_guidance.flap_config==3);
    assert(!pfd_guidance.clean_performance_valid&&!pfd_guidance.vls_knots);
    assert(pfd_guidance.flap_target_ratio==.25&&pfd_guidance.flap_ratio==.25);
    assert(pfd_guidance.vmax_knots==220&&pfd_guidance.vfe_next_knots==196);
    assert(guidance_inputs.vmax_knots==220);
    /* Surface-target mapping follows the actual aircraft schedule. */
    r(pfd_input_refs[PI_FLAP_ANGLES])->array_f[3]=12;tick(1);
    assert(fabs(pfd_guidance.flap_target_ratio-.3)<.001);
    r(pfd_input_refs[PI_FLAP_ANGLES])->array_f[3]=10;
    r(pfd_input_refs[PI_FLAP_HANDLE])->value=1;r(pfd_input_refs[PI_FLAP_ACTUAL])->value=1;tick(1);
    assert(pfd_guidance.vmax_knots==182&&pfd_guidance.vfe_next_knots==0);
    r(pfd_input_refs[PI_FLAP_HANDLE])->value=0;tick(1);
    assert(pfd_guidance.vmax_knots==182&&pfd_guidance.flap_target_ratio==0);
    r(pfd_input_refs[PI_FLAP_ACTUAL])->value=.625;tick(1);
    assert(pfd_guidance.vmax_knots==196); /* actual25deg, native detent5 -> CONF3 */
    r(pfd_input_refs[PI_FLAP_ACTUAL])->value=.5;tick(1);
    assert(pfd_guidance.vmax_knots==196); /* actual20deg, native detent4 -> CONF3 */
    r(pfd_input_refs[PI_FLAP_ACTUAL])->value=.25;tick(1);
    assert(pfd_guidance.vmax_knots==220); /* actual10deg -> CONF2 */
    r(pfd_input_refs[PI_FLAP_ACTUAL])->value=.125;tick(1);
    assert(pfd_guidance.vmax_knots==222); /* actual5deg -> CONF1+F */
    r(pfd_input_refs[PI_FLAP_ANGLES])->array_f[3]=NAN;tick(1);
    assert(!pfd_guidance.flap_valid); /* no invented mapping on invalid schedule */
    r(pfd_input_refs[PI_FLAP_ANGLES])->array_f[3]=10;
    r(pfd_input_refs[PI_FLAP_ACTUAL])->value=0;tick(1);
    assert(pfd_guidance.vmax_knots==263); /* remaining slat extension */
    r(pfd_input_refs[PI_SLAT_ACTUAL])->value=0;
    for(i=0;i<5;++i)r(pfd_input_refs[PI_GEAR])->array_f[i]=1;
    tick(1);assert(pfd_guidance.gear_down&&pfd_guidance.vmax_knots<=250);
    r(pfd_input_refs[PI_SPOILER_HANDLE])->value=-.5;
    r(pfd_input_refs[PI_SPOILER_ACTUAL])->value=.7;tick(1);
    assert(pfd_guidance.spoilers_armed&&fabs(pfd_guidance.spoiler_ratio-.7)<.001);
    r(pfd_input_refs[PI_FLAP_FAILURE])->value=6;tick(1);assert(pfd_guidance.flap_fault);
    r(pfd_input_refs[PI_STALL])->value=1;tick(1);assert(pfd_guidance.stall_warning);
    r(refs.on_ground)->value=1;tick(1);assert(!pfd_guidance.stall_warning&&pfd_guidance.on_ground);
    envelope->value=1;tick(1);
    assert(pfd_guidance.envelope_valid&&pfd_guidance.vls_knots==170);
    assert(!pfd_guidance.clean_performance_valid);
    assert(pfd_guidance.alpha_prot_knots==155&&pfd_guidance.alpha_limit_knots==145);
    assert(guidance_inputs.vls_knots==170);
    limit->value=160;tick(1);assert(!pfd_guidance.alpha_prot_knots&&!pfd_guidance.alpha_limit_knots);
    vls->value=NAN;tick(1);assert(!pfd_guidance.vls_knots);
    envelope->value=0;tick(1);assert(!pfd_guidance.vls_knots&&!pfd_guidance.alpha_prot_knots);
    assert(fabs(XPLMGetDataf(XPLMFindDataRef("a380x/xp12/pfd/vmax_knots"))-pfd_guidance.vmax_knots)<.001);
    fcu_shutdown();for(i=0;i<ref_count;++i)assert(!mock_refs[i].registered);
}
static void test_clean_performance_boundary(void) {
    setup();assert(fcu_init());tick(1);
    /* Known pinned PRIM nodes include the rows that differ from the TS table. */
    assert(pfd_clean_vls(250000,0)==149&&pfd_clean_vls(600000,43000)==250);
    assert(pfd_clean_vls(400000,15000)==193);
    assert(pfd_clean_vls(550000,38000)==262&&pfd_clean_vls(600000,38000)==280);
    assert(fabs(pfd_clean_vls(425000,12500)-198.25)<.0001);
    assert(pfd_clean_vls(249999,10000)==0&&pfd_clean_vls(600001,10000)==0);
    assert(pfd_clean_vls(400000,-1)==0&&pfd_clean_vls(400000,43001)==0);
    assert(pfd_clean_vls(NAN,10000)==0&&pfd_clean_vls(400000,NAN)==0);
    assert(pfd_guidance.clean_performance_valid&&guidance_inputs.vls_knots>0);
    assert(!pfd_guidance.envelope_valid&&!pfd_guidance.alpha_prot_knots&&
           !pfd_guidance.alpha_limit_knots&&!pfd_guidance.f_speed_knots&&!pfd_guidance.s_speed_knots);
    /* Port VSW without a provider (FBW VS1g tables, max(1.05 VS1g, VS1g + 5 kt) at 1 g): shown, below VLS. */
    assert(pfd_guidance.stall_warning_knots>100&&pfd_guidance.stall_warning_knots<guidance_inputs.vls_knots);
    r(pfd_input_refs[PI_FLAP_HANDLE])->value=.01;tick(1);
    assert(!pfd_guidance.clean_performance_valid&&!pfd_guidance.vls_knots);
    r(pfd_input_refs[PI_FLAP_HANDLE])->value=0;r(pfd_input_refs[PI_FLAP_ACTUAL])->value=.001;tick(1);
    assert(!pfd_guidance.clean_performance_valid&&!pfd_guidance.vls_knots);
    r(pfd_input_refs[PI_FLAP_ACTUAL])->value=0;r(pfd_input_refs[PI_SLAT_ACTUAL])->value=.001;tick(1);
    assert(!pfd_guidance.clean_performance_valid&&!pfd_guidance.vls_knots);
    r(pfd_input_refs[PI_SLAT_ACTUAL])->value=0;r(pfd_input_refs[PI_SPOILER_HANDLE])->value=.1;tick(1);
    assert(!pfd_guidance.clean_performance_valid&&!pfd_guidance.vls_knots);
    r(pfd_input_refs[PI_SPOILER_HANDLE])->value=0;r(pfd_input_refs[PI_SPOILER_ACTUAL])->value=.001;tick(1);
    assert(!pfd_guidance.clean_performance_valid&&!pfd_guidance.vls_knots);
    r(pfd_input_refs[PI_SPOILER_ACTUAL])->value=0;r(pfd_input_refs[PI_SPOILER_HANDLE])->value=-.5;tick(1);
    assert(pfd_guidance.clean_performance_valid&&pfd_guidance.vls_knots>0&&pfd_guidance.spoilers_armed);
    refs.side[0].air_valid=0;tick(1);assert(!pfd_guidance.clean_performance_valid&&!pfd_guidance.vls_knots);
    refs.side[0].air_valid=1;r(fcu.weight)->value=249999;tick(1);
    assert(!pfd_guidance.clean_performance_valid&&!pfd_guidance.vls_knots);
    r(fcu.weight)->value=400000;r(fcu.pressure_altitude)->value=43001;tick(1);
    assert(!pfd_guidance.clean_performance_valid&&!pfd_guidance.vls_knots);
    r(fcu.pressure_altitude)->value=10000;r(pfd_input_refs[PI_FLAP_FAILURE])->value=6;tick(1);
    assert(!pfd_guidance.clean_performance_valid&&!pfd_guidance.vls_knots);
    r(pfd_input_refs[PI_FLAP_FAILURE])->value=0;tick(1);
    assert(pfd_guidance.clean_performance_valid&&pfd_guidance.vls_knots==189);
    assert(XPLMGetDatai(XPLMFindDataRef("a380x/xp12/pfd/clean_performance_valid"))==1);
    fcu_shutdown();
}
static void test_raw_sensor_and_radio_contracts(void) {
    setup();assert(fcu_init());r(fcu.magvar)->value=7;r(refs.ground_track)->value=101;
    refs.side[1].ias_value=260;refs.side[1].heading_value=120;guidance.ap2=1;
    r(refs.side[0].baro)->value=30;r(refs.side[1].baro)->value=29;
    r(refs.side[1].baro_standard)->value=1;r(refs.side[1].nav_loc)->value=.5;r(refs.side[1].nav_gs)->value=-.7;
    r(fcu.g_axial)->value=-.1;r(fcu.g_side)->value=.2;r(fcu.g_normal)->value=1.3;
    read_guidance_inputs(.1f);
    assert(guidance_inputs.ias_knots==260&&guidance_inputs.heading_deg==120);
    assert(fabs(guidance_inputs.true_heading_deg-127)<.001&&fabs(guidance_inputs.track_deg-94)<.001);
    assert(fabs(guidance_inputs.pressure_altitude_ft-10020.125)<.001);
    assert(fabs(guidance_inputs.accel_x_m_s2-.981)<.001&&fabs(guidance_inputs.accel_y_m_s2-1.962)<.001);
    assert(fabs(guidance_inputs.accel_z_m_s2-2.943)<.001);
    assert(fabs(refs.side[0].baro_hpa_value-1015.9166)<.01&&refs.side[1].baro_std);
    assert(guidance_inputs.ils[1].gs_valid&&fabs(guidance_inputs.ils[1].gs_deviation_dots-.7)<.001);
    assert(fabs(guidance_inputs.ils[1].gs_angle_deg-3.2)<.001);
    assert(!strcmp(refs.side[1].nav_ident,"ITEST")&&fabs(refs.side[1].nav_frequency_mhz-110.30)<.001);
    assert(refs.side[1].nav_dme_valid&&fabs(refs.side[1].nav_dme_nm-12.5)<.001);
    refs.side[1].mach_value=.89f;r(fcu.mmo)->value=.89;read_guidance_inputs(.1f);
    assert(fabs(guidance_inputs.vmax_knots-260)<.001);
    r(fcu.mmo)->value=.85;read_guidance_inputs(.1f);assert(guidance_inputs.vmax_knots<260);
    r(fcu.mmo)->value=0;read_guidance_inputs(.1f);assert(guidance_inputs.vmax_knots==340);
    r(refs.side[1].nav_type)->value=4;r(refs.side[1].nav_frequency)->value=11390;read_guidance_inputs(.1f);assert(!guidance_inputs.ils[1].loc_valid&&!guidance_inputs.ils[1].gs_valid);
    r(refs.side[1].nav_frequency)->value=11020;read_guidance_inputs(.1f);assert(!guidance_inputs.ils[1].loc_valid);
    r(refs.side[1].nav_frequency)->value=11030;r(refs.side[1].nav_type)->value=8;read_guidance_inputs(.1f);assert(guidance_inputs.ils[1].loc_valid);
    r(refs.side[1].nav_type)->value=4;r(fcu.gs_angle[1])->value=NAN;read_guidance_inputs(.1f);assert(!guidance_inputs.ils[1].gs_valid);
    r(refs.side[1].nav_course)->value=NAN;read_guidance_inputs(.1f);assert(!guidance_inputs.ils[1].loc_valid);
    r(refs.side[1].nav_has_dme)->value=0;read_guidance_inputs(.1f);assert(!refs.side[1].nav_dme_valid&&guidance_inputs.ils[1].distance_nm==0);
    r(fcu.g_normal)->value=NAN;read_guidance_inputs(.1f);assert(!guidance_inputs.attitude_valid);
    fcu_shutdown();
}
static void test_override_lifecycles(void) {
    setup();assert(fcu_init());tick(70);refs.side[1].attitude_valid=0;command("ap2_toggle");assert(!guidance.ap2);refs.side[1].attitude_valid=1;engage();assert(command_once_count==2);
    command("athr_toggle");tick(1);assert(guidance.athr_active&&fcu.own_throttle);
    command("athr_disconnect");assert(fcu.own_pitch&&fcu.own_roll&&!fcu.own_throttle);
    command("athr_toggle");tick(1);command("ap_disconnect");assert(!fcu.own_pitch&&!fcu.own_roll&&fcu.own_throttle);
    command("ap1_toggle");tick(1);assert(guidance.ap1);
    r(fcu.paused)->value=1;tick(1);assert(!fcu.own_pitch&&!fcu.own_roll&&!fcu.own_throttle);
    r(fcu.paused)->value=0;tick(1);assert(fcu.own_pitch&&fcu.own_roll&&fcu.own_throttle);
    r(fcu.replay)->value=1;tick(1);assert(!fcu.own_pitch&&!fcu.own_roll&&!fcu.own_throttle);
    r(fcu.replay)->value=0;tick(1);
    refs.side[0].attitude_valid=0;tick(1);
    assert(!guidance.ap1&&!fcu.own_pitch&&!fcu.own_roll&&guidance.athr_active&&fcu.own_throttle);
    refs.side[0].air_valid=0;tick(1);assert(!guidance.athr_active&&!fcu.own_throttle);
    fcu_shutdown();
    setup();assert(fcu_init());engage();r(fcu.override_roll)->value=0;tick(1);assert(!guidance.ap1&&!fcu.own_pitch&&!fcu.own_roll);
    command("athr_toggle");tick(1);assert(fcu.own_throttle);r(fcu.override_throttle)->value=0;tick(1);assert(!guidance.athr_armed&&!guidance.athr_active&&!fcu.own_throttle);
    r(fcu.override_throttle)->value=1;command("athr_toggle");assert(!guidance.athr_armed);
    assert(strstr(mock_spoken,"A/THR not armed"));
    guidance.athr_armed=1;tick(1);assert(!guidance.athr_active&&XPLMGetDatai(fcu.override_throttle)==1);
    r(fcu.override_pitch)->value=1;command("ap1_toggle");assert(!guidance.ap1);
    assert(strstr(mock_spoken,"AP1 not engaged: the flight controls are held by another add-on"));
    fcu_shutdown();assert(XPLMGetDatai(fcu.override_pitch)==1&&XPLMGetDatai(fcu.override_throttle)==1);
}
/* Rudder/nosewheel lease (a380x_autoland.inc ALIGN / ROLL OUT): same ownership and release rules. */
static void land_rollout_state(void) {
    guidance.land.land_active=guidance.land.flare_active=guidance.land.rollout_active=1;
    guidance.land.rollout_confirm_s=guidance.land.flare_confirm_s=1;
}
/* LAND capability redundancy inputs and the annunciated D1/D2 message (a380x_autoland.inc). */
static void test_land_capability_inputs(void) {
    setup();assert(fcu_init());
    r(fcu.hydraulic_pressure[0])->value=3000;r(fcu.hydraulic_pressure[1])->value=3000;read_guidance_inputs(.1f);
    assert(guidance_inputs.hydraulics_valid&&guidance_inputs.rudder_valid&&guidance_inputs.yaw_channel_available);
    assert(guidance_inputs.side_attitude_valid[0]&&guidance_inputs.side_attitude_valid[1]);
    assert(guidance_inputs.side_airdata_valid[0]&&guidance_inputs.side_airdata_valid[1]);
    assert(fabs(guidance_inputs.ils[0].frequency_mhz-110.30)<.001&&fabs(guidance_inputs.ils[1].frequency_mhz-110.30)<.001);
    r(fcu.hydraulic_pressure[1])->value=500;read_guidance_inputs(.1f);assert(!guidance_inputs.hydraulics_valid);
    r(fcu.rudder_failure[1])->value=6;read_guidance_inputs(.1f);assert(!guidance_inputs.rudder_valid);
    r(fcu.rudder_failure[1])->value=0;r(fcu.override_heading)->value=1;read_guidance_inputs(.1f);
    assert(guidance_inputs.rudder_valid&&!guidance_inputs.yaw_channel_available);
    r(fcu.override_heading)->value=0;refs.side[1].radio_altitude_valid=0;read_guidance_inputs(.1f);
    assert(guidance_inputs.side_radio_height_valid[0]&&!guidance_inputs.side_radio_height_valid[1]);
    refs.side[1].radio_altitude_valid=1;
    guidance.land.capability=A380X_LAND_CAPABILITY_LAND3_DUAL;publish_guidance();assert(published.fma_d1d2==A380X_D1D2_LAND_3_DUAL);
    guidance.land.capability=A380X_LAND_CAPABILITY_LAND3_SINGLE;publish_guidance();assert(published.fma_d1d2==A380X_D1D2_LAND_3_SINGLE);
    guidance.land.capability=A380X_LAND_CAPABILITY_LAND2;publish_guidance();assert(published.fma_d1d2==A380X_D1D2_LAND_2);
    guidance.land.capability=A380X_LAND_CAPABILITY_NONE;guidance.approach_selected=1;publish_guidance();
    assert(published.fma_d1d2==A380X_D1D2_APPR_1);
    fcu_shutdown();
}
static void test_yaw_lease(void) {
    setup();assert(fcu_init());engage();assert(!fcu.own_yaw);
    land_rollout_state();tick(1);
    assert(guidance.land.yaw_active&&fcu.own_yaw&&XPLMGetDatai(fcu.override_heading)==1);
    assert(!pilot_override());   /* our own pedal output is not a pilot pedal input */
    r(fcu.paused)->value=1;tick(1);assert(!fcu.own_yaw&&XPLMGetDatai(fcu.override_heading)==0);
    r(fcu.paused)->value=0;tick(1);assert(fcu.own_yaw&&guidance.ap1);
    r(fcu.replay)->value=1;tick(1);assert(!fcu.own_yaw);r(fcu.replay)->value=0;tick(1);assert(fcu.own_yaw);
    command("ap_disconnect");assert(!fcu.own_yaw&&XPLMGetDatai(fcu.override_heading)==0&&XPLMGetDataf(fcu.pilot_rudder)==0);
    command("ap1_toggle");tick(1);assert(guidance.ap1&&fcu.own_yaw);
    r(fcu.override_heading)->value=0;tick(1);   /* lease taken away by someone else */
    assert(!guidance.ap1&&!fcu.own_yaw&&!fcu.own_pitch);
    r(fcu.override_heading)->value=1;command("ap1_toggle");tick(1);   /* foreign owner when yaw is needed */
    assert(!guidance.ap1&&!fcu.own_yaw&&XPLMGetDatai(fcu.override_heading)==1);
    fcu_shutdown();assert(XPLMGetDatai(fcu.override_heading)==1);   /* a foreign lease is never cleared */
    setup();assert(fcu_init());engage();land_rollout_state();tick(1);assert(fcu.own_yaw);
    r(fcu.axis_assignments)->array_i[2]=3;r(fcu.axis_minimum)->array_f[2]=0;r(fcu.axis_maximum)->array_f[2]=1;
    r(fcu.axis_values)->array_f[2]=.95f;tick(1);   /* physical pedals pushed: pilot override */
    assert(!guidance.ap1&&!fcu.own_yaw&&XPLMGetDatai(fcu.override_heading)==0);
    r(fcu.axis_values)->array_f[2]=.5f;command("ap1_toggle");tick(1);assert(fcu.own_yaw);
    fcu_shutdown();assert(XPLMGetDatai(fcu.override_heading)==0&&!fcu.own_yaw);
}
static void test_hardware_and_disconnect(void) {
    setup();assert(fcu_init());r(fcu.axis_assignments)->array_i[0]=1;
    r(fcu.axis_minimum)->array_f[0]=.1f;r(fcu.axis_maximum)->array_f[0]=.5f;r(fcu.axis_values)->array_f[0]=.3f;
    engage();assert(!pilot_override());r(fcu.axis_values)->array_f[0]=.45f;tick(1);assert(!guidance.ap1&&!fcu.own_pitch);
    r(fcu.axis_values)->array_f[0]=.3f;command("ap1_toggle");tick(1);assert(guidance.ap1);
    XPLMCommandOnce(fcu.native_disconnect);assert(!guidance.ap1&&!fcu.own_pitch&&!fcu.own_roll);
    command("ap1_toggle");tick(1);r(refs.avionics_power)->value=0;tick(1);assert(!guidance.ap1&&!guidance.athr_armed&&!fcu.own_pitch);
    fcu_shutdown();
}
/* AP pressed on the ground is kept and engages after liftoff above 100 ft; a second press cancels. */
static void test_ap_after_liftoff(void) {
    setup();assert(fcu_init());
    r(refs.on_ground)->value=1;refs.side[0].radio_altitude_value=0;tick(10);
    command("ap1_toggle");tick(1);assert(!guidance.ap1&&afs_ap_pending==1&&strstr(mock_spoken,"AP1 armed"));
    r(refs.on_ground)->value=0;refs.side[0].radio_altitude_value=50;tick(2);assert(!guidance.ap1&&afs_ap_pending==1);
    refs.side[0].radio_altitude_value=150;tick(2);
    assert(guidance.ap1&&!afs_ap_pending&&fcu.own_pitch&&strstr(mock_spoken,"AP1 engaged"));
    command("ap1_toggle");tick(1);assert(!guidance.ap1&&!fcu.own_pitch);
    r(refs.on_ground)->value=1;refs.side[0].radio_altitude_value=0;tick(2);
    command("ap1_toggle");assert(afs_ap_pending==1);command("ap1_toggle");assert(!afs_ap_pending&&!guidance.ap1);
    command("ap1_toggle");assert(afs_ap_pending==1);command("ap_disconnect");assert(!afs_ap_pending);
    refs.side[0].radio_altitude_value=8000;r(refs.on_ground)->value=0;
    fcu_shutdown();
}
static int calibration_text(const char *text,size_t length) {
    char path[1200];FILE *file;FcuThrustBoundary parsed={0};int valid;
    snprintf(path,sizeof(path),"%s/parser-fixture.ini",mock_aircraft_root);
    file=fopen(path,"wb");assert(file);assert(fwrite(text,1,length,file)==length);assert(!fclose(file));
    file=fopen(path,"rb");assert(file);valid=fcu_thrust_parse(file,&parsed);fclose(file);unlink(path);return valid;
}
static void test_thrust_calibration_parser(void) {
    char text[5000];size_t n;
    setup();assert(calibration_text(mock_calibration,strlen(mock_calibration)));
    snprintf(text,sizeof(text),"%scl_detent_ratio=.5\n",mock_calibration);assert(!calibration_text(text,strlen(text)));
    snprintf(text,sizeof(text),"%sunknown=1\n",mock_calibration);assert(!calibration_text(text,strlen(text)));
    strcpy(text,"version=1\nidle_detent_ratio=.01\ncl_detent_ratio=nan\nmct_detent_ratio=.78\ntoga_detent_ratio=1\ndetent_tolerance_ratio=.02\n");assert(!calibration_text(text,strlen(text)));
    strcpy(text,"version=1\nidle_detent_ratio=.01\ncl_detent_ratio=.6\nmct_detent_ratio=.6\ntoga_detent_ratio=1\ndetent_tolerance_ratio=.02\n");assert(!calibration_text(text,strlen(text)));
    strcpy(text,"version=1\nidle_detent_ratio=.01\ncl_detent_ratio=.6\nmct_detent_ratio=.63\ntoga_detent_ratio=1\ndetent_tolerance_ratio=.02\n");assert(!calibration_text(text,strlen(text)));
    strcpy(text,"version=1\nidle_detent_ratio=.01\ncl_detent_ratio=.6\nmct_detent_ratio=.78\ntoga_detent_ratio=1.1\ndetent_tolerance_ratio=.02\n");assert(!calibration_text(text,strlen(text)));
    strcpy(text,"version=1\nidle_detent_ratio=.01\ncl_detent_ratio=.6\nmct_detent_ratio=.78\ntoga_detent_ratio=1\n");assert(!calibration_text(text,strlen(text)));
    snprintf(text,sizeof(text),"%s",mock_calibration);n=strlen(text);text[n++]='\0';text[n++]='x';assert(!calibration_text(text,n));
    memset(text,'#',sizeof(text));assert(!calibration_text(text,sizeof(text)));
    n=strlen(mock_calibration);memcpy(text,mock_calibration,n);text[n-1]=0;assert(calibration_text(text,n-1));
}
static void test_raw_pilot_thrust_boundary(void) {
    A380XGuidanceInputs in={0};int i;char saved_path[1200];MockRef *valid,*request,*mode,*temp,*healthy;
    setup();assert(fcu_init());assert(fcu_thrust.calibrated);
    assert(strstr(fcu_thrust.calibration_path,"fcu-calibration.ini")&&strstr(fcu_thrust.aircraft_path,"User Selected Aircraft.acf"));
    for(i=0;i<4;++i) {r(fcu_thrust.levers)->array_f[i]=.55f;r(fcu_thrust.prop_mode)->array_i[i]=1;r(fcu.engine_throttle)->array_f[i]=.99f;}
    fcu_thrust_read_into(&in);assert(in.throttle_levers_valid&&fabs(in.throttle_lever_ratio[0]-.55)<.0001);
    assert(in.idle_detent_ratio==.01&&fabs(in.cl_detent_ratio-25./45)<1e-12);
    r(fcu_thrust.prop_mode)->array_i[2]=3;fcu_thrust_read_into(&in);assert(in.reverse_selected[2]&&in.throttle_levers_valid);
    r(fcu_thrust.levers)->array_f[3]=NAN;fcu_thrust_read_into(&in);assert(!in.throttle_levers_valid&&strstr(mock_debug,"invalid pilot throttle"));
    r(fcu_thrust.levers)->array_f[3]=1.01f;fcu_thrust_read_into(&in);assert(!in.throttle_levers_valid);
    r(fcu_thrust.levers)->array_f[3]=.5f;r(fcu_thrust.prop_mode)->count=3;fcu_thrust_read_into(&in);assert(!in.throttle_levers_valid);
    r(fcu_thrust.prop_mode)->count=4;r(fcu_thrust.prop_mode)->array_i[2]=4;fcu_thrust_read_into(&in);assert(!in.throttle_levers_valid);
    r(fcu_thrust.prop_mode)->array_i[2]=1;fcu_thrust_read_into(&in);assert(in.throttle_levers_valid);
    assert(!in.alpha_floor_valid&&!in.alpha_floor_request&&!in.flex_valid);
    valid=new_ref("a380x/xp12/flight_envelope/valid");request=new_ref("a380x/xp12/flight_envelope/alpha_floor_request");
    valid->value=1;request->value=1;fcu_thrust_read_into(&in);assert(in.alpha_floor_valid&&in.alpha_floor_request);
    valid->value=0;fcu_thrust_read_into(&in);assert(!in.alpha_floor_valid&&!in.alpha_floor_request);
    valid->value=1;request->value=2;fcu_thrust_read_into(&in);assert(!in.alpha_floor_valid&&!in.alpha_floor_request);
    request->value=0;fcu_thrust_read_into(&in);assert(in.alpha_floor_valid&&!in.alpha_floor_request);
    mode=new_ref("a380x/xp12/fms/to_power_setting");temp=new_ref("a380x/xp12/fms/to_flex_temp_c");healthy=new_ref("a380x/xp12/fms/fmc_b_healthy");
    mode->value=1;temp->value=.1;healthy->value=1;fcu_thrust_read_into(&in);assert(in.flex_valid);
    temp->value=0;fcu_thrust_read_into(&in);assert(!in.flex_valid);
    temp->value=50;mode->value=0;fcu_thrust_read_into(&in);assert(!in.flex_valid);
    mode->value=1;healthy->value=0;fcu_thrust_read_into(&in);assert(!in.flex_valid);
    healthy->value=1;temp->value=NAN;fcu_thrust_read_into(&in);assert(!in.flex_valid);
    strcpy(saved_path,mock_aircraft_path);strcpy(mock_aircraft_path,"/different/Unverified.acf");
    fcu_thrust_init();fcu_thrust_read_into(&in);assert(!in.throttle_levers_valid&&!fcu_thrust.calibrated);
    strcpy(mock_aircraft_path,saved_path);fcu_thrust_init();fcu_thrust_read_into(&in);assert(in.throttle_levers_valid);
    fcu_shutdown();
}
static void stock_command(const char *name) {XPLMCommandRef c=XPLMFindCommand(name);assert(c);XPLMCommandOnce(c);}
static void test_stock_authority_bridge(void) {
    MockCommand *stock;int i;
    setup();assert(fcu_init());tick(70);
    stock_command("sim/autopilot/servos_on");tick(1);assert(guidance.ap1&&fcu.own_pitch&&!XPLMGetDatai(fcu_authority.stock_servos[0]));
    stock_command("sim/autopilot/servos_on");assert(guidance.ap1); /* ON remains idempotent. */
    stock=XPLMFindCommand("sim/autopilot/servos_toggle");assert(!stock->callback(stock,xplm_CommandContinue,stock->refcon));assert(guidance.ap1);
    stock_command("sim/autopilot/servos2_on");tick(1);assert(!guidance.ap1&&guidance.ap2&&!XPLMGetDatai(fcu_authority.stock_servos[1]));
    stock_command("sim/autopilot/servos2_toggle");assert(!guidance.ap2&&!fcu.own_pitch);
    stock_command("sim/autopilot/servos3_on");assert(!guidance.ap1&&!guidance.ap2&&!XPLMGetDatai(fcu_authority.stock_servos[2]));
    stock_command("sim/autopilot/servos_on");tick(1);r(fcu_authority.stock_servos[2])->value=1;tick(1);
    assert(!guidance.ap1&&!fcu.own_pitch&&!fcu.own_roll&&XPLMGetDatai(fcu_authority.stock_servos[2])==1);
    stock_command("sim/autopilot/servos_off_any");assert(!XPLMGetDatai(fcu_authority.stock_servos[2]));
    stock_command("sim/autopilot/autothrottle_on");tick(1);assert(guidance.athr_armed&&guidance.athr_active&&fcu.own_throttle);
    assert(XPLMGetDatai(fcu_authority.stock_athr)==-1);
    stock_command("sim/autopilot/autothrottle_on");assert(guidance.athr_armed);
    stock_command("sim/autopilot/autothrottle_off");assert(!guidance.athr_armed&&!fcu.own_throttle&&XPLMGetDatai(fcu_authority.stock_athr)==-1);
    stock_command("sim/autopilot/autothrottle_on");tick(1);r(fcu_authority.stock_athr)->value=2;tick(1);
    assert(!guidance.athr_active&&!guidance.athr_armed&&!fcu.own_throttle&&XPLMGetDatai(fcu_authority.stock_athr)==2);
    plugin_enabled=0;stock_command("sim/autopilot/servos_on");assert(XPLMGetDatai(fcu_authority.stock_servos[0])==1&&!guidance.ap1);
    fcu_shutdown();for(i=0;i<command_count;++i)assert(!mock_commands[i].active);
}
static void test_individual_engine_lever_limits(void) {
    const float levers[4]={0,.2f,25.f/45,.4f};int i;MockRef *valid,*request;
    setup();assert(fcu_init());refs.side[0].ias_value=200;
    for(i=0;i<4;++i) {r(fcu_thrust.levers)->array_f[i]=levers[i];throttle_values[i]=levers[i];}
    tick(70);command("athr_toggle");tick(100);assert(guidance.athr_active&&fcu.own_throttle);
    assert(r(fcu.engine_throttle)->array_f[0]==0);
    for(i=1;i<4;++i) {
        double limit=fmin(.88,levers[i]/guidance_inputs.cl_detent_ratio*.88);
        assert(r(fcu.engine_throttle)->array_f[i]<=limit+1e-6);
        assert(r(fcu.engine_throttle)->array_f[i]>0);
    }
    assert(r(fcu.engine_throttle)->array_f[2]>r(fcu.engine_throttle)->array_f[1]);
    valid=new_ref("a380x/xp12/flight_envelope/valid");request=new_ref("a380x/xp12/flight_envelope/alpha_floor_request");
    valid->value=request->value=1;tick(1);assert(guidance.thrust_mode==A380X_A1A2_A_FLOOR);
    for(i=0;i<4;++i)assert(r(fcu.engine_throttle)->array_f[i]==1);
    request->value=0;tick(1);assert(guidance.thrust_mode==A380X_A1A2_TOGA_LK);
    for(i=0;i<4;++i)assert(r(fcu.engine_throttle)->array_f[i]==1);
    command("athr_disconnect");assert(!guidance.athr_active&&!fcu.own_throttle);fcu_shutdown();
}
int main(int argc,char **argv) {
    int i;
    if(argc>1&&!strcmp(argv[1],"--contracts")){
        setup();assert(fcu_init());assert(lighting_init());
        for(i=0;i<ARRAY_LEN(fcu_commands);++i)printf("C %s\n",fcu_commands[i].name);
        for(i=0;i<ARRAY_LEN(fcu_outputs);++i)printf("D %s\n",fcu_outputs[i].name);
        for(i=0;i<ARRAY_LEN(fcu_motion_outputs);++i)printf("M %s\n",fcu_motion_outputs[i].name);
        for(i=0;i<FCU_LIGHT_COUNT;++i)printf("L a380x/xp12/fcu/%s\n",fcu_light_names[i]);
        for(i=0;i<ARRAY_LEN(pfd_outputs);++i)printf("P %s\n",pfd_outputs[i].name);
        for(i=0;i<ref_count;++i)if(!strncmp(mock_refs[i].name,"sim/",4))printf("R %s\n",mock_refs[i].name);
        lighting_shutdown();fcu_shutdown();return 0;
    }
    test_registry_and_partial_init();test_raw_sensor_and_radio_contracts();test_override_lifecycles();test_hardware_and_disconnect();
    test_yaw_lease();test_land_capability_inputs();
    test_glareshield_lighting();
    test_display_unit_controls();
    test_pfd_aircraft_inputs();
    test_clean_performance_boundary();
    test_thrust_calibration_parser();test_raw_pilot_thrust_boundary();test_stock_authority_bridge();
    test_individual_engine_lever_limits();
    test_ap_after_liftoff();
    puts("a380x FCU XPLM adapter tests passed");return 0;
}
