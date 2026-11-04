/* SPDX-License-Identifier: GPL-3.0-or-later
 * ECAM SD boundary. Shares only the SDK symbol table and tested GL renderer
 * with the MFD, not its mouse queue, menus, WebKit transport or page controller.
 */
#include "a380x_sd.h"
#include "a380x_ecam_button.h"
#include "a380x_elec_power.h"
/* CDS boot / self-test screen hook (a380x_avionics.c defines it before this include). */
#ifndef SDA_CDS_OVERLAY
#define SDA_CDS_OVERLAY() 0
#endif

#define SDA_DEVICE "a380x.xp12.ecam.sd"
#define SDA_PREFIX "a380x/xp12/ecam/sd/"
#define SDA_ENG_PREFIX "a380x/xp12/ecam/buttons/eng/"
/* ENG pilot only. Press distance/axis are authored in the generated OBJ,
 * not guessed in Lua. Add other keys only after the cockpit acceptance test. */
static const EcamButtonConfig sda_eng_config={"eng","ENG",SD_ENGINE,.10,.15};
enum { SDA_N2, SDA_FF, SDA_OIL_TEMP, SDA_OIL_PRESS, SDA_OIL_C,
       SDA_APU_N1, SDA_APU_EGT, SDA_APU_RUNNING, SDA_APU_BLEED, SDA_RHEO,
       SDA_N3, SDA_OIL_QT, SDA_VIB1, SDA_VIB2, SDA_VIB3, SDA_NAC,
       SDA_APU_N2, SDA_APU_PRESS, SDA_APU_FUEL, SDA_GEN_LOAD, SDA_GEN_V, SDA_GEN_HZ,
       SDA_INPUTS };
static struct { const char *name; XPLMDataRef ref; } sda_inputs[SDA_INPUTS] = {
    {"sim/cockpit2/engine/indicators/N2_percent",NULL},
    {"sim/cockpit2/engine/indicators/fuel_flow_kg_sec",NULL},
    {"sim/cockpit2/engine/indicators/oil_temperature_deg_C",NULL},
    {"sim/cockpit2/engine/indicators/oil_pressure_psi",NULL},
    {"sim/aircraft/engine/acf_oilT_is_C",NULL},
    {"sim/cockpit2/electrical/APU_N1_percent",NULL},
    {"sim/cockpit2/electrical/APU_EGT_c",NULL},
    {"sim/cockpit2/electrical/APU_running",NULL},
    {"sim/cockpit2/bleedair/actuators/apu_bleed",NULL},
    {"sim/cockpit2/switches/instrument_brightness_ratio",NULL},
    /* Optional aircraft-local producers, not registered or synthesized here. */
    {"a380x/xp12/engine/n3_percent",NULL},
    {"a380x/xp12/engine/oil_quantity_quarts",NULL},
    {"a380x/xp12/engine/vibration_n1",NULL},
    {"a380x/xp12/engine/vibration_n2",NULL},
    {"a380x/xp12/engine/vibration_n3",NULL},
    {"a380x/xp12/engine/nacelle_temperature_c",NULL},
    {"a380x/xp12/apu/n2_percent",NULL},
    {"a380x/xp12/apu/bleed_pressure_psi",NULL},
    {"a380x/xp12/apu/fuel_used_kg",NULL},
    {"a380x/xp12/apu/generator_load_percent",NULL},
    {"a380x/xp12/apu/generator_volts",NULL},
    {"a380x/xp12/apu/generator_hz",NULL},
};
enum { SDA_NEXT=16, SDA_BRIGHT_UP, SDA_BRIGHT_DOWN, SDA_POPUP, SDA_ALL, SDA_MORE,
       SDA_TOCFG, SDA_CLR, SDA_CLR2, SDA_RCL, SDA_RCLLAST, SDA_CL, SDA_ABN, SDA_UP,
       SDA_DOWN, SDA_CHECK_L, SDA_CHECK_R, SDA_EMERCANC, SDA_EWD_UP, SDA_EWD_DOWN, SDA_COMMANDS };
static const char *const sda_page_names[]={"ENGINE","APU","BLEED","COND","CAB PRESS","DOOR / OXY",
    "ELEC AC","ELEC DC","FUEL","WHEEL","HYD","F/CTL","CB","CRUISE","STATUS","VIDEO"};
static struct {
    int initialized, page, powered, bound, draw_count, select_count, force;
    int manual, more, all, action_count, last_action, checklist, abnormal, cursor, checklist_group;
    unsigned checklist_bits[10], cleared[16], cancelled[16];
    float all_elapsed;
    EwdData warnings;
    char feedback[96];
    float brightness, ewd_brightness, since_read, since_find;
    uint64_t hash;
    SdData data;
    MfdDrawList list,overlay;
    XPLMAvionicsID device;
    XPLMDataRef published[16];
    XPLMDataRef button_refs[2];
    EcamButtonMotion eng_motion;
    XPLMCommandRef commands[SDA_COMMANDS];
    XPLMMenuID menu,parent;
    int menu_item;
} sda;
static const char *const sda_command_names[]={"engine","apu","bleed","cond","press","door","elec_ac","elec_dc","fuel","wheel","hyd","fctl","cb","cruise","status","video",
    "next_page","brightness_up","brightness_down","popup","all","more","to_config","clear_left","clear_right","recall","recall_last","checklist","abnormal_procedures","up","down","check_left","check_right","emergency_cancel","ewd_brightness_up","ewd_brightness_down"};
static const char *const sda_ref_names[]={"active_page","powered","device_bound","draw_count","select_count","brightness",
    "manual_selection","more","all_held","action_count","last_action","checklist_visible","checklist_cursor","checklist_group","abnormal_visible","ewd_brightness"};

static const char *const sda_checklist_names[]={"COCKPIT PREPARATION","BEFORE START","AFTER START","TAXI","LINE-UP","DEPARTURE CHANGE","APPROACH","LANDING","PARKING","SECURING AIRCRAFT"};
static const int sda_checklist_counts[]={4,3,3,8,3,4,4,4,4,4};
/* Same pinned FBW checklist entries already used by this aircraft's flyPad.
 * Deliberate manual acknowledgement, not a second automatic FWS. */
static const char *const sda_checklists[10][8]={
 {"GEAR PINS & COVERS ... REMOVED","FUEL QTY ... CHECKED","SEAT BELTS ... ON","BARO REF ... SET"},
 {"PARKING BRAKE ... SET","T.O SPEEDS & THRUST ... CHECKED","BEACON ... ON"},
 {"ANTI ICE ... AS RQRD","PITCH TRIM ... T.O","RUDDER TRIM ... NEUTRAL"},
 {"FLIGHT CONTROLS ... CHECKED","FLAPS SETTING ... CONF","RADAR ... ON","T.O SEAT BELTS ... ON","T.O GND SPLRS ... ARMED","T.O FLAPS ... SET","T.O AUTO BRAKE ... RTO","T.O CONFIG ... TESTED"},
 {"T.O RWY ... CONFIRMED","CABIN CREW ... ADVISED","PACKS 1+2 ... ON"},
 {"RWY & SID ... CONFIRMED","FLAPS SETTING ... CONFIRMED","T.O SPEEDS & THRUST ... CHECKED","FCU ALT ... SET"},
 {"BARO REF ... SET","SEAT BELTS ... ON","MINIMUM ... SET","AUTO BRAKE ... SET"},
 {"LDG SEAT BELTS ... ON","LDG GEAR ... DOWN","GND SPLRS ... ARM","FLAPS ... LDG"},
 {"PARK BRAKE OR CHOCKS ... AS RQRD","ENGINES ... OFF","WING LIGHTS ... OFF","FUEL PUMPS ... OFF"},
 {"OXYGEN ... OFF","EMER EXIT LT ... OFF","EFBs ... OFF","BATTERIES ... OFF"}};
static unsigned sda_line_hash(const char *t) { unsigned h=2166136261u;for(;*t;++t)h=(h^(unsigned char)*t)*16777619u;return h; }
static int sda_hidden_warning(const EwdLine *line) { unsigned h=sda_line_hash(line->text);for(int i=0;i<16;++i)if(sda.cleared[i]==h||sda.cancelled[i]==h)return 1;return 0; }
static void sda_filter_ewd(EwdData *out,const EwdData *in) {
    *out=*in;out->warning_count=0;
    for(int i=0;i<in->warning_count&&i<16;++i)if(!sda_hidden_warning(&in->warning[i]))out->warning[out->warning_count++]=in->warning[i];
}
static void sda_clear_warning(int cancel) {
    for(int i=0;i<sda.warnings.warning_count&&i<16;++i)if(!sda_hidden_warning(&sda.warnings.warning[i])) {
        unsigned *list=cancel?sda.cancelled:sda.cleared;
        for(int k=0;k<16;++k)if(!list[k]){list[k]=sda_line_hash(sda.warnings.warning[i].text);break;}
        return;
    }
}

static SdValue sda_array(int input,int index,float lo,float hi)
{
    float v=NAN; XPLMDataRef r=sda_inputs[input].ref;
    int ok=r && (XPLMGetDataRefTypes(r)&xplmType_FloatArray) && XPLMGetDatavf(r,&v,index,1)==1;
    return a380x_sd_value(v,ok,lo,hi);
}
static SdValue sda_scalar(int input,float lo,float hi)
{
    XPLMDataRef r=sda_inputs[input].ref;
    if(!r) return (SdValue){0,0};
    XPLMDataTypeID type=XPLMGetDataRefTypes(r);
    float v=type&xplmType_Float?XPLMGetDataf(r):type&xplmType_Int?(float)XPLMGetDatai(r):NAN;
    return a380x_sd_value(v,1,lo,hi);
}
static void sda_find_inputs(void)
{ for(int i=0;i<SDA_INPUTS;++i) sda_inputs[i].ref=XPLMFindDataRef(sda_inputs[i].name); }

static int sda_get_int(void *refcon)
{
    int index=(int)(intptr_t)refcon;
    switch(index) {
        case 0:return sda.page; case 1:return sda.powered; case 2:return sda.bound;
        case 3:return sda.draw_count;case 4:return sda.select_count;
        case 5:return (int)lroundf(sda.brightness*100);
        case 6:return sda.manual;case 7:return sda.more;case 8:return sda.all;
        case 9:return sda.action_count;case 10:return sda.last_action;case 11:return sda.checklist;
        case 12:return sda.cursor;case 13:return sda.checklist_group;case 14:return sda.abnormal;
        case 15:return (int)lroundf(sda.ewd_brightness*100);
        default:return 0;
    }
}
static float sda_get_float(void *refcon)
{ return (intptr_t)refcon==5?sda.brightness:(intptr_t)refcon==15?sda.ewd_brightness:(float)sda_get_int(refcon); }

static float sda_button_get(void *refcon)
{
    if((intptr_t)refcon==0)return sda.eng_motion.ratio;
    /* Illumination reflects the displayed page, not the transient click.
     * Checklist/ABN overlays and power loss must not falsely indicate ENG. */
    return plugin_enabled && sda.powered && !sda.checklist && !sda.abnormal
        && sda.page==sda_eng_config.page?1.0f:0.0f;
}

static void sda_screen_draw(void *refcon)
{
    (void)refcon;
    GLboolean cull=glIsEnabled(GL_CULL_FACE);
    XPLMSetGraphicsState(0,0,0,0,1,0,0);glDisable(GL_CULL_FACE);
    glClearColor(0,0,0,1);glClear(GL_COLOR_BUFFER_BIT);
    glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_FALSE);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    if(plugin_enabled && !sda.powered && SDA_CDS_OVERLAY()) {
        /* Thales boot / self-test (A380 network) */
    } else if(plugin_enabled && sda.powered) {
        /* ENGINE: FBW's page with the FBW-named data (a380x_sd_data.inc) when the xp-kit view is live,
         * the native page otherwise. APU stays native. */
        if(sda.checklist || sda.abnormal || sda.page==SD_APU || !xpkit_draw_sd())mfd_gl_draw(&sda.list,0,0,1,0);
        else if(sda.overlay.cmd_count)mfd_gl_draw(&sda.overlay,0,0,1,0);
    }
    glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);glColor4f(1,1,1,1);
    if(cull) glEnable(GL_CULL_FACE);
    ++sda.draw_count;
}

static int sda_command(XPLMCommandRef command,XPLMCommandPhase phase,void *refcon)
{
    int action=(int)(intptr_t)refcon;(void)command;
    if(!plugin_enabled || action<0 || action>=SDA_COMMANDS) return 0;
    if(action==SDA_ALL && phase==xplm_CommandEnd) { sda.all=0;sda.force=1;return 0; }
    if(phase!=xplm_CommandBegin) return 0;
    if(action==SD_ENGINE) {
        ecam_button_trigger(&sda.eng_motion);
        XPLMDebugString("A380X ECAM button ENG: click -> automatic .10s press / .15s return; ENGINE selected\n");
    }
    ++sda.action_count;sda.last_action=action;
    snprintf(sda.feedback,sizeof(sda.feedback),"%s",sda_command_names[action]);
    if(action<SD_PAGES || action==SDA_NEXT) {
        int page=action==SDA_NEXT?(sda.page+1)%SD_PAGES:action;
        if(page==SD_STS&&sda.page==SD_STS)sda.more=!sda.more;
        else {sda.manual=sda.manual&&sda.page==page?0:1;sda.page=page;sda.more=0;}
        sda.all=sda.checklist=sda.abnormal=0;++sda.select_count;
    } else if(action==SDA_BRIGHT_UP || action==SDA_BRIGHT_DOWN) {
        sda.brightness=fminf(1,fmaxf(0,roundf((sda.brightness+(action==SDA_BRIGHT_UP?.05f:-.05f))*20)/20));
    } else if(action==SDA_POPUP && sda.device) {
        mfda_av.set_popup_visible(sda.device,!mfda_av.is_popup_visible(sda.device));
    } else if(action==SDA_ALL) {sda.all=1;sda.all_elapsed=0;sda.manual=1;sda.page=(sda.page+1)%SD_PAGES;sda.checklist=sda.abnormal=0;++sda.select_count;}
    else if(action==SDA_MORE)sda.more=!sda.more;
    /* Native FWS (a380x_fws_adapter.inc) owns CLR / RCL / EMER CANC / C/L / ABN PROC / UP / DOWN / CHECK
     * and T.O CONFIG through its pass-through command taps: the SD keeps no checklist of its own. */
    else if(fwsa_available()&&(action==SDA_CLR||action==SDA_CLR2||action==SDA_EMERCANC||action==SDA_RCL||
            action==SDA_RCLLAST||action==SDA_CL||action==SDA_ABN||action==SDA_UP||action==SDA_DOWN||
            action==SDA_CHECK_L||action==SDA_CHECK_R||action==SDA_TOCFG)) sda.checklist=sda.abnormal=0;
    else if(action==SDA_CLR||action==SDA_CLR2)sda_clear_warning(0);
    else if(action==SDA_EMERCANC)sda_clear_warning(1);
    else if(action==SDA_RCL||action==SDA_RCLLAST){memset(sda.cleared,0,sizeof(sda.cleared));memset(sda.cancelled,0,sizeof(sda.cancelled));}
    else if(action==SDA_CL){sda.checklist=!sda.checklist;sda.abnormal=0;sda.cursor=0;}
    else if(action==SDA_ABN){sda.abnormal=!sda.abnormal;sda.checklist=0;sda.cursor=0;}
    else if(action==SDA_UP||action==SDA_DOWN){
        int count=sda.abnormal?sda.warnings.warning_count:sda_checklist_counts[sda.checklist_group];
        sda.cursor+=action==SDA_UP?-1:1;
        if(sda.checklist&&sda.cursor<0){sda.checklist_group=(sda.checklist_group+9)%10;sda.cursor=sda_checklist_counts[sda.checklist_group]-1;}
        else if(sda.checklist&&sda.cursor>=count){sda.checklist_group=(sda.checklist_group+1)%10;sda.cursor=0;}
        if(sda.cursor<0)sda.cursor=0;
        if(sda.abnormal&&sda.cursor>=count)sda.cursor=count>0?count-1:0;
    } else if(action==SDA_CHECK_L||action==SDA_CHECK_R){
        if(sda.checklist)sda.checklist_bits[sda.checklist_group]^=1u<<sda.cursor;
        else if(sda.abnormal)sda_clear_warning(0);
    } else if(action==SDA_TOCFG)snprintf(sda.feedback,sizeof(sda.feedback),"T.O CONFIG: FULL FWS NOT AVAILABLE");
    else if(action==SDA_EWD_UP||action==SDA_EWD_DOWN)sda.ewd_brightness=fminf(1,fmaxf(0,sda.ewd_brightness+(action==SDA_EWD_UP?.05f:-.05f)));
    sda.force=1;
    return 0;
}

static void sda_shutdown(void)
{
    for(int i=0;i<(int)ARRAY_LEN(sda.button_refs);++i)
        if(sda.button_refs[i])XPLMUnregisterDataAccessor(sda.button_refs[i]);
    for(int i=0;i<(int)ARRAY_LEN(sda.published);++i) {
        if(sda.published[i]) XPLMUnregisterDataAccessor(sda.published[i]);
    }
    for(int i=0;i<SDA_COMMANDS;++i)if(sda.commands[i])XPLMUnregisterCommandHandler(sda.commands[i],sda_command,1,(void *)(intptr_t)i);
    if(sda.device && mfda_av.loaded) mfda_av.destroy(sda.device);
    if(sda.menu) { XPLMDestroyMenu(sda.menu);XPLMRemoveMenuItem(sda.parent,sda.menu_item); }
    memset(&sda,0,sizeof(sda));
}

static int sda_init(void)
{
    char name[96];
    memset(&sda,0,sizeof(sda));sda.brightness=sda.ewd_brightness=1;sda.force=1;sda.manual=1;
    for(int i=0;i<(int)ARRAY_LEN(sda.published);++i) {
        snprintf(name,sizeof(name),SDA_PREFIX "%s",sda_ref_names[i]);
        if(XPLMFindDataRef(name)) { sda_shutdown();return 0; }
        sda.published[i]=XPLMRegisterDataAccessor(name,xplmType_Int|xplmType_Float,0,
            sda_get_int,NULL,sda_get_float,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
            (void *)(intptr_t)i,NULL);
        if(!sda.published[i]) { sda_shutdown();return 0; }
    }
    const char *const button_names[]={SDA_ENG_PREFIX "press_ratio",SDA_ENG_PREFIX "selected"};
    for(int i=0;i<(int)ARRAY_LEN(sda.button_refs);++i) {
        if(XPLMFindDataRef(button_names[i])) {sda_shutdown();return 0;}
        sda.button_refs[i]=XPLMRegisterDataAccessor(button_names[i],xplmType_Float,0,
            NULL,NULL,sda_button_get,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,
            (void *)(intptr_t)i,NULL);
        if(!sda.button_refs[i]) {sda_shutdown();return 0;}
    }
    for(int i=0;i<SDA_COMMANDS;++i) {
        snprintf(name,sizeof(name),SDA_PREFIX "%s",sda_command_names[i]);
        sda.commands[i]=XPLMFindCommand(name);
        if(!sda.commands[i]) sda.commands[i]=XPLMCreateCommand(name,name);
        if(!sda.commands[i]) { sda_shutdown();return 0; }
        XPLMRegisterCommandHandler(sda.commands[i],sda_command,1,(void *)(intptr_t)i);
    }
    sda_find_inputs();
    if(mfda_av.loaded) {
        XPLMCreateAvionics_t p;memset(&p,0,sizeof(p));p.structSize=sizeof(p);
        p.screenWidth=MFD_W;p.screenHeight=MFD_H;p.bezelWidth=MFD_W;p.bezelHeight=MFD_H;
        p.drawOnDemand=1;p.drawCallback=sda_screen_draw;p.deviceID=SDA_DEVICE;
        p.deviceName="A380X ECAM System Display";
        sda.device=mfda_av.create(&p);
        if(sda.device) mfda_av.needs_drawing(sda.device);
    }
    sda.parent=XPLMFindAircraftMenu();
    if(sda.parent) {
        sda.menu_item=XPLMAppendMenuItem(sda.parent,"ECAM System Display",NULL,0);
        sda.menu=XPLMCreateMenu("ECAM System Display",sda.parent,sda.menu_item,NULL,NULL);
        if(sda.menu) {
            for(int i=0;i<SD_PAGES;++i)XPLMAppendMenuItemWithCommand(sda.menu,sda_page_names[i],sda.commands[i]);
            XPLMAppendMenuItemWithCommand(sda.menu,"Pop up System Display",sda.commands[SDA_POPUP]);
        }
    }
    sda.initialized=1;
    XPLMDebugString("A380X ECAM SD: all 16 page selections and 36 ECP commands registered\n");
    XPLMDebugString("A380X ECAM button ENG: independent press_ratio / selected datarefs ready\n");
    return 1;
}

static void sda_update(float dt,int powered,const EwdData *engines)
{
    if(!sda.initialized) return;
    /* This runs EVERY simulator frame, before the 4 Hz display-data gate. */
    ecam_button_update(&sda.eng_motion,&sda_eng_config,dt);
    if(engines)sda.warnings=*engines;
    if(sda.all) {sda.all_elapsed+=fmaxf(0,dt);if(sda.all_elapsed>=1){sda.all_elapsed=fmodf(sda.all_elapsed,1);sda.page=(sda.page+1)%SD_PAGES;++sda.select_count;sda.force=1;}}
    int on=powered && sda.brightness>.001f;
    {   /* SD on DC 2: a CDS state change (boot / self-test / on) redraws the device */
        static int cds_state=-2;
        int state=a380x_power_du_state(A380X_ELN_DU_SD);
        if(state!=cds_state) { cds_state=state; sda.force=1; }
    }
    sda.since_read+=fmaxf(0,dt);sda.since_find+=fmaxf(0,dt);
    if(sda.since_find>=1) { sda_find_inputs();sda.since_find=0; }
    if(!sda.force && on==sda.powered && sda.since_read<.25f) return;
    sda.since_read=0;sda.powered=on;
    SdData *s=&sda.data;memset(s,0,sizeof(*s));s->powered=on;
    SdValue oil_c=sda_scalar(SDA_OIL_C,0,1);
    for(int i=0;i<4;++i) {
        SdEngine *e=&s->engine[i];
        e->active=engines && engines->engine[i].active;
        e->starting=engines && engines->engine[i].starting;
        e->n2=sda_array(SDA_N2,i,0,200);e->n3=sda_array(SDA_N3,i,0,200);
        e->fuel_flow_kg_h=sda_array(SDA_FF,i,0,100);
        if(e->fuel_flow_kg_h.valid) e->fuel_flow_kg_h.value*=3600;
        e->oil_temperature_c=sda_array(SDA_OIL_TEMP,i,-200,1000);
        if(!oil_c.valid) e->oil_temperature_c.valid=0;
        else if(!oil_c.value && e->oil_temperature_c.valid)
            e->oil_temperature_c.value=(e->oil_temperature_c.value-32)*5/9;
        e->oil_pressure_psi=sda_array(SDA_OIL_PRESS,i,0,1000);
        e->oil_quantity_qt=sda_array(SDA_OIL_QT,i,0,100);
        for(int k=0;k<3;++k) e->vibration[k]=sda_array(SDA_VIB1+k,i,0,20);
        e->nacelle_temperature_c=sda_array(SDA_NAC,i,-100,1500);
    }
    s->apu_n1=sda_scalar(SDA_APU_N1,0,150);s->apu_n2=sda_scalar(SDA_APU_N2,0,150);
    s->apu_egt_c=sda_scalar(SDA_APU_EGT,-100,1400);
    SdValue running=sda_scalar(SDA_APU_RUNNING,0,1),bleed=sda_scalar(SDA_APU_BLEED,0,1);
    s->apu_available=running.valid && running.value && s->apu_n1.valid && s->apu_n1.value>=95;
    s->apu_bleed_valid=bleed.valid;s->apu_bleed_open=bleed.valid && bleed.value;
    s->apu_bleed_psi=sda_scalar(SDA_APU_PRESS,0,200);
    s->apu_fuel_used_kg=sda_scalar(SDA_APU_FUEL,0,100000);
    for(int i=0;i<2;++i) {
        s->apu_gen_load[i]=sda_array(SDA_GEN_LOAD,i,0,300);
        s->apu_gen_volts[i]=sda_array(SDA_GEN_V,i,0,500);
        s->apu_gen_hz[i]=sda_array(SDA_GEN_HZ,i,0,1000);
    }
    a380x_sd_draw(s,sda.page,&sda.list);
    if(on && (sda.checklist||sda.abnormal)) {
        mfd_draw_reset(&sda.list);mfd_draw_fill_rect(&sda.list,mfd_rect(0,0,768,1024),MFD_COL_BLACK);
        mfd_draw_text(&sda.list,MFD_FONT_ECAM,32,MFD_COL_WHITE,MFD_ANCHOR_LEFT,20,45,sda.abnormal?"ABNORMAL / ACTIVE ALERTS":sda_checklist_names[sda.checklist_group]);
        int count=sda.abnormal?sda.warnings.warning_count:sda_checklist_counts[sda.checklist_group];
        if(count>16)count=16;
        for(int i=0;i<count;++i) {
            if(i==sda.cursor)mfd_draw_stroke_rect(&sda.list,mfd_rect(12,80+i*46,742,43),2,MFD_COL_CYAN);
            const char *t=sda.abnormal?sda.warnings.warning[i].text:sda_checklists[sda.checklist_group][i];
            mfd_draw_text(&sda.list,MFD_FONT_ECAM,24,sda.abnormal?MFD_COL_AMBER:(sda.checklist_bits[sda.checklist_group]&(1u<<i)?MFD_COL_GREEN:MFD_COL_WHITE),MFD_ANCHOR_LEFT,24,110+i*46,t);
        }
        mfd_draw_text(&sda.list,MFD_FONT_ECAM,21,MFD_COL_CYAN,MFD_ANCHOR_LEFT,20,940,sda.abnormal?"ACTIVE ALERTS ONLY - FULL FWS NOT AVAILABLE":"MANUAL ACKNOWLEDGEMENT - UP / DOWN / CHECK");
    }
    mfd_draw_reset(&sda.overlay);
    if(on && sda.feedback[0]) {
        mfd_draw_text(&sda.list,MFD_FONT_ECAM,18,MFD_COL_CYAN,MFD_ANCHOR_LEFT,15,1000,sda.feedback);
        mfd_draw_text(&sda.overlay,MFD_FONT_ECAM,18,MFD_COL_CYAN,MFD_ANCHOR_LEFT,15,1000,sda.feedback);
    }
    uint64_t h=mfda_list_hash(&sda.list,on);
    if(sda.device && mfda_av.loaded) {
        sda.bound=mfda_av.is_bound(sda.device);
        if(sda.force || h!=sda.hash) mfda_av.needs_drawing(sda.device);
        if(sda.force) {
            mfda_av.set_rheo(sda.device,sda.brightness);
            XPLMDataRef r=sda_inputs[SDA_RHEO].ref;
            if(r && (XPLMGetDataRefTypes(r)&xplmType_FloatArray)) XPLMSetDatavf(r,&sda.brightness,18,1);
        }
    }
    sda.hash=h;sda.force=0;
}

static void sda_disable(void)
{
    ecam_button_reset(&sda.eng_motion);
    sda.all=0;
    sda.powered=0;sda.data.powered=0;sda.force=1;
    a380x_sd_draw(&sda.data,sda.page,&sda.list);
    if(sda.device && mfda_av.loaded) {
        mfda_av.set_popup_visible(sda.device,0);mfda_av.needs_drawing(sda.device);
    }
}

static void sda_plane_loaded(void)
{
    ecam_button_reset(&sda.eng_motion);
    sda.page=SD_ENGINE;sda.force=1;sda.manual=1;
    sda.all=sda.checklist=sda.abnormal=0;sda.all_elapsed=0;
    memset(sda.cleared,0,sizeof(sda.cleared));
    memset(sda.cancelled,0,sizeof(sda.cancelled));
}
