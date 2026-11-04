/* SPDX-License-Identifier: GPL-3.0-or-later
 * Reuse the mock SDK/GL fixture, not browser events or a running simulator.
 */
#define main mfd_fixture_main
#include "test_mfd_adapter.c"
#undef main
#include "a380x_ewd_data.h"
static int xpkit_draw_sd(void) {return 0;}
/* The native FWS is absent in this test: the SD keeps its own ECP handling. */
static int fwsa_available(void) { return 0; }
#include "a380x_sd_adapter.inc"

static float raw[32];
static int raw_count=4;
static float raw_scalar(void *r) { (void)r;return raw[0]; }
static int raw_int(void *r) { (void)r;return (int)raw[0]; }
static int raw_array(void *r,float *out,int at,int count)
{
    (void)r;
    if(!out) return raw_count;
    if(at<0 || at>=raw_count) return 0;
    if(count>raw_count-at)count=raw_count-at;
    memcpy(out,raw+at,(size_t)count*sizeof(float));return count;
}
static void raw_write(void *r,float *in,int at,int count)
{ (void)r;assert(at==18&&count==1);raw[18]=in[0]; }
static void add_raw(const char *name,XPLMDataTypeID type)
{
    XPLMRegisterDataAccessor(name,type,0,raw_int,NULL,raw_scalar,NULL,NULL,NULL,NULL,NULL,
                            raw_array,raw_write,NULL,NULL,NULL,NULL);
}
static MockDevice *sd_device(void)
{
    for(int i=mock_device_count-1;i>=0;--i)
        if(mock_devices[i].alive&&!strcmp(mock_devices[i].id,SDA_DEVICE))return &mock_devices[i];
    return NULL;
}

static void test_eng_button(const EwdData *engines)
{
    MockRef *press=XPLMFindDataRef(SDA_ENG_PREFIX "press_ratio");
    MockRef *selected=XPLMFindDataRef(SDA_ENG_PREFIX "selected");
    CHECK(press&&selected&&!press->writable&&!selected->writable);
    CHECK(press->types==xplmType_Float&&selected->types==xplmType_Float);
    sda_plane_loaded();sda_update(.01f,1,engines);
    CHECK(XPLMGetDataf(press)==0&&XPLMGetDataf(selected)==1);
    int before=sda.action_count;
    run_command(SDA_PREFIX "engine",xplm_CommandBegin);
    run_command(SDA_PREFIX "engine",xplm_CommandEnd); /* release immediately */
    CHECK(sda.action_count==before+1&&sda.page==SD_ENGINE);
    sda_update(.05f,1,engines);
    CHECK(fabsf(XPLMGetDataf(press)-.5f)<.0001f);
    CHECK(XPLMGetDataf(selected)==1);
    sda_update(.05f,1,engines);CHECK(fabsf(XPLMGetDataf(press)-1)<.0001f);
    sda_update(.075f,1,engines);CHECK(fabsf(XPLMGetDataf(press)-.5f)<.0001f);
    sda_update(.08f,1,engines);CHECK(XPLMGetDataf(press)==0&&XPLMGetDataf(selected)==1);
    /* No command End at all: motion still returns and Continue never acts. */
    before=sda.action_count;run_command(SDA_PREFIX "engine",xplm_CommandBegin);
    for(int i=0;i<30;++i){run_command(SDA_PREFIX "engine",xplm_CommandContinue);sda_update(.01f,1,engines);}
    CHECK(sda.action_count==before+1&&XPLMGetDataf(press)==0);
    run_command(SDA_PREFIX "hyd",xplm_CommandBegin);
    CHECK(sda.page==SD_HYD&&XPLMGetDataf(selected)==0);
    run_command(SDA_PREFIX "engine",xplm_CommandBegin);sda_update(.03f,1,engines);
    CHECK(XPLMGetDataf(press)>0&&XPLMGetDataf(selected)==1);
    float position=XPLMGetDataf(press);
    run_command(SDA_PREFIX "engine",xplm_CommandBegin);
    CHECK(XPLMGetDataf(press)==position);
    sda_update(.30f,1,engines);CHECK(XPLMGetDataf(press)==0);
    run_command(SDA_PREFIX "checklist",xplm_CommandBegin);CHECK(XPLMGetDataf(selected)==0);
    run_command(SDA_PREFIX "checklist",xplm_CommandBegin);CHECK(XPLMGetDataf(selected)==1);
    run_command(SDA_PREFIX "abnormal_procedures",xplm_CommandBegin);CHECK(XPLMGetDataf(selected)==0);
    run_command(SDA_PREFIX "engine",xplm_CommandBegin);sda_update(.05f,1,engines);
    sda_disable();CHECK(XPLMGetDataf(press)==0&&XPLMGetDataf(selected)==0);
    sda_update(.01f,1,engines);run_command(SDA_PREFIX "engine",xplm_CommandBegin);
    sda_update(.05f,1,engines);sda_plane_loaded();CHECK(XPLMGetDataf(press)==0);
    sda_update(.01f,0,engines);CHECK(XPLMGetDataf(selected)==0);
    sda_update(.01f,1,engines);CHECK(XPLMGetDataf(selected)==1);
    /* The same elapsed time yields the same position at multiple FPS. */
    const double steps[]={1.0/15,1.0/20,1.0/30,1.0/60,1.0/144};
    for(int rate=0;rate<5;++rate) {
        EcamButtonMotion m={0};ecam_button_trigger(&m);double elapsed=0;
        while(elapsed+steps[rate]<.175){ecam_button_update(&m,&sda_eng_config,steps[rate]);elapsed+=steps[rate];CHECK(m.ratio>=0&&m.ratio<=1);}
        ecam_button_update(&m,&sda_eng_config,.175-elapsed);
        CHECK(fabsf(m.ratio-.5f)<.0001f);
        ecam_button_update(&m,&sda_eng_config,1);CHECK(m.ratio==0&&!m.phase);
    }
    EcamButtonMotion m={0};ecam_button_trigger(&m);
    ecam_button_update(&m,&sda_eng_config,NAN);CHECK(m.ratio==0&&!m.phase);
    ecam_button_trigger(&m);ecam_button_update(&m,&sda_eng_config,-1);CHECK(m.ratio==0&&m.phase==1);
    ecam_button_update(&m,&sda_eng_config,.3);CHECK(m.ratio==0&&!m.phase);
    sda_plane_loaded();
}

int main(int argc,char **argv)
{
    EwdData engines={0};
    CHECK(mfda_av_load());
    CHECK(sda_init());
    if(argc>1&&!strcmp(argv[1],"--contracts")) {
        for(int i=0;i<mock_command_count;++i)if(mock_commands[i].active)printf("C %s\n",mock_commands[i].name);
        for(int i=0;i<mock_ref_count;++i)if(mock_refs[i].active)printf("D %s\n",mock_refs[i].name);
        printf("V %s\n",SDA_DEVICE);
        for(int i=0;i<SDA_INPUTS;++i)if(!strncmp(sda_inputs[i].name,"sim/",4))printf("R %s\n",sda_inputs[i].name);
        sda_shutdown();return 0;
    }
    CHECK(sda.initialized&&sd_device()&&sd_device()->p.drawOnDemand);
    CHECK(sd_device()->p.screenWidth==768&&sd_device()->p.screenHeight==1024);
    CHECK(sd_device()->p.bezelDrawCallback!=NULL); /* opaque black bezel: never a see-through screen */
    CHECK(!sd_device()->p.screenTouchCallback); /* SD is selected by ECP, not the MFD mouse path. */
    sda_update(.25f,1,&engines);
    CHECK(sda.bound&&sda.powered&&sda.page==SD_ENGINE);
    test_eng_button(&engines);
    sda.select_count=0;sda.force=1;sda_update(.01f,1,&engines);
    CHECK(!sda.data.engine[0].n2.valid); /* Missing data cannot become a live zero. */
    int redraws=sd_device()->redraws;
    for(int i=0;i<100;++i)sda_update(.05f,1,&engines);
    CHECK(sd_device()->redraws==redraws); /* Static pages do not redraw on flight-loop ticks. */
    run_command(SDA_PREFIX "apu",xplm_CommandBegin);
    sda_update(.01f,1,&engines);
    CHECK(sda.page==SD_APU&&sda.select_count==1);
    CHECK(sd_device()->redraws==redraws+1);
    run_command(SDA_PREFIX "next_page",xplm_CommandBegin);
    sda_update(.01f,1,&engines);
    CHECK(sda.page==SD_BLEED);
    for(int i=0;i<SD_PAGES;++i){sda_command(NULL,xplm_CommandBegin,(void*)(intptr_t)i);sda_update(.01f,1,&engines);CHECK(sda.page==i);}
    run_command(SDA_PREFIX "all",xplm_CommandBegin);int all_page=sda.page;
    sda_update(1.01f,1,&engines);CHECK(sda.all&&sda.page==(all_page+1)%SD_PAGES);
    run_command(SDA_PREFIX "all",xplm_CommandEnd);all_page=sda.page;sda_update(2,1,&engines);CHECK(!sda.all&&sda.page==all_page);
    run_command(SDA_PREFIX "checklist",xplm_CommandBegin);CHECK(sda.checklist);
    run_command(SDA_PREFIX "down",xplm_CommandBegin);run_command(SDA_PREFIX "check_left",xplm_CommandBegin);CHECK(sda.cursor==1&&sda.checklist_bits[0]==2);
    run_command(SDA_PREFIX "check_right",xplm_CommandBegin);CHECK(sda.checklist_bits[0]==0);
    run_command(SDA_PREFIX "up",xplm_CommandBegin);run_command(SDA_PREFIX "up",xplm_CommandBegin);CHECK(sda.checklist_group==9&&sda.cursor==3);
    engines.warning_count=1;strcpy(engines.warning[0].text,"ACTUAL NATIVE WARNING");sda_update(.3f,1,&engines);
    run_command(SDA_PREFIX "clear_right",xplm_CommandBegin);EwdData filtered;sda_filter_ewd(&filtered,&engines);CHECK(filtered.warning_count==0);
    run_command(SDA_PREFIX "recall_last",xplm_CommandBegin);sda_filter_ewd(&filtered,&engines);CHECK(filtered.warning_count==1);
    run_command(SDA_PREFIX "emergency_cancel",xplm_CommandBegin);sda_filter_ewd(&filtered,&engines);CHECK(filtered.warning_count==0);
    run_command(SDA_PREFIX "recall",xplm_CommandBegin);sda_filter_ewd(&filtered,&engines);CHECK(filtered.warning_count==1);
    run_command(SDA_PREFIX "clear_left",xplm_CommandBegin);sda_filter_ewd(&filtered,&engines);CHECK(filtered.warning_count==0);
    run_command(SDA_PREFIX "engine",xplm_CommandBegin);sda_update(.01f,1,&engines);CHECK(!sda.checklist&&!sda.abnormal);
    for(int i=0;i<SDA_COMMANDS;++i){int previous=sda.action_count;sda_command(NULL,xplm_CommandBegin,(void*)(intptr_t)i);CHECK(sda.action_count==previous+1);sda_command(NULL,xplm_CommandEnd,(void*)(intptr_t)i);}
    mfda_av.set_popup_visible(sda.device,0);
    for(int i=0;i<20;++i)run_command(SDA_PREFIX "ewd_brightness_down",xplm_CommandBegin);
    CHECK(sda.ewd_brightness==0);
    run_command(SDA_PREFIX "ewd_brightness_up",xplm_CommandBegin);CHECK(fabsf(sda.ewd_brightness-.05f)<.001f);
    run_command(SDA_PREFIX "engine",xplm_CommandBegin);sda.brightness=1;
    add_raw(sda_inputs[SDA_N2].name,xplmType_FloatArray);
    add_raw(sda_inputs[SDA_FF].name,xplmType_FloatArray);
    add_raw(sda_inputs[SDA_OIL_TEMP].name,xplmType_FloatArray);
    add_raw(sda_inputs[SDA_OIL_C].name,xplmType_Int);
    add_raw(sda_inputs[SDA_RHEO].name,xplmType_FloatArray);
    sda_find_inputs();
    raw[0]=0;raw[1]=212;raw[2]=NAN;raw[3]=40;
    sda.force=1;sda_update(.25f,1,&engines);
    CHECK(sda.data.engine[0].n2.valid&&sda.data.engine[0].n2.value==0);
    CHECK(!sda.data.engine[1].n2.valid&&!sda.data.engine[2].n2.valid);
    CHECK(fabsf(sda.data.engine[1].oil_temperature_c.value-100)<.001f); /* Fahrenheit flag honored. */
    CHECK(fabsf(sda.data.engine[3].fuel_flow_kg_h.value-144000)<.01f); /* kg/s -> kg/h */
    raw_count=2;sda.force=1;sda_update(.25f,1,&engines);
    CHECK(!sda.data.engine[3].n2.valid); /* Short arrays do not fabricate missing engines. */
    for(int i=0;i<20;++i)run_command(SDA_PREFIX "brightness_down",xplm_CommandBegin);
    sda_update(.01f,1,&engines);
    CHECK(sda.brightness==0&&!sda.powered&&sda.list.cmd_count==1&&raw[18]==0);
    run_command(SDA_PREFIX "brightness_up",xplm_CommandBegin);sda_update(.01f,1,&engines);
    CHECK(sda.powered&&fabsf(sda.brightness-.05f)<.0001f);
    run_command(SDA_PREFIX "popup",xplm_CommandBegin);CHECK(sd_device()->popup);
    sda_disable();CHECK(!sda.powered&&!sd_device()->popup);
    int disabled_page=sda.page;plugin_enabled=0;run_command(SDA_PREFIX "apu",xplm_CommandBegin);CHECK(sda.page==disabled_page);
    plugin_enabled=1;
    MockRef *ref=XPLMFindDataRef(SDA_PREFIX "active_page");
    CHECK(ref&&!ref->writable&&!ref->seti);
    sda_shutdown();CHECK(!sd_device()&&active_commands()==0&&mock_menus==0);
    CHECK(sda_init());sda_update(.25f,1,&engines);CHECK(sda.page==SD_ENGINE);
    sd_device()->p.drawCallback(NULL);CHECK(sda.draw_count==1);
    sda_shutdown();
    if(g_fail){printf("test_sd_adapter: %d of %d checks FAILED\n",g_fail,g_checks);return 1;}
    printf("test_sd_adapter: %d checks OK\n",g_checks);return 0;
}
