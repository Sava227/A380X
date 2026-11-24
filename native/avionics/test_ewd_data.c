/* SPDX-License-Identifier: GPL-3.0-or-later
 * Execute the production EWD input adapter against a typed in-memory XPLM.
 * Link with a380x_ewd_fadec.cpp so the native fallback is exercised too.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "XPLMDataAccess.h"

static struct { int flight_phase; } published;
#include "a380x_guidance.h"
static A380XGuidanceInputs guidance_inputs;
#include "a380x_ewd_data.inc"

typedef struct {
    int present, type, count;
    double scalar;
    float vf[4];
    int vi[4];
} EwdMockRef;
static EwdMockRef mock[ER_REF_COUNT];

XPLMDataRef XPLMFindDataRef(const char *name)
{
    int i;
    for (i=0;i<ER_REF_COUNT;++i)
        if (!strcmp(name,ewd_inputs[i].name)) return mock[i].present ? &mock[i] : NULL;
    return NULL;
}
XPLMDataTypeID XPLMGetDataRefTypes(XPLMDataRef ref)
{ assert(ref);return ((EwdMockRef *)ref)->type; }
float XPLMGetDataf(XPLMDataRef ref)
{ assert(ref && ((EwdMockRef *)ref)->type==xplmType_Float);return (float)((EwdMockRef *)ref)->scalar; }
double XPLMGetDatad(XPLMDataRef ref)
{ assert(ref && ((EwdMockRef *)ref)->type==xplmType_Double);return ((EwdMockRef *)ref)->scalar; }
int XPLMGetDatai(XPLMDataRef ref)
{ assert(ref && ((EwdMockRef *)ref)->type==xplmType_Int);return (int)((EwdMockRef *)ref)->scalar; }
int XPLMGetDatavf(XPLMDataRef ref,float *out,int offset,int count)
{
    EwdMockRef *r=(EwdMockRef *)ref;
    assert(r && r->type==xplmType_FloatArray);
    if (offset>=r->count) return 0;
    if (count>r->count-offset) count=r->count-offset;
    memcpy(out,r->vf+offset,(size_t)count*sizeof(*out));return count;
}
int XPLMGetDatavi(XPLMDataRef ref,int *out,int offset,int count)
{
    EwdMockRef *r=(EwdMockRef *)ref;
    assert(r && r->type==xplmType_IntArray);
    if (offset>=r->count) return 0;
    if (count>r->count-offset) count=r->count-offset;
    memcpy(out,r->vi+offset,(size_t)count*sizeof(*out));return count;
}

static void setup(void)
{
    int i,j;
    memset(mock,0,sizeof(mock));
    for (i=0;i<ER_REF_COUNT;++i) {
        mock[i].present=strncmp(ewd_inputs[i].name,"a380x/",6)!=0 || i==ER_APU_N1;   /* FBW APU N (ELEC) */
        mock[i].type=ewd_inputs[i].type;
        mock[i].count=4;
    }
    for (j=0;j<4;++j) {
        mock[ER_FADEC].vi[j]=1;
        mock[ER_BURNING].vi[j]=1;
        mock[ER_REV_MODE].vi[j]=1;
        mock[ER_N1].vf[j]=50.0f;
        mock[ER_EGT].vf[j]=420.0f;
    }
    mock[ER_N1_LIMIT].scalar=111.0;
    mock[ER_EGT_LIMIT].scalar=900.0;
    mock[ER_SAT].scalar=15.0;
    mock[ER_PRESSURE].scalar=101325.0;
    mock[ER_PRESSURE_ALT].scalar=0.0;
    mock[ER_MACH].scalar=0.0;
    mock[ER_ON_GROUND].scalar=1.0;
    published.flight_phase=1;
    ewd_init();
}

static int has_line(EwdLine *lines,int count,const char *text,int color)
{
    int i;
    for(i=0;i<count;++i)
        if (!strcmp(lines[i].text,text) && lines[i].color==color) return 1;
    return 0;
}

/* FBW A380FadecComputer thrust_limit_type selection (engine 1 FADEC). */
static void test_thrust_limit_type_selection(void)
{
    int latch=0;
    /* Lever ratio to TLA: calibrated detents snap, ranges interpolate, reverse is negative. */
    assert(ewd_lever_tla(.10,.10,.40,.70,1.0,.02,0)==0.0);
    assert(ewd_lever_tla(.41,.10,.40,.70,1.0,.02,0)==25.0);
    assert(ewd_lever_tla(.69,.10,.40,.70,1.0,.02,0)==35.0);
    assert(ewd_lever_tla(.99,.10,.40,.70,1.0,.02,0)==45.0);
    assert(fabs(ewd_lever_tla(.55,.10,.40,.70,1.0,.02,0)-30.0)<1e-9);
    assert(ewd_lever_tla(.10,.10,.40,.70,1.0,.02,1)<0.0);
    /* On ground with the engine running: TOGA unless FLEX; reverse = MREV. */
    assert(ewd_thrust_limit_type(0,1,1,0,0,&latch)==4);
    assert(ewd_thrust_limit_type(0,1,1,1,0,&latch)==3 && !latch);
    assert(ewd_thrust_limit_type(45,1,1,1,0,&latch)==4);
    assert(ewd_thrust_limit_type(-20,1,1,0,0,&latch)==5);
    /* FLX/MCT detent with FLEX on the ground latches; airborne it stays FLX until CL or TOGA. */
    assert(ewd_thrust_limit_type(35,1,1,1,0,&latch)==3 && latch);
    assert(ewd_thrust_limit_type(35,0,1,1,0,&latch)==3 && latch);
    assert(ewd_thrust_limit_type(25,0,1,1,0,&latch)==1 && !latch);
    assert(ewd_thrust_limit_type(35,0,1,1,0,&latch)==2);
    assert(ewd_thrust_limit_type(45,0,1,0,0,&latch)==4);
    /* Airborne the reverse range is clamped to CLB; A/THR TOGA request forces TOGA. */
    assert(ewd_thrust_limit_type(-20,0,1,0,0,&latch)==1);
    assert(ewd_thrust_limit_type(10,0,1,0,1,&latch)==4);
    /* Engine not operative on the ground uses the in-flight table. */
    assert(ewd_thrust_limit_type(25,1,0,0,0,&latch)==1);
}

int main(void)
{
    test_thrust_limit_type_selection();
    A380xEwdFadecLimits limits;
    float expected;
    int i;

    setup();
    ewd_update(0.05f,1);
    assert(a380x_ewd_fadec_limits(15,1013.25,0,0,0,0,0,&limits));
    expected=ewd_thrust_from_n1(50,(float)limits.idle_n1,(float)limits.max_n1,1);
    for(i=0;i<4;++i) {
        assert(ewd.engine[i].active && ewd.engine[i].n1_valid && ewd.engine[i].egt_valid);
        assert(ewd.engine[i].thrust_valid && fabsf(ewd.engine[i].thrust-expected)<0.001f);
        assert(!ewd.engine[i].commanded_valid && !ewd.engine[i].throttle_valid);
        assert(ewd.engine[i].running && ewd.engine[i].running_valid);
        assert(ewd.engine[i].available_valid);
        assert(fabsf(ewd.engine[i].available_thrust-
            ewd_thrust_from_n1(76.5f,(float)limits.idle_n1,(float)limits.max_n1,1))<0.001f);
    }
    assert(!ewd.thrust_limit_valid && !ewd.flex_valid && !ewd.limit_count);

    /* The available-thrust pie uses the source METOTS cap below 35 CAS. */
    mock[ER_CAS].scalar=35.0;ewd_update(0,1);
    assert(ewd.engine[0].available_valid && ewd.engine[0].available_thrust==100.0f);
    mock[ER_CAS].scalar=NAN;ewd_update(0,1);
    assert(!ewd.engine[0].available_valid);

    /* Source aggregate activation: with all engines stopped, FADEC power
     * alone does not replace the XX state with live zero indications. */
    setup();
    for(i=0;i<4;++i) {mock[ER_BURNING].vi[i]=0;mock[ER_N1].vf[i]=0;}
    ewd_update(0,1);
    for(i=0;i<4;++i) assert(ewd.engine[i].active_valid && !ewd.engine[i].active);
    mock[ER_ENG_SELECTOR].scalar=1;ewd_update(0,1);
    for(i=0;i<4;++i) assert(ewd.engine[i].active);
    mock[ER_ENG_SELECTOR].scalar=0;mock[ER_STARTER].vi[2]=1;
    ewd_update(0,1);
    for(i=0;i<4;++i) assert(ewd.engine[i].active);
    assert(ewd.engine[2].starting && !ewd.engine[2].avail && !ewd.engine[2].running);
    mock[ER_N1].vf[2]=(float)limits.idle_n1+1.0f;ewd_update(0,1);
    assert(ewd.engine[2].avail && ewd.engine[2].avail_valid);
    mock[ER_FIRE].vi[2]=1;ewd_update(0,1);
    assert(ewd.engine[2].active && ewd.engine[2].fire);
    mock[ER_FIRE_PB].present=1;mock[ER_FIRE_PB].vi[2]=1;ewd_update(1.1f,1);
    assert(!ewd.engine[2].active && ewd.engine[1].active);
    mock[ER_FADEC].vi[1]=0;ewd_update(0,1);
    assert(!ewd.engine[1].active);

    /* Missing environmental data cannot become a plausible nominal THR. */
    mock[ER_PRESSURE].present=0;
    ewd_init();ewd_update(0,1);
    assert(!ewd.engine[0].thrust_valid && ewd.engine[0].n1_degraded);
    assert(ewd.engine[0].n1_valid && ewd.engine[0].egt_valid);

    /* Independent channel failures, short arrays, NaN and type mismatch. */
    setup();
    mock[ER_N1].vf[0]=NAN;
    mock[ER_EGT].count=2;
    mock[ER_N1_FAIL_2].scalar=6;
    mock[ER_FADEC].type=xplmType_Float;
    ewd_init();ewd_update(0,1);
    assert(!ewd.engine[0].n1_valid && !ewd.engine[1].n1_valid);
    assert(ewd.engine[2].n1_valid && !ewd.engine[2].egt_valid);
    assert(!ewd.engine[0].active_valid);

    /* Runtime registration is picked up; external limits override the
     * numerical fallback, and an invalid producer is not silently ignored. */
    setup();
    mock[ER_IDLE_N1].present=mock[ER_MAX_N1].present=1;
    mock[ER_IDLE_N1].scalar=20;mock[ER_MAX_N1].scalar=100;
    ewd_update(1.1f,1);
    assert(fabsf(ewd.engine[0].thrust-40.125f)<0.001f);
    mock[ER_MAX_N1].scalar=NAN;
    ewd_update(0,1);
    assert(!ewd.engine[0].thrust_valid);
    assert(ewd.engine[0].idle_n1_valid && ewd.engine[0].idle_n1==20.0f);

    /* Demand data survive degraded THR. Transient visibility requires an
     * independently validated A/THR engagement state. */
    setup();
    mock[ER_COMMANDED_N1].present=mock[ER_THROTTLE_N1].present=1;
    for(i=0;i<4;++i) {mock[ER_COMMANDED_N1].vf[i]=80;mock[ER_THROTTLE_N1].vf[i]=70;}
    ewd_update(1.1f,1);
    assert(ewd.engine[0].commanded_n1_valid && ewd.engine[0].commanded_n1==80);
    assert(ewd.engine[0].throttle_n1_valid && ewd.engine[0].throttle_n1==70);
    assert(!ewd.athr_valid && !ewd.engine[0].commanded_valid && ewd.engine[0].throttle_valid);
    mock[ER_ATHR_STATUS].present=1;mock[ER_ATHR_STATUS].scalar=2;
    ewd_update(1.1f,1);
    assert(ewd.athr_valid && ewd.athr_engaged && ewd.engine[0].commanded_valid);
    mock[ER_ATHR_STATUS].scalar=0;ewd_update(0,1);
    assert(ewd.athr_valid && !ewd.athr_engaged && !ewd.engine[0].commanded_valid);
    assert(ewd.engine[0].commanded_n1_valid);
    mock[ER_ATHR_STATUS].scalar=3;ewd_update(0,1);
    assert(!ewd.athr_valid && !ewd.engine[0].commanded_valid);
    mock[ER_IDLE_N1].present=mock[ER_MAX_N1].present=1;
    mock[ER_IDLE_N1].scalar=20;mock[ER_MAX_N1].scalar=NAN;
    ewd_update(1.1f,1);
    assert(!ewd.engine[0].thrust_valid && !ewd.engine[0].throttle_valid);
    assert(ewd.engine[0].idle_n1_valid && ewd.engine[0].commanded_n1_valid && ewd.engine[0].throttle_n1_valid);

    /* Reverse donut spans positive N1 magnitude from idle to reverse
     * limit. A missing forward limit does not invalidate reverse sources. */
    mock[ER_REV_N1].present=1;mock[ER_REV_N1].scalar=80;
    mock[ER_REV_MODE].vi[1]=3;mock[ER_REV_DEPLOY].vf[1]=1;
    mock[ER_THROTTLE_N1].vf[1]=80;ewd_update(1.1f,1);
    assert(ewd.engine[1].throttle_valid && ewd.engine[1].throttle==100.0f);
    assert(ewd.engine[1].thrust_valid && fabsf(ewd.engine[1].thrust-50.0f)<0.001f);
    mock[ER_THROTTLE_N1].vf[1]=20;ewd_update(0,1);
    assert(ewd.engine[1].throttle_valid && ewd.engine[1].throttle==0.0f);
    mock[ER_THROTTLE_N1].vf[1]=-20;ewd_update(0,1);
    assert(!ewd.engine[1].throttle_n1_valid && !ewd.engine[1].throttle_valid);

    /* Reverse deploy motion only produces REV on the two inner engines. */
    setup();
    for(i=0;i<4;++i) {mock[ER_REV_MODE].vi[i]=3;mock[ER_REV_DEPLOY].vf[i]=0.4f;}
    ewd_update(0,1);
    assert(!ewd.engine[0].reverse && ewd.engine[1].reverse && ewd.engine[2].reverse && !ewd.engine[3].reverse);
    assert(!ewd.engine[1].reverse_deployed);
    mock[ER_REV_DEPLOY].vf[1]=1.0f;
    ewd_update(0,1);assert(ewd.engine[1].reverse_deployed);

    /* Exact memo wording, APU condition, phase inhibition and caution color. */
    setup();
    mock[ER_APU_RUNNING].scalar=1;mock[ER_APU_N1].scalar=100;
    mock[ER_APU_BLEED].scalar=1;mock[ER_SEAT_BELTS].scalar=1;
    mock[ER_PARK_BRAKE].scalar=1;mock[ER_EXT_PWR].scalar=1;
    ewd_update(0,1);
    assert(has_line(ewd.memo,ewd.memo_count,"APU BLEED",EWD_COLOR_GREEN));
    assert(has_line(ewd.memo,ewd.memo_count,"SEAT BELTS",EWD_COLOR_GREEN));
    assert(has_line(ewd.memo,ewd.memo_count,"PARK BRK ON",EWD_COLOR_GREEN));
    assert(has_line(ewd.memo,ewd.memo_count,"ELEC EXT PWR",EWD_COLOR_AMBER));
    published.flight_phase=6;ewd_update(0,1);
    assert(!has_line(ewd.memo,ewd.memo_count,"PARK BRK ON",EWD_COLOR_GREEN));
    assert(!has_line(ewd.memo,ewd.memo_count,"ELEC EXT PWR",EWD_COLOR_AMBER));
    assert(has_line(ewd.memo,ewd.memo_count,"SEAT BELTS",EWD_COLOR_GREEN));
    mock[ER_APU_N1].scalar=50;ewd_update(0,1);
    assert(!has_line(ewd.memo,ewd.memo_count,"APU BLEED",EWD_COLOR_GREEN));

    /* Red alerts precede cautions; absent/invalid measurements cannot trip. */
    setup();mock[ER_FIRE].vi[3]=1;mock[ER_EGT].vf[0]=950;
    mock[ER_EGT].vf[1]=NAN;mock[ER_N1_HIGH].vi[2]=1;
    mock[ER_GEAR_NOT_LOCKED_DOWN].present=1;mock[ER_GEAR_NOT_LOCKED_DOWN].scalar=1;
    ewd_update(1.1f,1);
    assert(ewd.warning_count==4);
    assert(!strcmp(ewd.warning[0].text,"ENG 4 FIRE") && ewd.warning[0].color==EWD_COLOR_RED);
    assert(has_line(ewd.warning,ewd.warning_count,"L/G GEAR NOT LOCKED DOWN",EWD_COLOR_RED));
    assert(has_line(ewd.warning,ewd.warning_count,"ENG 1 EGT OVER LIMIT",EWD_COLOR_AMBER));
    assert(has_line(ewd.warning,ewd.warning_count,"ENG 3 N1/N2 OVER LIMIT",EWD_COLOR_AMBER));

    /* Loss of power clears all visible state on the next update. */
    ewd_update(0,0);
    assert(!ewd.powered && !ewd.warning_count && !ewd.memo_count);
    assert(!ewd.engine[0].n1_valid && !ewd.engine[3].fire);
    puts("EWD data adapter: live inputs, FADEC conversion, missing-data validity, reverse, memos and warnings passed");
    return 0;
}
