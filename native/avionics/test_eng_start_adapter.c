/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { N2=1, RUN, MIX, IGN, CLOCK, PAUSE, REPLAY, APU_N1, DUCT };
typedef void *XPLMCommandRef;
typedef void *XPLMDataRef;
static int plugin_enabled = 1;
static struct { struct { int position; } position[1]; } cfb;
static struct { int apu_avail, apu_bleed, pb[4]; } ovhd_in;
static float n2[4], mix[4], wall, sim, apu_n1, duct;
static int run[4], ign[8], holding[4], starts[4], paused, replay, short_n2;
static int cfb_index(const char *name) { return strcmp(name, "eng_start/selector") ? -1 : 0; }
static int a380x_ovhd_find(const char *name) { return name[9]-'1'; }
static XPLMCommandRef XPLMFindCommand(const char *name) { return (void *)(intptr_t)(name[strlen(name)-1]-'0'); }
static XPLMDataRef XPLMFindDataRef(const char *name)
{
    static const char *const refs[] = { "", "N2_percent", "ENGN_running", "mixture_ratio",
        "igniter_on", "total_flight_time_sec", "paused", "is_in_replay", "APU_N1_percent", "bleed_available_center" };
    for (unsigned i=1; i<sizeof(refs)/sizeof(refs[0]); ++i)
        if (strstr(name, refs[i])) return (void *)(intptr_t)i;
    return NULL;
}
static void XPLMCommandBegin(XPLMCommandRef ref) { ++holding[(intptr_t)ref-1]; ++starts[(intptr_t)ref-1]; }
static void XPLMCommandEnd(XPLMCommandRef ref) { --holding[(intptr_t)ref-1]; }
static void XPLMDebugString(const char *s) { (void)s; }
float XPLMGetElapsedTime(void) { return wall; }
float XPLMGetDataf(XPLMDataRef ref)
{
    switch ((intptr_t)ref) { case CLOCK: return sim; case APU_N1: return apu_n1; case DUCT: return duct; default: return NAN; }
}
int XPLMGetDatai(XPLMDataRef ref) { return (intptr_t)ref == PAUSE ? paused : replay; }
static int XPLMGetDatavf(XPLMDataRef ref, float *dst, int offset, int count)
{
    if ((intptr_t)ref == N2 && short_n2) return 0;
    memcpy(dst, ((intptr_t)ref == N2 ? n2 : mix)+offset, (size_t)count*sizeof(*dst)); return count;
}
static int XPLMGetDatavi(XPLMDataRef ref, int *dst, int offset, int count)
{
    memcpy(dst, ((intptr_t)ref == RUN ? run : ign)+offset, (size_t)count*sizeof(*dst)); return count;
}
static void XPLMSetDatavi(XPLMDataRef ref, int *src, int offset, int count)
{
    if ((intptr_t)ref == IGN) memcpy(ign+offset, src, (size_t)count*sizeof(*src));
}
#ifndef ENG_START_SOURCE
#define ENG_START_SOURCE "a380x_eng_start.inc"
#endif
#include ENG_START_SOURCE

static int errors, checks;
#define CHECK(expr) do { ++checks; if (!(expr)) { ++errors; fprintf(stderr, "line %d: %s\n", __LINE__, #expr); } } while (0)
static void advance(float dt, float real) { sim += dt; wall += real; engs_update(); }
static void reset(void)
{
    engs_shutdown();
    memset(n2,0,sizeof(n2)); memset(run,0,sizeof(run)); memset(mix,0,sizeof(mix));
    memset(ign,0,sizeof(ign)); memset(holding,0,sizeof(holding)); memset(starts,0,sizeof(starts));
    memset(&ovhd_in,0,sizeof(ovhd_in));
    apu_n1=100; duct=.9f; sim=wall=0; paused=replay=short_n2=0; cfb.position[0].position=ENGS_NORM;
    CHECK(engs_init());
}
static void prepare_auto(void)
{
    reset();
    ovhd_in.apu_avail=ovhd_in.apu_bleed=1;
    advance(0,0);
    cfb.position[0].position=ENGS_IGN_START;
    mix[0]=1;
    advance(.1f, .1f);
}
int main(void)
{
    prepare_auto();
    paused=1;
    advance(0, 50); CHECK(!holding[0]);
    paused=0;
    advance(.4f, 10); CHECK(!holding[0]);
    for (int i=0; i<4; ++i) advance(.4f, .8f);
    CHECK(holding[0]==1 && ign[0] && starts[0]==1);
    ign[3]=1;  /* another owner */
    for (int i=0; i<4; ++i) advance(.25f, 40);
    CHECK(holding[0]==1 && !engs_start_fault(0));
    run[0]=1; n2[0]=60; advance(.25f, .25f);
    n2[0]=58; advance(.25f, .25f);
    CHECK(holding[0]==1);
    for (int i=0; i<5; ++i) advance(.25f, .25f);
    CHECK(holding[0]==0 && ign[0]); /* continuous ignition in IGN START */
    engs_release(); CHECK(!ign[0] && ign[3]);

    prepare_auto();
    mix[1]=1;
    apu_n1=65; duct=.38f;
    for (int i=0; i<6; ++i) advance(.4f,.4f);
    CHECK(!holding[0] && !holding[1]); /* APU annunciation alone is not ready */
    apu_n1=100; duct=0;
    advance(.2f,.2f); CHECK(!holding[0] && !holding[1]);
    duct=1.1f;
    advance(.2f,.2f); CHECK(holding[0]==1 && holding[1]==0 && !ign[1]);
    duct=0;
    advance(.2f,.2f); CHECK(holding[0]==1 && holding[1]==0 && !ign[1]); /* under-load pressure pulse */
    apu_n1=0;
    advance(.2f,.2f); CHECK(!holding[0] && !holding[1] && !ign[0] && !ign[1]);

    prepare_auto();
    mix[1]=1;
    for (int i=0; i<5; ++i) advance(.4f,.4f);
    CHECK(holding[0]==1 && !holding[1] && !ign[1]);
    run[0]=1; n2[0]=60; advance(.25f,.25f);
    CHECK(holding[0]==1 && !holding[1]); /* still accelerating */
    for (int i=0; i<5; ++i) advance(.25f,.25f);
    CHECK(!holding[0] && holding[1]==1 && ign[1] && !engs_start_fault(1));
    cfb.position[0].position=ENGS_NORM; advance(.1f,.1f);
    CHECK(!holding[0] && !holding[1] && !ign[0] && !ign[1]);

    prepare_auto();
    for (int i=0; i<5; ++i) advance(.4f, .4f);
    CHECK(holding[0]==1);
    replay=1; advance(0, .1f); CHECK(!holding[0] && !ign[0]);
    replay=0; run[0]=1; n2[0]=63; advance(.1f,.1f);
    CHECK(!holding[0] && ign[0]); /* observed already-running engine */
    short_n2=1; advance(.1f,.1f); CHECK(!ign[0]);

    reset();
    advance(0,0);
    cfb.position[0].position=ENGS_IGN_START; mix[0]=1;
    run[0]=1; n2[0]=54;
    advance(.1f,.1f);
    for (int i=0; i<5; ++i) { n2[0]+=.4f; advance(.4f,.4f); }
    CHECK(!holding[0]); /* the recipient is not its own air source */
    run[1]=1; n2[1]=69; advance(.1f,.1f); CHECK(holding[0]==1);

    reset();
    ovhd_in.apu_avail=ovhd_in.apu_bleed=1;
    advance(0,0);
    cfb.position[0].position=ENGS_CRANK; ovhd_in.pb[2]=1;
    advance(.1f,.1f); CHECK(holding[2]==1 && !ign[2]);
    for (int i=0; i<243; ++i) advance(.5f,.5f);
    CHECK(!holding[2] && engs_start_fault(2));
    cfb.position[0].position=ENGS_IGN_START; mix[2]=0;
    advance(.1f,.1f); CHECK(holding[2]==1 && !ign[2] && !engs_start_fault(2));
    mix[2]=1; advance(.1f,.1f); CHECK(holding[2]==1 && ign[2]);
    cfb.position[0].position=ENGS_NORM; advance(.1f,.1f);
    CHECK(!holding[2] && !ign[2]);
    engs_shutdown();
    printf("eng_start_adapter: %d/%d passed\n", checks-errors, checks);
    return errors ? EXIT_FAILURE : EXIT_SUCCESS;
}
