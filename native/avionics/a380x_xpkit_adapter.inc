/* SPDX-License-Identifier: GPL-3.0-or-later
 * Actual xp-kit PFDs, composited into the existing A380 cockpit atlas.
 * Only the native aircraft controller is authoritative. No HTTP, stock AP
 * mode mirroring, synthetic flight state, or WebKit code in X-Plane's process.
 */
#if (APL || IBM) && defined(A380X_XPKIT)
#include "a380x_xpkit_shared.h"
#include <stdarg.h>
#if APL
#include <spawn.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
extern char **environ;
static A380XXpKitShared *xpkit_shared;
static pid_t xpkit_pid;
static char xpkit_shared_path[128];
#else
static A380XXpKitShared *xpkit_shared;
static HANDLE xpkit_pid, xpkit_mapping, xpkit_job;
static wchar_t xpkit_shared_name[128];
#endif
static GLuint xpkit_texture[XPKIT_VIEWS];
static unsigned xpkit_uploaded[XPKIT_VIEWS];
static unsigned char xpkit_staging[XPKIT_MAX_VIEW_BYTES];
static int xpkit_health[26]; /* Existing PFD/ND contracts retain their indices; 22..25 = EFB tablets. */
static XPLMDataRef xpkit_health_refs[26];
static const char *xpkit_health_names[]={"helper_running","captain_active","first_officer_active","errors",
    "captain_presented_frames","first_officer_presented_frames","captain_nd_active","first_officer_nd_active",
    "captain_nd_presented_frames","first_officer_nd_presented_frames","nd_devices_active","captain_nd_bound","first_officer_nd_bound",
    "captain_mfd_active","first_officer_mfd_active","captain_mfd_presented_frames","first_officer_mfd_presented_frames","sd_active","sd_presented_frames",
    "pfd_devices_active","captain_pfd_bound","first_officer_pfd_bound",
    "captain_efb_active","first_officer_efb_active","captain_efb_presented_frames","first_officer_efb_presented_frames"};
static int xpkit_draw_view(int view,float x,float y,float width,float height);
static double xpkit_now(void);
static void draw_nd(const Side *side,float x,float y,const char *name);
static void draw_pfd(const Side *side,float x,float y,const char *name);
#include "a380x_xpkit_nd_input.inc"
#include "a380x_xpkit_pfd.inc"
#include "a380x_xpkit_efb.inc"
static XPLMDataRef xpkit_clock;
static double xpkit_last_update;
static int xpkit_logged;
static double xpkit_now(void) {
#if IBM
    LARGE_INTEGER counter, frequency;
    QueryPerformanceCounter(&counter); QueryPerformanceFrequency(&frequency);
    return (double)counter.QuadPart/(double)frequency.QuadPart;
#else
    struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec+t.tv_nsec*1e-9;
#endif
}
#if IBM
#include "a380x_xpkit_windows.inc"
#endif
static int xpkit_get_health(void *refcon) { return *(int *)refcon; }
#if APL
/* SkunkCrafts replaces files through a regular temporary file, so the
 * executable bit of the out-of-process renderer is not preserved.  Restore
 * execute permission only on the expected, non-symlink regular file. */
static int xpkit_prepare_helper(const char *path) {
    struct stat st;
    int fd=open(path,O_RDONLY|O_NOFOLLOW|O_CLOEXEC);
    if(fd<0) return 0;
    if(fstat(fd,&st) || !S_ISREG(st.st_mode) || st.st_size<=0) { close(fd); return 0; }
    if(!(st.st_mode&S_IXUSR)) {
        mode_t mode=(st.st_mode&0777)|((st.st_mode&0444)>>2);
        if(fchmod(fd,mode)) { close(fd); return 0; }
    }
    close(fd);
    return access(path,X_OK)==0;
}
#endif
static void xpkit_init(void) {
    int i; char name[128];
    xpkit_clock=XPLMFindDataRef("sim/time/total_flight_time_sec");
    for(i=0;i<ARRAY_LEN(xpkit_health);++i) {
        snprintf(name,sizeof(name),"a380x/xp12/xpkit/%s",xpkit_health_names[i]);
        xpkit_health_refs[i]=XPLMRegisterDataAccessor(name,xplmType_Int,0,xpkit_get_health,NULL,
            NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,&xpkit_health[i],NULL);
    }
    xpkit_nd_init();
    xpkit_pfd_init();
    xpkit_efb_init();
}
static void xpkit_stop(void) {
#if IBM
    xpkit_windows_stop();
#else
    if(xpkit_pid>0) {
        int status, n; kill(xpkit_pid,SIGTERM);
        for(n=0;n<20;++n) {
            if(waitpid(xpkit_pid,&status,WNOHANG)!=0) break;
            struct timespec pause={0,5000000}; nanosleep(&pause,NULL);
        }
        if(n==20) { kill(xpkit_pid,SIGKILL); waitpid(xpkit_pid,&status,0); }
    }
    xpkit_pid=0;
    if(xpkit_shared) munmap(xpkit_shared,sizeof(*xpkit_shared));
    xpkit_shared=NULL;
    if(xpkit_shared_path[0]) unlink(xpkit_shared_path);
    xpkit_shared_path[0]=0;
#endif
    for(int i=0;i<10;++i) xpkit_health[i]=0;
    for(int i=13;i<19;++i) xpkit_health[i]=0;
    xpkit_efb_reset();
    xpkit_nd_reset();
    memset(xpkit_uploaded,0,sizeof(xpkit_uploaded));
    xpkit_logged=0;
}
static void xpkit_start(void) {
    char plugin[2048], helper[2304], assets[2304], system[2048], *slash;
#if APL
    int fd;
#endif
    xpkit_stop();
    XPLMGetPluginInfo(XPLMGetMyID(),NULL,plugin,NULL,NULL);
    slash=strrchr(plugin,'/');
#if IBM
    { char *backslash=strrchr(plugin,'\\'); if(backslash && (!slash||backslash>slash))slash=backslash; }
#endif
    if(!slash) return;
    *slash=0;
#if IBM
    snprintf(helper,sizeof(helper),"%s/A380XDisplayHost.exe",plugin);
    snprintf(assets,sizeof(assets),"%s/xp-kit",plugin);
    XPLMGetSystemPath(system);
    if(!xpkit_windows_start(helper,assets,system)) {
        XPLMDebugString("A380X xp-kit: Windows helper could not start; inspect aircraft plugin files and display-host.log.\n");
        xpkit_stop();return;
    }
#else
    snprintf(helper,sizeof(helper),"%s/A380XDisplayHost",plugin);
    snprintf(assets,sizeof(assets),"%s/xp-kit",plugin);
    if(!xpkit_prepare_helper(helper)) {
        XPLMDebugString("A380X xp-kit: display helper absent; native PFD fallback.\n"); return;
    }
    strcpy(xpkit_shared_path,"/tmp/a380x-cockpit-XXXXXX");
    fd=mkstemp(xpkit_shared_path);
    if(fd<0) { xpkit_shared_path[0]=0; return; }
    if(ftruncate(fd,sizeof(A380XXpKitShared))) { close(fd); xpkit_stop(); return; }
    xpkit_shared=mmap(NULL,sizeof(*xpkit_shared),PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
    close(fd);
    if(xpkit_shared==MAP_FAILED) { xpkit_shared=NULL; xpkit_stop(); return; }
    memset(xpkit_shared,0,sizeof(*xpkit_shared));
    xpkit_shared->magic=XPKIT_MAGIC; xpkit_shared->version=XPKIT_VERSION;
    XPLMGetSystemPath(system);
    char *argv[]={helper,xpkit_shared_path,assets,system,NULL};
    if(posix_spawn(&xpkit_pid,helper,NULL,NULL,argv,environ)) { xpkit_pid=0; xpkit_stop(); return; }
#endif
    xpkit_health[0]=1; xpkit_last_update=0;
    XPLMDebugString("A380X xp-kit: aircraft-local display renderer started (no web server).\n");
}
static void xpkit_shutdown(void) {
    int i; xpkit_stop();
    xpkit_efb_shutdown();
    xpkit_nd_shutdown();
    xpkit_pfd_shutdown();
    for(i=0;i<ARRAY_LEN(xpkit_health_refs);++i) if(xpkit_health_refs[i]) {
        XPLMUnregisterDataAccessor(xpkit_health_refs[i]); xpkit_health_refs[i]=NULL;
    }
    for(i=0;i<XPKIT_VIEWS;++i) if(xpkit_texture[i]) { glDeleteTextures(1,&xpkit_texture[i]); xpkit_texture[i]=0; }
}
typedef struct { char data[XPKIT_JSON_CAP]; size_t size; int failed; } XpKitJson;
static void xpkit_append(XpKitJson *j,const char *format,...) {
    if(j->failed) return;
    va_list args; va_start(args,format);
    int n=vsnprintf(j->data+j->size,sizeof(j->data)-j->size,format,args); va_end(args);
    if(n<0 || (size_t)n>=sizeof(j->data)-j->size) j->failed=1; else j->size+=(size_t)n;
}
static void xpkit_number(XpKitJson *j,const char *name,double value) {
    if(isfinite(value)) xpkit_append(j,"\"%s\":%.10g,",name,value);
}
static void xpkit_word(XpKitJson *j,const char *prefix,const char *suffix,double value,int valid) {
    xpkit_append(j,"\"%s%s\":[%.10g,%s],",prefix,suffix,isfinite(value)?value:0,
                 valid && isfinite(value)?"true":"false");
}
#define XPKIT_BIT(n,condition) ((condition)?(UINT32_C(1)<<((n)-1)):0)
static void xpkit_guidance_words(XpKitJson *j) {
    const A380XGuidanceState *g=&guidance;
    uint32_t d1=XPKIT_BIT(11,g->ap1)|XPKIT_BIT(12,g->ap2)|XPKIT_BIT(13,g->fd1)|XPKIT_BIT(14,g->fd2);
    uint32_t d2=XPKIT_BIT(11,g->altitude_armed)|XPKIT_BIT(13,g->gs_armed)|XPKIT_BIT(23,g->loc_armed);
    uint32_t d3=0,d4=0,ats=0;
    switch(g->vertical_mode) {
        case A380X_B1_CLB:d3=XPKIT_BIT(11,1);break;
        case A380X_B1_DES:d3=XPKIT_BIT(12,1);break;
        case A380X_B1_OP_CLB:d3=XPKIT_BIT(13,1);break;
        case A380X_B1_OP_DES:d3=XPKIT_BIT(14,1);break;
        case A380X_B1_VS:d3=XPKIT_BIT(17,1);break;
        case A380X_B1_FPA:d3=XPKIT_BIT(18,1);break;
        case A380X_B1_ALT_STAR:d3=XPKIT_BIT(19,1);break;
        case A380X_B1_ALT:d3=XPKIT_BIT(20,1);break;
        case A380X_B1_GS_STAR:d3=XPKIT_BIT(21,1);break;
        case A380X_B1_GS:d3=XPKIT_BIT(22,1);break;
        default:break; /* Unsupported modes are not invented. */
    }
    switch(g->lateral_mode) {
        case A380X_C1_RWY:d4=XPKIT_BIT(11,1);break;
        case A380X_C1_NAV:d4=XPKIT_BIT(12,1);break;
        case A380X_C1_LOC_STAR:d4=XPKIT_BIT(13,1);break;
        case A380X_C1_LOC:d4=XPKIT_BIT(14,1);break;
        case A380X_C1_GA_TRK:d4=XPKIT_BIT(15,1);break;
        case A380X_C1_HDG:d4=XPKIT_BIT(16,1);break;
        case A380X_C1_TRACK:d4=XPKIT_BIT(17,1);break;
        case A380X_C1_RWY_TRK:d4=XPKIT_BIT(19,1);break;
        default:break;
    }
    uint32_t d5=XPKIT_BIT(11,g->trk_fpa_mode)|XPKIT_BIT(12,g->mach_mode)|XPKIT_BIT(13,g->true_heading)|
        XPKIT_BIT(14,g->metric_alt)|XPKIT_BIT(17,g->speed_managed)|XPKIT_BIT(18,!g->speed_managed)|XPKIT_BIT(28,g->mode_reversion);
    switch(g->thrust_mode) {
        case A380X_A1A2_MAN_TOGA:ats=XPKIT_BIT(11,1);break;
        case A380X_A1A2_MAN_MCT:case A380X_A1A2_THR_MCT:ats=XPKIT_BIT(12,1);break;
        case A380X_A1A2_MAN_FLEX:ats=XPKIT_BIT(13,1);break;
        case A380X_A1A2_THR_CLB:ats=XPKIT_BIT(14,1);break;
        case A380X_A1A2_MAN_THR:case A380X_A1A2_THR_LVR:ats=XPKIT_BIT(15,1);break;
        case A380X_A1A2_THR_IDLE:ats=XPKIT_BIT(16,1);break;
        case A380X_A1A2_A_FLOOR:ats=XPKIT_BIT(17,1);break;
        case A380X_A1A2_TOGA_LK:ats=XPKIT_BIT(18,1);break;
        case A380X_A1A2_SPEED:ats=XPKIT_BIT(19,1);break;
        case A380X_A1A2_MACH:ats=XPKIT_BIT(20,1);break;
        default:break;
    }
    for(int unit=1;unit<=3;++unit) {
        char prefix[64]; snprintf(prefix,sizeof(prefix),"L:A32NX_PRIM_%d_",unit);
        /* These are display encodings of ONE native controller, not three simulated PRIMs. */
#define XPW(s,v,ok) xpkit_word(j,prefix,s,v,ok)
        XPW("PFD_SELECTED_SPEED",g->speed_target_knots,g->initialized);
        XPW("SELECTED_AIRSPEED",g->selected_speed_knots,g->initialized);
        XPW("SELECTED_MACH",g->selected_mach,g->initialized);
        XPW("SELECTED_HEADING",g->selected_heading_deg,g->initialized);
        XPW("SELECTED_TRACK",g->selected_heading_deg,g->initialized);
        XPW("SELECTED_ALTITUDE",g->selected_altitude_ft,g->initialized);
        XPW("SELECTED_VERTICAL_SPEED",g->selected_vs_fpm,g->initialized);
        XPW("SELECTED_FPA",g->selected_fpa_deg,g->initialized);
        XPW("FG_DISCRETE_WORD_1",d1,g->initialized);
        XPW("FG_DISCRETE_WORD_2",d2,g->initialized);
        XPW("FG_DISCRETE_WORD_3",d3,g->initialized);
        XPW("FG_DISCRETE_WORD_4",d4,g->initialized);
        XPW("FG_DISCRETE_WORD_5",d5,g->initialized);
        XPW("FG_DISCRETE_WORD_6",0,g->initialized);
        XPW("FG_ATS_DISCRETE_WORD",XPKIT_BIT(11,g->athr_armed)|XPKIT_BIT(12,g->athr_active),g->initialized);
        XPW("FG_ATS_FMA_DISCRETE_WORD",ats,g->initialized);
        XPW("ROLL_FD_COMMAND_1",g->fd_roll_error_deg,g->guidance_valid);
        XPW("ROLL_FD_COMMAND_2",g->fd_roll_error_deg,g->guidance_valid);
        XPW("PITCH_FD_COMMAND_1",-g->fd_pitch_error_deg,g->guidance_valid && g->vertical_guidance_selected);
        XPW("PITCH_FD_COMMAND_2",-g->fd_pitch_error_deg,g->guidance_valid && g->vertical_guidance_selected);
        XPW("V_MAX",pfd_guidance.vmax_knots,pfd_guidance.envelope_valid && pfd_guidance.vmax_knots>0);
        XPW("V_LS",pfd_guidance.vls_knots,pfd_guidance.clean_performance_valid && pfd_guidance.vls_knots>0);
        XPW("V_MAN",pfd_guidance.green_dot_knots,pfd_guidance.clean_performance_valid && pfd_guidance.green_dot_knots>0);
        XPW("V_FE_NEXT",pfd_guidance.vfe_next_knots,pfd_guidance.envelope_valid && pfd_guidance.vfe_next_knots>0);
        /* F (V_3) and S (V_4): the FMS speeds, already limited to the PRIM's visible configurations
         * (a380x_pfd_apply_fms_flap_speeds), so a zero is "not shown" (NCD). */
        XPW("V_3",pfd_guidance.f_speed_knots,pfd_guidance.f_speed_knots>0);
        XPW("V_4",pfd_guidance.s_speed_knots,pfd_guidance.s_speed_knots>0);
#undef XPW
    }
    for(int side=0;side<2;++side) {
        const EfisSide *e=&efis[side]; char prefix[64];
        snprintf(prefix,sizeof(prefix),"L:A32NX_FCU_EFIS_%c_",side?'R':'L');
        uint32_t d=XPKIT_BIT(11,e->baro_std)|XPKIT_BIT(12,!e->baro_std)|XPKIT_BIT(13,e->baro_inhg)|
            XPKIT_BIT(14,e->ls)|XPKIT_BIT(15,e->vv)|XPKIT_BIT(17,e->cstr)|XPKIT_BIT(18,e->wpt)|
            XPKIT_BIT(19,e->vord)|XPKIT_BIT(20,e->ndb)|XPKIT_BIT(21,e->arpt);
        xpkit_word(j,prefix,"DISCRETE_WORD_2",d,e->powered);
        xpkit_word(j,prefix,"BARO_HPA",e->qnh_hpa,e->powered);
        xpkit_word(j,prefix,"BARO_INHG",e->qnh_hpa/33.8638867,e->powered);
        snprintf(prefix,sizeof(prefix),"L:A32NX_ADIRS_ADR_%d_",adirs.adr_source[side]+1);
        xpkit_word(j,prefix,side?"BARO_CORRECTED_ALTITUDE_2":"BARO_CORRECTED_ALTITUDE_1",
                   refs.side[side].altitude_value,refs.side[side].altitude_valid);
    }
    /* FBW SFCC words for the PFD slat/flap display (a380x_flaps.inc). Both SFCC label sets carry the one
     * native controller, as FBW's two SFCCs compute the same words. */
    if(flaps_feed.ready) for(int unit=1;unit<=2;++unit) {
        char prefix[64]; snprintf(prefix,sizeof(prefix),"L:A32NX_SFCC_%d_",unit);
        xpkit_word(j,prefix,"SLAT_FLAP_SYSTEM_STATUS_WORD",(double)flaps_feed.status_word,1);
        xpkit_word(j,prefix,"SLAT_FLAP_ACTUAL_POSITION_WORD",(double)flaps_feed.actual_position_word,1);
        xpkit_word(j,prefix,"SLAT_ACTUAL_POSITION_WORD",flaps_feed.slat_fppu,1);
        xpkit_word(j,prefix,"FLAP_ACTUAL_POSITION_WORD",flaps_feed.flap_fppu,1);
    }
}
#undef XPKIT_BIT
#include "a380x_xpkit_nd_data.inc"
#include "a380x_xpkit_mfd.inc"
#include "a380x_xpkit_sd.inc"
static void xpkit_update(void) {
    if(!xpkit_shared || !xpkit_pid) return;
#if APL
    int status;
    int child_exited=waitpid(xpkit_pid,&status,WNOHANG)==xpkit_pid;
#else
    int child_exited=WaitForSingleObject(xpkit_pid,0)!=WAIT_TIMEOUT;
#endif
    if(child_exited) {
#if IBM
        CloseHandle(xpkit_pid);
        if(xpkit_job){CloseHandle(xpkit_job);xpkit_job=NULL;}
#endif
        xpkit_pid=0; xpkit_health[0]=xpkit_health[1]=xpkit_health[2]=xpkit_health[6]=xpkit_health[7]=xpkit_health[13]=xpkit_health[14]=xpkit_health[17]=0;
        xpkit_health[XPKIT_EFB_HEALTH]=xpkit_health[XPKIT_EFB_HEALTH+1]=0;
        a380x_efb_needs_drawing(0); a380x_efb_needs_drawing(1);   /* repaint the native fallback */
        xpkit_nd_renderer_lost();
        xpkit_pfd_renderer_lost();
        XPLMDebugString("A380X xp-kit: helper exited; native PFD fallback.\n"); return;
    }
    double now=xpkit_now(); if(now-xpkit_last_update<.045) return;
    xpkit_last_update=now;
    static XpKitJson j; j.size=0;j.failed=0; float bus[1]={0};
    xpkit_nd_poll();
    xpkit_mfd_publish();
    xpkit_sd_publish();
    xpkit_efb_drain();
    xpkit_efb_publish();
    XPLMGetDatavf(adirs_raw[AR_BUS],bus,0,1);
    xpkit_append(&j,"{\"ok\":true,\"mode\":\"native\",\"values\":{");
    xpkit_append(&j,"\"sim/cockpit2/electrical/bus_volts\":[%.5g],",isfinite(bus[0])?bus[0]:0);
    xpkit_number(&j,"sim/time/total_flight_time_sec",xpkit_clock?XPLMGetDataf(xpkit_clock):0);
    xpkit_number(&j,"sim/time/sim_speed_actual",XPLMGetDataf(adirs_raw[AR_SPEED]));
    xpkit_number(&j,"sim/time/paused",XPLMGetDatai(adirs_raw[AR_PAUSED]));
    xpkit_number(&j,"sim/flightmodel/failures/onground_any",XPLMGetDatai(refs.on_ground));
    for(int i=0;i<ARRAY_LEN(adirs_refs);++i) xpkit_number(&j,adirs_refs[i].name,adirs_get_double(&adirs_refs[i]));
    for(int side=0;side<2;++side) {
        const Side *s=&refs.side[side]; char name[160];
        snprintf(name,sizeof(name),"a380x/xp12/du/%s_pfd",side?"first_officer":"captain");
        xpkit_number(&j,name,s->display_valid?du_channels[side*DU_CHANNELS_PER_SIDE+DU_PFD].level:0);
        snprintf(name,sizeof(name),"a380x/xp12/du/%s_nd",side?"first_officer":"captain");
        xpkit_number(&j,name,s->nd_valid?du_channels[side*DU_CHANNELS_PER_SIDE+DU_ND].level:0);
        snprintf(name,sizeof(name),"sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_%s",side?"copilot":"pilot");
        xpkit_number(&j,name,s->radio_altitude_value);
        snprintf(name,sizeof(name),"sim/operation/failures/rel_%s_radalt",side?"cop":"pil");
        xpkit_number(&j,name,failed(s->fail_radio_altitude)?6:0);
    }
    xpkit_append(&j,"\"_end\":0},\"words\":{");
    xpkit_guidance_words(&j);
    xpkit_append(&j,"\"_end\":[0,false]},\"numbers\":{");
    if(flaps_feed.ready) {   /* plain L: vars of the FBW PFD (pitch-trim display, TRUE reference) */
        xpkit_number(&j,"L:A32NX_FLAPS_HANDLE_INDEX",flaps_feed.handle_index);
        xpkit_number(&j,"L:A32NX_FLAPS_CONF_INDEX",flaps_feed.conf);
        xpkit_number(&j,"L:A32NX_LEFT_SLATS_ANGLE",flaps_feed.slat_deg);
        xpkit_number(&j,"L:A32NX_RIGHT_SLATS_ANGLE",flaps_feed.slat_deg);
        xpkit_number(&j,"L:A32NX_LEFT_FLAPS_ANGLE",flaps_feed.flap_deg);
        xpkit_number(&j,"L:A32NX_RIGHT_FLAPS_ANGLE",flaps_feed.flap_deg);
    }
    if(speedbrake_feed.ready) {   /* FBW PFD lower area: speed brake symbol, GND SPLRS armed triangle */
        xpkit_number(&j,"L:A32NX_SPOILERS_ARMED",speedbrake_feed.armed);
        xpkit_number(&j,"L:A32NX_SPOILERS_HANDLE_POSITION",speedbrake_feed.handle);
        xpkit_number(&j,"L:A32NX_LEFT_SPOILER_1_COMMANDED_POSITION",speedbrake_feed.panel_deg[0]);
    }
    xpkit_append(&j,"\"_end\":0},\"nav\":");
    xpkit_nd_json(&j);
    xpkit_append(&j,"}");
    if(j.failed) { xpkit_health[3]++; return; }
    uint32_t seq=atomic_load_explicit(&xpkit_shared->state_seq,memory_order_relaxed);
    atomic_store_explicit(&xpkit_shared->state_seq,seq+1,memory_order_release);
    memcpy(xpkit_shared->state,j.data,j.size+1); xpkit_shared->state_size=(uint32_t)j.size;
    xpkit_shared->state_time=now;
    atomic_store_explicit(&xpkit_shared->state_seq,seq+2,memory_order_release);
    xpkit_health[3]=(int)atomic_load_explicit(&xpkit_shared->errors,memory_order_acquire);
    for(int i=0;i<2;++i) xpkit_health[i+1]=xpkit_health[0] && atomic_load(&xpkit_shared->ready[i]) &&
        atomic_load(&xpkit_shared->frame_seq[i])>0 && now-xpkit_shared->frame_time[i]<1.0;
    for(int i=0;i<2;++i) xpkit_health[i+6]=xpkit_health[0] && atomic_load(&xpkit_shared->ready[i+2]) &&
        atomic_load(&xpkit_shared->frame_seq[i+2])>0 && now-xpkit_shared->frame_time[i+2]<1.0;
    for(int i=0;i<2;++i) {
        xpkit_health[i+13]=xpkit_health[0] && atomic_load(&xpkit_shared->ready[i+4]) &&
            atomic_load(&xpkit_shared->frame_seq[i+4])>0 && now-xpkit_shared->frame_time[i+4]<1.0;
        /* mfda_update requests page/cursor redraws; only a newly completed
         * texture needs another request here (not an unconditional 20 Hz draw). */
        if(mfda_dev[i].handle && mfda_av.loaded &&
           atomic_load(&xpkit_shared->frame_seq[i+4])!=xpkit_uploaded[i+4])
            mfda_av.needs_drawing(mfda_dev[i].handle);
    }
    for(int i=0;i<2;++i) {
        /* power value also changes with the CDS boot / self-test state (device redraw) */
        XpKitNd *d=&xpkit_nd[i]; int live=xpkit_health[i+6],powered=xpkit_nd_on(d)?8:a380x_power_du_state(xpkit_nd_du(d))+1;
        if(d->device && mfda_av.loaded &&
           xpkit_nd_frame_pending(d,atomic_load(&xpkit_shared->frame_seq[i+2]),live,powered))
            mfda_av.needs_drawing(d->device);
        d->draw_live=live;d->draw_powered=powered;
    }
    xpkit_health[17]=atomic_load(&xpkit_shared->ready[6])&&atomic_load(&xpkit_shared->frame_seq[6])>0&&now-xpkit_shared->frame_time[6]<1.0;
    if(sda.device&&mfda_av.loaded&&atomic_load(&xpkit_shared->frame_seq[6])!=xpkit_uploaded[6])mfda_av.needs_drawing(sda.device);
    xpkit_efb_poll(now);
    xpkit_pfd_poll();
    if(!xpkit_logged && xpkit_health[1] && xpkit_health[2]) {
        XPLMDebugString("A380X xp-kit: BOTH FlyByWire library PFDs active (768x1024 cockpit devices, atlas fallback).\n"); xpkit_logged=1;
    }
}
static int xpkit_draw_view(int side,float x,float y,float width,float height) {
    int health=side<2?side+1:side<4?side+4:side<6?side+9:side<XPKIT_EFB_VIEW0?17:XPKIT_EFB_HEALTH+side-XPKIT_EFB_VIEW0;
    if(!xpkit_shared || !xpkit_health[0] || !xpkit_health[health] ||
       xpkit_now()-xpkit_shared->frame_time[side]>=1.0) return 0;
    unsigned seq=atomic_load_explicit(&xpkit_shared->frame_seq[side],memory_order_acquire);
    if(!(seq&1) && seq!=xpkit_uploaded[side]) {
        memcpy(xpkit_staging,xpkit_view_pixels(xpkit_shared,side),xpkit_view_bytes(side));
        atomic_thread_fence(memory_order_acquire);
        if(atomic_load_explicit(&xpkit_shared->frame_seq[side],memory_order_acquire)==seq) {
            if(!xpkit_texture[side]) {
                int texture; XPLMGenerateTextureNumbers(&texture,1); xpkit_texture[side]=(GLuint)texture;
                XPLMBindTexture2d(texture,0);
                glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
                glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
                glTexImage2D(GL_TEXTURE_2D,0,side>=2&&side<4?GL_RGBA8:GL_RGBA,
                    xpkit_view_width(side),xpkit_view_height(side),0,GL_RGBA,GL_UNSIGNED_BYTE,xpkit_staging);
            } else {
                XPLMBindTexture2d((int)xpkit_texture[side],0);
                glTexSubImage2D(GL_TEXTURE_2D,0,0,0,xpkit_view_width(side),xpkit_view_height(side),GL_RGBA,GL_UNSIGNED_BYTE,xpkit_staging);
            }
            xpkit_uploaded[side]=seq;
            if(side>=2 && side<4) {
                atomic_fetch_add_explicit(&xpkit_shared->nd_profile[side-2].upload_count,1,memory_order_relaxed);
                atomic_fetch_add_explicit(&xpkit_shared->nd_profile[side-2].upload_bytes,xpkit_view_bytes(side),memory_order_relaxed);
            }
            xpkit_health[side<2?side+4:side<4?side+6:side<6?side+11:side<XPKIT_EFB_VIEW0?18:XPKIT_EFB_HEALTH+2+side-XPKIT_EFB_VIEW0]=(int)(seq/2);
        }
    }
    if(!xpkit_uploaded[side]) return 0;
    XPLMSetGraphicsState(0,1,0,0,1,0,0); XPLMBindTexture2d((int)xpkit_texture[side],0);
    glColor4f(1,1,1,1); glBegin(GL_QUADS);
    glTexCoord2f(0,1);glVertex2f(x,y);glTexCoord2f(1,1);glVertex2f(x+width,y);
    glTexCoord2f(1,0);glVertex2f(x+width,y+height);glTexCoord2f(0,0);glVertex2f(x,y+height);
    glEnd(); XPLMSetGraphicsState(0,0,0,0,1,0,0); return 1;
}
static int xpkit_draw(int side,float x,float y) { return xpkit_draw_view(side,x,y,320,416); }
static int xpkit_draw_nd(int side,float x,float y) { return xpkit_draw_view(side+2,x,y,180,234); }
static int xpkit_draw_mfd(int side,float x,float y,float scale) { return xpkit_draw_view(side+4,x,y,768*scale,1024*scale); }
static int xpkit_draw_sd(void) { return xpkit_draw_view(6,0,0,768,1024); }
#else
static void xpkit_init(void) {}
static void xpkit_start(void) {}
static void xpkit_update(void) {}
static void xpkit_stop(void) {}
static void xpkit_shutdown(void) {}
static int xpkit_draw(int side,float x,float y) { (void)side;(void)x;(void)y;return 0; }
static int xpkit_pfd_atlas_needed(int side) { (void)side; return 1; }
static int xpkit_draw_nd(int side,float x,float y) { (void)side;(void)x;(void)y;return 0; }
static int xpkit_draw_mfd(int side,float x,float y,float scale) { (void)side;(void)x;(void)y;(void)scale;return 0; }
static int xpkit_draw_sd(void) { return 0; }
#endif
