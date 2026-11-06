/* SPDX-License-Identifier: GPL-3.0-or-later
 * Actual xp-kit PFDs, composited into the existing A380 cockpit atlas.
 * Only the native aircraft controller is authoritative. No HTTP, stock AP
 * mode mirroring, synthetic flight state, or WebKit code in X-Plane's process.
 *
 * Frames (protocol v9, a380x_xpkit_shared.h): on macOS the helper writes each captured page into
 * IOSurfaces created here and the cockpit devices sample them directly (zero-copy, after a one-time
 * GL probe); otherwise frames are uploaded with glTexSubImage2D, at most XPKIT_UPLOADS_PER_FRAME
 * PFD/ND uploads per simulator frame, oldest pending first.
 * PFD/ND state is published at up to 75 Hz, the MFD/SD/EFB publishers keep their 45 ms cadence, and
 * every presentation poll (health, device redraw requests, ND cursor sampling) runs every frame.
 * The helper is supervised: an exited helper is restarted after 2 s (at most 3 times per enable), an
 * unresponsive one (no PFD frame or heartbeat for 15 s) is killed and restarted. The PFD/ND devices
 * hold their last web frame meanwhile; the native PFD/ND renderers draw only when no web renderer
 * can run (helper absent, spawn failure, Windows without a host, restarts used up).
 */
#if (APL || IBM) && defined(A380X_XPKIT)
#include "a380x_xpkit_shared.h"
#include <stdarg.h>
#if APL
#include <spawn.h>
#include <signal.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include "a380x_xpkit_surface.h"
extern char **environ;
static A380XXpKitShared *xpkit_shared;
static pid_t xpkit_pid;
static char xpkit_shared_path[128];
static char xpkit_helper_path[2304], xpkit_assets_path[2304], xpkit_system_path[2048];
#else
static A380XXpKitShared *xpkit_shared;
static HANDLE xpkit_pid, xpkit_mapping, xpkit_job;
static wchar_t xpkit_shared_name[128];
#endif
static GLuint xpkit_texture[XPKIT_VIEWS];
static unsigned xpkit_uploaded[XPKIT_VIEWS];   /* frame_seq of the frame each view presents (0: none) */
static unsigned char xpkit_staging[XPKIT_MAX_VIEW_BYTES];
static int xpkit_health[26]; /* Existing PFD/ND contracts retain their indices; 22..25 = EFB tablets. */
static XPLMDataRef xpkit_health_refs[26];
static const char *xpkit_health_names[]={"helper_running","captain_active","first_officer_active","errors",
    "captain_presented_frames","first_officer_presented_frames","captain_nd_active","first_officer_nd_active",
    "captain_nd_presented_frames","first_officer_nd_presented_frames","nd_devices_active","captain_nd_bound","first_officer_nd_bound",
    "captain_mfd_active","first_officer_mfd_active","captain_mfd_presented_frames","first_officer_mfd_presented_frames","sd_active","sd_presented_frames",
    "pfd_devices_active","captain_pfd_bound","first_officer_pfd_bound",
    "captain_efb_active","first_officer_efb_active","captain_efb_presented_frames","first_officer_efb_presented_frames"};
/* Web renderer of this plugin enable: running, waiting for a restart, or impossible (native PFD/ND). */
enum { XPKIT_WEB_NONE, XPKIT_WEB_RUNNING, XPKIT_WEB_RESTARTING };
#define XPKIT_MAX_RESTARTS 3
#define XPKIT_RESTART_DELAY 2.0
#define XPKIT_WATCHDOG 15.0              /* seconds without a PFD frame or heartbeat */
#define XPKIT_STATE_PERIOD (1.0 / 75.0)   /* PFD/ND state publication */
#define XPKIT_SLOW_PERIOD .045            /* MFD/SD/EFB publishers (unchanged) */
#define XPKIT_UPLOADS_PER_FRAME 2         /* upload path: PFD/ND uploads per simulator frame... */
#define XPKIT_UPLOAD_BYTES_PER_FRAME (16u << 20)   /* ...and bytes (one PFD or ND frame is 7.1 MB) */
static int xpkit_web;
static int xpkit_web_possible(void) { return xpkit_web != XPKIT_WEB_NONE; }
static int xpkit_draw_view(int view,float x,float y,float width,float height);
static void xpkit_device_draw_begin(void);   /* top of the PFD/ND device draws: one-time zero-copy probe */
static double xpkit_now(void);
static void draw_nd(const Side *side,float x,float y,const char *name);
static void draw_pfd(const Side *side,float x,float y,const char *name);
#include "a380x_xpkit_nd_input.inc"
#include "a380x_xpkit_pfd.inc"
#include "a380x_xpkit_efb.inc"
static XPLMDataRef xpkit_clock;
static double xpkit_last_state, xpkit_last_slow, xpkit_loop_time;
static double xpkit_watch_since, xpkit_watch_progress, xpkit_restart_at;
static int xpkit_restarts;
static int xpkit_logged;
/* What xpkit_uploaded[] names per view: 0 nothing, 1 the uploaded 2D texture, 2 surface xpkit_front[]. */
static unsigned char xpkit_kind[XPKIT_VIEWS];
static unsigned xpkit_requested[XPKIT_VIEWS];      /* MFD/SD/EFB: frame a redraw was requested for */
static double xpkit_requested_at[XPKIT_VIEWS];
/* Upload budget of the PFD/ND views (upload path only). */
static unsigned xpkit_frame_no, xpkit_grant[4], xpkit_waiting_since[4];
static unsigned char xpkit_grant_used[4];
/* Zero-copy for this X-Plane session: 0 not probed yet, 1 probe passed, -1 upload path only. */
static int xpkit_zero_copy;
#if APL
static unsigned char xpkit_front[XPKIT_VIEWS];
static XpKitSurface xpkit_surface[XPKIT_VIEWS][XPKIT_SURFACES];
static GLuint xpkit_surface_tex[XPKIT_VIEWS][XPKIT_SURFACES];
static XpKitSurface xpkit_surface_bound[XPKIT_VIEWS][XPKIT_SURFACES];   /* surface each texture samples */
#endif
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
static int xpkit_spawn(void) {
    char *argv[]={xpkit_helper_path,xpkit_shared_path,xpkit_assets_path,xpkit_system_path,NULL};
    if(posix_spawn(&xpkit_pid,xpkit_helper_path,NULL,NULL,argv,environ)) { xpkit_pid=0; return 0; }
    return 1;
}
/* 1 when the helper is gone (reaped here, or no longer our child); status for the log. */
static int xpkit_helper_gone(int *status) {
    pid_t r=waitpid(xpkit_pid,status,WNOHANG);
    return r==xpkit_pid || (r<0 && errno==ECHILD);
}
static void xpkit_kill_helper(void) {
    int status;
    if(xpkit_pid>0 && waitpid(xpkit_pid,&status,WNOHANG)==0) { kill(xpkit_pid,SIGKILL); waitpid(xpkit_pid,&status,0); }
}

/* ---- zero-copy surfaces (a380x_xpkit_surface.h) ---- */
static void xpkit_surfaces_release(void) {
    for(int v=0;v<XPKIT_VIEWS;++v) for(int i=0;i<XPKIT_SURFACES;++i) {
        if(xpkit_surface_tex[v][i]) { glDeleteTextures(1,&xpkit_surface_tex[v][i]); xpkit_surface_tex[v][i]=0; }
        xpkit_surface_bound[v][i]=NULL;
        if(xpkit_surface[v][i]) { xpkit_surface_release(xpkit_surface[v][i]); xpkit_surface[v][i]=NULL; }
        if(xpkit_shared) atomic_store(&xpkit_shared->surface_id[v][i],0);
    }
    if(xpkit_shared) for(int v=0;v<XPKIT_VIEWS;++v) atomic_store(&xpkit_shared->surface_mode[v],0);
}
/* Once per enable, before the helper starts. surface_mode turns 1 only once the probe has passed. */
static void xpkit_surfaces_create(void) {
    if(xpkit_zero_copy<0 || !xpkit_shared) return;
    for(int v=0;v<XPKIT_VIEWS;++v) for(int i=0;i<XPKIT_SURFACES;++i) {
        XpKitSurface s=xpkit_surface_create(xpkit_view_width(v),xpkit_view_height(v));
        if(!s) {
            xpkit_surfaces_release(); xpkit_zero_copy=-1;
            XPLMDebugString("A380X xp-kit: IOSurfaceCreate failed; display path: texture upload.\n");
            return;
        }
        xpkit_surface[v][i]=s;
        atomic_store(&xpkit_shared->surface_id[v][i],(uint32_t)xpkit_surface_id(s));
    }
    if(xpkit_zero_copy>0) {
        for(int v=0;v<XPKIT_VIEWS;++v) atomic_store_explicit(&xpkit_shared->surface_mode[v],1,memory_order_release);
        XPLMDebugString("A380X xp-kit: display path: zero-copy IOSurface frames (GL probe passed earlier this session).\n");
    }
}
/* Zero-copy off for the session: the helper re-captures every view into the pixel arrays. Surfaces a
 * texture may still reference stay allocated until the plugin is disabled. */
static void xpkit_zero_copy_off(const char *why) {
    char line[320];
    xpkit_zero_copy=-1;
    if(xpkit_shared) for(int v=0;v<XPKIT_VIEWS;++v) atomic_store_explicit(&xpkit_shared->surface_mode[v],0,memory_order_release);
    for(int v=0;v<XPKIT_VIEWS;++v) if(xpkit_kind[v]==2) { xpkit_kind[v]=0; xpkit_uploaded[v]=0; }
    snprintf(line,sizeof(line),"A380X xp-kit: zero-copy display frames unavailable (%s); display path: texture upload "
             "(at most %d PFD/ND uploads per frame).\n",why,XPKIT_UPLOADS_PER_FRAME);
    XPLMDebugString(line);
}
/* First device draw (GL current): prove that this context samples a CPU-written IOSurface. */
static void xpkit_zero_copy_probe(void) {
    char why[160]; int names[2]; unsigned textures[2]; int passed;
    if(xpkit_zero_copy || !xpkit_surface[0][0]) return;
    XPLMGenerateTextureNumbers(names,2); textures[0]=(unsigned)names[0]; textures[1]=(unsigned)names[1];
    passed=xpkit_surface_probe(getenv("A380X_XPKIT_FORCE_UPLOAD")!=NULL,textures,why,sizeof(why));
    glDeleteTextures(2,(const GLuint *)textures);
    /* No texture samples a surface before the probe passed: free them (about 200 MB) right away. */
    if(!passed) { xpkit_zero_copy_off(why); xpkit_surfaces_release(); return; }
    xpkit_zero_copy=1;
    if(xpkit_shared) for(int v=0;v<XPKIT_VIEWS;++v) atomic_store_explicit(&xpkit_shared->surface_mode[v],1,memory_order_release);
    XPLMDebugString("A380X xp-kit: display path: zero-copy IOSurface frames (GL probe passed, no texture uploads).\n");
}
/* The rectangle texture sampling surface [view][index], attached lazily with CGLTexImageIOSurface2D. */
static GLuint xpkit_surface_texture(int view,int index) {
    XpKitSurface s=xpkit_surface[view][index];
    if(!s) return 0;
    if(xpkit_surface_tex[view][index] && xpkit_surface_bound[view][index]==s) return xpkit_surface_tex[view][index];
    if(!xpkit_surface_tex[view][index]) { int name; XPLMGenerateTextureNumbers(&name,1); xpkit_surface_tex[view][index]=(GLuint)name; }
    if(!xpkit_surface_attach(xpkit_surface_tex[view][index],s,xpkit_view_width(view),xpkit_view_height(view))) return 0;
    xpkit_surface_bound[view][index]=s;
    return xpkit_surface_tex[view][index];
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
            pid_t r=waitpid(xpkit_pid,&status,WNOHANG);
            if(r!=0) break;
            struct timespec pause={0,5000000}; nanosleep(&pause,NULL);
        }
        if(n==20) { kill(xpkit_pid,SIGKILL); waitpid(xpkit_pid,&status,0); }
    }
    xpkit_pid=0;
    xpkit_surfaces_release();
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
    memset(xpkit_kind,0,sizeof(xpkit_kind));
    memset(xpkit_requested,0,sizeof(xpkit_requested));
    memset(xpkit_waiting_since,0,sizeof(xpkit_waiting_since));
    xpkit_logged=0;
    xpkit_web=XPKIT_WEB_NONE; xpkit_restarts=0;
}
static void xpkit_start(void) {
    char plugin[2048], *slash;
#if IBM
    char helper[2304], assets[2304], system[2048];
#else
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
        XPLMDebugString("A380X xp-kit: Windows helper could not start; native PFD/ND (inspect aircraft plugin files and display-host.log).\n");
        xpkit_stop();return;
    }
#else
    snprintf(xpkit_helper_path,sizeof(xpkit_helper_path),"%s/A380XDisplayHost",plugin);
    snprintf(xpkit_assets_path,sizeof(xpkit_assets_path),"%s/xp-kit",plugin);
    if(!xpkit_prepare_helper(xpkit_helper_path)) {
        XPLMDebugString("A380X xp-kit: display helper absent; native PFD/ND.\n"); return;
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
    for(int v=0;v<XPKIT_VIEWS;++v) atomic_store(&xpkit_shared->surface_front[v],XPKIT_SURFACE_PIXELS);
    xpkit_surfaces_create();
    XPLMGetSystemPath(xpkit_system_path);
    if(!xpkit_spawn()) {
        XPLMDebugString("A380X xp-kit: display helper could not be started; native PFD/ND.\n");
        xpkit_stop(); return;
    }
#endif
    xpkit_web=XPKIT_WEB_RUNNING; xpkit_restarts=0;
    xpkit_health[0]=1; xpkit_last_state=xpkit_last_slow=0;
    xpkit_watch_since=xpkit_now(); xpkit_watch_progress=0;
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
    /* Armed modes: ALT 11, G/S 13, CLB 15, DES 16, NAV 22, LOC 23 (FBW A380 PFD B2/C2 cells). */
    uint32_t d2=XPKIT_BIT(11,g->altitude_armed)|XPKIT_BIT(13,g->gs_armed)|XPKIT_BIT(23,g->loc_armed)|
        XPKIT_BIT(15,g->clb_armed)|XPKIT_BIT(16,g->des_armed)|XPKIT_BIT(22,g->nav_armed);
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
        /* SRS: takeoff (FMGC) or go-around (autoland, PITCH GA). */
        case A380X_B1_SRS:d3=g->land.go_around_active?XPKIT_BIT(16,1):XPKIT_BIT(15,1);break;
        default:break; /* Unsupported modes are not invented. */
    }
    /* ALT CST(*) / ALT CRZ(*) qualifiers of ALT / ALT* (bit 28 also makes an armed ALT magenta). */
    d3|=XPKIT_BIT(28,g->alt_constraint)|XPKIT_BIT(29,g->alt_cruise&&!g->alt_constraint);
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
    {
        /* PrimFgPublisher.ts bit map: word 1 bit 23 LAND, 27/28/29 LAND 2 / LAND 3 fail-passive (SINGLE) /
         * fail-op (DUAL) capability, only the annunciated one; word 2 bit 28 LAND armed; word 3 bit 24
         * FLARE; word 4 bits 25 ALIGN / 26 ROLL OUT. LAND replaces the G/S and LOC words (BC1Cell). */
        const A380XLandingState *l=&g->land;
        int cap_bit=a380x_land_capability_bit(l->capability);
        if(l->land_active) {
            d3&=~(XPKIT_BIT(21,1)|XPKIT_BIT(22,1)); d4&=~(XPKIT_BIT(13,1)|XPKIT_BIT(14,1));
        }
        d1|=XPKIT_BIT(23,l->land_active)|(cap_bit>=0?XPKIT_BIT(27+cap_bit,1):0);
        d2|=XPKIT_BIT(28,g->approach_selected && !l->land_active);
        d3|=XPKIT_BIT(24,l->land_active && l->flare_active);
        d4|=XPKIT_BIT(25,l->align_active)|XPKIT_BIT(26,l->rollout_active);
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
        XPW("FG_ATS_DISCRETE_WORD",XPKIT_BIT(11,g->athr_armed)|XPKIT_BIT(12,g->athr_active)|
            XPKIT_BIT(19,g->land.retard_active),g->initialized);
        XPW("FG_ATS_FMA_DISCRETE_WORD",ats,g->initialized);
        /* FM altitude constraint (magenta target) while it limits the managed CLB/DES target. */
        XPW("FM_ALTITUDE_CONSTRAINT",guidance_inputs.managed_altitude_ft,
            g->initialized && g->alt_constraint && guidance_inputs.managed_altitude_is_constraint);
        XPW("ROLL_FD_COMMAND_1",g->fd_roll_error_deg,g->guidance_valid);
        XPW("ROLL_FD_COMMAND_2",g->fd_roll_error_deg,g->guidance_valid);
        XPW("PITCH_FD_COMMAND_1",-g->fd_pitch_error_deg,g->guidance_valid && g->vertical_guidance_selected);
        XPW("PITCH_FD_COMMAND_2",-g->fd_pitch_error_deg,g->guidance_valid && g->vertical_guidance_selected);
        /* VMAX: VMO/MMO, VFE and landing-gear limits (a380x_pfd_inputs.inc): the overspeed barber pole. */
        XPW("V_MAX",pfd_guidance.vmax_knots,pfd_guidance.vmax_knots>0);
        /* Low-speed envelope: VSW (red/black barber pole) from the port stall-warning speed; alpha prot /
         * alpha max only from an aircraft-systems provider (no alpha protection in this port). */
        XPW("V_STALL_WARN",pfd_guidance.stall_warning_knots,pfd_guidance.stall_warning_knots>0);
        XPW("V_ALPHA_PROT",pfd_guidance.alpha_prot_knots,pfd_guidance.alpha_prot_knots>0);
        XPW("V_ALPHA_LIM",pfd_guidance.alpha_limit_knots,pfd_guidance.alpha_limit_knots>0);
        XPW("V_LS",pfd_guidance.vls_knots,pfd_guidance.clean_performance_valid && pfd_guidance.vls_knots>0);
        XPW("V_MAN",pfd_guidance.green_dot_knots,pfd_guidance.clean_performance_valid && pfd_guidance.green_dot_knots>0);
        XPW("V_FE_NEXT",pfd_guidance.vfe_next_knots,pfd_guidance.envelope_valid && pfd_guidance.vfe_next_knots>0);
        /* F (V_3) and S (V_4): the FMS speeds, already limited to the PRIM's visible configurations
         * (a380x_pfd_apply_fms_flap_speeds), so a zero is "not shown" (NCD). */
        XPW("V_3",pfd_guidance.f_speed_knots,pfd_guidance.f_speed_knots>0);
        XPW("V_4",pfd_guidance.s_speed_knots,pfd_guidance.s_speed_knots>0);
#undef XPW
    }
    /* FcdcPublisher.ts FG word 1 (FMA D1D2 capability): bit 18 LAND, 19 GA, 24/25/26 LAND 2 / LAND 3
     * SINGLE / LAND 3 DUAL (exactly one: computeD1D2Message tests LAND 2 first). */
    for(int unit=1;unit<=2;++unit) {
        char prefix[64]; int cap_bit=a380x_land_capability_bit(g->land.capability);
        snprintf(prefix,sizeof(prefix),"L:A32NX_FCDC_%d_",unit);
        xpkit_word(j,prefix,"FG_DISCRETE_WORD_1",XPKIT_BIT(18,g->land.land_active)|
            XPKIT_BIT(19,g->land.go_around_active)|(cap_bit>=0?XPKIT_BIT(24+cap_bit,1):0),
            g->initialized);
    }
    /* FMA D3 minimums (FMA.tsx D3Cell, PFDSimvarPublisher.tsx fm1/fm2 words): EIS discrete word 2
     * bit 29 NO DH, DECISION_HEIGHT (RADIO) and MINIMUM_DESCENT_ALTITUDE (BARO, also the altitude
     * tape minimum) from the native FMS PERF APPR entry (fms_feed.minimum_mode). */
    for(int unit=1;unit<=2;++unit) {
        char prefix[64]; int mode=fms_feed.valid?fms_feed.minimum_mode:0;
        snprintf(prefix,sizeof(prefix),"L:A32NX_FM%d_",unit);
        xpkit_word(j,prefix,"EIS_DISCRETE_WORD_2",XPKIT_BIT(29,mode==3),fms_feed.valid);
        xpkit_word(j,prefix,"DECISION_HEIGHT",mode==1?fms_feed.minimum_ft:0,mode==1);
        xpkit_word(j,prefix,"MINIMUM_DESCENT_ALTITUDE",mode==2?fms_feed.minimum_ft:0,mode==2);
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
#include "a380x_xpkit_terrain.inc"   /* TERRAIN on ND rasters for the web NDs */
#include "a380x_xpkit_mfd.inc"
#include "a380x_xpkit_sd.inc"
/* ---- helper supervision ---- */
static void xpkit_helper_lost(double now,const char *what) {
    char line[320];
#if IBM
    CloseHandle(xpkit_pid);
    if(xpkit_job){CloseHandle(xpkit_job);xpkit_job=NULL;}
#endif
    xpkit_pid=0; xpkit_health[0]=xpkit_health[1]=xpkit_health[2]=xpkit_health[6]=xpkit_health[7]=xpkit_health[13]=xpkit_health[14]=xpkit_health[17]=0;
    xpkit_health[XPKIT_EFB_HEALTH]=xpkit_health[XPKIT_EFB_HEALTH+1]=0;
    if(xpkit_restarts<XPKIT_MAX_RESTARTS) {
        xpkit_web=XPKIT_WEB_RESTARTING; xpkit_restart_at=now+XPKIT_RESTART_DELAY;
        snprintf(line,sizeof(line),"A380X xp-kit: display helper %s; restart %d/%d in %.0f s (PFD/ND keep their last frame).\n",
                 what,xpkit_restarts+1,XPKIT_MAX_RESTARTS,XPKIT_RESTART_DELAY);
    } else {
        xpkit_web=XPKIT_WEB_NONE;
        snprintf(line,sizeof(line),"A380X xp-kit: display helper %s; %d restarts used, native PFD/ND until the plugin is re-enabled.\n",
                 what,XPKIT_MAX_RESTARTS);
    }
    XPLMDebugString(line);
    a380x_efb_needs_drawing(0); a380x_efb_needs_drawing(1);   /* repaint the native fallback */
    xpkit_nd_renderer_lost();
    xpkit_pfd_renderer_lost();
}
/* Same mapping, same surfaces: the restarted helper continues the sequences and writes the next
 * surface, so the last presented frame stays valid. A frame the dead helper left half-written (odd
 * sequence) is closed and never presented. */
static void xpkit_restart(double now) {
    char line[200];
    A380XXpKitShared *s=xpkit_shared;
    int started;
    ++xpkit_restarts;
    if(s) for(int v=0;v<XPKIT_VIEWS;++v) {
        atomic_store(&s->ready[v],0);
        unsigned seq=atomic_load(&s->frame_seq[v]);
        if(seq&1) { atomic_store(&s->frame_seq[v],seq+1); xpkit_uploaded[v]=seq+1; }
    }
#if IBM
    started=s && xpkit_windows_spawn();
#else
    started=s && xpkit_spawn();
#endif
    if(!started) {
        xpkit_web=XPKIT_WEB_NONE;
        XPLMDebugString("A380X xp-kit: display helper restart failed; native PFD/ND until the plugin is re-enabled.\n");
        xpkit_nd_renderer_lost(); xpkit_pfd_renderer_lost();
        return;
    }
    xpkit_web=XPKIT_WEB_RUNNING; xpkit_health[0]=1;
    xpkit_watch_since=now; xpkit_watch_progress=s->frame_time[0];
    snprintf(line,sizeof(line),"A380X xp-kit: display helper restarted (%d/%d).\n",xpkit_restarts,XPKIT_MAX_RESTARTS);
    XPLMDebugString(line);
}
static void xpkit_supervise(double now) {
    if(xpkit_web==XPKIT_WEB_RUNNING && xpkit_pid) {
        char what[96];
        int exited;
#if APL
        int status=0;
        exited=xpkit_helper_gone(&status);
        if(exited) {
            if(WIFSIGNALED(status)) snprintf(what,sizeof(what),"exited (signal %d)",WTERMSIG(status));
            else snprintf(what,sizeof(what),"exited (status %d)",WIFEXITED(status)?WEXITSTATUS(status):-1);
        }
#else
        DWORD code=0;
        exited=WaitForSingleObject(xpkit_pid,0)!=WAIT_TIMEOUT;
        if(exited) { GetExitCodeProcess(xpkit_pid,&code); snprintf(what,sizeof(what),"exited (code %lu)",(unsigned long)code); }
#endif
        if(!exited && xpkit_shared) {
            /* Progress = a PFD frame or heartbeat. A stalled simulator loop (loading) is not the helper's fault. */
            double progress=xpkit_shared->frame_time[0];
            if(progress!=xpkit_watch_progress) { xpkit_watch_progress=progress; xpkit_watch_since=now; }
            if(xpkit_loop_time>0 && now-xpkit_loop_time>1.0) xpkit_watch_since=now;
            if(now-xpkit_watch_since>XPKIT_WATCHDOG) {
#if APL
                xpkit_kill_helper();
#else
                xpkit_windows_kill();
#endif
                snprintf(what,sizeof(what),"unresponsive (no PFD frame or heartbeat for %.0f s), killed",XPKIT_WATCHDOG);
                exited=1;
            }
        }
        if(exited) xpkit_helper_lost(now,what);
    }
    if(xpkit_web==XPKIT_WEB_RESTARTING && now>=xpkit_restart_at) xpkit_restart(now);
    xpkit_loop_time=now;
}

/* ---- frames: acquire (surface or upload) and present ---- */
static void xpkit_presented(int view,unsigned seq) {
    xpkit_uploaded[view]=seq;
    xpkit_health[view<2?view+4:view<4?view+6:view<6?view+11:view<XPKIT_EFB_VIEW0?18:XPKIT_EFB_HEALTH+2+view-XPKIT_EFB_VIEW0]=(int)(seq/2);
}
/* Take the newest complete frame of a view, if it is new: a surface index (zero-copy) or an upload
 * (PFD/ND only with this frame's upload grant). Otherwise the view keeps presenting its last frame. */
static void xpkit_acquire(int view) {
    if(!xpkit_shared) return;
    unsigned seq=atomic_load_explicit(&xpkit_shared->frame_seq[view],memory_order_acquire);
    if((seq&1) || seq==xpkit_uploaded[view]) return;
    uint32_t front=atomic_load_explicit(&xpkit_shared->surface_front[view],memory_order_acquire);
    if(front<XPKIT_SURFACES) {
#if APL
        if(xpkit_zero_copy>0 && atomic_load_explicit(&xpkit_shared->frame_seq[view],memory_order_acquire)==seq) {
            if(!xpkit_surface_texture(view,(int)front)) { xpkit_zero_copy_off("CGLTexImageIOSurface2D failed for a display surface"); return; }
            xpkit_kind[view]=2; xpkit_front[view]=(unsigned char)front; xpkit_presented(view,seq);
        }
#endif
        return;   /* a surface frame this session cannot sample: the helper re-captures into the pixel arrays */
    }
    if(view<4 && (xpkit_grant[view]!=xpkit_frame_no || xpkit_grant_used[view])) return;
    memcpy(xpkit_staging,xpkit_view_pixels(xpkit_shared,view),xpkit_view_bytes(view));
    atomic_thread_fence(memory_order_acquire);
    if(atomic_load_explicit(&xpkit_shared->frame_seq[view],memory_order_acquire)!=seq) return;
    if(!xpkit_texture[view]) {
        int texture; XPLMGenerateTextureNumbers(&texture,1); xpkit_texture[view]=(GLuint)texture;
        XPLMBindTexture2d(texture,0);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D,0,view>=2&&view<4?GL_RGBA8:GL_RGBA,
            xpkit_view_width(view),xpkit_view_height(view),0,GL_RGBA,GL_UNSIGNED_BYTE,xpkit_staging);
    } else {
        XPLMBindTexture2d((int)xpkit_texture[view],0);
        glTexSubImage2D(GL_TEXTURE_2D,0,0,0,xpkit_view_width(view),xpkit_view_height(view),GL_RGBA,GL_UNSIGNED_BYTE,xpkit_staging);
    }
    if(view<4) { xpkit_grant_used[view]=1; xpkit_waiting_since[view]=0; }
    if(view>=2 && view<4) {
        atomic_fetch_add_explicit(&xpkit_shared->nd_profile[view-2].upload_count,1,memory_order_relaxed);
        atomic_fetch_add_explicit(&xpkit_shared->nd_profile[view-2].upload_bytes,xpkit_view_bytes(view),memory_order_relaxed);
    }
    xpkit_kind[view]=1; xpkit_presented(view,seq);
}
static int xpkit_present(int view,float x,float y,float width,float height) {
#if APL
    if(xpkit_kind[view]==2) {
        GLuint texture=xpkit_surface_texture(view,xpkit_front[view]);
        if(!texture) return 0;
        XPLMSetGraphicsState(0,0,0,0,1,0,0);
        xpkit_surface_quad(texture,xpkit_view_width(view),xpkit_view_height(view),x,y,width,height);
        return 1;
    }
#endif
    if(xpkit_kind[view]!=1 || !xpkit_texture[view]) return 0;
    XPLMSetGraphicsState(0,1,0,0,1,0,0); XPLMBindTexture2d((int)xpkit_texture[view],0);
    glColor4f(1,1,1,1); glBegin(GL_QUADS);
    glTexCoord2f(0,1);glVertex2f(x,y);glTexCoord2f(1,1);glVertex2f(x+width,y);
    glTexCoord2f(1,0);glVertex2f(x+width,y+height);glTexCoord2f(0,0);glVertex2f(x,y+height);
    glEnd(); XPLMSetGraphicsState(0,0,0,0,1,0,0); return 1;
}
/* PFD/ND (views 0..3) present their last frame whatever its age; the MFD/SD/EFB views only a fresh
 * frame of a live helper (their callers then draw the native page). device: a cockpit-device draw
 * callback (runs the one-time zero-copy probe), not the panel atlas. */
static int xpkit_draw_view_from(int view,float x,float y,float width,float height,int device) {
#if APL
    if(device) xpkit_zero_copy_probe();
#else
    (void)device;
#endif
    if(view>=4) {
        int health=view<6?view+9:view<XPKIT_EFB_VIEW0?17:XPKIT_EFB_HEALTH+view-XPKIT_EFB_VIEW0;
        if(!xpkit_shared || !xpkit_health[0] || !xpkit_health[health] ||
           xpkit_now()-xpkit_shared->frame_time[view]>=1.0) return 0;
    }
    xpkit_acquire(view);
    return xpkit_present(view,x,y,width,height);
}
static int xpkit_draw_view(int view,float x,float y,float width,float height) {
    return xpkit_draw_view_from(view,x,y,width,height,1);
}
static void xpkit_device_draw_begin(void) {
#if APL
    xpkit_zero_copy_probe();
#endif
}
static int xpkit_draw(int side,float x,float y) { return xpkit_web_possible() && xpkit_draw_view_from(side,x,y,320,416,0); }
static int xpkit_draw_nd(int side,float x,float y) { return xpkit_web_possible() && xpkit_draw_view_from(side+2,x,y,180,234,0); }
static int xpkit_draw_mfd(int side,float x,float y,float scale) { return xpkit_draw_view(side+4,x,y,768*scale,1024*scale); }
static int xpkit_draw_sd(void) { return xpkit_draw_view(6,0,0,768,1024); }
/* The panel-atlas PFD/ND regions are drawn only while their cockpit devices are not active: exactly
 * the values a380x_mfd_screens.obj / a380x_cockpit_controls.obj hide them with. */
static int xpkit_pfd_atlas_needed(void) { return !xpkit_health[19]; }
static int xpkit_nd_atlas_needed(void) { return !xpkit_health[10]; }

/* ---- per-frame presentation polls ---- */
/* Upload path: grant at most XPKIT_UPLOADS_PER_FRAME PFD/ND uploads (XPKIT_UPLOAD_BYTES_PER_FRAME) to
 * the views whose frame has waited longest. A grant its device did not draw sends the view to the back. */
static void xpkit_grant_uploads(void) {
    int order[4], n=0, granted=0; size_t bytes=0;
    ++xpkit_frame_no;
    for(int v=0;v<4;++v) {
        unsigned seq=xpkit_shared?atomic_load(&xpkit_shared->frame_seq[v]):0;
        int pending=seq && !(seq&1) && seq!=xpkit_uploaded[v] &&
                    atomic_load(&xpkit_shared->surface_front[v])>=XPKIT_SURFACES;
        if(!pending) { xpkit_waiting_since[v]=0; continue; }
        if(!xpkit_waiting_since[v] || (xpkit_grant[v]==xpkit_frame_no-1 && !xpkit_grant_used[v]))
            xpkit_waiting_since[v]=xpkit_frame_no;
        int k=n++;
        while(k>0 && xpkit_waiting_since[order[k-1]]>xpkit_waiting_since[v]) { order[k]=order[k-1]; --k; }
        order[k]=v;
    }
    for(int k=0;k<n;++k) {
        int v=order[k]; size_t size=xpkit_view_bytes(v);
        if(granted>=XPKIT_UPLOADS_PER_FRAME || (granted && bytes+size>XPKIT_UPLOAD_BYTES_PER_FRAME)) break;
        xpkit_grant[v]=xpkit_frame_no; xpkit_grant_used[v]=0; bytes+=size; ++granted;
    }
}
/* MFD/SD/EFB: request one device redraw per new frame (retried every 250 ms while not drawn). */
static int xpkit_frame_unclaimed(int view,double now) {
    unsigned seq=xpkit_shared?atomic_load(&xpkit_shared->frame_seq[view]):0;
    if(!seq || (seq&1) || seq==xpkit_uploaded[view]) return 0;
    if(seq==xpkit_requested[view] && now-xpkit_requested_at[view]<.25) return 0;
    xpkit_requested[view]=seq; xpkit_requested_at[view]=now;
    return 1;
}
static int xpkit_view_live(int view,double now) {
    A380XXpKitShared *s=xpkit_shared;
    return s && atomic_load(&s->ready[view]) && atomic_load(&s->frame_seq[view])>0 && now-s->frame_time[view]<1.0;
}
static void xpkit_present_poll(double now) {
    int web=xpkit_web_possible();
    if(xpkit_shared) xpkit_health[3]=(int)atomic_load_explicit(&xpkit_shared->errors,memory_order_acquire);
    for(int i=0;i<2;++i) xpkit_health[i+1]=xpkit_health[0] && xpkit_view_live(i,now);
    for(int i=0;i<2;++i) xpkit_health[i+6]=xpkit_health[0] && xpkit_view_live(i+2,now);
    for(int i=0;i<2;++i) {
        xpkit_health[i+13]=xpkit_health[0] && xpkit_view_live(i+4,now);
        /* mfda_update requests page/cursor redraws; only a newly completed
         * texture needs another request here (not an unconditional per-frame draw). */
        if(mfda_dev[i].handle && mfda_av.loaded && xpkit_frame_unclaimed(i+4,now))
            mfda_av.needs_drawing(mfda_dev[i].handle);
    }
    for(int i=0;i<2;++i) {
        /* power value also changes with the CDS boot / self-test state (device redraw) */
        XpKitNd *d=&xpkit_nd[i]; int live=xpkit_health[i+6],powered=xpkit_nd_on(d)?8:a380x_power_du_state(xpkit_nd_du(d))+1;
        unsigned seq=xpkit_shared?atomic_load(&xpkit_shared->frame_seq[i+2]):0;
        /* Native ND (no web renderer): drawOnDemand device, animated every frame. */
        if(d->device && mfda_av.loaded &&
           (xpkit_nd_frame_pending(d,seq,live,powered) || (!web && powered==8)))
            mfda_av.needs_drawing(d->device);
        d->draw_live=live;d->draw_powered=powered;
    }
    xpkit_health[17]=xpkit_view_live(6,now);
    if(sda.device&&mfda_av.loaded&&xpkit_frame_unclaimed(6,now))mfda_av.needs_drawing(sda.device);
    xpkit_efb_poll(now);
    xpkit_pfd_poll();
    xpkit_grant_uploads();
    if(!xpkit_logged && xpkit_health[1] && xpkit_health[2]) {
        char line[200];
        snprintf(line,sizeof(line),"A380X xp-kit: BOTH FlyByWire library PFDs active (%dx%d cockpit devices, %s).\n",
                 xpkit_view_width(0),xpkit_view_height(0),xpkit_zero_copy>0?"zero-copy frames":"texture uploads");
        XPLMDebugString(line); xpkit_logged=1;
    }
}
static void xpkit_publish_state(double now) {
    static XpKitJson j; j.size=0;j.failed=0; float bus[1]={0};
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
    xpkit_taws_words(&j);   /* a380x_xpkit_terrain.inc: PULL UP / TERRAIN on the FBW PFD */
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
    {   /* FMGC VNAV indications (FBW PFD L:vars): linear deviation, DES speed margins, T/D REACHED. */
        const A380XFmgcOutputs *v=a380x_fmgc_outputs(&fmsa_fmgc);
        int des=guidance.vertical_mode==A380X_B1_DES, path=des&&guidance_inputs.managed_path_valid;
        int margins=path&&guidance.des_submode==0&&guidance.speed_managed;
        xpkit_number(&j,"L:A32NX_PFD_LINEAR_DEVIATION_ACTIVE",path);
        xpkit_number(&j,"L:A32NX_PFD_TARGET_ALTITUDE",path?guidance_inputs.managed_path_altitude_ft:0);
        xpkit_number(&j,"L:A32NX_PFD_VERTICAL_PROFILE_LATCHED",0);
        xpkit_number(&j,"L:A32NX_PFD_SHOW_SPEED_MARGINS",margins);
        xpkit_number(&j,"L:A32NX_PFD_UPPER_SPEED_MARGIN",margins?guidance.speed_target_knots+10:0);
        xpkit_number(&j,"L:A32NX_PFD_LOWER_SPEED_MARGIN",margins?guidance.speed_target_knots-10:0);
        xpkit_number(&j,"L:A32NX_PFD_MSG_TD_REACHED",a380x_fms_phase(&fmsa_fms)==A380X_FMS_PHASE_CRUISE&&v->vnav_valid
            &&isfinite(v->tod_dist_nm)&&!v->before_tod&&!des&&guidance.vertical_mode!=A380X_B1_OP_DES);
        /* FBW sign (PFD/ND show "R" for a negative value): positive = aircraft left of the path. */
        xpkit_number(&j,"L:A32NX_FG_CROSS_TRACK_ERROR",v->lnav_valid?v->fbw_xtk_nm:0);
    }
    xpkit_append(&j,"\"_end\":0},\"nav\":");
    xpkit_nd_json(&j);
    xpkit_append(&j,",\"terrain\":");
    xpkit_terrain_json(&j);
    xpkit_append(&j,"}");
    if(j.failed) { xpkit_health[3]++; return; }
    uint32_t seq=atomic_load_explicit(&xpkit_shared->state_seq,memory_order_relaxed);
    atomic_store_explicit(&xpkit_shared->state_seq,seq+1,memory_order_release);
    memcpy(xpkit_shared->state,j.data,j.size+1); xpkit_shared->state_size=(uint32_t)j.size;
    xpkit_shared->state_time=now;
    atomic_store_explicit(&xpkit_shared->state_seq,seq+2,memory_order_release);
}
/* Every flight loop: supervision, ND cursor sampling, publication (PFD/ND state up to 75 Hz, the
 * MFD/SD/EFB publishers every 45 ms) and the presentation polls - also without a helper, so the
 * on-demand devices are still redrawn (native PFD/ND every frame). */
static void xpkit_update(void) {
    double now=xpkit_now();
    xpkit_supervise(now);
    xpkit_nd_poll();
    if(xpkit_shared && xpkit_pid) {
        if(now-xpkit_last_slow>=XPKIT_SLOW_PERIOD) {
            xpkit_last_slow=now;
            xpkit_mfd_publish();
            xpkit_sd_publish();
            xpkit_efb_drain();
            xpkit_efb_publish();
        }
        /* 0.5 ms tolerance: a 75 Hz simulator frame publishes every frame despite timer jitter. */
        if(now-xpkit_last_state>=XPKIT_STATE_PERIOD-.0005) { xpkit_last_state=now; xpkit_publish_state(now); }
    }
    xpkit_present_poll(now);
}
#else
static void xpkit_init(void) {}
static void xpkit_start(void) {}
static void xpkit_update(void) {}
static void xpkit_stop(void) {}
static void xpkit_shutdown(void) {}
static int xpkit_draw(int side,float x,float y) { (void)side;(void)x;(void)y;return 0; }
static int xpkit_draw_nd(int side,float x,float y) { (void)side;(void)x;(void)y;return 0; }
static int xpkit_draw_mfd(int side,float x,float y,float scale) { (void)side;(void)x;(void)y;(void)scale;return 0; }
static int xpkit_draw_sd(void) { return 0; }
#endif
