/* Minimal mock of the X-Plane SDK for the offline A380XDoors tests. */
#include <stdio.h>
#include <string.h>
#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
float m_door[20]; int m_armed[16]; float m_steer[10]; int m_gground[10]; float m_tx[10], m_ty[10], m_tz[10], m_dep[10], m_defl[10], m_agl = 0, m_theta = 0, m_phi = 0; XPLMCommandCallback_f m_stow;
float m_order[8], m_gs = 0, m_sp2[6], m_sbr = 0, m_sb1[6], m_hyd = 3000; int m_ground = 1, m_have_orders = 1; double m_t = 0;
static float cycle[20];
typedef struct { const char *name; int kind; int idx; } R;
static R refs[40]; static int nrefs;
static R *mk(const char *n, int kind, int idx) { refs[nrefs] = (R){n, kind, idx}; return &refs[nrefs++]; }
XPLMDataRef XPLMFindDataRef(const char *n) {
    int k;
    if (!strcmp(n, "sim/flightmodel2/misc/door_cycle_time")) return mk(n, 0, 0);
    if (sscanf(n, "a380x/xp12/speedbrake/spoiler_%d_deg", &k) == 1) return m_have_orders ? mk(n, 1, k - 1) : NULL;
    if (!strcmp(n, "a380x/xp12/speedbrake/ground_spoiler_command_deg")) return m_have_orders ? mk(n, 2, 0) : NULL;
    if (!strcmp(n, "sim/flightmodel2/wing/spoiler2_deg")) return mk(n, 3, 0);
    if (!strcmp(n, "sim/flightmodel2/controls/speedbrake_ratio")) return mk(n, 4, 0);
    if (!strcmp(n, "sim/flightmodel2/wing/speedbrake1_deg")) return mk(n, 5, 0);
    if (!strcmp(n, "sim/flightmodel/failures/onground_any")) return mk(n, 6, 0);
    if (!strncmp(n, "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_", 54)) return mk(n, 7, 0);
    if (!strcmp(n, "sim/flightmodel2/misc/door_open_ratio")) return mk(n, 8, 0);
    if (!strcmp(n, "a380x/xp12/efb/slide_armed")) return mk(n, 9, 0);
    if (!strcmp(n, "sim/flightmodel2/gear/tire_steer_actual_deg")) return mk(n, 10, 0);
    if (!strcmp(n, "sim/flightmodel2/gear/on_ground")) return mk(n, 11, 0);
    if (!strcmp(n, "sim/flightmodel/parts/tire_x_no_deflection")) return mk(n, 12, 0);
    if (!strcmp(n, "sim/flightmodel/parts/tire_y_no_deflection")) return mk(n, 13, 0);
    if (!strcmp(n, "sim/flightmodel/parts/tire_z_no_deflection")) return mk(n, 14, 0);
    if (!strcmp(n, "sim/flightmodel/position/y_agl")) return mk(n, 15, 0);
    if (!strcmp(n, "sim/flightmodel/position/theta")) return mk(n, 16, 0);
    if (!strcmp(n, "sim/flightmodel/position/phi")) return mk(n, 17, 0);
    if (!strcmp(n, "sim/flightmodel2/gear/deploy_ratio")) return mk(n, 18, 0);
    if (!strcmp(n, "sim/flightmodel2/gear/tire_vertical_deflection_mtr")) return mk(n, 19, 0);
    return NULL;
}
float XPLMGetDataf(XPLMDataRef r) { R *x = r; switch (x->kind) { case 1: return m_order[x->idx]; case 2: return m_gs; case 4: return m_sbr; case 7: return m_hyd; case 15: return m_agl; case 16: return m_theta; case 17: return m_phi; } return 0; }
int XPLMGetDatavi(XPLMDataRef r, int *v, int o, int c) { R *x = r; if (x->kind == 11) { if (!v) return 10; memcpy(v, m_gground + o, c * sizeof(int)); return c; } if (!v) return 16; memcpy(v, m_armed + o, c * sizeof(int)); return c; }
int XPLMGetDatai(XPLMDataRef r) { R *x = r; return x->kind == 6 ? m_ground : 0; }
int XPLMGetDatavf(XPLMDataRef r, float *v, int o, int c) { R *x = r; float *src = x->kind == 12 ? m_tx : x->kind == 13 ? m_ty : x->kind == 14 ? m_tz : x->kind == 18 ? m_dep : x->kind == 19 ? m_defl : x->kind == 10 ? m_steer : x->kind == 8 ? m_door : x->kind == 3 ? m_sp2 : x->kind == 5 ? m_sb1 : cycle; if (!v) return 20; memcpy(v, src + o, c * sizeof(float)); return c; }
void XPLMSetDatavf(XPLMDataRef r, float *v, int o, int c) { (void)r; memcpy(cycle + o, v, c * sizeof(float)); }
float XPLMGetElapsedTime(void) { return (float)m_t; }
XPLMPluginID XPLMGetMyID(void) { return 1; }
void XPLMGetPluginInfo(XPLMPluginID id, char *n, char *p, char *s, char *d) { (void)id; (void)n; (void)s; (void)d; strcpy(p, "test-plugin/A380XDoors/mac_x64/A380XDoors.xpl"); }
XPLMFlightLoop_f loop_cb;
void XPLMRegisterFlightLoopCallback(XPLMFlightLoop_f f, float i, void *r) { (void)i; (void)r; loop_cb = f; }
void XPLMUnregisterFlightLoopCallback(XPLMFlightLoop_f f, void *r) { (void)f; (void)r; }
void XPLMDebugString(const char *s) { fputs(s, stdout); }
void XPLMEnableFeature(const char *f, int e) { (void)f; (void)e; }
typedef struct { const char *name; XPLMGetDatavf_f gvf; XPLMGetDataf_f gf; void *ref; } Acc;
Acc acc[16]; int nacc;
XPLMDataRef XPLMRegisterDataAccessor(const char *n, XPLMDataTypeID t, int w, XPLMGetDatai_f a, XPLMSetDatai_f b, XPLMGetDataf_f c, XPLMSetDataf_f d, XPLMGetDatad_f e, XPLMSetDatad_f f, XPLMGetDatavi_f g, XPLMSetDatavi_f h, XPLMGetDatavf_f gvf, XPLMSetDatavf_f svf, XPLMGetDatab_f i, XPLMSetDatab_f j, void *rr, void *wr)
{ (void)t;(void)w;(void)a;(void)b;(void)d;(void)e;(void)f;(void)g;(void)h;(void)svf;(void)i;(void)j;(void)wr; acc[nacc] = (Acc){n, gvf, c, rr}; return &acc[nacc++]; }
void XPLMUnregisterDataAccessor(XPLMDataRef r) { (void)r; }
XPLMCommandRef XPLMCreateCommand(const char *n, const char *d) { (void)d; return (XPLMCommandRef)n; }
void XPLMRegisterCommandHandler(XPLMCommandRef c, XPLMCommandCallback_f cb, int b, void *rc) { if (!strcmp((const char *)c, "a380x/xp12/doors/slides_stow_all")) m_stow = cb; (void)c; (void)b; (void)rc; }
void XPLMUnregisterCommandHandler(XPLMCommandRef c, XPLMCommandCallback_f cb, int b, void *rc) { (void)c; (void)cb; (void)b; (void)rc; }
