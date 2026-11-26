/* SPDX-License-Identifier: GPL-3.0-or-later
 * Engine pylon wobble (a380x_eng_wobble.h) for the exterior: publishes
 * a380x/xp12/eng/wobble_pitch_deg[4] and a380x/xp12/eng/wobble_yaw_deg[4] (degrees, read-only, engines 1..4;
 * pitch + = intake up, yaw + = intake right). The engine OBJs rotate about each pylon's wing attachment by these,
 * inside the wing flex bone the engine rides on.
 * Drive: body load factor + the wing flex node the pylon hangs from (inboard node 1, outboard node 2 of
 * a380x_wing_flex.h) + roll acceleration x span for the vertical; side load factor for the lateral.
 * shortcut: no fore-aft term (thrust changes, braking) and no tyre-deflection term on the ground; taxi bumps
 * and touchdown come only through the load factor X-Plane computes. Add them if ground wobble looks too calm.
 * Needs a380x_wing_flex.inc included first (reads its node speeds). Unity-build include of a380x_avionics.c.
 */
#include "a380x_eng_wobble.h"

enum { EW_G, EW_G_SIDE, EW_P, EW_PAUSED, EW_REFS };
static struct {
    A380XEngWobble model;
    XPLMDataRef refs[EW_REFS], out_pitch, out_yaw;
    double node_v[A380X_EW_ENGINES], p_prev, p_dot;
    int primed;
} ew;

static const int ew_wing[A380X_EW_ENGINES] = {0, 0, 1, 1}, ew_node[A380X_EW_ENGINES] = {2, 1, 1, 2};
static const double ew_span_m[A380X_EW_ENGINES] = {-25.7, -14.6, 14.6, 25.7};   /* pylon y, + right */

static int ew_get(const float *src, float *values, int offset, int max)
{
    if (!values) return A380X_EW_ENGINES;
    int n = 0;
    for (int i = offset; i < A380X_EW_ENGINES && n < max; ++i) values[n++] = src[i];
    return n;
}
static int ew_get_pitch(void *ref, float *values, int offset, int max) { (void)ref; return ew_get(ew.model.pitch_deg, values, offset, max); }
static int ew_get_yaw(void *ref, float *values, int offset, int max) { (void)ref; return ew_get(ew.model.yaw_deg, values, offset, max); }

static void ew_init(void)
{
    static const char *const names[EW_REFS] = {"sim/flightmodel/forces/g_nrml", "sim/flightmodel/forces/g_side",
                                               "sim/flightmodel/position/P", "sim/time/paused"};
    memset(&ew, 0, sizeof(ew));
    for (int r = 0; r < EW_REFS; ++r) ew.refs[r] = XPLMFindDataRef(names[r]);
    ew.out_pitch = XPLMRegisterDataAccessor("a380x/xp12/eng/wobble_pitch_deg", xplmType_FloatArray, 0, NULL, NULL,
                                            NULL, NULL, NULL, NULL, NULL, NULL, ew_get_pitch, NULL, NULL, NULL, NULL, NULL);
    ew.out_yaw = XPLMRegisterDataAccessor("a380x/xp12/eng/wobble_yaw_deg", xplmType_FloatArray, 0, NULL, NULL,
                                          NULL, NULL, NULL, NULL, NULL, NULL, ew_get_yaw, NULL, NULL, NULL, NULL, NULL);
}

static void ew_update(float dt)
{
    double vert[A380X_EW_ENGINES], lat[A380X_EW_ENGINES];
    for (int r = 0; r < EW_REFS; ++r) if (!ew.refs[r]) return;
    if (!(dt > 0.f) || XPLMGetDatai(ew.refs[EW_PAUSED])) return;
    double nz = XPLMGetDataf(ew.refs[EW_G]), ny = XPLMGetDataf(ew.refs[EW_G_SIDE]);
    double p = XPLMGetDataf(ew.refs[EW_P]) * M_PI / 180.;
    if (!ew.primed) {                                       /* first frame: no derivative yet */
        for (int i = 0; i < A380X_EW_ENGINES; ++i) ew.node_v[i] = wf.model.wing[ew_wing[i]].n[ew_node[i]].speed;
        ew.p_prev = p;
        ew.primed = 1;
        return;
    }
    ew.p_dot = a380x_wf_lp(ew.p_dot, (p - ew.p_prev) / dt, dt, 0.05);   /* frame-time jitter makes raw d/dt spiky */
    ew.p_prev = p;
    for (int i = 0; i < A380X_EW_ENGINES; ++i) {
        double v = wf.model.wing[ew_wing[i]].n[ew_node[i]].speed;
        double wing_accel = (v - ew.node_v[i]) / dt;
        ew.node_v[i] = v;
        /* X-Plane P is + right wing down, so a right pylon accelerates down with P_dot */
        vert[i] = (nz - 1.) * 9.8 + wing_accel - ew.p_dot * ew_span_m[i];
        lat[i] = ny * 9.8;
    }
    a380x_eng_wobble_update(&ew.model, vert, lat, dt);
}

static void ew_shutdown(void)
{
    if (ew.out_pitch) XPLMUnregisterDataAccessor(ew.out_pitch);
    if (ew.out_yaw) XPLMUnregisterDataAccessor(ew.out_yaw);
    memset(&ew, 0, sizeof(ew));
}
