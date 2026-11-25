/* SPDX-License-Identifier: GPL-3.0-or-later
 * Wing flex (a380x_wing_flex.h) for the exterior: publishes FBW's eight WING_FLEX bend angles as
 * a380x/xp12/wing/flex_<left|right>_<inboard|inboard_mid|outboard_mid|outboard> (degrees, read-only).
 * config/animation_datarefs.json maps FBW's four flex clips per wing onto them with A380_WINGS.xml's
 * gain/offset, so the wings bend once the wing OBJs are re-exported from FBW's glTF.
 * Registered in XPluginStart (before the aircraft OBJs bind them). Unity-build include of a380x_avionics.c.
 */
#include "a380x_wing_flex.h"

enum { WF_MASS, WF_GEAR_N, WF_G, WF_THETA, WF_PHI, WF_SPEEDBRAKE, WF_ROLL, WF_FLAPS, WF_FUEL, WF_PAUSED, WF_REFS };
static struct { A380XWingFlex model; XPLMDataRef refs[WF_REFS], out[8]; } wf;

static float wf_get(void *ref) { return *(const float *)ref; }

static void wf_init(void)
{
    static const char *const sides[2] = {"left", "right"};
    static const char *const sections[4] = {"inboard", "inboard_mid", "outboard_mid", "outboard"};
    static const char *const names[WF_REFS] = {
        "sim/flightmodel/weight/m_total", "sim/flightmodel/forces/fnrml_gear", "sim/flightmodel/forces/g_nrml",
        "sim/flightmodel/position/theta", "sim/flightmodel/position/phi", "sim/flightmodel2/controls/speedbrake_ratio",
        "sim/flightmodel2/controls/roll_ratio", "sim/flightmodel2/controls/flap1_deploy_ratio",
        "sim/flightmodel/weight/m_fuel", "sim/time/paused"};
    char name[64];
    memset(&wf, 0, sizeof(wf));
    for (int r = 0; r < WF_REFS; ++r) wf.refs[r] = XPLMFindDataRef(names[r]);
    for (int w = 0; w < 2; ++w)
        for (int i = 0; i < 4; ++i) {
            snprintf(name, sizeof(name), "a380x/xp12/wing/flex_%s_%s", sides[w], sections[i]);
            wf.out[w * 4 + i] = XPLMRegisterDataAccessor(name, xplmType_Float, 0, NULL, NULL, wf_get, NULL,
                                                         NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                                         &wf.model.angle_deg[w][i], NULL);
        }
}

static void wf_update(float dt)
{
    A380XWingFlexIn in;
    float fuel[9] = {0};
    for (int r = 0; r < WF_REFS; ++r) if (!wf.refs[r]) return;
    if (XPLMGetDatai(wf.refs[WF_PAUSED])) return;
    memset(&in, 0, sizeof(in));
    in.mass_kg = XPLMGetDataf(wf.refs[WF_MASS]);
    in.wow_kg = fmax(0., XPLMGetDataf(wf.refs[WF_GEAR_N]) / 9.8);
    in.accel_no_grav = (XPLMGetDataf(wf.refs[WF_G]) - 1.) * 9.8;
    in.pitch_deg = XPLMGetDataf(wf.refs[WF_THETA]);
    in.roll_deg = XPLMGetDataf(wf.refs[WF_PHI]);
    in.spoilers = XPLMGetDataf(wf.refs[WF_SPEEDBRAKE]);
    in.roll = XPLMGetDataf(wf.refs[WF_ROLL]);
    in.flaps = XPLMGetDataf(wf.refs[WF_FLAPS]);
    XPLMGetDatavf(wf.refs[WF_FUEL], fuel, 0, 9);
    /* .acf tanks: 1 No.1 main, 2 No.2 main, 3 No.3 main, 4 No.4 main, 5 No.2 reserve, 6 No.3 reserve.
     * shortcut: inner main + its reserve on node 1, outer main on node 2 (FBW also loads nodes 2/3 from its
     * mid and outer tanks, which this .acf does not have); revisit if the .acf fuel system is split further. */
    in.fuel_kg[0][1] = fuel[2] + fuel[5]; in.fuel_kg[0][2] = fuel[1];
    in.fuel_kg[1][1] = fuel[3] + fuel[6]; in.fuel_kg[1][2] = fuel[4];
    for (int w = 0; w < 2; ++w) for (int i = 0; i < 5; ++i) if (!isfinite(in.fuel_kg[w][i]) || in.fuel_kg[w][i] < 0.) in.fuel_kg[w][i] = 0.;
    a380x_wing_flex_update(&wf.model, &in, dt);
}

static void wf_shutdown(void)
{
    for (int k = 0; k < 8; ++k) if (wf.out[k]) XPLMUnregisterDataAccessor(wf.out[k]);
    memset(&wf, 0, sizeof(wf));
}
