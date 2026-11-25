/* SPDX-License-Identifier: GPL-3.0-or-later
 * Exterior fan rotation (a380x_eng_fan.h): takes sim/flightmodel2/engines/prop_disc/override for the four
 * engines and writes prop_rotation_angle_deg, which a380x_fans.obj already animates. Axial airflow into the
 * inlets = ground speed along the heading + the headwind component of the wind at the aircraft.
 * Unity-build include of a380x_avionics.c; fan_release() hands the angles back to X-Plane.
 */
#include "a380x_eng_fan.h"

enum { FAN_N1, FAN_ANGLE, FAN_OVERRIDE, FAN_WIND_DIR, FAN_WIND_MS, FAN_PSI, FAN_VX, FAN_VZ, FAN_PAUSED, FAN_REFS };
static struct { A380XFan fan[4]; XPLMDataRef refs[FAN_REFS]; int overridden; } fan;

static void fan_release(void)
{
    int off[4] = {0};
    if (fan.overridden && fan.refs[FAN_OVERRIDE]) XPLMSetDatavi(fan.refs[FAN_OVERRIDE], off, 0, 4);
    fan.overridden = 0;
}

static void fan_init(void)
{
    static const char *const names[FAN_REFS] = {
        "sim/flightmodel/engine/ENGN_N1_", "sim/flightmodel2/engines/prop_rotation_angle_deg",
        "sim/flightmodel2/engines/prop_disc/override", "sim/weather/aircraft/wind_now_direction_degt",
        "sim/weather/aircraft/wind_now_speed_msc", "sim/flightmodel/position/psi",
        "sim/flightmodel/position/local_vx", "sim/flightmodel/position/local_vz", "sim/time/paused"};
    memset(&fan, 0, sizeof(fan));
    for (int r = 0; r < FAN_REFS; ++r) fan.refs[r] = XPLMFindDataRef(names[r]);
    if (!fan.refs[FAN_N1] || !fan.refs[FAN_ANGLE] || !fan.refs[FAN_OVERRIDE])
        XPLMDebugString("A380X ENG FAN: prop datarefs missing; the fans stay still\n");
}

static void fan_update(float dt)
{
    float n1[4] = {0}, angle[4], air = 0.0f;
    int on[4] = {1, 1, 1, 1};
    if (!fan.refs[FAN_N1] || !fan.refs[FAN_ANGLE] || !fan.refs[FAN_OVERRIDE]) return;
    if (!plugin_enabled) { fan_release(); return; }
    if (!fan.overridden) { XPLMSetDatavi(fan.refs[FAN_OVERRIDE], on, 0, 4); fan.overridden = 1; }
    if (fan.refs[FAN_PAUSED] && XPLMGetDatai(fan.refs[FAN_PAUSED])) return;
    XPLMGetDatavf(fan.refs[FAN_N1], n1, 0, 4);
    if (fan.refs[FAN_PSI] && fan.refs[FAN_VX] && fan.refs[FAN_VZ]) {
        float psi = XPLMGetDataf(fan.refs[FAN_PSI]) * (float)(M_PI / 180.0);
        air = XPLMGetDataf(fan.refs[FAN_VX]) * sinf(psi) - XPLMGetDataf(fan.refs[FAN_VZ]) * cosf(psi);   /* OGL: -z north */
        if (fan.refs[FAN_WIND_DIR] && fan.refs[FAN_WIND_MS])
            air += XPLMGetDataf(fan.refs[FAN_WIND_MS]) * cosf(XPLMGetDataf(fan.refs[FAN_WIND_DIR]) * (float)(M_PI / 180.0) - psi);
    }
    for (int i = 0; i < 4; ++i) { a380x_fan_update(&fan.fan[i], n1[i], air, dt); angle[i] = fan.fan[i].angle_deg; }
    XPLMSetDatavf(fan.refs[FAN_ANGLE], angle, 0, 4);
}
