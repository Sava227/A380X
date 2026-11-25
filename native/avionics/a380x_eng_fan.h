/* SPDX-License-Identifier: GPL-3.0-or-later
 * Engine fan rotation for the exterior model (objects/exterior/wings/a380x_fans.obj spins each fan on
 * sim/flightmodel2/engines/prop_rotation_angle_deg[i]). The .acf has no props (acf/_num_prop 0), so X-Plane
 * never turns that angle; the adapter (a380x_eng_fan.inc) overrides it with this model. Pure C, no XPLM.
 *
 *   Driven:     fan speed = N1 x 2900 rpm (Trent 970 / GP7270 N1 100 %), so start and spool-up follow N1.
 *   Windmill:   air through the nacelle turns a fan the engine is not driving, forwards with a headwind,
 *               backwards with a tailwind. Speed follows the axial airflow (ground speed + headwind) with
 *               fan inertia (4 s lag) and a 2 m/s breakaway below which bearing drag holds the fan still.
 *               4.5 rpm per m/s puts in-flight windmilling near 20 % N1 at 250 kt (port estimate).
 */
#ifndef A380X_ENG_FAN_H
#define A380X_ENG_FAN_H
#include <math.h>

#define A380X_FAN_RPM_100_N1 2900.0f
#define A380X_FAN_WINDMILL_RPM_PER_MS 4.5f
#define A380X_FAN_WINDMILL_BREAKAWAY_MS 2.0f
#define A380X_FAN_WINDMILL_LAG_S 4.0f

typedef struct { float rpm, angle_deg; } A380XFan;

/* Advances one fan by dt seconds; n1_pct from X-Plane, axial_air_ms = air speed into the inlet (signed). */
static inline void a380x_fan_update(A380XFan *f, float n1_pct, float axial_air_ms, float dt)
{
    float wind_rpm = 0.0f, driven;
    if (!isfinite(dt) || dt <= 0.0f) return;
    dt = fminf(dt, 0.25f);
    if (isfinite(axial_air_ms) && fabsf(axial_air_ms) > A380X_FAN_WINDMILL_BREAKAWAY_MS)
        wind_rpm = A380X_FAN_WINDMILL_RPM_PER_MS * axial_air_ms;
    driven = isfinite(n1_pct) && n1_pct > 0.0f ? n1_pct * (A380X_FAN_RPM_100_N1 / 100.0f) : 0.0f;
    f->rpm += (wind_rpm - f->rpm) * fminf(dt / A380X_FAN_WINDMILL_LAG_S, 1.0f);   /* inertia: coasts, windmills */
    if (driven > 0.0f && driven > f->rpm) f->rpm = driven;                                        /* the engine drives it */
    f->angle_deg = fmodf(f->angle_deg + f->rpm * 6.0f * dt, 360.0f);
    if (f->angle_deg < 0.0f) f->angle_deg += 360.0f;
}

#endif
