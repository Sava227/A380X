/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Exterior control-surface motion: a C port of the two FlyWithLua visual drivers
 *   FlyWithLua/Modules/a380x_aileron_dance.lua     + Scripts/A380X_XP12_Ailerons.lua
 *   FlyWithLua/Modules/a380x_hydraulic_surfaces.lua + Scripts/A380X_XP12_Hydraulics.lua
 * for the A380XAileronDatarefs aircraft plugin (docs/HYDRAULIC_SURFACES.md, docs/AILERON_DANCE.md).
 *
 * VISUAL ONLY. Nothing here reads or writes X-Plane: the plugin reads the simulator inputs into an
 * A380XSurfaceSources and publishes A380XSurfaceDriver.pub[] through its own early-registered Float
 * datarefs. No flight control, override or flight-model value is ever written.
 *
 * Equivalence contract (tests/surface_golden_lua.lua + test_surface_golden.c): every expression is
 * the Lua one in the same order, in double precision, with Lua's math.min/math.max/clamp/% semantics;
 * the published values are the datarefs' float storage (the plugin's clamp-and-keep-finite write).
 * The plugin and the tests are compiled with -ffp-contract=off so no multiply-add is fused.
 */
#ifndef A380X_SURFACE_MOTION_H
#define A380X_SURFACE_MOTION_H

/* Published value order = the plugin's dataref order (aileron_datarefs.c). */
enum {
    A380X_PUB_VISUAL = 0,      /* 6 x a380x/xp12/visual/aileron_{l,r}_{inboard,midboard,outboard}_ratio */
    A380X_PUB_HYD = 6,         /* 16 x a380x/xp12/hydraulics/... in the hydraulic module's KEYS order */
    A380X_PUB_COUNT = 22,
    A380X_HYD_KEYS = 16
};

/* ---- a380x_aileron_dance.lua ---- */
#define A380X_DANCE_DELAY_SECONDS 0.350
#define A380X_DANCE_MAX_STEP_SECONDS 0.010
#define A380X_DANCE_MAX_DT_SECONDS 1.0
#define A380X_DANCE_HISTORY 128   /* the Lua history never exceeds ~38 entries */

typedef struct { double time, value; } A380XDanceSample;
typedef struct {
    int has_roll;      /* input.roll ~= nil */
    double roll;
    int grounded;      /* input.grounded == true */
    int paused;        /* input.paused == true */
} A380XDanceInput;
typedef struct {
    int initialized;
    double time, q, next_sample;
    A380XDanceSample history[A380X_DANCE_HISTORY];
    int history_first, history_count, history_peak;
    double degrees[6], outputs[6];
} A380XDanceState;

void a380x_dance_reset(A380XDanceState *s, const A380XDanceInput *in);
void a380x_dance_new(A380XDanceState *s);
void a380x_dance_update(A380XDanceState *s, const A380XDanceInput *in, double dt);

/* ---- a380x_hydraulic_surfaces.lua ---- */
#define A380X_HYD_PRESSURE_RELEASE_PSI 450.0
#define A380X_HYD_PRESSURE_RESTORE_PSI 900.0

typedef struct {
    int grounded, paused;           /* booleans (the driver only builds them from 0 / 1) */
    double ailerons[6], spoilers[8], pressures[3];
    double pitch, rudder, heading, wind_direction, wind_speed, airspeed;
} A380XHydInput;
typedef struct {
    int powered;
    double out[A380X_HYD_KEYS];
} A380XHydState;

int a380x_hyd_valid(const A380XHydInput *in);
void a380x_hyd_new(A380XHydState *s, const A380XHydInput *in);
void a380x_hyd_update(A380XHydState *s, const A380XHydInput *in, double dt);

/* ---- The two FlyWithLua driver scripts, in their FlyWithLua order (Ailerons, then Hydraulics) ---- */
typedef struct {
    float roll, pitch, yaw;                 /* sim/flightmodel2/controls/{roll,pitch,heading}_ratio */
    int grounded, paused;                   /* sim/flightmodel/failures/onground_any, sim/time/paused */
    float dt;                               /* sim/operation/misc/frame_rate_period */
    float heading;                          /* sim/flightmodel/position/psi */
    float wind_direction, wind_speed;       /* sim/weather/aircraft/wind_now_{direction_degt,speed_msc} */
    float airspeed;                         /* sim/flightmodel/position/true_airspeed */
    float pressures[3];                     /* sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1..3 */
    float speedbrake1[2];                   /* sim/flightmodel2/wing/speedbrake1_deg[0..1] */
    float spoiler1[4];                      /* sim/flightmodel2/wing/spoiler1_deg[0..3] */
    float spoiler2[2];                      /* sim/flightmodel2/wing/spoiler2_deg[4..5] */
} A380XSurfaceSources;

typedef struct {
    float pub[A380X_PUB_COUNT];    /* the 22 published dataref values (the plugin's Float storage) */
    int loaded;                    /* 0: the next step is a script load (aircraft load / reload) */
    /* Ailerons script */
    A380XDanceState dance;
    int dance_enabled, dance_active, dance_warned, dance_reported_nonzero;
    /* Hydraulics script */
    A380XHydState hyd;
    int hyd_enabled, hyd_active, hyd_warned;
    unsigned long frames;
} A380XSurfaceDriver;

/* The plugin's write accessor: finite values are clamped and stored as float, others are ignored. */
void a380x_surface_store(float *slot, double value, float minimum, float maximum);
float a380x_surface_minimum(int index);
float a380x_surface_maximum(int index);
/* Neutral, both drivers enabled, a script load pending. pub[] is left as it is (the datarefs keep
 * their value across a FlyWithLua reload, too). */
void a380x_surface_driver_reset(A380XSurfaceDriver *d);
/* One FlyWithLua frame: the first call after a reset is the script load (Ailerons load, then the
 * Hydraulics load with its immediate update), every later call is a frame (Ailerons, then Hydraulics). */
void a380x_surface_driver_step(A380XSurfaceDriver *d, const A380XSurfaceSources *src);

#endif
