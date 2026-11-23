/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * C port of the FlyWithLua exterior surface drivers (see a380x_surface_motion.h). Each function names
 * the Lua it reproduces; keep the two in step (tests/surface_golden_lua.lua compares them frame by
 * frame). The aileron distribution is derived from FlyByWire Simulations' GPL-3.0 A380X source,
 * revision 2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2 (A380PrimComputerFctl.cpp:2435-2473, :2495-2513,
 * :2625-2713), as documented in a380x_aileron_dance.lua.
 */
#include "a380x_surface_motion.h"

#include <math.h>
#include <string.h>

/* Lua 5.4 math.min(a, b) / math.max(a, b): the first argument unless the second compares smaller /
 * larger (so a NaN never wins); clamp(x, lo, hi) = math.max(lo, math.min(hi, x)). */
static double lmin(double a, double b) { return b < a ? b : a; }
static double lmax(double a, double b) { return a < b ? b : a; }
static double lclamp(double x, double lo, double hi) { return lmax(lo, lmin(hi, x)); }
static int lfinite(double x) { return x == x && x > -HUGE_VAL && x < HUGE_VAL; }
/* Lua 5.4 float a % b (luai_nummod). */
static double lmod(double a, double b)
{
    double m = fmod(a, b);
    if ((m > 0) ? b < 0 : (m < 0 && b != m)) m += b;
    return m;
}
static const double LUA_PI = 3.141592653589793238462643383279502884;

/* ===================== a380x_aileron_dance.lua ===================== */
#define DANCE_EPSILON 1e-10

static double dance_command(const A380XDanceInput *in)                 /* command(input) */
{
    double roll = in != NULL && in->has_roll ? in->roll : 0;
    if (!lfinite(roll)) roll = 0;
    return -30 * lclamp(roll, -1, 1);
}

static double move_toward(double previous, double target, double rate, double dt)
{
    return previous + lclamp(target - previous, -rate * dt, rate * dt);
}

static void distributed(double q, double u, double delayed, double *inboard, double *midboard, double *outboard)
{
    *inboard = lclamp(u, -30, 30);
    *midboard = lclamp(delayed, -30, 30);
    *outboard = lclamp((u - *inboard) + (delayed - *midboard) + q, -30, 30);
}

static void targets(double inboard, double midboard, double outboard, double out[6])
{
    out[0] = lclamp(-inboard, -30, 20); out[1] = lclamp(-midboard, -30, 20); out[2] = lclamp(-outboard, -30, 20);
    out[3] = lclamp(inboard, -30, 20); out[4] = lclamp(midboard, -30, 20); out[5] = lclamp(outboard, -30, 20);
}

static void dance_publish(A380XDanceState *s)                          /* publish(state) */
{
    int i;
    for (i = 0; i < 6; ++i) {
        double side = i < 3 ? 1 : -1;
        s->outputs[i] = side * s->degrees[i] / 30;
    }
}

static A380XDanceSample *history_at(A380XDanceState *s, int index)
{
    return &s->history[(s->history_first + index) % A380X_DANCE_HISTORY];
}

static void history_push(A380XDanceState *s, double time, double value)
{
    A380XDanceSample *slot;
    if (s->history_count == A380X_DANCE_HISTORY) {     /* unreachable (the Lua stays below ~38) */
        s->history_first = (s->history_first + 1) % A380X_DANCE_HISTORY;
        --s->history_count;
    }
    slot = history_at(s, s->history_count++);
    slot->time = time;
    slot->value = value;
    if (s->history_count > s->history_peak) s->history_peak = s->history_count;
}

static void reset_delay(A380XDanceState *s, double value)               /* reset_delay(state, value) */
{
    s->history_first = s->history_count = 0;
    history_push(s, s->time - A380X_DANCE_DELAY_SECONDS, value);
    history_push(s, s->time, value);
    s->next_sample = s->time + A380X_DANCE_MAX_STEP_SECONDS;
}

static double delayed_value(A380XDanceState *s)                         /* delayed_value(state) */
{
    double wanted = s->time - A380X_DANCE_DELAY_SECONDS, fraction;
    const A380XDanceSample *low, *high;
    while (s->history_count > 2 && history_at(s, 1)->time <= wanted) {
        s->history_first = (s->history_first + 1) % A380X_DANCE_HISTORY;
        --s->history_count;
    }
    low = history_at(s, 0);
    high = history_at(s, 1);
    if (wanted <= low->time) return low->value;
    if (wanted >= high->time) return high->value;
    fraction = (wanted - low->time) / (high->time - low->time);
    return low->value + (high->value - low->value) * fraction;
}

void a380x_dance_reset(A380XDanceState *s, const A380XDanceInput *in)   /* M.reset(state, input) */
{
    static const A380XDanceInput empty = {0, 0, 0, 0};
    double c, q, u, inboard, midboard, outboard;
    int grounded;
    if (in == NULL) in = &empty;
    c = dance_command(in);
    s->time = 0;
    s->q = 0.5 * c;
    grounded = in->grounded;
    q = grounded ? c : s->q;
    u = grounded ? c : 2.5 * q;
    distributed(q, u, u, &inboard, &midboard, &outboard);
    targets(inboard, midboard, outboard, s->degrees);
    reset_delay(s, u);
    s->initialized = 1;
    dance_publish(s);
}

void a380x_dance_new(A380XDanceState *s)                                /* M.new() */
{
    memset(s, 0, sizeof(*s));
    a380x_dance_reset(s, NULL);
}

void a380x_dance_update(A380XDanceState *s, const A380XDanceInput *in, double dt)   /* M.update */
{
    static const A380XDanceInput empty = {0, 0, 0, 0};
    double c, remaining;
    int grounded;
    if (in == NULL) in = &empty;
    if (!s->initialized) a380x_dance_reset(s, in);
    if (in->paused) return;
    if (!lfinite(dt) || dt < 0 || dt > A380X_DANCE_MAX_DT_SECONDS) {
        a380x_dance_reset(s, in);
        return;
    }
    if (dt == 0) return;
    c = dance_command(in);
    grounded = in->grounded;
    remaining = dt;
    while (remaining > DANCE_EPSILON) {
        double to_sample = s->next_sample - s->time;
        double step = remaining, q, u, delayed, inboard, midboard, outboard, target[6];
        int i;
        if (A380X_DANCE_MAX_STEP_SECONDS < step) step = A380X_DANCE_MAX_STEP_SECONDS;  /* math.min(...) */
        if (to_sample < step) step = to_sample;
        /* Keep the hidden splitter state active while the ground bypass is selected. */
        s->q = move_toward(s->q, 0.5 * c, 12, step);
        s->time = s->time + step;
        q = grounded ? c : s->q;
        u = grounded ? c : 2.5 * q;
        if (grounded) {
            reset_delay(s, u);
            delayed = u;
        } else {
            if (s->time >= s->next_sample - DANCE_EPSILON) {
                history_push(s, s->time, u);
                s->next_sample = s->next_sample + A380X_DANCE_MAX_STEP_SECONDS;
            }
            delayed = delayed_value(s);
        }
        distributed(q, u, delayed, &inboard, &midboard, &outboard);
        targets(inboard, midboard, outboard, target);
        for (i = 0; i < 6; ++i) s->degrees[i] = move_toward(s->degrees[i], target[i], 30, step);
        remaining = remaining - step;
    }
    dance_publish(s);
}

/* ===================== a380x_hydraulic_surfaces.lua ===================== */
static const double DROOP_RATIO = 20.0 / 30.0;                         /* M.DROOP_RATIO = 20 / 30 */

int a380x_hyd_valid(const A380XHydInput *in)                           /* M.valid(input) */
{
    int i;
    if (in == NULL) return 0;
    for (i = 0; i < 6; ++i) if (!lfinite(in->ailerons[i])) return 0;
    for (i = 0; i < 8; ++i) if (!lfinite(in->spoilers[i])) return 0;
    for (i = 0; i < 3; ++i) if (!lfinite(in->pressures[i])) return 0;
    return lfinite(in->pitch) && lfinite(in->rudder) && lfinite(in->heading) && lfinite(in->wind_direction)
           && lfinite(in->wind_speed) && lfinite(in->airspeed);
}

static double hyd_pressure(const A380XHydInput *in)                    /* math.max(0, p1, p2, p3) */
{
    return lmax(lmax(lmax(0, in->pressures[0]), in->pressures[1]), in->pressures[2]);
}

static double slew(double previous, double target, double rate, double dt)
{
    return previous + lclamp(target - previous, -rate * dt, rate * dt);
}

static double damp(double previous, double target, double tau, double rate, double dt)
{
    return slew(previous, previous + (target - previous) * (1 - exp(-dt / tau)), rate, dt);
}

void a380x_hyd_new(A380XHydState *s, const A380XHydInput *in)          /* M.new(input) */
{
    int i, follows_controls;
    s->powered = hyd_pressure(in) >= A380X_HYD_PRESSURE_RESTORE_PSI;
    follows_controls = s->powered || !in->grounded;
    for (i = 0; i < A380X_HYD_KEYS; ++i) {
        double value = 0;
        if (follows_controls) {
            if (i < 6) value = lclamp(in->ailerons[i], -1, 1);
            else if (i == 6) value = lclamp(in->pitch, -1, 1);
            else if (i == 7) value = lclamp(in->rudder, -1, 1);
            else if (s->powered) value = lclamp(in->spoilers[i - 8], 0, 65);
        }
        s->out[i] = value;
    }
}

void a380x_hyd_update(A380XHydState *s, const A380XHydInput *in, double dt)   /* M.update */
{
    double p, airflow, relaxed, wind;
    int i;
    /* Invalid input, pauses and reload-sized time gaps hold the previous pose. */
    if (!a380x_hyd_valid(in) || in->paused || !lfinite(dt) || dt <= 0 || dt > 1) return;
    p = hyd_pressure(in);
    if (p >= A380X_HYD_PRESSURE_RESTORE_PSI) s->powered = 1;
    else if (p <= A380X_HYD_PRESSURE_RELEASE_PSI) s->powered = 0;
    if (s->powered || !in->grounded) {
        for (i = 0; i < 6; ++i) s->out[i] = slew(s->out[i], lclamp(in->ailerons[i], -1, 1), 1, dt);
        s->out[6] = slew(s->out[6], lclamp(in->pitch, -1, 1), 1, dt);
        s->out[7] = slew(s->out[7], lclamp(in->rudder, -1, 1), 1, dt);
        if (s->powered)
            for (i = 0; i < 8; ++i) s->out[8 + i] = slew(s->out[8 + i], lclamp(in->spoilers[i], 0, 65), 45, dt);
        return;
    }
    /* Airflow progressively unloads parked droop during a fast ground roll. */
    airflow = lclamp((lmax(0, in->airspeed) - 10) / 25, 0, 1);
    relaxed = DROOP_RATIO * (1 - airflow);
    for (i = 0; i < 6; ++i) {
        double sign = i < 3 ? 1 : -1;
        s->out[i] = damp(s->out[i], sign * relaxed, 8, 0.045, dt);
    }
    s->out[6] = damp(s->out[6], -relaxed, 10, 0.04, dt);
    /* Wind is FROM true north; calm air holds the last free pose. */
    wind = lmax(0, in->wind_speed);
    if (wind >= 0.25) {
        double relative = lmod(in->wind_direction - in->heading, 360) * (LUA_PI / 180.0);   /* math.rad */
        double target = -sin(relative) * wind * wind / (wind * wind + 25);
        s->out[7] = damp(s->out[7], target, 8, 0.05, dt);
    }
    /* Spoilers stay at the last supported position. */
}

/* ===================== the two driver scripts ===================== */
void a380x_surface_store(float *slot, double value, float minimum, float maximum)
{
    float v = (float)value;    /* XPLMSetDataf(float) */
    if (!isfinite(v)) return;  /* the plugin keeps the last finite value */
    *slot = v < minimum ? minimum : v > maximum ? maximum : v;
}

float a380x_surface_minimum(int index) { return index < A380X_PUB_HYD + 8 ? -1.0f : 0.0f; }
float a380x_surface_maximum(int index) { return index < A380X_PUB_HYD + 8 ? 1.0f : 65.0f; }

static void put(A380XSurfaceDriver *d, int index, double value)
{
    a380x_surface_store(&d->pub[index], value, a380x_surface_minimum(index), a380x_surface_maximum(index));
}

void a380x_surface_driver_reset(A380XSurfaceDriver *d)
{
    float pub[A380X_PUB_COUNT];
    memcpy(pub, d->pub, sizeof(pub));
    memset(d, 0, sizeof(*d));
    memcpy(d->pub, pub, sizeof(pub));
    d->dance_enabled = d->hyd_enabled = 1;
}

/* A380X_XP12_Ailerons.lua input(): nil (0) unless roll is finite and both flags are 0 / 1. */
static int dance_input(const A380XSurfaceSources *src, A380XDanceInput *in)
{
    double roll = src->roll;
    if (!lfinite(roll) || (src->grounded != 0 && src->grounded != 1) || (src->paused != 0 && src->paused != 1))
        return 0;
    in->has_roll = 1;
    in->roll = lmax(-1, lmin(1, roll));
    in->grounded = src->grounded == 1;
    in->paused = src->paused == 1;
    return 1;
}

static int dance_outputs_valid(const A380XDanceState *s)
{
    int i;
    for (i = 0; i < 6; ++i) if (!lfinite(s->outputs[i])) return 0;
    return 1;
}

static void dance_clear(A380XSurfaceDriver *d)
{
    int i;
    for (i = 0; i < 6; ++i) put(d, A380X_PUB_VISUAL + i, 0);
}

static void dance_disable(A380XSurfaceDriver *d)
{
    d->dance_enabled = d->dance_active = 0;
    dance_clear(d);
}

/* publish(values): every value finite, else a runtime error disables the provider (and clears). */
static int dance_publish_outputs(A380XSurfaceDriver *d)
{
    int i;
    if (!dance_outputs_valid(&d->dance)) { dance_disable(d); return 0; }
    for (i = 0; i < 6; ++i) {
        put(d, A380X_PUB_VISUAL + i, lmax(-1, lmin(1, d->dance.outputs[i])));
        if (fabs(d->dance.outputs[i]) > 0.000001) d->dance_reported_nonzero = 1;
    }
    return 1;
}

static void dance_load(A380XSurfaceDriver *d, const A380XSurfaceSources *src)
{
    static const A380XDanceInput neutral = {1, 0, 1, 0};   /* {roll = 0, grounded = true, paused = false} */
    A380XDanceInput first;
    int have_first;
    a380x_dance_new(&d->dance);
    a380x_dance_reset(&d->dance, &neutral);
    have_first = dance_input(src, &first);
    if (have_first) a380x_dance_reset(&d->dance, &first);
    if (!dance_publish_outputs(d)) return;
    d->dance_active = have_first;
}

static void dance_frame(A380XSurfaceDriver *d, const A380XSurfaceSources *src)
{
    static const A380XDanceInput neutral = {1, 0, 1, 0};
    A380XDanceInput current;
    double dt = src->dt;
    if (!d->dance_enabled) return;
    if (!dance_input(src, &current)) {
        if (d->dance_active) {
            a380x_dance_reset(&d->dance, &neutral);
            dance_clear(d);
            d->dance_active = 0;
        }
        d->dance_warned = 1;
        return;
    }
    d->dance_warned = 0;
    if (!d->dance_active) {
        a380x_dance_reset(&d->dance, &current);
        if (dance_publish_outputs(d)) d->dance_active = 1;
        return;
    }
    if (current.paused) return;
    /* The core handles zero, negative, nonfinite and discontinuous frame periods. */
    a380x_dance_update(&d->dance, &current, dt);
    dance_publish_outputs(d);
}

/* A380X_XP12_Hydraulics.lua input(): the visual aileron inputs are read back from the published
 * a380x/xp12/visual refs (float storage), exactly as the script reads them through dataref_table. */
static int hyd_input(const A380XSurfaceDriver *d, const A380XSurfaceSources *src, A380XHydInput *in)
{
    int i;
    if ((src->grounded != 0 && src->grounded != 1) || (src->paused != 0 && src->paused != 1)) return 0;
    in->grounded = src->grounded == 1;
    in->paused = src->paused == 1;
    in->pitch = src->pitch;
    in->rudder = src->yaw;
    in->heading = src->heading;
    in->wind_direction = src->wind_direction;
    in->wind_speed = src->wind_speed;
    in->airspeed = src->airspeed;
    for (i = 0; i < 3; ++i) in->pressures[i] = src->pressures[i];
    for (i = 0; i < 6; ++i) in->ailerons[i] = d->pub[A380X_PUB_VISUAL + i];
    in->spoilers[0] = src->speedbrake1[0]; in->spoilers[1] = src->speedbrake1[1];
    for (i = 0; i < 4; ++i) in->spoilers[2 + i] = src->spoiler1[i];
    in->spoilers[6] = src->spoiler2[0]; in->spoilers[7] = src->spoiler2[1];
    return a380x_hyd_valid(in);
}

/* publish(values): validate the complete update before publishing any part of it; a failure is a
 * runtime error that disables the provider and holds the last positions. */
static void hyd_publish(A380XSurfaceDriver *d)
{
    int i;
    for (i = 0; i < A380X_HYD_KEYS; ++i) {
        double value = d->hyd.out[i], lo = i < 8 ? -1 : 0, hi = i < 8 ? 1 : 65;
        if (!lfinite(value) || !(value >= lo && value <= hi)) { d->hyd_enabled = 0; d->hyd_active = 0; return; }
    }
    for (i = 0; i < A380X_HYD_KEYS; ++i) put(d, A380X_PUB_HYD + i, d->hyd.out[i]);
}

static void hyd_frame(A380XSurfaceDriver *d, const A380XSurfaceSources *src)
{
    A380XHydInput current;
    if (!d->hyd_enabled) return;
    memset(&current, 0, sizeof(current));
    if (!hyd_input(d, src, &current)) {
        d->hyd_warned = 1;   /* holding last surface positions */
        return;
    }
    d->hyd_warned = 0;
    if (!d->hyd_active) {
        a380x_hyd_new(&d->hyd, &current);
        d->hyd_active = 1;
        hyd_publish(d);
    } else {
        a380x_hyd_update(&d->hyd, &current, src->dt);
        hyd_publish(d);
    }
}

void a380x_surface_driver_step(A380XSurfaceDriver *d, const A380XSurfaceSources *src)
{
    if (!d->loaded) {
        /* FlyWithLua loads A380X_XP12_Ailerons.lua (initial reset and publication), then
         * A380X_XP12_Hydraulics.lua, which runs its first update at load. */
        d->dance_enabled = d->hyd_enabled = 1;
        dance_load(d, src);
        hyd_frame(d, src);
        d->loaded = 1;
        return;
    }
    ++d->frames;
    /* do_every_frame order: A380X_AILERON_VISUALS.update(), then A380X_HYDRAULIC_SURFACES.update(). */
    dance_frame(d, src);
    hyd_frame(d, src);
}
