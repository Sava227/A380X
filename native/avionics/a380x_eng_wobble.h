/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 engine pylon wobble: each engine nods (pitch) and sways (yaw) on its pylon relative to the wing.
 *
 * One lightly damped spring-mass per axis and engine, driven by the acceleration at the pylon attachment
 * (plane axes, gravity removed). The input is high-passed, so steady loads (cruise, a sustained turn, parked)
 * leave the engines centred: the steady droop is wing flex's job, this only adds the ringing after a bump.
 * The engine hangs ahead of the pylon pivot, so the mount accelerating up nods the intake down, and the mount
 * accelerating right swings the intake left.
 * Outputs in degrees: pitch + = intake up, yaw + = intake right. Pure C, no XPLM.
 */
#ifndef A380X_ENG_WOBBLE_H
#define A380X_ENG_WOBBLE_H
#include <math.h>

#define A380X_EW_ENGINES 4

typedef struct { double x, v; } A380XEwAxis;                       /* deg, deg/s */
typedef struct { A380XEwAxis pitch, yaw; double lp_vert, lp_lat; } A380XEwEngine;
typedef struct { A380XEwEngine e[A380X_EW_ENGINES]; float pitch_deg[A380X_EW_ENGINES], yaw_deg[A380X_EW_ENGINES]; } A380XEngWobble;

static const double a380x_ew_hz_pitch = 2.5, a380x_ew_hz_yaw = 1.8;   /* pylon modes, Hz */
static const double a380x_ew_zeta = 0.06;                              /* light damping: a bump rings a few cycles */
static const double a380x_ew_gain_pitch = 0.035, a380x_ew_gain_yaw = 0.03;   /* deg per m/s^2 of a held step */
static const double a380x_ew_hp_tau = 1.0;                             /* s: steady loads bleed out */
static const double a380x_ew_max_deg = 1.5;

static inline void a380x_ew_axis(A380XEwAxis *a, double accel, double hz, double gain, double h)
{
    double w = 2. * M_PI * hz;
    a->v += (-w * w * (a->x + gain * accel) - 2. * a380x_ew_zeta * w * a->v) * h;   /* semi-implicit Euler */
    a->x += a->v * h;
    if (fabs(a->x) > a380x_ew_max_deg) { a->x = copysign(a380x_ew_max_deg, a->x); a->v = 0.; }
}

/* vert / lat: acceleration at each pylon attachment, m/s^2, + up / + right. Engines 1..4 left to right. */
static inline void a380x_eng_wobble_update(A380XEngWobble *w, const double vert[A380X_EW_ENGINES],
                                           const double lat[A380X_EW_ENGINES], double dt)
{
    if (!(dt > 0.)) return;
    for (int i = 0; i < A380X_EW_ENGINES; ++i) if (!isfinite(vert[i]) || !isfinite(lat[i])) return;
    if (dt > 0.25) dt = 0.25;
    int steps = (int)ceil(dt / 0.005);
    double h = dt / steps, k = 1. - exp(-h / a380x_ew_hp_tau);
    for (int i = 0; i < A380X_EW_ENGINES; ++i) {
        A380XEwEngine *e = &w->e[i];
        for (int s = 0; s < steps; ++s) {
            e->lp_vert += (vert[i] - e->lp_vert) * k;
            e->lp_lat += (lat[i] - e->lp_lat) * k;
            a380x_ew_axis(&e->pitch, vert[i] - e->lp_vert, a380x_ew_hz_pitch, a380x_ew_gain_pitch, h);
            a380x_ew_axis(&e->yaw, lat[i] - e->lp_lat, a380x_ew_hz_yaw, a380x_ew_gain_yaw, h);
        }
        w->pitch_deg[i] = (float)e->pitch.x;
        w->yaw_deg[i] = (float)e->yaw.x;
    }
}

#endif
