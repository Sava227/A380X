/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 PRIM flight-control laws and nosewheel / body-wheel steering for the X-Plane port
 * (docs/FLIGHT_CONTROLS.md). XPLM-free core; the X-Plane adapter is a380x_fctl.inc.
 *
 * THIS IS SIMULATION SOFTWARE, NOT REAL-WORLD FLIGHT-CONTROL SOFTWARE. It is an original port law
 * written from the public Airbus normal-law description (FlyByWire A32NX/A380X documentation, Airbus
 * flight-crew training material as summarised there), not FBW's PRIM code: the structure and the
 * limits are Airbus', the gains are port values for X-Plane's flight model and need in-sim tuning.
 *
 * Pitch (normal law, flight mode): the sidestick commands a load-factor increment (C*-type law,
 * flown as a pitch-rate demand Δnz·g/V) with automatic pitch trim. Stick released = 1 g with the
 * flight path held and turn compensation up to 33° bank. Protections: load factor (−1..+2.5 g clean,
 * 0..+2 g flaps), pitch attitude (+30° / +25° CONF FULL, −15°), high angle of attack (α prot..α max,
 * auto-trim frozen, A.FLOOR request), high speed (VMO+6 / MMO+0.01: nose-up demand, reduced nose-down
 * authority, trim frozen). Ground mode = direct, blended into flight mode over 5 s after liftoff; flare
 * mode below 100 ft RA = attitude memorised and lowered 2° over 8 s (the pilot flares), ground mode
 * again after touchdown.
 * Roll: the sidestick commands a roll rate (max 15°/s); stick released holds the bank (≤ 33°) or
 * returns it to 33°; bank limit 67° (45° in high-AoA protection, 40° in high-speed protection, where a
 * released stick returns to wings level). Yaw: yaw damper + turn coordination added to the pedals.
 * Direct law (pass-through, manual trim) without IR/ADR data. The AP keeps its own actuator path.
 *
 * Steering: hand wheel (tiller) ±70° (full ≤ 20 kt, 0 at 70 kt), pedals ±6° (full ≤ 40 kt, 0 at
 * 130 kt). Without a hardware tiller the pedals also steer the full ±70° at taxi speeds (≤ 10 kt,
 * blended out by 30 kt). Body-wheel steering opposite to the nose gear beyond 20° of nose-wheel angle
 * below 20 kt, up to 15°. The actuators are rate limited (heavy-aircraft feel).
 */
#ifndef A380X_FCTL_H
#define A380X_FCTL_H

#include <math.h>
#include <string.h>

#define A380X_FCTL_R2D 57.29577951308232
#define A380X_FCTL_G 9.80665
#define A380X_FCTL_KT 0.5144444444444444

enum { A380X_FCTL_LAW_OFF = 0, A380X_FCTL_LAW_NORMAL = 1, A380X_FCTL_LAW_DIRECT = 2 };
enum { A380X_FCTL_GROUND = 0, A380X_FCTL_FLIGHT = 1, A380X_FCTL_FLARE = 2 };
/* Protection bits (a380x/xp12/fbw/protections). */
enum { A380X_FCTL_PROT_ALPHA = 1, A380X_FCTL_PROT_HIGH_SPEED = 2, A380X_FCTL_PROT_PITCH = 4,
       A380X_FCTL_PROT_BANK = 8, A380X_FCTL_PROT_LOAD = 16, A380X_FCTL_PROT_ALPHA_FLOOR = 32 };

typedef struct {
    double dt;                     /* simulated seconds of this step (0: paused) */
    int enabled, powered;          /* crew switch (command), PRIM power */
    int adr_valid, ir_valid;       /* air data and inertial data for the normal law */
    int on_ground;                 /* any gear on the ground */
    int ap_pitch_roll, ap_yaw;     /* an AP flies pitch/roll (own actuator path); AP rudder (ALIGN / ROLL OUT) */
    double ap_pitch, ap_roll;      /* the AP's yoke orders, for a bumpless handover at disconnect */
    double stick_pitch, stick_roll, pedals;   /* pilot, X-Plane yoke ratios: + nose up / right / right */
    int tiller_hardware;           /* a hardware tiller axis is assigned */
    double tiller;                 /* hand wheel −1..1 (+ right): hardware + keyboard */
    double ias_kt, tas_kt, gs_kt, mach, vs_fpm;
    double pitch_deg, roll_deg, alpha_deg;
    double q_dps, p_dps, r_dps;    /* body rates: + nose up, + right wing down, + nose right */
    double nz;                     /* normal load factor, g */
    int ra_valid;
    double ra_ft;
    int flap_index;                /* FLAPS lever 0 (clean) .. 4 (FULL) */
    double vmo_kt, mmo;            /* 0: A380 defaults 340 kt / M 0.89 */
} A380XFctlInputs;

typedef struct {
    int law, mode, protections, active;   /* active: artificial-stability corrections are being written */
    double pitch, roll, yaw;              /* total surface orders −1..1 */
    double artstab_pitch, artstab_roll, artstab_yaw;   /* = order − pilot input (X-Plane artstab) */
    int auto_trim;
    double trim_rate_deg_s;               /* THS order, + nose up */
    int steering;                         /* nose / body steering orders valid (on the ground) */
    double nose_deg, body_deg;            /* + right (body gear: opposite to the nose gear) */
    double nz_cmd, q_cmd, p_cmd, alpha_prot, alpha_max;
    int alpha_floor;                      /* A.FLOOR request for the A/THR */
} A380XFctlOutputs;

typedef struct {
    int initialized, law, mode, prot;
    double blend;                 /* 0 ground (direct) .. 1 flight law */
    double airborne_s, ground_s, release_s, roll_release_s;
    double q_f, pitch_int, gamma_ref; int gamma_hold;
    double p_f, roll_int, bank_ref; int bank_hold;
    double flare_theta0, flare_t;
    double nz_f, alpha_f, alpha_prev, alpha_dot, ias_prev, ias_rate;
    int prot_alpha, prot_hs;
    double nose_deg, body_deg;
} A380XFctlState;

static inline double a380x_fctl_clamp(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }
static inline double a380x_fctl_finite(double v, double fallback) { return isfinite(v) ? v : fallback; }

/* Sidestick shaping: 3 % deadband, then half linear / half cubic (fine control near neutral). */
static inline double a380x_fctl_shape(double s)
{
    double a;
    s = a380x_fctl_clamp(a380x_fctl_finite(s, 0.0), -1.0, 1.0);
    a = fabs(s);
    if (a <= 0.03) return 0.0;
    a = (a - 0.03) / 0.97;
    a = 0.5 * a + 0.5 * a * a * a;
    return s < 0 ? -a : a;
}

static inline double a380x_fctl_rate(double current, double target, double rate, double dt)
{
    double step = rate * dt;
    return current + a380x_fctl_clamp(target - current, -step, step);
}

static inline double a380x_fctl_lag(double current, double target, double tau, double dt)
{
    return tau > 0 ? current + (target - current) * (1.0 - exp(-dt / tau)) : target;
}

/* α prot / α max (deg, X-Plane body AoA) by FLAPS lever, reduced at high Mach. Port values below the
 * installed model's 15° stall-warning AoA; the A320 documentation's α floor (9.5° clean .. 15°) is the
 * reference for the spread. */
static inline void a380x_fctl_alpha_limits(int flap_index, double mach, double *prot, double *max)
{
    static const double p[5] = {8.5, 11.0, 11.0, 11.0, 10.5};
    static const double m[5] = {11.0, 14.0, 14.0, 13.5, 13.0};
    int k = flap_index < 0 ? 0 : flap_index > 4 ? 4 : flap_index;
    double mach_cut = mach > 0.5 ? 6.0 * (mach - 0.5) : 0.0;
    *prot = p[k] - mach_cut;
    *max = m[k] - mach_cut;
}

/* Inner-loop gains per unit of the dynamic-pressure schedule G (elevator / aileron ratio per deg/s and
 * per deg): proportional, integral and demand feed-forward. */
#define A380X_FCTL_PITCH_KP 0.10
#define A380X_FCTL_PITCH_KI 0.045
#define A380X_FCTL_PITCH_KFF 0.04
#define A380X_FCTL_ROLL_KP 0.05
#define A380X_FCTL_ROLL_KI 0.05
#define A380X_FCTL_ROLL_KFF 0.03

/* Gain schedule on dynamic pressure: the surfaces' effect grows with IAS². */
static inline double a380x_fctl_gain(double ias_kt)
{
    double v = fmax(ias_kt, 60.0);
    return a380x_fctl_clamp((250.0 / v) * (250.0 / v), 0.35, 2.5);
}

static inline void a380x_fctl_init(A380XFctlState *s)
{
    memset(s, 0, sizeof(*s));
    s->initialized = 1;
    s->nz_f = 1.0;
}

/* Steering: nose wheel (tiller + pedals, speed scheduled) and opposite body-wheel steering. */
static inline void a380x_fctl_steering(A380XFctlState *s, const A380XFctlInputs *i, A380XFctlOutputs *o)
{
    double gs = fabs(a380x_fctl_finite(i->gs_kt, 0.0)), dt = i->dt, nose, body = 0.0;
    double tiller_auth = 70.0 * a380x_fctl_clamp((70.0 - gs) / 50.0, 0.0, 1.0);
    double pedal_auth = 6.0 * a380x_fctl_clamp((130.0 - gs) / 90.0, 0.0, 1.0);
    if (!i->tiller_hardware)   /* keyboard / twist-grip pilots: the pedals turn the aircraft at taxi speed */
        pedal_auth += 64.0 * a380x_fctl_clamp((30.0 - gs) / 20.0, 0.0, 1.0);
    nose = a380x_fctl_clamp(a380x_fctl_finite(i->tiller, 0.0), -1.0, 1.0) * tiller_auth
         + a380x_fctl_clamp(a380x_fctl_finite(i->pedals, 0.0), -1.0, 1.0) * pedal_auth;
    nose = a380x_fctl_clamp(nose, -70.0, 70.0);
    if (!i->on_ground) nose = 0.0;               /* centring in flight */
    s->nose_deg = a380x_fctl_rate(s->nose_deg, nose, 20.0, dt);   /* nose-gear slew, deg/s */
    if (i->on_ground && gs < 20.0 && fabs(s->nose_deg) > 20.0)
        body = -(s->nose_deg > 0 ? 1.0 : -1.0) * fmin(15.0, (fabs(s->nose_deg) - 20.0) * 15.0 / 50.0);
    s->body_deg = a380x_fctl_rate(s->body_deg, body, 5.0, dt);
    o->steering = 1;
    o->nose_deg = s->nose_deg;
    o->body_deg = s->body_deg;
}

/* One PRIM frame. */
static inline void a380x_fctl_step(A380XFctlState *s, const A380XFctlInputs *i, A380XFctlOutputs *o)
{
    double dt = a380x_fctl_clamp(a380x_fctl_finite(i->dt, 0.0), 0.0, 0.25);
    double sp = a380x_fctl_shape(i->stick_pitch), sr = a380x_fctl_shape(i->stick_roll);
    double pilot_pitch = a380x_fctl_clamp(a380x_fctl_finite(i->stick_pitch, 0.0), -1, 1);
    double pilot_roll = a380x_fctl_clamp(a380x_fctl_finite(i->stick_roll, 0.0), -1, 1);
    double pilot_yaw = a380x_fctl_clamp(a380x_fctl_finite(i->pedals, 0.0), -1, 1);
    double ias = a380x_fctl_finite(i->ias_kt, 0.0), V = fmax(a380x_fctl_finite(i->tas_kt, 0.0) * A380X_FCTL_KT, 50.0);
    double theta = a380x_fctl_finite(i->pitch_deg, 0.0), phi = a380x_fctl_finite(i->roll_deg, 0.0);
    double q = a380x_fctl_finite(i->q_dps, 0.0), p = a380x_fctl_finite(i->p_dps, 0.0), r = a380x_fctl_finite(i->r_dps, 0.0);
    double mach = a380x_fctl_finite(i->mach, 0.0), G = a380x_fctl_gain(ias);
    double vmo = i->vmo_kt > 100 ? i->vmo_kt : 340.0, mmo = i->mmo > 0.3 ? i->mmo : 0.89;
    double gamma, g_over_v = A380X_FCTL_G / V * A380X_FCTL_R2D;   /* deg/s of pitch rate per g */
    int full = i->flap_index >= 4, flaps = i->flap_index > 0, prot = 0;
    double nz_max = flaps ? 2.0 : 2.5, nz_min = flaps ? 0.0 : -1.0;
    double theta_max = full ? 25.0 : 30.0, theta_min = -15.0, phi_max = 67.0;
    double dnz, q_dem, p_dem, pitch, roll, yaw, a_prot, a_max, a_floor, ias_pred, mach_pred;
    int was_mode = s->mode;

    if (!s->initialized) a380x_fctl_init(s);
    memset(o, 0, sizeof(*o));
    a380x_fctl_alpha_limits(i->flap_index, mach, &a_prot, &a_max);
    o->alpha_prot = a_prot; o->alpha_max = a_max;
    if (!i->enabled || !i->powered) {           /* released: X-Plane's own controls and steering */
        a380x_fctl_init(s);
        o->law = A380X_FCTL_LAW_OFF;
        o->pitch = pilot_pitch; o->roll = pilot_roll; o->yaw = pilot_yaw;
        return;
    }
    s->law = i->adr_valid && i->ir_valid ? A380X_FCTL_LAW_NORMAL : A380X_FCTL_LAW_DIRECT;
    o->law = s->law;
    a380x_fctl_steering(s, i, o);

    /* ---- mode: ground / flight / flare ---- */
    if (i->on_ground) {
        s->ground_s += dt; s->airborne_s = 0;
        if (s->mode != A380X_FCTL_GROUND && s->ground_s > 0.0) s->mode = A380X_FCTL_GROUND;
        s->blend = a380x_fctl_rate(s->blend, 0.0, 1.0 / 2.0, dt);    /* touchdown: direct within 2 s */
    } else {
        s->airborne_s += dt; s->ground_s = 0;
        if (s->mode == A380X_FCTL_GROUND) s->mode = A380X_FCTL_FLIGHT;
        s->blend = a380x_fctl_rate(s->blend, 1.0, 1.0 / 5.0, dt);    /* liftoff: flight law over 5 s */
        if (s->mode == A380X_FCTL_FLIGHT && i->ra_valid && i->ra_ft < 100.0 && i->vs_fpm < 0 && s->airborne_s > 10.0) {
            s->mode = A380X_FCTL_FLARE; s->flare_theta0 = theta; s->flare_t = 0;
        } else if (s->mode == A380X_FCTL_FLARE && (!i->ra_valid || i->ra_ft > 120.0)) s->mode = A380X_FCTL_FLIGHT;
    }
    o->mode = s->mode;
    s->nz_f = a380x_fctl_lag(s->nz_f, a380x_fctl_finite(i->nz, 1.0), 0.15, dt);
    s->alpha_f = a380x_fctl_lag(s->alpha_f, a380x_fctl_finite(i->alpha_deg, 0.0), 0.3, dt);
    if (dt > 0) s->alpha_dot = a380x_fctl_lag(s->alpha_dot, (a380x_fctl_finite(i->alpha_deg, 0.0) - s->alpha_prev) / dt, 0.3, dt);
    s->alpha_prev = a380x_fctl_finite(i->alpha_deg, 0.0);
    if (dt > 0 && s->ias_prev > 0) s->ias_rate = a380x_fctl_lag(s->ias_rate, a380x_fctl_clamp((ias - s->ias_prev) / dt, -20, 20), 1.0, dt);
    s->ias_prev = ias;
    gamma = asin(a380x_fctl_clamp(a380x_fctl_finite(i->vs_fpm, 0.0) * 0.00508 / V, -1, 1)) * A380X_FCTL_R2D;

    /* Direct law, the AP's own path, or ground mode: no flight-law state; track for a bumpless entry. */
    if (s->law == A380X_FCTL_LAW_DIRECT || i->ap_pitch_roll || s->blend <= 0.0) {
        double hand_pitch = i->ap_pitch_roll ? a380x_fctl_finite(i->ap_pitch, 0.0) : pilot_pitch;
        double hand_roll = i->ap_pitch_roll ? a380x_fctl_finite(i->ap_roll, 0.0) : pilot_roll;
        /* The integrators take the order less the feed-forward of the current rate: no step at the handover. */
        s->pitch_int = a380x_fctl_clamp(hand_pitch - A380X_FCTL_PITCH_KFF * G * q, -0.8, 0.8);
        s->roll_int = a380x_fctl_clamp(hand_roll - A380X_FCTL_ROLL_KFF * G * p, -0.6, 0.6);
        s->q_f = q; s->p_f = p; s->gamma_ref = gamma; s->gamma_hold = 0; s->bank_ref = phi; s->bank_hold = 0;
        s->prot_alpha = s->prot_hs = 0; s->prot = 0;
        o->pitch = hand_pitch; o->roll = hand_roll; o->yaw = pilot_yaw;
        if (s->law == A380X_FCTL_LAW_NORMAL && !i->on_ground && !i->ap_yaw && s->blend > 0.0) {
            double r_turn = g_over_v * tan(a380x_fctl_clamp(phi, -60, 60) / A380X_FCTL_R2D) * cos(phi / A380X_FCTL_R2D);
            o->yaw = a380x_fctl_clamp(pilot_yaw - s->blend * a380x_fctl_clamp(0.10 * G * (r - r_turn), -0.25, 0.25), -1, 1);
        }
        o->artstab_yaw = i->ap_yaw ? 0.0 : o->yaw - pilot_yaw;
        o->active = s->law == A380X_FCTL_LAW_NORMAL;
        o->q_cmd = q; o->p_cmd = p; o->nz_cmd = s->nz_f;
        return;
    }

    /* ---- protections (normal law, airborne) ---- */
    if (!s->prot_alpha && !i->on_ground && s->alpha_f > (i->ra_valid && i->ra_ft < 100 ? a_max : a_prot)) s->prot_alpha = 1;
    else if (s->prot_alpha && (sp < -0.5 || (s->alpha_f < a_prot - 1.0 && sp <= 0.3))) s->prot_alpha = 0;
    /* High speed: VMO+6 / MMO+0.01, anticipated with the 5 s speed trend. */
    ias_pred = ias + 5.0 * fmax(0.0, s->ias_rate);
    mach_pred = mach * ias_pred / fmax(ias, 60.0);
    if (!s->prot_hs && (ias_pred > vmo + 6.0 || mach_pred > mmo + 0.01) && ias > vmo - 15.0) s->prot_hs = 1;
    else if (s->prot_hs && ias < vmo && mach < mmo) s->prot_hs = 0;
    if (s->prot_alpha) { prot |= A380X_FCTL_PROT_ALPHA; phi_max = 45.0; }
    if (s->prot_hs) { prot |= A380X_FCTL_PROT_HIGH_SPEED; phi_max = 40.0; }

    /* ---- pitch ---- */
    if (s->mode == A380X_FCTL_FLARE) {
        /* Flare law: attitude memorised at entry, lowered 2° over 8 s; the stick adds up to 4°/s. */
        double target;
        s->flare_t += dt;
        target = s->flare_theta0 - 2.0 * fmin(s->flare_t, 8.0) / 8.0;
        q_dem = 0.6 * (target - theta) + 4.0 * sp;
        dnz = q_dem / g_over_v;
    } else {
        double q_turn = 0.0, phi_c = a380x_fctl_clamp(phi, -33.0, 33.0) / A380X_FCTL_R2D;
        double s_eff = sp, k_hs = 0.0;
        if (s->prot_hs) {
            /* Nose-down authority fades to zero by VMO+16 / MMO+0.04 (full forward stick then holds about
             * that speed) and an automatic nose-up demand grows with the overspeed. */
            k_hs = a380x_fctl_clamp(fmax((ias_pred - vmo) / 16.0, (mach_pred - mmo) / 0.04), 0.0, 1.0);
            if (s_eff < 0) s_eff *= 1.0 - k_hs;
        }
        dnz = s_eff >= 0 ? s_eff * (nz_max - 1.0) : s_eff * (1.0 - nz_min);
        if (s->prot_hs)
            dnz += 0.3 * k_hs + 0.03 * fmax(0.0, fmax(ias - vmo - 16.0, (mach - mmo - 0.04) * 600.0));
        q_turn = g_over_v * tan(phi_c) * sin(phi_c);                    /* level turn up to 33° bank */
        q_dem = dnz * g_over_v + q_turn;
        /* Flight-path stability: stick released (after 1 s) holds the flight path angle. */
        if (fabs(sp) > 0.0) { s->release_s = 0; s->gamma_hold = 0; }
        else if ((s->release_s += dt) > 1.0 && !s->gamma_hold && fabs(q - q_turn) < 1.0) {
            s->gamma_hold = 1; s->gamma_ref = gamma;
        }
        if (s->gamma_hold && fabs(phi) <= 33.0 && !s->prot_alpha && !s->prot_hs)
            q_dem += a380x_fctl_clamp(0.25 * (s->gamma_ref - gamma), -1.5, 1.5);
        /* Load factor protection. */
        if (s->nz_f > nz_max) { q_dem -= 2.0 * (s->nz_f - nz_max) * g_over_v; prot |= A380X_FCTL_PROT_LOAD; }
        if (s->nz_f < nz_min) { q_dem += 2.0 * (nz_min - s->nz_f) * g_over_v; prot |= A380X_FCTL_PROT_LOAD; }
    }
    q_dem = a380x_fctl_clamp(q_dem, -6.0, 8.0);
    /* Heavy-aircraft response: lag + pitch acceleration limit on the demand. */
    s->q_f = a380x_fctl_rate(s->q_f, a380x_fctl_lag(s->q_f, q_dem, 0.45, dt), 4.0, dt);
    /* Hard limits act after the shaping (no lag), on predicted attitude and α. */
    {
        double theta_pred = theta + 0.8 * q, hi = 0.6 * (theta_max - theta_pred), lo = -0.6 * (theta_pred - theta_min);
        if (s->prot_alpha && s->mode != A380X_FCTL_FLARE) {
            /* High-AoA protection: the stick commands α between α prot (released) and α max (full aft). */
            double a_cmd = a_prot + fmax(sp, 0.0) * (a_max - a_prot) + fmin(sp, 0.0) * 4.0;
            double a_pred = a380x_fctl_finite(i->alpha_deg, 0.0) + 0.6 * s->alpha_dot;
            hi = fmin(hi, 2.0 * (a_cmd - a_pred));
        }
        if (s->q_f > hi) { s->q_f = fmax(hi, -6.0); if (theta > theta_max - 3) prot |= A380X_FCTL_PROT_PITCH; }
        if (s->q_f < lo) { s->q_f = fmin(lo, 8.0); if (theta < theta_min + 3) prot |= A380X_FCTL_PROT_PITCH; }
    }
    {
        double e = s->q_f - q, kp = A380X_FCTL_PITCH_KP * G, ki = A380X_FCTL_PITCH_KI * G, kff = A380X_FCTL_PITCH_KFF * G, out;
        out = s->pitch_int + kp * e + kff * s->q_f;
        if (!((out >= 1.0 && e > 0) || (out <= -1.0 && e < 0)))         /* anti-windup */
            s->pitch_int = a380x_fctl_clamp(s->pitch_int + ki * e * dt, -0.8, 0.8);
        pitch = a380x_fctl_clamp(s->pitch_int + kp * e + kff * s->q_f, -1.0, 1.0);
    }
    o->q_cmd = s->q_f; o->nz_cmd = 1.0 + dnz;

    /* Auto-trim: THS unloads the elevator (not in protections, steep bank, flare or transitions). */
    if (s->mode == A380X_FCTL_FLIGHT && s->blend >= 0.999 && !s->prot_alpha && !s->prot_hs &&
        fabs(phi) <= 33.0 && s->nz_f > 0.5 && s->nz_f < 1.5 && ias > 100.0) {
        double limit = flaps ? 0.5 : 0.3;
        o->auto_trim = 1;
        o->trim_rate_deg_s = a380x_fctl_clamp(2.0 * s->pitch_int, -limit, limit);
    }

    /* ---- roll ---- */
    p_dem = sr * 15.0;
    if (sr == 0.0) {
        if (!s->bank_hold && ((s->roll_release_s += dt) > 0.5 || fabs(p) < 2.0)) {
            s->bank_hold = 1; s->bank_ref = a380x_fctl_clamp(phi, -33.0, 33.0);
        }
        if (s->bank_hold) {
            if (s->prot_hs) s->bank_ref = 0.0;                          /* high speed: wings level */
            if (fabs(phi) > 33.0) s->bank_ref = phi > 0 ? 33.0 : -33.0;
            p_dem = a380x_fctl_clamp(0.6 * (s->bank_ref - phi), -5.0, 5.0);
        }
    } else {
        s->bank_hold = 0; s->roll_release_s = 0;
        if (fabs(phi) > 33.0) p_dem -= 0.44 * (fabs(phi) - 33.0) * (phi > 0 ? 1.0 : -1.0);   /* spiral stability */
    }
    if (p_dem * phi > 0 && fabs(p_dem) > 0.8 * (phi_max - fabs(phi))) {
        p_dem = (p_dem > 0 ? 1.0 : -1.0) * fmax(0.0, 0.8 * (phi_max - fabs(phi)));
        if (fabs(phi) > phi_max - 5.0) prot |= A380X_FCTL_PROT_BANK;
    }
    if (fabs(phi) > phi_max) { p_dem = -(phi > 0 ? 1.0 : -1.0) * fmin(5.0, 0.8 * (fabs(phi) - phi_max)) ; prot |= A380X_FCTL_PROT_BANK; }
    s->p_f = a380x_fctl_rate(s->p_f, a380x_fctl_lag(s->p_f, p_dem, 0.25, dt), 15.0, dt);
    {
        double e = s->p_f - p, kp = A380X_FCTL_ROLL_KP * G, ki = A380X_FCTL_ROLL_KI * G, kff = A380X_FCTL_ROLL_KFF * G, out;
        out = s->roll_int + kp * e + kff * s->p_f;
        if (!((out >= 1.0 && e > 0) || (out <= -1.0 && e < 0)))
            s->roll_int = a380x_fctl_clamp(s->roll_int + ki * e * dt, -0.6, 0.6);
        roll = a380x_fctl_clamp(s->roll_int + kp * e + kff * s->p_f, -1.0, 1.0);
    }
    o->p_cmd = s->p_f;

    /* ---- yaw: damper + turn coordination added to the pedals ---- */
    {
        double r_turn = g_over_v * tan(a380x_fctl_clamp(phi, -60, 60) / A380X_FCTL_R2D) * cos(phi / A380X_FCTL_R2D);
        yaw = a380x_fctl_clamp(pilot_yaw - a380x_fctl_clamp(0.10 * G * (r - r_turn), -0.25, 0.25), -1.0, 1.0);
    }

    /* α floor (A/THR TOGA): between α prot and α max, from liftoff down to 100 ft RA, below M 0.6. */
    a_floor = a_prot + 0.6 * (a_max - a_prot);
    o->alpha_floor = !i->on_ground && (!i->ra_valid || i->ra_ft > 100.0 || s->mode != A380X_FCTL_FLARE) && mach < 0.6 &&
        (s->alpha_f > a_floor || (s->prot_alpha && sp > 0.55));
    if (o->alpha_floor) prot |= A380X_FCTL_PROT_ALPHA_FLOOR;

    /* Blend from direct (ground) into the flight law over the first 5 s. */
    o->pitch = a380x_fctl_clamp((1.0 - s->blend) * pilot_pitch + s->blend * pitch, -1, 1);
    o->roll = a380x_fctl_clamp((1.0 - s->blend) * pilot_roll + s->blend * roll, -1, 1);
    o->yaw = a380x_fctl_clamp((1.0 - s->blend) * pilot_yaw + s->blend * yaw, -1, 1);
    o->artstab_pitch = a380x_fctl_clamp(o->pitch - pilot_pitch, -1.0, 1.0);
    o->artstab_roll = a380x_fctl_clamp(o->roll - pilot_roll, -1.0, 1.0);
    o->artstab_yaw = i->ap_yaw ? 0.0 : a380x_fctl_clamp(o->yaw - pilot_yaw, -1.0, 1.0);
    o->active = 1;
    o->protections = s->prot = prot;
    (void)was_mode;
}

#endif
