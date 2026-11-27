/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 wing flex, ported from FBW (fbw-a380x structural_flex.rs WingFlexA380 + A380WingLiftModifier,
 * fbw-common structural_flex/wing_flex.rs WingLift, FlexPhysicsNG, WingAnimationMapper). Pure C, no XPLM.
 *
 * Each wing is 5 spanwise nodes (0, 11.5, 22.05, 29, 36.85 m) joined by 4 exponential spring-dampers.
 * Lift (weight x load factor in flight, blended out by weight on wheels on the ground) is spread over the
 * nodes (0 / 42 / 40 / 15 / 3 %, less where spoilers or ailerons spoil it, more with flaps); gravity pulls
 * structure and fuel down. Outputs are FBW's WING_FLEX_<side>_<section> bend angles in degrees, which
 * A380_WINGS.xml turns into the four flex clips per wing (frame = angle x gain + offset, 0..100).
 *
 * shortcut: no elevator-lift term, no ground-strike limits on the nodes, no rotational part of the wing
 * root acceleration (FBW reads pitch/roll accelerations and gear contact geometry); add them if
 * touchdowns or hard pitch manoeuvres look wrong.
 */
#ifndef A380X_WING_FLEX_H
#define A380X_WING_FLEX_H
#include <math.h>

#define A380X_WF_NODES 5
#define A380X_WF_LINKS 4

typedef struct { double fuel, speed, pos, sum, offset; } A380XWfNode;
typedef struct { double prev_len, damping; } A380XWfLink;
typedef struct { A380XWfNode n[A380X_WF_NODES]; A380XWfLink l[A380X_WF_LINKS]; double root_accel; } A380XWfWing;

typedef struct {
    A380XWfWing wing[2];                 /* 0 left, 1 right */
    double wow_lift, ground_time, lift;
    float angle_deg[2][A380X_WF_LINKS];  /* inboard, inboard-mid, outboard-mid, outboard */
} A380XWingFlex;

typedef struct {
    double mass_kg, wow_kg;              /* total weight, weight carried by the gear */
    double accel_no_grav;                /* plane-vertical acceleration without gravity, m/s^2 ((Nz - 1) g) */
    double pitch_deg, roll_deg;
    double spoilers, roll, flaps;        /* 0..1 speed brakes, -1..1 roll (+ right), 0..1 flaps */
    double fuel_kg[2][A380X_WF_NODES];   /* fuel carried at each node */
} A380XWingFlexIn;

static const double a380x_wf_x[A380X_WF_NODES] = {0., 11.5, 22.05, 29., 36.85};
static const double a380x_wf_empty[A380X_WF_NODES] = {0., 25000., 20000., 5000., 400.};
static const double a380x_wf_spring[A380X_WF_LINKS] = {16000000., 6000000., 3500000., 200000.};
static const double a380x_wf_damping[A380X_WF_LINKS] = {1600000., 1000000., 300000., 10000.};   /* 2x FBW: user saw the wing ripple (2026-11-26) */
static const double a380x_wf_spread[A380X_WF_NODES] = {0., 0.42, 0.40, 0.15, 0.03};

static inline double a380x_wf_lp(double out, double in, double dt, double tau)   /* FBW LowPassFilter */
{
    return out + (in - out) * (1. - exp(-dt / tau));
}

/* FBW WingAnimationMapper: angle between consecutive (dx, node height) vectors, degrees. */
static inline void a380x_wf_angles(const double h[A380X_WF_NODES], const double x[A380X_WF_NODES],
                                   float out[A380X_WF_LINKS])
{
    double px = 1., py = 0.;
    for (int i = 1; i < A380X_WF_NODES; ++i) {
        double cx = x[i] - x[i - 1], cy = h[i];
        double dot = (px * cx + py * cy) / (hypot(px, py) * hypot(cx, cy));
        double a = acos(fmax(-1., fmin(1., dot))) * 180. / M_PI;
        out[i - 1] = (float)(px * cy - py * cx >= 0. ? a : -a);
        px = cx; py = cy;
    }
}

static inline void a380x_wf_link(A380XWfLink *l, A380XWfNode *root, A380XWfNode *tip, double k, double c, double dt)
{
    double len = (tip->pos + tip->offset) - (root->pos + root->offset);
    double spring = len < 0. ? -(exp(-len) * k - k) * 1.4 : exp(len) * k - k;   /* stiffer downwards */
    l->damping = a380x_wf_lp(l->damping, -((l->prev_len - len) / dt) * c, dt, 0.01);
    l->prev_len = len;
    root->sum += spring + l->damping;
    tip->sum -= spring + l->damping;
}

static inline void a380x_wf_node(A380XWfNode *n, double empty, double gravity_y, double dt)
{
    double m = empty + n->fuel;
    n->sum += gravity_y * m;
    if (empty > 0.) {
        n->speed += n->sum / m * dt;
        n->pos += n->speed * dt;
        if (n->pos < -10. || n->pos > 10.) { n->pos = fmax(-10., fmin(10., n->pos)); n->speed = 0.; }
    }
    n->sum = 0.;
}

static inline void a380x_wing_flex_update(A380XWingFlex *f, const A380XWingFlexIn *in, double dt)
{
    double lift_1g, ground, flight, blend, lateral, wing_lift[2], gravity_y;
    int steps;
    if (!(dt > 0.) || !(in->mass_kg > 1.) || !isfinite(in->mass_kg + in->wow_kg + in->accel_no_grav + in->pitch_deg +
                                                        in->roll_deg + in->spoilers + in->roll + in->flaps)) return;
    if (dt > 0.25) dt = 0.25;
    /* WingLift (elevator term omitted) */
    lift_1g = 9.8 * in->mass_kg;
    f->wow_lift = a380x_wf_lp(f->wow_lift, -9.8 * in->wow_kg, dt, 0.7);
    f->ground_time = in->wow_kg > 1. ? f->ground_time + dt : 0.;
    ground = fmax(lift_1g + f->wow_lift, 0.);
    flight = lift_1g + in->accel_no_grav * in->mass_kg;
    blend = fmax(0., fmin(1., in->wow_kg / (in->mass_kg * 0.7)));
    f->lift = in->wow_kg <= 1. ? flight : f->ground_time < 0.35 ? ground : ground * blend + (1. - blend) * flight;
    /* A380WingLiftModifier: aileron up (+1) and spoilers spoil lift, flaps add it.
     * Deviation from FBW's code, following FBW's own comment ("0.3 means a fully deployed surface reduce lift by
     * 30%"): node factor = base x (1 - gain x surface). FBW's code computes (1 - surface) x gain, which zeroes a
     * node's lift at full deflection, so a held roll unloads the whole outer wing and bends it into a "~".
     * The neutral spread (all surfaces at 0) is unchanged. */
    {
        double roll = fmax(-1., fmin(1., in->roll)), ail[2] = {-roll, roll}, sp = fmax(0., fmin(1., in->spoilers)), fl = fmax(0., fmin(1., in->flaps));
        lateral = 0.25 * (2. * (ail[1] - ail[0])) / 4.;
        wing_lift[0] = 0.5 * (f->lift + lateral * f->lift);
        wing_lift[1] = f->lift - wing_lift[0];
        gravity_y = -9.8 * cos(in->pitch_deg * M_PI / 180.) * cos(in->roll_deg * M_PI / 180.);
        steps = (int)ceil(dt / 0.005);
        for (int w = 0; w < 2; ++w) {
            A380XWfWing *wing = &f->wing[w];
            double factor[A380X_WF_NODES] = {0., 0.4 * (1. - 0.4 * sp) + fl * 0.3, 0.4 * (1. - 0.4 * sp) + fl * 0.3,
                                             0.2 * (1. - 0.2 * ail[w]), 0.2 * (1. - 0.2 * ail[w])}, total = 0., node_lift[A380X_WF_NODES];
            for (int i = 0; i < A380X_WF_NODES; ++i) total += factor[i] * a380x_wf_spread[i];
            for (int i = 0; i < A380X_WF_NODES; ++i)
                node_lift[i] = total > 0. ? factor[i] * a380x_wf_spread[i] / total * wing_lift[w] : 0.;
            for (int s = 0; s < steps; ++s) {
                double h = dt / steps;
                wing->root_accel = a380x_wf_lp(wing->root_accel, in->accel_no_grav, h, 0.05);
                wing->n[0].offset = -0.006 * fmax(-150., fmin(150., wing->root_accel));
                for (int i = 0; i < A380X_WF_NODES; ++i) {
                    wing->n[i].fuel = in->fuel_kg[w][i];
                    wing->n[i].sum += node_lift[i];
                    a380x_wf_node(&wing->n[i], a380x_wf_empty[i], gravity_y, h);
                    if (i < A380X_WF_LINKS)
                        a380x_wf_link(&wing->l[i], &wing->n[i], &wing->n[i + 1], a380x_wf_spring[i], a380x_wf_damping[i], h);
                }
            }
            {
                double heights[A380X_WF_NODES] = {0.};
                for (int i = 1; i < A380X_WF_NODES; ++i) heights[i] = wing->n[i].pos + wing->n[i].offset;
                a380x_wf_angles(heights, a380x_wf_x, f->angle_deg[w]);
            }
        }
    }
}

#endif
