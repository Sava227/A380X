/* SPDX-License-Identifier: GPL-3.0-or-later
 * Offline test of the A380 autobrake core (a380x_autobrake.h): mode logic, engagement, disarm, selector solenoid,
 * DECEL, and the governor closed loop on a simple braking model (brake 0..1 -> up to 6.3 m/s^2 through a 0.25 s
 * hydraulic lag, plus 0.3 m/s^2 of drag).
 */
#include <assert.h>
#include <stdio.h>
#include "a380x_autobrake.h"

#define DT 0.02
typedef struct { A380XAutobrake ab; A380XAutobrakeInputs in; double v, accel, pressure; } Sim;

static void sim_init(Sim *s, int in_flight, double speed_ms)
{
    memset(s, 0, sizeof(*s));
    a380x_ab_init(&s->ab, in_flight);
    s->in.arming_allowed = 1; s->in.main_compressed = !in_flight; s->in.nose_compressed = !in_flight;
    s->v = speed_ms;
}
static void step(Sim *s, int on_runway)
{
    a380x_ab_step(&s->ab, &s->in, DT);
    s->in.knob_moved = s->in.rto_pressed = s->in.athr_disconnect = 0;
    if (s->ab.knob_return) s->in.knob = A380X_AB_DISARM;          /* solenoid: not a crew selection */
    s->pressure += (s->ab.brake - s->pressure) * DT / (0.25 + DT);
    s->accel = on_runway && s->v > 0.0 ? -(6.3 * s->pressure) - 0.3 : 0.0;
    s->v += s->accel * DT; if (s->v < 0.0) s->v = 0.0;
    s->in.long_accel = s->accel;
}
static void run(Sim *s, double secs, int on_runway) { int i, n = (int)(secs / DT + 0.5); for (i = 0; i < n; ++i) step(s, on_runway); }
static void select_knob(Sim *s, int knob) { s->in.knob = knob; s->in.knob_moved = 1; }

static void landing(int knob, double expect, const char *name)
{
    Sim s;
    double settled;
    sim_init(&s, 1, 72.0);
    select_knob(&s, knob); run(&s, 1.0, 0);
    assert(s.ab.mode == knob && !s.ab.engaged);
    s.in.main_compressed = 1; run(&s, 0.6, 1);                     /* main gear touchdown */
    s.in.ground_spoilers_out = 1; run(&s, 1.0, 1);                 /* spoilers out, nose still up */
    assert(!s.ab.engaged && s.ab.brake == 0.0);
    s.in.nose_compressed = 1; run(&s, 0.06, 1);                    /* nose down: engagement */
    assert(s.ab.engaged && s.ab.mode == knob);
    run(&s, 5.0, 1);
    settled = s.in.long_accel;
    printf("%s: decel after 5 s %.2f m/s^2 (target %.1f), brake %.2f, DECEL %d, active %d\n", name, settled, expect,
           s.ab.brake, s.ab.decel_light, a380x_ab_active(&s.ab));
    assert(fabs(settled - expect) < 0.15 && s.ab.decel_light && a380x_ab_active(&s.ab));
    s.in.pedal_left = 0.6; run(&s, 0.04, 1);                       /* pilot takes over */
    assert(s.ab.mode == A380X_AB_DISARM && !s.ab.engaged && s.ab.brake == 0.0 && !s.ab.decel_light);
    s.in.pedal_left = 0.0; run(&s, 1.1, 1);
    assert(s.in.knob == A380X_AB_DISARM);                          /* selector back to DISARM */
}

int main(void)
{
    Sim s;
    landing(A380X_AB_LO, -2.0, "LO");
    landing(A380X_AB_L2, -2.5, "2");
    landing(A380X_AB_L3, -3.0, "3");
    landing(A380X_AB_HI, -3.5, "HI");

    /* Ramp: LO no braking during the first 0.1 s, HI steps to 2 m/s^2 first. */
    sim_init(&s, 1, 70.0); select_knob(&s, A380X_AB_HI); run(&s, 0.1, 0);
    s.in.main_compressed = 1; s.in.ground_spoilers_out = 1; run(&s, 0.04, 1);
    assert(!s.ab.engaged);
    s.in.nose_compressed = 1; run(&s, 0.04, 1); assert(s.ab.engaged && s.ab.target > -2.0 - 1e-9);
    run(&s, 0.1, 1); assert(s.ab.target < -2.0 && s.ab.target > -2.2);
    run(&s, 2.5, 1); assert(fabs(s.ab.target + 3.5) < 1e-9);
    printf("HI ramp: target %.2f after 2.6 s\n", s.ab.target);

    /* Nose kept up: engagement after the ground spoilers have been out for 5 s. */
    sim_init(&s, 1, 70.0); select_knob(&s, A380X_AB_L3); run(&s, 0.1, 0);
    s.in.main_compressed = 1; s.in.ground_spoilers_out = 1; run(&s, 4.9, 1);
    assert(!s.ab.engaged);
    run(&s, 0.2, 1); assert(s.ab.engaged);
    printf("nose up: engaged after 5 s of ground spoilers\n");

    /* A/THR instinctive disconnect disarms a landing mode while braking. */
    s.in.athr_disconnect = 1; run(&s, 0.04, 1); assert(s.ab.mode == A380X_AB_DISARM && s.ab.brake == 0.0);
    printf("A/THR disconnect: disarmed\n");

    /* Ground spoilers retracting disarms. */
    sim_init(&s, 1, 60.0); select_knob(&s, A380X_AB_LO); run(&s, 0.1, 0);
    s.in.main_compressed = s.in.nose_compressed = s.in.ground_spoilers_out = 1; run(&s, 2.0, 1);
    assert(s.ab.engaged);
    s.in.ground_spoilers_out = 0; run(&s, 0.04, 1); assert(s.ab.mode == A380X_AB_DISARM);
    printf("spoilers retracted: disarmed\n");

    /* RTO: armed on the ground, engages with the ground spoilers (nose on the ground), 6 m/s^2, DECEL > 2.7. */
    sim_init(&s, 0, 0.0);
    s.in.rto_pressed = 1; run(&s, 0.04, 1); assert(s.ab.mode == A380X_AB_RTO);
    s.v = 70.0; run(&s, 1.0, 1); assert(!s.ab.engaged);
    s.in.ground_spoilers_out = 1; run(&s, 3.0, 1);
    printf("RTO: decel %.2f m/s^2, DECEL %d\n", s.in.long_accel, s.ab.decel_light);
    assert(s.ab.engaged && s.ab.target == -6.0 && s.in.long_accel < -5.6 && s.ab.decel_light);
    s.in.athr_disconnect = 1; run(&s, 0.04, 1); assert(s.ab.mode == A380X_AB_RTO);   /* not for RTO */
    s.in.pedal_left = 0.6; run(&s, 0.04, 1); assert(s.ab.mode == A380X_AB_RTO);      /* RTO pedal threshold 77 % */
    s.in.pedal_left = 0.8; run(&s, 0.04, 1); assert(s.ab.mode == A380X_AB_DISARM);
    printf("RTO: kept at 60 %% pedal and A/THR disconnect, disarmed at 80 %%\n");

    /* RTO pushbutton again: RTO off. RTO with the knob away from DISARM: the solenoid returns the knob, RTO stays. */
    sim_init(&s, 0, 0.0);
    s.in.rto_pressed = 1; run(&s, 0.04, 1); s.in.rto_pressed = 1; run(&s, 0.04, 1);
    assert(s.ab.mode == A380X_AB_DISARM);
    select_knob(&s, A380X_AB_LO); run(&s, 0.04, 1); assert(s.ab.mode == A380X_AB_LO);
    s.in.rto_pressed = 1; run(&s, 0.04, 1); assert(s.ab.mode == A380X_AB_RTO);
    run(&s, 1.2, 1); assert(s.in.knob == A380X_AB_DISARM && s.ab.mode == A380X_AB_RTO);
    printf("RTO armed with the knob at LO: knob back to DISARM after 1 s, RTO kept\n");

    /* Take-off with RTO armed: rejected 10 s after lift-off. A landing mode armed on the ground: disarmed then too. */
    s.in.main_compressed = s.in.nose_compressed = 0; run(&s, 9.9, 0); assert(s.ab.mode == A380X_AB_RTO);
    run(&s, 0.2, 0); assert(s.ab.mode == A380X_AB_DISARM);
    sim_init(&s, 0, 0.0); select_knob(&s, A380X_AB_L2); run(&s, 0.04, 1); assert(s.ab.mode == A380X_AB_L2);
    s.in.main_compressed = s.in.nose_compressed = 0; run(&s, 10.1, 0); assert(s.ab.mode == A380X_AB_DISARM);
    select_knob(&s, A380X_AB_L2); run(&s, 20.0, 0); assert(s.ab.mode == A380X_AB_L2);      /* re-armed in flight: kept */
    s.in.rto_pressed = 1; run(&s, 0.04, 0); assert(s.ab.mode == A380X_AB_L2);          /* no RTO in flight */
    printf("10 s after lift-off: RTO and ground-armed modes disarmed; re-armed in flight kept; RTO refused in flight\n");

    /* BTV without an OANS exit cannot arm: DISARM, selector back after 1 s. With an exit it arms (scheduler not ported). */
    sim_init(&s, 1, 70.0); select_knob(&s, A380X_AB_BTV); run(&s, 0.04, 0);
    assert(s.ab.mode == A380X_AB_DISARM); run(&s, 1.1, 0); assert(s.in.knob == A380X_AB_DISARM);
    s.in.btv_armable = 1; select_knob(&s, A380X_AB_BTV); run(&s, 1.5, 0); assert(s.ab.mode == A380X_AB_BTV && s.in.knob == A380X_AB_BTV);
    printf("BTV: no exit -> DISARM; with an exit -> armed\n");

    /* BCU arming lost (anti-skid off, brakes, ADIRS): disarm. */
    sim_init(&s, 1, 70.0); select_knob(&s, A380X_AB_HI); run(&s, 0.04, 0); s.in.arming_allowed = 0; run(&s, 0.04, 0);
    assert(s.ab.mode == A380X_AB_DISARM);
    printf("arming lost: disarmed\n");
    puts("autobrake core tests passed");
    return 0;
}
