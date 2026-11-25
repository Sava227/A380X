/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 autobrake (brake control unit autobrake function): XPLM-free core. X-Plane side: a380x_center_right.inc.
 *
 * Translation of FlyByWire's A380AutobrakeController (fbw-a380x/src/wasm/systems/a380_systems/src/hydraulic/
 * autobrakes.rs), its AutobrakeDecelerationGovernor (fbw-common/.../hydraulic/brake_circuit.rs) and PidController
 * (fbw-common/.../shared/pid.rs), FBW revision pinned in docs/ATTRIBUTION.md:
 *
 *  - Modes: DISARM, BTV, LO, L2 ("2"), L3 ("3"), HI (AUTO BRK selector) and RTO (RTO pushbutton).
 *  - Engagement: armed, ground spoilers extended, and either the nose gear has touched down (since arming) or the
 *    ground spoilers have been out for 5 s.
 *  - Deceleration targets (m/s^2, ramped from engagement over 0.1 / 2.5 s): LO 2, L2 2.5, L3 3, HI 3.5 (HI steps to
 *    2 at 0.1 s first); RTO 6. A PI governor (Kp 0.08, Ki 0.6, velocity form, output 0..1 brake) tracks the target
 *    on the measured longitudinal deceleration (0.1 s low-pass).
 *  - DECEL: landing modes on at 80 % of the target, off below 70 %; RTO on beyond 2.7 m/s^2, off below 2.0.
 *  - Disarm: pedals while braking (LO..HI/BTV: one pedal > 53 % or both > 11 %; RTO: one > 77 % or both > 53 %),
 *    A/THR instinctive disconnect while braking (not RTO), BCU arming lost (anti-skid, normal brakes, ADIRS,
 *    power), ground spoilers retracting, 10 s after lift-off (a mode armed on the ground), RTO more than 10 s in
 *    flight, BTV that cannot arm. The selector solenoid returns the knob to DISARM 1 s after the mode is DISARM or
 *    RTO; that return is not a crew selection (FBW's panel would read it as one and cancel an RTO armed with the
 *    knob away from DISARM).
 *  - BTV needs a runway and exit selected on the OANS; the port has no exit selection yet, so BTV does not arm
 *    (btv_armable 0): as in FBW, the selector then springs back to DISARM.
 * Runway overrun protection (ROW/ROP) and the BTV scheduler are not ported.
 */
#ifndef A380X_AUTOBRAKE_H
#define A380X_AUTOBRAKE_H
#include <math.h>
#include <string.h>

enum { A380X_AB_DISARM = 0, A380X_AB_BTV, A380X_AB_LO, A380X_AB_L2, A380X_AB_L3, A380X_AB_HI, A380X_AB_RTO };

#define A380X_AB_FLIGHT_DISARM_S 10.0      /* DURATION_OF_FLIGHT_TO_DISARM_AUTOBRAKE */
#define A380X_AB_SPOILERS_ARM_S 5.0        /* DURATION_OF_GROUND_SPOILERS_BEFORE_ARMING */
#define A380X_AB_KNOB_RETURN_S 1.0         /* KNOB_SOLENOID_DISARM_DELAY */
#define A380X_AB_RTO_TARGET (-6.0)
#define A380X_AB_OFF_TARGET 5.0
#define A380X_AB_KP 0.08
#define A380X_AB_KI 0.6
#define A380X_AB_ACCEL_TAU_S 0.1

typedef struct {
    int knob;                  /* selector: 0 DISARM, 1 BTV, 2 LO, 3 L2 (2), 4 L3 (3), 5 HI */
    int knob_moved;            /* the crew moved the selector since the last step */
    int rto_pressed;           /* RTO pushbutton pressed since the last step */
    int arming_allowed;        /* BCU: anti-skid, normal braking, ADIRS, power */
    int ground_spoilers_out;   /* PRIM ground spoiler extension */
    int nose_compressed;       /* nose gear on the ground */
    int main_compressed;       /* any main gear on the ground (both LGCIUs "in flight" = none) */
    double pedal_left, pedal_right;   /* 0..1 */
    int athr_disconnect;       /* A/THR instinctive disconnect pressed since the last step */
    double long_accel;         /* m/s^2 along the track, negative when decelerating */
    int btv_armable;           /* BTV scheduler armed (runway + exit selected): 0 in the port */
} A380XAutobrakeInputs;

typedef struct {
    int mode;
    /* deceleration governor */
    int engaged;
    double time_engaged, accel_filtered, pid_output, error_k1;
    int error_k1_valid;
    double target;
    double brake;              /* 0..1 brake demand */
    int decel_light;
    /* logic gates */
    int spoilers_out, last_spoilers_out;
    double spoilers_out_time;
    double flight_time;
    int flight_pulse_fired;
    double knob_return_time;
    int knob_return;           /* selector solenoid: return the knob to DISARM */
    int nose_once;
    int filter_primed;
} A380XAutobrake;

/* Start: on the ground (in_flight 0) or in flight (no 10 s disarm pulse, RTO rejected at once). */
static void a380x_ab_init(A380XAutobrake *a, int in_flight)
{
    memset(a, 0, sizeof(*a));
    a->mode = A380X_AB_DISARM;
    a->target = A380X_AB_OFF_TARGET;
    if (in_flight) { a->flight_time = A380X_AB_FLIGHT_DISARM_S; a->flight_pulse_fired = 1; }
}

static double a380x_ab_interp(const double *x, const double *y, int n, double v)
{
    int i;
    if (v <= x[0]) return y[0];
    for (i = 1; i < n; ++i)
        if (v <= x[i]) return y[i - 1] + (y[i] - y[i - 1]) * (v - x[i - 1]) / (x[i] - x[i - 1]);
    return y[n - 1];
}

static double a380x_ab_target(const A380XAutobrake *a)
{
    static const double t[3] = {0.0, 0.1, 2.5};
    static const double lo[3] = {4.0, 0.0, -2.0}, l2[3] = {4.0, 0.0, -2.5}, l3[3] = {4.0, 0.0, -3.0},
                        hi[3] = {4.0, -2.0, -3.5};
    switch (a->mode) {
    case A380X_AB_LO: return a380x_ab_interp(t, lo, 3, a->time_engaged);
    case A380X_AB_L2: return a380x_ab_interp(t, l2, 3, a->time_engaged);
    case A380X_AB_L3: return a380x_ab_interp(t, l3, 3, a->time_engaged);
    case A380X_AB_HI: return a380x_ab_interp(t, hi, 3, a->time_engaged);
    case A380X_AB_RTO: return A380X_AB_RTO_TARGET;
    case A380X_AB_BTV: return A380X_AB_OFF_TARGET;     /* BTV scheduler not ported */
    default: return A380X_AB_OFF_TARGET;
    }
}

static int a380x_ab_pedal_disarm(const A380XAutobrake *a, const A380XAutobrakeInputs *in)
{
    double l = in->pedal_left, r = in->pedal_right;
    if (a->mode == A380X_AB_DISARM) return 0;
    if (a->mode == A380X_AB_RTO) return l > 0.77 || r > 0.77 || (l > 0.53 && r > 0.53);
    return l > 0.53 || r > 0.53 || (l > 0.11 && r > 0.11);
}

static int a380x_ab_should_disarm(const A380XAutobrake *a, const A380XAutobrakeInputs *in, int flight_pulse)
{
    int rto_rejected = !in->main_compressed && a->flight_time >= A380X_AB_FLIGHT_DISARM_S;
    return (a->engaged && a380x_ab_pedal_disarm(a, in))
        || (a->engaged && in->athr_disconnect && a->mode != A380X_AB_RTO)
        || !in->arming_allowed
        || (!a->spoilers_out && a->last_spoilers_out)                 /* ground spoilers retracted */
        || flight_pulse
        || (a->mode == A380X_AB_RTO && rto_rejected)
        || (a->mode == A380X_AB_BTV && !in->btv_armable);
}

/* One BCU step. */
static void a380x_ab_step(A380XAutobrake *a, const A380XAutobrakeInputs *in, double dt)
{
    int in_flight = !in->main_compressed, flight_pulse = 0, rto_disable, rto_rejected, engage;
    if (!(dt >= 0.0) || !isfinite(dt)) dt = 0.0;
    /* inputs */
    a->last_spoilers_out = a->spoilers_out;
    a->spoilers_out = in->ground_spoilers_out != 0;
    a->spoilers_out_time = a->spoilers_out ? a->spoilers_out_time + dt : 0.0;
    a->nose_once = a->nose_once || in->nose_compressed;
    if (in_flight) {
        double before = a->flight_time;
        a->flight_time += dt;
        if (before < A380X_AB_FLIGHT_DISARM_S && a->flight_time >= A380X_AB_FLIGHT_DISARM_S && !a->flight_pulse_fired) {
            flight_pulse = 1; a->flight_pulse_fired = 1;
        }
    } else {
        a->flight_time = 0.0; a->flight_pulse_fired = 0;
    }
    rto_rejected = in_flight && a->flight_time >= A380X_AB_FLIGHT_DISARM_S;
    /* mode */
    rto_disable = a->mode == A380X_AB_RTO && in->rto_pressed;
    if (a->mode != A380X_AB_RTO && in->rto_pressed && !rto_rejected) a->mode = A380X_AB_RTO;
    else if (in->knob_moved) a->mode = in->knob >= A380X_AB_DISARM && in->knob <= A380X_AB_HI ? in->knob : A380X_AB_DISARM;
    if (rto_disable || a380x_ab_should_disarm(a, in, flight_pulse)) { a->mode = A380X_AB_DISARM; a->nose_once = 0; }
    /* selector solenoid */
    if ((a->mode == A380X_AB_DISARM || a->mode == A380X_AB_RTO) && in->knob != A380X_AB_DISARM)
        a->knob_return_time += dt;
    else
        a->knob_return_time = 0.0;
    a->knob_return = a->knob_return_time >= A380X_AB_KNOB_RETURN_S && in->knob != A380X_AB_DISARM;
    /* deceleration governor */
    engage = a->mode != A380X_AB_DISARM && a->spoilers_out
          && (a->spoilers_out_time >= A380X_AB_SPOILERS_ARM_S || a->nose_once)
          && !a380x_ab_should_disarm(a, in, 0);
    if (engage) a->engaged = 1;
    else { a->engaged = 0; a->time_engaged = 0.0; a->pid_output = 0.0; a->error_k1_valid = 0; }
    a->target = a380x_ab_target(a);
    if (!a->filter_primed) { a->accel_filtered = isfinite(in->long_accel) ? in->long_accel : 0.0; a->filter_primed = 1; }
    else if (isfinite(in->long_accel) && dt > 0.0)
        a->accel_filtered += (in->long_accel - a->accel_filtered) * dt / (A380X_AB_ACCEL_TAU_S + dt);
    if (a->engaged) {
        double error, out;
        a->time_engaged += dt;          /* FBW: the target was computed before the governor's time advanced */
        if (dt > 0.0) {
            error = a->target - a->accel_filtered;
            out = a->pid_output + (error - (a->error_k1_valid ? a->error_k1 : 0.0)) * A380X_AB_KP + error * A380X_AB_KI * dt;
            a->pid_output = out < -1.0 ? -1.0 : out > 0.0 ? 0.0 : out;
            a->error_k1 = error; a->error_k1_valid = 1;
        }
        a->brake = -a->pid_output;
    } else {
        a->brake = 0.0;
    }
    /* DECEL */
    if (!(a->engaged && a->target < 0.0) || a->mode == A380X_AB_DISARM) a->decel_light = 0;
    else if (a->mode == A380X_AB_RTO) {
        if (a->accel_filtered < -2.7) a->decel_light = 1;
        else if (!(a->accel_filtered < -2.0)) a->decel_light = 0;
    } else {
        if (a->accel_filtered < a->target * 0.8) a->decel_light = 1;
        else if (!(a->accel_filtered < a->target * 0.7)) a->decel_light = 0;
    }
}

/* Braking demanded (FBW AUTOBRAKES_ACTIVE): the FMA shows the mode green. */
static int a380x_ab_active(const A380XAutobrake *a) { return a->engaged && a->target < 0.0; }

#endif
