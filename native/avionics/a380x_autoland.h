/* SPDX-License-Identifier: GPL-3.0-or-later
 * Approach/landing sub-state of the native FCU guidance (a380x_autoland.inc, docs/AUTOLAND.md).
 *
 * The LAND / FLARE / ROLL OUT / RETARD / SRS GA / GA TRK transitions are a hand port of the
 * FlyByWire A380 PRIM flight-guidance mode logic, A380PrimComputerFg.cpp at aircraft revision
 * 2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2 (GPL-3.0). The guidance laws themselves remain the
 * unmodified vendored A380FgOuterLoops (lateral law LOC TRACK / ROLL OUT, vertical law FLARE / SRS).
 * The PRIM redundancy, monitoring and certified LAND 3 capability are NOT ported.
 */
#ifndef A380X_AUTOLAND_H
#define A380X_AUTOLAND_H

typedef enum {
    A380X_LAND_CAPABILITY_NONE = 0,
    A380X_LAND_CAPABILITY_APPR1,   /* APPR selected or LAND engaged, LAND 2 conditions not met */
    A380X_LAND_CAPABILITY_LAND2    /* highest level this single-computer port annunciates */
} A380XLandCapability;

typedef enum {
    A380X_BC1_NONE = 0, A380X_BC1_LAND, A380X_BC1_FLARE, A380X_BC1_ROLL_OUT
} A380XBc1Message;

typedef struct {
    /* Mode logic (FBW PRIM FG names in parentheses). */
    int land_active;        /* lateral_modes.land_active: GS TRK + LOC TRK below 400 ft RA */
    int flare_active;       /* longitudinal_modes.flare_active, latched until LAND resets */
    int rollout_active;     /* lateral_modes.rollout_submode_active, latched until LAND resets */
    int align_active;       /* lateral_modes.align_submode_active: LAND below 30 ft RA */
    int retard_active;      /* retard_mode_active: A/THR to idle, FMA IDLE */
    int go_around_active;   /* pitch_goaround_active (SRS) with roll_goaround_active (GA TRK) */
    int autoland;           /* an AP was engaged while LAND was active (AUTOLAND light window) */
    int capability;         /* A380XLandCapability */
    int bc1;                /* A380XBc1Message for the shared B1/C1 FMA cell */
    /* Confirm/trigger timers and edges. */
    double land_confirm_s, flare_confirm_s, rollout_confirm_s, ground_s;
    int previous_toga_count, cancel_request;
    /* Memorized references. */
    int course_valid;
    double course_deg, ga_track_deg, ga_speed_knots;
    /* Imported FgOuterLoops outputs of the last law step. */
    int law_flare_condition;
    double law_beta_c_deg, law_nosewheel_c, law_rudder_storage_deg;
    /* Port-specific yaw actuator (rudder pedal ratio, +1 full right) and the A/THR retard ramp. */
    int yaw_active;
    double yaw_servo, yaw_integral, retard_command, athr_previous, flare_integral;
    double touchdown_s;
} A380XLandingState;

#endif
