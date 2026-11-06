/* SPDX-License-Identifier: GPL-3.0-or-later
 * Approach/landing sub-state of the native FCU guidance (a380x_autoland.inc, docs/AUTOLAND.md).
 *
 * The LAND / FLARE / ROLL OUT / RETARD / SRS GA / GA TRK transitions are a hand port of the
 * FlyByWire A380 PRIM flight-guidance mode logic, A380PrimComputerFg.cpp at aircraft revision
 * 2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2 (GPL-3.0). The guidance laws themselves remain the
 * unmodified vendored A380FgOuterLoops (lateral law LOC TRACK / ROLL OUT, vertical law FLARE / SRS).
 * The PRIM redundancy and monitoring are NOT ported. The landing capability (APPR 1 / LAND 2 /
 * LAND 3 SINGLE / LAND 3 DUAL) is annunciated from the redundancy the simulator actually provides
 * (two NAV receivers, two radio altimeters, two ADIRS sides, AP1/AP2, A/THR, hydraulics, rudder):
 * it is not a certified fail-passive / fail-operational monitor (docs/AUTOLAND.md).
 */
#ifndef A380X_AUTOLAND_H
#define A380X_AUTOLAND_H
#include "a380x_avionics_core.h"

/* Ordered: a lower value is a downgrade. */
typedef enum {
    A380X_LAND_CAPABILITY_NONE = 0,
    A380X_LAND_CAPABILITY_APPR1,         /* APPR selected or LAND engaged, LAND 2 conditions not met */
    A380X_LAND_CAPABILITY_LAND2,         /* one AP, both ILS, an RA, rudder */
    A380X_LAND_CAPABILITY_LAND3_SINGLE,  /* + A/THR active, two agreeing RAs, rollout (yaw) channel */
    A380X_LAND_CAPABILITY_LAND3_DUAL     /* + AP1 and AP2, both ADIRS sides, both hydraulic systems */
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
    int capability;         /* A380XLandCapability, as annunciated (confirmed, frozen below 200 ft) */
    int capability_raw;     /* this frame's conditions, before confirmation and freezing */
    int capability_frozen;  /* LAND below the 200 ft alert height: no downgrade/upgrade annunciated */
    int capability_downgrade;    /* level held 3 s after a confirmed airborne downgrade (triple click) */
    double capability_down_s, capability_downgrade_s;
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

/* FMA D1/D2 cell (FBW FMADefinitions.ts computeD1D2Message tests LAND 2 before LAND 3, so exactly
 * one capability bit is published: the annunciated one). */
static inline A380XD1D2Message a380x_land_capability_d1d2(int capability) {
    switch (capability) {
    case A380X_LAND_CAPABILITY_LAND3_DUAL: return A380X_D1D2_LAND_3_DUAL;
    case A380X_LAND_CAPABILITY_LAND3_SINGLE: return A380X_D1D2_LAND_3_SINGLE;
    case A380X_LAND_CAPABILITY_LAND2: return A380X_D1D2_LAND_2;
    case A380X_LAND_CAPABILITY_APPR1: return A380X_D1D2_APPR_1;
    default: return A380X_D1D2_NONE;
    }
}
/* PRIM FG discrete word 1 bits 27/28/29 and FCDC FG discrete word 1 bits 24/25/26 (LAND 2, LAND 3
 * fail-passive = SINGLE, LAND 3 fail-operational = DUAL): bit offset 0, 1 or 2, or -1 for none. */
static inline int a380x_land_capability_bit(int capability) {
    return capability == A380X_LAND_CAPABILITY_LAND2 ? 0 :
        capability == A380X_LAND_CAPABILITY_LAND3_SINGLE ? 1 :
        capability == A380X_LAND_CAPABILITY_LAND3_DUAL ? 2 : -1;
}

#endif
