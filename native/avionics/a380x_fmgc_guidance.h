/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * A380X FMGC managed guidance (LNAV / VNAV / managed speed producer) for the native FCU guidance
 * (a380x_guidance.c) and the vendored FBW outer loops (a380x_fbw_laws.cpp).
 *
 * This is the "FMS guidance" half of the FlyByWire FMGC (fbw-common/src/systems/fmgc/src/guidance:
 * GuidanceController, LnavDriver, lnav/LegsProcedure, Geometry/transitions, vnav/VnavDriver) at
 * revision 2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2, re-implemented in C for the port's own
 * ARINC 424 flight-plan store (a380x_fms_fpln.h). It is NOT a line-by-line port of the FBW
 * TypeScript: the FBW path vectors, transition classes and the VNAV performance predictor (engine
 * model, wind/temperature integration) are not imported. What follows FBW:
 *   - the FG interface: cross-track error, track-angle error and bank feed-forward fed to the
 *     A380FgOuterLoops HPATH law (fms_xtk_nmi / fms_tae_deg / fms_phi_deg), heading legs flown with
 *     zero XTK and a heading error (LnavDriver ControlLaw.HEADING), the VPATH law profile altitude
 *     and vertical speed (fms_H_c_profile_ft / fms_H_dot_c_profile_ft_min);
 *   - the managed-profile constraint logic (FmcAircraftInterface managedProfile, spec 07 section
 *     5.7 and 6.3) - moved here from the FMS adapter so the closed-loop test shares it;
 *   - approach managed speed = max(VAPP, maneuvering speed of the current flap lever)
 *     (A32NX FMCMainDisplay.updateManagedSpeed / getAppManagedSpeed, spec 07 section 5.7 table).
 * Port models (documented in docs/AUTOPILOT_DEVELOPMENT.md, need in-sim validation):
 *   - fly-by transitions: circular arcs of a bank-limited radius (20 deg design bank at ground
 *     speed) tangent to both legs, anticipation R*tan(turn/2); fly-over legs sequence at the fix;
 *   - DF legs: pursuit of the fix while turning, then a frozen great-circle line;
 *   - course legs (CA/CD/CI/CR, VA/VD/VI/VR/VM): track / heading hold, no XTK;
 *   - holds: racetrack (inbound leg, 180 deg turns of the hold radius), entered directly;
 *   - descent path: built backwards from the destination with a constant idle gradient (3.0 deg
 *     below / 3.1 deg above the speed-limit altitude), coded FPA legs as geometric segments,
 *     altitude constraints as clamps, a decelerating segment at the speed-limit altitude.
 *
 * Simulator independent C11, no XPLM, no heap. Not thread safe.
 *
 * Sign conventions (FBW A380FgOuterLoops HPATH law, verified by the closed-loop test):
 *   fbw_xtk_nm   > 0 : the path lies to the RIGHT of the aircraft (aircraft left of path);
 *   fbw_tae_deg  > 0 : the desired track lies to the RIGHT of the current track;
 *   bank_ff_deg  > 0 : right-wing-down feed-forward (right turn).
 * xtk_right_nm (display) is the opposite of fbw_xtk_nm: > 0 when the aircraft is right of the path.
 */
#ifndef A380X_FMGC_GUIDANCE_H
#define A380X_FMGC_GUIDANCE_H

#include "a380x_fms_fpln.h"

#ifdef __cplusplus
extern "C" {
#endif

#define A380X_FMGC_DESIGN_BANK_DEG   20.0   /* transition radius design bank */
#define A380X_FMGC_MAX_PROFILE      (A380X_FPLN_MAX_ELEMENTS + 4)

/* Lateral path element kinds of the active guidance segment. */
typedef enum {
    A380X_LNAV_NONE = 0,
    A380X_LNAV_LINE,          /* great circle start -> end (TF, CF, frozen DF, FA/FC/FD/FM, hold legs) */
    A380X_LNAV_ARC,           /* RF / AF / fly-by transition / hold turn */
    A380X_LNAV_TRACK,         /* course legs: hold a true track */
    A380X_LNAV_HEADING,       /* heading legs: hold a true heading */
    A380X_LNAV_DIRECT         /* DF pursuit of the fix (while turning onto it) */
} A380XLnavKind;

typedef struct {
    /* aircraft state */
    int    position_valid;
    double lat_deg, lon_deg;
    double alt_ft;                  /* baro-corrected indicated altitude */
    double vs_fpm, gs_kt, tas_kt, cas_kt, mach;
    double true_track_deg, true_heading_deg;
    double magvar_deg;              /* east positive: magnetic = true - magvar */
    int    on_ground;
    double radio_alt_ft;
    /* FMS (a380x_fms.h outputs and perf data) */
    int    fms_valid, phase;        /* A380XFmsPhase */
    double cruise_alt_ft;           /* <= 0 none */
    double fcu_alt_ft;              /* FCU selected altitude */
    double managed_speed_kt;        /* FMS managed speed target (CAS); 0 = none */
    int    managed_is_mach;
    double managed_mach;
    double descent_speed_kt, descent_mach;   /* managed descent law; 0 = none */
    double des_limit_kt, des_limit_alt_ft;   /* descent speed limit; 0 = none */
    double green_dot_kt, s_speed_kt, f_speed_kt, vapp_kt, vls_kt;  /* 0 = none */
    int    flap_lever;              /* 0..4 */
    int    landing_conf3;
    double static_pressure_hpa;     /* for Mach targets; <= 0 unknown */
    /* FG state (previous frame) */
    int    nav_engaged;             /* lateral mode NAV */
    int    nav_armed;
    int    vertical_managed;        /* CLB, DES, ALT CST engaged */
} A380XFmgcInputs;

typedef struct {
    double dist_to_dest_nm;         /* distance before the destination point */
    double alt_ft;                  /* path altitude at that point */
    int    leg_index;               /* leg whose termination is at this point */
    int    segment;                 /* 0 idle, 1 geometric (shallower than idle), 2 too steep, 3 decel */
} A380XFmgcProfilePoint;

typedef struct {
    /* ---- LNAV ---- */
    int    lnav_valid;              /* a path (or course/heading) exists for the active leg */
    int    nav_capture;             /* 1: NAV may engage now (inside the capture zone) */
    int    discontinuity;           /* the TO leg is finished and followed by a discontinuity */
    int    manual_leg;              /* FM / VM leg active (no automatic sequencing) */
    A380XLnavKind kind;
    double fbw_xtk_nm, fbw_tae_deg, bank_ff_deg;  /* FBW HPATH law inputs (see header) */
    double xtk_right_nm;            /* display convention */
    double desired_track_true_deg;  /* NAN for heading legs */
    double managed_heading_mag_deg; /* heading legs, else the desired track (magnetic) */
    int    in_transition;           /* flying a fly-by transition arc */
    int    active_leg;              /* index of the TO leg in ACTIVE.pri, -1 */
    int    sequenced;               /* the TO leg advanced this frame */
    double dtg_active_nm;           /* distance to go to the active leg termination, NAN */
    double turn_radius_nm;
    /* ---- VNAV ---- */
    int    vnav_valid;              /* descent profile computed */
    double dist_to_dest_nm;         /* along the plan from PPOS, NAN */
    double path_alt_ft;             /* descent path altitude at PPOS (extended above cruise) */
    double path_vs_fpm;             /* path vertical speed at the current ground speed */
    double vdev_ft;                 /* altitude - path altitude (+ above path) */
    int    path_segment;            /* segment kind at PPOS (A380XFmgcProfilePoint.segment) */
    double tod_dist_nm;             /* distance-to-destination of the T/D, NAN */
    int    before_tod;              /* PPOS is before the T/D (cruise part) */
    double decel_dist_nm;           /* distance-to-destination of the DECEL point, NAN */
    int    decel_reached;           /* approach phase activation (L:A32NX_FM_ENABLE_APPROACH_PHASE) */
    double speed_limit_dist_nm;     /* distance-to-destination where the speed-limit decel begins */
    double managed_altitude_ft;     /* CLB: min(FCU, next climb constraint); DES: max(FCU, floor) */
    int    managed_altitude_is_constraint;
    double toc_dist_from_ppos_nm;   /* T/C estimate (current climb rate), NAN */
    /* ---- managed speed (CAS kt); valid = 0 leaves the FMS target unchanged ---- */
    int    speed_valid;
    double speed_kt;
    int    speed_is_mach;
    double speed_mach;
    /* profile (for the MFD predictions / ND pseudo waypoints) */
    int    profile_count;
    A380XFmgcProfilePoint profile[A380X_FMGC_MAX_PROFILE];
    double tod_lat, tod_lon, decel_lat, decel_lon, toc_lat, toc_lon;  /* NAN when unknown */
    int    tod_leg, decel_leg, toc_leg; /* leg index the pseudo waypoint lies on (-1) */
} A380XFmgcOutputs;

typedef struct {
    /* leg tracking */
    int      leg;                   /* active leg the state below belongs to */
    int      leg_type;              /* identity of that leg (an edit / DIR TO replaces it) */
    char     leg_ident[A380X_FPLN_IDENT_LEN];
    double   leg_lat, leg_lon;
    int      have_prev;
    double   prev_track_true;       /* inbound track at the previous termination */
    /* fly-by transition arc into the active leg */
    int      arc_active;
    double   arc_clat, arc_clon, arc_radius_nm, arc_start_brg, arc_sweep_deg;
    int      arc_dir;               /* +1 right / -1 left */
    /* DF */
    int      df_frozen;
    double   df_lat, df_lon;
    /* DME / radial legs: crossing direction at activation */
    int      cross_sign;
    /* hold */
    int      hold_phase;            /* 0 inbound, 1 turn at fix, 2 outbound, 3 turn back */
    int      hold_entered, hold_circuits;
    double   hold_radius_nm, hold_leg_nm;
    /* FMS phase / decel latch */
    int      decel_latched;
    double   timer;
    A380XFmgcOutputs out;
} A380XFmgc;

void a380x_fmgc_init(A380XFmgc *g);
/* Every guidance frame, after the FMS cycle. fp may be NULL (no plan: outputs invalid).
 * Sequences the ACTIVE plan (a380x_fpln_sequence_to) when the TO leg is complete. */
void a380x_fmgc_update(A380XFmgc *g, A380XFlightPlans *fp, const A380XFmgcInputs *in, double dt);
const A380XFmgcOutputs *a380x_fmgc_outputs(const A380XFmgc *g);

/* ---- managed profile of the ACTIVE plan (FAI.ts:1459-1603, spec 07 sections 5.7 / 6.3) ----
 * Shared by the FMS plan hooks of the X-Plane adapter and the offline tests. */
/* Lowest CLB altitude constraint of a plan before the missed approach, NAN if none. */
double a380x_fmgc_lowest_climb_constraint(const A380XFlightPlans *fp, int plan);
/* Speed constraint for the FMGC phase (CLIMB/GOAROUND: climb, DESCENT: descent with decelDist),
 * NAN if none. ac supplies the distance to the active leg termination. */
double a380x_fmgc_speed_constraint(const A380XFlightPlans *fp, const A380XFplnAircraft *ac, int phase);
/* climbAltitude / descentAltitude of the active leg; returns 0 without an active leg. */
int    a380x_fmgc_constraint_altitudes(const A380XFlightPlans *fp, double *climb_alt_ft, double *descent_alt_ft);

/* ---- pure geometry helpers (exported for the unit tests) ---- */
/* Bank-limited turn radius at a ground speed, NM. */
double a380x_fmgc_turn_radius_nm(double gs_kt, double bank_deg);
/* Fly-by anticipation distance for a track change, NM (0 for a fly-over). */
double a380x_fmgc_anticipation_nm(double gs_kt, double track_change_deg);
/* Idle descent gradient of the port's profile model, ft per NM. */
double a380x_fmgc_idle_gradient_ft_nm(double alt_ft, double speed_limit_alt_ft);

/* Basic F-PLN predictions for the MFD (A380XFplnPredFeed): pseudo waypoints T/C, T/D, DECEL and
 * per-leg distance/time/altitude from the profile. legs must hold A380X_FPLN_MAX_ELEMENTS entries. */
void a380x_fmgc_predictions(const A380XFmgc *g, const A380XFlightPlans *fp, const A380XFmgcInputs *in,
                            A380XFplnPredFeed *feed, A380XFplnWptPred *legs);

#ifdef __cplusplus
}
#endif

#endif
