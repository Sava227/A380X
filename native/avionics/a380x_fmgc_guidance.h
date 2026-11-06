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
 *   - holds: racetrack (inbound leg, 180 deg turns of the hold radius) with direct, teardrop and
 *     parallel entries chosen by the ICAO / Airbus entry sectors (+-5 deg in favour of direct); hold
 *     speed as FBW's getHoldingSpeed (spec 07 section 5.8);
 *   - descent path: built backwards from the destination with an idle gradient (3.0 deg nominal,
 *     scaled to ground distance with the current TAS/GS ratio and by an idle factor learned while
 *     descending at idle with the speed on the elevator, 0.7..1.3), coded FPA legs as geometric
 *     segments, altitude constraints as clamps, decelerating segments (1/3 of the idle gradient) at
 *     the speed-limit altitude and before descent speed constraints (FBW decelDist).
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
#define A380X_FMGC_MAX_ND_PWP        10

/* ND pseudo-waypoint symbols: the FBW ND PseudoWaypointLayer `typePwp` flags (NdPwpSymbolTypeFlags
 * as used by the pinned FBW A380X ND bundle) and colours (NdSymbolTypeFlags MagentaColor 131072,
 * AmberColor 65536, CyanColor 32768, else white). */
enum {
    A380X_PWP_DECEL = 2, A380X_PWP_START_OF_DESCENT = 4, A380X_PWP_SPEED_CHANGE = 8,
    A380X_PWP_CLIMB_LEVEL_OFF = 16, A380X_PWP_DESCENT_LEVEL_OFF = 32, A380X_PWP_START_OF_CLIMB = 64,
    A380X_PWP_INTERCEPT_PROFILE = 128
};
typedef enum { A380X_PWP_WHITE = 0, A380X_PWP_MAGENTA, A380X_PWP_CYAN, A380X_PWP_AMBER } A380XPwpColour;
typedef enum {
    A380X_PWP_ND_NONE = 0,          /* MFD F-PLN only */
    A380X_PWP_ND_PATH,              /* on the flight-plan path at lat/lon (NAV engaged) */
    A380X_PWP_ND_TRACK              /* along the present track, dist_ahead_nm (HDG / TRK) */
} A380XPwpNdPlacement;
typedef struct {
    char ident[12];                 /* MFD label "(T/C)", "(T/D)", "(DECEL)", "(SPDLIM)"; ND only:
                                       "LEVEL", "START", "INTCPT" */
    unsigned symbol;                /* A380X_PWP_* (one flag) */
    A380XPwpColour colour;
    A380XPwpNdPlacement nd;
    int mfd;                        /* shown on the MFD F-PLN (ACTIVE) */
    double lat, lon;                /* NAN when not on the path */
    double dist_ahead_nm;           /* along the path (or track) from PPOS */
    double alt_ft, speed_kt;        /* predictions (NAN) */
    int leg_index;                  /* ACTIVE leg it lies on, -1 */
    double before_term_nm;          /* distance before that leg's termination */
} A380XFmgcNdPwp;

/* FG vertical mode classes for the pseudo waypoints (the adapter maps the FMA B1 message). */
typedef enum {
    A380X_FMGC_VM_OTHER = 0, A380X_FMGC_VM_SRS, A380X_FMGC_VM_CLB, A380X_FMGC_VM_OP_CLB,
    A380X_FMGC_VM_DES, A380X_FMGC_VM_OP_DES, A380X_FMGC_VM_VS_FPA, A380X_FMGC_VM_ALT, A380X_FMGC_VM_APPROACH
} A380XFmgcVertMode;

/* Hold entry types (ICAO PANS-OPS Vol I / Airbus FCOM sectors). */
typedef enum { A380X_HOLD_ENTRY_NONE = 0, A380X_HOLD_ENTRY_DIRECT, A380X_HOLD_ENTRY_TEARDROP, A380X_HOLD_ENTRY_PARALLEL } A380XHoldEntry;

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
    int    idle_speed_descent;      /* OP DES / DES above path at THR IDLE, speed on target (idle learning) */
    /* FG modes for the ND pseudo-waypoint symbols (managed magenta / selected cyan / armed) */
    int    fg_vertical_mode;        /* A380XFmgcVertMode (ALT: ALT, ALT*, ALT CST, ALT CRZ) */
    int    clb_armed, des_armed, alt_constraint_mode, speed_managed, des_submode;
    double clb_limit_kt, clb_limit_alt_ft;   /* climb speed limit; 0 = none */
    int    engine_out;              /* hold speed + 25 kt */
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
    int    more_drag;               /* above the path at idle with the speed on target: speed brakes */
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
    /* pseudo waypoints ahead of the aircraft, nearest first: the ND symbols and the MFD F-PLN lines
     * (a380x_fmgc_predictions) come from this one list */
    int    nd_pwp_count;
    A380XFmgcNdPwp nd_pwp[A380X_FMGC_MAX_ND_PWP];
    /* holding */
    A380XHoldEntry hold_entry;      /* entry chosen for the active hold leg */
    double hold_speed_kt;           /* holding speed of the active / next hold, NAN */
    int    hold_leg_index;          /* that hold leg, -1 */
    int    hold_decel_reached;      /* the hold speed is the managed target */
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
    double   entry_antic_nm, entry_arc_nm;   /* entry transition of the active leg (flown-path length) */
    /* DF */
    int      df_frozen;
    double   df_lat, df_lon;
    /* DME / radial legs: crossing direction at activation */
    int      cross_sign;
    /* hold */
    int      hold_phase;            /* 0 inbound, 1 turn at fix, 2 outbound, 3 turn back, 4 IMM EXIT,
                                       5/6 teardrop leg/turn, 7/8/9 parallel leg/turn/direct */
    int      hold_entered, hold_circuits;
    double   hold_radius_nm, hold_leg_nm, hold_teardrop_deg;
    A380XHoldEntry hold_entry;
    double   vs_filt_fpm;           /* smoothed V/S for the level-off predictions */
    /* FMS phase / decel latch, learned idle descent gradient factor */
    int      decel_latched;
    double   idle_factor;            /* learned idle gradient / nominal (0.7..1.3) */
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
/* Climb distance to a level at the present V/S (rate falling linearly to zero at a 43000 ft ceiling),
 * NM at gs_kt: the T/C / climb level-off / climb speed-limit pseudo waypoints. */
double a380x_fmgc_climb_distance_nm(double alt_ft, double target_ft, double vs_fpm, double gs_kt);
/* Hold entry for an arrival track (true) at a hold with inbound course `inbound_true` and turns
 * right (> 0) or left. Sectors relative to the inbound course, right turns: direct -110..+70 deg,
 * teardrop +70..+140, parallel otherwise; within 5 deg of a direct-sector boundary direct is kept,
 * within 5 deg of the teardrop/parallel boundary teardrop (ICAO +-5 deg flexibility). */
A380XHoldEntry a380x_fmgc_hold_entry(double inbound_true, double arrival_track_true, int right_turns);
/* FBW getHoldingSpeed (spec 07 section 5.8): cstr_* NAN when none. */
double a380x_fmgc_holding_speed(const A380XFmgcInputs *in, double cstr_speed_kt, double cstr_alt_ft);

/* JSON array of the ND pseudo waypoints (those with an ND placement) for the FBW ND bundle's
 * PseudoWaypointLayer, as the xp-kit ND state `nav.pseudo` carries it: [{"ident","pwp" (typePwp),
 * "type" (colour flag), "lat","lon" (on the path), "dist" (NM ahead), "alt"}]. Returns the length,
 * or 0 (and "[]" when cap allows) if it does not fit. */
size_t a380x_fmgc_nd_pwp_json(const A380XFmgcNdPwp *list, int count, char *out, size_t cap);

/* Basic F-PLN predictions for the MFD (A380XFplnPredFeed): pseudo waypoints T/C, T/D, DECEL and
 * per-leg distance/time/altitude from the profile. legs must hold A380X_FPLN_MAX_ELEMENTS entries. */
void a380x_fmgc_predictions(const A380XFmgc *g, const A380XFlightPlans *fp, const A380XFmgcInputs *in,
                            A380XFplnPredFeed *feed, A380XFplnWptPred *legs);

#ifdef __cplusplus
}
#endif

#endif
