/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 FCU state adapter; actual AP/FD outer-loop laws are vendored FlyByWire.
 * This is not the complete PRIM/FCU/FMS computer network. LAND/FLARE/ROLL OUT, RETARD and
 * SRS GA / GA TRK follow the FBW PRIM FG mode logic (a380x_autoland.h, docs/AUTOLAND.md).
 */
#ifndef A380X_GUIDANCE_H
#define A380X_GUIDANCE_H
#include <stddef.h>
#include "a380x_avionics_core.h"
#include "a380x_autoland.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int valid, loc_valid, gs_valid;
    double loc_deviation_dots; /* positive: localizer needle RIGHT */
    double gs_deviation_dots;  /* positive: glideslope needle UP */
    double course_deg, gs_angle_deg, distance_nm; /* course is MAGNETIC */
    /* Optional: distance to the LOCALIZER antenna (the DME is often at the glide antenna). The
     * ROLL OUT law converts the angular deviation to metres with it; zero/invalid = use DME. */
    int localizer_distance_valid;
    double localizer_distance_nm;
    double frequency_mhz;   /* tuned frequency, 0 = unknown (LAND capability: both receivers on one ILS) */
} A380XIlsInputs;

typedef struct {
    int powered, paused, on_ground, attitude_valid, airdata_valid;
    double ias_knots, tas_knots, mach, ground_speed_knots;
    double altitude_ft, pressure_altitude_ft, radio_height_ft, vertical_speed_fpm;
    double pitch_deg, roll_deg, heading_deg, track_deg, true_heading_deg, true_track_deg;
    double pitch_rate_deg_s, roll_rate_deg_s, yaw_rate_deg_s;
    double alpha_deg, beta_deg, accel_x_m_s2, accel_y_m_s2, accel_z_m_s2;
    double weight_kg, vls_knots, vmax_knots, throttle_ratio;
    A380XIlsInputs ils[2];
    /* Only validated FMS guidance may activate a managed mode. Never bind
     * these to X-Plane's stock autopilot/FMS mode flags or invent a route. */
    int managed_speed_valid, managed_route_valid, managed_vertical_valid;
    double managed_speed_knots, managed_heading_deg, managed_altitude_ft, managed_vs_fpm;
    double route_xtk_nm, route_track_error_deg, route_bank_deg;
    /* Native FMS managed target type (spec 07 section 10.2): when set, managed_speed_knots is the
     * CAS equivalent of managed_mach at the current pressure, as FBW passes it to the FG. */
    int managed_speed_is_mach;
    double managed_mach;
    /* Physical PILOT handles, never A/THR actuator outputs. The X-Plane
     * boundary validates an aircraft-local calibration before asserting valid.
     * Ratios are not engine N1; default TLA mapping is a port assumption. */
    int throttle_levers_valid;
    double throttle_lever_ratio[4];
    int reverse_selected[4];
    double idle_detent_ratio, cl_detent_ratio, mct_detent_ratio, toga_detent_ratio;
    double detent_tolerance_ratio;
    int flex_valid;
    /* A qualified FE provider owns alpha-floor computation/inhibits. Missing
     * FE input must remain invalid; never infer this request from raw AoA. */
    int alpha_floor_valid, alpha_floor_request;
    int athr_speed_valid; /* ADR speed validity, independent of IR/VS/navigation */
    /* Thrust-lever / FADEC module (docs/THRUST_LEVERS.md); zero = legacy behaviour.
     * athr_inhibited: instinctive disconnect held > 15 s (FBW FADEC ATHR_disabled): A/THR cannot
     * arm or engage, alpha floor included, until FMGC power-up on the ground.
     * athr_ceiling: normalised engine command of the highest lever's FADEC N1 (FBW clamps the A/THR
     * N1 to the lever's N1), replacing the 0.88-at-CL approximation when valid. */
    int athr_inhibited, athr_ceiling_valid;
    double athr_ceiling;
    /* FMGC managed guidance (a380x_fmgc_guidance.c through the FMS adapter, docs/AUTOPILOT_DEVELOPMENT.md);
     * all zero = absent. route_xtk_nm / route_track_error_deg / route_bank_deg above follow the FBW
     * HPATH law convention: xtk > 0 = path to the RIGHT of the aircraft, track error > 0 = desired
     * track to the RIGHT, bank > 0 = right-wing-down feed-forward. */
    int managed_nav_capture;            /* NAV may engage now (capture zone reached / course leg) */
    int managed_altitude_is_constraint; /* managed_altitude_ft is an FMS constraint, not the FCU altitude */
    double managed_cruise_altitude_ft;  /* FMS CRZ altitude (ALT CRZ), 0 = none */
    int managed_path_valid;             /* descent path computed */
    double managed_path_altitude_ft, managed_path_vs_fpm, managed_vdev_ft; /* vdev > 0: above the path */
    int managed_path_idle, managed_before_tod;
    int fms_phase_valid, fms_phase;     /* FMGC flight phase (A380XFmsPhase numbering) */
    double fms_v2_knots;                /* takeoff V2 for SRS, 0 = none */
    /* Landing (a380x_autoland.inc). Missing data keeps LAND/GA unavailable: radio_height_ft is 0
     * when invalid, so LAND requires radio_height_valid; GA requires a flap handle index >= 1. */
    int radio_height_valid, flap_handle_index;
    double rudder_ratio; /* pilot rudder pedal position [-1,1], +right; FBW ROLL OUT entry value */
    /* LAND capability redundancy as the simulator provides it (a380x_autoland.inc landing_capability):
     * captain/F-O radio altimeters and ADIRS sides, both hydraulic systems pressurized, no rudder
     * failure, the rudder (yaw) override available to the AP. Zero = not available. */
    int side_radio_height_valid[2];
    double side_radio_height_ft[2];
    int side_attitude_valid[2], side_airdata_valid[2];
    int hydraulics_valid, rudder_valid, yaw_channel_available;
} A380XGuidanceInputs;

typedef enum {
    A380X_FCU_SPD_ROTATE, A380X_FCU_SPD_PUSH, A380X_FCU_SPD_PULL,
    A380X_FCU_HDG_ROTATE, A380X_FCU_HDG_PUSH, A380X_FCU_HDG_PULL,
    A380X_FCU_ALT_ROTATE, A380X_FCU_ALT_PUSH, A380X_FCU_ALT_PULL,
    A380X_FCU_VS_ROTATE, A380X_FCU_VS_PUSH, A380X_FCU_VS_PULL,
    A380X_FCU_AP1, A380X_FCU_AP2, A380X_FCU_AP_DISCONNECT,
    A380X_FCU_FD1, A380X_FCU_FD2, A380X_FCU_ATHR,
    A380X_FCU_LOC, A380X_FCU_APPR, A380X_FCU_ALT_HOLD,
    A380X_FCU_SPD_MACH, A380X_FCU_TRK_FPA, A380X_FCU_ALT_INCREMENT,
    A380X_FCU_LS1, A380X_FCU_LS2, A380X_FCU_METRIC_ALT,
    A380X_FCU_TRUE_MAG, A380X_FCU_ATHR_DISCONNECT
} A380XFcuCommand;

typedef struct {
    /* Public FCU selections, live mode annunciations and control outputs. */
    double selected_speed_knots, selected_mach, selected_heading_deg;
    double selected_altitude_ft, selected_vs_fpm, selected_fpa_deg;
    int speed_managed, heading_managed, altitude_managed, mach_mode, trk_fpa_mode;
    int altitude_increment_1000, ap1, ap2, fd1, fd2, athr_armed, athr_active;
    int loc_armed, gs_armed, altitude_armed, approach_selected;
    int ls1, ls2, metric_alt, source_side, true_heading;
    int speed_dashes, heading_dashes, vs_dashes;
    int guidance_valid, ap_engagement_inhibited, managed_unavailable;
    int vertical_guidance_selected; /* 0 = supported FD standby; no SRS port */
    int mode_reversion, disconnect_warning;
    A380XA1A2Message thrust_mode;
    A380XB1Message vertical_mode;
    A380XC1Message lateral_mode;
    double speed_target_knots, altitude_target_ft, vs_target_fpm;
    double pitch_command_deg, roll_command_deg;
    double fd_pitch_error_deg, fd_roll_error_deg;
    double pitch_servo, roll_servo, throttle_command; /* normalized [-1,1], [0,1] */
    /* Private persistent state; caller must not modify these fields. */
    void *fbw_laws;
    double airborne_seconds, simulation_seconds, capture_seconds, loc_capture_seconds;
    double speed_preselect_seconds, heading_preselect_seconds, vs_preselect_seconds;
    double previous_gs_dots, previous_loc_dots, athr_integral, pitch_integral;
    double unavailable_seconds, reversion_seconds, disconnect_seconds;
    double held_altitude_ft, last_selected_altitude_ft;
    int have_previous_ils, previous_powered, takeoff_detent_active, all_levers_idle;
    int alpha_floor_latched, alpha_floor_inhibited, initialized;
    /* FMGC managed modes (a380x_guidance_managed.inc). Public: armed modes and the ALT CST / ALT CRZ
     * qualifiers of ALT / ALT* (FG discrete words 2 and 3, bits 28/29). */
    int nav_armed, clb_armed, des_armed, alt_constraint, alt_cruise;
    int des_submode;                    /* DES: 0 on path, 1 above path / no path (idle, speed on pitch), 2 below (V/S) */
    int des_thrust_idle;                /* DES A/THR THR IDLE (else SPEED/MACH) */
    int cst_capture_inhibit;            /* DES on a path that descends through the constraint: no ALT CST* */
    int vertical_law_override;          /* FBW vertical law selector for the bridge, 0 = mode default */
    int vpath_valid;                    /* bridge: fms_H_c_profile / fms_H_dot_c_profile below */
    double vpath_altitude_ft, vpath_vs_fpm;
    /* private */
    int managed_climb, nav_auto_armed, clb_disarmed, athr_speed_prev;
    double cst_capture_ft, ias_prev, ias_rate;
    /* LAND/FLARE/ROLL OUT/RETARD/GA sub-state and the yaw (rudder) actuator output. */
    A380XLandingState land;
} A380XGuidanceState;

/* FMA vertical message including the ALT CST / ALT CRZ qualifiers (for the native PFD and FMS). */
A380XB1Message a380x_guidance_fma_vertical(const A380XGuidanceState *state);

void a380x_guidance_init(A380XGuidanceState *state, const A380XGuidanceInputs *inputs);
void a380x_guidance_destroy(A380XGuidanceState *state);
/* Why AP/FD/A-THR cannot engage now (for the on-screen and log diagnostics); returns the number of
 * problems written to out (0 = can engage). */
int a380x_guidance_engage_problems(const A380XGuidanceState *state, const A380XGuidanceInputs *inputs,
                                   char *out, size_t size);
/* amount is signed detents for ROTATE; other commands ignore it. */
void a380x_guidance_command(A380XGuidanceState *state, const A380XGuidanceInputs *inputs,
                           A380XFcuCommand command, double amount);
void a380x_guidance_update(A380XGuidanceState *state, const A380XGuidanceInputs *inputs,
                          double dt_seconds);
#ifdef __cplusplus
}
#endif
#endif
