/* SPDX-License-Identifier: GPL-3.0-or-later
 * LAND / FLARE / ROLL OUT, A/THR RETARD, SRS GA / GA TRK and the yaw actuator of the native guidance.
 * Unity include of a380x_guidance.c (uses its static helpers). State: A380XLandingState
 * (a380x_autoland.h); behaviour and limits: docs/AUTOLAND.md.
 *
 * Provenance. The mode transitions below are a hand port of the FlyByWire A380 PRIM flight-guidance
 * mode logic, fbw-a380x/src/wasm/fbw_a380/src/model/A380PrimComputerFg.cpp and
 * A380PrimComputerFg_data.cpp at aircraft revision 2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2
 * (GPL-3.0, https://github.com/flybywiresim/aircraft). Each threshold names its Simulink parameter.
 * The guidance laws are the unmodified vendored A380FgOuterLoops (vendor/fbw): lateral law
 * LOC TRACK (with its align/decrab beta command below 30 ft) and ROLL OUT, vertical law FLARE
 * (flare, touchdown derotation) and SRS. Port-specific, needing in-simulator tuning: the rudder
 * pedal servo (the FBW A380 lateral normal law that tracks the beta command is not ported), the
 * RETARD ramp in normalized engine command, the ground pitch-servo bound and the mapping of the
 * LAND capability terms onto the simulated sensors. The three-PRIM monitoring is not ported: LAND 3
 * SINGLE / DUAL is the capability annunciated from the simulated redundancy, not a certified
 * fail-passive / fail-operational monitor.
 */

#define LAND_ENGAGE_RA_FT 400.0     /* CompareToConstant_const_b: GS TRK + LOC TRK below 400 ft RA */
#define LAND_ENGAGE_CONFIRM_S 1.2   /* ConfirmNode1_timeDelay_ll */
#define FLARE_CONFIRM_S 0.36        /* ConfirmNode1_timeDelay_l3: flare condition latch */
#define ROLLOUT_RA_FT 50.0          /* CompareToConstant2_const_o4 */
#define ROLLOUT_CONFIRM_S 0.36      /* ConfirmNode1_timeDelay_f: on ground below 50 ft RA */
#define ALIGN_RA_FT 30.0            /* CompareToConstant1_const_c: align submode */
#define RETARD_RA_FT 50.0           /* CompareToConstant_const_d4: LAND + AP + A/THR below 50 ft RA */
#define LAND_RESET_GROUND_S 10.0    /* ConfirmNode2_timeDelay_i */
#define GA_TLA_DEG 36.7             /* CompareToConstant3_const_b and siblings (TLA, TOGA = 45 deg) */
#define GA_MIN_LEVERS 2             /* CompareToConstant4_const_b */
#define GA_GROUND_INHIBIT_S 30.0    /* ConfirmNode_timeDelay_a: no GA after 30 s on ground */
/* FBW RETARD_Value is -10 (N1 %/s). Port approximation: the existing .12/s engine-command slew,
 * which spans this aircraft's normalized idle-to-CL command in about 4 s. */
#define RETARD_COMMAND_RATE 0.12
#define RUDDER_MAX_DEG 30.0         /* A380 rudder travel; FBW zeta degrees <-> pedal ratio */
#define ALIGN_KP 0.12               /* port: pedal per degree of heading-to-course error */
#define ALIGN_KI 0.02               /* port: pedal per degree-second */
#define ALIGN_KR 0.45               /* port: pedal per deg/s of yaw rate (damping) */
#define FLARE_SERVO_KP 0.30         /* port: extra yoke per degree of FLARE/SRS GA attitude error */
#define FLARE_SERVO_KI 0.20         /* port: yoke per degree-second of FLARE/SRS GA attitude error */
/* FBW sideslip sign is opposite to X-Plane's sim/flightmodel/position/beta ("heading relative to the
 * flown path", positive nose right): the FBW lateral normal law integrates zeta += h*(beta_c-beta)
 * with zeta positive nose-left (N_zeta < 0), which only converges with beta positive nose-left.
 * The same convention makes the LOC TRACK align command stable. Verify in-simulator. */
#define FBW_BETA_FROM_XPLANE(beta_deg) (-(beta_deg))

static int landing_sensors_valid(const A380XGuidanceInputs *i) {
    return i->powered && !i->paused && i->airdata_valid && i->attitude_valid &&
        isfinite(i->pitch_deg) && isfinite(i->roll_deg) && isfinite(i->heading_deg) &&
        isfinite(i->track_deg) && isfinite(i->altitude_ft) && isfinite(i->vertical_speed_fpm) &&
        isfinite(i->ias_knots) && isfinite(i->tas_knots) && isfinite(i->ground_speed_knots) &&
        isfinite(i->mach) && isfinite(i->pressure_altitude_ft) && isfinite(i->radio_height_ft) &&
        isfinite(i->pitch_rate_deg_s) && isfinite(i->roll_rate_deg_s) && isfinite(i->yaw_rate_deg_s) &&
        isfinite(i->alpha_deg) && isfinite(i->beta_deg) && isfinite(i->true_heading_deg) &&
        isfinite(i->true_track_deg) && isfinite(i->accel_x_m_s2) && isfinite(i->accel_y_m_s2) &&
        isfinite(i->accel_z_m_s2);
}
/* LAND/FLARE/ROLL OUT (and a GA started from the runway) stay valid through touchdown and the
 * ground roll: airborne_valid() deliberately rejects on_ground and IAS below 40 kt. */
static int landing_ground_valid(const A380XGuidanceState *s, const A380XGuidanceInputs *i) {
    return (s->land.land_active || s->land.go_around_active) && landing_sensors_valid(i);
}
static int landing_ra_valid(const A380XGuidanceInputs *i) {
    return i->radio_height_valid && isfinite(i->radio_height_ft);
}
static double landing_tla(const A380XGuidanceInputs *i, double r) {
    /* Calibrated pilot-handle ratio to TLA: IDLE 0, CL 25, MCT 35, TOGA 45 deg. */
    if (r <= i->cl_detent_ratio)
        return 25 * (r - i->idle_detent_ratio) / (i->cl_detent_ratio - i->idle_detent_ratio);
    if (r <= i->mct_detent_ratio)
        return 25 + 10 * (r - i->cl_detent_ratio) / (i->mct_detent_ratio - i->cl_detent_ratio);
    return 35 + 10 * (r - i->mct_detent_ratio) / (i->toga_detent_ratio - i->mct_detent_ratio);
}
static void landing_clear(A380XLandingState *l) {
    l->land_active = l->flare_active = l->rollout_active = l->align_active = 0;
    l->autoland = l->cancel_request = 0;
    l->land_confirm_s = l->flare_confirm_s = l->rollout_confirm_s = l->ground_s = 0;
    l->yaw_active = 0; l->yaw_servo = l->yaw_integral = 0;
}

/* FCU selections while LAND is engaged: LOC/APPR and the lateral/vertical mode selections are
 * ineffective (the FBW PRIM keeps land_active until GA or the ground reset). Returns 1 when the
 * command is consumed. */
static int landing_command_filter(A380XGuidanceState *s, A380XFcuCommand c) {
    A380XLandingState *l = &s->land;
    if (!l->land_active) return 0;
    switch (c) {
    case A380X_FCU_APPR: case A380X_FCU_LOC:
        l->cancel_request = 1;   /* only effective on the ground after the LAND reset delay */
        return 1;
    case A380X_FCU_HDG_PUSH: case A380X_FCU_HDG_PULL: case A380X_FCU_ALT_PUSH: case A380X_FCU_ALT_PULL:
    case A380X_FCU_VS_PULL: case A380X_FCU_ALT_HOLD:
        return 1;
    default: return 0;
    }
}

static void landing_exit_on_ground(A380XGuidanceState *s, const A380XGuidanceInputs *i) {
    landing_clear(&s->land);
    s->loc_armed = s->gs_armed = s->approach_selected = 0;
    s->selected_heading_deg = norm(current_heading(s, i));
    s->lateral_mode = s->trk_fpa_mode ? A380X_C1_TRACK : A380X_C1_HDG;
    s->vertical_mode = A380X_B1_NONE; s->vertical_guidance_selected = 0;   /* ground standby */
    if (s->ap1 && s->ap2) s->ap2 = 0;
}

static void landing_go_around(A380XGuidanceState *s, const A380XGuidanceInputs *i, int valid) {
    A380XLandingState *l = &s->land;
    int n, count = 0, rising;   /* previous_toga_count: levers at or beyond the GA TLA */
    if (lever_inputs_valid(i))
        for (n = 0; n < 4; ++n) if (landing_tla(i, i->throttle_lever_ratio[n]) >= GA_TLA_DEG) ++count;
    rising = count >= GA_MIN_LEVERS && l->previous_toga_count < GA_MIN_LEVERS;
    l->previous_toga_count = count;
    /* pitch_goaround_active set: TOGA rising edge, flaps >= 1, not on ground for 30 s, guidance valid. */
    if (rising && i->flap_handle_index >= 1 && l->touchdown_s < GA_GROUND_INHIBIT_S &&
        (valid || landing_sensors_valid(i)) && (!i->on_ground || l->land_active)) {
        double vapp = s->speed_target_knots, ias = isfinite(i->ias_knots) ? i->ias_knots : vapp;
        landing_clear(l); l->retard_active = 0;
        l->go_around_active = 1;
        /* SRS GA target: the higher of VAPP (current speed target) and the engagement speed,
         * limited to VLS + 25 kt (Airbus all-engine SRS GA; the FBW speed logic is not ported). */
        l->ga_speed_knots = fmax(vapp, ias);
        if (isfinite(i->vls_knots) && i->vls_knots > 0) l->ga_speed_knots = fmin(l->ga_speed_knots, i->vls_knots + 25);
        l->ga_track_deg = norm(i->track_deg);
        s->vertical_guidance_selected = 1;
        s->vertical_mode = A380X_B1_SRS; s->lateral_mode = A380X_C1_GA_TRK;
        s->loc_armed = s->gs_armed = s->approach_selected = 0;
        s->altitude_managed = s->heading_managed = 0;
        s->capture_seconds = s->loc_capture_seconds = 0; s->pitch_integral = 0;
        /* TOGA in go-around arms A/THR (MAN TOGA until the levers return to CL). The regulator
         * restarts from the lever thrust, not from the approach-idle integrator. */
        if (!s->athr_armed && !i->athr_inhibited && i->powered) s->athr_armed = 1;
        if (s->athr_armed) s->athr_integral = clamp(clamp(i->throttle_ratio, 0, 1) - .5, -.48, .48);
    }
    if (l->go_around_active && s->vertical_mode != A380X_B1_SRS && s->lateral_mode != A380X_C1_GA_TRK)
        l->go_around_active = 0;
}

/* LAND capability (FMA D1/D2). Terms of the FBW A380 PRIM FG (A380PrimComputerFg.cpp
 * land_2_capability / land_3_fail_passive_capability / land_3_fail_op_capability at 2baa2b35):
 *   LAND 2   AP1 or AP2 available, both ILS valid, not all RA failed, rudder, normal law,
 *            no double ADR/IR failure;
 *   SINGLE   LAND 2 and A/THR available and not two RA failed;
 *   DUAL     AP1 and AP2 available, A/THR, rudder, normal law, green and yellow hydraulics,
 *            not two RA failed, both ILS valid, no ADR/IR rejected.
 * Port mapping, decided from what the simulator provides (docs/AUTOLAND.md): "available" becomes
 * engaged (an Airbus FMA shows the capability of the engaged AP/A/THR, so one AP gives SINGLE),
 * both ILS valid = both NAV receivers valid on LOC and G/S, the same frequency and course and
 * agreeing deviations; "not two RA failed" of three = both captain and F/O radio altimeters valid
 * and agreeing; no ADR/IR rejected = both ADIRS sides valid; hydraulics = X-Plane systems 1 and 2
 * pressurized; LAND 3 also needs the yaw (rollout) channel. There is no flight-control law in
 * the port; it is not a term. */
#define LAND_ALERT_HEIGHT_FT 200.0     /* below it (in LAND) the capability is frozen */
#define LAND_DOWNGRADE_CONFIRM_S 1.0   /* port: a lower capability must persist before it shows */
#define LAND_DOWNGRADE_HOLD_S 3.0      /* triple-click request level */
static int landing_ils_pair(const A380XGuidanceInputs *i) {
    const A380XIlsInputs *a = &i->ils[0], *b = &i->ils[1];
    if (!(ils_loc_valid(a) && ils_gs_valid(a) && ils_loc_valid(b) && ils_gs_valid(b))) return 0;
    if (fabs(diff(a->course_deg, b->course_deg)) >= 1) return 0;
    if (a->frequency_mhz > 0 && b->frequency_mhz > 0 && fabs(a->frequency_mhz - b->frequency_mhz) > .004) return 0;
    return fabs(a->loc_deviation_dots - b->loc_deviation_dots) <= .25 &&
        fabs(a->gs_deviation_dots - b->gs_deviation_dots) <= .5;
}
static int landing_ra_pair(const A380XGuidanceInputs *i) {
    double a = i->side_radio_height_ft[0], b = i->side_radio_height_ft[1];
    return i->side_radio_height_valid[0] && i->side_radio_height_valid[1] && isfinite(a) && isfinite(b) &&
        fabs(a - b) <= fmax(15, .05 * fmax(fabs(a), fabs(b)));
}
static int landing_capability_conditions(const A380XGuidanceState *s, const A380XGuidanceInputs *i) {
    const A380XLandingState *l = &s->land;
    int any_ra = landing_ra_valid(i) || i->side_radio_height_valid[0] || i->side_radio_height_valid[1];
    int land2, single;
    if (!i->powered || !(s->approach_selected || l->land_active)) return A380X_LAND_CAPABILITY_NONE;
    land2 = (s->ap1 || s->ap2) && landing_ils_pair(i) && any_ra && i->rudder_valid &&
        i->attitude_valid && i->airdata_valid;
    single = land2 && s->athr_armed && s->athr_active && landing_ra_pair(i) && i->yaw_channel_available;
    if (single && s->ap1 && s->ap2 && i->hydraulics_valid &&
        i->side_attitude_valid[0] && i->side_attitude_valid[1] && i->side_airdata_valid[0] && i->side_airdata_valid[1])
        return A380X_LAND_CAPABILITY_LAND3_DUAL;
    return single ? A380X_LAND_CAPABILITY_LAND3_SINGLE : land2 ? A380X_LAND_CAPABILITY_LAND2 : A380X_LAND_CAPABILITY_APPR1;
}
static void landing_capability(A380XGuidanceState *s, const A380XGuidanceInputs *i, double dt) {
    A380XLandingState *l = &s->land;
    int raw = landing_capability_conditions(s, i);
    l->capability_raw = raw;
    l->capability_downgrade_s = fmax(0, l->capability_downgrade_s - dt);
    if (raw == A380X_LAND_CAPABILITY_NONE) {
        /* APPR cancelled, GA or LAND reset: no capability, no downgrade alert. */
        l->capability = A380X_LAND_CAPABILITY_NONE; l->capability_frozen = 0; l->capability_down_s = 0;
    } else {
        /* Alert height: in LAND below 200 ft the annunciated capability freezes (failures then
         * rely on the AUTOLAND light); it stays frozen through flare and roll-out. */
        if (l->land_active && (l->capability_frozen || (landing_ra_valid(i) && i->radio_height_ft < LAND_ALERT_HEIGHT_FT)))
            l->capability_frozen = 1;
        if (!l->capability_frozen) {
            if (raw >= l->capability || l->capability == A380X_LAND_CAPABILITY_NONE) {
                l->capability = raw; l->capability_down_s = 0;   /* upgrades show at once */
            } else if ((l->capability_down_s += dt) >= LAND_DOWNGRADE_CONFIRM_S - 1e-9) {
                l->capability = raw; l->capability_down_s = 0;
                if (!i->on_ground) l->capability_downgrade_s = LAND_DOWNGRADE_HOLD_S;   /* triple click */
            }
        } else l->capability_down_s = 0;
    }
    l->capability_downgrade = l->capability_downgrade_s > 0;
}

/* Called once per update after the LOC/GS capture logic and before the laws step. */
static void landing_modes(A380XGuidanceState *s, const A380XGuidanceInputs *i, double dt,
                          const A380XIlsInputs *ils, int valid) {
    A380XLandingState *l = &s->land;
    int ap = s->ap1 || s->ap2, fd = s->fd1 || s->fd2, ra_ok = landing_ra_valid(i);
    double ra = ra_ok ? i->radio_height_ft : 1e9;
    l->touchdown_s = i->on_ground ? l->touchdown_s + dt : 0;
    if (!i->powered) {
        landing_clear(l); l->go_around_active = l->retard_active = 0; l->bc1 = 0;
        l->capability = l->capability_raw = l->capability_frozen = l->capability_downgrade = 0;
        l->capability_down_s = l->capability_downgrade_s = 0;
        return;
    }
    landing_go_around(s, i, valid);
    /* APEngagedLogic groundGa: on the ground a go-around or any lever at TLA >= 36.7 deg disengages
     * the AP (FD guidance continues). */
    if (i->on_ground && ap && (l->go_around_active || l->previous_toga_count > 0)) { disconnect(s, 1); ap = 0; }
    if (!l->land_active) {
        int engage = valid && !i->on_ground && s->approach_selected && (ap || fd) && ra_ok &&
            ra < LAND_ENGAGE_RA_FT && s->lateral_mode == A380X_C1_LOC && s->vertical_mode == A380X_B1_GS &&
            ils_loc_valid(ils) && ils_gs_valid(ils);
        l->land_confirm_s = engage ? l->land_confirm_s + dt : 0;
        if (engage && l->land_confirm_s >= LAND_ENGAGE_CONFIRM_S - 1e-9) {
            l->land_active = 1; s->loc_armed = s->gs_armed = 0;
            l->course_valid = 1; l->course_deg = ils->course_deg;
        }
    }
    if (l->land_active) {
        int flare, rollout;
        if (ap) l->autoland = 1;
        if (ils_loc_valid(ils)) { l->course_valid = 1; l->course_deg = ils->course_deg; }
        l->align_active = ra_ok && ra <= ALIGN_RA_FT;
        /* FLARE: FgOuterLoops flare_law.condition_Flare (below 80 ft when RA*14 <= |H dot|, or 42 ft),
         * all RA valid, latched after 0.36 s. */
        flare = ra_ok && l->law_flare_condition;
        if (flare) l->flare_confirm_s += dt; else if (l->flare_confirm_s < FLARE_CONFIRM_S) l->flare_confirm_s = 0;
        l->flare_active = flare || l->flare_confirm_s >= FLARE_CONFIRM_S - 1e-9;
        rollout = ra_ok && ra <= ROLLOUT_RA_FT && i->on_ground;
        if (rollout) l->rollout_confirm_s += dt; else if (l->rollout_confirm_s < ROLLOUT_CONFIRM_S) l->rollout_confirm_s = 0;
        l->rollout_active = l->rollout_confirm_s >= ROLLOUT_CONFIRM_S - 1e-9;
        if (l->rollout_active) l->flare_active = 1;   /* the FLARE law derotates on the ground */
        /* Without a radio altimeter there is no flare: an engaged AP disconnects (AUTOLAND light). */
        if (!ra_ok && !i->on_ground && !l->flare_active && ap) disconnect(s, 1);
        l->ground_s = i->on_ground ? l->ground_s + dt : 0;
        if (i->on_ground && l->ground_s >= LAND_RESET_GROUND_S && !(s->ap1 || s->ap2) &&
            (l->cancel_request || (!s->fd1 && !s->fd2))) landing_exit_on_ground(s, i);
    }
    l->cancel_request = 0;
    /* SRS GA speed target (the laws step below uses speed_target_knots as V_c). */
    if (l->go_around_active && s->vertical_mode == A380X_B1_SRS) s->speed_target_knots = l->ga_speed_knots;
    landing_capability(s, i, dt);
    l->bc1 = !l->land_active ? A380X_BC1_NONE : l->rollout_active ? A380X_BC1_ROLL_OUT :
        l->flare_active ? A380X_BC1_FLARE : A380X_BC1_LAND;
}

/* After the pitch/roll servo: yaw (rudder pedal) channel in ALIGN and ROLL OUT, bounded ground pitch. */
static void landing_actuators(A380XGuidanceState *s, const A380XGuidanceInputs *i, double dt) {
    A380XLandingState *l = &s->land;
    int ap = s->ap1 || s->ap2;
    if (ap && s->guidance_valid && !i->on_ground &&
        ((l->land_active && l->flare_active && !l->rollout_active) ||
         (l->go_around_active && s->vertical_mode == A380X_B1_SRS))) {
        /* FLARE and SRS GA: the FBW laws issue attitude increments for the A380 pitch normal law, which
         * tracks them tightly; the generic port yoke servo lags by about a degree (most of the flare,
         * and the SRS pitch limit under TOGA thrust pitch-up). Add attitude-error gain and an
         * integrator in these two modes (port gains, in-sim tuning). */
        double error = s->pitch_command_deg - i->pitch_deg;
        l->flare_integral = clamp(l->flare_integral + error * FLARE_SERVO_KI * dt, -.3, .3);
        s->pitch_servo = clamp(s->pitch_servo + error * FLARE_SERVO_KP + l->flare_integral, -1, 1);
    } else l->flare_integral = 0;
    l->yaw_active = ap && s->guidance_valid && l->land_active && (l->align_active || l->rollout_active);
    if (!l->yaw_active) { l->yaw_servo = l->yaw_integral = 0; return; }
    if (l->rollout_active) {
        /* ROLL OUT: on the ground FBW's autopilot Beta_c is the rudder position command (the
         * A380LateralNormalLaw passes ap_beta_c through as zeta on ground); X-Plane nosewheel
         * steering follows the pedals (ACF nose gear 7 deg at speed). */
        l->yaw_servo = clamp(-l->law_beta_c_deg / RUDDER_MAX_DEG, -1, 1);
        l->yaw_integral = l->yaw_servo;
        /* Derotation: the FLARE law commands -2 deg pitch on ground; bound the yoke so the nose gear
         * is held down without full forward elevator (port actuator limit). */
        if (ap) { s->pitch_servo = clamp(s->pitch_servo, -.3, .15); s->pitch_integral = 0; }
    } else {
        /* ALIGN (decrab) in the air: the rudder aligns the heading with the localizer course. The FBW
         * LOC TRACK law still computes its align beta command (used by its own roll term); tracked
         * by a direct sideslip servo that command settles at 150 % of the drift angle (Beta_c =
         * 1.5 (beta - heading error) is constant in a steady decrab), because the A380 lateral
         * normal law that integrates it is not ported. Port law and gains: in-sim tuning. */
        double error = l->course_valid ? diff(l->course_deg, i->heading_deg) : 0;
        l->yaw_integral = clamp(l->yaw_integral + error * ALIGN_KI * dt, -.8, .8);
        l->yaw_servo = clamp(error * ALIGN_KP + l->yaw_integral - i->yaw_rate_deg_s * ALIGN_KR, -1, 1);
    }
}

/* After the regular A/THR regulator: RETARD in autoland and the SRS thrust mode. */
static void landing_athr(A380XGuidanceState *s, const A380XGuidanceInputs *i, double dt) {
    A380XLandingState *l = &s->land;
    int levers_valid = lever_inputs_valid(i), ap = s->ap1 || s->ap2, ra_ok = landing_ra_valid(i);
    double lever = levers_valid ? max_lever(i) : 0, ceiling;
    int all_idle = levers_valid && lever <= i->idle_detent_ratio;
    int inner_reverse = levers_valid && i->reverse_selected[1] && i->reverse_selected[2];
    int levers_in_range = levers_valid && !all_idle && !inner_reverse &&
        lever <= i->cl_detent_ratio + i->detent_tolerance_ratio;
    ceiling = clamp(lever / (i->cl_detent_ratio > 0 ? i->cl_detent_ratio : 1) * .88, 0, .88);
    if (i->athr_ceiling_valid && isfinite(i->athr_ceiling)) ceiling = clamp(i->athr_ceiling, 0, 1);
    /* retard_mode_active: A/THR engaged, LAND, AP engaged, RA <= 50 ft; reset by A/THR off, GA,
     * alpha floor or the LAND reset. Manual landings have no RETARD mode (the RETARD aural is a
     * reminder to retard the levers, which disconnects A/THR). */
    if (!l->retard_active && s->athr_armed && s->athr_active && l->land_active && ap && ra_ok &&
        i->radio_height_ft <= RETARD_RA_FT && !s->alpha_floor_latched) {
        l->retard_active = 1; l->retard_command = l->athr_previous;
    }
    if (!s->athr_armed || !l->land_active || l->go_around_active || s->alpha_floor_latched) l->retard_active = 0;
    if (l->retard_active) {
        /* RETARD stays active through touchdown and the roll-out until the crew sets the levers to
         * IDLE (A/THR disconnect, a380x_guidance.c) or selects reverse. */
        if (levers_in_range && i->powered && !i->paused && i->athr_speed_valid) {
            l->retard_command = fmax(0, fmin(l->retard_command, ceiling) - RETARD_COMMAND_RATE * dt);
            s->athr_active = 1;
            s->throttle_command = l->retard_command;
            s->thrust_mode = A380X_A1A2_THR_IDLE;
        }
    } else if (s->athr_active && !s->alpha_floor_latched && s->guidance_valid && s->vertical_mode == A380X_B1_SRS) {
        /* SRS is a speed-on-elevator law: A/THR is in a thrust mode (THR CLB, THR LVR below CL),
         * slewed from the previous frame's command like the regular regulator. */
        s->throttle_command = fmin(ceiling, l->athr_previous +
            clamp(ceiling - l->athr_previous, -.12 * dt, .12 * dt));
        s->thrust_mode = lever < i->cl_detent_ratio - i->detent_tolerance_ratio ? A380X_A1A2_THR_LVR : A380X_A1A2_THR_CLB;
    }
    l->athr_previous = s->throttle_command;
}
