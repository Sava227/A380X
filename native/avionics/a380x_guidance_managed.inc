/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Managed (FMS-coupled) FG modes of the native FCU guidance: NAV arming / capture / engagement,
 * takeoff SRS, managed CLB / ALT CST / DES with its path submodes, ALT CRZ, the managed speed
 * corrections and the A/THR mode for those vertical modes. Included by a380x_guidance.c after its
 * static helpers; the targets come from the FMGC module (a380x_fmgc_guidance.c) through
 * A380XGuidanceInputs.managed_*.
 *
 * Mode logic follows the Airbus FMA behaviour documented by FlyByWire (A380X FMA / AFS pages,
 * docs.flybywiresim.com) and the A32NX/A380X PRIM FG state machine semantics at revision
 * 2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2:
 *   - NAV arms on the ground with a flight plan (FD on, HDG not preselected) and engages at 30 ft RA
 *     after liftoff; in flight HDG/TRK push engages NAV inside its capture zone, else arms it.
 *   - SRS engages on the ground with T.O thrust (TOGA, or FLEX with a FLEX temperature) and a V2,
 *     guides V2 + 10 kt airborne (FBW SRS law, vertical law 8), and hands over at the acceleration
 *     altitude (FMGC CLIMB phase) to CLB if armed with NAV, else OP CLB.
 *   - CLB (speed on pitch, A/THR THR CLB) to min(FCU, next climb constraint); a constraint level
 *     off is ALT CST* / ALT CST with CLB armed, resumed when the constraint is sequenced.
 *   - DES on the computed path (VPATH law 9, A/THR THR IDLE on idle segments, SPEED/MACH on
 *     geometric ones), above the path / without a path idle with speed on pitch (+10 kt in the
 *     speed range), below the path / before T/D -1000 ft/min (A/THR SPEED); floors = ALT CST.
 *   - NAV lost (HDG pull, discontinuity, invalid route): CLB -> OP CLB, DES -> V/S.
 * Port choices: ALT push without NAV engaged gives OP CLB / OP DES (CLB / DES armed when NAV is
 * armed); hold entries are direct entries; there are no APP DES / FINAL APP modes.
 */

enum { FG_PH_PREFLIGHT = 0, FG_PH_TAKEOFF, FG_PH_CLIMB, FG_PH_CRUISE, FG_PH_DESCENT, FG_PH_APPROACH, FG_PH_GOAROUND };

static int nav_mode(const A380XGuidanceState *s) { return s->lateral_mode == A380X_C1_NAV; }
static int alt_hold_mode(const A380XGuidanceState *s) { return s->vertical_mode == A380X_B1_ALT; }
static int managed_vertical_mode(const A380XGuidanceState *s)
{
    return s->vertical_mode == A380X_B1_CLB || s->vertical_mode == A380X_B1_DES;
}

A380XB1Message a380x_guidance_fma_vertical(const A380XGuidanceState *s)
{
    if (!s) return A380X_B1_NONE;
    if (s->vertical_mode == A380X_B1_ALT && s->alt_constraint) return A380X_B1_ALT_CST;
    if (s->vertical_mode == A380X_B1_ALT_STAR && s->alt_constraint) return A380X_B1_ALT_CST_STAR;
    if (s->vertical_mode == A380X_B1_ALT && s->alt_cruise) return A380X_B1_ALT_CRZ;
    if (s->vertical_mode == A380X_B1_ALT_STAR && s->alt_cruise) return A380X_B1_ALT_CRZ_STAR;
    return s->vertical_mode;
}

static void engage_nav(A380XGuidanceState *s)
{
    s->heading_managed = 1; s->lateral_mode = A380X_C1_NAV;
    s->heading_preselect_seconds = 0; s->heading_dashes = 1; s->nav_armed = 0;
}

static void engage_managed_vertical(A380XGuidanceState *s, const A380XGuidanceInputs *i, int climb)
{
    s->altitude_managed = 1; s->vertical_guidance_selected = 1; s->gs_armed = 0;
    s->vertical_mode = climb ? A380X_B1_CLB : A380X_B1_DES;
    s->managed_climb = climb; s->clb_armed = s->des_armed = 0;
    s->alt_constraint = s->alt_cruise = 0;
    s->des_submode = 0; s->capture_seconds = 0;
    (void)i;
}

/* HDG/TRK knob push (managed lateral). */
static void managed_hdg_push(A380XGuidanceState *s, const A380XGuidanceInputs *i)
{
    if (!i->managed_route_valid) { reject_managed(s); return; }
    if (i->on_ground) {          /* NAV armed for takeoff; HDG remains the active ground mode */
        s->nav_armed = 1; s->heading_preselect_seconds = 0;
        return;
    }
    if (i->managed_nav_capture) {
        cancel_approach(s, i);
        engage_nav(s);
    } else {
        s->nav_armed = 1;
    }
}

/* ALT knob push (managed vertical). */
static void managed_alt_push(A380XGuidanceState *s, const A380XGuidanceInputs *i)
{
    int climb = s->selected_altitude_ft >= i->altitude_ft;
    if (!i->managed_route_valid || !i->managed_vertical_valid) { reject_managed(s); return; }
    s->clb_disarmed = 0;
    if (i->on_ground || s->vertical_mode == A380X_B1_SRS) {
        s->clb_armed = climb; s->des_armed = 0;
        return;
    }
    if (nav_mode(s)) { engage_managed_vertical(s, i, climb); return; }
    /* NAV not engaged: open mode, the managed mode armed when NAV is armed (port choice). */
    s->vertical_guidance_selected = 1; s->altitude_managed = 0; s->gs_armed = 0; s->capture_seconds = 0;
    s->alt_constraint = s->alt_cruise = 0;
    if (fabs(s->selected_altitude_ft - i->altitude_ft) < 40) {
        s->held_altitude_ft = s->selected_altitude_ft; s->vertical_mode = A380X_B1_ALT;
    } else s->vertical_mode = climb ? A380X_B1_OP_CLB : A380X_B1_OP_DES;
    s->clb_armed = s->nav_armed && climb;
    s->des_armed = s->nav_armed && !climb;
}

/* Selected vertical/lateral commands cancel the managed arming they replace. */
static void managed_after_command(A380XGuidanceState *s, A380XFcuCommand c)
{
    switch (c) {
    case A380X_FCU_HDG_PULL: s->nav_armed = 0; break;
    case A380X_FCU_ALT_PULL: case A380X_FCU_VS_PULL: case A380X_FCU_ALT_HOLD:
        s->clb_armed = s->des_armed = 0; s->alt_constraint = s->alt_cruise = 0;
        if (c != A380X_FCU_ALT_HOLD) s->clb_disarmed = 1;
        break;
    default: break;
    }
}

/* Leaving NAV: CLB -> OP CLB, DES -> V/S, ALT CST -> ALT (managed modes need NAV). */
static void managed_vertical_without_nav(A380XGuidanceState *s, const A380XGuidanceInputs *i)
{
    if (s->vertical_mode == A380X_B1_CLB) {
        s->vertical_mode = A380X_B1_OP_CLB; s->altitude_managed = 0;
    } else if (s->vertical_mode == A380X_B1_DES) {
        revert_vertical(s, i);
    } else if (s->altitude_managed) {
        s->altitude_managed = 0; s->alt_constraint = 0;
        if (s->vertical_mode == A380X_B1_ALT) s->held_altitude_ft = i->altitude_ft;
    }
    s->clb_armed = s->des_armed = 0;
}

/* NAV is lost while engaged: HDG/TRK on the current heading (mode reversion). */
static void managed_nav_lost(A380XGuidanceState *s, const A380XGuidanceInputs *i)
{
    s->heading_managed = 0; s->selected_heading_deg = norm(current_heading(s, i));
    s->lateral_mode = s->trk_fpa_mode ? A380X_C1_TRACK : A380X_C1_HDG; s->reversion_seconds = 5;
    managed_vertical_without_nav(s, i);
}

/* Managed altitude target: CLB min(FCU, constraint), DES max(FCU, floor); ALT holds. */
static double managed_altitude_target(const A380XGuidanceState *s, const A380XGuidanceInputs *i)
{
    if (s->vertical_mode == A380X_B1_ALT) return s->held_altitude_ft;
    if (!isfinite(i->managed_altitude_ft) || i->managed_altitude_ft <= 0) return s->selected_altitude_ft;
    return s->managed_climb ? fmin(i->managed_altitude_ft, s->selected_altitude_ft)
                            : fmax(i->managed_altitude_ft, s->selected_altitude_ft);
}

/* Commanded vertical speed of the managed modes (direction for the ALT capture logic). */
static double managed_vs_target(const A380XGuidanceState *s, const A380XGuidanceInputs *i)
{
    if (s->vertical_mode == A380X_B1_CLB) return 2000;
    if (s->vertical_mode == A380X_B1_DES) {
        if (s->des_submode == 2) return -1000;
        if (s->des_submode == 0 && i->managed_path_valid && isfinite(i->managed_path_vs_fpm)) return fmin(-100, i->managed_path_vs_fpm);
        return -1500;
    }
    return 0;
}

/* DES path submode with hysteresis and its A/THR mode. */
static void managed_des_submode(A380XGuidanceState *s, const A380XGuidanceInputs *i)
{
    double vdev = i->managed_vdev_ft;
    double serr = i->ias_knots - s->speed_target_knots;
    if (!i->managed_path_valid || !isfinite(vdev)) { s->des_submode = 1; s->des_thrust_idle = 1; return; }
    if (i->managed_before_tod) s->des_submode = 2;                  /* early descent */
    else switch (s->des_submode) {
    case 0:
        if (vdev > 500) s->des_submode = 1;
        else if (vdev < -500) s->des_submode = 2;
        else if (s->des_thrust_idle && serr > 10) s->des_submode = 1;   /* too fast at idle: leave the path */
        break;
    case 1: if (vdev < 100) s->des_submode = 0; break;
    default: if (vdev > -100) s->des_submode = 0; break;
    }
    if (s->des_submode == 1) s->des_thrust_idle = 1;
    else if (s->des_submode == 2) s->des_thrust_idle = 0;
    else if (!i->managed_path_idle) s->des_thrust_idle = 0;          /* geometric segment: SPEED */
    else if (serr < -10) s->des_thrust_idle = 0;                     /* too slow on the idle path */
    else if (serr > -2) s->des_thrust_idle = 1;
}

/* Every update, after the managed-input validity checks and before the altitude target. */
static void managed_update(A380XGuidanceState *s, const A380XGuidanceInputs *i, int valid)
{
    int phase_climb = i->fms_phase_valid ? i->fms_phase >= FG_PH_CLIMB : i->radio_height_ft > 1500;
    s->vertical_law_override = 0;
    s->vpath_valid = 0;
    if (!i->powered) { s->nav_armed = s->clb_armed = s->des_armed = 0; return; }
    /* ---- ground: automatic NAV / CLB arming, takeoff SRS ---- */
    if (!i->managed_route_valid) { s->nav_auto_armed = 0; s->nav_armed = 0; }
    if (i->on_ground) {
        if (!s->nav_auto_armed && i->managed_route_valid && (s->fd1 || s->fd2)
            && (s->lateral_mode == A380X_C1_HDG || s->lateral_mode == A380X_C1_TRACK) && s->heading_preselect_seconds <= 0) {
            s->nav_armed = 1; s->nav_auto_armed = 1;
        }
        if (s->nav_armed && i->managed_vertical_valid && !s->clb_disarmed && s->selected_altitude_ft > i->altitude_ft + 400)
            s->clb_armed = 1;
        if (takeoff_detent(i) && i->fms_v2_knots >= 100 && (s->fd1 || s->fd2) &&
            (s->vertical_mode == A380X_B1_NONE || s->vertical_mode == A380X_B1_SRS) && (!i->fms_phase_valid || i->fms_phase <= FG_PH_TAKEOFF)) {
            if (s->vertical_mode != A380X_B1_SRS && i->managed_speed_valid && s->speed_preselect_seconds <= 0) {
                s->speed_managed = 1; s->speed_dashes = 1;   /* managed T.O speed (V2) */
            }
            s->vertical_mode = A380X_B1_SRS; s->vertical_guidance_selected = 1;
        } else if (s->vertical_mode == A380X_B1_SRS && !takeoff_detent(i)) {
            s->vertical_mode = A380X_B1_NONE; s->vertical_guidance_selected = 0;   /* rejected takeoff / thrust reduced */
        }
    } else {
        s->clb_disarmed = 0;
    }
    if (!valid) return;
    /* ---- lateral ---- */
    if (nav_mode(s) && !i->managed_route_valid) managed_nav_lost(s, i);
    if (s->nav_armed && i->managed_route_valid && !loc_mode(s)) {
        int takeoff = s->airborne_seconds < 180 && i->radio_height_ft < 1500 && !phase_climb;
        if ((takeoff && i->radio_height_ft >= 30) || (!takeoff && i->managed_nav_capture)) engage_nav(s);
    }
    /* ---- vertical ---- */
    if (s->vertical_mode == A380X_B1_SRS && phase_climb) {
        if (s->clb_armed && nav_mode(s) && i->managed_vertical_valid) engage_managed_vertical(s, i, 1);
        else { s->vertical_mode = A380X_B1_OP_CLB; s->altitude_managed = 0; s->capture_seconds = 0; }
    }
    if (nav_mode(s) && i->managed_vertical_valid) {
        double target = managed_altitude_target(s, i);
        if ((s->vertical_mode == A380X_B1_OP_CLB && s->clb_armed) || (s->vertical_mode == A380X_B1_OP_DES && s->des_armed))
            engage_managed_vertical(s, i, s->vertical_mode == A380X_B1_OP_CLB);
        /* ALT CST: resume once the constraint is sequenced */
        if (alt_hold_mode(s) && s->altitude_managed && s->alt_constraint) {
            double next = s->managed_climb ? fmin(i->managed_altitude_ft, s->selected_altitude_ft)
                                           : fmax(i->managed_altitude_ft, s->selected_altitude_ft);
            if (isfinite(next) && (s->managed_climb ? next > s->held_altitude_ft + 100 : next < s->held_altitude_ft - 100))
                engage_managed_vertical(s, i, s->managed_climb);
        }
        (void)target;
    }
    /* armed CLB / DES display while levelled at a constraint */
    if (alt_hold_mode(s) && s->altitude_managed && s->alt_constraint) {
        s->clb_armed = s->managed_climb; s->des_armed = !s->managed_climb;
    }
    if (s->vertical_mode == A380X_B1_DES) {
        managed_des_submode(s, i);
        if (s->des_submode == 0) {
            s->vertical_law_override = 9; s->vpath_valid = 1;
            s->vpath_altitude_ft = i->managed_path_altitude_ft; s->vpath_vs_fpm = i->managed_path_vs_fpm;
        } else if (s->des_submode == 2) {
            s->vertical_law_override = 4;                      /* V/S -1000 ft/min */
        } else {
            s->vertical_law_override = 3;                      /* idle, speed on pitch */
        }
    } else s->des_submode = 0;
    /* ALT CST* / ALT CST qualifier while climbing or descending towards a constraint */
    if (managed_vertical_mode(s)) {
        s->alt_constraint = i->managed_altitude_is_constraint && isfinite(i->managed_altitude_ft) &&
            (s->managed_climb ? i->managed_altitude_ft < s->selected_altitude_ft : i->managed_altitude_ft > s->selected_altitude_ft);
        s->alt_cruise = !s->alt_constraint && i->managed_cruise_altitude_ft > 0 &&
            fabs(s->selected_altitude_ft - i->managed_cruise_altitude_ft) < 50;
    } else if (s->vertical_mode != A380X_B1_ALT && s->vertical_mode != A380X_B1_ALT_STAR) {
        s->alt_constraint = s->alt_cruise = 0;
    }
}

/* After the ALT capture logic: keep the managed qualifiers through ALT* -> ALT. */
static void managed_after_capture(A380XGuidanceState *s, const A380XGuidanceInputs *i, A380XB1Message before, int was_managed)
{
    if (before == s->vertical_mode) return;
    if (s->vertical_mode == A380X_B1_ALT_STAR && (before == A380X_B1_CLB || before == A380X_B1_DES)) {
        /* alt_constraint / alt_cruise were computed for the target being captured */
        return;
    }
    if (s->vertical_mode == A380X_B1_ALT && before == A380X_B1_ALT_STAR) {
        if (was_managed && s->alt_constraint) {
            s->altitude_managed = 1;
            s->clb_armed = s->managed_climb; s->des_armed = !s->managed_climb;
        } else {
            s->alt_constraint = 0;
            s->alt_cruise = s->alt_cruise || (i->managed_cruise_altitude_ft > 0 &&
                fabs(s->held_altitude_ft - i->managed_cruise_altitude_ft) < 50 && i->fms_phase_valid &&
                i->fms_phase >= FG_PH_CLIMB && i->fms_phase <= FG_PH_CRUISE);
            s->clb_armed = s->des_armed = 0;
        }
        return;
    }
    if (s->vertical_mode != A380X_B1_ALT && s->vertical_mode != A380X_B1_ALT_STAR) s->alt_constraint = s->alt_cruise = 0;
}

/* Speed target corrections of the managed vertical modes (before the VLS/VMAX limits). */
static void managed_speed_adjust(A380XGuidanceState *s, const A380XGuidanceInputs *i)
{
    if (s->vertical_mode == A380X_B1_SRS && i->fms_v2_knots >= 100)
        s->speed_target_knots = i->on_ground ? i->fms_v2_knots : i->fms_v2_knots + 10;   /* all engines: V2 + 10 */
    if (s->vertical_mode == A380X_B1_DES && s->speed_managed && s->des_submode == 1 && i->managed_path_valid && i->managed_vdev_ft > 500)
        s->speed_target_knots += 10;   /* upper end of the managed speed range above the path */
}

/* A/THR mode of the vertical mode: 1 = THR CLB, 2 = THR IDLE, 0 = SPEED/MACH. */
static int managed_thrust_kind(const A380XGuidanceState *s)
{
    if (s->vertical_mode == A380X_B1_OP_CLB || s->vertical_mode == A380X_B1_CLB || s->vertical_mode == A380X_B1_SRS) return 1;
    if (s->vertical_mode == A380X_B1_OP_DES) return 2;
    if (s->vertical_mode == A380X_B1_DES) return s->des_thrust_idle ? 2 : 0;
    return 0;
}

static int speed_is_mach(const A380XGuidanceState *s, const A380XGuidanceInputs *i)
{
    return s->speed_managed ? i->managed_speed_is_mach : s->mach_mode;
}
