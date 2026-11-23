/* SPDX-License-Identifier: GPL-3.0-or-later
 * Mode-logic tests of LAND / FLARE / ROLL OUT, RETARD, SRS GA / GA TRK, the AUTOLAND window inputs,
 * LAND capability and the yaw actuator (a380x_autoland.inc) through the public guidance API.
 * Open-loop: sensor inputs are set directly; the closed-loop behaviour is test_autoland_sim.c.
 */
#include "a380x_guidance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static A380XGuidanceInputs approach(void) {
    A380XGuidanceInputs i;
    memset(&i, 0, sizeof(i));
    i.powered = i.attitude_valid = i.airdata_valid = i.athr_speed_valid = 1;
    i.ias_knots = 138; i.tas_knots = i.ground_speed_knots = 140; i.mach = .21;
    i.altitude_ft = i.pressure_altitude_ft = 1500; i.radio_height_ft = 1000; i.radio_height_valid = 1;
    i.vertical_speed_fpm = -720; i.pitch_deg = 2.5; i.weight_kg = 386000; i.vls_knots = 132; i.vmax_knots = 182;
    i.heading_deg = i.track_deg = 142; i.true_heading_deg = i.true_track_deg = 143;
    i.flap_handle_index = 4; i.throttle_ratio = 25.0 / 45.0;
    i.throttle_levers_valid = 1; i.idle_detent_ratio = .01; i.cl_detent_ratio = 25.0 / 45.0;
    i.mct_detent_ratio = 35.0 / 45.0; i.toga_detent_ratio = 1; i.detent_tolerance_ratio = .02;
    for (int n = 0; n < 4; ++n) i.throttle_lever_ratio[n] = i.cl_detent_ratio;
    for (int n = 0; n < 2; ++n) {
        i.ils[n].valid = i.ils[n].loc_valid = i.ils[n].gs_valid = 1;
        i.ils[n].course_deg = 142; i.ils[n].gs_angle_deg = 3; i.ils[n].distance_nm = 3;
        i.ils[n].frequency_mhz = 110.70;
        i.side_radio_height_valid[n] = i.side_attitude_valid[n] = i.side_airdata_valid[n] = 1;
    }
    i.hydraulics_valid = i.rudder_valid = i.yaw_channel_available = 1;
    return i;
}
/* Both radio altimeters follow the selected one; ra2_offset splits the F/O altimeter. */
static double ra2_offset;
static void run(A380XGuidanceState *s, A380XGuidanceInputs *i, double seconds) {
    for (int n = 0; n < (int)round(seconds / .02); ++n) {
        i->side_radio_height_ft[0] = i->radio_height_ft; i->side_radio_height_ft[1] = i->radio_height_ft + ra2_offset;
        a380x_guidance_update(s, i, .02);
        assert(isfinite(s->pitch_servo) && isfinite(s->roll_servo) && isfinite(s->land.yaw_servo));
        assert(fabs(s->land.yaw_servo) <= 1 && s->throttle_command >= 0 && s->throttle_command <= 1);
    }
}
/* Descend at the current vertical speed to a radio height, keeping the RA rate consistent (the FBW
 * flare condition uses the complementary RA rate). */
static void glide(A380XGuidanceState *s, A380XGuidanceInputs *i, double ra) {
    while (i->radio_height_ft > ra + 1e-9) {
        i->radio_height_ft = fmax(ra, i->radio_height_ft + i->vertical_speed_fpm / 60 * .02);
        i->altitude_ft = i->pressure_altitude_ft = 500 + i->radio_height_ft;
        run(s, i, .02);
    }
}
static void cmd(A380XGuidanceState *s, A380XGuidanceInputs *i, A380XFcuCommand c) { a380x_guidance_command(s, i, c, 0); }
static void levers(A380XGuidanceInputs *i, double r) {
    for (int n = 0; n < 4; ++n) i->throttle_lever_ratio[n] = r;
    i->throttle_ratio = r;
}
/* Established on LOC/G/S in APPR with AP1+AP2 and A/THR active (as after the FBW captures). */
static void established(A380XGuidanceState *s, A380XGuidanceInputs *i) {
    *i = approach();
    a380x_guidance_init(s, i);
    run(s, i, 6);
    cmd(s, i, A380X_FCU_AP1); assert(s->ap1);
    cmd(s, i, A380X_FCU_APPR); cmd(s, i, A380X_FCU_AP2); assert(s->ap1 && s->ap2);
    cmd(s, i, A380X_FCU_ATHR);
    s->selected_speed_knots = 138;
    s->lateral_mode = A380X_C1_LOC; s->vertical_mode = A380X_B1_GS; s->loc_armed = s->gs_armed = 0;
    run(s, i, 1);
    assert(s->guidance_valid && s->athr_active && !s->land.land_active);
}

static void test_land_engagement_and_inhibits(void) {
    A380XGuidanceState s; A380XGuidanceInputs i;
    established(&s, &i);
    i.radio_height_ft = 450; run(&s, &i, 2); assert(!s.land.land_active);
    assert(s.land.capability == A380X_LAND_CAPABILITY_LAND3_DUAL);
    i.radio_height_ft = 395; run(&s, &i, 1.0); assert(!s.land.land_active);       /* 1.2 s confirm */
    run(&s, &i, .3); assert(s.land.land_active && s.land.bc1 == A380X_BC1_LAND && s.land.autoland);
    assert(s.ap1 && s.ap2 && s.approach_selected && !s.loc_armed && !s.gs_armed);
    /* FCU lateral/vertical selections and LOC/APPR are ineffective in LAND. */
    cmd(&s, &i, A380X_FCU_APPR); cmd(&s, &i, A380X_FCU_LOC); cmd(&s, &i, A380X_FCU_HDG_PULL);
    cmd(&s, &i, A380X_FCU_ALT_PULL); cmd(&s, &i, A380X_FCU_VS_PULL); cmd(&s, &i, A380X_FCU_ALT_HOLD);
    run(&s, &i, .1);
    assert(s.land.land_active && s.lateral_mode == A380X_C1_LOC && s.vertical_mode == A380X_B1_GS && s.approach_selected);
    cmd(&s, &i, A380X_FCU_SPD_ROTATE); a380x_guidance_command(&s, &i, A380X_FCU_SPD_ROTATE, 2);
    assert(s.selected_speed_knots == 140);   /* speed selection stays available */
    /* Localizer loss in LAND keeps the mode (AUTOLAND light), no reversion; speed below VLS keeps the AP. */
    i.ils[0].loc_valid = 0; i.ias_knots = 128; run(&s, &i, .5);
    assert(s.land.land_active && s.lateral_mode == A380X_C1_LOC && s.ap1 && !s.mode_reversion);
    i.ils[0].loc_valid = 1; i.ias_knots = 138;
    /* Below 100 ft in LAND the AP stays engaged (the old G/S limit does not apply). */
    i.radio_height_ft = 90; run(&s, &i, .5); assert(s.ap1 && s.ap2 && !s.disconnect_warning);
    a380x_guidance_destroy(&s);
}

static void test_land_gating(void) {
    A380XGuidanceState s; A380XGuidanceInputs i;
    /* No valid radio altimeter: no LAND, and the retained 100 ft G/S disconnect still applies. */
    established(&s, &i);
    i.radio_height_valid = 0; i.radio_height_ft = 0; run(&s, &i, .1);
    assert(!s.land.land_active && !s.ap1 && !s.ap2 && s.disconnect_warning);
    a380x_guidance_destroy(&s);
    /* G/S without APPR (no LAND armed): disconnect below 100 ft as before. */
    established(&s, &i);
    s.approach_selected = 0; i.radio_height_ft = 380; run(&s, &i, 2); assert(!s.land.land_active);
    assert(s.land.capability == A380X_LAND_CAPABILITY_NONE);
    i.radio_height_ft = 99; run(&s, &i, .1); assert(!s.ap1 && s.disconnect_warning);
    a380x_guidance_destroy(&s);
    /* FD-only LAND: mode engages for the FD, no autoland window, no RETARD, no yaw lease. */
    established(&s, &i);
    cmd(&s, &i, A380X_FCU_AP_DISCONNECT); assert(!s.ap1 && !s.ap2 && s.fd1);
    i.radio_height_ft = 380; run(&s, &i, 1.4);
    assert(s.land.land_active && !s.land.autoland && s.land.capability == A380X_LAND_CAPABILITY_APPR1);
    i.radio_height_ft = 40; run(&s, &i, .5);
    assert(!s.land.retard_active && s.thrust_mode == A380X_A1A2_SPEED && !s.land.yaw_active && s.land.yaw_servo == 0);
    a380x_guidance_destroy(&s);
}

/* Capability truth table: one condition removed at a time from the full LAND 3 DUAL set. */
static int capability_with(void (*change)(A380XGuidanceState *, A380XGuidanceInputs *)) {
    A380XGuidanceState s; A380XGuidanceInputs i; int capability;
    established(&s, &i);
    i.radio_height_ft = 1500; run(&s, &i, .1);
    assert(s.land.capability == A380X_LAND_CAPABILITY_LAND3_DUAL);
    if (change) change(&s, &i);
    run(&s, &i, 1.2);   /* > the 1 s downgrade confirmation */
    capability = s.land.capability;
    assert(capability == s.land.capability_raw);
    a380x_guidance_destroy(&s);
    return capability;
}
static void c_ap2_off(A380XGuidanceState *s, A380XGuidanceInputs *i) { cmd(s, i, A380X_FCU_AP2); }
static void c_aps_off(A380XGuidanceState *s, A380XGuidanceInputs *i) { cmd(s, i, A380X_FCU_AP_DISCONNECT); }
static void c_athr_off(A380XGuidanceState *s, A380XGuidanceInputs *i) { cmd(s, i, A380X_FCU_ATHR_DISCONNECT); }
static void c_ra2_failed(A380XGuidanceState *s, A380XGuidanceInputs *i) { (void)s; i->side_radio_height_valid[1] = 0; }
static void c_ra_disagree(A380XGuidanceState *s, A380XGuidanceInputs *i) { (void)s; (void)i; ra2_offset = 90; }
static void c_ras_failed(A380XGuidanceState *s, A380XGuidanceInputs *i) {
    (void)s; i->side_radio_height_valid[0] = i->side_radio_height_valid[1] = i->radio_height_valid = 0;
}
static void c_ils2_failed(A380XGuidanceState *s, A380XGuidanceInputs *i) { (void)s; i->ils[1].gs_valid = 0; }
static void c_ils_freq(A380XGuidanceState *s, A380XGuidanceInputs *i) { (void)s; i->ils[1].frequency_mhz = 109.50; }
static void c_ils_course(A380XGuidanceState *s, A380XGuidanceInputs *i) { (void)s; i->ils[1].course_deg = 150; }
static void c_ils_disagree(A380XGuidanceState *s, A380XGuidanceInputs *i) { (void)s; i->ils[1].loc_deviation_dots = .4; }
static void c_adirs2(A380XGuidanceState *s, A380XGuidanceInputs *i) { (void)s; i->side_airdata_valid[1] = 0; }
static void c_attitude2(A380XGuidanceState *s, A380XGuidanceInputs *i) { (void)s; i->side_attitude_valid[1] = 0; }
static void c_hydraulics(A380XGuidanceState *s, A380XGuidanceInputs *i) { (void)s; i->hydraulics_valid = 0; }
static void c_rudder(A380XGuidanceState *s, A380XGuidanceInputs *i) { (void)s; i->rudder_valid = 0; }
static void c_yaw(A380XGuidanceState *s, A380XGuidanceInputs *i) { (void)s; i->yaw_channel_available = 0; }
static void c_ap2_athr_off(A380XGuidanceState *s, A380XGuidanceInputs *i) { c_ap2_off(s, i); c_athr_off(s, i); }
static void test_capability_truth_table(void) {
    static const struct { const char *name; void (*change)(A380XGuidanceState *, A380XGuidanceInputs *); int expected; } rows[] = {
        {"all", NULL, A380X_LAND_CAPABILITY_LAND3_DUAL},
        {"AP2 off (one AP)", c_ap2_off, A380X_LAND_CAPABILITY_LAND3_SINGLE},
        {"F/O ADIRS air data", c_adirs2, A380X_LAND_CAPABILITY_LAND3_SINGLE},
        {"F/O ADIRS attitude", c_attitude2, A380X_LAND_CAPABILITY_LAND3_SINGLE},
        {"hydraulic system 1/2", c_hydraulics, A380X_LAND_CAPABILITY_LAND3_SINGLE},
        {"A/THR off", c_athr_off, A380X_LAND_CAPABILITY_LAND2},
        {"F/O radio altimeter failed", c_ra2_failed, A380X_LAND_CAPABILITY_LAND2},
        {"radio altimeters disagree", c_ra_disagree, A380X_LAND_CAPABILITY_LAND2},
        {"yaw override unavailable", c_yaw, A380X_LAND_CAPABILITY_LAND2},
        {"AP2 and A/THR off", c_ap2_athr_off, A380X_LAND_CAPABILITY_LAND2},
        {"no AP", c_aps_off, A380X_LAND_CAPABILITY_APPR1},
        {"both radio altimeters failed", c_ras_failed, A380X_LAND_CAPABILITY_APPR1},
        {"ILS 2 glide failed", c_ils2_failed, A380X_LAND_CAPABILITY_APPR1},
        {"ILS 2 other frequency", c_ils_freq, A380X_LAND_CAPABILITY_APPR1},
        {"ILS 2 other course", c_ils_course, A380X_LAND_CAPABILITY_APPR1},
        {"ILS deviations disagree", c_ils_disagree, A380X_LAND_CAPABILITY_APPR1},
        {"rudder failed", c_rudder, A380X_LAND_CAPABILITY_APPR1},
    };
    static const char *const names[] = {"none", "APPR 1", "LAND 2", "LAND 3 SINGLE", "LAND 3 DUAL"};
    for (unsigned r = 0; r < sizeof(rows) / sizeof(rows[0]); ++r) {
        int got;
        ra2_offset = 0;
        got = capability_with(rows[r].change);
        printf("  capability %-28s %s\n", rows[r].name, names[got]);
        assert(got == rows[r].expected);
    }
    ra2_offset = 0;
    /* FMA mapping: exactly one capability word bit, D1/D2 message. */
    assert(a380x_land_capability_bit(A380X_LAND_CAPABILITY_LAND2) == 0 && a380x_land_capability_bit(A380X_LAND_CAPABILITY_LAND3_SINGLE) == 1 &&
           a380x_land_capability_bit(A380X_LAND_CAPABILITY_LAND3_DUAL) == 2 && a380x_land_capability_bit(A380X_LAND_CAPABILITY_APPR1) < 0);
    assert(a380x_land_capability_d1d2(A380X_LAND_CAPABILITY_LAND3_DUAL) == A380X_D1D2_LAND_3_DUAL &&
           a380x_land_capability_d1d2(A380X_LAND_CAPABILITY_LAND3_SINGLE) == A380X_D1D2_LAND_3_SINGLE &&
           a380x_land_capability_d1d2(A380X_LAND_CAPABILITY_LAND2) == A380X_D1D2_LAND_2 &&
           a380x_land_capability_d1d2(A380X_LAND_CAPABILITY_APPR1) == A380X_D1D2_APPR_1 &&
           a380x_land_capability_d1d2(A380X_LAND_CAPABILITY_NONE) == A380X_D1D2_NONE);
}

/* Downgrade confirmation and triple-click level, upgrades, cancellation, freezing below 200 ft. */
static void test_capability_downgrade_and_freeze(void) {
    A380XGuidanceState s; A380XGuidanceInputs i;
    established(&s, &i);
    i.radio_height_ft = 1500; run(&s, &i, .1);
    assert(s.land.capability == A380X_LAND_CAPABILITY_LAND3_DUAL && !s.land.capability_downgrade);
    /* AP2 off: shown 1 s later (confirmation), then the downgrade level holds 3 s. */
    cmd(&s, &i, A380X_FCU_AP2); run(&s, &i, .9);
    assert(s.land.capability == A380X_LAND_CAPABILITY_LAND3_DUAL && s.land.capability_raw == A380X_LAND_CAPABILITY_LAND3_SINGLE);
    run(&s, &i, .2); assert(s.land.capability == A380X_LAND_CAPABILITY_LAND3_SINGLE && s.land.capability_downgrade);
    run(&s, &i, 3); assert(!s.land.capability_downgrade);
    /* A brief loss (< 1 s) is not annunciated; an upgrade is immediate and silent. */
    i.ils[1].gs_valid = 0; run(&s, &i, .5); i.ils[1].gs_valid = 1; run(&s, &i, .5);
    assert(s.land.capability == A380X_LAND_CAPABILITY_LAND3_SINGLE && !s.land.capability_downgrade);
    cmd(&s, &i, A380X_FCU_AP2); run(&s, &i, .02);
    assert(s.land.capability == A380X_LAND_CAPABILITY_LAND3_DUAL && !s.land.capability_downgrade);
    /* LAND below 400 ft, then the 200 ft alert height freezes the annunciation. */
    i.radio_height_ft = 390; run(&s, &i, 1.4); assert(s.land.land_active && !s.land.capability_frozen);
    i.radio_height_ft = 190; run(&s, &i, .1); assert(s.land.capability_frozen);
    cmd(&s, &i, A380X_FCU_AP2); i.side_radio_height_valid[1] = 0; run(&s, &i, 2);
    assert(s.land.capability == A380X_LAND_CAPABILITY_LAND3_DUAL && s.land.capability_raw == A380X_LAND_CAPABILITY_LAND2);
    assert(!s.land.capability_downgrade);
    /* Through touchdown and roll-out it stays frozen; the go-around/ground reset clear it. */
    i.radio_height_ft = 0; i.on_ground = 1; run(&s, &i, 1);
    assert(s.land.rollout_active && s.land.capability == A380X_LAND_CAPABILITY_LAND3_DUAL);
    a380x_guidance_destroy(&s);
    /* Cancelling APPR above LAND removes the capability without a downgrade alert. */
    established(&s, &i);
    i.radio_height_ft = 1500; run(&s, &i, .1); cmd(&s, &i, A380X_FCU_APPR); run(&s, &i, 1.5);
    assert(s.land.capability == A380X_LAND_CAPABILITY_NONE && !s.land.capability_downgrade && !s.land.capability_frozen);
    a380x_guidance_destroy(&s);
    /* A go-around below 200 ft ends LAND, the freeze and the capability. */
    established(&s, &i);
    i.radio_height_ft = 390; run(&s, &i, 1.4); i.radio_height_ft = 150; run(&s, &i, .2);
    assert(s.land.capability_frozen);
    levers(&i, 1); run(&s, &i, .1);
    assert(s.land.go_around_active && !s.land.capability_frozen && s.land.capability == A380X_LAND_CAPABILITY_NONE &&
           !s.land.capability_downgrade);
    a380x_guidance_destroy(&s);
}

static void test_flare_retard_rollout_and_athr(void) {
    A380XGuidanceState s; A380XGuidanceInputs i;
    double command;
    established(&s, &i);
    glide(&s, &i, 380); run(&s, &i, 1.4); assert(s.land.land_active);
    /* RETARD at 50 ft with AP + A/THR in LAND: FMA IDLE, command ramps down. */
    glide(&s, &i, 60); assert(!s.land.retard_active && !s.land.flare_active);
    command = s.throttle_command;
    glide(&s, &i, 50); run(&s, &i, .02); assert(s.land.retard_active && s.thrust_mode == A380X_A1A2_THR_IDLE);
    /* FLARE from the imported law: 14 x RA <= |H dot| below 80 ft (720 fpm: 51 ft), or 42 ft. */
    assert(s.land.flare_active && s.land.bc1 == A380X_BC1_FLARE);
    glide(&s, &i, 25); assert(s.land.align_active && s.land.yaw_active);
    assert(s.throttle_command < command && s.athr_active);
    i.vertical_speed_fpm = -150; i.radio_height_ft = 60; run(&s, &i, .1);
    assert(s.land.flare_active);   /* latched */
    run(&s, &i, 5); assert(s.throttle_command == 0);
    /* Touchdown: ROLL OUT after 0.36 s on ground below 50 ft; guidance stays valid at low speed. */
    i.radio_height_ft = 0; i.on_ground = 1; i.vertical_speed_fpm = 0; i.pitch_deg = 1; run(&s, &i, .3);
    assert(!s.land.rollout_active);
    run(&s, &i, .1); assert(s.land.rollout_active && s.land.bc1 == A380X_BC1_ROLL_OUT && s.land.yaw_active);
    assert(s.ap1 && s.ap2 && s.guidance_valid && s.pitch_servo >= -.3 && s.pitch_servo <= .15);
    /* RETARD keeps A/THR at idle on the ground with the levers still at CL ... */
    run(&s, &i, 1); assert(s.athr_active && s.throttle_command == 0 && s.thrust_mode == A380X_A1A2_THR_IDLE);
    i.ias_knots = i.tas_knots = i.ground_speed_knots = 30; run(&s, &i, 1); assert(s.ap1 && s.guidance_valid);
    /* ... until the crew sets IDLE: A/THR disconnects, no RETARD. */
    levers(&i, 0); run(&s, &i, .1); assert(!s.athr_armed && !s.athr_active && !s.land.retard_active);
    /* AP disconnect releases the yaw command; ROLL OUT stays for the FD. */
    cmd(&s, &i, A380X_FCU_AP_DISCONNECT); run(&s, &i, .1);
    assert(!s.ap1 && !s.land.yaw_active && s.land.yaw_servo == 0 && s.land.rollout_active);
    /* Ground reset: 10 s on ground without AP, then APPR (or both FD off). */
    run(&s, &i, 9); cmd(&s, &i, A380X_FCU_APPR); run(&s, &i, .1);
    assert(!s.land.land_active && !s.approach_selected && s.vertical_mode == A380X_B1_NONE && !s.guidance_valid);
    a380x_guidance_destroy(&s);
    /* Reverse selection disconnects A/THR from RETARD as well. */
    established(&s, &i);
    i.radio_height_ft = 380; run(&s, &i, 1.4); i.radio_height_ft = 30; run(&s, &i, .5);
    assert(s.land.retard_active);
    i.radio_height_ft = 0; i.on_ground = 1; run(&s, &i, 1);
    i.reverse_selected[1] = i.reverse_selected[2] = 1; run(&s, &i, .1);
    assert(!s.athr_armed && !s.land.retard_active);
    a380x_guidance_destroy(&s);
}

static void test_go_around(void) {
    A380XGuidanceState s; A380XGuidanceInputs i;
    established(&s, &i);
    i.radio_height_ft = 380; run(&s, &i, 1.4); i.radio_height_ft = 45; run(&s, &i, .5);
    assert(s.land.land_active && s.land.retard_active);
    i.ias_knots = 141;
    levers(&i, 1); run(&s, &i, .1);
    assert(s.land.go_around_active && !s.land.land_active && !s.land.retard_active && !s.land.flare_active);
    assert(s.vertical_mode == A380X_B1_SRS && s.lateral_mode == A380X_C1_GA_TRK && !s.approach_selected);
    assert(s.ap1 && s.ap2 && s.athr_armed && !s.athr_active && s.thrust_mode == A380X_A1A2_MAN_TOGA);
    assert(fabs(s.speed_target_knots - 141) < .5 && s.guidance_valid);
    i.vertical_speed_fpm = 1500; i.radio_height_ft = 300; i.pitch_deg = 12; run(&s, &i, 1);
    assert(s.pitch_command_deg > 0 && s.ap1 && s.ap2);
    /* Back to CL: A/THR active in a thrust mode under SRS. */
    levers(&i, 25.0 / 45.0); run(&s, &i, .5);
    assert(s.athr_active && s.thrust_mode == A380X_A1A2_THR_CLB);
    /* Leaving SRS and GA TRK ends the go-around state; dual AP then reduces to one. */
    cmd(&s, &i, A380X_FCU_VS_PULL); run(&s, &i, .1); assert(s.land.go_around_active);
    cmd(&s, &i, A380X_FCU_HDG_PULL); run(&s, &i, .1); assert(!s.land.go_around_active && s.ap1 && !s.ap2);
    a380x_guidance_destroy(&s);
    /* No GA without flaps, nor 30 s after touchdown (FBW ConfirmNode_timeDelay_a). */
    established(&s, &i);
    i.flap_handle_index = 0; levers(&i, 1); run(&s, &i, .2); assert(!s.land.go_around_active);
    a380x_guidance_destroy(&s);
    established(&s, &i);
    i.radio_height_ft = 380; run(&s, &i, 1.4);
    i.radio_height_ft = 0; i.on_ground = 1; i.vertical_speed_fpm = 0; run(&s, &i, 31);
    levers(&i, 1); run(&s, &i, .2); assert(!s.land.go_around_active && s.land.rollout_active);
    a380x_guidance_destroy(&s);
    /* GA from the roll-out (within 30 s): SRS/GA TRK for the FD, the AP disengages on the ground. */
    established(&s, &i);
    i.radio_height_ft = 380; run(&s, &i, 1.4);
    i.radio_height_ft = 0; i.on_ground = 1; i.vertical_speed_fpm = 0; run(&s, &i, 2);
    assert(s.land.rollout_active && s.ap1);
    levers(&i, 1); run(&s, &i, .1);
    assert(s.land.go_around_active && s.vertical_mode == A380X_B1_SRS && !s.ap1 && !s.ap2 && s.disconnect_warning);
    assert(s.guidance_valid);
    a380x_guidance_destroy(&s);
    /* Pause holds the yaw output at zero (the adapter releases the lease). */
    established(&s, &i);
    i.radio_height_ft = 380; run(&s, &i, 1.4); i.radio_height_ft = 0; i.on_ground = 1; run(&s, &i, .5);
    assert(s.land.yaw_active);
    i.paused = 1; a380x_guidance_update(&s, &i, .02); assert(s.land.yaw_servo == 0 && s.pitch_servo == 0);
    a380x_guidance_destroy(&s);
}

int main(void) {
    test_land_engagement_and_inhibits();
    test_land_gating();
    test_capability_truth_table();
    test_capability_downgrade_and_freeze();
    test_flare_retard_rollout_and_athr();
    test_go_around();
    puts("A380 autoland mode logic: LAND/FLARE/ROLL OUT, RETARD, ground A/THR, SRS GA/GA TRK, LAND 2/3 capability truth table, downgrade/freeze and yaw lease passed");
    return 0;
}
