/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "a380x_guidance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static A380XGuidanceInputs fixture(void) {
    A380XGuidanceInputs i;
    memset(&i,0,sizeof(i));
    i.powered=i.attitude_valid=i.airdata_valid=1;
    i.athr_speed_valid=1;
    i.ias_knots=250; i.tas_knots=i.ground_speed_knots=300; i.mach=.45;
    i.altitude_ft=i.pressure_altitude_ft=10000; i.radio_height_ft=7000;
    i.pitch_deg=3; i.weight_kg=450000; i.vls_knots=140; i.vmax_knots=340;
    i.throttle_ratio=.5;
    i.throttle_levers_valid=1;
    i.idle_detent_ratio=.01; i.cl_detent_ratio=25.0/45.0;
    i.mct_detent_ratio=35.0/45.0; i.toga_detent_ratio=1;
    i.detent_tolerance_ratio=.02;
    for(int n=0;n<4;n++) i.throttle_lever_ratio[n]=i.cl_detent_ratio;
    i.ils[0].course_deg=i.ils[1].course_deg=0;
    i.ils[0].gs_angle_deg=i.ils[1].gs_angle_deg=3;
    i.ils[0].distance_nm=i.ils[1].distance_nm=10;
    return i;
}
static void run(A380XGuidanceState *s,A380XGuidanceInputs *i,double sec) {
    for(int n=0;n<(int)round(sec/.02);n++) {
        a380x_guidance_update(s,i,.02);
        assert(isfinite(s->pitch_servo) && isfinite(s->roll_servo));
        assert(isfinite(s->pitch_command_deg) && isfinite(s->roll_command_deg));
        assert(fabs(s->pitch_servo)<=1 && fabs(s->roll_servo)<=1);
        assert(s->throttle_command>=0 && s->throttle_command<=1);
    }
}
static void cmd(A380XGuidanceState *s,A380XGuidanceInputs *i,A380XFcuCommand c,double a) {
    a380x_guidance_command(s,i,c,a);
}
static void engage(A380XGuidanceState *s,A380XGuidanceInputs *i) {
    run(s,i,6); cmd(s,i,A380X_FCU_AP1,0); assert(s->ap1); run(s,i,5);
}
static void test_controls_and_gating(void) {
    A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
    a380x_guidance_init(&s,&i);
    assert(s.fbw_laws);
    cmd(&s,&i,A380X_FCU_AP1,0); assert(!s.ap1);
    cmd(&s,&i,A380X_FCU_SPD_ROTATE,1000); assert(s.selected_speed_knots==399);
    cmd(&s,&i,A380X_FCU_SPD_ROTATE,-1000); assert(s.selected_speed_knots==100);
    cmd(&s,&i,A380X_FCU_HDG_ROTATE,-1); assert(s.selected_heading_deg==359);
    cmd(&s,&i,A380X_FCU_HDG_ROTATE,2); assert(s.selected_heading_deg==1);
    cmd(&s,&i,A380X_FCU_VS_PUSH,0); assert(s.vertical_mode==A380X_B1_ALT);
    cmd(&s,&i,A380X_FCU_HDG_PUSH,0); assert(!s.heading_managed && s.managed_unavailable);
    cmd(&s,&i,A380X_FCU_ALT_PUSH,0); assert(!s.altitude_managed);
    cmd(&s,&i,A380X_FCU_SPD_PUSH,0); assert(!s.speed_managed);
    i.true_heading_deg=12;
    cmd(&s,&i,A380X_FCU_TRUE_MAG,0); assert(s.true_heading && s.selected_heading_deg==13);
    cmd(&s,&i,A380X_FCU_TRUE_MAG,0); assert(!s.true_heading && s.selected_heading_deg==1);
    s.selected_speed_knots=250;
    cmd(&s,&i,A380X_FCU_SPD_MACH,0); assert(fabs(s.selected_mach-.45)<.001);
    cmd(&s,&i,A380X_FCU_SPD_MACH,0); assert(s.selected_speed_knots==250);
    cmd(&s,&i,A380X_FCU_VS_ROTATE,2); assert(s.selected_vs_fpm==100);
    cmd(&s,&i,A380X_FCU_VS_PULL,0); assert(s.vertical_mode==A380X_B1_VS);
    cmd(&s,&i,A380X_FCU_VS_PUSH,0); assert(s.vertical_mode==A380X_B1_VS && s.selected_vs_fpm==100);
    cmd(&s,&i,A380X_FCU_ALT_HOLD,0); assert(s.vertical_mode==A380X_B1_ALT && s.held_altitude_ft==i.altitude_ft);
    i.managed_speed_valid=1; i.managed_speed_knots=280;
    cmd(&s,&i,A380X_FCU_SPD_PUSH,0); run(&s,&i,.1); assert(s.speed_managed && s.speed_dashes);
    cmd(&s,&i,A380X_FCU_SPD_ROTATE,1); assert(s.selected_speed_knots==281 && !s.speed_dashes);
    run(&s,&i,46); assert(s.speed_dashes && s.speed_target_knots==280);
    i.managed_speed_valid=0; run(&s,&i,.1); assert(!s.speed_managed && s.managed_unavailable);
    a380x_guidance_destroy(&s);
}
static void test_actual_fbw_demands(void) {
    A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
    a380x_guidance_init(&s,&i); engage(&s,&i);
    cmd(&s,&i,A380X_FCU_HDG_ROTATE,90); run(&s,&i,10);
    assert(s.guidance_valid && s.roll_command_deg>5 && s.roll_servo>0);
    printf("FBW HDG right: roll=%.2f servo=%.3f FD=%.2f\n",s.roll_command_deg,s.roll_servo,s.fd_roll_error_deg);
    cmd(&s,&i,A380X_FCU_HDG_ROTATE,-180); run(&s,&i,15);
    assert(s.roll_command_deg < -5 && s.roll_servo<0);
    s.selected_altitude_ft=15000;
    cmd(&s,&i,A380X_FCU_VS_ROTATE,30); cmd(&s,&i,A380X_FCU_VS_PULL,0); run(&s,&i,15);
    printf("FBW VS up: pitch=%.2f servo=%.3f FD=%.2f\n",s.pitch_command_deg,s.pitch_servo,s.fd_pitch_error_deg);
    assert(s.pitch_command_deg>i.pitch_deg && s.pitch_servo>0);
    cmd(&s,&i,A380X_FCU_ATHR,0); s.selected_speed_knots=280; run(&s,&i,5);
    assert(s.athr_active && s.thrust_mode==A380X_A1A2_SPEED && s.throttle_command>.6);
    cmd(&s,&i,A380X_FCU_ALT_PULL,0); run(&s,&i,2); assert(s.thrust_mode==A380X_A1A2_THR_CLB);
    cmd(&s,&i,A380X_FCU_AP_DISCONNECT,0); assert(!s.ap1 && s.pitch_servo==0 && s.roll_servo==0);
    cmd(&s,&i,A380X_FCU_ATHR_DISCONNECT,0); run(&s,&i,.1); assert(!s.athr_armed && !s.athr_active);
    a380x_guidance_destroy(&s);
}
static void test_manual_ap_timing_and_fd_powerup(void) {
    A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
    i.on_ground=1; i.radio_height_ft=0;
    a380x_guidance_init(&s,&i);
    run(&s,&i,10); cmd(&s,&i,A380X_FCU_AP1,0); assert(!s.ap1 && s.airborne_seconds==0);
    i.on_ground=0; i.radio_height_ft=1000;
    run(&s,&i,4.98); cmd(&s,&i,A380X_FCU_AP1,0); assert(!s.ap1);
    run(&s,&i,.02); cmd(&s,&i,A380X_FCU_AP1,0); assert(s.ap1);
    cmd(&s,&i,A380X_FCU_AP2,0); assert(!s.ap1 && s.ap2);
    cmd(&s,&i,A380X_FCU_AP1,0); assert(s.ap1 && !s.ap2);
    cmd(&s,&i,A380X_FCU_APPR,0); cmd(&s,&i,A380X_FCU_AP2,0); assert(s.ap1 && s.ap2);
    a380x_guidance_destroy(&s);
    i.on_ground=1; i.radio_height_ft=0;
    a380x_guidance_init(&s,&i);
    assert(s.fd1 && s.fd2 && !s.guidance_valid);
    cmd(&s,&i,A380X_FCU_FD1,0); cmd(&s,&i,A380X_FCU_FD2,0); assert(!s.fd1 && !s.fd2);
    i.powered=0; run(&s,&i,.1); assert(!s.fd1 && !s.fd2);
    i.powered=1; run(&s,&i,.1);
    assert(s.fd1 && s.fd2 && !s.guidance_valid && s.vertical_mode==A380X_B1_NONE);
    a380x_guidance_destroy(&s);
}
static void pilot_levers(A380XGuidanceInputs *i,double ratio) {
    for(int n=0;n<4;n++) i->throttle_lever_ratio[n]=ratio;
    i->throttle_ratio=ratio;
}
static void test_manual_athr_detents_and_disconnects(void) {
    A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
    i.on_ground=1; pilot_levers(&i,0);
    a380x_guidance_init(&s,&i);
    cmd(&s,&i,A380X_FCU_ATHR,0); run(&s,&i,.1);
    assert(s.athr_armed && !s.athr_active); /* Ground IDLE permits manual arming. */
    pilot_levers(&i,i.toga_detent_ratio); run(&s,&i,.1);
    assert(s.athr_armed && !s.athr_active && s.thrust_mode==A380X_A1A2_MAN_TOGA);
    cmd(&s,&i,A380X_FCU_ATHR,0); run(&s,&i,1); assert(!s.athr_armed);
    pilot_levers(&i,i.cl_detent_ratio); run(&s,&i,.1); assert(!s.athr_armed);
    pilot_levers(&i,i.toga_detent_ratio); run(&s,&i,.1); assert(s.athr_armed);
    i.on_ground=0; run(&s,&i,.1); assert(s.athr_armed && !s.athr_active);
    pilot_levers(&i,i.cl_detent_ratio); run(&s,&i,.1);
    assert(s.athr_armed && s.athr_active && s.thrust_mode==A380X_A1A2_SPEED);
    pilot_levers(&i,0); run(&s,&i,.1); assert(!s.athr_armed && !s.athr_active);
    pilot_levers(&i,i.cl_detent_ratio); cmd(&s,&i,A380X_FCU_ATHR,0); run(&s,&i,.1);
    assert(s.athr_active);
    i.reverse_selected[1]=1; run(&s,&i,.1); assert(s.athr_active);
    i.reverse_selected[2]=1; run(&s,&i,.1); assert(!s.athr_armed && !s.athr_active);
    i.reverse_selected[1]=i.reverse_selected[2]=0;
    cmd(&s,&i,A380X_FCU_ATHR,0); run(&s,&i,.1); assert(s.athr_active);
    cmd(&s,&i,A380X_FCU_ATHR_DISCONNECT,0); run(&s,&i,.1); assert(!s.athr_armed && !s.athr_active);
    a380x_guidance_destroy(&s);

    i=fixture(); i.on_ground=1;
    a380x_guidance_init(&s,&i);
    pilot_levers(&i,i.mct_detent_ratio); run(&s,&i,.1);
    assert(!s.athr_armed && s.thrust_mode==A380X_A1A2_MAN_MCT);
    i.flex_valid=1; run(&s,&i,.1);
    assert(s.athr_armed && !s.athr_active && s.thrust_mode==A380X_A1A2_MAN_FLEX);
    a380x_guidance_destroy(&s);
}
static void test_athr_validation_envelope_and_alpha_floor(void) {
    A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
    a380x_guidance_init(&s,&i);
    s.selected_speed_knots=399; run(&s,&i,.1);
    assert(s.selected_speed_knots==399 && s.speed_target_knots==340);
    s.selected_speed_knots=100; run(&s,&i,.1);
    assert(s.selected_speed_knots==100 && s.speed_target_knots==140);
    s.selected_speed_knots=399;
    cmd(&s,&i,A380X_FCU_ATHR,0); run(&s,&i,20);
    assert(s.athr_active && s.throttle_command<=.88);
    i.attitude_valid=0; i.airdata_valid=0; run(&s,&i,.1);
    assert(!s.guidance_valid && s.athr_active); /* ADR and IR operate independently. */
    i.athr_speed_valid=0; run(&s,&i,.1); assert(!s.athr_active);
    i.athr_speed_valid=1; run(&s,&i,.1); assert(s.athr_active);
    i.attitude_valid=i.airdata_valid=1;
    pilot_levers(&i,i.cl_detent_ratio*.25); run(&s,&i,.1);
    assert(s.athr_active && s.throttle_command<=.22000001);
    i.throttle_levers_valid=0; run(&s,&i,.1); assert(!s.athr_active);
    i.throttle_levers_valid=1; i.throttle_lever_ratio[2]=NAN;
    run(&s,&i,.1); assert(!s.athr_active);
    pilot_levers(&i,i.cl_detent_ratio); i.cl_detent_ratio=i.mct_detent_ratio+.1;
    run(&s,&i,.1); assert(!s.athr_active);
    i.cl_detent_ratio=25.0/45.0;
    cmd(&s,&i,A380X_FCU_ATHR_DISCONNECT,0);
    i.alpha_floor_request=1; run(&s,&i,.1);
    assert(!s.athr_active && !s.alpha_floor_latched); /* No qualified FE source. */
    i.alpha_floor_valid=1; run(&s,&i,.1);
    assert(s.athr_active && s.thrust_mode==A380X_A1A2_A_FLOOR && s.throttle_command==1);
    pilot_levers(&i,0); run(&s,&i,.1);
    assert(s.athr_active && s.throttle_command==1); /* FE demand overrides handle position. */
    i.alpha_floor_request=0; run(&s,&i,.1);
    assert(s.athr_active && s.thrust_mode==A380X_A1A2_TOGA_LK);
    i.alpha_floor_request=1; cmd(&s,&i,A380X_FCU_ATHR_DISCONNECT,0);
    assert(s.thrust_mode==A380X_A1A2_NONE); /* Immediate authority-guard publishing. */
    run(&s,&i,1);
    assert(!s.athr_armed && !s.athr_active && !s.alpha_floor_latched);
    i.alpha_floor_request=0; run(&s,&i,.1);
    i.alpha_floor_request=1; run(&s,&i,.1);
    assert(s.athr_active && s.thrust_mode==A380X_A1A2_A_FLOOR);
    i.powered=0; run(&s,&i,.1); assert(!s.athr_armed && !s.athr_active);
    a380x_guidance_destroy(&s);
}
/* Thrust-lever / FADEC module inputs (docs/THRUST_LEVERS.md): FBW FADEC ATHR_disabled after a
 * 15 s instinctive disconnect, and the A/THR authority from the lever's FADEC N1. */
static void test_athr_inhibit_and_fadec_ceiling(void) {
    A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
    a380x_guidance_init(&s,&i);
    cmd(&s,&i,A380X_FCU_ATHR,0); run(&s,&i,.1); assert(s.athr_active);
    i.athr_inhibited=1; run(&s,&i,.1); assert(!s.athr_armed && !s.athr_active);
    cmd(&s,&i,A380X_FCU_ATHR,0); run(&s,&i,.1); assert(!s.athr_armed && !s.athr_active);
    i.alpha_floor_valid=i.alpha_floor_request=1; run(&s,&i,.1);
    assert(!s.athr_active && !s.alpha_floor_latched && s.thrust_mode!=A380X_A1A2_A_FLOOR);
    i.alpha_floor_request=0; i.on_ground=1; pilot_levers(&i,i.toga_detent_ratio); run(&s,&i,.1);
    assert(!s.athr_armed && s.thrust_mode==A380X_A1A2_MAN_TOGA); /* take-off detent cannot arm */
    i.athr_inhibited=0; i.on_ground=0; pilot_levers(&i,i.cl_detent_ratio); run(&s,&i,.1);
    cmd(&s,&i,A380X_FCU_ATHR,0); run(&s,&i,.1); assert(s.athr_active);
    i.athr_ceiling_valid=1; i.athr_ceiling=.97; s.selected_speed_knots=399; run(&s,&i,20);
    assert(s.athr_active && s.throttle_command>.88 && s.throttle_command<=.97+1e-9);
    i.athr_ceiling=.6; run(&s,&i,5); assert(s.throttle_command<=.6+1e-9);
    a380x_guidance_destroy(&s);
}
static void test_open_altitude_capture_paths(void) {
    /* Sensor-replay mode transitions, not an aircraft performance claim. The
     * imported speed-on-pitch laws remain authoritative for OP CLB/OP DES. */
    for(int direction=-1;direction<=1;direction+=2) {
        A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
        double target=10000+direction*2000;
        a380x_guidance_init(&s,&i); engage(&s,&i);
        s.selected_altitude_ft=target;
        cmd(&s,&i,A380X_FCU_ALT_PULL,0); cmd(&s,&i,A380X_FCU_ATHR,0); run(&s,&i,.1);
        assert(s.vertical_mode==(direction>0?A380X_B1_OP_CLB:A380X_B1_OP_DES));
        assert(s.thrust_mode==(direction>0?A380X_A1A2_THR_CLB:A380X_A1A2_THR_IDLE));
        i.altitude_ft=i.pressure_altitude_ft=target-direction*200;
        i.vertical_speed_fpm=direction*1500; run(&s,&i,.1);
        assert(s.vertical_mode==A380X_B1_ALT_STAR && s.ap1);
        i.altitude_ft=i.pressure_altitude_ft=target-direction*10;
        i.vertical_speed_fpm=direction*20; run(&s,&i,1.2);
        assert(s.vertical_mode==A380X_B1_ALT && s.held_altitude_ft==target && s.ap1);
        assert(s.thrust_mode==A380X_A1A2_SPEED);
        cmd(&s,&i,A380X_FCU_ALT_ROTATE,direction*10); run(&s,&i,.1);
        assert(s.altitude_target_ft==target && s.selected_altitude_ft==target+direction*1000);
        a380x_guidance_destroy(&s);
    }
}
static void test_capture_and_failures(void) {
    A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
    a380x_guidance_init(&s,&i); engage(&s,&i);
    s.selected_altitude_ft=12000; cmd(&s,&i,A380X_FCU_ALT_PULL,0); run(&s,&i,.1);
    assert(s.vertical_mode==A380X_B1_OP_CLB && s.altitude_armed);
    i.altitude_ft=11800; i.vertical_speed_fpm=1800; run(&s,&i,.1);
    assert(s.vertical_mode==A380X_B1_ALT_STAR);
    i.altitude_ft=11990; i.vertical_speed_fpm=20; run(&s,&i,1.2);
    assert(s.vertical_mode==A380X_B1_ALT && s.held_altitude_ft==12000);
    cmd(&s,&i,A380X_FCU_ALT_ROTATE,10); run(&s,&i,.1); assert(s.altitude_target_ft==12000);
    i.attitude_valid=0; run(&s,&i,.1); assert(!s.ap1 && !s.guidance_valid && s.disconnect_warning);
    i.attitude_valid=1; i.roll_deg=41; cmd(&s,&i,A380X_FCU_AP1,0); assert(!s.ap1);
    i.roll_deg=0; cmd(&s,&i,A380X_FCU_AP1,0); assert(s.ap1);
    i.paused=1; run(&s,&i,.1); assert(s.pitch_servo==0 && s.roll_servo==0);
    i.paused=0; i.pitch_deg=NAN; run(&s,&i,.1); assert(!s.ap1 && !s.guidance_valid);
    a380x_guidance_destroy(&s);
}
static void test_ils(void) {
    A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
    a380x_guidance_init(&s,&i); engage(&s,&i);
    cmd(&s,&i,A380X_FCU_APPR,0); cmd(&s,&i,A380X_FCU_AP2,0); assert(s.ap1 && s.ap2);
    run(&s,&i,1); assert(s.loc_armed && s.gs_armed && s.lateral_mode==A380X_C1_HDG);
    i.ils[0].valid=i.ils[0].loc_valid=i.ils[0].gs_valid=1;
    i.ils[0].loc_deviation_dots=.6; i.ils[0].gs_deviation_dots=1;
    run(&s,&i,.1); assert(s.lateral_mode==A380X_C1_LOC_STAR && s.gs_armed);
    run(&s,&i,5);
    printf("FBW LOC needle right: roll=%.2f FD=%.2f\n",s.roll_command_deg,s.fd_roll_error_deg);
    assert(s.roll_command_deg>0);
    i.ils[0].loc_deviation_dots=.05; i.ils[0].gs_deviation_dots=-.3;
    run(&s,&i,2.2); assert(s.lateral_mode==A380X_C1_LOC && s.gs_armed); /* above GS: no capture */
    i.ils[0].gs_deviation_dots=.5; run(&s,&i,.1);
    i.ils[0].gs_deviation_dots=.1; run(&s,&i,.1); assert(s.vertical_mode==A380X_B1_GS_STAR);
    run(&s,&i,2.2); assert(s.vertical_mode==A380X_B1_GS);
    i.ils[0].gs_valid=0; run(&s,&i,.1); assert(s.vertical_mode==A380X_B1_VS && s.mode_reversion && !s.ap2);
    i.ils[0].loc_valid=0; run(&s,&i,.1); assert(s.lateral_mode==A380X_C1_HDG);
    cmd(&s,&i,A380X_FCU_AP2,0); assert(s.ap2 && !s.ap1);
    run(&s,&i,.1); assert(s.source_side==1);
    a380x_guidance_destroy(&s);
}
static void test_vertical_and_fpa_signs(void) {
    double up,down;
    for(int n=0;n<2;n++) {
        A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
        a380x_guidance_init(&s,&i); engage(&s,&i);
        s.selected_altitude_ft=n?5000:15000;
        cmd(&s,&i,A380X_FCU_VS_ROTATE,n?-30:30); cmd(&s,&i,A380X_FCU_VS_PULL,0);
        run(&s,&i,15);
        if(n) { down=s.pitch_command_deg; assert(s.fd_pitch_error_deg<0 && s.pitch_servo<0); }
        else { up=s.pitch_command_deg; assert(s.fd_pitch_error_deg>0 && s.pitch_servo>0); }
        cmd(&s,&i,A380X_FCU_TRK_FPA,0); run(&s,&i,1);
        assert(s.vertical_mode==A380X_B1_FPA);
        assert(n?s.selected_fpa_deg<0:s.selected_fpa_deg>0);
        a380x_guidance_destroy(&s);
    }
    printf("FBW VS up/down pitch: %.3f / %.3f\n",up,down);
    assert(up>down);
    for(int n=0;n<2;n++) {
        A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
        a380x_guidance_init(&s,&i); engage(&s,&i);
        i.altitude_ft+=n?100:-100; i.pressure_altitude_ft=i.altitude_ft; run(&s,&i,15);
        printf("FBW ALT %s: pitch=%.3f FD=%.3f servo=%.3f\n",n?"above":"below",s.pitch_command_deg,s.fd_pitch_error_deg,s.pitch_servo);
        assert(n?s.fd_pitch_error_deg<0:s.fd_pitch_error_deg>0);
        assert(n?s.pitch_servo<0:s.pitch_servo>0);
        a380x_guidance_destroy(&s);
    }
}
static void test_ils_magnetic_reference(void) {
    const double magnetic[]={0,0,355,5}, actual[]={12,348,7,353};
    for(int n=0;n<4;n++) {
        A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
        i.heading_deg=i.track_deg=magnetic[n];
        i.true_heading_deg=i.true_track_deg=actual[n];
        i.ils[0].course_deg=magnetic[n];
        a380x_guidance_init(&s,&i); engage(&s,&i);
        i.ils[0].valid=i.ils[0].loc_valid=1; s.lateral_mode=A380X_C1_LOC;
        run(&s,&i,15); assert(fabs(s.roll_command_deg)<.1);
        a380x_guidance_destroy(&s);
    }
}
static void test_closed_loop_plant(void) {
    /* Synthetic actuator/airframe response, NOT an X-Plane A380 flight model:
     * inertial pitch/roll acceleration, coordinated turns, 3s vertical lag.
     * Regression proves bounded closed-loop feedback and mode convergence. */
    for(int direction=-1;direction<=1;direction+=2) {
        A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
        double max_bank=0,overshoot=0,target=10000+direction*1000;
        a380x_guidance_init(&s,&i); engage(&s,&i);
        cmd(&s,&i,A380X_FCU_HDG_ROTATE,direction*90);
        cmd(&s,&i,A380X_FCU_ALT_ROTATE,direction*10);
        cmd(&s,&i,A380X_FCU_VS_ROTATE,direction*20);
        cmd(&s,&i,A380X_FCU_VS_PULL,0);
        for(int n=0;n<18000;n++) {
            a380x_guidance_update(&s,&i,.02);
            assert(s.ap1 && s.guidance_valid);
            i.pitch_rate_deg_s+=(3*s.pitch_servo-.7*i.pitch_rate_deg_s)*.02;
            i.roll_rate_deg_s+=(10*s.roll_servo-i.roll_rate_deg_s)*.02;
            i.pitch_deg+=i.pitch_rate_deg_s*.02; i.roll_deg+=i.roll_rate_deg_s*.02;
            i.heading_deg=fmod(i.heading_deg+9.80665*tan(i.roll_deg*.0174532925)/
                (300*.514444)*57.2957795*.02+360,360);
            i.true_heading_deg=i.track_deg=i.true_track_deg=i.heading_deg;
            double vs_target=tan((i.pitch_deg-3)*.0174532925)*300*101.2686;
            i.accel_z_m_s2=(vs_target-i.vertical_speed_fpm)/3*.3048/60;
            i.alpha_deg=i.pitch_deg-atan2(i.vertical_speed_fpm,300*101.2686)/.0174532925;
            i.vertical_speed_fpm+=(vs_target-i.vertical_speed_fpm)*.02/3;
            i.altitude_ft+=i.vertical_speed_fpm*.02/60; i.pressure_altitude_ft=i.altitude_ft;
            max_bank=fmax(max_bank,fabs(i.roll_deg));
            overshoot=fmax(overshoot,(i.altitude_ft-target)*direction);
        }
        printf("Synthetic %s: heading %.3f alt error %.3fft VS %.3f maxbank %.2f overshoot %.2fft\n",
            direction>0?"climb":"descent",i.heading_deg,i.altitude_ft-target,i.vertical_speed_fpm,max_bank,overshoot);
        assert(fabs(i.heading_deg-(direction>0?90:270))<.2);
        assert(fabs(i.altitude_ft-target)<5 && fabs(i.vertical_speed_fpm)<10);
        assert(max_bank<=30 && overshoot<60 && s.vertical_mode==A380X_B1_ALT);
        a380x_guidance_destroy(&s);
    }
}
static void test_ground_initialization_standby(void) {
    A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
    i.on_ground=1; i.altitude_ft=i.pressure_altitude_ft=500; i.radio_height_ft=0;
    a380x_guidance_init(&s,&i);
    assert(!s.vertical_guidance_selected && s.vertical_mode==A380X_B1_NONE);
    run(&s,&i,2); assert(!s.guidance_valid && !s.ap1);
    i.on_ground=0; i.altitude_ft=i.pressure_altitude_ft=2000; i.radio_height_ft=1500;
    run(&s,&i,6);
    assert(!s.guidance_valid && !s.vertical_guidance_selected && s.vertical_mode==A380X_B1_NONE);
    assert(s.fd_pitch_error_deg==0 && s.pitch_servo==0);
    cmd(&s,&i,A380X_FCU_AP1,0);
    assert(s.ap1 && s.vertical_guidance_selected && s.held_altitude_ft==2000);
    run(&s,&i,10);
    assert(s.guidance_valid && s.altitude_target_ft==2000 && fabs(s.fd_pitch_error_deg)<.1);
    a380x_guidance_destroy(&s);
    i.on_ground=1; i.altitude_ft=i.pressure_altitude_ft=500; i.radio_height_ft=0;
    a380x_guidance_init(&s,&i);
    cmd(&s,&i,A380X_FCU_ALT_PULL,0);
    assert(s.vertical_guidance_selected && s.vertical_mode==A380X_B1_OP_CLB);
    i.on_ground=0; i.altitude_ft=i.pressure_altitude_ft=2000; i.radio_height_ft=1500;
    run(&s,&i,1);
    assert(s.guidance_valid && s.vertical_mode==A380X_B1_OP_CLB);
    a380x_guidance_destroy(&s);
}
static void test_frame_timing_and_invalid_data(void) {
    A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
    a380x_guidance_init(&s,&i);
    for(int n=0;n<12;n++) a380x_guidance_update(&s,&i,.5);
    assert(fabs(s.airborne_seconds-6)<1e-6);
    cmd(&s,&i,A380X_FCU_AP1,0); assert(s.ap1);
    i.paused=1; a380x_guidance_update(&s,&i,.5); assert(s.ap1 && s.pitch_servo==0);
    assert(fabs(s.airborne_seconds-6)<1e-6);
    i.paused=0; i.pitch_rate_deg_s=NAN; a380x_guidance_update(&s,&i,.02);
    assert(!s.ap1 && !s.guidance_valid);
    i.pitch_rate_deg_s=0; run(&s,&i,1); assert(s.guidance_valid);
    i.ils[0].valid=i.ils[0].loc_valid=1; i.ils[0].course_deg=NAN;
    cmd(&s,&i,A380X_FCU_LOC,0); run(&s,&i,1);
    assert(s.loc_armed && s.guidance_valid && s.lateral_mode==A380X_C1_HDG);
    i.tas_knots=NAN; run(&s,&i,.1); assert(!s.guidance_valid);
    a380x_guidance_destroy(&s);
}
static void test_gs_sign(void) {
    double pitch[2];
    for(int n=0;n<2;n++) {
        A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
        a380x_guidance_init(&s,&i); engage(&s,&i);
        i.vertical_speed_fpm=-1500; i.pitch_deg=0;
        i.ils[0].valid=i.ils[0].loc_valid=i.ils[0].gs_valid=1;
        i.ils[0].gs_deviation_dots=n?-.5:.5;
        s.vertical_mode=A380X_B1_GS; s.lateral_mode=A380X_C1_LOC;
        run(&s,&i,15); pitch[n]=s.pitch_command_deg;
        a380x_guidance_destroy(&s);
    }
    printf("FBW GS needle up/down pitch: %.3f / %.3f\n",pitch[0],pitch[1]);
    assert(pitch[0]>pitch[1]);
}
/* Managed (FMGC-coupled) modes: NAV arm/capture/loss, takeoff SRS, CLB/ALT CST, DES submodes, A/THR. */
static void managed_route(A380XGuidanceInputs *i, int capture) {
    i->managed_route_valid=1; i->managed_nav_capture=capture; i->managed_vertical_valid=1;
    i->managed_speed_valid=1; i->managed_speed_knots=250;
    i->fms_phase_valid=1;
}
static void test_managed_nav(void) {
    A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
    a380x_guidance_init(&s,&i); engage(&s,&i);
    managed_route(&i,0); i.fms_phase=3;
    cmd(&s,&i,A380X_FCU_HDG_PUSH,0);
    assert(s.nav_armed && s.lateral_mode==A380X_C1_HDG && !s.managed_unavailable);
    run(&s,&i,1); assert(s.nav_armed && s.lateral_mode==A380X_C1_HDG);
    i.managed_nav_capture=1; run(&s,&i,.1);
    assert(!s.nav_armed && s.lateral_mode==A380X_C1_NAV && s.heading_managed);
    /* FBW HPATH convention: path to the right (xtk>0) and desired track to the right (tae>0) bank right */
    i.route_xtk_nm=.5; i.route_track_error_deg=5; run(&s,&i,3);
    assert(s.roll_command_deg>5);
    i.route_xtk_nm=-.5; i.route_track_error_deg=-5; run(&s,&i,6);
    assert(s.roll_command_deg<-5);
    /* bank feed-forward: the AP roll command is phi + K(phi_c - phi), so it settles at phi_c when the
     * aircraft holds the commanded bank */
    i.route_xtk_nm=0; i.route_track_error_deg=0; i.route_bank_deg=15; i.roll_deg=15; run(&s,&i,12);
    assert(fabs(s.roll_command_deg-15)<1 && fabs(s.fd_roll_error_deg)<1);   /* 5 deg/s command rate limit */
    i.route_bank_deg=0; i.roll_deg=0;
    /* managed climb, then NAV lost (discontinuity): CLB -> OP CLB, HDG on the current heading */
    s.selected_altitude_ft=20000; i.managed_altitude_ft=20000; i.fms_phase=2;
    cmd(&s,&i,A380X_FCU_ALT_PUSH,0); assert(s.vertical_mode==A380X_B1_CLB && s.altitude_managed);
    run(&s,&i,1);
    i.managed_route_valid=0; i.managed_vertical_valid=0; run(&s,&i,.1);
    assert(s.lateral_mode==A380X_C1_HDG && s.vertical_mode==A380X_B1_OP_CLB && !s.altitude_managed && s.mode_reversion);
    a380x_guidance_destroy(&s);
    /* HDG pull disarms NAV; ALT push without NAV engaged opens the climb and arms CLB with NAV armed */
    i=fixture(); a380x_guidance_init(&s,&i); engage(&s,&i); managed_route(&i,0); i.fms_phase=2;
    cmd(&s,&i,A380X_FCU_HDG_PUSH,0); assert(s.nav_armed);
    s.selected_altitude_ft=15000; i.managed_altitude_ft=15000;
    cmd(&s,&i,A380X_FCU_ALT_PUSH,0); assert(s.vertical_mode==A380X_B1_OP_CLB && s.clb_armed);
    i.managed_nav_capture=1; run(&s,&i,.2);
    assert(s.lateral_mode==A380X_C1_NAV && s.vertical_mode==A380X_B1_CLB && !s.clb_armed);
    cmd(&s,&i,A380X_FCU_HDG_PULL,0);
    assert(s.lateral_mode==A380X_C1_HDG && s.vertical_mode==A380X_B1_OP_CLB && !s.nav_armed);
    a380x_guidance_destroy(&s);
}
static void test_managed_takeoff(void) {
    A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
    i.on_ground=1; i.altitude_ft=i.pressure_altitude_ft=100; i.radio_height_ft=0; i.ias_knots=0; i.tas_knots=0;
    a380x_guidance_init(&s,&i);
    managed_route(&i,0); i.fms_phase=0; i.fms_v2_knots=160;
    s.selected_altitude_ft=6000;
    run(&s,&i,.2);
    assert(s.nav_armed && s.clb_armed && s.vertical_mode==A380X_B1_NONE);   /* armed on the ground */
    for(int n=0;n<4;n++) i.throttle_lever_ratio[n]=i.toga_detent_ratio;
    run(&s,&i,.2);
    assert(s.vertical_mode==A380X_B1_SRS && s.speed_managed && s.speed_target_knots==160);
    assert(s.lateral_mode==A380X_C1_HDG && s.nav_armed);
    /* liftoff: NAV engages at 30 ft RA, SRS guides V2+10 */
    i.on_ground=0; i.ias_knots=i.tas_knots=170; i.radio_height_ft=20; i.fms_phase=1; run(&s,&i,.2);
    assert(s.nav_armed && s.lateral_mode==A380X_C1_HDG);
    i.radio_height_ft=40; run(&s,&i,.2);
    assert(s.lateral_mode==A380X_C1_NAV && s.vertical_mode==A380X_B1_SRS && s.speed_target_knots==170);
    assert(s.guidance_valid);
    /* acceleration altitude (FMGC CLIMB phase): SRS -> CLB with NAV */
    i.altitude_ft=i.pressure_altitude_ft=1600; i.radio_height_ft=1500; i.managed_altitude_ft=6000; i.fms_phase=2; run(&s,&i,.2);
    assert(s.vertical_mode==A380X_B1_CLB && s.altitude_managed && !s.clb_armed);
    a380x_guidance_destroy(&s);
}
static void test_managed_constraint_and_descent(void) {
    A380XGuidanceInputs i=fixture(); A380XGuidanceState s;
    a380x_guidance_init(&s,&i); engage(&s,&i);
    managed_route(&i,1); i.fms_phase=2;
    cmd(&s,&i,A380X_FCU_HDG_PUSH,0); assert(s.lateral_mode==A380X_C1_NAV);
    s.selected_altitude_ft=20000; i.managed_altitude_ft=11000; i.managed_altitude_is_constraint=1;
    cmd(&s,&i,A380X_FCU_ALT_PUSH,0); run(&s,&i,.1);
    assert(s.vertical_mode==A380X_B1_CLB && s.alt_constraint && s.altitude_target_ft==11000 && s.altitude_armed);
    assert(a380x_guidance_fma_vertical(&s)==A380X_B1_CLB);
    /* capture of the constraint: ALT CST* then ALT CST with CLB armed */
    i.altitude_ft=i.pressure_altitude_ft=10900; i.vertical_speed_fpm=1500; run(&s,&i,.1);
    assert(s.vertical_mode==A380X_B1_ALT_STAR && a380x_guidance_fma_vertical(&s)==A380X_B1_ALT_CST_STAR);
    i.altitude_ft=i.pressure_altitude_ft=11000; i.vertical_speed_fpm=0; run(&s,&i,1.5);
    assert(s.vertical_mode==A380X_B1_ALT && a380x_guidance_fma_vertical(&s)==A380X_B1_ALT_CST);
    assert(s.altitude_managed && s.clb_armed && s.held_altitude_ft==11000);
    /* constraint sequenced: CLB resumes towards the FCU altitude */
    i.managed_altitude_ft=20000; i.managed_altitude_is_constraint=0; run(&s,&i,.1);
    assert(s.vertical_mode==A380X_B1_CLB && !s.clb_armed && s.altitude_target_ft==20000 && !s.alt_constraint);
    /* A/THR THR CLB in CLB */
    cmd(&s,&i,A380X_FCU_ATHR,0); run(&s,&i,.5);
    assert(s.athr_active && s.thrust_mode==A380X_A1A2_THR_CLB);
    /* level at the cruise altitude: ALT CRZ */
    i.managed_cruise_altitude_ft=20000;
    i.altitude_ft=i.pressure_altitude_ft=19950; i.vertical_speed_fpm=500; run(&s,&i,.1);
    i.altitude_ft=i.pressure_altitude_ft=20000; i.vertical_speed_fpm=0; run(&s,&i,1.5);
    assert(s.vertical_mode==A380X_B1_ALT && a380x_guidance_fma_vertical(&s)==A380X_B1_ALT_CRZ);
    assert(s.thrust_mode==A380X_A1A2_SPEED);
    i.managed_speed_is_mach=1; s.speed_managed=1; run(&s,&i,.1); assert(s.thrust_mode==A380X_A1A2_MACH);
    i.managed_speed_is_mach=0;
    /* DES: early descent before T/D (V/S -1000, SPEED), path capture (VPATH law, THR IDLE), above path */
    i.fms_phase=4; s.selected_altitude_ft=6000; i.managed_altitude_ft=6000;
    i.managed_path_valid=1; i.managed_path_altitude_ft=20300; i.managed_path_vs_fpm=-2200; i.managed_vdev_ft=-300;
    i.managed_before_tod=1; i.managed_path_idle=1;
    cmd(&s,&i,A380X_FCU_ALT_PUSH,0); run(&s,&i,.1);
    assert(s.vertical_mode==A380X_B1_DES && s.des_submode==2 && s.vertical_law_override==4 && s.vs_target_fpm==-1000);
    assert(s.thrust_mode==A380X_A1A2_SPEED && s.altitude_managed);
    i.managed_before_tod=0; i.managed_vdev_ft=-50; i.managed_path_altitude_ft=20050; run(&s,&i,.1);
    assert(s.des_submode==0 && s.vertical_law_override==9 && s.vpath_valid && s.vpath_altitude_ft==20050);
    assert(s.thrust_mode==A380X_A1A2_THR_IDLE);
    i.managed_path_idle=0; run(&s,&i,.1); assert(s.thrust_mode==A380X_A1A2_SPEED);   /* geometric segment */
    i.managed_path_idle=1; i.managed_vdev_ft=800; run(&s,&i,.1);
    assert(s.des_submode==1 && s.vertical_law_override==3 && s.thrust_mode==A380X_A1A2_THR_IDLE);
    assert(s.speed_target_knots==260);                                            /* upper end of the range */
    i.managed_vdev_ft=50; run(&s,&i,.1); assert(s.des_submode==0);
    /* DES without NAV: V/S */
    cmd(&s,&i,A380X_FCU_HDG_PULL,0); assert(s.vertical_mode==A380X_B1_VS && !s.altitude_managed);
    a380x_guidance_destroy(&s);
}
int main(void) {
    test_controls_and_gating(); test_actual_fbw_demands(); test_capture_and_failures(); test_ils(); test_gs_sign(); test_vertical_and_fpa_signs(); test_ils_magnetic_reference();
    test_closed_loop_plant(); test_frame_timing_and_invalid_data(); test_ground_initialization_standby(); test_manual_ap_timing_and_fd_powerup();
    test_manual_athr_detents_and_disconnects(); test_athr_validation_envelope_and_alpha_floor();
    test_athr_inhibit_and_fadec_ceiling();
    test_open_altitude_capture_paths();
    test_managed_nav(); test_managed_takeoff(); test_managed_constraint_and_descent();
    puts("A380 guidance: controls, imported FBW laws, captures, sensor guards, manual AP/FD and A/THR detents/envelope/disconnects, managed NAV/SRS/CLB/ALT CST/DES passed");
    return 0;
}
