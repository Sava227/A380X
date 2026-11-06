/* SPDX-License-Identifier: GPL-3.0-or-later
 * X-Plane command/state adapter for the imported FBW A380 guidance laws.
 * Source: pinned A380FcuComputer.cpp / A380PrimComputerFg.cpp; user-facing
 * controls: https://docs.flybywiresim.com/pilots-corner/a380x/a380x-briefing/flight-deck/glareshield/afs/
 * This adapter does not implement the three PRIMs, FMS, FADEC detent network,
 * SRS takeoff guidance or the full flight-envelope protections. LAND/FLARE/ROLL OUT,
 * RETARD and SRS GA / GA TRK: a380x_autoland.inc (FBW PRIM FG mode logic).
 * A/THR and final pitch/roll/yaw actuator servos are explicitly port-specific.
 */
#include "a380x_guidance.h"
#include "a380x_fbw_laws.h"
#include <math.h>
#include <string.h>

#define RAD (0.017453292519943295)
static double clamp(double x,double a,double b) { return fmax(a,fmin(x,b)); }
static double norm(double x) { x=fmod(x,360); return x<0?x+360:x; }
static double diff(double x,double y) { return fmod(norm(x-y)+180,360)-180; }
static double decay(double x,double dt) { return fmax(0,x-dt); }
static int lever_inputs_valid(const A380XGuidanceInputs *i) {
    int n;
    if(!i->throttle_levers_valid || !isfinite(i->idle_detent_ratio) || !isfinite(i->cl_detent_ratio) ||
       !isfinite(i->mct_detent_ratio) || !isfinite(i->toga_detent_ratio) || !isfinite(i->detent_tolerance_ratio) ||
       i->idle_detent_ratio<0 || i->idle_detent_ratio>=i->cl_detent_ratio ||
       i->cl_detent_ratio>=i->mct_detent_ratio || i->mct_detent_ratio>=i->toga_detent_ratio ||
       i->toga_detent_ratio>1 || i->detent_tolerance_ratio<0 ||
       2*i->detent_tolerance_ratio>=fmin(i->cl_detent_ratio-i->idle_detent_ratio,
         fmin(i->mct_detent_ratio-i->cl_detent_ratio,i->toga_detent_ratio-i->mct_detent_ratio))) return 0;
    for(n=0;n<4;n++) if(!isfinite(i->throttle_lever_ratio[n]) ||
        i->throttle_lever_ratio[n]<0 || i->throttle_lever_ratio[n]>1) return 0;
    return 1;
}
static int all_levers_at(const A380XGuidanceInputs *i,double detent) {
    int n;
    for(n=0;n<4;n++) if(fabs(i->throttle_lever_ratio[n]-detent)>i->detent_tolerance_ratio) return 0;
    return 1;
}
static int takeoff_detent(const A380XGuidanceInputs *i) {
    return lever_inputs_valid(i) && (all_levers_at(i,i->toga_detent_ratio) ||
        (i->flex_valid && all_levers_at(i,i->mct_detent_ratio)));
}
static double max_lever(const A380XGuidanceInputs *i) {
    return fmax(fmax(i->throttle_lever_ratio[0],i->throttle_lever_ratio[1]),
        fmax(i->throttle_lever_ratio[2],i->throttle_lever_ratio[3]));
}
static A380XA1A2Message manual_thrust_mode(const A380XGuidanceInputs *i) {
    double lever;
    if(!i->powered || !lever_inputs_valid(i)) return A380X_A1A2_NONE;
    lever=max_lever(i);
    if(lever<=i->cl_detent_ratio+i->detent_tolerance_ratio) return A380X_A1A2_NONE;
    if(fabs(lever-i->toga_detent_ratio)<=i->detent_tolerance_ratio) return A380X_A1A2_MAN_TOGA;
    if(fabs(lever-i->mct_detent_ratio)<=i->detent_tolerance_ratio)
        return i->flex_valid?A380X_A1A2_MAN_FLEX:A380X_A1A2_MAN_MCT;
    return A380X_A1A2_MAN_THR;
}
static void disconnect_athr(A380XGuidanceState *s,const A380XGuidanceInputs *i) {
    s->athr_armed=s->athr_active=s->alpha_floor_latched=0;
    s->alpha_floor_inhibited=i->alpha_floor_valid && i->alpha_floor_request;
    /* An unchanged takeoff detent cannot undo a crew disconnect next frame. */
    s->takeoff_detent_active=takeoff_detent(i);
    s->athr_integral=0;
    s->thrust_mode=manual_thrust_mode(i);
    s->throttle_command=isfinite(i->throttle_ratio)?clamp(i->throttle_ratio,0,1):0;
}
static int airborne_valid(const A380XGuidanceInputs *i) {
    return i->powered && !i->paused && !i->on_ground && i->airdata_valid && i->attitude_valid &&
        isfinite(i->pitch_deg) && isfinite(i->roll_deg) && isfinite(i->heading_deg) &&
        isfinite(i->track_deg) && isfinite(i->altitude_ft) && isfinite(i->vertical_speed_fpm) &&
        isfinite(i->ias_knots) && isfinite(i->tas_knots) && isfinite(i->ground_speed_knots) &&
        isfinite(i->mach) && isfinite(i->pressure_altitude_ft) && isfinite(i->radio_height_ft) &&
        isfinite(i->pitch_rate_deg_s) && isfinite(i->roll_rate_deg_s) && isfinite(i->yaw_rate_deg_s) &&
        isfinite(i->alpha_deg) && isfinite(i->beta_deg) && isfinite(i->true_heading_deg) && isfinite(i->true_track_deg) &&
        isfinite(i->accel_x_m_s2) && isfinite(i->accel_y_m_s2) && isfinite(i->accel_z_m_s2) &&
        i->ias_knots>40 && i->tas_knots>40;
}
static int can_engage(const A380XGuidanceState *s,const A380XGuidanceInputs *i) {
    /* The manual permits engagement at five seconds after liftoff. Allow
     * sub-nanosecond integration roundoff, never an earlier simulation frame. */
    return airborne_valid(i) && s->airborne_seconds>=5-1e-9 && i->pitch_deg>=-11 && i->pitch_deg<=22.5 &&
        fabs(i->roll_deg)<40 && (i->vls_knots<=0 || i->ias_knots>=i->vls_knots) &&
        (i->vmax_knots<=0 || i->ias_knots<=i->vmax_knots);
}
static double speed_ratio(const A380XGuidanceInputs *i) {
    return i->mach>0.05 && isfinite(i->mach)?clamp(i->ias_knots/i->mach,100,900):661.47;
}
static double current_heading(const A380XGuidanceState *s,const A380XGuidanceInputs *i) {
    return s->trk_fpa_mode?(s->true_heading?i->true_track_deg:i->track_deg):
        (s->true_heading?i->true_heading_deg:i->heading_deg);
}
static int vs_mode(const A380XGuidanceState *s) { return s->vertical_mode==A380X_B1_VS || s->vertical_mode==A380X_B1_FPA; }
static int ils_loc_valid(const A380XIlsInputs *i) {
    return i->valid && i->loc_valid && isfinite(i->course_deg) && isfinite(i->loc_deviation_dots);
}
static int ils_gs_valid(const A380XIlsInputs *i) {
    return i->valid && i->gs_valid && isfinite(i->gs_deviation_dots) &&
        isfinite(i->gs_angle_deg) && i->gs_angle_deg>0 && i->gs_angle_deg<10;
}
static int loc_mode(const A380XGuidanceState *s) { return s->lateral_mode==A380X_C1_LOC || s->lateral_mode==A380X_C1_LOC_STAR; }
static int gs_mode(const A380XGuidanceState *s) { return s->vertical_mode==A380X_B1_GS || s->vertical_mode==A380X_B1_GS_STAR; }
static void reject_managed(A380XGuidanceState *s) { s->unavailable_seconds=5; s->managed_unavailable=1; }
static void revert_vertical(A380XGuidanceState *s,const A380XGuidanceInputs *i) {
    s->altitude_managed=0; s->selected_vs_fpm=round(i->vertical_speed_fpm/100)*100;
    s->selected_fpa_deg=atan2(i->vertical_speed_fpm,fmax(i->ground_speed_knots,30)*101.2686)/RAD;
    s->vertical_mode=s->trk_fpa_mode?A380X_B1_FPA:A380X_B1_VS;
    s->capture_seconds=0; s->reversion_seconds=5; s->mode_reversion=1;
}
static void disconnect(A380XGuidanceState *s,int warning) {
    if ((s->ap1 || s->ap2) && warning) s->disconnect_seconds=5;
    s->ap1=0; s->ap2=0; s->pitch_servo=0; s->roll_servo=0; s->pitch_integral=0;
}
static void cancel_approach(A380XGuidanceState *s,const A380XGuidanceInputs *i) {
    s->loc_armed=s->gs_armed=s->approach_selected=0;
    if (loc_mode(s)) {
        s->selected_heading_deg=norm(current_heading(s,i));
        s->lateral_mode=s->trk_fpa_mode?A380X_C1_TRACK:A380X_C1_HDG;
    }
    if (gs_mode(s)) revert_vertical(s,i);
    if (s->ap1 && s->ap2) s->ap2=0;
}
#include "a380x_autoland.inc"

void a380x_guidance_init(A380XGuidanceState *s,const A380XGuidanceInputs *i) {
    memset(s,0,sizeof(*s));
    s->selected_speed_knots=250; s->selected_mach=.78; s->selected_altitude_ft=10000;
    s->selected_heading_deg=i?norm(i->heading_deg):0;
    s->held_altitude_ft=i?i->altitude_ft:0;
    s->selected_vs_fpm=i?round(i->vertical_speed_fpm/100)*100:0;
    s->fd1=s->fd2=1; s->vs_dashes=1;
    s->previous_powered=i && i->powered;
    s->all_levers_idle=i && lever_inputs_valid(i) && max_lever(i)<=i->idle_detent_ratio;
    s->lateral_mode=A380X_C1_HDG;
    /* Never latch runway altitude as a post-liftoff FD command. SRS is not
     * imported, so ground initialization leaves FD in explicit standby. */
    s->vertical_guidance_selected=i && airborne_valid(i);
    s->vertical_mode=s->vertical_guidance_selected?A380X_B1_ALT:A380X_B1_NONE;
    s->throttle_command=i?clamp(i->throttle_ratio,0,1):0;
    s->fbw_laws=a380x_fbw_laws_create(); s->initialized=s->fbw_laws!=0;
}
void a380x_guidance_destroy(A380XGuidanceState *s) {
    a380x_fbw_laws_destroy(s->fbw_laws); s->fbw_laws=0; s->initialized=0;
}
void a380x_guidance_command(A380XGuidanceState *s,const A380XGuidanceInputs *i,A380XFcuCommand c,double a) {
    A380XB1Message previous_vertical;
    if (!s || !i || !s->initialized || !isfinite(a)) return;
    previous_vertical=s->vertical_mode;
    if (c==A380X_FCU_AP_DISCONNECT) {
        if (s->ap1 || s->ap2) disconnect(s,1);
        else { s->disconnect_seconds=0; s->disconnect_warning=0; }
        return;
    }
    if (c==A380X_FCU_ATHR_DISCONNECT) { disconnect_athr(s,i); return; }
    if (!i->powered) return;
    if (landing_command_filter(s,c)) return;
    switch(c) {
    case A380X_FCU_SPD_ROTATE:
        if (s->speed_dashes) {
            s->selected_speed_knots=round(s->speed_target_knots>0?s->speed_target_knots:i->ias_knots);
            s->selected_mach=round(s->selected_speed_knots/speed_ratio(i)*100)/100;
        }
        if(s->mach_mode) s->selected_mach=clamp(s->selected_mach+a*.01,.01,.99);
        else s->selected_speed_knots=clamp(s->selected_speed_knots+a,100,399);
        s->speed_preselect_seconds=45; s->speed_dashes=0; break;
    case A380X_FCU_SPD_PUSH:
        if(i->managed_speed_valid && isfinite(i->managed_speed_knots) && i->managed_speed_knots>=100) {
            s->speed_managed=1; s->speed_preselect_seconds=0; s->speed_dashes=1;
        } else reject_managed(s);
        break;
    case A380X_FCU_SPD_PULL: s->speed_managed=0; s->speed_dashes=0; break;
    case A380X_FCU_HDG_ROTATE:
        if(s->heading_dashes) s->selected_heading_deg=round(current_heading(s,i));
        s->selected_heading_deg=norm(s->selected_heading_deg+a); s->heading_preselect_seconds=45; s->heading_dashes=0; break;
    case A380X_FCU_HDG_PUSH:
        if(i->managed_route_valid) {
            cancel_approach(s,i); s->heading_managed=1; s->lateral_mode=A380X_C1_NAV;
            s->heading_preselect_seconds=0; s->heading_dashes=1;
        } else reject_managed(s);
        break;
    case A380X_FCU_HDG_PULL:
        cancel_approach(s,i); s->heading_managed=0; s->heading_dashes=0;
        s->lateral_mode=s->trk_fpa_mode?A380X_C1_TRACK:A380X_C1_HDG; break;
    case A380X_FCU_ALT_ROTATE:
        s->selected_altitude_ft=clamp(s->selected_altitude_ft+a*(s->altitude_increment_1000?1000:100),100,49000);
        if(s->vertical_mode==A380X_B1_ALT_STAR) {
            revert_vertical(s,i);
        }
        break;
    case A380X_FCU_ALT_PUSH:
        if(i->managed_route_valid && i->managed_vertical_valid) {
            s->altitude_managed=1; s->vertical_guidance_selected=1;
            s->vertical_mode=s->selected_altitude_ft>=i->altitude_ft?A380X_B1_CLB:A380X_B1_DES;
            s->gs_armed=0;
        } else reject_managed(s);
        break;
    case A380X_FCU_ALT_PULL:
        s->vertical_guidance_selected=1;
        s->altitude_managed=0; s->gs_armed=0; s->capture_seconds=0;
        if(fabs(s->selected_altitude_ft-i->altitude_ft)<40) {
            s->held_altitude_ft=s->selected_altitude_ft; s->vertical_mode=A380X_B1_ALT;
        } else s->vertical_mode=s->selected_altitude_ft>i->altitude_ft?A380X_B1_OP_CLB:A380X_B1_OP_DES;
        break;
    case A380X_FCU_VS_ROTATE:
        if(s->vs_dashes) {
            s->selected_vs_fpm=round(i->vertical_speed_fpm/100)*100;
            s->selected_fpa_deg=round(atan2(i->vertical_speed_fpm,fmax(i->ground_speed_knots,30)*101.2686)/RAD*10)/10;
        }
        if(s->trk_fpa_mode) s->selected_fpa_deg=clamp(s->selected_fpa_deg+a*.1,-9.9,9.9);
        else s->selected_vs_fpm=clamp(s->selected_vs_fpm+a*50,-6000,6000);
        s->vs_preselect_seconds=45; s->vs_dashes=0; break;
    case A380X_FCU_VS_PUSH: break; /* Retained port behavior; full manufacturer V/S pages unavailable. */
    case A380X_FCU_VS_PULL:
        s->vertical_guidance_selected=1;
        if(s->vs_dashes) {
            s->selected_vs_fpm=round(i->vertical_speed_fpm/100)*100;
            s->selected_fpa_deg=round(atan2(i->vertical_speed_fpm,fmax(i->ground_speed_knots,30)*101.2686)/RAD*10)/10;
        }
        s->altitude_managed=0; s->gs_armed=0; s->vs_dashes=0;
        s->vertical_mode=s->trk_fpa_mode?A380X_B1_FPA:A380X_B1_VS; break;
    case A380X_FCU_ALT_HOLD:
        s->vertical_guidance_selected=1;
        s->held_altitude_ft=i->altitude_ft; s->vertical_mode=A380X_B1_ALT;
        s->altitude_managed=0; s->gs_armed=0; s->selected_vs_fpm=0; s->selected_fpa_deg=0; break;
    case A380X_FCU_AP1: case A380X_FCU_AP2: {
        int *ap=c==A380X_FCU_AP1?&s->ap1:&s->ap2;
        if(*ap) { *ap=0; if(!s->ap1 && !s->ap2) s->disconnect_seconds=5; }
        else if(can_engage(s,i)) {
            if(!s->approach_selected && !s->land.land_active && !s->land.go_around_active) { s->ap1=0; s->ap2=0; }
            if(!s->vertical_guidance_selected) {
                s->vertical_guidance_selected=1; s->held_altitude_ft=i->altitude_ft;
                s->vertical_mode=A380X_B1_ALT;
            }
            *ap=1; s->fd1=s->fd2=1; s->pitch_integral=0;
        } else { s->ap_engagement_inhibited=1; s->reversion_seconds=5; }
        break;
    }
    case A380X_FCU_FD1: s->fd1=!s->fd1; break;
    case A380X_FCU_FD2: s->fd2=!s->fd2; break;
    case A380X_FCU_ATHR:
        if(s->athr_armed) { disconnect_athr(s,i); break; }
        if(i->athr_inhibited) break;   /* FBW FADEC ATHR_disabled: the pushbutton cannot engage */
        s->athr_armed=1; s->athr_active=0;
        s->athr_integral=clamp(i->throttle_ratio,0,1)-.5;
        s->throttle_command=clamp(i->throttle_ratio,0,1); break;
    case A380X_FCU_LOC:
        if(s->loc_armed || loc_mode(s)) cancel_approach(s,i);
        else { s->loc_armed=1; s->gs_armed=s->approach_selected=0; s->heading_managed=0; }
        break;
    case A380X_FCU_APPR:
        if(s->approach_selected) cancel_approach(s,i);
        else { s->approach_selected=1; s->loc_armed=!loc_mode(s); s->gs_armed=1; s->heading_managed=0; }
        break;
    case A380X_FCU_SPD_MACH:
        if(!s->mach_mode) s->selected_mach=clamp(round(s->selected_speed_knots/speed_ratio(i)*100)/100,.01,.99);
        else s->selected_speed_knots=clamp(round(s->selected_mach*speed_ratio(i)),100,399);
        s->mach_mode=!s->mach_mode; break;
    case A380X_FCU_TRK_FPA:
        s->selected_heading_deg=norm(s->selected_heading_deg+(s->trk_fpa_mode?diff(i->heading_deg,i->track_deg):diff(i->track_deg,i->heading_deg)));
        if(s->trk_fpa_mode) s->selected_vs_fpm=round(tan(s->selected_fpa_deg*RAD)*fmax(i->ground_speed_knots,30)*101.2686/100)*100;
        else s->selected_fpa_deg=round(atan2(s->selected_vs_fpm,fmax(i->ground_speed_knots,30)*101.2686)/RAD*10)/10;
        s->trk_fpa_mode=!s->trk_fpa_mode;
        if(s->lateral_mode==A380X_C1_HDG || s->lateral_mode==A380X_C1_TRACK) s->lateral_mode=s->trk_fpa_mode?A380X_C1_TRACK:A380X_C1_HDG;
        if(vs_mode(s)) s->vertical_mode=s->trk_fpa_mode?A380X_B1_FPA:A380X_B1_VS;
        break;
    case A380X_FCU_ALT_INCREMENT: s->altitude_increment_1000=!s->altitude_increment_1000; break;
    case A380X_FCU_LS1: s->ls1=!s->ls1; break;
    case A380X_FCU_LS2: s->ls2=!s->ls2; break;
    case A380X_FCU_METRIC_ALT: s->metric_alt=!s->metric_alt; break;
    case A380X_FCU_TRUE_MAG:
        s->selected_heading_deg=norm(s->selected_heading_deg+(s->true_heading?-1:1)*diff(i->true_heading_deg,i->heading_deg));
        s->true_heading=!s->true_heading; break;
    default: break;
    }
    if (s->vertical_mode!=previous_vertical) s->pitch_integral=0;
}

void a380x_guidance_update(A380XGuidanceState *s,const A380XGuidanceInputs *i,double dt) {
    double alt_error, target_vs, capture_distance;
    int valid;
    const A380XIlsInputs *ils;
    if(!s || !i || !s->initialized) return;
    if(!isfinite(dt) || dt>1) {
        disconnect(s,1); s->athr_active=0; s->guidance_valid=0; return;
    }
    /* Freeze display/mode state on pause; simulator adapter releases controls. */
    if(dt<=0 || i->paused) { s->pitch_servo=s->roll_servo=s->land.yaw_servo=0; return; }
    /* Substep slower frames so timers use elapsed simulation time and the
     * imported filters retain their maximum 100ms integration step. */
    if(dt>.1) {
        while(dt>1e-9) { double step=fmin(dt,.1); a380x_guidance_update(s,i,step); dt-=step; }
        return;
    }
    s->simulation_seconds+=dt;
    /* The single FCU FD control affects both channels; electrical power-up
     * restores the automatic ON selection after a previous crew selection OFF. */
    if(i->powered && !s->previous_powered) s->fd1=s->fd2=1;
    s->previous_powered=i->powered;
    s->airborne_seconds=i->on_ground?0:s->airborne_seconds+dt;
    if(s->source_side!=(s->ap2 && !s->ap1?1:0)) {
        s->have_previous_ils=0; a380x_fbw_laws_reset(s->fbw_laws);
    }
    s->source_side=s->ap2 && !s->ap1?1:0; ils=&i->ils[s->source_side];
    valid=airborne_valid(i) || landing_ground_valid(s,i);
    s->ap_engagement_inhibited=!can_engage(s,i);
    s->speed_preselect_seconds=decay(s->speed_preselect_seconds,dt);
    s->heading_preselect_seconds=decay(s->heading_preselect_seconds,dt);
    s->vs_preselect_seconds=decay(s->vs_preselect_seconds,dt);
    s->unavailable_seconds=decay(s->unavailable_seconds,dt); s->managed_unavailable=s->unavailable_seconds>0;
    s->reversion_seconds=decay(s->reversion_seconds,dt); s->mode_reversion=s->reversion_seconds>0;
    s->disconnect_seconds=decay(s->disconnect_seconds,dt); s->disconnect_warning=s->disconnect_seconds>0;
    s->speed_dashes=s->speed_managed && s->speed_preselect_seconds<=0;
    s->heading_dashes=(s->heading_managed || loc_mode(s)) && s->heading_preselect_seconds<=0;
    s->vs_dashes=!vs_mode(s) && s->vs_preselect_seconds<=0;
    if(!valid || i->pitch_deg < -13 || i->pitch_deg>25 || fabs(i->roll_deg)>45 ||
        /* Port speed guard. In approach modes it stands in for the alpha-prot + 1 deg disconnect
         * (roughly VLS - 10 kt in landing configuration) and is not applied in LAND. */
        (i->vls_knots>0 && !s->land.land_active &&
         i->ias_knots<i->vls_knots-(s->approach_selected || loc_mode(s) || gs_mode(s) ||
                                    s->land.go_around_active ? 10 : 0)) ||
        (i->vmax_knots>0 && i->ias_knots>i->vmax_knots+15)) {
        disconnect(s,1);
    }
    if(!i->powered) { s->athr_armed=0; s->loc_armed=s->gs_armed=s->approach_selected=0; }
    if(s->speed_managed && !i->managed_speed_valid) { s->speed_managed=0; reject_managed(s); }
    if(s->heading_managed && !i->managed_route_valid) {
        s->heading_managed=0; s->selected_heading_deg=norm(current_heading(s,i));
        s->lateral_mode=s->trk_fpa_mode?A380X_C1_TRACK:A380X_C1_HDG; reject_managed(s); s->reversion_seconds=5;
    }
    if(s->altitude_managed && (!i->managed_vertical_valid || !i->managed_route_valid)) {
        revert_vertical(s,i); reject_managed(s);
    }
    s->speed_target_knots=s->speed_managed?i->managed_speed_knots:
        (s->mach_mode?s->selected_mach*speed_ratio(i):s->selected_speed_knots);
    s->speed_target_knots=clamp(s->speed_target_knots,100,399);
    /* The window remains the crew's selection; effective guidance observes
     * the FE limits shown on the PFD. Invalid/missing limits are not invented. */
    {
        double lower=isfinite(i->vls_knots) && i->vls_knots>0?fmax(100,i->vls_knots):100;
        double upper=isfinite(i->vmax_knots) && i->vmax_knots>0?fmin(399,i->vmax_knots):399;
        if(lower<=upper) s->speed_target_knots=clamp(s->speed_target_knots,lower,upper);
    }
    s->altitude_target_ft=s->vertical_mode==A380X_B1_ALT?s->held_altitude_ft:s->selected_altitude_ft;
    if(s->altitude_managed) {
        s->altitude_target_ft=s->vertical_mode==A380X_B1_CLB?fmin(i->managed_altitude_ft,s->selected_altitude_ft):
            fmax(i->managed_altitude_ft,s->selected_altitude_ft);
    }
    if(valid && loc_mode(s) && !ils_loc_valid(ils) && !s->land.land_active) {
        cancel_approach(s,i); s->reversion_seconds=5;
    }
    if(valid && gs_mode(s) && !ils_gs_valid(ils) && !s->land.land_active) {
        s->gs_armed=0; s->approach_selected=0; revert_vertical(s,i);
        if(s->ap1 && s->ap2) s->ap2=0;
    }
    if(valid && s->loc_armed && ils_loc_valid(ils) &&
       fabs(ils->loc_deviation_dots)<1 && fabs(diff(ils->course_deg,i->heading_deg))<110 &&
       (!s->have_previous_ils || fabs(ils->loc_deviation_dots)<=fabs(s->previous_loc_dots)+.001)) {
        s->loc_armed=0; s->lateral_mode=A380X_C1_LOC_STAR; s->loc_capture_seconds=0;
    }
    if(s->lateral_mode==A380X_C1_LOC_STAR) {
        s->loc_capture_seconds=fabs(ils->loc_deviation_dots)<.15?s->loc_capture_seconds+dt:0;
        if(s->loc_capture_seconds>=2) s->lateral_mode=A380X_C1_LOC;
    }
    /* Front-course capture only; intercept the glideslope from below. */
    if(valid && s->gs_armed && loc_mode(s) && ils_gs_valid(ils) &&
       ils->gs_deviation_dots>=-.05 && ils->gs_deviation_dots<.35 &&
       (!s->have_previous_ils || s->previous_gs_dots>=ils->gs_deviation_dots-.001)) {
        s->vertical_guidance_selected=1;
        s->gs_armed=0; s->vertical_mode=A380X_B1_GS_STAR; s->capture_seconds=0; s->altitude_managed=0;
    }
    if(s->vertical_mode==A380X_B1_GS_STAR) {
        s->capture_seconds=fabs(ils->gs_deviation_dots)<.12?s->capture_seconds+dt:0;
        if(s->capture_seconds>=2) s->vertical_mode=A380X_B1_GS;
    }
    landing_modes(s,i,dt,ils,valid);
    /* Without LAND engaged (e.g. no APPR, no RA, LOC/GS not in track) the AP still disconnects below 100 ft. */
    if(gs_mode(s) && !s->land.land_active && i->radio_height_ft<100 && (s->ap1 || s->ap2)) disconnect(s,1);
    s->previous_loc_dots=ils->loc_deviation_dots; s->previous_gs_dots=ils->gs_deviation_dots;
    s->have_previous_ils=ils->valid;
    target_vs=s->trk_fpa_mode?tan(s->selected_fpa_deg*RAD)*fmax(i->ground_speed_knots,30)*101.2686:s->selected_vs_fpm;
    if(s->altitude_managed) target_vs=i->managed_vs_fpm;
    if(s->vertical_mode==A380X_B1_OP_CLB) target_vs=2000;
    if(s->vertical_mode==A380X_B1_OP_DES) target_vs=-1500;
    if(s->vertical_mode==A380X_B1_SRS) target_vs=2000; /* GA: arms ALT* toward a higher FCU altitude */
    alt_error=s->altitude_target_ft-i->altitude_ft;
    s->altitude_armed=!gs_mode(s) && s->vertical_mode!=A380X_B1_ALT && s->vertical_mode!=A380X_B1_ALT_STAR &&
        alt_error*target_vs>0;
    capture_distance=fmax(150,fabs(i->vertical_speed_fpm)/60*20);
    if(valid && s->altitude_armed && fabs(alt_error)<=capture_distance) {
        s->vertical_mode=A380X_B1_ALT_STAR; s->capture_seconds=0; s->altitude_armed=0;
    }
    if(s->vertical_mode==A380X_B1_ALT_STAR) {
        target_vs=clamp(alt_error*3,-1500,1500);
        s->capture_seconds=fabs(alt_error)<30 && fabs(i->vertical_speed_fpm)<150?s->capture_seconds+dt:0;
        if(s->capture_seconds>=1) {
            s->held_altitude_ft=s->altitude_target_ft; s->vertical_mode=A380X_B1_ALT; s->altitude_managed=0;
        }
    }
    if(s->vertical_mode==A380X_B1_ALT) target_vs=0;
    if(gs_mode(s)) target_vs=-tan((ils->gs_angle_deg>0?ils->gs_angle_deg:3)*RAD)*fmax(i->ground_speed_knots,30)*101.2686;
    s->vs_target_fpm=clamp(target_vs,-6000,6000);
    s->guidance_valid=valid && s->vertical_guidance_selected && (s->fd1 || s->fd2 || s->ap1 || s->ap2) &&
        a380x_fbw_laws_step(s->fbw_laws,i,s,dt);
    if(!s->guidance_valid) {
        s->fd_pitch_error_deg=s->fd_roll_error_deg=0; s->pitch_servo=s->roll_servo=0;
        s->pitch_integral=0; a380x_fbw_laws_reset(s->fbw_laws);
        if(s->ap1 || s->ap2) disconnect(s,1);
    } else if(s->ap1 || s->ap2) {
        /* X-Plane actuator adapter, downstream of FBW attitude commands.
         * Anti-windup, bounded gains; these require in-simulator tuning. */
        double error=s->pitch_command_deg-i->pitch_deg;
        s->pitch_integral=clamp(s->pitch_integral+error*dt*.0005,-.35,.35);
        s->pitch_servo=clamp(error*.2-i->pitch_rate_deg_s*.2+s->pitch_integral,-1,1);
        s->roll_servo=clamp((s->roll_command_deg-i->roll_deg)*.035-i->roll_rate_deg_s*.025,-1,1);
    } else { s->pitch_servo=s->roll_servo=0; s->pitch_integral=0; }
    landing_actuators(s,i,dt);
    {
        int levers_valid=lever_inputs_valid(i), at_takeoff=takeoff_detent(i);
        /* A/THR consumes ADR speed data. An independent IR/heading/navigation
         * failure invalidates AP/FD guidance without invalidating this loop. */
        int athr_valid=i->powered && !i->paused && !i->on_ground && i->athr_speed_valid &&
            isfinite(i->ias_knots) && i->ias_knots>40 && isfinite(i->mach);
        int alpha_floor=i->alpha_floor_valid && i->alpha_floor_request;
        double lever=levers_valid?max_lever(i):0;
        int all_idle=levers_valid && lever<=i->idle_detent_ratio;
        int idle_entry=all_idle && !s->all_levers_idle;
        int inner_reverse=levers_valid && i->reverse_selected[1] && i->reverse_selected[2];
        if(!alpha_floor) s->alpha_floor_inhibited=0;
        if(i->athr_inhibited) s->athr_armed=s->athr_active=s->alpha_floor_latched=0;
        if(i->powered && i->on_ground && at_takeoff && !s->takeoff_detent_active && !i->athr_inhibited) {
            s->athr_armed=1;
            s->athr_integral=clamp(i->throttle_ratio,0,1)-.5;
        }
        s->takeoff_detent_active=at_takeoff;
        if(athr_valid && alpha_floor && !s->alpha_floor_inhibited && !i->athr_inhibited) {
            s->alpha_floor_latched=1; s->athr_armed=1;
        }
        /* Moving all handles to IDLE disconnects. An unchanged ground IDLE
         * position still permits manual arming before takeoff. A qualified
         * alpha-floor demand owns its handle-position override; instinctive
         * disconnect always has priority. */
        if(!i->powered || ((!alpha_floor || s->alpha_floor_inhibited) && (idle_entry || inner_reverse))) {
            s->athr_armed=s->alpha_floor_latched=0;
        }
        s->all_levers_idle=all_idle;
        s->athr_active=s->athr_armed && athr_valid && (s->alpha_floor_latched ||
            (levers_valid && !all_idle && !inner_reverse &&
             lever<=i->cl_detent_ratio+i->detent_tolerance_ratio));
        s->thrust_mode=A380X_A1A2_NONE;
        if(s->athr_active) {
            double speed_error=s->speed_target_knots-i->ias_knots;
            if(s->alpha_floor_latched) {
                s->throttle_command=1;
                s->thrust_mode=alpha_floor?A380X_A1A2_A_FLOOR:A380X_A1A2_TOGA_LK;
            } else {
                /* Existing native actuator approximation: CL permits .88 normalized
                 * engine command, reduced proportionally below the CL pilot detent.
                 * This is not a FADEC/N1 thrust-limit computation. */
                double throttle, ceiling=clamp(lever/i->cl_detent_ratio*.88,0,.88);
                int lever_below_cl=ceiling<.879;
                if(i->athr_ceiling_valid && isfinite(i->athr_ceiling)) {
                    /* FADEC N1 of the lever (thrust-lever module); THR LVR below the CL detent. */
                    ceiling=clamp(i->athr_ceiling,0,1);
                    lever_below_cl=lever<i->cl_detent_ratio-i->detent_tolerance_ratio;
                }
                if(s->guidance_valid && (s->vertical_mode==A380X_B1_OP_CLB || s->vertical_mode==A380X_B1_CLB)) {
                    throttle=ceiling; s->thrust_mode=lever_below_cl?A380X_A1A2_THR_LVR:A380X_A1A2_THR_CLB;
                } else if(s->guidance_valid && s->vertical_mode==A380X_B1_OP_DES) {
                    throttle=.04; s->thrust_mode=A380X_A1A2_THR_IDLE;
                } else {
                    double demand=.5+s->athr_integral+speed_error*.008;
                    if((demand<ceiling || speed_error<0) && (demand>0 || speed_error>0))
                        s->athr_integral=clamp(s->athr_integral+speed_error*.0005*dt,-.48,.48);
                    throttle=.5+s->athr_integral+speed_error*.008;
                    s->thrust_mode=s->mach_mode?A380X_A1A2_MACH:A380X_A1A2_SPEED;
                }
                s->throttle_command=fmin(ceiling,s->throttle_command+
                    clamp(clamp(throttle,0,ceiling)-s->throttle_command,-.12*dt,.12*dt));
            }
        } else {
            s->throttle_command=isfinite(i->throttle_ratio)?clamp(i->throttle_ratio,0,1):0;
            s->thrust_mode=manual_thrust_mode(i);
        }
        landing_athr(s,i,dt);
    }
    s->disconnect_warning=s->disconnect_seconds>0;
}
