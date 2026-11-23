/* SPDX-License-Identifier: GPL-3.0-or-later
 * X-Plane raw sensor/actuator boundary for the imported FBW guidance laws.
 * Guidance never derives modes/targets from native AP/FD/FMS state. A separate
 * authority monitor reads native servo status solely to detect contention.
 * Overrides are leased while engaged and released on pause/failure/unload.
 * The yaw (rudder pedal / nosewheel) lease is taken only in LAND ALIGN and ROLL OUT
 * (a380x_autoland.inc) with the same ownership and release rules as pitch/roll.
 */
#include "a380x_elec_power.h"
typedef struct {
    XPLMDataRef paused, replay, magvar, tas, pitch_rate, roll_rate, yaw_rate;
    XPLMDataRef alpha, beta, weight, vmax, mmo, pressure_altitude;
    XPLMDataRef g_axial, g_side, g_normal, gs_angle[2];
    XPLMDataRef override_pitch, override_roll, override_throttle, override_all, override_heading;
    XPLMDataRef hydraulic_pressure[2], rudder_failure[2];   /* LAND capability redundancy terms */
    XPLMDataRef pitch, roll, engine_throttle, pilot_rudder;
    XPLMDataRef axis_assignments, axis_values, axis_minimum, axis_maximum;
    XPLMCommandRef native_disconnect;
    int own_pitch, own_roll, own_throttle, own_yaw, suppress_native_disconnect;
} FcuBoundary;
static FcuBoundary fcu;
/* Thrust-lever / FADEC module link (a380x_thrust.inc, docs/THRUST_LEVERS.md). All zero while the
 * module is absent: the legacy lever-ratio behaviour below is then unchanged.
 *   lease          the module holds override_throttles for the FADEC N1 command (not foreign);
 *   flex_rating    FBW FLEX rating of engine 1 (MAN FLX / take-off detent arming);
 *   athr_inhibited instinctive disconnect held > 15 s: no A/THR, no alpha floor;
 *   ceiling[]      A/THR authority per engine = FADEC engine command of the lever's own N1. */
static struct { int ready, lease, flex_rating, athr_inhibited, ceiling_valid; double ceiling[4]; } fcu_thrust_link;

#include "a380x_pfd_inputs.inc"

typedef struct {
    const char *name;
    A380XFcuCommand command;
    double amount;
    XPLMCommandRef handle;
    float next_repeat;
} FcuCommand;
#define FCUC(name, event, amount) {"a380x/xp12/fcu/" name, event, amount, NULL, 0}
static FcuCommand fcu_commands[] = {
    FCUC("speed_up", A380X_FCU_SPD_ROTATE, 1), FCUC("speed_down", A380X_FCU_SPD_ROTATE, -1),
    FCUC("speed_push", A380X_FCU_SPD_PUSH, 0), FCUC("speed_pull", A380X_FCU_SPD_PULL, 0),
    FCUC("heading_up", A380X_FCU_HDG_ROTATE, 1), FCUC("heading_down", A380X_FCU_HDG_ROTATE, -1),
    FCUC("heading_push", A380X_FCU_HDG_PUSH, 0), FCUC("heading_pull", A380X_FCU_HDG_PULL, 0),
    FCUC("altitude_up", A380X_FCU_ALT_ROTATE, 1), FCUC("altitude_down", A380X_FCU_ALT_ROTATE, -1),
    FCUC("altitude_push", A380X_FCU_ALT_PUSH, 0), FCUC("altitude_pull", A380X_FCU_ALT_PULL, 0),
    FCUC("vertical_speed_up", A380X_FCU_VS_ROTATE, 1), FCUC("vertical_speed_down", A380X_FCU_VS_ROTATE, -1),
    FCUC("vertical_speed_push", A380X_FCU_VS_PUSH, 0), FCUC("vertical_speed_pull", A380X_FCU_VS_PULL, 0),
    FCUC("altitude_step_toggle", A380X_FCU_ALT_INCREMENT, 0),
    FCUC("ap1_toggle", A380X_FCU_AP1, 0), FCUC("ap2_toggle", A380X_FCU_AP2, 0),
    FCUC("fd_toggle", A380X_FCU_FD1, 0), FCUC("athr_toggle", A380X_FCU_ATHR, 0),
    FCUC("speed_mach_toggle", A380X_FCU_SPD_MACH, 0), FCUC("trk_fpa_toggle", A380X_FCU_TRK_FPA, 0),
    FCUC("altitude_hold", A380X_FCU_ALT_HOLD, 0), FCUC("loc_toggle", A380X_FCU_LOC, 0),
    FCUC("approach_toggle", A380X_FCU_APPR, 0), FCUC("ap_disconnect", A380X_FCU_AP_DISCONNECT, 0),
    FCUC("athr_disconnect", A380X_FCU_ATHR_DISCONNECT, 0),
    FCUC("true_mag_toggle", A380X_FCU_TRUE_MAG, 0), FCUC("metric_toggle", A380X_FCU_METRIC_ALT, 0),
    FCUC("ls_captain_toggle", A380X_FCU_LS1, 0), FCUC("ls_first_officer_toggle", A380X_FCU_LS2, 0),
};
#undef FCUC

typedef struct { const char *name; XPLMDataRef handle; float value; } FcuPublished;
#define FCUR(name) {"a380x/xp12/fcu/" name, NULL, 0}
static FcuPublished fcu_outputs[] = {
    FCUR("selected_speed"), FCUR("selected_mach"), FCUR("selected_heading"),
    FCUR("selected_altitude"), FCUR("selected_vertical_speed"), FCUR("selected_fpa"),
    FCUR("speed_managed"), FCUR("heading_managed"), FCUR("altitude_managed"),
    FCUR("mach_mode"), FCUR("trk_fpa"), FCUR("true_heading"), FCUR("metric_altitude"),
    FCUR("ap1_active"), FCUR("ap2_active"), FCUR("fd_active"), FCUR("athr_active"),
    FCUR("loc_active"), FCUR("approach_active"), FCUR("altitude_step_ft"),
    FCUR("servo_active"), FCUR("ls_captain"), FCUR("ls_first_officer"),
    FCUR("athr_armed"), FCUR("altitude_hold_active"),
    FCUR("land_active"), FCUR("flare_active"), FCUR("rollout_active"), FCUR("retard_active"),
    FCUR("go_around_active"), FCUR("land_capability"), FCUR("yaw_servo_active"),
};
#undef FCUR
static float fcu_get_float(void *refcon) { return ((FcuPublished *)refcon)->value; }
static int fcu_get_int(void *refcon) { return (int)((FcuPublished *)refcon)->value; }
static float normalized_heading(float value) { return fmodf(fmodf(value,360.0f)+360.0f,360.0f); }

#include "a380x_fcu_motion.inc"

static void release_yaw_control(void)
{
    if (fcu.own_yaw) { XPLMSetDataf(fcu.pilot_rudder,0); XPLMSetDatai(fcu.override_heading,0); fcu.own_yaw=0; }
}
static void release_ap_controls(void)
{
    if (fcu.own_pitch) { XPLMSetDataf(fcu.pitch,0); XPLMSetDatai(fcu.override_pitch,0); fcu.own_pitch=0; }
    if (fcu.own_roll) { XPLMSetDataf(fcu.roll,0); XPLMSetDatai(fcu.override_roll,0); fcu.own_roll=0; }
    release_yaw_control();
}
static void release_thrust_control(void)
{
    if (fcu.own_throttle) { XPLMSetDatai(fcu.override_throttle,0); fcu.own_throttle=0; }
}
static void release_guidance_controls(void) { release_ap_controls(); release_thrust_control(); }

static int controls_available(void)
{
    return !XPLMGetDatai(fcu.override_all)
        && (!XPLMGetDatai(fcu.override_pitch) || fcu.own_pitch)
        && (!XPLMGetDatai(fcu.override_roll) || fcu.own_roll);
}

/* Cockpit AFS diagnostics (docs/AUTOPILOT_DEVELOPMENT.md, "When AP or A/THR will not engage"): when an AP
 * or the A/THR does not engage, or disconnects, the reason is shown in the simulator window
 * (XPLMSpeakString) and written to Log.txt as "A380X AFS: ..."; an "A380X AFS status" line is logged
 * every 10 s in flight while AP/FD/A-THR are in use. Nothing here changes the guidance. */
#include <stdarg.h>
static char afs_last_notice[256];
static float afs_last_notice_time=-100.0f, afs_athr_check_at=-1.0f, afs_status_at;
static int afs_override_axis=-1;
static float afs_override_value;
/* AP pressed on the ground: engaged by update_guidance after liftoff (0 none, 1 AP1, 2 AP2). */
static int afs_ap_pending;
/* Own copy of the AFS lines, kept beyond X-Plane's Log.txt rotation: Output/A380X-XP12_AFS.log (this
 * X-Plane session) and Output/A380X-XP12_AFS.previous.log (the session before). */
static FILE *afs_file;
static int afs_file_state;   /* 0 not opened yet, 1 open, -1 unavailable */
static void afs_file_write(const char *line)
{
    static int rotated;
    if (afs_file_state==0) {
        char root[1024], path[1200], previous[1220];
        root[0]=0; afs_file_state=-1;
        XPLMGetSystemPath(root);
        if (root[0] && snprintf(path,sizeof(path),"%sOutput/A380X-XP12_AFS.log",root)<(int)sizeof(path)
            && snprintf(previous,sizeof(previous),"%sOutput/A380X-XP12_AFS.previous.log",root)<(int)sizeof(previous)) {
            if (!rotated) { remove(previous); rename(path,previous); rotated=1; }
            if ((afs_file=fopen(path,"a"))) afs_file_state=1;
        }
    }
    if (afs_file_state==1 && afs_file) {
        fprintf(afs_file,"%9.1f %s",XPLMGetElapsedTime(),line);
        fflush(afs_file);
    }
}
static void afs_log(const char *format, ...)
{
    char text[256], line[300];
    va_list args;
    va_start(args,format); vsnprintf(text,sizeof(text),format,args); va_end(args);
    snprintf(line,sizeof(line),"A380X AFS: %s\n",text); XPLMDebugString(line); afs_file_write(line);
}
static void afs_notice(const char *format, ...)
{
    char text[256], line[300];
    va_list args;
    float now=XPLMGetElapsedTime();
    va_start(args,format); vsnprintf(text,sizeof(text),format,args); va_end(args);
    if (!strcmp(text,afs_last_notice) && now-afs_last_notice_time<5.0f) return;
    snprintf(afs_last_notice,sizeof(afs_last_notice),"%s",text); afs_last_notice_time=now;
    snprintf(line,sizeof(line),"A380X AFS: %s\n",text); XPLMDebugString(line); afs_file_write(line);
    XPLMSpeakString(text);
}
static double afs_max_lever(void)
{
    int i; double m=0;
    for (i=0;i<4;++i) if (isfinite(guidance_inputs.throttle_lever_ratio[i])) m=fmax(m,guidance_inputs.throttle_lever_ratio[i]);
    return m;
}
/* Why an armed A/THR is not active (the guidance activation terms, in plain language). */
static void afs_athr_reason(void)
{
    char why[220]; size_t len=0; int n=0;
    double lever=afs_max_lever(), cl=guidance_inputs.cl_detent_ratio, tol=guidance_inputs.detent_tolerance_ratio;
    why[0]=0;
#define AFS_WHY(cond,...) do { if(cond) { int w; if(n && len<sizeof(why)) { w=snprintf(why+len,sizeof(why)-len,"; "); if(w>0) len+=(size_t)w; } \
        if(len<sizeof(why)) { w=snprintf(why+len,sizeof(why)-len,__VA_ARGS__); if(w>0) len+=(size_t)w; } len=len>=sizeof(why)?sizeof(why)-1:len; ++n; } } while(0)
    AFS_WHY(!guidance_inputs.powered,"FCU not powered");
    AFS_WHY(guidance_inputs.paused,"simulator paused");
    AFS_WHY(guidance_inputs.on_ground,"on the ground (A/THR becomes active in flight)");
    AFS_WHY(!guidance_inputs.athr_speed_valid,"speed data invalid (ADR)");
    AFS_WHY(!guidance_inputs.throttle_levers_valid,"thrust levers not recognised (fcu-calibration.ini / lever input)");
    AFS_WHY(guidance_inputs.throttle_levers_valid && lever>=guidance_inputs.mct_detent_ratio-tol,
            "thrust levers in FLX/MCT or TOGA (manual thrust): bring them below FLX/MCT, e.g. to CL "
            "(throttle keys stop at CL, or use a380x/xp12/thrust/levers_cl)");
    (void)cl;
    AFS_WHY(guidance_inputs.throttle_levers_valid && lever<=guidance_inputs.idle_detent_ratio,"thrust levers at IDLE");
#undef AFS_WHY
    afs_notice("A/THR armed but not active: %s",n?why:"conditions met, activation pending");
}
static void afs_status_log(void)
{
    char line[600];
    snprintf(line,sizeof(line),
        "A380X AFS status: AP %d/%d FD %d/%d ATHR armed %d active %d thr %d lat %d vert %d | IAS %.0f VLS %.0f VMAX %.0f "
        "ALT %.0f V/S %.0f HDG %.0f | sel SPD %.0f HDG %.0f ALT %.0f V/S %.0f | levers %.3f/%.3f/%.3f/%.3f valid %d CL %.3f | "
        "override J%d P%d R%d T%d own P%d R%d T%d lease %d | adr %d ir %d gnd %d pending %d\n",
        guidance.ap1,guidance.ap2,guidance.fd1,guidance.fd2,guidance.athr_armed,guidance.athr_active,(int)guidance.thrust_mode,
        (int)guidance.lateral_mode,(int)guidance.vertical_mode,guidance_inputs.ias_knots,guidance_inputs.vls_knots,
        guidance_inputs.vmax_knots,guidance_inputs.altitude_ft,guidance_inputs.vertical_speed_fpm,guidance_inputs.heading_deg,
        guidance.selected_speed_knots,guidance.selected_heading_deg,guidance.selected_altitude_ft,guidance.selected_vs_fpm,
        guidance_inputs.throttle_lever_ratio[0],guidance_inputs.throttle_lever_ratio[1],guidance_inputs.throttle_lever_ratio[2],
        guidance_inputs.throttle_lever_ratio[3],guidance_inputs.throttle_levers_valid,guidance_inputs.cl_detent_ratio,
        XPLMGetDatai(fcu.override_all),XPLMGetDatai(fcu.override_pitch),XPLMGetDatai(fcu.override_roll),
        XPLMGetDatai(fcu.override_throttle),fcu.own_pitch,fcu.own_roll,fcu.own_throttle,fcu_thrust_link.lease,
        guidance_inputs.airdata_valid,guidance_inputs.attitude_valid,guidance_inputs.on_ground,afs_ap_pending);
    XPLMDebugString(line); afs_file_write(line);
}

/* Optional observer of applied FCU commands (the native FMS knob events, spec 07 section 2.1). */
static void (*fcu_command_observer)(A380XFcuCommand command);
/* Optional distance (nm) to the tuned localizer antenna of a receiver side (a380x_autoland_adapter.inc);
 * returns a negative value when unknown. NULL keeps the DME distance. */
static double (*fcu_localizer_distance_provider)(int side);

#include "a380x_fcu_thrust.inc"
#include "a380x_fcu_authority.inc"

static int fcu_command_handler(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    FcuCommand *c=(FcuCommand *)refcon;
    A380XGuidanceInputs command_inputs=guidance_inputs;
    float now=XPLMGetElapsedTime();
    (void)command;
    if(plugin_enabled || phase==xplm_CommandEnd)
        a380x_fcu_motion_phase(&fcu_motion,c->command,(int)phase,now);
    if (!plugin_enabled || phase==xplm_CommandEnd) return 1;
    if (phase==xplm_CommandContinue && (c->amount==0 || now<c->next_repeat)) return 1;
    c->next_repeat=now+(phase==xplm_CommandBegin ? 0.4f : 0.08f);
    int ap_request=(c->command==A380X_FCU_AP1 && !guidance.ap1) ? 1 : (c->command==A380X_FCU_AP2 && !guidance.ap2) ? 2 : 0;
    if (c->command==A380X_FCU_AP_DISCONNECT && afs_ap_pending) { afs_log("AP%d engagement after liftoff cancelled",afs_ap_pending); afs_ap_pending=0; }
    if (ap_request && afs_ap_pending==ap_request) {      /* second press on the ground: cancel */
        afs_notice("AP%d: engagement after liftoff cancelled",ap_request);
        afs_ap_pending=0;
        return 1;
    }
    if ((c->command==A380X_FCU_AP1 && !guidance.ap1) || (c->command==A380X_FCU_AP2 && !guidance.ap2)) {
        Side *side=&refs.side[c->command==A380X_FCU_AP2 ? 1 : 0];
        if (!controls_available()) {
            afs_notice("AP%d not engaged: the flight controls are held by another add-on (override_joystick %d, pitch %d, roll %d)",
                       ap_request,XPLMGetDatai(fcu.override_all),XPLMGetDatai(fcu.override_pitch),XPLMGetDatai(fcu.override_roll));
            return 1;
        }
        /* Engagement is evaluated against the requested channel, before the
         * AP state changes. Never borrow a healthy captain sensor set for AP2. */
        command_inputs.attitude_valid=side->display_valid && side->attitude_valid && side->heading_valid && side->navigation_valid;
        command_inputs.airdata_valid=side->air_valid && side->altitude_valid && side->vertical_speed_valid;
        command_inputs.ias_knots=side->ias_value; command_inputs.mach=side->mach_value;
        command_inputs.altitude_ft=side->altitude_value; command_inputs.vertical_speed_fpm=side->vertical_speed_value;
        command_inputs.pitch_deg=side->pitch_value; command_inputs.roll_deg=side->roll_value;
        command_inputs.heading_deg=side->heading_value;
        command_inputs.true_heading_deg=normalized_heading(side->heading_value+XPLMGetDataf(fcu.magvar));
        command_inputs.radio_height_ft=side->radio_altitude_valid ? side->radio_altitude_value : 0;
        /* Native servos must not add a second actuator controller. This one-
         * shot only disconnects stock servos; no native AP mode is engaged. */
        fcu.suppress_native_disconnect=1;
        XPLMCommandOnce(fcu.native_disconnect);
        fcu.suppress_native_disconnect=0;
    }
    if (c->command==A380X_FCU_ATHR && !guidance.athr_armed
        && XPLMGetDatai(fcu.override_throttle) && !fcu.own_throttle && !fcu_thrust_link.lease) {
        afs_notice("A/THR not armed: the throttles are held by another controller (override_throttles set, no thrust-lever lease)");
        return 1;
    }
    if (c->command==A380X_FCU_ATHR && !guidance.athr_armed) fcu_authority_prepare_athr();
    a380x_guidance_command(&guidance,&command_inputs,c->command,c->amount);
    if (ap_request && !(ap_request==1 ? guidance.ap1 : guidance.ap2)) {
        char why[220];
        if (command_inputs.on_ground && command_inputs.powered) {
            afs_ap_pending=ap_request;
            afs_notice("AP%d armed: it engages by itself after liftoff (above 100 ft, stick released). Press again to cancel",
                       ap_request);
        } else {
            a380x_guidance_engage_problems(&guidance,&command_inputs,why,sizeof(why));
            afs_notice("AP%d not engaged: %s",ap_request,why[0]?why:"engagement refused");
        }
    } else if (ap_request) { afs_ap_pending=0; afs_log("AP%d engaged",ap_request); }
    else if (c->command==A380X_FCU_AP1 || c->command==A380X_FCU_AP2) afs_log("AP%d off (pushbutton)",c->command==A380X_FCU_AP1?1:2);
    if (c->command==A380X_FCU_ATHR) afs_log("A/THR pushbutton: armed %d active %d",guidance.athr_armed,guidance.athr_active);
    if (c->command==A380X_FCU_ATHR) {
        if (guidance.athr_armed && !guidance.athr_active) afs_athr_check_at=now+2.0f;
        else if (!guidance.athr_armed && command_inputs.athr_inhibited)
            afs_notice("A/THR not armed: A/THR disabled (instinctive disconnect held more than 15 s) until the FMGC resets on the ground");
    }
    a380x_fcu_motion_applied(&fcu_motion,c->command,c->amount);
    if (fcu_command_observer) fcu_command_observer(c->command);
    if (c->command==A380X_FCU_FD1 && guidance.fd1!=guidance.fd2)
        a380x_guidance_command(&guidance,&guidance_inputs,A380X_FCU_FD2,0);
    if (!guidance.ap1 && !guidance.ap2) release_ap_controls();
    if (!guidance.athr_armed) release_thrust_control();
    return 1;
}

static int native_disconnect_handler(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    (void)command; (void)refcon;
    if (phase==xplm_CommandBegin && !fcu.suppress_native_disconnect) {
        a380x_guidance_command(&guidance,&guidance_inputs,A380X_FCU_AP_DISCONNECT,0);
        release_ap_controls();
    }
    return 1; /* Preserve X-Plane's normal disconnect action too. */
}

static int pilot_override(void)
{
    int assignments[500], n, i;
    float values[500], minimum[500], maximum[500];
    /* Read the physical axes, not our own overridden yoke output. */
    n=XPLMGetDatavi(fcu.axis_assignments,assignments,0,500);
    if (n>500) n=500;
    if (n<0) n=0;
    if (XPLMGetDatavf(fcu.axis_values,values,0,n)==n
        && XPLMGetDatavf(fcu.axis_minimum,minimum,0,n)==n
        && XPLMGetDatavf(fcu.axis_maximum,maximum,0,n)==n) {
        for (i=0;i<n;++i) {
            float range=maximum[i]-minimum[i], value=values[i];
            if (isfinite(range) && range>0.001f) value=(value-minimum[i])/range;
            if (assignments[i]>=1 && assignments[i]<=3 && isfinite(value) && fabsf(value-0.5f)>0.20f) {
                afs_override_axis=i; afs_override_value=value; return 1;
            }
        }
    }
    /* While the yaw lease is held, yoke_heading_ratio is this plugin's own pedal output: only the
     * physical rudder axis above (assignment 3) can then signal a pilot pedal override. */
    afs_override_axis=-2;
    return !fcu.own_yaw && fabsf(XPLMGetDataf(fcu.pilot_rudder))>0.35f;
}

static void read_guidance_inputs(float dt)
{
    int i, side_index=guidance.ap2 && !guidance.ap1 ? 1 : 0;
    Side *s=&refs.side[side_index];
    float variation=XPLMGetDataf(fcu.magvar);
    memset(&guidance_inputs,0,sizeof(guidance_inputs));
    /* Managed speed target from the native FMS only (spec 07 section 10.2); the stock X-Plane
     * FMS/autopilot state is never consumed. */
    if(fms_feed.valid&&fms_feed.managed_speed_valid&&isfinite(fms_feed.managed_speed_knots)
       &&fms_feed.managed_speed_knots>0){
        guidance_inputs.managed_speed_valid=1;
        guidance_inputs.managed_speed_knots=fms_feed.managed_speed_knots;
        guidance_inputs.managed_speed_is_mach=fms_feed.managed_speed_is_mach;
        guidance_inputs.managed_mach=fms_feed.managed_mach;
    }
    /* Managed lateral/vertical guidance from the native FMGC (a380x_fmgc_guidance.c). */
    if(fms_feed.valid&&fms_feed.route_valid){
        guidance_inputs.managed_route_valid=1;
        guidance_inputs.managed_nav_capture=fms_feed.nav_capture;
        guidance_inputs.route_xtk_nm=fms_feed.route_xtk_nm;
        guidance_inputs.route_track_error_deg=fms_feed.route_track_error_deg;
        guidance_inputs.route_bank_deg=fms_feed.route_bank_deg;
        guidance_inputs.managed_heading_deg=fms_feed.managed_heading_deg;
    }
    if(fms_feed.valid&&fms_feed.route_valid&&fms_feed.vertical_valid){
        guidance_inputs.managed_vertical_valid=1;
        guidance_inputs.managed_altitude_ft=fms_feed.managed_altitude_ft;
        guidance_inputs.managed_altitude_is_constraint=fms_feed.altitude_is_constraint;
        guidance_inputs.managed_cruise_altitude_ft=fms_feed.cruise_altitude_ft;
        guidance_inputs.managed_path_valid=fms_feed.path_valid;
        guidance_inputs.managed_path_altitude_ft=fms_feed.path_altitude_ft;
        guidance_inputs.managed_path_vs_fpm=fms_feed.path_vs_fpm;
        guidance_inputs.managed_vdev_ft=fms_feed.vdev_ft;
        guidance_inputs.managed_path_idle=fms_feed.path_idle;
        guidance_inputs.managed_before_tod=fms_feed.before_tod;
    }
    guidance_inputs.fms_phase_valid=fms_feed.valid&&fms_feed.phase_valid;
    guidance_inputs.fms_phase=fms_feed.phase;
    guidance_inputs.fms_v2_knots=fms_feed.valid?fms_feed.v2_knots:0;
    ground_track_degrees=normalized_heading(XPLMGetDataf(refs.ground_track)-variation);
    /* FCU channel 1 on 108PH, channel 2 on DC 2 (FBWI:2372); stock avionics master without the network. */
    guidance_inputs.powered=a380x_power_fcu(XPLMGetDatai(refs.avionics_power)!=0);
    guidance_inputs.paused=XPLMGetDatai(fcu.paused)||XPLMGetDatai(fcu.replay);
    guidance_inputs.on_ground=XPLMGetDatai(refs.on_ground)!=0;
    guidance_inputs.attitude_valid=s->display_valid && s->attitude_valid && s->heading_valid && s->navigation_valid;
    guidance_inputs.airdata_valid=s->air_valid && s->altitude_valid && s->vertical_speed_valid;
    guidance_inputs.ias_knots=s->ias_value; guidance_inputs.mach=s->mach_value;
    guidance_inputs.athr_speed_valid=s->air_valid && isfinite(s->ias_value) && isfinite(s->mach_value);
    guidance_inputs.tas_knots=XPLMGetDataf(fcu.tas)*1.94384449;
    guidance_inputs.ground_speed_knots=ground_speed_knots;
    guidance_inputs.altitude_ft=s->altitude_value;
    guidance_inputs.pressure_altitude_ft=XPLMGetDatad(fcu.pressure_altitude);
    guidance_inputs.radio_height_ft=s->radio_altitude_valid ? s->radio_altitude_value : 0;
    guidance_inputs.vertical_speed_fpm=s->vertical_speed_value;
    guidance_inputs.pitch_deg=s->pitch_value; guidance_inputs.roll_deg=s->roll_value;
    guidance_inputs.heading_deg=s->heading_value; guidance_inputs.track_deg=ground_track_degrees;
    guidance_inputs.true_heading_deg=normalized_heading(s->heading_value+variation);
    guidance_inputs.true_track_deg=XPLMGetDataf(refs.ground_track);
    guidance_inputs.pitch_rate_deg_s=XPLMGetDataf(fcu.pitch_rate);
    guidance_inputs.roll_rate_deg_s=XPLMGetDataf(fcu.roll_rate);
    guidance_inputs.yaw_rate_deg_s=XPLMGetDataf(fcu.yaw_rate);
    guidance_inputs.alpha_deg=XPLMGetDataf(fcu.alpha); guidance_inputs.beta_deg=XPLMGetDataf(fcu.beta);
    guidance_inputs.weight_kg=XPLMGetDataf(fcu.weight);
    fcu_thrust_read_into(&guidance_inputs);
    guidance_inputs.vmax_knots=XPLMGetDataf(fcu.vmax);
    /* At the measured air-data pressure, CAS scales nonlinearly with Mach.
     * The isentropic pitot relation converts MMO to the current IAS scale. */
    if (s->air_valid && s->mach_value>.1f && s->ias_value>0) {
        double mmo=XPLMGetDataf(fcu.mmo);
        if (isfinite(mmo) && mmo>0 && mmo<1) {
            double impact=pow(1+0.2*pow(s->ias_value/661.4788,2),3.5)-1;
            double pressure_ratio=impact/(pow(1+0.2*s->mach_value*s->mach_value,3.5)-1);
            double mmo_ias=661.4788*sqrt(5*(pow(1+pressure_ratio*(pow(1+.2*mmo*mmo,3.5)-1),2.0/7.0)-1));
            guidance_inputs.vmax_knots=fmin(guidance_inputs.vmax_knots,mmo_ias);
        }
    }
    /* X-Plane's aircraft axial +Z points aft, side +X points right:
     * developer.x-plane.com/article/movingtheplane/. The pinned FBW PRIM
     * maps body longitudinal/lateral g with gain 9.81, and normal (g-1)
     * with gain 9.81 into OuterLoops bx/by/bz (A380PrimComputerFg.cpp).
     * A steady 1g flight therefore supplies zero incremental normal accel. */
    guidance_inputs.accel_x_m_s2=-XPLMGetDataf(fcu.g_axial)*9.81;
    guidance_inputs.accel_y_m_s2=XPLMGetDataf(fcu.g_side)*9.81;
    guidance_inputs.accel_z_m_s2=(XPLMGetDataf(fcu.g_normal)-1.0)*9.81;
    if (!isfinite(guidance_inputs.accel_x_m_s2) || !isfinite(guidance_inputs.accel_y_m_s2)
        || !isfinite(guidance_inputs.accel_z_m_s2) || !isfinite(guidance_inputs.pitch_rate_deg_s)
        || !isfinite(guidance_inputs.roll_rate_deg_s) || !isfinite(guidance_inputs.yaw_rate_deg_s))
        guidance_inputs.attitude_valid=0;
    if (!isfinite(guidance_inputs.pressure_altitude_ft)) guidance_inputs.airdata_valid=0;
    for (i=0;i<4;++i) guidance_inputs.throttle_ratio+=throttle_values[i]*0.25;
    read_pfd_aircraft();
    guidance_inputs.vmax_knots=pfd_aircraft.vmax_knots;
    guidance_inputs.vls_knots=pfd_aircraft.vls_knots;
    guidance_inputs.radio_height_valid=s->radio_altitude_valid;
    guidance_inputs.flap_handle_index=pfd_aircraft.flap_handle_index;
    guidance_inputs.rudder_ratio=fcu.own_yaw ? guidance.land.yaw_servo : XPLMGetDataf(fcu.pilot_rudder);
    /* LAND capability redundancy (a380x_autoland.inc): both radio altimeters and ADIRS sides, X-Plane
     * hydraulic systems 1 and 2 (green / yellow) above 900 Plane Maker units (the surfaces adapter
     * support threshold; nominal 3000), no rudder failure, the rudder override free or ours. */
    for (i=0;i<2;++i) {
        Side *side=&refs.side[i];
        float pressure=XPLMGetDataf(fcu.hydraulic_pressure[i]);
        guidance_inputs.side_radio_height_valid[i]=side->radio_altitude_valid;
        guidance_inputs.side_radio_height_ft[i]=side->radio_altitude_valid ? side->radio_altitude_value : 0;
        guidance_inputs.side_attitude_valid[i]=side->display_valid && side->attitude_valid && side->heading_valid && side->navigation_valid;
        guidance_inputs.side_airdata_valid[i]=side->air_valid && side->altitude_valid && side->vertical_speed_valid;
        if (i==0) guidance_inputs.hydraulics_valid=1;
        if (!(isfinite(pressure) && pressure>=900.0f)) guidance_inputs.hydraulics_valid=0;
    }
    guidance_inputs.rudder_valid=XPLMGetDatai(fcu.rudder_failure[0])<6 && XPLMGetDatai(fcu.rudder_failure[1])<6;
    guidance_inputs.yaw_channel_available=fcu.own_yaw || !XPLMGetDatai(fcu.override_heading);
    for (i=0;i<2;++i) {
        A380XIlsInputs *ils=&guidance_inputs.ils[i];
        Side *side=&refs.side[i];
        int type=XPLMGetDatai(side->nav_type);
        int frequency=XPLMGetDatai(side->nav_frequency);
        int localizer_channel=frequency>=10810 && frequency<=11195
            && frequency%5==0 && (frequency/10)%2==1;
        float course=XPLMGetDataf(side->nav_course), angle=XPLMGetDataf(fcu.gs_angle[i]);
        side->baro_hpa_value=XPLMGetDataf(side->baro)*33.8638867f;
        side->baro_std=XPLMGetDatai(side->baro_standard);
        side->loc_dots=XPLMGetDataf(side->nav_loc);
        side->gs_dots=XPLMGetDataf(side->nav_gs);
        /* nav*_type is an undocumented receiver enum, not necessarily the
         * SDK XPLMNavType bitmask. Require a received localizer-channel signal
         * instead: FAA AIM 1-1-9 frequency table (40 odd-tenth 50-kHz channels).
         * VORs use even tenths below 112 MHz, then 112.00 through 117.95. */
        side->loc_valid=type!=0 && localizer_channel && XPLMGetDatai(side->nav_horizontal)
            && isfinite(side->loc_dots) && isfinite(course);
        side->gs_valid=side->loc_valid && XPLMGetDatai(side->nav_vertical)
            && isfinite(side->gs_dots) && isfinite(angle) && angle>0 && angle<10;
        side->nav_dme_nm=XPLMGetDataf(side->nav_distance);
        side->nav_dme_valid=XPLMGetDatai(side->nav_has_dme) && isfinite(side->nav_dme_nm) && side->nav_dme_nm>=0;
        side->nav_frequency_mhz=(float)frequency*0.01f;
        memset(side->nav_ident,0,sizeof(side->nav_ident));
        XPLMGetDatab(side->nav_identifier,side->nav_ident,0,(int)sizeof(side->nav_ident)-1);
        side->ls_visible=i ? guidance.ls2 : guidance.ls1;
        ils->valid=ils->loc_valid=side->loc_valid; ils->gs_valid=side->gs_valid;
        ils->frequency_mhz=side->loc_valid ? side->nav_frequency_mhz : 0;
        ils->loc_deviation_dots=side->loc_dots; ils->gs_deviation_dots=-side->gs_dots;
        ils->course_deg=course;
        ils->gs_angle_deg=angle;
        ils->distance_nm=side->nav_dme_valid ? side->nav_dme_nm : 0;
        if (fcu_localizer_distance_provider && side->loc_valid) {
            double distance=fcu_localizer_distance_provider(i);
            ils->localizer_distance_valid=isfinite(distance) && distance>0 && distance<60;
            ils->localizer_distance_nm=ils->localizer_distance_valid ? distance : 0;
        }
        if (side->air_valid && side->navigation_valid && dt>0 && dt<=0.5f && !guidance_inputs.paused) {
            if (side->trend_initialized) {
                float target=fmaxf(-50,fminf(50,(side->ias_value-side->previous_ias_value)/dt*10));
                float delta=(target-side->ias_trend_value)*fminf(1,dt/1.5f);
                side->ias_trend_value+=fmaxf(-12*dt,fminf(12*dt,delta));
                side->ias_trend_visible=fabsf(side->ias_trend_value)>(side->ias_trend_visible ? 1:2);
            }
            side->previous_ias_value=side->ias_value; side->trend_initialized=1;
        } else { side->trend_initialized=side->ias_trend_visible=0; side->ias_trend_value=0; }
    }
}

static void publish_guidance(void)
{
    int i=0;
#define A380_FCU_OUT(v) fcu_outputs[i++].value=(float)(v)
    A380_FCU_OUT(guidance.selected_speed_knots); A380_FCU_OUT(guidance.selected_mach); A380_FCU_OUT(guidance.selected_heading_deg);
    A380_FCU_OUT(guidance.selected_altitude_ft); A380_FCU_OUT(guidance.selected_vs_fpm); A380_FCU_OUT(guidance.selected_fpa_deg);
    A380_FCU_OUT(guidance.speed_managed); A380_FCU_OUT(guidance.heading_managed); A380_FCU_OUT(guidance.altitude_managed);
    A380_FCU_OUT(guidance.mach_mode); A380_FCU_OUT(guidance.trk_fpa_mode); A380_FCU_OUT(guidance.true_heading); A380_FCU_OUT(guidance.metric_alt);
    A380_FCU_OUT(guidance.ap1); A380_FCU_OUT(guidance.ap2); A380_FCU_OUT(guidance.fd1||guidance.fd2); A380_FCU_OUT(guidance.athr_active);
    A380_FCU_OUT(guidance.loc_armed || guidance.lateral_mode==A380X_C1_LOC || guidance.lateral_mode==A380X_C1_LOC_STAR);
    A380_FCU_OUT(guidance.approach_selected); A380_FCU_OUT(guidance.altitude_increment_1000 ? 1000 : 100);
    A380_FCU_OUT(fcu.own_pitch && fcu.own_roll); A380_FCU_OUT(guidance.ls1); A380_FCU_OUT(guidance.ls2);
    A380_FCU_OUT(guidance.athr_armed);
    A380_FCU_OUT(guidance.guidance_valid && guidance.vertical_mode==A380X_B1_ALT);
    A380_FCU_OUT(guidance.land.land_active); A380_FCU_OUT(guidance.land.flare_active);
    A380_FCU_OUT(guidance.land.rollout_active); A380_FCU_OUT(guidance.land.retard_active);
    A380_FCU_OUT(guidance.land.go_around_active); A380_FCU_OUT(guidance.land.capability);
    A380_FCU_OUT(fcu.own_yaw);
#undef A380_FCU_OUT
    memset(&pfd_guidance,0,sizeof(pfd_guidance));
#define COPY(v) pfd_guidance.v=guidance.v
    COPY(selected_speed_knots); COPY(selected_mach); COPY(selected_heading_deg);
    COPY(selected_altitude_ft); COPY(selected_vs_fpm); COPY(selected_fpa_deg);
    COPY(speed_managed); COPY(heading_managed); COPY(altitude_managed); COPY(mach_mode); COPY(trk_fpa_mode);
    COPY(ap1); COPY(ap2); COPY(fd1); COPY(fd2); COPY(athr_armed); COPY(athr_active);
    COPY(loc_armed); COPY(gs_armed); COPY(altitude_armed); COPY(guidance_valid);
    COPY(nav_armed); COPY(clb_armed); COPY(des_armed);
    COPY(fd_pitch_error_deg); COPY(fd_roll_error_deg);
    COPY(true_heading); COPY(managed_unavailable); COPY(mode_reversion); COPY(disconnect_warning);
#undef COPY
    pfd_guidance.heading_reference_offset_deg=guidance.true_heading ? XPLMGetDataf(fcu.magvar) : 0;
    pfd_guidance.selected_heading_deg=normalized_heading((float)(guidance.selected_heading_deg-pfd_guidance.heading_reference_offset_deg));
    pfd_guidance.selected_speed_knots=guidance.speed_target_knots;
    pfd_guidance.guidance_fault=guidance.vertical_guidance_selected && (guidance.fd1||guidance.fd2) && guidance_inputs.powered
        && !guidance_inputs.paused && !guidance_inputs.on_ground && !guidance.guidance_valid;
    pfd_guidance.metric_altitude=guidance.metric_alt;
    pfd_guidance.vmax_knots=guidance_inputs.vmax_knots;
    publish_pfd_aircraft();
    published.fma_a1a2=guidance.thrust_mode; published.fma_b1=a380x_guidance_fma_vertical(&guidance);
    published.fma_c1=guidance.lateral_mode; published.fma_a3=0;
    {
        /* Landing capability (a380x_autoland.inc): the annunciated APPR 1 / LAND 2 / LAND 3 SINGLE /
         * LAND 3 DUAL, exactly one (FBW computeD1D2Message would show LAND 2 before LAND 3). */
        A380XD1D2Inputs d1d2; memset(&d1d2,0,sizeof(d1d2));
        d1d2.land_mode_armed=guidance.approach_selected && !guidance.land.land_active;
        d1d2.land_mode_active=guidance.land.land_active;
        d1d2.land2_capacity=guidance.land.capability==A380X_LAND_CAPABILITY_LAND2;
        d1d2.land3_single_capacity=guidance.land.capability==A380X_LAND_CAPABILITY_LAND3_SINGLE;
        d1d2.land3_dual_capacity=guidance.land.capability==A380X_LAND_CAPABILITY_LAND3_DUAL;
        published.fma_d1d2=guidance_inputs.powered ? a380x_compute_d1d2(&d1d2) : A380X_D1D2_NONE;
    }
    pfd_guidance.fma_bc1=guidance.land.bc1;
    /* FMA D3 minimums (FBW D3Cell): NO DH, RADIO (DH) or BARO (MDA) from the native FMS. */
    pfd_guidance.fma_minimum_mode=fms_feed.valid ? fms_feed.minimum_mode : 0;
    pfd_guidance.fma_minimum_ft=fms_feed.valid ? fms_feed.minimum_ft : 0;
}

static void update_guidance(float dt)
{
    float thrust[4]; int i;
    a380x_fcu_motion_update(&fcu_motion,dt,XPLMGetElapsedTime(),guidance.altitude_increment_1000);
    read_guidance_inputs(dt);
    if ((guidance.ap1||guidance.ap2) && (!controls_available() || pilot_override()
        || (fcu.own_pitch && !XPLMGetDatai(fcu.override_pitch))
        || (fcu.own_roll && !XPLMGetDatai(fcu.override_roll))
        || (fcu.own_yaw && !XPLMGetDatai(fcu.override_heading)))) {
        int which=guidance.ap1?1:2;
        if (!controls_available()) afs_notice("AP%d disconnected: the flight controls were taken by another add-on",which);
        else if (pilot_override()) {
            if (afs_override_axis>=0)
                afs_notice("AP%d disconnected: joystick axis %d (pitch/roll/yaw) at %.2f, more than 20%% off centre - "
                           "centre the stick/rudder or recalibrate it",which,afs_override_axis,afs_override_value);
            else afs_notice("AP%d disconnected: rudder pedal input",which);
        } else afs_notice("AP%d disconnected: control lease lost",which);
        a380x_guidance_command(&guidance,&guidance_inputs,A380X_FCU_AP_DISCONNECT,0);
    }
    if ((guidance.athr_armed || guidance.athr_active)
        && ((XPLMGetDatai(fcu.override_throttle) && !fcu.own_throttle && !fcu_thrust_link.lease)
            || (fcu.own_throttle && !XPLMGetDatai(fcu.override_throttle))))
        a380x_guidance_command(&guidance,&guidance_inputs,A380X_FCU_ATHR_DISCONNECT,0);
    {
        int ap_before=guidance.ap1||guidance.ap2, athr_before=guidance.athr_armed;
        float now=XPLMGetElapsedTime();
        a380x_guidance_update(&guidance,&guidance_inputs,dt);
        if (ap_before && !guidance.ap1 && !guidance.ap2) {
            char why[220];
            a380x_guidance_engage_problems(&guidance,&guidance_inputs,why,sizeof(why));
            afs_notice("AP disconnected by the guidance: %s",why[0]?why:"mode or sensor limit");
        }
        if (athr_before && !guidance.athr_armed) afs_notice("A/THR disconnected");
        /* AP pressed on the ground: engage once airborne above 100 ft RA (5 s airborne without a radio
         * altitude), with the pilot's stick and rudder released so the engagement is not overridden. */
        if (afs_ap_pending) {
            if (guidance.ap1 || guidance.ap2 || !guidance_inputs.powered) afs_ap_pending=0;
            else if (!guidance_inputs.on_ground && !guidance_inputs.paused
                     && (guidance_inputs.radio_height_valid ? guidance_inputs.radio_height_ft>=100
                                                            : guidance.airborne_seconds>=5)) {
                int which=afs_ap_pending;
                if (!controls_available())
                    afs_notice("AP%d waiting to engage: the flight controls are held by another add-on",which);
                else if (pilot_override())
                    afs_notice("AP%d waiting to engage: release the stick/rudder",which);
                else {
                    a380x_guidance_command(&guidance,&guidance_inputs,which==1?A380X_FCU_AP1:A380X_FCU_AP2,0);
                    if (guidance.ap1 || guidance.ap2) { afs_notice("AP%d engaged",which); afs_ap_pending=0; }
                    else {
                        char why[220];
                        a380x_guidance_engage_problems(&guidance,&guidance_inputs,why,sizeof(why));
                        afs_notice("AP%d waiting to engage: %s",which,why[0]?why:"engagement refused");
                    }
                }
            }
        }
        if (afs_athr_check_at>0.0f && now>=afs_athr_check_at) {
            afs_athr_check_at=-1.0f;
            if (guidance.athr_armed && !guidance.athr_active) afs_athr_reason();
        }
        if (now>=afs_status_at && !guidance_inputs.on_ground &&
            (guidance.fd1||guidance.fd2||guidance.ap1||guidance.ap2||guidance.athr_armed)) {
            afs_status_at=now+10.0f;
            afs_status_log();
        }
    }
    /* A direct foreign native activation must be caught after automatic
     * detent arming and before either custom actuator lease is acquired. */
    fcu_authority_guard();
    if (guidance_inputs.paused || !guidance_inputs.powered) release_guidance_controls();
    else {
        if ((guidance.ap1||guidance.ap2) && guidance.guidance_valid && controls_available()) {
            if (!fcu.own_pitch) { XPLMSetDatai(fcu.override_pitch,1); fcu.own_pitch=1; }
            if (!fcu.own_roll) { XPLMSetDatai(fcu.override_roll,1); fcu.own_roll=1; }
            XPLMSetDataf(fcu.pitch,(float)guidance.pitch_servo);
            XPLMSetDataf(fcu.roll,(float)guidance.roll_servo);
            /* Yaw only in ALIGN/ROLL OUT. A foreign heading override at that moment is contention:
             * the AP cannot land without its rudder channel, so it disconnects (AP OFF warning). */
            if (guidance.land.yaw_active) {
                if (!fcu.own_yaw && XPLMGetDatai(fcu.override_heading)) {
                    a380x_guidance_command(&guidance,&guidance_inputs,A380X_FCU_AP_DISCONNECT,0);
                    release_ap_controls();
                } else {
                    if (!fcu.own_yaw) { XPLMSetDatai(fcu.override_heading,1); fcu.own_yaw=1; }
                    XPLMSetDataf(fcu.pilot_rudder,(float)guidance.land.yaw_servo);
                }
            } else release_yaw_control();
        } else {
            if (fcu.own_pitch) { XPLMSetDataf(fcu.pitch,0); XPLMSetDatai(fcu.override_pitch,0); fcu.own_pitch=0; }
            if (fcu.own_roll) { XPLMSetDataf(fcu.roll,0); XPLMSetDatai(fcu.override_roll,0); fcu.own_roll=0; }
            release_yaw_control();
        }
        if (guidance.athr_active && (!XPLMGetDatai(fcu.override_throttle)||fcu.own_throttle||fcu_thrust_link.lease)) {
            XPLMSetDatai(fcu.override_throttle,1); fcu.own_throttle=1;
            for (i=0;i<4;++i) {
                double command=fmax(0,fmin(1,guidance.throttle_command));
                /* Normal speed regulation respects each pilot lever. A CL
                 * lever on one engine must never advance another idle engine.
                 * Qualified A.FLOOR / TOGA LK intentionally override levers. */
                if (!(guidance.alpha_floor_latched &&
                      (guidance.thrust_mode==A380X_A1A2_A_FLOOR || guidance.thrust_mode==A380X_A1A2_TOGA_LK))) {
                    double ceiling=0;
                    if (fcu_thrust_link.ready && fcu_thrust_link.ceiling_valid && isfinite(fcu_thrust_link.ceiling[i]))
                        ceiling=fmax(0,fmin(1,fcu_thrust_link.ceiling[i]));   /* FADEC N1 of the lever */
                    else if (guidance_inputs.throttle_levers_valid && isfinite(guidance_inputs.cl_detent_ratio)
                        && guidance_inputs.cl_detent_ratio>0 && isfinite(guidance_inputs.throttle_lever_ratio[i]))
                        ceiling=fmax(0,fmin(.88,guidance_inputs.throttle_lever_ratio[i]/guidance_inputs.cl_detent_ratio*.88));
                    command=fmin(command,ceiling);
                }
                thrust[i]=(float)command;
            }
            XPLMSetDatavf(fcu.engine_throttle,thrust,0,4);
        } else if (fcu.own_throttle) { XPLMSetDatai(fcu.override_throttle,0); fcu.own_throttle=0; }
    }
    publish_guidance();
}

static void fcu_shutdown(void)
{
    afs_ap_pending=0;
    if (afs_file) { fclose(afs_file); afs_file=NULL; }
    afs_file_state=0;
    fcu_authority_shutdown();
    fcu_thrust_shutdown();
    fcu_motion_shutdown();
    pfd_outputs_shutdown();
    int i;
    release_guidance_controls();
    for (i=0;i<ARRAY_LEN(fcu_commands);++i) if (fcu_commands[i].handle) {
        XPLMUnregisterCommandHandler(fcu_commands[i].handle,fcu_command_handler,1,&fcu_commands[i]);
        fcu_commands[i].handle=NULL;
    }
    if (fcu.native_disconnect) {
        XPLMUnregisterCommandHandler(fcu.native_disconnect,native_disconnect_handler,1,NULL);
        fcu.native_disconnect=NULL;
    }
    for (i=0;i<ARRAY_LEN(fcu_outputs);++i) if (fcu_outputs[i].handle) {
        XPLMUnregisterDataAccessor(fcu_outputs[i].handle); fcu_outputs[i].handle=NULL;
    }
    a380x_guidance_destroy(&guidance);
}

static int fcu_init(void)
{
    int ok=1,i;
    char name[180];
    memset(&fcu,0,sizeof(fcu));
#define RAW(field,name) fcu.field=find_required(name,&ok)
    RAW(paused,"sim/time/paused"); RAW(replay,"sim/time/is_in_replay");
    RAW(magvar,"sim/flightmodel/position/magnetic_variation"); RAW(tas,"sim/flightmodel/position/true_airspeed");
    RAW(pitch_rate,"sim/flightmodel/position/Q"); RAW(roll_rate,"sim/flightmodel/position/P");
    RAW(yaw_rate,"sim/flightmodel/position/R"); RAW(alpha,"sim/flightmodel/position/alpha"); RAW(beta,"sim/flightmodel/position/beta");
    RAW(weight,"sim/flightmodel/weight/m_total"); RAW(vmax,"sim/aircraft/view/acf_Vne");
    RAW(mmo,"sim/aircraft/view/acf_Mmo");
    RAW(pressure_altitude,"sim/flightmodel2/position/pressure_altitude");
    RAW(g_axial,"sim/flightmodel/forces/g_axil"); RAW(g_side,"sim/flightmodel/forces/g_side");
    RAW(g_normal,"sim/flightmodel/forces/g_nrml");
    RAW(override_pitch,"sim/operation/override/override_joystick_pitch");
    RAW(override_roll,"sim/operation/override/override_joystick_roll");
    RAW(override_throttle,"sim/operation/override/override_throttles");
    RAW(override_all,"sim/operation/override/override_joystick");
    RAW(override_heading,"sim/operation/override/override_joystick_heading");
    RAW(hydraulic_pressure[0],"sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1");
    RAW(hydraulic_pressure[1],"sim/cockpit2/hydraulics/indicators/hydraulic_pressure_2");
    RAW(rudder_failure[0],"sim/operation/failures/rel_rud_L"); RAW(rudder_failure[1],"sim/operation/failures/rel_rud_R");
    RAW(pitch,"sim/joystick/yoke_pitch_ratio"); RAW(roll,"sim/joystick/yoke_roll_ratio");
    RAW(engine_throttle,"sim/flightmodel/engine/ENGN_thro_use"); RAW(pilot_rudder,"sim/joystick/yoke_heading_ratio");
    RAW(axis_assignments,"sim/joystick/joystick_axis_assignments"); RAW(axis_values,"sim/joystick/joystick_axis_values");
    RAW(axis_minimum,"sim/joystick/joystick_axis_minimum"); RAW(axis_maximum,"sim/joystick/joystick_axis_maximum");
#undef RAW
    for (i=0;i<2;++i) {
        Side *s=&refs.side[i]; const char *suffix=i ? "copilot" : "pilot";
        snprintf(name,sizeof(name),"sim/cockpit2/gauges/actuators/barometer_setting_in_hg_%s",suffix); s->baro=find_required(name,&ok);
        snprintf(name,sizeof(name),"sim/cockpit2/gauges/actuators/barometer_setting_is_std_%s",suffix); s->baro_standard=find_required(name,&ok);
#define NAV(field,tail) do { snprintf(name,sizeof(name),"sim/cockpit2/radios/indicators/nav%d_" tail,i+1); s->field=find_required(name,&ok); } while(0)
        NAV(nav_type,"type"); NAV(nav_horizontal,"display_horizontal"); NAV(nav_vertical,"display_vertical");
        NAV(nav_distance,"dme_distance_nm");
        NAV(nav_identifier,"nav_id"); NAV(nav_has_dme,"has_dme");
#undef NAV
        snprintf(name,sizeof(name),"sim/cockpit2/radios/indicators/nav%d_hdef_dots_%s",i+1,suffix); s->nav_loc=find_required(name,&ok);
        snprintf(name,sizeof(name),"sim/cockpit2/radios/indicators/nav%d_vdef_dots_%s",i+1,suffix); s->nav_gs=find_required(name,&ok);
        snprintf(name,sizeof(name),"sim/cockpit2/radios/actuators/nav%d_course_deg_mag_%s",i+1,suffix); s->nav_course=find_required(name,&ok);
        snprintf(name,sizeof(name),"sim/cockpit2/radios/actuators/nav%d_frequency_hz",i+1); s->nav_frequency=find_required(name,&ok);
        snprintf(name,sizeof(name),"sim/cockpit/radios/nav%d_slope_degt",i+1); fcu.gs_angle[i]=find_required(name,&ok);
    }
    if (!ok) return 0;
    if (!fcu_thrust_init()) return 0;
    pfd_inputs_find();
    read_side(&refs.side[0],XPLMGetDatai(refs.avionics_power));
    read_side(&refs.side[1],XPLMGetDatai(refs.avionics_power));
    read_guidance_inputs(0);
    a380x_guidance_init(&guidance,&guidance_inputs);
    if (!guidance.initialized) return 0;
    for (i=0;i<ARRAY_LEN(fcu_outputs);++i) {
        if (XPLMFindDataRef(fcu_outputs[i].name)) { fcu_shutdown(); return 0; }
        fcu_outputs[i].handle=XPLMRegisterDataAccessor(fcu_outputs[i].name,xplmType_Int|xplmType_Float,0,
            fcu_get_int,NULL,fcu_get_float,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,&fcu_outputs[i],NULL);
        if (!fcu_outputs[i].handle) { fcu_shutdown(); return 0; }
    }
    if(!fcu_motion_init()){ fcu_shutdown();return 0; }
    if(!pfd_outputs_init()){ fcu_shutdown();return 0; }
    for (i=0;i<ARRAY_LEN(fcu_commands);++i) {
        fcu_commands[i].handle=XPLMCreateCommand(fcu_commands[i].name,fcu_commands[i].name+15);
        if (!fcu_commands[i].handle) { fcu_shutdown(); return 0; }
        XPLMRegisterCommandHandler(fcu_commands[i].handle,fcu_command_handler,1,&fcu_commands[i]);
    }
    fcu.native_disconnect=XPLMFindCommand("sim/autopilot/servos_off_any");
    if (!fcu.native_disconnect) { fcu_shutdown(); return 0; }
    XPLMRegisterCommandHandler(fcu.native_disconnect,native_disconnect_handler,1,NULL);
    if (!fcu_authority_init()) { fcu_shutdown(); return 0; }
    publish_guidance();
    return 1;
}
