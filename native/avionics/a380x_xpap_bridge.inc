/* SPDX-License-Identifier: GPL-3.0-or-later
 * X-Plane autopilot / autothrust bridge (2026-11-15).
 *
 * At the user's request the A380X flies its AP and A/THR the way Laminar's A330 does: the FCU drives
 * X-Plane's own autopilot and autothrottle (sim/autopilot commands, sim/cockpit2/autopilot datarefs,
 * the ACF's Airbus autopilot logic) instead of the imported FBW guidance laws. This is an independent
 * implementation of that approach (no Laminar code): the FCU commands are mapped onto X-Plane's AP, and
 * X-Plane's modes, targets and FD orders are written back into the guidance state so the PFD FMA, FD bars,
 * FCU windows and FWS keep working unchanged.
 *
 *   AP1 / AP2        sim/autopilot/servos_toggle / servos2_toggle; sidestick/FCU disconnect servos_off_any
 *   FD               X-Plane flight director on/off (kept on while an AP is engaged)
 *   A/THR            off -> arm (autothrottle_arm), else hard off; instinctive disconnect = hard off.
 *                    In flight, armed, levers above IDLE and below FLX/MCT: active in SPEED (autothrottle_on);
 *                    on the ground or levers at FLX/MCT, TOGA: armed only; all levers to IDLE: off.
 *                    While X-Plane's A/THR drives, the thrust-lever module releases its engine lease.
 *   SPD knob         turn: X-Plane speed target; pull: selected speed (window open) + SPEED; push: managed
 *                    speed (the native FMS speed is written into X-Plane's speed target)
 *   HDG knob         turn: heading target; pull: HDG/TRK; push: NAV = X-Plane heading mode steering the
 *                    native FMS managed heading (the route stays in the A380 FMS)
 *   ALT knob         turn: altitude target (100/1000 ft); pull: OP CLB/OP DES (X-Plane level change);
 *                    push: CLB/DES (level change at managed speed)
 *   V/S knob         turn: V/S (FPA) target; pull: V/S (FPA) at the target; push: level off (V/S 0)
 *   LOC / APPR       X-Plane NAV (LOC) / approach (LOC + G/S) on NAV1
 *   SPD/MACH, TRK/FPA  X-Plane knots/Mach and TRK/FPA toggles
 *
 * a380x/xp12/afs/xplane_autopilot (int, writable): 1 (default) X-Plane AP/A/THR, 0 the native guidance.
 * Unity-build include of a380x_fcu_adapter.inc (after a380x_fcu_authority.inc).
 */
/* The plugin build sets A380X_XPAP_DEFAULT=1 (build.py); the unit tests keep the native guidance. */
#ifndef A380X_XPAP_DEFAULT
#define A380X_XPAP_DEFAULT 0
#endif
static int xpap_enabled = A380X_XPAP_DEFAULT;
#define A380X_HAVE_XPAP 1
static XPLMDataRef xpap_enable_ref;
static int lever_inputs_valid_for_xpap(void)
{
    const A380XGuidanceInputs *i = &guidance_inputs;
    return i->throttle_levers_valid && i->idle_detent_ratio < i->cl_detent_ratio && i->cl_detent_ratio < i->mct_detent_ratio;
}

enum {
    XPD_SERVOS, XPD_SERVOS2, XPD_FD, XPD_FD2, XPD_AT_ENABLED, XPD_AT_ON, XPD_HDG_STATUS, XPD_NAV_STATUS,
    XPD_GPSS_STATUS, XPD_APP_STATUS, XPD_GS_STATUS, XPD_ALT_STATUS, XPD_VVI_STATUS, XPD_SPEED_STATUS,
    XPD_FPA_STATUS, XPD_TOGA_STATUS, XPD_TOGA_LAT_STATUS, XPD_IS_MACH, XPD_SPD_WINDOW, XPD_FD_PITCH,
    XPD_FD_ROLL, XPD_TRK_FPA, XPD_ALT_DIAL, XPD_VVI_DIAL, XPD_HDG_DIAL, XPD_SPD_DIAL, XPD_ALT_ARMED,
    XPD_FPA_DIAL, XPD_AP_POWER, XPD_HSI_PILOT, XPD_HSI_COPILOT, XPD_COUNT
};
static const char *const xpap_dataref_names[XPD_COUNT] = {
    "sim/cockpit2/autopilot/servos_on", "sim/cockpit2/autopilot/servos2_on",
    "sim/cockpit2/autopilot/flight_director_mode", "sim/cockpit2/autopilot/flight_director2_mode",
    "sim/cockpit2/autopilot/autothrottle_enabled", "sim/cockpit2/autopilot/autothrottle_on",
    "sim/cockpit2/autopilot/heading_status", "sim/cockpit2/autopilot/nav_status",
    "sim/cockpit2/autopilot/gpss_status", "sim/cockpit2/autopilot/approach_status",
    "sim/cockpit2/autopilot/glideslope_status", "sim/cockpit2/autopilot/altitude_hold_status",
    "sim/cockpit2/autopilot/vvi_status", "sim/cockpit2/autopilot/speed_status",
    "sim/cockpit2/autopilot/fpa_status", "sim/cockpit2/autopilot/TOGA_status",
    "sim/cockpit2/autopilot/TOGA_lateral_status", "sim/cockpit2/autopilot/airspeed_is_mach",
    "sim/cockpit2/autopilot/vnav_speed_window_open", "sim/cockpit2/autopilot/flight_director_pitch_deg",
    "sim/cockpit2/autopilot/flight_director_roll_deg", "sim/cockpit2/autopilot/trk_fpa",
    "sim/cockpit2/autopilot/altitude_dial_ft", "sim/cockpit2/autopilot/vvi_dial_fpm",
    "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot", "sim/cockpit2/autopilot/airspeed_dial_kts_mach",
    "sim/cockpit2/autopilot/altitude_hold_armed", "sim/cockpit2/autopilot/fpa",
    "sim/cockpit2/autopilot/autopilot_has_power", "sim/cockpit2/radios/actuators/HSI_source_select_pilot",
    "sim/cockpit2/radios/actuators/HSI_source_select_copilot",
};
enum {
    XPC_SERVOS, XPC_SERVOS2, XPC_SERVOS_OFF, XPC_AT_ARM, XPC_AT_ON, XPC_AT_HARD_OFF, XPC_HEADING, XPC_NAV,
    XPC_APPROACH, XPC_LEVEL_CHANGE, XPC_VS, XPC_FPA, XPC_ALT_HOLD, XPC_KNOTS_MACH, XPC_TRKFPA, XPC_SPD_INTV,
    XPC_TOGA, XPC_COUNT
};
static const char *const xpap_command_names[XPC_COUNT] = {
    "sim/autopilot/servos_toggle", "sim/autopilot/servos2_toggle", "sim/autopilot/servos_off_any",
    "sim/autopilot/autothrottle_arm", "sim/autopilot/autothrottle_on", "sim/autopilot/autothrottle_hard_off",
    "sim/autopilot/heading", "sim/autopilot/NAV", "sim/autopilot/approach", "sim/autopilot/level_change",
    "sim/autopilot/vertical_speed", "sim/autopilot/fpa", "sim/autopilot/altitude_hold",
    "sim/autopilot/knots_mach_toggle", "sim/autopilot/trkfpa", "sim/autopilot/spd_intv",
    "sim/autopilot/take_off_go_around",
};
static struct {
    XPLMDataRef d[XPD_COUNT];
    XPLMCommandRef c[XPC_COUNT];
    int ready, athr_wanted, nav_active, speed_managed, alt_managed, fd_on, was_servos, initialized;
    int last_lat, last_vert, last_athr, takeoff_levers, toga_levers, tried, crew_off;
    double disconnect_seconds, status_at;
    float own_heading;                 /* the FCU's selected heading while NAV steers X-Plane's heading bug */
} xpap;

static int xpap_geti(int k) { return xpap.d[k] ? XPLMGetDatai(xpap.d[k]) : 0; }
static float xpap_getf(int k) { float v = xpap.d[k] ? XPLMGetDataf(xpap.d[k]) : 0.0f; return isfinite(v) ? v : 0.0f; }
static void xpap_seti(int k, int v) { if (xpap.d[k]) XPLMSetDatai(xpap.d[k], v); }
static void xpap_setf(int k, float v) { if (xpap.d[k] && isfinite(v)) XPLMSetDataf(xpap.d[k], v); }
static void xpap_cmd(int k) { if (xpap.c[k]) XPLMCommandOnce(xpap.c[k]); }
/* Is X-Plane's AP / A/THR flying (for the FBW module and the thrust-lever lease)? */
__attribute__((unused)) static int xpap_servos_active(void) { return xpap_enabled && xpap.ready && (xpap_geti(XPD_SERVOS) || xpap_geti(XPD_SERVOS2)); }
__attribute__((unused)) static int xpap_athr_driving(void) { return xpap_enabled && xpap.ready && xpap_geti(XPD_AT_ON) && xpap_geti(XPD_AT_ENABLED) >= 1; }

__attribute__((unused)) static int xpap_get_enable(void *refcon) { (void)refcon; return xpap_enabled; }
__attribute__((unused)) static void xpap_set_enable(void *refcon, int value)
{
    (void)refcon;
    if ((value != 0) == xpap_enabled) return;
    xpap_enabled = value != 0;
    afs_notice("AP/A-THR now %s", xpap_enabled ? "X-Plane autopilot and autothrottle (A330 style)" : "native FBW guidance");
    if (xpap_enabled) {   /* hand over: the native AP and A/THR let go */
        guidance.ap1 = guidance.ap2 = 0; guidance.athr_armed = guidance.athr_active = 0;
    } else if (xpap.ready) {
        if (xpap_geti(XPD_SERVOS) || xpap_geti(XPD_SERVOS2)) xpap_cmd(XPC_SERVOS_OFF);
        if (xpap_geti(XPD_AT_ENABLED) >= 0) xpap_cmd(XPC_AT_HARD_OFF);
    }
}

static void xpap_init(void)
{
    int k;
    for (k = 0; k < XPD_COUNT; ++k) xpap.d[k] = XPLMFindDataRef(xpap_dataref_names[k]);
    for (k = 0; k < XPC_COUNT; ++k) xpap.c[k] = XPLMFindCommand(xpap_command_names[k]);
    xpap.ready = xpap.d[XPD_SERVOS] && xpap.d[XPD_AT_ENABLED] && xpap.c[XPC_SERVOS] && xpap.c[XPC_AT_ARM];
    xpap.initialized = 0;
    if (A380X_XPAP_DEFAULT && !xpap_enable_ref)   /* plugin build only (the adapter tests forbid writable refs) */
        xpap_enable_ref = XPLMRegisterDataAccessor("a380x/xp12/afs/xplane_autopilot", xplmType_Int, 1, xpap_get_enable,
                                                   xpap_set_enable, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                                   NULL, NULL, NULL, NULL);
}
static int xpap_on(void)
{
    if (!xpap_enabled) return 0;
    if (!xpap.ready && !xpap.tried) { xpap.tried = 1; xpap_init(); }
    return xpap.ready;
}
static void xpap_shutdown(void)
{
    if (xpap_enable_ref) XPLMUnregisterDataAccessor(xpap_enable_ref);
    xpap_enable_ref = NULL;
    memset(&xpap, 0, sizeof(xpap));
}

static double xpap_max_lever(void)
{
    double lever = 0; int n;
    for (n = 0; n < 4; ++n) lever = fmax(lever, guidance_inputs.throttle_lever_ratio[n]);
    return lever;
}

/* One FCU command. Returns 1 when handled here. */
static int xpap_command(A380XFcuCommand c, double amount)
{
    float v;
    if (!xpap_enabled || !xpap.ready) return 0;
    switch (c) {
    case A380X_FCU_AP1: case A380X_FCU_AP2: {
        int engaged = c == A380X_FCU_AP1 ? xpap_geti(XPD_SERVOS) : xpap_geti(XPD_SERVOS2);
        xpap_cmd(c == A380X_FCU_AP1 ? XPC_SERVOS : XPC_SERVOS2);
        if (engaged) { xpap.crew_off = 1; afs_log("AP%d off (pushbutton, X-Plane servos)", c == A380X_FCU_AP1 ? 1 : 2); }
        else {
            xpap.fd_on = 1;
            if (c == A380X_FCU_AP1 ? xpap_geti(XPD_SERVOS) : xpap_geti(XPD_SERVOS2))
                afs_log("AP%d engaged (X-Plane autopilot)", c == A380X_FCU_AP1 ? 1 : 2);
            else afs_notice("AP%d: X-Plane autopilot did not engage (AP power %d, servos %d/%d, FD %d)",
                            c == A380X_FCU_AP1 ? 1 : 2, xpap_geti(XPD_AP_POWER), xpap_geti(XPD_SERVOS),
                            xpap_geti(XPD_SERVOS2), xpap_geti(XPD_FD));
        }
        return 1;
    }
    case A380X_FCU_AP_DISCONNECT:
        if (xpap_geti(XPD_SERVOS) || xpap_geti(XPD_SERVOS2)) { xpap_cmd(XPC_SERVOS_OFF); afs_log("AP disconnected (X-Plane servos off)"); }
        xpap.crew_off = 1;               /* instinctive disconnect: short AP OFF warning */
        return 1;
    case A380X_FCU_FD1: case A380X_FCU_FD2:
        xpap.fd_on = !xpap.fd_on;
        if (!xpap_geti(XPD_SERVOS) && !xpap_geti(XPD_SERVOS2)) xpap_seti(XPD_FD, xpap.fd_on ? 1 : 0);
        return 1;
    case A380X_FCU_ATHR:
        if (xpap_geti(XPD_AT_ENABLED) < 0 && !xpap.athr_wanted) {
            xpap_cmd(XPC_AT_ARM); xpap.athr_wanted = 1;
            afs_log("A/THR pushbutton: armed (X-Plane autothrottle)");
        } else {
            xpap_cmd(XPC_AT_HARD_OFF); xpap.athr_wanted = 0;
            afs_log("A/THR pushbutton: off (X-Plane autothrottle)");
        }
        return 1;
    case A380X_FCU_ATHR_DISCONNECT:
        if (xpap_geti(XPD_AT_ENABLED) >= 0) xpap_cmd(XPC_AT_HARD_OFF);
        xpap.athr_wanted = 0;
        return 1;
    case A380X_FCU_SPD_ROTATE:
        v = xpap_getf(XPD_SPD_DIAL);
        if (xpap_geti(XPD_IS_MACH)) v = (float)fmin(.99, fmax(.10, v + amount * .01));
        else v = (float)fmin(399, fmax(100, v + amount));
        xpap_setf(XPD_SPD_DIAL, v);
        if (xpap.speed_managed) xpap.speed_managed = 0;   /* turning previews a selected speed, as the A380 */
        return 1;
    case A380X_FCU_SPD_PULL:
        xpap.speed_managed = 0;
        if (xpap.d[XPD_SPD_WINDOW] && !xpap_geti(XPD_SPD_WINDOW)) xpap_cmd(XPC_SPD_INTV);
        if (xpap_geti(XPD_AT_ENABLED) == 0) xpap_cmd(XPC_AT_ON);
        return 1;
    case A380X_FCU_SPD_PUSH:
        if (guidance_inputs.managed_speed_valid && isfinite(guidance_inputs.managed_speed_knots)) xpap.speed_managed = 1;
        else afs_notice("Managed speed not available (no FMS speed): selected speed kept");
        return 1;
    case A380X_FCU_SPD_MACH: xpap_cmd(XPC_KNOTS_MACH); return 1;
    case A380X_FCU_HDG_ROTATE:
        if (xpap.nav_active) { xpap.own_heading = normalized_heading(xpap.own_heading + (float)amount); return 1; }
        xpap_setf(XPD_HDG_DIAL, normalized_heading(xpap_getf(XPD_HDG_DIAL) + (float)amount));
        return 1;
    case A380X_FCU_HDG_PULL:
        if (xpap.nav_active) { xpap_setf(XPD_HDG_DIAL, xpap.own_heading); xpap.nav_active = 0; }
        if (xpap_geti(XPD_HDG_STATUS) != 2) xpap_cmd(XPC_HEADING);
        return 1;
    case A380X_FCU_HDG_PUSH:
        if (!guidance_inputs.managed_route_valid) { afs_notice("NAV not available: no active FMS route"); return 1; }
        if (!xpap.nav_active) xpap.own_heading = xpap_getf(XPD_HDG_DIAL);
        xpap.nav_active = 1;
        if (xpap_geti(XPD_HDG_STATUS) != 2) xpap_cmd(XPC_HEADING);
        return 1;
    case A380X_FCU_ALT_ROTATE: {
        float step = guidance.altitude_increment_1000 ? 1000.0f : 100.0f;
        v = xpap_getf(XPD_ALT_DIAL) + (float)amount * step;
        v = (float)(round(v / step) * step);
        xpap_setf(XPD_ALT_DIAL, (float)fmin(45000, fmax(100, v)));
        return 1;
    }
    case A380X_FCU_ALT_PULL: case A380X_FCU_ALT_PUSH:
        xpap.alt_managed = c == A380X_FCU_ALT_PUSH && guidance_inputs.managed_vertical_valid;
        if (c == A380X_FCU_ALT_PUSH && xpap.alt_managed && guidance_inputs.managed_speed_valid) xpap.speed_managed = 1;
        if (xpap_geti(XPD_SPEED_STATUS) != 2) xpap_cmd(XPC_LEVEL_CHANGE);
        return 1;
    case A380X_FCU_ALT_HOLD:
        xpap.alt_managed = 0;
        if (xpap_geti(XPD_ALT_STATUS) != 2) xpap_cmd(XPC_ALT_HOLD);
        return 1;
    case A380X_FCU_VS_ROTATE:
        if (xpap_geti(XPD_TRK_FPA)) xpap_setf(XPD_FPA_DIAL, (float)fmin(9.9, fmax(-9.9, xpap_getf(XPD_FPA_DIAL) + amount * .1)));
        else xpap_setf(XPD_VVI_DIAL, (float)fmin(6000, fmax(-6000, xpap_getf(XPD_VVI_DIAL) + amount * 100)));
        return 1;
    case A380X_FCU_VS_PULL:
        xpap.alt_managed = 0;
        if (xpap_geti(XPD_TRK_FPA)) { if (xpap_geti(XPD_FPA_STATUS) != 2) xpap_cmd(XPC_FPA); }
        else if (xpap_geti(XPD_VVI_STATUS) != 2) {
            float target = xpap_getf(XPD_VVI_DIAL);
            xpap_cmd(XPC_VS);                         /* X-Plane starts V/S at the current rate ... */
            if (fabsf(target) >= 100) xpap_setf(XPD_VVI_DIAL, target);   /* ... the preset target wins */
        }
        return 1;
    case A380X_FCU_VS_PUSH:          /* level off */
        xpap.alt_managed = 0;
        if (xpap_geti(XPD_TRK_FPA)) { xpap_setf(XPD_FPA_DIAL, 0); if (xpap_geti(XPD_FPA_STATUS) != 2) xpap_cmd(XPC_FPA); }
        else { if (xpap_geti(XPD_VVI_STATUS) != 2) xpap_cmd(XPC_VS); xpap_setf(XPD_VVI_DIAL, 0); }
        return 1;
    case A380X_FCU_LOC: case A380X_FCU_APPR: {
        int st = c == A380X_FCU_LOC ? xpap_geti(XPD_NAV_STATUS) : xpap_geti(XPD_APP_STATUS);
        xpap_seti(XPD_HSI_PILOT, 0); xpap_seti(XPD_HSI_COPILOT, 0);    /* NAV1 (ILS) as the AP source */
        if (st == 0 || c == A380X_FCU_APPR || st == 1) xpap_cmd(c == A380X_FCU_LOC ? XPC_NAV : XPC_APPROACH);
        if (st == 0) xpap.nav_active = 0;
        return 1;
    }
    case A380X_FCU_TRK_FPA: xpap_cmd(XPC_TRKFPA); return 0;   /* also flips the FCU display state */
    default: return 0;   /* LS, METRIC, TRUE/MAG, ALT 100/1000: display state of the native FCU */
    }
}

/* Every frame instead of the native guidance: drive NAV / managed speed / A/THR activation, and publish
 * X-Plane's AP state as the guidance state the displays read. */
static void xpap_update(float dt)
{
    A380XGuidanceState *g = &guidance;
    int servos, servos2, at_en, at_on, mach, lever_valid = lever_inputs_valid_for_xpap();
    double lever = lever_valid ? xpap_max_lever() : 0, pitch_cmd, roll_cmd;
    if (!xpap.ready) return;
    /* never the native actuators */
    if (fcu.own_pitch || fcu.own_roll) release_ap_controls();
    if (fcu.own_throttle) release_thrust_control();
    if (!xpap.initialized && guidance_inputs.powered) {
        xpap.initialized = 1; xpap.fd_on = 1;
        if (!xpap_geti(XPD_FD)) xpap_seti(XPD_FD, 1);
        if (!guidance_inputs.on_ground) xpap.athr_wanted = xpap_geti(XPD_AT_ENABLED) >= 0;
    }
    servos = xpap_geti(XPD_SERVOS); servos2 = xpap_geti(XPD_SERVOS2);
    at_en = xpap_geti(XPD_AT_ENABLED); at_on = xpap_geti(XPD_AT_ON); mach = xpap_geti(XPD_IS_MACH);
    /* ---- A/THR, A330 style: armed with the pushbutton; active in flight with the levers in the A/THR range */
    if (guidance_inputs.athr_inhibited && at_en >= 0) { xpap_cmd(XPC_AT_HARD_OFF); xpap.athr_wanted = 0; }
    if (xpap.athr_wanted && at_en >= 0 && lever_valid) {
        int idle = lever <= guidance_inputs.idle_detent_ratio;
        int above_cl = lever >= guidance_inputs.mct_detent_ratio - guidance_inputs.detent_tolerance_ratio;
        if (!guidance_inputs.on_ground && idle && at_en >= 1) {          /* levers to IDLE: A/THR off */
            xpap_cmd(XPC_AT_HARD_OFF); xpap.athr_wanted = 0; afs_notice("A/THR off: thrust levers at IDLE");
        } else if (guidance_inputs.on_ground || above_cl || guidance_inputs.paused) {
            if (at_en >= 1) xpap_seti(XPD_AT_ENABLED, 0);                 /* armed, not active (manual thrust) */
        } else if (at_en == 0 && guidance_inputs.powered) {
            xpap_cmd(XPC_AT_ON);                                         /* active: SPEED / MACH */
        }
    }
    /* Takeoff: levers to FLX/MCT or TOGA with the FD on arm the A/THR and give the FD takeoff guidance (SRS);
     * in flight a TOGA rising edge with flaps out is a go-around. */
    if (lever_valid) {
        int at_takeoff = lever >= guidance_inputs.mct_detent_ratio - guidance_inputs.detent_tolerance_ratio;
        int at_toga = lever >= guidance_inputs.toga_detent_ratio - guidance_inputs.detent_tolerance_ratio;
        if (guidance_inputs.on_ground && at_takeoff && !xpap.takeoff_levers && xpap.fd_on && guidance_inputs.powered) {
            if (at_en < 0) { xpap_cmd(XPC_AT_ARM); xpap.athr_wanted = 1; }
            if (xpap_geti(XPD_TOGA_STATUS) != 2) xpap_cmd(XPC_TOGA);
            afs_log("Takeoff thrust: A/THR armed, FD takeoff guidance (X-Plane TO/GA)");
        } else if (!guidance_inputs.on_ground && at_toga && !xpap.toga_levers && guidance_inputs.flap_handle_index >= 1) {
            xpap_cmd(XPC_TOGA); afs_log("Go-around (X-Plane TO/GA)");
        }
        xpap.takeoff_levers = at_takeoff; xpap.toga_levers = at_toga;
    }
    at_en = xpap_geti(XPD_AT_ENABLED);
    if (at_en < 0) xpap.athr_wanted = 0;
    at_en = xpap_geti(XPD_AT_ENABLED); at_on = xpap_geti(XPD_AT_ON);
    /* ---- managed speed / NAV from the native FMS */
    if (xpap.speed_managed) {
        if (!guidance_inputs.managed_speed_valid || !isfinite(guidance_inputs.managed_speed_knots)) xpap.speed_managed = 0;
        else if (mach && guidance_inputs.managed_speed_is_mach && isfinite(guidance_inputs.managed_mach))
            xpap_setf(XPD_SPD_DIAL, (float)guidance_inputs.managed_mach);
        else if (!mach) xpap_setf(XPD_SPD_DIAL, (float)fmin(399, fmax(100, guidance_inputs.managed_speed_knots)));
    }
    if (xpap.nav_active) {
        if (!guidance_inputs.managed_route_valid || !isfinite(guidance_inputs.managed_heading_deg)) {
            xpap.nav_active = 0; xpap_setf(XPD_HDG_DIAL, xpap.own_heading);
            afs_notice("NAV lost (no FMS route): HDG");
        } else {
            xpap_setf(XPD_HDG_DIAL, normalized_heading((float)guidance_inputs.managed_heading_deg));
            if (xpap_geti(XPD_HDG_STATUS) != 2 && (servos || servos2 || xpap.fd_on) && !xpap_geti(XPD_NAV_STATUS))
                xpap_cmd(XPC_HEADING);
        }
    }
    if (xpap_geti(XPD_NAV_STATUS) == 2 || xpap_geti(XPD_APP_STATUS) == 2) xpap.nav_active = 0;   /* LOC took over */
    /* ---- publish as the guidance state (PFD FMA, FD, FCU windows, FWS) */
    g->ap1 = servos != 0; g->ap2 = servos2 != 0 && !servos;
    g->fd1 = g->fd2 = xpap.fd_on || servos || servos2;
    g->athr_armed = at_en >= 0 && xpap.athr_wanted;
    g->athr_active = at_on && at_en >= 1;
    g->mach_mode = mach; g->trk_fpa_mode = xpap_geti(XPD_TRK_FPA);
    if (mach) {
        g->selected_mach = xpap_getf(XPD_SPD_DIAL);
        g->speed_target_knots = guidance_inputs.mach > .05 ? g->selected_mach * guidance_inputs.ias_knots / guidance_inputs.mach : 250;
    } else { g->selected_speed_knots = xpap_getf(XPD_SPD_DIAL); g->speed_target_knots = g->selected_speed_knots; }
    g->selected_heading_deg = xpap.nav_active ? xpap.own_heading : xpap_getf(XPD_HDG_DIAL);
    g->selected_altitude_ft = xpap_getf(XPD_ALT_DIAL); g->altitude_target_ft = g->selected_altitude_ft;
    g->selected_vs_fpm = xpap_getf(XPD_VVI_DIAL); g->selected_fpa_deg = xpap_getf(XPD_FPA_DIAL);
    g->speed_managed = xpap.speed_managed; g->heading_managed = xpap.nav_active; g->altitude_managed = xpap.alt_managed;
    g->speed_dashes = xpap.speed_managed; g->heading_dashes = xpap.nav_active;
    /* lateral */
    if (xpap_geti(XPD_TOGA_LAT_STATUS) == 2) g->lateral_mode = A380X_C1_GA_TRK;
    else if (xpap_geti(XPD_NAV_STATUS) == 2 || xpap_geti(XPD_APP_STATUS) == 2) g->lateral_mode = A380X_C1_LOC;
    else if (xpap.nav_active || xpap_geti(XPD_GPSS_STATUS) == 2) g->lateral_mode = A380X_C1_NAV;
    else if (xpap_geti(XPD_HDG_STATUS) == 2) g->lateral_mode = g->trk_fpa_mode ? A380X_C1_TRACK : A380X_C1_HDG;
    else g->lateral_mode = A380X_C1_NONE;
    g->loc_armed = xpap_geti(XPD_NAV_STATUS) == 1 || xpap_geti(XPD_APP_STATUS) == 1;
    g->approach_selected = xpap_geti(XPD_APP_STATUS) != 0;
    /* vertical */
    {
        double alt = guidance_inputs.altitude_ft;
        int climb = g->selected_altitude_ft > alt;
        if (xpap_geti(XPD_GS_STATUS) == 2) g->vertical_mode = A380X_B1_GS;
        else if (xpap_geti(XPD_TOGA_STATUS) == 2) g->vertical_mode = A380X_B1_SRS;
        else if (xpap_geti(XPD_ALT_STATUS) == 2)
            g->vertical_mode = fabs(alt - g->selected_altitude_ft) > 60 && fabs(guidance_inputs.vertical_speed_fpm) > 300 ? A380X_B1_ALT_STAR : A380X_B1_ALT;
        else if (xpap_geti(XPD_SPEED_STATUS) == 2)
            g->vertical_mode = xpap.alt_managed ? (climb ? A380X_B1_CLB : A380X_B1_DES) : (climb ? A380X_B1_OP_CLB : A380X_B1_OP_DES);
        else if (xpap_geti(XPD_FPA_STATUS) == 2) g->vertical_mode = A380X_B1_FPA;
        else if (xpap_geti(XPD_VVI_STATUS) == 2) g->vertical_mode = A380X_B1_VS;
        else g->vertical_mode = A380X_B1_NONE;
        g->gs_armed = xpap_geti(XPD_GS_STATUS) == 1;
        g->altitude_armed = xpap_geti(XPD_ALT_ARMED) != 0;
        g->vs_dashes = g->vertical_mode != A380X_B1_VS && g->vertical_mode != A380X_B1_FPA;
        g->vertical_guidance_selected = g->vertical_mode != A380X_B1_NONE;
        /* thrust mode (FMA column 1) */
        if (g->athr_active) {
            if (g->vertical_mode == A380X_B1_OP_CLB || g->vertical_mode == A380X_B1_CLB) g->thrust_mode = A380X_A1A2_THR_CLB;
            else if (g->vertical_mode == A380X_B1_OP_DES || g->vertical_mode == A380X_B1_DES) g->thrust_mode = A380X_A1A2_THR_IDLE;
            else g->thrust_mode = mach ? A380X_A1A2_MACH : A380X_A1A2_SPEED;
        } else g->thrust_mode = A380X_A1A2_NONE;
    }
    /* FD orders: X-Plane's commanded attitude minus the aircraft attitude */
    pitch_cmd = xpap_getf(XPD_FD_PITCH); roll_cmd = xpap_getf(XPD_FD_ROLL);
    g->guidance_valid = (g->fd1 || g->ap1 || g->ap2) && (g->lateral_mode != A380X_C1_NONE || g->vertical_mode != A380X_B1_NONE);
    g->pitch_command_deg = pitch_cmd; g->roll_command_deg = roll_cmd;
    g->fd_pitch_error_deg = g->guidance_valid ? pitch_cmd - guidance_inputs.pitch_deg : 0;
    g->fd_roll_error_deg = g->guidance_valid ? roll_cmd - guidance_inputs.roll_deg : 0;
    g->pitch_servo = g->roll_servo = 0; g->throttle_command = guidance_inputs.throttle_ratio;
    /* AP OFF warning after an AP loss that the crew did not command */
    if (xpap.was_servos && !servos && !servos2) {
        xpap.disconnect_seconds = xpap.crew_off ? 1.5 : 5;
        if (!xpap.crew_off) afs_notice("AP disconnected by X-Plane's autopilot (not commanded from the FCU)");
    }
    if (!xpap.was_servos && !servos && !servos2) xpap.crew_off = 0;
    xpap.was_servos = servos || servos2;
    if (xpap.disconnect_seconds > 0) xpap.disconnect_seconds = fmax(0, xpap.disconnect_seconds - dt);
    g->disconnect_warning = xpap.disconnect_seconds > 0;
    if (servos || servos2) { xpap.disconnect_seconds = 0; xpap.crew_off = 0; }
    /* log the modes when they change, and a status line every 10 s in flight */
    if ((int)g->lateral_mode != xpap.last_lat || (int)g->vertical_mode != xpap.last_vert || g->athr_active != xpap.last_athr) {
        afs_log("X-Plane AFS modes: AP %d/%d FD %d A/THR armed %d active %d (XP at_enabled %d) lat %d vert %d thr %d",
                g->ap1, g->ap2, g->fd1, g->athr_armed, g->athr_active, at_en, g->lateral_mode, g->vertical_mode, g->thrust_mode);
        xpap.last_lat = (int)g->lateral_mode; xpap.last_vert = (int)g->vertical_mode; xpap.last_athr = g->athr_active;
    }
    if (XPLMGetElapsedTime() >= xpap.status_at && !guidance_inputs.on_ground && (servos || servos2 || at_en >= 0)) {
        xpap.status_at = XPLMGetElapsedTime() + 10.0;
        afs_log("X-Plane AFS status: AP %d/%d A/THR %d/%d (at_enabled %d) | IAS %.0f target %.0f | ALT %.0f target %.0f | V/S %.0f | "
                "HDG %.0f target %.0f | levers %.3f | lease %d own_thr %d",
                servos, servos2, g->athr_armed, at_on, at_en, guidance_inputs.ias_knots, g->speed_target_knots,
                guidance_inputs.altitude_ft, g->selected_altitude_ft, guidance_inputs.vertical_speed_fpm,
                guidance_inputs.heading_deg, xpap_getf(XPD_HDG_DIAL), lever, fcu_thrust_link.lease, fcu.own_throttle);
    }
}
