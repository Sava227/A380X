/* SPDX-License-Identifier: GPL-3.0-or-later
 * Hardware stock AP/A/THR commands enter the aircraft's custom command path.
 * Stock servo feedback below is used only to detect actuator contention, never
 * as a guidance mode or target. A direct foreign takeover releases our leases.
 */
enum { FCUA_AP1_ON, FCUA_AP1_TOGGLE, FCUA_AP2_ON, FCUA_AP2_TOGGLE,
       FCUA_AP_OFF, FCUA_ATHR_ON, FCUA_ATHR_TOGGLE, FCUA_ATHR_OFF, FCUA_UNSUPPORTED };
typedef struct { const char *name; int action; XPLMCommandRef handle; } FcuAuthorityCommand;
static FcuAuthorityCommand fcu_authority_commands[]={
    {"sim/autopilot/servos_on",FCUA_AP1_ON,NULL},
    {"sim/autopilot/servos_toggle",FCUA_AP1_TOGGLE,NULL},
    {"sim/autopilot/servos2_on",FCUA_AP2_ON,NULL},
    {"sim/autopilot/servos2_toggle",FCUA_AP2_TOGGLE,NULL},
    {"sim/autopilot/servos3_on",FCUA_UNSUPPORTED,NULL},
    {"sim/autopilot/servos3_toggle",FCUA_UNSUPPORTED,NULL},
    /* servos_off_any already has the adapter's native_disconnect_handler. */
    {"sim/autopilot/servos_fdir_off",FCUA_AP_OFF,NULL},
    {"sim/autopilot/servos_fdir2_off",FCUA_AP_OFF,NULL},
    {"sim/autopilot/servos_fdir3_off",FCUA_AP_OFF,NULL},
    {"sim/autopilot/servos_fdir_yawd_off",FCUA_AP_OFF,NULL},
    {"sim/autopilot/servos_fdir_yawd_trim_off",FCUA_AP_OFF,NULL},
    {"sim/autopilot/servos_yawd_off_any",FCUA_AP_OFF,NULL},
    {"sim/autopilot/servos_yawd_trim_off_any",FCUA_AP_OFF,NULL},
    {"sim/autopilot/ap_disc_trim_interrupt",FCUA_AP_OFF,NULL},
    {"sim/autopilot/autothrottle_on",FCUA_ATHR_ON,NULL},
    {"sim/autopilot/autothrottle_arm",FCUA_ATHR_ON,NULL},
    {"sim/autopilot/autothrottle_toggle",FCUA_ATHR_TOGGLE,NULL},
    {"sim/autopilot/autothrottle_n1epr",FCUA_ATHR_ON,NULL},
    {"sim/autopilot/autothrottle_n1epr_toggle",FCUA_ATHR_TOGGLE,NULL},
    {"sim/autopilot/autothrottle_off",FCUA_ATHR_OFF,NULL},
    {"sim/autopilot/autothrottle_hard_off",FCUA_ATHR_OFF,NULL},
    {"sim/autopilot/autothrottle_retard",FCUA_UNSUPPORTED,NULL},
};
typedef struct {
    XPLMCommandRef ap1,ap2,ap_off,athr,athr_off,native_athr_off;
    XPLMDataRef stock_servos[3],stock_athr;
    int internal,ap_warning,athr_warning;
} FcuAuthority;
static FcuAuthority fcu_authority;

static int fcu_authority_handler(XPLMCommandRef command,XPLMCommandPhase phase,void *refcon)
{
    FcuAuthorityCommand *item=(FcuAuthorityCommand *)refcon;
    int pass=item->action==FCUA_AP_OFF || item->action==FCUA_ATHR_OFF;
    (void)command;
    if (!plugin_enabled || fcu_authority.internal) return 1;
    if (phase==xplm_CommandBegin) {
        switch(item->action) {
        case FCUA_AP1_ON: if (!guidance.ap1) XPLMCommandOnce(fcu_authority.ap1); break;
        case FCUA_AP2_ON: if (!guidance.ap2) XPLMCommandOnce(fcu_authority.ap2); break;
        case FCUA_AP1_TOGGLE: XPLMCommandOnce(fcu_authority.ap1); break;
        case FCUA_AP2_TOGGLE: XPLMCommandOnce(fcu_authority.ap2); break;
        case FCUA_AP_OFF:
            if (guidance.ap1 || guidance.ap2 || fcu.own_pitch || fcu.own_roll || fcu.own_yaw)
                XPLMCommandOnce(fcu_authority.ap_off);
            break;
        case FCUA_ATHR_ON: if (!guidance.athr_armed) XPLMCommandOnce(fcu_authority.athr); break;
        case FCUA_ATHR_TOGGLE: XPLMCommandOnce(fcu_authority.athr); break;
        case FCUA_ATHR_OFF:
            if (guidance.athr_armed || guidance.athr_active || fcu.own_throttle)
                XPLMCommandOnce(fcu_authority.athr_off);
            break;
        default:
            XPLMDebugString("A380X FCU: unsupported stock AP3/retard command suppressed; use aircraft FCU controls.\n");
            break;
        }
    }
    /* Off/disconnect commands still reach X-Plane, so a foreign native
     * controller can be safely disconnected too. Engagement is consumed. */
    return pass;
}

static void fcu_authority_shutdown(void)
{
    int i;
    for (i=0;i<ARRAY_LEN(fcu_authority_commands);++i) if(fcu_authority_commands[i].handle) {
        XPLMUnregisterCommandHandler(fcu_authority_commands[i].handle,fcu_authority_handler,1,&fcu_authority_commands[i]);
        fcu_authority_commands[i].handle=NULL;
    }
    memset(&fcu_authority,0,sizeof(fcu_authority));
}

static int fcu_authority_init(void)
{
    int i;
    static const char *const servo_names[]={"sim/cockpit2/autopilot/servos_on",
        "sim/cockpit2/autopilot/servos2_on","sim/cockpit2/autopilot/servos3_on"};
    fcu_authority_shutdown();
    fcu_authority.ap1=XPLMFindCommand("a380x/xp12/fcu/ap1_toggle");
    fcu_authority.ap2=XPLMFindCommand("a380x/xp12/fcu/ap2_toggle");
    fcu_authority.ap_off=XPLMFindCommand("a380x/xp12/fcu/ap_disconnect");
    fcu_authority.athr=XPLMFindCommand("a380x/xp12/fcu/athr_toggle");
    fcu_authority.athr_off=XPLMFindCommand("a380x/xp12/fcu/athr_disconnect");
    fcu_authority.native_athr_off=XPLMFindCommand("sim/autopilot/autothrottle_hard_off");
    fcu_authority.stock_athr=XPLMFindDataRef("sim/cockpit2/autopilot/autothrottle_enabled");
    for(i=0;i<3;++i) fcu_authority.stock_servos[i]=XPLMFindDataRef(servo_names[i]);
    if (!fcu_authority.ap1 || !fcu_authority.ap2 || !fcu_authority.ap_off
        || !fcu_authority.athr || !fcu_authority.athr_off || !fcu_authority.native_athr_off
        || !fcu_authority.stock_athr) goto fail;
    for(i=0;i<3;++i) if(!fcu_authority.stock_servos[i]) goto fail;
    for(i=0;i<ARRAY_LEN(fcu_authority_commands);++i) {
        FcuAuthorityCommand *item=&fcu_authority_commands[i];
        item->handle=XPLMFindCommand(item->name);
        if(!item->handle) goto fail;
        XPLMRegisterCommandHandler(item->handle,fcu_authority_handler,1,item);
    }
    return 1;
fail:
    XPLMDebugString("A380X FCU: native AP/A/THR authority boundary unavailable; FCU initialization failed safely.\n");
    fcu_authority_shutdown(); return 0;
}

static void fcu_authority_prepare_athr(void)
{
    /* An explicit custom A/THR ON request takes over from native A/THR.
     * Automatic detent arming does not silently take over another controller. */
    if (!fcu_authority.native_athr_off) return;
    ++fcu_authority.internal;
    XPLMCommandOnce(fcu_authority.native_athr_off);
    --fcu_authority.internal;
}

static void fcu_authority_guard(void)
{
    int i,native_ap=0,native_athr;
    if(!plugin_enabled) return;
    for(i=0;i<3;++i) if(fcu_authority.stock_servos[i]
        && XPLMGetDatai(fcu_authority.stock_servos[i])) native_ap=1;
    native_athr=fcu_authority.stock_athr && XPLMGetDatai(fcu_authority.stock_athr)>0;
    if(native_ap && (guidance.ap1 || guidance.ap2 || fcu.own_pitch || fcu.own_roll || fcu.own_yaw)) {
        XPLMCommandOnce(fcu_authority.ap_off);
        release_ap_controls();
        if(!fcu_authority.ap_warning)
            XPLMDebugString("A380X FCU: foreign native AP servo activation detected; custom AP controls released.\n");
        fcu_authority.ap_warning=1;
    } else if(!native_ap) fcu_authority.ap_warning=0;
    if(native_athr && (guidance.athr_armed || guidance.athr_active || fcu.own_throttle)) {
        XPLMCommandOnce(fcu_authority.athr_off);
        release_thrust_control();
        if(!fcu_authority.athr_warning)
            XPLMDebugString("A380X FCU: foreign native A/THR activation detected; custom thrust controls released.\n");
        fcu_authority.athr_warning=1;
    } else if(!native_athr) fcu_authority.athr_warning=0;
}
