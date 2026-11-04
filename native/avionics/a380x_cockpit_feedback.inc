/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Cockpit feedback: press travel, selector / switch positions and lamps for the FBW controls the port
 * left static (contract: tools/cockpit_feedback_contract.py, generated a380x_cockpit_feedback_table.h;
 * geometry: tools/cockpit_feedback.py).
 *
 *   press    the dataref travels in on command Begin and back on End (A380XPress). The commands of the
 *            ECAM control panel and the KCCUs belong to other modules: this module only adds a handler in
 *            front of theirs that returns 1, so their action still runs. SURV commands are created here.
 *   selector SWITCHING ATT HDG / AIR DATA / FMS, int 0..2 (FBW enum 0 CAPT / NORM / F/O), _left / _right.
 *   switch   glareshield side CONSOLE & FLOOR LT (0 OFF, 1 DIM, 2 BRT) and OIT SIDE (0 FLT OPS, 1 NSS
 *            AVNCS), state only for now, _up / _down.
 *   lamps    ECAM page keys (green bar on the displayed SD page), CLR (an ECAM warning to clear), SURV
 *            SYS / ABV / BLW bars, TA ONLY, G/S MODE OFF.
 *   toggle   guards, circuit breakers, FIRE pushbuttons, furniture: float 0..1 eased toward 0 or 1 over the
 *            row's travel time, command <dataref>_toggle. The ENG FIRE pushbuttons publish
 *            a380x/xp12/engine/fire_pushbutton[0..3] (released) for the ECAM.
 *   rotary   knobs without logic of their own and the seat / armrest adjusters: float 0..1, commands _up /
 *            _down (or hooked ones) step it, eased at 1/travel per second; `wrap` turns endlessly.
 *   wipers   WIPER selector 0 OFF / 1 SLOW / 2 FAST sweeps the blade (0 parked .. 1 and back); OFF parks it.
 *
 * Every dataref is registered in XPluginStart, before the aircraft OBJs bind them.
 * Included after the SD, ADIRS and FMS adapters (reads sda, calls adirs_set_source).
 */
#include "a380x_control_motion.h"
#include "a380x_cockpit_feedback_table.h"

#define CFB_PRESS_COUNT ((int)ARRAY_LEN(cfb_press_rows))
#define CFB_POSITION_COUNT ((int)ARRAY_LEN(cfb_position_rows))
#define CFB_SURV_COUNT ((int)ARRAY_LEN(cfb_surv_buttons))
#define CFB_ECAM_LAMPS ((int)ARRAY_LEN(cfb_ecam_lamps))
#define CFB_TOGGLE_COUNT ((int)ARRAY_LEN(cfb_toggle_rows))
#define CFB_ROTARY_COUNT ((int)ARRAY_LEN(cfb_rotary_rows))
#define CFB_WIPERS ((int)ARRAY_LEN(cfb_wiper_sweeps))
#define CFB_FIRE_PBS ((int)ARRAY_LEN(cfb_fire_pushbuttons))

enum { CFB_SURV_XPDR_TCAS_SYS, CFB_SURV_WXR_TAWS_SYS, CFB_SURV_TCAS_ALERT_LEVEL, CFB_SURV_TCAS_ALT_SELECT,
       CFB_SURV_GS_MODE_OFF, CFB_SURV_REFS };
static const char *const cfb_surv_ref_names[CFB_SURV_REFS] = {
    "a380x/xp12/surv/xpdr_tcas_sys", "a380x/xp12/surv/wxr_taws_sys", "a380x/xp12/surv/tcas_alert_level",
    "a380x/xp12/surv/tcas_alt_select", "a380x/xp12/surv/gs_mode_off"};

typedef struct {
    A380XPress press;
    float value;
    XPLMDataRef ref;
    XPLMCommandRef command;
    int registered;
} CfbPress;

typedef struct {
    int position;
    XPLMDataRef ref;
    XPLMCommandRef dec, inc;     /* _left / _down, _right / _up */
} CfbPosition;

typedef struct {
    float value, target;
    XPLMDataRef ref;
    XPLMCommandRef command;
} CfbToggle;

typedef struct {
    float value, target;
    XPLMDataRef ref;
    XPLMCommandRef up, down;
} CfbRotary;

typedef struct {
    int selector_index, direction;
    float sweep;
    XPLMDataRef ref;
} CfbWiper;

static struct {
    int initialized, fms_seen, fire_toggle[CFB_FIRE_PBS];
    float now;
    CfbPress press[CFB_PRESS_COUNT];
    CfbPosition position[CFB_POSITION_COUNT];
    CfbToggle toggle[CFB_TOGGLE_COUNT];
    CfbRotary rotary[CFB_ROTARY_COUNT];
    CfbWiper wiper[CFB_WIPERS];
    XPLMDataRef fire_ref;
    XPLMCommandRef surv_commands[CFB_SURV_COUNT];
    XPLMDataRef surv_lamp_refs[CFB_SURV_COUNT], ecam_lamp_refs[CFB_ECAM_LAMPS];
    XPLMDataRef surv_refs[CFB_SURV_REFS], fms_knob;
} cfb;

static int cfb_index(const char *suffix)
{
    for (int i = 0; i < CFB_POSITION_COUNT; ++i) {
        size_t n = strlen(cfb_position_rows[i].dataref), m = strlen(suffix);
        if (n >= m && !strcmp(cfb_position_rows[i].dataref + n - m, suffix)) return i;
    }
    return -1;
}

static int cfb_lamp_power(void)
{
    return plugin_enabled && a380x_power_any(A380X_ELN_BIT(A380X_ELN_DC_ESS) | A380X_ELN_BIT(A380X_ELN_DC1) |
                                             A380X_ELN_BIT(A380X_ELN_DC2), 1);
}

/* ------------------------------------------------------------------ press travel */

static float cfb_press_get(void *refcon) { return ((CfbPress *)refcon)->value; }

static int cfb_press_command(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    (void)command;
    CfbPress *p = refcon;
    if (!plugin_enabled) return 1;
    if (phase == xplm_CommandBegin) a380x_press_begin(&p->press, cfb.now, 1.0f);
    else if (phase == xplm_CommandEnd) a380x_press_end(&p->press);
    return 1;   /* the owning module's handler runs as before */
}

/* ------------------------------------------------------------------ selectors and switches */

static void cfb_apply_position(int index)
{
    int p = cfb.position[index].position;
    if (!plugin_enabled) return;
    if (index == cfb_index("switching/att_hdg")) {          /* CAPT ON 3 / NORM / F/O ON 3 */
        adirs_set_source(0, 1, p == 0);
        adirs_set_source(1, 1, p == 2);
    } else if (index == cfb_index("switching/air_data")) {
        adirs_set_source(0, 0, p == 0);
        adirs_set_source(1, 0, p == 2);
    } else if (index == cfb_index("switching/fms")) {       /* 0 BOTH ON 2, 1 NORM, 2 BOTH ON 1 */
        if (!cfb.fms_knob) cfb.fms_knob = XPLMFindDataRef("a380x/xp12/fms/switching_knob");
        if (cfb.fms_knob) { XPLMSetDatai(cfb.fms_knob, p); cfb.fms_seen = p; }
    }
}

static void cfb_set_position(int index, int value)
{
    const CfbPositionRow *row = &cfb_position_rows[index];
    if (value < 0) value = 0;
    if (value > row->positions - 1) value = row->positions - 1;
    if (value == cfb.position[index].position) return;
    cfb.position[index].position = value;
    cfb_apply_position(index);
}

static int cfb_position_geti(void *refcon) { return cfb.position[(int)(intptr_t)refcon].position; }
static float cfb_position_getf(void *refcon) { return (float)cfb.position[(int)(intptr_t)refcon].position; }
static void cfb_position_seti(void *refcon, int value) { if (plugin_enabled) cfb_set_position((int)(intptr_t)refcon, value); }
static void cfb_position_setf(void *refcon, float value)
{
    if (plugin_enabled && isfinite(value)) cfb_set_position((int)(intptr_t)refcon, (int)lroundf(value));
}

static int cfb_position_command(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    int index = (int)(intptr_t)refcon;
    if (!plugin_enabled || phase != xplm_CommandBegin) return 0;
    cfb_set_position(index, cfb.position[index].position + (command == cfb.position[index].inc ? 1 : -1));
    return 0;
}

/* ------------------------------------------------------------------ SURV panel */

static int cfb_surv_get(int ref, int fallback)
{
    if (!cfb.surv_refs[ref]) cfb.surv_refs[ref] = XPLMFindDataRef(cfb_surv_ref_names[ref]);
    return cfb.surv_refs[ref] ? XPLMGetDatai(cfb.surv_refs[ref]) : fallback;
}

static void cfb_surv_set(int ref, int value)
{
    if (!cfb.surv_refs[ref]) cfb.surv_refs[ref] = XPLMFindDataRef(cfb_surv_ref_names[ref]);
    if (cfb.surv_refs[ref]) XPLMSetDatai(cfb.surv_refs[ref], value);
}

static int cfb_surv_command(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    const char *key = cfb_surv_buttons[(int)(intptr_t)refcon];
    (void)command;
    if (!plugin_enabled || phase != xplm_CommandBegin) return 0;
    if (!strcmp(key, "xpdr_tcas_sys1")) cfb_surv_set(CFB_SURV_XPDR_TCAS_SYS, 1);
    else if (!strcmp(key, "xpdr_tcas_sys2")) cfb_surv_set(CFB_SURV_XPDR_TCAS_SYS, 2);
    else if (!strcmp(key, "wxr_taws_sys1"))   /* pressing the active system switches it off */
        cfb_surv_set(CFB_SURV_WXR_TAWS_SYS, cfb_surv_get(CFB_SURV_WXR_TAWS_SYS, 1) == 1 ? 0 : 1);
    else if (!strcmp(key, "wxr_taws_sys2"))
        cfb_surv_set(CFB_SURV_WXR_TAWS_SYS, cfb_surv_get(CFB_SURV_WXR_TAWS_SYS, 1) == 2 ? 0 : 2);
    else if (!strcmp(key, "tcas_abv"))
        cfb_surv_set(CFB_SURV_TCAS_ALT_SELECT, cfb_surv_get(CFB_SURV_TCAS_ALT_SELECT, 0) == 1 ? 0 : 1);
    else if (!strcmp(key, "tcas_blw"))
        cfb_surv_set(CFB_SURV_TCAS_ALT_SELECT, cfb_surv_get(CFB_SURV_TCAS_ALT_SELECT, 0) == 2 ? 0 : 2);
    else if (!strcmp(key, "tcas_ta_only"))   /* 2 TA/RA, 1 TA ONLY, 0 STBY */
        cfb_surv_set(CFB_SURV_TCAS_ALERT_LEVEL, cfb_surv_get(CFB_SURV_TCAS_ALERT_LEVEL, 2) == 1 ? 2 : 1);
    else if (!strcmp(key, "gs_mode"))
        cfb_surv_set(CFB_SURV_GS_MODE_OFF, !cfb_surv_get(CFB_SURV_GS_MODE_OFF, 0));
    return 0;
}

static float cfb_surv_lamp_get(void *refcon)
{
    const char *key = cfb_surv_buttons[(int)(intptr_t)refcon];
    int on = 0;
    if (!cfb_lamp_power()) return 0.0f;
    if (!strcmp(key, "xpdr_tcas_sys1")) on = cfb_surv_get(CFB_SURV_XPDR_TCAS_SYS, 1) == 1;
    else if (!strcmp(key, "xpdr_tcas_sys2")) on = cfb_surv_get(CFB_SURV_XPDR_TCAS_SYS, 1) == 2;
    else if (!strcmp(key, "wxr_taws_sys1")) on = cfb_surv_get(CFB_SURV_WXR_TAWS_SYS, 1) == 1;
    else if (!strcmp(key, "wxr_taws_sys2")) on = cfb_surv_get(CFB_SURV_WXR_TAWS_SYS, 1) == 2;
    else if (!strcmp(key, "tcas_abv")) on = cfb_surv_get(CFB_SURV_TCAS_ALT_SELECT, 0) == 1;
    else if (!strcmp(key, "tcas_blw")) on = cfb_surv_get(CFB_SURV_TCAS_ALT_SELECT, 0) == 2;
    else if (!strcmp(key, "tcas_ta_only")) on = cfb_surv_get(CFB_SURV_TCAS_ALERT_LEVEL, 2) == 1;
    else if (!strcmp(key, "gs_mode")) on = cfb_surv_get(CFB_SURV_GS_MODE_OFF, 0) != 0;
    return on ? 1.0f : 0.0f;
}

/* ------------------------------------------------------------------ ECAM control panel lamps */

static float cfb_ecam_lamp_get(void *refcon)
{
    int page = cfb_ecam_lamps[(int)(intptr_t)refcon].page;
    if (!plugin_enabled || !sda.initialized || !sda.powered) return 0.0f;
    if (page < 0) {                                  /* CLR: an ECAM warning is waiting to be cleared */
        for (int i = 0; i < sda.warnings.warning_count && i < 16; ++i)
            if (!sda_hidden_warning(&sda.warnings.warning[i])) return 1.0f;
        return 0.0f;
    }
    return !sda.checklist && !sda.abnormal && sda.page == page ? 1.0f : 0.0f;
}

/* ------------------------------------------------------------------ toggles */

static float cfb_toggle_get(void *refcon) { return ((CfbToggle *)refcon)->value; }

static void cfb_toggle_set(void *refcon, float value)
{
    CfbToggle *t = refcon;
    if (plugin_enabled && isfinite(value)) t->target = value >= 0.5f ? 1.0f : 0.0f;
}

static int cfb_toggle_command(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    CfbToggle *t = refcon;
    (void)command;
    if (plugin_enabled && phase == xplm_CommandBegin) t->target = t->target >= 0.5f ? 0.0f : 1.0f;
    return 0;
}

static int cfb_fire_get(XPLMDataRef ref, int *out, int offset, int count)
{
    (void)ref;
    if (!out) return CFB_FIRE_PBS;
    if (offset < 0 || offset >= CFB_FIRE_PBS) return 0;
    if (count > CFB_FIRE_PBS - offset) count = CFB_FIRE_PBS - offset;
    for (int i = 0; i < count; ++i) {
        int k = cfb.fire_toggle[offset + i];
        out[i] = k >= 0 && cfb.toggle[k].value > 0.5f;
    }
    return count;
}

/* ------------------------------------------------------------------ rotaries */

static float cfb_rotary_get(void *refcon) { return ((CfbRotary *)refcon)->value; }

static void cfb_rotary_set(void *refcon, float value)
{
    CfbRotary *r = refcon;
    if (!plugin_enabled || !isfinite(value)) return;
    r->target = value < 0.0f ? 0.0f : value > 1.0f ? 1.0f : value;
    r->value = r->target;
}

static int cfb_rotary_command(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    int index = (int)(intptr_t)refcon;
    const CfbRotaryRow *row = &cfb_rotary_rows[index];
    CfbRotary *r = &cfb.rotary[index];
    float target;
    if (!plugin_enabled || phase != xplm_CommandBegin) return row->hooked;
    target = r->target + (command == r->up ? row->step : -row->step);
    if (row->wrap) {
        target -= floorf(target);                  /* endless: 0..1 is one turn */
        r->value = target;
    } else {
        target = target < 0.0f ? 0.0f : target > 1.0f ? 1.0f : target;
        if (target < 0.001f) target = 0.0f;
        if (target > 0.999f) target = 1.0f;
    }
    r->target = target;
    if (row->travel <= 0.0f) r->value = target;
    return row->hooked;          /* a hooked command still runs its own handler */
}

/* ------------------------------------------------------------------ wipers */

static float cfb_wiper_get(void *refcon) { return ((CfbWiper *)refcon)->sweep; }

static void cfb_wiper_update(float dt)
{
    for (int i = 0; i < CFB_WIPERS; ++i) {
        CfbWiper *w = &cfb.wiper[i];
        int mode = w->selector_index >= 0 ? cfb.position[w->selector_index].position : 0;
        float rate = mode == 2 ? 2.0f : mode == 1 ? 1.2f : 0.0f;   /* strokes per second (FAST / SLOW) */
        if (!plugin_enabled || !a380x_power_any(A380X_ELN_BIT(A380X_ELN_AC1) | A380X_ELN_BIT(A380X_ELN_AC2) |
                                                A380X_ELN_BIT(A380X_ELN_AC_ESS), 1)) rate = 0.0f;
        if (rate <= 0.0f) {                         /* OFF: finish the stroke back to the park position */
            if (w->sweep > 0.0f) { w->direction = -1; rate = 1.2f; }
            else { w->sweep = 0.0f; w->direction = 1; continue; }
        }
        if (w->direction == 0) w->direction = 1;
        w->sweep += (float)w->direction * rate * dt;
        if (w->sweep >= 1.0f) { w->sweep = 1.0f; w->direction = -1; }
        if (w->sweep <= 0.0f) { w->sweep = 0.0f; w->direction = 1; }
    }
}

/* ------------------------------------------------------------------ lifecycle */

static void cfb_shutdown(void)
{
    for (int i = 0; i < CFB_PRESS_COUNT; ++i) {
        CfbPress *p = &cfb.press[i];
        if (p->registered) XPLMUnregisterCommandHandler(p->command, cfb_press_command, 1, p);
        if (p->ref) XPLMUnregisterDataAccessor(p->ref);
    }
    for (int i = 0; i < CFB_POSITION_COUNT; ++i) {
        CfbPosition *p = &cfb.position[i];
        if (p->dec) XPLMUnregisterCommandHandler(p->dec, cfb_position_command, 1, (void *)(intptr_t)i);
        if (p->inc) XPLMUnregisterCommandHandler(p->inc, cfb_position_command, 1, (void *)(intptr_t)i);
        if (p->ref) XPLMUnregisterDataAccessor(p->ref);
    }
    for (int i = 0; i < CFB_SURV_COUNT; ++i) {
        if (cfb.surv_commands[i]) XPLMUnregisterCommandHandler(cfb.surv_commands[i], cfb_surv_command, 0, (void *)(intptr_t)i);
        if (cfb.surv_lamp_refs[i]) XPLMUnregisterDataAccessor(cfb.surv_lamp_refs[i]);
    }
    for (int i = 0; i < CFB_ECAM_LAMPS; ++i)
        if (cfb.ecam_lamp_refs[i]) XPLMUnregisterDataAccessor(cfb.ecam_lamp_refs[i]);
    for (int i = 0; i < CFB_TOGGLE_COUNT; ++i) {
        CfbToggle *g = &cfb.toggle[i];
        if (g->command) XPLMUnregisterCommandHandler(g->command, cfb_toggle_command, 1, g);
        if (g->ref) XPLMUnregisterDataAccessor(g->ref);
    }
    for (int i = 0; i < CFB_ROTARY_COUNT; ++i) {
        CfbRotary *r = &cfb.rotary[i];
        if (r->up) XPLMUnregisterCommandHandler(r->up, cfb_rotary_command, 1, (void *)(intptr_t)i);
        if (r->down) XPLMUnregisterCommandHandler(r->down, cfb_rotary_command, 1, (void *)(intptr_t)i);
        if (r->ref) XPLMUnregisterDataAccessor(r->ref);
    }
    for (int i = 0; i < CFB_WIPERS; ++i)
        if (cfb.wiper[i].ref) XPLMUnregisterDataAccessor(cfb.wiper[i].ref);
    if (cfb.fire_ref) XPLMUnregisterDataAccessor(cfb.fire_ref);
    memset(&cfb, 0, sizeof(cfb));
}

static XPLMDataRef cfb_float_ref(const char *name, XPLMGetDataf_f get, void *refcon)
{
    if (XPLMFindDataRef(name)) return NULL;
    return XPLMRegisterDataAccessor(name, xplmType_Float, 0, NULL, NULL, get, NULL, NULL, NULL, NULL, NULL,
                                    NULL, NULL, NULL, NULL, refcon, NULL);
}

static XPLMDataRef cfb_float_rw(const char *name, XPLMGetDataf_f get, XPLMSetDataf_f set, void *refcon)
{
    if (XPLMFindDataRef(name)) return NULL;
    return XPLMRegisterDataAccessor(name, xplmType_Float, 1, NULL, NULL, get, set, NULL, NULL, NULL, NULL,
                                    NULL, NULL, NULL, NULL, refcon, refcon);
}

static XPLMCommandRef cfb_command(const char *name, const char *description)
{
    XPLMCommandRef c = XPLMFindCommand(name);
    return c ? c : XPLMCreateCommand(name, description);
}

/* XPluginStart, after the SD / ADIRS / FMS modules. */
static int cfb_init(void)
{
    char name[160];
    memset(&cfb, 0, sizeof(cfb));
    cfb.now = XPLMGetElapsedTime();
    for (int i = 0; i < CFB_PRESS_COUNT; ++i) {
        CfbPress *p = &cfb.press[i];
        p->ref = cfb_float_ref(cfb_press_rows[i].dataref, cfb_press_get, p);
        p->command = cfb_command(cfb_press_rows[i].command, cfb_press_rows[i].command);
        if (!p->ref || !p->command) { cfb_shutdown(); return 0; }
        XPLMRegisterCommandHandler(p->command, cfb_press_command, 1, p);
        p->registered = 1;
    }
    for (int i = 0; i < CFB_POSITION_COUNT; ++i) {
        const CfbPositionRow *row = &cfb_position_rows[i];
        CfbPosition *p = &cfb.position[i];
        p->position = row->initial;
        if (XPLMFindDataRef(row->dataref)) { cfb_shutdown(); return 0; }
        p->ref = XPLMRegisterDataAccessor(row->dataref, xplmType_Int | xplmType_Float, 1, cfb_position_geti,
                                          cfb_position_seti, cfb_position_getf, cfb_position_setf, NULL, NULL,
                                          NULL, NULL, NULL, NULL, NULL, NULL, (void *)(intptr_t)i, (void *)(intptr_t)i);
        snprintf(name, sizeof(name), "%s%s", row->dataref, row->switch_kind ? "_down" : "_left");
        p->dec = cfb_command(name, name);
        snprintf(name, sizeof(name), "%s%s", row->dataref, row->switch_kind ? "_up" : "_right");
        p->inc = cfb_command(name, name);
        if (!p->ref || !p->dec || !p->inc) { cfb_shutdown(); return 0; }
        XPLMRegisterCommandHandler(p->dec, cfb_position_command, 1, (void *)(intptr_t)i);
        XPLMRegisterCommandHandler(p->inc, cfb_position_command, 1, (void *)(intptr_t)i);
    }
    for (int i = 0; i < CFB_SURV_COUNT; ++i) {
        snprintf(name, sizeof(name), "a380x/xp12/surv/%s", cfb_surv_buttons[i]);
        cfb.surv_commands[i] = cfb_command(name, name);
        snprintf(name, sizeof(name), "a380x/xp12/surv/lamp/%s", cfb_surv_buttons[i]);
        cfb.surv_lamp_refs[i] = cfb_float_ref(name, cfb_surv_lamp_get, (void *)(intptr_t)i);
        if (!cfb.surv_commands[i] || !cfb.surv_lamp_refs[i]) { cfb_shutdown(); return 0; }
        /* after the press handler (registered in front), so the travel starts first */
        XPLMRegisterCommandHandler(cfb.surv_commands[i], cfb_surv_command, 0, (void *)(intptr_t)i);
    }
    for (int i = 0; i < CFB_ECAM_LAMPS; ++i) {
        cfb.ecam_lamp_refs[i] = cfb_float_ref(cfb_ecam_lamps[i].dataref, cfb_ecam_lamp_get, (void *)(intptr_t)i);
        if (!cfb.ecam_lamp_refs[i]) { cfb_shutdown(); return 0; }
    }
    for (int i = 0; i < CFB_TOGGLE_COUNT; ++i) {
        CfbToggle *g = &cfb.toggle[i];
        g->value = g->target = cfb_toggle_rows[i].initial;
        g->ref = cfb_float_rw(cfb_toggle_rows[i].dataref, cfb_toggle_get, cfb_toggle_set, g);
        snprintf(name, sizeof(name), "%s_toggle", cfb_toggle_rows[i].dataref);
        g->command = cfb_command(name, name);
        if (!g->ref || !g->command) { cfb_shutdown(); return 0; }
        XPLMRegisterCommandHandler(g->command, cfb_toggle_command, 1, g);
    }
    for (int i = 0; i < CFB_ROTARY_COUNT; ++i) {
        const CfbRotaryRow *row = &cfb_rotary_rows[i];
        CfbRotary *r = &cfb.rotary[i];
        r->value = r->target = row->initial;
        r->ref = cfb_float_rw(row->dataref, cfb_rotary_get, cfb_rotary_set, r);
        r->up = cfb_command(row->up, row->up);
        r->down = cfb_command(row->down, row->down);
        if (!r->ref || !r->up || !r->down) { cfb_shutdown(); return 0; }
        XPLMRegisterCommandHandler(r->up, cfb_rotary_command, 1, (void *)(intptr_t)i);
        XPLMRegisterCommandHandler(r->down, cfb_rotary_command, 1, (void *)(intptr_t)i);
    }
    for (int i = 0; i < CFB_WIPERS; ++i) {
        const char *sel = cfb_wiper_selectors[i];
        CfbWiper *w = &cfb.wiper[i];
        w->selector_index = cfb_index(sel + strlen("a380x/xp12/"));
        w->direction = 1;
        w->ref = cfb_float_ref(cfb_wiper_sweeps[i], cfb_wiper_get, w);
        if (!w->ref) { cfb_shutdown(); return 0; }
    }
    for (int i = 0; i < CFB_FIRE_PBS; ++i) {
        cfb.fire_toggle[i] = -1;
        for (int k = 0; k < CFB_TOGGLE_COUNT; ++k)
            if (!strcmp(cfb_toggle_rows[k].dataref, cfb_fire_pushbuttons[i])) cfb.fire_toggle[i] = k;
    }
    if (!XPLMFindDataRef("a380x/xp12/engine/fire_pushbutton"))
        cfb.fire_ref = XPLMRegisterDataAccessor("a380x/xp12/engine/fire_pushbutton", xplmType_IntArray, 0, NULL,
                                                NULL, NULL, NULL, NULL, NULL, cfb_fire_get, NULL, NULL, NULL,
                                                NULL, NULL, NULL, NULL);
    cfb.fms_seen = -1;
    cfb.initialized = 1;
    snprintf(name, sizeof(name), "A380X cockpit feedback: %d keys, %d selectors/switches, %d toggles, %d knobs, "
             "%d lamps registered\n", CFB_PRESS_COUNT, CFB_POSITION_COUNT, CFB_TOGGLE_COUNT, CFB_ROTARY_COUNT,
             CFB_SURV_COUNT + CFB_ECAM_LAMPS);
    XPLMDebugString(name);
    return 1;
}

/* Every frame: key travel by wall time; FMS knob follows a change made elsewhere (MFD, datarefs). */
static void cfb_update(void)
{
    float now, dt;
    if (!cfb.initialized) return;
    now = XPLMGetElapsedTime();
    dt = now - cfb.now;
    cfb.now = now;
    for (int i = 0; i < CFB_PRESS_COUNT; ++i) {
        a380x_press_update(&cfb.press[i].press, now, dt);
        cfb.press[i].value = cfb.press[i].press.position;
    }
    if (dt < 0.0f || dt > 0.5f) dt = 0.0f;        /* paused, or the first frame after a stall */
    for (int i = 0; i < CFB_TOGGLE_COUNT; ++i) {
        CfbToggle *g = &cfb.toggle[i];
        float travel = cfb_toggle_rows[i].travel, step = travel > 0.0f ? dt / travel : 1.0f;
        if (g->value < g->target) g->value = fminf(g->target, g->value + step);
        else if (g->value > g->target) g->value = fmaxf(g->target, g->value - step);
    }
    for (int i = 0; i < CFB_ROTARY_COUNT; ++i) {
        CfbRotary *r = &cfb.rotary[i];
        float travel = cfb_rotary_rows[i].travel, step = travel > 0.0f ? dt / travel : 1.0f;
        if (r->value < r->target) r->value = fminf(r->target, r->value + step);
        else if (r->value > r->target) r->value = fmaxf(r->target, r->value - step);
    }
    cfb_wiper_update(dt);
    {
        int fms = cfb_index("switching/fms");
        if (!cfb.fms_knob) cfb.fms_knob = XPLMFindDataRef("a380x/xp12/fms/switching_knob");
        if (fms >= 0 && cfb.fms_knob) {
            int v = XPLMGetDatai(cfb.fms_knob);
            if (v != cfb.fms_seen && v >= 0 && v <= 2) { cfb.position[fms].position = v; cfb.fms_seen = v; }
        }
    }
}

/* Aircraft (re)load: knobs back to NORM, sources back to normal. */
static void cfb_reset(void)
{
    if (!cfb.initialized) return;
    for (int i = 0; i < CFB_PRESS_COUNT; ++i) a380x_press_end(&cfb.press[i].press);
    for (int i = 0; i < CFB_POSITION_COUNT; ++i) {
        cfb.position[i].position = cfb_position_rows[i].initial;
        cfb_apply_position(i);
    }
    for (int i = 0; i < CFB_TOGGLE_COUNT; ++i) cfb.toggle[i].value = cfb.toggle[i].target = cfb_toggle_rows[i].initial;
    for (int i = 0; i < CFB_ROTARY_COUNT; ++i) cfb.rotary[i].value = cfb.rotary[i].target = cfb_rotary_rows[i].initial;
    for (int i = 0; i < CFB_WIPERS; ++i) { cfb.wiper[i].sweep = 0.0f; cfb.wiper[i].direction = 1; }
}
