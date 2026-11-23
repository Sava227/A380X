/* SPDX-License-Identifier: GPL-3.0-or-later
 * Independent captain/first-officer EFIS controls. Stock EFIS datarefs mirror
 * physical selector positions; our native PFD/ND renderers consume this state.
 * WX and ETACS selections are recorded for an explicit INOP annunciation only.
 * WX and TERR select the ND's one surveillance overlay (FBW A380FcuComputer pSurvFilter NONE/WX/TERR):
 * pressing the selected one deselects it, pressing the other replaces it (docs/ND_TERRAIN.md).
 */
#include "a380x_elec_power.h"
#include <stddef.h>

#include "a380x_efis.h"
#include "a380x_control_motion.h"
static EfisSide efis[2];
enum { EFIS_PRESS_KEYS = 13, EFIS_PRESS_BARO = 13, EFIS_PRESS_COUNT = 14 };
static A380XPress efis_press[2][EFIS_PRESS_COUNT];
static float efis_motion_clock;

typedef struct {
    XPLMDataRef cstr, wpt, vord, ndb, arpt, vor1, vor2, terr, traf, wx;
    XPLMDataRef mode, range, range_nm, baro, standard;
    float written_baro_hpa;
    int written_std;
} EfisBoundary;
static EfisBoundary efis_boundary[2];
static int efis_previous_loc_active;
static const float efis_ranges_nm[] = {1.0f, 10.0f, 20.0f, 40.0f, 80.0f, 160.0f, 320.0f, 640.0f};
static const float efis_hpa_per_inhg = 33.8638867f;

enum {
    EFIS_CSTR, EFIS_WPT, EFIS_VORD, EFIS_NDB, EFIS_ARPT, EFIS_VOR1, EFIS_VOR2,
    EFIS_TERR, EFIS_TRAF, EFIS_WX, EFIS_TAXI, EFIS_LS, EFIS_VV,
    EFIS_MODE_UP, EFIS_MODE_DOWN, EFIS_RANGE_UP, EFIS_RANGE_DOWN,
    EFIS_BARO_UP, EFIS_BARO_DOWN, EFIS_BARO_STD, EFIS_BARO_QNH, EFIS_BARO_UNIT,
    EFIS_PLAN_NEXT, EFIS_PLAN_PREVIOUS,
    EFIS_ACTION_COUNT
};
static const char *efis_action_names[] = {
    "cstr_toggle", "wpt_toggle", "vord_toggle", "ndb_toggle", "arpt_toggle",
    "vor1_toggle", "vor2_toggle", "terr_toggle", "traf_toggle", "wx_toggle",
    "taxi_toggle", "ls_toggle", "vv_toggle", "mode_up", "mode_down",
    "range_up", "range_down", "baro_up", "baro_down", "baro_std", "baro_qnh", "baro_unit_toggle",
    "plan_next", "plan_previous"
};
typedef struct {
    int side, action;
    XPLMCommandRef handle;
    float next_repeat;
    char name[112];
} EfisCommand;
static EfisCommand efis_commands[2][EFIS_ACTION_COUNT];

typedef struct { const char *name; size_t offset; int floating; } EfisField;
#define EFIS_INT(n) {#n, offsetof(EfisSide,n), 0}
#define EFIS_FLOAT(n) {#n, offsetof(EfisSide,n), 1}
static const EfisField efis_fields[] = {
    EFIS_INT(cstr), EFIS_INT(wpt), EFIS_INT(vord), EFIS_INT(ndb), EFIS_INT(arpt),
    EFIS_INT(vor1), EFIS_INT(vor2), EFIS_INT(terr), EFIS_INT(traf), EFIS_INT(ls), EFIS_INT(vv),
    EFIS_INT(wx_requested), EFIS_INT(taxi_requested), EFIS_INT(mode), EFIS_INT(range_index),
    EFIS_INT(zoom), EFIS_INT(baro_std), EFIS_INT(baro_inhg), EFIS_INT(powered),
    EFIS_FLOAT(range_nm), EFIS_FLOAT(qnh_hpa), EFIS_FLOAT(baro_preset_hpa),
#define EFIS_PRESSED(n, i) {#n, offsetof(EfisSide, pressed[i]), 1}
    EFIS_PRESSED(cstr_pressed, 0), EFIS_PRESSED(wpt_pressed, 1), EFIS_PRESSED(vord_pressed, 2),
    EFIS_PRESSED(ndb_pressed, 3), EFIS_PRESSED(arpt_pressed, 4), EFIS_PRESSED(vor1_pressed, 5),
    EFIS_PRESSED(vor2_pressed, 6), EFIS_PRESSED(terr_pressed, 7), EFIS_PRESSED(traf_pressed, 8),
    EFIS_PRESSED(wx_pressed, 9), EFIS_PRESSED(taxi_pressed, 10), EFIS_PRESSED(ls_pressed, 11),
    EFIS_PRESSED(vv_pressed, 12), EFIS_PRESSED(baro_knob_axial, 13)
#undef EFIS_PRESSED
};
#undef EFIS_INT
#undef EFIS_FLOAT
typedef struct {
    XPLMDataRef handle;
    int side, field;
    char name[112];
} EfisPublished;
static EfisPublished efis_outputs[2][ARRAY_LEN(efis_fields)];

static float efis_get_float(void *refcon)
{
    EfisPublished *p = (EfisPublished *)refcon;
    const EfisField *field = &efis_fields[p->field];
    const char *address = (const char *)&efis[p->side] + field->offset;
    return field->floating ? *(const float *)address : (float)*(const int *)address;
}
static int efis_get_int(void *refcon) { return (int)efis_get_float(refcon); }
static float efis_clamp_baro(float hpa)
{
    return isfinite(hpa) ? fmaxf(745.0f, fminf(1100.0f, hpa)) : 1013.25f;
}
static void efis_set_range(EfisSide *s, int index)
{
    s->range_index = index < 0 ? 0 : index > 7 ? 7 : index;
    s->zoom = s->range_index == 0;
    s->range_nm = efis_ranges_nm[s->range_index];
}
static void efis_set_mode(EfisSide *s, int mode)
{
    if (mode < 0 || mode > 4) {
        s->mode = 2;
        efis_set_range(s, 4);
        return;
    }
    if (s->mode == 3 && mode != 3) efis_set_range(s, s->range_index + 1);
    else if (s->mode != 3 && mode == 3) efis_set_range(s, s->range_index - 1);
    s->mode = mode;
}
static void efis_sync_side(int side)
{
    EfisSide *s = &efis[side];
    EfisBoundary *b = &efis_boundary[side];
    Side *display = &refs.side[side];
#define EFIS_WRITE(field) XPLMSetDatai(b->field, s->field)
    EFIS_WRITE(cstr); EFIS_WRITE(wpt); EFIS_WRITE(vord); EFIS_WRITE(ndb); EFIS_WRITE(arpt);
    EFIS_WRITE(terr); EFIS_WRITE(traf); EFIS_WRITE(mode);
#undef EFIS_WRITE
    XPLMSetDatai(b->vor1, s->vor1 ? 2 : 1);
    XPLMSetDatai(b->vor2, s->vor2 ? 2 : 1);
    XPLMSetDatai(b->wx, 0); /* No weather image provider exists in this port. */
    XPLMSetDatai(b->range, s->range_index);
    b->written_baro_hpa = s->baro_std ? 1013.25f : s->qnh_hpa;
    b->written_std = s->baro_std;
    XPLMSetDatai(b->standard, s->baro_std);
    XPLMSetDataf(b->baro, b->written_baro_hpa / efis_hpa_per_inhg);
    display->nd_mode = s->mode;
    display->nd_range_nm = s->range_nm;
    display->baro_std = s->baro_std;
    display->baro_hpa_value = b->written_baro_hpa;
    display->ls_visible = s->ls;
}

static int efis_command_handler(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    EfisCommand *c = (EfisCommand *)refcon;
    EfisSide *s = &efis[c->side];
    float now = XPLMGetElapsedTime();
    int repeatable = c->action >= EFIS_MODE_UP && c->action <= EFIS_BARO_DOWN;
    (void)command;
    if (plugin_enabled) {
        /* Mechanical feedback follows the command phases (mouse held = key held). */
        int press = c->action < EFIS_PRESS_KEYS ? c->action
            : (c->action == EFIS_BARO_STD || c->action == EFIS_BARO_QNH) ? EFIS_PRESS_BARO : -1;
        if (press >= 0 && phase == xplm_CommandBegin)
            a380x_press_begin(&efis_press[c->side][press], now, c->action == EFIS_BARO_QNH ? -1.0f : 1.0f);
        else if (press >= 0 && phase == xplm_CommandEnd) a380x_press_end(&efis_press[c->side][press]);
    }
    if (!plugin_enabled || phase == xplm_CommandEnd) return 1;
    if (phase == xplm_CommandContinue && (!repeatable || now < c->next_repeat)) return 1;
    c->next_repeat = now + (phase == xplm_CommandBegin ? 0.4f : 0.10f);
    switch (c->action) {
#define EFIS_TOGGLE(action, field) case action: s->field = !s->field; break
        EFIS_TOGGLE(EFIS_CSTR, cstr); EFIS_TOGGLE(EFIS_WPT, wpt); EFIS_TOGGLE(EFIS_VORD, vord);
        EFIS_TOGGLE(EFIS_NDB, ndb); EFIS_TOGGLE(EFIS_ARPT, arpt);
        EFIS_TOGGLE(EFIS_VOR1, vor1); EFIS_TOGGLE(EFIS_VOR2, vor2);
        EFIS_TOGGLE(EFIS_TRAF, traf);
        EFIS_TOGGLE(EFIS_TAXI, taxi_requested);
        case EFIS_TERR: s->terr = !s->terr; if (s->terr) s->wx_requested = 0; break;
        case EFIS_WX: s->wx_requested = !s->wx_requested; if (s->wx_requested) s->terr = 0; break;
        EFIS_TOGGLE(EFIS_VV, vv);
#undef EFIS_TOGGLE
        case EFIS_LS:
            a380x_guidance_command(&guidance, &guidance_inputs, c->side ? A380X_FCU_LS2 : A380X_FCU_LS1, 0);
            s->ls = c->side ? guidance.ls2 : guidance.ls1;
            break;
        case EFIS_MODE_UP: if (s->mode < 4) efis_set_mode(s, s->mode + 1); break;
        case EFIS_MODE_DOWN: if (s->mode > 0) efis_set_mode(s, s->mode - 1); break;
        case EFIS_RANGE_UP: efis_set_range(s, s->range_index + 1); break;
        case EFIS_RANGE_DOWN: efis_set_range(s, s->range_index - 1); break;
        case EFIS_BARO_UP:
        case EFIS_BARO_DOWN: {
            float direction = c->action == EFIS_BARO_UP ? 1.0f : -1.0f;
            float next = s->baro_inhg
                ? (roundf(s->baro_preset_hpa / efis_hpa_per_inhg * 100.0f) + direction) * efis_hpa_per_inhg * 0.01f
                : roundf(s->baro_preset_hpa) + direction;
            s->baro_preset_hpa = efis_clamp_baro(next);
            if (!s->baro_std) s->qnh_hpa = s->baro_preset_hpa;
            else s->baro_preset_visible = 1;
            break;
        }
        case EFIS_BARO_STD: s->baro_std = 1; s->baro_preset_visible = 0; break;
        case EFIS_BARO_QNH: s->baro_std = 0; s->baro_preset_visible = 0; s->qnh_hpa = s->baro_preset_hpa; break;
        case EFIS_BARO_UNIT: s->baro_inhg = !s->baro_inhg; break;
        case EFIS_PLAN_NEXT:
        case EFIS_PLAN_PREVIOUS: {
            int count = XPLMCountFMSEntries();
            int entry = XPLMGetDisplayedFMSEntry() + (c->action == EFIS_PLAN_NEXT ? 1 : -1);
            if (count > 0) XPLMSetDisplayedFMSEntry(entry < 0 ? 0 : entry >= count ? count - 1 : entry);
            break;
        }
        default: break;
    }
    efis_sync_side(c->side);
    return 1;
}

static void efis_update(float dt)
{
    int side;
    int loc_active = guidance.loc_armed || guidance.lateral_mode == A380X_C1_LOC
        || guidance.lateral_mode == A380X_C1_LOC_STAR;
    float now = XPLMGetElapsedTime(), wall = now - efis_motion_clock;
    (void)dt;
    /* Key travel uses wall time so it also works while the simulation is paused. */
    if (!(wall >= 0.0f) || wall > 0.5f) wall = 0.0f;
    efis_motion_clock = now;
    for (side = 0; side < 2; ++side) {
        int k;
        for (k = 0; k < EFIS_PRESS_COUNT; ++k) {
            a380x_press_update(&efis_press[side][k], now, wall);
            efis[side].pressed[k] = efis_press[side][k].position;
        }
    }
    if (loc_active && !efis_previous_loc_active) guidance.ls1 = guidance.ls2 = 1;
    efis_previous_loc_active = loc_active;
    for (side = 0; side < 2; ++side) {
        EfisSide *s = &efis[side];
        EfisBoundary *b = &efis_boundary[side];
        int mode = XPLMGetDatai(b->mode), range = XPLMGetDatai(b->range);
        int standard = XPLMGetDatai(b->standard) != 0;
        float hpa = XPLMGetDataf(b->baro) * efis_hpa_per_inhg;
        int changed_pressure = !isfinite(hpa) || fabsf(hpa - b->written_baro_hpa) > 0.02f;
        /* EFIS CP: CAPT DC ESS, F/O DC 2 (A380 network, spec 9.2 assumption). */
        s->powered = a380x_power_bus(side ? A380X_ELN_DC2 : A380X_ELN_DC_ESS,
                                     guidance_inputs.powered && refs.side[side].display_valid);
#define EFIS_READ(field) s->field = XPLMGetDatai(b->field) != 0
        EFIS_READ(cstr); EFIS_READ(wpt); EFIS_READ(vord); EFIS_READ(ndb); EFIS_READ(arpt);
        EFIS_READ(traf);
#undef EFIS_READ
        {   /* stock EFIS_terrain_on (hardware, other plugins): selecting TERR there replaces WX */
            int terr = XPLMGetDatai(b->terr) != 0;
            if (terr && !s->terr) s->wx_requested = 0;
            s->terr = terr;
        }
        s->vor1 = XPLMGetDatai(b->vor1) == 2;
        s->vor2 = XPLMGetDatai(b->vor2) == 2;
        if (XPLMGetDatai(b->wx)) { s->wx_requested = 1; s->terr = 0; }
        s->ls = side ? guidance.ls2 : guidance.ls1;
        if (range < 0 || range > 7 || mode < 0 || mode > 4) {
            s->mode = 2;
            efis_set_range(s, 4);
        } else {
            if (range != s->range_index) efis_set_range(s, range);
            if (mode != s->mode) efis_set_mode(s, mode);
        }
        /* Hardware/keyboard changes use the same STD preset semantics. While
         * STD is active, the indicated reference remains exactly 1013.25 hPa. */
        if (changed_pressure && !(standard && !b->written_std && fabsf(hpa - 1013.25f) < 0.02f)) {
            s->baro_preset_hpa = efis_clamp_baro(hpa);
            if (!standard) s->qnh_hpa = s->baro_preset_hpa;
            else s->baro_preset_visible = 1;
        }
        if (standard != b->written_std) {
            s->baro_std = standard;
            s->baro_preset_visible = 0;
            if (!standard) s->qnh_hpa = s->baro_preset_hpa;
        }
        efis_sync_side(side);
    }
}

static void efis_shutdown(void)
{
    int side, i;
    for (side = 0; side < 2; ++side) {
        for (i = 0; i < EFIS_ACTION_COUNT; ++i) {
            EfisCommand *c = &efis_commands[side][i];
            if (c->handle) {
                XPLMUnregisterCommandHandler(c->handle, efis_command_handler, 1, c);
                c->handle = NULL;
            }
        }
        for (i = 0; i < ARRAY_LEN(efis_fields); ++i) {
            if (efis_outputs[side][i].handle) {
                XPLMUnregisterDataAccessor(efis_outputs[side][i].handle);
                efis_outputs[side][i].handle = NULL;
            }
        }
    }
}

static int efis_init(void)
{
    int side, i, ok = 1;
    char name[128];
    int steps[8] = {1, 10, 20, 40, 80, 160, 320, 640};
    XPLMDataRef range_steps = find_required("sim/cockpit2/EFIS/map_range_steps", &ok);
    memset(efis, 0, sizeof(efis));
    memset(efis_boundary, 0, sizeof(efis_boundary));
    memset(efis_press, 0, sizeof(efis_press));
    efis_motion_clock = 0.0f;
    efis_previous_loc_active = 0;
    for (side = 0; side < 2; ++side) {
        EfisSide *s = &efis[side];
        EfisBoundary *b = &efis_boundary[side];
        const char *suffix = side ? "_copilot" : "";
        const char *pilot = side ? "copilot" : "pilot";
#define EFIS_REF(field, base) do { \
        snprintf(name, sizeof(name), "sim/cockpit2/EFIS/" base "%s", suffix); \
        b->field = find_required(name, &ok); \
    } while (0)
        EFIS_REF(cstr, "EFIS_data_on"); EFIS_REF(wpt, "EFIS_fix_on"); EFIS_REF(vord, "EFIS_vor_on");
        EFIS_REF(ndb, "EFIS_ndb_on"); EFIS_REF(arpt, "EFIS_airport_on");
        EFIS_REF(terr, "EFIS_terrain_on"); EFIS_REF(traf, "EFIS_tcas_on"); EFIS_REF(wx, "EFIS_weather_on");
        EFIS_REF(mode, "map_mode"); EFIS_REF(range, "map_range"); EFIS_REF(range_nm, "map_range_nm");
#undef EFIS_REF
        snprintf(name, sizeof(name), "sim/cockpit2/EFIS/EFIS_1_selection_%s", pilot);
        b->vor1 = find_required(name, &ok);
        snprintf(name, sizeof(name), "sim/cockpit2/EFIS/EFIS_2_selection_%s", pilot);
        b->vor2 = find_required(name, &ok);
        b->baro = refs.side[side].baro;
        b->standard = refs.side[side].baro_standard;
        if (!b->baro || !b->standard) ok = 0;
        if (!ok) { efis_shutdown(); return 0; }
        s->mode = XPLMGetDatai(b->mode);
        i = XPLMGetDatai(b->range);
        if (s->mode < 0 || s->mode > 4 || i < 0 || i > 7) { s->mode = 2; i = 4; }
        else {
            int stop;
            float previous_nm = XPLMGetDataf(b->range_nm);
            /* A saved stock selector index may use a different range table.
             * Preserve its actual distance when replacing that table with
             * the eight FBW stops, rather than silently halving the range. */
            for (stop = 0; stop < ARRAY_LEN(efis_ranges_nm); ++stop)
                if (isfinite(previous_nm) && fabsf(previous_nm - efis_ranges_nm[stop]) < 0.01f) break;
            if (stop < ARRAY_LEN(efis_ranges_nm)) i = stop;
            else { s->mode = 2; i = 4; }
        }
        efis_set_range(s, i);
        XPLMSetDatai(b->mode, s->mode);
        XPLMSetDatai(b->range, s->range_index);
        s->baro_std = XPLMGetDatai(b->standard) != 0;
        s->qnh_hpa = efis_clamp_baro(XPLMGetDataf(b->baro) * efis_hpa_per_inhg);
        if (s->baro_std) s->qnh_hpa = 1013.25f;
        s->baro_preset_hpa = s->qnh_hpa;
        b->written_baro_hpa = s->baro_std ? 1013.25f : s->qnh_hpa;
        b->written_std = s->baro_std;
        for (i = 0; i < ARRAY_LEN(efis_fields); ++i) {
            EfisPublished *p = &efis_outputs[side][i];
            p->side = side; p->field = i;
            snprintf(p->name, sizeof(p->name), "a380x/xp12/efis/%s/%s",
                side ? "first_officer" : "captain", efis_fields[i].name);
            if (XPLMFindDataRef(p->name)) { efis_shutdown(); return 0; }
            p->handle = XPLMRegisterDataAccessor(p->name, xplmType_Int | xplmType_Float, 0,
                efis_get_int, NULL, efis_get_float, NULL, NULL, NULL, NULL, NULL,
                NULL, NULL, NULL, NULL, p, NULL);
            if (!p->handle) { efis_shutdown(); return 0; }
        }
        for (i = 0; i < EFIS_ACTION_COUNT; ++i) {
            EfisCommand *c = &efis_commands[side][i];
            c->side = side; c->action = i; c->next_repeat = 0;
            snprintf(c->name, sizeof(c->name), "a380x/xp12/efis/%s/%s",
                side ? "first_officer" : "captain", efis_action_names[i]);
            c->handle = XPLMCreateCommand(c->name, c->name + 16);
            if (!c->handle) { efis_shutdown(); return 0; }
            XPLMRegisterCommandHandler(c->handle, efis_command_handler, 1, c);
        }
    }
    XPLMSetDatavi(range_steps, steps, 0, 8);
    efis_update(0);
    return 1;
}
