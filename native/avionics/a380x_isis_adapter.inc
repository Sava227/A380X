/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * X-Plane boundary of the two A380X ISIS: cockpit devices a380x.xp12.isis.1 / .2 (1024 x 1024
 * texels each, drawn on demand only when their display list changes), their pop-ups (click the
 * screen or a380x/xp12/isis[2]/popup), the physical pushbutton/knob commands with mechanical
 * feedback datarefs, per-unit brightness from the unit's own +/- level and X-Plane's photo-cell
 * value, and the a380x/xp12/isis/ (unit 1) and a380x/xp12/isis2/ (unit 2) datarefs.
 *
 * Roles: exactly one unit is the Standby Flight Display (FBW ISISlegacy), the other the Standby
 * Navigation Display. Unit 1 starts as SFD; MODE on either unit swaps (isis_pair_swap), and an
 * unpowered SFD hands over to a live SND (isis_pair_update). The logic and display lists live
 * in the XPLM-free core a380x_isis.inc (tested by test_isis.c); this file replays them.
 *
 * Inputs: standby pitot/static (airspeed_kts_stby, altitude_ft_stby, standby baro), attitude from
 * the aircraft's inertial attitude (X-Plane has no standby attitude source), magnetic heading
 * (mag_psi), NAV1/NAV2 bearing/DME/ident, LOC/GS of the captain's receiver (refs.side[0]), MDA
 * from the native FMS, power from the DC ESS (bus 0) / battery (DC HOT) voltages.
 *
 * Unity-build include of a380x_avionics.c after a380x_mfd_adapter.inc (mfda_av) and
 * a380x_fms_adapter.inc (fmsa_fms). Statics: isis_.
 */
#include "a380x_elec_power.h"
#include "a380x_avionics_popup.h"
#include "a380x_isis.h"
#include "a380x_isis_font.h"
#include "a380x_isis.inc"
#include "a380x_control_motion.h"

#define ISIS_REDRAW_INTERVAL (1.0f / 30.0f)
#define ISIS_UNITS 2

static const char *const isis_unit_prefixes[ISIS_UNITS] = {"a380x/xp12/isis/", "a380x/xp12/isis2/"};
static const char *const isis_device_ids[ISIS_UNITS] = {"a380x.xp12.isis.1", "a380x.xp12.isis.2"};
static const char *const isis_device_names[ISIS_UNITS] = {"A380X ISIS 1 (standby display)", "A380X ISIS 2 (standby display)"};

/* Command order: IsisButton values, then popup. Same names under both unit prefixes. */
static const char *const isis_command_names[] = {
    "mode", "ls", "plus", "minus", "menu", "set_push", "set_up", "set_down", "popup"};
static const char *const isis_command_descriptions[] = {
    "A380X ISIS: MODE pushbutton (swap SFD / SND)", "A380X ISIS: LS pushbutton (LOC/GS scales)",
    "A380X ISIS: + pushbutton (brightness / previous item)", "A380X ISIS: - pushbutton (brightness / next item)",
    "A380X ISIS: MENU pushbutton (hold 2 s: ATT RST)", "A380X ISIS: SET/SEL knob push (STD / select)",
    "A380X ISIS: SET/SEL knob clockwise", "A380X ISIS: SET/SEL knob anticlockwise", "A380X ISIS: pop-up toggle"};
enum { ISIS_COMMAND_COUNT = ISIS_BTN_COUNT + 1 };

static const char *const isis_ref_names[] = {
    "powered", "state", "page", "device_active", "device_bound", "draw_count", "brightness",
    "ls_active", "baro_std", "baro_unit_inhg", "metric_altitude", "init_seconds", "fail_att",
    "role", "mode_pressed", "ls_pressed", "plus_pressed", "minus_pressed", "menu_pressed",
    "set_sel_axial", "set_sel_rotation"};
enum { ISIS_REF_POWERED, ISIS_REF_STATE, ISIS_REF_PAGE, ISIS_REF_ACTIVE, ISIS_REF_BOUND, ISIS_REF_DRAWS,
       ISIS_REF_BRIGHTNESS, ISIS_REF_LS, ISIS_REF_STD, ISIS_REF_INHG, ISIS_REF_METRIC, ISIS_REF_INIT,
       ISIS_REF_FAIL_ATT, ISIS_REF_ROLE, ISIS_REF_PRESS0, ISIS_REF_AXIAL = ISIS_REF_PRESS0 + 5,
       ISIS_REF_ROTATION, ISIS_REF_COUNT };

enum { ISIS_RAW_IAS, ISIS_RAW_ALT, ISIS_RAW_BARO, ISIS_RAW_PITCH, ISIS_RAW_ROLL, ISIS_RAW_MACH,
       ISIS_RAW_G_SIDE, ISIS_RAW_BUS, ISIS_RAW_BATTERY, ISIS_RAW_PITOT_FAIL, ISIS_RAW_STATIC_FAIL,
       ISIS_RAW_HEADING, ISIS_RAW_NAV1, ISIS_RAW_NAV2 = ISIS_RAW_NAV1 + 6, ISIS_RAW_COUNT = ISIS_RAW_NAV2 + 6 };
/* NAV block order: type, relative bearing, has DME, DME distance, ident, frequency. */
static const char *const isis_raw_names[ISIS_RAW_COUNT] = {
    "sim/cockpit2/gauges/indicators/airspeed_kts_stby",
    "sim/cockpit2/gauges/indicators/altitude_ft_stby",
    "sim/cockpit2/gauges/actuators/barometer_setting_in_hg_stby",
    "sim/flightmodel/position/theta",
    "sim/flightmodel/position/phi",
    "sim/flightmodel/misc/machno",
    "sim/flightmodel/forces/g_side",
    "sim/cockpit2/electrical/bus_volts",
    "sim/cockpit2/electrical/battery_voltage_actual_volts",
    "sim/operation/failures/rel_pitot_stby",
    "sim/operation/failures/rel_static_stby",
    "sim/flightmodel/position/mag_psi",
    "sim/cockpit2/radios/indicators/nav1_type",
    "sim/cockpit2/radios/indicators/nav1_relative_bearing_deg",
    "sim/cockpit2/radios/indicators/nav1_has_dme",
    "sim/cockpit2/radios/indicators/nav1_dme_distance_nm",
    "sim/cockpit2/radios/indicators/nav1_nav_id",
    "sim/cockpit2/radios/actuators/nav1_frequency_hz",
    "sim/cockpit2/radios/indicators/nav2_type",
    "sim/cockpit2/radios/indicators/nav2_relative_bearing_deg",
    "sim/cockpit2/radios/indicators/nav2_has_dme",
    "sim/cockpit2/radios/indicators/nav2_dme_distance_nm",
    "sim/cockpit2/radios/indicators/nav2_nav_id",
    "sim/cockpit2/radios/actuators/nav2_frequency_hz",
};

typedef struct {
    int index;
    IsisInputs inputs;
    XPLMCommandRef command[ISIS_COMMAND_COUNT];
    XPLMDataRef published[ISIS_REF_COUNT];
    XPLMAvionicsID device;
    int bound, fail_att, draw_count, force, popup_placed;
    uint64_t hash;
    float since_build;
    A380XPress press[5], shaft;  /* MODE LS + - MENU, SET/SEL push */
    float press_value[5], shaft_value, rotation;
} IsisUnit;

static IsisState isis_states[ISIS_UNITS];  /* contiguous for isis_pair_swap / isis_pair_update */
static IsisUnit isis_units[ISIS_UNITS];
static IsisList isis_lists[ISIS_UNITS];
static struct {
    XPLMDataRef raw[ISIS_RAW_COUNT];
    int initialized, font_texture;
    float motion_clock;
} isis;

static int isis_raw_failed(XPLMDataRef ref) { return ref && XPLMGetDatai(ref) >= 6; }

static void isis_read_inputs(void)
{
    IsisInputs in;
    float bus[6] = {0}, battery[8] = {0};
    const A380XFmsOutputs *fms;
    int i, n;
    memset(&in, 0, sizeof(in));
    in.ias_kts = XPLMGetDataf(isis.raw[ISIS_RAW_IAS]);
    in.altitude_ft = XPLMGetDataf(isis.raw[ISIS_RAW_ALT]);
    in.baro_inhg = XPLMGetDataf(isis.raw[ISIS_RAW_BARO]);
    in.pitch_deg = XPLMGetDataf(isis.raw[ISIS_RAW_PITCH]);
    in.roll_deg = XPLMGetDataf(isis.raw[ISIS_RAW_ROLL]);
    in.mach = XPLMGetDataf(isis.raw[ISIS_RAW_MACH]);
    in.lateral_g = XPLMGetDataf(isis.raw[ISIS_RAW_G_SIDE]);
    XPLMGetDatavf(isis.raw[ISIS_RAW_BUS], bus, 0, 6);
    XPLMGetDatavf(isis.raw[ISIS_RAW_BATTERY], battery, 0, 8);
    in.dc_ess = isfinite(bus[0]) && bus[0] >= 18.0f;
    for (i = 0; i < 8; ++i) if (isfinite(battery[i]) && battery[i] >= 18.0f) in.dc_hot = 1;
    /* A380 network (spec 6.3): DC ESS, or HOT 1 above 50 kt. */
    in.dc_ess = a380x_power_bus(A380X_ELN_DC_ESS, in.dc_ess);
    in.dc_hot = a380x_power_bus(A380X_ELN_HOT1, in.dc_hot);
    /* Standby pitot/static blockage flags the data it corrupts. */
    in.spd_valid = !isis_raw_failed(isis.raw[ISIS_RAW_PITOT_FAIL]) && !isis_raw_failed(isis.raw[ISIS_RAW_STATIC_FAIL]);
    in.alt_valid = !isis_raw_failed(isis.raw[ISIS_RAW_STATIC_FAIL]);
    in.loc_valid = refs.side[0].loc_valid; in.loc_dots = refs.side[0].loc_dots;
    in.gs_valid = refs.side[0].gs_valid; in.gs_dots = refs.side[0].gs_dots;
    fms = a380x_fms_outputs(&fmsa_fms);
    in.mda_ft = fms && fms->valid && fms->mda_ft > 0.0 ? (float)fms->mda_ft : 0.0f;
    in.heading_mag_deg = XPLMGetDataf(isis.raw[ISIS_RAW_HEADING]);
    in.heading_valid = isfinite(in.heading_mag_deg);
    for (n = 0; n < 2; ++n) {
        int base = n ? ISIS_RAW_NAV2 : ISIS_RAW_NAV1, frequency = XPLMGetDatai(isis.raw[base + 5]);
        char ident[16] = {0};
        in.nav[n].received = XPLMGetDatai(isis.raw[base]) != 0;
        in.nav[n].relative_bearing_deg = XPLMGetDataf(isis.raw[base + 1]);
        in.nav[n].has_dme = XPLMGetDatai(isis.raw[base + 2]) != 0;
        in.nav[n].dme_nm = XPLMGetDataf(isis.raw[base + 3]);
        XPLMGetDatab(isis.raw[base + 4], ident, 0, (int)sizeof(ident) - 1);
        for (i = 0; i < 7 && ident[i]; ++i) in.nav[n].ident[i] = ident[i] >= 'a' && ident[i] <= 'z' ? (char)(ident[i] - 32) : ident[i];
        in.nav[n].frequency_10khz = frequency > 0 && frequency < 20000 ? frequency : 0;
        /* FAA AIM 1-1-9 localizer channels: 108.10-111.95 MHz, odd tenths (as a380x_fcu_adapter.inc). */
        in.nav[n].localizer = frequency >= 10810 && frequency <= 11195 && frequency % 5 == 0 && (frequency / 10) % 2 == 1;
    }
    for (i = 0; i < ISIS_UNITS; ++i) {
        isis_units[i].inputs = in;
        isis_units[i].inputs.att_valid = !isis_units[i].fail_att;
    }
}

/* ---- OpenGL replay ------------------------------------------------------------------------ */

static int isis_gl_font(void)
{
    GLint unpack_alignment;
    unsigned char *alpha;
    size_t i, offset = 0;
    if (isis.font_texture) return 1;
    alpha = (unsigned char *)malloc((size_t)ISIS_FONT_WIDTH * ISIS_FONT_HEIGHT);
    if (!alpha) return 0;
    for (i = 0; i + 1 < sizeof(isis_font_runs); i += 2) {
        if (offset + isis_font_runs[i] > (size_t)ISIS_FONT_WIDTH * ISIS_FONT_HEIGHT) break;
        memset(alpha + offset, isis_font_runs[i + 1], isis_font_runs[i]);
        offset += isis_font_runs[i];
    }
    XPLMGenerateTextureNumbers(&isis.font_texture, 1);
    XPLMBindTexture2d(isis.font_texture, 0);
    glGetIntegerv(GL_UNPACK_ALIGNMENT, &unpack_alignment);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, ISIS_FONT_WIDTH, ISIS_FONT_HEIGHT, 0, GL_ALPHA, GL_UNSIGNED_BYTE, alpha);
    glPixelStorei(GL_UNPACK_ALIGNMENT, unpack_alignment);
    free(alpha);
    return 1;
}

static void isis_gl_draw(const IsisList *l)
{
    int b, i, font = isis_gl_font();
    for (b = 0; b < l->batch_count; ++b) {
        const IsisBatch *batch = &l->batch[b];
        if (batch->textured && !font) continue;
        if (batch->textured) { XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0); XPLMBindTexture2d(isis.font_texture, 0); }
        else XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
        glBegin(GL_TRIANGLES);
        for (i = 0; i < batch->count; ++i) {
            const IsisVertex *v = &l->vertex[batch->first + i];
            glColor4f(v->r, v->g, v->b, v->a);
            glTexCoord2f(v->u, v->v);
            glVertex2f(v->x, (float)ISIS_H - v->y); /* device texels are y up */
        }
        glEnd();
    }
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
}

static void isis_device_draw(void *refcon)
{
    IsisUnit *u = (IsisUnit *)refcon;
    GLboolean cull = glIsEnabled(GL_CULL_FACE);
    glDisable(GL_CULL_FACE);
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    glClearColor(0, 0, 0, 1);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glClear(GL_COLOR_BUFFER_BIT);                      /* X-Plane does not clear device FBOs */
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_FALSE);  /* keep alpha = 1: the screen never blends */
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    if (plugin_enabled && isis.initialized) isis_gl_draw(&isis_lists[u->index]);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glColor4f(1, 1, 1, 1);
    if (cull) glEnable(GL_CULL_FACE);
    ++u->draw_count;
}

static float isis_device_brightness(float rheo, float ambient, float bus, void *refcon)
{
    IsisUnit *u = (IsisUnit *)refcon;
    IsisState *s = &isis_states[u->index];
    (void)rheo; (void)bus;
    if (isfinite(ambient)) s->ambient = ambient;
    return plugin_enabled && s->du != ISIS_DU_OFF ? s->brightness : 0.0f;
}

static void isis_popup_toggle(IsisUnit *u)
{
    int l, t, r, b, sl, st, sr, sb, size;
    if (!u->device || !mfda_av.loaded) return;
    if (mfda_av.is_popup_visible(u->device)) { mfda_av.set_popup_visible(u->device, 0); return; }
    mfda_av.get_geometry(u->device, &l, &t, &r, &b);
    XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);
    if (!u->popup_placed && !(r - l >= 100 && t - b >= 100 && l >= sl && r <= sr && b >= sb && t <= st)) {
        size = (int)((float)(st - sb) * 0.42f);
        l = sl + 60 + u->index * (size + 20);
        mfda_av.set_geometry(u->device, l, st - 80, l + size, st - 80 - size);
    }
    u->popup_placed = 1;
    mfda_av.set_popup_visible(u->device, 1);
    u->force = 1;
}

/* The ISIS screens have no touch function: a click (release) toggles the pop-up, as on the PFDs. */
static int isis_device_touch(int x, int y, XPLMMouseStatus phase, void *refcon)
{
    (void)x; (void)y;
    if (phase == xplm_MouseUp && plugin_enabled) isis_popup_toggle((IsisUnit *)refcon);
    return 1;
}
static int isis_device_right(int x, int y, XPLMMouseStatus phase, void *refcon)
{
    (void)x; (void)y; (void)phase; (void)refcon;
    return 0; /* keep right-drag camera look */
}

static int isis_command(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    int unit = (int)(intptr_t)refcon / 16, index = (int)(intptr_t)refcon % 16, i;
    IsisUnit *u;
    float now = XPLMGetElapsedTime();
    (void)command;
    if (!plugin_enabled || !isis.initialized || unit < 0 || unit >= ISIS_UNITS || index >= ISIS_COMMAND_COUNT) return 1;
    u = &isis_units[unit];
    if (index == ISIS_BTN_COUNT) { if (phase == xplm_CommandBegin) isis_popup_toggle(u); return 1; }
    /* Mechanical feedback first: it follows the command phases whatever the unit's state. */
    if (index <= ISIS_BTN_MENU) {
        if (phase == xplm_CommandBegin) a380x_press_begin(&u->press[index], now, 1.0f);
        else if (phase == xplm_CommandEnd) a380x_press_end(&u->press[index]);
    } else if (index == ISIS_BTN_SET_PUSH) {
        if (phase == xplm_CommandBegin) a380x_press_begin(&u->shaft, now, 1.0f);
        else if (phase == xplm_CommandEnd) a380x_press_end(&u->shaft);
    } else if (phase == xplm_CommandBegin) {
        a380x_knob_turn(&u->rotation, index == ISIS_BTN_SET_UP ? 1.0f : -1.0f, 10.0f); /* FBW COUNT = 36 */
    }
    if (phase == xplm_CommandBegin && index == ISIS_BTN_MODE && isis_states[unit].du != ISIS_DU_OFF) {
        isis_pair_swap(isis_states);
        for (i = 0; i < ISIS_UNITS; ++i) isis_units[i].force = 1;
    }
    if (phase == xplm_CommandBegin) isis_button(&isis_states[unit], &u->inputs, (IsisButton)index, 1);
    else if (phase == xplm_CommandEnd) isis_button(&isis_states[unit], &u->inputs, (IsisButton)index, 0);
    u->force = 1;
    return 1;
}

static float isis_get_float(void *refcon)
{
    int unit = (int)(intptr_t)refcon / 64, ref = (int)(intptr_t)refcon % 64;
    const IsisUnit *u = &isis_units[unit];
    const IsisState *s = &isis_states[unit];
    switch (ref) {
    case ISIS_REF_POWERED: return (float)(plugin_enabled && s->du != ISIS_DU_OFF);
    case ISIS_REF_STATE: return (float)s->du;
    case ISIS_REF_PAGE: return (float)s->page;
    case ISIS_REF_ACTIVE: return (float)(u->device != NULL && u->bound);
    case ISIS_REF_BOUND: return (float)u->bound;
    case ISIS_REF_DRAWS: return (float)u->draw_count;
    case ISIS_REF_BRIGHTNESS: return s->brightness;
    case ISIS_REF_LS: return (float)s->ls_active;
    case ISIS_REF_STD: return (float)s->baro_std;
    case ISIS_REF_INHG: return (float)s->baro_unit_inhg;
    case ISIS_REF_METRIC: return (float)s->metric_alt;
    case ISIS_REF_INIT: return s->du == ISIS_DU_SELFTEST ? fmaxf(0.0f, s->du_timer) : 0.0f;
    case ISIS_REF_FAIL_ATT: return (float)u->fail_att;
    case ISIS_REF_ROLE: return (float)s->role;
    case ISIS_REF_AXIAL: return u->shaft_value;
    case ISIS_REF_ROTATION: return u->rotation;
    default: return ref >= ISIS_REF_PRESS0 && ref < ISIS_REF_AXIAL ? u->press_value[ref - ISIS_REF_PRESS0] : 0.0f;
    }
}
static int isis_get_int(void *refcon)
{
    int ref = (int)(intptr_t)refcon % 64;
    float value = isis_get_float(refcon);
    if (ref == ISIS_REF_BRIGHTNESS) return (int)lroundf(value * 100.0f);
    if (ref == ISIS_REF_INIT) return (int)ceilf(value);
    return (int)lroundf(value);
}
static void isis_set_int(void *refcon, int value)
{
    int unit = (int)(intptr_t)refcon / 64;
    if ((int)(intptr_t)refcon % 64 == ISIS_REF_FAIL_ATT) { isis_units[unit].fail_att = value != 0; isis_units[unit].force = 1; }
}
static void isis_set_float(void *refcon, float value) { if (isfinite(value)) isis_set_int(refcon, (int)value); }

static void isis_adapter_shutdown(void)
{
    int unit, i;
    for (unit = 0; unit < ISIS_UNITS; ++unit) {
        IsisUnit *u = &isis_units[unit];
        for (i = 0; i < ISIS_COMMAND_COUNT; ++i)
            if (u->command[i]) XPLMUnregisterCommandHandler(u->command[i], isis_command, 1, (void *)(intptr_t)(unit * 16 + i));
        for (i = 0; i < ISIS_REF_COUNT; ++i) if (u->published[i]) XPLMUnregisterDataAccessor(u->published[i]);
        if (u->device && mfda_av.loaded) mfda_av.destroy(u->device);
        memset(u, 0, sizeof(*u));
    }
    if (isis.font_texture) { GLuint texture = (GLuint)isis.font_texture; glDeleteTextures(1, &texture); }
    memset(&isis, 0, sizeof(isis));
}

static void isis_reset_pair(int powered)
{
    isis_reset(&isis_states[0], powered);
    isis_reset(&isis_states[1], powered);
    isis_states[0].role = ISIS_ROLE_SFD;
    isis_states[1].role = ISIS_ROLE_SND;
}

/* XPluginStart, after mfda_devices_enable(): the devices must exist before the aircraft OBJs
 * that reference them are loaded. Never fatal for the rest of the avionics. */
static void isis_adapter_init(void)
{
    char name[96];
    int unit, i, ok = 1;
    memset(&isis, 0, sizeof(isis));
    memset(isis_units, 0, sizeof(isis_units));
    for (i = 0; i < ISIS_RAW_COUNT; ++i) {
        isis.raw[i] = XPLMFindDataRef(isis_raw_names[i]);
        if (!isis.raw[i]) {
            XPLMDebugString("A380X ISIS: missing raw input ");
            XPLMDebugString(isis_raw_names[i]);
            XPLMDebugString("\n");
            ok = 0;
        }
    }
    if (!ok) { isis_adapter_shutdown(); return; }
    isis_reset_pair(0);
    if (!mfda_av.loaded) mfda_av_load();
    for (unit = 0; unit < ISIS_UNITS; ++unit) {
        IsisUnit *u = &isis_units[unit];
        u->index = unit;
        for (i = 0; i < ISIS_REF_COUNT; ++i) {
            int writable = i == ISIS_REF_FAIL_ATT;
            void *refcon = (void *)(intptr_t)(unit * 64 + i);
            snprintf(name, sizeof(name), "%s%s", isis_unit_prefixes[unit], isis_ref_names[i]);
            if (XPLMFindDataRef(name)) { XPLMDebugString("A380X ISIS: dataref collision\n"); isis_adapter_shutdown(); return; }
            u->published[i] = XPLMRegisterDataAccessor(name, xplmType_Int | xplmType_Float, writable,
                isis_get_int, writable ? isis_set_int : NULL, isis_get_float, writable ? isis_set_float : NULL,
                NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, refcon, writable ? refcon : NULL);
            if (!u->published[i]) { isis_adapter_shutdown(); return; }
        }
        for (i = 0; i < ISIS_COMMAND_COUNT; ++i) {
            snprintf(name, sizeof(name), "%s%s", isis_unit_prefixes[unit], isis_command_names[i]);
            u->command[i] = XPLMCreateCommand(name, isis_command_descriptions[i]);
            if (!u->command[i]) { isis_adapter_shutdown(); return; }
            XPLMRegisterCommandHandler(u->command[i], isis_command, 1, (void *)(intptr_t)(unit * 16 + i));
        }
        if (mfda_av.loaded) {
            XPLMCreateAvionics_t p;
            memset(&p, 0, sizeof(p));
            p.structSize = (int)sizeof(p);
            p.screenWidth = p.bezelWidth = ISIS_W;
            p.screenHeight = p.bezelHeight = ISIS_H;
            p.drawOnDemand = 1;
            p.drawCallback = isis_device_draw;
            p.bezelDrawCallback = a380x_black_bezel;
            p.screenTouchCallback = isis_device_touch;
            p.screenRightTouchCallback = isis_device_right;
            p.brightnessCallback = isis_device_brightness;
            p.deviceID = (char *)isis_device_ids[unit];
            p.deviceName = (char *)isis_device_names[unit];
            p.refcon = u;
            u->device = mfda_av.create(&p);
            if (u->device) mfda_av.needs_drawing(u->device);
            else XPLMDebugString("A380X ISIS: XPLMCreateAvionicsEx failed; that ISIS stays dark\n");
        }
        u->force = 1;
    }
    if (!mfda_av.loaded) XPLMDebugString("A380X ISIS: avionics device API unavailable (X-Plane < 12.1); ISIS stay dark\n");
    isis.initialized = 1;
    XPLMDebugString("A380X ISIS: devices a380x.xp12.isis.1 (SFD) / a380x.xp12.isis.2 (SND) and 18 commands registered\n");
}

static void isis_adapter_update(float dt)
{
    int unit, k;
    float now, wall;
    if (!isis.initialized) return;
    isis_read_inputs();
    now = XPLMGetElapsedTime();
    wall = now - isis.motion_clock;
    if (!(wall >= 0.0f) || wall > 0.5f) wall = 0.0f;  /* mechanical motion also runs while paused */
    isis.motion_clock = now;
    for (unit = 0; unit < ISIS_UNITS; ++unit) isis_update(&isis_states[unit], &isis_units[unit].inputs, dt);
    isis_pair_update(isis_states);
    for (unit = 0; unit < ISIS_UNITS; ++unit) {
        IsisUnit *u = &isis_units[unit];
        IsisState *s = &isis_states[unit];
        uint64_t hash;
        for (k = 0; k < 5; ++k) { a380x_press_update(&u->press[k], now, wall); u->press_value[k] = u->press[k].position; }
        a380x_press_update(&u->shaft, now, wall);
        u->shaft_value = u->shaft.position;
        if (s->baro_write) {  /* only the SFD writes the standby baro */
            if (s->role == ISIS_ROLE_SFD) {
                XPLMSetDataf(isis.raw[ISIS_RAW_BARO], s->baro_write_inhg);
                for (k = 0; k < ISIS_UNITS; ++k) isis_units[k].inputs.baro_inhg = s->baro_write_inhg;
            }
            s->baro_write = 0;
        }
        u->bound = u->device && mfda_av.loaded && mfda_av.is_bound(u->device);
        u->since_build += isfinite(dt) && dt > 0.0f ? dt : 0.0f;
        if (!u->force && u->since_build < ISIS_REDRAW_INTERVAL) continue;
        u->since_build = 0.0f;
        isis_build(&isis_lists[unit], s, &u->inputs);
        hash = isis_list_hash(&isis_lists[unit]);
        if ((u->force || hash != u->hash) && u->device && mfda_av.loaded) mfda_av.needs_drawing(u->device);
        u->hash = hash;
        u->force = 0;
    }
}

static void isis_adapter_plane_loaded(void)
{
    int unit;
    if (!isis.initialized) return;
    isis_read_inputs();
    isis_reset_pair(isis_has_power(&isis_units[0].inputs));
    for (unit = 0; unit < ISIS_UNITS; ++unit) isis_units[unit].force = 1;
}

static void isis_adapter_disable(void)
{
    int unit;
    if (!isis.initialized) return;
    for (unit = 0; unit < ISIS_UNITS; ++unit) {
        IsisUnit *u = &isis_units[unit];
        IsisState *s = &isis_states[unit];
        int k;
        s->plus_held = s->minus_held = s->menu_held = 0;
        for (k = 0; k < 5; ++k) a380x_press_end(&u->press[k]);
        a380x_press_end(&u->shaft);
        if (u->device && mfda_av.loaded) { mfda_av.set_popup_visible(u->device, 0); mfda_av.needs_drawing(u->device); }
    }
}
