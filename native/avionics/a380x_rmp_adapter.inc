/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * RMP 1-3: X-Plane adapter of a380x_rmp.h (FlyByWire A380X RMP logic).
 *
 * Commands a380x/xp12/rmp/<n>/<key> (keys of tools/cockpit_feedback_contract.py RMP_KEYS, same order as
 * rmp_key_names), <channel>_knob_push / _up / _down for the reception knobs, brightness_up / _down for the
 * OFF / BRT knob. Datarefs <channel>_rx, <channel>_volume, brightness, state, page, standby_led,
 * device_active and lamp/vhf<k>_call (transmit selected). Screens: cockpit devices a380x.xp12.rmp.<n>
 * (768 x 466), drawn with the MFD display-list renderer.
 *
 * Radios: VHF 1 / VHF 2 are X-Plane COM 1 / COM 2 (active and standby, 8.33 kHz datarefs, kHz), followed both
 * ways so ATC tuning or another tool is shown. VHF 3 has no X-Plane radio (DATA by default). The transponder
 * code is X-Plane's; IDENT runs sim/transponder/transponder_ident. RMP 1 (captain) also drives X-Plane's
 * transmit selector (COM 1 / COM 2) and COM 1 / COM 2 reception and volume.
 * Power: RMP 1 / 2 on DC ESS, RMP 3 on DC 1 (FBW RmpStateController).
 * Unity-build include of a380x_avionics.c after the MFD adapter (renderer, device API).
 */
#include "a380x_rmp.h"

#define RMP_PREFIX "a380x/xp12/rmp/"
#define RMP_KNOB_COUNT 12
#define RMP_W 768
#define RMP_H 466

static const char *const rmp_key_names[] = {
    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "dot", "clr", "up", "down", "adk1", "adk2", "adk3",
    "lsk1", "lsk2", "lsk3", "vhf", "hf", "tel", "sqwk", "menu", "nav", "msg_clr", "rst", "vhf1_call",
    "vhf2_call", "vhf3_call", "hf1_call", "hf2_call", "tel1_call", "tel2_call", "int_call", "cab_call",
    "pa_call", "voice_call", "stby_nav", "blank", "blank2", "dummy2"};
#define RMP_KEY_COUNT ((int)ARRAY_LEN(rmp_key_names))
static const char *const rmp_knob_names[RMP_KNOB_COUNT] = {
    "vhf1", "vhf2", "vhf3", "hf1", "hf2", "tel1", "tel2", "int", "cab", "pa", "voice", "dummy"};
static const char *const rmp_device_ids[A380X_RMP_COUNT] = {"a380x.xp12.rmp.1", "a380x.xp12.rmp.2", "a380x.xp12.rmp.3"};
static const char *const rmp_device_names[A380X_RMP_COUNT] = {"A380X RMP 1", "A380X RMP 2", "A380X RMP 3"};

static int rmp_key_enum(int index)
{
    return index <= A380X_RMP_KEY_VHF3_CALL ? index : A380X_RMP_KEY_OTHER;   /* rmp_key_names follows the enum */
}

enum { RR_RX, RR_VOLUME, RR_BRIGHTNESS, RR_STATE, RR_PAGE, RR_STANDBY_LED, RR_DEVICE_ACTIVE, RR_CALL_LAMP };
typedef struct { int kind, rmp, index; XPLMDataRef handle; char name[96]; } RmpRef;
typedef struct { int rmp, kind, index; XPLMCommandRef handle; } RmpCommand;   /* kind 0 key, 1 knob push, 2/3 knob up/down, 4/5 brightness */

enum { RX_COM1, RX_COM1_STBY, RX_COM2, RX_COM2_STBY, RX_XPDR, RX_XPDR_MODE, RX_AUDIO_SEL, RX_RX1, RX_RX2,
       RX_VOL1, RX_VOL2, RX_SURV_AUTO, RX_COUNT };
static const char *const rmp_xp_names[RX_COUNT] = {
    "sim/cockpit2/radios/actuators/com1_frequency_hz_833", "sim/cockpit2/radios/actuators/com1_standby_frequency_hz_833",
    "sim/cockpit2/radios/actuators/com2_frequency_hz_833", "sim/cockpit2/radios/actuators/com2_standby_frequency_hz_833",
    "sim/cockpit2/radios/actuators/transponder_code", "sim/cockpit2/radios/actuators/transponder_mode",
    "sim/cockpit2/radios/actuators/audio_com_selection", "sim/cockpit2/radios/actuators/audio_selection_com1",
    "sim/cockpit2/radios/actuators/audio_selection_com2", "sim/cockpit2/radios/actuators/audio_volume_com1",
    "sim/cockpit2/radios/actuators/audio_volume_com2", "a380x/xp12/surv/xpdr_auto"};

static struct {
    int initialized;
    A380XRmp unit[A380X_RMP_COUNT];
    A380XRmpRadios radios;
    float volume[A380X_RMP_COUNT][RMP_KNOB_COUNT];
    int rx[A380X_RMP_COUNT][RMP_KNOB_COUNT];
    XPLMAvionicsID device[A380X_RMP_COUNT];
    int bound[A380X_RMP_COUNT];
    XPLMDataRef xp[RX_COUNT];
    XPLMCommandRef ident;
    int last_khz[4], last_xpdr, last_tx, last_rx[2];
    float last_vol[2];
    RmpRef refs[A380X_RMP_COUNT * (2 * RMP_KNOB_COUNT + 6 + 3)];
    int ref_count;
    RmpCommand commands[A380X_RMP_COUNT * (RMP_KEY_COUNT + 3 * RMP_KNOB_COUNT + 2)];
    int command_count;
    float now;
} rmpa;
static MfdDrawList rmpa_list;

/* ---------------------------------------------------------------- X-Plane radios */

static int rmpa_geti(int i, int fallback) { return rmpa.xp[i] ? XPLMGetDatai(rmpa.xp[i]) : fallback; }
static void rmpa_seti(int i, int v) { if (rmpa.xp[i]) XPLMSetDatai(rmpa.xp[i], v); }

/* VHF 1 / 2 <-> COM 1 / 2 (active, standby): an outside change is adopted, an RMP change is written. */
static void rmpa_sync_radios(void)
{
    static const int refs[4] = {RX_COM1, RX_COM1_STBY, RX_COM2, RX_COM2_STBY};
    A380XRmpRadios *r = &rmpa.radios;
    for (int k = 0; k < 4; ++k) {
        int vhf = k / 2, stby = k & 1, xp, want;
        uint32_t *bcd = stby ? &r->standby[vhf] : &r->active[vhf];
        if (!rmpa.xp[refs[k]]) continue;
        xp = XPLMGetDatai(rmpa.xp[refs[k]]);
        if (xp != rmpa.last_khz[k] && xp >= 118000 && xp <= 136990) { *bcd = a380x_rmp_khz_to_bcd(xp); rmpa.last_khz[k] = xp; }
        want = a380x_rmp_bcd_to_khz(*bcd);
        if (a380x_rmp_valid_frequency(*bcd) && want != xp) { rmpa_seti(refs[k], want); rmpa.last_khz[k] = want; }
    }
    {
        int xp = rmpa_geti(RX_XPDR, r->xpdr_code);
        if (xp != rmpa.last_xpdr) { r->xpdr_code = xp; rmpa.last_xpdr = xp; }
        if (r->xpdr_code != xp) { rmpa_seti(RX_XPDR, r->xpdr_code); rmpa.last_xpdr = r->xpdr_code; }
    }
    r->xpdr_auto = rmpa.xp[RX_SURV_AUTO] ? XPLMGetDatai(rmpa.xp[RX_SURV_AUTO]) != 0 : rmpa_geti(RX_XPDR_MODE, 2) >= 2;
    /* captain RMP: X-Plane's transmit selector and COM 1 / 2 reception */
    {
        A380XRmp *u = &rmpa.unit[0];
        if (u->tx != rmpa.last_tx && (u->tx == 1 || u->tx == 2)) rmpa_seti(RX_AUDIO_SEL, u->tx == 1 ? 6 : 7);
        rmpa.last_tx = u->tx;
        for (int v = 0; v < 2; ++v) {
            int rx = rmpa.rx[0][v];
            float vol = rmpa.volume[0][v];
            if (rx != rmpa.last_rx[v]) { rmpa_seti(v ? RX_RX2 : RX_RX1, rx); rmpa.last_rx[v] = rx; }
            if (vol != rmpa.last_vol[v] && rmpa.xp[v ? RX_VOL2 : RX_VOL1]) {
                XPLMSetDataf(rmpa.xp[v ? RX_VOL2 : RX_VOL1], vol); rmpa.last_vol[v] = vol;
            }
        }
    }
}

/* ---------------------------------------------------------------- screen */

static void rmpa_text(MfdColor c, MfdAnchor anchor, float size, float x, float baseline, const char *s)
{
    mfd_draw_text(&rmpa_list, MFD_FONT_SZ, size, c, anchor, x, baseline, s);
}

static void rmpa_loudspeaker(float x, float y, MfdColor c)
{
    MfdPoint body[4] = {{x, y + 10}, {x + 12, y + 10}, {x + 12, y + 30}, {x, y + 30}};
    MfdPoint cone[4] = {{x + 12, y + 10}, {x + 26, y}, {x + 26, y + 40}, {x + 12, y + 30}};
    mfd_draw_polygon(&rmpa_list, body, 4, c);
    mfd_draw_polygon(&rmpa_list, cone, 4, c);
}

static void rmpa_draw_vhf(const A380XRmp *u, const A380XRmpRadios *r)
{
    char text[24], digits[8];
    for (int row = 0; row < 3; ++row) {
        float top = 8.0f + row * 134.0f, mid = top + 70.0f;
        int selected = u->row == row, mode = r->active_mode[row], smode = r->standby_mode[row];
        const A380XRmpEntry *e = &u->entry[row];
        if (row) mfd_draw_line(&rmpa_list, 12, top - 4, RMP_W - 12, top - 4, 3, MFD_COL_WHITE);
        /* transceiver ident, reverse video while transmitting on it */
        snprintf(text, sizeof(text), "VHF%d", row + 1);
        if (u->tx == row + 1) {
            mfd_draw_fill_rect(&rmpa_list, (MfdRect){58, mid - 52, 116, 44}, MFD_COL_GREEN);
            rmpa_text(MFD_COL_BLACK, MFD_ANCHOR_LEFT, 40, 62, mid - 16, text);
        } else rmpa_text(MFD_COL_WHITE, MFD_ANCHOR_LEFT, 40, 62, mid - 16, text);
        if (u->rx[row]) rmpa_loudspeaker(20, mid - 54, MFD_COL_WHITE);
        /* active */
        if (mode == A380X_RMP_MODE_DATA) snprintf(text, sizeof(text), "DATA");
        else a380x_rmp_format(r->active[row], '.', text, sizeof(text));
        rmpa_text(u->tx == row + 1 ? MFD_COL_GREEN : MFD_COL_WHITE, MFD_ANCHOR_CENTER, 58, 330, mid + 30, text);
        /* standby cell */
        {
            int invalid = a380x_rmp_entry_invalid(e);
            MfdColor c = invalid ? MFD_COL_AMBER : selected ? MFD_COL_CYAN : MFD_COL_WHITE;
            MfdRect box = {506, top + 6, 248, 120};
            if (selected) mfd_draw_stroke_rect(&rmpa_list, box, 4, MFD_COL_CYAN);
            rmpa_text(c, MFD_ANCHOR_CENTER, 28, 630, top + 40, smode == A380X_RMP_MODE_EMERGENCY ? "EMER" : "STBY");
            if (e->entered[0]) {
                snprintf(digits, sizeof(digits), "%s", e->displayed);
                snprintf(text, sizeof(text), "%.3s.%.3s", digits, digits + 3);
            } else if (smode == A380X_RMP_MODE_DATA) snprintf(text, sizeof(text), "DATA");
            else if (!r->standby[row]) snprintf(text, sizeof(text), "---.---");
            else a380x_rmp_format(r->standby[row], '.', text, sizeof(text));
            rmpa_text(c, MFD_ANCHOR_CENTER, 50, 630, top + 104, text);
        }
    }
}

static void rmpa_draw_sqwk(const A380XRmp *u, const A380XRmpRadios *r)
{
    char code[8];
    MfdColor c = u->xpdr_invalid ? MFD_COL_AMBER : u->xpdr_entry[0] ? MFD_COL_CYAN : MFD_COL_GREEN;
    rmpa_text(MFD_COL_GREEN, MFD_ANCHOR_CENTER, 44, RMP_W / 2.0f, 90, r->xpdr_auto ? "AUTO" : "STBY");
    if (u->xpdr_entry[0]) snprintf(code, sizeof(code), "%s", u->xpdr_entry);
    else snprintf(code, sizeof(code), "%04d", r->xpdr_code);
    rmpa_text(MFD_COL_WHITE, MFD_ANCHOR_RIGHT, 30, 300, 250, "SQWK");
    rmpa_text(c, MFD_ANCHOR_LEFT, 76, 330, 262, code);
    rmpa_text(MFD_COL_CYAN, MFD_ANCHOR_LEFT, 34, 62, 370, r->ident_left > 0 ? " IDENT" : "*IDENT");
}

static void rmpa_draw(void *refcon)
{
    int n = (int)(intptr_t)refcon;
    const A380XRmp *u = &rmpa.unit[n];
    GLboolean cull = glIsEnabled(GL_CULL_FACE);
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    glDisable(GL_CULL_FACE);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    if (plugin_enabled && rmpa.initialized && u->state == A380X_RMP_ON) {
        char line[64];
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_FALSE);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        mfd_draw_reset(&rmpa_list);
        if (u->page == A380X_RMP_PAGE_SQWK) rmpa_draw_sqwk(u, &rmpa.radios);
        else rmpa_draw_vhf(u, &rmpa.radios);
        mfd_draw_line(&rmpa_list, 12, 410, RMP_W - 12, 410, 3, MFD_COL_WHITE);
        if (u->message[0]) rmpa_text(MFD_COL_WHITE, MFD_ANCHOR_LEFT, 32, 20, 452, u->message);
        else if (u->page == A380X_RMP_PAGE_VHF) {
            snprintf(line, sizeof(line), "SQUAWK: %04d", rmpa.radios.xpdr_code);
            rmpa_text(MFD_COL_WHITE, MFD_ANCHOR_LEFT, 32, 20, 452, line);
        }
        /* the 768 x 1024 canvas transform: y = oy + (1024 - y) * s; the device is 466 high */
        mfd_gl_draw(&rmpa_list, 0, (float)RMP_H - (float)MFD_H, 1, 0);
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    }
    glColor4f(1, 1, 1, 1);
    if (cull) glEnable(GL_CULL_FACE);
}

static float rmpa_brightness(float rheo, float ambient, float bus, void *refcon)
{
    const A380XRmp *u = &rmpa.unit[(int)(intptr_t)refcon];
    (void)rheo; (void)ambient; (void)bus;
    return plugin_enabled && u->state == A380X_RMP_ON ? fmaxf(u->brightness, 0.05f) : 0.0f;
}

/* ---------------------------------------------------------------- datarefs */

static float rmpa_ref_value(const RmpRef *r)
{
    const A380XRmp *u = &rmpa.unit[r->rmp];
    switch (r->kind) {
    case RR_RX: return (float)rmpa.rx[r->rmp][r->index];
    case RR_VOLUME: return rmpa.volume[r->rmp][r->index];
    case RR_BRIGHTNESS: return u->brightness;
    case RR_STATE: return (float)u->state;
    case RR_PAGE: return (float)u->page;
    case RR_STANDBY_LED: return plugin_enabled && u->state == A380X_RMP_OFF_STANDBY ? 1.0f : 0.0f;
    case RR_DEVICE_ACTIVE: return (float)rmpa.bound[r->rmp];
    case RR_CALL_LAMP: return plugin_enabled && u->state == A380X_RMP_ON && u->tx == r->index + 1 ? 1.0f : 0.0f;
    default: return 0.0f;
    }
}
static float rmpa_getf(void *refcon) { return rmpa_ref_value(refcon); }
static int rmpa_geti_ref(void *refcon) { return (int)lroundf(rmpa_ref_value(refcon)); }
static void rmpa_setf(void *refcon, float value)
{
    RmpRef *r = refcon;
    if (!plugin_enabled || !isfinite(value)) return;
    if (r->kind == RR_BRIGHTNESS) rmpa.unit[r->rmp].brightness = fminf(1, fmaxf(0, value));
    else if (r->kind == RR_VOLUME) rmpa.volume[r->rmp][r->index] = fminf(1, fmaxf(0, value));
    else if (r->kind == RR_RX) { rmpa.rx[r->rmp][r->index] = value >= 0.5f; if (r->index < 3) rmpa.unit[r->rmp].rx[r->index] = value >= 0.5f; }
}
static void rmpa_seti_ref(void *refcon, int value) { rmpa_setf(refcon, (float)value); }

static void rmpa_add_ref(int kind, int rmp, int index, const char *suffix)
{
    RmpRef *r = &rmpa.refs[rmpa.ref_count++];
    r->kind = kind; r->rmp = rmp; r->index = index;
    snprintf(r->name, sizeof(r->name), RMP_PREFIX "%d/%s", rmp + 1, suffix);
}

/* Order matches tools/cockpit_feedback_contract.py rmp_datarefs(). */
static void rmpa_build_refs(void)
{
    char name[64];
    rmpa.ref_count = 0;
    for (int n = 0; n < A380X_RMP_COUNT; ++n) {
        for (int k = 0; k < RMP_KNOB_COUNT; ++k) {
            snprintf(name, sizeof(name), "%s_rx", rmp_knob_names[k]); rmpa_add_ref(RR_RX, n, k, name);
            snprintf(name, sizeof(name), "%s_volume", rmp_knob_names[k]); rmpa_add_ref(RR_VOLUME, n, k, name);
        }
        rmpa_add_ref(RR_BRIGHTNESS, n, 0, "brightness");
        rmpa_add_ref(RR_STATE, n, 0, "state");
        rmpa_add_ref(RR_PAGE, n, 0, "page");
        rmpa_add_ref(RR_STANDBY_LED, n, 0, "standby_led");
        rmpa_add_ref(RR_DEVICE_ACTIVE, n, 0, "device_active");
        for (int v = 0; v < 3; ++v) {
            snprintf(name, sizeof(name), "lamp/vhf%d_call", v + 1); rmpa_add_ref(RR_CALL_LAMP, n, v, name);
        }
    }
}

/* ---------------------------------------------------------------- commands */

static int rmpa_command(XPLMCommandRef handle, XPLMCommandPhase phase, void *refcon)
{
    RmpCommand *c = refcon;
    A380XRmp *u = &rmpa.unit[c->rmp];
    (void)handle;
    if (!plugin_enabled || !rmpa.initialized) return 1;
    if (c->kind == 0) {
        int key = rmp_key_enum(c->index);
        if (phase == xplm_CommandBegin) {
            int ident_before = rmpa.radios.ident_left > 0;
            a380x_rmp_key(u, &rmpa.radios, key);
            if (!ident_before && rmpa.radios.ident_left > 0 && rmpa.ident) XPLMCommandOnce(rmpa.ident);
            rmpa_sync_radios();
        } else if (phase == xplm_CommandEnd) a380x_rmp_key_up(u, key);
        return 1;   /* the cockpit-feedback key travel handler still runs */
    }
    if (phase != xplm_CommandBegin) return 1;
    switch (c->kind) {
    case 1:
        rmpa.rx[c->rmp][c->index] = !rmpa.rx[c->rmp][c->index];
        if (c->index < 3) u->rx[c->index] = rmpa.rx[c->rmp][c->index];
        break;
    case 2: case 3:
        rmpa.volume[c->rmp][c->index] = fminf(1, fmaxf(0, rmpa.volume[c->rmp][c->index] + (c->kind == 2 ? 0.1f : -0.1f)));
        break;
    case 4: case 5:
        u->brightness = fminf(1, fmaxf(0, roundf((u->brightness + (c->kind == 4 ? 0.1f : -0.1f)) * 10) / 10));
        break;
    default: break;
    }
    return 1;
}

static int rmpa_add_command(int rmp, int kind, int index, const char *suffix, const char *description)
{
    char name[96];
    RmpCommand *c = &rmpa.commands[rmpa.command_count];
    snprintf(name, sizeof(name), RMP_PREFIX "%d/%s", rmp + 1, suffix);
    c->rmp = rmp; c->kind = kind; c->index = index;
    c->handle = XPLMFindCommand(name);
    if (!c->handle) c->handle = XPLMCreateCommand(name, description);
    if (!c->handle) return 0;
    XPLMRegisterCommandHandler(c->handle, rmpa_command, 1, c);
    ++rmpa.command_count;
    return 1;
}

/* ---------------------------------------------------------------- lifecycle */

static void rmpa_shutdown(void)
{
    for (int i = 0; i < rmpa.command_count; ++i)
        XPLMUnregisterCommandHandler(rmpa.commands[i].handle, rmpa_command, 1, &rmpa.commands[i]);
    for (int i = 0; i < rmpa.ref_count; ++i) if (rmpa.refs[i].handle) XPLMUnregisterDataAccessor(rmpa.refs[i].handle);
    for (int n = 0; n < A380X_RMP_COUNT; ++n) if (rmpa.device[n] && mfda_av.loaded) mfda_av.destroy(rmpa.device[n]);
    memset(&rmpa, 0, sizeof(rmpa));
}

static void rmpa_reset(void)
{
    a380x_rmp_radios_init(&rmpa.radios);
    for (int n = 0; n < A380X_RMP_COUNT; ++n) {
        a380x_rmp_init(&rmpa.unit[n], n);
        for (int k = 0; k < RMP_KNOB_COUNT; ++k) { rmpa.rx[n][k] = k < 3 ? rmpa.unit[n].rx[k] : 0; rmpa.volume[n][k] = 0.6f; }
    }
    for (int k = 0; k < 4; ++k) rmpa.last_khz[k] = -1;   /* adopt X-Plane's COM frequencies */
    rmpa.last_xpdr = -1; rmpa.last_tx = -1; rmpa.last_rx[0] = rmpa.last_rx[1] = -1;
    rmpa.last_vol[0] = rmpa.last_vol[1] = -1;
    if (rmpa.initialized) rmpa_sync_radios();
}

/* XPluginStart, before the aircraft OBJs bind the datarefs and devices. */
static int rmpa_init(void)
{
    char name[96];
    memset(&rmpa, 0, sizeof(rmpa));
    for (int i = 0; i < RX_COUNT; ++i) rmpa.xp[i] = XPLMFindDataRef(rmp_xp_names[i]);
    rmpa.ident = XPLMFindCommand("sim/transponder/transponder_ident");
    rmpa_reset();
    rmpa_build_refs();
    for (int i = 0; i < rmpa.ref_count; ++i) {
        RmpRef *r = &rmpa.refs[i];
        int writable = r->kind == RR_RX || r->kind == RR_VOLUME || r->kind == RR_BRIGHTNESS;
        if (XPLMFindDataRef(r->name)) { rmpa_shutdown(); return 0; }
        r->handle = XPLMRegisterDataAccessor(r->name, xplmType_Int | xplmType_Float, writable,
                                             rmpa_geti_ref, writable ? rmpa_seti_ref : NULL, rmpa_getf,
                                             writable ? rmpa_setf : NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                             NULL, r, writable ? r : NULL);
        if (!r->handle) { rmpa_shutdown(); return 0; }
    }
    for (int n = 0; n < A380X_RMP_COUNT; ++n) {
        for (int k = 0; k < RMP_KEY_COUNT; ++k) {
            snprintf(name, sizeof(name), "RMP %d %s key", n + 1, rmp_key_names[k]);
            if (!rmpa_add_command(n, 0, k, rmp_key_names[k], name)) { rmpa_shutdown(); return 0; }
        }
        for (int k = 0; k < RMP_KNOB_COUNT; ++k) {
            static const char *const acts[3] = {"push", "up", "down"};
            for (int a = 0; a < 3; ++a) {
                char suffix[48];
                snprintf(suffix, sizeof(suffix), "%s_knob_%s", rmp_knob_names[k], acts[a]);
                snprintf(name, sizeof(name), "RMP %d %s reception knob %s", n + 1, rmp_knob_names[k], acts[a]);
                if (!rmpa_add_command(n, 1 + a, k, suffix, name)) { rmpa_shutdown(); return 0; }
            }
        }
        if (!rmpa_add_command(n, 4, 0, "brightness_up", "RMP OFF / BRT brighter") ||
            !rmpa_add_command(n, 5, 0, "brightness_down", "RMP OFF / BRT dimmer")) { rmpa_shutdown(); return 0; }
        if (mfda_av.loaded) {
            XPLMCreateAvionics_t p;
            memset(&p, 0, sizeof(p));
            p.structSize = (int)sizeof(p);
            p.screenWidth = p.bezelWidth = RMP_W;
            p.screenHeight = p.bezelHeight = RMP_H;
            p.drawOnDemand = 0;
            p.drawCallback = rmpa_draw;
            p.brightnessCallback = rmpa_brightness;
            p.deviceID = (char *)rmp_device_ids[n];
            p.deviceName = (char *)rmp_device_names[n];
            p.refcon = (void *)(intptr_t)n;
            rmpa.device[n] = mfda_av.create(&p);
        }
    }
    rmpa.now = XPLMGetElapsedTime();
    rmpa.initialized = 1;
    snprintf(name, sizeof(name), "A380X RMP: 3 panels, %d commands, %d datarefs, devices %s\n", rmpa.command_count,
             rmpa.ref_count, mfda_av.loaded ? "created" : "unavailable");
    XPLMDebugString(name);
    return 1;
}

static void rmpa_update(void)
{
    float now, dt;
    if (!rmpa.initialized) return;
    now = XPLMGetElapsedTime();
    dt = now - rmpa.now;
    rmpa.now = now;
    for (int n = 0; n < A380X_RMP_COUNT; ++n) {
        int powered = a380x_power_any(A380X_ELN_BIT(n == 2 ? A380X_ELN_DC1 : A380X_ELN_DC_ESS), 1);
        a380x_rmp_update(&rmpa.unit[n], &rmpa.radios, plugin_enabled && powered, 0, dt);
        rmpa.bound[n] = rmpa.device[n] && mfda_av.loaded && mfda_av.is_bound(rmpa.device[n]);
    }
    a380x_rmp_radios_tick(&rmpa.radios, rmpa.unit, A380X_RMP_COUNT, dt);
    rmpa_sync_radios();
}
