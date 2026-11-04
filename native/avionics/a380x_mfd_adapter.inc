/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * X-Plane boundary of the two A380X MFDs (WP-J, docs/mfd-spec/08-xplane-cockpit-device.md and
 * 09-port-integration.md): plugin cockpit devices a380x.xp12.mfd.captain / .first_officer
 * created through XPLMFindSymbol (no hard XPLM410 imports), drawOnDemand redraws only when a
 * display list changed, 3D touch / drag / wheel / hover routed into the simulator-independent
 * MFD core (a380x_mfd.h), brightness from the DU MFD knobs and power, popup / pop-out commands
 * and menu, PC keyboard capture (popup keyboard focus or an after-windows key sniffer, only
 * while a field is being edited), the KCCU commands of spec 01 section 12.5 and the
 * MFD / KCCU datarefs. When the avionics API is missing or a device cannot be created the MFD
 * opens in a plain XPLM window instead (spec 08 section 10.6, F0).
 *
 * Unity-build include of a380x_avionics.c. Needs, before inclusion: plugin_enabled, ARRAY_LEN,
 * du_channels / DU_CHANNELS_PER_SIDE / DU_MFD (a380x_du.inc), mfd_gl_draw (a380x_mfd_render.inc)
 * and a380x_mfd.h. Statics are prefixed mfda_ (MFD) and kccu_ (KCCU). The FMS, navdata and
 * aircraft-systems side lives in a380x_fms_adapter.inc, which fills mfda_inputs' FMS fields
 * and supplies the A380XMfdSystemsHost.
 */
#include "a380x_du_scale.h"
#if !defined(XPLM400) || !defined(XPLM410)
#error "MFD adapter needs -DXPLM400=1 -DXPLM410=1"
#endif
#include "XPLMMenus.h"
#include "a380x_elec_power.h"
/* CDS boot / self-test screen hook (a380x_avionics.c defines it before this include). */
#ifndef MFDA_CDS_OVERLAY
#define MFDA_CDS_OVERLAY(side) 0
#endif
#define MFDA_DU(side) ((side) ? A380X_ELN_DU_FO_MFD : A380X_ELN_DU_CAPT_MFD)

void a380x_mfd_pages_register(void);
int a380x_mfd_pages_linked(char *names, size_t n);
#if defined(A380X_MFD_HAVE_PAGE_FCUBKUP)
void a380x_mfd_page_fcubkup_release_held(A380XMfd *m);
#endif

/* ---- Avionics entry points, resolved at run time (spec 08 section 8.3) -------------------- */

typedef XPLMAvionicsID (*MfdaPfnCreate)(XPLMCreateAvionics_t *);
typedef void (*MfdaPfnHandle)(XPLMAvionicsID);
typedef int (*MfdaPfnQuery)(XPLMAvionicsID);
typedef void (*MfdaPfnSetInt)(XPLMAvionicsID, int);
typedef void (*MfdaPfnSetFloat)(XPLMAvionicsID, float);
typedef int (*MfdaPfnCursorOver)(XPLMAvionicsID, int *, int *);
typedef void (*MfdaPfnGetGeom)(XPLMAvionicsID, int *, int *, int *, int *);
typedef void (*MfdaPfnSetGeom)(XPLMAvionicsID, int, int, int, int);

static struct {
    MfdaPfnCreate create;
    MfdaPfnHandle destroy, needs_drawing, pop_out, take_keyboard;
    MfdaPfnQuery is_bound, is_popup_visible, is_popped_out, has_keyboard;
    MfdaPfnSetInt set_popup_visible;
    MfdaPfnSetFloat set_rheo;
    MfdaPfnCursorOver cursor_over;
    MfdaPfnGetGeom get_geometry;
    MfdaPfnSetGeom set_geometry;
    int xplm_version;
    int loaded;
} mfda_av;

static int mfda_av_load(void)
{
    int xp_version = 0;
    XPLMHostApplicationID host;
    memset(&mfda_av, 0, sizeof(mfda_av));
    XPLMGetVersions(&xp_version, &mfda_av.xplm_version, &host);
    if (mfda_av.xplm_version < 410) return 0; /* SDK 4.1 = X-Plane 12.1.0 */
#define MFDA_AV_LOAD(field, type, name) \
    if ((mfda_av.field = (type)XPLMFindSymbol(name)) == NULL) return 0
    MFDA_AV_LOAD(create, MfdaPfnCreate, "XPLMCreateAvionicsEx");
    MFDA_AV_LOAD(destroy, MfdaPfnHandle, "XPLMDestroyAvionics");
    MFDA_AV_LOAD(needs_drawing, MfdaPfnHandle, "XPLMAvionicsNeedsDrawing");
    MFDA_AV_LOAD(pop_out, MfdaPfnHandle, "XPLMPopOutAvionics");
    MFDA_AV_LOAD(take_keyboard, MfdaPfnHandle, "XPLMTakeAvionicsKeyboardFocus");
    MFDA_AV_LOAD(is_bound, MfdaPfnQuery, "XPLMIsAvionicsBound");
    MFDA_AV_LOAD(is_popup_visible, MfdaPfnQuery, "XPLMIsAvionicsPopupVisible");
    MFDA_AV_LOAD(is_popped_out, MfdaPfnQuery, "XPLMIsAvionicsPoppedOut");
    MFDA_AV_LOAD(has_keyboard, MfdaPfnQuery, "XPLMHasAvionicsKeyboardFocus");
    MFDA_AV_LOAD(set_popup_visible, MfdaPfnSetInt, "XPLMSetAvionicsPopupVisible");
    MFDA_AV_LOAD(set_rheo, MfdaPfnSetFloat, "XPLMSetAvionicsBrightnessRheo");
    MFDA_AV_LOAD(cursor_over, MfdaPfnCursorOver, "XPLMIsCursorOverAvionics");
    MFDA_AV_LOAD(get_geometry, MfdaPfnGetGeom, "XPLMGetAvionicsGeometry");
    MFDA_AV_LOAD(set_geometry, MfdaPfnSetGeom, "XPLMSetAvionicsGeometry");
#undef MFDA_AV_LOAD
    mfda_av.loaded = 1;
    return 1;
}

/* ---- Devices --------------------------------------------------------------------------------- */

enum {
    MFDA_BEZEL_SIDE = 16, MFDA_BEZEL_TOP = 40,
    MFDA_BEZEL_W = MFD_W + 2 * MFDA_BEZEL_SIDE, MFDA_BEZEL_H = MFD_H + MFDA_BEZEL_SIDE + MFDA_BEZEL_TOP
};
/* The cockpit device is A380X_MFD_SCALE times denser than the 768x1024 canvas (a380x_du_scale.h):
 * the bezel layout above is in canvas units, the device and its texels in pixels. */
#define MFDA_PX(v) A380X_MFD_PX(v)

typedef struct {
    MfdSide side;
    const char *device_id;       /* must match ATTR_cockpit_device / ATTR_manip_device */
    const char *device_name;     /* popped-out window title */
    const char *seat;            /* "CAPT" / "F/O" */
    const char *side_name;       /* "captain" / "first_officer" */
    char popup_name[64], popout_name[64];
    XPLMAvionicsID handle;
    XPLMWindowID window;         /* F0 fallback window (no device) */
    XPLMCommandRef popup_cmd, popout_cmd;
    int pointer_inside, last_x, last_y, cursor_seen, touching;
    int du_on;
    int bound_warned;
    float since_create;
    float last_knob;
    uint64_t list_hash;
    int force_redraw;
    int prev_wants;
    MfdId prev_focus;
    int prev_popup;
    int keyboard_captured;
    int click_held;              /* KCCU LBUTTON held (button down sent) */
    int scissor_logged;
    /* Read-only input diagnostics; never an alternative/synthetic input path. */
    int device_bound, touch_down_count, touch_up_count;
    int cursor_x, cursor_y, cursor_visible, menu_open, events_dropped;
    int last_down_x, last_down_y, last_up_x, last_up_y;
    int press_target, click_target, click_count, menu_open_count;
    int trace_frames;
} MfdaDevice;

static MfdaDevice mfda_dev[MFD_SIDE_COUNT] = {
    {.side=MFD_CAPT, .device_id="a380x.xp12.mfd.captain", .device_name="A380X MFD CAPT",
     .seat="CAPT", .side_name="captain", .popup_name="a380x/xp12/mfd/captain/popup",
     .popout_name="a380x/xp12/mfd/captain/popout", .last_x=-1, .last_y=-1, .last_knob=-1, .force_redraw=1},
    {.side=MFD_FO, .device_id="a380x.xp12.mfd.first_officer", .device_name="A380X MFD F/O",
     .seat="F/O", .side_name="first_officer", .popup_name="a380x/xp12/mfd/first_officer/popup",
     .popout_name="a380x/xp12/mfd/first_officer/popout", .last_x=-1, .last_y=-1, .last_knob=-1, .force_redraw=1},
};

/* ---- Core state ------------------------------------------------------------------------------ */

static A380XMfd mfda_mfd;            /* two sides, two display lists: static, never per frame */
static A380XMfdInputs mfda_inputs;   /* FMS fields filled by a380x_fms_adapter.inc each frame */
static int mfda_initialized;
static int mfda_devices_created;     /* devices exist (else F0 fallback windows) */
static int mfda_pc_capture_pref = 1; /* a380x/xp12/mfd/pc_keyboard_capture: 0 never, 1 auto, 2 FBW */
static int mfda_capture_side = -1;   /* side owning the after-windows key sniffer, -1 none */
static int mfda_sniffer_registered;
static XPLMMenuID mfda_menu, mfda_menu_parent;
static int mfda_menu_item = -1;
static int mfda_kccu_kbd_on[MFD_SIDE_COUNT] = {1, 1};
static int mfda_kccu_ccd_on[MFD_SIDE_COUNT] = {1, 1};
static int mfda_powered;

static float mfda_clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

static int mfda_side_ok(int s) { return s >= 0 && s < MFD_SIDE_COUNT; }

static float mfda_du_knob(int s)
{
    return mfda_side_ok(s) ? du_channels[s * DU_CHANNELS_PER_SIDE + DU_MFD].level : 0.0f;
}

/* Device texel (x right, y up, origin bottom-left) -> canvas pixel centre (y down). */
static void mfda_to_canvas(int x, int y, float *cx, float *cy)
{
    *cx = mfda_clampf(A380X_DU_LOGICAL(x, A380X_MFD_SCALE), 0.0f, (float)MFD_W);
    *cy = mfda_clampf((float)MFD_H - A380X_DU_LOGICAL(y, A380X_MFD_SCALE), 0.0f, (float)MFD_H);
}

static int mfda_popup_shown(const MfdaDevice *d)
{
    if (d->handle && mfda_av.loaded)
        return mfda_av.is_popup_visible(d->handle) || mfda_av.is_popped_out(d->handle);
    return d->window != NULL && XPLMGetWindowIsVisible(d->window);
}

static void mfda_capture_release(void) { mfda_capture_side = -1; }

/* FNV-1a over the display list content: XPLMAvionicsNeedsDrawing only on a change. */
static uint64_t mfda_hash_bytes(uint64_t h, const void *data, size_t n)
{
    const unsigned char *p = (const unsigned char *)data;
    size_t i;
    for (i = 0; i < n; ++i) { h ^= p[i]; h *= 1099511628211ull; }
    return h;
}

static uint64_t mfda_list_hash(const MfdDrawList *dl, int du_on)
{
    uint64_t h = 1469598103934665603ull ^ (uint64_t)(du_on ? 1u : 2u);
    if (!dl) return h;
    h = mfda_hash_bytes(h, dl->cmds, (size_t)dl->cmd_count * sizeof(dl->cmds[0]));
    h = mfda_hash_bytes(h, dl->points, (size_t)dl->point_count * sizeof(dl->points[0]));
    h = mfda_hash_bytes(h, dl->text, (size_t)dl->text_len);
    h = mfda_hash_bytes(h, dl->clips, (size_t)dl->clip_count * sizeof(dl->clips[0]));
    return h;
}

/* ---- Draw callbacks (GL only; the display list is built in the flight loop) ----------------- */

/* Scissor boxes are framebuffer coordinates: only use them when the device framebuffer is the
 * plain y-up device-size target (risk R2 of spec 08: Metal/Vulkan reverse-Y). */
static int mfda_scissor_ok(MfdaDevice *d)
{
    GLint vp[4] = {0, 0, 0, 0};
    GLfloat proj[16], model[16];
    int ok;
    glGetIntegerv(GL_VIEWPORT, vp);
    glGetFloatv(GL_PROJECTION_MATRIX, proj);
    glGetFloatv(GL_MODELVIEW_MATRIX, model);
    ok = vp[0] == 0 && vp[1] == 0 && vp[2] == MFDA_PX(MFD_W) && vp[3] == MFDA_PX(MFD_H) && proj[5] * model[5] > 0.0f;
    if (!ok && !d->scissor_logged) {
        XPLMDebugString("A380X MFD: device framebuffer is not a plain y-up device-size target; clipping uses no scissor\n");
        d->scissor_logged = 1;
    }
    return ok;
}

static void mfda_screen_draw(void *refcon)
{
    MfdaDevice *d = (MfdaDevice *)refcon;
    GLboolean cull = glIsEnabled(GL_CULL_FACE);
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    glDisable(GL_CULL_FACE);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);                     /* X-Plane does not clear the device FBO */
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_FALSE); /* keep alpha = 1: the screen never blends */
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    if (plugin_enabled && d && !d->du_on) {
        /* Thales boot / self-test (A380 network): nothing else on the screen */
        glPushMatrix(); glScalef(A380X_MFD_SCALE, A380X_MFD_SCALE, 1);
        (void)MFDA_CDS_OVERLAY(d->side);
        glPopMatrix();
    } else if (plugin_enabled && d && d->du_on) {
        const MfdDrawList *dl=a380x_mfd_display_list(&mfda_mfd,d->side);
        if(xpkit_draw_mfd(d->side,0,0,A380X_MFD_SCALE))mfd_gl_cursor(dl,0,0,A380X_MFD_SCALE);
        else mfd_gl_draw(dl,0,0,A380X_MFD_SCALE,mfda_scissor_ok(d));
    }
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    if (cull) glEnable(GL_CULL_FACE);
}

static void mfda_bezel_draw(float r, float g, float b, void *refcon)
{
    (void)refcon;
    XPLMSetGraphicsState(0, 0, 0, 0, 0, 0, 0);
    glColor3f(0.18f * r, 0.19f * g, 0.20f * b);
    glPushMatrix(); glScalef(A380X_MFD_SCALE, A380X_MFD_SCALE, 1);   /* bezel layout in canvas units */
    glBegin(GL_QUADS);
    glVertex2i(0, 0); glVertex2i(MFDA_BEZEL_W, 0); glVertex2i(MFDA_BEZEL_W, MFDA_BEZEL_H); glVertex2i(0, MFDA_BEZEL_H);
    glColor3f(0.0f, 0.0f, 0.0f);
    glVertex2i(MFDA_BEZEL_SIDE, MFDA_BEZEL_SIDE); glVertex2i(MFDA_BEZEL_SIDE + MFD_W, MFDA_BEZEL_SIDE);
    glVertex2i(MFDA_BEZEL_SIDE + MFD_W, MFDA_BEZEL_SIDE + MFD_H); glVertex2i(MFDA_BEZEL_SIDE, MFDA_BEZEL_SIDE + MFD_H);
    glEnd();
    glPopMatrix();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
}

/* ---- Mouse callbacks (3D cockpit through ATTR_manip_device, and the popup) ------------------ */

/* (0,0) or any texel outside the device screen: the event carries no usable position. */
static int mfda_positionless(int x, int y)
{
    return (x == 0 && y == 0) || x < 0 || y < 0 || x >= MFDA_PX(MFD_W) || y >= MFDA_PX(MFD_H);
}

static int mfda_screen_touch(int x, int y, XPLMMouseStatus status, void *refcon)
{
    MfdaDevice *d = (MfdaDevice *)refcon;
    float cx, cy;
    int raw_x = x, raw_y = y;
    if (!plugin_enabled || !d) return 1;
    if (status == xplm_MouseDown) ++d->touch_down_count;
    else if (status == xplm_MouseUp) ++d->touch_up_count;
    /* Real 3D cockpit releases can arrive with no position: the simulator
     * supplies (0,0), or (since the X-Plane 12.x update) an off-screen sentinel
     * such as (-16,-16), even after a press on a cyan field. Moving there first
     * makes the last-frame hit test reject the click, so keyboard focus is
     * never acquired. Keep the latest position of this held gesture. Drag
     * callbacks/cursor tracking still update it, and on-screen release positions
     * (including a release over a different widget) remain authoritative. */
    if (status != xplm_MouseDown && d->touching && mfda_positionless(x, y) &&
        d->last_x >= 0 && d->last_y >= 0) {
        x = d->last_x;
        y = d->last_y;
    }
    mfda_to_canvas(x, y, &cx, &cy);
    if (status == xplm_MouseDown || status == xplm_MouseUp) {
        char line[256];
        if (status == xplm_MouseDown) { d->last_down_x = (int)cx; d->last_down_y = (int)cy; }
        else { d->last_up_x = (int)cx; d->last_up_y = (int)cy; }
        d->trace_frames = 6;
        snprintf(line, sizeof(line), "A380X MFD input %s %s texel=%d,%d canvas=%d,%d pending=%d power=%d\n",
            d->seat, status == xplm_MouseDown ? "DOWN" : "UP", raw_x, raw_y, (int)cx, (int)cy,
            mfda_mfd.sides[d->side].ui.ev_count, d->du_on);
        XPLMDebugString(line);
    }
    d->last_x = x; d->last_y = y;
    d->cursor_seen = 1; /* A click is also direct evidence that the pointer is over this device. */
    if (!d->du_on) { d->touching = status == xplm_MouseDown ? 1 : status == xplm_MouseUp ? 0 : d->touching; return 1; }
    mfd_pointer_move(&mfda_mfd, d->side, cx, cy);
    d->pointer_inside = 1;
    if (status == xplm_MouseDown) { mfd_pointer_button(&mfda_mfd, d->side, 1); d->touching = 1; }
    else if (status == xplm_MouseUp) { mfd_pointer_button(&mfda_mfd, d->side, 0); d->touching = 0; }
    return 1; /* always consume: returning 0 on MouseDown suppresses Drag/Up */
}

static int mfda_screen_right_touch(int x, int y, XPLMMouseStatus status, void *refcon)
{
    (void)x; (void)y; (void)status; (void)refcon;
    return 0; /* no secondary button on the MFD: keep right-drag view look */
}

static int mfda_screen_scroll(int x, int y, int wheel, int clicks, void *refcon)
{
    MfdaDevice *d = (MfdaDevice *)refcon;
    float cx, cy;
    if (!plugin_enabled || !d || !d->du_on || wheel != 0 || clicks == 0) return 0;
    mfda_to_canvas(x, y, &cx, &cy);
    mfd_pointer_move(&mfda_mfd, d->side, cx, cy);
    d->pointer_inside = 1; d->last_x = x; d->last_y = y;
    mfd_pointer_wheel(&mfda_mfd, d->side, clicks);
    return 1;
}

static XPLMCursorStatus mfda_screen_cursor(int x, int y, void *refcon)
{
    MfdaDevice *d = (MfdaDevice *)refcon;
    if (!plugin_enabled || !d) return xplm_CursorDefault;
    d->cursor_seen = 1;
    if (d->du_on && !mfda_positionless(x, y) && (!d->pointer_inside || x != d->last_x || y != d->last_y)) {
        float cx, cy;
        mfda_to_canvas(x, y, &cx, &cy);
        mfd_pointer_move(&mfda_mfd, d->side, cx, cy);
        d->pointer_inside = 1; d->last_x = x; d->last_y = y;
    }
    /* The MFD draws its own yellow KCCU cursor (spec 01 section 11.1). */
    return d->du_on ? xplm_CursorHidden : xplm_CursorDefault;
}

static int mfda_bezel_click(int x, int y, XPLMMouseStatus status, void *refcon)
{
    (void)x; (void)y; (void)status; (void)refcon;
    return 1;
}

static int mfda_bezel_scroll(int x, int y, int wheel, int clicks, void *refcon)
{
    (void)x; (void)y; (void)wheel; (void)clicks; (void)refcon;
    return 1;
}

static XPLMCursorStatus mfda_bezel_cursor(int x, int y, void *refcon)
{
    (void)x; (void)y; (void)refcon;
    return xplm_CursorDefault;
}

/* ---- Keyboard: one map for the popup keyboardCallback, the fallback window and the sniffer -- */

/* Returns 1 when the key belongs to the MFD. */
static int mfda_map_key(char ch, char vkey_signed, MfdKey *key, char *out)
{
    unsigned char vkey = (unsigned char)vkey_signed; /* XPLM vkeys are declared signed */
    *out = 0;
    if (vkey >= XPLM_VK_A && vkey <= XPLM_VK_Z) { *key = MFD_KEY_CHAR; *out = (char)('A' + vkey - XPLM_VK_A); return 1; }
    if (vkey >= XPLM_VK_0 && vkey <= XPLM_VK_9 && ch >= '0' && ch <= '9') { *key = MFD_KEY_CHAR; *out = ch; return 1; }
    if (vkey >= XPLM_VK_NUMPAD0 && vkey <= XPLM_VK_NUMPAD9) { *key = MFD_KEY_CHAR; *out = (char)('0' + vkey - XPLM_VK_NUMPAD0); return 1; }
    if (vkey == XPLM_VK_BACK || vkey == XPLM_VK_DELETE) { *key = MFD_KEY_BACKSPACE; return 1; }
    if (vkey == XPLM_VK_RETURN || vkey == XPLM_VK_ENTER || vkey == XPLM_VK_NUMPAD_ENT) { *key = MFD_KEY_ENT; return 1; }
    if (vkey == XPLM_VK_ESCAPE) { *key = MFD_KEY_ESC; return 1; }
    if (vkey == XPLM_VK_UP) { *key = MFD_KEY_UP; return 1; }
    if (vkey == XPLM_VK_DOWN) { *key = MFD_KEY_DOWN; return 1; }
    if (vkey == XPLM_VK_LEFT) { *key = MFD_KEY_LEFT; return 1; }
    if (vkey == XPLM_VK_RIGHT) { *key = MFD_KEY_RIGHT; return 1; }
    if (vkey == XPLM_VK_DECIMAL) { *key = MFD_KEY_CHAR; *out = '.'; return 1; }
    if (vkey == XPLM_VK_ADD) { *key = MFD_KEY_CHAR; *out = '+'; return 1; }
    if (vkey == XPLM_VK_SUBTRACT) { *key = MFD_KEY_CHAR; *out = '-'; return 1; }
    if (vkey == XPLM_VK_DIVIDE) { *key = MFD_KEY_CHAR; *out = '/'; return 1; }
    switch (ch) { /* layout-dependent symbols by character */
    case ' ': case '/': case '.': case '+': case '-': *key = MFD_KEY_CHAR; *out = ch; return 1;
    case ',': *key = MFD_KEY_CHAR; *out = '.'; return 1;
    default: return 0;
    }
}

/* Popup / pop-out keyboard (only while the popup has avionics keyboard focus). */
static int mfda_popup_keyboard(char ch, XPLMKeyFlags flags, char vkey, void *refcon, int losing_focus)
{
    MfdaDevice *d = (MfdaDevice *)refcon;
    MfdKey key;
    char out;
    if (losing_focus || !d || !plugin_enabled) return 1;
    if (flags & (xplm_ControlFlag | xplm_OptionAltFlag)) return 0;
    if (!mfda_map_key(ch, vkey, &key, &out)) return 0;
    if ((flags & xplm_DownFlag) && d->du_on) mfd_key(&mfda_mfd, d->side, key, out);
    return 1;
}

/* 3D cockpit: registered AFTER windows (inBeforeWindows = 0), passive unless a side captures.
 * Consumes both the down and the up event of every mapped key while capturing. */
static int mfda_key_sniffer(char ch, XPLMKeyFlags flags, char vkey, void *refcon)
{
    MfdKey key;
    char out;
    int s = mfda_capture_side;
    (void)refcon;
    if (!plugin_enabled || !mfda_side_ok(s)) return 1;             /* not capturing: pass */
    if (flags & (xplm_ControlFlag | xplm_OptionAltFlag)) return 1;  /* keep sim shortcuts */
    if (!mfda_map_key(ch, vkey, &key, &out)) return 1;              /* F-keys and Tab: pass */
    if (flags & xplm_DownFlag) {
        mfd_key(&mfda_mfd, (MfdSide)s, key, out);
        if (key == MFD_KEY_ENT || key == MFD_KEY_ESC) mfda_capture_release();
    }
    return 0;
}

/* ---- Brightness (3D screen luminance). 0 makes the 3D screen black. ------------------------ */

static float mfda_brightness_law(int du_on, float knob, float ambient)
{
    if (!du_on || !(knob > 0.0f)) return 0.0f;
    /* ambient = level that makes the screen usable in the current light; scaled by the DU knob. */
    return mfda_clampf(knob * (ambient > 0.0f ? ambient : 1.0f), 0.0f, 1.0f);
}

static float mfda_screen_brightness(float rheo, float ambient, float bus_ratio, void *refcon)
{
    MfdaDevice *d = (MfdaDevice *)refcon;
    (void)rheo; (void)bus_ratio; /* MFD power is decided by the aircraft logic, not the XP bus */
    if (!d || !plugin_enabled) return 0.0f;
    return mfda_brightness_law(d->du_on || a380x_power_du_lit(MFDA_DU(d->side), 0), mfda_du_knob(d->side), ambient);
}

/* ---- F0 fallback: the MFD in a plain XPLM window (no device API) -------------------------- */

static int mfda_window_canvas(MfdaDevice *d, int x, int y, float *cx, float *cy, float *scale_out,
                              int *ox_out, int *oy_out)
{
    int l, t, r, b, w, h, ox, oy;
    float scale;
    XPLMGetWindowGeometry(d->window, &l, &t, &r, &b);
    w = r - l; h = t - b;
    if (w <= 0 || h <= 0) return 0;
    scale = fminf((float)w / (float)MFD_W, (float)h / (float)MFD_H);
    ox = l + (int)(((float)w - scale * (float)MFD_W) * 0.5f);
    oy = b + (int)(((float)h - scale * (float)MFD_H) * 0.5f);
    if (cx) *cx = mfda_clampf(((float)x - (float)ox) / scale, 0.0f, (float)MFD_W);
    if (cy) *cy = mfda_clampf((float)MFD_H - ((float)y - (float)oy) / scale, 0.0f, (float)MFD_H);
    if (scale_out) *scale_out = scale;
    if (ox_out) *ox_out = ox;
    if (oy_out) *oy_out = oy;
    return 1;
}

static void mfda_window_draw(XPLMWindowID window, void *refcon)
{
    MfdaDevice *d = (MfdaDevice *)refcon;
    int l, t, r, b, ox, oy;
    float scale;
    GLboolean cull;
    (void)window;
    if (!d || !d->window) return;
    XPLMGetWindowGeometry(d->window, &l, &t, &r, &b);
    cull = glIsEnabled(GL_CULL_FACE);
    glDisable(GL_CULL_FACE);
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
    glBegin(GL_QUADS);
    glVertex2i(l, b); glVertex2i(r, b); glVertex2i(r, t); glVertex2i(l, t);
    glEnd();
    if (plugin_enabled && d->du_on && mfda_window_canvas(d,l,t,NULL,NULL,&scale,&ox,&oy)) {
        const MfdDrawList *dl=a380x_mfd_display_list(&mfda_mfd,d->side);
        if(xpkit_draw_mfd(d->side,(float)ox,(float)oy,scale))mfd_gl_cursor(dl,(float)ox,(float)oy,scale);
        else mfd_gl_draw(dl,(float)ox,(float)oy,scale,0);
    }
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    if (cull) glEnable(GL_CULL_FACE);
}

static int mfda_window_mouse(XPLMWindowID window, int x, int y, XPLMMouseStatus status, void *refcon)
{
    MfdaDevice *d = (MfdaDevice *)refcon;
    float cx, cy;
    (void)window;
    if (!d || !plugin_enabled || !mfda_window_canvas(d, x, y, &cx, &cy, NULL, NULL, NULL)) return 1;
    if (status == xplm_MouseDown && !XPLMIsWindowInFront(d->window)) XPLMBringWindowToFront(d->window);
    if (!d->du_on) return 1;
    mfd_pointer_move(&mfda_mfd, d->side, cx, cy);
    d->pointer_inside = 1;
    if (status == xplm_MouseDown) { mfd_pointer_button(&mfda_mfd, d->side, 1); d->touching = 1; }
    else if (status == xplm_MouseUp) { mfd_pointer_button(&mfda_mfd, d->side, 0); d->touching = 0; }
    return 1;
}

static int mfda_window_right_mouse(XPLMWindowID window, int x, int y, XPLMMouseStatus status, void *refcon)
{
    (void)window; (void)x; (void)y; (void)status; (void)refcon;
    return 1;
}

static XPLMCursorStatus mfda_window_cursor(XPLMWindowID window, int x, int y, void *refcon)
{
    MfdaDevice *d = (MfdaDevice *)refcon;
    float cx, cy;
    (void)window;
    if (!d || !plugin_enabled) return xplm_CursorDefault;
    d->cursor_seen = 1;
    if (d->du_on && mfda_window_canvas(d, x, y, &cx, &cy, NULL, NULL, NULL)
        && (!d->pointer_inside || x != d->last_x || y != d->last_y)) {
        mfd_pointer_move(&mfda_mfd, d->side, cx, cy);
        d->pointer_inside = 1; d->last_x = x; d->last_y = y;
    }
    return d->du_on ? xplm_CursorHidden : xplm_CursorDefault;
}

static int mfda_window_wheel(XPLMWindowID window, int x, int y, int wheel, int clicks, void *refcon)
{
    MfdaDevice *d = (MfdaDevice *)refcon;
    float cx, cy;
    (void)window;
    if (!d || !plugin_enabled || !d->du_on || wheel != 0 || clicks == 0) return 1;
    if (!mfda_window_canvas(d, x, y, &cx, &cy, NULL, NULL, NULL)) return 1;
    mfd_pointer_move(&mfda_mfd, d->side, cx, cy);
    mfd_pointer_wheel(&mfda_mfd, d->side, clicks);
    return 1;
}

static void mfda_window_key(XPLMWindowID window, char ch, XPLMKeyFlags flags, char vkey, void *refcon, int losing_focus)
{
    MfdaDevice *d = (MfdaDevice *)refcon;
    MfdKey key;
    char out;
    (void)window;
    if (losing_focus || !d || !plugin_enabled) return;
    if (flags & (xplm_ControlFlag | xplm_OptionAltFlag)) return;
    if (!mfda_map_key(ch, vkey, &key, &out)) return;
    if ((flags & xplm_DownFlag) && d->du_on) mfd_key(&mfda_mfd, d->side, key, out);
}

static int mfda_window_create(MfdaDevice *d)
{
    XPLMCreateWindow_t w;
    int sl, st, sr, sb, h, wd;
    if (d->window) return 1;
    memset(&w, 0, sizeof(w));
    XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);
    h = (int)((float)(st - sb) * 0.8f);
    if (h < 256) h = 256;
    wd = h * MFD_W / MFD_H;
    w.structSize = sizeof(w);
    w.left = d->side == MFD_CAPT ? sl + 40 : sr - 40 - wd;
    w.top = st - 60;
    w.right = w.left + wd;
    w.bottom = w.top - h;
    w.visible = 0;
    w.drawWindowFunc = mfda_window_draw;
    w.handleMouseClickFunc = mfda_window_mouse;
    w.handleKeyFunc = mfda_window_key;
    w.handleCursorFunc = mfda_window_cursor;
    w.handleMouseWheelFunc = mfda_window_wheel;
    w.handleRightClickFunc = mfda_window_right_mouse;
    w.refcon = d;
    w.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
    w.layer = xplm_WindowLayerFloatingWindows;
    d->window = XPLMCreateWindowEx(&w);
    if (!d->window) return 0;
    XPLMSetWindowPositioningMode(d->window, xplm_WindowPositionFree, -1);
    XPLMSetWindowResizingLimits(d->window, 192, 256, 1536, 2048);
    XPLMSetWindowTitle(d->window, d->device_name);
    return 1;
}

/* ---- Popup / pop-out ------------------------------------------------------------------------ */

static void mfda_ensure_popup_geometry(MfdaDevice *d)
{
    int l, t, r, b, sl, st, sr, sb, h, w, left, top;
    mfda_av.get_geometry(d->handle, &l, &t, &r, &b);
    XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);
    if (r - l >= 100 && t - b >= 100 && l >= sl && r <= sr && b >= sb && t <= st) return; /* saved */
    h = (int)((float)(st - sb) * 0.8f);
    w = h * MFDA_BEZEL_W / MFDA_BEZEL_H;
    left = d->side == MFD_CAPT ? sl + 40 : sr - 40 - w;
    top = st - 60;
    mfda_av.set_geometry(d->handle, left, top, left + w, top - h);
}

static void mfda_popup_toggle(MfdaDevice *d)
{
    if (d->handle && mfda_av.loaded) {
        if (mfda_av.is_popup_visible(d->handle)) {
            mfda_av.set_popup_visible(d->handle, 0); /* also closes a popped-out window */
        } else {
            mfda_ensure_popup_geometry(d);
            mfda_av.set_popup_visible(d->handle, 1);
        }
        d->force_redraw = 1;
        return;
    }
    if (!mfda_window_create(d)) return;
    XPLMSetWindowIsVisible(d->window, !XPLMGetWindowIsVisible(d->window));
    if (XPLMGetWindowIsVisible(d->window)) XPLMBringWindowToFront(d->window);
}

static void mfda_popout(MfdaDevice *d)
{
    if (d->handle && mfda_av.loaded) {
        if (!mfda_av.is_popup_visible(d->handle)) {
            mfda_ensure_popup_geometry(d);
            mfda_av.set_popup_visible(d->handle, 1);
        }
        mfda_av.pop_out(d->handle);
        d->force_redraw = 1;
        return;
    }
    if (!mfda_window_create(d)) return;
    XPLMSetWindowIsVisible(d->window, 1);
    XPLMSetWindowPositioningMode(d->window, xplm_WindowPopOut, -1);
}

static int mfda_popup_command(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
    MfdaDevice *d = (MfdaDevice *)refcon;
    (void)cmd;
    if (!plugin_enabled || phase != xplm_CommandBegin || !d) return 1;
    mfda_popup_toggle(d);
    return 0;
}

static int mfda_popout_command(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
    MfdaDevice *d = (MfdaDevice *)refcon;
    (void)cmd;
    if (!plugin_enabled || phase != xplm_CommandBegin || !d) return 1;
    mfda_popout(d);
    return 0;
}

/* ---- KCCU commands (spec 01 section 12.5, spec 09 section 4) -------------------------------- */

enum { KCCU_ACT_KEY = 0, KCCU_ACT_WHEEL_UP, KCCU_ACT_WHEEL_DOWN, KCCU_ACT_CLICK, KCCU_ACT_KBD, KCCU_ACT_CCD };

static const char *const kccu_key_names[] = {
    "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r", "s",
    "t", "u", "v", "w", "x", "y", "z", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
    "dot", "plusminus", "ent", "backspace", "sp", "slash", "esc", "esc2", "up", "down", "left",
    "right", "dir", "perf", "init", "navaid", "mailbox", "fpln", "dest", "secindex", "surv",
    "atccom", "nd", "kbd", "clrinfo", "rewind", "forward",
};
static const char *const kccu_action_names[] = {
    "wheel_up", "wheel_down", "click", "kbd_switch_toggle", "ccd_switch_toggle",
};
#define KCCU_KEY_COUNT ((int)(sizeof(kccu_key_names) / sizeof(kccu_key_names[0])))
#define KCCU_ACTION_COUNT ((int)(sizeof(kccu_action_names) / sizeof(kccu_action_names[0])))
#define KCCU_PER_SIDE (KCCU_KEY_COUNT + KCCU_ACTION_COUNT)

typedef struct {
    char name[64];
    int side, action;
    MfdKey key;
    char ch;
    XPLMCommandRef ref;
} KccuCommand;

static KccuCommand kccu_commands[MFD_SIDE_COUNT * KCCU_PER_SIDE];

static int kccu_command_handler(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
    KccuCommand *k = (KccuCommand *)refcon;
    MfdaDevice *d;
    (void)cmd;
    if (!plugin_enabled || !k || !mfda_side_ok(k->side)) return 1;
    d = &mfda_dev[k->side];
    if (k->action == KCCU_ACT_CLICK) {
        if (phase == xplm_CommandBegin) {
            /* Only at a visible MFD cursor (spec 09 section 4.1); DU-off sides ignore input. */
            if (d->du_on && mfda_mfd.sides[k->side].ui.cursor_visible) {
                mfd_pointer_button(&mfda_mfd, (MfdSide)k->side, 1);
                d->click_held = 1;
            }
        } else if (phase == xplm_CommandEnd && d->click_held) {
            mfd_pointer_button(&mfda_mfd, (MfdSide)k->side, 0);
            d->click_held = 0;
        }
        return 0;
    }
    if (phase != xplm_CommandBegin) return 0;
    switch (k->action) {
    case KCCU_ACT_KEY:
        /* Same side only (quirk 1); ignored while that side's DU is not On (quirk 17). */
        if (d->du_on) mfd_key(&mfda_mfd, (MfdSide)k->side, k->key, k->ch);
        break;
    case KCCU_ACT_WHEEL_UP:
        if (d->du_on) mfd_pointer_wheel(&mfda_mfd, (MfdSide)k->side, 1);
        break;
    case KCCU_ACT_WHEEL_DOWN:
        if (d->du_on) mfd_pointer_wheel(&mfda_mfd, (MfdSide)k->side, -1);
        break;
    case KCCU_ACT_KBD:
        mfda_kccu_kbd_on[k->side] = !mfda_kccu_kbd_on[k->side];
        break;
    case KCCU_ACT_CCD:
        mfda_kccu_ccd_on[k->side] = !mfda_kccu_ccd_on[k->side];
        break;
    default:
        break;
    }
    return 0;
}

/* ---- Datarefs ------------------------------------------------------------------------------- */

enum { MFDA_REF_INT_RW = 0, MFDA_REF_INT_RO, MFDA_REF_BYTES };

typedef struct {
    char name[80];
    int kind;
    int *value;      /* int datarefs */
    int lo, hi;      /* writable range */
    int side;        /* byte dataref: active page of this side */
    XPLMDataRef handle;
} MfdaRef;

#define MFDA_REF_COUNT 41
static MfdaRef mfda_refs[MFDA_REF_COUNT];
static int mfda_ref_count;

static int mfda_ref_get_int(void *refcon)
{
    const MfdaRef *r = (const MfdaRef *)refcon;
    return r && r->value ? *r->value : 0;
}

static void mfda_ref_set_int(void *refcon, int v)
{
    MfdaRef *r = (MfdaRef *)refcon;
    if (!r || !r->value || r->kind != MFDA_REF_INT_RW) return;
    *r->value = v < r->lo ? r->lo : v > r->hi ? r->hi : v;
}

static float mfda_ref_get_float(void *refcon) { return (float)mfda_ref_get_int(refcon); }
static void mfda_ref_set_float(void *refcon, float v) { mfda_ref_set_int(refcon, (int)lroundf(v)); }

static int mfda_ref_get_bytes(void *refcon, void *out, int offset, int max)
{
    const MfdaRef *r = (const MfdaRef *)refcon;
    const char *uri;
    int n, len;
    if (!r || !mfda_side_ok(r->side)) return 0;
    uri = a380x_mfd_active_uri(&mfda_mfd, (MfdSide)r->side);
    len = (int)strlen(uri) + 1; /* include the terminator */
    if (len > MFD_URI_LEN) len = MFD_URI_LEN;
    if (!out) return MFD_URI_LEN;
    if (offset < 0 || offset >= len || max <= 0) return 0;
    n = len - offset < max ? len - offset : max;
    memcpy(out, uri + offset, (size_t)n);
    return n;
}

static void mfda_unregister_refs(void)
{
    int i;
    for (i = 0; i < mfda_ref_count; ++i) {
        if (mfda_refs[i].handle) { XPLMUnregisterDataAccessor(mfda_refs[i].handle); mfda_refs[i].handle = NULL; }
    }
    mfda_ref_count = 0;
}

static void mfda_add_ref(const char *name, int kind, int *value, int lo, int hi, int side)
{
    MfdaRef *r;
    if (mfda_ref_count >= MFDA_REF_COUNT) return;
    r = &mfda_refs[mfda_ref_count++];
    memset(r, 0, sizeof(*r));
    snprintf(r->name, sizeof(r->name), "%s", name);
    r->kind = kind; r->value = value; r->lo = lo; r->hi = hi; r->side = side;
}

static int mfda_register_refs(void)
{
    int s, i;
    char name[96];
    mfda_ref_count = 0;
    for (s = 0; s < MFD_SIDE_COUNT; ++s) {
        snprintf(name, sizeof(name), "a380x/xp12/kccu/%s/kbd_on", mfda_dev[s].side_name);
        mfda_add_ref(name, MFDA_REF_INT_RW, &mfda_kccu_kbd_on[s], 0, 1, s);
        snprintf(name, sizeof(name), "a380x/xp12/kccu/%s/ccd_on", mfda_dev[s].side_name);
        mfda_add_ref(name, MFDA_REF_INT_RW, &mfda_kccu_ccd_on[s], 0, 1, s);
    }
    for (s = 0; s < MFD_SIDE_COUNT; ++s) {
        snprintf(name, sizeof(name), "a380x/xp12/mfd/%s/active_page", mfda_dev[s].side_name);
        mfda_add_ref(name, MFDA_REF_BYTES, NULL, 0, 0, s);
        snprintf(name, sizeof(name), "a380x/xp12/mfd/%s/keyboard_captured", mfda_dev[s].side_name);
        mfda_add_ref(name, MFDA_REF_INT_RO, &mfda_dev[s].keyboard_captured, 0, 1, s);
        const char *const labels[] = {"device_bound", "touch_down_count", "touch_up_count",
            "cursor_x", "cursor_y", "cursor_visible", "menu_open", "events_dropped",
            "last_down_x", "last_down_y", "last_up_x", "last_up_y",
            "press_target", "click_target", "click_count", "menu_open_count"};
        int *values[] = {&mfda_dev[s].device_bound, &mfda_dev[s].touch_down_count,
            &mfda_dev[s].touch_up_count, &mfda_dev[s].cursor_x, &mfda_dev[s].cursor_y,
            &mfda_dev[s].cursor_visible, &mfda_dev[s].menu_open, &mfda_dev[s].events_dropped,
            &mfda_dev[s].last_down_x, &mfda_dev[s].last_down_y,
            &mfda_dev[s].last_up_x, &mfda_dev[s].last_up_y,
            &mfda_dev[s].press_target, &mfda_dev[s].click_target,
            &mfda_dev[s].click_count, &mfda_dev[s].menu_open_count};
        for (int a=0; a<ARRAY_LEN(labels); ++a) {
            snprintf(name, sizeof(name), "a380x/xp12/mfd/%s/%s", mfda_dev[s].side_name, labels[a]);
            mfda_add_ref(name, MFDA_REF_INT_RO, values[a], 0, 0, s);
        }
    }
    mfda_add_ref("a380x/xp12/mfd/pc_keyboard_capture", MFDA_REF_INT_RW, &mfda_pc_capture_pref, 0, 2, -1);
    for (i = 0; i < mfda_ref_count; ++i) {
        if (XPLMFindDataRef(mfda_refs[i].name)) {
            XPLMDebugString("A380X MFD: custom dataref collision ");
            XPLMDebugString(mfda_refs[i].name);
            XPLMDebugString("\n");
            mfda_ref_count = 0;
            return 0;
        }
    }
    for (i = 0; i < mfda_ref_count; ++i) {
        MfdaRef *r = &mfda_refs[i];
        int writable = r->kind == MFDA_REF_INT_RW;
        if (r->kind == MFDA_REF_BYTES) {
            r->handle = XPLMRegisterDataAccessor(r->name, xplmType_Data, 0,
                NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                mfda_ref_get_bytes, NULL, r, NULL);
        } else {
            r->handle = XPLMRegisterDataAccessor(r->name, xplmType_Int | xplmType_Float, writable,
                mfda_ref_get_int, writable ? mfda_ref_set_int : NULL,
                mfda_ref_get_float, writable ? mfda_ref_set_float : NULL,
                NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, r, writable ? r : NULL);
        }
        if (!r->handle) { mfda_unregister_refs(); return 0; }
    }
    return 1;
}

/* ---- Commands and menu ---------------------------------------------------------------------- */

static void mfda_unregister_commands(void)
{
    int i, s;
    for (i = 0; i < MFD_SIDE_COUNT * KCCU_PER_SIDE; ++i) {
        if (kccu_commands[i].ref) {
            XPLMUnregisterCommandHandler(kccu_commands[i].ref, kccu_command_handler, 1, &kccu_commands[i]);
            kccu_commands[i].ref = NULL;
        }
    }
    for (s = 0; s < MFD_SIDE_COUNT; ++s) {
        MfdaDevice *d = &mfda_dev[s];
        if (d->popup_cmd) { XPLMUnregisterCommandHandler(d->popup_cmd, mfda_popup_command, 1, d); d->popup_cmd = NULL; }
        if (d->popout_cmd) { XPLMUnregisterCommandHandler(d->popout_cmd, mfda_popout_command, 1, d); d->popout_cmd = NULL; }
    }
}

static XPLMCommandRef mfda_command(const char *name, const char *description)
{
    XPLMCommandRef ref = XPLMFindCommand(name); /* commands outlive a plugin reload */
    return ref ? ref : XPLMCreateCommand(name, description);
}

static int mfda_register_commands(void)
{
    int s, i;
    char description[128];
    for (s = 0; s < MFD_SIDE_COUNT; ++s) {
        for (i = 0; i < KCCU_PER_SIDE; ++i) {
            KccuCommand *k = &kccu_commands[s * KCCU_PER_SIDE + i];
            memset(k, 0, sizeof(*k));
            k->side = s;
            if (i < KCCU_KEY_COUNT) {
                k->action = KCCU_ACT_KEY;
                if (!mfd_key_from_name(kccu_key_names[i], &k->key, &k->ch)) return 0;
                snprintf(k->name, sizeof(k->name), "a380x/xp12/kccu/%s/%s", mfda_dev[s].side_name, kccu_key_names[i]);
                snprintf(description, sizeof(description), "%s KCCU key %s", mfda_dev[s].seat, kccu_key_names[i]);
            } else {
                k->action = KCCU_ACT_WHEEL_UP + (i - KCCU_KEY_COUNT);
                snprintf(k->name, sizeof(k->name), "a380x/xp12/kccu/%s/%s", mfda_dev[s].side_name,
                         kccu_action_names[i - KCCU_KEY_COUNT]);
                snprintf(description, sizeof(description), "%s KCCU %s", mfda_dev[s].seat,
                         kccu_action_names[i - KCCU_KEY_COUNT]);
            }
            k->ref = mfda_command(k->name, description);
            if (!k->ref) return 0;
            XPLMRegisterCommandHandler(k->ref, kccu_command_handler, 1, k);
        }
    }
    for (s = 0; s < MFD_SIDE_COUNT; ++s) {
        MfdaDevice *d = &mfda_dev[s];
        snprintf(description, sizeof(description), "Show/hide the %s MFD pop-up", d->seat);
        d->popup_cmd = mfda_command(d->popup_name, description);
        if (!d->popup_cmd) return 0;
        XPLMRegisterCommandHandler(d->popup_cmd, mfda_popup_command, 1, d);
        snprintf(description, sizeof(description), "Pop out the %s MFD window", d->seat);
        d->popout_cmd = mfda_command(d->popout_name, description);
        if (!d->popout_cmd) return 0;
        XPLMRegisterCommandHandler(d->popout_cmd, mfda_popout_command, 1, d);
    }
    return 1;
}

static void mfda_menu_destroy(void)
{
    if (mfda_menu) { XPLMDestroyMenu(mfda_menu); mfda_menu = NULL; }
    if (mfda_menu_parent && mfda_menu_item >= 0) XPLMRemoveMenuItem(mfda_menu_parent, mfda_menu_item);
    mfda_menu_parent = NULL;
    mfda_menu_item = -1;
}

static void mfda_menu_create(void)
{
    int s;
    char item[64];
    /* Aircraft menu (spec 08 section 6); the plugins menu when X-Plane offers none. */
    mfda_menu_parent = XPLMFindAircraftMenu();
    if (mfda_menu_parent) {
        mfda_menu_item = XPLMAppendMenuItem(mfda_menu_parent, "A380X MFD", NULL, 0);
        if (mfda_menu_item < 0) { mfda_menu_parent = NULL; mfda_menu_item = -1; }
    }
    mfda_menu = XPLMCreateMenu("A380X MFD", mfda_menu_parent, mfda_menu_parent ? mfda_menu_item : 0, NULL, NULL);
    if (!mfda_menu) { mfda_menu_destroy(); return; }
    for (s = 0; s < MFD_SIDE_COUNT; ++s) {
        snprintf(item, sizeof(item), "%s MFD pop-up", mfda_dev[s].seat);
        XPLMAppendMenuItemWithCommand(mfda_menu, item, mfda_dev[s].popup_cmd);
        snprintf(item, sizeof(item), "%s MFD pop-out window", mfda_dev[s].seat);
        XPLMAppendMenuItemWithCommand(mfda_menu, item, mfda_dev[s].popout_cmd);
    }
}

/* ---- Lifecycle ------------------------------------------------------------------------------ */

static void mfda_devices_destroy(void)
{
    int s;
    for (s = 0; s < MFD_SIDE_COUNT; ++s) {
        MfdaDevice *d = &mfda_dev[s];
        if (d->handle && mfda_av.loaded) mfda_av.destroy(d->handle); /* also removes the popup */
        d->handle = NULL;
        if (d->window) { XPLMDestroyWindow(d->window); d->window = NULL; }
    }
    mfda_devices_created = 0;
}

/* Called from XPluginStart (before the aircraft OBJs are loaded) and again from XPluginEnable
 * as a no-op safety net. Never fatal. */
static void mfda_devices_enable(void)
{
    int s;
    XPLMCreateAvionics_t p;
    if (!mfda_initialized || mfda_devices_created) return;
    if (!mfda_av_load()) {
        XPLMDebugString("A380X MFD: avionics device API unavailable (X-Plane < 12.1) - MFD opens in a window (F0)\n");
        return;
    }
    if (mfda_av.xplm_version < 421)
        XPLMDebugString("A380X MFD: X-Plane older than 12.4.0 - known cockpit-device hover/drag bugs\n");
    for (s = 0; s < MFD_SIDE_COUNT; ++s) {
        MfdaDevice *d = &mfda_dev[s];
        memset(&p, 0, sizeof(p));
        p.structSize = (int)sizeof(p);
        p.screenWidth = MFDA_PX(MFD_W); p.screenHeight = MFDA_PX(MFD_H);
        p.bezelWidth = MFDA_PX(MFDA_BEZEL_W); p.bezelHeight = MFDA_PX(MFDA_BEZEL_H);
        p.screenOffsetX = MFDA_PX(MFDA_BEZEL_SIDE); p.screenOffsetY = MFDA_PX(MFDA_BEZEL_SIDE);
        p.drawOnDemand = 1;
        p.bezelDrawCallback = mfda_bezel_draw;
        p.drawCallback = mfda_screen_draw;
        p.bezelClickCallback = mfda_bezel_click;
        p.bezelRightClickCallback = mfda_bezel_click;
        p.bezelScrollCallback = mfda_bezel_scroll;
        p.bezelCursorCallback = mfda_bezel_cursor;
        p.screenTouchCallback = mfda_screen_touch;
        p.screenRightTouchCallback = mfda_screen_right_touch;
        p.screenScrollCallback = mfda_screen_scroll;
        p.screenCursorCallback = mfda_screen_cursor;
        p.keyboardCallback = mfda_popup_keyboard;
        p.brightnessCallback = mfda_screen_brightness;
        p.deviceID = (char *)d->device_id;
        p.deviceName = (char *)d->device_name;
        p.refcon = d;
        d->handle = mfda_av.create(&p);
        if (!d->handle) {
            XPLMDebugString("A380X MFD: XPLMCreateAvionicsEx failed (duplicate id or struct size) - F0 window fallback\n");
            break;
        }
        d->pointer_inside = 0; d->last_knob = -1.0f; d->since_create = 0.0f; d->bound_warned = 0;
        d->touch_down_count = d->touch_up_count = 0;
        d->device_bound = d->menu_open = d->cursor_visible = d->events_dropped = 0;
        d->last_down_x = d->last_down_y = d->last_up_x = d->last_up_y = -1;
        d->press_target = d->click_target = d->click_count = d->menu_open_count = d->trace_frames = 0;
        d->list_hash = 0; d->force_redraw = 1;
    }
    if (s < MFD_SIDE_COUNT) { mfda_devices_destroy(); return; }
    mfda_devices_created = 1;
    for (s = 0; s < MFD_SIDE_COUNT; ++s) mfda_av.needs_drawing(mfda_dev[s].handle);
}

static void mfda_release_input(void)
{
    int s;
    mfda_capture_release();
    for (s = 0; s < MFD_SIDE_COUNT; ++s) {
        MfdaDevice *d = &mfda_dev[s];
        if (d->click_held || d->touching) mfd_pointer_button(&mfda_mfd, d->side, 0);
        d->click_held = d->touching = 0;
        if (d->pointer_inside) mfd_pointer_leave(&mfda_mfd, d->side);
        d->pointer_inside = 0;
        d->prev_wants = 0; d->prev_focus = 0; d->prev_popup = 0; d->keyboard_captured = 0;
    }
}

/* XPluginDisable releases input but retains devices until XPluginStop. Register early
 * so the same handles are used for screen drawing and 3D manipulators. The SDK also
 * supports retroactive mapping, so an OBJ parser warning alone does not prove that
 * a custom device is unbound: use device_bound and the real touch counters to check. */
static void mfda_devices_disable(void)
{
    int s;
    if (!mfda_initialized) return;
    mfda_release_input();
    for (s = 0; s < MFD_SIDE_COUNT; ++s)
        if (mfda_dev[s].window && XPLMHasKeyboardFocus(mfda_dev[s].window)) XPLMTakeKeyboardFocus(0);
}

static void mfda_shutdown(void)
{
    mfda_devices_disable();
    mfda_devices_destroy();
    if (mfda_sniffer_registered) { XPLMUnregisterKeySniffer(mfda_key_sniffer, 0, NULL); mfda_sniffer_registered = 0; }
    mfda_menu_destroy();
    mfda_unregister_commands();
    mfda_unregister_refs();
    mfda_initialized = 0;
}

/* XPluginStart: pages, core, datarefs, commands, menu, key sniffer. A dataref collision is
 * fatal (returns 0 after cleaning up); devices are created in mfda_devices_enable. */
static int mfda_init(struct A380XFms *fms, MfdMsgQueue *msgs, const A380XMfdSystemsHost *host)
{
    char names[160], line[256];
    int count;
    if (mfda_initialized) return 1;
    a380x_mfd_pages_register();
    a380x_mfd_init(&mfda_mfd, fms, msgs, host);
    memset(&mfda_inputs, 0, sizeof(mfda_inputs));
    mfda_inputs.fms_knob = 1;
    mfda_inputs.fmc_a_healthy = mfda_inputs.fmc_b_healthy = 1;
    mfda_inputs.metric_units = 1;
    mfda_kccu_kbd_on[0] = mfda_kccu_kbd_on[1] = 1;
    mfda_kccu_ccd_on[0] = mfda_kccu_ccd_on[1] = 1;
    mfda_pc_capture_pref = 1;
    mfda_capture_side = -1;
    if (!mfda_register_refs()) return 0;
    if (!mfda_register_commands()) { mfda_unregister_commands(); mfda_unregister_refs(); return 0; }
    mfda_initialized = 1;
    mfda_menu_create();
    mfda_sniffer_registered = XPLMRegisterKeySniffer(mfda_key_sniffer, 0, NULL);
    count = a380x_mfd_pages_linked(names, sizeof(names));
    snprintf(line, sizeof(line), "A380X MFD: %d page modules linked (%s), %d routes\n", count, names, mfd_page_count());
    XPLMDebugString(line);
    return 1;
}

/* XPLM_MSG_PLANE_LOADED: both sides back to DATA/STATUS, focus and menus cleared. */
static void mfda_plane_loaded(void)
{
    int s;
    if (!mfda_initialized) return;
    mfda_release_input();
    for (s = 0; s < MFD_SIDE_COUNT; ++s) a380x_mfd_navigate(&mfda_mfd, (MfdSide)s, "fms/data/status");
}

/* PC keyboard routing for one side (spec 08 sections 5.2, 10.5). Edge triggered so X-Plane or
 * another window can take the focus back. */
static void mfda_keyboard_route(MfdaDevice *d)
{
    MfdId focus = mfda_mfd.sides[d->side].ui.focus_id;
    int wants = mfda_pc_capture_pref != 0 && d->du_on && a380x_mfd_wants_keyboard(&mfda_mfd, d->side);
    int popup = mfda_popup_shown(d);
    if (wants && (!d->prev_wants || focus != d->prev_focus || popup != d->prev_popup)) {
        if (popup) {
            if (d->handle && mfda_av.loaded) mfda_av.take_keyboard(d->handle);
            else if (d->window) XPLMTakeKeyboardFocus(d->window);
            if (mfda_capture_side == (int)d->side) mfda_capture_release();
        } else {
            mfda_capture_side = (int)d->side; /* the last focused side owns the PC keyboard */
        }
    } else if (!wants && d->prev_wants) {
        if (mfda_capture_side == (int)d->side) mfda_capture_release();
        if (popup) {
            if (d->handle && mfda_av.loaded) { if (mfda_av.has_keyboard(d->handle)) XPLMTakeKeyboardFocus(0); }
            else if (d->window && XPLMHasKeyboardFocus(d->window)) XPLMTakeKeyboardFocus(0);
        }
    }
    if (!wants && mfda_capture_side == (int)d->side) mfda_capture_release();
    d->prev_wants = wants;
    d->prev_focus = focus;
    d->prev_popup = popup;
    d->keyboard_captured = mfda_capture_side == (int)d->side
        || (popup && wants && (d->handle && mfda_av.loaded ? mfda_av.has_keyboard(d->handle)
                                                             : d->window && XPLMHasKeyboardFocus(d->window)));
}

/* Flight loop, after the FMS update (a380x_fms_adapter.inc filled mfda_inputs' FMS fields). */
static void mfda_update(float dt, int powered)
{
    int s;
    if (!mfda_initialized) return;
    mfda_powered = powered;
    mfda_inputs.now = (double)XPLMGetElapsedTime();
    for (s = 0; s < MFD_SIDE_COUNT; ++s) {
        MfdaDevice *d = &mfda_dev[s];
        int x, y;
        /* CAPT MFD: DC ESS or DC 1, F/O MFD: DC 1 or DC 2 (A380 network, CDS state On). */
        d->du_on = a380x_power_du_on(MFDA_DU(s), powered) && mfda_du_knob(s) > 0.001f;
        mfda_inputs.du_on[s] = (unsigned char)d->du_on;
        /* Capture policy 1 (auto) requests capture regardless of the KCCU KBD switch; policy 2
         * (FBW) only in touchscreen mode (KBD OFF). Policy 0 never captures (routing below). */
        mfda_inputs.kbd_on[s] = (unsigned char)(mfda_pc_capture_pref == 2 ? mfda_kccu_kbd_on[s] != 0 : 0);
        if (!d->du_on) {
            if (d->click_held) { mfd_pointer_button(&mfda_mfd, d->side, 0); d->click_held = 0; }
            d->pointer_inside = 0;
            continue;
        }
        /* Hover: XPLMIsCursorOverAvionics (fixed in 12.4.0) first, cursor callback second. */
        if (d->handle && mfda_av.loaded && mfda_av.cursor_over(d->handle, &x, &y)) {
            if (!d->pointer_inside || x != d->last_x || y != d->last_y) {
                float cx, cy;
                mfda_to_canvas(x, y, &cx, &cy);
                mfd_pointer_move(&mfda_mfd, d->side, cx, cy);
                d->pointer_inside = 1; d->last_x = x; d->last_y = y;
            }
        } else if (d->pointer_inside && !d->cursor_seen && !d->touching && !d->click_held) {
            mfd_pointer_leave(&mfda_mfd, d->side);
            d->pointer_inside = 0;
        }
        d->cursor_seen = 0;
    }
    a380x_mfd_frame(&mfda_mfd, &mfda_inputs);
#if defined(A380X_MFD_HAVE_PAGE_FCUBKUP)
    a380x_mfd_page_fcubkup_release_held(&mfda_mfd); /* no -/+ arrow left begun after navigation */
#endif
    for (s = 0; s < MFD_SIDE_COUNT; ++s) {
        MfdaDevice *d = &mfda_dev[s];
        const MfdUi *ui = &mfda_mfd.sides[s].ui;
        d->device_bound = d->handle && mfda_av.loaded && mfda_av.is_bound(d->handle);
        d->cursor_x = (int)ui->px; d->cursor_y = (int)ui->py;
        d->cursor_visible = ui->cursor_visible;
        if (ui->down_id) d->press_target = (int)ui->down_id;
        if (ui->click_happened) {
            d->click_target = (int)ui->click_id;
            if (ui->click_id) ++d->click_count;
        }
        if (!d->menu_open && (ui->open_menu || ui->ctx_menu)) ++d->menu_open_count;
        d->menu_open = ui->open_menu != 0 || ui->ctx_menu != 0;
        d->events_dropped = (int)ui->ev_dropped;
        if (d->trace_frames > 0) {
            char line[320];
            snprintf(line, sizeof(line), "A380X MFD resolve %s cursor=%d,%d inside=%d held=%d press=%08x down=%08x click=%08x released=%d menu=%08x pending=%d page=%s\n",
                d->seat, d->cursor_x, d->cursor_y, ui->pointer_inside, ui->button_down,
                (unsigned)ui->press_id, (unsigned)ui->down_id, (unsigned)ui->click_id,
                ui->click_happened, (unsigned)ui->open_menu, ui->ev_count,
                a380x_mfd_active_uri(&mfda_mfd, d->side));
            XPLMDebugString(line);
            --d->trace_frames;
        }
        uint64_t h = mfda_list_hash(a380x_mfd_display_list(&mfda_mfd, (MfdSide)s), d->du_on) +
                     (uint64_t)(a380x_power_du_state(MFDA_DU(s)) + 2) * 0x9E3779B97F4A7C15ull;  /* CDS state redraw */
        mfda_keyboard_route(d);
        if (!d->handle || !mfda_av.loaded) { d->list_hash = h; continue; }
        if (h != d->list_hash || d->force_redraw) {
            mfda_av.needs_drawing(d->handle);
            d->list_hash = h;
            d->force_redraw = 0;
        }
        /* Keep sim/cockpit2/switches/instrument_brightness_ratio[14/15] in step with the knob. */
        if (fabsf(mfda_du_knob(s) - d->last_knob) > 0.001f) {
            d->last_knob = mfda_du_knob(s);
            mfda_av.set_rheo(d->handle, d->last_knob);
        }
        d->since_create += dt;
        if (!d->bound_warned && d->since_create > 10.0f) {
            d->bound_warned = 1;
            if (!mfda_av.is_bound(d->handle))
                XPLMDebugString("A380X MFD: device not bound - check ATTR_cockpit_device ids in the OBJs\n");
        }
    }
}
