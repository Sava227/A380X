/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Native PFDs and NDs on their dedicated 768x1024 X-Plane cockpit devices, for builds without
 * xp-kit (Windows today, and any build where the xp-kit renderers are not linked).
 *
 * a380x_mfd_screens.obj already carries the device quads for a380x.xp12.pfd.* and a380x.xp12.nd.*
 * (shown only while a380x/xp12/xpkit/pfd_devices_active / nd_devices_active is 1). xp-kit builds
 * create those devices for the FlyByWire library views (a380x_xpkit_pfd.inc, a380x_xpkit_nd_input.inc);
 * without xp-kit nothing did, so the PFD/ND stayed on the 320x416 / 180x234 panel-atlas regions.
 * This file creates the same devices with the native renderers:
 *   PFD: draw_pfd scaled from its 320x416 layout (as the xp-kit fallback does), lines scaled too;
 *   ND:  the FlyByWire layout for ARC, ROSE and PLAN on the full 768x1024 canvas (a380x_nd_fbw.inc,
 *        including the vertical display and its terrain profile); the airport zoom keeps the native
 *        180x234 layout, scaled up.
 * The panel-atlas PFD/ND stay drawn underneath, as in the xp-kit builds, so a display never goes
 * blank if the simulator does not present the device surface. Devices are created from XPluginStart,
 * before the aircraft OBJs load, or X-Plane drops their click manipulators.
 */

#include "a380x_avionics_popup.h"

#if !defined(A380X_XPKIT)

#define NATIVE_DU_W 768
#define NATIVE_DU_H 1024
#define NATIVE_ND_LAYOUT_W 180.0f
#define NATIVE_ND_LAYOUT_H 234.0f

typedef struct {
    int side, is_nd;
    XPLMAvionicsID device;
} NativeDisplay;

static NativeDisplay native_displays[4] = {{0, 0, NULL}, {1, 0, NULL}, {0, 1, NULL}, {1, 1, NULL}};
static XPLMDataRef native_displays_active_ref[2];

static int native_display_du(const NativeDisplay *d)
{
    if (d->is_nd) return d->side ? A380X_ELN_DU_FO_ND : A380X_ELN_DU_CAPT_ND;
    return d->side ? A380X_ELN_DU_FO_PFD : A380X_ELN_DU_CAPT_PFD;
}

static float native_display_level(const NativeDisplay *d)
{
    return du_channels[d->side * DU_CHANNELS_PER_SIDE + (d->is_nd ? DU_ND : DU_PFD)].level;
}

static int native_display_on(const NativeDisplay *d)
{
    return plugin_enabled && refs.side[d->side].display_valid && native_display_level(d) > 0;
}

/* 1 once both devices of the kind exist and are bound to their OBJ quads. */
static int native_displays_active(int is_nd)
{
    int s;
    for (s = 0; s < 2; ++s) {
        const NativeDisplay *d = &native_displays[is_nd * 2 + s];
        if (!d->device || !mfda_av.loaded || !mfda_av.is_bound(d->device)) return 0;
    }
    return 1;
}
static int native_displays_active_get(void *refcon) { return native_displays_active(refcon != NULL); }

static void native_display_draw(void *refcon)
{
    NativeDisplay *d = refcon;
    const char *name = d->side ? "F/O" : "CAPT";
    GLboolean cull = glIsEnabled(GL_CULL_FACE);
    glDisable(GL_CULL_FACE);
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    glClearColor(0, 0, 0, 1);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glClear(GL_COLOR_BUFFER_BIT);   /* X-Plane does not clear the device framebuffer */
    if (plugin_enabled && cds_overlay(native_display_du(d), 0, 0, NATIVE_DU_W, NATIVE_DU_H)) {
        if (cull) glEnable(GL_CULL_FACE);
        return;
    }
    if (native_display_on(d)) {
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_FALSE); /* keep alpha = 1: the screen never blends */
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        if (!d->is_nd) {
            glPushMatrix();
            glScalef(NATIVE_DU_W / PFD_PIXEL_WIDTH, NATIVE_DU_H / PFD_PIXEL_HEIGHT, 1);
            pfd_line_scale = NATIVE_DU_H / PFD_PIXEL_HEIGHT;
            pfd_line_width(1.0f);
            draw_pfd(&refs.side[d->side], 0, 0, name);
            pfd_line_scale = 1.0f;
            glPopMatrix();
        } else if (!efis[d->side].zoom) {
            text_use_atlas = 1;
            draw_nd_fbw(&refs.side[d->side], d->side);   /* ARC, ROSE LS/VOR/NAV, PLAN */
            text_use_atlas = 0;
        } else {                                         /* airport zoom: native layout, scaled */
            const float scale = NATIVE_DU_W / NATIVE_ND_LAYOUT_W;
            text_use_atlas = 1;
            text_atlas_size = 9.0f;
            glPushMatrix();
            glTranslatef(0, (NATIVE_DU_H - NATIVE_ND_LAYOUT_H * scale) * 0.5f, 0);
            glScalef(scale, scale, 1);
            glLineWidth(scale * 0.75f);
            draw_nd(&refs.side[d->side], 0, 0, name);
            glPopMatrix();
            text_use_atlas = 0;
        }
        text_atlas_size = 9.0f;
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    }
    glLineWidth(1.0f);
    glColor4f(1, 1, 1, 1);
    if (cull) glEnable(GL_CULL_FACE);
}

/* Same law as the xp-kit PFD devices: the DU knob, while the DU is powered (or self-testing). */
static float native_display_brightness(float rheo, float ambient, float bus, void *refcon)
{
    NativeDisplay *d = refcon;
    (void)rheo; (void)ambient; (void)bus;
    int lit = native_display_on(d) || (plugin_enabled && a380x_power_du_lit(native_display_du(d), 0));
    return lit ? native_display_level(d) : 0;
}

/* The PFD has no touch functions: a click (release) toggles its pop-up, as in the xp-kit builds. */
static int native_display_touch(int x, int y, XPLMMouseStatus phase, void *refcon)
{
    NativeDisplay *d = refcon;
    (void)x; (void)y;
    if (phase == xplm_MouseUp && plugin_enabled && d->device && mfda_av.loaded) {
        int show = !mfda_av.is_popup_visible(d->device);
        if (show) a380x_fit_popup(d->device, d->side, mfda_av.get_geometry, mfda_av.set_geometry);
        mfda_av.set_popup_visible(d->device, show);
    }
    return 1;
}
static int native_display_right(int x, int y, XPLMMouseStatus phase, void *refcon)
{
    (void)x; (void)y; (void)phase; (void)refcon;
    return 0; /* keep right-drag camera look */
}

static void native_displays_shutdown(void)
{
    int i;
    for (i = 0; i < 4; ++i) {
        if (native_displays[i].device && mfda_av.loaded) mfda_av.destroy(native_displays[i].device);
        native_displays[i].device = NULL;
    }
    for (i = 0; i < 2; ++i)
        if (native_displays_active_ref[i]) {
            XPLMUnregisterDataAccessor(native_displays_active_ref[i]);
            native_displays_active_ref[i] = NULL;
        }
}

/* XPluginStart, next to xpkit_init(). Never fatal: without the device API the atlas regions stay. */
static void native_displays_init(void)
{
    static const char *const active_names[2] = {"a380x/xp12/xpkit/pfd_devices_active",
                                                "a380x/xp12/xpkit/nd_devices_active"};
    static const char *const ids[4] = {"a380x.xp12.pfd.captain", "a380x.xp12.pfd.first_officer",
                                       "a380x.xp12.nd.captain", "a380x.xp12.nd.first_officer"};
    static const char *const names[4] = {"A380X PFD CAPT", "A380X PFD F/O", "A380X ND CAPT", "A380X ND F/O"};
    int i, created = 0;
    for (i = 0; i < 2; ++i)
        if (!native_displays_active_ref[i] && !XPLMFindDataRef(active_names[i]))
            native_displays_active_ref[i] = XPLMRegisterDataAccessor(active_names[i], xplmType_Int, 0,
                native_displays_active_get, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                i ? (void *)&native_displays[2] : NULL, NULL);
    if (!mfda_av.loaded) mfda_av_load();
    if (!mfda_av.loaded) return;
    for (i = 0; i < 4; ++i) {
        NativeDisplay *d = &native_displays[i];
        XPLMCreateAvionics_t p;
        if (d->device) { ++created; continue; }
        memset(&p, 0, sizeof(p));
        p.structSize = sizeof(p);
        p.screenWidth = p.bezelWidth = NATIVE_DU_W;
        p.screenHeight = p.bezelHeight = NATIVE_DU_H;
        p.drawOnDemand = 0;   /* the native PFD/ND animate every frame */
        p.drawCallback = native_display_draw;
        p.bezelDrawCallback = a380x_black_bezel;
        p.brightnessCallback = native_display_brightness;
        if (!d->is_nd) {
            p.screenTouchCallback = native_display_touch;
            p.screenRightTouchCallback = native_display_right;
        }
        p.refcon = d;
        p.deviceID = (char *)ids[i];
        p.deviceName = (char *)names[i];
        d->device = mfda_av.create(&p);
        if (d->device) ++created;
    }
    XPLMDebugString(created == 4 ? "A380X native PFD/ND: cockpit devices created\n"
                                 : "A380X native PFD/ND: device creation failed - panel-atlas displays only\n");
}

#else

static void native_displays_init(void) {}
static void native_displays_shutdown(void) {}

#endif
