/* SPDX-License-Identifier: GPL-3.0-or-later
 * xp-kit FlyByWire PFDs on dedicated 768x1024 X-Plane cockpit devices.
 *
 * The helper now captures the PFD at its full 768x1024 logical size; the device
 * presents that texture 1:1 at the authored SCREEN_DU_PFDL/R location, redraws
 * only when a new frame (or a power/health change) arrives, and offers the
 * standard X-Plane pop-up / pop-out window. Clicking the PFD in the 3D cockpit
 * toggles its pop-up. While a device is bound, the legacy 320x416 panel-atlas
 * PFD is not drawn at all (the device quad covers it), which removes that
 * per-frame atlas work. If the library feed is unavailable the device draws the
 * native PFD renderer instead, so the PFD never freezes.
 *
 * Unity-build include of a380x_xpkit_adapter.inc (needs xpkit_draw_view,
 * xpkit_uploaded, xpkit_health, mfda_av, refs, du_channels, draw_pfd).
 */
#include "a380x_avionics_popup.h"
typedef struct {
    int side, draw_live, draw_powered;
    unsigned presented_frame, draws, rendered_draws;
    XPLMAvionicsID device;
    XPLMCommandRef command[2];
} XpKitPfd;
static XpKitPfd xpkit_pfd[2];
static const char *xpkit_pfd_actions[] = {"popup", "popout"};

static int xpkit_pfd_on(const XpKitPfd *d) {
    return plugin_enabled && refs.side[d->side].display_valid &&
           du_channels[d->side * DU_CHANNELS_PER_SIDE + DU_PFD].level > 0;
}
static int xpkit_pfd_bound(const XpKitPfd *d) {
    return d->device && mfda_av.loaded && mfda_av.is_bound(d->device);
}
/* The panel-atlas PFD stays drawn underneath the device quad, exactly like the ND:
 * it shares the same uploaded frame (no extra upload), and it keeps the PFD visible
 * if the device surface is ever not presented by the simulator. */
static int xpkit_pfd_atlas_needed(int side) { (void)side; return 1; }
static double xpkit_pfd_started;
static int xpkit_pfd_reported;

static void xpkit_pfd_draw_device(void *refcon) {
    XpKitPfd *d = refcon;
    GLboolean cull = glIsEnabled(GL_CULL_FACE);
    glDisable(GL_CULL_FACE);
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    glClearColor(0, 0, 0, 1);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glClear(GL_COLOR_BUFFER_BIT);
    /* Keep alpha = 1 (see a380x_xpkit_nd_input.inc): never a see-through screen or pop-up. */
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_FALSE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    if (plugin_enabled && cds_overlay(d->side ? A380X_ELN_DU_FO_PFD : A380X_ELN_DU_CAPT_PFD, 0, 0, 768, 1024)) {
        d->presented_frame = 0; d->draws++;
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        if (cull) glEnable(GL_CULL_FACE);
        return;
    }
    int on = xpkit_pfd_on(d);
    int rendered = on && xpkit_draw_view(d->side, 0, 0, 768, 1024);
    if (on && !rendered) {
        /* Native fallback: the translated PFD renders its 320x416 layout. */
        glPushMatrix();
        glScalef(768.0f / 320.0f, 1024.0f / 416.0f, 1);
        glLineWidth(1.0f);
        draw_pfd(&refs.side[d->side], 0, 0, d->side ? "F/O" : "CAPT");
        glPopMatrix();
        glLineWidth(1.0f);
    }
    d->presented_frame = rendered ? xpkit_uploaded[d->side] : 0;
    d->draws++; if (rendered) d->rendered_draws++;
    glColor4f(1, 1, 1, 1);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    if (cull) glEnable(GL_CULL_FACE);
}

static float xpkit_pfd_brightness(float rheo, float ambient, float bus, void *refcon) {
    XpKitPfd *d = refcon;
    (void)rheo; (void)ambient; (void)bus;
    int lit = xpkit_pfd_on(d) || (plugin_enabled && a380x_power_du_lit(d->side ? A380X_ELN_DU_FO_PFD : A380X_ELN_DU_CAPT_PFD, 0));
    return lit ? du_channels[d->side * DU_CHANNELS_PER_SIDE + DU_PFD].level : 0;
}

static void xpkit_pfd_toggle_popup(XpKitPfd *d) {
    if (d->device && mfda_av.loaded) {
        int show = !mfda_av.is_popup_visible(d->device);
        if (show) a380x_fit_popup(d->device, d->side, mfda_av.get_geometry, mfda_av.set_geometry);
        mfda_av.set_popup_visible(d->device, show);
    }
}
/* The PFD has no touch functions: a click (release) in the 3D cockpit toggles
 * the pop-up, ToLiss-style. Clicks inside the pop-up itself close it again. */
static int xpkit_pfd_touch(int x, int y, XPLMMouseStatus phase, void *refcon) {
    (void)x; (void)y;
    if (phase == xplm_MouseUp && plugin_enabled) xpkit_pfd_toggle_popup(refcon);
    return 1;
}
static int xpkit_pfd_right(int x, int y, XPLMMouseStatus phase, void *refcon) {
    (void)x; (void)y; (void)phase; (void)refcon;
    return 0; /* keep right-drag camera look */
}
static int xpkit_pfd_command(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon) {
    XpKitPfd *d = refcon;
    if (phase != xplm_CommandBegin || !plugin_enabled || !d->device || !mfda_av.loaded) return 1;
    if (cmd == d->command[0]) xpkit_pfd_toggle_popup(d);
    else if (cmd == d->command[1]) {
        if (!mfda_av.is_popup_visible(d->device)) {
            a380x_fit_popup(d->device, d->side, mfda_av.get_geometry, mfda_av.set_geometry);
            mfda_av.set_popup_visible(d->device, 1);
        }
        mfda_av.pop_out(d->device);
    }
    return 1;
}

/* Called from xpkit_update(): request a device draw only when it can change. */
static void xpkit_pfd_poll(void) {
    for (int s = 0; s < 2; ++s) {
        XpKitPfd *d = &xpkit_pfd[s];
        /* The CDS state (boot / self-test / on) also triggers a redraw of the device. */
        int powered = xpkit_pfd_on(d) ? 8 : a380x_power_du_state(s ? A380X_ELN_DU_FO_PFD : A380X_ELN_DU_CAPT_PFD) + 1;
        int live = powered == 8 && xpkit_shared && xpkit_health[0] && xpkit_health[s + 1];
        unsigned seq = xpkit_shared ? atomic_load(&xpkit_shared->frame_seq[s]) : 0;
        xpkit_health[20 + s] = xpkit_pfd_bound(d);
        if (!d->device || !mfda_av.loaded) continue;
        /* New library frame, a live/power transition, or a continuously
         * animating native fallback. A dark, unchanged device costs nothing. */
        if ((live && seq != d->presented_frame) || live != d->draw_live ||
            powered != d->draw_powered || (powered == 8 && !live))
            mfda_av.needs_drawing(d->device);
        d->draw_live = live;
        d->draw_powered = powered;
    }
    /* Show the device quads (a380x/xp12/xpkit/pfd_devices_active) only once both devices
     * are bound and have presented a library frame; until then the atlas PFD shows. */
    xpkit_health[19] = xpkit_pfd[0].device && xpkit_pfd[1].device && xpkit_health[20] && xpkit_health[21] &&
                       xpkit_pfd[0].rendered_draws > 0 && xpkit_pfd[1].rendered_draws > 0;
    /* One diagnostic line ~10 s after the helper starts: proves binding and presentation. */
    if (!xpkit_pfd_started) xpkit_pfd_started = xpkit_now();
    if (!xpkit_pfd_reported && xpkit_now() - xpkit_pfd_started > 10.0) {
        char line[320];
        xpkit_pfd_reported = 1;
        snprintf(line, sizeof(line), "A380X PFD devices: api=%d created=%d/%d bound=%d/%d live=%d/%d powered=%d/%d "
                 "device_draws=%u/%u library_draws=%u/%u frames=%d/%d quads=%d\n", mfda_av.loaded,
                 xpkit_pfd[0].device != NULL, xpkit_pfd[1].device != NULL, xpkit_health[20], xpkit_health[21],
                 xpkit_pfd[0].draw_live, xpkit_pfd[1].draw_live, xpkit_pfd[0].draw_powered, xpkit_pfd[1].draw_powered,
                 xpkit_pfd[0].draws, xpkit_pfd[1].draws, xpkit_pfd[0].rendered_draws, xpkit_pfd[1].rendered_draws,
                 xpkit_health[4], xpkit_health[5], xpkit_health[19]);
        XPLMDebugString(line);
    }
}

static void xpkit_pfd_renderer_lost(void) {
    for (int s = 0; s < 2; ++s) {
        xpkit_pfd[s].presented_frame = 0;
        if (xpkit_pfd[s].device && mfda_av.loaded) mfda_av.needs_drawing(xpkit_pfd[s].device);
    }
}

/* XPluginStart (via xpkit_init): devices must exist before the aircraft OBJs
 * that reference them load, or X-Plane drops their click manipulators. */
static void xpkit_pfd_init(void) {
    if (!mfda_av.loaded) mfda_av_load();
    for (int s = 0; s < 2; ++s) {
        XpKitPfd *d = &xpkit_pfd[s];
        char name[128];
        d->side = s;
        for (int a = 0; a < 2; ++a) {
            snprintf(name, sizeof(name), "a380x/xp12/pfd/%s/%s", s ? "first_officer" : "captain", xpkit_pfd_actions[a]);
            d->command[a] = XPLMCreateCommand(name, a ? "A380X PFD pop-out window" : "A380X PFD pop-up toggle");
            XPLMRegisterCommandHandler(d->command[a], xpkit_pfd_command, 1, d);
        }
        if (mfda_av.loaded) {
            XPLMCreateAvionics_t p;
            memset(&p, 0, sizeof(p));
            p.structSize = sizeof(p);
            p.screenWidth = p.bezelWidth = 768;
            p.screenHeight = p.bezelHeight = 1024;
            p.drawOnDemand = 1;
            p.drawCallback = xpkit_pfd_draw_device;
            p.bezelDrawCallback = a380x_black_bezel;
            p.screenTouchCallback = xpkit_pfd_touch;
            p.screenRightTouchCallback = xpkit_pfd_right;
            p.brightnessCallback = xpkit_pfd_brightness;
            p.refcon = d;
            p.deviceID = s ? "a380x.xp12.pfd.first_officer" : "a380x.xp12.pfd.captain";
            p.deviceName = s ? "A380X FlyByWire PFD F/O" : "A380X FlyByWire PFD CAPT";
            d->device = mfda_av.create(&p);
        }
    }
    xpkit_health[19] = 0; /* raised by xpkit_pfd_poll once both devices present frames */
}

static void xpkit_pfd_shutdown(void) {
    for (int s = 0; s < 2; ++s) {
        XpKitPfd *d = &xpkit_pfd[s];
        if (d->device && mfda_av.loaded) mfda_av.destroy(d->device);
        for (int a = 0; a < 2; ++a)
            if (d->command[a]) XPLMUnregisterCommandHandler(d->command[a], xpkit_pfd_command, 1, d);
        memset(d, 0, sizeof(*d));
    }
    xpkit_health[19] = xpkit_health[20] = xpkit_health[21] = 0;
    xpkit_pfd_started = 0; xpkit_pfd_reported = 0;
}
