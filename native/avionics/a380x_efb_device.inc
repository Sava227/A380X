/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * The two cockpit EFB tablets as X-Plane plugin cockpit devices (a380x.xp12.efb.captain /
 * .first_officer, 1440x1000 texels: twice the old 720x500 atlas region, same aspect). Each device
 * presents its web page texture (xp-kit view 7/8, a380x_xpkit_efb.inc) when that renderer is live,
 * and otherwise the native flyPad renderer of a380x_efb_render.inc, so a tablet never freezes or
 * goes blank because the helper is missing (e.g. Windows builds without the WebView2 host).
 * Touches (tap and drag) are recorded in order and sent to the page in its frame; on the native
 * fallback a press is mapped to the legacy atlas hit test. Devices are created in XPluginStart
 * (a380x_efb_init), before X-Plane loads the cockpit OBJs that reference them.
 *
 * Included by a380x_efb.c after a380x_efb_web.inc.
 */
#define EFB_DEVICE_W 1440
#define EFB_DEVICE_H 1000
static const char *const efb_device_ids[2] = {"a380x.xp12.efb.captain", "a380x.xp12.efb.first_officer"};
static const char *const efb_device_names[2] = {"A380X EFB CAPT", "A380X EFB F/O"};
static A380XEfbWebHost efb_web_host;
static float efb_fallback_clock[2];

void a380x_efb_set_web_host(const A380XEfbWebHost *host)
{
    if (host != NULL) efb_web_host = *host;
    else memset(&efb_web_host, 0, sizeof(efb_web_host));
}

static int efb_web_live(int tablet)
{
    return efb_web_host.live != NULL && efb_web_host.live(tablet) != 0;
}

#if defined(XPLM410)
/* ---- Touch events --------------------------------------------------------------------------------- */
static void efb_touch_event(EfbTablet *t, int type, int x, int y)
{
    unsigned seq = ++t->touch_serial;
    if (seq == 0) seq = t->touch_serial = 1;
    t->events[(seq - 1) % EFB_TOUCH_EVENTS] = (EfbTouchEvent){seq, type, x, y};
    ++efb_touch_count;
    efb_changed();
}

/* Native fallback renderer: the legacy layout at twice its atlas size, full ink (the device's
 * brightness callback applies the tablet brightness). */
static void efb_draw_fallback(void)
{
    efb_host_left = 0; efb_host_bottom = 0;
    efb_host_right = EFB_DEVICE_W; efb_host_top = EFB_DEVICE_H;
    efb_in_window = 1;
    efb_render_shared();
    efb_in_window = 0;
    efb_host_left = 152; efb_host_bottom = 516;
    efb_host_right = 872; efb_host_top = 1016;
}

/* Device texels (x right, y up from the bottom-left) -> legacy atlas coordinates. */
static void efb_device_to_atlas(int x, int y, float *atlas_x, float *atlas_y)
{
    *atlas_x = EFB_LEFT + (float)x * (EFB_RIGHT - EFB_LEFT) / EFB_DEVICE_W;
    *atlas_y = EFB_BOTTOM + (float)y * (EFB_TOP - EFB_BOTTOM) / EFB_DEVICE_H;
}

/* One touch sample: returns 1 when it was handled (always: the tablet consumes its screen). */
static int efb_tablet_touch(int index, int x, int y, int phase)
{
    EfbTablet *t = &tablets[index];
    int positionless = x < 0 || y < 0 || x >= EFB_DEVICE_W || y >= EFB_DEVICE_H || (x == 0 && y == 0);
    /* X-Plane 12.4 sends drags/releases without a position ((-16,-16) or (0,0)): finish the
     * gesture where it was instead of in a corner. */
    if (phase != 1 && positionless && t->last_x >= 0) { x = t->last_x; y = t->last_y; }
    if (x < 0) x = 0;
    if (x >= EFB_DEVICE_W) x = EFB_DEVICE_W - 1;
    if (y < 0) y = 0;
    if (y >= EFB_DEVICE_H) y = EFB_DEVICE_H - 1;
    t->last_x = x; t->last_y = y;
    if (efb_tablet_blank()) {
        if (phase == 1 && !powered) { powered = 1; set_status("Tablet woken"); efb_changed(); }
        t->pressed = 0;
        return 1;
    }
    if (efb_web_live(index)) {
        /* Page coordinates: origin top-left, CSS pixels of the 1440x1000 page. */
        efb_touch_event(t, phase == 1 ? 1 : phase == 2 ? 2 : 0, x, EFB_DEVICE_H - 1 - y);
        t->pressed = phase != 2;
        return 1;
    }
    if (phase == 1) {
        float atlas_x, atlas_y;
        efb_device_to_atlas(x, y, &atlas_x, &atlas_y);
        a380x_efb_click(atlas_x, atlas_y);
        efb_changed();
    }
    t->pressed = phase != 2;
    return 1;
}

typedef XPLMAvionicsID (*EfbAvCreate)(XPLMCreateAvionics_t *);
typedef void (*EfbAvHandle)(XPLMAvionicsID);
typedef int (*EfbAvQuery)(XPLMAvionicsID);
typedef void (*EfbAvSetInt)(XPLMAvionicsID, int);
static struct {
    EfbAvCreate create;
    EfbAvHandle destroy, needs_drawing;
    EfbAvQuery is_bound, is_popup_visible;
    EfbAvSetInt set_popup_visible;
    int loaded;
} efb_av;

static int efb_av_load(void)
{
    int xp = 0, xplm = 0;
    XPLMHostApplicationID host;
    memset(&efb_av, 0, sizeof(efb_av));
    XPLMGetVersions(&xp, &xplm, &host);
    if (xplm < 410) return 0;   /* SDK 4.1 = X-Plane 12.1: no cockpit devices before it */
#define EFB_AV(field, type, name) if ((efb_av.field = (type)XPLMFindSymbol(name)) == NULL) return 0
    EFB_AV(create, EfbAvCreate, "XPLMCreateAvionicsEx");
    EFB_AV(destroy, EfbAvHandle, "XPLMDestroyAvionics");
    EFB_AV(needs_drawing, EfbAvHandle, "XPLMAvionicsNeedsDrawing");
    EFB_AV(is_bound, EfbAvQuery, "XPLMIsAvionicsBound");
    EFB_AV(is_popup_visible, EfbAvQuery, "XPLMIsAvionicsPopupVisible");
    EFB_AV(set_popup_visible, EfbAvSetInt, "XPLMSetAvionicsPopupVisible");
#undef EFB_AV
    efb_av.loaded = 1;
    return 1;
}

static void efb_device_draw(void *refcon)
{
    EfbTablet *t = refcon;
    int index = (int)(t - tablets);
    GLboolean cull = glIsEnabled(GL_CULL_FACE);
    glDisable(GL_CULL_FACE);
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    t->blank_drawn = efb_tablet_blank();
    if (!t->blank_drawn) {
        if (efb_web_live(index) && efb_web_host.draw != NULL && efb_web_host.draw(index, EFB_DEVICE_W, EFB_DEVICE_H)) {
            ++t->web_draws;
        } else {
            efb_draw_fallback();
            ++t->fallback_draws;
        }
    }
    ++t->draws;
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    if (cull) glEnable(GL_CULL_FACE);
}

static int efb_device_touch(int x, int y, XPLMMouseStatus status, void *refcon)
{
    EfbTablet *t = refcon;
    int index = (int)(t - tablets);
    efb_tablet_touch(index, x, y, status == xplm_MouseDown ? 1 : status == xplm_MouseUp ? 2 : 0);
    if (t->device && efb_av.loaded) efb_av.needs_drawing(t->device);
    return 1;
}

static int efb_device_right(int x, int y, XPLMMouseStatus status, void *refcon)
{
    (void)x; (void)y; (void)status; (void)refcon;
    return 0;   /* keep right-drag camera look */
}

static XPLMCursorStatus efb_device_cursor(int x, int y, void *refcon)
{
    (void)x; (void)y; (void)refcon;
    return xplm_CursorDefault;   /* the OBJ's ATTR_manip_device "button" (pointing finger) */
}

static float efb_device_brightness(float rheo, float ambient, float bus, void *refcon)
{
    (void)rheo; (void)ambient; (void)bus; (void)refcon;
    /* The tablet runs on its own battery: neither the cockpit bus nor a rheostat gates it. */
    return efb_tablet_blank() ? 0.0f : brightness / 100.0f;
}

static void efb_devices_create(void)
{
    int i;
    if (!efb_av_load()) {
        XPLMDebugString("A380X EFB: X-Plane cockpit-device API unavailable; tablets keep the panel-atlas renderer.\n");
        return;
    }
    for (i = 0; i < 2; ++i) {
        XPLMCreateAvionics_t p;
        if (tablets[i].device) continue;
        memset(&p, 0, sizeof(p));
        p.structSize = sizeof(p);
        p.screenWidth = p.bezelWidth = EFB_DEVICE_W;
        p.screenHeight = p.bezelHeight = EFB_DEVICE_H;
        p.drawOnDemand = 1;
        p.drawCallback = efb_device_draw;
        p.screenTouchCallback = efb_device_touch;
        p.screenRightTouchCallback = efb_device_right;
        p.screenCursorCallback = efb_device_cursor;
        p.brightnessCallback = efb_device_brightness;
        p.deviceID = (char *)efb_device_ids[i];
        p.deviceName = (char *)efb_device_names[i];
        p.refcon = &tablets[i];
        tablets[i].device = efb_av.create(&p);
        if (tablets[i].device == NULL) XPLMDebugString("A380X EFB: cockpit device creation failed; panel-atlas renderer stays active.\n");
    }
}

static void efb_devices_destroy(void)
{
    int i;
    for (i = 0; i < 2; ++i) {
        if (tablets[i].device && efb_av.loaded) efb_av.destroy(tablets[i].device);
        tablets[i].device = NULL;
        tablets[i].bound = 0;
    }
}

void a380x_efb_needs_drawing(int tablet)
{
    if (tablet >= 0 && tablet < 2 && tablets[tablet].device && efb_av.loaded) efb_av.needs_drawing(tablets[tablet].device);
}

static void efb_toggle_popup(int tablet)
{
    EfbTablet *t = &tablets[tablet];
    if (t->device && efb_av.loaded) efb_av.set_popup_visible(t->device, !efb_av.is_popup_visible(t->device));
    else set_status("Tablet pop-up needs X-Plane 12.1 cockpit devices");
}

static void efb_devices_update(float dt)
{
    int i;
    for (i = 0; i < 2; ++i) {
        EfbTablet *t = &tablets[i];
        int live = efb_web_live(i), blank = efb_tablet_blank();
        t->bound = t->device && efb_av.loaded && efb_av.is_bound(t->device);
        if (!t->device || !efb_av.loaded) { t->web_live = live; continue; }
        /* Redraw on a renderer or power transition, and keep the native fallback animated at 4 Hz. */
        efb_fallback_clock[i] += dt;
        if (live != t->web_live || blank != t->blank_drawn || (!live && !blank && efb_fallback_clock[i] >= 0.25f)) {
            efb_fallback_clock[i] = 0;
            efb_av.needs_drawing(t->device);
        }
        t->web_live = live;
    }
}

static int efb_devices_bound(void)
{
    return tablets[0].bound && tablets[1].bound;
}
#else
static void efb_devices_create(void) {}
static void efb_devices_destroy(void) {}
static void efb_devices_update(float dt) { int i; (void)dt; for (i = 0; i < 2; ++i) tablets[i].web_live = efb_web_live(i); }
static int efb_devices_bound(void) { return 0; }
static void efb_toggle_popup(int tablet) { (void)tablet; set_status("Tablet pop-up needs X-Plane 12.1 cockpit devices"); }
void a380x_efb_needs_drawing(int tablet) { (void)tablet; }
#endif

/* The legacy panel-atlas tablet is drawn only while the OBJ has not bound both devices (an aircraft
 * package from before the device geometry); with devices it would be wasted work. */
int a380x_efb_atlas_needed(void) { return !efb_devices_bound(); }

/* ---- Datarefs ------------------------------------------------------------------------------------ */
enum {
    EFBR_SLIDES_COUNT, EFBR_DOORS_OPEN, EFBR_DEVICES, EFBR_BOUND0, EFBR_BOUND1, EFBR_LIVE0, EFBR_LIVE1,
    EFBR_PROCESSED, EFBR_REJECTED, EFBR_TOUCHES, EFBR_COUNT
};
static const char *const efb_dataref_names[EFBR_COUNT] = {
    "a380x/xp12/efb/slides_armed_count", "a380x/xp12/efb/doors_open_count", "a380x/xp12/efb/devices_active",
    "a380x/xp12/efb/captain/device_bound", "a380x/xp12/efb/first_officer/device_bound",
    "a380x/xp12/efb/captain/web_live", "a380x/xp12/efb/first_officer/web_live",
    "a380x/xp12/efb/actions_processed", "a380x/xp12/efb/actions_rejected", "a380x/xp12/efb/touch_events"
};
static XPLMDataRef efb_datarefs[EFBR_COUNT], efb_slides_ref;

static int efb_read_int(void *refcon)
{
    float doors[A380X_EFB_DOORS];
    switch ((int)(intptr_t)refcon) {
    case EFBR_SLIDES_COUNT: return a380x_efb_slides_armed(slide_armed);
    case EFBR_DOORS_OPEN: efb_read_doors(doors); return a380x_efb_doors_open(doors);
    case EFBR_DEVICES: return tablets[0].device != NULL && tablets[1].device != NULL;
    case EFBR_BOUND0: return tablets[0].bound;
    case EFBR_BOUND1: return tablets[1].bound;
    case EFBR_LIVE0: return tablets[0].web_live;
    case EFBR_LIVE1: return tablets[1].web_live;
    case EFBR_PROCESSED: return (int)efb_actions_processed;
    case EFBR_REJECTED: return (int)efb_actions_rejected;
    case EFBR_TOUCHES: return (int)efb_touch_count;
    default: return 0;
    }
}

static int efb_read_slides(void *refcon, int *values, int offset, int count)
{
    int i;
    (void)refcon;
    if (values == NULL) return A380X_EFB_SLIDES;
    if (offset < 0 || offset >= A380X_EFB_SLIDES || count <= 0) return 0;
    if (count > A380X_EFB_SLIDES - offset) count = A380X_EFB_SLIDES - offset;
    for (i = 0; i < count; ++i) values[i] = slide_armed[offset + i];
    return count;
}

static int efb_datarefs_register(void)
{
    int i;
    efb_slides_ref = XPLMRegisterDataAccessor("a380x/xp12/efb/slide_armed", xplmType_IntArray, 0,
        NULL, NULL, NULL, NULL, NULL, NULL, efb_read_slides, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    if (efb_slides_ref == NULL) return 0;
    for (i = 0; i < EFBR_COUNT; ++i) {
        efb_datarefs[i] = XPLMRegisterDataAccessor(efb_dataref_names[i], xplmType_Int, 0, efb_read_int, NULL,
            NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, (void *)(intptr_t)i, NULL);
        if (efb_datarefs[i] == NULL) return 0;
    }
    return 1;
}

static void efb_datarefs_unregister(void)
{
    int i;
    if (efb_slides_ref) XPLMUnregisterDataAccessor(efb_slides_ref);
    efb_slides_ref = NULL;
    for (i = 0; i < EFBR_COUNT; ++i) {
        if (efb_datarefs[i]) XPLMUnregisterDataAccessor(efb_datarefs[i]);
        efb_datarefs[i] = NULL;
    }
}
