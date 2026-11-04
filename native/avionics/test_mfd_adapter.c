/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Mock-XPLM test of the MFD X-Plane boundary (a380x_mfd_adapter.inc, docs/mfd-spec/09 section 2.4):
 * device struct and XPLMFindSymbol loading, texel -> canvas conversion, side isolation of the
 * KCCU commands, DU-off gating, keyboard routing (sniffer vs popup focus, ENT release, Ctrl pass),
 * brightness law, F0 fallback when a symbol is missing, and reload cleanup.
 *
 *   test_mfd_adapter              run the checks (one summary line)
 *   test_mfd_adapter --contracts  print C <command>, D <dataref>, V <device id> lines
 *
 * Link with the MFD core objects (a380x_mfd*.c, a380x_fms*.c, a380x_navdata.c, page modules).
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if APL
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMMenus.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

#include "a380x_mfd.h"
#include "a380x_fms.h"

#define ARRAY_LEN(a) ((int)(sizeof(a) / sizeof((a)[0])))

static int g_checks, g_fail;
#define CHECK(c) do { ++g_checks; if (!(c)) { ++g_fail; fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); } } while (0)

/* ---- XPLM mocks ------------------------------------------------------------------------------ */

typedef struct {
    char name[96];
    int active;
    XPLMDataTypeID types;
    int writable;
    XPLMGetDatai_f geti;
    XPLMSetDatai_f seti;
    XPLMGetDataf_f getf;
    XPLMGetDatavf_f getvf;
    XPLMSetDatavf_f setvf;
    XPLMGetDatab_f getb;
    void *refcon;
} MockRef;
static MockRef mock_refs[512];
static int mock_ref_count;

XPLMDataRef XPLMFindDataRef(const char *name)
{
    int i;
    for (i = 0; i < mock_ref_count; ++i) if (mock_refs[i].active && !strcmp(mock_refs[i].name, name)) return &mock_refs[i];
    return NULL;
}

XPLMDataRef XPLMRegisterDataAccessor(const char *name, XPLMDataTypeID types, int writable,
    XPLMGetDatai_f geti, XPLMSetDatai_f seti, XPLMGetDataf_f getf, XPLMSetDataf_f setf,
    XPLMGetDatad_f getd, XPLMSetDatad_f setd, XPLMGetDatavi_f getvi, XPLMSetDatavi_f setvi,
    XPLMGetDatavf_f getvf, XPLMSetDatavf_f setvf, XPLMGetDatab_f getb, XPLMSetDatab_f setb,
    void *readref, void *writeref)
{
    MockRef *r;
    (void)getf; (void)setf; (void)getd; (void)setd; (void)getvi; (void)setvi; (void)getvf; (void)setvf; (void)setb;
    (void)writeref;
    assert(!XPLMFindDataRef(name));
    assert(mock_ref_count < ARRAY_LEN(mock_refs));
    r = &mock_refs[mock_ref_count++];
    memset(r, 0, sizeof(*r));
    snprintf(r->name, sizeof(r->name), "%s", name);
    r->active = 1; r->types = types; r->writable = writable;
    r->geti = geti; r->seti = seti; r->getf = getf; r->getvf = getvf; r->setvf = setvf;
    r->getb = getb; r->refcon = readref;
    return r;
}

void XPLMUnregisterDataAccessor(XPLMDataRef ref) { MockRef *r = (MockRef *)ref; assert(r && r->active); r->active = 0; }
XPLMDataTypeID XPLMGetDataRefTypes(XPLMDataRef ref) { return ((MockRef *)ref)->types; }
int XPLMGetDatai(XPLMDataRef ref) { MockRef *r=ref;return r->geti?r->geti(r->refcon):0; }
float XPLMGetDataf(XPLMDataRef ref) { MockRef *r=ref;return r->getf?r->getf(r->refcon):0; }
int XPLMGetDatavf(XPLMDataRef ref,float *out,int offset,int count)
{ MockRef *r=ref;return r->getvf?r->getvf(r->refcon,out,offset,count):0; }
void XPLMSetDatavf(XPLMDataRef ref,float *in,int offset,int count)
{ MockRef *r=ref;if(r->setvf)r->setvf(r->refcon,in,offset,count); }

static int active_refs(void)
{
    int i, n = 0;
    for (i = 0; i < mock_ref_count; ++i) n += mock_refs[i].active;
    return n;
}

typedef struct {
    char name[96];
    XPLMCommandCallback_f handler;
    void *refcon;
    int active;
} MockCommand;
static MockCommand mock_commands[192];
static int mock_command_count;

XPLMCommandRef XPLMFindCommand(const char *name)
{
    int i;
    for (i = 0; i < mock_command_count; ++i) if (!strcmp(mock_commands[i].name, name)) return &mock_commands[i];
    return NULL;
}

XPLMCommandRef XPLMCreateCommand(const char *name, const char *description)
{
    MockCommand *c;
    (void)description;
    assert(!XPLMFindCommand(name) && mock_command_count < ARRAY_LEN(mock_commands));
    c = &mock_commands[mock_command_count++];
    memset(c, 0, sizeof(*c));
    snprintf(c->name, sizeof(c->name), "%s", name);
    return c;
}

void XPLMRegisterCommandHandler(XPLMCommandRef ref, XPLMCommandCallback_f handler, int before, void *refcon)
{
    MockCommand *c = (MockCommand *)ref;
    assert(before && !c->active);
    c->handler = handler; c->refcon = refcon; c->active = 1;
}

void XPLMUnregisterCommandHandler(XPLMCommandRef ref, XPLMCommandCallback_f handler, int before, void *refcon)
{
    MockCommand *c = (MockCommand *)ref;
    assert(before && c->active && c->handler == handler && c->refcon == refcon);
    c->active = 0;
}

static int active_commands(void)
{
    int i, n = 0;
    for (i = 0; i < mock_command_count; ++i) n += mock_commands[i].active;
    return n;
}

static void run_command(const char *name, XPLMCommandPhase phase)
{
    MockCommand *c = (MockCommand *)XPLMFindCommand(name);
    assert(c && c->active);
    c->handler(c, phase, c->refcon);
}

static float mock_clock = 100.0f;
float XPLMGetElapsedTime(void) { return mock_clock; }
void XPLMDebugString(const char *s) { (void)s; }
void XPLMSetGraphicsState(int a, int b, int c, int d, int e, int f, int g)
{ (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; (void)g; }

static int mock_xplm_version = 430;
void XPLMGetVersions(int *xp, int *xplm, XPLMHostApplicationID *host) { *xp = 12430; *xplm = mock_xplm_version; *host = xplm_Host_XPlane; }

static XPLMKeySniffer_f mock_sniffer;
static int mock_sniffers;
int XPLMRegisterKeySniffer(XPLMKeySniffer_f cb, int before, void *refcon)
{ (void)refcon; assert(!before); mock_sniffer = cb; ++mock_sniffers; return 1; }
int XPLMUnregisterKeySniffer(XPLMKeySniffer_f cb, int before, void *refcon)
{ (void)refcon; assert(!before && cb == mock_sniffer); --mock_sniffers; return 1; }

static XPLMWindowID mock_focus;
void XPLMTakeKeyboardFocus(XPLMWindowID w) { mock_focus = w; }
int XPLMHasKeyboardFocus(XPLMWindowID w) { return w != NULL && w == mock_focus; }
void XPLMGetScreenBoundsGlobal(int *l, int *t, int *r, int *b) { *l = 0; *t = 1600; *r = 2560; *b = 0; }

static int mock_menus;
XPLMMenuID XPLMFindAircraftMenu(void) { return (XPLMMenuID)(intptr_t)7; }
int XPLMAppendMenuItem(XPLMMenuID m, const char *s, void *r, int f) { (void)m; (void)s; (void)r; (void)f; return 3; }
XPLMMenuID XPLMCreateMenu(const char *s, XPLMMenuID p, int i, XPLMMenuHandler_f f, void *r)
{ (void)s; (void)p; (void)i; (void)f; (void)r; ++mock_menus; return (XPLMMenuID)(intptr_t)8; }
int XPLMAppendMenuItemWithCommand(XPLMMenuID m, const char *s, XPLMCommandRef c) { (void)m; (void)s; assert(c); return 0; }
void XPLMDestroyMenu(XPLMMenuID m) { assert(m == (XPLMMenuID)(intptr_t)8); --mock_menus; }
void XPLMRemoveMenuItem(XPLMMenuID m, int i) { assert(m == (XPLMMenuID)(intptr_t)7 && i == 3); }

typedef struct { XPLMCreateWindow_t p; int visible, alive, mode; } MockWindow;
static MockWindow mock_windows[16];
static int mock_window_count;
XPLMWindowID XPLMCreateWindowEx(XPLMCreateWindow_t *p)
{
    MockWindow *w;
    assert(mock_window_count < ARRAY_LEN(mock_windows));
    w = &mock_windows[mock_window_count++];
    w->p = *p; w->visible = p->visible; w->alive = 1;
    return w;
}
void XPLMDestroyWindow(XPLMWindowID id) { MockWindow *w = (MockWindow *)id; assert(w->alive); w->alive = 0; }
void XPLMGetWindowGeometry(XPLMWindowID id, int *l, int *t, int *r, int *b)
{ MockWindow *w = (MockWindow *)id; *l = w->p.left; *t = w->p.top; *r = w->p.right; *b = w->p.bottom; }
void XPLMSetWindowPositioningMode(XPLMWindowID id, XPLMWindowPositioningMode m, int i) { (void)i; ((MockWindow *)id)->mode = (int)m; }
void XPLMSetWindowResizingLimits(XPLMWindowID id, int a, int b, int c, int d) { (void)id; (void)a; (void)b; (void)c; (void)d; }
void XPLMSetWindowTitle(XPLMWindowID id, const char *s) { (void)id; assert(!strncmp(s, "A380X MFD", 9)); }
void XPLMSetWindowIsVisible(XPLMWindowID id, int v) { ((MockWindow *)id)->visible = v; }
int XPLMGetWindowIsVisible(XPLMWindowID id) { return ((MockWindow *)id)->visible; }
void XPLMBringWindowToFront(XPLMWindowID id) { (void)id; }
int XPLMIsWindowInFront(XPLMWindowID id) { (void)id; return 1; }

/* GL: the callbacks only touch state; nothing is rasterised here. */
static GLboolean mock_cull = GL_TRUE;
GLboolean glIsEnabled(GLenum cap) { assert(cap == GL_CULL_FACE); return mock_cull; }
void glDisable(GLenum cap) { if (cap == GL_CULL_FACE) mock_cull = GL_FALSE; }
void glEnable(GLenum cap) { if (cap == GL_CULL_FACE) mock_cull = GL_TRUE; }
void glClearColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a) { (void)r; (void)g; (void)b; assert(a == 1.0f); }
static int mock_clears;
void glClear(GLbitfield m) { (void)m; ++mock_clears; }
void glColorMask(GLboolean r, GLboolean g, GLboolean b, GLboolean a) { (void)r; (void)g; (void)b; (void)a; }
void glBlendFunc(GLenum a, GLenum b) { (void)a; (void)b; }
void glColor4f(GLfloat r, GLfloat g, GLfloat b, GLfloat a) { (void)r; (void)g; (void)b; (void)a; }
void glColor3f(GLfloat r, GLfloat g, GLfloat b) { (void)r; (void)g; (void)b; }
void glBegin(GLenum m) { (void)m; }
void glEnd(void) {}
void glVertex2f(GLfloat x, GLfloat y) { (void)x; (void)y; }
void glVertex2i(GLint x, GLint y) { (void)x; (void)y; }
void glGetIntegerv(GLenum p, GLint *v) { (void)p; v[0] = 0; v[1] = 0; v[2] = MFD_W; v[3] = MFD_H; }
void glGetFloatv(GLenum p, GLfloat *v) { int i; (void)p; for (i = 0; i < 16; ++i) v[i] = (i % 5 == 0) ? 1.0f : 0.0f; }

/* ---- Avionics API mocks (reached only through XPLMFindSymbol) ------------------------------- */

typedef struct {
    XPLMCreateAvionics_t p;
    char id[80];
    int alive, popup, popped_out, keyboard, redraws, bound;
    float rheo;
    int over, over_x, over_y;
} MockDevice;
static MockDevice mock_devices[32];
static int mock_device_count, mock_missing_symbol;

static XPLMAvionicsID av_create(XPLMCreateAvionics_t *p)
{
    MockDevice *d;
    assert(mock_device_count < ARRAY_LEN(mock_devices));
    d = &mock_devices[mock_device_count++];
    memset(d, 0, sizeof(*d));
    d->p = *p;
    snprintf(d->id, sizeof(d->id), "%s", p->deviceID);
    d->alive = 1; d->bound = 1; d->rheo = -1.0f;
    return d;
}
static void av_destroy(XPLMAvionicsID h) { MockDevice *d = (MockDevice *)h; assert(d->alive); d->alive = 0; }
static void av_needs_drawing(XPLMAvionicsID h) { ((MockDevice *)h)->redraws++; }
static void av_pop_out(XPLMAvionicsID h) { ((MockDevice *)h)->popped_out = 1; }
static void av_take_keyboard(XPLMAvionicsID h) { MockDevice *d = (MockDevice *)h; d->keyboard = d->popup; }
static int av_is_bound(XPLMAvionicsID h) { return ((MockDevice *)h)->bound; }
static int av_is_popup_visible(XPLMAvionicsID h) { return ((MockDevice *)h)->popup; }
static int av_is_popped_out(XPLMAvionicsID h) { return ((MockDevice *)h)->popped_out; }
static int av_has_keyboard(XPLMAvionicsID h) { return ((MockDevice *)h)->keyboard; }
static void av_set_popup_visible(XPLMAvionicsID h, int v) { MockDevice *d = (MockDevice *)h; d->popup = v; if (!v) d->popped_out = d->keyboard = 0; }
static void av_set_rheo(XPLMAvionicsID h, float v) { ((MockDevice *)h)->rheo = v; }
static int av_cursor_over(XPLMAvionicsID h, int *x, int *y) { MockDevice *d = (MockDevice *)h; *x = d->over_x; *y = d->over_y; return d->over; }
static void av_get_geometry(XPLMAvionicsID h, int *l, int *t, int *r, int *b) { (void)h; *l = *t = *r = *b = 0; }
static int mock_set_geometry;
static void av_set_geometry(XPLMAvionicsID h, int l, int t, int r, int b) { (void)h; assert(r > l && t > b); ++mock_set_geometry; }

void *XPLMFindSymbol(const char *name)
{
    static const struct { const char *name; void *fn; } table[] = {
        {"XPLMCreateAvionicsEx", (void *)av_create}, {"XPLMDestroyAvionics", (void *)av_destroy},
        {"XPLMAvionicsNeedsDrawing", (void *)av_needs_drawing}, {"XPLMPopOutAvionics", (void *)av_pop_out},
        {"XPLMTakeAvionicsKeyboardFocus", (void *)av_take_keyboard}, {"XPLMIsAvionicsBound", (void *)av_is_bound},
        {"XPLMIsAvionicsPopupVisible", (void *)av_is_popup_visible}, {"XPLMIsAvionicsPoppedOut", (void *)av_is_popped_out},
        {"XPLMHasAvionicsKeyboardFocus", (void *)av_has_keyboard}, {"XPLMSetAvionicsPopupVisible", (void *)av_set_popup_visible},
        {"XPLMSetAvionicsBrightnessRheo", (void *)av_set_rheo}, {"XPLMIsCursorOverAvionics", (void *)av_cursor_over},
        {"XPLMGetAvionicsGeometry", (void *)av_get_geometry}, {"XPLMSetAvionicsGeometry", (void *)av_set_geometry},
    };
    size_t i;
    for (i = 0; i < sizeof(table) / sizeof(table[0]); ++i)
        if (!strcmp(table[i].name, name)) return mock_missing_symbol == (int)i + 1 ? NULL : table[i].fn;
    return NULL;
}

/* ---- What a380x_avionics.c provides before the adapter --------------------------------------- */

static int plugin_enabled = 1;
typedef struct { float level; } DuChannel;
enum { DU_PFD, DU_ND, DU_WX_TERR, DU_MFD, DU_CHANNELS_PER_SIDE };
static DuChannel du_channels[8] = {{1}, {1}, {1}, {1}, {1}, {1}, {1}, {1}};
static int gl_draws;
static uint32_t mfd_gl_draw(const MfdDrawList *dl, float ox, float oy, float scale, int use_scissor)
{
    (void)ox; (void)oy; (void)scale; (void)use_scissor;
    ++gl_draws;
    return dl ? dl->cmd_count : 0;
}

static int xpkit_draw_mfd(int side,float x,float y,float scale) {
    (void)side;(void)x;(void)y;(void)scale;return 0;
}
static void mfd_gl_cursor(const MfdDrawList *dl,float x,float y,float scale) {
    (void)dl;(void)x;(void)y;(void)scale;
}
#include "a380x_mfd_adapter.inc"
#include "a380x_xpkit_shared.h"
static A380XXpKitShared mock_shared;
static A380XXpKitShared *xpkit_shared=&mock_shared;
#include "a380x_xpkit_mfd.inc"

/* ---- Test page: one input field ------------------------------------------------------------- */

static MfdFieldValue g_value;
static int g_commits;
static void kbd_page(MfdPageContext *ctx)
{
    MfdFieldDesc d;
    MfdValue out;
    mfd_field_desc_init(&d, MFD_FMT_SPEED_KT);
    d.fmt.min = 90.0; d.fmt.max = 350.0;
    if (mfd_input_field(ctx->ui, MFD_ID(MFD_PAGE_ID_TEST, 1), 100.0f, 200.0f, &d, &g_value, &out) == MFD_FIELD_COMMITTED) {
        g_value.has_value = 1; g_value.num = out.num; ++g_commits;
    }
}

static A380XFms g_fms;

static void frame(int n)
{
    while (n-- > 0) { mock_clock += 0.05f; mfda_update(0.05f, 1); }
}

static void start(void)
{
    a380x_fms_init(&g_fms, NULL, NULL, NULL);
    assert(mfda_init(&g_fms, &g_fms.msgs, NULL));
    mfd_register_page("fms/test/kbd", MFD_PAGE_ID_TEST, "KBD", kbd_page, MFD_PAGE_FMS);
    mfda_devices_enable();
}

static void stop(void)
{
    mfda_devices_disable();
    mfda_shutdown();
}

static MockDevice *device(int side)
{
    int i;
    for (i = mock_device_count - 1; i >= 0; --i)
        if (mock_devices[i].alive && !strcmp(mock_devices[i].id, side ? "a380x.xp12.mfd.first_officer" : "a380x.xp12.mfd.captain"))
            return &mock_devices[i];
    return NULL;
}

/* Touch at a canvas pixel through the device callback (device texels are y-up). */
static void touch(int side, float cx, float cy, XPLMMouseStatus st)
{
    MockDevice *d = device(side);
    d->p.screenTouchCallback((int)cx, MFD_H - 1 - (int)cy, st, d->p.refcon);
}

static int contracts(void)
{
    int i;
    start();
    for (i = 0; i < mock_command_count; ++i) if (mock_commands[i].active) printf("C %s\n", mock_commands[i].name);
    for (i = 0; i < mock_ref_count; ++i) if (mock_refs[i].active) printf("D %s\n", mock_refs[i].name);
    for (i = 0; i < mock_device_count; ++i) if (mock_devices[i].alive) printf("V %s\n", mock_devices[i].id);
    stop();
    return 0;
}

static void test_devices(void)
{
    int s;
    float cx, cy;
    start();
    CHECK(mfda_devices_created && mock_device_count == 2);
    for (s = 0; s < 2; ++s) {
        MockDevice *d = device(s);
        CHECK(d != NULL);
        if (!d) continue;
        CHECK(d->p.structSize == (int)sizeof(XPLMCreateAvionics_t));
        CHECK(d->p.screenWidth == 768 && d->p.screenHeight == 1024 && d->p.drawOnDemand == 1);
        CHECK(d->p.bezelWidth == 800 && d->p.bezelHeight == 1080 && d->p.screenOffsetX == 16 && d->p.screenOffsetY == 16);
        CHECK(d->p.drawCallback && d->p.bezelDrawCallback && d->p.screenTouchCallback && d->p.screenScrollCallback
              && d->p.screenCursorCallback && d->p.keyboardCallback && d->p.brightnessCallback
              && d->p.screenRightTouchCallback && d->p.bezelClickCallback && d->p.bezelCursorCallback);
        CHECK(!strcmp(d->p.deviceName, s ? "A380X MFD F/O" : "A380X MFD CAPT"));
    }
    /* texel -> canvas at the four corners (device y up, canvas y down, pixel centres) */
    mfda_to_canvas(0, 0, &cx, &cy); CHECK(cx == 0.5f && cy == 1023.5f);
    mfda_to_canvas(767, 0, &cx, &cy); CHECK(cx == 767.5f && cy == 1023.5f);
    mfda_to_canvas(0, 1023, &cx, &cy); CHECK(cx == 0.5f && cy == 0.5f);
    mfda_to_canvas(767, 1023, &cx, &cy); CHECK(cx == 767.5f && cy == 0.5f);
    mfda_to_canvas(-20, 2000, &cx, &cy); CHECK(cx == 0.0f && cy == 0.0f);
    /* drawOnDemand: redraw only on a list change */
    frame(2);
    {
        MockDevice *d = device(0);
        int before = d->redraws;
        frame(1);
        CHECK(d->redraws == before);           /* static DATA/STATUS page, no cursor */
        touch(0, 300.0f, 600.0f, xplm_MouseDown);
        frame(1);
        CHECK(d->redraws == before + 1);       /* cursor appeared */
        touch(0, 300.0f, 600.0f, xplm_MouseUp);
        frame(1);
        /* draw callback: clears, keeps the cull state, runs the GL executor while powered */
        mock_cull = GL_TRUE;
        gl_draws = 0;
        d->p.drawCallback(d->p.refcon);
        CHECK(gl_draws == 1 && mock_cull == GL_TRUE);
        CHECK(d->rheo == 1.0f);                /* knob mirrored into the bound rheostat */
        CHECK(d->p.screenCursorCallback(10, 10, d->p.refcon) == xplm_CursorHidden);
    }
    stop();
}

static void test_side_isolation_and_du(void)
{
    start();
    frame(2);
    /* A captain KCCU key never reaches the F/O side (quirk 1). */
    run_command("a380x/xp12/kccu/captain/a", xplm_CommandBegin);
    CHECK(mfda_mfd.sides[0].ui.ev_count == 1 && mfda_mfd.sides[1].ui.ev_count == 0);
    frame(1);
    run_command("a380x/xp12/kccu/first_officer/perf", xplm_CommandBegin);
    frame(1);
    CHECK(!strcmp(a380x_mfd_active_uri(&mfda_mfd, MFD_FO), "fms/active/perf"));
    CHECK(strcmp(a380x_mfd_active_uri(&mfda_mfd, MFD_CAPT), "fms/active/perf") != 0);
    /* active_page dataref */
    {
        char buf[MFD_URI_LEN];
        MockRef *r = (MockRef *)XPLMFindDataRef("a380x/xp12/mfd/first_officer/active_page");
        int n = r ? r->getb(r->refcon, buf, 0, (int)sizeof(buf)) : 0;
        CHECK(n > 0 && !strcmp(buf, "fms/active/perf"));
    }
    /* DU off (MFD knob at 0): input ignored, screen black, brightness 0. */
    du_channels[1 * DU_CHANNELS_PER_SIDE + DU_MFD].level = 0.0f;
    frame(1);
    touch(1, 100.0f, 100.0f, xplm_MouseDown);
    touch(1, 100.0f, 100.0f, xplm_MouseUp);
    run_command("a380x/xp12/kccu/first_officer/init", xplm_CommandBegin);
    CHECK(mfda_mfd.sides[1].ui.ev_count == 0);
    CHECK(device(1)->p.brightnessCallback(1.0f, 0.7f, 1.0f, device(1)->p.refcon) == 0.0f);
    gl_draws = 0;
    device(1)->p.drawCallback(device(1)->p.refcon);
    CHECK(gl_draws == 0);
    du_channels[1 * DU_CHANNELS_PER_SIDE + DU_MFD].level = 1.0f;
    /* KBD / CCD switches */
    run_command("a380x/xp12/kccu/captain/kbd_switch_toggle", xplm_CommandBegin);
    CHECK(mfda_kccu_kbd_on[0] == 0 && mfda_kccu_kbd_on[1] == 1);
    run_command("a380x/xp12/kccu/first_officer/ccd_switch_toggle", xplm_CommandBegin);
    CHECK(mfda_kccu_ccd_on[1] == 0 && mfda_kccu_ccd_on[0] == 1);
    stop();
}

static void test_keyboard(void)
{
    MockDevice *d;
    start();
    a380x_mfd_navigate(&mfda_mfd, MFD_CAPT, "fms/test/kbd");
    frame(3);
    CHECK(!strcmp(a380x_mfd_active_uri(&mfda_mfd, MFD_CAPT), "fms/test/kbd"));
    CHECK(mfda_capture_side == -1);
    /* Unfocused: the sniffer passes everything. */
    CHECK(mock_sniffer('a', xplm_DownFlag, (char)XPLM_VK_A, NULL) == 1);
    touch(0, 120.0f, 215.0f, xplm_MouseDown);
    frame(1);
    touch(0, 120.0f, 215.0f, xplm_MouseUp);
    frame(1);
    CHECK(a380x_mfd_wants_keyboard(&mfda_mfd, MFD_CAPT));
    CHECK(mfda_capture_side == 0 && mfda_dev[0].keyboard_captured);
    /* Mapped keys are consumed (down and up); Ctrl/Alt shortcuts and F-keys pass. */
    CHECK(mock_sniffer('1', xplm_DownFlag, (char)XPLM_VK_1, NULL) == 0);
    CHECK(mock_sniffer('1', xplm_UpFlag, (char)XPLM_VK_1, NULL) == 0);
    CHECK(mock_sniffer('5', xplm_DownFlag, (char)XPLM_VK_5, NULL) == 0);
    CHECK(mock_sniffer('0', xplm_DownFlag, (char)XPLM_VK_0, NULL) == 0);
    CHECK(mock_sniffer('p', xplm_DownFlag | xplm_ControlFlag, (char)XPLM_VK_P, NULL) == 1);
    CHECK(mock_sniffer(0, xplm_DownFlag, (char)XPLM_VK_F1, NULL) == 1);
    frame(1);
    CHECK(mock_sniffer('\r', xplm_DownFlag, (char)XPLM_VK_RETURN, NULL) == 0);
    CHECK(mfda_capture_side == -1);  /* ENT releases the capture at once */
    frame(15);                       /* 0.5 s validation */
    CHECK(g_commits == 1 && g_value.num == 150.0);
    CHECK(!a380x_mfd_wants_keyboard(&mfda_mfd, MFD_CAPT));
    CHECK(mock_sniffer('a', xplm_DownFlag, (char)XPLM_VK_A, NULL) == 1);
    /* Popup visible: avionics keyboard focus instead of the sniffer. */
    run_command("a380x/xp12/mfd/captain/popup", xplm_CommandBegin);
    d = device(0);
    CHECK(d->popup == 1 && mock_set_geometry == 1);
    touch(0, 120.0f, 215.0f, xplm_MouseDown);
    frame(1);
    touch(0, 120.0f, 215.0f, xplm_MouseUp);
    frame(1);
    CHECK(d->keyboard == 1 && mfda_capture_side == -1 && mfda_dev[0].keyboard_captured);
    CHECK(d->p.keyboardCallback('2', xplm_DownFlag, (char)XPLM_VK_2, d->p.refcon, 0) == 1);
    CHECK(d->p.keyboardCallback(0, xplm_DownFlag, (char)XPLM_VK_F2, d->p.refcon, 0) == 0);
    /* Policy 0: never capture. */
    mfda_pc_capture_pref = 0;
    frame(1);
    CHECK(!mfda_dev[0].keyboard_captured && mfda_capture_side == -1);
    mfda_pc_capture_pref = 1;
    run_command("a380x/xp12/mfd/captain/popup", xplm_CommandBegin);
    CHECK(d->popup == 0);
    /* Aircraft reload: both sides back on DATA/STATUS. */
    mfda_plane_loaded();
    frame(1);
    CHECK(!strcmp(a380x_mfd_active_uri(&mfda_mfd, MFD_CAPT), "fms/data/status"));
    stop();
    CHECK(mfda_capture_side == -1);
}

static void test_zero_coordinate_release_and_data_entry(void)
{
    int s;
    start();
    g_commits = 0;
    memset(&g_value, 0, sizeof(g_value));
    for (s = 0; s < MFD_SIDE_COUNT; ++s) {
        MockDevice *d = device(s);
        a380x_mfd_navigate(&mfda_mfd, (MfdSide)s, "fms/test/kbd");
        frame(3);
        /* Reproduce the real log: valid press, then MouseUp at raw (0,0),
         * both before the next flight loop and with no hover callback. */
        touch(s, 120.0f, 215.0f, xplm_MouseDown);
        d->p.screenTouchCallback(0, 0, xplm_MouseUp, d->p.refcon);
        frame(5);
        CHECK(mfda_mfd.sides[s].ui.focus_id != 0);
        /* X-Plane 12 (Oct 2026 log): the release arrives at the off-screen
         * sentinel (-16,-16). It must still click the pressed field. */
        a380x_mfd_navigate(&mfda_mfd, (MfdSide)s, "fms/data/status");
        frame(3);
        a380x_mfd_navigate(&mfda_mfd, (MfdSide)s, "fms/test/kbd");
        frame(3);
        CHECK(mfda_mfd.sides[s].ui.focus_id == 0);
        touch(s, 120.0f, 215.0f, xplm_MouseDown);
        d->p.screenTouchCallback(-16, -16, xplm_MouseUp, d->p.refcon);
        frame(5);
        CHECK(mfda_mfd.sides[s].ui.focus_id != 0);
        CHECK(mfda_capture_side == s && mfda_dev[s].keyboard_captured);
        CHECK(mfda_dev[s].last_up_x == 120 && mfda_dev[s].last_up_y == 215);
        const char *digits = s ? "160" : "150";
        for (int n = 0; digits[n]; ++n)
            CHECK(mock_sniffer(digits[n], xplm_DownFlag,
                  (char)(XPLM_VK_0 + digits[n] - '0'), NULL) == 0);
        frame(4);
        CHECK(!strcmp(mfda_mfd.sides[s].ui.buf, digits));
        CHECK(mock_sniffer('\r', xplm_DownFlag, (char)XPLM_VK_RETURN, NULL) == 0);
        frame(15);
        CHECK(g_commits == s + 1 && g_value.num == (s ? 160.0 : 150.0));
        CHECK(mfda_capture_side == -1);
        /* Do not turn a real drag away from a field into a click on it. */
        touch(s, 120.0f, 215.0f, xplm_MouseDown);
        touch(s, 700.0f, 900.0f, xplm_MouseDrag);
        d->p.screenTouchCallback(-16, -16, xplm_MouseUp, d->p.refcon);
        frame(5);
        CHECK(mfda_mfd.sides[s].ui.focus_id == 0);
        CHECK(mfda_dev[s].last_up_x == 700 && mfda_dev[s].last_up_y == 900);
    }
    stop();
}

static void test_brightness(void)
{
    CHECK(mfda_brightness_law(1, 0.5f, 0.8f) == 0.4f);
    CHECK(mfda_brightness_law(1, 0.5f, 0.0f) == 0.5f);
    CHECK(mfda_brightness_law(0, 1.0f, 1.0f) == 0.0f);
    CHECK(mfda_brightness_law(1, 0.0f, 1.0f) == 0.0f);
    CHECK(mfda_brightness_law(1, 1.0f, 3.0f) == 1.0f);
}

/* Use the actual registered XPLM callbacks, not the browser input bridge. A
 * quick down/up can both arrive before the next simulator flight-loop frame. */
static void tap(int side, float x, float y)
{
    touch(side, x, y, xplm_MouseDown);
    touch(side, x, y, xplm_MouseUp);
    frame(5);
}

static void test_cockpit_dropdowns(void)
{
    static const struct { float x, y, item_x, item_y; const char *uri; } cases[] = {
        {175, 22, 80, 98, "atccom/connect"},
        {175, 22, 80, 134, "surv/controls"},
        {175, 22, 80, 170, "fcubkup/afs"},
        {100, 70, 80, 116, "fms/active/f-pln"},
        {100, 70, 80, 152, "fms/active/perf"},
        {100, 70, 80, 188, "fms/active/fuel-load"},
        {100, 70, 80, 260, "fms/active/init"},
        {290, 70, 280, 224, "fms/position/irs"},
        {480, 70, 470, 152, "fms/sec/index/2"},
        {680, 70, 660, 260, "fms/data/airport"},
    };
    start();
    frame(2);
    for (int side=0; side<2; ++side) {
        for (int n=0; n<ARRAY_LEN(cases); ++n) {
            a380x_mfd_navigate(&mfda_mfd, (MfdSide)side, "fms/data/status");
            frame(3);
            tap(side, cases[n].x, cases[n].y);
            CHECK(mfda_dev[side].menu_open);
            CHECK(mfda_mfd.sides[side].ui.open_menu != 0);
            CHECK(mfda_dev[side].last_down_x == (int)cases[n].x &&
                  mfda_dev[side].last_down_y == (int)cases[n].y);
            CHECK(mfda_dev[side].last_up_x == (int)cases[n].x &&
                  mfda_dev[side].last_up_y == (int)cases[n].y);
            CHECK(mfda_dev[side].press_target == mfda_dev[side].click_target);
            CHECK(mfda_dev[side].menu_open_count == n + 1);
            tap(side, cases[n].item_x, cases[n].item_y);
            if (strcmp(a380x_mfd_active_uri(&mfda_mfd, (MfdSide)side), cases[n].uri))
                fprintf(stderr, "dropdown %d seat %d: expected %s, got %s\n", n, side, cases[n].uri,
                        a380x_mfd_active_uri(&mfda_mfd, (MfdSide)side));
            CHECK(!strcmp(a380x_mfd_active_uri(&mfda_mfd, (MfdSide)side), cases[n].uri));
            CHECK(mfda_mfd.sides[side].page_index >= 0); /* not just a changed URI / NOT FOUND page */
            CHECK(!mfda_dev[side].menu_open);
            CHECK(!mfda_dev[side].events_dropped);
            CHECK(mfda_dev[side].device_bound);
        }
    }
    CHECK(mfda_dev[0].touch_down_count >= 20 && mfda_dev[0].touch_up_count >= 20);
    CHECK(mfda_dev[1].touch_down_count >= 20 && mfda_dev[1].touch_up_count >= 20);
    /* Read-only diagnostics cannot be used to fake a physical cockpit click. */
    MockRef *r = XPLMFindDataRef("a380x/xp12/mfd/captain/touch_down_count");
    CHECK(r && !r->writable && !r->seti && r->geti(r->refcon) >= 20);
    stop();
}

static void test_arrow_keys_and_held_kccu(void)
{
    MfdKey key;
    char out;
    CHECK(mfda_map_key(0, XPLM_VK_UP, &key, &out) && key == MFD_KEY_UP);
    CHECK(mfda_map_key(0, XPLM_VK_DOWN, &key, &out) && key == MFD_KEY_DOWN);
    CHECK(mfda_map_key(0, XPLM_VK_LEFT, &key, &out) && key == MFD_KEY_LEFT);
    CHECK(mfda_map_key(0, XPLM_VK_RIGHT, &key, &out) && key == MFD_KEY_RIGHT);
    start();
    frame(2);
    MockDevice *d = device(0);
    d->p.screenCursorCallback(100, MFD_H-1-70, d->p.refcon);
    frame(1);
    run_command("a380x/xp12/kccu/captain/click", xplm_CommandBegin);
    frame(3); /* No hover callback for several frames while the button is held. */
    CHECK(mfda_mfd.sides[0].ui.button_down);
    run_command("a380x/xp12/kccu/captain/click", xplm_CommandEnd);
    frame(3);
    CHECK(mfda_dev[0].menu_open);
    stop();
}

static void test_cursor_transport(void)
{
    start();
    memset(&mock_shared,0,sizeof(mock_shared));
    frame(3);
    xpkit_mfd_publish();
    uint32_t before=atomic_load(&mock_shared.mfd[0].sequence);
    CHECK(before>0);
    uint64_t full_before=mfda_list_hash(a380x_mfd_display_list(&mfda_mfd,MFD_CAPT),1);
    /* 1,000 pointer positions in empty page space: no page snapshots/uploads.
     * Raw display lists still change, so the local cursor is not frozen. */
    for(int n=0;n<1000;++n) {
        MockDevice *d=device(0);
        d->p.screenCursorCallback(300+n%80,MFD_H-1-(280+n%100),d->p.refcon);
        frame(1);
        xpkit_mfd_publish();
        if(atomic_load(&mock_shared.mfd[0].sequence)!=before) {
            CHECK(0);break;
        }
    }
    CHECK(full_before!=mfda_list_hash(a380x_mfd_display_list(&mfda_mfd,MFD_CAPT),1));
    CHECK(mfda_mfd.sides[0].ui.cursor_visible);
    for(uint32_t n=0;n<mock_shared.mfd[0].list.cmd_count;++n)
        CHECK(mock_shared.mfd[0].list.cmds[n].layer!=MFD_LAYER_CURSOR);
    CHECK(mock_shared.mfd[0].list.point_count+6==a380x_mfd_display_list(&mfda_mfd,MFD_CAPT)->point_count);
    tap(0,100,70);
    xpkit_mfd_publish();
    CHECK(atomic_load(&mock_shared.mfd[0].sequence)>before);
    CHECK(mfda_dev[0].menu_open);
    before=atomic_load(&mock_shared.mfd[0].sequence);
    tap(0,80,152);
    xpkit_mfd_publish();
    CHECK(!strcmp(mock_shared.mfd[0].uri,"fms/active/perf"));
    CHECK(atomic_load(&mock_shared.mfd[0].sequence)>before);
    stop();
}

static void test_fallback(void)
{
    int created = mock_device_count;
    mock_missing_symbol = 12; /* XPLMIsCursorOverAvionics */
    start();
    CHECK(!mfda_devices_created && mock_device_count == created);
    run_command("a380x/xp12/mfd/first_officer/popup", xplm_CommandBegin);
    CHECK(mfda_dev[1].window != NULL && XPLMGetWindowIsVisible(mfda_dev[1].window));
    frame(2);
    stop();
    CHECK(mfda_dev[1].window == NULL);
    mock_missing_symbol = 0;
    mock_xplm_version = 400; /* X-Plane 12.0: no devices either */
    start();
    CHECK(!mfda_devices_created);
    stop();
    mock_xplm_version = 430;
}

static void test_reload(void)
{
    int i, commands = -1;
    for (i = 0; i < 3; ++i) {
        start();
        CHECK(active_refs() == MFDA_REF_COUNT && active_commands() == 140 && mock_sniffers == 1 && mock_menus == 1);
        if (commands < 0) commands = mock_command_count;
        CHECK(mock_command_count == commands);
        frame(2);
        stop();
        CHECK(active_refs() == 0 && active_commands() == 0 && mock_sniffers == 0 && mock_menus == 0);
        CHECK(device(0) == NULL && device(1) == NULL);
    }
}

int main(int argc, char **argv)
{
    if (argc > 1 && !strcmp(argv[1], "--contracts")) return contracts();
    test_devices();
    test_side_isolation_and_du();
    test_keyboard();
    test_zero_coordinate_release_and_data_entry();
    test_brightness();
    test_cockpit_dropdowns();
    test_arrow_keys_and_held_kccu();
    test_cursor_transport();
    test_fallback();
    test_reload();
    if (g_fail) {
        printf("test_mfd_adapter: %d of %d checks FAILED\n", g_fail, g_checks);
        return 1;
    }
    printf("test_mfd_adapter: %d checks OK\n", g_checks);
    return 0;
}
