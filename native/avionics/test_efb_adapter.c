/* SPDX-License-Identifier: GPL-3.0-or-later
 * Production web-EFB adapter (a380x_efb.c + a380x_efb_web.inc + a380x_efb_device.inc) against a
 * mocked XPLM with the X-Plane 12.1 cockpit-device API: device creation, touch (tap, drag,
 * positionless release), web / native-fallback drawing, slides and the door veto, typed payload,
 * fuel, radios, failures, sounds, presets, checklists, navigation and the JSON frame (cold & dark,
 * unpowered tablet, NaN inputs, size cap), reload and cleanup. No simulator, GL context or browser.
 *
 *   test_efb_adapter              run the checks
 *   test_efb_adapter --contracts  "C/D/V/R" lines: commands, datarefs, devices, raw sim inputs
 *   test_efb_adapter --frame <section> [tab] [door-mode]   a realistic frame (page tests, previews)
 */
#include <assert.h>
#include <stdlib.h>
#define XPLM200 1
#define XPLM210 1
#define XPLM300 1
#define XPLM301 1
#define XPLM400 1
#define XPLM410 1
#include "a380x_efb.c"

static int failures;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

/* ---- Data ------------------------------------------------------------------------------------- */
typedef struct {
    char name[160];
    double value;
    int ints[32];
    float floats[32];
    int owned;                       /* registered by the plugin (accessor) */
    XPLMGetDatai_f read_int;
    XPLMGetDatavi_f read_ints;
    void *refcon;
    int registered;
} MockData;
static MockData mock_data[256];
static int mock_data_count;
static char requested[256][160];
static int requested_count;

static MockData *data_named(const char *name)
{
    int i;
    for (i = 0; i < mock_data_count; ++i) if (!strcmp(mock_data[i].name, name)) return &mock_data[i];
    assert(mock_data_count < ARRAY_LEN(mock_data));
    snprintf(mock_data[mock_data_count].name, sizeof(mock_data[0].name), "%s", name);
    return &mock_data[mock_data_count++];
}
static void note_request(const char *name)
{
    int i;
    if (strncmp(name, "sim/", 4)) return;
    for (i = 0; i < requested_count; ++i) if (!strcmp(requested[i], name)) return;
    snprintf(requested[requested_count++], sizeof(requested[0]), "%s", name);
}
XPLMDataRef XPLMFindDataRef(const char *name)
{
    int i;
    note_request(name);
    if (!strncmp(name, "a380x/", 6)) {
        for (i = 0; i < mock_data_count; ++i)
            if (!strcmp(mock_data[i].name, name) && (mock_data[i].registered || !mock_data[i].owned)) return &mock_data[i];
        return NULL;
    }
    return data_named(name);
}
static MockData *foreign(const char *name) { MockData *d = data_named(name); d->owned = 0; d->registered = 1; return d; }
float XPLMGetDataf(XPLMDataRef ref) { return (float)((MockData *)ref)->value; }
double XPLMGetDatad(XPLMDataRef ref) { return ((MockData *)ref)->value; }
void XPLMGetPrefsPath(char *out) { out[0] = 0; }   /* no preferences folder: default GPWS voice */
int XPLMGetDatai(XPLMDataRef ref)
{
    MockData *d = ref;
    return d->read_int ? d->read_int(d->refcon) : (int)d->value;
}
int XPLMGetDatab(XPLMDataRef ref, void *values, int offset, int max) { (void)ref; (void)values; (void)offset; (void)max; return 0; }
void XPLMSetDataf(XPLMDataRef ref, float value) { ((MockData *)ref)->value = value; }
void XPLMSetDatai(XPLMDataRef ref, int value) { ((MockData *)ref)->value = value; }
int XPLMGetDatavi(XPLMDataRef ref, int *out, int offset, int count)
{
    MockData *d = ref;
    if (d->read_ints) return d->read_ints(d->refcon, out, offset, count);
    if (!out) return 32;
    if (offset + count > 32) count = 32 - offset;
    memcpy(out, d->ints + offset, (size_t)count * sizeof(*out));
    return count;
}
int XPLMGetDatavf(XPLMDataRef ref, float *out, int offset, int count)
{
    if (!out) return 32;
    if (offset + count > 32) count = 32 - offset;
    memcpy(out, ((MockData *)ref)->floats + offset, (size_t)count * sizeof(*out));
    return count;
}
void XPLMSetDatavf(XPLMDataRef ref, float *values, int offset, int count)
{
    assert(offset >= 0 && offset + count <= 32);
    memcpy(((MockData *)ref)->floats + offset, values, (size_t)count * sizeof(*values));
}
XPLMDataRef XPLMRegisterDataAccessor(const char *name, XPLMDataTypeID type, int writable,
    XPLMGetDatai_f ri, XPLMSetDatai_f wi, XPLMGetDataf_f rf, XPLMSetDataf_f wf,
    XPLMGetDatad_f rd, XPLMSetDatad_f wd, XPLMGetDatavi_f riv, XPLMSetDatavi_f wiv,
    XPLMGetDatavf_f rfv, XPLMSetDatavf_f wfv, XPLMGetDatab_f rb, XPLMSetDatab_f wb, void *rr, void *wr)
{
    MockData *d = data_named(name);
    (void)type; (void)writable; (void)wi; (void)rf; (void)wf; (void)rd; (void)wd; (void)wiv; (void)rfv; (void)wfv; (void)rb; (void)wb; (void)wr;
    assert(!d->registered);
    d->owned = d->registered = 1; d->read_int = ri; d->read_ints = riv; d->refcon = rr;
    return d;
}
void XPLMUnregisterDataAccessor(XPLMDataRef ref) { MockData *d = ref; assert(d->registered && d->owned); d->registered = 0; }
void XPLMDebugString(const char *text) { (void)text; }
void XPLMGetVersions(int *xp, int *xplm, XPLMHostApplicationID *host) { *xp = 12400; *xplm = 411; *host = xplm_Host_XPlane; }

/* ---- Commands --------------------------------------------------------------------------------- */
typedef struct { XPLMCommandCallback_f callback; int before; void *refcon; } MockHandler;
typedef struct { char name[160]; int calls, blocked; MockHandler handlers[4]; int handler_count; } MockCommand;
static MockCommand mock_commands[192];
static int mock_command_count;

XPLMCommandRef XPLMCreateCommand(const char *name, const char *description)
{
    MockCommand *entry;
    (void)description;
    assert(mock_command_count < ARRAY_LEN(mock_commands));
    entry = &mock_commands[mock_command_count++];
    snprintf(entry->name, sizeof(entry->name), "%s", name);
    return entry;
}
XPLMCommandRef XPLMFindCommand(const char *name)
{
    int i;
    for (i = 0; i < mock_command_count; ++i) if (!strcmp(mock_commands[i].name, name)) return &mock_commands[i];
    return !strncmp(name, "sim/", 4) ? XPLMCreateCommand(name, "") : NULL;
}
void XPLMRegisterCommandHandler(XPLMCommandRef ref, XPLMCommandCallback_f cb, int before, void *refcon)
{
    MockCommand *c = ref;
    assert(c->handler_count < 4);
    c->handlers[c->handler_count++] = (MockHandler){cb, before, refcon};
}
void XPLMUnregisterCommandHandler(XPLMCommandRef ref, XPLMCommandCallback_f cb, int before, void *refcon)
{
    MockCommand *c = ref;
    int i;
    for (i = 0; i < c->handler_count; ++i)
        if (c->handlers[i].callback == cb && c->handlers[i].before == before && c->handlers[i].refcon == refcon) {
            c->handlers[i] = c->handlers[--c->handler_count];
            return;
        }
    assert(!"unregistering an unknown handler");
}
static int run_handlers(MockCommand *c, XPLMCommandPhase phase)
{
    int i;
    for (i = 0; i < c->handler_count; ++i)
        if (c->handlers[i].before && !c->handlers[i].callback(c, phase, c->handlers[i].refcon)) return 0;
    return 1;
}
void XPLMCommandOnce(XPLMCommandRef ref)
{
    MockCommand *c = ref;
    int door, allowed;
    ++c->calls;
    allowed = run_handlers(c, xplm_CommandBegin);
    run_handlers(c, xplm_CommandEnd);
    if (!allowed) { ++c->blocked; return; }
    if (sscanf(c->name, "sim/flight_controls/door_toggle_%d", &door) == 1) {
        MockData *doors = refs.doors;
        doors->floats[door - 1] = doors->floats[door - 1] > 0.5f ? 0.0f : 1.0f;
    } else if (!strcmp(c->name, "sim/flight_controls/toggle_chocks")) {
        MockData *chocks = data_named("sim/flightmodel2/gear/is_chocked");
        int k;
        for (k = 0; k < 10; ++k) chocks->ints[k] = !chocks->ints[k];
    } else if (!strcmp(c->name, "sim/operation/fix_all_systems")) {
        int k;
        for (k = 0; k < 17; ++k) if (refs.failures[k]) ((MockData *)refs.failures[k])->value = 0;
    }
}
static MockCommand *command(const char *name) { return XPLMFindCommand(name); }
static void press(const char *name)
{
    MockCommand *c = command(name);
    int i;
    assert(c && c->handler_count);
    for (i = 0; i < c->handler_count; ++i) c->handlers[i].callback(c, xplm_CommandBegin, c->handlers[i].refcon);
    for (i = 0; i < c->handler_count; ++i) c->handlers[i].callback(c, xplm_CommandEnd, c->handlers[i].refcon);
}

/* ---- Window / menu (the ISCS host) ------------------------------------------------------------ */
static XPLMCreateWindow_t mock_window;
static int menu_token;
void XPLMGetScreenBoundsGlobal(int *l, int *t, int *r, int *b) { *l = 0; *t = 1080; *r = 1920; *b = 0; }
XPLMWindowID XPLMCreateWindowEx(XPLMCreateWindow_t *params) { mock_window = *params; return &mock_window; }
void XPLMGetWindowGeometry(XPLMWindowID id, int *l, int *t, int *r, int *b)
{ XPLMCreateWindow_t *w = id; *l = w->left; *t = w->top; *r = w->right; *b = w->bottom; }
void XPLMSetWindowIsVisible(XPLMWindowID id, int visible) { ((XPLMCreateWindow_t *)id)->visible = visible; }
int XPLMGetWindowIsVisible(XPLMWindowID id) { return ((XPLMCreateWindow_t *)id)->visible; }
void XPLMBringWindowToFront(XPLMWindowID id) { (void)id; }
void XPLMDestroyWindow(XPLMWindowID id) { (void)id; }
void XPLMSetWindowPositioningMode(XPLMWindowID id, XPLMWindowPositioningMode mode, int monitor) { (void)id; (void)mode; (void)monitor; }
void XPLMSetWindowGravity(XPLMWindowID id, float l, float t, float r, float b) { (void)id; (void)l; (void)t; (void)r; (void)b; }
void XPLMSetWindowResizingLimits(XPLMWindowID id, int a, int b, int c, int d) { (void)id; (void)a; (void)b; (void)c; (void)d; }
void XPLMSetWindowTitle(XPLMWindowID id, const char *title) { (void)id; (void)title; }
int XPLMAppendMenuItem(XPLMMenuID id, const char *name, void *ref, int ignored) { (void)id; (void)name; (void)ref; (void)ignored; return 0; }
XPLMMenuID XPLMCreateMenu(const char *name, XPLMMenuID parent, int item, XPLMMenuHandler_f handler, void *ref)
{ (void)name; (void)parent; (void)item; (void)handler; (void)ref; return &menu_token; }
void XPLMDestroyMenu(XPLMMenuID id) { (void)id; }

/* ---- GL / text -------------------------------------------------------------------------------- */
static int vertex_count, label_count, clears;
static char last_label[192];
void glColor3f(GLfloat r, GLfloat g, GLfloat b) { (void)r; (void)g; (void)b; }
void glLineWidth(GLfloat width) { (void)width; }
void glBegin(GLenum mode) { (void)mode; }
void glVertex2f(GLfloat x, GLfloat y) { assert(isfinite(x) && isfinite(y)); assert(x >= -1 && x <= 1441 && y >= -1 && y <= 1017); ++vertex_count; }
void glEnd(void) {}
void glClearColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a) { (void)r; (void)g; (void)b; (void)a; }
void glClear(GLbitfield mask) { (void)mask; ++clears; }
GLboolean glIsEnabled(GLenum cap) { (void)cap; return GL_FALSE; }
void glDisable(GLenum cap) { (void)cap; }
void glEnable(GLenum cap) { (void)cap; }
void XPLMSetGraphicsState(int a, int b, int c, int d, int e, int f, int g) { (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; (void)g; }
float XPLMMeasureString(XPLMFontID font, const char *value, int length) { (void)font; (void)value; return length * 6.0f; }
void XPLMDrawString(float *rgb, int x, int y, const char *value, int *wrap, XPLMFontID font)
{ (void)rgb; (void)x; (void)y; (void)wrap; (void)font; ++label_count; snprintf(last_label, sizeof(last_label), "%s", value); }

/* ---- Cockpit-device API (resolved through XPLMFindSymbol) ------------------------------------- */
typedef struct { XPLMCreateAvionics_t params; char id[64]; int bound, popup, redraws, destroyed; } MockDevice;
static MockDevice devices[4];
static int device_count;
static XPLMAvionicsID av_create(XPLMCreateAvionics_t *p)
{
    MockDevice *d;
    assert(device_count < 4 && p->structSize == (int)sizeof(*p));
    d = &devices[device_count++];
    d->params = *p;
    snprintf(d->id, sizeof(d->id), "%s", p->deviceID);
    return d;
}
static void av_destroy(XPLMAvionicsID id) { ((MockDevice *)id)->destroyed = 1; }
static void av_needs(XPLMAvionicsID id) { ((MockDevice *)id)->redraws++; }
static int av_bound(XPLMAvionicsID id) { return ((MockDevice *)id)->bound; }
static int av_popup_visible(XPLMAvionicsID id) { return ((MockDevice *)id)->popup; }
static void av_set_popup(XPLMAvionicsID id, int visible) { ((MockDevice *)id)->popup = visible; }
void *XPLMFindSymbol(const char *name)
{
    if (!strcmp(name, "XPLMCreateAvionicsEx")) return (void *)av_create;
    if (!strcmp(name, "XPLMDestroyAvionics")) return (void *)av_destroy;
    if (!strcmp(name, "XPLMAvionicsNeedsDrawing")) return (void *)av_needs;
    if (!strcmp(name, "XPLMIsAvionicsBound")) return (void *)av_bound;
    if (!strcmp(name, "XPLMIsAvionicsPopupVisible")) return (void *)av_popup_visible;
    if (!strcmp(name, "XPLMSetAvionicsPopupVisible")) return (void *)av_set_popup;
    return NULL;
}

/* ---- Web host --------------------------------------------------------------------------------- */
static int web_live[2], web_draws[2];
static int host_live(int tablet) { return web_live[tablet]; }
static int host_draw(int tablet, float w, float h) { assert(w == 1440 && h == 1000); ++web_draws[tablet]; return 1; }
static const A380XEfbWebHost host = {host_live, host_draw};

/* ---- JSON validator --------------------------------------------------------------------------- */
static const char *json_value(const char *s);
static const char *json_ws(const char *s) { while (*s == ' ' || *s == '\n' || *s == '\t' || *s == '\r') ++s; return s; }
static const char *json_string(const char *s)
{
    if (*s != '"') return NULL;
    for (++s; *s && *s != '"'; ++s) {
        if ((unsigned char)*s < 32) return NULL;
        if (*s == '\\') { ++s; if (!*s) return NULL; }
    }
    return *s == '"' ? s + 1 : NULL;
}
static const char *json_number(const char *s)
{
    const char *start = s;
    if (*s == '-') ++s;
    if (!(*s >= '0' && *s <= '9')) return NULL;
    while (*s >= '0' && *s <= '9') ++s;
    if (*s == '.') { ++s; if (!(*s >= '0' && *s <= '9')) return NULL; while (*s >= '0' && *s <= '9') ++s; }
    return s > start ? s : NULL;
}
static const char *json_value(const char *s)
{
    s = json_ws(s);
    if (*s == '{' || *s == '[') {
        char close = *s == '{' ? '}' : ']';
        s = json_ws(s + 1);
        if (*s == close) return s + 1;
        for (;;) {
            if (close == '}') {
                if (!(s = json_string(s))) return NULL;
                s = json_ws(s);
                if (*s++ != ':') return NULL;
            }
            if (!(s = json_value(s))) return NULL;
            s = json_ws(s);
            if (*s == ',') { s = json_ws(s + 1); continue; }
            return *s == close ? s + 1 : NULL;
        }
    }
    if (*s == '"') return json_string(s);
    if (!strncmp(s, "true", 4)) return s + 4;
    if (!strncmp(s, "false", 5)) return s + 5;
    if (!strncmp(s, "null", 4)) return s + 4;
    return json_number(s);
}
static int json_ok(const char *text) { const char *end = json_value(text); return end && *json_ws(end) == 0; }

static char frame[65536];
static long frame_size;
static long tablet_frame(int tablet)
{
    frame_size = a380x_efb_frame_json(tablet, frame, sizeof(frame));
    return frame_size;
}
static int frame_has(const char *text) { return tablet_frame(0) > 0 && strstr(frame, text) != NULL; }
static unsigned long next_id = 1;
static void web(int tablet, const char *action)
{
    char text[160];
    snprintf(text, sizeof(text), "%lu %s", next_id++, action);
    a380x_efb_web_action(tablet, text);
}

/* ---- Scenario --------------------------------------------------------------------------------- */
static MockData *dref(const char *name) { return data_named(name); }
static void aircraft_parked(void)
{
    int i;
    float ratios[9] = {0.30f, 0.078f, 0.22f, 0.22f, 0.078f, 0.023f, 0.023f, 0.058f, 0.0f};
    float fuel[9] = {30000, 15000, 50000, 20000, 15000, 5000, 5000, 10000, 0};
    powered = 1; battery_level = 82; battery_simulation = 1; brightness = 85; metric_units = 1;
    dref("sim/flightmodel/failures/onground_any")->value = 1;
    for (i = 0; i < 4; ++i) dref("sim/flightmodel2/engines/engine_is_burning_fuel")->ints[i] = 0;
    for (i = 0; i < 20; ++i) dref("sim/flightmodel2/misc/door_open_ratio")->floats[i] = 0;
    dref("sim/cockpit2/electrical/GPU_generator_on")->value = 1;
    dref("sim/time/zulu_time_sec")->value = 45240 + 30;
    dref("sim/time/local_time_sec")->value = 52440;
    dref("sim/cockpit2/gauges/indicators/altitude_ft_pilot")->value = 83;
    dref("sim/cockpit2/gauges/indicators/ground_speed_kt")->value = 0;
    dref("sim/flightmodel/weight/m_fixed")->value = 0;
    dref("sim/flightmodel/weight/m_fuel_total")->value = 150000;
    dref("sim/flightmodel/weight/m_total")->value = 150000 + 276800;
    memcpy(dref("sim/aircraft/overflow/acf_tank_rat")->floats, ratios, sizeof(ratios));
    memcpy(dref("sim/flightmodel/weight/m_fuel")->floats, fuel, sizeof(fuel));
    dref("sim/weather/aircraft/qnh_pas")->value = 101325;
    dref("sim/weather/aircraft/wind_direction_degt")->value = 250;
    dref("sim/weather/aircraft/wind_speed_kts")->value = 8;
    dref("sim/weather/aircraft/temperature_ambient_deg_c")->value = 14;
    dref("sim/flightmodel/position/latitude")->value = 51.4706;
    dref("sim/flightmodel/position/longitude")->value = -0.4619;
    dref("sim/cockpit2/radios/actuators/com1_frequency_hz_833")->value = 121900;
    dref("sim/cockpit2/radios/actuators/com1_standby_frequency_hz_833")->value = 118505;
    dref("sim/cockpit2/radios/actuators/com2_frequency_hz_833")->value = 121500;
    dref("sim/cockpit2/radios/actuators/com2_standby_frequency_hz_833")->value = 123450;
    dref("sim/cockpit2/radios/actuators/transponder_code")->value = 4621;
    dref("sim/cockpit2/radios/actuators/transponder_mode")->value = 1;
    dref("sim/flightmodel/controls/parkbrake")->value = 1;
    {
        static const double volumes[EFB_SOUNDS] = {0.8, 0.9, 0.75, 0.7, 0.6, 0.65, 0.7, 0.55, 1.0, 0.85};
        for (i = 0; i < EFB_SOUNDS; ++i) dref(efb_sound_refs_names[i])->value = volumes[i];
    }
}

/* ---- Tests ------------------------------------------------------------------------------------ */
static void test_registration(void)
{
    int i;
    CHECK(device_count == 2);
    CHECK(!strcmp(devices[0].id, "a380x.xp12.efb.captain") && !strcmp(devices[1].id, "a380x.xp12.efb.first_officer"));
    for (i = 0; i < 2; ++i) {
        XPLMCreateAvionics_t *p = &devices[i].params;
        CHECK(p->screenWidth == 1440 && p->screenHeight == 1000 && p->drawOnDemand == 1);
        CHECK(p->drawCallback && p->screenTouchCallback && p->brightnessCallback && p->screenRightTouchCallback && p->refcon == &tablets[i]);
    }
    CHECK(XPLMFindDataRef("a380x/xp12/efb/slide_armed") && XPLMFindDataRef("a380x/xp12/efb/devices_active"));
    CHECK(XPLMGetDatai(XPLMFindDataRef("a380x/xp12/efb/devices_active")) == 1);
    CHECK(command("a380x/xp12/efb/slides_arm_all") && command("a380x/xp12/efb/captain/popup") && command("a380x/xp12/efb/page/atc"));
    for (i = 1; i <= 20; ++i) {
        char name[64];
        snprintf(name, sizeof(name), "sim/flight_controls/door_toggle_%d", i);
        CHECK(command(name)->handler_count == (i <= 16));   /* passenger doors only: cargo / cockpit have no slides */
    }
}

static void test_slides_and_doors(void)
{
    MockData *doors = refs.doors;
    int armed[16];
    aircraft_parked();
    web(0, "slide M1L 1");
    CHECK(slide_armed[0] == 1 && frame_has("\"slides\":[1,0,0"));
    web(0, "door M1L");
    CHECK(doors->floats[0] == 0 && strstr(status_message, "Disarm the M1L slide"));
    XPLMCommandOnce(command("sim/flight_controls/door_toggle_1"));   /* any other path is vetoed too */
    CHECK(doors->floats[0] == 0 && command("sim/flight_controls/door_toggle_1")->blocked == 1);
    web(0, "slide M1L 0");
    web(0, "door M1L");
    CHECK(doors->floats[0] == 1 && !slide_armed[0]);
    web(1, "slide M1L 1");
    CHECK(!slide_armed[0] && strstr(status_message, "Close M1L before arming"));
    web(1, "slide FWD 1");
    CHECK(strstr(status_message, "FWD has no escape slide"));
    web(0, "slides arm");
    CHECK(a380x_efb_slides_armed(slide_armed) == 15 && !slide_armed[0] && strstr(status_message, "1 open doors left disarmed"));
    CHECK(XPLMGetDatai(XPLMFindDataRef("a380x/xp12/efb/slides_armed_count")) == 15);
    CHECK(XPLMGetDatavi(XPLMFindDataRef("a380x/xp12/efb/slide_armed"), armed, 0, 16) == 16 && armed[1] == 1 && armed[0] == 0);
    CHECK(XPLMGetDatavi(XPLMFindDataRef("a380x/xp12/efb/slide_armed"), NULL, 0, 0) == 16);
    doors->floats[3] = 0.4f;    /* opened through a path the EFB could not veto */
    a380x_efb_update(0.1f);
    CHECK(!slide_armed[3] && strstr(status_message, "1 slide disarmed: door opened"));
    doors->floats[3] = 0;
    web(0, "doors open");       /* armed doors stay closed; cargo and cockpit open */
    CHECK(doors->floats[17] == 1 && doors->floats[16] == 1 && doors->floats[3] == 1 && doors->floats[1] == 0);
    CHECK(strstr(status_message, "with armed slides stayed closed") != NULL);
    web(0, "doors close");
    CHECK(a380x_efb_doors_open(doors->floats) == 0);
    press("a380x/xp12/efb/slides_disarm_all");
    CHECK(a380x_efb_slides_armed(slide_armed) == 0);
    press("a380x/xp12/efb/slides_arm_all");
    CHECK(a380x_efb_slides_armed(slide_armed) == 16);
    toggle_all_doors();         /* the legacy ALL DOORS button and ISCS keep armed doors closed */
    CHECK(doors->floats[0] == 0 && doors->floats[17] == 1 && strstr(status_message, "16 with armed slides stayed closed"));
    toggle_all_doors();
    web(0, "slides disarm");
    dref("sim/flightmodel/failures/onground_any")->value = 0;
    web(0, "door M2L");
    CHECK(doors->floats[2] == 0 && strstr(status_message, "ground-only"));
    dref("sim/flightmodel/failures/onground_any")->value = 1;
}

static void test_payload_and_fuel(void)
{
    MockData *payload = refs.payload, *fuel = refs.fuel;
    float total = 0;
    int i;
    aircraft_parked();
    web(0, "pax 3 338");
    CHECK(load_plan.pax[3] == 338);
    web(0, "pax 3 339");
    CHECK(load_plan.pax[3] == 338 && strstr(status_message, "Economy: above the limit (338 seats)"));
    web(0, "pax 0 1a");
    CHECK(load_plan.pax[0] == 0 && strstr(status_message, "digits only"));
    web(0, "pax 4 1");          /* no fifth class: rejected request */
    web(0, "cargo 0 28577");
    metric_units = 0;
    web(0, "cargo 2 5540");     /* typed in pounds */
    CHECK(load_plan.cargo_kg[2] == 2513);
    web(0, "cargo 1 44779");    /* 20311 kg > 20310 */
    CHECK(load_plan.cargo_kg[1] == 0 && strstr(status_message, "Aft hold: above the limit"));
    metric_units = 1;
    web(0, "load apply");
    CHECK(fabs(payload->value - (338 * 104 + 28577 + 2513)) < 0.5);
    CHECK(frame_has("\"liveMatches\":true"));
    dref("sim/flightmodel2/engines/engine_is_burning_fuel")->ints[2] = 1;
    web(0, "load clear");
    CHECK(payload->value > 0 && strstr(status_message, "engines off"));   /* loading rule: engines off */
    dref("sim/flightmodel2/engines/engine_is_burning_fuel")->ints[2] = 0;
    web(0, "load clear");
    CHECK(payload->value == 0);
    web(0, "load full");
    CHECK(load_plan.pax[0] == 14 && load_plan.pax[3] == 338);
    web(0, "fuel target 400000");
    CHECK(fuel_target_kg < 0 && strstr(status_message, "above the limit"));
    web(0, "fuel target 100000");
    web(0, "fuel rate instant");
    web(0, "fuel start");
    for (i = 0; i < 9; ++i) total += fuel->floats[i];
    CHECK(fabsf(total - 100000) < 1);
    dref("sim/flightmodel/weight/m_fuel_total")->value = 100000;
    web(0, "fuel target 100500");
    web(0, "fuel rate real");
    web(0, "fuel start");
    CHECK(refuel_active);
    a380x_efb_update(2.0f);     /* 50 kg/s */
    for (total = 0, i = 0; i < 9; ++i) total += fuel->floats[i];
    CHECK(fabsf(total - 100100) < 1 && refuel_active);
    dref("sim/flightmodel/failures/onground_any")->value = 0;
    a380x_efb_update(1.0f);
    CHECK(!refuel_active && strstr(status_message, "Refuel stopped"));
    dref("sim/flightmodel/failures/onground_any")->value = 1;
    web(0, "fuel rate warp");
    CHECK(refuel_rate == 2);
}

static void test_radios_failures_sounds(void)
{
    unsigned long rejected = efb_actions_rejected;
    aircraft_parked();
    web(0, "radio stby1 121.9");
    CHECK(dref("sim/cockpit2/radios/actuators/com1_standby_frequency_hz_833")->value == 121900);
    web(0, "radio stby2 99.000");
    CHECK(dref("sim/cockpit2/radios/actuators/com2_standby_frequency_hz_833")->value == 123450 && strstr(status_message, "118.000-136.990"));
    web(0, "radio stby2 132.105");
    CHECK(dref("sim/cockpit2/radios/actuators/com2_standby_frequency_hz_833")->value == 132105);
    web(0, "radio squawk 7700");
    CHECK(dref("sim/cockpit2/radios/actuators/transponder_code")->value == 7700);
    web(0, "radio squawk 7800");
    CHECK(dref("sim/cockpit2/radios/actuators/transponder_code")->value == 7700);
    web(0, "radio mode 7");
    CHECK(dref("sim/cockpit2/radios/actuators/transponder_mode")->value == 7);
    web(0, "radio mode 2");
    CHECK(dref("sim/cockpit2/radios/actuators/transponder_mode")->value == 7 && efb_actions_rejected == rejected + 1);
    web(0, "radio swap1");
    CHECK(command("sim/radios/com1_standy_flip")->calls == 1);
    web(0, "fail 0 1");
    CHECK(dref("sim/operation/failures/rel_engfai0")->value == 6 && frame_has("[\"ENGINE 1\",\"ENG\",\"72\",6]"));
    web(0, "fail 16 1");
    web(0, "fail_clear");
    CHECK(dref("sim/operation/failures/rel_engfai0")->value == 0 && dref("sim/operation/failures/rel_pitot2")->value == 0);
    web(0, "fail 17 1");        /* out of range */
    CHECK(efb_actions_rejected == rejected + 2);
    web(0, "vol master 55");
    CHECK(fabs(dref("sim/operation/sound/master_volume_ratio")->value - 0.55) < 1e-6 && frame_has("[\"master\",55]"));
    web(0, "vol radios 101");
    CHECK(efb_actions_rejected == rejected + 3);
    web(0, "vol restore");
    CHECK(fabs(dref("sim/operation/sound/master_volume_ratio")->value - 0.8) < 1e-6);
    web(0, "set brightness 5");
    CHECK(brightness == 20);
    web(0, "set brightness 70");
    CHECK(brightness == 70);
    web(0, "tod target 3000");              /* metric display: metres */
    CHECK(target_altitude_ft == 9843);
    metric_units = 0;
    web(0, "tod target 12000");
    CHECK(target_altitude_ft == 12000);
    web(0, "tod up");
    CHECK(target_altitude_ft == 13000 && frame_has("\"targetFt\":13000"));
    web(0, "tod target 50001");
    CHECK(target_altitude_ft == 13000 && strstr(status_message, "Target altitude"));
    metric_units = 1;
    web(0, "option hud 0");
    web(0, "cabin 1 1 80");
    web(0, "cabin 2 3 9999");
    CHECK(!option_hud && cabin_layout[1][1] == 80 && cabin_layout[2][3] == 0 && strstr(status_message, "up to 900"));
    web(0, "cabin reset");
    CHECK(cabin_layout[1][1] == 76);
}

static void test_navigation_and_frame(void)
{
    MockData *altitude = refs.altitude;
    char small[100];
    aircraft_parked();
    a380x_efb_web_action(1, "900 nav loading payload");
    CHECK(tablets[1].section == A380X_EFB_LOADING && tablets[1].ack == 900 && active_page == PAGE_DISPATCH);
    CHECK(tablet_frame(1) > 0 && strstr(frame, "\"ack\":900") && strstr(frame, "\"section\":\"loading\""));
    a380x_efb_web_action(1, "901 tab ground push");
    CHECK(tablets[1].section == A380X_EFB_GROUND && tablets[1].tab[A380X_EFB_GROUND] == 2);
    a380x_efb_web_action(1, "800 nav home");   /* an older id never lowers the ack */
    CHECK(tablets[1].ack == 901 && tablets[1].section == A380X_EFB_HOME);
    a380x_efb_web_action(1, "902 nav bogus");
    a380x_efb_web_action(1, "garbage");
    CHECK(tablets[1].section == A380X_EFB_HOME);
    press("a380x/xp12/efb/page/atc");          /* legacy bindings: both tablets follow */
    CHECK(tablets[0].section == A380X_EFB_FLIGHT && tablets[0].tab[A380X_EFB_FLIGHT] == 2 && tablets[1].section == A380X_EFB_FLIGHT);
    press("a380x/xp12/efb/next_page");
    CHECK(active_page == PAGE_FAILURES && tablets[0].section == A380X_EFB_FAILURES);
    /* A complete, valid frame: realistic, NaN inputs, cold & dark, unpowered. */
    CHECK(tablet_frame(0) > 2000 && frame_size < 16384 && json_ok(frame));
    CHECK(strstr(frame, "\"tanks\":[[\"L OUTR\",5000,") && strstr(frame, "\"zulu\":754") && strstr(frame, "\"blank\":false"));
    CHECK(strstr(frame, "\"cg\":null") && strstr(frame, "\"jetway\":null"));
    altitude->value = NAN;
    dref("sim/flightmodel/weight/m_fuel_total")->value = NAN;
    dref("sim/flightmodel2/misc/door_open_ratio")->floats[4] = NAN;
    CHECK(tablet_frame(0) > 0 && json_ok(frame) && !strstr(frame, "nan") && !strstr(frame, "inf"));
    CHECK(strstr(frame, "\"altFt\":null") && strstr(frame, "\"fob\":null"));
    aircraft_parked();
    dref("sim/cockpit2/electrical/GPU_generator_on")->value = 0;
    dref("sim/cockpit2/electrical/battery_on")->ints[0] = dref("sim/cockpit2/electrical/battery_on")->ints[1] = 0;
    battery_level = 0;                         /* cold & dark, empty tablet battery */
    a380x_efb_update(1.0f);
    CHECK(!powered && tablet_frame(0) > 0 && json_ok(frame) && strstr(frame, "\"blank\":true") && strstr(frame, "\"charging\":false"));
    aircraft_parked();
    powered = 0;                               /* unpowered (standby) tablet with aircraft power */
    CHECK(tablet_frame(1) > 0 && json_ok(frame) && strstr(frame, "\"blank\":true") && strstr(frame, "\"on\":false"));
    powered = 1;
    CHECK(a380x_efb_frame_json(0, small, sizeof(small)) == -1 && small[0] == 0);
    CHECK(a380x_efb_frame_json(2, frame, sizeof(frame)) == -1);
    /* Checklists: manual items, CHECK ITEM, auto conditions. */
    web(0, "cl_item 0 0");
    CHECK(checklist_bits[0] & 1u);
    web(0, "cl_next 0");
    CHECK(checklist_bits[0] & 2u);
    dref("sim/cockpit2/switches/fasten_seat_belts")->value = 1;
    CHECK(tablet_frame(0) > 0 && strstr(frame, "\"name\":\"COCKPIT PREPARATION\",\"done\":7,\"auto\":4"));
    web(0, "cl_next 0");
    CHECK(checklist_bits[0] == 11u && checklist_completed_bits(0) == 15u);   /* the auto item is not re-ticked */
    web(0, "cl_next 0");                        /* complete: CHECK ITEM moves to the next group */
    CHECK(tablets[0].group == 1 && selected_checklist == 1);
    web(0, "cl_reset 0");
    CHECK(checklist_bits[0] == 0);
}

static void test_touch_and_devices(void)
{
    XPLMCreateAvionics_t *p0 = &devices[0].params, *p1 = &devices[1].params;
    int redraws;
    aircraft_parked();
    /* Native fallback (no web renderer): a press maps to the legacy atlas, here the FAULTS tab. */
    a380x_efb_set_web_host(NULL);
    active_page = PAGE_DASHBOARD;
    p0->screenTouchCallback((614 - 152) * 2, (963 - 516) * 2, xplm_MouseDown, p0->refcon);
    p0->screenTouchCallback(-16, -16, xplm_MouseUp, p0->refcon);
    CHECK(active_page == PAGE_FAILURES && tablets[0].touch_serial == 0);
    vertex_count = label_count = 0;
    p0->drawCallback(p0->refcon);
    CHECK(tablets[0].fallback_draws == 1 && vertex_count > 20 && label_count > 10);
    /* Web renderer live: ordered events, page coordinates (y down), positionless release. */
    a380x_efb_set_web_host(&host);
    web_live[0] = web_live[1] = 1;
    p0->screenTouchCallback(300, 500, xplm_MouseDown, p0->refcon);
    p0->screenTouchCallback(320, 500, xplm_MouseDrag, p0->refcon);
    p0->screenTouchCallback(0, 0, xplm_MouseDrag, p0->refcon);
    p0->screenTouchCallback(-16, -16, xplm_MouseUp, p0->refcon);
    CHECK(tablets[0].touch_serial == 4);
    CHECK(tablet_frame(0) > 0 && strstr(frame, "\"events\":[[1,1,300,499],[2,0,320,499],[3,0,320,499],[4,2,320,499]]"));
    CHECK(tablets[1].touch_serial == 0);
    p0->drawCallback(p0->refcon);
    CHECK(web_draws[0] == 1);
    CHECK(fabsf(p0->brightnessCallback(1, 1, 1, p0->refcon) - brightness / 100.0f) < 1e-6);
    CHECK(p0->screenRightTouchCallback(1, 1, xplm_MouseDown, p0->refcon) == 0);   /* camera look */
    /* Blank tablet: a tap wakes it; it never becomes a page event. */
    powered = 0;
    CHECK(p1->brightnessCallback(1, 1, 1, p1->refcon) == 0);
    clears = vertex_count = 0;
    p1->drawCallback(p1->refcon);
    CHECK(clears == 1 && vertex_count == 0 && web_draws[1] == 0);
    p1->screenTouchCallback(700, 400, xplm_MouseDown, p1->refcon);
    CHECK(powered == 1 && tablets[1].touch_serial == 0);
    /* Redraw scheduling: renderer transitions and the 4 Hz native fallback. */
    devices[0].bound = devices[1].bound = 1;
    a380x_efb_update(0.05f);
    redraws = devices[0].redraws;
    web_live[0] = 0;
    a380x_efb_update(0.05f);
    CHECK(devices[0].redraws == redraws + 1);
    a380x_efb_update(0.3f);
    CHECK(devices[0].redraws == redraws + 2);
    a380x_efb_needs_drawing(1);
    CHECK(devices[1].redraws > 0);
    /* Bound devices retire the panel-atlas tablet; unbound ones keep it. */
    CHECK(!a380x_efb_atlas_needed() && XPLMGetDatai(XPLMFindDataRef("a380x/xp12/efb/captain/device_bound")) == 1);
    vertex_count = 0;
    a380x_efb_draw();
    CHECK(vertex_count == 0);
    devices[1].bound = 0;
    a380x_efb_update(0.05f);
    vertex_count = 0;
    a380x_efb_draw();
    CHECK(a380x_efb_atlas_needed() && vertex_count > 20);
    press("a380x/xp12/efb/first_officer/popup");
    CHECK(devices[1].popup == 1 && devices[0].popup == 0);
    web_live[0] = web_live[1] = 0;
}

static void test_presets(void)
{
    MockData *network, *done, *active;
    float *doors = ((MockData *)refs.doors)->floats;
    aircraft_parked();
    memset(slide_armed, 0, sizeof(slide_armed));
    web(0, "preset 3");                         /* no network: only presets 1, 2, 4 exist */
    CHECK(strstr(status_message, "needs the native A380 electrical network"));
    web(0, "preset 4");
    CHECK(a380x_efb_slides_armed(slide_armed) == 16);
    web(0, "preset 1");
    CHECK(a380x_efb_slides_armed(slide_armed) == 0);
    network = foreign("a380x/xp12/elec/network_active"); network->value = 1;
    done = foreign("a380x/xp12/elec/preset/done"); done->value = 2;
    active = foreign("a380x/xp12/elec/preset/active"); active->value = 0;
    a380x_efb_update(0.1f);
    doors[4] = 1.0f;
    active->value = 3;
    a380x_efb_update(0.1f);                     /* still running: nothing yet */
    CHECK(a380x_efb_slides_armed(slide_armed) == 0);
    active->value = 0; done->value = 3;
    a380x_efb_update(0.1f);                     /* Ready for pushback complete: closed doors armed */
    CHECK(a380x_efb_slides_armed(slide_armed) == 15 && !slide_armed[4]);
    CHECK(tablet_frame(0) > 0 && strstr(frame, "\"native\":true,\"active\":0,\"done\":3"));
    done->value = 2;
    a380x_efb_update(0.1f);                     /* Turnaround complete: disarmed */
    CHECK(a380x_efb_slides_armed(slide_armed) == 0);
    network->value = 0;
    doors[4] = 0;
}

static void test_reload(void)
{
    int i;
    a380x_efb_shutdown();
    CHECK(devices[0].destroyed && devices[1].destroyed);
    for (i = 1; i <= 16; ++i) {
        char name[64];
        snprintf(name, sizeof(name), "sim/flight_controls/door_toggle_%d", i);
        CHECK(command(name)->handler_count == 0);
    }
    CHECK(XPLMFindDataRef("a380x/xp12/efb/slide_armed") == NULL);
    CHECK(a380x_efb_init());
    CHECK(device_count == 4 && command("sim/flight_controls/door_toggle_1")->handler_count == 1);
    CHECK(command("a380x/xp12/efb/slides_arm_all")->handler_count == 1);   /* persistent command, new handler */
    a380x_efb_shutdown();
    for (i = 0; i < mock_command_count; ++i) CHECK(mock_commands[i].handler_count == 0);
}

static void print_contracts(void)
{
    int i;
    for (i = 0; i < command_count; ++i) printf("C %s\n", commands[i].name);
    for (i = 0; i < (int)(sizeof(efb_window_commands) / sizeof(efb_window_commands[0])); ++i) printf("C %s\n", efb_window_commands[i].name);
    for (i = 0; i < mock_data_count; ++i) if (mock_data[i].owned && mock_data[i].registered) printf("D %s\n", mock_data[i].name);
    for (i = 0; i < device_count; ++i) printf("V %s\n", devices[i].id);
    for (i = 0; i < requested_count; ++i) printf("R %s\n", requested[i]);
}

/* A realistic turnaround frame for the page tests and the headless previews. */
static void print_frame(int argc, char **argv)
{
    int section = a380x_efb_section_find(argc > 2 ? argv[2] : "home"), tab = section >= 0 && argc > 3 ? a380x_efb_tab_find(section, argv[3]) : -1;
    float *doors;
    aircraft_parked();
    doors = ((MockData *)refs.doors)->floats;
    web(0, "slides arm");
    doors[0] = 1; doors[10] = 1; doors[17] = 0.6f;
    a380x_efb_update(0.1f);
    web(0, "pax 0 12"); web(0, "pax 1 70"); web(0, "pax 2 48"); web(0, "pax 3 301");
    web(0, "cargo 0 9200"); web(0, "cargo 1 7400"); web(0, "cargo 2 900");
    web(0, "load apply");
    web(0, "fuel target 168000");
    dref("sim/flightmodel/weight/m_total")->value = 150000 + 276800 + dref("sim/flightmodel/weight/m_fixed")->value;
    web(0, "cl_item 0 0"); web(0, "cl_item 0 1"); dref("sim/cockpit2/switches/fasten_seat_belts")->value = 1;
    web(0, "cl_group 1"); web(0, "fail 5 1");
    dref("sim/flightmodel2/gear/is_chocked")->ints[0] = 1;
    if (section < 0) section = A380X_EFB_HOME;
    efb_tablet_show(&tablets[0], section, tab);
    set_status("Turnaround: 3 doors open, 13 slides armed");
    tablet_frame(0);
    puts(frame);
}

int main(int argc, char **argv)
{
    int i;
    for (i = 0; i < 17; ++i) data_named(failure_refs[i]);
    if (!a380x_efb_init()) { fputs("init failed\n", stderr); return 1; }
    if (argc > 1 && !strcmp(argv[1], "--contracts")) {
        aircraft_parked();
        tablet_frame(0);
        a380x_efb_update(0.1f);
        print_contracts();
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "--frame")) { print_frame(argc, argv); return 0; }
    test_registration();
    test_slides_and_doors();
    test_payload_and_fuel();
    test_radios_failures_sounds();
    test_navigation_and_frame();
    test_touch_and_devices();
    test_presets();
    test_reload();
    if (failures) { fprintf(stderr, "%d EFB adapter check(s) failed\n", failures); return 1; }
    printf("EFB adapter: two 1440x1000 cockpit devices, ordered touch with positionless release, web/native "
           "drawing, slides with door veto, typed payload/fuel/radios, failures, sounds, presets, checklists, "
           "frames (cold & dark, standby, NaN, cap) and reload passed (frame %ld bytes)\n", frame_size);
    return 0;
}
