/* SPDX-License-Identifier: GPL-3.0-or-later
 * Headless regression harness for the real EFB renderer and native-window host.
 * No simulator or OpenGL context is used. Optional argv[1] is an existing
 * directory for twenty SVGs (all pages as tablet and native popup).
 *
 * clang -std=c99 -Wno-deprecated-declarations -DAPL=1 -DIBM=0 -DLIN=0
 *   -I<SDK>/CHeaders/XPLM test_efb_render.c a380x_avionics_core.c -lm -o test_efb
 */
#include <assert.h>
#include <stdlib.h>
#define XPLM200 1
#define XPLM210 1
#define XPLM300 1
#define XPLM301 1
#include "a380x_efb.c"

typedef struct {
    char name[160];
    double value;
    int integers[32];
    float floats[32];
} MockData;
typedef struct {
    char name[160];
    int calls;
    XPLMCommandCallback_f callback;
    void *refcon;
} MockCommand;
typedef struct { float x, y; } Vertex;
typedef struct { float x, y; char value[192]; } Label;
static MockData mock_data[128];
static MockCommand mock_commands[128];
static int mock_data_count, mock_command_count, command_calls;
static XPLMCreateWindow_t mock_window;
static int window_destroyed, menu_destroyed, menu_removed;
static int plugins_menu_token, child_menu_token;
static Vertex vertices[20000];
static Label labels[2000];
static int vertex_count, label_count, primitive_start;
static GLenum primitive;
static float rgb_color[3], line_width = 1;
static FILE *svg;
static float svg_left, svg_top;

static MockData *data_named(const char *name)
{
    int i;
    for (i = 0; i < mock_data_count; ++i)
        if (!strcmp(mock_data[i].name, name)) return &mock_data[i];
    assert(mock_data_count < ARRAY_LEN(mock_data));
    snprintf(mock_data[mock_data_count].name, sizeof(mock_data[0].name), "%s", name);
    return &mock_data[mock_data_count++];
}

XPLMDataRef XPLMFindDataRef(const char *name)
{
    if (!strncmp(name, "a380x/", 6)) return NULL;
    return data_named(name);
}
float XPLMGetDataf(XPLMDataRef ref) { return (float)((MockData *)ref)->value; }
double XPLMGetDatad(XPLMDataRef ref) { return ((MockData *)ref)->value; }
void XPLMGetPrefsPath(char *out) { out[0] = 0; }   /* no preferences folder: default GPWS voice */
int XPLMGetDatai(XPLMDataRef ref) { return (int)((MockData *)ref)->value; }
int XPLMGetDatab(XPLMDataRef ref, void *values, int offset, int max) { (void)ref; (void)values; (void)offset; (void)max; return 0; }
void XPLMSetDataf(XPLMDataRef ref, float value) { ((MockData *)ref)->value = value; }
void XPLMSetDatai(XPLMDataRef ref, int value) { ((MockData *)ref)->value = value; }
int XPLMGetDatavi(XPLMDataRef ref, int *out, int offset, int count)
{
    if (!out) return 32;
    assert(offset >= 0 && offset + count <= 32);
    memcpy(out, ((MockData *)ref)->integers + offset, count * sizeof(*out));
    return count;
}
int XPLMGetDatavf(XPLMDataRef ref, float *out, int offset, int count)
{
    if (!out) return 32;
    assert(offset >= 0 && offset + count <= 32);
    memcpy(out, ((MockData *)ref)->floats + offset, count * sizeof(*out));
    return count;
}
void XPLMSetDatavf(XPLMDataRef ref, float *values, int offset, int count)
{
    assert(offset >= 0 && offset + count <= 32);
    memcpy(((MockData *)ref)->floats + offset, values, count * sizeof(*values));
}
XPLMDataRef XPLMRegisterDataAccessor(const char *name, XPLMDataTypeID type, int writable,
    XPLMGetDatai_f ri, XPLMSetDatai_f wi, XPLMGetDataf_f rf, XPLMSetDataf_f wf,
    XPLMGetDatad_f rd, XPLMSetDatad_f wd, XPLMGetDatavi_f riv, XPLMSetDatavi_f wiv,
    XPLMGetDatavf_f rfv, XPLMSetDatavf_f wfv, XPLMGetDatab_f rb, XPLMSetDatab_f wb,
    void *rr, void *wr)
{
    (void)type; (void)writable; (void)ri; (void)wi; (void)rf; (void)wf;
    (void)rd; (void)wd; (void)riv; (void)wiv; (void)rfv; (void)wfv;
    (void)rb; (void)wb; (void)rr; (void)wr;
    return data_named(name);
}
void XPLMUnregisterDataAccessor(XPLMDataRef ref) { assert(ref); }
void XPLMDebugString(const char *text) { (void)text; }

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
    for (i = 0; i < mock_command_count; ++i)
        if (!strcmp(mock_commands[i].name, name)) return &mock_commands[i];
    return !strncmp(name, "sim/", 4) ? XPLMCreateCommand(name, "") : NULL;
}
void XPLMRegisterCommandHandler(XPLMCommandRef ref, XPLMCommandCallback_f cb,
                                int before, void *refcon)
{
    (void)before;
    ((MockCommand *)ref)->callback = cb;
    ((MockCommand *)ref)->refcon = refcon;
}
void XPLMUnregisterCommandHandler(XPLMCommandRef ref, XPLMCommandCallback_f cb,
                                  int before, void *refcon)
{
    MockCommand *entry = ref;
    (void)before;
    assert(entry->callback == cb && entry->refcon == refcon);
    entry->callback = NULL;
}
void XPLMCommandOnce(XPLMCommandRef ref)
{
    MockCommand *entry = ref;
    int door;
    ++entry->calls;
    ++command_calls;
    if (sscanf(entry->name, "sim/flight_controls/door_toggle_%d", &door) == 1) {
        MockData *doors = refs.doors;
        assert(door >= 1 && door <= 20);
        doors->floats[door - 1] = 1 - doors->floats[door - 1];
    }
}

void XPLMGetScreenBoundsGlobal(int *l, int *t, int *r, int *b)
{ *l = 0; *t = 1080; *r = 1920; *b = 0; }
XPLMWindowID XPLMCreateWindowEx(XPLMCreateWindow_t *params)
{ mock_window = *params; return &mock_window; }
void XPLMGetWindowGeometry(XPLMWindowID id, int *l, int *t, int *r, int *b)
{
    XPLMCreateWindow_t *w = id;
    *l = w->left; *t = w->top; *r = w->right; *b = w->bottom;
}
void XPLMSetWindowIsVisible(XPLMWindowID id, int visible)
{ ((XPLMCreateWindow_t *)id)->visible = visible; }
int XPLMGetWindowIsVisible(XPLMWindowID id)
{ return ((XPLMCreateWindow_t *)id)->visible; }
void XPLMBringWindowToFront(XPLMWindowID id) { assert(id); }
void XPLMDestroyWindow(XPLMWindowID id) { assert(id); ++window_destroyed; }
void XPLMSetWindowPositioningMode(XPLMWindowID id, XPLMWindowPositioningMode mode, int monitor)
{ (void)mode; (void)monitor; assert(id); }
void XPLMSetWindowGravity(XPLMWindowID id, float l, float t, float r, float b)
{ (void)l; (void)t; (void)r; (void)b; assert(id); }
void XPLMSetWindowResizingLimits(XPLMWindowID id, int minw, int minh, int maxw, int maxh)
{ assert(id && minw == 720 && minh == 500 && maxw >= minw && maxh >= minh); }
void XPLMSetWindowTitle(XPLMWindowID id, const char *title)
{ assert(id && !strcmp(title, "A380 ISCS")); }
XPLMMenuID XPLMFindPluginsMenu(void) { return &plugins_menu_token; }
int XPLMAppendMenuItem(XPLMMenuID id, const char *name, void *ref, int ignored)
{ (void)ref; (void)ignored; assert(id && name); return 0; }
XPLMMenuID XPLMCreateMenu(const char *name, XPLMMenuID parent, int item,
                         XPLMMenuHandler_f handler, void *ref)
{ (void)item; (void)ref; assert(name && !strcmp(name, "ISCS") && !parent && handler); return &child_menu_token; }
void XPLMDestroyMenu(XPLMMenuID id) { assert(id); ++menu_destroyed; }
void XPLMRemoveMenuItem(XPLMMenuID id, int index)
{ assert(id && index >= 0); ++menu_removed; }

void glColor3f(GLfloat r, GLfloat g, GLfloat b)
{ rgb_color[0] = r; rgb_color[1] = g; rgb_color[2] = b; }
void glLineWidth(GLfloat width) { line_width = width; }
void glBegin(GLenum mode) { primitive = mode; primitive_start = vertex_count; }
void glVertex2f(GLfloat x, GLfloat y)
{
    assert(isfinite(x) && isfinite(y) && vertex_count < ARRAY_LEN(vertices));
    vertices[vertex_count++] = (Vertex){x, y};
}
static void svg_color(const float *rgb)
{ fprintf(svg, "rgb(%d,%d,%d)", (int)(255 * rgb[0]), (int)(255 * rgb[1]), (int)(255 * rgb[2])); }
void glEnd(void)
{
    int i;
    if (!svg) return;
    if (primitive == GL_LINES) {
        for (i = primitive_start; i + 1 < vertex_count; i += 2) {
            fprintf(svg, "<path d=\"M%.2f %.2fL%.2f %.2f\" fill=\"none\" stroke=\"",
                    vertices[i].x - svg_left, svg_top - vertices[i].y,
                    vertices[i + 1].x - svg_left, svg_top - vertices[i + 1].y);
            svg_color(rgb_color); fprintf(svg, "\" stroke-width=\"%.2f\"/>\n", line_width);
        }
    } else {
        assert(primitive == GL_QUADS);
        fprintf(svg, "<polygon points=\"");
        for (i = primitive_start; i < vertex_count; ++i)
            fprintf(svg, "%.2f,%.2f ", vertices[i].x - svg_left, svg_top - vertices[i].y);
        fprintf(svg, "\" fill=\""); svg_color(rgb_color); fprintf(svg, "\"/>\n");
    }
}
void XPLMSetGraphicsState(int a, int b, int c, int d, int e, int f, int g)
{ (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; (void)g; }
float XPLMMeasureString(XPLMFontID font, const char *value, int length)
{ (void)font; (void)value; return length * 6.0f; }
static void xml_text(const char *value)
{
    for (; *value; ++value) {
        if (*value == '&') fputs("&amp;", svg);
        else if (*value == '<') fputs("&lt;", svg);
        else if (*value == '>') fputs("&gt;", svg);
        else fputc(*value, svg);
    }
}
void XPLMDrawString(float *rgb, int x, int y, const char *value, int *wrap, XPLMFontID font)
{
    Label *label;
    (void)wrap; (void)font;
    assert(label_count < ARRAY_LEN(labels));
    label = &labels[label_count++]; label->x = x; label->y = y;
    snprintf(label->value, sizeof(label->value), "%s", value);
    if (svg) {
        fprintf(svg, "<text x=\"%.2f\" y=\"%.2f\" font-family=\"monospace\" font-size=\"10\" fill=\"",
                x - svg_left, svg_top - y);
        svg_color(rgb); fputs("\">", svg); xml_text(value); fputs("</text>\n", svg);
    }
}

static void clear_capture(void) { vertex_count = label_count = 0; }
static int has_label(const char *value)
{
    int i;
    for (i = 0; i < label_count; ++i) if (!strcmp(labels[i].value, value)) return 1;
    return 0;
}
static void realistic_state(void)
{
    powered = 1; battery_level = 82; brightness = 85; battery_simulation = 1;
    ((MockData *)refs.on_ground)->value = 1;
    ((MockData *)refs.gpu)->value = 1;
    ((MockData *)refs.zulu)->value = 45240;
    ((MockData *)refs.altitude)->value = 2235;
    ((MockData *)refs.ground_speed)->value = 0;
    ((MockData *)refs.fuel_total)->value = 156000;
    ((MockData *)refs.payload)->value = 70336;
    ((MockData *)refs.gross_weight)->value = 491000;
    ((MockData *)refs.qnh)->value = 101325;
    ((MockData *)refs.wind_direction)->value = 250;
    ((MockData *)refs.wind_speed)->value = 8;
    ((MockData *)refs.latitude)->value = 43.3521;
    ((MockData *)refs.longitude)->value = 77.0405;
    ((MockData *)refs.com1)->value = 121900;
    ((MockData *)refs.com1_standby)->value = 118100;
    ((MockData *)refs.transponder)->value = 2000;
    ((MockData *)refs.transponder_mode)->value = 1;
    snprintf(status_message, sizeof(status_message), "Native A380 control panel ready");
}
static void validate_capture(int left, int bottom, int right, int top)
{
    int i;
    assert(vertex_count > 8 && label_count > 8);
    for (i = 0; i < vertex_count; ++i)
        assert(vertices[i].x >= left && vertices[i].x <= right &&
               vertices[i].y >= bottom && vertices[i].y <= top);
    for (i = 0; i < label_count; ++i) {
        assert(labels[i].x >= left && labels[i].x <= right &&
               labels[i].y >= bottom && labels[i].y <= top);
        assert(!strstr(labels[i].value, "ToLiss") && !strstr(labels[i].value, "ToLiSS")
               && !strstr(labels[i].value, "A330") && !strstr(labels[i].value, "1:1"));
    }
    assert(has_label("A380 CONTROL PANEL") || has_label("ISCS"));
}
static void render_page(const char *directory, int page, int popup)
{
    int left = popup ? 50 : 152, bottom = popup ? 40 : 516;
    int right = left + 720, top = bottom + 500;
    active_page = (EfbPage)page; iscs_tab = page % 8; clear_capture();
    if (directory) {
        char path[1024];
        snprintf(path, sizeof(path), "%s/efb-%02d-%s.svg", directory, page,
                 popup ? "popup" : "tablet");
        svg = fopen(path, "w"); assert(svg);
        svg_left = left; svg_top = top;
        fputs("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"720\" height=\"500\" viewBox=\"0 0 720 500\">\n", svg);
    }
    if (popup) a380x_iscs_draw_window(left, bottom, right, top);
    else a380x_efb_draw();
    if (svg) { fputs("</svg>\n", svg); fclose(svg); svg = NULL; }
    validate_capture(left, bottom, right, top);
    if (popup) assert(has_label("ISCS"));
    else assert(has_label(efb_action_names[page][0]));
}
static void test_clicks(void)
{
    static const char *navigation_commands[6] = {
        "sim/map/show_current", "sim/map/show_low_enroute", "sim/map/show_high_enroute",
        "sim/instruments/map_zoom_in", "sim/instruments/map_zoom_out", "sim/instruments/EFIS_apt"
    };
    int i, before;
    for (i = 0; i < PAGE_COUNT; ++i) {
        a380x_efb_click(194 + i * 70, 963);
        assert(active_page == (EfbPage)i);
    }
    active_page = PAGE_SETTINGS; metric_units = 1;
    a380x_efb_click(280, 616); assert(metric_units == 0);
    a380x_efb_click(280, 616); assert(metric_units == 1);
    a380x_efb_click(100, 616); assert(metric_units == 1);
    active_page = PAGE_NAVIGATION;
    for (i = 0; i < 6; ++i) {
        MockCommand *expected = XPLMFindCommand(navigation_commands[i]);
        int previous_calls = expected->calls;
        before = command_calls;
        a380x_efb_click(280 + (i % 3) * 230, i < 3 ? 616 : 560);
        assert(command_calls == before + 1 && expected->calls == previous_calls + 1);
    }
    active_page = PAGE_GROUND;
    for (i = 0; i < 20; ++i) ((MockData *)refs.doors)->floats[i] = 0;
    before = command_calls;
    a380x_efb_click(510, 560); assert(command_calls == before + 20);
    for (i = 0; i < 20; ++i) assert(((MockData *)refs.doors)->floats[i] == 1);
    a380x_efb_click(510, 560); assert(command_calls == before + 40);
    for (i = 0; i < 20; ++i) assert(((MockData *)refs.doors)->floats[i] == 0);
}
static void test_checklists_and_units(void)
{
    MockData *seatbelts = refs.seatbelts;
    MockData *gear = refs.gear;
    int i;
    active_page = PAGE_CHECKLISTS; selected_checklist = 0;
    memset(checklist_bits, 0, sizeof(checklist_bits));
    automatic_checklists = 1;
    seatbelts->value = 0;
    clear_capture(); a380x_efb_draw();
    assert(has_label("[ ] SEAT BELTS           ON"));
    seatbelts->value = 1;
    assert(checklist_completed_bits(0) & (1u << 2));
    clear_capture(); a380x_efb_draw();
    assert(has_label("[X] SEAT BELTS           ON"));
    automatic_checklists = 0;
    assert(!(checklist_completed_bits(0) & (1u << 2)));
    a380x_efb_click(200, 832);
    assert(checklist_bits[0] & 1u);
    a380x_efb_click(200, 832);
    assert(!(checklist_bits[0] & 1u));
    selected_checklist = 7;
    for (i = 0; i < 5; ++i) gear->floats[i] = 1.0f;
    automatic_checklists = 1;
    assert(checklist_completed_bits(7) & (1u << 1));
    gear->floats[4] = 0.5f;
    assert(!(checklist_completed_bits(7) & (1u << 1)));
    active_page = PAGE_DISPATCH;
    metric_units = 0;
    clear_capture(); a380x_efb_draw();
    assert(has_label("1082470 LB"));
    metric_units = 1;
    clear_capture(); a380x_efb_draw();
    assert(has_label("491000 KG"));
}
static void test_command_phases(void)
{
    EfbCommand entry = {"test", "test", COMMAND_ACTION, 1, NULL};
    active_page = PAGE_PERFORMANCE; target_altitude_ft = 3000;
    command_handler(NULL, xplm_CommandBegin, &entry);
    command_handler(NULL, xplm_CommandContinue, &entry);
    command_handler(NULL, xplm_CommandContinue, &entry);
    command_handler(NULL, xplm_CommandEnd, &entry);
    assert(target_altitude_ft == 4000);
    entry.kind = COMMAND_POWER; powered = 1;
    command_handler(NULL, xplm_CommandBegin, &entry);
    command_handler(NULL, xplm_CommandContinue, &entry);
    command_handler(NULL, xplm_CommandEnd, &entry);
    assert(powered == 0);
}
static int popup_mouse(float atlas_x, float atlas_y, XPLMMouseStatus phase)
{
    int x = mock_window.left + (int)((atlas_x - 152) * (mock_window.right - mock_window.left) / 720);
    int y = mock_window.bottom + (int)((atlas_y - 516) * (mock_window.top - mock_window.bottom) / 500);
    return mock_window.handleMouseClickFunc(&mock_window, x, y, phase, NULL);
}
static void test_popup(void)
{
    MockCommand *toggle = XPLMFindCommand("a380x/xp12/efb/window_toggle");
    assert(toggle && toggle->callback && !mock_window.visible);
    toggle->callback(toggle, xplm_CommandBegin, toggle->refcon);
    toggle->callback(toggle, xplm_CommandContinue, toggle->refcon);
    toggle->callback(toggle, xplm_CommandEnd, toggle->refcon);
    assert(mock_window.visible);
    powered = 0; battery_level = 0; ((MockData *)refs.gpu)->value = 0;
    clear_capture(); a380x_efb_draw(); assert(label_count == 0);
    clear_capture(); mock_window.drawWindowFunc(&mock_window, NULL);
    validate_capture(mock_window.left, mock_window.bottom, mock_window.right, mock_window.top);
    /* Resize and offset: the real host must map mouse coordinates back to atlas. */
    mock_window.left = 85; mock_window.bottom = 72;
    mock_window.right = 1525; mock_window.top = 1072;
    assert(popup_mouse(385, 946, xplm_MouseDown));
    assert(popup_mouse(385, 946, xplm_MouseUp));
    assert(iscs_tab == 2);
    clear_capture(); mock_window.drawWindowFunc(&mock_window, NULL);
    assert(has_label("MTOW 510000 KG  /  MLW 395000 KG"));
    assert(popup_mouse(400, 585, xplm_MouseDown));
    assert(popup_mouse(400, 585, xplm_MouseDrag));
    assert(popup_mouse(400, 585, xplm_MouseUp));
    assert(iscs_fuel_target > 0 && powered == 0);
    clear_capture(); mock_window.drawWindowFunc(&mock_window, NULL);
    validate_capture(85, 72, 1525, 1072);
    assert(mock_window.handleCursorFunc(&mock_window, 100, 100, NULL) == xplm_CursorArrow);
    powered = 1; battery_level = 82;
    clear_capture(); a380x_efb_draw();
    validate_capture(152, 516, 872, 1016);
}
static void test_iscs_aircraft_controls(void)
{
    MockData *doors = refs.doors;
    MockData *fuel = refs.fuel;
    float total = 0.0f;
    int i, before;
    realistic_state();
    for (i = 0; i < 20; ++i) doors->floats[i] = 0.0f;
    iscs_tab = 3; clear_capture(); a380x_iscs_draw_window(152, 516, 872, 1016);
    assert(has_label("AIRCRAFT DOORS"));
    before = command_calls;
    a380x_iscs_click(600, 845);
    assert(doors->floats[0] == 1.0f && command_calls == before + 1);
    a380x_iscs_click(600, 592);
    assert(doors->floats[16] == 1.0f);
    for (i = 0; i < 3; ++i) {
        int d, count = command_calls;
        a380x_iscs_click(580 + i * 105, 551);
        assert(doors->floats[17 + i] == 1.0f && command_calls == count + 1);
        for (d = 0; d < 3; ++d) assert(doors->floats[17 + d] == (d <= i ? 1.0f : 0.0f));
        a380x_iscs_click(580 + i * 105, 551);
        assert(doors->floats[17 + i] == 0.0f && command_calls == count + 2);
        /* Restore for the independence assertions on the next door. */
        a380x_iscs_click(580 + i * 105, 551);
    }
    ((MockData *)refs.on_ground)->value = 0;
    before = command_calls;
    a380x_iscs_click(580, 551);
    assert(command_calls == before);
    ((MockData *)refs.on_ground)->value = 1;
    iscs_tab = 2; clear_capture(); a380x_iscs_draw_window(152, 516, 872, 1016);
    assert(has_label("MTOW 510000 KG  /  MLW 395000 KG"));
    assert(has_label("LIVE ZFW 335000 / 373000 KG"));
    a380x_iscs_click(495, 585);
    a380x_iscs_click(240, 558);
    for (i = 0; i < 9; ++i) total += fuel->floats[i];
    assert(fabsf(total - MAX_FUEL_KG) < 1.0f);
    iscs_tab = 5; clear_capture(); a380x_iscs_draw_window(152, 516, 872, 1016);
    a380x_iscs_click(335, 770);
    assert(fabs(data_named("sim/operation/sound/engine_volume_ratio")->value - .5) < .01);
}
int main(int argc, char **argv)
{
    int page, popup;
    assert(a380x_efb_init()); realistic_state();
    for (popup = 0; popup < 2; ++popup)
        for (page = 0; page < PAGE_COUNT; ++page)
            render_page(argc > 1 ? argv[1] : NULL, page, popup);
    test_clicks(); test_checklists_and_units(); test_command_phases(); test_popup();
    test_iscs_aircraft_controls();
    a380x_efb_shutdown();
    /* An in-simulator aircraft reload must reuse persistent XPLM commands. */
    assert(a380x_efb_init()); realistic_state();
    active_page = PAGE_CHECKLISTS; clear_capture(); a380x_efb_draw();
    assert(has_label("CHECK ITEM"));
    a380x_efb_shutdown();
    assert(window_destroyed == 2 && menu_destroyed == 2 && menu_removed == 0);
    for (page = 0; page < mock_command_count; ++page) assert(!mock_commands[page].callback);
    puts("EFB: 10 tablet pages, eight ISCS tabs, A380 limits, native fuel and doors, popup clicks and cleanup passed");
    return 0;
}
