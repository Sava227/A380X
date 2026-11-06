/* SPDX-License-Identifier: GPL-3.0-or-later
 * TERRAIN on ND, production X-Plane adapter (a380x_terrain_adapter.inc), web JSON (a380x_xpkit_terrain.inc)
 * and native drawing (a380x_terrain_render.inc + the real EIS font atlas) against a mock XPLM whose
 * terrain probes answer from a synthetic world, with a real-time probe cost, and a small software
 * rasterizer standing in for OpenGL.
 *
 *   test_terrain_adapter                                  unit checks
 *   test_terrain_adapter --json <scenario> <out.json>     state "terrain" object after a full sweep
 *   test_terrain_adapter --capture <scenario> <out.pam>   native ND terrain layer, TERR box, pointer readout
 *   test_terrain_adapter --overlay <scenario> <out.pam>   the pointer readout alone (RGBA, for compositing)
 * Scenarios: arc (6000 ft, ARC 40 NM), rose (6000 ft, ROSE NAV 40 NM), peaks (FL350, ARC 160 NM),
 * approach (2500 ft, gear down, ARC 20 NM). Each with the pointer over a mountain.
 */
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMScenery.h"
#include "a380x_du_scale.h"
#include "a380x_efis.h"
#include "a380x_efis_nd.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- the unity-build environment the adapter needs ------------------------------------------- */
typedef struct { float r, g, b; } Color;
typedef struct { int nd_valid, navigation_valid, heading_valid; } Side;
static struct { Side side[2]; } refs;
static EfisSide efis[2];
static EfisNdData nd_data;
static int plugin_enabled = 1;
static float gear_values[5];
enum { SURV_XPDR_AUTO, SURV_ALT_RPTG, SURV_TCAS_LEVEL, SURV_TCAS_ALT_SELECT, SURV_TERR_OFF, SURV_GPWS_OFF,
       SURV_GS_OFF, SURV_FLAP_OFF, SURV_WXR_TAWS_SYS, SURV_XPDR_TCAS_SYS, SURV_STATE_COUNT };
static int survh_state[SURV_STATE_COUNT];
static int mock_cursor_inside[2], mock_cursor_x[2], mock_cursor_y[2], mock_redraws[2];
static XPLMAvionicsID mock_device[2] = {(XPLMAvionicsID)1, (XPLMAvionicsID)2};
static int mock_cursor_over(XPLMAvionicsID id, int *x, int *y)
{
    int side = id == mock_device[1];
    if (!mock_cursor_inside[side]) return 0;
    *x = mock_cursor_x[side]; *y = mock_cursor_y[side];
    return 1;
}
static void mock_needs_drawing(XPLMAvionicsID id) { ++mock_redraws[id == mock_device[1]]; }
static struct { int loaded; int (*cursor_over)(XPLMAvionicsID, int *, int *); void (*needs_drawing)(XPLMAvionicsID); } mfda_av = {
    1, mock_cursor_over, mock_needs_drawing};
static XPLMAvionicsID terrain_nd_device(int side) { return mock_device[side ? 1 : 0]; }

/* ---- mock XPLM: datarefs, probes, local coordinates --------------------------------------------- */
static double sim_time, probe_cost_s = 5e-6;
static int probe_calls, probe_enabled = 1, registered_refs;
static float vs_fpm;
static double wall(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
#define TERRAIN_NOW() sim_time

static const double REF_LAT = 45.0, REF_LON = 7.0, M_PER_DEG = 111320.0;
/* Synthetic world around (45 N, 7 E): rolling ground, three mountains, a lake at 2400 ft, the sea
 * west of 6.75 E, and no scenery (probe miss) north of 45.55 N. */
static double peak(double de, double dn, double bearing_deg, double distance_nm, double height_ft, double sigma_nm)
{
    double pe = distance_nm * sin(bearing_deg * M_PI / 180), pn = distance_nm * cos(bearing_deg * M_PI / 180);
    double d2 = (de - pe) * (de - pe) + (dn - pn) * (dn - pn);
    return height_ft * exp(-d2 / (2 * sigma_nm * sigma_nm));
}
static int world(double lat, double lon, float *elev_ft, int *wet)
{
    double dn = (lat - REF_LAT) * 60.0, de = (lon - REF_LON) * 60.0 * cos(REF_LAT * M_PI / 180), e, lake;
    if (lat > 45.55) return 0;
    *wet = 0;
    if (lon < 6.75) { *wet = 1; *elev_ft = 0; return 1; }
    e = 1200 + 500 * sin(de / 4.0) * cos(dn / 5.0) + peak(de, dn, 30, 18, 10500, 4.5) + peak(de, dn, 80, 24, 7500, 6) +
        peak(de, dn, 350, 30, 6000, 5) + peak(de, dn, 120, 12, 3000, 3);
    lake = hypot(de - 10 * sin(70 * M_PI / 180), dn - 10 * cos(70 * M_PI / 180));
    if (lake < 1.6) { *wet = 1; e = 2400; }
    *elev_ft = (float)e;
    return 1;
}
void XPLMWorldToLocal(double lat, double lon, double alt, double *x, double *y, double *z)
{
    *x = (lon - REF_LON) * M_PER_DEG * cos(REF_LAT * M_PI / 180); *y = alt; *z = -(lat - REF_LAT) * M_PER_DEG;
}
void XPLMLocalToWorld(double x, double y, double z, double *lat, double *lon, double *alt)
{
    *lat = REF_LAT - z / M_PER_DEG; *lon = REF_LON + x / (M_PER_DEG * cos(REF_LAT * M_PI / 180)); *alt = y;
}
XPLMProbeRef XPLMCreateProbe(XPLMProbeType type) { return probe_enabled && type == xplm_ProbeY ? (XPLMProbeRef)0x1 : NULL; }
void XPLMDestroyProbe(XPLMProbeRef probe) { (void)probe; }
XPLMProbeResult XPLMProbeTerrainXYZ(XPLMProbeRef probe, float x, float y, float z, XPLMProbeInfo_t *info)
{
    double lat, lon, alt, until = wall() + probe_cost_s;
    float elev;
    int wet;
    (void)y;
    assert(probe && info && info->structSize == (int)sizeof(*info));
    ++probe_calls;
    while (wall() < until) {}
    XPLMLocalToWorld(x, 0, z, &lat, &lon, &alt);
    if (!world(lat, lon, &elev, &wet)) return xplm_ProbeMissed;
    info->locationX = x; info->locationY = elev / 3.280839895f; info->locationZ = z; info->is_wet = wet;
    return xplm_ProbeHitTerrain;
}
XPLMDataRef XPLMFindDataRef(const char *name) { return !strcmp(name, "sim/flightmodel/position/vh_ind_fpm") ? (XPLMDataRef)&vs_fpm : NULL; }
float XPLMGetDataf(XPLMDataRef ref) { return *(float *)ref; }
XPLMDataRef XPLMRegisterDataAccessor(const char *name, XPLMDataTypeID type, int writable, XPLMGetDatai_f gi, XPLMSetDatai_f si,
    XPLMGetDataf_f gf, XPLMSetDataf_f sf, XPLMGetDatad_f gd, XPLMSetDatad_f sd, XPLMGetDatavi_f gvi, XPLMSetDatavi_f svi,
    XPLMGetDatavf_f gvf, XPLMSetDatavf_f svf, XPLMGetDatab_f gb, XPLMSetDatab_f sb, void *read, void *write)
{
    (void)type;(void)writable;(void)si;(void)sf;(void)gd;(void)sd;(void)gvi;(void)svi;(void)gvf;(void)svf;(void)gb;(void)sb;(void)write;
    assert(!strncmp(name, "a380x/xp12/terrain/", 19) && (gi || gf) && read);
    ++registered_refs;
    return (XPLMDataRef)read;
}
void XPLMUnregisterDataAccessor(XPLMDataRef ref) { (void)ref; --registered_refs; }

/* ---- software OpenGL: axis-aligned (textured) quads into a 768 x 1024 RGBA canvas, y up --------- */
typedef unsigned int GLuint;
typedef int GLint;
#define GL_QUADS 7
#define GL_TEXTURE_2D 0x0DE1
#define GL_TEXTURE_MIN_FILTER 0x2801
#define GL_TEXTURE_MAG_FILTER 0x2800
#define GL_TEXTURE_WRAP_S 0x2802
#define GL_TEXTURE_WRAP_T 0x2803
#define GL_NEAREST 0x2600
#define GL_LINEAR 0x2601
#define GL_CLAMP_TO_EDGE 0x812F
#define GL_RGBA 0x1908
#define GL_ALPHA 0x1906
#define GL_UNSIGNED_BYTE 0x1401
#define GL_UNPACK_ALIGNMENT 0x0CF5
#define CANVAS_W 768
#define CANVAS_H 1024
static float canvas[CANVAS_H][CANVAS_W][4];
static struct { int width, height, format; unsigned char *data; } textures[16];
static int texture_count, bound, texturing, quad_n, sub_uploads;
static float color[4] = {1, 1, 1, 1}, quad[4][4], next_u, next_v;
void XPLMGenerateTextureNumbers(int *ids, int count) { int i; for (i = 0; i < count; ++i) ids[i] = ++texture_count; assert(texture_count < 16); }
void XPLMBindTexture2d(int id, int unit) { (void)unit; bound = id; }
void XPLMSetGraphicsState(int fog, int tex, int light, int alpha_test, int blend, int depth, int depth_write)
{ (void)fog;(void)light;(void)alpha_test;(void)blend;(void)depth;(void)depth_write; texturing = tex > 0; }
static void glGetIntegerv(int name, GLint *value) { (void)name; *value = 4; }
static void glPixelStorei(int name, int value) { (void)name; (void)value; }
static void glTexParameteri(int target, int name, int value) { (void)target; (void)name; (void)value; }
static void glTexImage2D(int target, int level, int internal, int width, int height, int border, int format, int type, const void *data)
{
    size_t bytes = (size_t)width * height * (format == GL_ALPHA ? 1 : 4);
    (void)target; (void)level; (void)internal; (void)border; (void)type;
    free(textures[bound].data);
    textures[bound].data = calloc(bytes, 1);
    textures[bound].width = width; textures[bound].height = height; textures[bound].format = format;
    if (data) memcpy(textures[bound].data, data, bytes);
}
static void glTexSubImage2D(int target, int level, int x, int y, int width, int height, int format, int type, const void *data)
{
    int row;
    (void)target; (void)level; (void)format; (void)type;
    assert(x == 0 && width == textures[bound].width && y + height <= textures[bound].height && textures[bound].format == GL_RGBA);
    for (row = 0; row < height; ++row)
        memcpy(textures[bound].data + ((size_t)(y + row) * width) * 4, (const unsigned char *)data + (size_t)row * width * 4, (size_t)width * 4);
    ++sub_uploads;
}
static void glDeleteTextures(int count, const GLuint *ids) { (void)count; (void)ids; }
static void glColor3f(float r, float g, float b) { color[0] = r; color[1] = g; color[2] = b; color[3] = 1; }
static void glColor4f(float r, float g, float b, float a) { color[0] = r; color[1] = g; color[2] = b; color[3] = a; }
static void glBegin(int mode) { assert(mode == GL_QUADS); quad_n = 0; }
static void glTexCoord2f(float u, float v) { next_u = u; next_v = v; }
static void glVertex2f(float x, float y)
{
    quad[quad_n][0] = x; quad[quad_n][1] = y; quad[quad_n][2] = next_u; quad[quad_n][3] = next_v;
    if (++quad_n < 4) return;
    quad_n = 0;
    {   /* axis-aligned quad: v0 (left, top) v1 (right, top) v2 (right, bottom) v3 (left, bottom) in some winding */
        float x0 = fminf(fminf(quad[0][0], quad[1][0]), fminf(quad[2][0], quad[3][0]));
        float x1 = fmaxf(fmaxf(quad[0][0], quad[1][0]), fmaxf(quad[2][0], quad[3][0]));
        float y0 = fminf(fminf(quad[0][1], quad[1][1]), fminf(quad[2][1], quad[3][1]));
        float y1 = fmaxf(fmaxf(quad[0][1], quad[1][1]), fmaxf(quad[2][1], quad[3][1]));
        int k, ix, iy, lo = 0, hi = 0;
        float u_lo = 0, u_hi = 0, v_lo = 0, v_hi = 0;
        for (k = 0; k < 4; ++k) {   /* texture coordinates at the left/right and bottom/top edges */
            if (quad[k][0] == x0) u_lo = quad[k][2]; else u_hi = quad[k][2];
            if (quad[k][1] == y0) v_lo = quad[k][3]; else v_hi = quad[k][3];
        }
        (void)lo; (void)hi;
        for (iy = (int)ceilf(y0 - .5f); iy < (int)ceilf(y1 - .5f); ++iy) for (ix = (int)ceilf(x0 - .5f); ix < (int)ceilf(x1 - .5f); ++ix) {
            float src[4] = {color[0], color[1], color[2], color[3]}, a, *dst;
            if (ix < 0 || iy < 0 || ix >= CANVAS_W || iy >= CANVAS_H) continue;
            if (texturing && bound > 0 && textures[bound].data) {
                float u = u_lo + (u_hi - u_lo) * ((ix + .5f - x0) / (x1 - x0)), v = v_lo + (v_hi - v_lo) * ((iy + .5f - y0) / (y1 - y0));
                int tx = (int)floorf(u * textures[bound].width), ty = (int)floorf(v * textures[bound].height);
                const unsigned char *t;
                if (tx < 0) tx = 0;
                if (ty < 0) ty = 0;
                if (tx >= textures[bound].width) tx = textures[bound].width - 1;
                if (ty >= textures[bound].height) ty = textures[bound].height - 1;
                if (textures[bound].format == GL_ALPHA) src[3] *= textures[bound].data[(size_t)ty * textures[bound].width + tx] / 255.0f;
                else {
                    t = textures[bound].data + ((size_t)ty * textures[bound].width + tx) * 4;
                    for (k = 0; k < 3; ++k) src[k] *= t[k] / 255.0f;
                    src[3] *= t[3] / 255.0f;
                }
            }
            a = src[3]; dst = canvas[iy][ix];
            for (k = 0; k < 3; ++k) dst[k] = src[k] * a + dst[k] * (1 - a);
            dst[3] = a + dst[3] * (1 - a);
        }
    }
}
static void glEnd(void) { assert(quad_n == 0); }

/* ---- production sources -------------------------------------------------------------------------- */
#include "a380x_terrain_adapter.inc"
#include "a380x_ewd_font.inc"
#include "a380x_terrain_render.inc"
typedef struct { char data[262144]; size_t size; int failed; } XpKitJson;
static void xpkit_append(XpKitJson *j, const char *format, ...)
{
    va_list args;
    int n;
    if (j->failed) return;
    va_start(args, format);
    n = vsnprintf(j->data + j->size, sizeof(j->data) - j->size, format, args);
    va_end(args);
    if (n < 0 || (size_t)n >= sizeof(j->data) - j->size) j->failed = 1; else j->size += (size_t)n;
}
#include "a380x_xpkit_terrain.inc"

/* ---- scenarios ------------------------------------------------------------------------------------ */
typedef struct { const char *name; int mode; float range_nm, altitude_ft, heading; int gear; float px, py; } Scenario;
static const Scenario scenarios[] = {
    {"arc", 3, 40, 6000, 60, 0, 0, 0},
    {"rose", 2, 40, 6000, 60, 0, 0, 0},
    {"peaks", 3, 160, 35000, 60, 0, 0, 0},
    {"approach", 3, 20, 2500, 60, 1, 0, 0},
};

static void aircraft(const Scenario *s, int side)
{
    EfisSide *e = &efis[side];
    nd_data.position_valid = 1; nd_data.lat = REF_LAT; nd_data.lon = REF_LON;
    nd_data.true_heading = s->heading; nd_data.altitude_ft = s->altitude_ft;
    e->terr = 1; e->wx_requested = 0; e->zoom = 0; e->mode = s->mode; e->range_nm = s->range_nm; e->range_index = 3;
    refs.side[side].nd_valid = refs.side[side].navigation_valid = refs.side[side].heading_valid = 1;
    gear_values[0] = s->gear ? 1.0f : 0.0f;
    survh_state[SURV_WXR_TAWS_SYS] = 1; survh_state[SURV_TERR_OFF] = 0;
}

/* Logical ND pixel (y down) -> device texel the mock cursor reports (bottom-left origin, ND scale). */
static void point_cursor(int side, float x, float y_down)
{
    mock_cursor_inside[side] = 1;
    mock_cursor_x[side] = (int)floorf(x * A380X_ND_SCALE);
    mock_cursor_y[side] = (int)floorf((1023.0f - y_down) * A380X_ND_SCALE);
}

/* Simulated frames at 50 Hz; returns the worst real terrain_update time (s). */
static double run(double seconds)
{
    double worst = 0;
    int frames = (int)(seconds * 50 + .5), k;
    for (k = 0; k < frames; ++k) {
        double t0 = wall(), spent;
        sim_time += 0.02;
        terrain_update();
        spent = wall() - t0;
        if (spent > worst) worst = spent;
    }
    return worst;
}

/* ND pixel of the mountain at bearing 030 / 18 NM for a scenario (where the pointer goes). */
static void mountain_pixel(const Scenario *s, float *x, float *y)
{
    TerrGeometry g;
    double rel = (30.0 - s->heading) * M_PI / 180;
    assert(terr_geometry(s->mode, s->range_nm, &g));
    *x = g.ac_x + (float)(18.0 * sin(rel)) * g.px_per_nm;
    *y = g.ac_y - (float)(18.0 * cos(rel)) * g.px_per_nm;
}

static void scenario_setup(const Scenario *s, double seconds)
{
    float px, py;
    terrain_shutdown();
    memset(&nd_data, 0, sizeof(nd_data));
    memset(efis, 0, sizeof(efis));
    memset(mock_cursor_inside, 0, sizeof(mock_cursor_inside));
    terrain_init();
    aircraft(s, 0);
    mountain_pixel(s, &px, &py);
    point_cursor(0, px, py);
    run(seconds);
}

static void canvas_clear(void) { memset(canvas, 0, sizeof(canvas)); }

/* The native FBW-layout ND (a380x_nd_fbw.inc placement): the map aircraft reference at (384, 620) in
 * ARC and (384, 384) in ROSE, 498 / 250 px per range / range-half, its clip box; the TERR box; the
 * pointer readout. */
static void draw_native(int side, int with_overlay)
{
    const TerrDisplay *d = &nd_data.terrain[side];
    int arc = d->geo.arc;
    float cy = 1024.0f - (arc ? 620.0f : 384.0f), units = arc ? 498.0f / d->geo.range_nm : 250.0f / (d->geo.range_nm / 2);
    terrain_gl_draw(side, 384.0f, cy, units, arc ? 6.0f : 45.0f, arc ? 762.0f : 723.0f, 1024.0f - (arc ? 690.0f : 625.0f),
                    1024.0f - (arc ? 80.0f : 155.0f));
    terrain_draw_box_fbw(side);
    if (with_overlay) terrain_draw_overlay(side);
}

static void write_pam(const char *path, int rgba)
{
    FILE *f = fopen(path, "wb");
    int x, y;
    assert(f);
    fprintf(f, "P7\nWIDTH %d\nHEIGHT %d\nDEPTH %d\nMAXVAL 255\nTUPLTYPE %s\nENDHDR\n", CANVAS_W, CANVAS_H, rgba ? 4 : 3, rgba ? "RGB_ALPHA" : "RGB");
    for (y = CANVAS_H - 1; y >= 0; --y) for (x = 0; x < CANVAS_W; ++x) {
        unsigned char px[4];
        int k;
        for (k = 0; k < 4; ++k) px[k] = (unsigned char)lroundf(fminf(1, fmaxf(0, canvas[y][x][k])) * 255);
        fwrite(px, 1, rgba ? 4 : 3, f);
    }
    fclose(f);
}

/* canvas pixel (ND x, y down) */
static const float *at(int x, int y_down) { return canvas[1023 - y_down][x]; }

static void checks(void)
{
    const Scenario *arc = &scenarios[0];
    TerrDisplay *d = &nd_data.terrain[0];
    XpKitJson *j = calloc(1, sizeof(XpKitJson));
    int row, col, counts[TERR_CLASS_COUNT] = {0}, probes_before;
    double worst;
    float px, py;
    assert(j);
    /* Registration; nothing happens while TERR is off. */
    terrain_init();
    assert(registered_refs == 14 && terrain.probe);
    sim_time = 100;
    aircraft(arc, 0); efis[0].terr = 0;
    probes_before = probe_calls; run(1); assert(probe_calls == probes_before && !d->on && !terrain.active[0]);
    /* TERR on: the forward sector fills within a few seconds of 5 us probes, within the frame target. */
    efis[0].terr = 1;
    worst = run(1.0);
    assert(d->on && terrain.active[0] && !terrain.active[1] && !nd_data.terrain[1].on);
    worst = fmax(worst, run(4.0));
    printf("terrain adapter: ARC 40 NM, 5 us probes: %d probes in 5 s, coverage %.0f %%, worst frame %.0f us (target 500 + one probe), "
           "probe %.1f us avg\n", probe_calls - probes_before, terrain.coverage[0] * 100, worst * 1e6, terrain.probe_us);
    assert(terrain.coverage[0] > 0.999f);
    assert(worst < 0.0005 + 0.0002);   /* one 5 us probe of overrun plus scheduling noise */
    for (row = 0; row < d->geo.rows; ++row) for (col = 0; col < d->geo.cols; ++col) ++counts[d->cls[row][col]];
    assert(counts[TERR_RED_HIGH] && counts[TERR_YELLOW_HIGH] && counts[TERR_YELLOW_LOW] && counts[TERR_GREEN_HIGH] && counts[TERR_GREEN_LOW]);
    assert(counts[TERR_UNKNOWN] > 0 && counts[TERR_WATER] > 0);   /* north of 45.55 N: no scenery; the sea to the left */
    assert(d->shown.max_mode == TERR_LEVEL_RED && d->shown.max_elev >= 11000 && d->shown.max_elev <= 12500);
    /* Pointer over the 030 / 18 NM mountain: one direct probe; text and dataref in feet. */
    mountain_pixel(arc, &px, &py);
    {
        float expected;
        int wet, redraws = mock_redraws[0];
        double lat, lon;
        point_cursor(0, px, py);
        run(0.2);
        assert(nd_data.terrain_pointer[0].shown && fabsf(nd_data.terrain_pointer[0].x - px) < 1 && fabsf(nd_data.terrain_pointer[0].y - py) < 1);
        assert(terr_pointer_target(&d->geo, REF_LAT, REF_LON, arc->heading, nd_data.terrain_pointer[0].x, nd_data.terrain_pointer[0].y, &lat, &lon));
        assert(world(lat, lon, &expected, &wet) && fabsf(terrain.pointer_ft[0] - expected) < 1.0f && expected > 10000);
        assert(!strncmp(nd_data.terrain_pointer[0].text, "SIM ELEV ", 9) && strstr(nd_data.terrain_pointer[0].text, " FT"));
        assert(mock_redraws[0] > redraws);
        printf("terrain adapter: pointer at ND (%.0f, %.0f) -> \"%s\" (synthetic %.0f ft)\n", px, py, nd_data.terrain_pointer[0].text, expected);
        /* outside the terrain area (behind the aircraft): no readout */
        point_cursor(0, 384, 700); run(0.1);
        assert(!nd_data.terrain_pointer[0].shown && !nd_data.terrain_pointer[0].text[0] && terrain.pointer_ft[0] == -9999.0f);
        /* over missing scenery (true north 34 NM: 45.57 N): NO DATA */
        point_cursor(0, 384 + 34 * sinf(-60 * (float)M_PI / 180) * d->geo.px_per_nm, 626 - 34 * cosf(-60 * (float)M_PI / 180) * d->geo.px_per_nm);
        run(0.6);
        assert(nd_data.terrain_pointer[0].shown && !strcmp(nd_data.terrain_pointer[0].text, "SIM ELEV NO DATA"));
        mock_cursor_inside[0] = 0; run(0.1);
        assert(!nd_data.terrain_pointer[0].shown);
    }
    /* Web state: the run-length string decodes to the displayed raster. */
    xpkit_terrain_json(j);
    assert(!j->failed && strstr(j->data, "\"sys\":1,\"terrOff\":0,\"failed\":0,\"overlay\":[2,0]") && strstr(j->data, "\"on\":1"));
    {
        const char *cells = strstr(j->data, "\"cells\":\"") + 9;
        char *copy = strndup(cells, (size_t)(strchr(cells, '"') - cells));
        uint8_t (*decoded)[TERR_COLS_MAX] = calloc(TERR_ROWS_MAX, TERR_COLS_MAX);
        assert(copy && decoded && terr_rle_decode(copy, decoded, d->geo.rows, d->geo.cols) == d->geo.rows * d->geo.cols);
        for (row = 0; row < d->geo.rows; ++row) assert(!memcmp(decoded[row], d->cls[row], d->geo.cols));
        printf("terrain adapter: web state %d bytes for one ND (raster RLE %d bytes)\n", (int)j->size, (int)strlen(copy));
        free(decoded); free(copy);
    }
    /* Availability: TAWS SYS OFF, TERR SYS OFF, PLAN, ZOOM, ADIRS, ND power, WX selected -> nothing. */
    {
        int k;
        for (k = 0; k < 7; ++k) {
            aircraft(arc, 0);
            switch (k) {
            case 0: survh_state[SURV_WXR_TAWS_SYS] = 0; break;
            case 1: survh_state[SURV_TERR_OFF] = 1; break;
            case 2: efis[0].mode = 4; break;
            case 3: efis[0].zoom = 1; break;
            case 4: refs.side[0].heading_valid = 0; break;
            case 5: refs.side[0].nd_valid = 0; break;
            default: efis[0].terr = 0; efis[0].wx_requested = 1; break;
            }
            run(0.1);
            assert(!d->on && !terrain.active[0]);
            j->size = 0; j->failed = 0; xpkit_terrain_json(j);
            assert(strstr(j->data, "{\"on\":0"));
            if (k == 0) assert(strstr(j->data, "\"sys\":0"));
            if (k == 1) assert(strstr(j->data, "\"terrOff\":1"));
            if (k == 6) assert(strstr(j->data, "\"overlay\":[1,0]"));
        }
        aircraft(arc, 0); run(0.1); assert(d->on);
    }
    /* Runway cut-off: the nearest airport's probed elevation + 400 ft (FBW ramp near the runway). */
    {
        TerrAircraft ac = {REF_LAT, REF_LON, 60, 6000, 0, -500, 0};
        float elev;
        int wet;
        nd_data.nav_count[0] = 1;
        snprintf(nd_data.nav[0][0].id, sizeof(nd_data.nav[0][0].id), "TEST");
        nd_data.nav[0][0].lat = REF_LAT + 0.05; nd_data.nav[0][0].lon = REF_LON + 0.05;
        world(nd_data.nav[0][0].lat, nd_data.nav[0][0].lon, &elev, &wet);
        terrain.cutoff_at = -1e9;
        assert(fabsf(terrain_cutoff(&ac, sim_time) - (elev + 400)) < 1);
        ac.lat = nd_data.nav[0][0].lat; ac.lon = nd_data.nav[0][0].lon; ac.altitude_ft = elev + 50;
        terrain.cutoff_at = -1e9;
        assert(fabsf(terrain_cutoff(&ac, sim_time) - (elev + 200)) < 1);
        nd_data.nav_count[0] = 0; terrain.cutoff_at = -1e9;
        assert(terrain_cutoff(&ac, sim_time) == -500);
    }
    /* Both NDs on one range share one cache; different ranges get their own (at most three). */
    aircraft(arc, 1); run(0.5);
    assert(nd_data.terrain[1].on && terrain.levels[0] && !terrain.levels[1]);
    efis[1].range_nm = 80; run(0.5);
    assert(terrain.levels[1] && terrain.levels[1]->range_nm == 80 && nd_data.terrain[1].geo.range_nm == 80);
    /* No probe object: TAWS failed (TERR INOP on the FBW ND), nothing displayed. */
    terrain_shutdown(); probe_enabled = 0; terrain_init(); aircraft(arc, 0); run(0.2);
    assert(!d->on && terrain.failed);
    j->size = 0; j->failed = 0; xpkit_terrain_json(j); assert(strstr(j->data, "\"failed\":1"));
    probe_enabled = 1;
    /* Native drawing: the terrain image, the TERR box and the readout at the pointer. */
    scenario_setup(arc, 6.0);
    canvas_clear(); sub_uploads = 0;
    draw_native(0, 1);
    assert(sub_uploads == d->geo.rows);
    mountain_pixel(arc, &px, &py);
    {   /* the mountain: high-density red dots near it (native ARC reference is 6 px above the web one) */
        int x, y, red = 0, white = 0, cyan = 0;
        for (y = (int)py - 6 - 12; y < (int)py - 6 + 12; ++y) for (x = (int)px - 12; x < (int)px + 12; ++x)
            red += at(x, y)[0] > .99f && at(x, y)[1] < .01f;
        assert(red > 50);
        for (y = 590; y < 620; ++y) for (x = 690; x < 760; ++x) cyan += at(x, y)[0] < .1f && at(x, y)[1] > .9f && at(x, y)[2] > .9f;
        assert(cyan > 20);   /* "TERR" */
        for (y = (int)py + 28; y < (int)py + 58; ++y) for (x = (int)px + 24; x < (int)px + 220 && x < 768; ++x) white += at(x, y)[0] > .9f && at(x, y)[1] > .9f && at(x, y)[2] > .9f;
        assert(white > 100);   /* readout text */
    }
    canvas_clear(); sub_uploads = 0;
    draw_native(0, 0);
    assert(sub_uploads == 0);   /* texture unchanged: no upload */
    terrain_gl_shutdown();
    ewd_font_clip_y(0, 0);
    ewd_font_shutdown();
    terrain_shutdown();
    assert(registered_refs == 0);
    free(j);
}

int main(int argc, char **argv)
{
    if (argc == 4) {
        const Scenario *s = NULL;
        size_t k;
        for (k = 0; k < sizeof(scenarios) / sizeof(scenarios[0]); ++k) if (!strcmp(scenarios[k].name, argv[2])) s = &scenarios[k];
        assert(s);
        probe_cost_s = 0;
        sim_time = 1000;
        scenario_setup(s, 12.0);
        if (!strcmp(argv[1], "--json")) {
            XpKitJson *j = calloc(1, sizeof(XpKitJson));
            FILE *f = fopen(argv[3], "w");
            assert(j && f);
            xpkit_terrain_json(j);
            assert(!j->failed);
            fprintf(f, "{\"terrain\":%s,\"pointer\":{\"x\":%.1f,\"y\":%.1f,\"text\":\"%s\"}}\n", j->data,
                    nd_data.terrain_pointer[0].x, nd_data.terrain_pointer[0].y, nd_data.terrain_pointer[0].text);
            fclose(f); free(j);
        } else {
            canvas_clear();
            if (!strcmp(argv[1], "--capture")) draw_native(0, 1);
            else terrain_draw_overlay(0);
            write_pam(argv[3], 1);
        }
        return 0;
    }
    checks();
    puts("TERRAIN adapter: activation/availability, probe budget and coverage, misses (no data), cache sharing, "
         "runway cut-off, pointer probe/readout, web JSON and native image/box/readout drawing passed");
    return 0;
}
