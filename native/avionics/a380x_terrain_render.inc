/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * TERRAIN on ND for the native ND renderers (docs/ND_TERRAIN.md): the side's terrain raster
 * (nd_data.terrain[side], a380x_terrain.h) as one RGBA texture per ND - pattern pixels made by
 * terr_render_row, only the raster rows that changed re-uploaded (glTexSubImage2D) - drawn as a single
 * quad under the map symbols; the FBW TERR box (the 768 x 1024 layout); the simulator-aid elevation
 * readout next to the ND pointer (both renderers and the xp-kit web ND device).
 * Included by a380x_pfd_render.inc. Needs Color, ewd_font_draw, nd_data and the GL/XPLM graphics API.
 */
static void ewd_font_draw(float x, float y, const char *text, float size, Color color, float tracking, int anchor);

typedef struct {
    int texture, width, height, uploaded;
    TerrGeometry geo;
    uint8_t cls[TERR_ROWS_MAX][TERR_COLS_MAX];
} TerrainGl;
static TerrainGl terrain_gl[2];
static unsigned char terrain_gl_band[TERR_COLS_MAX * TERR_CELL_PX * TERR_CELL_PX * 4];

/* Upload the rows of the side's raster that differ from the texture (all of them after a geometry change). */
static void terrain_gl_upload(int side)
{
    const TerrDisplay *d = &nd_data.terrain[side];
    TerrainGl *g = &terrain_gl[side];
    const int width = d->geo.cols * TERR_CELL_PX, height = d->geo.rows * TERR_CELL_PX;
    int row;
    if (!g->texture) XPLMGenerateTextureNumbers(&g->texture, 1);
    XPLMBindTexture2d(g->texture, 0);
    if (!g->uploaded || width != g->width || height != g->height || memcmp(&g->geo, &d->geo, sizeof(g->geo))) {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
        g->width = width; g->height = height; g->geo = d->geo; g->uploaded = 1;
        memset(g->cls, 0xff, sizeof(g->cls));
    }
    for (row = 0; row < d->geo.rows; ++row) {
        if (!memcmp(g->cls[row], d->cls[row], (size_t)d->geo.cols)) continue;
        terr_render_row(d, row, terrain_gl_band, (size_t)width * 4);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, row * TERR_CELL_PX, width, TERR_CELL_PX, GL_RGBA, GL_UNSIGNED_BYTE, terrain_gl_band);
        memcpy(g->cls[row], d->cls[row], (size_t)d->geo.cols);
    }
}

/* The side's terrain image on a native map: (cx, cy) the renderer position of the aircraft reference,
 * units_per_nm its scale (y up), clipped to the box [clip_l, clip_r] x [clip_b, clip_t]. The raster
 * keeps the position, heading and scale it was swept for, as the real display does between sweeps. */
static void terrain_gl_draw(int side, float cx, float cy, float units_per_nm,
                            float clip_l, float clip_r, float clip_b, float clip_t)
{
    const TerrDisplay *d = &nd_data.terrain[side];
    float k, left, right, top, bottom, u0, u1, v0, v1;
    if (side < 0 || side > 1 || !d->on || !d->geo.valid || !(units_per_nm > 0.0f)) return;
    terrain_gl_upload(side);
    k = units_per_nm / d->geo.px_per_nm;
    {
        const float l0 = cx + ((float)d->geo.x0 - d->geo.ac_x) * k, r0 = l0 + (float)terrain_gl[side].width * k;
        const float t0 = cy - ((float)d->geo.y0 - d->geo.ac_y) * k, b0 = t0 - (float)terrain_gl[side].height * k;
        left = fmaxf(l0, clip_l); right = fminf(r0, clip_r); top = fminf(t0, clip_t); bottom = fmaxf(b0, clip_b);
        u0 = (left - l0) / (r0 - l0); u1 = (right - l0) / (r0 - l0);
        v0 = (t0 - top) / (t0 - b0); v1 = (t0 - bottom) / (t0 - b0);
    }
    if (right <= left || top <= bottom) return;
    XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0);
    XPLMBindTexture2d(terrain_gl[side].texture, 0);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    glBegin(GL_QUADS);
    glTexCoord2f(u0, v0); glVertex2f(left, top);
    glTexCoord2f(u1, v0); glVertex2f(right, top);
    glTexCoord2f(u1, v1); glVertex2f(right, bottom);
    glTexCoord2f(u0, v1); glVertex2f(left, bottom);
    glEnd();
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
}

static void terrain_gl_shutdown(void)
{
    int side;
    for (side = 0; side < 2; ++side) {
        if (terrain_gl[side].texture) {
            GLuint texture = (GLuint)terrain_gl[side].texture;
            glDeleteTextures(1, &texture);
        }
        memset(&terrain_gl[side], 0, sizeof(terrain_gl[side]));
    }
}

static void terrain_fill(float x0, float y0, float x1, float y1, float r, float g, float b, float a)
{
    glColor4f(r, g, b, a);
    glBegin(GL_QUADS);
    glVertex2f(x0, y0); glVertex2f(x1, y0); glVertex2f(x1, y1); glVertex2f(x0, y1);
    glEnd();
}

/* One-pixel outline of an FBW rect (y down) in the y-up 768 x 1024 layout. */
static void terrain_outline(float x, float y, float w, float h, float r, float g, float b)
{
    float top = 1024.0f - y, bottom = 1024.0f - y - h;
    terrain_fill(x - .5f, top - .5f, x + w + .5f, top + .5f, r, g, b, 1);
    terrain_fill(x - .5f, bottom - .5f, x + w + .5f, bottom + .5f, r, g, b, 1);
    terrain_fill(x - .5f, bottom, x + .5f, top, r, g, b, 1);
    terrain_fill(x + w - .5f, bottom, x + w + .5f, top, r, g, b, 1);
}

#if !defined(A380X_XPKIT)   /* the FBW-layout native ND (a380x_nd_fbw.inc); xp-kit builds show the FBW bundle's own box */
static Color terrain_level_color(int mode)
{
    Color c = {0.0f, 1.0f, 0.0f};                        /* FBW TerrainMapThresholds rgb(0,255,0) */
    if (mode == TERR_LEVEL_RED) { c.r = 1.0f; c.g = 0.0f; }
    else if (mode == TERR_LEVEL_YELLOW) c.r = 1.0f;
    return c;
}

/* FBW TerrainMapThresholds (fbw-common/src/systems/instruments/src/ND/TerrainMapThresholds.tsx) on the
 * native 768 x 1024 ND: cyan "TERR" (688, 612, translated 5, 4), the highest elevation over the lowest
 * in hundreds of feet in two yellow-outlined boxes at (700, 619) and (700, 643), each digit string in
 * the colour of its band. Hidden while there is no terrain in view. */
static void terrain_draw_box_fbw(int side)
{
    const TerrDisplay *d = &nd_data.terrain[side];
    const Color cyan = {0.0f, 1.0f, 1.0f};
    char upper[16], lower[16];
    if (!d->on || !terr_threshold_text(&d->shown, upper, lower)) return;
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    ewd_font_draw(693.0f, 1024.0f - 616.0f, "TERR", 23.0f, cyan, 0, 0);
    terrain_outline(700.0f, 619.0f, 54.0f, 24.0f, 1, 1, 0);
    terrain_outline(700.0f, 643.0f, 54.0f, 24.0f, 1, 1, 0);
    ewd_font_draw(709.0f, 1024.0f - 639.0f, upper, 22.0f, terrain_level_color(d->shown.max_mode), 0, 0);
    if (lower[0]) ewd_font_draw(709.0f, 1024.0f - 663.0f, lower, 22.0f, terrain_level_color(d->shown.min_mode), 0, 0);
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
}
#endif

/* The simulator-aid readout: a dark grey box with a light outline beside the ND pointer, white
 * "SIM ELEV <ft> FT" (terrain_pointer_update). Drawn in the device's 768 x 1024 logical space (y up),
 * after the map and the pointer. Not an avionics indication. */
static void terrain_draw_overlay(int side)
{
    const TerrPointer *p = &nd_data.terrain_pointer[side];
    const Color white = {1.0f, 1.0f, 1.0f};
    const float size = 20.0f, h = 30.0f;
    float w, x, y_down, top, bottom;
    if (side < 0 || side > 1 || !p->shown || !p->text[0]) return;
    w = (float)strlen(p->text) * size * 0.625f + 18.0f;   /* EIS glyph advance 40 / 64 */
    x = p->x + 24.0f; y_down = p->y + 28.0f;
    if (x + w > 764.0f) x = p->x - 24.0f - w;
    if (y_down + h > 1020.0f) y_down = p->y - 28.0f - h;
    top = 1024.0f - y_down; bottom = top - h;
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    terrain_fill(x, bottom, x + w, top, 0.10f, 0.11f, 0.13f, 0.88f);
    terrain_outline(x, y_down, w, h, 0.75f, 0.78f, 0.82f);
    ewd_font_draw(x + 9.0f, bottom + 8.0f, p->text, size, white, 0, 0);
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    glColor4f(1, 1, 1, 1);
}
