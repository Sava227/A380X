/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380X OIT drawing layer (see oit_draw.h). Backend-independent: shapes become triangles, text becomes atlas
 * quads from the generated Manrope / JetBrains Mono atlas.
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "oit_draw.h"
#include "oit_font.h"

static OitBackend *be;

void oit_set_backend(OitBackend *b) { be = b; }
void oit_frame_begin(void) { if (be) be->begin(be); }
void oit_frame_end(void) { if (be) be->end(be); }
const unsigned char *oit_atlas_pixels(int *w, int *h) { *w = OIT_ATLAS_W; *h = OIT_ATLAS_H; return oit_atlas; }

void oit_triangle(float x0, float y0, float x1, float y1, float x2, float y2, OitColor c)
{
    if (be && c.a > 0.0f) be->tri(be, x0, y0, x1, y1, x2, y2, c);
}

void oit_rect(float x, float y, float w, float h, OitColor c)
{
    if (w <= 0 || h <= 0) return;
    oit_triangle(x, y, x + w, y, x + w, y + h, c);
    oit_triangle(x, y, x + w, y + h, x, y + h, c);
}

void oit_frame(float x, float y, float w, float h, float t, OitColor c)
{
    oit_rect(x, y, w, t, c);
    oit_rect(x, y + h - t, w, t, c);
    oit_rect(x, y + t, t, h - 2 * t, c);
    oit_rect(x + w - t, y + t, t, h - 2 * t, c);
}

void oit_line(float x0, float y0, float x1, float y1, float t, OitColor c)
{
    float dx = x1 - x0, dy = y1 - y0, len = sqrtf(dx * dx + dy * dy), nx, ny;
    if (len < 1e-3f) return;
    nx = -dy / len * t * 0.5f; ny = dx / len * t * 0.5f;
    oit_triangle(x0 + nx, y0 + ny, x1 + nx, y1 + ny, x1 - nx, y1 - ny, c);
    oit_triangle(x0 + nx, y0 + ny, x1 - nx, y1 - ny, x0 - nx, y0 - ny, c);
}

static const OitGlyph *glyph_for(int face, unsigned char ch)
{
    int idx;
    if (face < 0 || face >= OIT_F_COUNT) return NULL;
    if (ch >= 32 && ch < 127) idx = ch - 32;
    else if (ch >= 128 && ch < 128 + (OIT_GLYPHS_PER_FACE - 95)) idx = 95 + (ch - 128);
    else idx = '?' - 32;
    return &oit_glyphs[oit_faces[face].first + idx];
}

int oit_face_height(int face) { return face >= 0 && face < OIT_F_COUNT ? oit_faces[face].ascent + oit_faces[face].descent : 0; }

float oit_text_width(int face, const char *s)
{
    int adv = 0;
    for (; s && *s; ++s) { const OitGlyph *g = glyph_for(face, (unsigned char)*s); if (g) adv += g->adv64; }
    return adv / 64.0f;
}

float oit_text(int face, float x, float y, const char *s, OitColor c, int align)
{
    float w = oit_text_width(face, s), pen;
    if (!s || face < 0 || face >= OIT_F_COUNT) return 0.0f;
    if (align == OIT_CENTER) x -= w * 0.5f; else if (align == OIT_RIGHT) x -= w;
    pen = floorf(x + 0.5f);
    y = floorf(y + 0.5f);
    for (; *s; ++s) {
        const OitGlyph *g = glyph_for(face, (unsigned char)*s);
        if (!g) continue;
        if (g->w && g->h && be && c.a > 0.0f)
            be->glyph(be, floorf(pen + g->xoff + 0.5f), y + g->yoff, g->w, g->h, g->x, g->y, g->w, g->h, c);
        pen += g->adv64 / 64.0f;
    }
    return w;
}

float oit_textf(int face, float x, float y, OitColor c, int align, const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return oit_text(face, x, y, buf, c, align);
}
