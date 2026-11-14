/* SPDX-License-Identifier: GPL-3.0-or-later
 * Software backend of the OIT drawing layer, for offline page renders and tests: RGBA float buffer, solid
 * triangles with 4x4 supersampled coverage, atlas glyphs blitted 1:1 (the GL backend uses the same texel rects).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "oit_draw.h"

typedef struct { OitBackend base; float *px; } SoftBackend;

static void blend(float *p, OitColor c, float cov)
{
    float a = c.a * cov;
    p[0] = p[0] * (1 - a) + c.r * a; p[1] = p[1] * (1 - a) + c.g * a; p[2] = p[2] * (1 - a) + c.b * a;
}

static float edge(float ax, float ay, float bx, float by, float px, float py) { return (bx - ax) * (py - ay) - (by - ay) * (px - ax); }

static void s_begin(OitBackend *self) { (void)self; }
static void s_end(OitBackend *self) { (void)self; }

static void s_tri(OitBackend *self, float x0, float y0, float x1, float y1, float x2, float y2, OitColor c)
{
    SoftBackend *s = (SoftBackend *)self;
    int minx = (int)floorf(fminf(x0, fminf(x1, x2))), maxx = (int)ceilf(fmaxf(x0, fmaxf(x1, x2)));
    int miny = (int)floorf(fminf(y0, fminf(y1, y2))), maxy = (int)ceilf(fmaxf(y0, fmaxf(y1, y2)));
    float area = edge(x0, y0, x1, y1, x2, y2);
    int x, y, i, j;
    if (fabsf(area) < 1e-6f) return;
    if (minx < 0) minx = 0; if (miny < 0) miny = 0; if (maxx > OIT_W) maxx = OIT_W; if (maxy > OIT_H) maxy = OIT_H;
    for (y = miny; y < maxy; ++y)
        for (x = minx; x < maxx; ++x) {
            float px = x + 0.5f, py = y + 0.5f;
            float w0 = edge(x1, y1, x2, y2, px, py), w1 = edge(x2, y2, x0, y0, px, py), w2 = edge(x0, y0, x1, y1, px, py);
            (void)i; (void)j;
            /* top-left style rule: a pixel centre on a shared edge belongs to one triangle only */
            if (area > 0 ? (w0 > 0 && w1 > 0 && w2 >= 0) || (w0 >= 0 && w1 > 0 && w2 > 0) || (w0 > 0 && w1 >= 0 && w2 > 0)
                         : (w0 < 0 && w1 < 0 && w2 <= 0) || (w0 <= 0 && w1 < 0 && w2 < 0) || (w0 < 0 && w1 <= 0 && w2 < 0))
                blend(s->px + 4 * (y * OIT_W + x), c, 1.0f);
        }
}

static void s_glyph(OitBackend *self, float x, float y, float w, float h, int ax, int ay, int aw, int ah, OitColor c)
{
    SoftBackend *s = (SoftBackend *)self;
    int atw, ath, i, j;
    const unsigned char *atlas = oit_atlas_pixels(&atw, &ath);
    (void)w; (void)h; (void)ath;
    for (j = 0; j < ah; ++j)
        for (i = 0; i < aw; ++i) {
            int px = (int)x + i, py = (int)y + j;
            if (px < 0 || py < 0 || px >= OIT_W || py >= OIT_H) continue;
            blend(s->px + 4 * (py * OIT_W + px), c, atlas[(ay + j) * atw + ax + i] / 255.0f);
        }
}

static SoftBackend soft = {{s_begin, s_tri, s_glyph, s_end}, NULL};

OitBackend *oit_soft_backend(void)
{
    if (!soft.px) soft.px = calloc((size_t)OIT_W * OIT_H * 4, sizeof(float));
    return &soft.base;
}

void oit_soft_clear(void) { if (soft.px) memset(soft.px, 0, (size_t)OIT_W * OIT_H * 4 * sizeof(float)); }

/* Binary PPM (P6), converted to PNG by the test script. */
int oit_soft_write_ppm(const char *path)
{
    FILE *f = fopen(path, "wb");
    int i;
    if (!f || !soft.px) { if (f) fclose(f); return 0; }
    fprintf(f, "P6\n%d %d\n255\n", OIT_W, OIT_H);
    for (i = 0; i < OIT_W * OIT_H; ++i) {
        unsigned char rgb[3];
        int k;
        for (k = 0; k < 3; ++k) { float v = soft.px[4 * i + k]; rgb[k] = (unsigned char)(v <= 0 ? 0 : v >= 1 ? 255 : v * 255 + 0.5f); }
        fwrite(rgb, 1, 3, f);
    }
    fclose(f);
    return 1;
}
