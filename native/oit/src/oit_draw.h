/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380X OIT drawing layer: a tiny 2D primitive set (rectangles, lines, triangles, atlas text) with two backends,
 * OpenGL inside X-Plane (oit_gl.c) and a software rasterizer for offline page renders (oit_soft.c).
 * Coordinates: pixels, origin top-left, y down; the screen is OIT_W x OIT_H.
 */
#pragma once
#include <stdint.h>

#define OIT_W 1024
#define OIT_H 798

typedef struct { float r, g, b, a; } OitColor;

enum { OIT_LEFT = 0, OIT_CENTER = 1, OIT_RIGHT = 2 };

typedef struct OitBackend {
    void (*begin)(struct OitBackend *self);
    /* Solid triangle (screen pixels, y down). */
    void (*tri)(struct OitBackend *self, float x0, float y0, float x1, float y1, float x2, float y2, OitColor c);
    /* Textured quad from the font atlas: dst rect (pixels) <- atlas rect (texels), alpha-modulated colour. */
    void (*glyph)(struct OitBackend *self, float x, float y, float w, float h, int ax, int ay, int aw, int ah, OitColor c);
    void (*end)(struct OitBackend *self);
} OitBackend;

void oit_set_backend(OitBackend *b);
void oit_frame_begin(void);
void oit_frame_end(void);
void oit_rect(float x, float y, float w, float h, OitColor c);
void oit_frame(float x, float y, float w, float h, float t, OitColor c);
void oit_line(float x0, float y0, float x1, float y1, float t, OitColor c);
void oit_triangle(float x0, float y0, float x1, float y1, float x2, float y2, OitColor c);
/* Text: y is the top of the line box; returns the advance width. face = OIT_F_* (oit_font.h). */
float oit_text(int face, float x, float y, const char *s, OitColor c, int align);
float oit_text_width(int face, const char *s);
int oit_face_height(int face);
/* printf-style convenience. */
float oit_textf(int face, float x, float y, OitColor c, int align, const char *fmt, ...);

/* Font atlas access for the backends. */
const unsigned char *oit_atlas_pixels(int *w, int *h);

static inline OitColor oit_rgb(unsigned hex) { OitColor c = {((hex >> 16) & 255) / 255.0f, ((hex >> 8) & 255) / 255.0f, (hex & 255) / 255.0f, 1.0f}; return c; }
static inline OitColor oit_rgba(unsigned hex, float a) { OitColor c = oit_rgb(hex); c.a = a; return c; }
