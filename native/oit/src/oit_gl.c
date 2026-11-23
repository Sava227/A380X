/* SPDX-License-Identifier: GPL-3.0-or-later
 * OpenGL backend of the OIT drawing layer, used inside X-Plane's avionics-device draw callback (the context is
 * set up by X-Plane in device pixels, origin bottom-left). Everything is one textured-triangle stream from the
 * font atlas (GL_ALPHA + GL_MODULATE: colour from glColor, alpha = colour alpha x atlas alpha); solid shapes
 * sample the atlas' opaque corner block.
 */
#if APL
#include <OpenGL/gl.h>
#elif IBM
#include <windows.h>
#include <GL/gl.h>
#else
#include <GL/gl.h>
#endif
#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F   /* GL 1.2; Windows headers stop at 1.1 */
#endif
#include "XPLMGraphics.h"
#include "oit_draw.h"
#include "oit_font.h"

typedef struct { OitBackend base; int tex; int open; } GlBackend;

static void put(float x, float y, float u, float v, OitColor c)
{
    glColor4f(c.r, c.g, c.b, c.a);
    glTexCoord2f(u, v);
    glVertex2f(x, (float)OIT_H - y);
}

static void g_begin(OitBackend *self)
{
    GlBackend *g = (GlBackend *)self;
    if (!g->tex) {
        int w, h;
        const unsigned char *px = oit_atlas_pixels(&w, &h);
        XPLMGenerateTextureNumbers(&g->tex, 1);
        XPLMBindTexture2d(g->tex, 0);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, w, h, 0, GL_ALPHA, GL_UNSIGNED_BYTE, px);
    }
    XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0);
    XPLMBindTexture2d(g->tex, 0);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glBegin(GL_TRIANGLES);
    g->open = 1;
}

static void g_tri(OitBackend *self, float x0, float y0, float x1, float y1, float x2, float y2, OitColor c)
{
    (void)self;
    put(x0, y0, OIT_WHITE_U, OIT_WHITE_V, c);
    put(x1, y1, OIT_WHITE_U, OIT_WHITE_V, c);
    put(x2, y2, OIT_WHITE_U, OIT_WHITE_V, c);
}

static void g_glyph(OitBackend *self, float x, float y, float w, float h, int ax, int ay, int aw, int ah, OitColor c)
{
    float u0 = (float)ax / OIT_ATLAS_W, v0 = (float)ay / OIT_ATLAS_H;
    float u1 = (float)(ax + aw) / OIT_ATLAS_W, v1 = (float)(ay + ah) / OIT_ATLAS_H;
    (void)self;
    put(x, y, u0, v0, c); put(x + w, y, u1, v0, c); put(x + w, y + h, u1, v1, c);
    put(x, y, u0, v0, c); put(x + w, y + h, u1, v1, c); put(x, y + h, u0, v1, c);
}

static void g_end(OitBackend *self)
{
    GlBackend *g = (GlBackend *)self;
    if (g->open) glEnd();
    g->open = 0;
}

static GlBackend gl = {{g_begin, g_tri, g_glyph, g_end}, 0, 0};

OitBackend *oit_gl_backend(void) { return &gl.base; }

/* Called from XPluginStop / disable: textures are re-created on the next draw. */
void oit_gl_release(void)
{
    if (gl.tex) {
        GLuint t = (GLuint)gl.tex;
        glDeleteTextures(1, &t);
        gl.tex = 0;
    }
}
