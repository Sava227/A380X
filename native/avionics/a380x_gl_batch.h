/* SPDX-License-Identifier: GPL-3.0-or-later
 * Deferred immediate mode for the native cockpit screens (PFD, ND).
 *
 * Through X-Plane's OpenGL bridge (Metal on macOS, Vulkan or Zink on Windows) every glBegin..glEnd
 * block is its own driver round trip: the native PFD issues about 600 per frame, the ND up to 300.
 * Between gb_scope_begin() and gb_scope_end(), glBegin / glVertex2f / glTexCoord2f / glColor / glEnd
 * are recorded into one vertex array (quads, fans, polygons and triangle strips as triangles; lines,
 * strips and loops as line segments) and drawn with one glDrawArrays per run. The run is flushed
 * before anything that changes how it draws: any other GL or XPLM drawing call, a real change of line
 * width, graphics state or texture, or a switch between lines and triangles. So the picture is what
 * immediate mode drew, in the same order; the PFD drops to about 34 draws per frame, the ND to about 20.
 * Repeated identical line widths, graphics states and texture binds inside a scope are dropped.
 * Outside a scope every call goes straight to OpenGL.
 *
 * Include once, after the GL and XPLM headers and before the drawing code; the routing macros at the
 * end send that code's calls here (define A380X_GL_BATCH_NO_ROUTE to get the functions only).
 */
#ifndef A380X_GL_BATCH_H
#define A380X_GL_BATCH_H
#include <stdlib.h>
#include <string.h>

#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER 0x8892
#endif
#ifndef GL_TEXTURE0
#define GL_TEXTURE0 0x84C0
#endif
#ifndef GL_POLYGON
#define GL_POLYGON 0x0009
#endif

typedef struct { float x, y, u, v, r, g, b, a; } GbVertex;

static struct {
    int scope, kind;                 /* kind: 0 nothing pending, GL_LINES or GL_TRIANGLES */
    GbVertex *out, *prim;            /* pending run; vertices of the open glBegin block */
    int n, cap, pn, pcap, mode, open;
    float color[4], uv[2];
    float line_width;                /* < 0: unknown */
    int gs_key, tex[2];              /* -1: unknown */
    int ready;                       /* 1 batching possible, -1 not (immediate mode only), 0 unprobed */
} gb;

#if IBM
typedef void (APIENTRY *GbBindBufferFn)(GLenum, GLuint);
typedef void (APIENTRY *GbClientActiveTextureFn)(GLenum);
static GbBindBufferFn gb_bind_buffer;
static GbClientActiveTextureFn gb_client_active_texture;
/* opengl32.dll exports OpenGL 1.1 only; a current context is needed, so probe on the first scope. */
typedef void (*GbProc)(void);   /* the generic function pointer GCC lets any function pointer pass through */
static GbProc gb_proc(const char *name, const char *alias)
{
    GbProc p = (GbProc)wglGetProcAddress(name);
    return p ? p : (GbProc)wglGetProcAddress(alias);
}
static int gb_probe(void)
{
    gb_bind_buffer = (GbBindBufferFn)gb_proc("glBindBuffer", "glBindBufferARB");
    gb_client_active_texture = (GbClientActiveTextureFn)gb_proc("glClientActiveTexture", "glClientActiveTextureARB");
    return gb_bind_buffer && gb_client_active_texture;
}
#else
#define gb_bind_buffer glBindBuffer
#define gb_client_active_texture glClientActiveTexture
static int gb_probe(void) { return 1; }
#endif

static int gb_grow(GbVertex **v, int *cap, int need)
{
    if (need <= *cap) return 1;
    int c = *cap ? *cap : 4096;
    while (c < need) c *= 2;
    GbVertex *p = (GbVertex *)realloc(*v, (size_t)c * sizeof(GbVertex));
    if (!p) return 0;
    *v = p; *cap = c;
    return 1;
}

static void gb_flush(void)
{
    if (gb.n > 0) {
        gb_bind_buffer(GL_ARRAY_BUFFER, 0);               /* client-side arrays: no buffer object bound */
        gb_client_active_texture(GL_TEXTURE0);
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        glVertexPointer(2, GL_FLOAT, sizeof(GbVertex), &gb.out[0].x);
        glTexCoordPointer(2, GL_FLOAT, sizeof(GbVertex), &gb.out[0].u);
        glColorPointer(4, GL_FLOAT, sizeof(GbVertex), &gb.out[0].r);
        glDrawArrays((GLenum)gb.kind, 0, gb.n);
        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
        /* the current colour / texcoord are undefined after drawing with those arrays */
        glColor4f(gb.color[0], gb.color[1], gb.color[2], gb.color[3]);
        glTexCoord2f(gb.uv[0], gb.uv[1]);
    }
    gb.n = 0;
    gb.kind = 0;
}

/* Before any call the batch does not model: draw what is pending, forget the cached state. */
static void gb_generic(void)
{
    if (!gb.scope) return;
    gb_flush();
    gb.line_width = -1.0f;
    gb.gs_key = gb.tex[0] = gb.tex[1] = -1;
}

static void gb_put(int kind, const GbVertex *v)
{
    if (kind != gb.kind) { gb_flush(); gb.kind = kind; }
    if (!gb_grow(&gb.out, &gb.cap, gb.n + 1)) { gb_flush(); gb.kind = kind; if (!gb_grow(&gb.out, &gb.cap, 1)) return; }
    gb.out[gb.n++] = *v;
}

/* Immediate-mode replay of one block (modes the batch does not convert, or out of memory). */
static void gb_replay(int mode, const GbVertex *v, int count)
{
    int i;
    gb_flush();
    glBegin((GLenum)mode);
    for (i = 0; i < count; ++i) { glColor4f(v[i].r, v[i].g, v[i].b, v[i].a); glTexCoord2f(v[i].u, v[i].v); glVertex2f(v[i].x, v[i].y); }
    glEnd();
    glColor4f(gb.color[0], gb.color[1], gb.color[2], gb.color[3]);
    glTexCoord2f(gb.uv[0], gb.uv[1]);
}

static void gb_glBegin(GLenum mode)
{
    if (!gb.scope) { glBegin(mode); return; }
    gb.mode = (int)mode; gb.pn = 0; gb.open = 1;
}

static void gb_glVertex2f(GLfloat x, GLfloat y)
{
    if (!gb.scope) { glVertex2f(x, y); return; }
    if (!gb.open || !gb_grow(&gb.prim, &gb.pcap, gb.pn + 1)) return;
    gb.prim[gb.pn++] = (GbVertex){x, y, gb.uv[0], gb.uv[1], gb.color[0], gb.color[1], gb.color[2], gb.color[3]};
}

static void gb_glEnd(void)
{
    const GbVertex *p = gb.prim;
    int i, n = gb.pn;
    if (!gb.scope) { glEnd(); return; }
    if (!gb.open) return;
    gb.open = 0;
    switch (gb.mode) {
    case GL_LINES:      for (i = 0; i + 1 < n; i += 2) { gb_put(GL_LINES, &p[i]); gb_put(GL_LINES, &p[i + 1]); } break;
    case GL_LINE_STRIP: for (i = 0; i + 1 < n; ++i) { gb_put(GL_LINES, &p[i]); gb_put(GL_LINES, &p[i + 1]); } break;
    case GL_LINE_LOOP:
        for (i = 0; i + 1 < n; ++i) { gb_put(GL_LINES, &p[i]); gb_put(GL_LINES, &p[i + 1]); }
        if (n > 1) { gb_put(GL_LINES, &p[n - 1]); gb_put(GL_LINES, &p[0]); }
        break;
    case GL_TRIANGLES:  for (i = 0; i + 2 < n; i += 3) { gb_put(GL_TRIANGLES, &p[i]); gb_put(GL_TRIANGLES, &p[i + 1]); gb_put(GL_TRIANGLES, &p[i + 2]); } break;
    case GL_QUADS:
        for (i = 0; i + 3 < n; i += 4) {
            gb_put(GL_TRIANGLES, &p[i]); gb_put(GL_TRIANGLES, &p[i + 1]); gb_put(GL_TRIANGLES, &p[i + 2]);
            gb_put(GL_TRIANGLES, &p[i]); gb_put(GL_TRIANGLES, &p[i + 2]); gb_put(GL_TRIANGLES, &p[i + 3]);
        }
        break;
    case GL_TRIANGLE_FAN: case GL_POLYGON:
        for (i = 1; i + 1 < n; ++i) { gb_put(GL_TRIANGLES, &p[0]); gb_put(GL_TRIANGLES, &p[i]); gb_put(GL_TRIANGLES, &p[i + 1]); }
        break;
    case GL_TRIANGLE_STRIP:
        for (i = 0; i + 2 < n; ++i) {
            gb_put(GL_TRIANGLES, &p[i & 1 ? i + 1 : i]); gb_put(GL_TRIANGLES, &p[i & 1 ? i : i + 1]); gb_put(GL_TRIANGLES, &p[i + 2]);
        }
        break;
    default: gb_replay(gb.mode, p, n); break;             /* points, quad strips: as drawn */
    }
}

static void gb_color(float r, float g, float b, float a)
{
    gb.color[0] = r; gb.color[1] = g; gb.color[2] = b; gb.color[3] = a;
    if (!gb.scope) glColor4f(r, g, b, a);
}
static void gb_glTexCoord2f(GLfloat u, GLfloat v)
{
    gb.uv[0] = u; gb.uv[1] = v;
    if (!gb.scope) glTexCoord2f(u, v);
}

static void gb_glLineWidth(GLfloat width)
{
    if (gb.scope && width == gb.line_width) return;
    if (gb.scope) { gb_flush(); gb.line_width = width; }
    glLineWidth(width);
}

static void gb_XPLMSetGraphicsState(int fog, int tex, int light, int alpha_test, int blend, int depth_test, int depth_write)
{
    int key = (fog & 1) | (tex & 3) << 1 | (light & 1) << 3 | (alpha_test & 1) << 4 | (blend & 1) << 5 | (depth_test & 1) << 6 | (depth_write & 1) << 7;
    if (gb.scope && key == gb.gs_key) return;
    if (gb.scope) { gb_flush(); gb.gs_key = key; }
    XPLMSetGraphicsState(fog, tex, light, alpha_test, blend, depth_test, depth_write);
}

static void gb_XPLMBindTexture2d(int texture, int unit)
{
    if (gb.scope && unit >= 0 && unit < 2 && texture == gb.tex[unit]) return;
    if (gb.scope) { gb_flush(); if (unit >= 0 && unit < 2) gb.tex[unit] = texture; }
    XPLMBindTexture2d(texture, unit);
}

/* Around one native screen draw. Both set a white current colour, so a screen draws the same with or
 * without batching; batching stays off when the GL entry points it needs are missing. */
static void gb_scope_begin(void)
{
    if (!gb.ready) gb.ready = gb_probe() ? 1 : -1;
    gb.color[0] = gb.color[1] = gb.color[2] = gb.color[3] = 1.0f;
    gb.uv[0] = gb.uv[1] = 0.0f;
    glColor4f(1, 1, 1, 1);
    gb.scope = gb.ready > 0;
    gb.n = gb.pn = gb.open = gb.kind = 0;
    gb.line_width = -1.0f;
    gb.gs_key = gb.tex[0] = gb.tex[1] = -1;
}

static void gb_scope_end(void)
{
    if (!gb.scope) return;
    gb_flush();
    gb.scope = 0;
}

#ifndef A380X_GL_BATCH_NO_ROUTE
/* Batched calls */
#define glBegin(m) gb_glBegin(m)
#define glEnd() gb_glEnd()
#define glVertex2f(x, y) gb_glVertex2f(x, y)
#define glTexCoord2f(u, v) gb_glTexCoord2f(u, v)
#define glColor3f(r, g, b) gb_color(r, g, b, 1.0f)
#define glColor4f(r, g, b, a) gb_color(r, g, b, a)
#define glColor4fv(c) gb_color((c)[0], (c)[1], (c)[2], (c)[3])
#define glColor3fv(c) gb_color((c)[0], (c)[1], (c)[2], 1.0f)
#define glColor4ub(r, g, b, a) gb_color((r) / 255.0f, (g) / 255.0f, (b) / 255.0f, (a) / 255.0f)
#define glLineWidth(w) gb_glLineWidth(w)
#define XPLMSetGraphicsState(a, b, c, d, e, f, g) gb_XPLMSetGraphicsState(a, b, c, d, e, f, g)
#define XPLMBindTexture2d(t, u) gb_XPLMBindTexture2d(t, u)
/* Everything else the drawing code may call: draw what is pending first */
#define GB_GENERIC(call) (gb_generic(), (call))
#define glPushMatrix() GB_GENERIC(glPushMatrix())
#define glPopMatrix() GB_GENERIC(glPopMatrix())
#define glTranslatef(x, y, z) GB_GENERIC(glTranslatef(x, y, z))
#define glRotatef(a, x, y, z) GB_GENERIC(glRotatef(a, x, y, z))
#define glScalef(x, y, z) GB_GENERIC(glScalef(x, y, z))
#define glLoadIdentity() GB_GENERIC(glLoadIdentity())
#define glMultMatrixf(m) GB_GENERIC(glMultMatrixf(m))
#define glEnable(c) GB_GENERIC(glEnable(c))
#define glDisable(c) GB_GENERIC(glDisable(c))
#define glBlendFunc(s, d) GB_GENERIC(glBlendFunc(s, d))
#define glColorMask(r, g, b, a) GB_GENERIC(glColorMask(r, g, b, a))
#define glClear(m) GB_GENERIC(glClear(m))
#define glClearColor(r, g, b, a) GB_GENERIC(glClearColor(r, g, b, a))
#define glScissor(x, y, w, h) GB_GENERIC(glScissor(x, y, w, h))
#define glStencilFunc(f, r, m) GB_GENERIC(glStencilFunc(f, r, m))
#define glStencilOp(a, b, c) GB_GENERIC(glStencilOp(a, b, c))
#define glStencilMask(m) GB_GENERIC(glStencilMask(m))
#define glPointSize(s) GB_GENERIC(glPointSize(s))
#define glBindTexture(t, id) GB_GENERIC(glBindTexture(t, id))
#define glTexParameteri(t, n, v) GB_GENERIC(glTexParameteri(t, n, v))
#define glTexImage2D(t, l, i, w, h, b, f, ty, d) GB_GENERIC(glTexImage2D(t, l, i, w, h, b, f, ty, d))
#define glTexSubImage2D(t, l, x, y, w, h, f, ty, d) GB_GENERIC(glTexSubImage2D(t, l, x, y, w, h, f, ty, d))
#define glDrawArrays(m, f, c) GB_GENERIC(glDrawArrays(m, f, c))
#define glVertex2i(x, y) gb_glVertex2f((GLfloat)(x), (GLfloat)(y))
#define glPixelStorei(n, v) GB_GENERIC(glPixelStorei(n, v))
#define glDeleteTextures(n, t) GB_GENERIC(glDeleteTextures(n, t))
#define glGetFloatv(n, v) GB_GENERIC(glGetFloatv(n, v))
#define glGetIntegerv(n, v) GB_GENERIC(glGetIntegerv(n, v))
#define glIsEnabled(c) GB_GENERIC(glIsEnabled(c))
#define glEnableClientState(a) GB_GENERIC(glEnableClientState(a))
#define glDisableClientState(a) GB_GENERIC(glDisableClientState(a))
#define glVertexPointer(s, t, st, p) GB_GENERIC(glVertexPointer(s, t, st, p))
#define glTexCoordPointer(s, t, st, p) GB_GENERIC(glTexCoordPointer(s, t, st, p))
#define glColorPointer(s, t, st, p) GB_GENERIC(glColorPointer(s, t, st, p))
#define XPLMDrawString(c, x, y, s, w, f) GB_GENERIC(XPLMDrawString(c, x, y, s, w, f))
#define XPLMDrawNumber(c, x, y, v, d, dec, sh, f) GB_GENERIC(XPLMDrawNumber(c, x, y, v, d, dec, sh, f))
#define XPLMDrawTranslucentDarkBox(l, t, r, b) GB_GENERIC(XPLMDrawTranslucentDarkBox(l, t, r, b))
#endif

#endif
