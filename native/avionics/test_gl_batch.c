/* SPDX-License-Identifier: GPL-3.0-or-later
 * a380x_gl_batch.h: the same scene drawn in immediate mode and batched gives the same triangles and line
 * segments (positions after the transform, colours, texture coordinates, line width, graphics state and
 * texture), in the same order, with far fewer draw calls. A fake OpenGL normalises both paths. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef unsigned int GLenum;
typedef unsigned int GLuint;
typedef int GLint;
typedef int GLsizei;
typedef float GLfloat;
typedef unsigned char GLboolean;
typedef unsigned char GLubyte;
#define IBM 0
#define GL_POINTS 0x0000
#define GL_LINES 0x0001
#define GL_LINE_LOOP 0x0002
#define GL_LINE_STRIP 0x0003
#define GL_TRIANGLES 0x0004
#define GL_TRIANGLE_STRIP 0x0005
#define GL_TRIANGLE_FAN 0x0006
#define GL_QUADS 0x0007
#define GL_FLOAT 0x1406
#define GL_VERTEX_ARRAY 0x8074
#define GL_COLOR_ARRAY 0x8076
#define GL_TEXTURE_COORD_ARRAY 0x8078
#define GL_BLEND 0x0BE2

/* ---- fake OpenGL: a 2D transform stack and a normalised primitive log ---- */
typedef struct { int kind; float p[3][2], c[3][4], t[3][2]; float width; int gs, tex; } Prim;
static Prim log_[20000];
static int log_n, draw_calls, gl_errors;
static float m[6] = {1, 0, 0, 1, 0, 0}, stack_[16][6], cur_c[4] = {1, 1, 1, 1}, cur_t[2], width_ = 1;
static int depth, gs_, tex_, begun = -1, nb;
static float bv[64][8];
static struct { const float *v, *t, *c; int sv, st, sc, on; } arrays;

static void xform(float x, float y, float out[2]) { out[0] = m[0] * x + m[2] * y + m[4]; out[1] = m[1] * x + m[3] * y + m[5]; }
static void emit(int kind, float (*v)[8], const int *idx, int n)
{
    Prim *p = &log_[log_n++];
    assert(log_n < 20000);
    memset(p, 0, sizeof(*p));
    p->kind = kind; p->width = kind == GL_LINES ? width_ : 0; p->gs = gs_; p->tex = gs_ & 2 ? tex_ : 0;
    for (int i = 0; i < n; ++i) {
        xform(v[idx[i]][0], v[idx[i]][1], p->p[i]);
        memcpy(p->c[i], &v[idx[i]][2], sizeof(p->c[i]));
        if (gs_ & 2) memcpy(p->t[i], &v[idx[i]][6], sizeof(p->t[i]));
    }
}
/* canonical form: triangles and line segments, as a380x_gl_batch.h converts them */
static void normalise(int mode, float (*v)[8], int n)
{
    int i;
    switch (mode) {
    case GL_LINES: for (i = 0; i + 1 < n; i += 2) emit(GL_LINES, v, (int[]){i, i + 1}, 2); break;
    case GL_LINE_STRIP: for (i = 0; i + 1 < n; ++i) emit(GL_LINES, v, (int[]){i, i + 1}, 2); break;
    case GL_LINE_LOOP: for (i = 0; i + 1 < n; ++i) emit(GL_LINES, v, (int[]){i, i + 1}, 2); if (n > 1) emit(GL_LINES, v, (int[]){n - 1, 0}, 2); break;
    case GL_TRIANGLES: for (i = 0; i + 2 < n; i += 3) emit(GL_TRIANGLES, v, (int[]){i, i + 1, i + 2}, 3); break;
    case GL_QUADS: for (i = 0; i + 3 < n; i += 4) { emit(GL_TRIANGLES, v, (int[]){i, i + 1, i + 2}, 3); emit(GL_TRIANGLES, v, (int[]){i, i + 2, i + 3}, 3); } break;
    case GL_TRIANGLE_FAN: for (i = 1; i + 1 < n; ++i) emit(GL_TRIANGLES, v, (int[]){0, i, i + 1}, 3); break;
    case GL_TRIANGLE_STRIP: for (i = 0; i + 2 < n; ++i) emit(GL_TRIANGLES, v, (int[]){i & 1 ? i + 1 : i, i & 1 ? i : i + 1, i + 2}, 3); break;
    case GL_POINTS: for (i = 0; i < n; ++i) emit(GL_POINTS, v, (int[]){i}, 1); break;
    default: assert(!"mode");
    }
}
static void glBegin(GLenum mode) { assert(begun < 0); begun = (int)mode; nb = 0; ++draw_calls; }
static void glVertex2f(GLfloat x, GLfloat y)
{
    assert(begun >= 0 && nb < 64);
    float *v = bv[nb++];
    v[0] = x; v[1] = y; memcpy(v + 2, cur_c, sizeof(cur_c)); memcpy(v + 6, cur_t, sizeof(cur_t));
}
static void glEnd(void) { assert(begun >= 0); normalise(begun, bv, nb); begun = -1; }
static void glColor4f(GLfloat r, GLfloat g, GLfloat b, GLfloat a) { cur_c[0] = r; cur_c[1] = g; cur_c[2] = b; cur_c[3] = a; }
static void glTexCoord2f(GLfloat u, GLfloat v) { cur_t[0] = u; cur_t[1] = v; }
static void glLineWidth(GLfloat w) { width_ = w; }
static void glPushMatrix(void) { assert(depth < 16); memcpy(stack_[depth++], m, sizeof(m)); }
static void glPopMatrix(void) { assert(depth > 0); memcpy(m, stack_[--depth], sizeof(m)); }
static void glTranslatef(GLfloat x, GLfloat y, GLfloat z) { (void)z; m[4] += m[0] * x + m[2] * y; m[5] += m[1] * x + m[3] * y; }
static void glScalef(GLfloat x, GLfloat y, GLfloat z) { (void)z; m[0] *= x; m[1] *= x; m[2] *= y; m[3] *= y; }
static void glRotatef(GLfloat deg, GLfloat x, GLfloat y, GLfloat z)
{
    float a = deg * (float)M_PI / 180.f, c = cosf(a), s = sinf(a), n[4] = {m[0] * c + m[2] * s, m[1] * c + m[3] * s, -m[0] * s + m[2] * c, -m[1] * s + m[3] * c};
    (void)x; (void)y; assert(z == 1.f); memcpy(m, n, sizeof(n));
}
static void XPLMSetGraphicsState(int f, int t, int l, int a, int b, int d, int w) { (void)f; (void)l; (void)a; (void)d; (void)w; gs_ = (t ? 2 : 0) | (b ? 1 : 0); }
static void XPLMBindTexture2d(int t, int unit) { if (unit == 0) tex_ = t; }
static void XPLMDrawString(float *c, int x, int y, char *s, int *w, int f) { (void)c; (void)s; (void)w; (void)f; float v[1][8] = {{(float)x, (float)y, 9, 9, 9, 9, 0, 0}}; emit(GL_POINTS, v, (int[]){0}, 1); ++draw_calls; }
static void glBindBuffer(GLenum target, GLuint b) { (void)target; assert(b == 0); }
static void glClientActiveTexture(GLenum t) { (void)t; }
static void glEnableClientState(GLenum a) { if (a == GL_VERTEX_ARRAY) arrays.on |= 1; if (a == GL_TEXTURE_COORD_ARRAY) arrays.on |= 2; if (a == GL_COLOR_ARRAY) arrays.on |= 4; }
static void glDisableClientState(GLenum a) { if (a == GL_VERTEX_ARRAY) arrays.on &= ~1; if (a == GL_TEXTURE_COORD_ARRAY) arrays.on &= ~2; if (a == GL_COLOR_ARRAY) arrays.on &= ~4; }
static void glVertexPointer(GLint s, GLenum t, GLsizei st, const void *p) { assert(s == 2 && t == GL_FLOAT); arrays.v = p; arrays.sv = st; }
static void glTexCoordPointer(GLint s, GLenum t, GLsizei st, const void *p) { assert(s == 2 && t == GL_FLOAT); arrays.t = p; arrays.st = st; }
static void glColorPointer(GLint s, GLenum t, GLsizei st, const void *p) { assert(s == 4 && t == GL_FLOAT); arrays.c = p; arrays.sc = st; }
static void glDrawArrays(GLenum mode, GLint first, GLsizei count)
{
    assert(arrays.on == 7 && first == 0 && (mode == GL_LINES || mode == GL_TRIANGLES));
    ++draw_calls;
    int per = mode == GL_LINES ? 2 : 3;
    assert(count % per == 0);
    for (int i = 0; i < count; i += per) {
        float v[3][8];
        for (int k = 0; k < per; ++k) {
            const char *b = (const char *)arrays.v + (i + k) * arrays.sv;
            memcpy(v[k], b, 2 * sizeof(float));
            memcpy(v[k] + 2, (const char *)arrays.c + (i + k) * arrays.sc, 4 * sizeof(float));
            memcpy(v[k] + 6, (const char *)arrays.t + (i + k) * arrays.st, 2 * sizeof(float));
        }
        emit((int)mode, v, (int[]){0, 1, 2}, per);
    }
}

#include "a380x_gl_batch.h"

/* ---- the scene: every primitive kind, transforms, textures, state repeats, text, points, growth ---- */
static void quad(float x, float y, float w, float h) { glBegin(GL_QUADS); glVertex2f(x, y); glVertex2f(x + w, y); glVertex2f(x + w, y + h); glVertex2f(x, y + h); glEnd(); }
static void scene(void)
{
    int i;
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    glLineWidth(2.f);
    glColor3f(1, 0, 0); quad(0, 0, 10, 5);
    glColor3f(0, 1, 0); glBegin(GL_LINES); glVertex2f(0, 0); glVertex2f(5, 5); glVertex2f(1, 1); glVertex2f(2, 9); glEnd();
    glBegin(GL_LINE_STRIP); for (i = 0; i < 5; ++i) glVertex2f((float)i, (float)(i * i)); glEnd();
    glLineWidth(2.f);                                                     /* repeat: no flush */
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    glColor4f(0, 0, 1, .5f); glBegin(GL_TRIANGLE_FAN); for (i = 0; i < 7; ++i) glVertex2f(cosf(i), sinf(i)); glEnd();
    glPushMatrix(); glTranslatef(10, 5, 0); glRotatef(30, 0, 0, 1);
    quad(1, 1, 3, 2);
    glBegin(GL_LINE_LOOP); glVertex2f(0, 0); glVertex2f(4, 0); glVertex2f(4, 4); glEnd();
    glPopMatrix();
    glLineWidth(3.f);
    glBegin(GL_LINES); glVertex2f(-1, -1); glVertex2f(-3, 2); glEnd();
    XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0); XPLMBindTexture2d(7, 0); glColor4f(.5f, .5f, .5f, 1);
    glBegin(GL_QUADS);
    for (i = 0; i < 2; ++i) {
        glTexCoord2f(0, 0); glVertex2f(20.f + i, 0); glTexCoord2f(1, 0); glVertex2f(21.f + i, 0);
        glTexCoord2f(1, 1); glVertex2f(21.f + i, 1); glTexCoord2f(0, 1); glVertex2f(20.f + i, 1);
    }
    glEnd();
    XPLMBindTexture2d(7, 0);                                              /* repeat: no flush */
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    glColor4ub(255, 128, 0, 255);
    glBegin(GL_TRIANGLE_STRIP); for (i = 0; i < 5; ++i) glVertex2f((float)i, (float)(i & 1)); glEnd();
    glBegin(GL_TRIANGLES); glColor3f(1, 1, 0); glVertex2f(0, 0); glColor3f(0, 1, 1); glVertex2f(1, 0); glVertex2f(0, 1);
    glVertex2f(5, 5); glVertex2f(6, 5); glVertex2f(5, 6); glEnd();          /* colour changes inside a block */
    float white[3] = {1, 1, 1};
    XPLMDrawString(white, 3, 4, "TEXT", NULL, 0);
    glBegin(GL_POINTS); glVertex2f(1, 2); glVertex2f(3, 4); glEnd();       /* not converted: replayed as drawn */
    glScalef(2, 2, 1);
    for (i = 0; i < 3000; ++i) { glColor3f(i / 3000.f, 0, 1); quad((float)(i % 50), (float)(i / 50), .8f, .8f); }  /* buffer growth */
    glColor3f(1, 1, 1); glBegin(GL_LINES); glVertex2f(0, 0); glVertex2f(1, 1); glEnd();
}

static int run(int batch)
{
    memset(m, 0, sizeof(m)); m[0] = m[3] = 1; depth = 0; log_n = draw_calls = 0; begun = -1; width_ = 1; gs_ = tex_ = 0;
    gb.ready = batch ? 1 : -1;
    gb_scope_begin();
    scene();
    gb_scope_end();
    assert(depth == 0 && begun < 0 && arrays.on == 0);
    return draw_calls;
}

static int same(const Prim *a, const Prim *b)
{
    if (a->kind != b->kind || a->width != b->width || a->gs != b->gs || a->tex != b->tex) return 0;
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 2; ++k)
            if (fabsf(a->p[i][k] - b->p[i][k]) > 1e-4f || fabsf(a->t[i][k] - b->t[i][k]) > 1e-6f) return 0;
    return !memcmp(a->c, b->c, sizeof(a->c));
}

static Prim ref_[20000];
int main(void)
{
    int immediate = run(0), n = log_n;
    memcpy(ref_, log_, sizeof(Prim) * (size_t)n);
    assert(fabsf(cur_c[0] - 1.f) < 1e-6f);
    int batched = run(1);
    assert(log_n == n);
    for (int i = 0; i < n; ++i)
        if (!same(&ref_[i], &log_[i])) { fprintf(stderr, "primitive %d differs (kind %d vs %d)\n", i, ref_[i].kind, log_[i].kind); return 1; }
    assert(batched * 20 < immediate);
    assert(cur_c[0] == 1.f && cur_c[1] == 1.f && cur_c[2] == 1.f);            /* current colour restored after a flush */
    (void)gl_errors;
    printf("GL BATCH OK %d primitives identical; %d immediate blocks -> %d draws\n", n, immediate, batched);
    return 0;
}
