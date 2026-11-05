/* SPDX-License-Identifier: GPL-3.0-or-later
 * Zero-copy display frames (xp-kit protocol v9, macOS): implementation of a380x_xpkit_surface.h.
 * A separate translation unit, so the IOSurface / CoreFoundation / MacTypes names (Point, Rect, ...)
 * never meet the avionics unity build. Linked into the macOS plugin and test_xpkit_zero_copy.
 */
#include "a380x_xpkit_surface.h"
#include <IOSurface/IOSurface.h>
#include <CoreFoundation/CoreFoundation.h>
#include <OpenGL/OpenGL.h>
#include <OpenGL/CGLIOSurface.h>
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"

static void xpkit_surface_dict_int(CFMutableDictionaryRef d, CFStringRef key, int value)
{
    CFNumberRef number = CFNumberCreate(NULL, kCFNumberIntType, &value);
    if (number) { CFDictionarySetValue(d, key, number); CFRelease(number); }
}

/* A global 'BGRA' surface of width x height, 4 bytes per element; NULL on failure. */
XpKitSurface xpkit_surface_create(int width, int height)
{
    CFMutableDictionaryRef p = CFDictionaryCreateMutable(NULL, 0, &kCFTypeDictionaryKeyCallBacks,
                                                         &kCFTypeDictionaryValueCallBacks);
    IOSurfaceRef surface;
    if (!p) return NULL;
    xpkit_surface_dict_int(p, kIOSurfaceWidth, width);
    xpkit_surface_dict_int(p, kIOSurfaceHeight, height);
    xpkit_surface_dict_int(p, kIOSurfaceBytesPerElement, 4);
    xpkit_surface_dict_int(p, kIOSurfacePixelFormat, (int)'BGRA');
    CFDictionarySetValue(p, kIOSurfaceIsGlobal, kCFBooleanTrue);
    surface = IOSurfaceCreate(p);
    CFRelease(p);
    return (XpKitSurface)surface;
}

/* Bind a surface to a rectangle texture name in the current CGL context (no pixels are copied).
 * Leaves the previous rectangle-texture binding in place. 1 when GL accepted it. */
int xpkit_surface_attach(unsigned texture, XpKitSurface handle, int width, int height)
{
    IOSurfaceRef surface = (IOSurfaceRef)handle;
    GLint previous = 0;
    CGLContextObj context = CGLGetCurrentContext();
    CGLError error;
    if (!context || !surface || !texture) return 0;
    glGetIntegerv(GL_TEXTURE_BINDING_RECTANGLE_ARB, &previous);
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture);
    error = CGLTexImageIOSurface2D(context, GL_TEXTURE_RECTANGLE_ARB, GL_RGBA8, (GLsizei)width, (GLsizei)height,
                                   GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, surface, 0);
    if (error == kCGLNoError) {
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, (GLuint)previous);
    return error == kCGLNoError;
}

/* Draw a width x height surface texture over (x, y, w, h) with the texture's first row (the page
 * top) at y + h: the orientation of the 2D-texture quad of the upload path. The caller has set the
 * graphics state with GL_TEXTURE_2D disabled on unit 0; the rectangle target is disabled again and
 * its previous binding restored afterwards. */
void xpkit_surface_quad(unsigned texture, int width, int height, float x, float y, float w, float h)
{
    GLint previous = 0;
    glGetIntegerv(GL_TEXTURE_BINDING_RECTANGLE_ARB, &previous);
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture);
    glEnable(GL_TEXTURE_RECTANGLE_ARB);
    glColor4f(1, 1, 1, 1);
    glBegin(GL_QUADS);
    glTexCoord2f(0, (float)height); glVertex2f(x, y);
    glTexCoord2f((float)width, (float)height); glVertex2f(x + w, y);
    glTexCoord2f((float)width, 0); glVertex2f(x + w, y + h);
    glTexCoord2f(0, 0); glVertex2f(x, y + h);
    glEnd();
    glDisable(GL_TEXTURE_RECTANGLE_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, (GLuint)previous);
}

static int xpkit_surface_has_extension(const char *name)
{
    const char *all = (const char *)glGetString(GL_EXTENSIONS);
    size_t length = strlen(name);
    for (const char *p = all; p && (p = strstr(p, name)) != NULL; p += length)
        if ((p == all || p[-1] == ' ') && (p[length] == ' ' || p[length] == 0)) return 1;
    return 0;
}

/* One-time capability probe in the CURRENT context: a small global surface filled by the CPU with a
 * known colour is bound with CGLTexImageIOSurface2D, drawn into a private framebuffer object and read
 * back. Every piece of GL state it touches is saved and restored (attribute stacks, matrices by value,
 * both framebuffer bindings, pixel-pack buffer), and X-Plane's cached state is never changed. Returns
 * 1 on an exact match; otherwise 0 with the reason. force_failure is the test hook (forced fallback). */
int xpkit_surface_probe(int force_failure, const unsigned names[2], char *reason, size_t reason_size)
{
    enum { S = 16 };
    static const unsigned char bgra[4] = {48, 160, 224, 255};   /* B, G, R, A in memory */
    unsigned char rgba[S * S * 4];
    GLint draw_fbo = 0, read_fbo = 0, pack_buffer = 0, matrix_mode = GL_MODELVIEW;
    GLfloat projection[16], modelview[16], texture_matrix[16];
    GLuint textures[2] = {names ? names[0] : 0, names ? names[1] : 0}, fbo = 0;
    GLenum status = 0, gl_error = GL_NO_ERROR;
    int blit, attached = 0, ok = 0, mismatches = 0;
    IOSurfaceRef surface;
    snprintf(reason, reason_size, "ok");
    if (force_failure) { snprintf(reason, reason_size, "disabled by the test hook"); return 0; }
    if (!CGLGetCurrentContext()) { snprintf(reason, reason_size, "no current CGL context"); return 0; }
    if (!textures[0] || !textures[1]) { snprintf(reason, reason_size, "no texture names"); return 0; }
    if (!xpkit_surface_has_extension("GL_EXT_framebuffer_object") ||
        !(xpkit_surface_has_extension("GL_ARB_texture_rectangle") || xpkit_surface_has_extension("GL_EXT_texture_rectangle"))) {
        snprintf(reason, reason_size, "missing GL_EXT_framebuffer_object or rectangle textures");
        return 0;
    }
    blit = xpkit_surface_has_extension("GL_EXT_framebuffer_blit");
    surface = (IOSurfaceRef)xpkit_surface_create(S, S);
    if (!surface) { snprintf(reason, reason_size, "IOSurfaceCreate failed"); return 0; }
    if (IOSurfaceLock(surface, 0, NULL) != kIOReturnSuccess) {
        CFRelease(surface); snprintf(reason, reason_size, "IOSurfaceLock failed"); return 0;
    }
    {
        unsigned char *base = IOSurfaceGetBaseAddress(surface);
        size_t row = IOSurfaceGetBytesPerRow(surface);
        for (int y = 0; y < S; ++y)
            for (int x = 0; x < S; ++x) memcpy(base + (size_t)y * row + (size_t)x * 4, bgra, 4);
    }
    IOSurfaceUnlock(surface, 0, NULL);

    for (int n = 0; n < 64 && glGetError() != GL_NO_ERROR; ++n) {}   /* errors raised before the probe */
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushClientAttrib(GL_CLIENT_PIXEL_STORE_BIT);
    glGetIntegerv(GL_MATRIX_MODE, &matrix_mode);
    glGetFloatv(GL_PROJECTION_MATRIX, projection);
    glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
    glGetFloatv(GL_TEXTURE_MATRIX, texture_matrix);
    if (blit) {
        glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING_EXT, &draw_fbo);
        glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING_EXT, &read_fbo);
    } else glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, &draw_fbo);
    glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &pack_buffer);

    attached = xpkit_surface_attach(textures[0], (XpKitSurface)surface, S, S);
    if (attached) {
        glBindTexture(GL_TEXTURE_2D, textures[1]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, S, S, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
        glGenFramebuffersEXT(1, &fbo);
        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, textures[1], 0);
        status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
        if (status == GL_FRAMEBUFFER_COMPLETE_EXT) {
            glViewport(0, 0, S, S);
            glDisable(GL_SCISSOR_TEST); glDisable(GL_BLEND); glDisable(GL_DEPTH_TEST); glDisable(GL_ALPHA_TEST);
            glDisable(GL_STENCIL_TEST); glDisable(GL_LIGHTING); glDisable(GL_FOG); glDisable(GL_CULL_FACE);
            glDisable(GL_TEXTURE_2D); glDisable(GL_TEXTURE_1D);
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            glMatrixMode(GL_TEXTURE); glLoadIdentity();
            glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, S, 0, S, -1, 1);
            glMatrixMode(GL_MODELVIEW); glLoadIdentity();
            glClearColor(0, 0, 0, 0);
            glClear(GL_COLOR_BUFFER_BIT);
            xpkit_surface_quad(textures[0], S, S, 0, 0, S, S);
            glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glPixelStorei(GL_PACK_ROW_LENGTH, 0);
            glPixelStorei(GL_PACK_SKIP_ROWS, 0);
            glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
            memset(rgba, 0, sizeof(rgba));
            glReadPixels(0, 0, S, S, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
        }
    }
    gl_error = glGetError();

    if (blit) {
        glBindFramebufferEXT(GL_DRAW_FRAMEBUFFER_EXT, (GLuint)draw_fbo);
        glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, (GLuint)read_fbo);
    } else glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, (GLuint)draw_fbo);
    if (fbo) glDeleteFramebuffersEXT(1, &fbo);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, (GLuint)pack_buffer);
    glMatrixMode(GL_TEXTURE); glLoadMatrixf(texture_matrix);
    glMatrixMode(GL_PROJECTION); glLoadMatrixf(projection);
    glMatrixMode(GL_MODELVIEW); glLoadMatrixf(modelview);
    glMatrixMode((GLenum)matrix_mode);
    glPopClientAttrib();
    glPopAttrib();
    CFRelease(surface);

    if (!attached) { snprintf(reason, reason_size, "CGLTexImageIOSurface2D failed"); return 0; }
    if (status != GL_FRAMEBUFFER_COMPLETE_EXT) {
        snprintf(reason, reason_size, "probe framebuffer incomplete (0x%04x)", (unsigned)status); return 0;
    }
    if (gl_error != GL_NO_ERROR) { snprintf(reason, reason_size, "GL error 0x%04x", (unsigned)gl_error); return 0; }
    for (int i = 0; i < S * S; ++i) {
        const unsigned char *p = rgba + i * 4;
        if (abs(p[0] - bgra[2]) > 2 || abs(p[1] - bgra[1]) > 2 || abs(p[2] - bgra[0]) > 2) ++mismatches;
    }
    ok = mismatches == 0;
    if (!ok) snprintf(reason, reason_size, "read back %d/%d wrong pixels (first %u,%u,%u)", mismatches, S * S,
                      rgba[0], rgba[1], rgba[2]);
    return ok;
}

unsigned xpkit_surface_id(XpKitSurface surface) { return surface ? (unsigned)IOSurfaceGetID((IOSurfaceRef)surface) : 0; }
void xpkit_surface_release(XpKitSurface surface) { if (surface) CFRelease((IOSurfaceRef)surface); }

#pragma clang diagnostic pop
