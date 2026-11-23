/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Zero-copy display frames (xp-kit protocol v9, macOS only): the plugin creates global IOSurfaces,
 * the out-of-process WebKit helper writes the captured pages into them and the plugin's OpenGL
 * context samples them as rectangle textures (CGLTexImageIOSurface2D), so no frame is uploaded
 * with glTexSubImage2D on X-Plane's main thread.
 *
 * Plain CGL / OpenGL / IOSurface, no XPLM, implemented in a380x_xpkit_surface.c (its own translation
 * unit). Used by a380x_xpkit_adapter.inc and by test_xpkit_zero_copy.c, which runs the same probe and
 * binding code in its own CGL context.
 *
 * Global IOSurfaces (kIOSurfaceIsGlobal, found by ID from another process) and CGL are deprecated
 * but present on every supported macOS (11+); the deprecation warnings are silenced here only.
 * Before zero-copy is trusted, xpkit_surface_probe() proves in the current context that a surface
 * written by the CPU is sampled back by GL with the expected colour; any failure keeps the texture
 * upload path.
 */
#ifndef A380X_XPKIT_SURFACE_H
#define A380X_XPKIT_SURFACE_H
#if defined(__APPLE__)
#include <stddef.h>

/* An IOSurfaceRef, opaque here (the avionics unity build must not see the Apple headers). */
typedef void *XpKitSurface;
/* A global 'BGRA' surface of width x height, 4 bytes per element; NULL on failure. */
XpKitSurface xpkit_surface_create(int width, int height);
unsigned xpkit_surface_id(XpKitSurface surface);          /* IOSurfaceGetID */
void xpkit_surface_release(XpKitSurface surface);         /* CFRelease */
/* Bind a surface to a rectangle texture name in the current CGL context (no pixels are copied).
 * Leaves the previous rectangle-texture binding in place. 1 when GL accepted it. */
int xpkit_surface_attach(unsigned texture, XpKitSurface surface, int width, int height);
/* Draw a width x height surface texture over (x, y, w, h) with the texture's first row (the page
 * top) at y + h: the orientation of the 2D-texture quad of the upload path. The caller has set the
 * graphics state with GL_TEXTURE_2D disabled on unit 0; the rectangle target is disabled again and
 * its previous binding restored afterwards. */
void xpkit_surface_quad(unsigned texture, int width, int height, float x, float y, float w, float h);
/* One-time capability probe in the CURRENT context: a small global surface filled by the CPU with a
 * known colour is bound with CGLTexImageIOSurface2D, drawn into a private framebuffer object and read
 * back. Every piece of GL state it touches is saved and restored (attribute stacks, matrices by value,
 * both framebuffer bindings, pixel-pack buffer), and X-Plane's cached state is never changed. Returns
 * 1 on an exact match; otherwise 0 with the reason. force_failure is the test hook (forced fallback).
 * names: two unused texture names from the caller (the plugin takes them from XPLMGenerateTextureNumbers),
 * re-specified here; the caller deletes them afterwards. */
int xpkit_surface_probe(int force_failure, const unsigned names[2], char *reason, size_t reason_size);

#endif
#endif
