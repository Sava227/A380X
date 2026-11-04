/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Shared helpers for the screen-only cockpit devices (PFD, ND, ECAM SD, ISIS).
 *
 * a380x_black_bezel: X-Plane draws a device's screen as a light-emitting layer over its bezel,
 * both in the 3D cockpit and in the pop-up. Without a bezel callback the bezel stays empty, so
 * every black screen pixel is transparent: the pop-up showed the cockpit through the ND/PFD, and
 * in the 3D cockpit the 180x234 / 320x416 panel-atlas copies 3 mm behind showed through (the
 * "blurry ND", the slightly soft PFD). The MFD always painted its bezel; the others now paint it
 * black, the colour of an unlit LCD.
 *
 * Pop-up window geometry for the 768x1024 cockpit devices (PFD, ND), like the MFD's
 * mfda_ensure_popup_geometry: without it X-Plane opened them stretched over the whole screen.
 * Keeps a pilot-sized, on-screen window and only restores the 3:4 shape; otherwise 80 % of the
 * screen height, captain on the left and first officer on the right.
 */
#ifndef A380X_AVIONICS_POPUP_H
#define A380X_AVIONICS_POPUP_H

#include "XPLMDisplay.h"
#include "XPLMGraphics.h"

/* Bezel callback: solid black over the whole bezel (panel coordinates, unknown GL state). */
static inline void a380x_black_bezel(float ambient_r, float ambient_g, float ambient_b, void *refcon)
{
    (void)ambient_r; (void)ambient_g; (void)ambient_b; (void)refcon;
    XPLMSetGraphicsState(0, 0, 0, 0, 0, 0, 0);
    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
    glBegin(GL_QUADS);
    glVertex2f(-1.0f, -1.0f); glVertex2f(32768.0f, -1.0f);
    glVertex2f(32768.0f, 32768.0f); glVertex2f(-1.0f, 32768.0f);
    glEnd();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
}

typedef void (*A380XPopupGetGeom)(XPLMAvionicsID, int *, int *, int *, int *);
typedef void (*A380XPopupSetGeom)(XPLMAvionicsID, int, int, int, int);

static inline void a380x_fit_popup(XPLMAvionicsID id, int side, A380XPopupGetGeom get, A380XPopupSetGeom set)
{
    int l = 0, t = 0, r = 0, b = 0, sl, st, sr, sb, max_h, h, w, left, top;
    if (!id || !get || !set) return;
    get(id, &l, &t, &r, &b);
    XPLMGetScreenBoundsGlobal(&sl, &st, &sr, &sb);
    max_h = (int)((float)(st - sb) * 0.8f);
    h = t - b;
    if (h < 200 || h > max_h) h = max_h;
    w = h * 768 / 1024;
    left = (r - l >= 100 && l >= sl && l + w <= sr) ? l : (side ? sr - 40 - w : sl + 40);
    top = (r - l >= 100 && t <= st && t - h >= sb) ? t : st - 60;
    set(id, left, top, left + w, top - h);
}

#endif
