/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef A380X_EFB_H
#define A380X_EFB_H

#include <stddef.h>

int a380x_efb_init(void);
void a380x_efb_shutdown(void);
void a380x_efb_update(float dt_seconds);
void a380x_efb_draw(void);
void a380x_efb_draw_window(int left, int bottom, int right, int top);
void a380x_efb_click(float atlas_x, float atlas_y);

/* Web EFB boundary (docs/EFB_DEVELOPMENT.md). The xp-kit adapter (a380x_xpkit_efb.inc, other
 * translation unit) registers how a tablet's web texture is drawn and whether it is live; the EFB
 * owns the cockpit devices, the touch events, the JSON frame and the validation of page actions. */
typedef struct {
    int (*live)(int tablet);                              /* fresh WebKit frame uploaded for this tablet */
    int (*draw)(int tablet, float width, float height);   /* draw it into the bound device framebuffer */
} A380XEfbWebHost;
void a380x_efb_set_web_host(const A380XEfbWebHost *host); /* NULL: no web renderer (native fallback) */
long a380x_efb_frame_json(int tablet, char *out, size_t cap); /* -1 on overflow; never NaN/Inf */
void a380x_efb_web_action(int tablet, const char *text);  /* "<id> <verb> [args]" from the page */
unsigned a380x_efb_change_serial(void);                   /* bumps on every touch, action or state change */
void a380x_efb_needs_drawing(int tablet);                 /* a new web frame for this tablet arrived */
/* GPWS / TAWS voice chosen on the EFB (Settings > Sounds): 0 = new A380, 1 = old A380. Saved in
 * X-Plane's preferences folder (A380X-XP12_efb.prf), so it survives restarts. */
int a380x_efb_gpws_voice(void);
void a380x_efb_set_gpws_voice(int old_voice);
int a380x_efb_atlas_needed(void);                         /* legacy panel-atlas tablet still needed */

#endif
