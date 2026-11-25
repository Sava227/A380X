/* SPDX-License-Identifier: GPL-3.0-or-later
 * System pressures of the A380 hydraulic model (a380x_hyd.inc) for the port's consumers (flaps, FCU LAND
 * capability, brakes). When the model is not running (registration failed, or a unit test that includes one
 * module on its own), available is 0 and each consumer keeps its X-Plane native pressure reading.
 * Header-only, XPLM-free; one instance per translation unit (unity build of a380x_avionics.c).
 */
#ifndef A380X_HYD_VIEW_H
#define A380X_HYD_VIEW_H
typedef struct {
    int available;          /* model running this frame */
    double psi[2];          /* GREEN, YELLOW system pressure, psi (nominal 5000) */
    unsigned char sys_sw[2];/* system pressure switch (3700 / 2900 psi) */
} A380XHydView;
static A380XHydView a380x_hyd_view;
#endif
