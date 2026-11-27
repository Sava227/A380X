/* SPDX-License-Identifier: GPL-3.0-or-later
 * Engine pylon wobble (a380x_eng_wobble.h): still at steady g, a bump nods the intake down and rings at the
 * pylon mode, dies out, steady loads bleed out, outputs stay bounded, bad input leaves the state alone. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "a380x_eng_wobble.h"

static A380XEngWobble w;
static double vert[4], lat[4];

static void step(double a_vert, double a_lat)
{
    for (int i = 0; i < 4; ++i) { vert[i] = a_vert; lat[i] = a_lat; }
    a380x_eng_wobble_update(&w, vert, lat, 1. / 60.);
}

int main(void)
{
    memset(&w, 0, sizeof(w));
    for (int i = 0; i < 600; ++i) step(0., 0.);
    assert(w.pitch_deg[0] == 0.f && w.yaw_deg[3] == 0.f);          /* steady flight / parked: centred */

    /* 0.4 g, 0.15 s bump: first swing intake down, peak a few tenths of a degree, rings near 2.5 Hz */
    double peak = 0., prev = 0., cross[16];
    int n = 0;
    for (int i = 0; i < 240; ++i) {
        double t = i / 60.;
        step(t < 0.15 ? 0.4 * 9.8 : 0., 0.);
        double p = w.pitch_deg[1];
        if (fabs(p) > fabs(peak)) peak = p;
        if (i > 0 && (p > 0.) != (prev > 0.) && n < 16) cross[n++] = t;
        prev = p;
    }
    assert(peak < -0.15 && peak > -0.6);
    assert(n >= 6);
    double hz = (n - 2) / (2. * (cross[n - 1] - cross[1]));
    assert(fabs(hz - a380x_ew_hz_pitch) < 0.3);
    assert(fabs(w.pitch_deg[1]) < 0.05);                            /* rung out after 4 s */

    for (int i = 0; i < 600; ++i) step(0.5 * 9.8, 0.3 * 9.8);         /* sustained turn: bleeds back to centre */
    assert(fabs(w.pitch_deg[0]) < 0.02 && fabs(w.yaw_deg[0]) < 0.02);

    for (int i = 0; i < 60; ++i) step((i & 1 ? 1 : -1) * 50. * 9.8, 0.); /* violent shaking stays bounded */
    for (int i = 0; i < 4; ++i) assert(fabs(w.pitch_deg[i]) <= a380x_ew_max_deg);

    A380XEngWobble before = w;
    vert[2] = NAN;
    a380x_eng_wobble_update(&w, vert, lat, 1. / 60.);
    assert(!memcmp(&before, &w, sizeof(w)));
    printf("ENG WOBBLE OK bump peak %.2f deg, rings at %.2f Hz\n", peak, hz);
    return 0;
}
