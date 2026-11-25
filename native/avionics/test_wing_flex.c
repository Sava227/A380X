/* SPDX-License-Identifier: GPL-3.0-or-later
 * Wing flex (a380x_wing_flex.h): FBW's animation-mapper cases, sag on the ground, bend up in flight, more
 * with g, less on the down-going aileron side, outputs inside the FBW clip ranges. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "a380x_wing_flex.h"

/* FBW A380_WINGS.xml clip = angle x gain + offset over 0..100: the angle range each clip covers. */
static const double gain[4] = {23.25, 16.125, 29.41, 29.41}, offset[4] = {55.35, 67., 31.9, 50.};

static void run(A380XWingFlex *f, const A380XWingFlexIn *in, double seconds)
{
    for (double t = 0; t < seconds; t += 1. / 60.) a380x_wing_flex_update(f, in, 1. / 60.);
}

static double tip(const A380XWingFlex *f, int w) { return f->angle_deg[w][0] + f->angle_deg[w][1] + f->angle_deg[w][2] + f->angle_deg[w][3]; }

int main(void)
{
    const double x[5] = {0, 1, 2, 3, 4};
    float a[4];
    double h1[5] = {0, 0, 0, 0, 1}, h2[5] = {0, 1, 1, 1, 1};
    A380XWingFlex f;
    A380XWingFlexIn in;
    a380x_wf_angles(h1, x, a);                     /* FBW: last node 1 up -> 45 deg on the last link only */
    assert(fabsf(a[3] - 45.f) < 1.f && fabsf(a[0]) < 1e-3f && fabsf(a[2]) < 1e-3f);
    a380x_wf_angles(h2, x, a);                     /* FBW: first node 1 up -> 45 deg on the first link only */
    assert(fabsf(a[0] - 45.f) < 1.f && fabsf(a[1]) < 1e-3f && fabsf(a[3]) < 1e-3f);

    memset(&in, 0, sizeof(in));
    in.mass_kg = 400000.; in.wow_kg = 400000.;   /* parked, full fuel in the wing */
    for (int w = 0; w < 2; ++w) { in.fuel_kg[w][1] = 52000.; in.fuel_kg[w][2] = 48000.; in.fuel_kg[w][3] = 7200.; }
    memset(&f, 0, sizeof(f));
    run(&f, &in, 10.);
    assert(fabs(f.lift / 9.8) < 1000.);            /* FBW steady_on_ground_full_fuel */
    double parked = tip(&f, 0);
    assert(parked < 0. && fabs(tip(&f, 0) - tip(&f, 1)) < 1e-6);

    in.wow_kg = 0.;                                /* 1 g flight */
    run(&f, &in, 10.);
    assert(fabs(f.lift / 9.8 - 400000.) < 40000.); /* FBW in_straight_flight_has_plane_lift_equal_to_weight */
    double cruise = tip(&f, 0);
    assert(cruise > parked && cruise > 0.);
    for (int w = 0; w < 2; ++w)
        for (int i = 0; i < 4; ++i) {
            double frame = f.angle_deg[w][i] * gain[i] + offset[i];
            assert(isfinite(frame) && frame > 0. && frame < 100.);
        }

    in.accel_no_grav = 1.5 * 9.8;                 /* 2.5 g pull */
    run(&f, &in, 10.);
    assert(tip(&f, 0) > cruise);

    in.accel_no_grav = 0.; in.roll = 1.;           /* right roll: right aileron up, right wing unloaded */
    run(&f, &in, 10.);
    assert(tip(&f, 1) < tip(&f, 0));
    A380XWingFlex before = f;
    in.accel_no_grav = NAN;                        /* bad input: state untouched, outputs stay finite */
    a380x_wing_flex_update(&f, &in, 1. / 60.);
    assert(!memcmp(&before, &f, sizeof(f)) && isfinite(tip(&f, 0)));
    printf("WING FLEX OK parked %.2f cruise %.2f deg (sum of 4 links)\n", parked, cruise);
    return 0;
}
