/* SPDX-License-Identifier: GPL-3.0-or-later
 * Engine fan rotation: N1-driven spin, windmilling and coast-down (a380x_eng_fan.h). */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "a380x_eng_fan.h"

static void run(A380XFan *f, float n1, float air, float seconds)
{
    for (float t = 0; t < seconds - 1e-6f; t += 0.02f) {
        a380x_fan_update(f, n1, air, 0.02f);
        assert(f->angle_deg >= 0.0f && f->angle_deg < 360.0f);
    }
}

int main(void)
{
    A380XFan f = {0};
    run(&f, 0, 1.5f, 30);                      /* cold, light breeze below breakaway: still */
    assert(f.rpm == 0.0f && f.angle_deg == 0.0f);
    run(&f, 0, 10, 30);                        /* 20 kt headwind: windmills forwards near 45 rpm */
    assert(f.rpm > 40 && f.rpm < 45.1f);
    run(&f, 0, -10, 30);                       /* tailwind: backwards */
    assert(f.rpm < -40);
    run(&f, 20, -10, 0.02f);                   /* the engine drives it: N1 x 2900 rpm at once */
    assert(fabsf(f.rpm - 580) < 0.01f);
    float before = f.angle_deg;
    a380x_fan_update(&f, 20, 0, 0.01f);        /* 580 rpm = 3480 deg/s: 34.8 deg in 10 ms */
    assert(fabsf(fmodf(f.angle_deg - before + 360.0f, 360.0f) - 34.8f) < 0.05f);
    run(&f, 0, 0, 2);                          /* shut down: coasts, not an instant stop */
    assert(f.rpm > 100 && f.rpm < 580);
    run(&f, 0, 0, 60);
    assert(f.rpm < 1);
    a380x_fan_update(&f, NAN, NAN, 0.02f);     /* bad inputs: no NaN reaches the OBJ */
    assert(isfinite(f.rpm) && isfinite(f.angle_deg));
    puts("ENG FAN OK");
    return 0;
}
