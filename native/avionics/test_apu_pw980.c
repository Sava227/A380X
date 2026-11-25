/* SPDX-License-Identifier: GPL-3.0-or-later
 * FBW PW980 APU speeds (a380x_apu_pw980.h): start, running, cooldown after MASTER off, spool-down.
 *   clang -Wall -Wextra test_apu_pw980.c -lm -o test_apu_pw980 && ./test_apu_pw980
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "a380x_apu_pw980.h"

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

/* Runs until `until` returns true or `limit` s; returns the elapsed time. */
static double run(A380XPw980 *a, int master, int motor, double limit, int (*until)(const A380XPw980 *))
{
    double t = 0.0;
    while (t < limit && !(until && until(a))) {
        int motor_now = motor && a->n < A380X_PW980_STARTER_UNTIL_N;
        a380x_pw980_update(a, a380x_pw980_signal(a, 1, master, motor_now, 0), master, 0, 0.05);
        t += 0.05;
    }
    return t;
}
static int at_95(const A380XPw980 *a) { return a->n >= 95.0; }
static int running(const A380XPw980 *a) { return a->state == A380X_PW980_RUNNING; }
static int stopping(const A380XPw980 *a) { return a->state == A380X_PW980_STOPPING; }
static int shut(const A380XPw980 *a) { return a->state == A380X_PW980_SHUTDOWN; }

int main(void)
{
    A380XPw980 a = {0};
    double t = run(&a, 1, 1, 60.0, at_95);
    CHECK(fabs(t - 40.35) < 0.2);                     /* FBW: N 95 % at 40.3 s */
    t += run(&a, 1, 1, 60.0, running);
    CHECK(fabs(t - 45.0) < 0.2 && a.n == 100.0);      /* RUNNING at 44.9 s */
    CHECK(run(&a, 0, 0, 120.0, stopping) > 59.9);     /* 60 s cooldown after MASTER off */
    t = run(&a, 0, 0, 200.0, shut);
    CHECK(t > 30.0 && t < 90.0 && a.n == 0.0);
    /* A start stopped before the 55 % starter cut-out spools down (FBW Starting + no signal). */
    memset(&a, 0, sizeof(a));
    run(&a, 1, 1, 20.0, NULL);
    CHECK(a.state == A380X_PW980_STARTING && a.n > 0.0);
    a380x_pw980_update(&a, a380x_pw980_signal(&a, 1, 1, 0, 0), 1, 0, 0.05);
    CHECK(a.state == A380X_PW980_STOPPING);
    /* Quick mode (expedited preset) runs at once. */
    memset(&a, 0, sizeof(a));
    a380x_pw980_update(&a, A380X_PW980_START, 1, 1, 0.05);
    a380x_pw980_update(&a, A380X_PW980_START, 1, 1, 0.05);
    CHECK(a.state == A380X_PW980_RUNNING);
    printf(failures ? "test_apu_pw980: %d FAILED\n" : "test_apu_pw980: all passed%.0d\n", failures);
    return failures ? 1 : 0;
}
