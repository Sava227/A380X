/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 APU (PW980) speeds N and N2: XPLM-free translation of FlyByWire
 * fbw-common/src/wasm/systems/systems/src/apu/pw980.rs (Starting / Running / Stopping turbine, pinned 2baa2b35)
 * and the ECB turbine signal of apu/electronic_control_box.rs (Pw980Constants: 60 s cooldown after MASTER
 * off, starter kept until N 55 %). X-Plane's own APU N1 is read-only and spools on X-Plane's schedule, so the
 * native ECB (a380x_elec.c) and the displays use this N instead (a380x/xp12/apu/n_percent, n2_percent).
 * FBW times from the start signal: N 95 % at 40.3 s, 100 % (RUNNING) at 44.9 s. EGT stays X-Plane's.
 */
#ifndef A380X_APU_PW980_H
#define A380X_APU_PW980_H
#include <math.h>

enum { A380X_PW980_SHUTDOWN, A380X_PW980_STARTING, A380X_PW980_RUNNING, A380X_PW980_STOPPING };
enum { A380X_PW980_NONE, A380X_PW980_START, A380X_PW980_STOP };

#define A380X_PW980_COOLDOWN_S 60.0
#define A380X_PW980_STARTER_UNTIL_N 55.0

typedef struct {
    int state;
    double since, n, n2, n_factor, n2_factor, master_off_s;
} A380XPw980;

static inline double a380x_pw980_poly(const double *c, int count, double x)
{
    double y = 0.0;
    for (int i = count - 1; i >= 0; --i) y = y * x + c[i];
    return y;
}
static inline double a380x_pw980_clamp(double x, double hi) { return x < 0.0 ? 0.0 : x > hi ? hi : x; }

static inline void a380x_pw980_starting(A380XPw980 *a)
{
    static const double n2_t[14] = {0.102461912951123, -4.213276063673831, 4.904714363158679, -1.580490532662764,
        0.2609079965518872, -0.0245239023843422, 0.001333286573520723, -0.0000368228114602448,
        0.00000000978478072071, 0.00000003595869237689, -0.00000000127836115070, 0.00000000002201303409,
        -0.00000000000019674992, 0.00000000000000073146};
    static const double n_n2[14] = {-1.980342480028972, 3.042566978305871, -1.540881433592518, 0.3718632222716785,
        -0.05012708901533932, 0.004113538145844002, -0.000217068158239272, 0.0000076482322527515,
        -0.00000018368358856566, 0.00000000301784702552, -0.00000000003338059452, 0.00000000000023768495,
        -0.00000000000000098402, 0.00000000000000000180};
    static const double n_t[12] = {20346.47871003967, -10589.258178152319, 2431.729387231063, -325.5237277312483,
        28.2488372642810, -1.669888674851867, 0.06866582085373031, -0.00196551590823214, 0.000038410143909137,
        -0.00000048843308292031, 0.00000000364059072727, -0.00000000001206089876};
    double t = fmin(a->since, 45.0);
    if (t <= 0.0) { a->n = a->n2 = 0.0; return; }
    a->n2 = a380x_pw980_clamp(a380x_pw980_poly(n2_t, 14, t), 100.0);
    a->n = a380x_pw980_clamp(a->n2 < 75.0 ? a380x_pw980_poly(n_n2, 14, a->n2) : a380x_pw980_poly(n_t, 12, t), 100.0);
}

static inline void a380x_pw980_stopping(A380XPw980 *a)
{
    static const double n_t[13] = {101.2793953502211, -7.046603616716463, 2.720645871014408, -0.5598348705144645,
        0.05333946247710394, -0.002912738145135647, 0.0001009025994427405, -0.00000232434829734961,
        0.00000003616620771100, -0.00000000037630807292, 0.00000000000251143967, -0.00000000000000972688,
        0.00000000000000001663};
    static const double n2_early[4] = {84.98989898989898, -1.026936026936026, 0.03282828282828282,
        -0.005892255892255892};
    static const double n2_late[13] = {86.64042983954318, -8.396890071756419, 4.265566873959583, -0.8713377975625458,
        0.08440922199990456, -0.004751685242570211, 0.0001707504726515346, -0.00000409191213933807,
        0.00000006632366421256, -0.00000000071927363877, 0.00000000000500426125, -0.00000000000002020512,
        0.00000000000000003601};
    double t = fmin(a->since, 86.1);
    a->n = a380x_pw980_clamp(a380x_pw980_poly(n_t, 13, t), 100.0) * a->n_factor;
    a->n2 = a380x_pw980_clamp(t < 8.0 ? a380x_pw980_poly(n2_early, 4, t) : a380x_pw980_poly(n2_late, 13, t), 85.0)
            * a->n2_factor;
}

static inline void a380x_pw980_begin_stop(A380XPw980 *a)
{
    if (a->state != A380X_PW980_STARTING && a->state != A380X_PW980_RUNNING) return;
    a->n_factor = a->n / 100.0;
    a->n2_factor = a->n2 / 85.0;
    a->since = 0.0;
    a->state = A380X_PW980_STOPPING;
}

/* ECB turbine signal (electronic_control_box.rs ControllerSignal<TurbineSignal>). */
static inline int a380x_pw980_signal(const A380XPw980 *a, int ecb_powered, int master, int start_motor, int shutdown)
{
    if (!ecb_powered) return A380X_PW980_NONE;
    if (shutdown || (!master && a->state != A380X_PW980_STARTING && a->master_off_s > A380X_PW980_COOLDOWN_S))
        return A380X_PW980_STOP;
    if (start_motor || a->n >= A380X_PW980_STARTER_UNTIL_N) return A380X_PW980_START;
    return A380X_PW980_NONE;
}

/* One step. quick: FBW aircraft-preset quick mode (N jumps to 100 % / 0 %). */
static inline void a380x_pw980_update(A380XPw980 *a, int signal, int master, int quick, double dt)
{
    if (!(dt > 0.0)) dt = 0.0;
    a->master_off_s = master ? 0.0 : a->master_off_s + dt;
    switch (a->state) {
    case A380X_PW980_SHUTDOWN:
        a->n = a->n2 = 0.0;
        if (signal == A380X_PW980_START) { a->state = A380X_PW980_STARTING; a->since = 0.0; }
        break;
    case A380X_PW980_STARTING:
        a->since += dt;
        a380x_pw980_starting(a);
        if (quick) a->n = 100.0;
        if (signal != A380X_PW980_START) a380x_pw980_begin_stop(a);
        else if (fabs(a->n - 100.0) < 1e-9) a->state = A380X_PW980_RUNNING;
        break;
    case A380X_PW980_RUNNING:
        a->n = 100.0;
        a->n2 = 85.0;   /* FBW: +2 % over 8 s with APU bleed in use (not modelled) */
        if (signal != A380X_PW980_START) a380x_pw980_begin_stop(a);
        break;
    case A380X_PW980_STOPPING:
        a->since += dt;
        a380x_pw980_stopping(a);
        if (quick) a->n = 0.0;
        if (a->n < 0.5) { a->state = A380X_PW980_SHUTDOWN; a->n = a->n2 = 0.0; }
        break;
    }
}

/* Loaded with X-Plane's APU already running (preset / situation): adopt it as RUNNING. */
static inline void a380x_pw980_set_running(A380XPw980 *a)
{
    a->state = A380X_PW980_RUNNING;
    a->n = 100.0;
    a->n2 = 85.0;
    a->master_off_s = 0.0;
}
#endif
