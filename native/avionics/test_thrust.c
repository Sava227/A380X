/* SPDX-License-Identifier: GPL-3.0-or-later
 * Unit tests of the XPLM-free detented lever (a380x_detent_lever.h) and thrust-lever / FADEC core
 * (a380x_thrust.h): sticky CL/FLX detents, end stops, click events, drag timeout, interlocks,
 * hardware adoption, FBW FLEX latch / rating / N1 law / low-speed rules / FLEX->CLB transition,
 * engine schedule, the 15 s instinctive-disconnect inhibit and its FMGC power-up reset.
 *
 *   clang -std=c11 -O2 -Wall -Wextra -Werror test_thrust.c -lm -o test_thrust && ./test_thrust
 */
#include "a380x_thrust.h"

#include <stdio.h>

static int failures, checks;
#define CHECK(cond, ...)                                                       \
    do {                                                                       \
        ++checks;                                                              \
        if (!(cond)) {                                                         \
            ++failures;                                                        \
            printf("FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond);           \
            printf(__VA_ARGS__);                                               \
            printf("\n");                                                      \
        }                                                                      \
    } while (0)
#define NEAR(a, b, tol) (fabs((a) - (b)) <= (tol))

static const A380XDetent lever_detents[] = {
    {0.0, 0.0, 0.0, A380X_CLICK_IN_DEG, A380X_CLICK_OUT_DEG},
    {25.0, A380X_DETENT_CAPTURE_DEG, A380X_DETENT_ESCAPE_DEG, A380X_CLICK_IN_DEG, A380X_CLICK_OUT_DEG},
    {35.0, A380X_DETENT_CAPTURE_DEG, A380X_DETENT_ESCAPE_DEG, A380X_CLICK_IN_DEG, A380X_CLICK_OUT_DEG},
    {45.0, 0.0, 0.0, A380X_CLICK_IN_DEG, A380X_CLICK_OUT_DEG},
};

/* A slow mouse drag: hand moves in small steps from `from` to `to`. */
static void sweep(A380XDetentLever *l, double from, double to, double step)
{
    double h = from;
    if (to >= from) for (; h <= to + 1e-9; h += step) a380x_detent_lever_drag(l, h);
    else for (; h >= to - 1e-9; h -= step) a380x_detent_lever_drag(l, h);
}

static void test_detent_lever_stickiness(void)
{
    A380XDetentLever l;
    unsigned clicks;
    a380x_detent_lever_init(&l, 0.0, 45.0, lever_detents, 4, 0.0);
    CHECK(a380x_detent_lever_take_clicks(&l) == 0, "initialisation never clicks");
    sweep(&l, 0.0, 23.5, 0.1);
    CHECK(NEAR(l.value, 23.5, 1e-6) && l.captured < 0, "free travel below CL: %g", l.value);
    CHECK(a380x_detent_lever_take_clicks(&l) == 0, "leaving IDLE does not click");
    a380x_detent_lever_drag(&l, 24.2);
    CHECK(l.value == 25.0 && l.captured == A380X_DETENT_CL, "captured by CL within 1 deg: %g", l.value);
    clicks = a380x_detent_lever_take_clicks(&l);
    CHECK(clicks == (1u << A380X_DETENT_CL), "one CL click: %x", clicks);
    sweep(&l, 24.3, 28.4, 0.1);
    CHECK(l.value == 25.0, "held in CL while the hand is within 3.5 deg: hand %g lever %g", l.hand, l.value);
    a380x_detent_lever_drag(&l, 28.6);
    CHECK(NEAR(l.value, 28.6, 1e-9) && l.captured < 0, "breaks free beyond 3.5 deg: %g", l.value);
    CHECK(a380x_detent_lever_take_clicks(&l) == 0, "no click while held / on escape");
    sweep(&l, 28.7, 34.0, 0.1);
    CHECK(l.value == 35.0 && l.captured == A380X_DETENT_FLX, "captured by FLX/MCT: %g", l.value);
    CHECK(a380x_detent_lever_take_clicks(&l) == (1u << A380X_DETENT_FLX), "FLX click");
    sweep(&l, 34.1, 38.4, 0.1);
    CHECK(l.value == 35.0, "held in FLX: %g", l.value);
    sweep(&l, 38.6, 46.0, 0.1);
    CHECK(l.value == 45.0, "TOGA end stop: %g", l.value);
    CHECK(a380x_detent_lever_take_clicks(&l) == (1u << A380X_DETENT_TOGA), "TOGA click");
    /* Coming back: FLX captures from above, then a slow pull is held, CL captures again. */
    sweep(&l, 45.0, 36.0, 0.1);
    CHECK(l.value == 35.0, "FLX captures from above: %g", l.value);
    CHECK(a380x_detent_lever_take_clicks(&l) == (1u << A380X_DETENT_FLX), "FLX click again (re-armed)");
    sweep(&l, 35.9, 0.0, 0.1);
    CHECK(l.value == 0.0, "IDLE end stop: %g", l.value);
    clicks = a380x_detent_lever_take_clicks(&l);
    CHECK(clicks == ((1u << A380X_DETENT_CL) | (1u << A380X_DETENT_IDLE)), "CL and IDLE clicks: %x", clicks);
}

static void test_detent_lever_fast_push_and_timeout(void)
{
    A380XDetentLever l;
    a380x_detent_lever_init(&l, 0.0, 45.0, lever_detents, 4, 0.0);
    a380x_detent_lever_drag(&l, 10.0);
    a380x_detent_lever_drag(&l, 40.0);      /* one frame across CL and FLX, beyond escape */
    CHECK(NEAR(l.value, 40.0, 1e-9), "a deliberate fast push passes the gates: %g", l.value);
    a380x_detent_lever_drag(&l, 24.0);      /* back across FLX (passed) and CL (within escape) */
    CHECK(l.value == 25.0, "first detent on the path that still holds the hand: %g", l.value);
    CHECK(a380x_detent_lever_input(&l) == 24.0, "the input dataref reports the hand while dragging");
    a380x_detent_lever_tick(&l, 0.1);
    CHECK(l.dragging, "drag continues within the timeout");
    a380x_detent_lever_tick(&l, 0.2);
    CHECK(!l.dragging && a380x_detent_lever_input(&l) == 25.0, "drag ends after 0.25 s; input = lever");
    /* A new drag starts captured in the resting detent: the first small move does not leave it. */
    a380x_detent_lever_drag(&l, 26.0);
    CHECK(l.value == 25.0 && l.captured == A380X_DETENT_CL, "resting detent is sticky: %g", l.value);
    a380x_detent_lever_drag(&l, NAN);
    CHECK(l.value == 25.0, "non-finite input ignored");
    a380x_detent_lever_drag(&l, 99.0);
    CHECK(l.value == 45.0, "clamped to the end stop");
}

static void test_detent_lever_external_and_lock(void)
{
    A380XDetentLever l;
    a380x_detent_lever_init(&l, 0.0, 45.0, lever_detents, 4, 25.0);
    CHECK(a380x_detent_lever_take_clicks(&l) == 0, "starting in CL is not an entry");
    a380x_detent_lever_set(&l, 25.6);
    CHECK(l.value == 25.6, "external positions have no stickiness");
    CHECK(a380x_detent_lever_take_clicks(&l) == 0, "within click_out: no re-arm, no click");
    a380x_detent_lever_set(&l, 30.0);
    a380x_detent_lever_set(&l, 25.3);
    CHECK(a380x_detent_lever_take_clicks(&l) == (1u << A380X_DETENT_CL), "hardware entry clicks once");
    a380x_detent_lever_set(&l, 25.0);
    a380x_detent_lever_set(&l, 25.4);
    CHECK(a380x_detent_lever_take_clicks(&l) == 0, "axis noise inside the band does not chatter");
    a380x_detent_lever_drag_locked(&l, 40.0);
    CHECK(l.value == 25.4 && l.dragging && l.hand == 40.0, "locked lever tracks the hand, does not move");
}

/* ------------------------------------------------------------------ thrust core */
static A380XThrustLimits limits(void)
{
    A380XThrustLimits l;
    memset(&l, 0, sizeof(l));
    l.valid = 1;
    l.idle = 20.0; l.clb = 80.0; l.mct = 84.0; l.toga = 88.0; l.rev = 71.5;
    l.flex_valid = 1; l.flex = 82.0;
    return l;
}
static A380XThrustInputs ground_inputs(void)
{
    A380XThrustInputs in;
    int i;
    memset(&in, 0, sizeof(in));
    in.on_ground = 1;
    for (i = 0; i < 4; ++i) { in.engine_running[i] = 1; in.n1[i] = 20.0; }
    in.ias_knots = 100.0;
    in.tat_valid = 1; in.tat_c = 15.0;
    in.fmgc_powered = 1;
    in.limits = limits();
    return in;
}
static void all_levers(A380XThrust *t, double tla)
{
    int i;
    for (i = 0; i < 4; ++i) a380x_detent_lever_set(&t->lever[i], tla);
}
static void step(A380XThrust *t, A380XThrustInputs *in, double seconds)
{
    double s;
    for (s = 0.0; s < seconds - 1e-9; s += 0.05) {
        a380x_thrust_latch_flex_temp(t, in);
        a380x_thrust_update(t, in, 0.05);
    }
}

static void test_mappings(void)
{
    A380XThrustCalibration c = a380x_thrust_default_calibration();
    CHECK(a380x_thrust_tla_to_ratio(&c, 0.0) == 0.0, "IDLE writes a true 0");
    CHECK(NEAR(a380x_thrust_tla_to_ratio(&c, 25.0), 25.0 / 45.0, 1e-12), "CL ratio");
    CHECK(NEAR(a380x_thrust_tla_to_ratio(&c, 35.0), 35.0 / 45.0, 1e-12), "FLX/MCT ratio");
    CHECK(a380x_thrust_tla_to_ratio(&c, 45.0) == 1.0, "TOGA ratio");
    CHECK(a380x_thrust_ratio_to_tla(&c, 0.0) == 0.0 && a380x_thrust_ratio_to_tla(&c, 0.025) == 0.0, "idle band");
    CHECK(a380x_thrust_ratio_to_tla(&c, 25.0 / 45.0 + 0.015) == 25.0, "CL within tolerance is exactly 25");
    CHECK(a380x_thrust_ratio_to_tla(&c, 35.0 / 45.0 - 0.015) == 35.0, "FLX within tolerance is exactly 35");
    CHECK(a380x_thrust_ratio_to_tla(&c, 0.99) == 45.0, "TOGA within tolerance");
    CHECK(NEAR(a380x_thrust_ratio_to_tla(&c, a380x_thrust_tla_to_ratio(&c, 30.0)), 30.0, 1e-9), "round trip");
    CHECK(a380x_thrust_lever_anim_percent(-20.0) == 0.0 && a380x_thrust_lever_anim_percent(25.0) == 55.0
          && a380x_thrust_lever_anim_percent(35.0) == 78.0 && a380x_thrust_lever_anim_percent(45.0) == 100.0,
          "FBW mappingTable3d");
    CHECK(a380x_thrust_reverse_tla(1.0) == -20.0 && NEAR(a380x_thrust_reverse_tla(A380X_REV_IDLE_POSITION), -6.0, 1e-12),
          "reverse lever: IDLE REV -6, MAX REV -20");
    CHECK(a380x_thrust_reverse_ratio(A380X_REV_IDLE_POSITION) == 0.0 && a380x_thrust_reverse_ratio(1.0) == 1.0,
          "X-Plane reverse handle 0 at IDLE REV, 1 at MAX REV");
}

static void test_interlocks_and_reverse(void)
{
    A380XThrust t;
    A380XXpLever x;
    a380x_thrust_init(&t, NULL);
    a380x_thrust_drag_reverse(&t, 0, 1.0);
    CHECK(t.reverse[0].value == 0.0, "engine 1 has no reverser");
    a380x_thrust_drag_lever(&t, 1, 10.0);
    a380x_detent_lever_end_drag(&t.lever[1]);
    a380x_thrust_drag_reverse(&t, 1, 0.5);
    CHECK(t.reverse[1].value == 0.0, "reverse locked with the thrust lever above IDLE");
    a380x_detent_lever_end_drag(&t.reverse[1]);
    a380x_thrust_drag_lever(&t, 1, 0.0);
    a380x_detent_lever_end_drag(&t.lever[1]);
    CHECK(a380x_thrust_take_clicks(&t) == (1u << (8 + A380X_DETENT_IDLE)), "lever 2 back at IDLE clicks");
    a380x_thrust_drag_reverse(&t, 1, 0.27);
    CHECK(t.reverse[1].value == A380X_REV_IDLE_POSITION, "IDLE REV captures: %g", t.reverse[1].value);
    a380x_thrust_drag_reverse(&t, 1, 0.38);
    CHECK(t.reverse[1].value == A380X_REV_IDLE_POSITION, "held at IDLE REV");
    a380x_thrust_drag_reverse(&t, 1, 0.7);
    a380x_thrust_drag_reverse(&t, 1, 1.0);
    CHECK(t.reverse[1].value == 1.0, "MAX REV");
    CHECK(a380x_thrust_take_clicks(&t) == ((1u << (8 + 4 + A380X_REV_DETENT_IDLE)) | (1u << (8 + 4 + A380X_REV_DETENT_MAX))),
          "IDLE REV and MAX REV clicks on lever 2");
    a380x_thrust_drag_lever(&t, 1, 20.0);
    CHECK(t.lever[1].value == 0.0, "thrust lever locked while reversed");
    x = a380x_thrust_xp_lever(&t, 1);
    CHECK(x.reverse && x.ratio == 1.0, "X-Plane: prop_mode 3, full reverse handle");
    t.reverse_inhibited = 1;
    x = a380x_thrust_xp_lever(&t, 1);
    CHECK(!x.reverse && x.ratio == 0.0, "in flight X-Plane stays in forward idle (FBW floors TLA at 0)");
}

static void test_hardware_adoption(void)
{
    A380XThrust t;
    A380XXpLever read[4];
    int i;
    a380x_thrust_init(&t, NULL);
    for (i = 0; i < 4; ++i) { read[i].reverse = 0; read[i].ratio = 25.0 / 45.0 + 0.01; }
    a380x_thrust_sync_external(&t, read);
    CHECK(t.lever[0].value == 25.0 && t.lever[3].value == 25.0, "load: levers adopted, CL snapped");
    CHECK(a380x_thrust_take_clicks(&t) == 0, "levers found at load do not click");
    CHECK(!t.dirty[0] && !t.dirty[2], "adopted values are not written back (hardware never fought)");
    for (i = 0; i < 4; ++i) read[i].ratio = 1.0;
    a380x_thrust_sync_external(&t, read);
    CHECK(t.lever[2].value == 45.0, "hardware to TOGA");
    CHECK(a380x_thrust_take_clicks(&t) != 0, "hardware detent entries click (FBW plays on position)");
    a380x_thrust_sync_external(&t, read);
    CHECK(a380x_thrust_take_clicks(&t) == 0, "unchanged hardware: nothing");
    /* X-Plane reverse toggle on all engines with the throttles forward: refused everywhere. */
    for (i = 0; i < 4; ++i) read[i].reverse = 1;
    a380x_thrust_sync_external(&t, read);
    CHECK(t.reverse[1].value == 0.0 && t.dirty[0] && t.dirty[1] && t.dirty[2] && t.dirty[3],
          "reverse refused: levers not at IDLE / outer engines");
    for (i = 0; i < 4; ++i) a380x_thrust_mark_written(&t, i, a380x_thrust_xp_lever(&t, i));
    /* Throttles to idle, then reverse toggle: inner engines reverse at IDLE REV, outer refused. */
    for (i = 0; i < 4; ++i) { read[i].reverse = 0; read[i].ratio = 0.0; }
    a380x_thrust_sync_external(&t, read);
    for (i = 0; i < 4; ++i) read[i].reverse = 1;
    a380x_thrust_sync_external(&t, read);
    CHECK(t.reverse[1].value == A380X_REV_IDLE_POSITION && t.reverse[2].value == A380X_REV_IDLE_POSITION,
          "inner engines reverse at IDLE REV");
    CHECK(t.reverse[0].value == 0.0 && t.dirty[0] && t.dirty[3] && !t.dirty[1], "outer reverse refused");
    for (i = 0; i < 4; i += 3) a380x_thrust_mark_written(&t, i, a380x_thrust_xp_lever(&t, i));
    /* The single hardware axis then drives the reverse amount; outer levers stay at IDLE. */
    read[0].reverse = read[3].reverse = 0;
    for (i = 0; i < 4; ++i) read[i].ratio = 1.0;
    a380x_thrust_sync_external(&t, read);
    CHECK(t.reverse[1].value == 1.0 && t.reverse[2].value == 1.0, "MAX REV from the axis");
    CHECK(t.lever[0].value == 0.0 && t.lever[3].value == 0.0 && t.dirty[0] && t.dirty[3],
          "outer levers held at IDLE while inner engines reverse");
    /* Reverse stowed by X-Plane: inner levers follow the forward handle again. */
    for (i = 0; i < 4; ++i) { read[i].reverse = 0; read[i].ratio = 0.0; }
    a380x_thrust_sync_external(&t, read);
    CHECK(t.reverse[1].value == 0.0 && t.lever[1].value == 0.0, "stowed");
    /* In flight reverse is never accepted. */
    t.reverse_inhibited = 1;
    for (i = 1; i <= 2; ++i) read[i].reverse = 1;
    a380x_thrust_sync_external(&t, read);
    CHECK(t.reverse[1].value == 0.0 && t.dirty[1], "in-flight reverse refused");
}

static void test_flex_latch_and_rating(void)
{
    A380XThrust t;
    A380XThrustInputs in = ground_inputs();
    a380x_thrust_init(&t, NULL);
    in.flex_temp_valid = 1; in.flex_temp_raw = 60.0;
    all_levers(&t, 35.0);
    step(&t, &in, 0.1);
    CHECK(t.flex_rating[0] && t.limit_type == A380X_LIMIT_FLX && t.limit_n1 == 82.0, "ground FLX: %d %g",
          t.limit_type, t.limit_n1);
    CHECK(t.n1_command[0] == 82.0, "FLX detent = FLEX N1: %g", t.n1_command[0]);
    CHECK(t.flex_latch[0], "latched at TLA 35 on ground");
    in.tat_c = 61.0; step(&t, &in, 0.1);
    CHECK(!t.flex_rating[0] && t.limit_type == A380X_LIMIT_TOGA && t.n1_command[0] == 84.0,
          "FLEX temp not above TAT: MCT N1 on the ground, TOGA rating (FBW): %d %g", t.limit_type, t.n1_command[0]);
    in.tat_c = 15.0; step(&t, &in, 0.1);
    /* Between detents the higher detent's rating applies. */
    all_levers(&t, 30.0); step(&t, &in, 0.1);
    CHECK(t.limit_type == A380X_LIMIT_FLX && NEAR(t.n1_command[0], 81.0, 1e-9), "ground 25..35 with FLEX: %d %g",
          t.limit_type, t.n1_command[0]);
    all_levers(&t, 40.0); step(&t, &in, 0.1);
    CHECK(t.limit_type == A380X_LIMIT_TOGA && NEAR(t.n1_command[0], 85.0, 1e-9), "35..45 FLEX->TOGA: %g",
          t.n1_command[0]);
    /* Take-off at FLX, airborne: the latch keeps FLEX. */
    all_levers(&t, 35.0); step(&t, &in, 0.1);
    in.on_ground = 0; in.ias_knots = 180.0; step(&t, &in, 0.1);
    CHECK(t.flex_rating[0] && t.limit_type == A380X_LIMIT_FLX, "airborne FLX latched");
    all_levers(&t, 31.0); step(&t, &in, 0.1);
    CHECK(t.limit_type == A380X_LIMIT_FLX && t.flex_latch[0], "leaving FLX without CL/TOGA: no effect");
    all_levers(&t, 35.0); step(&t, &in, 0.1);
    CHECK(t.limit_type == A380X_LIMIT_FLX, "back in FLX: still FLEX");
    all_levers(&t, 25.0); step(&t, &in, 0.1);
    CHECK(t.limit_type == A380X_LIMIT_CLB && !t.flex_latch[0], "CL clears the latch");
    all_levers(&t, 35.0); step(&t, &in, 0.1);
    CHECK(t.limit_type == A380X_LIMIT_MCT && t.n1_command[0] == 84.0, "no return to FLEX: MCT %d %g",
          t.limit_type, t.n1_command[0]);
    /* TOGA after a FLEX take-off also selects MCT at the FLX/MCT detent. */
    a380x_thrust_init(&t, NULL);
    in = ground_inputs(); in.flex_temp_valid = 1; in.flex_temp_raw = 60.0;
    all_levers(&t, 35.0); step(&t, &in, 0.1);
    in.on_ground = 0; step(&t, &in, 0.1);
    all_levers(&t, 45.0); step(&t, &in, 0.1);
    CHECK(t.limit_type == A380X_LIMIT_TOGA && t.n1_command[0] == 88.0, "TOGA");
    all_levers(&t, 35.0); step(&t, &in, 0.1);
    CHECK(t.limit_type == A380X_LIMIT_MCT, "TOGA cleared FLEX: MCT");
    /* In flight, CL..FLX without latch: MCT rating; CL and below: CLB. */
    all_levers(&t, 20.0); step(&t, &in, 0.1);
    CHECK(t.limit_type == A380X_LIMIT_CLB && NEAR(t.n1_command[0], 20.0 + 60.0 * 20.0 / 25.0, 1e-9),
          "CLB, IDLE->CLB interpolation: %g", t.n1_command[0]);
    /* No FLEX entry: FLX/MCT on the ground gives MCT. */
    a380x_thrust_init(&t, NULL);
    in = ground_inputs();
    all_levers(&t, 35.0); step(&t, &in, 0.1);
    CHECK(!t.flex_rating[0] && t.n1_command[0] == 84.0, "no FLEX temperature: MCT N1");
    /* FLEX 0 C (raw 0.1) above a negative TAT is a valid FLEX. */
    in.flex_temp_valid = 1; in.flex_temp_raw = 0.1; in.tat_c = -5.0; step(&t, &in, 0.1);
    CHECK(t.flex_rating[0], "FLEX 0 C above TAT -5 C");
    in.tat_c = 2.0; step(&t, &in, 0.1);
    CHECK(!t.flex_rating[0], "FLEX 0 C below TAT 2 C");
}

static void test_low_speed_and_reverse_n1(void)
{
    A380XThrust t;
    A380XThrustInputs in = ground_inputs();
    a380x_thrust_init(&t, NULL);
    all_levers(&t, 45.0);
    in.ias_knots = 20.0; step(&t, &in, 0.1);
    CHECK(t.n1_lever[0] == 88.0 && t.n1_command[0] == 76.5, "IAS < 35 kt: N1 <= 76.5 (%g)", t.n1_command[0]);
    in.ias_knots = 50.0;
    all_levers(&t, 25.0 * (65.0 - 20.0) / 60.0); step(&t, &in, 0.1);
    CHECK(t.n1_command[0] == 62.5, "IAS < 60 kt keep-out band: down to 62.5 (%g)", t.n1_command[0]);
    all_levers(&t, 25.0 * (70.0 - 20.0) / 60.0); step(&t, &in, 0.1);
    CHECK(t.n1_command[0] == 73.5, "keep-out band: up to 73.5 (%g)", t.n1_command[0]);
    in.ias_knots = 100.0; step(&t, &in, 0.1);
    CHECK(NEAR(t.n1_command[0], 70.0, 1e-9), "above 60 kt: no keep-out (%g)", t.n1_command[0]);
    all_levers(&t, 0.0);
    a380x_detent_lever_set(&t.reverse[1], 1.0);
    a380x_detent_lever_set(&t.reverse[2], A380X_REV_IDLE_POSITION);
    step(&t, &in, 0.1);
    CHECK(t.tla[1] == -20.0 && t.n1_command[1] == 71.5 && t.limit_type == A380X_LIMIT_TOGA,
          "MAX REV = REV limit: %g", t.n1_command[1]);
    CHECK(NEAR(t.n1_command[2], 21.0, 1e-9), "IDLE REV = IDLE + 1: %g", t.n1_command[2]);
    CHECK(a380x_thrust_schedule(&t, 1, t.n1_command[1], &in.limits) == 1.0
          && a380x_thrust_schedule(&t, 2, t.n1_command[2], &in.limits) == 0.0, "reverse engine commands");
}

static void test_schedule_and_loop(void)
{
    A380XThrust t;
    A380XThrustInputs in = ground_inputs();
    double plant[4] = {20, 20, 20, 20}, u = 0.0;
    int i, k;
    a380x_thrust_init(&t, NULL);
    in.flex_temp_valid = 1; in.flex_temp_raw = 60.0;
    all_levers(&t, 45.0); step(&t, &in, 0.1);
    CHECK(a380x_thrust_engine_command(&t, 0, &in, 0.05) == 1.0, "TOGA keeps X-Plane's full throttle");
    all_levers(&t, 35.0); step(&t, &in, 0.1);
    CHECK(NEAR(a380x_thrust_engine_command(&t, 0, &in, 0.05), (82.0 - 20.0) / 68.0, 1e-9), "FLEX fraction");
    CHECK(NEAR(a380x_thrust_athr_ceiling(&t, 0, &in.limits), (82.0 - 20.0) / 68.0, 1e-9), "A/THR ceiling = lever N1");
    all_levers(&t, 0.0); step(&t, &in, 0.1);
    CHECK(a380x_thrust_engine_command(&t, 0, &in, 0.05) == 0.0, "IDLE");
    /* N1 loop on a plant whose N1 is NOT the scheduled fraction: converges to the FLEX N1. */
    t.fadec_mode = A380X_FADEC_N1_LOOP;
    all_levers(&t, 35.0);
    for (k = 0; k < 1200; ++k) {      /* 60 s at 20 Hz */
        a380x_thrust_latch_flex_temp(&t, &in);
        a380x_thrust_update(&t, &in, 0.05);
        for (i = 0; i < 4; ++i) {
            u = a380x_thrust_engine_command(&t, i, &in, 0.05);
            plant[i] += (18.0 + 95.0 * pow(u, 1.4) - plant[i]) * 0.05 / 1.6;   /* convex N1(u), 1.6 s spool */
            in.n1[i] = plant[i];
        }
    }
    CHECK(NEAR(plant[0], 82.0, 0.3), "closed loop reaches the FLEX N1: %g (u %g)", plant[0], u);
    t.fadec_mode = A380X_FADEC_SCHEDULED;
    a380x_thrust_engine_command(&t, 0, &in, 0.05);
    CHECK(t.loop_trim[0] == 0.0, "trim cleared outside the loop mode");
    in.limits.valid = 0; step(&t, &in, 0.1);
    CHECK(t.limit_type == A380X_LIMIT_NONE && a380x_thrust_engine_command(&t, 0, &in, 0.05) == 0.0,
          "invalid limits: no rating");
}

static void test_flex_temperature_latch_and_transition(void)
{
    A380XThrust t;
    A380XThrustInputs in = ground_inputs();
    a380x_thrust_init(&t, NULL);
    in.flex_temp_valid = 1; in.flex_temp_raw = 55.0;
    all_levers(&t, 35.0); step(&t, &in, 0.1);
    CHECK(t.latched_flex_temp == 55.0, "latched 55");
    in.flex_temp_raw = 40.0; step(&t, &in, 0.1);
    CHECK(t.latched_flex_temp == 55.0, "FMS change while every lever is in FLX does not apply");
    a380x_detent_lever_set(&t.lever[3], 34.0); step(&t, &in, 0.1);
    CHECK(t.latched_flex_temp == 40.0, "one lever out of FLX: re-latched");
    a380x_detent_lever_set(&t.lever[3], 35.0); step(&t, &in, 0.1);
    /* Airborne, CL: CLB limited to FLEX (82 > CLB 80 here -> CLB 80; make CLB > FLEX). */
    in.on_ground = 0; in.limits.clb = 83.0; step(&t, &in, 0.1);
    all_levers(&t, 25.0); step(&t, &in, 0.1);
    CHECK(t.limit_type == A380X_LIMIT_CLB && t.transition && t.limit_n1 == 82.0,
          "after FLEX take-off CLB = min(CLB, FLEX): %g", t.limit_n1);
    step(&t, &in, 9.5);
    CHECK(t.limit_n1 == 82.0, "held for 10 s");
    step(&t, &in, 1.0);
    CHECK(t.limit_n1 == 83.0 && !t.was_flex, "then CLB: %g", t.limit_n1);
}

static void test_instinctive_disconnect_inhibit(void)
{
    A380XThrust t;
    A380XThrustInputs in = ground_inputs();
    a380x_thrust_init(&t, NULL);
    in.on_ground = 0;
    in.athr_disconnect_held = 1;
    step(&t, &in, 14.9);
    CHECK(!t.athr_disabled && NEAR(t.disc_held_s, 14.9, 0.06), "14.9 s: still available (%g)", t.disc_held_s);
    in.athr_disconnect_held = 0; step(&t, &in, 0.05);
    CHECK(t.disc_held_s == 0.0, "release restarts the timer");
    in.athr_disconnect_held = 1;
    a380x_thrust_update(&t, &in, 0.0);   /* paused: no simulated time */
    step(&t, &in, 15.0);
    CHECK(t.athr_disabled, "held >= 15 s: A/THR disabled");
    in.athr_disconnect_held = 0; step(&t, &in, 1.0);
    CHECK(t.athr_disabled, "latched after release");
    in.fmgc_powered = 0; step(&t, &in, 0.1);
    in.fmgc_powered = 1; step(&t, &in, 0.1);
    CHECK(t.athr_disabled, "FMGC power cycle in flight does not reset");
    in.on_ground = 1; step(&t, &in, 0.1);
    CHECK(t.athr_disabled, "landing alone does not reset");
    in.fmgc_powered = 0; step(&t, &in, 0.1);
    in.fmgc_powered = 1; step(&t, &in, 0.1);
    CHECK(!t.athr_disabled, "FMGC power-up on the ground resets (ATHR_RESET_DISABLE)");
}

static void test_disconnect_buttons(void)
{
    A380XThrust t;
    a380x_thrust_init(&t, NULL);
    t.disc_pressed[0] = 1;
    a380x_thrust_disc_buttons(&t, 0.04f);
    CHECK(NEAR(t.disc_anim[0], 0.5f, 1e-6) && t.disc_anim[1] == 0.0f, "left button half travel in 40 ms");
    a380x_thrust_disc_buttons(&t, 0.1f);
    CHECK(t.disc_anim[0] == 1.0f, "pressed");
    t.disc_pressed[0] = 0;
    a380x_thrust_disc_buttons(&t, 0.2f);
    CHECK(t.disc_anim[0] == 0.0f, "released");
}

/* Keyboard throttle steps stop at CL and FLX/MCT while the key is held; the next press moves on. */
static void test_key_detents(void)
{
    A380XThrustCalibration c = a380x_thrust_default_calibration();
    double hold = -1.0, r;
    r = a380x_thrust_key_detent(&c, c.cl - 0.10, c.cl + 0.03, &hold);
    CHECK(fabs(r - c.cl) < 1e-12 && fabs(hold - c.cl) < 1e-12, "step up across CL stops at CL");
    r = a380x_thrust_key_detent(&c, c.cl, c.cl + 0.05, &hold);
    CHECK(fabs(r - c.cl) < 1e-12, "held at CL while the key stays down");
    hold = -1.0;
    r = a380x_thrust_key_detent(&c, c.cl, c.cl + 0.05, &hold);
    CHECK(fabs(r - (c.cl + 0.05)) < 1e-12 && hold < 0.0, "next press leaves CL");
    r = a380x_thrust_key_detent(&c, c.cl - 0.10, c.cl - c.tolerance + 0.001, &hold);
    CHECK(fabs(r - c.cl) < 1e-12, "reaching the CL tolerance band snaps to CL");
    hold = -1.0;
    r = a380x_thrust_key_detent(&c, c.cl - 0.10, c.mct + 0.05, &hold);
    CHECK(fabs(r - c.cl) < 1e-12, "a big step up stops at the first detent (CL)");
    hold = -1.0;
    r = a380x_thrust_key_detent(&c, c.toga, c.mct - 0.01, &hold);
    CHECK(fabs(r - c.mct) < 1e-12, "step down from TOGA stops at FLX/MCT");
    hold = -1.0;
    r = a380x_thrust_key_detent(&c, c.mct, c.cl - 0.05, &hold);
    CHECK(fabs(r - c.cl) < 1e-12, "step down from FLX/MCT stops at CL");
    hold = -1.0;
    r = a380x_thrust_key_detent(&c, c.cl - 0.20, c.cl - 0.15, &hold);
    CHECK(fabs(r - (c.cl - 0.15)) < 1e-12 && hold < 0.0, "below CL the steps are free");
}

int main(void)
{
    test_detent_lever_stickiness();
    test_detent_lever_fast_push_and_timeout();
    test_detent_lever_external_and_lock();
    test_mappings();
    test_interlocks_and_reverse();
    test_hardware_adoption();
    test_flex_latch_and_rating();
    test_low_speed_and_reverse_n1();
    test_schedule_and_loop();
    test_flex_temperature_latch_and_transition();
    test_instinctive_disconnect_inhibit();
    test_disconnect_buttons();
    test_key_detents();
    if (failures) {
        printf("%d of %d checks failed\n", failures, checks);
        return 1;
    }
    printf("Thrust levers: %d checks passed (detents, clicks, interlocks, hardware, FLEX, rating, N1 law, "
           "schedule, 15 s disconnect)\n", checks);
    return 0;
}
