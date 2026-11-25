/* SPDX-License-Identifier: GPL-3.0-or-later
 * Unit tests of the A380 hydraulic model (a380x_hyd.h) against the A380 TTM ATA 29 behaviour
 * (docs/HYDRAULICS.md). Build: cc -std=c99 -Wall -Wextra test_hyd.c -lm */
#include <stdio.h>
#include <stdlib.h>
#include "a380x_hyd.h"

static int failures, checks;
#define CHECK(cond, ...) do { ++checks; if (!(cond)) { ++failures; printf("FAIL %s:%d: ", __FILE__, __LINE__); \
    printf(__VA_ARGS__); printf("\n"); } } while (0)

static A380XHydIn base(void)
{
    A380XHydIn in;
    int e, k;
    memset(&in, 0, sizeof in);
    in.dt = 1.0 / 30.0;
    in.power_valid = 1;
    in.ac[0] = in.ac[1] = in.ac[2] = in.ac[3] = 1;
    in.dc1 = in.dc2 = in.dc_ess = in.dc_gnd_svc = in.ac_gnd_svc = 1;
    in.on_ground = 1;
    in.gear_deploy = 1.0;
    in.ambient_c = 15.0;
    in.ambient_psi = 14.696;
    in.flight_parity = -1;
    for (e = 0; e < 4; ++e) in.edp_pb_auto[e][0] = in.edp_pb_auto[e][1] = 1;
    for (k = 0; k < 2; ++k) in.emp_off_pb_auto[k][0] = in.emp_off_pb_auto[k][1] = 1;
    return in;
}
static void run(A380XHyd *h, const A380XHydIn *in, double seconds)
{
    int n = (int)(seconds / in->dt + 0.5), i;
    for (i = 0; i < n; ++i) a380x_hyd_update(h, in);
}
static void engines(A380XHydIn *in, double n3) { int e; for (e = 0; e < 4; ++e) in->n3_pct[e] = n3; }

static void test_cold_dark(void)
{
    A380XHyd h;
    A380XHydIn in = base();
    int e, k;
    in.power_valid = 1;
    memset(in.ac, 0, sizeof in.ac);
    in.dc1 = in.dc2 = in.dc_ess = in.dc_gnd_svc = in.ac_gnd_svc = 0;
    a380x_hyd_reset(&h, 15.0, 0);
    run(&h, &in, 10.0);
    for (k = 0; k < 2; ++k) {
        CHECK(h.c[k].psi == 0.0, "cold & dark %d psi %.1f", k, h.c[k].psi);
        CHECK(!h.c[k].sys_sw, "cold & dark switch");
        CHECK(fabs(h.c[k].level_l - h.c[k].nominal_l) < 1e-9, "level at nominal");
        CHECK(h.c[k].level_l > 45.0 && h.c[k].level_l < 48.0, "unpressurised level %.2f l", h.c[k].level_l);
        CHECK(!h.emp_fault[k][0] && !h.emp_fault[k][1], "no EMP fault cold");
    }
    for (e = 0; e < 4; ++e) CHECK(!h.edp_fault[e][0] && !h.edp_fault[e][1] && !h.disc_fault[e], "no EDP fault on ground, engines off");
}

static void test_engine_start(void)
{
    A380XHyd h;
    A380XHydIn in = base();
    int e, k, i;
    double peak = 0.0;
    a380x_hyd_reset(&h, 15.0, 0);
    engines(&in, 62.0);
    for (i = 0; i < 60; ++i) {   /* 2 s */
        a380x_hyd_update(&h, &in);
        if (h.c[0].psi > peak) peak = h.c[0].psi;
    }
    for (k = 0; k < 2; ++k) {
        CHECK(h.c[k].psi > 4900.0 && h.c[k].psi < 5200.0, "engines running: %d at %.0f psi", k, h.c[k].psi);
        CHECK(h.c[k].sys_sw, "system switch on");
        for (i = 0; i < 4; ++i) CHECK(h.c[k].sec_sw[i], "EDP section %d/%d pressurised", k, i);
        CHECK(h.c[k].level_l < 46.0 && h.c[k].level_l > 44.0, "pressurised level %.2f", h.c[k].level_l);
        CHECK(h.c[k].acc_fluid_gal > 0.2, "accumulator charged %.3f gal", h.c[k].acc_fluid_gal);
    }
    CHECK(peak < 5300.0, "no overshoot %.0f", peak);
    for (e = 0; e < 4; ++e) {
        CHECK(h.native_engine_pump[e], "native engine pump %d on", e);
        CHECK(!h.edp_fault[e][0] && !h.edp_fault[e][1], "no EDP fault %d", e);
        CHECK(h.edp_active[e][0] && h.edp_active[e][1], "EDPs active %d", e);
    }
    /* variable frame rates stay steady */
    for (i = 0; i < 400; ++i) {
        in.dt = (i % 7 == 0) ? 0.2 : (i % 3 == 0 ? 0.004 : 0.03);
        a380x_hyd_update(&h, &in);
        CHECK(h.c[0].psi > 4800.0 && h.c[0].psi < 5250.0, "steady psi %.0f at dt %.3f", h.c[0].psi, in.dt);
    }
}

static void test_engine_failure_in_flight(void)
{
    A380XHyd h;
    A380XHydIn in = base();
    a380x_hyd_reset(&h, 15.0, 1);
    engines(&in, 90.0);
    in.on_ground = 0;
    in.ambient_c = -50.0; in.ambient_psi = 3.0;
    run(&h, &in, 5.0);
    CHECK(h.c[0].psi > 4900.0, "cruise green %.0f", h.c[0].psi);
    in.n3_pct[0] = 12.0;   /* ENG 1 shut down, windmilling */
    run(&h, &in, 5.0);
    CHECK(h.c[0].sys_sw && h.c[0].psi > 4800.0, "green held by ENG 2: %.0f", h.c[0].psi);
    CHECK(!h.c[0].sec_sw[0] && !h.c[0].sec_sw[1], "ENG 1 sections low");
    CHECK(h.edp_fault[0][0] && h.edp_fault[0][1], "ENG 1 PMP A/B FAULT in flight");
    CHECK(!h.native_engine_pump[0] && h.native_engine_pump[1], "native ENG 1 pump off, ENG 2 on");
    in.edp_pb_auto[0][0] = in.edp_pb_auto[0][1] = 0;   /* crew: ENG 1 PMP A and B OFF */
    run(&h, &in, 1.0);
    CHECK(!h.edp_fault[0][0] && !h.edp_fault[0][1], "FAULT out with the pb OFF");
    in.n3_pct[1] = 0.0;   /* ENG 2 out too: green lost */
    run(&h, &in, 30.0);
    CHECK(!h.c[0].sys_sw && h.c[0].psi < 2900.0, "green lost: %.0f", h.c[0].psi);
    CHECK(h.c[1].sys_sw, "yellow unaffected");
}

static void test_decay(void)
{
    A380XHyd h;
    A380XHydIn in = base();
    double t = 0.0;
    a380x_hyd_reset(&h, 15.0, 1);
    engines(&in, 62.0);
    run(&h, &in, 3.0);
    engines(&in, 0.0);
    while (h.c[0].psi >= 2900.0 && t < 120.0) { a380x_hyd_update(&h, &in); t += in.dt; }
    CHECK(t > 3.0 && t < 30.0, "accumulator holds then pressure decays below 2900 in %.1f s", t);
}

static void test_emp_manual(void)
{
    A380XHyd h;
    A380XHydIn in = base();
    a380x_hyd_reset(&h, 15.0, 0);
    in.emp_on_pb[0][0] = 1;   /* GND HYD: G ELEC PMP A ON */
    run(&h, &in, 6.0);
    CHECK(h.c[0].emp_mode == A380X_HYD_EMP_MANUAL, "manual mode");
    CHECK(h.c[0].emp_run[0] && !h.c[0].emp_run[1], "G EMP A runs");
    CHECK(h.c[0].psi > 4900.0, "EMP pressurises green: %.0f", h.c[0].psi);
    CHECK(h.emp_ctl_memo[0][0], "memo G ELEC PMP A CTL");
    CHECK(h.emp_on_lamp[0][0], "ON light");
    CHECK(!h.emp_fault[0][0], "no FAULT");
    CHECK(h.native_elec_pump[0] && !h.native_elec_pump[1], "native electric pump 1 (green)");
    CHECK(h.c[1].psi == 0.0, "yellow stays unpressurised");
    /* engine started: EEC inhibition */
    in.n3_pct[3] = 60.0; in.edp_pb_auto[3][0] = in.edp_pb_auto[3][1] = 0;
    run(&h, &in, 1.0);
    CHECK(!h.c[0].emp_active[0], "EMP inhibited with an engine running");
    CHECK(!h.emp_fault[0][0], "inhibited EMP: no FAULT");
    in.n3_pct[3] = 0.0;
    /* AC BUS 1 lost: G EMP A unpowered, FAULT after the pump stops */
    in.ac[0] = in.ac_gnd_svc = 0;
    run(&h, &in, 5.0);
    CHECK(!h.c[0].emp_run[0], "unpowered EMP stops");
    in.ac[0] = in.ac_gnd_svc = 1;
    /* failed pump: commanded, low pressure -> FAULT; OFF clears it */
    in.fail_emp[0][0] = 1;
    run(&h, &in, 20.0);
    CHECK(h.emp_fault[0][0], "failed EMP: FAULT (low pressure)");
    in.emp_off_pb_auto[0][0] = 0;
    run(&h, &in, 0.5);
    CHECK(!h.emp_fault[0][0], "FAULT cleared with OFF");
}

static void test_emp_auto_cargo(void)
{
    A380XHyd h;
    A380XHydIn in = base();
    a380x_hyd_reset(&h, 15.0, 0);
    in.flight_parity = 1;   /* odd: main A */
    in.cargo_door_request = 1;
    run(&h, &in, 5.0);
    CHECK(h.c[0].emp_mode == A380X_HYD_EMP_AUTO, "auto mode");
    CHECK(h.c[0].auto_pump == 0 && h.c[0].emp_run[0] && !h.c[0].emp_run[1], "odd flight: EMP A");
    CHECK(h.emp_on_lamp[0][0], "auto copies the ON light");
    CHECK(h.c[0].aux_psi > 4500.0, "cargo door section pressurised %.0f", h.c[0].aux_psi);
    CHECK(h.c[0].psi < 100.0, "system box stays at 0 psi in cargo auto: %.0f", h.c[0].psi);
    CHECK(!h.emp_ctl_memo[0][0], "no CTL memo in auto");
    /* manual request ignored while auto runs */
    in.emp_on_pb[0][1] = 1;
    run(&h, &in, 1.0);
    CHECK(!h.c[0].emp_run[1], "ON B ignored in auto");
    in.emp_on_pb[0][1] = 0;
    /* run-on 20 s after the door stops */
    in.cargo_door_request = 0;
    run(&h, &in, 15.0);
    CHECK(h.c[0].emp_run[0], "still running 15 s after");
    run(&h, &in, 6.0);
    CHECK(!h.c[0].emp_run[0] && h.c[0].emp_mode == A380X_HYD_EMP_IDLE, "stopped after 20 s");
    /* even flight: B; B OFF -> back-up A */
    in.flight_parity = 0;
    in.cargo_door_request = 1;
    run(&h, &in, 1.0);
    CHECK(h.c[0].auto_pump == 1, "even flight: EMP B");
    in.cargo_door_request = 0; run(&h, &in, 25.0);
    in.emp_off_pb_auto[0][1] = 0;
    in.cargo_door_request = 1;
    run(&h, &in, 1.0);
    CHECK(h.c[0].auto_pump == 0 && h.c[0].emp_run[0], "B OFF: back-up A");
    in.emp_off_pb_auto[0][1] = 1;
    in.cargo_door_request = 0; run(&h, &in, 25.0);
    /* manual running: auto request ignored */
    in.emp_on_pb[0][1] = 1;
    run(&h, &in, 3.0);
    in.cargo_door_request = 1;
    run(&h, &in, 2.0);
    CHECK(h.c[0].emp_mode == A380X_HYD_EMP_MANUAL && !h.c[0].emp_to_aux, "auto ignored while manual runs");
    /* yellow BWS auto pressurises the system */
    in.emp_on_pb[0][1] = 0; in.cargo_door_request = 0;
    in.bws_request = 1;
    run(&h, &in, 5.0);
    CHECK(h.c[1].emp_mode == A380X_HYD_EMP_AUTO && h.c[1].psi > 4500.0, "BWS auto: yellow %.0f", h.c[1].psi);
}

static void test_leak_and_low_level(void)
{
    A380XHyd h;
    A380XHydIn in = base();
    double t = 0.0;
    a380x_hyd_reset(&h, 15.0, 1);
    engines(&in, 90.0);
    in.on_ground = 0;
    run(&h, &in, 2.0);
    in.fail_leak[1] = 1;
    while (!h.c[1].level_low && t < 1200.0) { a380x_hyd_update(&h, &in); t += in.dt; }
    CHECK(t > 60.0 && t < 900.0, "yellow RSVR LEVEL LO after %.0f s", t);
    CHECK(h.edp_fault[2][0] && h.edp_fault[3][1], "EDP FAULT lights (low level)");
    CHECK(h.disc_fault[2] && h.disc_fault[3], "DISC FAULT lights");
    CHECK(!h.disc_fault[0], "green DISC dark");
    run(&h, &in, 600.0);
    CHECK(h.c[1].psi < 2900.0, "fluid gone: yellow lost (%.0f psi, %.1f l)", h.c[1].psi, h.c[1].level_l);
    /* disconnect ENG 3: DISC FAULT out, clutch open, irreversible in flight */
    in.disc_pb[2] = 1;
    run(&h, &in, 0.2);
    in.disc_pb[2] = 0;
    run(&h, &in, 0.2);
    CHECK(!h.clutch[2] && h.disc_cmd[2] && !h.disc_fault[2] && h.edp_rpm[2] == 0.0, "ENG 3 EDPs declutched");
    in.edp_reset[2] = 1;
    run(&h, &in, 0.2);
    CHECK(!h.clutch[2], "no reset in flight");
    in.on_ground = 1; in.n3_pct[2] = 0.0;
    run(&h, &in, 0.2);
    CHECK(h.clutch[2] && !h.disc_cmd[2], "reset port on the ground, engine shut down");
    in.edp_reset[2] = 0;
    /* GSP refill */
    in.fail_leak[1] = 0; in.refill[1] = 1;
    run(&h, &in, 400.0);
    CHECK(fabs(h.c[1].level_l - h.c[1].nominal_l) < 0.01 && !h.c[1].level_low, "refilled to nominal");
}

static void test_air_and_overheat(void)
{
    A380XHyd h;
    A380XHydIn in = base();
    a380x_hyd_reset(&h, 15.0, 1);
    engines(&in, 0.0);
    in.fail_air_leak[0] = 1;
    run(&h, &in, 60.0);
    CHECK(h.c[0].air_low, "green RSVR AIR PRESS LO");
    CHECK(h.edp_fault[0][0] && h.edp_fault[1][1], "EDP FAULT lights (air)");
    CHECK(h.emp_fault[0][0] && h.emp_fault[0][1], "EMP FAULT lights on the ground, engines off");
    in.fail_air_leak[0] = 0;
    in.hp_air[0] = 1;   /* ENG 1 HP air */
    run(&h, &in, 30.0);
    CHECK(!h.c[0].air_low && h.c[0].air_psi > 60.0, "repressurised %.1f", h.c[0].air_psi);
    /* reservoir overheat */
    in.fail_rsvr_ovht[1] = 1;
    run(&h, &in, 300.0);
    CHECK(h.c[1].ovht && h.edp_fault[2][0] && h.emp_fault[1][0] && h.disc_fault[3], "yellow SYS OVERHEAT lights");
    in.fail_rsvr_ovht[1] = 0;
    run(&h, &in, 3000.0);
    CHECK(!h.c[1].ovht, "overheat clears when cool (%.0f C)", h.c[1].fluid_c);
    /* EMP overheat: relay opens, OVHT/FAULT until cool */
    in.emp_on_pb[1][0] = 1;
    in.fail_emp_ovht[1][0] = 1;
    run(&h, &in, 200.0);
    CHECK(h.c[1].emp_ovht[0] && !h.c[1].emp_run[0] && h.emp_fault[1][0], "EMP OVHT stops the pump");
    in.fail_emp_ovht[1][0] = 0;
    in.emp_off_pb_auto[1][0] = 0;
    run(&h, &in, 60.0);
    CHECK(h.emp_fault[1][0], "OVHT FAULT stays until cool, even OFF");
    run(&h, &in, 3000.0);
    CHECK(!h.c[1].emp_ovht[0] && !h.emp_fault[1][0], "cooled");
}

static void test_cavitation_altitude(void)
{
    A380XHyd h;
    A380XHydIn in = base();
    a380x_hyd_reset(&h, -50.0, 1);
    engines(&in, 90.0);
    in.on_ground = 0; in.ambient_psi = 2.7; in.ambient_c = -55.0;
    in.fail_air_leak[0] = 1;
    run(&h, &in, 120.0);
    CHECK(h.c[0].air_psi == 0.0, "reservoir air gone");
    {
        double lo = 1e9, hi = 0.0, ylo = 1e9, yhi = 0.0;
        int i;
        for (i = 0; i < 30 * 30; ++i) {
            a380x_hyd_update(&h, &in);
            lo = fmin(lo, h.c[0].psi); hi = fmax(hi, h.c[0].psi);
            ylo = fmin(ylo, h.c[1].psi); yhi = fmax(yhi, h.c[1].psi);
        }
        CHECK(hi - lo > 150.0 && lo < ylo - 1000.0, "G SYS PRESSURE FLUCTUATES and degrades: %.0f..%.0f psi", lo, hi);
        CHECK(yhi - ylo < 50.0, "yellow steady: %.0f..%.0f psi", ylo, yhi);
    }
}

static void test_gear_and_ground_cart(void)
{
    A380XHyd h;
    A380XHydIn in = base();
    double down;
    a380x_hyd_reset(&h, 15.0, 1);
    engines(&in, 80.0);
    run(&h, &in, 3.0);
    down = h.c[0].level_l;
    in.gear_deploy = 0.0; in.on_ground = 0;
    run(&h, &in, 1.0);
    CHECK(fabs(down - h.c[0].level_l - A380X_HYD_GEAR_STORED_L) < 0.05, "gear up stores %.2f l", down - h.c[0].level_l);
    a380x_hyd_reset(&h, 15.0, 0);
    in = base();
    in.ground_cart[1] = 1;
    run(&h, &in, 5.0);
    CHECK(h.c[1].psi > 4800.0 && h.native_elec_pump[1], "ground cart pressurises yellow (%.0f)", h.c[1].psi);
    CHECK(h.c[0].psi == 0.0, "green not");
}

int main(void)
{
    test_cold_dark();
    test_engine_start();
    test_engine_failure_in_flight();
    test_decay();
    test_emp_manual();
    test_emp_auto_cargo();
    test_leak_and_low_level();
    test_air_and_overheat();
    test_cavitation_altitude();
    test_gear_and_ground_cart();
    printf("test_hyd: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
