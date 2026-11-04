/* SPDX-License-Identifier: GPL-3.0-or-later
 * Unit tests of the native A380 electrical core and the flyPad preset runner
 * (docs/ELEC_SYSTEM_SPEC.md section 11.1). Configurations mirror the FBW Rust tests
 * (a380_systems/src/electrical/mod.rs) and the spec tables 3.2, 4.2, 6.1 and 6.5.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "a380x_elec.h"
#include "a380x_elec_presets.h"
#include "a380x_elec_power.h"

static int checks, failures;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; \
    fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define SRC(x) (1u << A380X_ELN_SRC_##x)
#define ON(b) a380x_eln_powered(&s, A380X_ELN_##b)
#define ORIGIN(b) s.origin[A380X_ELN_##b]
#define K(k) a380x_eln_contactor(&s, A380X_ELN_K_##k)

static A380XElnState s;
static A380XElnControls c;
static A380XElnInputs in;

static void reset(int spawn)
{
    a380x_eln_init(&s, spawn);
    a380x_eln_spawn_controls(spawn, &c);
    memset(&in, 0, sizeof(in));
    in.on_ground = 1; in.sim_ready = 1; in.gear_compressed = 1;
}
static void all_bat(int on) { int i; for (i = 0; i < 4; ++i) c.bat_auto[i] = (unsigned char)on; }
static void run(double seconds)
{
    int i, n = (int)ceil(seconds / 0.05);
    in.dt = 0.05;
    for (i = 0; i < n; ++i) a380x_eln_step(&s, &c, &in);
}
static void engines(double n2) { int i; for (i = 0; i < 4; ++i) in.eng_n2[i] = n2; }
static void ext(int connected) { int i; for (i = 0; i < 4; ++i) in.ext_connected[i] = (unsigned char)connected; }

static void test_everything_off(void)
{
    int b;
    reset(A380X_ELN_SPAWN_COLD_DARK);
    run(1.0);
    for (b = A380X_ELN_AC1; b <= A380X_ELN_DC_GND_SVC; ++b) {
        int hot = b >= A380X_ELN_HOT1 && b <= A380X_ELN_HOT_APU;
        CHECK(a380x_eln_powered(&s, b) == hot);
    }
    CHECK(ORIGIN(HOT1) == SRC(BAT1) && ORIGIN(HOT2) == SRC(BAT2));
    CHECK(ORIGIN(HOT_ESS) == SRC(BAT_ESS) && ORIGIN(HOT_APU) == SRC(BAT_APU));
    CHECK(s.volts[A380X_ELN_HOT1] > 27.0f && s.volts[A380X_ELN_HOT1] < 29.0f);
    /* ground power plugged, EXT pbs and batteries OFF: ground service only (EM:677-718) */
    ext(1); run(1.0);
    CHECK(ON(AC_GND_SVC) && ORIGIN(AC_GND_SVC) == SRC(EXT1));
    CHECK(ON(DC_GND_SVC) && ORIGIN(DC_GND_SVC) == SRC(TR2));
    CHECK(!ON(AC1) && !ON(AC2) && !ON(AC3) && !ON(AC4) && !ON(DC1) && !ON(DC2) && !ON(DC_ESS));
    CHECK(K(991PU6) && K(990PX) && !K(990PU2));
}

static void test_batteries_only(void)
{
    reset(A380X_ELN_SPAWN_COLD_DARK);
    all_bat(1); run(1.0);
    CHECK(ON(DC_ESS) && ORIGIN(DC_ESS) == (SRC(BAT1) | SRC(BAT_ESS)));
    CHECK(ON(AC_EMER) && ORIGIN(AC_EMER) == SRC(STAT_INV));
    CHECK(ON(DC_APU) && ORIGIN(DC_APU) == SRC(BAT_APU));
    CHECK(!ON(DC1) && !ON(DC2) && !ON(DC_ESS_INFLT) && !ON(AC_ESS));
    CHECK(K(6PC1) && K(6PB3) && K(7XB) && K(3XB2) && !K(3XB1) && K(5PB));
    /* spec 6.5 battery picture: CAPT PFD off (108PH shed), EWD + CAPT MFD powered, ISIS powered */
    CHECK(!a380x_eln_du_bus_powered(&s, A380X_ELN_DU_CAPT_PFD));
    CHECK(a380x_eln_du_bus_powered(&s, A380X_ELN_DU_EWD) && a380x_eln_du_bus_powered(&s, A380X_ELN_DU_CAPT_MFD));
    CHECK(!a380x_eln_du_bus_powered(&s, A380X_ELN_DU_SD) && !a380x_eln_du_bus_powered(&s, A380X_ELN_DU_FO_PFD));
    CHECK(a380x_eln_isis_powered(&s, 0.0) && !a380x_eln_fcu_powered(&s));
    /* each battery alone: no DC ESS (ELC needs BAT 1 and ESS) */
    { int b; for (b = 0; b < 4; ++b) { reset(A380X_ELN_SPAWN_COLD_DARK); c.bat_auto[b] = 1; run(1.0);
        CHECK(!ON(DC_ESS)); CHECK(ON(DC_APU) == (b == A380X_ELN_BAT_APU)); } }
    /* SOP order BAT 1 -> ESS: DC ESS appears at ESS */
    reset(A380X_ELN_SPAWN_COLD_DARK);
    c.bat_auto[A380X_ELN_BAT1] = 1; run(1.0); CHECK(!ON(DC_ESS));
    c.bat_auto[A380X_ELN_BAT_ESS] = 1; run(0.1); CHECK(ON(DC_ESS) && ON(AC_EMER));
    /* EXT connected, EXT pb off: nothing more (EM:2726-2780) */
    all_bat(1); ext(1); run(1.0);
    CHECK(!ON(AC1) && ON(DC_ESS) && ON(AC_GND_SVC));
}

static void test_ground_sources(void)
{
    reset(A380X_ELN_SPAWN_COLD_DARK); all_bat(1); ext(1);
    c.ext_on[1] = 1; run(6.0);                                   /* EXT 2 only (EM:826-892) */
    CHECK(ON(AC1) && ON(AC2) && !ON(AC3) && !ON(AC4));
    CHECK(ORIGIN(AC1) == SRC(EXT2) && ORIGIN(AC2) == SRC(EXT2));
    CHECK(ON(AC_ESS) && ORIGIN(AC_ESS) == SRC(EXT2) && ON(AC_EMER) && ORIGIN(AC_EMER) == SRC(EXT2));
    CHECK(ON(DC1) && (ORIGIN(DC1) & SRC(TR1)) && ON(DC2) && (ORIGIN(DC2) & SRC(TR1)) && K(980PC));
    CHECK(ON(DC_ESS) && (ORIGIN(DC_ESS) & SRC(TR_ESS)) && !(ORIGIN(DC_ESS) & SRC(TR1)));
    CHECK(ORIGIN(DC_APU) == SRC(BAT_APU) && ON(DC_ESS_INFLT));
    CHECK(K(990PB1) && !K(6PC1) && !K(7XB) && K(3XB1));
    CHECK(a380x_eln_fcu_powered(&s) && a380x_eln_du_bus_powered(&s, A380X_ELN_DU_CAPT_PFD));
    c.ext_on[2] = 1; run(6.0);                                   /* EXT 2 + 3 */
    CHECK(ORIGIN(AC3) == SRC(EXT3) && ORIGIN(AC4) == SRC(EXT3));
    CHECK((ORIGIN(DC2) & SRC(TR2)) && !K(980PC) && (ORIGIN(DC_APU) & SRC(TR_APU)) && K(7PU) && !K(5PB));
    CHECK(K(990PB2) && ON(AC_GND_SVC) && K(991PU2) && ON(AC_EHA) && K(911XN));
    c.ext_on[0] = c.ext_on[3] = 1; run(1.0);                     /* all four: own EXT */
    CHECK(ORIGIN(AC1) == SRC(EXT1) && ORIGIN(AC4) == SRC(EXT4));
    /* APU only */
    reset(A380X_ELN_SPAWN_READY); in.apu_n = 100.0; run(6.0);
    CHECK(ORIGIN(AC1) == SRC(APU_A) && ORIGIN(AC2) == SRC(APU_A) && ORIGIN(AC3) == SRC(APU_B) && ORIGIN(AC4) == SRC(APU_B));
    CHECK(K(990XS1) && K(990XS2) && !K(900XU) && ORIGIN(AC_ESS) == SRC(APU_A));
    /* EXT 2 + APU (EM:894-973) */
    ext(1); c.ext_on[1] = 1; run(1.0);
    CHECK(ORIGIN(AC1) == SRC(APU_A) && ORIGIN(AC2) == SRC(EXT2) && ORIGIN(AC3) == SRC(APU_B) && ORIGIN(AC4) == SRC(APU_B));
    c.ext_on[2] = 1; run(1.0);                                   /* EXT 2 + 3 + APU */
    CHECK(ORIGIN(AC1) == SRC(APU_A) && ORIGIN(AC2) == SRC(EXT2) && ORIGIN(AC3) == SRC(EXT3) && ORIGIN(AC4) == SRC(APU_B));
    /* APU GEN pbs off: APU running supplies nothing */
    ext(0); c.apu_gen_on[0] = c.apu_gen_on[1] = 0; run(1.0);
    CHECK(!ON(AC1) && !ON(AC4));
    /* four engine generators */
    reset(A380X_ELN_SPAWN_READY); engines(70.0); run(6.0);
    CHECK(ORIGIN(AC1) == SRC(GEN1) && ORIGIN(AC2) == SRC(GEN2) && ORIGIN(AC3) == SRC(GEN3) && ORIGIN(AC4) == SRC(GEN4));
    CHECK(ORIGIN(AC_ESS) == SRC(GEN1) && (ORIGIN(DC_APU) & SRC(TR_APU)) && !s.galley_main_shed);
    CHECK(!s.lamps.gen_fault[0] && s.hz[A380X_ELN_AC1] > 500.0f);
    /* APU + one engine: special table not used (gen 1 is outer on the APU A side) */
    reset(A380X_ELN_SPAWN_READY); in.apu_n = 100.0; in.eng_n2[0] = 70.0; run(6.0);
    CHECK(ORIGIN(AC1) == SRC(GEN1) && ORIGIN(AC2) == SRC(APU_A) && ORIGIN(AC3) == SRC(APU_B) && ORIGIN(AC4) == SRC(APU_B));
    /* special table: GEN 2 + APU B only (EM:1438) */
    reset(A380X_ELN_SPAWN_READY); in.apu_n = 100.0; in.eng_n2[1] = 70.0; c.apu_gen_on[0] = 0; run(6.0);
    CHECK(ORIGIN(AC2) == SRC(GEN2) && ORIGIN(AC1) == SRC(GEN2) && ORIGIN(AC4) == SRC(APU_B) && ORIGIN(AC3) == SRC(APU_B));
}

static void test_flight(void)
{
    reset(A380X_ELN_SPAWN_READY); engines(80.0);
    in.on_ground = 0; in.gear_compressed = 0; in.ias_kt = 250.0; in.adr_lsw_valid = 1;
    run(6.0);
    CHECK(s.in_flight_lsw1 && s.flt_condition && !s.tefo);
    in.fail.gen[0] = 1; run(1.0);                                /* GEN 1 lost: AC 1 from AC 2 */
    CHECK(ORIGIN(AC1) == SRC(GEN2) && K(980XU1) && K(980XU2) && s.lamps.gen_fault[0]);
    CHECK(ORIGIN(AC_ESS) == SRC(GEN2));
    in.fail.gen[1] = 1; run(1.0);                                /* GEN 1+2 lost: AC 1/2 cross from the right */
    CHECK(ON(AC1) && ON(AC2) && ORIGIN(AC1) == SRC(GEN4) && K(900XU));   /* AC 1 from AC 4 via the SIC */
    /* in flight only one APU generator supplies (spec 3.1) */
    in.apu_n = 100.0; run(1.0);
    CHECK((ORIGIN(AC1) & SRC(APU_A)) && !K(990XS2));
    /* total loss of generators above 100 kt: emergency elec, RAT, emergency generator */
    reset(A380X_ELN_SPAWN_READY); engines(80.0);
    in.on_ground = 0; in.gear_compressed = 0; in.ias_kt = 250.0; in.adr_lsw_valid = 1; run(6.0);
    engines(0.0); run(0.2);
    CHECK(s.emer_active_s > 0.0 && s.rat_should_deploy && s.rat_latched && !ON(AC1));
    CHECK(s.lamps.rat_fault && ON(DC_ESS) && ON(DC_ESS_INFLT));                /* gap: batteries */
    CHECK(K(6PC1) && K(6PB3) && s.emer_config && s.tefo);
    run(8.0);
    CHECK(s.rat_stow >= 1.0 && s.emer_gen_supplying && K(5XE) && ON(AC_ESS) && ORIGIN(AC_ESS) == SRC(EMER_GEN));
    CHECK(ON(AC_EMER) && (ORIGIN(DC_ESS) & SRC(TR_ESS)) && !s.lamps.rat_fault && ON(DC_ESS_INFLT));
    CHECK(!ON(DC2) && a380x_eln_du_bus_powered(&s, A380X_ELN_DU_CAPT_PFD));
    /* RAT MAN ON on the ground with gear compressed: GCU stays inactive */
    reset(A380X_ELN_SPAWN_READY); c.rat_man_on = 1; run(3.0);
    CHECK(s.rat_latched && !s.gcu_active && !s.emer_gen_supplying);
}

static void test_failures_and_feed(void)
{
    reset(A380X_ELN_SPAWN_READY); engines(70.0); run(6.0);
    in.fail.tr[A380X_ELN_TR_ESS] = 1; run(6.0);                /* TR ESS: DC ESS from DC 1 (6PC2), EM:2400 */
    CHECK(ON(DC_ESS) && K(6PC2) && ORIGIN(DC_ESS) == SRC(TR1) && !K(6PE) && !K(6PB3));
    CHECK(ORIGIN(HOT_ESS) == SRC(BAT_ESS) && ORIGIN(HOT1) == SRC(TR1) && ORIGIN(DC2) == SRC(TR2));
    in.fail.tr[A380X_ELN_TR_ESS] = 0; in.fail.tr[A380X_ELN_TR_APU] = 1; run(1.0);
    CHECK(!K(7PU) && K(5PB) && ORIGIN(DC_APU) == SRC(BAT_APU));
    in.fail.tr[A380X_ELN_TR_APU] = 0; run(1.0);
    in.fail.stat_inv = 1; all_bat(1); in.fail.gen[0] = in.fail.gen[1] = in.fail.gen[2] = in.fail.gen[3] = 1; run(0.2);
    CHECK(!ON(AC_EMER) && s.lamps.ac_ess_feed_fault);            /* no SI output on batteries */
    /* AC ESS FEED (EM:2870-2885) */
    reset(A380X_ELN_SPAWN_READY); engines(70.0); run(6.0);
    CHECK(K(3XC1) && !K(3XC2) && !s.lamps.ac_ess_feed_fault);
    c.ac_ess_feed_normal = 0; run(0.1);
    CHECK(!K(3XC1) && K(3XC2) && ORIGIN(AC_ESS) == SRC(GEN4));
    c.ac_ess_feed_normal = 1; in.fail.ac_bus[0] = 1; run(0.1);
    CHECK(!ON(AC1) && K(3XC2) && ON(AC_ESS));
    /* BUS TIE OFF with EXT 2 only: right side dead, DC 1/DC 2 separated */
    reset(A380X_ELN_SPAWN_READY); ext(1); c.ext_on[1] = 1; c.bus_tie_auto = 0; run(6.0);
    CHECK(ON(AC1) && ON(AC2) && !ON(AC3) && !ON(AC4) && !K(980PC) && !ON(DC2));
    /* galley shedding */
    reset(A380X_ELN_SPAWN_READY); in.apu_n = 100.0; run(1.0);
    CHECK(s.galley_main_shed && !s.galley_sec_shed);
    reset(A380X_ELN_SPAWN_READY); engines(70.0); run(1.0);
    CHECK(!s.galley_main_shed && !s.galley_sec_shed);
    c.comm_on[0] = 0; run(0.1); CHECK(s.galley_main_shed && s.galley_sec_shed);
    c.comm_on[0] = 1; c.pax_sys_on = 0; run(0.1); CHECK(s.galley_sec_shed);
    /* bus failure overrides and battery fault */
    reset(A380X_ELN_SPAWN_READY); engines(70.0); in.fail.dc_bus[2] = 1; run(1.0);
    CHECK(!ON(DC_ESS) && !ON(DC_ESS_INFLT) && ON(DC1));
    in.fail.dc_bus[2] = 0; in.fail.bat[A380X_ELN_BAT1] = 1; run(1.0);
    CHECK(!K(990PB1) && s.lamps.bat_fault[A380X_ELN_BAT1] && !ON(HOT1));
}

static void test_timers(void)
{

    int i;
    /* VFG: 990XU closes 0.5 s after the frequency enters 360-800 Hz */
    reset(A380X_ELN_SPAWN_READY); run(1.0);
    in.eng_n2[0] = 70.0; in.dt = 0.05;
    for (i = 0; i < 9; ++i) a380x_eln_step(&s, &c, &in);
    CHECK(!K(990XU1));
    run(0.1); CHECK(K(990XU1) && ORIGIN(AC1) == SRC(GEN1));
    /* Take-off: X-Plane N2 overshoots 100 % at TOGA; the VFGs stay in band (793 Hz) and online. */
    reset(A380X_ELN_SPAWN_READY); engines(70.0); run(2.0);
    engines(100.4); in.ias_kt = 110.0; in.adr_lsw_valid = 1; run(2.0);
    for (i = 0; i < 4; ++i) CHECK(s.gen_avail[i] && s.gen_hz[i] > 790.0 && s.gen_hz[i] <= 800.0);
    CHECK(ORIGIN(AC1) == SRC(GEN1) && ORIGIN(AC4) == SRC(GEN4));
    /* Below 45.4 % N2 (357 Hz at 45 %) a VFG still drops out. */
    engines(45.0); run(1.0); CHECK(!s.gen_avail[0]);
    /* BCRU line contactor holds 3 s, battery contactor opens at 5 s, 108PH holds 10 s */
    reset(A380X_ELN_SPAWN_READY); ext(1); c.ext_on[1] = c.ext_on[2] = 1; run(8.0);
    CHECK(K(990PU1) && K(990PU2) && K(990PB1) && K(990PB2) && ON(DC_ESS_INFLT));
    ext(0); run(2.9);                                            /* GPU removed (no ground servicing) */
    /* BAT 1: ELC closes at once (BCRU 1 emergency config); BAT 2 keeps 990PB2 for 5 s */
    CHECK(K(990PU1) && !K(990PB1) && K(6PC1) && K(990PB2) && ON(DC_ESS_INFLT));
    run(0.2); CHECK(!K(990PU1) && !K(990PU2));                  /* line contactors open after 3 s */
    run(1.8); CHECK(K(990PB2));
    run(0.2); CHECK(!K(990PB2));                                 /* battery contactor open after 5 s */
    run(4.6); CHECK(ON(DC_ESS_INFLT));
    run(0.6); CHECK(!ON(DC_ESS_INFLT) && ON(DC_ESS));            /* 108PH shed 10 s after AC loss */
    /* DU state machine: boot 0.25-0.45 s, self-test 15 s, standby 10 s */
    {
        A380XElnDu du = {A380X_ELN_DU_OFF, 0.0};
        a380x_eln_du_step(&du, 1, 1.0f, 0, 0.0, 0.3, 15.0); CHECK(du.state == A380X_ELN_DU_BOOT);
        a380x_eln_du_step(&du, 1, 1.0f, 0, 0.31, 0.3, 15.0); CHECK(du.state == A380X_ELN_DU_SELFTEST);
        a380x_eln_du_step(&du, 1, 1.0f, 0, 14.9, 0.3, 15.0); CHECK(du.state == A380X_ELN_DU_SELFTEST);
        a380x_eln_du_step(&du, 1, 1.0f, 0, 0.2, 0.3, 15.0); CHECK(du.state == A380X_ELN_DU_ON);
        a380x_eln_du_step(&du, 0, 1.0f, 0, 0.1, 0.3, 15.0); CHECK(du.state == A380X_ELN_DU_STANDBY);
        a380x_eln_du_step(&du, 0, 1.0f, 0, 9.0, 0.3, 15.0); CHECK(du.state == A380X_ELN_DU_STANDBY);
        a380x_eln_du_step(&du, 1, 1.0f, 0, 0.1, 0.3, 15.0); CHECK(du.state == A380X_ELN_DU_ON); /* no self test */
        a380x_eln_du_step(&du, 1, 0.0f, 0, 0.1, 0.3, 15.0); CHECK(du.state == A380X_ELN_DU_STANDBY); /* brightness 0 */
        a380x_eln_du_step(&du, 1, 0.0f, 0, 10.1, 0.3, 15.0); CHECK(du.state == A380X_ELN_DU_OFF);
        a380x_eln_du_step(&du, 1, 1.0f, 0, 0.1, 0.3, 15.0); CHECK(du.state == A380X_ELN_DU_BOOT);
        a380x_eln_du_step(&du, 0, 1.0f, 0, 0.1, 0.3, 15.0); CHECK(du.state == A380X_ELN_DU_OFF); /* power lost in boot */
        a380x_eln_du_step(&du, 1, 1.0f, 1, 0.1, 0.3, 15.0); CHECK(du.state == A380X_ELN_DU_OFF); /* failed */
    }
    /* whole-aircraft DU picture: batteries -> EWD / CAPT MFD boot, CAPT PFD dark */
    reset(A380X_ELN_SPAWN_COLD_DARK); all_bat(1);
    for (i = 0; i < 20; ++i) { run(0.05); a380x_eln_du_update(&s, NULL, NULL, 0.05, 15.0); }
    CHECK(s.du[A380X_ELN_DU_EWD].state == A380X_ELN_DU_SELFTEST && s.du[A380X_ELN_DU_CAPT_MFD].state == A380X_ELN_DU_SELFTEST);
    CHECK(s.du[A380X_ELN_DU_CAPT_PFD].state == A380X_ELN_DU_OFF && s.du[A380X_ELN_DU_SD].state == A380X_ELN_DU_OFF);
    for (i = 0; i < 300; ++i) { run(0.05); a380x_eln_du_update(&s, NULL, NULL, 0.05, 15.0); }
    CHECK(s.du[A380X_ELN_DU_EWD].state == A380X_ELN_DU_ON);
}

static void test_battery(void)
{
    CHECK(a380x_eln_battery_potential(0.0) == 0.0);
    CHECK(fabs(a380x_eln_battery_potential(3.488) - (13.95303731988 * 3.488 - 2.0 * 3.488 * 3.488)) < 1e-9);
    CHECK(fabs(a380x_eln_battery_potential(10.0) - 25.25) < 1e-9);
    CHECK(fabs(a380x_eln_battery_potential(22.449) - 27.0) < 0.05);
    CHECK(fabs(a380x_eln_battery_potential(23.0) - 27.964) < 0.01);  /* FBW polynomial verbatim */
    /* charging current clamped at 10 A, battery recharges on TR power */
    reset(A380X_ELN_SPAWN_READY); s.bat_ah[A380X_ELN_BAT1] = 12.0; engines(70.0); run(8.0);
    CHECK(s.bat_a[A380X_ELN_BAT1] > 9.9f && s.bat_a[A380X_ELN_BAT1] <= 10.0f && s.bat_ah[A380X_ELN_BAT1] > 12.0);
    /* discharge on batteries: DC ESS + static inverter load drains BAT 1 and ESS */
    reset(A380X_ELN_SPAWN_COLD_DARK); all_bat(1); run(60.0);
    CHECK(s.bat_a[A380X_ELN_BAT1] < -10.0f && s.bat_ah[A380X_ELN_BAT1] < 23.0 && s.bat_ah[A380X_ELN_BAT2] > 22.99);
    /* BAT 1 below 24.5 V (SOC < 20 %): ELC opens, ESS battery keeps DC ESS */
    reset(A380X_ELN_SPAWN_COLD_DARK); all_bat(1); s.bat_ah[A380X_ELN_BAT1] = 4.0; run(1.0);
    CHECK(!K(6PC1) && K(6PB3) && ON(DC_ESS) && !K(7XB));
    /* endurance: about 30-60 minutes on BAT 1 + ESS with the constant load table */
    reset(A380X_ELN_SPAWN_COLD_DARK); all_bat(1); run(1.0);
    { int minutes = 0; while (minutes < 120 && ON(DC_ESS)) { run(60.0); ++minutes; }
      CHECK(minutes >= 25 && minutes <= 75); }
}

static void test_lamps_and_apu(void)
{
    reset(A380X_ELN_SPAWN_COLD_DARK); run(0.5);
    CHECK(!s.lamps.pw_gen[0] && !s.lamps.pw_bat[0] && !s.lamps.pw_ext_avail[0]);
    CHECK(s.lamps.gen_fault[0]);                                 /* FBW: lit with engines stopped */
    ext(1); run(0.1); CHECK(s.lamps.pw_ext_avail[0] && s.lamps.pw_ext_avail[3] && !s.lamps.pw_ext_on[0]);
    all_bat(1); run(0.5);
    CHECK(s.lamps.pw_gen[0] && s.lamps.pw_gen[1] && !s.lamps.pw_gen[2] && s.lamps.pw_ext_on[1]);
    CHECK(s.lamps.pw_bat[A380X_ELN_BAT1] && !s.lamps.pw_bat[A380X_ELN_BAT2] && !s.lamps.ac_ess_feed_fault);
    CHECK(!s.lamps.pw_comm[0] && s.lamps.pw_bus_tie && !s.lamps.pw_galley);
    c.ext_on[1] = 1; run(6.0); CHECK(s.lamps.pw_comm[0] && s.lamps.pw_gen[3] && s.lamps.pw_galley);
    /* APU: MASTER -> flap -> START -> start contactor (DC APU) -> AVAIL */
    reset(A380X_ELN_SPAWN_COLD_DARK); all_bat(1); in.apu_master = 1; run(1.0);
    CHECK(s.apu_ecb_powered && s.apu_flap > 0.0 && s.apu_flap < 1.0);
    CHECK(a380x_eln_apu_start_press(&s)); run(0.5); CHECK(!K(10KA_5KA));
    run(A380X_ELN_APU_FLAP_S); CHECK(K(10KA_5KA) && ON(APU_START) && a380x_eln_apu_starter_command(&s) == 2);
    in.apu_n = 60.0; run(0.1); CHECK(!K(10KA_5KA) && a380x_eln_apu_starter_command(&s) == -1);
    in.apu_n = 96.0; run(1.0); CHECK(!s.apu_avail); run(1.2); CHECK(s.apu_avail && !s.apu_start_latched);
    in.apu_master = 0; run(0.1); CHECK(!s.apu_avail);
    /* APU BAT off: no DC APU, start contactors stay open (EM:2966) */
    reset(A380X_ELN_SPAWN_COLD_DARK); all_bat(1); c.bat_auto[A380X_ELN_BAT_APU] = 0; in.apu_master = 1; run(1.0);
    a380x_eln_apu_start_press(&s); run(A380X_ELN_APU_FLAP_S + 1.0);
    CHECK(s.apu_start_contactor && !K(10KA_5KA) && !ON(DC_APU));
    /* batteries off while the APU turns: emergency shutdown request */
    reset(A380X_ELN_SPAWN_COLD_DARK); all_bat(1); in.apu_master = 1; in.apu_n = 50.0; run(1.0);
    all_bat(0); run(0.2); CHECK(s.apu_emergency_shutdown);
}

static void test_spawn(void)
{
    A380XElnControls cd;
    int i;
    a380x_eln_spawn_controls(A380X_ELN_SPAWN_COLD_DARK, &cd);
    CHECK(!cd.bat_auto[0] && !cd.bat_auto[3] && cd.gen_on[0] && cd.apu_gen_on[1] && !cd.ext_on[0] && cd.bus_tie_auto);
    a380x_eln_spawn_controls(A380X_ELN_SPAWN_TURNAROUND, &cd);
    CHECK(cd.bat_auto[0] && cd.ext_on[0] && cd.ext_on[3]);
    reset(A380X_ELN_SPAWN_READY); engines(70.0);
    a380x_eln_settle(&s, &c, &in); a380x_eln_du_snap(&s, 1);
    CHECK(ON(AC1) && ON(DC_ESS_INFLT) && fabs(s.bat_ah[0] - 23.0) < 1e-9);
    for (i = 0; i < A380X_ELN_DU_COUNT; ++i) CHECK(s.du[i].state == A380X_ELN_DU_ON);
    reset(A380X_ELN_SPAWN_COLD_DARK); a380x_eln_settle(&s, &c, &in); a380x_eln_du_snap(&s, 0);
    for (i = 0; i < A380X_ELN_DU_COUNT; ++i) CHECK(s.du[i].state == A380X_ELN_DU_OFF);
    /* power view fallbacks */
    memset(&a380x_power, 0, sizeof(a380x_power));
    CHECK(a380x_power_bus(A380X_ELN_DC1, 1) && !a380x_power_bus(A380X_ELN_DC1, 0) && a380x_power_du_on(0, 1));
    a380x_power.available = 1; a380x_power.bus = A380X_ELN_BIT(A380X_ELN_DC2);
    a380x_power.du_state[A380X_ELN_DU_SD] = A380X_ELN_DU_SELFTEST;
    CHECK(a380x_power_bus(A380X_ELN_DC2, 0) && !a380x_power_bus(A380X_ELN_DC1, 1));
    CHECK(!a380x_power_du_on(A380X_ELN_DU_SD, 1) && a380x_power_du_lit(A380X_ELN_DU_SD, 0) && a380x_power_du_state(A380X_ELN_DU_SD) == 2);
}

/* ---------------------------------------------------------------- presets with a mock host */
static int mock[A380X_PI_COUNT][4], mock_sets, mock_eng_timer[4];
static int mock_get(void *ctx, int item, int index)
{
    (void)ctx;
    if (item == A380X_PI_ENG_RUNNING) return mock[A380X_PI_ENG_MASTER][index] && mock_eng_timer[index] > 3;
    if (item == A380X_PI_ENG_STOPPED) return !mock[A380X_PI_ENG_MASTER][index];
    if (item == A380X_PI_AC1_POWERED) return mock[A380X_PI_EXT_PB][1] || (mock[A380X_PI_APU_AVAIL][0] && mock[A380X_PI_APU_MASTER][0]);
    if (item == A380X_PI_FLAPS_AT) return mock[A380X_PI_FLAPS][0];
    if (item == A380X_PI_RUDDER_TRIM_ZERO) return 1;
    if (item == A380X_PI_APU_AVAIL) return mock[A380X_PI_APU_AVAIL][0] && mock[A380X_PI_APU_MASTER][0];
    if (item == A380X_PI_APU_START) return mock[A380X_PI_APU_START][0] || mock[A380X_PI_APU_AVAIL][0];
    return mock[item][index];
}
static void mock_set(void *ctx, int item, int index, int value)
{
    (void)ctx;
    mock[item][index] = value; ++mock_sets;
}
static void test_presets(void)
{
    A380XPresetRunner r;
    A380XPresetHost host = {mock_get, mock_set, NULL};
    int procs[4], n, i, frames;
    const A380XPresetStep *steps;
    CHECK(a380x_preset_procedures(1, procs) == 4 && procs[3] == A380X_PROC_POWERUP_OFF && procs[0] == A380X_PROC_TAKEOFF_OFF);
    CHECK(a380x_preset_procedures(5, procs) == 4 && procs[0] == A380X_PROC_POWERUP_ON && procs[3] == A380X_PROC_TAKEOFF_ON);
    CHECK(a380x_preset_procedures(0, procs) == 0 && a380x_preset_procedures(6, procs) == 0);
    steps = a380x_preset_procedure(A380X_PROC_POWERUP_ON, &n);
    CHECK(n > 10 && !strcmp(steps[0].name, "BAT1 On") && steps[1].index == A380X_ELN_BAT_ESS && steps[3].delay_ms == 3000);
    /* normal mode: BAT 1 first, 1 s later ESS */
    memset(mock, 0, sizeof(mock)); mock_sets = 0;
    a380x_preset_start(&r, A380X_PRESET_TURNAROUND, 0);
    a380x_preset_update(&r, &host, 0.0);
    CHECK(mock[A380X_PI_STROBE][0] == 1 && mock[A380X_PI_BAT][0] == 0);   /* TAKEOFF_OFF: strobe AUTO first (FBW) */
    for (frames = 0; frames < 100 && !mock[A380X_PI_BAT][0]; ++frames) a380x_preset_update(&r, &host, 0.05);
    CHECK(mock[A380X_PI_BAT][0] == 1 && mock[A380X_PI_BAT][2] == 0 && r.current && !strcmp(r.current, "BAT1 On"));
    a380x_preset_update(&r, &host, 0.5); CHECK(mock[A380X_PI_BAT][2] == 0);
    a380x_preset_update(&r, &host, 0.6); CHECK(mock[A380X_PI_BAT][2] == 1 && mock[A380X_PI_BAT][1] == 0);
    /* waits at "Await APU Avail" until the mock APU is available */
    for (frames = 0; frames < 2000 && r.running; ++frames) a380x_preset_update(&r, &host, 0.1);
    CHECK(r.running && r.current && !strcmp(r.current, "Await APU Avail"));
    CHECK(mock[A380X_PI_EXT_PB][0] && mock[A380X_PI_EXT_PB][3] && mock[A380X_PI_ADIRS_MODE][2] == 1 && mock[A380X_PI_FUEL_PUMP][3]);
    CHECK(mock[A380X_PI_GPU][0] && mock[A380X_PI_APU_MASTER][0] && mock[A380X_PI_APU_START][0]);
    mock[A380X_PI_APU_AVAIL][0] = 1;
    for (frames = 0; frames < 200 && r.running; ++frames) a380x_preset_update(&r, &host, 0.1);
    CHECK(!r.running && r.done && mock[A380X_PI_APU_BLEED][0] == 1 && r.steps_done == r.steps_total);
    /* re-running with everything satisfied sets nothing */
    mock_sets = 0; a380x_preset_start(&r, A380X_PRESET_TURNAROUND, 0);
    for (frames = 0; frames < 400 && r.running; ++frames) a380x_preset_update(&r, &host, 0.1);
    CHECK(!r.running && mock_sets == 0);  /* satisfied conditions still honour their delay (FBW) */
    /* expedited ready-for-taxi: PROC/NCON skipped, delays skipped, ADIRS align-now EXON runs */
    memset(mock, 0, sizeof(mock)); mock[A380X_PI_APU_AVAIL][0] = 1;
    for (i = 0; i < 3; ++i) mock[A380X_PI_ADIRS_ALIGNED][i] = 1;
    a380x_preset_start(&r, A380X_PRESET_TAXI, 1);
    for (frames = 0; frames < 400 && r.running; ++frames) {
        for (i = 0; i < 4; ++i) if (mock[A380X_PI_ENG_MASTER][i]) ++mock_eng_timer[i];
        a380x_preset_update(&r, &host, 0.1);
    }
    CHECK(!r.running && r.done && frames < 60);
    CHECK(mock[A380X_PI_ADIRS_ALIGN_NOW][0] == 1 && mock[A380X_PI_ENG_MASTER][3] == 1 && mock[A380X_PI_FLAPS][0] == 1);
    CHECK(mock[A380X_PI_GPU][0] == 0 && mock[A380X_PI_EXT_PB][1] == 0 && mock[A380X_PI_BEACON][0] == 1);
    CHECK(mock[A380X_PI_APU_MASTER][0] == 0 && mock[A380X_PI_AUTOBRAKE_RTO][0] == 1);
    /* cold & dark from there: engines off, everything off in reverse */
    a380x_preset_start(&r, A380X_PRESET_COLD_DARK, 1);
    for (frames = 0; frames < 400 && r.running; ++frames) a380x_preset_update(&r, &host, 0.1);
    CHECK(!r.running && r.done && mock[A380X_PI_BAT][0] == 0 && mock[A380X_PI_ENG_MASTER][0] == 0 && mock[A380X_PI_BEACON][0] == 0);
    /* cancel */
    a380x_preset_start(&r, A380X_PRESET_TURNAROUND, 0); a380x_preset_update(&r, &host, 0.0);
    a380x_preset_cancel(&r); CHECK(!a380x_preset_update(&r, &host, 1.0) && !r.done);
}

int main(void)
{
    test_everything_off();
    test_batteries_only();
    test_ground_sources();
    test_flight();
    test_failures_and_feed();
    test_timers();
    test_battery();
    test_lamps_and_apu();
    test_spawn();
    test_presets();
    if (failures) { fprintf(stderr, "%d of %d checks failed\n", failures, checks); return 1; }
    printf("a380x elec core tests passed (%d checks)\n", checks);
    return 0;
}
