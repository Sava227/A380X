/* SPDX-License-Identifier: GPL-3.0-or-later
 * Overhead annunciator lamp logic (a380x_ovhd_lamps.h): table integrity, FBW per-pushbutton
 * supply, conditions of the simulated pushbuttons, ANN LT TEST / BRT / DIM, the simplified
 * supply setting and the legacy (no network) supply.
 */
#include <assert.h>
#include <stdio.h>
#include "a380x_ovhd_lamps.h"

#define BIT(bus) A380X_ELN_BIT(A380X_ELN_##bus)
static int pb(const char *id) { int p = a380x_ovhd_find(id); assert(p >= 0); return p; }
static float up(const A380XOvhdInputs *in, const char *id) { return a380x_ovhd_lamp(in, pb(id), 0); }
static float lo(const A380XOvhdInputs *in, const char *id) { return a380x_ovhd_lamp(in, pb(id), 1); }
static void network(A380XOvhdInputs *in, uint32_t buses)
{
    memset(in, 0, sizeof(*in));
    in->network = 1; in->buses = buses; in->ann_lt = A380X_OVHD_ANN_BRT;
    /* FBW running-aircraft normal positions: bleeds, packs, feed pumps and crew oxygen ON */
    memset(in->eng_bleed, 1, 4); memset(in->pack, 1, 2); memset(in->feed_pump, 1, 4); in->crew_oxy = 1;
    for (int i = 0; i < A380X_OVHD_PB_COUNT; ++i) in->pb[i] = (unsigned char)a380x_ovhd_pb_default(i);  /* normal positions */
}

static void test_table(void)
{
    int p, s, lamps = 0;
    assert(A380X_OVHD_PB_COUNT == 127);
    for (p = 0; p < A380X_OVHD_PB_COUNT; ++p) {
        const A380XOvhdLampInfo *info = &a380x_ovhd_pbs[p];
        assert(info->id && info->node && !strncmp(info->node, "PUSH_OVHD_", 10));
        assert(a380x_ovhd_find(info->id) == p);
        assert(info->any_of || info->all_of);                  /* every pushbutton has a supply */
        assert(info->legend[0] != A380X_OVHD_LEGEND_NONE || info->legend[1] != A380X_OVHD_LEGEND_NONE);
        for (s = 0; s < 2; ++s) {
            assert(info->legend[s] < A380X_OVHD_LEGEND_COUNT && info->source[s] < A380X_OVHD_SRC_COUNT);
            assert((info->legend[s] == A380X_OVHD_LEGEND_NONE) == (info->source[s] == A380X_OVHD_SRC_NONE));
            lamps += info->legend[s] != A380X_OVHD_LEGEND_NONE;
        }
    }
    assert(lamps == 217 && a380x_ovhd_find("nope") < 0 && a380x_ovhd_find(NULL) < 0);
    /* FBW SEQ_POWERED examples (A380_Cockpit_Behavior.xml) */
    assert(a380x_ovhd_pbs[pb("apu_master")].any_of == BIT(DC_APU));
    assert(a380x_ovhd_pbs[pb("apu_start")].any_of == (BIT(AC_ESS) | BIT(DC_APU)));
    assert(a380x_ovhd_pbs[pb("pack1")].all_of == (BIT(DC1) | BIT(DC_ESS)) && !a380x_ovhd_pbs[pb("pack1")].any_of);
    assert(a380x_ovhd_pbs[pb("eng1_bleed")].any_of == BIT(DC2) && a380x_ovhd_pbs[pb("eng2_bleed")].any_of == BIT(DC_ESS));
    assert(a380x_ovhd_pbs[pb("prim1")].any_of == (BIT(DC_ESS) | BIT(DC1) | BIT(DC2)));
    assert(!strcmp(a380x_ovhd_legend_text[a380x_ovhd_pbs[pb("apu_start")].legend[0]], "AVAIL"));
}

static void test_supply_and_conditions(void)
{
    A380XOvhdInputs in;
    /* cold & dark: only the hot buses -> every legend dark, even in ANN LT TEST */
    network(&in, BIT(HOT1) | BIT(HOT2) | BIT(HOT_ESS) | BIT(HOT_APU));
    in.ann_lt = A380X_OVHD_ANN_TEST;
    assert(up(&in, "apu_bleed") == 0.0f && lo(&in, "rcdr_gnd_ctl") == 0.0f && lo(&in, "feed1_main") == 0.0f);
    /* BAT 1 + ESS: DC ESS (+ AC EMER). DC ESS legends light, DC 2 / AC ones stay dark (FBW) */
    network(&in, BIT(DC_ESS) | BIT(AC_EMER) | BIT(HOT1) | BIT(HOT_ESS));
    in.eng_bleed[1] = 0; in.eng_bleed[0] = 0;                   /* ENG 1 and 2 BLEED OFF */
    assert(lo(&in, "eng2_bleed") == 1.0f && lo(&in, "eng1_bleed") == 0.0f);   /* DC ESS vs DC 2 */
    in.apu_bleed = 1; assert(lo(&in, "apu_bleed") == 1.0f && up(&in, "apu_bleed") == 0.0f);
    in.feed_pump[0] = 0; assert(lo(&in, "feed1_main") == 0.0f);               /* AC ESS dead */
    in.apu_master = 1; assert(lo(&in, "apu_master") == 0.0f);                  /* DC APU dead */
    in.ann_lt = A380X_OVHD_ANN_TEST;
    assert(lo(&in, "rcdr_gnd_ctl") == 1.0f && up(&in, "eng3_bleed") == 1.0f && lo(&in, "fadec1") == 1.0f);
    assert(lo(&in, "hot_air1") == 0.0f && up(&in, "pack2") == 0.0f && lo(&in, "man_start1") == 0.0f);
    /* + APU BAT: DC APU -> APU MASTER SW ON, START ON while starting, AVAIL after */
    network(&in, BIT(DC_ESS) | BIT(DC_APU));
    in.apu_master = 1; assert(lo(&in, "apu_master") == 1.0f && up(&in, "apu_master") == 0.0f);
    in.apu_start = 1; assert(lo(&in, "apu_start") == 1.0f && up(&in, "apu_start") == 0.0f);
    in.apu_avail = 1; assert(lo(&in, "apu_start") == 0.0f && up(&in, "apu_start") == 1.0f);
    in.apu_failed = 1; assert(up(&in, "apu_master") == 1.0f);
    in.apu_master = 0; assert(up(&in, "apu_master") == 0.0f && lo(&in, "apu_master") == 0.0f);
    /* AC 1/2/ESS + DC 1/2/ESS (EXT power): fuel, anti-ice, packs, crew oxygen */
    network(&in, BIT(AC1) | BIT(AC2) | BIT(AC_ESS) | BIT(DC1) | BIT(DC2) | BIT(DC_ESS));
    assert(lo(&in, "feed1_main") == 0.0f);                                     /* pump ON */
    in.feed_pump[0] = in.feed_pump[3] = 0;
    assert(lo(&in, "feed1_main") == 1.0f && lo(&in, "feed4_main") == 1.0f && lo(&in, "feed2_main") == 0.0f);
    in.pack[0] = 0; assert(lo(&in, "pack1") == 1.0f && lo(&in, "pack2") == 0.0f);
    in.eng_ai[2] = 1; assert(lo(&in, "eng3_anti_ice") == 1.0f && lo(&in, "eng1_anti_ice") == 0.0f);
    in.eng_ai_failed[2] = 1; assert(up(&in, "eng3_anti_ice") == 1.0f);
    in.wing_ai = 1; assert(lo(&in, "wing_anti_ice") == 1.0f);
    in.window_heat = 1; assert(lo(&in, "probes_window_heat") == 1.0f);
    in.crew_oxy = 0; assert(lo(&in, "crew_oxy") == 1.0f);
    in.eng_bleed_failed[3] = 1; assert(up(&in, "eng4_bleed") == 1.0f);         /* selected on + failure */
    in.eng_bleed[3] = 0; assert(up(&in, "eng4_bleed") == 0.0f && lo(&in, "eng4_bleed") == 1.0f);
    in.fault[pb("feed2_stby")] = 1; assert(up(&in, "feed2_stby") == 1.0f && lo(&in, "feed2_stby") == 0.0f);
    assert(lo(&in, "xfeed1") == 0.0f && up(&in, "fire_eng1_agent1") == 0.0f);   /* normal positions: dark */
    /* pushbutton positions (dark cockpit): normal positions dark, OFF while released, ON while pressed in */
    assert(lo(&in, "hyd_eng1a") == 0.0f && lo(&in, "prim1") == 0.0f && lo(&in, "man_start1") == 0.0f);
    in.pb[pb("hyd_eng1a")] = 0; assert(lo(&in, "hyd_eng1a") == 1.0f);           /* ENG 1 PUMP A OFF */
    in.pb[pb("man_start1")] = 1; assert(lo(&in, "man_start1") == 1.0f);         /* MAN START ON (AC ESS) */
    in.pb[pb("xfeed1")] = 1; assert(lo(&in, "xfeed1") == 1.0f && up(&in, "xfeed1") == 1.0f);   /* ON + OPEN */
    in.pb[pb("fire_eng1_agent1")] = 1; assert(up(&in, "fire_eng1_agent1") == 0.0f);          /* SQUIB stays test */
    assert(a380x_ovhd_has_pb(pb("prim1")) && !a380x_ovhd_has_pb(pb("feed1_main")) && a380x_ovhd_pb_default(pb("prim1")));
    in.pb[pb("xfeed1")] = 0;
    in.ann_lt = A380X_OVHD_ANN_DIM;
    assert(fabsf(lo(&in, "feed1_main") - A380X_OVHD_DIM_LEVEL) < 1e-6f && lo(&in, "feed2_main") == 0.0f);
    in.ann_lt = A380X_OVHD_ANN_TEST;
    assert(lo(&in, "xfeed1") == 1.0f && up(&in, "xfeed1") == 1.0f && up(&in, "cargo_btl1") == 1.0f);
    assert(lo(&in, "cargo_btl1") == 0.0f);                                      /* no authored face */
}

static void test_simple_and_legacy(void)
{
    A380XOvhdInputs in;
    network(&in, BIT(HOT1) | BIT(HOT2));
    in.feed_pump[0] = 0;
    in.supply_mode = A380X_OVHD_SUPPLY_SIMPLE; in.simple_power = 1;   /* BAT 1 + BAT 2 AUTO */
    assert(lo(&in, "feed1_main") == 1.0f && lo(&in, "eng1_bleed") == 0.0f);
    in.simple_power = 0; assert(lo(&in, "feed1_main") == 0.0f);
    memset(&in, 0, sizeof(in));                                          /* no network */
    in.ann_lt = A380X_OVHD_ANN_BRT; in.apu_master = 1;
    assert(lo(&in, "apu_master") == 0.0f);
    in.legacy_power = 1; assert(lo(&in, "apu_master") == 1.0f && lo(&in, "eng1_bleed") == 1.0f);
}

static void test_ann_switch(void)
{
    assert(a380x_ovhd_ann_step(A380X_OVHD_ANN_BRT, 1) == A380X_OVHD_ANN_TEST);
    assert(a380x_ovhd_ann_step(A380X_OVHD_ANN_TEST, 1) == A380X_OVHD_ANN_TEST);
    assert(a380x_ovhd_ann_step(A380X_OVHD_ANN_BRT, 0) == A380X_OVHD_ANN_DIM);
    assert(a380x_ovhd_ann_step(A380X_OVHD_ANN_DIM, 0) == A380X_OVHD_ANN_DIM);
    assert(a380x_ovhd_lamp(NULL, 0, 0) == 0.0f && a380x_ovhd_powered(NULL, 0) == 0);
}

int main(void)
{
    test_table();
    test_supply_and_conditions();
    test_simple_and_legacy();
    test_ann_switch();
    puts("A380 overhead lamps: table, FBW supply buses, conditions, ANN LT, simplified and legacy supply passed");
    return 0;
}
