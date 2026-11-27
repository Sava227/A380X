/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 electrical network: X-Plane adapter (phase 2, docs/ELEC_SYSTEM.md).
 *
 * Owns the native network core (a380x_elec.c) at the XPLM boundary:
 *   - takes over the ELEC panel (a380x_elec_panel.inc external_logic): crew pb states feed
 *     the core, the core feeds FAULT conditions and per-legend lamp power back;
 *   - publishes a380x_power (a380x_elec_power.h) for every consumer, before they run;
 *   - slaves X-Plane's stock electrical system (spec 9.3): stock batteries follow native DC,
 *     the stock battery charge is pinned, generators / APU generator / GPU / avionics switch
 *     follow the native pbs and network, so stock-dependent parts never contradict it;
 *   - intercepts stock battery / generator / APU / GPU / avionics commands (hardware bindings)
 *     and routes them to the native pbs (spec 9.5.4);
 *   - runs the APU electronic control box on X-Plane's APU turbine (spec 9.3 option A);
 *   - decides the spawn state (cold & dark / turnaround / ready, spec 7.2) and runs the flyPad
 *     aircraft-state presets (a380x_elec_presets.c) on native controls;
 *   - publishes a380x/xp12/elec/... (bus, contactor, source, battery, DU, preset datarefs).
 * Unity-build include of a380x_avionics.c after a380x_elec_panel.inc.
 */
#include "a380x_elec.h"
#include "a380x_elec_presets.h"
#include "a380x_elec_power.h"
#include "a380x_apu_pw980.h"   /* FBW PW980 N / N2: the ECB and the displays use it, not X-Plane's APU N1 */

/* CDS boot / self-test screen, drawn by a380x_avionics.c (needs the EIS font). */
static int cds_overlay(int du, float x, float y, float w, float h);
/* A late engine-running detection switched the spawn to READY: consumers re-initialise. */
static void elec_spawn_changed(void);

enum {
    ENR_N1, ENR_N2, ENR_RUNNING, ENR_ON_GROUND, ENR_GROUNDSPEED, ENR_IAS, ENR_PAUSED, ENR_REPLAY, ENR_SPEED,
    ENR_APU_N, ENR_APU_SWITCH, ENR_APU_GEN_ON, ENR_APU_RUNNING, ENR_BATTERY_ON, ENR_NUM_BAT, ENR_CHARGE, ENR_CHARGE_MAX,
    ENR_GEN_ON, ENR_GPU_ON, ENR_AVIONICS, ENR_CROSS_TIE, ENR_INVERTER, ENR_GPU_CALLABLE,
    ENR_FAIL_GEN0, ENR_FAIL_GEN1, ENR_FAIL_GEN2, ENR_FAIL_GEN3,
    ENR_FAIL_BAT0, ENR_FAIL_BAT1, ENR_FAIL_BAT2, ENR_FAIL_BAT3,
    ENR_FAIL_ESYS1, ENR_FAIL_ESYS2, ENR_FAIL_ESYS3, ENR_FAIL_ESYS4, ENR_FAIL_ESYS5, ENR_FAIL_ESYS6,
    /* flyPad preset items (optional) */
    ENR_MIXTURE, ENR_FUEL_PUMP, ENR_STROBE, ENR_NAV, ENR_BEACON, ENR_SEATBELT, ENR_NO_SMOKING,
    ENR_LANDING, ENR_TAXI, ENR_O2, ENR_APU_BLEED, ENR_PARKBRAKE, ENR_FLAP_RATIO, ENR_FLAP_DEPLOY,
    ENR_SPEEDBRAKE, ENR_RUDDER_TRIM, ENR_TERR, ENR_XPDR,
    ENR_COUNT, ENR_REQUIRED = ENR_GPU_CALLABLE
};
#define A380X_ELN_TO_LIGHT 0   /* generic light index of the nose T.O. lights (tools/exterior_lights.py) */
static const char *const eln_raw_names[ENR_COUNT] = {
    "sim/cockpit2/engine/indicators/N1_percent",
    "sim/flightmodel/engine/ENGN_N2_",
    "sim/flightmodel/engine/ENGN_running",
    "sim/flightmodel/failures/onground_any",
    "sim/flightmodel/position/groundspeed",
    "sim/cockpit2/gauges/indicators/airspeed_kts_pilot",
    "sim/time/paused",
    "sim/operation/prefs/replay_mode",
    "sim/time/sim_speed_actual",
    "sim/cockpit2/electrical/APU_N1_percent",
    "sim/cockpit2/electrical/APU_starter_switch",
    "sim/cockpit2/electrical/APU_generator_on",
    "sim/cockpit2/electrical/APU_running",
    "sim/cockpit2/electrical/battery_on",
    "sim/aircraft/electrical/num_batteries",
    "sim/cockpit/electrical/battery_charge_watt_hr",
    "sim/aircraft/electrical/battery_watt_hr_max",
    "sim/cockpit2/electrical/generator_on",
    "sim/cockpit2/electrical/GPU_generator_on",
    "sim/cockpit2/switches/avionics_power_on",
    "sim/cockpit2/electrical/cross_tie",
    "sim/cockpit2/electrical/inverter_on",
    "sim/cockpit2/electrical/GPU_can_be_called_for",
    "sim/operation/failures/rel_genera0", "sim/operation/failures/rel_genera1",
    "sim/operation/failures/rel_genera2", "sim/operation/failures/rel_genera3",
    "sim/operation/failures/rel_batter0", "sim/operation/failures/rel_batter1",
    "sim/operation/failures/rel_batter2", "sim/operation/failures/rel_batter3",
    "sim/operation/failures/rel_esys", "sim/operation/failures/rel_esys2", "sim/operation/failures/rel_esys3",
    "sim/operation/failures/rel_esys4", "sim/operation/failures/rel_esys5", "sim/operation/failures/rel_esys6",
    "sim/cockpit2/engine/actuators/mixture_ratio",
    "sim/cockpit2/engine/actuators/fuel_pump_on",
    "sim/cockpit2/switches/strobe_lights_on",
    "sim/cockpit2/switches/navigation_lights_on",
    "sim/cockpit2/switches/beacon_on",
    "sim/cockpit2/switches/fasten_seat_belts",
    "sim/cockpit2/switches/no_smoking",
    "sim/cockpit2/switches/landing_lights_on",
    "sim/cockpit2/switches/taxi_light_on",
    "sim/cockpit2/oxygen/actuators/o2_valve_on",
    "sim/cockpit2/bleedair/actuators/apu_bleed",
    "sim/cockpit2/controls/parking_brake_ratio",
    "sim/cockpit2/controls/flap_ratio",
    "sim/flightmodel2/controls/flap_handle_deploy_ratio",
    "sim/cockpit2/controls/speedbrake_ratio",
    "sim/cockpit2/controls/rudder_trim",
    "sim/cockpit2/EFIS/EFIS_terrain_on",
    "sim/cockpit2/radios/actuators/transponder_mode",
};

/* Extra native failure inputs (the panel already owns fail/<pb id>). */
enum {
    EF_TR1, EF_TR2, EF_TR_ESS, EF_TR_APU, EF_STAT_INV, EF_AC_BUS1, EF_AC_BUS2, EF_AC_BUS3, EF_AC_BUS4,
    EF_DC_BUS1, EF_DC_BUS2, EF_DC_ESS_BUS, EF_EXT1, EF_EXT2, EF_EXT3, EF_EXT4, EF_GALLEY, EF_COUNT
};
static const char *const eln_fail_names[EF_COUNT] = {
    "tr1", "tr2", "tr_ess", "tr_apu", "stat_inv", "ac_bus1", "ac_bus2", "ac_bus3", "ac_bus4",
    "dc_bus1", "dc_bus2", "dc_ess_bus", "ext1", "ext2", "ext3", "ext4", "galley"
};

static struct {
    int ready, active, mode, default_spawn, spawn_checks;
    double since_load;
    XPLMDataRef raw[ENR_COUNT];
    A380XElnState st;
    A380XElnControls ctl;
    A380XElnInputs in;
    unsigned char fail[EF_COUNT];
    float selftest_s;
    int bat_selector;                 /* 0 ESS, 1 APU, 2 OFF, 3 BAT 1, 4 BAT 2 */
    int written_starter, avionics_logged;
    /* a380x/xp12/ovhd/settings/lamp_supply = 1 (a380x_ovhd_lamps.inc): simplified, non-FBW
     * annunciator supply for the ELEC panel legends too (eln_simple_lamp_power). */
    int lamp_supply_simple;
    /* preset runner + native item states without an X-Plane equivalent */
    A380XPresetRunner preset;
    int preset_expedited, preset_last, nose, strobe_auto, no_smoking_auto, rwy_turnoff;
    int nose_taxi;                    /* taxi_light_on as last written from nose (0 OFF, 1 TAXI, 2 T.O.) */
    XPLMDataRef generic_lights;       /* the T.O. lights are generic light A380X_ELN_TO_LIGHT */
    char preset_text[64];
    XPLMCommandRef flaps_down, to_config;
    int to_config_held;
    A380XPw980 apu;
} eln;

static int eln_rawi(int i) { return eln.raw[i] ? XPLMGetDatai(eln.raw[i]) : 0; }
static float eln_rawf(int i)
{
    float v = eln.raw[i] ? XPLMGetDataf(eln.raw[i]) : 0.0f;
    return isfinite(v) ? v : 0.0f;
}
static void eln_seti(int i, int v) { if (eln.raw[i] && XPLMGetDatai(eln.raw[i]) != v) XPLMSetDatai(eln.raw[i], v); }
static void eln_setf(int i, float v) { if (eln.raw[i]) XPLMSetDataf(eln.raw[i], v); }
static int eln_failed(int i) { return eln_rawi(i) >= 6; }
/* One element at `index`: X-Plane writes it to the first slot of the buffer. */
static int eln_vi(int i, int index)
{
    int v = 0;
    if (!eln.raw[i] || index < 0 || index >= 16) return 0;
    XPLMGetDatavi(eln.raw[i], &v, index, 1);
    return v;
}
static float eln_vf(int i, int index)
{
    float v = 0.0f;
    if (!eln.raw[i] || index < 0 || index >= 16) return 0.0f;
    XPLMGetDatavf(eln.raw[i], &v, index, 1);
    return isfinite(v) ? v : 0.0f;
}
static void eln_set_vi(int i, int index, int value)
{
    if (eln.raw[i] && eln_vi(i, index) != value) XPLMSetDatavi(eln.raw[i], &value, index, 1);
}
static void eln_set_vf(int i, int index, float value)
{
    if (eln.raw[i]) XPLMSetDatavf(eln.raw[i], &value, index, 1);
}

/* ---------------------------------------------------------------- panel <-> core */
static void eln_controls_from_panel(A380XElnControls *c)
{
    int i;
    c->bat_auto[A380X_ELN_BAT1] = elec.normal[A380X_ELEC_BAT1];
    c->bat_auto[A380X_ELN_BAT2] = elec.normal[A380X_ELEC_BAT2];
    c->bat_auto[A380X_ELN_BAT_ESS] = elec.normal[A380X_ELEC_BAT_ESS];
    c->bat_auto[A380X_ELN_BAT_APU] = elec.normal[A380X_ELEC_BAT_APU];
    for (i = 0; i < 4; ++i) {
        c->gen_on[i] = elec.normal[A380X_ELEC_GEN1 + i];
        c->ext_on[i] = elec.ext_on[i];
        c->drive_disc[i] = elec.drive_disc[i];
    }
    c->apu_gen_on[0] = elec.normal[A380X_ELEC_APU_GEN_A];
    c->apu_gen_on[1] = elec.normal[A380X_ELEC_APU_GEN_B];
    c->bus_tie_auto = elec.normal[A380X_ELEC_BUS_TIE];
    c->ac_ess_feed_normal = elec.normal[A380X_ELEC_AC_ESS_FEED];
    c->galley_auto = elec.normal[A380X_ELEC_GALLEY];
    c->pax_sys_on = elec.normal[A380X_ELEC_PAX_SYS];
    c->comm_on[0] = elec.normal[A380X_ELEC_COMM1];
    c->comm_on[1] = elec.normal[A380X_ELEC_COMM2];
    c->elmu_on = elec.normal[A380X_ELEC_ELMU];
    c->rat_man_on = elec.rat_requested;
}
static void eln_controls_to_panel(const A380XElnControls *c)
{
    int i;
    elec.normal[A380X_ELEC_BAT1] = c->bat_auto[A380X_ELN_BAT1];
    elec.normal[A380X_ELEC_BAT2] = c->bat_auto[A380X_ELN_BAT2];
    elec.normal[A380X_ELEC_BAT_ESS] = c->bat_auto[A380X_ELN_BAT_ESS];
    elec.normal[A380X_ELEC_BAT_APU] = c->bat_auto[A380X_ELN_BAT_APU];
    for (i = 0; i < 4; ++i) {
        elec.normal[A380X_ELEC_GEN1 + i] = c->gen_on[i];
        elec.ext_on[i] = c->ext_on[i];
        elec.drive_disc[i] = c->drive_disc[i];
    }
    elec.normal[A380X_ELEC_APU_GEN_A] = c->apu_gen_on[0];
    elec.normal[A380X_ELEC_APU_GEN_B] = c->apu_gen_on[1];
    elec.normal[A380X_ELEC_BUS_TIE] = c->bus_tie_auto;
    elec.normal[A380X_ELEC_AC_ESS_FEED] = c->ac_ess_feed_normal;
    elec.normal[A380X_ELEC_GALLEY] = c->galley_auto;
    elec.normal[A380X_ELEC_PAX_SYS] = c->pax_sys_on;
    elec.normal[A380X_ELEC_COMM1] = c->comm_on[0];
    elec.normal[A380X_ELEC_COMM2] = c->comm_on[1];
    elec.normal[A380X_ELEC_ELMU] = c->elmu_on;
    elec.rat_requested = c->rat_man_on;
}
/* Simplified annunciator supply (not FBW, opt-in setting): any DC bus live, or a battery pb
 * AUTO while its hot bus is live. */
static int eln_simple_lamp_power(void)
{
    const uint32_t dc = A380X_ELN_BIT(A380X_ELN_DC_ESS) | A380X_ELN_BIT(A380X_ELN_DC1) |
                        A380X_ELN_BIT(A380X_ELN_DC2) | A380X_ELN_BIT(A380X_ELN_DC_APU);
    const uint32_t hot = A380X_ELN_BIT(A380X_ELN_HOT1) | A380X_ELN_BIT(A380X_ELN_HOT2) |
                         A380X_ELN_BIT(A380X_ELN_HOT_ESS) | A380X_ELN_BIT(A380X_ELN_HOT_APU);
    int bat_auto = elec.normal[A380X_ELEC_BAT1] || elec.normal[A380X_ELEC_BAT2] ||
                   elec.normal[A380X_ELEC_BAT_ESS] || elec.normal[A380X_ELEC_BAT_APU];
    return (eln.st.powered & dc) != 0 || (bat_auto && (eln.st.powered & hot) != 0);
}
static void eln_lamps_to_panel(void)
{
    const A380XElnLamps *l = &eln.st.lamps;
    int i;
    static const int bat_control[4] = {A380X_ELEC_BAT1, A380X_ELEC_BAT2, A380X_ELEC_BAT_ESS, A380X_ELEC_BAT_APU};
#define LP(control, upper, lower) do { elec_io.lamp_powered[control][0] = (unsigned char)(upper); \
                                       elec_io.lamp_powered[control][1] = (unsigned char)(lower); } while (0)
    memset(elec_io.network_fault, 0, sizeof(elec_io.network_fault));
    for (i = 0; i < 4; ++i) {
        elec_io.network_fault[bat_control[i]] = l->bat_fault[i];
        LP(bat_control[i], l->pw_bat[i], l->pw_bat[i]);
        elec_io.network_fault[A380X_ELEC_GEN1 + i] = l->gen_fault[i];
        LP(A380X_ELEC_GEN1 + i, l->pw_gen[i], l->pw_gen[i]);
        elec_io.network_fault[A380X_ELEC_DRIVE1 + i] = l->drive_fault[i];
        LP(A380X_ELEC_DRIVE1 + i, l->pw_drive[i], l->pw_drive[i]);
        LP(A380X_ELEC_EXT1 + i, l->pw_ext_avail[i], l->pw_ext_on[i]);
    }
    for (i = 0; i < 2; ++i) {
        elec_io.network_fault[A380X_ELEC_APU_GEN_A + i] = l->apu_gen_fault[i];
        LP(A380X_ELEC_APU_GEN_A + i, l->pw_apu_gen[i], l->pw_apu_gen[i]);
    }
    elec_io.network_fault[A380X_ELEC_AC_ESS_FEED] = l->ac_ess_feed_fault;
    elec_io.network_fault[A380X_ELEC_ELMU] = l->elmu_fault;
    elec_io.network_fault[A380X_ELEC_RAT_FAULT] = l->rat_fault;
    LP(A380X_ELEC_AC_ESS_FEED, l->pw_ac_ess_feed, l->pw_ac_ess_feed);
    LP(A380X_ELEC_ELMU, l->pw_elmu, l->pw_elmu);
    LP(A380X_ELEC_PAX_SYS, l->pw_pax_sys, l->pw_pax_sys);
    LP(A380X_ELEC_GALLEY, l->pw_galley, l->pw_galley);
    LP(A380X_ELEC_COMM1, l->pw_comm[0], l->pw_comm[0]);
    LP(A380X_ELEC_COMM2, l->pw_comm[1], l->pw_comm[1]);
    LP(A380X_ELEC_BUS_TIE, l->pw_bus_tie, l->pw_bus_tie);
    LP(A380X_ELEC_RAT_FAULT, l->pw_rat, l->pw_rat);
    LP(A380X_ELEC_RAT_MAN_ON, 0, 0);
    if (eln.lamp_supply_simple) {
        int simple = eln_simple_lamp_power();
        for (i = 0; i < A380X_ELEC_CONTROL_COUNT; ++i) if (i != A380X_ELEC_RAT_MAN_ON) LP(i, simple, simple);
    }
#undef LP
    elec_io.lamp_power = a380x_eln_powered(&eln.st, A380X_ELN_DC_ESS);
    elec_io.network_lamps = 1;
    elec_read_inputs();
}

/* ---------------------------------------------------------------- sim inputs */
static void eln_read_inputs(double sim_dt)
{
    A380XElnInputs *in = &eln.in;
    float n2[16] = {0};
    int i;
    in->dt = sim_dt;
    in->paused = eln_rawi(ENR_PAUSED) || eln_rawi(ENR_REPLAY);
    in->on_ground = eln_rawi(ENR_ON_GROUND) != 0;
    in->gear_compressed = in->on_ground;
    in->sim_ready = eln.since_load > 1.0;
    in->ias_kt = eln_rawf(ENR_IAS);
    /* ADR 1/3 discretes exist only with the ADR running (spec 3.1). */
    in->adr_lsw_valid = adirs.unit[0].adr_valid || adirs.unit[2].adr_valid;
    if (eln.raw[ENR_N2]) XPLMGetDatavf(eln.raw[ENR_N2], n2, 0, 4);
    for (i = 0; i < 4; ++i) in->eng_n2[i] = isfinite(n2[i]) ? n2[i] : 0.0;
    for (i = 0; i < 4; ++i) in->ext_connected[i] = (unsigned char)(elec_io.ext_connected != 0);
    in->apu_master = eln_rawi(ENR_APU_SWITCH) >= 1;
    /* FBW turbine on the ECB's last outputs (start motor = start contactor); a load with X-Plane's APU
     * already running adopts it. */
    if (eln.since_load <= 1.0 && eln.apu.state == A380X_PW980_SHUTDOWN && eln_rawi(ENR_APU_RUNNING) &&
        eln_rawf(ENR_APU_N) >= 95.0f)
        a380x_pw980_set_running(&eln.apu);
    a380x_pw980_update(&eln.apu, a380x_pw980_signal(&eln.apu, eln.st.apu_ecb_powered, in->apu_master,
                                                    a380x_eln_contactor(&eln.st, A380X_ELN_K_10KA_5KA), 0),
                       in->apu_master, eln.preset.running && eln.preset.expedited, sim_dt);
    in->apu_n = eln.apu.n;
    for (i = 0; i < 3; ++i) in->adiru_on[i] = adirs.unit[i].mode != A380X_ADIRS_OFF;
    memset(&in->fail, 0, sizeof(in->fail));
    for (i = 0; i < 4; ++i) {
        static const int bat_control[4] = {A380X_ELEC_BAT1, A380X_ELEC_BAT2, A380X_ELEC_BAT_ESS, A380X_ELEC_BAT_APU};
        in->fail.gen[i] = (unsigned char)(eln_failed(ENR_FAIL_GEN0 + i) || elec_io.fault_input[A380X_ELEC_GEN1 + i]);
        in->fail.bat[i] = (unsigned char)(eln_failed(ENR_FAIL_BAT0 + i) || elec_io.fault_input[bat_control[i]]);
        in->fail.tr[i] = eln.fail[EF_TR1 + i];
        in->fail.ac_bus[i] = (unsigned char)(eln_failed(ENR_FAIL_ESYS1 + i) || eln.fail[EF_AC_BUS1 + i]);
        in->fail.ext[i] = eln.fail[EF_EXT1 + i];
        in->fail.drive_oil[i] = (unsigned char)(elec_io.fault_input[A380X_ELEC_DRIVE1 + i] != 0);
    }
    in->fail.stat_inv = eln.fail[EF_STAT_INV];
    in->fail.dc_bus[0] = (unsigned char)(eln_failed(ENR_FAIL_ESYS5) || eln.fail[EF_DC_BUS1]);
    in->fail.dc_bus[1] = (unsigned char)(eln_failed(ENR_FAIL_ESYS6) || eln.fail[EF_DC_BUS2]);
    in->fail.dc_bus[2] = eln.fail[EF_DC_ESS_BUS];
    in->fail.apu_gen[0] = (unsigned char)(elec_io.fault_input[A380X_ELEC_APU_GEN_A] != 0);
    in->fail.apu_gen[1] = (unsigned char)(elec_io.fault_input[A380X_ELEC_APU_GEN_B] != 0);
    in->fail.elmu = (unsigned char)(elec_io.fault_input[A380X_ELEC_ELMU] != 0);
    in->fail.galley = eln.fail[EF_GALLEY];
}

/* ---------------------------------------------------------------- outputs to consumers */
static void eln_publish(void)
{
    int i;
    a380x_power.available = eln.active;
    a380x_power.bus = eln.st.powered;
    for (i = 0; i < A380X_ELN_DU_COUNT; ++i) a380x_power.du_state[i] = eln.st.du[i].state;
    a380x_power.isis_powered = (unsigned char)a380x_eln_isis_powered(&eln.st, eln.in.ias_kt);
    a380x_power.fcu_powered = (unsigned char)a380x_eln_fcu_powered(&eln.st);
    for (i = 0; i < 3; ++i) {
        int primary, backup;
        a380x_eln_adiru_supply(&eln.st, i, &primary, &backup);
        a380x_power.adiru_primary[i] = (unsigned char)primary;
        a380x_power.adiru_backup[i] = (unsigned char)backup;
    }
    a380x_power.spawn_mode = eln.mode;
}

/* Spec 9.3: X-Plane's electrical system is a slave of the native network. */
static void eln_slave_stock(void)
{
    const A380XElnState *s = &eln.st;
    int i, batteries = eln_rawi(ENR_NUM_BAT), dc = a380x_eln_powered(s, A380X_ELN_DC_ESS) ||
        a380x_eln_powered(s, A380X_ELN_DC1) || a380x_eln_powered(s, A380X_ELN_DC2);
    float max_charge = eln_rawf(ENR_CHARGE_MAX);
    if (batteries < 0) batteries = 0;
    if (batteries > 8) batteries = 8;
    for (i = 0; i < batteries; ++i) {
        eln_set_vi(ENR_BATTERY_ON, i, dc);
        if (dc && max_charge > 0.0f && eln_vf(ENR_CHARGE, i) < max_charge) eln_set_vf(ENR_CHARGE, i, max_charge);
    }
    for (i = 0; i < 4; ++i) eln_set_vi(ENR_GEN_ON, i, eln.ctl.gen_on[i] && !eln.ctl.drive_disc[i] && !eln.in.eng_fire_released[i]);
    eln_seti(ENR_APU_GEN_ON, eln.ctl.apu_gen_on[0] || eln.ctl.apu_gen_on[1]);
    /* Stock GPU only while a native EXT actually supplies (no stock bus with a dark cockpit). */
    {
        int gpu = 0;
        for (i = 0; i < 4; ++i) gpu |= a380x_eln_contactor(s, A380X_ELN_K_990XG1 + i);
        eln_seti(ENR_GPU_ON, gpu);
    }
    eln_seti(ENR_AVIONICS, a380x_eln_powered(s, A380X_ELN_DC_ESS));
    eln_seti(ENR_CROSS_TIE, 1);
    eln_set_vi(ENR_INVERTER, 0, s->stat_inv_v > 0.0f);
}

/* APU option A: X-Plane turbine, native ECB (spec 9.3). */
static void eln_apu(void)
{
    int stock = eln_rawi(ENR_APU_SWITCH), command = a380x_eln_apu_starter_command(&eln.st);
    if (eln.st.apu_emergency_shutdown) {
        eln.st.apu_emergency_shutdown = 0;
        a380x_pw980_begin_stop(&eln.apu);
        eln_seti(ENR_APU_SWITCH, 0);
        XPLMDebugString("A380X ELEC: battery power lost with the APU turning: APU emergency shutdown\n");
        return;
    }
    /* The native start contactors open at 55 % N (FBW); X-Plane's starter position is held
     * until its turbine runs on its own, then the switch returns to ON. */
    if (command == 2) eln_seti(ENR_APU_SWITCH, 2);
    else if (stock == 2 && (!eln.st.apu_start_latched || eln_rawi(ENR_APU_RUNNING) || eln_rawf(ENR_APU_N) >= 95.0f))
        eln_seti(ENR_APU_SWITCH, 1);
}

/* ---------------------------------------------------------------- spawn (spec 7.2) */
static int eln_detect_spawn(void)
{
    float n1[8] = {0};
    int i, running = 0;
    if (refs.n1) XPLMGetDatavf(refs.n1, n1, 0, 4);
    for (i = 0; i < 4; ++i) {
        float n2 = eln_vf(ENR_N2, i);
        if ((isfinite(n1[i]) && n1[i] > 15.0f) || (eln_vi(ENR_RUNNING, i) && n2 > 50.0f)) running = 1;
    }
    if (running || !eln_rawi(ENR_ON_GROUND)) return A380X_ELN_SPAWN_READY;
    return eln.default_spawn == A380X_ELN_SPAWN_TURNAROUND ? A380X_ELN_SPAWN_TURNAROUND : A380X_ELN_SPAWN_COLD_DARK;
}
static void eln_run_command(const char *name)
{
    XPLMCommandRef command = XPLMFindCommand(name);
    if (command) XPLMCommandOnce(command);
}
/* Cold & dark cockpit per spec 7.1 (stock items the plugin does not own are set once). */
static void eln_secure_cockpit(void)
{
    int i;
    eln_seti(ENR_APU_SWITCH, 0);
    for (i = 0; i < 4; ++i) eln_set_vi(ENR_FUEL_PUMP, i, 0);
    eln_seti(ENR_STROBE, 0); eln_seti(ENR_NAV, 0); eln_seti(ENR_BEACON, 0);
    eln_run_command("sim/lights/landing_lights_off"); eln_seti(ENR_TAXI, 0);   /* landing_lights_on is light 0 only */
    eln_seti(ENR_SEATBELT, 0); eln_seti(ENR_NO_SMOKING, 0); eln_seti(ENR_O2, 0); eln_seti(ENR_APU_BLEED, 0);
    if (eln.raw[ENR_PARKBRAKE]) eln_setf(ENR_PARKBRAKE, 1.0f);
    eln.nose = eln.strobe_auto = eln.no_smoking_auto = eln.rwy_turnoff = 0;
}
static void eln_apply_spawn(int mode, int secure)
{
    A380XElnControls c;
    eln.mode = mode;
    a380x_eln_spawn_controls(mode, &c);
    eln_controls_to_panel(&c);
    elec_io.ext_connected = mode == A380X_ELN_SPAWN_TURNAROUND;
    a380x_eln_init(&eln.st, mode);
    eln_read_inputs(0.0);
    eln_controls_from_panel(&eln.ctl);
    a380x_eln_settle(&eln.st, &eln.ctl, &eln.in);
    a380x_eln_du_snap(&eln.st, mode != A380X_ELN_SPAWN_COLD_DARK);
    if (secure && mode == A380X_ELN_SPAWN_COLD_DARK) eln_secure_cockpit();
    eln_publish();
    eln_lamps_to_panel();
    a380x_preset_cancel(&eln.preset);
}

/* ---------------------------------------------------------------- flyPad preset host */
static int eln_eng_running(int i) { return eln_vi(ENR_RUNNING, i) && eln_vf(ENR_N2, i) > 55.0f; }
static int eln_preset_get(void *ctx, int item, int index)
{
    float n1[8] = {0};
    (void)ctx;
    switch (item) {
    case A380X_PI_BAT: {
        static const int control[4] = {A380X_ELEC_BAT1, A380X_ELEC_BAT2, A380X_ELEC_BAT_ESS, A380X_ELEC_BAT_APU};
        return index >= 0 && index < 4 && elec.normal[control[index]];
    }
    case A380X_PI_EXT_PB: return index >= 0 && index < 4 && elec.ext_on[index];
    case A380X_PI_GPU: return elec_io.ext_connected != 0;
    case A380X_PI_APU_MASTER: return eln_rawi(ENR_APU_SWITCH) >= 1;
    case A380X_PI_APU_START: return eln.st.apu_start_latched || eln.st.apu_avail;
    case A380X_PI_APU_AVAIL: return eln.st.apu_avail;
    case A380X_PI_APU_BLEED: return eln_rawi(ENR_APU_BLEED) != 0;
    case A380X_PI_AC1_POWERED: return a380x_eln_powered(&eln.st, A380X_ELN_AC1);
    case A380X_PI_ADIRS_MODE: return index >= 0 && index < 3 ? adirs.unit[index].mode : 0;
    case A380X_PI_ADIRS_ALIGNED: return index >= 0 && index < 3 && adirs.unit[index].aligned;
    case A380X_PI_ADIRS_ALIGN_NOW: return adirs.unit[0].aligned && adirs.unit[1].aligned && adirs.unit[2].aligned;
    case A380X_PI_CREW_OXY: return eln_rawi(ENR_O2) != 0;
    case A380X_PI_STROBE: return eln_rawi(ENR_STROBE) ? 2 : eln.strobe_auto ? 1 : 0;
    case A380X_PI_NAV_LOGO: return eln_rawi(ENR_NAV) != 0;
    case A380X_PI_BEACON: return eln_rawi(ENR_BEACON) != 0;
    case A380X_PI_SEATBELTS: return eln_rawi(ENR_SEATBELT) != 0;
    case A380X_PI_NO_SMOKING: return eln_rawi(ENR_NO_SMOKING) ? (eln.no_smoking_auto ? 1 : 2) : 0;
    case A380X_PI_FUEL_PUMP: return eln_vi(ENR_FUEL_PUMP, index) != 0;
    case A380X_PI_COCKPIT_DOOR: return cockpit_door.locked != 0;
    case A380X_PI_ENG_MASTER: return eln_vf(ENR_MIXTURE, index) > 0.5f;
    case A380X_PI_ENG_RUNNING: return eln_eng_running(index);
    case A380X_PI_ENG_STOPPED:
        if (refs.n1) XPLMGetDatavf(refs.n1, n1, 0, 4);
        return index >= 0 && index < 4 && !(n1[index] >= 3.0f);
    case A380X_PI_FLAPS: return eln_rawf(ENR_FLAP_RATIO) > 0.01f;
    case A380X_PI_FLAPS_AT: {
        float handle = eln_rawf(ENR_FLAP_RATIO), deploy = eln_rawf(ENR_FLAP_DEPLOY);
        if (index != 0 || fabsf(handle - deploy) >= 0.02f) return -1;   /* travelling: no await passes */
        return handle > 0.01f;
    }
    case A380X_PI_SPOILER_ARM: {
        /* SPEED BRAKE module (docs/SPEED_BRAKE.md): X-Plane's handle carries the surface order there. */
        XPLMDataRef armed = XPLMFindDataRef("a380x/xp12/speedbrake/ground_spoilers_armed");
        return armed ? XPLMGetDatai(armed) != 0 : eln_rawf(ENR_SPEEDBRAKE) < -0.25f;
    }
    case A380X_PI_RUDDER_TRIM_ZERO: return fabsf(eln_rawf(ENR_RUDDER_TRIM)) < 0.001f;
    case A380X_PI_NOSE_LIGHT: return eln.nose != 0;
    case A380X_PI_NOSE_TO_LIGHT: return eln.nose == 2;
    case A380X_PI_RWY_TURNOFF: return eln.rwy_turnoff;
    case A380X_PI_LANDING_LIGHTS: return eln_rawi(ENR_LANDING) != 0;
    case A380X_PI_AUTOBRAKE_RTO: return center_right.rto_arm != 0;
    case A380X_PI_TERR_ND: return eln_rawi(ENR_TERR) != 0;
    case A380X_PI_TRANSPONDER: return eln_rawi(ENR_XPDR) >= 2;
    case A380X_PI_ENG_MODE_SEL: {
        XPLMDataRef sel = XPLMFindDataRef("a380x/xp12/eng_start/selector");
        return sel ? XPLMGetDatai(sel) : 1;
    }
    case A380X_PI_TO_CONFIG: return eln.to_config_held;
    default: return 0;
    }
}
static void eln_preset_set(void *ctx, int item, int index, int value)
{
    (void)ctx;
    switch (item) {
    case A380X_PI_BAT: {
        static const int control[4] = {A380X_ELEC_BAT1, A380X_ELEC_BAT2, A380X_ELEC_BAT_ESS, A380X_ELEC_BAT_APU};
        if (index >= 0 && index < 4) elec.normal[control[index]] = (unsigned char)(value != 0);  /* pb/<id> authority */
        break;
    }
    case A380X_PI_EXT_PB:
        if (index >= 0 && index < 4) elec.ext_on[index] = (unsigned char)(value && elec_inputs.ext_available[index]);
        break;
    case A380X_PI_GPU:
        if ((value != 0) != (elec_io.ext_connected != 0)) elec_gpu_connect_toggle();
        elec_read_inputs();
        break;
    case A380X_PI_APU_MASTER: eln_seti(ENR_APU_SWITCH, value ? 1 : 0); break;
    case A380X_PI_APU_START: if (value) a380x_eln_apu_start_press(&eln.st); break;
    case A380X_PI_APU_BLEED: eln_seti(ENR_APU_BLEED, value != 0); break;
    case A380X_PI_ADIRS_MODE: if (index >= 0 && index < 3) adirs_set_mode(index, value); break;
    case A380X_PI_ADIRS_ALIGN_NOW: if (value) adirs_align_now(); break;
    case A380X_PI_CREW_OXY: eln_seti(ENR_O2, value != 0); break;
    case A380X_PI_STROBE: eln.strobe_auto = value == 1; eln_seti(ENR_STROBE, value == 2); break;
    case A380X_PI_NAV_LOGO: eln_seti(ENR_NAV, value != 0); break;
    case A380X_PI_BEACON: eln_seti(ENR_BEACON, value != 0); break;
    case A380X_PI_SEATBELTS: eln_seti(ENR_SEATBELT, value != 0); break;
    case A380X_PI_NO_SMOKING: eln.no_smoking_auto = value == 1; eln_seti(ENR_NO_SMOKING, value != 0); break;
    case A380X_PI_FUEL_PUMP: eln_set_vi(ENR_FUEL_PUMP, index, value != 0); break;
    case A380X_PI_COCKPIT_DOOR: eln_run_command(value ? "a380x/xp12/cockpit_door/lock" : "a380x/xp12/cockpit_door/unlock"); break;
    case A380X_PI_ENG_MASTER:
        if (index < 0 || index > 3) break;
        if (eln.raw[ENR_MIXTURE]) eln_set_vf(ENR_MIXTURE, index, value ? 1.0f : 0.0f);  /* FADEC starts it */
        break;
    case A380X_PI_FLAPS:
        if (value && eln_rawf(ENR_FLAP_RATIO) <= 0.01f && eln.flaps_down) XPLMCommandOnce(eln.flaps_down);
        else if (!value) eln_setf(ENR_FLAP_RATIO, 0.0f);
        break;
    case A380X_PI_SPOILER_ARM: {
        /* The speed brake lever: ARM (RET, pulled up) or RET disarmed; X-Plane's handle without the module. */
        XPLMCommandRef lever = XPLMFindDataRef("a380x/xp12/speedbrake/ground_spoilers_armed")
            ? XPLMFindCommand(value ? "a380x/xp12/speedbrake/arm" : "a380x/xp12/speedbrake/retract") : NULL;
        if (lever) XPLMCommandOnce(lever);
        else eln_setf(ENR_SPEEDBRAKE, value ? -0.5f : 0.0f);
        break;
    }
    case A380X_PI_RUDDER_TRIM_ZERO: eln_setf(ENR_RUDDER_TRIM, 0.0f); break;
    case A380X_PI_NOSE_LIGHT: eln.nose = value ? (eln.nose ? eln.nose : 1) : 0; eln_seti(ENR_TAXI, eln.nose != 0); break;
    case A380X_PI_NOSE_TO_LIGHT: eln.nose = value ? 2 : (eln.nose == 2 ? 1 : eln.nose); eln_seti(ENR_TAXI, eln.nose != 0); break;
    case A380X_PI_RWY_TURNOFF: eln.rwy_turnoff = value != 0; break;
    case A380X_PI_LANDING_LIGHTS: eln_run_command(value ? "sim/lights/landing_lights_on" : "sim/lights/landing_lights_off"); break;
    case A380X_PI_AUTOBRAKE_RTO:
        if ((center_right.rto_arm != 0) != (value != 0)) eln_run_command("a380x/xp12/center_right/rto_toggle");
        break;
    case A380X_PI_TERR_ND: eln_seti(ENR_TERR, value != 0); break;
    case A380X_PI_TRANSPONDER: eln_seti(ENR_XPDR, value ? 3 : 1); break;
    case A380X_PI_ENG_MODE_SEL: {
        XPLMDataRef sel = XPLMFindDataRef("a380x/xp12/eng_start/selector");
        if (sel) XPLMSetDatai(sel, value);
        break;
    }
    case A380X_PI_TO_CONFIG:
        if (!eln.to_config) eln.to_config = XPLMFindCommand("a380x/xp12/ecam/sd/to_config");
        if (eln.to_config && (value != 0) != eln.to_config_held) {
            if (value) XPLMCommandBegin(eln.to_config); else XPLMCommandEnd(eln.to_config);
            eln.to_config_held = value != 0;
        }
        break;
    default: break;
    }
}
static const A380XPresetHost eln_preset_host = {eln_preset_get, eln_preset_set, NULL};

/* flyPad presets: ground only; COLD & DARK and TURNAROUND with the engines stopped (port rule). */
static int eln_preset_request(int preset)
{
    int i, running = 0;
    if (!eln.active) return 0;
    if (eln.preset.running && eln.preset.preset == preset) { a380x_preset_cancel(&eln.preset); return 1; }
    if (!eln_rawi(ENR_ON_GROUND)) { snprintf(eln.preset_text, sizeof(eln.preset_text), "PRESETS ARE GROUND ONLY"); return 0; }
    for (i = 0; i < 4; ++i) running |= eln_eng_running(i);
    if (running && preset <= A380X_PRESET_TURNAROUND) {
        snprintf(eln.preset_text, sizeof(eln.preset_text), "STOP THE ENGINES FIRST");
        return 0;
    }
    a380x_preset_start(&eln.preset, preset, eln.preset_expedited);
    eln.preset_last = preset;
    return eln.preset.running;
}

/* ---------------------------------------------------------------- per frame */
/* NOSE switch -> X-Plane lights: TAXI lights the taxi light, T.O. also the take-off lights (generic light
 * A380X_ELN_TO_LIGHT). A taxi light switched elsewhere (key binding) moves the switch to OFF or TAXI. */
static void eln_nose_lights(void)
{
    float to;
    int taxi = eln_rawi(ENR_TAXI) != 0;
    if (taxi != eln.nose_taxi) eln.nose = taxi ? (eln.nose ? eln.nose : 1) : 0;
    eln.nose_taxi = eln.nose != 0;
    eln_seti(ENR_TAXI, eln.nose_taxi);
    to = eln.nose == 2 ? 1.0f : 0.0f;
    if (eln.generic_lights) XPLMSetDatavf(eln.generic_lights, &to, A380X_ELN_TO_LIGHT, 1);
}
static void eln_update(float dt_real, const float *brightness)
{
    double speed, sim_dt;
    int paused;
    if (!eln.ready || !eln.active || !plugin_enabled) { a380x_power.available = 0; return; }
    elec_io.external_logic = 1;  /* a reload (panel reset) handed the panel back: take it over again */
    eln.since_load += isfinite(dt_real) && dt_real > 0.0f ? dt_real : 0.0f;
    speed = eln_rawf(ENR_SPEED);
    paused = eln_rawi(ENR_PAUSED) || eln_rawi(ENR_REPLAY);
    sim_dt = paused || !isfinite(dt_real) || dt_real < 0.0f ? 0.0 : dt_real * (isfinite(speed) && speed > 0.0 ? fmin(speed, 16.0) : 1.0);
    /* X-Plane starts the engines a frame or two after the load message (spec 7.2), and loads the aircraft's
     * plugins before it puts a new flight on the ground (Log.txt: plugins, then "Init ... ramp_start"), so
     * an "in the air" READY decided at load is re-checked: still on the ground with the engines stopped at
     * the end of the checks -> the cold & dark (or turnaround) spawn after all. */
    if (eln.spawn_checks > 0) {
        int spawn = eln_detect_spawn();
        --eln.spawn_checks;
        if (eln.mode != A380X_ELN_SPAWN_READY && spawn == A380X_ELN_SPAWN_READY) {
            eln_apply_spawn(A380X_ELN_SPAWN_READY, 0);
            elec_spawn_changed();
            XPLMDebugString("A380X ELEC: engines running after load: READY (no cold & dark)\n");
        } else if (!eln.spawn_checks && eln.mode == A380X_ELN_SPAWN_READY && spawn != A380X_ELN_SPAWN_READY) {
            eln_apply_spawn(spawn, 1);
            elec_spawn_changed();
            XPLMDebugString("A380X ELEC: on the ground with the engines stopped after load: cold & dark\n");
        }
    }
    if (eln.to_config_held && !eln.preset.running) eln_preset_set(NULL, A380X_PI_TO_CONFIG, 0, 0);  /* cancelled */
    eln_nose_lights();
    /* Ground power cannot stay plugged in once the aircraft rolls or flies (GPUM). */
    if (elec_io.ext_connected && (!eln_rawi(ENR_ON_GROUND) || eln_rawf(ENR_GROUNDSPEED) > 1.0f)) elec_io.ext_connected = 0;
    eln_read_inputs(sim_dt);
    eln_controls_from_panel(&eln.ctl);
    a380x_eln_step(&eln.st, &eln.ctl, &eln.in);
    a380x_eln_du_update(&eln.st, brightness, NULL, sim_dt, eln.selftest_s);
    eln_publish();
    eln_lamps_to_panel();
    eln_slave_stock();
    eln_apu();
    if (eln.preset.running) {
        a380x_preset_update(&eln.preset, &eln_preset_host, sim_dt);
        snprintf(eln.preset_text, sizeof(eln.preset_text), "%s", eln.preset.current ? eln.preset.current : "");
        if (!eln.preset.running && eln.preset.done) snprintf(eln.preset_text, sizeof(eln.preset_text), "%s DONE", a380x_preset_names[eln.preset.preset]);
    }
}

/* ---------------------------------------------------------------- hardware command interception */
enum { EH_BAT1, EH_BAT2, EH_GEN1, EH_GEN2, EH_GEN3, EH_GEN4, EH_GENS, EH_APU_GEN, EH_GPU, EH_APU_START, EH_AVIONICS };
enum { EHA_ON, EHA_OFF, EHA_TOGGLE };
typedef struct { const char *name; int target, action; XPLMCommandRef handle; } ElnIntercept;
static ElnIntercept eln_intercepts[] = {
    {"sim/electrical/battery_1_on", EH_BAT1, EHA_ON, NULL}, {"sim/electrical/battery_1_off", EH_BAT1, EHA_OFF, NULL},
    {"sim/electrical/battery_1_toggle", EH_BAT1, EHA_TOGGLE, NULL},
    {"sim/electrical/battery_2_on", EH_BAT2, EHA_ON, NULL}, {"sim/electrical/battery_2_off", EH_BAT2, EHA_OFF, NULL},
    {"sim/electrical/battery_2_toggle", EH_BAT2, EHA_TOGGLE, NULL},
    {"sim/electrical/generator_1_on", EH_GEN1, EHA_ON, NULL}, {"sim/electrical/generator_1_off", EH_GEN1, EHA_OFF, NULL},
    {"sim/electrical/generator_1_toggle", EH_GEN1, EHA_TOGGLE, NULL},
    {"sim/electrical/generator_2_on", EH_GEN2, EHA_ON, NULL}, {"sim/electrical/generator_2_off", EH_GEN2, EHA_OFF, NULL},
    {"sim/electrical/generator_2_toggle", EH_GEN2, EHA_TOGGLE, NULL},
    {"sim/electrical/generator_3_on", EH_GEN3, EHA_ON, NULL}, {"sim/electrical/generator_3_off", EH_GEN3, EHA_OFF, NULL},
    {"sim/electrical/generator_3_toggle", EH_GEN3, EHA_TOGGLE, NULL},
    {"sim/electrical/generator_4_on", EH_GEN4, EHA_ON, NULL}, {"sim/electrical/generator_4_off", EH_GEN4, EHA_OFF, NULL},
    {"sim/electrical/generator_4_toggle", EH_GEN4, EHA_TOGGLE, NULL},
    {"sim/electrical/generators_toggle", EH_GENS, EHA_TOGGLE, NULL},
    {"sim/electrical/APU_generator_on", EH_APU_GEN, EHA_ON, NULL}, {"sim/electrical/APU_generator_off", EH_APU_GEN, EHA_OFF, NULL},
    {"sim/electrical/GPU_on", EH_GPU, EHA_ON, NULL}, {"sim/electrical/GPU_off", EH_GPU, EHA_OFF, NULL},
    {"sim/electrical/GPU_toggle", EH_GPU, EHA_TOGGLE, NULL},
    {"sim/electrical/APU_start", EH_APU_START, EHA_ON, NULL},
    {"sim/systems/avionics_on", EH_AVIONICS, EHA_ON, NULL}, {"sim/systems/avionics_off", EH_AVIONICS, EHA_OFF, NULL},
    {"sim/systems/avionics_toggle", EH_AVIONICS, EHA_TOGGLE, NULL},
};
static void eln_set_latch(int control, int action)
{
    int value = action == EHA_ON ? 1 : action == EHA_OFF ? 0 : !elec.normal[control];
    unsigned int events;
    /* Hardware is not behind a guard: an explicit authority write with the panel's press sound. */
    if (elec.normal[control] == value) return;
    elec.normal[control] = (unsigned char)value;
    a380x_press_begin(&elec.press[control], XPLMGetElapsedTime(), 1.0f);
    if (!elec.held[control]) a380x_press_end(&elec.press[control]);
    events = A380X_ELEC_EVENT_PRESS | A380X_ELEC_EVENT_RELEASE;
    elec_play_events(events);
}
static int eln_intercept_handler(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    ElnIntercept *h = (ElnIntercept *)refcon;
    int i, any = 0;
    (void)command;
    if (!plugin_enabled || !eln.active) return 1;   /* network down: X-Plane handles it */
    if (phase != xplm_CommandBegin) return 0;
    switch (h->target) {
    case EH_BAT1: eln_set_latch(A380X_ELEC_BAT1, h->action); break;
    case EH_BAT2: eln_set_latch(A380X_ELEC_BAT2, h->action); break;
    case EH_GEN1: case EH_GEN2: case EH_GEN3: case EH_GEN4:
        eln_set_latch(A380X_ELEC_GEN1 + h->target - EH_GEN1, h->action); break;
    case EH_GENS: for (i = 0; i < 4; ++i) eln_set_latch(A380X_ELEC_GEN1 + i, EHA_TOGGLE); break;
    case EH_APU_GEN: eln_set_latch(A380X_ELEC_APU_GEN_A, h->action); eln_set_latch(A380X_ELEC_APU_GEN_B, h->action); break;
    case EH_GPU:
        for (i = 0; i < 4; ++i) any |= elec.ext_on[i];
        if (h->action == EHA_OFF || (h->action == EHA_TOGGLE && any)) memset(elec.ext_on, 0, sizeof(elec.ext_on));
        else {
            if (!elec_io.ext_connected) elec_gpu_connect_toggle();
            elec_read_inputs();
            for (i = 0; i < 4; ++i) elec.ext_on[i] = elec_inputs.ext_available[i];
        }
        break;
    case EH_APU_START: a380x_eln_apu_start_press(&eln.st); break;
    default:
        if (!eln.avionics_logged) {
            XPLMDebugString("A380X ELEC: stock avionics master commands are ignored (power follows the A380 network)\n");
            eln.avionics_logged = 1;
        }
        break;
    }
    return 0;
}

/* ---------------------------------------------------------------- published datarefs */
enum {
    EK_BUS_POWERED, EK_BUS_VOLTS, EK_CONTACTOR, EK_GEN_POT, EK_GEN_FREQ, EK_GEN_LOAD, EK_GEN_LINE,
    EK_DRIVE_CONN, EK_APU_GEN_POT, EK_APU_GEN_FREQ, EK_APU_GEN_LOAD, EK_EXT_POT, EK_EXT_FREQ,
    EK_EMER_POT, EK_EMER_LOAD, EK_RAT_STOW, EK_SI_POT, EK_SI_FREQ, EK_BAT_POT, EK_BAT_CUR, EK_BAT_AH,
    EK_BAT_SELECTOR, EK_BAT_DISPLAY, EK_TR_POT, EK_TR_CUR, EK_EMER_ACTIVE, EK_TEFO, EK_EMER_CONFIG,
    EK_GALLEY_SHED, EK_DU_STATE, EK_ISIS_POWERED, EK_SPAWN, EK_DEFAULT_SPAWN, EK_SELFTEST, EK_ACTIVE,
    EK_AC_MAIN, EK_FAIL, EK_PRESET_ACTIVE, EK_PRESET_STEP, EK_PRESET_TOTAL, EK_PRESET_TEXT,
    EK_PRESET_EXPEDITED, EK_PRESET_DONE, EK_APU_MASTER, EK_APU_START, EK_APU_AVAIL, EK_APU_FLAP,
    EK_APU_ECB, EK_APU_ALIAS_VOLTS, EK_APU_ALIAS_HZ, EK_APU_ALIAS_LOAD, EK_APU_N, EK_APU_N2, EK_NOSE_SWITCH
};
enum { ET_INT, ET_FLOAT, ET_INTV, ET_FLOATV, ET_BYTES };
typedef struct { char name[96]; int kind, index, count, type, writable; XPLMDataRef handle; } ElnRef;
static ElnRef eln_refs[192];
static int eln_ref_count;

static double eln_scalar(const ElnRef *r, int i)
{
    const A380XElnState *s = &eln.st;
    switch (r->kind) {
    case EK_BUS_POWERED: return a380x_eln_powered(s, r->index);
    case EK_BUS_VOLTS: return s->volts[r->index];
    case EK_CONTACTOR: return a380x_eln_contactor(s, r->index);
    case EK_GEN_POT: return s->gen_volts[i];
    case EK_GEN_FREQ: return s->gen_avail[i] ? s->gen_hz[i] : 0.0;
    case EK_GEN_LOAD: return s->gen_load_pct[i];
    case EK_GEN_LINE: return a380x_eln_contactor(s, A380X_ELN_K_990XU1 + i);
    case EK_DRIVE_CONN: return s->drive_connected[i];
    case EK_APU_GEN_POT: return s->apu_gen_volts[i];
    case EK_APU_GEN_FREQ: return s->apu_gen_hz[i];
    case EK_APU_GEN_LOAD: return s->apu_gen_load_pct[i];
    case EK_EXT_POT: return s->ext_volts;
    case EK_EXT_FREQ: return s->ext_hz;
    case EK_EMER_POT: return s->emer_gen_v;
    case EK_EMER_LOAD: return s->emer_gen_load_pct;
    case EK_RAT_STOW: return s->rat_stow;
    case EK_SI_POT: return s->stat_inv_v;
    case EK_SI_FREQ: return s->stat_inv_hz;
    case EK_BAT_POT: return s->bat_v[i];
    case EK_BAT_CUR: return s->bat_a[i];
    case EK_BAT_AH: return s->bat_ah[i];
    case EK_BAT_SELECTOR: return eln.bat_selector;
    case EK_NOSE_SWITCH: return eln.nose;
    case EK_BAT_DISPLAY: {
        static const int battery[5] = {A380X_ELN_BAT_ESS, A380X_ELN_BAT_APU, -1, A380X_ELN_BAT1, A380X_ELN_BAT2};
        if (elec_inputs.annunciator_test && a380x_eln_powered(s, A380X_ELN_DC2)) return 88.8;
        if (eln.bat_selector < 0 || eln.bat_selector > 4 || battery[eln.bat_selector] < 0) return -1.0;
        return roundf(s->bat_v[battery[eln.bat_selector]] * 10.0f) / 10.0f;
    }
    case EK_TR_POT: return s->tr_v[i];
    case EK_TR_CUR: return s->tr_a[i];
    case EK_EMER_ACTIVE: return s->emer_active_s > 0.0;
    case EK_TEFO: return s->tefo;
    case EK_EMER_CONFIG: return s->emer_config;
    case EK_GALLEY_SHED: return s->galley_main_shed || s->galley_sec_shed;
    case EK_DU_STATE: return s->du[r->index].state;
    case EK_ISIS_POWERED: return a380x_eln_isis_powered(s, eln.in.ias_kt);
    case EK_SPAWN: return eln.mode;
    case EK_DEFAULT_SPAWN: return eln.default_spawn;
    case EK_SELFTEST: return eln.selftest_s;
    case EK_ACTIVE: return eln.active && plugin_enabled;
    case EK_AC_MAIN: return a380x_eln_any_ac_main(s);
    case EK_FAIL: return eln.fail[r->index];
    case EK_PRESET_ACTIVE: return eln.preset.running ? eln.preset.preset : 0;
    case EK_PRESET_STEP: return eln.preset.steps_done;
    case EK_PRESET_TOTAL: return eln.preset.steps_total;
    case EK_PRESET_EXPEDITED: return eln.preset_expedited;
    case EK_PRESET_DONE: return eln.preset.done ? eln.preset.preset : 0;
    case EK_APU_MASTER: return eln_rawi(ENR_APU_SWITCH) >= 1;
    case EK_APU_START: return s->apu_start_latched;
    case EK_APU_AVAIL: return s->apu_avail;
    case EK_APU_FLAP: return s->apu_flap;
    case EK_APU_ECB: return s->apu_ecb_powered;
    case EK_APU_ALIAS_VOLTS: return fmaxf(s->apu_gen_volts[0], s->apu_gen_volts[1]);
    case EK_APU_ALIAS_HZ: return fmaxf(s->apu_gen_hz[0], s->apu_gen_hz[1]);
    case EK_APU_ALIAS_LOAD: return fmaxf(s->apu_gen_load_pct[0], s->apu_gen_load_pct[1]);
    /* Native network off: X-Plane's single-spool APU N1 for both. */
    case EK_APU_N: return eln.ready && eln.active ? eln.apu.n : eln_rawf(ENR_APU_N);
    case EK_APU_N2: return eln.ready && eln.active ? eln.apu.n2 : eln_rawf(ENR_APU_N);
    default: return 0.0;
    }
}
static int eln_get_int(void *refcon) { return (int)lround(eln_scalar((const ElnRef *)refcon, 0)); }
static float eln_get_float(void *refcon) { return (float)eln_scalar((const ElnRef *)refcon, 0); }
static double eln_get_double(void *refcon) { return eln_scalar((const ElnRef *)refcon, 0); }
static int eln_get_vf(void *refcon, float *values, int offset, int max)
{
    const ElnRef *r = (const ElnRef *)refcon;
    int i, n = 0;
    if (!values) return r->count;
    for (i = offset; i < r->count && n < max; ++i, ++n) values[n] = (float)eln_scalar(r, i);
    return n;
}
static int eln_get_vi(void *refcon, int *values, int offset, int max)
{
    const ElnRef *r = (const ElnRef *)refcon;
    int i, n = 0;
    if (!values) return r->count;
    for (i = offset; i < r->count && n < max; ++i, ++n) values[n] = (int)lround(eln_scalar(r, i));
    return n;
}
static int eln_get_bytes(void *refcon, void *values, int offset, int max)
{
    int length = (int)strlen(eln.preset_text) + 1, n;
    (void)refcon;
    if (!values) return length;
    if (offset < 0 || offset >= length || max <= 0) return 0;
    n = length - offset < max ? length - offset : max;
    memcpy(values, eln.preset_text + offset, (size_t)n);
    return n;
}
static void eln_set_scalar(ElnRef *r, int i, double value)
{
    int on = isfinite(value) && value >= 0.5;
    if (!plugin_enabled || !r->writable || !isfinite(value)) return;
    switch (r->kind) {
    case EK_BAT_AH: if (i >= 0 && i < 4) eln.st.bat_ah[i] = fmin(A380X_ELN_BAT_CAPACITY_AH, fmax(0.0, value)); break;
    case EK_BAT_SELECTOR: eln.bat_selector = (int)lround(fmin(4.0, fmax(0.0, value))); break;
    case EK_NOSE_SWITCH: eln.nose = (int)lround(fmin(2.0, fmax(0.0, value))); break;
    case EK_DEFAULT_SPAWN: eln.default_spawn = on ? A380X_ELN_SPAWN_TURNAROUND : A380X_ELN_SPAWN_COLD_DARK; break;
    case EK_SELFTEST: eln.selftest_s = (float)fmin(30.0, fmax(0.0, value)); break;
    case EK_FAIL: eln.fail[r->index] = (unsigned char)on; break;
    case EK_PRESET_EXPEDITED: eln.preset_expedited = on; break;
    default: break;
    }
}
static void eln_set_int(void *refcon, int value) { eln_set_scalar((ElnRef *)refcon, 0, value); }
static void eln_set_float(void *refcon, float value) { eln_set_scalar((ElnRef *)refcon, 0, value); }
static void eln_set_double(void *refcon, double value) { eln_set_scalar((ElnRef *)refcon, 0, value); }
static void eln_write_vf(void *refcon, float *values, int offset, int count)
{
    int i;
    for (i = 0; values && i < count; ++i) eln_set_scalar((ElnRef *)refcon, offset + i, values[i]);
}
static void eln_add(const char *name, int kind, int index, int count, int type, int writable)
{
    ElnRef *r;
    if (eln_ref_count >= (int)(sizeof(eln_refs) / sizeof(eln_refs[0]))) return;
    r = &eln_refs[eln_ref_count++];
    if (!strncmp(name, "apu/", 4)) snprintf(r->name, sizeof(r->name), "a380x/xp12/%s", name);
    else snprintf(r->name, sizeof(r->name), "a380x/xp12/elec/%s", name);
    r->kind = kind; r->index = index; r->count = count; r->type = type; r->writable = writable; r->handle = NULL;
}
static void eln_build_refs(void)
{
    char name[96];
    int i;
    eln_ref_count = 0;
    for (i = 0; i < A380X_ELN_BUS_COUNT; ++i) {
        snprintf(name, sizeof(name), "bus/%s_powered", a380x_eln_bus_names[i]); eln_add(name, EK_BUS_POWERED, i, 1, ET_INT, 0);
        snprintf(name, sizeof(name), "bus/%s_volts", a380x_eln_bus_names[i]); eln_add(name, EK_BUS_VOLTS, i, 1, ET_FLOAT, 0);
    }
    for (i = 0; i < A380X_ELN_K_COUNT; ++i) {
        snprintf(name, sizeof(name), "contactor/%s", a380x_eln_contactor_names[i]); eln_add(name, EK_CONTACTOR, i, 1, ET_INT, 0);
    }
    eln_add("gen/potential", EK_GEN_POT, 0, 4, ET_FLOATV, 0);
    eln_add("gen/freq", EK_GEN_FREQ, 0, 4, ET_FLOATV, 0);
    eln_add("gen/load_pct", EK_GEN_LOAD, 0, 4, ET_FLOATV, 0);
    eln_add("gen/line_closed", EK_GEN_LINE, 0, 4, ET_INTV, 0);
    eln_add("drive/connected", EK_DRIVE_CONN, 0, 4, ET_INTV, 0);
    eln_add("apu_gen/potential", EK_APU_GEN_POT, 0, 2, ET_FLOATV, 0);
    eln_add("apu_gen/freq", EK_APU_GEN_FREQ, 0, 2, ET_FLOATV, 0);
    eln_add("apu_gen/load_pct", EK_APU_GEN_LOAD, 0, 2, ET_FLOATV, 0);
    eln_add("ext/potential", EK_EXT_POT, 0, 1, ET_FLOAT, 0);
    eln_add("ext/freq", EK_EXT_FREQ, 0, 1, ET_FLOAT, 0);
    eln_add("emer_gen/potential", EK_EMER_POT, 0, 1, ET_FLOAT, 0);
    eln_add("emer_gen/load_pct", EK_EMER_LOAD, 0, 1, ET_FLOAT, 0);
    eln_add("rat/stow_ratio", EK_RAT_STOW, 0, 1, ET_FLOAT, 0);
    eln_add("stat_inv/potential", EK_SI_POT, 0, 1, ET_FLOAT, 0);
    eln_add("stat_inv/freq", EK_SI_FREQ, 0, 1, ET_FLOAT, 0);
    eln_add("bat/potential", EK_BAT_POT, 0, 4, ET_FLOATV, 0);
    eln_add("bat/current", EK_BAT_CUR, 0, 4, ET_FLOATV, 0);
    eln_add("bat/charge_ah", EK_BAT_AH, 0, 4, ET_FLOATV, 1);
    eln_add("bat/selector", EK_BAT_SELECTOR, 0, 1, ET_INT, 1);
    eln_add("lights/nose_switch", EK_NOSE_SWITCH, 0, 1, ET_INT, 1);   /* 0 OFF, 1 TAXI, 2 T.O. */
    eln_add("bat/display_volts", EK_BAT_DISPLAY, 0, 1, ET_FLOAT, 0);
    eln_add("tr/potential", EK_TR_POT, 0, 4, ET_FLOATV, 0);
    eln_add("tr/current", EK_TR_CUR, 0, 4, ET_FLOATV, 0);
    eln_add("emer/active", EK_EMER_ACTIVE, 0, 1, ET_INT, 0);
    eln_add("emer/tefo", EK_TEFO, 0, 1, ET_INT, 0);
    eln_add("emer/config", EK_EMER_CONFIG, 0, 1, ET_INT, 0);
    eln_add("galley_shed", EK_GALLEY_SHED, 0, 1, ET_INT, 0);
    for (i = 0; i < A380X_ELN_DU_COUNT; ++i) {
        snprintf(name, sizeof(name), "du/%s_state", a380x_eln_du_names[i]); eln_add(name, EK_DU_STATE, i, 1, ET_INT, 0);
    }
    eln_add("du/isis_powered", EK_ISIS_POWERED, 0, 1, ET_INT, 0);
    eln_add("spawn_mode", EK_SPAWN, 0, 1, ET_INT, 0);
    eln_add("settings/default_spawn", EK_DEFAULT_SPAWN, 0, 1, ET_INT, 1);
    eln_add("settings/du_selftest_s", EK_SELFTEST, 0, 1, ET_FLOAT, 1);
    eln_add("network_active", EK_ACTIVE, 0, 1, ET_INT, 0);
    eln_add("ac_main_powered", EK_AC_MAIN, 0, 1, ET_INT, 0);
    for (i = 0; i < EF_COUNT; ++i) {
        snprintf(name, sizeof(name), "fail/%s", eln_fail_names[i]); eln_add(name, EK_FAIL, i, 1, ET_INT, 1);
    }
    eln_add("preset/active", EK_PRESET_ACTIVE, 0, 1, ET_INT, 0);
    eln_add("preset/step", EK_PRESET_STEP, 0, 1, ET_INT, 0);
    eln_add("preset/step_count", EK_PRESET_TOTAL, 0, 1, ET_INT, 0);
    eln_add("preset/step_name", EK_PRESET_TEXT, 0, 1, ET_BYTES, 0);
    eln_add("preset/expedited", EK_PRESET_EXPEDITED, 0, 1, ET_INT, 1);
    eln_add("preset/done", EK_PRESET_DONE, 0, 1, ET_INT, 0);
    eln_add("apu/master_on", EK_APU_MASTER, 0, 1, ET_INT, 0);
    eln_add("apu/start_on", EK_APU_START, 0, 1, ET_INT, 0);
    eln_add("apu/avail", EK_APU_AVAIL, 0, 1, ET_INT, 0);
    eln_add("apu/flap_open_ratio", EK_APU_FLAP, 0, 1, ET_FLOAT, 0);
    eln_add("apu/ecb_powered", EK_APU_ECB, 0, 1, ET_INT, 0);
    eln_add("apu/generator_volts", EK_APU_ALIAS_VOLTS, 0, 1, ET_FLOAT, 0);
    eln_add("apu/generator_hz", EK_APU_ALIAS_HZ, 0, 1, ET_FLOAT, 0);
    eln_add("apu/generator_load_percent", EK_APU_ALIAS_LOAD, 0, 1, ET_FLOAT, 0);
    eln_add("apu/n_percent", EK_APU_N, 0, 1, ET_FLOAT, 0);
    eln_add("apu/n2_percent", EK_APU_N2, 0, 1, ET_FLOAT, 0);
}

/* ---------------------------------------------------------------- own commands */
enum { EC_PRESET, EC_PRESET_CANCEL, EC_EXPEDITE, EC_APU_START, EC_BAT_SEL_LEFT, EC_BAT_SEL_RIGHT, EC_NOSE_UP, EC_NOSE_DOWN };
typedef struct { char name[96]; int action, value; XPLMCommandRef handle; } ElnCommand;
static ElnCommand eln_commands[16];
static int eln_command_count;
static void eln_add_command(const char *name, int action, int value)
{
    ElnCommand *c;
    if (eln_command_count >= (int)(sizeof(eln_commands) / sizeof(eln_commands[0]))) return;
    c = &eln_commands[eln_command_count++];
    if (!strncmp(name, "apu/", 4)) snprintf(c->name, sizeof(c->name), "a380x/xp12/%s", name);
    else snprintf(c->name, sizeof(c->name), "a380x/xp12/elec/%s", name);
    c->action = action; c->value = value; c->handle = NULL;
}
static int eln_command_handler(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    ElnCommand *c = (ElnCommand *)refcon;
    (void)command;
    if (!plugin_enabled || !eln.active || phase != xplm_CommandBegin) return 1;
    switch (c->action) {
    case EC_PRESET: eln_preset_request(c->value); break;
    case EC_PRESET_CANCEL: a380x_preset_cancel(&eln.preset); break;
    case EC_EXPEDITE: eln.preset_expedited = !eln.preset_expedited; break;
    case EC_APU_START: a380x_eln_apu_start_press(&eln.st); break;
    case EC_BAT_SEL_LEFT: if (eln.bat_selector > 0) --eln.bat_selector; break;
    case EC_BAT_SEL_RIGHT: if (eln.bat_selector < 4) ++eln.bat_selector; break;
    case EC_NOSE_UP: if (eln.nose < 2) ++eln.nose; break;
    case EC_NOSE_DOWN: if (eln.nose > 0) --eln.nose; break;
    default: break;
    }
    return 1;
}

/* ---------------------------------------------------------------- lifecycle */
static void eln_shutdown(void)
{
    int i;
    for (i = 0; i < (int)(sizeof(eln_intercepts) / sizeof(eln_intercepts[0])); ++i) if (eln_intercepts[i].handle) {
        XPLMUnregisterCommandHandler(eln_intercepts[i].handle, eln_intercept_handler, 1, &eln_intercepts[i]);
        eln_intercepts[i].handle = NULL;
    }
    for (i = 0; i < eln_command_count; ++i) if (eln_commands[i].handle) {
        XPLMUnregisterCommandHandler(eln_commands[i].handle, eln_command_handler, 1, &eln_commands[i]);
        eln_commands[i].handle = NULL;
    }
    for (i = 0; i < eln_ref_count; ++i) if (eln_refs[i].handle) {
        XPLMUnregisterDataAccessor(eln_refs[i].handle); eln_refs[i].handle = NULL;
    }
    if (eln.active) elec_io.external_logic = 0;
    eln.ready = eln.active = 0;
    memset(&a380x_power, 0, sizeof(a380x_power));
}

/* XPluginStart, after elec_init(): never fatal. Without the panel there is no network. */
static int eln_init(void)
{
    int i;
    memset(&eln, 0, sizeof(eln));
    eln.selftest_s = (float)A380X_ELN_DU_SELFTEST_S;
    eln.bat_selector = 2;
    if (!elec_io.ready) return 0;
    for (i = 0; i < ENR_COUNT; ++i) {
        eln.raw[i] = XPLMFindDataRef(eln_raw_names[i]);
        if (!eln.raw[i] && i < ENR_REQUIRED) {
            XPLMDebugString("A380X ELEC network: missing raw input ");
            XPLMDebugString(eln_raw_names[i]);
            XPLMDebugString("\n");
            return 0;
        }
    }
    eln.flaps_down = XPLMFindCommand("sim/flight_controls/flaps_down");
    eln.generic_lights = XPLMFindDataRef("sim/cockpit2/switches/generic_lights_switch");
    eln_build_refs();
    for (i = 0; i < eln_ref_count; ++i) if (XPLMFindDataRef(eln_refs[i].name)) return 0;
    for (i = 0; i < eln_ref_count; ++i) {
        ElnRef *r = &eln_refs[i];
        void *w = r->writable ? r : NULL;
        switch (r->type) {
        case ET_INT:
            r->handle = XPLMRegisterDataAccessor(r->name, xplmType_Int, r->writable, eln_get_int, w ? eln_set_int : NULL,
                NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, r, w);
            break;
        case ET_FLOAT:
            r->handle = XPLMRegisterDataAccessor(r->name, xplmType_Float | xplmType_Double, r->writable, NULL, NULL,
                eln_get_float, w ? eln_set_float : NULL, eln_get_double, w ? eln_set_double : NULL,
                NULL, NULL, NULL, NULL, NULL, NULL, r, w);
            break;
        case ET_INTV:
            r->handle = XPLMRegisterDataAccessor(r->name, xplmType_IntArray, 0, NULL, NULL, NULL, NULL, NULL, NULL,
                eln_get_vi, NULL, NULL, NULL, NULL, NULL, r, NULL);
            break;
        case ET_FLOATV:
            r->handle = XPLMRegisterDataAccessor(r->name, xplmType_FloatArray, r->writable, NULL, NULL, NULL, NULL, NULL, NULL,
                NULL, NULL, eln_get_vf, w ? eln_write_vf : NULL, NULL, NULL, r, w);
            break;
        default:
            r->handle = XPLMRegisterDataAccessor(r->name, xplmType_Data, 0, NULL, NULL, NULL, NULL, NULL, NULL,
                NULL, NULL, NULL, NULL, eln_get_bytes, NULL, r, NULL);
            break;
        }
        if (!r->handle) { eln_shutdown(); return 0; }
    }
    eln_command_count = 0;
    for (i = 1; i <= 5; ++i) { char name[32]; snprintf(name, sizeof(name), "preset/%d", i); eln_add_command(name, EC_PRESET, i); }
    eln_add_command("preset/cancel", EC_PRESET_CANCEL, 0);
    eln_add_command("preset/expedite_toggle", EC_EXPEDITE, 0);
    eln_add_command("bat_selector_left", EC_BAT_SEL_LEFT, 0);
    eln_add_command("bat_selector_right", EC_BAT_SEL_RIGHT, 0);
    eln_add_command("lights/nose_up", EC_NOSE_UP, 0);
    eln_add_command("lights/nose_down", EC_NOSE_DOWN, 0);
    eln_add_command("apu/start", EC_APU_START, 0);
    for (i = 0; i < eln_command_count; ++i) {
        ElnCommand *c = &eln_commands[i];
        c->handle = XPLMCreateCommand(c->name, c->name);
        if (!c->handle) { eln_shutdown(); return 0; }
        XPLMRegisterCommandHandler(c->handle, eln_command_handler, 1, c);
    }
    for (i = 0; i < (int)(sizeof(eln_intercepts) / sizeof(eln_intercepts[0])); ++i) {
        ElnIntercept *h = &eln_intercepts[i];
        h->handle = XPLMFindCommand(h->name);
        if (h->handle) XPLMRegisterCommandHandler(h->handle, eln_intercept_handler, 1, h);
    }
    eln.ready = eln.active = 1;
    elec_io.external_logic = 1;
    eln_apply_spawn(eln_detect_spawn(), 1);
    eln.spawn_checks = 90;
    XPLMDebugString("A380X ELEC: native A380 network active (stock electrical slaved)\n");
    return 1;
}

/* XPLM_MSG_PLANE_LOADED, after the panel reset and before the consumers re-initialise. */
static void eln_plane_loaded(void)
{
    if (!eln.ready || !eln.active) return;
    elec_io.external_logic = 1;
    eln.since_load = 0.0;
    eln_apply_spawn(eln_detect_spawn(), 1);
    eln.spawn_checks = 90;
}
static void eln_disable(void)
{
    a380x_preset_cancel(&eln.preset);
}
