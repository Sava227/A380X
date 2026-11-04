/* SPDX-License-Identifier: GPL-3.0-or-later
 * Live FlyByWire SD page variables for the aircraft-local SD renderer (docs/SD_PAGES.md).
 *
 * The renderer runs FBW's own SD pages; each page reads FBW SimVars ("L:A32NX_...", MSFS A:
 * vars) and, for the SDv2 F/CTL page, FSComponent bus topics. This module publishes those exact
 * names into the SD frame's "vars" object (bus topics as "ev:<topic>"), computed from what the
 * port really simulates: X-Plane datarefs, and the port's own native modules through their
 * published datarefs (A380 electrical network and APU ECB, ADIRS, FMS, flight phase, surface
 * state). Every value is either live (a direct reading), derived (a documented, physically
 * consistent conversion of live data or the FBW model/constant applied to it), or absent
 * (the page draws amber XX). No failure is ever invented: a failure discrete is only ever set
 * from a port failure input; an unpowered computer reports failure-warning SSM.
 *
 * ARINC 429 words use the FBW "raw" encoding of fbw-sdk Arinc429Word: the float32 bit pattern
 * of the value in the low 32 bits plus SSM * 2^32 (discrete words: the bit field as a float).
 *
 * XPLM data access only (no globals of other modules), so test_sd_data.c can drive it with
 * mocked datarefs. Included by a380x_xpkit_sd.inc; the caller provides XpKitJson/xpkit_append.
 */
#include "a380x_engine_indications.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

enum { SDD_SSM_FW = 0, SDD_SSM_NCD = 1, SDD_SSM_FT = 2, SDD_SSM_NO = 3 };

/* STATUS page lists from the native FWS (FBW 9-digit EcamMessages codes, 0 = empty line). */
typedef struct {
    int available;                         /* FWS running: L:A32NX_FWS1/2_IS_HEALTHY */
    unsigned info[5], inop_all[10], inop_ldg[10];
    unsigned limitation_all, limitation_ldg, limitation_pfd;
} SddStatus;

typedef struct {
    double now;            /* monotonic seconds (dataref re-find pacing) */
    double idle_n1;        /* EWD/FADEC idle N1, percent */
    int idle_n1_valid;
    const SddStatus *status;               /* NULL: no FWS in this build (STATUS not published) */
} SddContext;

/* ------------------------------------------------------------------ dataref cache */
#define SDD_SLOTS 512
typedef struct { char name[96]; XPLMDataRef ref; unsigned char used; } SddSlot;
static SddSlot sdd_slots[SDD_SLOTS];
static int sdd_refind;

static uint32_t sdd_hash(const char *s)
{
    uint32_t h = 2166136261u;
    while (*s) h = (h ^ (unsigned char)*s++) * 16777619u;
    return h;
}
/* Cached XPLMFindDataRef; absent names are looked up again once per second (late producers). */
static XPLMDataRef sdd_ref(const char *name)
{
    uint32_t h = sdd_hash(name);
    size_t length = strlen(name);
    for (uint32_t probe = 0; probe < SDD_SLOTS; ++probe) {
        SddSlot *s = &sdd_slots[(h + probe) & (SDD_SLOTS - 1)];
        if (!s->used) {
            if (length >= sizeof(s->name)) return NULL;
            memcpy(s->name, name, length + 1);
            s->used = 1;
            s->ref = XPLMFindDataRef(name);
            return s->ref;
        }
        if (!strcmp(s->name, name)) {
            if (!s->ref && sdd_refind) s->ref = XPLMFindDataRef(name);
            return s->ref;
        }
    }
    return NULL;
}
/* Element `index` of an array dataref, or the scalar (index 0). 0 when absent or not finite. */
static int sdd_read(const char *name, int index, double *out)
{
    XPLMDataRef r = sdd_ref(name);
    double v;
    if (!r || index < 0) return 0;
    XPLMDataTypeID t = XPLMGetDataRefTypes(r);
    if (t & xplmType_FloatArray) {
        float f = NAN;
        if (XPLMGetDatavf(r, &f, index, 1) != 1) return 0;
        v = f;
    } else if (t & xplmType_IntArray) {
        int k = 0;
        if (XPLMGetDatavi(r, &k, index, 1) != 1) return 0;
        v = k;
    } else if (index) {
        return 0;
    } else if (t & xplmType_Double) {
        v = XPLMGetDatad(r);
    } else if (t & xplmType_Float) {
        v = XPLMGetDataf(r);
    } else if (t & xplmType_Int) {
        v = XPLMGetDatai(r);
    } else {
        return 0;
    }
    if (!isfinite(v)) return 0;
    *out = v;
    return 1;
}
static double sdd_get(const char *name, int index, double fallback)
{
    double v;
    return sdd_read(name, index, &v) ? v : fallback;
}
static double sdd_clamp(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }

/* ------------------------------------------------------------------ JSON output */
static void sdd_num(XpKitJson *j, const char *name, double v)
{
    if (isfinite(v)) xpkit_append(j, "\"%s\":%.6g,", name, v);
}
static void sdd_flag(XpKitJson *j, const char *name, int on) { xpkit_append(j, "\"%s\":%d,", name, on ? 1 : 0); }
static void sdd_uint(XpKitJson *j, const char *name, unsigned v) { xpkit_append(j, "\"%s\":%u,", name, v); }
/* fbw-sdk Arinc429Word.getRawWord(value, ssm). */
static double sdd_raw_word(double value, int ssm)
{
    float f = isfinite(value) ? (float)value : 0.0f;
    uint32_t bits;
    memcpy(&bits, &f, sizeof(bits));
    return (double)bits + (double)(ssm & 3) * 4294967296.0;
}
static void sdd_word(XpKitJson *j, const char *name, double value, int ssm)
{
    if (!isfinite(value)) { value = 0.0; if (ssm >= SDD_SSM_FT) ssm = SDD_SSM_NCD; }
    xpkit_append(j, "\"%s\":%.0f,", name, sdd_raw_word(value, ssm));
}
static void sdd_ev_num(XpKitJson *j, const char *topic, double v)
{
    if (isfinite(v)) xpkit_append(j, "\"ev:%s\":%.6g,", topic, v);
}
static void sdd_ev_bool(XpKitJson *j, const char *topic, int on)
{
    xpkit_append(j, "\"ev:%s\":%s,", topic, on ? "true" : "false");
}
static void sdd_ev_word(XpKitJson *j, const char *topic, double value, int ssm)
{
    xpkit_append(j, "\"ev:%s\":%.0f,", topic, sdd_raw_word(value, ssm));
}
#define SDD_BIT(n) (1u << ((n) - 1))

/* ------------------------------------------------------------------ port constants */
#define SDD_XP "sim/"
#define SDD_ELEC "a380x/xp12/elec/"
/* This ACF's hydraulic model runs at 3000 psi nominal (FlyWithLua a380x_hydraulic_surfaces);
 * the A380 system is 5000 psi (FBW target 5250, SD green above 2900). Displayed = scaled. */
#define SDD_HYD_SCALE (5000.0 / 3000.0)
#define SDD_HYD_SWITCH_ON 3700.0   /* FBW MIN_PRESS_PRESSURISED_HI_HYST */
#define SDD_HYD_SWITCH_OFF 2900.0  /* FBW MIN_PRESS_PRESSURISED_LO_HYST */
#define SDD_KG_PER_GAL 3.0387      /* MSFS/FBW fuel weight per gallon (6.699 lb) */
#define SDD_PRV_TARGET_PSI 40.0    /* FBW PRESSURE_REGULATING_VALVE_TARGET_PSI (FCOM) */
#define SDD_PRECOOLER_C 200.0      /* FBW fan-air-valve PID setpoint */
#define SDD_ISA_SL_PSI 14.696
#define SDD_CABIN_TARGET_C 24.0    /* FBW default CKPT/CABIN selectors (knob 150/200 -> 24 C) */
#define SDD_CARGO_TARGET_C 15.0    /* FBW default cargo regulators (knob 150 -> 15 C) */
#define SDD_ZONE_TAU_S 600.0       /* derived thermal lag, see docs/SD_PAGES.md */

static struct {
    int started;
    double last_sim_time, next_refind;
    double fuel_used_kg[4];
    int was_starting[4];
    double zone_c[18];             /* CKPT, MAIN 1..8, UPPER 1..7, CARGO FWD, BULK */
    int zone_valid;
    int hyd_switch[2];
} sdd;

static const char *const sdd_contactors[][2] = {
    {"990XU1", "990XU1"}, {"990XU2", "990XU2"}, {"990XU3", "990XU3"}, {"990XU4", "990XU4"},
    {"990XG1", "990XG1"}, {"990XG2", "990XG2"}, {"990XG3", "990XG3"}, {"990XG4", "990XG4"},
    {"990XS1", "990XS1"}, {"990XS2", "990XS2"},
    {"980XU1", "980XU1"}, {"980XU2", "980XU2"}, {"980XU3", "980XU3"}, {"980XU4", "980XU4"},
    {"980XU5", "980XU5"}, {"980XU6", "980XU6"}, {"900XU", "900XU"},
    {"3XC1", "3XC1"}, {"3XC2", "3XC2"}, {"5XE", "5XE"}, {"3XB.1", "3XB_1"}, {"3XB.2", "3XB_2"}, {"7XB", "7XB"},
    {"990PU1", "990PU1"}, {"990PU2", "990PU2"}, {"6PE", "6PE"}, {"7PU", "7PU"},
    {"990PB1", "990PB1"}, {"990PB2", "990PB2"}, {"6PC1", "6PC1"}, {"6PB3", "6PB3"}, {"5PB", "5PB"},
    {"6PC2", "6PC2"}, {"980PC", "980PC"},
};
/* FBW bus LVar -> port bus. FBW's AC_ESS_SHED is the real AC ESS 400XP and FBW's AC_ESS is the
 * real AC EMER 491XP (docs/ELEC_SYSTEM_SPEC.md 0.3). */
static const char *const sdd_buses[][2] = {
    {"L:A32NX_ELEC_AC_1_BUS_IS_POWERED", "ac1"}, {"L:A32NX_ELEC_AC_2_BUS_IS_POWERED", "ac2"},
    {"L:A32NX_ELEC_AC_3_BUS_IS_POWERED", "ac3"}, {"L:A32NX_ELEC_AC_4_BUS_IS_POWERED", "ac4"},
    {"L:A32NX_ELEC_AC_ESS_SHED_BUS_IS_POWERED", "ac_ess"}, {"L:A32NX_ELEC_AC_ESS_BUS_IS_POWERED", "ac_emer"},
    {"L:A32NX_ELEC_247XP_BUS_IS_POWERED", "ac_eha"},
    {"L:A32NX_ELEC_DC_1_BUS_IS_POWERED", "dc1"}, {"L:A32NX_ELEC_DC_2_BUS_IS_POWERED", "dc2"},
    {"L:A32NX_ELEC_DC_ESS_BUS_IS_POWERED", "dc_ess"}, {"L:A32NX_ELEC_309PP_BUS_IS_POWERED", "dc_apu"},
};
enum { SDB_AC1, SDB_AC2, SDB_AC3, SDB_AC4, SDB_AC_ESS, SDB_AC_EMER, SDB_AC_EHA, SDB_DC1, SDB_DC2,
       SDB_DC_ESS, SDB_DC_APU, SDB_COUNT };

/* Port bus state; without the native network (registration failure) every bus counts as live:
 * the legacy port has no per-bus model and no failure is invented. */
static int sdd_bus(int net, const int *bus, int id) { return !net || bus[id]; }

/* ------------------------------------------------------------------ the publisher */
static void sdd_emit(XpKitJson *j, const SddContext *ctx)
{
    char name[128];
    int i, k;
    if (ctx->now >= sdd.next_refind) { sdd_refind = 1; sdd.next_refind = ctx->now + 1.0; }

    /* ---- simulation time step (paused sim: no integration) */
    double sim_time = sdd_get(SDD_XP "time/total_running_time_sec", 0, NAN), dt = 0.0;
    if (isfinite(sim_time)) {
        if (sdd.started && sim_time >= sdd.last_sim_time && sim_time - sdd.last_sim_time < 5.0)
            dt = sim_time - sdd.last_sim_time;
        sdd.last_sim_time = sim_time;
    }
    int first = !sdd.started;
    sdd.started = 1;
    double oat = sdd_get(SDD_XP "cockpit2/temperature/outside_air_temp_degc", 0, NAN);
    int on_ground = sdd_get(SDD_XP "flightmodel/failures/onground_any", 0, 0.0) != 0.0;

    /* ---- electrical network (a380x_elec_adapter.inc publishes it) */
    int net = sdd_get(SDD_ELEC "network_active", 0, 0.0) != 0.0;
    int bus[SDB_COUNT] = {0};
    for (i = 0; i < SDB_COUNT; ++i) {
        snprintf(name, sizeof(name), SDD_ELEC "bus/%s_powered", sdd_buses[i][1]);
        bus[i] = sdd_get(name, 0, 0.0) != 0.0;
    }
    int ac_main = sdd_bus(net, bus, SDB_AC1) || sdd_bus(net, bus, SDB_AC2) ||
                  sdd_bus(net, bus, SDB_AC3) || sdd_bus(net, bus, SDB_AC4);
    int dc_avionics = sdd_bus(net, bus, SDB_DC_ESS) || sdd_bus(net, bus, SDB_DC1) || sdd_bus(net, bus, SDB_DC2);
    int computer_ssm = dc_avionics ? SDD_SSM_NO : SDD_SSM_FW;

    /* ---- engines (X-Plane two-spool model: X-Plane N2 is the HP core = FBW N3) */
    double n1[4], n3[4], ff_kgs[4];
    int running[4], starter[4], state[4], fire_pb[4], any_ignition = 0;
    for (i = 0; i < 4; ++i) {
        n1[i] = sdd_get(SDD_XP "cockpit2/engine/indicators/N1_percent", i, NAN);
        n3[i] = sdd_get(SDD_XP "cockpit2/engine/indicators/N2_percent", i, NAN);
        ff_kgs[i] = sdd_get(SDD_XP "cockpit2/engine/indicators/fuel_flow_kg_sec", i, NAN);
        running[i] = sdd_get(SDD_XP "flightmodel/engine/ENGN_running", i, 0.0) != 0.0;
        starter[i] = sdd_get(SDD_XP "flightmodel2/engines/starter_is_running", i, 0.0) != 0.0;
        fire_pb[i] = sdd_get("a380x/xp12/engine/fire_pushbutton", i, 0.0) != 0.0;
        /* FBW EngineState: OFF 0, ON 1, STARTING 2 (SHUTTING 4 is not distinguished here). */
        state[i] = starter[i] ? 2 : running[i] ? 1 : 0;
        any_ignition |= starter[i] || sdd_get(SDD_XP "cockpit2/engine/actuators/igniter_on", i, 0.0) != 0.0;
    }
    double apu_n = sdd_get(SDD_XP "cockpit2/electrical/APU_N1_percent", 0, NAN);
    int apu_master = sdd_get("a380x/xp12/apu/master_on", 0, 0.0) != 0.0;
    int apu_avail = sdd_get("a380x/xp12/apu/avail", 0, 0.0) != 0.0;
    int apu_start = sdd_get("a380x/xp12/apu/start_on", 0, 0.0) != 0.0;
    int apu_ecb = sdd_get("a380x/xp12/apu/ecb_powered", 0, 0.0) != 0.0;
    int apu_bleed_pb = sdd_get(SDD_XP "cockpit2/bleedair/actuators/apu_bleed", 0, 0.0) != 0.0;
    int apu_bleed_open = apu_bleed_pb && apu_avail;   /* FBW: valve opens on APU AVAIL with APU BLEED on */

    /* ---- pneumatics (X-Plane duct availability, FBW 40 psi regulation) */
    double avail_l = sdd_clamp(sdd_get(SDD_XP "cockpit2/bleedair/indicators/bleed_available_left", 0, 0.0), 0, 1);
    double avail_r = sdd_clamp(sdd_get(SDD_XP "cockpit2/bleedair/indicators/bleed_available_right", 0, 0.0), 0, 1);
    int xbleed_l = sdd_get(SDD_XP "cockpit2/bleedair/actuators/isol_valve_left", 0, 0.0) != 0.0;
    int xbleed_r = sdd_get(SDD_XP "cockpit2/bleedair/actuators/isol_valve_right", 0, 0.0) != 0.0;
    int prv[4], pack_on[2], pack_operating[2];
    for (i = 0; i < 4; ++i)
        prv[i] = running[i] && sdd_get(SDD_XP "cockpit2/bleedair/actuators/engine_bleed_sov", i, 0.0) != 0.0 &&
                 !apu_bleed_open;   /* FBW closes the PRVs while the APU bleed valve is open */
    pack_on[0] = sdd_get(SDD_XP "cockpit2/bleedair/actuators/pack_left", 0, 0.0) != 0.0;
    pack_on[1] = sdd_get(SDD_XP "cockpit2/bleedair/actuators/pack_right", 0, 0.0) != 0.0;
    pack_operating[0] = pack_on[0] && avail_l > 0.05 && ac_main;
    pack_operating[1] = pack_on[1] && avail_r > 0.05 && ac_main;

    /* ---- hydraulics: X-Plane system 1 = GREEN, 2 = YELLOW */
    double hyd_psi[2], hyd_level_gal[2];
    static const double reservoir_gal[2] = {12.0, 12.7};   /* FBW A380 reservoir normal fill */
    for (k = 0; k < 2; ++k) {
        hyd_psi[k] = sdd_get(k ? SDD_XP "cockpit2/hydraulics/indicators/hydraulic_pressure_2"
                               : SDD_XP "cockpit2/hydraulics/indicators/hydraulic_pressure_1", 0, NAN) * SDD_HYD_SCALE;
        hyd_level_gal[k] = sdd_get(k ? SDD_XP "cockpit2/hydraulics/indicators/hydraulic_fluid_ratio_2"
                                     : SDD_XP "cockpit2/hydraulics/indicators/hydraulic_fluid_ratio_1", 0, NAN) * reservoir_gal[k];
        if (isfinite(hyd_psi[k])) {
            if (hyd_psi[k] >= SDD_HYD_SWITCH_ON) sdd.hyd_switch[k] = 1;
            else if (hyd_psi[k] < SDD_HYD_SWITCH_OFF) sdd.hyd_switch[k] = 0;
        }
    }

    /* ================================================================ ELEC AC / ELEC DC */
    if (net) {
        for (i = 0; i < SDB_COUNT; ++i) sdd_flag(j, sdd_buses[i][0], bus[i]);
        for (i = 0; i < (int)(sizeof(sdd_contactors) / sizeof(sdd_contactors[0])); ++i) {
            char port[96];
            snprintf(port, sizeof(port), SDD_ELEC "contactor/%s", sdd_contactors[i][1]);
            snprintf(name, sizeof(name), "L:A32NX_ELEC_CONTACTOR_%s_IS_CLOSED", sdd_contactors[i][0]);
            sdd_flag(j, name, sdd_get(port, 0, 0.0) != 0.0);
        }
        for (i = 0; i < 4; ++i) {
            snprintf(name, sizeof(name), "L:A32NX_ELEC_ENG_GEN_%d_POTENTIAL", i + 1);
            sdd_num(j, name, sdd_get(SDD_ELEC "gen/potential", i, NAN));
            snprintf(name, sizeof(name), "L:A32NX_ELEC_ENG_GEN_%d_LOAD", i + 1);
            sdd_num(j, name, sdd_get(SDD_ELEC "gen/load_pct", i, NAN));
            snprintf(name, sizeof(name), "L:A32NX_ELEC_BAT_%d_POTENTIAL", i + 1);  /* 1, 2, ESS, APU */
            sdd_num(j, name, sdd_get(SDD_ELEC "bat/potential", i, NAN));
            snprintf(name, sizeof(name), "L:A32NX_ELEC_BAT_%d_CURRENT", i + 1);
            sdd_num(j, name, sdd_get(SDD_ELEC "bat/current", i, NAN));
            snprintf(name, sizeof(name), "L:A32NX_ELEC_TR_%d_POTENTIAL", i + 1);   /* TR 1, 2, ESS, APU */
            sdd_num(j, name, sdd_get(SDD_ELEC "tr/potential", i, NAN));
            snprintf(name, sizeof(name), "L:A32NX_ELEC_TR_%d_CURRENT", i + 1);
            sdd_num(j, name, sdd_get(SDD_ELEC "tr/current", i, NAN));
            char port[64];
            snprintf(name, sizeof(name), "GENERAL ENG MASTER ALTERNATOR:%d", i + 1);
            snprintf(port, sizeof(port), SDD_ELEC "pb/gen%d", i + 1);
            sdd_flag(j, name, sdd_get(port, 0, 0.0) != 0.0);
            snprintf(name, sizeof(name), "L:A32NX_EXT_PWR_AVAIL:%d", i + 1);
            snprintf(port, sizeof(port), SDD_ELEC "ext/avail%d", i + 1);
            sdd_flag(j, name, sdd_get(port, 0, 0.0) != 0.0);
        }
        for (i = 0; i < 2; ++i) {
            snprintf(name, sizeof(name), "L:A32NX_ELEC_APU_GEN_%d_POTENTIAL", i + 1);
            sdd_num(j, name, sdd_get(SDD_ELEC "apu_gen/potential", i, NAN));
            snprintf(name, sizeof(name), "L:A32NX_ELEC_APU_GEN_%d_FREQUENCY", i + 1);
            sdd_num(j, name, sdd_get(SDD_ELEC "apu_gen/freq", i, NAN));
            snprintf(name, sizeof(name), "L:A32NX_ELEC_APU_GEN_%d_LOAD", i + 1);
            sdd_num(j, name, sdd_get(SDD_ELEC "apu_gen/load_pct", i, NAN));
            snprintf(name, sizeof(name), "APU GENERATOR SWITCH:%d", i + 1);
            sdd_flag(j, name, sdd_get(i ? SDD_ELEC "pb/apu_gen_b" : SDD_ELEC "pb/apu_gen_a", 0, 0.0) != 0.0);
        }
        sdd_num(j, "L:A32NX_ELEC_EXT_PWR_POTENTIAL", sdd_get(SDD_ELEC "ext/potential", 0, NAN));
        sdd_num(j, "L:A32NX_ELEC_EXT_PWR_FREQUENCY", sdd_get(SDD_ELEC "ext/freq", 0, NAN));
        sdd_num(j, "L:A32NX_ELEC_EMER_GEN_POTENTIAL", sdd_get(SDD_ELEC "emer_gen/potential", 0, NAN));
        sdd_num(j, "L:A32NX_ELEC_EMER_GEN_LOAD", sdd_get(SDD_ELEC "emer_gen/load_pct", 0, NAN));
        sdd_num(j, "L:A32NX_ELEC_STAT_INV_POTENTIAL", sdd_get(SDD_ELEC "stat_inv/potential", 0, NAN));
        sdd_num(j, "L:A32NX_ELEC_STAT_INV_FREQUENCY", sdd_get(SDD_ELEC "stat_inv/freq", 0, NAN));
        sdd_num(j, "L:A32NX_RAT_STOW_POSITION", sdd_get(SDD_ELEC "rat/stow_ratio", 0, NAN));
        sdd_flag(j, "L:A32NX_OVHD_ELEC_BAT_1_PB_IS_AUTO", sdd_get(SDD_ELEC "pb/bat1", 0, 0.0) != 0.0);
        sdd_flag(j, "L:A32NX_OVHD_ELEC_BAT_2_PB_IS_AUTO", sdd_get(SDD_ELEC "pb/bat2", 0, 0.0) != 0.0);
        sdd_flag(j, "L:A32NX_OVHD_ELEC_BAT_ESS_PB_IS_AUTO", sdd_get(SDD_ELEC "pb/bat_ess", 0, 0.0) != 0.0);
        sdd_flag(j, "L:A32NX_OVHD_ELEC_BAT_APU_PB_IS_AUTO", sdd_get(SDD_ELEC "pb/bat_apu", 0, 0.0) != 0.0);
    }

    /* ================================================================ APU */
    sdd_flag(j, "L:A32NX_OVHD_APU_MASTER_SW_PB_IS_ON", apu_master);
    sdd_flag(j, "L:A32NX_OVHD_APU_START_PB_IS_AVAILABLE", apu_avail);
    sdd_flag(j, "L:A32NX_OVHD_PNEU_APU_BLEED_PB_IS_ON", apu_bleed_pb);
    sdd_flag(j, "L:A32NX_APU_BLEED_AIR_VALVE_OPEN", apu_bleed_open);
    /* FBW PW980: 40 psi + ISA sea-level pressure running, ambient (14.7) otherwise. */
    sdd_num(j, "L:A32NX_PNEU_APU_BLEED_CONTAINER_PRESSURE", SDD_ISA_SL_PSI + (apu_avail ? 40.0 : 0.0));
    sdd_num(j, "L:A32NX_APU_FLAP_OPEN_PERCENTAGE", 100.0 * sdd_get("a380x/xp12/apu/flap_open_ratio", 0, NAN));
    {
        int apu_ssm = apu_ecb ? SDD_SSM_NO : SDD_SSM_FW;
        double static_pa = sdd_get(SDD_XP "weather/aircraft/barometer_current_pas", 0, NAN);
        int starting = apu_start && !apu_avail;
        /* FBW ECB EGT warning: 900 C, 982 C while starting above FL250 (inlet < 5.45 psi). */
        double egt_warning = starting && isfinite(static_pa) && static_pa < 5.45 * 6894.757 ? 982.0 : 900.0;
        sdd_word(j, "L:A32NX_APU_N", apu_n, apu_ssm);
        sdd_word(j, "L:A32NX_APU_N2", apu_n, apu_ssm);   /* X-Plane APU is single-spool */
        sdd_word(j, "L:A32NX_APU_EGT", sdd_get(SDD_XP "cockpit2/electrical/APU_EGT_c", 0, NAN), apu_ssm);
        sdd_word(j, "L:A32NX_APU_EGT_WARNING", egt_warning, apu_ssm);
        sdd_word(j, "L:A32NX_APU_LOW_FUEL_PRESSURE_FAULT", 0.0, apu_ssm);
        /* No APU fuel-flow source in X-Plane: NCD (FUEL page then totals engines only). */
        sdd_word(j, "L:A32NX_APU_FUEL_USED", 0.0, SDD_SSM_NCD);
        for (k = 0; k < 2; ++k) {
            int unit = k ? 3 : 1;
            snprintf(name, sizeof(name), "a380x/xp12/adirs/adr%d_valid", unit);
            int valid = sdd_get(name, 0, 0.0) != 0.0;
            snprintf(name, sizeof(name), "L:A32NX_ADIRS_ADR_%d_CORRECTED_AVERAGE_STATIC_PRESSURE", unit);
            sdd_word(j, name, static_pa / 100.0, valid ? SDD_SSM_NO : SDD_SSM_NCD);   /* hPa */
        }
        int capt_adr = (int)sdd_get("a380x/xp12/adirs/captain_adr_source", 0, 0.0);
        int fo_adr = (int)sdd_get("a380x/xp12/adirs/first_officer_adr_source", 0, 1.0);
        sdd_num(j, "L:A32NX_ATT_HDG_SWITCHING_KNOB", capt_adr == 2 ? 0.0 : fo_adr == 2 ? 2.0 : 1.0);
        sdd_word(j, "L:A32NX_ADIRS_ADR_1_COMPUTED_AIRSPEED", sdd_get("a380x/xp12/adirs/adr1_ias_knots", 0, NAN),
                 sdd_get("a380x/xp12/adirs/adr1_valid", 0, 0.0) != 0.0 ? SDD_SSM_NO : SDD_SSM_NCD);
    }

    /* ================================================================ ENGINE */
    sdd_num(j, "L:XMLVAR_ENG_MODE_SEL", any_ignition ? 2.0 : 1.0);   /* no ENG MODE selector in the port */
    if (ctx->idle_n1_valid) sdd_num(j, "L:A32NX_ENGINE_IDLE_N1", ctx->idle_n1);
    for (i = 0; i < 4; ++i) {
        int n = i + 1;
        snprintf(name, sizeof(name), "L:A32NX_ENGINE_STATE:%d", n); sdd_num(j, name, state[i]);
        snprintf(name, sizeof(name), "L:A32NX_ENGINE_N1:%d", n); sdd_num(j, name, n1[i]);
        snprintf(name, sizeof(name), "L:A32NX_ENGINE_N3:%d", n); sdd_num(j, name, n3[i]);
        /* FBW FADEC: N2 = N3 + 0.7 ("arbitrary offset to get N2 from N3"), 0 when stopped. */
        snprintf(name, sizeof(name), "L:A32NX_ENGINE_N2:%d", n);
        sdd_num(j, name, isfinite(n3[i]) && n3[i] > 0.0 ? n3[i] + 0.7 : (isfinite(n3[i]) ? 0.0 : NAN));
        snprintf(name, sizeof(name), "L:A32NX_ENGINE_FF:%d", n); sdd_num(j, name, ff_kgs[i] * 3600.0);
        snprintf(name, sizeof(name), "L:A32NX_OVHD_FADEC_%d", n); sdd_flag(j, name, 0);
        snprintf(name, sizeof(name), "L:A32NX_FIRE_BUTTON_ENG%d", n); sdd_flag(j, name, fire_pb[i]);
        snprintf(name, sizeof(name), "L:A32NX_PNEU_ENG_%d_STARTER_VALVE_OPEN", n); sdd_flag(j, name, starter[i]);
        /* FBW oil quantity (a380x_engine_indications.h). */
        {
            double thrust_n = sdd_get(SDD_XP "flightmodel/engine/POINT_thrust", i, 0.0);
            double ratio = sdd_get(SDD_XP "cockpit2/engine/indicators/oil_quantity_ratio", i, NAN);
            snprintf(name, sizeof(name), "L:A32NX_ENGINE_OIL_QTY:%d", n);
            sdd_num(j, name, a380x_eng_oil_quantity_qt(ratio, thrust_n));
        }
        {
            double t = sdd_get(SDD_XP "cockpit2/engine/indicators/oil_temperature_deg_C", i, NAN);
            if (sdd_get(SDD_XP "aircraft/engine/acf_oilT_is_C", 0, 1.0) == 0.0) t = (t - 32.0) * 5.0 / 9.0;
            snprintf(name, sizeof(name), "GENERAL ENG OIL TEMPERATURE:%d", n); sdd_num(j, name, t);
        }
        /* FBW Polynomial_A380X::oilPressure(simN3). */
        snprintf(name, sizeof(name), "ENG OIL PRESSURE:%d", n);
        sdd_num(j, name, isfinite(n3[i]) ? fmax(0.0, -0.88921 + 0.23711 * n3[i] + 0.00682 * n3[i] * n3[i]) : NAN);
        /* X-Plane has no vibration model: a healthy engine reads 0.0. */
        snprintf(name, sizeof(name), "TURB ENG VIBRATION:%d", n); sdd_num(j, name, 0.0);
    }

    /* ================================================================ BLEED */
    sdd_flag(j, "L:A32NX_PNEU_XBLEED_VALVE_L_OPEN", xbleed_l);
    sdd_flag(j, "L:A32NX_PNEU_XBLEED_VALVE_R_OPEN", xbleed_r);
    sdd_flag(j, "L:A32NX_PNEU_XBLEED_VALVE_C_OPEN", xbleed_l && xbleed_r);
    sdd_flag(j, "L:A32NX_OVHD_COND_RAM_AIR_PB_IS_ON", 0);   /* RAM AIR pb not bound: normal (off) */
    for (i = 0; i < 4; ++i) {
        int n = i + 1;
        double manifold = SDD_PRV_TARGET_PSI * (i < 2 ? avail_l : avail_r);
        snprintf(name, sizeof(name), "L:A32NX_PNEU_ENG_%d_PR_VALVE_OPEN", n); sdd_flag(j, name, prv[i]);
        /* FBW opens the HP valve at low spool speed (IP pressure below 33.5 psi). */
        snprintf(name, sizeof(name), "L:A32NX_PNEU_ENG_%d_HP_VALVE_OPEN", n);
        sdd_flag(j, name, prv[i] && isfinite(n3[i]) && n3[i] < 80.0);
        snprintf(name, sizeof(name), "L:A32NX_PNEU_ENG_%d_REGULATED_TRANSDUCER_PRESSURE", n);
        sdd_num(j, name, manifold);
        snprintf(name, sizeof(name), "L:A32NX_PNEU_ENG_%d_PRECOOLER_OUTLET_TEMPERATURE", n);
        sdd_num(j, name, prv[i] ? SDD_PRECOOLER_C : oat);
    }
    for (k = 0; k < 2; ++k)
        for (i = 0; i < 2; ++i) {
            int open = pack_operating[k];
            snprintf(name, sizeof(name), "L:A32NX_COND_PACK_%d_FLOW_VALVE_%d_IS_OPEN", k + 1, i + 1);
            sdd_flag(j, name, open);
            snprintf(name, sizeof(name), "L:A32NX_PNEU_PACK_%d_FLOW_VALVE_%d_FLOW_RATE", k + 1, i + 1);
            sdd_num(j, name, open ? 0.5 * (k ? avail_r : avail_l) : 0.0);
        }
    for (k = 0; k < 2; ++k) {
        snprintf(name, sizeof(name), "L:A32NX_COND_PACK_%d_OUTLET_TEMPERATURE", k + 1);
        sdd_num(j, name, SDD_CABIN_TARGET_C);
        for (i = 0; i < 2; ++i) {
            snprintf(name, sizeof(name), "L:A32NX_COND_FDAC_%d_CHANNEL_%d_FAILURE", k + 1, i + 1);
            sdd_flag(j, name, 0);
        }
    }
    sdd_flag(j, "L:A32NX_LGCIU_1_LEFT_GEAR_COMPRESSED", sdd_get(SDD_XP "flightmodel2/gear/on_ground", 1, on_ground) != 0.0);

    /* ================================================================ COND (CPIOM B words, zones) */
    {
        uint32_t ags = (pack_operating[0] ? SDD_BIT(13) : 0) | (pack_operating[1] ? SDD_BIT(14) : 0);
        uint32_t tcs = (pack_operating[0] ? SDD_BIT(15) : 0) | (pack_operating[1] ? SDD_BIT(16) : 0);
        /* Cargo isolation valves and cabin fans pbs not bound (FBW default ON); fans need AC. */
        uint32_t vcs = ac_main ? (SDD_BIT(13) | SDD_BIT(14) | SDD_BIT(15) | SDD_BIT(16) | SDD_BIT(17)) : 0;
        for (k = 1; k <= 4; ++k) {
            snprintf(name, sizeof(name), "L:A32NX_COND_CPIOM_B%d_AGS_DISCRETE_WORD", k); sdd_word(j, name, ags, computer_ssm);
            snprintf(name, sizeof(name), "L:A32NX_COND_CPIOM_B%d_TCS_DISCRETE_WORD", k); sdd_word(j, name, tcs, computer_ssm);
            snprintf(name, sizeof(name), "L:A32NX_COND_CPIOM_B%d_VCS_DISCRETE_WORD", k); sdd_word(j, name, vcs, computer_ssm);
            snprintf(name, sizeof(name), "L:A32NX_COND_CPIOM_B%d_CPCS_DISCRETE_WORD", k); sdd_word(j, name, 0, computer_ssm);
        }
        sdd_flag(j, "L:A32NX_COND_TADD_CHANNEL_1_FAILURE", 0);
        sdd_flag(j, "L:A32NX_COND_TADD_CHANNEL_2_FAILURE", 0);
        sdd_flag(j, "L:A32NX_VENT_FWD_VCM_CHANNEL_1_FAILURE", 0);
        sdd_flag(j, "L:A32NX_VENT_FWD_VCM_CHANNEL_2_FAILURE", 0);
        sdd_flag(j, "L:A32NX_VENT_AFT_VCM_CHANNEL_1_FAILURE", 0);
        sdd_flag(j, "L:A32NX_VENT_AFT_VCM_CHANNEL_2_FAILURE", 0);
        sdd_flag(j, "L:A32NX_VENT_OVERPRESSURE_RELIEF_VALVE_IS_OPEN", 0);
        /* No zone thermal model in the port: each zone relaxes (tau 10 min) to the FBW default
         * selection while a pack operates, otherwise to the outside air temperature. */
        if (isfinite(oat)) {
            int packs = pack_operating[0] || pack_operating[1];
            for (k = 0; k < 18; ++k) {
                double target = packs ? (k >= 16 ? SDD_CARGO_TARGET_C : SDD_CABIN_TARGET_C) : oat;
                if (!sdd.zone_valid || first) sdd.zone_c[k] = target;
                else if (dt > 0.0) sdd.zone_c[k] += (target - sdd.zone_c[k]) * (1.0 - exp(-dt / SDD_ZONE_TAU_S));
            }
            sdd.zone_valid = 1;
        }
        if (sdd.zone_valid) {
            sdd_num(j, "L:A32NX_COND_CKPT_TEMP", sdd.zone_c[0]);
            for (k = 1; k <= 8; ++k) {
                snprintf(name, sizeof(name), "L:A32NX_COND_MAIN_DECK_%d_TEMP", k); sdd_num(j, name, sdd.zone_c[k]);
            }
            for (k = 1; k <= 7; ++k) {
                snprintf(name, sizeof(name), "L:A32NX_COND_UPPER_DECK_%d_TEMP", k); sdd_num(j, name, sdd.zone_c[8 + k]);
            }
            sdd_num(j, "L:A32NX_COND_CARGO_FWD_TEMP", sdd.zone_c[16]);
            sdd_num(j, "L:A32NX_COND_CARGO_BULK_TEMP", sdd.zone_c[17]);
        }
    }

    /* ================================================================ CAB PRESS (X-Plane pressurization) */
    {
        double cab_alt = sdd_get(SDD_XP "cockpit2/pressurization/indicators/cabin_altitude_ft", 0, NAN);
        double cab_vs = sdd_get(SDD_XP "cockpit2/pressurization/indicators/cabin_vvi_fpm", 0, NAN);
        double delta_p = sdd_get(SDD_XP "cockpit2/pressurization/indicators/pressure_diffential_psi", 0, NAN);
        double ofv = 100.0 * sdd_get(SDD_XP "cockpit2/pressurization/indicators/outflow_valve", 0, NAN);
        double alt_target = sdd_get(SDD_XP "cockpit2/pressurization/actuators/cabin_altitude_ft", 0, NAN);
        double vs_target = sdd_get(SDD_XP "cockpit2/pressurization/actuators/cabin_vvi_fpm", 0, NAN);
        for (k = 1; k <= 4; ++k) {
            snprintf(name, sizeof(name), "L:A32NX_PRESS_CABIN_ALTITUDE_B%d", k); sdd_word(j, name, cab_alt, computer_ssm);
            snprintf(name, sizeof(name), "L:A32NX_PRESS_CABIN_ALTITUDE_TARGET_B%d", k); sdd_word(j, name, alt_target, computer_ssm);
            snprintf(name, sizeof(name), "L:A32NX_PRESS_CABIN_VS_B%d", k); sdd_word(j, name, cab_vs, computer_ssm);
            snprintf(name, sizeof(name), "L:A32NX_PRESS_CABIN_VS_TARGET_B%d", k); sdd_word(j, name, vs_target, computer_ssm);
            snprintf(name, sizeof(name), "L:A32NX_PRESS_CABIN_DELTA_PRESSURE_B%d", k); sdd_word(j, name, delta_p, computer_ssm);
            for (i = 1; i <= 4; ++i) {   /* X-Plane has one outflow valve: all four show it */
                snprintf(name, sizeof(name), "L:A32NX_PRESS_OUTFLOW_VALVE_%d_OPEN_PERCENTAGE_B%d", i, k);
                sdd_word(j, name, ofv, computer_ssm);
            }
        }
        /* "_B0" is what CabinPressure reads when no CPCS word is valid; FBW never writes it and
         * MSFS returns 0 for an unset LVar (raw 0 = failure warning). */
        sdd_word(j, "L:A32NX_PRESS_CABIN_ALTITUDE_B0", 0.0, SDD_SSM_FW);
        sdd_word(j, "L:A32NX_PRESS_CABIN_ALTITUDE_TARGET_B0", 0.0, SDD_SSM_FW);
        sdd_word(j, "L:A32NX_PRESS_CABIN_VS_B0", 0.0, SDD_SSM_FW);
        sdd_word(j, "L:A32NX_PRESS_CABIN_VS_TARGET_B0", 0.0, SDD_SSM_FW);
        sdd_word(j, "L:A32NX_PRESS_CABIN_DELTA_PRESSURE_B0", 0.0, SDD_SSM_FW);
        for (i = 1; i <= 4; ++i) {
            snprintf(name, sizeof(name), "L:A32NX_PRESS_OUTFLOW_VALVE_%d_OPEN_PERCENTAGE_B0", i);
            sdd_word(j, name, 0.0, SDD_SSM_FW);
        }
        sdd_num(j, "L:A32NX_PRESS_MAN_CABIN_ALTITUDE", cab_alt);
        sdd_num(j, "L:A32NX_PRESS_MAN_CABIN_VS", cab_vs);
        sdd_num(j, "L:A32NX_PRESS_MAN_CABIN_DELTA_PRESSURE", delta_p);   /* psi (FBW asks "feet") */
        for (i = 1; i <= 4; ++i) {
            snprintf(name, sizeof(name), "L:A32NX_PRESS_MAN_OUTFLOW_VALVE_%d_OPEN_PERCENTAGE", i); sdd_num(j, name, ofv);
            snprintf(name, sizeof(name), "L:A32NX_PRESS_OCSM_%d_CHANNEL_1_FAILURE", i); sdd_flag(j, name, 0);
            snprintf(name, sizeof(name), "L:A32NX_PRESS_OCSM_%d_CHANNEL_2_FAILURE", i); sdd_flag(j, name, 0);
            snprintf(name, sizeof(name), "L:A32NX_PRESS_OCSM_%d_AUTO_PARTITION_FAILURE", i); sdd_flag(j, name, 0);
        }
        sdd_flag(j, "L:A32NX_OVHD_PRESS_MAN_ALTITUDE_PB_IS_AUTO", 1);   /* MAN pbs not bound: AUTO */
        sdd_flag(j, "L:A32NX_OVHD_PRESS_MAN_VS_CTL_PB_IS_AUTO", 1);
        double phase;
        if (sdd_read("a380x/xp12/avionics/flight_phase", 0, &phase)) sdd_num(j, "L:A32NX_FWC_FLIGHT_PHASE", phase);
        double elevation = sdd_get("a380x/xp12/fms/landing_elevation_ft", 0, NAN);
        int elevation_valid = sdd_get("a380x/xp12/fms/landing_elevation_valid", 0, 0.0) != 0.0 && isfinite(elevation);
        if (sdd_ref("a380x/xp12/fms/landing_elevation_ft")) {
            int ssm = !dc_avionics ? SDD_SSM_FW : elevation_valid ? SDD_SSM_NO : SDD_SSM_NCD;
            sdd_word(j, "L:A32NX_FM1_LANDING_ELEVATION", elevation_valid ? elevation : 0.0, ssm);
            sdd_word(j, "L:A32NX_FM2_LANDING_ELEVATION", elevation_valid ? elevation : 0.0, ssm);
        }
    }

    /* ================================================================ HYD */
    {
        static const char *const color[2] = {"GREEN", "YELLOW"};
        static const char *const epump[2][2] = {{"GA", "GB"}, {"YA", "YB"}};
        for (k = 0; k < 2; ++k) {
            snprintf(name, sizeof(name), "L:A32NX_HYD_%s_SYSTEM_1_SECTION_PRESSURE", color[k]); sdd_num(j, name, hyd_psi[k]);
            snprintf(name, sizeof(name), "L:A32NX_HYD_%s_SYSTEM_1_SECTION_PRESSURE_SWITCH", color[k]);
            sdd_flag(j, name, sdd.hyd_switch[k]);
            snprintf(name, sizeof(name), "L:A32NX_HYD_%s_RESERVOIR_LEVEL", color[k]); sdd_num(j, name, hyd_level_gal[k]);
            snprintf(name, sizeof(name), "L:A32NX_HYD_%s_RESERVOIR_LEVEL_IS_LOW", color[k]);
            sdd_flag(j, name, isfinite(hyd_level_gal[k]) && hyd_level_gal[k] < 5.0 / 3.785411784);  /* FBW 5 l */
            snprintf(name, sizeof(name), "L:A32NX_HYD_%s_RESERVOIR_AIR_PRESSURE_IS_LOW", color[k]); sdd_flag(j, name, 0);
            snprintf(name, sizeof(name), "L:A32NX_HYD_%s_RESERVOIR_OVHT", color[k]); sdd_flag(j, name, 0);
            /* Engine-driven pumps: GREEN 1/2 = ENG 1 A/B, 3/4 = ENG 2; YELLOW 1/2 = ENG 3, 3/4 = ENG 4. */
            for (i = 0; i < 4; ++i) {
                int engine = 2 * k + i / 2, b = i & 1;
                double generic = sdd_get(SDD_XP "cockpit2/hydraulics/actuators/engine_pump", engine, 0.0);
                int pb = generic != 0.0 || sdd_get(b ? SDD_XP "cockpit2/hydraulics/actuators/engine_pumpB"
                                                     : SDD_XP "cockpit2/hydraulics/actuators/engine_pumpA", engine, 0.0) != 0.0;
                snprintf(name, sizeof(name), "L:A32NX_OVHD_HYD_ENG_%d%c_PUMP_PB_IS_AUTO", engine + 1, b ? 'B' : 'A');
                sdd_flag(j, name, pb);
                snprintf(name, sizeof(name), "L:A32NX_HYD_%s_PUMP_%d_SECTION_PRESSURE_SWITCH", color[k], i + 1);
                sdd_flag(j, name, pb && isfinite(n3[engine]) && n3[engine] > 50.0 && sdd.hyd_switch[k]);
                snprintf(name, sizeof(name), "L:A32NX_HYD_%s_PUMP_%d_FIRE_VALVE_OPENED", color[k], i + 1);
                sdd_flag(j, name, !fire_pb[engine]);
            }
            /* Electric pumps A/B (PUMP 5/6): X-Plane electric pump 1 = GREEN A, pump 2 = YELLOW A. */
            for (i = 0; i < 2; ++i) {
                int active = !i && ac_main && sdd_get(k ? SDD_XP "cockpit2/hydraulics/actuators/electric_hydraulic_pump2_on"
                                                       : SDD_XP "cockpit2/hydraulics/actuators/electric_hydraulic_pump_on", 0, 0.0) != 0.0;
                snprintf(name, sizeof(name), "L:A32NX_HYD_%s_EPUMP_ACTIVE", epump[k][i]); sdd_flag(j, name, active);
                snprintf(name, sizeof(name), "L:A32NX_HYD_%s_EPUMP_OVHT", epump[k][i]); sdd_flag(j, name, 0);
                snprintf(name, sizeof(name), "L:A32NX_OVHD_HYD_EPUMP%s_OFF_PB_IS_AUTO", epump[k][i]); sdd_flag(j, name, 1);
                snprintf(name, sizeof(name), "L:A32NX_HYD_%s_PUMP_%d_SECTION_PRESSURE_SWITCH", color[k], 5 + i);
                sdd_flag(j, name, active && sdd.hyd_switch[k]);
            }
        }
        for (i = 1; i <= 4; ++i) {
            snprintf(name, sizeof(name), "L:A32NX_HYD_ENG_%dAB_PUMP_DISC", i); sdd_flag(j, name, 0);
        }
    }

    /* ================================================================ WHEEL (X-Plane gear 0 nose,
     * 1/2 left/right wing, 3/4 left/right body) */
    {
        double deploy[5], rat[5];
        int compressed[5];
        for (k = 0; k < 5; ++k) {
            deploy[k] = sdd_get(SDD_XP "flightmodel2/gear/deploy_ratio", k, NAN);
            rat[k] = sdd_clamp(sdd_get(SDD_XP "flightmodel2/gear/brake_absorbed_rat", k, 0.0), 0, 1);
            compressed[k] = sdd_get(SDD_XP "flightmodel2/gear/on_ground", k, on_ground) != 0.0;
        }
        static const char *const side[3] = {"CENTER", "LEFT", "RIGHT"};
        for (k = 0; k < 3; ++k) {
            double a = deploy[k], b = k ? deploy[k + 2] : deploy[0];
            double pos = fmin(a, b);
            int transit = (a > 0.01 && a < 0.99) || (b > 0.01 && b < 0.99);
            snprintf(name, sizeof(name), "L:A32NX_GEAR_%s_POSITION", side[k]); sdd_num(j, name, 100.0 * pos);
            /* Doors move with the gear (one animation): open while that gear is in transit. */
            if (isfinite(pos)) { snprintf(name, sizeof(name), "L:A32NX_GEAR_DOOR_%s_POSITION", side[k]); sdd_num(j, name, transit ? 100.0 : 0.0); }
        }
        sdd_flag(j, "L:A32NX_LGCIU_1_NOSE_GEAR_COMPRESSED", compressed[0]);
        sdd_flag(j, "L:A32NX_LGCIU_2_NOSE_GEAR_COMPRESSED", compressed[0]);
        /* Brake n -> X-Plane gear: wing L 1,2,5,6; wing R 3,4,7,8; body L 9,10,13,14; body R 11,12,15,16.
         * Brake temperature = OAT + 1000 C x absorbed-energy ratio (1.0 = X-Plane brake failure). */
        static const int brake_gear[16] = {1, 1, 2, 2, 1, 1, 2, 2, 3, 3, 4, 4, 3, 3, 4, 4};
        if (isfinite(oat)) {
            for (i = 0; i < 16; ++i) {
                snprintf(name, sizeof(name), "L:A32NX_REPORTED_BRAKE_TEMPERATURE_%d", i + 1);
                sdd_num(j, name, oat + 1000.0 * rat[brake_gear[i]]);
            }
        }
        /* Read by WheelPage but never written by FBW: MSFS returns 0 for an unset LVar. */
        for (i = 17; i <= 20; ++i) { snprintf(name, sizeof(name), "L:A32NX_REPORTED_BRAKE_TEMPERATURE_%d", i); sdd_num(j, name, 0.0); }
        sdd_num(j, "L:A32NX_REPORTED_BRAKE_TEMPERATURE_null", 0.0);
    }

    /* ================================================================ DOOR / OXY (door_open_ratio
     * 0..15 = M1L M1R .. M5R U1L .. U3R, 17/18 = FWD/AFT cargo, as animated by the cockpit OBJs) */
    for (k = 0; k < 16; ++k) {
        double v;
        if (!sdd_read(SDD_XP "flightmodel2/misc/door_open_ratio", k, &v)) continue;
        snprintf(name, sizeof(name), "INTERACTIVE POINT OPEN:%d", k); sdd_num(j, name, 100.0 * v);
    }
    {
        double v;
        if (sdd_read(SDD_XP "flightmodel2/misc/door_open_ratio", 17, &v)) sdd_flag(j, "L:A32NX_FWD_DOOR_CARGO_LOCKED", v < 0.01);
        if (sdd_read(SDD_XP "flightmodel2/misc/door_open_ratio", 18, &v)) sdd_flag(j, "L:A32NX_AFT_DOOR_CARGO_LOCKED", v < 0.01);
        if (sdd_read(SDD_XP "cockpit2/switches/beacon_on", 0, &v)) sdd_flag(j, "LIGHT BEACON ON", v != 0.0);
        if (sdd_read(SDD_XP "cockpit2/oxygen/actuators/o2_valve_on", 0, &v)) sdd_flag(j, "L:PUSH_OVHD_OXYGEN_CREW", v == 0.0);
        sdd_num(j, "L:CPT_SLIDING_WINDOW", 0.0);   /* sliding windows not animated: closed */
        sdd_num(j, "L:FO_SLIDING_WINDOW", 0.0);
    }

    /* ================================================================ FUEL */
    {
        /* Fuel used: FBW integrates FF (kg/h) while the sim runs and resets on a ground start. */
        for (i = 0; i < 4; ++i) {
            if (state[i] == 2 && !sdd.was_starting[i] && on_ground) sdd.fuel_used_kg[i] = 0.0;
            sdd.was_starting[i] = state[i] == 2;
            if (dt > 0.0 && isfinite(ff_kgs[i]) && ff_kgs[i] > 0.0) sdd.fuel_used_kg[i] += ff_kgs[i] * dt;
            snprintf(name, sizeof(name), "L:A32NX_FUEL_USED:%d", i + 1); sdd_num(j, name, sdd.fuel_used_kg[i]);
        }
        /* ACF tanks: 0 centre wing, 1..4 main 1..4, 5/6 reserve 2/3 (outboard), 7 horizontal stab.
         * A380 FQMS tanks: feed 2/3 hold up to the FBW feed capacity of their X-Plane main tank, the
         * rest is the inner tank (inner-to-feed transfer keeps feeds full); centre wing -> L/R mid. */
        float m[9];
        XPLMDataRef tanks = sdd_ref(SDD_XP "flightmodel/weight/m_fuel");
        int tank_count = tanks ? XPLMGetDatavf(tanks, m, 0, 9) : 0;
        if (tank_count >= 8) {
            double feed23 = 7753.2 * SDD_KG_PER_GAL;
            double q[11];
            for (i = 0; i < 8; ++i) if (!isfinite(m[i]) || m[i] < 0.0f) m[i] = 0.0f;
            q[0] = m[5]; q[1] = m[1]; q[2] = 0.5 * m[0]; q[3] = fmax(0.0, m[2] - feed23); q[4] = fmin(m[2], feed23);
            q[5] = fmin(m[3], feed23); q[6] = fmax(0.0, m[3] - feed23); q[7] = 0.5 * m[0]; q[8] = m[4]; q[9] = m[6]; q[10] = m[7];
            static const char *const tank[11] = {"LEFT_OUTER", "FEED_1", "LEFT_MID", "LEFT_INNER", "FEED_2", "FEED_3",
                                                 "RIGHT_INNER", "RIGHT_MID", "FEED_4", "RIGHT_OUTER", "TRIM"};
            for (i = 0; i < 11; ++i) {
                snprintf(name, sizeof(name), "L:A32NX_FQMS_%s_TANK_QUANTITY", tank[i]); sdd_word(j, name, q[i], computer_ssm);
                snprintf(name, sizeof(name), "L:A32NX_FQDC_1_%s_TANK_QUANTITY", tank[i]); sdd_word(j, name, q[i], computer_ssm);
                snprintf(name, sizeof(name), "L:A32NX_FQDC_2_%s_TANK_QUANTITY", tank[i]); sdd_word(j, name, q[i], computer_ssm);
            }
        }
        /* Pump running words (bits 12..21). Feed MAIN = the bound FEED pb (X-Plane engine fuel
         * pump); STBY pumps are not modelled; transfer pumps follow the mapped X-Plane tank pump. */
        uint32_t left = 0, right = 0;
        for (i = 0; i < 4; ++i)
            if (ac_main && sdd_get(SDD_XP "cockpit2/engine/actuators/fuel_pump_on", i, 0.0) != 0.0) {
                uint32_t bit = SDD_BIT(i & 1 ? 14 : 12);
                if (i < 2) left |= bit; else right |= bit;
            }
        if (ac_main) {
            static const int xp_tank[2][4] = {{5, 0, 2, 7}, {6, 0, 3, 7}};   /* outer, mid, inner, trim */
            for (k = 0; k < 2; ++k) {
                uint32_t *word = k ? &right : &left;
                if (sdd_get(SDD_XP "cockpit2/fuel/fuel_tank_pump_on", xp_tank[k][0], 0.0) != 0.0) *word |= SDD_BIT(16);
                if (sdd_get(SDD_XP "cockpit2/fuel/fuel_tank_pump_on", xp_tank[k][1], 0.0) != 0.0) *word |= SDD_BIT(17) | SDD_BIT(18);
                if (sdd_get(SDD_XP "cockpit2/fuel/fuel_tank_pump_on", xp_tank[k][2], 0.0) != 0.0) *word |= SDD_BIT(19) | SDD_BIT(20);
                if (sdd_get(SDD_XP "cockpit2/fuel/fuel_tank_pump_on", xp_tank[k][3], 0.0) != 0.0) *word |= SDD_BIT(21);
            }
        }
        sdd_word(j, "L:A32NX_FQMS_LEFT_FUEL_PUMP_RUNNING_WORD", left, computer_ssm);
        sdd_word(j, "L:A32NX_FQMS_RIGHT_FUEL_PUMP_RUNNING_WORD", right, computer_ssm);
        /* FUELSYSTEM VALVE OPEN:n (FBW flight_model.cfg numbering), 0..1. The port has no A380
         * transfer/trim/jettison model: those valves stay closed (no transfer ever happens). */
        int crossfeed = sdd_get(SDD_XP "cockpit2/fuel/fuel_crossfeed_selector", 0, 0.0) != 0.0;
        for (k = 1; k <= 59; ++k) {
            double open = 0.0;
            if (k <= 4) open = sdd_get(SDD_XP "cockpit2/engine/actuators/mixture_ratio", k - 1, 0.0) >= 0.5 && !fire_pb[k - 1];
            else if (k >= 46 && k <= 49) open = crossfeed;
            else if (k == 50 || k == 51) open = apu_master;   /* APU isolation / LP valve */
            snprintf(name, sizeof(name), "FUELSYSTEM VALVE OPEN:%d", k); sdd_num(j, name, open);
        }
    }

    /* ================================================================ F/CTL (SDv2 bus topics) */
    {
        static const char *const aileron_key[6] = {"aileron_l_inboard_ratio", "aileron_l_midboard_ratio",
            "aileron_l_outboard_ratio", "aileron_r_inboard_ratio", "aileron_r_midboard_ratio", "aileron_r_outboard_ratio"};
        static const char *const aileron_topic[6] = {"leftInnerAileronDeflection", "leftMiddleAileronDeflection",
            "leftOuterAileronDeflection", "rightInnerAileronDeflection", "rightMiddleAileronDeflection",
            "rightOuterAileronDeflection"};
        int valid = 1;
        double v;
        /* Surface state shown on the exterior (a380x_hydraulic_surfaces, ratio 1 = 30 deg; left
         * ratio + = TE down, right + = TE up: the FBW normalized convention). Fallback: the
         * A380XAileronDatarefs visual ratios / X-Plane commanded ratios. */
        for (i = 0; i < 6; ++i) {
            snprintf(name, sizeof(name), "a380x/xp12/hydraulics/%s", aileron_key[i]);
            if (!sdd_read(name, 0, &v)) {
                snprintf(name, sizeof(name), "a380x/xp12/visual/%s", aileron_key[i]);
                if (!sdd_read(name, 0, &v)) { valid = 0; continue; }
            }
            sdd_ev_num(j, aileron_topic[i], sdd_clamp(v, -1, 1));
        }
        if (sdd_read("a380x/xp12/hydraulics/elevator_ratio", 0, &v) || sdd_read(SDD_XP "flightmodel2/controls/pitch_ratio", 0, &v)) {
            v = sdd_clamp(v, -1, 1);
            sdd_ev_num(j, "leftOuterElevatorDeflection", v); sdd_ev_num(j, "leftInnerElevatorDeflection", v);
            sdd_ev_num(j, "rightInnerElevatorDeflection", v); sdd_ev_num(j, "rightOuterElevatorDeflection", v);
        } else valid = 0;
        if (sdd_read("a380x/xp12/hydraulics/rudder_ratio", 0, &v) || sdd_read(SDD_XP "flightmodel2/controls/heading_ratio", 0, &v)) {
            v = sdd_clamp(v, -1, 1);
            sdd_ev_num(j, "upperRudderDeflection", v); sdd_ev_num(j, "lowerRudderDeflection", v);
        } else valid = 0;
        /* Spoilers 1 (inboard) .. 8 (outboard) in pairs on the four exterior groups; deg / 50. */
        static const char *const group[4] = {"inboard", "inner", "middle", "outer"};
        static const char *const fallback[4] = {SDD_XP "flightmodel2/wing/speedbrake1_deg", SDD_XP "flightmodel2/wing/spoiler1_deg",
                                                SDD_XP "flightmodel2/wing/spoiler1_deg", SDD_XP "flightmodel2/wing/spoiler2_deg"};
        static const int fallback_index[4][2] = {{0, 1}, {0, 1}, {2, 3}, {4, 5}};
        for (k = 0; k < 2; ++k)
            for (i = 0; i < 4; ++i) {
                snprintf(name, sizeof(name), "a380x/xp12/hydraulics/spoiler_%c_%s_deg", k ? 'r' : 'l', group[i]);
                if (!sdd_read(name, 0, &v) && !sdd_read(fallback[i], fallback_index[i][k], &v)) { valid = 0; continue; }
                for (int p = 0; p < 2; ++p) {
                    snprintf(name, sizeof(name), "%sSpoiler%dDeflection", k ? "right" : "left", 2 * i + p + 1);
                    sdd_ev_num(j, name, sdd_clamp(v / 50.0, 0, 1.3));
                }
            }
        /* THS (A:ELEVATOR TRIM POSITION, radians, + = UP): X-Plane trim ratio x ACF stab travel. */
        double trim = sdd_get(SDD_XP "flightmodel2/controls/elevator_trim", 0, NAN);
        double up = sdd_get(SDD_XP "aircraft/controls/acf_hstb_trim_up", 0, NAN);
        double dn = sdd_get(SDD_XP "aircraft/controls/acf_hstb_trim_dn", 0, NAN);
        if (isfinite(trim) && isfinite(up) && isfinite(dn)) sdd_ev_num(j, "thsDeflection", trim * (trim >= 0 ? up : dn) * (3.14159265358979323846 / 180.0));
        else valid = 0;
        sdd_ev_num(j, "fctlInfoValid", valid ? 1.0 : 0.0);
        /* Computers: healthy unless the port's PRIM/SEC failure input is set; need DC power. */
        int sec_ok[3];
        for (i = 1; i <= 3; ++i) {
            snprintf(name, sizeof(name), "a380x/xp12/ovhd/fail/prim%d", i);
            int ok = dc_avionics && sdd_get(name, 0, 0.0) == 0.0;
            snprintf(name, sizeof(name), "prim%dHealthy", i); sdd_ev_bool(j, name, ok);
            snprintf(name, sizeof(name), "a380x/xp12/ovhd/fail/sec%d", i);
            sec_ok[i - 1] = dc_avionics && sdd_get(name, 0, 0.0) == 0.0;
            snprintf(name, sizeof(name), "sec%dHealthy", i); sdd_ev_bool(j, name, sec_ok[i - 1]);
        }
        /* SEC rudder status word: bit 27 trim available, bit 28 trim engaged (SEC 1 first). */
        int engaged1 = sec_ok[0], engaged3 = !sec_ok[0] && sec_ok[2];
        sdd_ev_word(j, "sec1RudderStatusWord", sec_ok[0] ? SDD_BIT(27) | (engaged1 ? SDD_BIT(28) : 0) : 0,
                    sec_ok[0] ? SDD_SSM_NO : SDD_SSM_FW);
        sdd_ev_word(j, "sec3RudderStatusWord", sec_ok[2] ? SDD_BIT(27) | (engaged3 ? SDD_BIT(28) : 0) : 0,
                    sec_ok[2] ? SDD_SSM_NO : SDD_SSM_FW);
        /* Rudder trim (deg, + = LEFT): X-Plane trim ratio x ACF trim limit x 30 deg rudder travel. */
        double rudder_trim = sdd_get(SDD_XP "flightmodel2/controls/rudder_trim", 0, NAN);
        double limit = rudder_trim >= 0 ? sdd_get(SDD_XP "aircraft/controls/acf_max_trim_rudd", 0, 1.0)
                                        : sdd_get(SDD_XP "aircraft/controls/acf_min_trim_rudd", 0, 1.0);
        double trim_deg = -rudder_trim * fabs(limit) * 30.0;
        int trim_ok = isfinite(trim_deg);
        sdd_ev_word(j, "sec1RudderTrimPosition", trim_ok ? trim_deg : 0.0, sec_ok[0] && trim_ok ? SDD_SSM_NO : SDD_SSM_FW);
        sdd_ev_word(j, "sec3RudderTrimPosition", trim_ok ? trim_deg : 0.0, sec_ok[2] && trim_ok ? SDD_SSM_NO : SDD_SSM_FW);
        sdd_ev_bool(j, "greenPressureSwitch", sdd.hyd_switch[0]);
        sdd_ev_bool(j, "yellowPressureSwitch", sdd.hyd_switch[1]);
        sdd_ev_bool(j, "acEssPowered", sdd_bus(net, bus, SDB_AC_EMER));   /* FBW AC_ESS = AC EMER 491XP */
        sdd_ev_bool(j, "ac1Powered", sdd_bus(net, bus, SDB_AC1));
        sdd_ev_bool(j, "acEhaPowered", sdd_bus(net, bus, SDB_AC_EHA));
        sdd_ev_bool(j, "dc1Powered", sdd_bus(net, bus, SDB_DC1));
        sdd_ev_bool(j, "dcEssPowered", sdd_bus(net, bus, SDB_DC_ESS));
        sdd_ev_bool(j, "lgciu1LeftGearCompressed", sdd_get(SDD_XP "flightmodel2/gear/on_ground", 1, on_ground) != 0.0);
    }

    /* ================================================================ STATUS (native FWS lists) */
    if (ctx->status) {
        const SddStatus *st = ctx->status;
        sdd_flag(j, "L:A32NX_FWS1_IS_HEALTHY", st->available);
        sdd_flag(j, "L:A32NX_FWS2_IS_HEALTHY", st->available);
        for (k = 0; k < 5; ++k) {
            snprintf(name, sizeof(name), "L:A32NX_SD_STATUS_INFO_LINE_%d", k + 1); sdd_uint(j, name, st->available ? st->info[k] : 0u);
        }
        for (k = 0; k < 10; ++k) {
            snprintf(name, sizeof(name), "L:A32NX_SD_STATUS_INOP_ALL_LINE_%d", k + 1); sdd_uint(j, name, st->available ? st->inop_all[k] : 0u);
            snprintf(name, sizeof(name), "L:A32NX_SD_STATUS_INOP_LDG_LINE_%d", k + 1); sdd_uint(j, name, st->available ? st->inop_ldg[k] : 0u);
        }
        sdd_uint(j, "L:A32NX_EWD_LIMITATIONS_ALL_LINE_1", st->available ? st->limitation_all : 0u);
        sdd_uint(j, "L:A32NX_EWD_LIMITATIONS_LDG_LINE_1", st->available ? st->limitation_ldg : 0u);
        sdd_uint(j, "L:A32NX_PFD_LIMITATIONS_LINE_1", st->available ? st->limitation_pfd : 0u);
    }

    /* ================================================================ permanent data (FBW SDv2
     * PermanentData: TAT/SAT/ISA, G LOAD, GW/GWCG/FOB; the clock uses sim/time/zulu_time_sec) */
    {
        /* FBW's switching publisher reads the captain-selected ADR under the "_1_" names. */
        int unit = (int)sdd_get("a380x/xp12/adirs/captain_adr_source", 0, 0.0) + 1;
        if (unit < 1 || unit > 3) unit = 1;
        snprintf(name, sizeof(name), "a380x/xp12/adirs/adr%d_valid", unit);
        int adr_ssm = sdd_get(name, 0, 0.0) != 0.0 ? SDD_SSM_NO : SDD_SSM_FW;
        snprintf(name, sizeof(name), "a380x/xp12/adirs/adr%d_sat_c", unit);
        sdd_word(j, "L:A32NX_ADIRS_ADR_1_STATIC_AIR_TEMPERATURE", sdd_get(name, 0, NAN), adr_ssm);
        snprintf(name, sizeof(name), "a380x/xp12/adirs/adr%d_tat_c", unit);
        sdd_word(j, "L:A32NX_ADIRS_ADR_1_TOTAL_AIR_TEMPERATURE", sdd_get(name, 0, NAN), adr_ssm);
        snprintf(name, sizeof(name), "a380x/xp12/adirs/adr%d_altitude_ft", unit);
        sdd_word(j, "L:A32NX_ADIRS_ADR_1_ALTITUDE", sdd_get(name, 0, NAN), adr_ssm);
        unit = (int)sdd_get("a380x/xp12/adirs/captain_ir_source", 0, 0.0) + 1;
        if (unit < 1 || unit > 3) unit = 1;
        snprintf(name, sizeof(name), "a380x/xp12/adirs/ir%d_attitude_valid", unit);
        sdd_word(j, "L:A32NX_ADIRS_IR_1_BODY_NORMAL_ACC", sdd_get(SDD_XP "flightmodel/forces/g_nrml", 0, NAN),
                 sdd_get(name, 0, 0.0) != 0.0 ? SDD_SSM_NO : SDD_SSM_FW);
        /* FCU EFIS discrete word 2, bit 11 = baro STD (ISA line). */
        sdd_word(j, "L:A32NX_FCU_EFIS_L_DISCRETE_WORD_2",
                 sdd_get("a380x/xp12/efis/captain/baro_std", 0, 0.0) != 0.0 ? SDD_BIT(11) : 0, computer_ssm);
        sdd_word(j, "L:A32NX_FCU_EFIS_R_DISCRETE_WORD_2",
                 sdd_get("a380x/xp12/efis/first_officer/baro_std", 0, 0.0) != 0.0 ? SDD_BIT(11) : 0, computer_ssm);
        sdd_word(j, "L:A32NX_FQMS_TOTAL_FUEL_ON_BOARD", sdd_get(SDD_XP "flightmodel/weight/m_fuel_total", 0, NAN), computer_ssm);
        sdd_word(j, "L:A32NX_FQMS_GROSS_WEIGHT", sdd_get(SDD_XP "flightmodel/weight/m_total", 0, NAN), computer_ssm);
        /* X-Plane publishes no gross-weight CG in %MAC: no computed data ("--", FBW). */
        sdd_word(j, "L:A32NX_FQMS_CENTER_OF_GRAVITY_MAC", 0.0, dc_avionics ? SDD_SSM_NCD : SDD_SSM_FW);
    }
    sdd_refind = 0;
}
