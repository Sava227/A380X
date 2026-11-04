/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 electrical network: simulator-independent core (no XPLM, no GL).
 *
 * Reproduces the FlyByWire A380X Rust electrical model (GPL-3.0) as specified in
 * docs/ELEC_SYSTEM_SPEC.md: main sources and the AC 1-4 priority scan, BUS TIE / system
 * isolation, AC ESS FEED, emergency generator (RAT), AC EMER / static inverter, BCRU 1/2/ESS
 * and TR APU, battery contactors and ELC, DC ties, 108PH in-flight sub-bus, TEFO / EMER
 * CONFIG relays, galley shedding, battery charge model, the FBW CDS display-unit state
 * machine and the APU electronic control box (MASTER / START / flap / AVAIL).
 *
 * Conductive elements are joined by a union-find graph in the FBW statement order
 * ("electricity.flow(a, b)"), so a bus is powered exactly when a closed path reaches a
 * supplying source. Names follow the real A380 (see spec 0.3: FBW's AC_ESS_SHED is the
 * real AC ESS 400XP, FBW's AC_ESS is AC EMER 491XP).
 *
 * The XPLM boundary is a380x_elec_adapter.inc; consumers query a380x_elec_power.h.
 */
#ifndef A380X_ELEC_H
#define A380X_ELEC_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- buses (spec 2.2) */
typedef enum {
    A380X_ELN_AC1, A380X_ELN_AC2, A380X_ELN_AC3, A380X_ELN_AC4,
    A380X_ELN_AC_ESS,        /* 400XP */
    A380X_ELN_AC_EMER,       /* 491XP */
    A380X_ELN_AC_EHA,        /* 247XP */
    A380X_ELN_AC_GND_SVC,
    A380X_ELN_DC1, A380X_ELN_DC2,
    A380X_ELN_DC_ESS,        /* 400PP */
    A380X_ELN_DC_ESS_INFLT,  /* 108PH */
    A380X_ELN_DC_EHA,        /* 247PP */
    A380X_ELN_DC_APU,        /* 309PP */
    A380X_ELN_APU_START,     /* 49-42-00 start motor sub-bus */
    A380X_ELN_HOT1, A380X_ELN_HOT2, A380X_ELN_HOT_ESS, A380X_ELN_HOT_APU,
    A380X_ELN_DC_GND_SVC,    /* 260PP */
    A380X_ELN_BUS_COUNT
} A380XElnBus;
#define A380X_ELN_BIT(bus) (1u << (unsigned)(bus))
extern const char *const a380x_eln_bus_names[A380X_ELN_BUS_COUNT];

/* ---------------------------------------------------------------- contactors (spec 2.3) */
typedef enum {
    A380X_ELN_K_990XU1, A380X_ELN_K_990XU2, A380X_ELN_K_990XU3, A380X_ELN_K_990XU4,
    A380X_ELN_K_990XG1, A380X_ELN_K_990XG2, A380X_ELN_K_990XG3, A380X_ELN_K_990XG4,
    A380X_ELN_K_990XS1, A380X_ELN_K_990XS2,
    A380X_ELN_K_980XU1, A380X_ELN_K_980XU2, A380X_ELN_K_980XU3, A380X_ELN_K_980XU4,
    A380X_ELN_K_980XU5, A380X_ELN_K_980XU6,
    A380X_ELN_K_900XU, A380X_ELN_K_3XC1, A380X_ELN_K_3XC2, A380X_ELN_K_5XE,
    A380X_ELN_K_3XB1, A380X_ELN_K_3XB2, A380X_ELN_K_911XN, A380X_ELN_K_911XH,
    A380X_ELN_K_991PU6, A380X_ELN_K_991PU2,
    A380X_ELN_K_990PU1, A380X_ELN_K_990PU2, A380X_ELN_K_6PE, A380X_ELN_K_7PU,
    A380X_ELN_K_990PB1, A380X_ELN_K_990PB2, A380X_ELN_K_6PC1, A380X_ELN_K_6PB3, A380X_ELN_K_5PB,
    A380X_ELN_K_6PC2, A380X_ELN_K_980PC, A380X_ELN_K_7XB, A380X_ELN_K_970PN2, A380X_ELN_K_14PH,
    A380X_ELN_K_990PX, A380X_ELN_K_970PN, A380X_ELN_K_10PH, A380X_ELN_K_10KA_5KA,
    A380X_ELN_K_COUNT
} A380XElnContactor;
extern const char *const a380x_eln_contactor_names[A380X_ELN_K_COUNT];

/* ---------------------------------------------------------------- source origins (bitmask) */
typedef enum {
    A380X_ELN_SRC_GEN1, A380X_ELN_SRC_GEN2, A380X_ELN_SRC_GEN3, A380X_ELN_SRC_GEN4,
    A380X_ELN_SRC_APU_A, A380X_ELN_SRC_APU_B,
    A380X_ELN_SRC_EXT1, A380X_ELN_SRC_EXT2, A380X_ELN_SRC_EXT3, A380X_ELN_SRC_EXT4,
    A380X_ELN_SRC_EMER_GEN, A380X_ELN_SRC_STAT_INV,
    A380X_ELN_SRC_BAT1, A380X_ELN_SRC_BAT2, A380X_ELN_SRC_BAT_ESS, A380X_ELN_SRC_BAT_APU,
    A380X_ELN_SRC_TR1, A380X_ELN_SRC_TR2, A380X_ELN_SRC_TR_ESS, A380X_ELN_SRC_TR_APU,
    A380X_ELN_SRC_COUNT
} A380XElnSource;

/* Battery index: FBW numbering 1, 2, ESS (3), APU (4). */
enum { A380X_ELN_BAT1, A380X_ELN_BAT2, A380X_ELN_BAT_ESS, A380X_ELN_BAT_APU, A380X_ELN_BATTERIES };
/* BCRU / TR index: TR 1, TR 2, TR ESS (BCRUs) and TR APU. */
enum { A380X_ELN_TR1, A380X_ELN_TR2, A380X_ELN_TR_ESS, A380X_ELN_TR_APU, A380X_ELN_TRS };

/* ---------------------------------------------------------------- display units (spec 6.1) */
typedef enum {
    A380X_ELN_DU_CAPT_PFD, A380X_ELN_DU_CAPT_ND, A380X_ELN_DU_CAPT_MFD,
    A380X_ELN_DU_FO_PFD, A380X_ELN_DU_FO_ND, A380X_ELN_DU_FO_MFD,
    A380X_ELN_DU_EWD, A380X_ELN_DU_SD, A380X_ELN_DU_CAPT_OIT, A380X_ELN_DU_FO_OIT,
    A380X_ELN_DU_COUNT
} A380XElnDuId;
extern const char *const a380x_eln_du_names[A380X_ELN_DU_COUNT];
/* Published state values (spec 10.3). */
typedef enum {
    A380X_ELN_DU_OFF = 0, A380X_ELN_DU_BOOT = 1, A380X_ELN_DU_SELFTEST = 2,
    A380X_ELN_DU_ON = 3, A380X_ELN_DU_STANDBY = 4
} A380XElnDuState;

/* ---------------------------------------------------------------- spawn (spec 7.2) */
typedef enum { A380X_ELN_SPAWN_COLD_DARK = 0, A380X_ELN_SPAWN_TURNAROUND = 1, A380X_ELN_SPAWN_READY = 2 } A380XElnSpawn;

/* ---------------------------------------------------------------- named constants
 * H: from FBW; M: FBW simplification / public data; L: assumption (spec section 12). */
#define A380X_ELN_VFG_STABLE_S 0.5          /* H EGEN */
#define A380X_ELN_VFG_HZ_MIN 360.0          /* H */
#define A380X_ELN_VFG_HZ_MAX 800.0          /* H */
/* VFG frequency = gearbox rpm * 1.95 * 4 / 120 (EGEN DirectDrive). FBW's Trent 900 gearbox runs on
 * the HP spool, 100 % = 12200 rpm (trent_engine.rs); X-Plane's N2 is that spool, so 7.93 Hz per %
 * (H) and 793 Hz at 100 %. X-Plane's spool model overshoots 100 % slightly at TOGA, which a real
 * Trent does not: the N2 used here is capped at 100 % (L), or all four VFGs left 360-800 Hz together
 * during the take-off roll. */
#define A380X_ELN_VFG_HZ_PER_N2_PCT (12200.0 / 100.0 * 1.95 * 4.0 / 120.0)
#define A380X_ELN_VFG_N2_CAP_PCT 100.0     /* L */
#define A380X_ELN_APU_GEN_MIN_N 77.0        /* H PW980 */
#define A380X_ELN_BCRU_LINE_HOLD_S 3.0      /* H */
#define A380X_ELN_BCRU_BAT_OPEN_S 5.0       /* H */
#define A380X_ELN_INFLT_HOLD_S 10.0         /* H sply2 delayed false */
#define A380X_ELN_BAT_CAPACITY_AH 23.0      /* H FBW (real A380 not verified, spec 12.1) */
#define A380X_ELN_BAT_NEEDS_CHARGE_AH 20.0  /* H */
#define A380X_ELN_BAT_SOC_LOW_V 24.5        /* H BCRU "SOC < 20 %" */
#define A380X_ELN_BAT_CHARGE_OHM 0.15       /* H */
#define A380X_ELN_BAT_CHARGE_MAX_A 10.0     /* H */
#define A380X_ELN_TR_VOLTS 28.0             /* H */
#define A380X_ELN_TR_APU_VOLTS 28.0         /* M (30.2 V no-load, 0.0135 ohm) */
#define A380X_ELN_SI_MIN_INPUT_V 16.0       /* H */
#define A380X_ELN_EMER_IAS_KT 100.0         /* H */
#define A380X_ELN_LOAD_DC_ESS_W 1000.0      /* L spec 3.10 */
#define A380X_ELN_LOAD_AC_EMER_W 900.0      /* L (static inverter input side) */
#define A380X_ELN_LOAD_DC_APU_W 100.0       /* L */
#define A380X_ELN_LOAD_APU_START_W 9000.0   /* L while the start motor runs */
#define A380X_ELN_LOAD_DC_MAIN_W 800.0      /* L DC 1 / DC 2 when a battery carries them */
#define A380X_ELN_LOAD_HOT_W 15.0           /* L hot-bus quiescent load (clock, ADIRU backup excluded) */
#define A380X_ELN_LOAD_ADIRU_BACKUP_W 120.0 /* L one ADIRU running on its hot-bus backup */
#define A380X_ELN_RAT_DEPLOY_PER_S 1.0      /* H RAT stow ratio rate */
#define A380X_ELN_RAT_GOVERNED_RPM 4150.0   /* H */
#define A380X_ELN_RAT_SPOOL_TAU_S 1.5       /* M first-order spool */
#define A380X_ELN_EMER_GEN_RATIO 2.4286     /* H gen rpm / turbine rpm */
#define A380X_ELN_DU_BOOT_MIN_S 0.25        /* H CDS Thales boot */
#define A380X_ELN_DU_BOOT_SPAN_S 0.20       /* H */
#define A380X_ELN_DU_SELFTEST_S 15.0        /* H CONFIG_SELF_TEST_TIME default */
#define A380X_ELN_DU_STANDBY_S 10.0         /* H */
#define A380X_ELN_APU_FLAP_S 8.0            /* M: FBW opens in 6-12 s (random per unit) */
#define A380X_ELN_APU_STARTER_CUTOUT_N 55.0 /* H */
#define A380X_ELN_APU_AVAIL_N 95.0          /* H */
#define A380X_ELN_APU_AVAIL_HOLD_S 2.0      /* H */
#define A380X_ELN_APU_AVAIL_FAST_N 99.5     /* H */
#define A380X_ELN_APU_STOPPED_N 8.0         /* L spec 8.2 step 5 */

/* ---------------------------------------------------------------- crew controls */
typedef struct {
    unsigned char bat_auto[A380X_ELN_BATTERIES];  /* BAT 1, BAT 2, ESS, APU: AUTO */
    unsigned char gen_on[4], apu_gen_on[2], ext_on[4], drive_disc[4];
    unsigned char bus_tie_auto, ac_ess_feed_normal;
    unsigned char galley_auto, pax_sys_on, comm_on[2], elmu_on;
    unsigned char rat_man_on;                      /* RAT MAN ON pressed (latched request) */
} A380XElnControls;

typedef struct {
    unsigned char gen[4], bat[4], tr[4], stat_inv, ac_bus[4], dc_bus[3] /* DC1, DC2, DC ESS */;
    unsigned char ext[4], drive_oil[4], apu_gen[2], elmu, galley;
} A380XElnFailures;

/* Simulator inputs (filled by the adapter each frame). */
typedef struct {
    double dt;                       /* simulator seconds; ignored while paused */
    int paused, on_ground, sim_ready;
    double ias_kt;                   /* ADR 1/3 CAS (or raw IAS) */
    int adr_lsw_valid;               /* ADR 1 or 3 running: low-speed discretes exist (spec 3.1) */
    int gear_compressed;             /* both main gear compressed */
    double eng_n2[4];                /* percent */
    unsigned char eng_fire_released[4];
    unsigned char ext_connected[4];  /* ground power plugged (all four together, GPUM) */
    double apu_n;                    /* APU N percent */
    int apu_master;                  /* APU MASTER SW pushed in */
    int adiru_on[3];                 /* ADIRU selector not OFF (backup supply draw) */
    A380XElnFailures fail;
} A380XElnInputs;

typedef struct { unsigned char state; double timer; } A380XElnDu;

typedef struct {
    int state;                       /* 0 BatOff, 1 Open, 2 Closed */
    double loss_of_ac;
    int line_closed, gnd_svc_closed, ground_servicing, soc_low, input_powered;
} A380XElnBcru;

/* Lamp conditions and lamp-bus power (spec 4.2) for the ELEC / EMER ELEC PWR panel. */
typedef struct {
    unsigned char bat_fault[4], gen_fault[4], apu_gen_fault[2], drive_fault[4];
    unsigned char ac_ess_feed_fault, elmu_fault, galley_fault, rat_fault;
    /* power of each legend: DC ESS, DC 1 && DC ESS (COMMERCIAL 1), DC 2, ... */
    unsigned char pw_bat[4], pw_gen[4], pw_apu_gen[2], pw_ext_avail[4], pw_ext_on[4], pw_drive[4];
    unsigned char pw_bus_tie, pw_ac_ess_feed, pw_galley, pw_pax_sys, pw_comm[2], pw_elmu, pw_rat;
} A380XElnLamps;

typedef struct {
    A380XElnControls ctl;            /* copy of the last controls stepped */
    /* network outputs */
    uint32_t powered;                /* A380X_ELN_BIT(bus) */
    float volts[A380X_ELN_BUS_COUNT], hz[A380X_ELN_BUS_COUNT];
    uint32_t origin[A380X_ELN_BUS_COUNT];   /* A380X_ELN_SRC_* bitmask feeding the bus */
    uint64_t contactors;             /* bit per A380XElnContactor */
    /* sources */
    double vfg_stable_s[4], gen_hz[4];
    float gen_volts[4], gen_load_pct[4];
    int gen_avail[4], drive_connected[4];
    float apu_gen_volts[2], apu_gen_hz[2], apu_gen_load_pct[2];
    int apu_gen_avail[2], ext_avail[4];
    float ext_volts, ext_hz;
    /* batteries / converters */
    double bat_ah[4];
    float bat_v[4], bat_a[4];
    float tr_v[4], tr_a[4];
    A380XElnBcru bcru[3];
    int dc_tie_6pc2, dc_tie_980pc, ess_inflt_closed;
    double sply2_timer;              /* time since the last AC main bus was powered */
    float stat_inv_v, stat_inv_hz;
    /* emergency */
    double emer_active_s, rat_stow, rat_rpm;
    int rat_should_deploy, rat_latched, gcu_active, emer_gen_supplying;
    float emer_gen_v, emer_gen_load_pct;
    int tefo, emer_config, flt_condition, in_flight_lsw1, lsw3;
    int prev_hot1, prev_hot_ess;
    int galley_main_shed, galley_sec_shed;
    /* APU ECB (option A: X-Plane turbine, native ECB) */
    int apu_master, apu_start_latched, apu_start_contactor, apu_avail, apu_ecb_powered;
    int apu_emergency_shutdown;      /* one-shot request: battery power lost while turning */
    double apu_flap, apu_avail_timer;
    /* lamps */
    A380XElnLamps lamps;
    /* display units */
    A380XElnDu du[A380X_ELN_DU_COUNT];
    uint32_t rng;
    int spawn_mode;
    double time_s;
} A380XElnState;

void a380x_eln_init(A380XElnState *s, int spawn_mode);
/* Crew control positions for a spawn mode (cold & dark: all BAT OFF; others: AUTO). */
void a380x_eln_spawn_controls(int spawn_mode, A380XElnControls *out);
/* One simulation step (sub-steps internally at <= 50 ms). */
void a380x_eln_step(A380XElnState *s, const A380XElnControls *c, const A380XElnInputs *in);
/* Snap converged state after a spawn/preset jump: runs the network with timers satisfied. */
void a380x_eln_settle(A380XElnState *s, const A380XElnControls *c, const A380XElnInputs *in);
static inline int a380x_eln_powered(const A380XElnState *s, int bus)
{
    return bus >= 0 && bus < A380X_ELN_BUS_COUNT && ((s->powered >> bus) & 1u) != 0;
}
static inline int a380x_eln_contactor(const A380XElnState *s, int k)
{
    return k >= 0 && k < A380X_ELN_K_COUNT && ((s->contactors >> k) & 1u) != 0;
}
int a380x_eln_any_ac_main(const A380XElnState *s);

/* APU START pb pressed (latched until AVAIL or MASTER off). Returns 0 when already AVAIL. */
int a380x_eln_apu_start_press(A380XElnState *s);
/* X-Plane APU_starter_switch value to command: 0 off, 1 on, 2 start (only meaningful when
 * the master is on); -1 = leave the stock switch alone. */
int a380x_eln_apu_starter_command(const A380XElnState *s);

/* Display units: bus table 6.1 + CDS state machine 6.2. brightness <= 0 counts as unpowered. */
int a380x_eln_du_bus_powered(const A380XElnState *s, int du);
void a380x_eln_du_update(A380XElnState *s, const float *brightness,
                         const unsigned char *failed, double dt, double selftest_s);
void a380x_eln_du_step(A380XElnDu *du, int powered, float brightness, int failed, double dt,
                       double boot_s, double selftest_s);
/* Force every DU to a steady state (spawn): on=1 -> On/Standby (On if powered), on=0 -> Off. */
void a380x_eln_du_snap(A380XElnState *s, int on);

/* ISIS supply (spec 6.3): DC ESS, or HOT 1 above 50 kt. */
int a380x_eln_isis_powered(const A380XElnState *s, double ias_kt);
/* ADIRU supply (spec 6.4 proposal, L): primary bus and hot-bus backup. */
void a380x_eln_adiru_supply(const A380XElnState *s, int unit, int *primary, int *backup);
/* FCU: channel 1 108PH, channel 2 DC 2 (FBWI:2372). */
static inline int a380x_eln_fcu_powered(const A380XElnState *s)
{
    return a380x_eln_powered(s, A380X_ELN_DC_ESS_INFLT) || a380x_eln_powered(s, A380X_ELN_DC2);
}

/* FBW battery open-circuit voltage for a charge in Ah (BAT:98-120). */
double a380x_eln_battery_potential(double ah);

#ifdef __cplusplus
}
#endif
#endif
