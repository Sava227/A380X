/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 hydraulic power (ATA 29) for the X-Plane port: the GREEN and YELLOW 5000 psi systems, their eight
 * engine-driven pumps, four electric motor pumps with the HSMU automatic logic, accumulators, reservoirs
 * (fluid, air pressurisation, temperature) and the overhead / ECAM indications. See docs/HYDRAULICS.md for
 * the Airbus A380 Technical Training Manual (ATA 29, Level II/III) behaviour this follows and FlyByWire's
 * fbw-a380x/src/wasm/systems/a380_systems/src/hydraulic/mod.rs for the pump controllers and constants.
 *
 * Header-only and XPLM-free (unit tests: test_hyd.c). The X-Plane adapter is a380x_hyd.inc.
 *
 * Indices: system 0 GREEN (engines 1+2), 1 YELLOW (engines 3+4). Pump sections per system (FBW PUMP_1..6):
 * 0 ENG(2k+1) A, 1 ENG(2k+1) B, 2 ENG(2k+2) A, 3 ENG(2k+2) B, 4 ELEC PMP A, 5 ELEC PMP B.
 */
#ifndef A380X_HYD_H
#define A380X_HYD_H
#include <math.h>
#include <string.h>

enum { A380X_HYD_GREEN = 0, A380X_HYD_YELLOW = 1, A380X_HYD_SYSTEMS = 2 };
enum { A380X_HYD_EMP_IDLE = 0, A380X_HYD_EMP_MANUAL = 1, A380X_HYD_EMP_AUTO = 2 };
enum { A380X_HYD_SECTIONS = 6 };

/* ---- constants (FBW A380HydraulicCircuitFactory / pumps.rs unless noted; doc = A380 TTM ATA 29) ---- */
#define A380X_HYD_SYS_SW_ON 3700.0        /* MIN_PRESS_PRESSURISED_HI_HYST */
#define A380X_HYD_SYS_SW_OFF 2900.0       /* MIN_PRESS_PRESSURISED_LO_HYST (doc: SD amber below 2900) */
#define A380X_HYD_EDP_SW_ON 3700.0        /* MIN_PRESS_EDP_SECTION_HI_HYST */
#define A380X_HYD_EDP_SW_OFF 2900.0
#define A380X_HYD_EMP_SW_ON 2900.0
#define A380X_HYD_EMP_SW_OFF 1885.0       /* doc: EMP pressure switch low below 1885 psi */
#define A380X_HYD_ACC_PRECHARGE 2612.0    /* helium pre-charge, psi */
#define A380X_HYD_ACC_GAL 0.5
#define A380X_HYD_SYS_COMPLIANCE (10.0 / 210305.0)   /* 10 gal system section / fluid bulk modulus, gal/psi */
#define A380X_HYD_SYS_LEAK 6.0e-6         /* static (internal) leak 0.03 gal/s at 5000 psi, gal/s/psi */
#define A380X_HYD_SEC_LEAK 4.6e-6         /* pump case drain: ~3 % of rated EDP flow, 0.023 gal/s at 5000 psi */
#define A380X_HYD_EDP_RPM_PER_PCT (12200.0 * 0.31 / 100.0)   /* Trent 900 N3 100 % = 12200 rpm, gearbox 0.31 */
#define A380X_HYD_EMP_RPM 8000.0
#define A380X_HYD_ENG_RUNNING_N3 50.0     /* doc: EMP SD indication hidden / EEC inhibition */
#define A380X_HYD_OIL_LOW_N3 25.0         /* FBW TrentEngine oil pressure 18 psi at 25 % */
#define A380X_HYD_RSVR_NOMINAL_L 45.2     /* doc: nominal level at 20 C */
#define A380X_HYD_RSVR_LOW_L 10.0         /* doc: low level area 0..10 l */
#define A380X_HYD_RSVR_MAX_L 100.0
#define A380X_HYD_RSVR_L_PER_C 0.1        /* thermal expansion of the system fluid, l per C (port estimate) */
#define A380X_HYD_GEAR_STORED_L (1.3 * 3.785411784)   /* FBW green reservoir offset with the gear up */
#define A380X_HYD_AIR_NOMINAL 65.0        /* doc: reservoir nominal air pressure 65 psi (4.1 bar) */
#define A380X_HYD_AIR_LOW_ON 21.76        /* FBW reservoir pressure switch (relative) */
#define A380X_HYD_AIR_LOW_OFF 25.0
#define A380X_HYD_OVHT_ON_C 110.0
#define A380X_HYD_OVHT_OFF_C 95.0
#define A380X_HYD_EMP_OVHT_ON_C 150.0
#define A380X_HYD_EMP_OVHT_OFF_C 120.0
#define A380X_HYD_CARGO_RUN_ON_S 20.0     /* FBW DURATION_OF_PUMP_ACTIVATION_AFTER_CARGO_DOOR_OPERATION */
#define A380X_HYD_BWS_RUN_ON_S 30.0       /* FBW ..._AFTER_BODY_STEERING_OPERATION */
#define A380X_HYD_L_PER_GAL 3.785411784

typedef struct {
    double dt;                              /* s */
    double n3_pct[4];                       /* HP spool (X-Plane N2) */
    unsigned char fire_pb[4];               /* ENG FIRE pb released: fire shutoff valve closed */
    /* overhead HYD panel (1 = pressed in) */
    unsigned char edp_pb_auto[4][2];        /* ENG n PUMP A/B FAULT/OFF: in = AUTO, out = OFF (depressurised) */
    unsigned char disc_pb[4];               /* ENG n A+B FAULT/DISC (guarded): in = disconnect */
    /* GND HYD panel */
    unsigned char emp_on_pb[2][2];          /* ELEC PMP A/B AUTO/ON: in = manual ON */
    unsigned char emp_off_pb_auto[2][2];    /* ELEC PMP A/B FAULT/OFF: in = AUTO, out = OFF */
    /* electrical (power_valid 0: everything powered) */
    int power_valid;
    unsigned char ac[4], ac_gnd_svc, dc1, dc2, dc_ess, dc_gnd_svc;
    /* aircraft */
    unsigned char on_ground;
    double gear_deploy;                     /* 0 up .. 1 down (fluid stored in the green gear actuators) */
    double ambient_c, ambient_psi;          /* outside air temperature, static pressure (psi absolute) */
    /* HSMU automatic requests */
    unsigned char cargo_door_request;       /* cargo door operating (green) */
    unsigned char bws_request;              /* body wheel steering operating (yellow) */
    int flight_parity;                      /* -1 unknown, 0 even, 1 odd flight number */
    /* reservoir air: ENG 1 (green) / ENG 4 (yellow) HP air, pneumatic system, GSP ground air cart */
    unsigned char hp_air[2], pneumatic, gsp_air_cart, gsp_depress[2];
    /* ground service */
    unsigned char ground_cart[2];           /* hydraulic ground cart on the ground connectors, running */
    unsigned char refill[2];                /* GSP reservoir filling (hand pump / filling cart) */
    unsigned char edp_reset[4];             /* EDP reset port (ground, engine shut down) */
    double demand_gps[2];                   /* consumer flow demand (actuators), gal/s */
    /* failures */
    unsigned char fail_edp[4][2], fail_emp[2][2], fail_emp_ovht[2][2];
    unsigned char fail_rsvr_ovht[2], fail_leak[2], fail_air_leak[2];
} A380XHydIn;

typedef struct {
    double psi;                             /* system section (pressure transmitter, HP belly fairing manifold) */
    double aux_psi;                         /* green auxiliary (cargo doors) section */
    double sec_psi[A380X_HYD_SECTIONS];
    unsigned char sec_sw[A380X_HYD_SECTIONS];
    unsigned char sys_sw;
    double acc_gas_psi, acc_fluid_gal;
    /* reservoir */
    double level_l, nominal_l, leaked_l, air_psi, fluid_c;
    unsigned char level_low, air_low, ovht;
    /* electric pumps */
    double emp_rpm[2], emp_c[2];
    unsigned char emp_ovht[2], emp_cmd[2], emp_active[2], emp_run[2];
    double emp_low_s[2];
    /* HSMU */
    int emp_mode, auto_pump;
    double auto_hold_s;
    unsigned char emp_to_aux;
    double cav_phase;                       /* cavitation pressure fluctuation */
} A380XHydCircuit;

typedef struct {
    int initialized, alternate;
    A380XHydCircuit c[A380X_HYD_SYSTEMS];
    unsigned char clutch[4], disc_cmd[4], fire_valve_open[4];
    double edp_rpm[4];
    unsigned char edp_press_cmd[4][2], edp_active[4][2];
    unsigned char any_engine_running;
    /* indications */
    unsigned char edp_fault[4][2], disc_fault[4];
    unsigned char emp_fault[2][2], emp_on_lamp[2][2], emp_ctl_memo[2][2];
    /* X-Plane native pumps (sim/cockpit2/hydraulics/actuators), see a380x_hyd.inc */
    unsigned char native_engine_pump[4], native_elec_pump[2];
} A380XHyd;

/* ------------------------------------------------------------------ helpers */
static inline double a380x_hyd_clamp(double v, double lo, double hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}
static inline double a380x_hyd_fin(double v, double fallback) { return isfinite(v) ? v : fallback; }
static inline double a380x_hyd_interp(const double *x, const double *y, int n, double v)
{
    int i;
    if (v <= x[0]) return y[0];
    for (i = 1; i < n; ++i)
        if (v <= x[i]) return y[i - 1] + (y[i] - y[i - 1]) * (v - x[i - 1]) / (x[i] - x[i - 1]);
    return y[n - 1];
}
static inline double a380x_hyd_lag(double v, double target, double tau, double dt)
{
    return tau <= 0.0 ? target : v + (target - v) * (1.0 - exp(-dt / tau));
}
/* EDP displacement, in^3/rev (FBW A380_EDP_DISPLACEMENT_MAP: regulates near 5150 psi) */
static inline double a380x_hyd_edp_disp(double p)
{
    static const double x[9] = {0.0, 500.0, 1000.0, 2900.0, 4790.0, 5150.0, 5225.0, 5350.0, 5500.0};
    static const double y[9] = {2.8, 2.8, 2.8, 2.8, 2.6, 0.0, 0.0, 0.0, 0.0};
    return a380x_hyd_interp(x, y, 9, p);
}
/* EMP displacement at 8000 rpm, in^3/rev (FBW A380_EPUMP_DISPLACEMENT_MAP) */
static inline double a380x_hyd_emp_disp(double p)
{
    static const double x[9] = {0.0, 2000.0, 3000.0, 4000.0, 5000.0, 5100.0, 5200.0, 5300.0, 5350.0};
    static const double y[9] = {0.294525, 0.28875, 0.2858625, 0.231, 0.17325, 0.0, 0.0, 0.0, 0.0};
    return a380x_hyd_interp(x, y, 9, p);
}
/* Pump suction efficiency from the reservoir absolute air pressure (FBW AIR_PRESSURE_CARAC_RATIO). */
static inline double a380x_hyd_cavitation(double psi_abs)
{
    static const double x[9] = {0.0, 5.0, 10.0, 15.0, 20.0, 30.0, 50.0, 70.0, 100.0};
    static const double y[9] = {0.0, 0.1, 0.6, 0.8, 0.9, 1.0, 1.0, 1.0, 1.0};
    return a380x_hyd_interp(x, y, 9, psi_abs);
}
/* Fluid stored above zero pressure: system compliance + accumulator (gas p*V = precharge * total). */
static inline double a380x_hyd_stored_gal(double p)
{
    double v = A380X_HYD_SYS_COMPLIANCE * (p > 0.0 ? p : 0.0);
    if (p > A380X_HYD_ACC_PRECHARGE) v += A380X_HYD_ACC_GAL * (1.0 - A380X_HYD_ACC_PRECHARGE / p);
    return v;
}
static inline double a380x_hyd_acc_fluid(double p)
{
    return p > A380X_HYD_ACC_PRECHARGE ? A380X_HYD_ACC_GAL * (1.0 - A380X_HYD_ACC_PRECHARGE / p) : 0.0;
}
static inline int a380x_hyd_hyst(int state, double v, double on, double off)
{
    if (!isfinite(v)) return 0;
    if (v >= on) return 1;
    if (v < off) return 0;
    return state;
}
/* Pump model: flow (gal/s) at pressure p for a pump kind (0 EDP, 1 EMP, 2 ground cart) at rpm and efficiency. */
static inline double a380x_hyd_pump_flow(int kind, double rpm, double eff, double p)
{
    double speed_eff;
    if (eff <= 0.0) return 0.0;
    if (kind == 2) return 0.6 * eff * a380x_hyd_clamp((5000.0 - p) / 200.0, 0.0, 1.0);
    if (kind == 0) {
        /* EDP: case-drain leakage of a slow pump (windmilling engine): no useful pressure below ~10 % N3,
         * full above ~25 % N3 (engine idle is above 60 %) - an engine shut down in flight shows PRESS LO */
        if (rpm <= 10.0 * A380X_HYD_EDP_RPM_PER_PCT) return 0.0;
        speed_eff = a380x_hyd_clamp((rpm - 10.0 * A380X_HYD_EDP_RPM_PER_PCT) / (15.0 * A380X_HYD_EDP_RPM_PER_PCT), 0.0, 1.0);
    } else {
        if (rpm <= 75.0) return 0.0;   /* FBW DEFAULT_ZERO_EFFICIENCY_SPEED_THRESHOLD_RPM */
        speed_eff = a380x_hyd_clamp((rpm - 75.0) / 225.0, 0.0, 1.0);
    }
    return (kind ? a380x_hyd_emp_disp(p) : a380x_hyd_edp_disp(p)) * rpm / 231.0 / 60.0 * eff * speed_eff;
}
/* Deadhead pressure of one pump section: where the pump flow equals the section leak. */
static inline double a380x_hyd_pump_alone(int kind, double rpm, double eff)
{
    double lo = 0.0, hi = 5600.0;
    int i;
    if (a380x_hyd_pump_flow(kind, rpm, eff, 0.0) <= 0.0) return 0.0;
    for (i = 0; i < 40; ++i) {
        double mid = 0.5 * (lo + hi);
        if (a380x_hyd_pump_flow(kind, rpm, eff, mid) - A380X_HYD_SEC_LEAK * mid > 0.0) lo = mid; else hi = mid;
    }
    return lo;
}

/* ------------------------------------------------------------------ lifecycle */
/* Aircraft load. pressurised: a running aircraft (systems at 5000 psi). */
static inline void a380x_hyd_reset(A380XHyd *h, double ambient_c, int pressurised)
{
    int k, i;
    memset(h, 0, sizeof *h);
    ambient_c = a380x_hyd_clamp(a380x_hyd_fin(ambient_c, 15.0), -60.0, 60.0);
    for (i = 0; i < 4; ++i) { h->clutch[i] = 1; h->fire_valve_open[i] = 1; }
    for (k = 0; k < A380X_HYD_SYSTEMS; ++k) {
        A380XHydCircuit *c = &h->c[k];
        c->psi = pressurised ? 5100.0 : 0.0;
        c->sys_sw = pressurised;
        c->air_psi = A380X_HYD_AIR_NOMINAL;
        c->fluid_c = pressurised ? ambient_c + 25.0 : ambient_c;
        c->level_l = c->nominal_l = A380X_HYD_RSVR_NOMINAL_L;
        c->emp_c[0] = c->emp_c[1] = ambient_c;
        c->auto_pump = 0;
        for (i = 0; i < 4; ++i) {
            c->sec_psi[i] = pressurised ? 5100.0 : 0.0;
            c->sec_sw[i] = pressurised;
        }
    }
    h->initialized = 1;
}

static inline int a380x_hyd_powered(const A380XHydIn *in, unsigned char bus)
{
    return !in->power_valid || bus;
}

/* ------------------------------------------------------------------ HSMU: electric pumps */
static inline void a380x_hyd_hsmu(A380XHyd *h, const A380XHydIn *in, int k, double dt)
{
    A380XHydCircuit *c = &h->c[k];
    int i, request, auto_active, manual, healthy[2], powered[2], hsmu;
    hsmu = a380x_hyd_powered(in, k == A380X_HYD_GREEN ? (unsigned char)(in->dc1 || in->dc_gnd_svc)
                                                     : (unsigned char)(in->dc1 || in->dc2));
    for (i = 0; i < 2; ++i) {
        /* doc: G-EMP A also supplied by the AC ground service bus (cargo doors with only that bus) */
        unsigned char bus = in->ac[2 * k + i] || (k == A380X_HYD_GREEN && i == 0 && in->ac_gnd_svc);
        powered[i] = a380x_hyd_powered(in, bus);
        healthy[i] = powered[i] && in->emp_off_pb_auto[k][i] && !in->fail_emp[k][i] && !c->emp_ovht[i];
    }
    request = hsmu && in->on_ground && !h->any_engine_running &&
              (k == A380X_HYD_GREEN ? in->cargo_door_request : in->bws_request);
    if (request) c->auto_hold_s = k == A380X_HYD_GREEN ? A380X_HYD_CARGO_RUN_ON_S : A380X_HYD_BWS_RUN_ON_S;
    else c->auto_hold_s = c->auto_hold_s > dt ? c->auto_hold_s - dt : 0.0;
    auto_active = request || c->auto_hold_s > 0.0;
    manual = in->emp_on_pb[k][0] || in->emp_on_pb[k][1];

    /* doc: the mode already running has priority (auto ignores manual requests and the reverse) */
    if (c->emp_mode == A380X_HYD_EMP_AUTO && !auto_active) c->emp_mode = A380X_HYD_EMP_IDLE;
    if (c->emp_mode == A380X_HYD_EMP_MANUAL && !manual) c->emp_mode = A380X_HYD_EMP_IDLE;
    if (c->emp_mode == A380X_HYD_EMP_IDLE) {
        if (manual) c->emp_mode = A380X_HYD_EMP_MANUAL;
        else if (auto_active && (healthy[0] || healthy[1])) {
            /* normal elec config: EVEN flight main B / back-up A, ODD flight main A / back-up B */
            int normal = !in->power_valid || (in->ac[0] && in->ac[1] && in->ac[2] && in->ac[3]);
            int main_b;
            if (in->flight_parity == 0) main_b = 1;
            else if (in->flight_parity == 1) main_b = 0;
            else { main_b = h->alternate & 1; h->alternate ^= 1; }   /* unknown: alternate per operation */
            if (!normal) main_b = !healthy[0];
            c->auto_pump = main_b && healthy[1] ? 1 : healthy[0] ? 0 : 1;
            c->emp_mode = A380X_HYD_EMP_AUTO;
        }
    }
    /* auto: the selected EMP not healthy -> the other one */
    if (c->emp_mode == A380X_HYD_EMP_AUTO && !healthy[c->auto_pump] && healthy[!c->auto_pump])
        c->auto_pump = !c->auto_pump;

    for (i = 0; i < 2; ++i) {
        c->emp_cmd[i] = (unsigned char)(c->emp_mode == A380X_HYD_EMP_MANUAL ? in->emp_on_pb[k][i]
                                        : c->emp_mode == A380X_HYD_EMP_AUTO ? c->auto_pump == i : 0);
        /* supplied: FAULT/OFF in AUTO, AC bus, HSMU, not inhibited by the EECs, overheat relay closed */
        c->emp_active[i] = (unsigned char)(c->emp_cmd[i] && in->emp_off_pb_auto[k][i] && powered[i] && hsmu &&
                                           !h->any_engine_running && !c->emp_ovht[i]);
        c->emp_run[i] = (unsigned char)(c->emp_active[i] && !in->fail_emp[k][i]);
        c->emp_rpm[i] = a380x_hyd_lag(c->emp_rpm[i], c->emp_run[i] ? A380X_HYD_EMP_RPM : 0.0,
                                      c->emp_run[i] ? 0.4 : 1.5, dt);
        if (c->emp_rpm[i] < 1.0 && !c->emp_run[i]) c->emp_rpm[i] = 0.0;
        h->emp_on_lamp[k][i] = (unsigned char)(in->emp_on_pb[k][i] ||
                                               (c->emp_mode == A380X_HYD_EMP_AUTO && c->auto_pump == i && c->emp_active[i]));
    }
    /* green auto (cargo doors): the EMP sections are routed to the auxiliary section, not the system */
    c->emp_to_aux = (unsigned char)(k == A380X_HYD_GREEN && c->emp_mode == A380X_HYD_EMP_AUTO);
}

/* ------------------------------------------------------------------ one system */
static inline void a380x_hyd_circuit(A380XHyd *h, const A380XHydIn *in, int k, double dt)
{
    A380XHydCircuit *c = &h->c[k];
    double rpm[A380X_HYD_SECTIONS], eff[A380X_HYD_SECTIONS], alone[A380X_HYD_SECTIONS];
    int kind[A380X_HYD_SECTIONS], to_sys[A380X_HYD_SECTIONS];
    double suction, fluid, v0, lo, hi, p, demand, cart_eff, aux = 0.0, amb_c, amb_psi, stored, gear_l, therm;
    int i, it, cart;

    amb_c = a380x_hyd_clamp(a380x_hyd_fin(in->ambient_c, 15.0), -70.0, 60.0);
    amb_psi = a380x_hyd_clamp(a380x_hyd_fin(in->ambient_psi, 14.696), 0.5, 16.0);

    /* ---- reservoir air pressurisation (doc: ENG 1 / ENG 4 HP air, pneumatic system, GSP) */
    if (in->gsp_depress[k] && in->on_ground) c->air_psi -= 10.0 * dt;
    else if (in->hp_air[k] || in->pneumatic || (in->gsp_air_cart && in->on_ground)) {
        if (c->air_psi < A380X_HYD_AIR_NOMINAL) c->air_psi = fmin(A380X_HYD_AIR_NOMINAL, c->air_psi + 3.0 * dt);
    } else c->air_psi -= 0.05 / 60.0 * dt;   /* trapped air: a few psi per hour */
    if (in->fail_air_leak[k]) c->air_psi -= 1.0 * dt;
    c->air_psi = a380x_hyd_clamp(c->air_psi, 0.0, 80.0);
    c->air_low = (unsigned char)!a380x_hyd_hyst(!c->air_low, c->air_psi, A380X_HYD_AIR_LOW_OFF, A380X_HYD_AIR_LOW_ON);

    /* ---- suction: cavitation from absolute reservoir pressure, fluid available */
    suction = a380x_hyd_cavitation(c->air_psi + amb_psi);
    /* cavitating pumps deliver unevenly: the system pressure fluctuates (doc: RSVR AIR PRESS LO procedure) */
    c->cav_phase = fmod(c->cav_phase + 1.7 * dt, 2.0 * 3.14159265358979);
    if (suction < 0.5) suction *= 1.0 - (1.0 - suction / 0.5) * (0.5 + 0.5 * sin(c->cav_phase) * sin(0.37 * c->cav_phase + 1.0));
    fluid = a380x_hyd_clamp((c->level_l - 1.0) / 3.0, 0.0, 1.0);   /* starved below ~1 l */

    /* ---- pump sections */
    for (i = 0; i < 4; ++i) {
        int e = 2 * k + i / 2, ab = i & 1;
        kind[i] = 0;
        rpm[i] = h->edp_rpm[e];
        eff[i] = h->edp_press_cmd[e][ab] && h->fire_valve_open[e] && !in->fail_edp[e][ab] ? suction * fluid : 0.0;
        to_sys[i] = 1;
    }
    for (i = 0; i < 2; ++i) {
        kind[4 + i] = 1;
        rpm[4 + i] = c->emp_rpm[i];
        eff[4 + i] = c->emp_run[i] ? suction * fluid : 0.0;
        to_sys[4 + i] = !c->emp_to_aux;
    }
    for (i = 0; i < A380X_HYD_SECTIONS; ++i) alone[i] = a380x_hyd_pump_alone(kind[i], rpm[i], eff[i]);
    cart = in->ground_cart[k] && in->on_ground;
    cart_eff = cart ? fluid : 0.0;

    /* ---- system section: implicit (backward Euler) volume balance, solved by bisection */
    demand = a380x_hyd_fin(in->demand_gps[k], 0.0);
    if (demand < 0.0) demand = 0.0;
    v0 = a380x_hyd_stored_gal(c->psi);
    lo = 0.0; hi = 6000.0;
    for (it = 0; it < 48; ++it) {
        double q_in = 0.0, q_out, f;
        p = 0.5 * (lo + hi);
        for (i = 0; i < A380X_HYD_SECTIONS; ++i)
            if (to_sys[i]) {
                double q = a380x_hyd_pump_flow(kind[i], rpm[i], eff[i], p) - A380X_HYD_SEC_LEAK * p;
                if (q > 0.0) q_in += q;
            }
        q_in += a380x_hyd_pump_flow(2, 0.0, cart_eff, p);
        q_out = A380X_HYD_SYS_LEAK * p + demand * fmin(1.0, p / 500.0) +
                (in->fail_leak[k] ? 0.06 * sqrt(p / 5000.0) : 0.0);
        f = a380x_hyd_stored_gal(p) - v0 - dt * (q_in - q_out);
        if (f > 0.0) hi = p; else lo = p;
    }
    p = 0.5 * (lo + hi);
    if (p < 1.0) p = 0.0;
    c->psi = p;
    c->sys_sw = (unsigned char)a380x_hyd_hyst(c->sys_sw, p, A380X_HYD_SYS_SW_ON, A380X_HYD_SYS_SW_OFF);
    c->acc_fluid_gal = a380x_hyd_acc_fluid(p);
    c->acc_gas_psi = p > A380X_HYD_ACC_PRECHARGE ? p : A380X_HYD_ACC_PRECHARGE;
    if (in->fail_leak[k]) c->leaked_l += 0.06 * sqrt(p / 5000.0) * dt * A380X_HYD_L_PER_GAL;

    /* ---- pump section pressures (check valve: a section never exceeds its own deadhead; behind a
     *      pumping section it sits at the system pressure) and pressure switches */
    for (i = 0; i < A380X_HYD_SECTIONS; ++i) {
        /* delivering into the system: section at system pressure; otherwise its own deadhead */
        double target = to_sys[i] && alone[i] >= p ? p : alone[i];
        c->sec_psi[i] = a380x_hyd_lag(c->sec_psi[i], target, target > c->sec_psi[i] ? 0.15 : 0.5, dt);
        if (c->sec_psi[i] < 1.0) c->sec_psi[i] = 0.0;
        c->sec_sw[i] = (unsigned char)(i < 4 ? a380x_hyd_hyst(c->sec_sw[i], c->sec_psi[i], A380X_HYD_EDP_SW_ON, A380X_HYD_EDP_SW_OFF)
                                             : a380x_hyd_hyst(c->sec_sw[i], c->sec_psi[i], A380X_HYD_EMP_SW_ON, A380X_HYD_EMP_SW_OFF));
        if (!to_sys[i]) aux = fmax(aux, alone[i]);
    }
    c->aux_psi = a380x_hyd_lag(c->aux_psi, k == A380X_HYD_GREEN ? fmax(aux, p) : 0.0, 0.3, dt);

    /* ---- fluid temperature, overheat (doc: OVHT while the reservoir inlet temperature is above normal) */
    {
        double target = amb_c + (p > 1000.0 ? 40.0 : 0.0);
        c->fluid_c = a380x_hyd_lag(c->fluid_c, target, p > 1000.0 ? 600.0 : 1800.0, dt);
        if (in->fail_rsvr_ovht[k]) c->fluid_c = fmin(130.0, c->fluid_c + 0.5 * dt);
        c->ovht = (unsigned char)a380x_hyd_hyst(c->ovht, c->fluid_c, A380X_HYD_OVHT_ON_C, A380X_HYD_OVHT_OFF_C);
    }
    /* ---- EMP motor temperature: overheat relay (doc: OVHT and FAULT until back to normal) */
    for (i = 0; i < 2; ++i) {
        double target = amb_c + (c->emp_run[i] ? 30.0 : 0.0);
        c->emp_c[i] = a380x_hyd_lag(c->emp_c[i], target, c->emp_run[i] ? 300.0 : 600.0, dt);
        if (in->fail_emp_ovht[k][i]) c->emp_c[i] = fmin(180.0, c->emp_c[i] + 1.0 * dt);
        c->emp_ovht[i] = (unsigned char)a380x_hyd_hyst(c->emp_ovht[i], c->emp_c[i], A380X_HYD_EMP_OVHT_ON_C, A380X_HYD_EMP_OVHT_OFF_C);
    }

    /* ---- reservoir level: what the system, accumulator and (green) gear actuators hold comes from it.
     * The movable nominal index follows temperature and those volumes, so level - nominal = -leaked. */
    if (in->refill[k] && in->on_ground) c->leaked_l = fmax(0.0, c->leaked_l - 0.3 * dt);
    stored = a380x_hyd_stored_gal(p) - a380x_hyd_stored_gal(5000.0);
    gear_l = k == A380X_HYD_GREEN ? A380X_HYD_GEAR_STORED_L * (1.0 - a380x_hyd_clamp(a380x_hyd_fin(in->gear_deploy, 1.0), 0.0, 1.0)) : 0.0;
    therm = A380X_HYD_RSVR_L_PER_C * (c->fluid_c - 20.0);
    c->nominal_l = A380X_HYD_RSVR_NOMINAL_L + therm - gear_l - stored * A380X_HYD_L_PER_GAL;
    c->level_l = a380x_hyd_clamp(c->nominal_l - c->leaked_l, 0.0, A380X_HYD_RSVR_MAX_L);
    c->level_low = (unsigned char)!a380x_hyd_hyst(!c->level_low, c->level_l, A380X_HYD_RSVR_LOW_L + 1.0, A380X_HYD_RSVR_LOW_L);
}

/* ------------------------------------------------------------------ step */
static inline void a380x_hyd_update(A380XHyd *h, const A380XHydIn *in)
{
    double dt;
    int e, k, i;
    if (!h || !in) return;
    if (!h->initialized) a380x_hyd_reset(h, in->ambient_c, 0);
    dt = a380x_hyd_fin(in->dt, 0.0);
    if (dt <= 0.0) return;
    if (dt > 0.25) dt = 0.25;   /* pauses / frame hitches: the implicit solve stays stable, timers do not jump */

    h->any_engine_running = 0;
    for (e = 0; e < 4; ++e) if (a380x_hyd_fin(in->n3_pct[e], 0.0) > A380X_HYD_ENG_RUNNING_N3) h->any_engine_running = 1;

    /* ---- engine-driven pumps: clutch (DISC), depressurisation solenoid, fire shutoff valve */
    for (e = 0; e < 4; ++e) {
        double n3 = a380x_hyd_clamp(a380x_hyd_fin(in->n3_pct[e], 0.0), 0.0, 120.0);
        /* FBW EnginePumpDisconnectionClutch: DC 2 for engines 1/4, DC 1 for 2/3 */
        int disc_power = a380x_hyd_powered(in, (e == 0 || e == 3) ? in->dc2 : in->dc1);
        if (in->disc_pb[e] && disc_power) { h->disc_cmd[e] = 1; h->clutch[e] = 0; }
        /* doc: re-engagement only on the ground, engine shut down, at the reset port */
        if (in->edp_reset[e] && in->on_ground && n3 < 5.0 && !in->disc_pb[e]) { h->clutch[e] = 1; h->disc_cmd[e] = 0; }
        h->edp_rpm[e] = h->clutch[e] ? n3 * A380X_HYD_EDP_RPM_PER_PCT : 0.0;
        h->fire_valve_open[e] = (unsigned char)!in->fire_pb[e];
        for (i = 0; i < 2; ++i) {
            /* FBW: an unpowered solenoid leaves the pump pressurising */
            int solenoid = a380x_hyd_powered(in, in->dc_ess);
            h->edp_press_cmd[e][i] = (unsigned char)(!solenoid || (in->edp_pb_auto[e][i] && !in->fire_pb[e]));
        }
    }
    for (k = 0; k < A380X_HYD_SYSTEMS; ++k) a380x_hyd_hsmu(h, in, k, dt);
    for (k = 0; k < A380X_HYD_SYSTEMS; ++k) a380x_hyd_circuit(h, in, k, dt);

    /* ---- indications (doc OPS/CTL & IND; FBW A380EngineDrivenPumpController / A380ElectricPumpController) */
    for (k = 0; k < A380X_HYD_SYSTEMS; ++k) {
        A380XHydCircuit *c = &h->c[k];
        int gnd_eng_off = in->on_ground && !h->any_engine_running;
        for (i = 0; i < 4; ++i) {
            int e = 2 * k + i / 2, ab = i & 1, auto_pb = in->edp_pb_auto[e][ab];
            double n3 = a380x_hyd_fin(in->n3_pct[e], 0.0);
            int low = h->edp_press_cmd[e][ab] && !c->sec_sw[i] && !(n3 < A380X_HYD_OIL_LOW_N3 && in->on_ground);
            h->edp_active[e][ab] = (unsigned char)(h->clutch[e] && h->edp_press_cmd[e][ab] && h->fire_valve_open[e] &&
                                                   !in->fail_edp[e][ab] && c->sec_psi[i] >= A380X_HYD_EDP_SW_OFF);
            h->edp_fault[e][ab] = (unsigned char)(low || (c->air_low && auto_pb) || (c->level_low && auto_pb) || c->ovht);
        }
        for (i = 0; i < 2; ++i) {
            int e = 2 * k + i;
            h->disc_fault[e] = (unsigned char)((c->level_low || c->ovht) && !h->disc_cmd[e]);
        }
        for (i = 0; i < 2; ++i) {
            int auto_pb = in->emp_off_pb_auto[k][i], low;
            /* low pressure while supplied, confirmed 2 s (spin-up) */
            if (c->emp_active[i] && !c->sec_sw[4 + i]) c->emp_low_s[i] += dt; else c->emp_low_s[i] = 0.0;
            low = c->emp_low_s[i] > 2.0;
            h->emp_fault[k][i] = (unsigned char)(low || c->emp_ovht[i] || c->ovht ||
                                                 (c->air_low && gnd_eng_off && auto_pb) ||
                                                 (c->level_low && gnd_eng_off && auto_pb));
            /* doc: memo G (Y) ELEC PMP A (B) CTL when manually ON and supplying normal pressure */
            h->emp_ctl_memo[k][i] = (unsigned char)(c->emp_mode == A380X_HYD_EMP_MANUAL && c->emp_run[i] &&
                                                    c->sec_sw[4 + i] && !c->emp_to_aux);
        }
    }
    /* ---- X-Plane's native systems follow: per engine its EDPs, per system any EMP / ground cart */
    for (e = 0; e < 4; ++e) {
        const A380XHydCircuit *c = &h->c[e / 2];
        h->native_engine_pump[e] = (unsigned char)(c->sys_sw && (h->edp_active[e][0] || h->edp_active[e][1]));
    }
    for (k = 0; k < A380X_HYD_SYSTEMS; ++k) {
        const A380XHydCircuit *c = &h->c[k];
        int elec = ((c->emp_run[0] && c->sec_sw[4]) || (c->emp_run[1] && c->sec_sw[5])) && !c->emp_to_aux;
        h->native_elec_pump[k] = (unsigned char)(c->sys_sw && (elec || (in->ground_cart[k] && in->on_ground)));
    }
}
#endif
