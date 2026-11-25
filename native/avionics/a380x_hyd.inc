/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 hydraulic power (a380x_hyd.h) in X-Plane (docs/HYDRAULICS.md).
 *
 * Reads the HYD and GND HYD overhead pushbuttons (a380x_ovhd_lamps.inc latches), the A380 electrical network,
 * engines (X-Plane N2 = Trent N3), ENG FIRE pbs, cargo doors, body-wheel steering, bleed air and weather.
 * Publishes the a380x/xp12/hyd/ datarefs, lights the pushbutton FAULT and ON legends, feeds the SD / FWS / flaps / FCU /
 * brakes (a380x_hyd_view.h), and drives X-Plane's native pumps so the flight model's gear, brakes, steering and
 * flaps follow the model: the ACF maps engines 1+2 and electric pump 1 to native system 1 (GREEN), engines 3+4
 * and electric pump 2 to native system 2 (YELLOW).
 *
 * Unity-build include of a380x_avionics.c, after a380x_ovhd_lamps.inc, a380x_center_right.inc and
 * a380x_fms_adapter.inc, before a380x_fws_adapter.inc.
 */
#include "a380x_hyd.h"
#include "a380x_hyd_view.h"
#define A380X_HYD_ADAPTER 1

enum { HR_N2, HR_FIRE, HR_ONGROUND, HR_GEAR, HR_OAT, HR_BARO, HR_DOORS, HR_BODY_STEER, HR_BLEED_L, HR_BLEED_R,
       HR_BLEED_C, HR_PAUSED, HR_REPLAY, HR_FLAP, HR_SLAT, HR_YOKE_P, HR_YOKE_R, HR_YOKE_H, HR_SBRK, HR_GS,
       HR_ENGINE_PUMP, HR_EPUMP1, HR_EPUMP2, HR_F_LEAK1, HR_F_LEAK2, HR_F_PMP1, HR_F_PMP2, HR_F_ELE1, HR_F_ELE2,
       HR_REFS };
static const char *const hyda_names[HR_REFS] = {
    "sim/cockpit2/engine/indicators/N2_percent", "a380x/xp12/engine/fire_pushbutton",
    "sim/flightmodel/failures/onground_any", "sim/flightmodel2/gear/deploy_ratio",
    "sim/weather/aircraft/temperature_ambient_deg_c", "sim/weather/aircraft/barometer_current_pas",
    "sim/flightmodel2/misc/door_open_ratio", "a380x/xp12/steering/body_deg",
    "sim/cockpit2/bleedair/indicators/bleed_available_left", "sim/cockpit2/bleedair/indicators/bleed_available_right",
    "sim/cockpit2/bleedair/indicators/bleed_available_center", "sim/time/paused", "sim/time/is_in_replay",
    "sim/flightmodel2/controls/flap1_deploy_ratio", "sim/flightmodel2/controls/slat1_deploy_ratio",
    "sim/joystick/yoke_pitch_ratio", "sim/joystick/yoke_roll_ratio", "sim/joystick/yoke_heading_ratio",
    "sim/flightmodel2/controls/speedbrake_ratio", "sim/flightmodel/position/groundspeed",
    "sim/cockpit2/hydraulics/actuators/engine_pump", "sim/cockpit2/hydraulics/actuators/electric_hydraulic_pump_on",
    "sim/cockpit2/hydraulics/actuators/electric_hydraulic_pump2_on",
    "sim/operation/failures/rel_hydleak", "sim/operation/failures/rel_hydleak2",
    "sim/operation/failures/rel_hydpmp", "sim/operation/failures/rel_hydpmp2",
    "sim/operation/failures/rel_hydpmp_ele", "sim/operation/failures/rel_hydpmp_el2"};

#define HYDA_MAX_VARS 64
typedef struct { int *i; float *f; int n, writable; } HydaVar;

static struct {
    int ready, reseed, service, reset_held[4];
    A380XHyd model;
    XPLMDataRef refs[HR_REFS];
    HydaVar vars[HYDA_MAX_VARS];
    XPLMDataRef pub[HYDA_MAX_VARS];
    int pub_count;
    XPLMCommandRef cmd[5];
    float door_prev[4], prev_gear, prev_flap, prev_slat, prev_yoke[3], prev_sbrk;
    int prev_valid;
    /* published (read-only) */
    float psi[2], aux_psi[2], acc_psi[2], level_l[2], nominal_l[2], air_psi[2], fluid_c[2], section_psi[2][6];
    int sys_sw[2], level_low[2], air_low[2], ovht[2], emp_mode[2], section_sw[2][6];
    int edp_active[8], edp_fault[8], edp_disc[4], clutch[4], disc_fault[4], fire_valve[4];
    int emp_active[4], emp_running[4], emp_fault[4], emp_ovht[4], emp_memo[4], emp_on_light[4];
    float emp_rpm[4];
    /* writable */
    int native_bridge, ground_cart[2], air_cart, depress[2], refill[2], flight_parity;
    int fail_edp[8], fail_emp[4], fail_emp_ovht[4], fail_rsvr_ovht[2], fail_leak[2], fail_air_leak[2];
} hyda;

/* ------------------------------------------------------------------ dataref accessors */
static int hyda_geti(void *r) { const HydaVar *v = (const HydaVar *)r; return v->i[0]; }
static void hyda_seti(void *r, int x) { HydaVar *v = (HydaVar *)r; if (v->writable) v->i[0] = x; }
static float hyda_getf(void *r) { const HydaVar *v = (const HydaVar *)r; return v->f[0]; }
static int hyda_getvi(void *r, int *out, int off, int max)
{
    const HydaVar *v = (const HydaVar *)r;
    int i, n;
    if (!out) return v->n;
    if (off < 0 || off >= v->n || max <= 0) return 0;
    n = v->n - off < max ? v->n - off : max;
    for (i = 0; i < n; ++i) out[i] = v->i[off + i];
    return n;
}
static void hyda_setvi(void *r, int *in, int off, int max)
{
    HydaVar *v = (HydaVar *)r;
    int i;
    if (!v->writable || !in || off < 0) return;
    for (i = 0; i < max && off + i < v->n; ++i) v->i[off + i] = in[i] != 0;
}
static int hyda_getvf(void *r, float *out, int off, int max)
{
    const HydaVar *v = (const HydaVar *)r;
    int i, n;
    if (!out) return v->n;
    if (off < 0 || off >= v->n || max <= 0) return 0;
    n = v->n - off < max ? v->n - off : max;
    for (i = 0; i < n; ++i) out[i] = v->f[off + i];
    return n;
}
static int hyda_reg(const char *name, int *i, float *f, int n, int writable)
{
    HydaVar *v;
    XPLMDataRef ref;
    if (hyda.pub_count >= HYDA_MAX_VARS) return 0;
    v = &hyda.vars[hyda.pub_count];
    v->i = i; v->f = f; v->n = n; v->writable = writable;
    if (f)
        ref = XPLMRegisterDataAccessor(name, n == 1 ? xplmType_Float : xplmType_FloatArray, 0, NULL, NULL,
                                       n == 1 ? hyda_getf : NULL, NULL, NULL, NULL, NULL, NULL,
                                       n == 1 ? NULL : hyda_getvf, NULL, NULL, NULL, v, NULL);
    else
        ref = XPLMRegisterDataAccessor(name, n == 1 ? xplmType_Int : xplmType_IntArray, writable,
                                       n == 1 ? hyda_geti : NULL, n == 1 && writable ? hyda_seti : NULL,
                                       NULL, NULL, NULL, NULL, n == 1 ? NULL : hyda_getvi,
                                       n > 1 && writable ? hyda_setvi : NULL, NULL, NULL, NULL, NULL, v, v);
    if (!ref) return 0;
    hyda.pub[hyda.pub_count++] = ref;
    return 1;
}

/* ------------------------------------------------------------------ commands */
static int hyda_cmd(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
    int id = (int)(intptr_t)refcon;
    (void)cmd;
    if (id >= 0 && id < 4) hyda.reset_held[id] = phase != xplm_CommandEnd;
    else if (id == 4 && phase == xplm_CommandBegin) hyda.service = 1;
    return 0;
}

static void hyda_shutdown(void)
{
    int i;
    for (i = 0; i < hyda.pub_count; ++i) if (hyda.pub[i]) XPLMUnregisterDataAccessor(hyda.pub[i]);
    for (i = 0; i < 5; ++i) if (hyda.cmd[i]) XPLMUnregisterCommandHandler(hyda.cmd[i], hyda_cmd, 1, (void *)(intptr_t)i);
    memset(&hyda, 0, sizeof hyda);
    memset(&a380x_hyd_view, 0, sizeof a380x_hyd_view);
}

static int hyda_init(void)
{
    static const char *const sys[2] = {"green", "yellow"};
    char name[96];
    int k, i, ok = 1;
    memset(&hyda, 0, sizeof hyda);
    hyda.native_bridge = 1;
    hyda.flight_parity = -1;
    for (k = 0; k < 2; ++k) {
#define HYDA_SYS(suffix, ip, fp, n) do { snprintf(name, sizeof name, "a380x/xp12/hyd/%s/" suffix, sys[k]); \
        ok &= hyda_reg(name, ip, fp, n, 0); } while (0)
        HYDA_SYS("pressure_psi", NULL, &hyda.psi[k], 1);
        HYDA_SYS("pressure_switch", &hyda.sys_sw[k], NULL, 1);
        HYDA_SYS("aux_pressure_psi", NULL, &hyda.aux_psi[k], 1);
        HYDA_SYS("accumulator_psi", NULL, &hyda.acc_psi[k], 1);
        HYDA_SYS("section_psi", NULL, hyda.section_psi[k], 6);
        HYDA_SYS("section_switch", hyda.section_sw[k], NULL, 6);
        HYDA_SYS("reservoir_level_l", NULL, &hyda.level_l[k], 1);
        HYDA_SYS("reservoir_nominal_l", NULL, &hyda.nominal_l[k], 1);
        HYDA_SYS("reservoir_level_low", &hyda.level_low[k], NULL, 1);
        HYDA_SYS("reservoir_air_psi", NULL, &hyda.air_psi[k], 1);
        HYDA_SYS("reservoir_air_low", &hyda.air_low[k], NULL, 1);
        HYDA_SYS("fluid_temp_c", NULL, &hyda.fluid_c[k], 1);
        HYDA_SYS("overheat", &hyda.ovht[k], NULL, 1);
        HYDA_SYS("emp_mode", &hyda.emp_mode[k], NULL, 1);
#undef HYDA_SYS
    }
#define HYDA_REG(n_, ip, fp, n, w) ok &= hyda_reg("a380x/xp12/hyd/" n_, ip, fp, n, w)
    HYDA_REG("edp_active", hyda.edp_active, NULL, 8, 0);
    HYDA_REG("edp_fault", hyda.edp_fault, NULL, 8, 0);
    HYDA_REG("edp_disconnected", hyda.edp_disc, NULL, 4, 0);
    HYDA_REG("edp_clutch_engaged", hyda.clutch, NULL, 4, 0);
    HYDA_REG("disc_fault", hyda.disc_fault, NULL, 4, 0);
    HYDA_REG("fire_valve_open", hyda.fire_valve, NULL, 4, 0);
    HYDA_REG("emp_active", hyda.emp_active, NULL, 4, 0);
    HYDA_REG("emp_running", hyda.emp_running, NULL, 4, 0);
    HYDA_REG("emp_fault", hyda.emp_fault, NULL, 4, 0);
    HYDA_REG("emp_ovht", hyda.emp_ovht, NULL, 4, 0);
    HYDA_REG("emp_ctl_memo", hyda.emp_memo, NULL, 4, 0);
    HYDA_REG("emp_on_light", hyda.emp_on_light, NULL, 4, 0);
    HYDA_REG("emp_rpm", NULL, hyda.emp_rpm, 4, 0);
    HYDA_REG("native_bridge", &hyda.native_bridge, NULL, 1, 1);
    HYDA_REG("gsp/ground_cart", hyda.ground_cart, NULL, 2, 1);
    HYDA_REG("gsp/air_cart", &hyda.air_cart, NULL, 1, 1);
    HYDA_REG("gsp/depress", hyda.depress, NULL, 2, 1);
    HYDA_REG("gsp/refill", hyda.refill, NULL, 2, 1);
    HYDA_REG("hsmu/flight_parity", &hyda.flight_parity, NULL, 1, 1);
    HYDA_REG("fail/edp", hyda.fail_edp, NULL, 8, 1);
    HYDA_REG("fail/emp", hyda.fail_emp, NULL, 4, 1);
    HYDA_REG("fail/emp_ovht", hyda.fail_emp_ovht, NULL, 4, 1);
    HYDA_REG("fail/rsvr_ovht", hyda.fail_rsvr_ovht, NULL, 2, 1);
    HYDA_REG("fail/leak", hyda.fail_leak, NULL, 2, 1);
    HYDA_REG("fail/air_leak", hyda.fail_air_leak, NULL, 2, 1);
#undef HYDA_REG
    for (i = 0; i < 5; ++i) {
        static const char *const desc[5] = {"A380 HYD: ENG 1 EDP reset port (ground, engine shut down)",
            "A380 HYD: ENG 2 EDP reset port (ground, engine shut down)", "A380 HYD: ENG 3 EDP reset port (ground, engine shut down)",
            "A380 HYD: ENG 4 EDP reset port (ground, engine shut down)",
            "A380 HYD: ground service panel - refill both reservoirs to nominal and recharge their air (ground)"};
        if (i < 4) snprintf(name, sizeof name, "a380x/xp12/hyd/edp_reset_%d", i + 1);
        else snprintf(name, sizeof name, "a380x/xp12/hyd/service");
        hyda.cmd[i] = XPLMCreateCommand(name, desc[i]);
        if (hyda.cmd[i]) XPLMRegisterCommandHandler(hyda.cmd[i], hyda_cmd, 1, (void *)(intptr_t)i);
        else ok = 0;
    }
    if (!ok) { hyda_shutdown(); return 0; }
    a380x_hyd_reset(&hyda.model, 15.0, 0);
    hyda.reseed = 1;
    hyda.ready = 1;
    return 1;
}

static void hyda_plane_loaded(void)
{
    if (!hyda.ready) return;
    memset(hyda.fail_edp, 0, sizeof hyda.fail_edp); memset(hyda.fail_emp, 0, sizeof hyda.fail_emp);
    memset(hyda.fail_emp_ovht, 0, sizeof hyda.fail_emp_ovht); memset(hyda.fail_rsvr_ovht, 0, sizeof hyda.fail_rsvr_ovht);
    memset(hyda.fail_leak, 0, sizeof hyda.fail_leak); memset(hyda.fail_air_leak, 0, sizeof hyda.fail_air_leak);
    memset(hyda.ground_cart, 0, sizeof hyda.ground_cart); memset(hyda.depress, 0, sizeof hyda.depress);
    memset(hyda.refill, 0, sizeof hyda.refill);
    hyda.air_cart = 0;
    hyda.reseed = 1;
}

/* ------------------------------------------------------------------ inputs */
static float hyda_f(int r, float fallback)
{
    float v;
    if (!hyda.refs[r]) return fallback;
    v = XPLMGetDataf(hyda.refs[r]);
    return isfinite(v) ? v : fallback;
}
static int hyda_i(int r) { return hyda.refs[r] ? XPLMGetDatai(hyda.refs[r]) : 0; }
static int hyda_failed(int r) { return hyda.refs[r] && XPLMGetDatai(hyda.refs[r]) >= 6; }   /* failure_enum 6 = inoperative */
static int hyda_pb(int pb) { return ovhd.ready ? ovhd_in.pb[pb] : a380x_ovhd_pb_default(pb); }
/* HSMU flight number parity: last digit of the FMS flight number (none: -1, alternate per operation). */
static int hyda_flight_number_parity(void)
{
    const char *s = fmsa_fms.plans[A380X_FMS_PLAN_ACTIVE].flight_number;
    int i, digit = -1;
    for (i = 0; i < (int)sizeof fmsa_fms.plans[0].flight_number && s[i]; ++i)
        if (s[i] >= '0' && s[i] <= '9') digit = s[i] - '0';
    return digit < 0 ? -1 : digit & 1;
}

static void hyda_update(float wall_dt)
{
    A380XHydIn in;
    A380XHyd *h = &hyda.model;
    float v[10], dt = wall_dt;
    int iv[4], e, k, i, any_running = 0;
    if (!hyda.ready) return;
    for (i = 0; i < HR_REFS; ++i) if (!hyda.refs[i]) hyda.refs[i] = XPLMFindDataRef(hyda_names[i]);
    if (hyda_i(HR_PAUSED) || hyda_i(HR_REPLAY) || !(dt > 0.0f)) return;   /* hold */
    memset(&in, 0, sizeof in);
    in.dt = dt;
    if (hyda.refs[HR_N2] && XPLMGetDatavf(hyda.refs[HR_N2], v, 0, 4) == 4)
        for (e = 0; e < 4; ++e) in.n3_pct[e] = isfinite(v[e]) ? v[e] : 0.0;
    for (e = 0; e < 4; ++e) if (in.n3_pct[e] > A380X_HYD_ENG_RUNNING_N3) any_running = 1;
    if (hyda.refs[HR_FIRE] && XPLMGetDatavi(hyda.refs[HR_FIRE], iv, 0, 4) == 4)
        for (e = 0; e < 4; ++e) in.fire_pb[e] = iv[e] != 0;
    /* overhead HYD / GND HYD pushbuttons */
    for (e = 0; e < 4; ++e) {
        in.edp_pb_auto[e][0] = (unsigned char)hyda_pb(A380X_OVHD_PB_HYD_ENG1A + 2 * e);
        in.edp_pb_auto[e][1] = (unsigned char)hyda_pb(A380X_OVHD_PB_HYD_ENG1A + 2 * e + 1);
        in.disc_pb[e] = (unsigned char)hyda_pb(A380X_OVHD_PB_HYD_ENG1_DISC + e);
    }
    for (k = 0; k < 2; ++k)
        for (i = 0; i < 2; ++i) {
            in.emp_on_pb[k][i] = (unsigned char)hyda_pb(A380X_OVHD_PB_HYD_G_ELEC_A_ON + 2 * k + i);
            in.emp_off_pb_auto[k][i] = (unsigned char)hyda_pb(A380X_OVHD_PB_HYD_G_ELEC_A + 2 * k + i);
        }
    /* A380 electrical network (without it everything counts as powered) */
    in.power_valid = a380x_power.available;
    for (i = 0; i < 4; ++i) in.ac[i] = (unsigned char)a380x_power_bus(A380X_ELN_AC1 + i, 1);
    in.ac_gnd_svc = (unsigned char)a380x_power_bus(A380X_ELN_AC_GND_SVC, 1);
    in.dc1 = (unsigned char)a380x_power_bus(A380X_ELN_DC1, 1);
    in.dc2 = (unsigned char)a380x_power_bus(A380X_ELN_DC2, 1);
    in.dc_ess = (unsigned char)a380x_power_bus(A380X_ELN_DC_ESS, 1);
    in.dc_gnd_svc = (unsigned char)a380x_power_bus(A380X_ELN_DC_GND_SVC, 1);
    /* aircraft */
    in.on_ground = (unsigned char)(hyda_i(HR_ONGROUND) != 0);
    in.gear_deploy = 1.0;
    if (hyda.refs[HR_GEAR] && XPLMGetDatavf(hyda.refs[HR_GEAR], v, 0, 5) == 5)
        in.gear_deploy = 0.25 * (v[1] + v[2] + v[3] + v[4]);
    in.ambient_c = hyda_f(HR_OAT, 15.0f);
    in.ambient_psi = hyda_f(HR_BARO, 101325.0f) / 6894.757;
    /* HSMU automatic requests: a cargo door moving (door_open_ratio 16..19), body-wheel steering ordered */
    if (hyda.refs[HR_DOORS] && XPLMGetDatavf(hyda.refs[HR_DOORS], v, 16, 4) == 4) {
        for (i = 0; i < 4; ++i) {
            if (hyda.prev_valid && fabsf(v[i] - hyda.door_prev[i]) > 1e-5f) in.cargo_door_request = 1;
            hyda.door_prev[i] = v[i];
        }
    }
    in.bws_request = (unsigned char)(fabsf(hyda_f(HR_BODY_STEER, 0.0f)) > 0.5f && hyda_f(HR_GS, 0.0f) < 15.0f);
    in.flight_parity = hyda.flight_parity == 0 || hyda.flight_parity == 1 ? hyda.flight_parity : hyda_flight_number_parity();
    /* reservoir air: ENG 1 / ENG 4 HP air, pneumatic system (any bleed source), GSP ground air cart */
    in.hp_air[0] = (unsigned char)(in.n3_pct[0] > A380X_HYD_ENG_RUNNING_N3);
    in.hp_air[1] = (unsigned char)(in.n3_pct[3] > A380X_HYD_ENG_RUNNING_N3);
    in.pneumatic = (unsigned char)(fmaxf(hyda_f(HR_BLEED_L, 0.0f), fmaxf(hyda_f(HR_BLEED_R, 0.0f), hyda_f(HR_BLEED_C, 0.0f))) > 0.1f);
    in.gsp_air_cart = (unsigned char)(hyda.air_cart != 0);
    for (k = 0; k < 2; ++k) {
        in.gsp_depress[k] = (unsigned char)(hyda.depress[k] != 0);
        in.ground_cart[k] = (unsigned char)(hyda.ground_cart[k] != 0);
        in.refill[k] = (unsigned char)(hyda.refill[k] != 0);
    }
    for (e = 0; e < 4; ++e) in.edp_reset[e] = (unsigned char)hyda.reset_held[e];
    /* consumer flow: gear (green), flaps (green + yellow PCU), slats (green), flight controls, spoilers */
    {
        float gear = (float)in.gear_deploy, flap = hyda_f(HR_FLAP, 0.0f), slat = hyda_f(HR_SLAT, 0.0f);
        float yoke[3] = {hyda_f(HR_YOKE_P, 0.0f), hyda_f(HR_YOKE_R, 0.0f), hyda_f(HR_YOKE_H, 0.0f)};
        float sbrk = hyda_f(HR_SBRK, 0.0f);
        if (hyda.prev_valid) {
            double controls = (fabs(yoke[0] - hyda.prev_yoke[0]) + fabs(yoke[1] - hyda.prev_yoke[1]) +
                               fabs(yoke[2] - hyda.prev_yoke[2])) / dt * 0.25 + fabs(sbrk - hyda.prev_sbrk) / dt * 0.2;
            if (controls > 1.0) controls = 1.0;
            in.demand_gps[0] = 0.5 * controls;
            in.demand_gps[1] = 0.5 * controls;
            if (fabsf(gear - hyda.prev_gear) > 1e-5f) in.demand_gps[0] += 0.5;
            if (fabsf(flap - hyda.prev_flap) > 1e-5f) { in.demand_gps[0] += 0.15; in.demand_gps[1] += 0.15; }
            if (fabsf(slat - hyda.prev_slat) > 1e-5f) in.demand_gps[0] += 0.12;
        }
        hyda.prev_gear = gear; hyda.prev_flap = flap; hyda.prev_slat = slat; hyda.prev_sbrk = sbrk;
        memcpy(hyda.prev_yoke, yoke, sizeof yoke);
    }
    hyda.prev_valid = 1;
    /* failures: ours, or X-Plane's own (System 1 = GREEN, 2 = YELLOW) */
    for (e = 0; e < 4; ++e)
        for (i = 0; i < 2; ++i)
            in.fail_edp[e][i] = (unsigned char)(hyda.fail_edp[2 * e + i] || hyda_failed(e < 2 ? HR_F_PMP1 : HR_F_PMP2));
    for (k = 0; k < 2; ++k) {
        for (i = 0; i < 2; ++i) {
            in.fail_emp[k][i] = (unsigned char)(hyda.fail_emp[2 * k + i] || hyda_failed(k ? HR_F_ELE2 : HR_F_ELE1));
            in.fail_emp_ovht[k][i] = (unsigned char)(hyda.fail_emp_ovht[2 * k + i] != 0);
        }
        in.fail_rsvr_ovht[k] = (unsigned char)(hyda.fail_rsvr_ovht[k] != 0);
        in.fail_leak[k] = (unsigned char)(hyda.fail_leak[k] || hyda_failed(k ? HR_F_LEAK2 : HR_F_LEAK1));
        in.fail_air_leak[k] = (unsigned char)(hyda.fail_air_leak[k] != 0);
    }

    /* aircraft (re)load: a running aircraft starts pressurised */
    if (hyda.reseed) { a380x_hyd_reset(h, in.ambient_c, any_running); hyda.reseed = 0; }
    if (hyda.service && in.on_ground)
        for (k = 0; k < 2; ++k) { h->c[k].leaked_l = 0.0; h->c[k].air_psi = A380X_HYD_AIR_NOMINAL; }
    hyda.service = 0;

    a380x_hyd_update(h, &in);

    /* ---- publish */
    for (k = 0; k < 2; ++k) {
        const A380XHydCircuit *c = &h->c[k];
        hyda.psi[k] = (float)c->psi; hyda.sys_sw[k] = c->sys_sw; hyda.aux_psi[k] = (float)c->aux_psi;
        hyda.acc_psi[k] = (float)c->acc_gas_psi;
        for (i = 0; i < 6; ++i) { hyda.section_psi[k][i] = (float)c->sec_psi[i]; hyda.section_sw[k][i] = c->sec_sw[i]; }
        hyda.level_l[k] = (float)c->level_l; hyda.nominal_l[k] = (float)c->nominal_l; hyda.level_low[k] = c->level_low;
        hyda.air_psi[k] = (float)c->air_psi; hyda.air_low[k] = c->air_low;
        hyda.fluid_c[k] = (float)c->fluid_c; hyda.ovht[k] = c->ovht; hyda.emp_mode[k] = c->emp_mode;
        for (i = 0; i < 2; ++i) {
            int n = 2 * k + i;
            hyda.emp_active[n] = c->emp_active[i]; hyda.emp_running[n] = c->emp_run[i];
            hyda.emp_fault[n] = h->emp_fault[k][i]; hyda.emp_ovht[n] = c->emp_ovht[i];
            hyda.emp_memo[n] = h->emp_ctl_memo[k][i]; hyda.emp_on_light[n] = h->emp_on_lamp[k][i];
            hyda.emp_rpm[n] = (float)c->emp_rpm[i];
        }
        a380x_hyd_view.psi[k] = c->psi;
        a380x_hyd_view.sys_sw[k] = c->sys_sw;
    }
    a380x_hyd_view.available = 1;
    for (e = 0; e < 4; ++e) {
        hyda.edp_active[2 * e] = h->edp_active[e][0]; hyda.edp_active[2 * e + 1] = h->edp_active[e][1];
        hyda.edp_fault[2 * e] = h->edp_fault[e][0]; hyda.edp_fault[2 * e + 1] = h->edp_fault[e][1];
        hyda.edp_disc[e] = h->disc_cmd[e]; hyda.clutch[e] = h->clutch[e];
        hyda.disc_fault[e] = h->disc_fault[e]; hyda.fire_valve[e] = h->fire_valve_open[e];
    }

    /* ---- overhead legends: FAULT of the EDP, DISC and EMP FAULT/OFF pbs, ON of the EMP AUTO/ON pbs */
    for (e = 0; e < 4; ++e) {
        ovhd_in.sys_fault[A380X_OVHD_PB_HYD_ENG1A + 2 * e] = h->edp_fault[e][0];
        ovhd_in.sys_fault[A380X_OVHD_PB_HYD_ENG1A + 2 * e + 1] = h->edp_fault[e][1];
        ovhd_in.sys_fault[A380X_OVHD_PB_HYD_ENG1_DISC + e] = h->disc_fault[e];
    }
    for (k = 0; k < 2; ++k)
        for (i = 0; i < 2; ++i) {
            ovhd_in.sys_fault[A380X_OVHD_PB_HYD_G_ELEC_A + 2 * k + i] = h->emp_fault[k][i];
            ovhd_in.sys_on[A380X_OVHD_PB_HYD_G_ELEC_A_ON + 2 * k + i] = h->emp_on_lamp[k][i];
        }

    /* ---- X-Plane's native pumps (the flight model's gear, brakes, steering, flaps) */
    if (hyda.native_bridge) {
        int want[4], have[4];
        if (hyda.refs[HR_ENGINE_PUMP] && XPLMGetDatavi(hyda.refs[HR_ENGINE_PUMP], have, 0, 4) == 4) {
            int change = 0;
            for (e = 0; e < 4; ++e) { want[e] = h->native_engine_pump[e]; if ((have[e] != 0) != want[e]) change = 1; }
            if (change) XPLMSetDatavi(hyda.refs[HR_ENGINE_PUMP], want, 0, 4);
        }
        /* the ACCUS REINFLATE pb (a380x_center_right.inc) also runs electric pump 1 for the brake accumulator */
        want[0] = h->native_elec_pump[0] || center_right.reinflate_active;
        want[1] = h->native_elec_pump[1];
        if (hyda.refs[HR_EPUMP1] && (XPLMGetDatai(hyda.refs[HR_EPUMP1]) != 0) != want[0]) XPLMSetDatai(hyda.refs[HR_EPUMP1], want[0]);
        if (hyda.refs[HR_EPUMP2] && (XPLMGetDatai(hyda.refs[HR_EPUMP2]) != 0) != want[1]) XPLMSetDatai(hyda.refs[HR_EPUMP2], want[1]);
    }
}
