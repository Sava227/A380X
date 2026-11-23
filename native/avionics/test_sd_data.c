/* SPDX-License-Identifier: GPL-3.0-or-later
 * a380x_sd_data.inc against mocked datarefs: per-page values, FBW raw ARINC 429 encoding,
 * well-formed JSON under the shared-memory cap, no invented failures, no NaN on absent inputs.
 *   test_sd_data             run the checks
 *   test_sd_data --dump      print the full-state "vars" JSON (tools/test_sd_pages_census.py)
 *   test_sd_data --contracts print "R <sim dataref>" for every X-Plane input (build.py)
 * Compile with -DSDD_COMPILE_ONLY to only type-check the module (MinGW gate in build.py).
 */
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "XPLMDataAccess.h"
#include "a380x_xpkit_shared.h"

typedef struct { char data[XPKIT_JSON_CAP]; size_t size; int failed; } XpKitJson;
static void xpkit_append(XpKitJson *j, const char *format, ...)
{
    if (j->failed) return;
    va_list args; va_start(args, format);
    int n = vsnprintf(j->data + j->size, sizeof(j->data) - j->size, format, args); va_end(args);
    if (n < 0 || (size_t)n >= sizeof(j->data) - j->size) j->failed = 1; else j->size += (size_t)n;
}
#include "a380x_sd_data.inc"

#ifdef SDD_COMPILE_ONLY
void sdd_compile_check(XpKitJson *j, const SddContext *context);
void sdd_compile_check(XpKitJson *j, const SddContext *context) { sdd_emit(j, context); }
#else
/* ------------------------------------------------------------------ mocked XPLM data access */
typedef struct { char name[96]; XPLMDataTypeID type; int n; double v[20]; } Mock;
static Mock mocks[400];
static int mock_count, failures;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #c); ++failures; } } while (0)

static Mock *mock(const char *name, XPLMDataTypeID type, int n, const double *v)
{
    Mock *m = NULL;
    for (int i = 0; i < mock_count; ++i) if (!strcmp(mocks[i].name, name)) m = &mocks[i];
    if (!m) { m = &mocks[mock_count++]; snprintf(m->name, sizeof(m->name), "%s", name); }
    m->type = type; m->n = n;
    for (int i = 0; i < n && i < 20; ++i) m->v[i] = v[i];
    return m;
}
static void setf(const char *name, double v) { mock(name, xplmType_Float, 1, &v); }
static void seti(const char *name, double v) { mock(name, xplmType_Int, 1, &v); }
static void setvf(const char *name, int n, ...)
{
    double v[20]; va_list a; va_start(a, n);
    for (int i = 0; i < n; ++i) v[i] = va_arg(a, double);
    va_end(a); mock(name, xplmType_FloatArray, n, v);
}
static void setvi(const char *name, int n, ...)
{
    double v[20]; va_list a; va_start(a, n);
    for (int i = 0; i < n; ++i) v[i] = va_arg(a, int);
    va_end(a); mock(name, xplmType_IntArray, n, v);
}
XPLMDataRef XPLMFindDataRef(const char *name)
{
    for (int i = 0; i < mock_count; ++i) if (!strcmp(mocks[i].name, name)) return &mocks[i];
    return NULL;
}
XPLMDataTypeID XPLMGetDataRefTypes(XPLMDataRef r) { return ((Mock *)r)->type; }
int XPLMGetDatai(XPLMDataRef r) { return (int)((Mock *)r)->v[0]; }
float XPLMGetDataf(XPLMDataRef r) { return (float)((Mock *)r)->v[0]; }
double XPLMGetDatad(XPLMDataRef r) { return ((Mock *)r)->v[0]; }
int XPLMGetDatavf(XPLMDataRef r, float *out, int offset, int max)
{
    Mock *m = r; int k = 0;
    if (!out) return m->n;
    for (; k < max && offset + k < m->n; ++k) out[k] = (float)m->v[offset + k];
    return k;
}
int XPLMGetDatavi(XPLMDataRef r, int *out, int offset, int max)
{
    Mock *m = r; int k = 0;
    if (!out) return m->n;
    for (; k < max && offset + k < m->n; ++k) out[k] = (int)m->v[offset + k];
    return k;
}

/* ------------------------------------------------------------------ JSON helpers */
static const char *json_value(const char *s);
static const char *json_ws(const char *s) { while (*s == ' ' || *s == '\n' || *s == '\t' || *s == '\r') ++s; return s; }
static const char *json_string(const char *s)
{
    if (*s != '"') return NULL;
    for (++s; *s && *s != '"'; ++s) {
        if ((unsigned char)*s < 32) return NULL;
        if (*s == '\\') { ++s; if (!*s) return NULL; }
    }
    return *s == '"' ? s + 1 : NULL;
}
static const char *json_number(const char *s)
{
    const char *start = s;
    if (*s == '-') ++s;
    if (!(*s >= '0' && *s <= '9')) return NULL;
    if (*s == '0' && s[1] >= '0' && s[1] <= '9') return NULL;
    while (*s >= '0' && *s <= '9') ++s;
    if (*s == '.') { ++s; if (!(*s >= '0' && *s <= '9')) return NULL; while (*s >= '0' && *s <= '9') ++s; }
    if (*s == 'e' || *s == 'E') {
        ++s; if (*s == '+' || *s == '-') ++s;
        if (!(*s >= '0' && *s <= '9')) return NULL;
        while (*s >= '0' && *s <= '9') ++s;
    }
    return s > start ? s : NULL;
}
static const char *json_value(const char *s)
{
    s = json_ws(s);
    if (*s == '{' || *s == '[') {
        char close = *s == '{' ? '}' : ']';
        s = json_ws(s + 1);
        if (*s == close) return s + 1;
        for (;;) {
            if (close == '}') {
                s = json_string(json_ws(s)); if (!s) return NULL;
                s = json_ws(s); if (*s != ':') return NULL; ++s;
            }
            s = json_value(s); if (!s) return NULL;
            s = json_ws(s);
            if (*s == ',') { ++s; continue; }
            return *s == close ? s + 1 : NULL;
        }
    }
    if (*s == '"') return json_string(s);
    if (!strncmp(s, "true", 4)) return s + 4;
    if (!strncmp(s, "false", 5)) return s + 5;
    if (!strncmp(s, "null", 4)) return s + 4;
    return json_number(s);
}
static int json_ok(const char *s) { const char *e = json_value(s); return e && !*json_ws(e); }

static XpKitJson out;
/* Value of "name": in the emitted vars (NAN if absent). */
static double var(const char *name)
{
    char key[160];
    snprintf(key, sizeof(key), "\"%s\":", name);
    const char *p = strstr(out.data, key);
    if (!p) return NAN;
    p += strlen(key);
    if (!strncmp(p, "true", 4)) return 1;
    if (!strncmp(p, "false", 5)) return 0;
    return strtod(p, NULL);
}
static int has(const char *name) { return !isnan(var(name)); }
static void decode(double raw, double *value, int *ssm)
{
    uint64_t r = (uint64_t)raw; uint32_t bits = (uint32_t)(r & 0xffffffffu); float f;
    memcpy(&f, &bits, sizeof(f)); *value = f; *ssm = (int)((r >> 32) & 3u);
}
static double word(const char *name, int expected_ssm)
{
    double v = NAN; int ssm = -1;
    if (!has(name)) { fprintf(stderr, "missing word %s\n", name); ++failures; return NAN; }
    decode(var(name), &v, &ssm);
    if (ssm != expected_ssm) { fprintf(stderr, "%s: ssm %d != %d\n", name, ssm, expected_ssm); ++failures; }
    return v;
}
static int bit(const char *name, int n) { double v; int ssm; decode(var(name), &v, &ssm); return ((int)v >> (n - 1)) & 1; }
static int near(double a, double b, double tol) { return fabs(a - b) <= tol; }

static SddStatus status = {1, {210200001u, 0, 0, 0, 0}, {210300009u, 0}, {0}, 220500001u, 0, 0};
static void emit(double now)
{
    SddContext context = {now, 18.7, 1, &status};
    out.size = 0; out.failed = 0; out.data[0] = 0;
    xpkit_append(&out, "{");
    sdd_emit(&out, &context);
    xpkit_append(&out, "\"_end\":0}");
}

/* Engines running at idle on the ground, APU and its bleed on, packs on, doors closed. */
static void full_state(void)
{
    static const char *const buses[] = {"ac1", "ac2", "ac3", "ac4", "ac_ess", "ac_emer", "ac_eha", "dc1", "dc2", "dc_ess", "dc_apu"};
    static const char *const contactors[] = {"990XU1", "990XU2", "990XU3", "990XU4", "990XG1", "990XG2", "990XG3", "990XG4",
        "990XS1", "990XS2", "980XU1", "980XU2", "980XU3", "980XU4", "980XU5", "980XU6", "900XU", "3XC1", "3XC2", "5XE",
        "3XB_1", "3XB_2", "7XB", "990PU1", "990PU2", "6PE", "7PU", "990PB1", "990PB2", "6PC1", "6PB3", "5PB", "6PC2", "980PC"};
    char name[128];
    seti("a380x/xp12/elec/network_active", 1);
    for (size_t i = 0; i < sizeof(buses) / sizeof(*buses); ++i) {
        snprintf(name, sizeof(name), "a380x/xp12/elec/bus/%s_powered", buses[i]); seti(name, 1);
    }
    for (size_t i = 0; i < sizeof(contactors) / sizeof(*contactors); ++i) {
        snprintf(name, sizeof(name), "a380x/xp12/elec/contactor/%s", contactors[i]);
        seti(name, !strcmp(contactors[i], "3XB_1") || !strncmp(contactors[i], "990XU", 5));
    }
    setvf("a380x/xp12/elec/gen/potential", 4, 115.0, 116.0, 115.0, 114.0);
    setvf("a380x/xp12/elec/gen/load_pct", 4, 20.0, 21.0, 22.0, 23.0);
    setvf("a380x/xp12/elec/apu_gen/potential", 2, 115.0, 115.0);
    setvf("a380x/xp12/elec/apu_gen/freq", 2, 400.0, 400.0);
    setvf("a380x/xp12/elec/apu_gen/load_pct", 2, 0.0, 0.0);
    setvf("a380x/xp12/elec/bat/potential", 4, 27.5, 27.4, 27.3, 27.2);
    setvf("a380x/xp12/elec/bat/current", 4, 1.0, 2.0, -3.0, 4.0);
    setvf("a380x/xp12/elec/tr/potential", 4, 28.0, 28.1, 28.2, 28.3);
    setvf("a380x/xp12/elec/tr/current", 4, 50.0, 51.0, 52.0, 53.0);
    setf("a380x/xp12/elec/ext/potential", 0); setf("a380x/xp12/elec/ext/freq", 0);
    setf("a380x/xp12/elec/emer_gen/potential", 0); setf("a380x/xp12/elec/emer_gen/load_pct", 0);
    setf("a380x/xp12/elec/stat_inv/potential", 0); setf("a380x/xp12/elec/stat_inv/freq", 0);
    setf("a380x/xp12/elec/rat/stow_ratio", 0);
    for (int i = 1; i <= 4; ++i) {
        snprintf(name, sizeof(name), "a380x/xp12/elec/pb/gen%d", i); seti(name, 1);
        snprintf(name, sizeof(name), "a380x/xp12/elec/ext/avail%d", i); seti(name, i == 2);
    }
    seti("a380x/xp12/elec/pb/apu_gen_a", 1); seti("a380x/xp12/elec/pb/apu_gen_b", 0);
    seti("a380x/xp12/elec/pb/bat1", 1); seti("a380x/xp12/elec/pb/bat2", 1);
    seti("a380x/xp12/elec/pb/bat_ess", 1); seti("a380x/xp12/elec/pb/bat_apu", 0);
    seti("a380x/xp12/apu/master_on", 1); seti("a380x/xp12/apu/avail", 1); seti("a380x/xp12/apu/start_on", 0);
    seti("a380x/xp12/apu/ecb_powered", 1); setf("a380x/xp12/apu/flap_open_ratio", 1.0);
    setf("sim/cockpit2/electrical/APU_N1_percent", 100.0); setf("sim/cockpit2/electrical/APU_EGT_c", 420.0);
    seti("sim/cockpit2/bleedair/actuators/apu_bleed", 0);
    setf("sim/time/total_running_time_sec", 100.0);
    setf("sim/cockpit2/temperature/outside_air_temp_degc", 10.0);
    seti("sim/flightmodel/failures/onground_any", 1);
    setvf("sim/cockpit2/engine/indicators/N1_percent", 4, 20.0, 20.5, 21.0, 21.5);
    setvf("sim/cockpit2/engine/indicators/N2_percent", 4, 60.0, 61.0, 62.0, 63.0);
    setvf("sim/cockpit2/engine/indicators/fuel_flow_kg_sec", 4, 0.25, 0.25, 0.25, 0.25);
    setvi("sim/flightmodel/engine/ENGN_running", 4, 1, 1, 1, 1);
    setvi("sim/flightmodel2/engines/starter_is_running", 4, 0, 0, 0, 0);
    setvi("sim/cockpit2/engine/actuators/igniter_on", 4, 0, 0, 0, 0);
    setvf("sim/cockpit2/engine/indicators/oil_temperature_deg_C", 4, 176.0, 176.0, 176.0, 176.0);
    seti("sim/aircraft/engine/acf_oilT_is_C", 0);     /* Fahrenheit ACF: 176 F = 80 C */
    setvf("sim/cockpit2/engine/indicators/oil_quantity_ratio", 4, 1.0, 1.0, 1.0, 1.0);
    setvf("sim/flightmodel/engine/POINT_thrust", 4, 0.0, 0.0, 0.0, 0.0);
    setvi("sim/cockpit2/bleedair/actuators/engine_bleed_sov", 4, 1, 1, 1, 1);
    setf("sim/cockpit2/bleedair/indicators/bleed_available_left", 1.0);
    setf("sim/cockpit2/bleedair/indicators/bleed_available_right", 0.5);
    seti("sim/cockpit2/bleedair/actuators/isol_valve_left", 1);
    seti("sim/cockpit2/bleedair/actuators/isol_valve_right", 0);
    seti("sim/cockpit2/bleedair/actuators/pack_left", 1); seti("sim/cockpit2/bleedair/actuators/pack_right", 1);
    setf("sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1", 3000.0);
    setf("sim/cockpit2/hydraulics/indicators/hydraulic_pressure_2", 1200.0);
    setf("sim/cockpit2/hydraulics/indicators/hydraulic_fluid_ratio_1", 1.0);
    setf("sim/cockpit2/hydraulics/indicators/hydraulic_fluid_ratio_2", 0.05);
    setvi("sim/cockpit2/hydraulics/actuators/engine_pump", 4, 1, 1, 1, 1);
    seti("sim/cockpit2/hydraulics/actuators/electric_hydraulic_pump_on", 0);
    seti("sim/cockpit2/hydraulics/actuators/electric_hydraulic_pump2_on", 1);
    setf("sim/cockpit2/pressurization/indicators/cabin_altitude_ft", 50.0);
    setf("sim/cockpit2/pressurization/indicators/cabin_vvi_fpm", -100.0);
    setf("sim/cockpit2/pressurization/indicators/pressure_diffential_psi", 0.1);
    setf("sim/cockpit2/pressurization/indicators/outflow_valve", 0.8);
    setf("sim/cockpit2/pressurization/actuators/cabin_altitude_ft", 6000.0);
    setf("sim/cockpit2/pressurization/actuators/cabin_vvi_fpm", 500.0);
    seti("a380x/xp12/avionics/flight_phase", 2);
    setf("a380x/xp12/fms/landing_elevation_ft", 1234.0); seti("a380x/xp12/fms/landing_elevation_valid", 1);
    setvf("sim/flightmodel2/gear/deploy_ratio", 5, 1.0, 1.0, 1.0, 0.5, 1.0);
    setvf("sim/flightmodel2/gear/brake_absorbed_rat", 5, 0.0, 0.2, 0.0, 0.0, 0.1);
    setvi("sim/flightmodel2/gear/on_ground", 5, 1, 1, 1, 1, 1);
    setvf("sim/flightmodel2/misc/door_open_ratio", 19, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0);
    seti("sim/cockpit2/switches/beacon_on", 1); seti("sim/cockpit2/oxygen/actuators/o2_valve_on", 1);
    setvi("a380x/xp12/efb/slide_armed", 16, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0);   /* EFB slides */
    setvf("sim/flightmodel/weight/m_fuel", 9, 30000.0, 15000.0, 50000.0, 20000.0, 15000.0, 5000.0, 5000.0, 10000.0, 0.0);
    setvi("sim/cockpit2/engine/actuators/fuel_pump_on", 4, 1, 1, 0, 1);
    setvi("sim/cockpit2/fuel/fuel_tank_pump_on", 9, 1, 0, 0, 0, 0, 1, 0, 0, 0);
    setvf("sim/cockpit2/engine/actuators/mixture_ratio", 4, 1.0, 1.0, 0.0, 1.0);
    seti("sim/cockpit2/fuel/fuel_crossfeed_selector", 0);
    setf("a380x/xp12/hydraulics/aileron_l_inboard_ratio", 0.5); setf("a380x/xp12/hydraulics/aileron_l_midboard_ratio", 0.5);
    setf("a380x/xp12/hydraulics/aileron_l_outboard_ratio", 0.5); setf("a380x/xp12/hydraulics/aileron_r_inboard_ratio", -0.5);
    setf("a380x/xp12/hydraulics/aileron_r_midboard_ratio", -0.5); setf("a380x/xp12/visual/aileron_r_outboard_ratio", -0.4);
    setf("a380x/xp12/hydraulics/elevator_ratio", 0.1); setf("a380x/xp12/hydraulics/rudder_ratio", -0.2);
    static const char *const groups[] = {"inboard", "inner", "middle", "outer"};
    for (int i = 0; i < 4; ++i) {
        snprintf(name, sizeof(name), "a380x/xp12/hydraulics/spoiler_l_%s_deg", groups[i]); setf(name, 10.0 * (i + 1));
        snprintf(name, sizeof(name), "a380x/xp12/hydraulics/spoiler_r_%s_deg", groups[i]); setf(name, 0.0);
    }
    setf("sim/flightmodel2/controls/elevator_trim", 0.5);
    setf("sim/aircraft/controls/acf_hstb_trim_up", 10.0); setf("sim/aircraft/controls/acf_hstb_trim_dn", 10.0);
    setf("sim/flightmodel2/controls/rudder_trim", 0.1);
    setf("sim/aircraft/controls/acf_max_trim_rudd", 1.0); setf("sim/aircraft/controls/acf_min_trim_rudd", 1.0);
    seti("a380x/xp12/ovhd/fail/sec1", 0); seti("a380x/xp12/ovhd/fail/prim2", 1);
    setf("sim/weather/aircraft/barometer_current_pas", 101325.0);
    for (int i = 1; i <= 3; ++i) {
        snprintf(name, sizeof(name), "a380x/xp12/adirs/adr%d_valid", i); seti(name, i != 3);
        snprintf(name, sizeof(name), "a380x/xp12/adirs/adr%d_sat_c", i); setf(name, 9.0);
        snprintf(name, sizeof(name), "a380x/xp12/adirs/adr%d_tat_c", i); setf(name, 11.0);
        snprintf(name, sizeof(name), "a380x/xp12/adirs/adr%d_altitude_ft", i); setf(name, 300.0);
        snprintf(name, sizeof(name), "a380x/xp12/adirs/ir%d_attitude_valid", i); seti(name, 1);
    }
    setf("a380x/xp12/adirs/adr1_ias_knots", 12.0);
    seti("a380x/xp12/adirs/captain_adr_source", 0); seti("a380x/xp12/adirs/first_officer_adr_source", 1);
    seti("a380x/xp12/adirs/captain_ir_source", 0);
    setf("sim/flightmodel/forces/g_nrml", 1.0);
    seti("a380x/xp12/efis/captain/baro_std", 1); seti("a380x/xp12/efis/first_officer/baro_std", 0);
    setf("sim/flightmodel/weight/m_fuel_total", 150000.0); setf("sim/flightmodel/weight/m_total", 450000.0);
}

static void test_full_state(void)
{
    full_state();
    emit(10.0);
    setf("sim/time/total_running_time_sec", 104.0);   /* 4 s of simulated time */
    emit(10.25);
    CHECK(!out.failed && json_ok(out.data));
    CHECK(out.size < XPKIT_SD_CAP / 2);   /* the whole SD frame must stay under XPKIT_SD_CAP */
    CHECK(!strstr(out.data, "nan") && !strstr(out.data, "inf"));
    /* ELEC AC / DC: FBW names, AC ESS = real AC EMER, contactor ids with '.' */
    CHECK(var("L:A32NX_ELEC_AC_1_BUS_IS_POWERED") == 1 && var("L:A32NX_ELEC_AC_ESS_BUS_IS_POWERED") == 1);
    CHECK(var("L:A32NX_ELEC_CONTACTOR_3XB.1_IS_CLOSED") == 1 && var("L:A32NX_ELEC_CONTACTOR_3XB.2_IS_CLOSED") == 0);
    CHECK(var("L:A32NX_ELEC_CONTACTOR_990XU3_IS_CLOSED") == 1 && var("L:A32NX_ELEC_CONTACTOR_980PC_IS_CLOSED") == 0);
    CHECK(var("L:A32NX_ELEC_ENG_GEN_2_POTENTIAL") == 116 && var("L:A32NX_ELEC_ENG_GEN_4_LOAD") == 23);
    CHECK(near(var("L:A32NX_ELEC_BAT_3_CURRENT"), -3, 1e-6) && near(var("L:A32NX_ELEC_TR_4_POTENTIAL"), 28.3, 1e-4));
    CHECK(var("GENERAL ENG MASTER ALTERNATOR:3") == 1 && var("L:A32NX_EXT_PWR_AVAIL:2") == 1 && var("L:A32NX_EXT_PWR_AVAIL:1") == 0);
    CHECK(var("APU GENERATOR SWITCH:1") == 1 && var("APU GENERATOR SWITCH:2") == 0);
    CHECK(var("L:A32NX_OVHD_ELEC_BAT_APU_PB_IS_AUTO") == 0 && var("L:A32NX_ELEC_309PP_BUS_IS_POWERED") == 1);
    /* APU: ARINC words with NO SSM; fuel used NCD (no source) */
    CHECK(near(word("L:A32NX_APU_N", SDD_SSM_NO), 100, 1e-4) && near(word("L:A32NX_APU_EGT", SDD_SSM_NO), 420, 1e-3));
    CHECK(near(word("L:A32NX_APU_EGT_WARNING", SDD_SSM_NO), 900, 1e-3));
    CHECK(near(word("L:A32NX_APU_FUEL_USED", SDD_SSM_NCD), 0, 0));
    CHECK(near(word("L:A32NX_ADIRS_ADR_1_CORRECTED_AVERAGE_STATIC_PRESSURE", SDD_SSM_NO), 1013.25, 0.01));
    word("L:A32NX_ADIRS_ADR_3_CORRECTED_AVERAGE_STATIC_PRESSURE", SDD_SSM_NCD);
    CHECK(near(var("L:A32NX_PNEU_APU_BLEED_CONTAINER_PRESSURE"), 54.696, 1e-3) && var("L:A32NX_APU_FLAP_OPEN_PERCENTAGE") == 100);
    CHECK(var("L:A32NX_ATT_HDG_SWITCHING_KNOB") == 1 && var("L:A32NX_OVHD_APU_START_PB_IS_AVAILABLE") == 1);
    /* ENGINE: X-Plane N2 = FBW N3; FBW N2 = N3 + 0.7; FF kg/h; FBW oil polynomials; F -> C */
    CHECK(var("L:A32NX_ENGINE_N3:2") == 61 && near(var("L:A32NX_ENGINE_N2:2"), 61.7, 1e-6));
    CHECK(var("L:A32NX_ENGINE_FF:1") == 900 && var("L:A32NX_ENGINE_STATE:4") == 1 && var("L:A32NX_ENGINE_IDLE_N1") == 18.7);
    CHECK(near(var("ENG OIL PRESSURE:1"), -0.88921 + 0.23711 * 60 + 0.00682 * 3600, 1e-3));
    CHECK(near(var("GENERAL ENG OIL TEMPERATURE:1"), 80, 1e-3));
    CHECK(near(var("L:A32NX_ENGINE_OIL_QTY:1"), 18.5 * (1 - 0.201968848), 1e-3));
    CHECK(var("L:XMLVAR_ENG_MODE_SEL") == 1 && var("L:A32NX_PNEU_ENG_1_STARTER_VALVE_OPEN") == 0);
    /* BLEED: APU bleed off, PRV open on running engines, FBW 40 psi / 200 C */
    CHECK(var("L:A32NX_PNEU_ENG_1_PR_VALVE_OPEN") == 1 && var("L:A32NX_PNEU_ENG_1_HP_VALVE_OPEN") == 1);
    CHECK(var("L:A32NX_PNEU_ENG_1_REGULATED_TRANSDUCER_PRESSURE") == 40 && var("L:A32NX_PNEU_ENG_4_REGULATED_TRANSDUCER_PRESSURE") == 20);
    CHECK(var("L:A32NX_PNEU_ENG_2_PRECOOLER_OUTLET_TEMPERATURE") == 200);
    CHECK(var("L:A32NX_PNEU_XBLEED_VALVE_L_OPEN") == 1 && var("L:A32NX_PNEU_XBLEED_VALVE_C_OPEN") == 0);
    CHECK(var("L:A32NX_COND_PACK_2_FLOW_VALVE_2_IS_OPEN") == 1 && var("L:A32NX_PNEU_PACK_2_FLOW_VALVE_1_FLOW_RATE") == 0.25);
    /* COND: CPIOM B words (pack operating bits 13/14) and the zone model */
    word("L:A32NX_COND_CPIOM_B3_AGS_DISCRETE_WORD", SDD_SSM_NO);
    CHECK(bit("L:A32NX_COND_CPIOM_B3_AGS_DISCRETE_WORD", 13) && bit("L:A32NX_COND_CPIOM_B3_AGS_DISCRETE_WORD", 14));
    CHECK(bit("L:A32NX_COND_CPIOM_B1_TCS_DISCRETE_WORD", 15) && bit("L:A32NX_COND_CPIOM_B1_VCS_DISCRETE_WORD", 17));
    CHECK(!bit("L:A32NX_COND_CPIOM_B1_VCS_DISCRETE_WORD", 22) && !bit("L:A32NX_COND_CPIOM_B2_CPCS_DISCRETE_WORD", 11));
    CHECK(var("L:A32NX_COND_MAIN_DECK_8_TEMP") == 24 && var("L:A32NX_COND_CARGO_BULK_TEMP") == 15);
    /* CAB PRESS */
    CHECK(near(word("L:A32NX_PRESS_CABIN_ALTITUDE_B2", SDD_SSM_NO), 50, 1e-4));
    CHECK(near(word("L:A32NX_PRESS_OUTFLOW_VALVE_3_OPEN_PERCENTAGE_B1", SDD_SSM_NO), 80, 1e-3));
    CHECK(near(word("L:A32NX_PRESS_CABIN_ALTITUDE_TARGET_B1", SDD_SSM_NO), 6000, 1e-3));
    CHECK(near(word("L:A32NX_FM1_LANDING_ELEVATION", SDD_SSM_NO), 1234, 1e-3) && var("L:A32NX_FWC_FLIGHT_PHASE") == 2);
    CHECK(near(var("L:A32NX_PRESS_MAN_CABIN_DELTA_PRESSURE"), 0.1, 1e-6) && var("L:A32NX_OVHD_PRESS_MAN_ALTITUDE_PB_IS_AUTO") == 1);
    /* HYD: 3000 psi ACF -> 5000 psi A380 scale, FBW switch hysteresis, reservoir */
    CHECK(near(var("L:A32NX_HYD_GREEN_SYSTEM_1_SECTION_PRESSURE"), 5000, 1e-3) && var("L:A32NX_HYD_GREEN_SYSTEM_1_SECTION_PRESSURE_SWITCH") == 1);
    CHECK(var("L:A32NX_HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE_SWITCH") == 0 && var("L:A32NX_HYD_YELLOW_RESERVOIR_LEVEL_IS_LOW") == 1);
    CHECK(var("L:A32NX_HYD_GREEN_RESERVOIR_LEVEL") == 12 && var("L:A32NX_HYD_GREEN_PUMP_4_SECTION_PRESSURE_SWITCH") == 1);
    CHECK(var("L:A32NX_HYD_YELLOW_PUMP_1_SECTION_PRESSURE_SWITCH") == 0 && var("L:A32NX_OVHD_HYD_ENG_3B_PUMP_PB_IS_AUTO") == 1);
    CHECK(var("L:A32NX_HYD_YA_EPUMP_ACTIVE") == 1 && var("L:A32NX_HYD_GA_EPUMP_ACTIVE") == 0 && var("L:A32NX_HYD_GREEN_PUMP_1_FIRE_VALVE_OPENED") == 1);
    /* WHEEL: left = min(wing, body); door open while that side's gear is in transit; brakes */
    CHECK(var("L:A32NX_GEAR_LEFT_POSITION") == 50 && var("L:A32NX_GEAR_DOOR_LEFT_POSITION") == 100 && var("L:A32NX_GEAR_DOOR_RIGHT_POSITION") == 0);
    CHECK(var("L:A32NX_REPORTED_BRAKE_TEMPERATURE_1") == 210 && var("L:A32NX_REPORTED_BRAKE_TEMPERATURE_16") == 110);
    CHECK(var("L:A32NX_REPORTED_BRAKE_TEMPERATURE_3") == 10 && var("L:A32NX_REPORTED_BRAKE_TEMPERATURE_null") == 0);
    /* DOOR */
    CHECK(var("INTERACTIVE POINT OPEN:3") == 100 && var("INTERACTIVE POINT OPEN:0") == 0);
    CHECK(var("L:A32NX_FWD_DOOR_CARGO_LOCKED") == 0 && var("L:A32NX_AFT_DOOR_CARGO_LOCKED") == 1 && var("LIGHT BEACON ON") == 1);
    /* Native per-door escape slides (EFB), never the beacon proxy */
    CHECK(var("L:A380X_SLIDE_ARMED:0") == 1 && var("L:A380X_SLIDE_ARMED:1") == 0 && var("L:A380X_SLIDE_ARMED:3") == 0);
    CHECK(var("L:A380X_SLIDE_ARMED:14") == 1 && var("L:A380X_SLIDE_ARMED:15") == 0 && !has("L:A380X_SLIDE_ARMED:16"));
    /* FUEL: tanks, FU integration (4 s at 0.25 kg/s), LP valves, pump words */
    CHECK(near(word("L:A32NX_FQMS_FEED_2_TANK_QUANTITY", SDD_SSM_NO), 7753.2 * 3.0387, 0.01));
    CHECK(near(word("L:A32NX_FQMS_LEFT_INNER_TANK_QUANTITY", SDD_SSM_NO), 50000 - 7753.2 * 3.0387, 0.01));
    CHECK(near(word("L:A32NX_FQDC_2_LEFT_MID_TANK_QUANTITY", SDD_SSM_NO), 15000, 0.01) && near(word("L:A32NX_FQMS_TRIM_TANK_QUANTITY", SDD_SSM_NO), 10000, 0.01));
    CHECK(near(var("L:A32NX_FUEL_USED:1"), 1.0, 1e-6));
    CHECK(var("FUELSYSTEM VALVE OPEN:1") == 1 && var("FUELSYSTEM VALVE OPEN:3") == 0 && var("FUELSYSTEM VALVE OPEN:51") == 1);
    CHECK(var("FUELSYSTEM VALVE OPEN:20") == 0 && var("FUELSYSTEM VALVE OPEN:59") == 0 && var("FUELSYSTEM VALVE OPEN:46") == 0);
    CHECK(bit("L:A32NX_FQMS_LEFT_FUEL_PUMP_RUNNING_WORD", 12) && bit("L:A32NX_FQMS_LEFT_FUEL_PUMP_RUNNING_WORD", 14));
    CHECK(bit("L:A32NX_FQMS_LEFT_FUEL_PUMP_RUNNING_WORD", 16) && bit("L:A32NX_FQMS_LEFT_FUEL_PUMP_RUNNING_WORD", 17));
    CHECK(!bit("L:A32NX_FQMS_RIGHT_FUEL_PUMP_RUNNING_WORD", 12) && bit("L:A32NX_FQMS_RIGHT_FUEL_PUMP_RUNNING_WORD", 14));
    /* F/CTL bus topics */
    CHECK(var("ev:leftInnerAileronDeflection") == 0.5 && near(var("ev:rightOuterAileronDeflection"), -0.4, 1e-6));
    CHECK(near(var("ev:leftOuterElevatorDeflection"), 0.1, 1e-6) && near(var("ev:lowerRudderDeflection"), -0.2, 1e-6));
    CHECK(near(var("ev:leftSpoiler8Deflection"), 0.8, 1e-6) && near(var("ev:leftSpoiler1Deflection"), 0.2, 1e-6));
    CHECK(near(var("ev:thsDeflection"), 5.0 * 3.14159265358979 / 180.0, 1e-6) && var("ev:fctlInfoValid") == 1);
    CHECK(strstr(out.data, "\"ev:prim2Healthy\":false") && strstr(out.data, "\"ev:sec1Healthy\":true"));
    CHECK(bit("ev:sec1RudderStatusWord", 28) && bit("ev:sec1RudderStatusWord", 27) && !bit("ev:sec3RudderStatusWord", 28));
    {
        double v; int ssm; decode(var("ev:sec1RudderTrimPosition"), &v, &ssm);
        CHECK(near(v, -3.0, 1e-5) && ssm == SDD_SSM_NO);   /* X-Plane right trim -> FBW + = LEFT */
    }
    CHECK(strstr(out.data, "\"ev:greenPressureSwitch\":true") && strstr(out.data, "\"ev:acEssPowered\":true"));
    /* STATUS (native FWS ids, exact integers) */
    CHECK(strstr(out.data, "\"L:A32NX_SD_STATUS_INFO_LINE_1\":210200001,") && var("L:A32NX_SD_STATUS_INOP_ALL_LINE_1") == 210300009);
    CHECK(var("L:A32NX_EWD_LIMITATIONS_ALL_LINE_1") == 220500001 && var("L:A32NX_FWS1_IS_HEALTHY") == 1);
    /* permanent data */
    CHECK(near(word("L:A32NX_ADIRS_ADR_1_TOTAL_AIR_TEMPERATURE", SDD_SSM_NO), 11, 1e-5) && bit("L:A32NX_FCU_EFIS_L_DISCRETE_WORD_2", 11));
    CHECK(near(word("L:A32NX_FQMS_GROSS_WEIGHT", SDD_SSM_NO), 450000, 0.1) && near(word("L:A32NX_FQMS_CENTER_OF_GRAVITY_MAC", SDD_SSM_NCD), 0, 0));
    /* never invent failures */
    static const char *const failures_off[] = {"L:A32NX_COND_FDAC_1_CHANNEL_1_FAILURE", "L:A32NX_COND_TADD_CHANNEL_2_FAILURE",
        "L:A32NX_VENT_AFT_VCM_CHANNEL_1_FAILURE", "L:A32NX_PRESS_OCSM_4_AUTO_PARTITION_FAILURE", "L:A32NX_HYD_GREEN_RESERVOIR_OVHT",
        "L:A32NX_HYD_YB_EPUMP_OVHT", "L:A32NX_HYD_ENG_2AB_PUMP_DISC", "L:A32NX_FIRE_BUTTON_ENG1"};
    for (size_t i = 0; i < sizeof(failures_off) / sizeof(*failures_off); ++i) CHECK(var(failures_off[i]) == 0);
}

static void test_ground_start_and_cold(void)
{
    /* Engine 1 starting on the ground resets its fuel used (FBW). */
    setvi("sim/flightmodel2/engines/starter_is_running", 4, 1, 0, 0, 0);
    setvf("sim/cockpit2/engine/indicators/fuel_flow_kg_sec", 4, 0.0, 0.25, 0.25, 0.25);
    setf("sim/time/total_running_time_sec", 105.0);
    emit(11.0);
    CHECK(var("L:A32NX_ENGINE_STATE:1") == 2 && var("L:A32NX_FUEL_USED:1") == 0 && var("L:A32NX_PNEU_ENG_1_STARTER_VALVE_OPEN") == 1);
    CHECK(var("L:XMLVAR_ENG_MODE_SEL") == 2 && near(var("L:A32NX_FUEL_USED:2"), 1.25, 1e-6));
    /* Cold & dark: unpowered computers report failure-warning SSM, not invented failures. */
    static const char *const buses[] = {"ac1", "ac2", "ac3", "ac4", "ac_ess", "ac_emer", "ac_eha", "dc1", "dc2", "dc_ess", "dc_apu"};
    char name[96];
    for (size_t i = 0; i < sizeof(buses) / sizeof(*buses); ++i) {
        snprintf(name, sizeof(name), "a380x/xp12/elec/bus/%s_powered", buses[i]); seti(name, 0);
    }
    seti("a380x/xp12/apu/ecb_powered", 0);
    emit(12.0);
    CHECK(json_ok(out.data));
    word("L:A32NX_COND_CPIOM_B1_AGS_DISCRETE_WORD", SDD_SSM_FW);
    word("L:A32NX_FQMS_FEED_1_TANK_QUANTITY", SDD_SSM_FW);
    word("L:A32NX_APU_N", SDD_SSM_FW);
    CHECK(var("L:A32NX_ELEC_DC_ESS_BUS_IS_POWERED") == 0 && strstr(out.data, "\"ev:dcEssPowered\":false"));
    CHECK(strstr(out.data, "\"ev:prim1Healthy\":false") && var("L:A32NX_COND_PACK_1_FLOW_VALVE_1_IS_OPEN") == 0);
}

static void test_absent_inputs(void)
{
    mock_count = 0;
    memset(sdd_slots, 0, sizeof(sdd_slots));
    memset(&sdd, 0, sizeof(sdd));
    emit(100.0);
    CHECK(!out.failed && json_ok(out.data) && !strstr(out.data, "nan"));
    CHECK(!has("L:A32NX_ELEC_AC_1_BUS_IS_POWERED") && !has("L:A32NX_FQMS_FEED_1_TANK_QUANTITY"));
    CHECK(!has("L:A32NX_ENGINE_N3:1") && !has("INTERACTIVE POINT OPEN:0") && !has("L:A32NX_COND_CKPT_TEMP"));
    CHECK(!has("L:A380X_SLIDE_ARMED:0"));   /* no EFB slide state: the bundle keeps FBW's beacon proxy */
    CHECK(var("ev:fctlInfoValid") == 0);
    /* A producer registered later is found by the once-per-second re-find. */
    seti("a380x/xp12/elec/network_active", 1);
    emit(100.5);
    CHECK(!has("L:A32NX_ELEC_AC_1_BUS_IS_POWERED"));
    emit(101.2);
    CHECK(has("L:A32NX_ELEC_AC_1_BUS_IS_POWERED"));
}

static void test_arinc_encoding(void)
{
    double v; int ssm;
    CHECK(sdd_raw_word(0.0, SDD_SSM_FW) == 0.0);
    CHECK(sdd_raw_word(1.0, SDD_SSM_NO) == 1065353216.0 + 3.0 * 4294967296.0);
    CHECK(sdd_raw_word(2048.0, SDD_SSM_NCD) == 1157627904.0 + 4294967296.0);
    decode(sdd_raw_word(-12.5, SDD_SSM_FT), &v, &ssm);
    CHECK(v == -12.5 && ssm == SDD_SSM_FT);
}

int main(int argc, char **argv)
{
    if (argc > 1 && !strcmp(argv[1], "--dump")) {
        full_state(); emit(10.0); setf("sim/time/total_running_time_sec", 104.0); emit(10.25);
        puts(out.data);
        return 0;
    }
    if (argc > 1 && !strcmp(argv[1], "--contracts")) {
        full_state(); emit(10.0);
        for (int i = 0; i < SDD_SLOTS; ++i)
            if (sdd_slots[i].used && !strncmp(sdd_slots[i].name, "sim/", 4)) printf("R %s\n", sdd_slots[i].name);
        return 0;
    }
    test_arinc_encoding();
    test_full_state();
    size_t full_size = out.size;
    test_ground_start_and_cold();
    test_absent_inputs();
    if (failures) { fprintf(stderr, "%d SD data check(s) failed\n", failures); return 1; }
    printf("SD data publisher: per-page values, ARINC raw words, JSON %zu bytes (cap %d), absent inputs OK\n",
           full_size, XPKIT_SD_CAP);
    return 0;
}
#endif
