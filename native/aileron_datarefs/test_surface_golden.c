/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Lua-vs-C golden test of the native exterior surface driver.
 *
 * tests/surface_golden_lua.lua runs the real FlyWithLua drivers (A380X_XP12_Ailerons.lua,
 * A380X_XP12_Hydraulics.lua and their modules) on scripted inputs and writes, per frame, the 21
 * simulator inputs and the 22 published animation values. This test links the REAL plugin
 * (aileron_datarefs.c + a380x_surface_motion.c) to a mock XPLM, feeds each frame's inputs through
 * the simulator datarefs, runs the plugin's flight loop and reads the 22 values back through the
 * plugin's own registered accessors. A scenario start is a fresh plugin (Stop/Start/Enable); an "R"
 * line is an XPLM_MSG_PLANE_LOADED (FlyWithLua reloads its scripts). Every value must match.
 *
 *   test_surface_golden <golden file> [tolerance]
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

int XPluginStart(char *, char *, char *);
void XPluginStop(void);
int XPluginEnable(void);
void XPluginDisable(void);
void XPluginReceiveMessage(XPLMPluginID, int, void *);

/* ---- mock XPLM ---- */
enum { SIM_SCALARS = 13, SIM_ARRAYS = 3, CUSTOM_MAX = 32, INPUTS = 21, OUTPUTS = 22 };
static const char *sim_names[SIM_SCALARS + SIM_ARRAYS] = {
    "sim/flightmodel2/controls/roll_ratio", "sim/flightmodel2/controls/pitch_ratio",
    "sim/flightmodel2/controls/heading_ratio", "sim/flightmodel/failures/onground_any", "sim/time/paused",
    "sim/operation/misc/frame_rate_period", "sim/flightmodel/position/psi",
    "sim/weather/aircraft/wind_now_direction_degt", "sim/weather/aircraft/wind_now_speed_msc",
    "sim/flightmodel/position/true_airspeed", "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1",
    "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_2",
    "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_3",
    "sim/flightmodel2/wing/speedbrake1_deg", "sim/flightmodel2/wing/spoiler1_deg",
    "sim/flightmodel2/wing/spoiler2_deg"};
static double sim_scalar[SIM_SCALARS];
static float sim_array[SIM_ARRAYS][48];
static int sim_handle[SIM_SCALARS + SIM_ARRAYS];
typedef struct {
    char name[128];
    int active, type;
    XPLMGetDataf_f read_float;
    XPLMGetDatai_f read_int;
    void *refcon;
} Custom;
static Custom custom[CUSTOM_MAX];
static int custom_count;
static XPLMFlightLoop_f loop_fn;
static void *loop_refcon;
static int loop_alive, loop_scheduled;

static int sim_index(XPLMDataRef ref)
{
    int i;
    for (i = 0; i < SIM_SCALARS + SIM_ARRAYS; ++i) if (ref == &sim_handle[i]) return i;
    return -1;
}

XPLMDataRef XPLMFindDataRef(const char *name)
{
    int i;
    for (i = 0; i < SIM_SCALARS + SIM_ARRAYS; ++i) if (!strcmp(name, sim_names[i])) return &sim_handle[i];
    for (i = 0; i < custom_count; ++i) if (custom[i].active && !strcmp(name, custom[i].name)) return &custom[i];
    return NULL;
}

XPLMDataTypeID XPLMGetDataRefTypes(XPLMDataRef ref)
{
    int i = sim_index(ref);
    assert(i >= 0);
    return i >= SIM_SCALARS ? xplmType_FloatArray : (i == 3 || i == 4) ? xplmType_Int : xplmType_Float;
}

float XPLMGetDataf(XPLMDataRef ref)
{
    int i = sim_index(ref);
    assert(i >= 0 && i < SIM_SCALARS && i != 3 && i != 4);
    return (float)sim_scalar[i];
}

int XPLMGetDatai(XPLMDataRef ref)
{
    int i = sim_index(ref);
    assert(i == 3 || i == 4);
    return (int)sim_scalar[i];
}

int XPLMGetDatavf(XPLMDataRef ref, float *values, int offset, int count)
{
    int i = sim_index(ref) - SIM_SCALARS;
    assert(i >= 0 && i < SIM_ARRAYS && offset >= 0 && offset + count <= 48);
    memcpy(values, &sim_array[i][offset], sizeof(float) * (size_t)count);
    return count;
}

XPLMDataRef XPLMRegisterDataAccessor(
    const char *name, XPLMDataTypeID type, int writable,
    XPLMGetDatai_f read_int, XPLMSetDatai_f write_int,
    XPLMGetDataf_f read_float, XPLMSetDataf_f write_float,
    XPLMGetDatad_f read_double, XPLMSetDatad_f write_double,
    XPLMGetDatavi_f read_ints, XPLMSetDatavi_f write_ints,
    XPLMGetDatavf_f read_floats, XPLMSetDatavf_f write_floats,
    XPLMGetDatab_f read_bytes, XPLMSetDatab_f write_bytes,
    void *read_refcon, void *write_refcon)
{
    Custom *c;
    (void)writable; (void)write_int; (void)write_float; (void)read_double; (void)write_double;
    (void)read_ints; (void)write_ints; (void)read_floats; (void)write_floats; (void)read_bytes;
    (void)write_bytes; (void)write_refcon;
    assert(custom_count < CUSTOM_MAX && XPLMFindDataRef(name) == NULL);
    c = &custom[custom_count++];
    snprintf(c->name, sizeof(c->name), "%s", name);
    c->active = 1; c->type = type; c->read_float = read_float; c->read_int = read_int; c->refcon = read_refcon;
    return c;
}

void XPLMUnregisterDataAccessor(XPLMDataRef ref)
{
    Custom *c = (Custom *)ref;
    assert(c >= custom && c < custom + CUSTOM_MAX && c->active);
    c->active = 0;
}

void XPLMDebugString(const char *text) { (void)text; }

XPLMFlightLoopID XPLMCreateFlightLoop(XPLMCreateFlightLoop_t *params)
{
    assert(params && params->callbackFunc && !loop_alive);
    loop_fn = params->callbackFunc; loop_refcon = params->refcon; loop_alive = 1; loop_scheduled = 0;
    return (XPLMFlightLoopID)&loop_alive;
}
void XPLMDestroyFlightLoop(XPLMFlightLoopID id) { assert(id == (XPLMFlightLoopID)&loop_alive && loop_alive); loop_alive = 0; }
void XPLMScheduleFlightLoop(XPLMFlightLoopID id, float interval, int relative)
{
    assert(id == (XPLMFlightLoopID)&loop_alive && interval < 0 && relative);
    loop_scheduled = 1;
}

/* ---- helpers ---- */
static void restart_plugin(void)
{
    char name[256], sig[256], desc[256];
    XPluginStop();
    custom_count = 0;
    memset(custom, 0, sizeof(custom));
    assert(XPluginStart(name, sig, desc) == 1);
    assert(XPluginEnable() == 1 && loop_alive && loop_scheduled);
}

static float published(int index)
{
    assert(index < custom_count && custom[index].active && custom[index].type == xplmType_Float);
    return custom[index].read_float(custom[index].refcon);
}

static int native_flag(void)
{
    Custom *c = (Custom *)XPLMFindDataRef("a380x/xp12/hydraulics/native_driver");
    assert(c && c->type == xplmType_Int && c->read_int);
    return c->read_int(c->refcon);
}

static double parse(const char *token)
{
    if (!strcmp(token, "nan") || !strcmp(token, "-nan")) return NAN;
    if (!strcmp(token, "inf")) return INFINITY;
    if (!strcmp(token, "-inf")) return -INFINITY;
    return strtod(token, NULL);
}

int main(int argc, char **argv)
{
    static char line[16384];
    FILE *golden;
    double tolerance = argc > 2 ? atof(argv[2]) : 0.0, worst = 0.0;
    long frames = 0, values = 0, bit_exact = 0, scenarios = 0, reloads = 0, mismatches = 0;
    double coverage_min[OUTPUTS], coverage_max[OUTPUTS];
    int pending_reload = 0, i;
    if (argc < 2) { fprintf(stderr, "usage: %s <golden file> [tolerance]\n", argv[0]); return 2; }
    golden = fopen(argv[1], "r");
    if (!golden) { perror(argv[1]); return 2; }
    for (i = 0; i < OUTPUTS; ++i) { coverage_min[i] = INFINITY; coverage_max[i] = -INFINITY; }
    while (fgets(line, sizeof(line), golden)) {
        char *tok, *save = NULL;
        double in[INPUTS], expect[OUTPUTS];
        int n = 0, m = 0;
        if (line[0] == '#') continue;
        if (line[0] == 'S') { restart_plugin(); ++scenarios; continue; }
        if (line[0] == 'R') { pending_reload = 1; ++reloads; continue; }
        assert(line[0] == 'F');
        for (tok = strtok_r(line + 1, " \n", &save); tok; tok = strtok_r(NULL, " \n", &save)) {
            if (!strcmp(tok, "|")) { assert(n == INPUTS); continue; }
            if (n < INPUTS) in[n++] = parse(tok);
            else { assert(m < OUTPUTS); expect[m++] = parse(tok); }
        }
        assert(n == INPUTS && m == OUTPUTS);
        for (i = 0; i < SIM_SCALARS; ++i) sim_scalar[i] = in[i];
        sim_array[0][0] = (float)in[13]; sim_array[0][1] = (float)in[14];
        for (i = 0; i < 4; ++i) sim_array[1][i] = (float)in[15 + i];
        sim_array[2][4] = (float)in[19]; sim_array[2][5] = (float)in[20];
        if (pending_reload) { XPluginReceiveMessage(0, XPLM_MSG_PLANE_LOADED, (void *)(intptr_t)0); pending_reload = 0; }
        assert(loop_fn(0.0f, 0.0f, (int)frames, loop_refcon) < 0.0f);
        assert(native_flag() == 1);
        for (i = 0; i < OUTPUTS; ++i) {
            double got = published(i), err = fabs(got - expect[i]);
            if (got == expect[i]) ++bit_exact;
            if (!(err <= tolerance)) {
                if (mismatches < 10)
                    fprintf(stderr, "frame %ld output %d (%s): C %.9g Lua %.9g\n", frames, i, custom[i].name, got, expect[i]);
                ++mismatches;
            }
            if (err > worst) worst = err;
            if (got < coverage_min[i]) coverage_min[i] = got;
            if (got > coverage_max[i]) coverage_max[i] = got;
            ++values;
        }
        ++frames;
    }
    fclose(golden);
    XPluginStop();
    if (mismatches || frames == 0) {
        fprintf(stderr, "surface golden FAILED: %ld of %ld values differ (worst %.3g)\n", mismatches, values, worst);
        return 1;
    }
    printf("Surface golden: %ld scenarios, %ld frames, %ld reloads, %ld values, %ld bit-exact, worst |C-Lua| %.3g;"
           " ranges", scenarios, frames, reloads, values, bit_exact, worst);
    for (i = 0; i < OUTPUTS; ++i) printf(" %.2f..%.2f", coverage_min[i], coverage_max[i]);
    printf("\n");
    return 0;
}
