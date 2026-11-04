/* SPDX-License-Identifier: GPL-3.0-or-later
 * Overhead annunciator lamps: X-Plane adapter (docs/OVERHEAD_LAMPS.md).
 *
 * Publishes a380x/xp12/ovhd/lamp/<id>_upper|_lower (float 0..1, registered in XPluginStart so
 * the cockpit objects bind them at load) for every authored overhead legend outside the ELEC
 * and ADIRS panels. Each legend follows a380x_ovhd_lamps.h: its FBW condition, driven from the
 * port's native state (APU ECB) or the X-Plane dataref the pushbutton controls
 * (config/cockpit_controls.json), and FBW's per-pushbutton supply bus from the native A380
 * network (a380x_elec_power.h). ANN LT (TEST / BRT / DIM) is shared with the ELEC panel
 * (a380x/xp12/elec/ann_lt is the same state) and also lights the ADIRS legends in TEST.
 * a380x/xp12/ovhd/settings/lamp_supply = 1 selects a simplified supply (not FBW) for every
 * overhead legend including the ELEC panel: lit whenever any DC bus is live or a battery pb
 * is AUTO.
 * Unity-build include of a380x_avionics.c after a380x_elec_adapter.inc and the ADIRS adapter.
 */
#include "a380x_ovhd_lamps.h"

enum {
    OVR_APU_SWITCH, OVR_APU_N1, OVR_APU_BLEED, OVR_ENG_BLEED, OVR_PACK_L, OVR_PACK_R, OVR_ENG_AI,
    OVR_WING_AI, OVR_WINDOW_HEAT, OVR_FUEL_PUMP, OVR_O2, OVR_BUS_VOLTS,
    OVR_FAIL_APU, OVR_FAIL_APU_FIRE, OVR_FAIL_BLEED_L, OVR_FAIL_BLEED_R,
    OVR_FAIL_INLET1, OVR_FAIL_INLET2, OVR_FAIL_INLET3, OVR_FAIL_INLET4, OVR_FAIL_SURF_HEAT,
    OVR_COUNT
};
static const char *const ovhd_raw_names[OVR_COUNT] = {
    "sim/cockpit2/electrical/APU_starter_switch",
    "sim/cockpit2/electrical/APU_N1_percent",
    "sim/cockpit2/bleedair/actuators/apu_bleed",
    "sim/cockpit2/bleedair/actuators/engine_bleed_sov",
    "sim/cockpit2/bleedair/actuators/pack_left",
    "sim/cockpit2/bleedair/actuators/pack_right",
    "sim/cockpit2/ice/ice_inlet_heat_on_per_engine",
    "sim/cockpit2/ice/ice_surface_hot_bleed_air_on",
    "sim/cockpit2/ice/ice_window_heat_on",
    "sim/cockpit2/engine/actuators/fuel_pump_on",
    "sim/cockpit2/oxygen/actuators/o2_valve_on",
    "sim/cockpit2/electrical/bus_volts",
    "sim/operation/failures/rel_apu",
    "sim/operation/failures/rel_apu_fire",
    "sim/operation/failures/rel_bleed_air_lft",
    "sim/operation/failures/rel_bleed_air_rgt",
    "sim/operation/failures/rel_ice_inlet_heat",
    "sim/operation/failures/rel_ice_inlet_heat2",
    "sim/operation/failures/rel_ice_inlet_heat3",
    "sim/operation/failures/rel_ice_inlet_heat4",
    "sim/operation/failures/rel_ice_surf_heat",
};
#define OVHD_ANN_SWITCH_SECONDS 0.12f   /* full lever travel TEST -> DIM */

static A380XOvhdInputs ovhd_in;
static struct {
    int ready, ann_lt, ann_pushed, lit_count;
    float switch_ratio;
    XPLMDataRef raw[OVR_COUNT];
} ovhd;

static int ovhd_rawi(int i) { return ovhd.raw[i] ? XPLMGetDatai(ovhd.raw[i]) : 0; }
static float ovhd_rawf(int i)
{
    float v = ovhd.raw[i] ? XPLMGetDataf(ovhd.raw[i]) : 0.0f;
    return isfinite(v) ? v : 0.0f;
}
static int ovhd_failed(int i) { return ovhd_rawi(i) >= 6; }

/* ANN LT is one state shared with the ELEC panel (elec_inputs.annunciator_test / _dim). */
static void ovhd_push_ann_lt(void)
{
    elec_inputs.annunciator_test = ovhd.ann_lt == A380X_OVHD_ANN_TEST;
    elec_inputs.annunciator_dim = ovhd.ann_lt == A380X_OVHD_ANN_DIM;
    ovhd.ann_pushed = ovhd.ann_lt;
    ovhd_in.ann_lt = ovhd.ann_lt;
}
static void ovhd_sync_ann_lt(void)
{
    if (elec_io.ready && elec_ann_lt() != ovhd.ann_pushed) ovhd.ann_lt = elec_ann_lt(); /* elec/ann_lt written */
    ovhd_push_ann_lt();
}
static void ovhd_set_supply(int mode)
{
    ovhd_in.supply_mode = mode == A380X_OVHD_SUPPLY_SIMPLE ? A380X_OVHD_SUPPLY_SIMPLE : A380X_OVHD_SUPPLY_FBW;
    eln.lamp_supply_simple = ovhd_in.supply_mode == A380X_OVHD_SUPPLY_SIMPLE;
}

static void ovhd_read_inputs(void)
{
    int n, pump[8] = {0}, bleed[8] = {0}, ai[8] = {0};
    float bus[6] = {0};
    int network = eln.ready && eln.active && a380x_power.available;
    ovhd_in.apu_master = ovhd_rawi(OVR_APU_SWITCH) >= 1;
    ovhd_in.apu_failed = ovhd_failed(OVR_FAIL_APU) || ovhd_failed(OVR_FAIL_APU_FIRE);
    if (network) {   /* native APU ECB (spec 9.3 option A) */
        ovhd_in.apu_start = (unsigned char)eln.st.apu_start_latched;
        ovhd_in.apu_avail = (unsigned char)eln.st.apu_avail;
    } else {
        ovhd_in.apu_start = ovhd_rawi(OVR_APU_SWITCH) == 2;
        ovhd_in.apu_avail = ovhd_rawf(OVR_APU_N1) >= 95.0f;
    }
    ovhd_in.apu_bleed = ovhd_rawi(OVR_APU_BLEED) != 0;
    if (ovhd.raw[OVR_ENG_BLEED]) XPLMGetDatavi(ovhd.raw[OVR_ENG_BLEED], bleed, 0, 4);
    if (ovhd.raw[OVR_ENG_AI]) XPLMGetDatavi(ovhd.raw[OVR_ENG_AI], ai, 0, 4);
    if (ovhd.raw[OVR_FUEL_PUMP]) XPLMGetDatavi(ovhd.raw[OVR_FUEL_PUMP], pump, 0, 4);
    for (n = 0; n < 4; ++n) {
        ovhd_in.eng_bleed[n] = bleed[n] != 0;
        ovhd_in.eng_bleed_failed[n] = ovhd_failed(n < 2 ? OVR_FAIL_BLEED_L : OVR_FAIL_BLEED_R);
        ovhd_in.eng_ai[n] = ai[n] != 0;
        ovhd_in.eng_ai_failed[n] = ovhd_failed(OVR_FAIL_INLET1 + n);
        ovhd_in.feed_pump[n] = pump[n] != 0;
    }
    ovhd_in.pack[0] = ovhd_rawi(OVR_PACK_L) != 0;
    ovhd_in.pack[1] = ovhd_rawi(OVR_PACK_R) != 0;
    ovhd_in.wing_ai = ovhd_rawi(OVR_WING_AI) != 0;
    ovhd_in.wing_ai_failed = ovhd_failed(OVR_FAIL_SURF_HEAT);
    ovhd_in.window_heat = ovhd_rawi(OVR_WINDOW_HEAT) != 0;
    ovhd_in.crew_oxy = ovhd_rawi(OVR_O2) != 0;
    ovhd_in.network = network;
    ovhd_in.buses = network ? a380x_power.bus : 0u;
    ovhd_in.simple_power = network && eln_simple_lamp_power();
    if (ovhd.raw[OVR_BUS_VOLTS]) XPLMGetDatavf(ovhd.raw[OVR_BUS_VOLTS], bus, 0, 6);
    ovhd_in.legacy_power = 0;
    for (n = 0; n < 6; ++n) if (isfinite(bus[n]) && bus[n] >= 18.0f) ovhd_in.legacy_power = 1;
}

/* ANN LT TEST also lights the ADIRS IR / ADR legends of every powered ADIRU (their OFF and FAULT
 * lamps are a380x_adirs.c integer outputs; recomputed each frame before this runs). */
static void ovhd_adirs_test(void)
{
    int i;
    if (ovhd.ann_lt != A380X_OVHD_ANN_TEST) return;
    for (i = 0; i < A380X_ADIRS_UNITS; ++i) {
        if (adirs_inputs.unit[i].ir_powered) adirs.unit[i].ir_off_lit = adirs.unit[i].ir_fault_lit = 1;
        if (adirs_inputs.unit[i].adr_powered) adirs.unit[i].adr_off_lit = adirs.unit[i].adr_fault_lit = 1;
    }
}

static void ovhd_update(float dt)
{
    float target, step;
    int pb, seq;
    if (!ovhd.ready) return;
    ovhd_sync_ann_lt();
    ovhd_read_inputs();
    target = 0.5f * (float)ovhd.ann_lt;
    step = (isfinite(dt) && dt > 0.0f ? fminf(dt, 0.1f) : 0.0f) * (1.0f / OVHD_ANN_SWITCH_SECONDS);
    ovhd.switch_ratio = ovhd.switch_ratio < target ? fminf(target, ovhd.switch_ratio + step)
                                                   : fmaxf(target, ovhd.switch_ratio - step);
    ovhd_adirs_test();
    ovhd.lit_count = 0;
    for (pb = 0; pb < A380X_OVHD_PB_COUNT; ++pb)
        for (seq = 0; seq < 2; ++seq) if (a380x_ovhd_lamp(&ovhd_in, pb, seq) > 0.0f) ++ovhd.lit_count;
}

/* ---------------------------------------------------------------- datarefs */
enum { OK_LAMP, OK_FAIL, OK_ANN_LT, OK_ANN_SWITCH, OK_SUPPLY, OK_COUNT_LIT, OK_PB };
typedef struct { char name[128]; int kind, pb, seq, writable, real; XPLMDataRef handle; } OvhdRef;
static OvhdRef ovhd_refs[A380X_OVHD_PB_COUNT * 4 + 8];
static int ovhd_ref_count;

static float ovhd_ref_value(const OvhdRef *r)
{
    switch (r->kind) {
    case OK_LAMP: return plugin_enabled && ovhd.ready ? a380x_ovhd_lamp(&ovhd_in, r->pb, r->seq) : 0.0f;
    case OK_FAIL: return ovhd_in.fault[r->pb];
    case OK_ANN_LT: return (float)ovhd.ann_lt;
    case OK_ANN_SWITCH: return ovhd.switch_ratio;
    case OK_SUPPLY: return (float)ovhd_in.supply_mode;
    case OK_COUNT_LIT: return (float)ovhd.lit_count;
    case OK_PB: return ovhd_in.pb[r->pb];
    default: return 0.0f;
    }
}
static int ovhd_get_int(void *refcon) { return (int)lroundf(ovhd_ref_value((const OvhdRef *)refcon)); }
static float ovhd_get_float(void *refcon) { return ovhd_ref_value((const OvhdRef *)refcon); }
static double ovhd_get_double(void *refcon) { return ovhd_ref_value((const OvhdRef *)refcon); }
static void ovhd_set_value(OvhdRef *r, float value)
{
    int position;
    if (!plugin_enabled || !r->writable || !isfinite(value)) return;
    switch (r->kind) {
    case OK_FAIL: ovhd_in.fault[r->pb] = value >= 0.5f; break;
    case OK_ANN_LT:
        position = (int)lroundf(value);
        ovhd.ann_lt = position <= 0 ? A380X_OVHD_ANN_TEST : position >= 2 ? A380X_OVHD_ANN_DIM : A380X_OVHD_ANN_BRT;
        ovhd_push_ann_lt();
        break;
    case OK_SUPPLY: ovhd_set_supply((int)lroundf(value)); break;
    case OK_PB: ovhd_in.pb[r->pb] = value >= 0.5f; break;
    default: return;
    }
    if (ovhd.ready) ovhd_read_inputs();
}
static void ovhd_set_int(void *refcon, int value) { ovhd_set_value((OvhdRef *)refcon, (float)value); }
static void ovhd_set_float(void *refcon, float value) { ovhd_set_value((OvhdRef *)refcon, value); }
static void ovhd_set_double(void *refcon, double value) { ovhd_set_value((OvhdRef *)refcon, (float)value); }

static void ovhd_add_ref(const char *suffix, int kind, int pb, int seq, int writable, int real)
{
    OvhdRef *r;
    if (ovhd_ref_count >= (int)(sizeof(ovhd_refs) / sizeof(ovhd_refs[0]))) return;
    r = &ovhd_refs[ovhd_ref_count++];
    snprintf(r->name, sizeof(r->name), "a380x/xp12/ovhd/%s", suffix);
    r->kind = kind; r->pb = pb; r->seq = seq; r->writable = writable; r->real = real; r->handle = NULL;
}
/* Order matches tools/ovhd_lamps_contract.py DATAREFS. */
static void ovhd_build_refs(void)
{
    char name[96];
    int pb, seq;
    ovhd_ref_count = 0;
    for (pb = 0; pb < A380X_OVHD_PB_COUNT; ++pb)
        for (seq = 0; seq < 2; ++seq) if (a380x_ovhd_pbs[pb].legend[seq] != A380X_OVHD_LEGEND_NONE) {
            snprintf(name, sizeof(name), "lamp/%s_%s", a380x_ovhd_pbs[pb].id, seq ? "lower" : "upper");
            ovhd_add_ref(name, OK_LAMP, pb, seq, 0, 1);
        }
    for (pb = 0; pb < A380X_OVHD_PB_COUNT; ++pb) {
        int legend = a380x_ovhd_pbs[pb].legend[0];
        if (legend == A380X_OVHD_LEGEND_FAULT || legend == A380X_OVHD_LEGEND_FAULT_WHITE) {
            snprintf(name, sizeof(name), "fail/%s", a380x_ovhd_pbs[pb].id);
            ovhd_add_ref(name, OK_FAIL, pb, 0, 1, 0);
        }
    }
    ovhd_add_ref("ann_lt", OK_ANN_LT, 0, 0, 1, 0);
    ovhd_add_ref("ann_lt_switch", OK_ANN_SWITCH, 0, 0, 0, 1);
    ovhd_add_ref("settings/lamp_supply", OK_SUPPLY, 0, 0, 1, 0);
    ovhd_add_ref("lamp_count", OK_COUNT_LIT, 0, 0, 0, 0);
    for (pb = 0; pb < A380X_OVHD_PB_COUNT; ++pb) if (a380x_ovhd_has_pb(pb)) {
        snprintf(name, sizeof(name), "pb/%s", a380x_ovhd_pbs[pb].id);
        ovhd_add_ref(name, OK_PB, pb, 0, 1, 0);
    }
}

/* ---------------------------------------------------------------- pushbutton positions */
static XPLMCommandRef ovhd_pb_commands[A380X_OVHD_PB_COUNT];
static int ovhd_pb_handler(XPLMCommandRef handle, XPLMCommandPhase phase, void *refcon)
{
    int pb = (int)(intptr_t)refcon;
    (void)handle;
    if (phase == xplm_CommandBegin && plugin_enabled && ovhd.ready) ovhd_in.pb[pb] = !ovhd_in.pb[pb];
    return 1;   /* the cockpit-feedback key travel handler on the same command still runs */
}
static void ovhd_pb_defaults(void)
{
    int pb;
    for (pb = 0; pb < A380X_OVHD_PB_COUNT; ++pb) ovhd_in.pb[pb] = (unsigned char)a380x_ovhd_pb_default(pb);
}
/* Airbus normal positions of the pushbuttons that drive X-Plane systems (fuel pumps, bleeds, packs):
 * X-Plane's cold & dark start leaves them off, which lit every OFF legend as soon as a battery was on.
 * Without AC power the pumps and packs do not run, so this only changes what the cockpit shows. */
static void ovhd_normal_xplane_positions(void)
{
    int on[4] = {1, 1, 1, 1};
    if (ovhd.raw[OVR_FUEL_PUMP]) XPLMSetDatavi(ovhd.raw[OVR_FUEL_PUMP], on, 0, 4);
    if (ovhd.raw[OVR_ENG_BLEED]) XPLMSetDatavi(ovhd.raw[OVR_ENG_BLEED], on, 0, 4);
    if (ovhd.raw[OVR_PACK_L]) XPLMSetDatai(ovhd.raw[OVR_PACK_L], 1);
    if (ovhd.raw[OVR_PACK_R]) XPLMSetDatai(ovhd.raw[OVR_PACK_R], 1);
}

/* ---------------------------------------------------------------- commands (ANN LT switch) */
typedef struct { const char *name; int up; XPLMCommandRef handle; } OvhdCommand;
static OvhdCommand ovhd_commands[] = {
    {"a380x/xp12/ovhd/ann_lt_up", 1, NULL},     /* toward TEST */
    {"a380x/xp12/ovhd/ann_lt_down", 0, NULL},   /* toward DIM */
};
static int ovhd_command_handler(XPLMCommandRef handle, XPLMCommandPhase phase, void *refcon)
{
    const OvhdCommand *command = (const OvhdCommand *)refcon;
    (void)handle;
    if (phase != xplm_CommandBegin || !plugin_enabled || !ovhd.ready) return 1;
    ovhd_sync_ann_lt();
    ovhd.ann_lt = a380x_ovhd_ann_step(ovhd.ann_lt, command->up);
    ovhd_push_ann_lt();
    return 1;
}

/* ---------------------------------------------------------------- lifecycle */
static void ovhd_shutdown(void)
{
    int i;
    ovhd.ready = 0;
    for (i = 0; i < (int)(sizeof(ovhd_commands) / sizeof(ovhd_commands[0])); ++i) if (ovhd_commands[i].handle) {
        XPLMUnregisterCommandHandler(ovhd_commands[i].handle, ovhd_command_handler, 1, &ovhd_commands[i]);
        ovhd_commands[i].handle = NULL;
    }
    for (i = 0; i < ovhd_ref_count; ++i) if (ovhd_refs[i].handle) {
        XPLMUnregisterDataAccessor(ovhd_refs[i].handle); ovhd_refs[i].handle = NULL;
    }
    for (i = 0; i < A380X_OVHD_PB_COUNT; ++i) if (ovhd_pb_commands[i]) {
        XPLMUnregisterCommandHandler(ovhd_pb_commands[i], ovhd_pb_handler, 1, (void *)(intptr_t)i);
        ovhd_pb_commands[i] = NULL;
    }
}
/* Aircraft (re)load: ANN LT BRT, no injected faults; the supply setting is kept. */
static void ovhd_plane_loaded(void)
{
    if (!ovhd.ready) return;
    memset(ovhd_in.fault, 0, sizeof(ovhd_in.fault));
    ovhd_pb_defaults();
    ovhd_normal_xplane_positions();
    ovhd.ann_lt = A380X_OVHD_ANN_BRT;
    ovhd.switch_ratio = 0.5f;
    ovhd_push_ann_lt();
    ovhd_read_inputs();
}
static int ovhd_init(void)
{
    int i;
    memset(&ovhd, 0, sizeof(ovhd));
    memset(&ovhd_in, 0, sizeof(ovhd_in));
    for (i = 0; i < OVR_COUNT; ++i) ovhd.raw[i] = XPLMFindDataRef(ovhd_raw_names[i]);
    ovhd_build_refs();
    for (i = 0; i < ovhd_ref_count; ++i) if (XPLMFindDataRef(ovhd_refs[i].name)) return 0;
    for (i = 0; i < ovhd_ref_count; ++i) {
        OvhdRef *r = &ovhd_refs[i];
        r->handle = r->real
            ? XPLMRegisterDataAccessor(r->name, xplmType_Float | xplmType_Double, r->writable,
                  NULL, NULL, ovhd_get_float, r->writable ? ovhd_set_float : NULL,
                  ovhd_get_double, r->writable ? ovhd_set_double : NULL,
                  NULL, NULL, NULL, NULL, NULL, NULL, r, r->writable ? r : NULL)
            : XPLMRegisterDataAccessor(r->name, xplmType_Int, r->writable,
                  ovhd_get_int, r->writable ? ovhd_set_int : NULL, NULL, NULL, NULL, NULL,
                  NULL, NULL, NULL, NULL, NULL, NULL, r, r->writable ? r : NULL);
        if (!r->handle) { ovhd_shutdown(); return 0; }
    }
    for (i = 0; i < (int)(sizeof(ovhd_commands) / sizeof(ovhd_commands[0])); ++i) {
        OvhdCommand *command = &ovhd_commands[i];
        command->handle = XPLMCreateCommand(command->name, command->up ? "ANN LT switch up (toward TEST)"
                                                                      : "ANN LT switch down (toward DIM)");
        if (!command->handle) { ovhd_shutdown(); return 0; }
        XPLMRegisterCommandHandler(command->handle, ovhd_command_handler, 1, command);
    }
    for (i = 0; i < A380X_OVHD_PB_COUNT; ++i) if (a380x_ovhd_has_pb(i)) {
        char name[96];
        snprintf(name, sizeof(name), "a380x/xp12/ovhd/pb/%s", a380x_ovhd_pbs[i].id);
        ovhd_pb_commands[i] = XPLMCreateCommand(name, name);
        if (!ovhd_pb_commands[i]) { ovhd_shutdown(); return 0; }
        XPLMRegisterCommandHandler(ovhd_pb_commands[i], ovhd_pb_handler, 1, (void *)(intptr_t)i);
    }
    ovhd_pb_defaults();
    ovhd.ready = 1;
    /* User decision: OFF/ON legends light as soon as any battery or DC bus is live
     * (simplified supply). lamp_supply = 0 restores FBW's per-legend supply buses. */
    ovhd_set_supply(A380X_OVHD_SUPPLY_SIMPLE);
    ovhd.ann_lt = elec_io.ready ? elec_ann_lt() : A380X_OVHD_ANN_BRT;
    ovhd.switch_ratio = 0.5f * (float)ovhd.ann_lt;
    ovhd_push_ann_lt();
    ovhd_read_inputs();
    return 1;
}
