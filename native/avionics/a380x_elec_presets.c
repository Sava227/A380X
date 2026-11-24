/* SPDX-License-Identifier: GPL-3.0-or-later
 * flyPad aircraft-state preset tables, ported from the FlyByWire A380X
 * config/a380x/a380-842/aircraft_preset_procedures.xml (GPL-3.0). Names, order, types and
 * delays are FBW's. Steps whose system the port does not model (ENG fire test, RCDR GND CTL,
 * EMER EXIT LT, RMP volumes, cabin ready, standby/transfer/trim fuel pumps, FWC init) are left out; docs/ELEC_SYSTEM.md lists them.
 */
#include "a380x_elec_presets.h"
#include <math.h>
#include <string.h>

const char *const a380x_preset_names[6] = {
    "", "COLD & DARK", "TURNAROUND", "READY FOR PUSHBACK", "READY FOR TAXI", "READY FOR TAKEOFF"
};

#define S(name, type, item, index, value, delay) \
    {name, A380X_PST_##type, A380X_PI_##item, index, value, delay}

static const A380XPresetStep powerup_on[] = {
    S("BAT1 On", STEP, BAT, 0, 1, 1000),
    S("BAT ESS On", STEP, BAT, 2, 1, 1000),
    S("BAT2 On", STEP, BAT, 1, 1, 1000),
    S("BAT APU On", STEP, BAT, 3, 1, 3000),
    S("GPU Connect", STEP, GPU, 0, 1, 1000),        /* FBW connects the GPU from the ground page */
    S("EXT PWR 2 On", STEP, EXT_PB, 1, 1, 1000),
    S("EXT PWR 3 On", STEP, EXT_PB, 2, 1, 1000),
    S("EXT PWR 1 On", STEP, EXT_PB, 0, 1, 1000),
    S("EXT PWR 4 On", STEP, EXT_PB, 3, 1, 3000),
    S("APU Master On", STEP, APU_MASTER, 0, 1, 3000),
    S("APU Start On", STEP, APU_START, 0, 1, 1000),
    S("Await AC BUS ON", COND, AC1_POWERED, 0, 1, 2000),
    S("Crew Oxy On", STEP, CREW_OXY, 0, 1, 1000),
    S("ADIRS 1 Nav", STEP, ADIRS_MODE, 0, 1, 500),
    S("ADIRS 2 Nav", STEP, ADIRS_MODE, 1, 1, 500),
    S("ADIRS 3 Nav", STEP, ADIRS_MODE, 2, 1, 1500),
    S("Strobe Auto", STEP, STROBE, 0, 1, 1000),
    S("Nav & Logo Lt On", STEP, NAV_LOGO, 0, 1, 1000),
    S("SEAT BELTS On", STEP, SEATBELTS, 0, 1, 1000),
    S("NO SMOKING Auto", STEP, NO_SMOKING, 0, 1, 1000),
    S("Fuel Pump Feed TK1 Main On", STEP, FUEL_PUMP, 0, 1, 100),
    S("Fuel Pump Feed TK2 Main On", STEP, FUEL_PUMP, 1, 1, 100),
    S("Fuel Pump Feed TK3 Main On", STEP, FUEL_PUMP, 2, 1, 100),
    S("Fuel Pump Feed TK4 Main On", STEP, FUEL_PUMP, 3, 1, 1000),
    S("Await APU Avail", COND, APU_AVAIL, 0, 1, 2000),
    S("APU Bleed On", STEP, APU_BLEED, 0, 1, 1000),
};
static const A380XPresetStep powerup_off[] = {
    S("APU Bleed Off", STEP, APU_BLEED, 0, 0, 1500),
    S("Fuel Pump Feed TK4 Main Off", STEP, FUEL_PUMP, 3, 0, 100),
    S("Fuel Pump Feed TK3 Main Off", STEP, FUEL_PUMP, 2, 0, 100),
    S("Fuel Pump Feed TK2 Main Off", STEP, FUEL_PUMP, 1, 0, 100),
    S("Fuel Pump Feed TK1 Main Off", STEP, FUEL_PUMP, 0, 0, 500),
    S("NO SMOKING Off", STEP, NO_SMOKING, 0, 0, 1000),
    S("SEAT BELTS Off", STEP, SEATBELTS, 0, 0, 2000),
    S("Nav & Logo Lt Off", STEP, NAV_LOGO, 0, 0, 500),
    S("Strobe Off", STEP, STROBE, 0, 0, 1000),
    S("ADIRS 3 Off", STEP, ADIRS_MODE, 2, 0, 500),
    S("ADIRS 2 Off", STEP, ADIRS_MODE, 1, 0, 500),
    S("ADIRS 1 Off", STEP, ADIRS_MODE, 0, 0, 1000),
    S("Crew Oxy Off", STEP, CREW_OXY, 0, 0, 1000),
    S("APU Master Off", STEP, APU_MASTER, 0, 0, 2000),
    S("EXT PWR 4 Off", STEP, EXT_PB, 3, 0, 3000),
    S("EXT PWR 1 Off", STEP, EXT_PB, 0, 0, 1000),
    S("EXT PWR 3 Off", STEP, EXT_PB, 2, 0, 1000),
    S("EXT PWR 2 Off", STEP, EXT_PB, 1, 0, 3000),
    S("BAT APU Off", STEP, BAT, 3, 0, 1000),
    S("BAT2 Off", STEP, BAT, 1, 0, 1000),
    S("BAT ESS Off", STEP, BAT, 2, 0, 1000),
    S("BAT1 Off", STEP, BAT, 0, 0, 1000),
    S("AC BUS Off Check", COND, AC1_POWERED, 0, 0, 2000),
    S("GPU Disconnect", STEP, GPU, 0, 0, 0),
};
static const A380XPresetStep pushback_on[] = {
    S("EXT PWR 4 Off", STEP, EXT_PB, 3, 0, 1000),
    S("EXT PWR 1 Off", STEP, EXT_PB, 0, 0, 1000),
    S("EXT PWR 3 Off", STEP, EXT_PB, 2, 0, 1000),
    S("EXT PWR 2 Off", STEP, EXT_PB, 1, 0, 3000),
    S("GPU Disconnect", STEP, GPU, 0, 0, 1000),
    S("Beacon On", STEP, BEACON, 0, 1, 1000),
    S("Cockpit Door Locked", STEP, COCKPIT_DOOR, 0, 1, 2000),
    S("ADIRS Align Now", EXON, ADIRS_ALIGN_NOW, 0, 1, 0),  /* port: expedited only */
    S("Await ADIRS 1 Alignment", COND, ADIRS_ALIGNED, 0, 1, 2000),
    S("Await ADIRS 2 Alignment", COND, ADIRS_ALIGNED, 1, 1, 2000),
    S("Await ADIRS 3 Alignment", COND, ADIRS_ALIGNED, 2, 1, 2000),
};
static const A380XPresetStep pushback_off[] = {
    S("Cockpit Door Open", STEP, COCKPIT_DOOR, 0, 0, 2000),
    S("Beacon Off", STEP, BEACON, 0, 0, 1000),
};
static const A380XPresetStep taxi_on[] = {
    S("ENG MODE SEL Start", STEP, ENG_MODE_SEL, 0, 2, 3000),
    S("ENG 1 On", STEP, ENG_MASTER, 0, 1, 1000),
    S("ENG 2 On", STEP, ENG_MASTER, 1, 1, 20000),
    S("Await ENG 1 Avail", NCON, ENG_RUNNING, 0, 1, 2000),
    S("Await ENG 2 Avail", NCON, ENG_RUNNING, 1, 1, 2000),
    S("ENG 3 On", STEP, ENG_MASTER, 2, 1, 1000),
    S("ENG 4 On", STEP, ENG_MASTER, 3, 1, 20000),
    S("Await ENG 3 Avail", COND, ENG_RUNNING, 2, 1, 2000),
    S("Await ENG 4 Avail", COND, ENG_RUNNING, 3, 1, 2000),
    S("Await ENG 1 Avail", ECON, ENG_RUNNING, 0, 1, 0),
    S("Await ENG 2 Avail", ECON, ENG_RUNNING, 1, 1, 0),
    S("ENG MODE SEL Norm", STEP, ENG_MODE_SEL, 0, 1, 3000),
    S("APU Bleed Off", STEP, APU_BLEED, 0, 0, 1500),
    S("APU Master Off", STEP, APU_MASTER, 0, 0, 2000),
    S("Spoiler Arm", STEP, SPOILER_ARM, 0, 1, 2000),
    S("Rudder Trim Reset", STEP, RUDDER_TRIM_ZERO, 0, 1, 2000),
    S("Flaps 1", STEP, FLAPS, 0, 1, 3000),
    S("NOSE Lt Taxi", STEP, NOSE_LIGHT, 0, 1, 1000),
    S("RWY TURN OFF Lt On", STEP, RWY_TURNOFF, 0, 1, 1000),
    S("Autobrake RTO", STEP, AUTOBRAKE_RTO, 0, 1, 200),
    S("TERR ON ND Capt. On", STEP, TERR_ND, 0, 1, 2000),
    S("Transponder Auto", STEP, TRANSPONDER, 0, 1, 1000),
    S("Await Flaps 1+F", COND, FLAPS_AT, 0, 1, 1000),
    S("T.O. Config Push", NOEX, TO_CONFIG, 0, 1, 500),    /* FBW 200 ms; the FWS confirms after 0.3 s */
    S("T.O. Config Release", STEP, TO_CONFIG, 0, 0, 2000),
};
static const A380XPresetStep taxi_off[] = {
    S("TERR ON ND Capt. Off", STEP, TERR_ND, 0, 0, 2000),
    S("Autobrake RTO Off", STEP, AUTOBRAKE_RTO, 0, 0, 2000),
    S("Transponder Off", STEP, TRANSPONDER, 0, 0, 1000),
    S("RWY TURN OFF Lt Off", STEP, RWY_TURNOFF, 0, 0, 2000),
    S("NOSE Lt Off", STEP, NOSE_LIGHT, 0, 0, 1000),
    S("Flaps 0", STEP, FLAPS, 0, 0, 2000),
    S("Rudder Trim Reset", STEP, RUDDER_TRIM_ZERO, 0, 1, 2000),
    S("Spoiler Disarm", STEP, SPOILER_ARM, 0, 0, 2000),
    S("Await Flaps 0", COND, FLAPS_AT, 0, 0, 1000),
    S("ENG 4 Off", STEP, ENG_MASTER, 3, 0, 2000),
    S("ENG 3 Off", STEP, ENG_MASTER, 2, 0, 2000),
    S("ENG 2 Off", STEP, ENG_MASTER, 1, 0, 2000),
    S("ENG 1 Off", STEP, ENG_MASTER, 0, 0, 2000),
    S("ENG 4 N1 <3%", COND, ENG_STOPPED, 3, 1, 500),
    S("ENG 3 N1 <3%", COND, ENG_STOPPED, 2, 1, 500),
    S("ENG 2 N1 <3%", COND, ENG_STOPPED, 1, 1, 500),
    S("ENG 1 N1 <3%", COND, ENG_STOPPED, 0, 1, 500),
};
static const A380XPresetStep takeoff_on[] = {
    S("Strobe On", STEP, STROBE, 0, 2, 1000),
    S("NOSE Lt Takeoff", STEP, NOSE_TO_LIGHT, 0, 1, 1000),
    S("Landing Lights On", STEP, LANDING_LIGHTS, 0, 1, 0),
};
static const A380XPresetStep takeoff_off[] = {
    S("Landing Lights Off", STEP, LANDING_LIGHTS, 0, 0, 0),
    S("NOSE Lt Takeoff", STEP, NOSE_TO_LIGHT, 0, 0, 2000),
    S("Strobe Auto", STEP, STROBE, 0, 1, 1000),
};
#undef S

#define PROC(table) {table, (int)(sizeof(table) / sizeof(table[0]))}
static const struct { const A380XPresetStep *steps; int count; } procedures[A380X_PROC_COUNT] = {
    PROC(powerup_on), PROC(powerup_off), PROC(pushback_on), PROC(pushback_off),
    PROC(taxi_on), PROC(taxi_off), PROC(takeoff_on), PROC(takeoff_off),
};
#undef PROC

const A380XPresetStep *a380x_preset_procedure(int proc, int *count)
{
    if (proc < 0 || proc >= A380X_PROC_COUNT) { if (count) *count = 0; return NULL; }
    if (count) *count = procedures[proc].count;
    return procedures[proc].steps;
}

/* FBW AircraftPresets: 1 = TAKEOFF_OFF, TAXI_OFF, PUSHBACK_OFF, POWERUP_OFF; 2 = TAKEOFF_OFF,
 * TAXI_OFF, PUSHBACK_OFF, POWERUP_ON; 3 = TAKEOFF_OFF, TAXI_OFF, POWERUP_ON, PUSHBACK_ON;
 * 4 = TAKEOFF_OFF, POWERUP_ON, PUSHBACK_ON, TAXI_ON; 5 = POWERUP_ON, PUSHBACK_ON, TAXI_ON,
 * TAKEOFF_ON. Conditions make every already-satisfied step a no-op. */
int a380x_preset_procedures(int preset, int out[4])
{
    static const int lists[5][4] = {
        {A380X_PROC_TAKEOFF_OFF, A380X_PROC_TAXI_OFF, A380X_PROC_PUSHBACK_OFF, A380X_PROC_POWERUP_OFF},
        {A380X_PROC_TAKEOFF_OFF, A380X_PROC_TAXI_OFF, A380X_PROC_PUSHBACK_OFF, A380X_PROC_POWERUP_ON},
        {A380X_PROC_TAKEOFF_OFF, A380X_PROC_TAXI_OFF, A380X_PROC_POWERUP_ON, A380X_PROC_PUSHBACK_ON},
        {A380X_PROC_TAKEOFF_OFF, A380X_PROC_POWERUP_ON, A380X_PROC_PUSHBACK_ON, A380X_PROC_TAXI_ON},
        {A380X_PROC_POWERUP_ON, A380X_PROC_PUSHBACK_ON, A380X_PROC_TAXI_ON, A380X_PROC_TAKEOFF_ON},
    };
    int i;
    if (preset < A380X_PRESET_COLD_DARK || preset > A380X_PRESET_TAKEOFF) return 0;
    for (i = 0; i < 4; ++i) out[i] = lists[preset - 1][i];
    return 4;
}

void a380x_preset_start(A380XPresetRunner *r, int preset, int expedited)
{
    int i;
    memset(r, 0, sizeof(*r));
    r->proc_count = a380x_preset_procedures(preset, r->proc_list);
    if (!r->proc_count) return;
    r->preset = preset;
    r->expedited = expedited != 0;
    r->running = 1;
    for (i = 0; i < r->proc_count; ++i) r->steps_total += procedures[r->proc_list[i]].count;
}

void a380x_preset_cancel(A380XPresetRunner *r)
{
    r->running = 0;
    r->current = NULL;
}

static int step_runs(const A380XPresetStep *s, int expedited)
{
    switch (s->type) {
    case A380X_PST_PROC: case A380X_PST_NCON: return !expedited;
    case A380X_PST_EXON: case A380X_PST_ECON: return expedited;
    default: return 1;
    }
}
static int step_is_condition(const A380XPresetStep *s)
{
    return s->type == A380X_PST_COND || s->type == A380X_PST_NCON || s->type == A380X_PST_ECON;
}

int a380x_preset_update(A380XPresetRunner *r, const A380XPresetHost *host, double dt)
{
    int guard = 0;
    if (!r->running) return 0;
    if (!isfinite(dt) || dt < 0.0) dt = 0.0;
    r->wait_s -= dt;
    /* Several satisfied steps may complete within one frame; a delay or a wait stops the loop. */
    while (r->running && r->wait_s <= 0.0 && guard++ < 256) {
        const A380XPresetStep *s;
        int count;
        if (r->proc_pos >= r->proc_count) { r->running = 0; r->done = 1; r->current = NULL; break; }
        count = procedures[r->proc_list[r->proc_pos]].count;
        if (r->step >= count) { ++r->proc_pos; r->step = 0; continue; }
        s = &procedures[r->proc_list[r->proc_pos]].steps[r->step];
        r->current = s->name;
        if (!step_runs(s, r->expedited)) { ++r->step; ++r->steps_done; continue; }
        if (step_is_condition(s)) {
            if (host->get(host->ctx, s->item, s->index) != s->value) { r->wait_s = 0.0; break; }  /* wait */
        } else {
            if (host->get(host->ctx, s->item, s->index) == s->value) { ++r->step; ++r->steps_done; continue; }
            host->set(host->ctx, s->item, s->index, s->value);
        }
        ++r->step; ++r->steps_done;
        if (!r->expedited || s->type == A380X_PST_NOEX) r->wait_s = s->delay_ms / 1000.0;
        else r->wait_s = 0.0;
    }
    if (r->wait_s < 0.0) r->wait_s = 0.0;
    return r->running;
}
