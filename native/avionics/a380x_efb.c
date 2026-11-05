/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Native cockpit-panel host for the FlyByWire-derived A380 flyPad adapter.
 * It owns the physical tablet pixels and touch commands. Simulator-specific
 * services are translated to guarded X-Plane datarefs and commands.
 */
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#if APL
#include <OpenGL/gl.h>
#elif IBM
#include <windows.h>
#include <GL/gl.h>
#else
#include <GL/gl.h>
#endif

/* The window host needs the XPLM301 window decoration; the cockpit devices need XPLM410
 * (supplied by the plugin build flags, absent in the legacy render test). */
#ifndef XPLM301
#define XPLM301 1
#endif
#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMUtilities.h"

#include "a380x_avionics_core.h"
#include "a380x_efb.h"
#include "a380x_efb_core.h"

#define ARRAY_LEN(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define EFB_LEFT 152.0f
#define EFB_BOTTOM 516.0f
#define EFB_RIGHT 872.0f
#define EFB_TOP 1016.0f
#define MAX_FUEL_KG 323546.0f
#define MAX_PAYLOAD_KG 101736.0f

typedef struct { float r, g, b; } EfbColor;
/* Flat, unbranded A380 control panel palette. */
static const EfbColor EFB_BODY = {0.000f, 0.169f, 0.208f};      /* #002B35 */
static const EfbColor EFB_SECONDARY = {0.018f, 0.207f, 0.247f}; /* #05353F */
static const EfbColor EFB_ACCENT = {0.157f, 0.682f, 0.925f};    /* #28AEEE */
static const EfbColor EFB_STATUS = {0.031f, 0.231f, 0.271f};    /* #083B45 */
static const EfbColor EFB_HIGHLIGHT = {0.161f, 0.682f, 0.925f}; /* #29AEEF */
static const EfbColor EFB_WHITE = {0.929f, 0.961f, 0.973f};
static const EfbColor EFB_UNSELECTED = {0.710f, 0.804f, 0.824f};
static const EfbColor EFB_GREEN = {0.486f, 0.820f, 0.651f};
static const EfbColor EFB_AMBER = {0.973f, 0.722f, 0.290f};
static const EfbColor EFB_RED = {0.929f, 0.396f, 0.388f};

typedef enum {
    PAGE_DASHBOARD = 0, PAGE_DISPATCH, PAGE_GROUND, PAGE_PERFORMANCE,
    PAGE_NAVIGATION, PAGE_ATC, PAGE_FAILURES, PAGE_CHECKLISTS,
    PAGE_PRESETS, PAGE_SETTINGS, PAGE_COUNT
} EfbPage;

static const char *page_names[PAGE_COUNT] = {
    "DASHBOARD", "DISPATCH", "GROUND", "PERFORMANCE", "NAVIGATION",
    "ATC", "FAILURES", "CHECKLISTS", "PRESETS", "SETTINGS"
};

static const char *page_titles[PAGE_COUNT] = {
    "Dashboard", "Dispatch", "Ground", "Performance", "Navigation & Charts",
    "Air Traffic Control", "Failures", "Checklists", "Presets", "Settings"
};

static const char *page_command_names[PAGE_COUNT] = {
    "a380x/xp12/efb/page/dashboard", "a380x/xp12/efb/page/dispatch",
    "a380x/xp12/efb/page/ground", "a380x/xp12/efb/page/performance",
    "a380x/xp12/efb/page/navigation", "a380x/xp12/efb/page/atc",
    "a380x/xp12/efb/page/failures", "a380x/xp12/efb/page/checklists",
    "a380x/xp12/efb/page/presets", "a380x/xp12/efb/page/settings"
};

typedef struct {
    XPLMDataRef zulu, on_ground, latitude, longitude, altitude, vertical_speed;
    XPLMDataRef ground_speed, gps_distance, gps_time, payload, gross_weight;
    XPLMDataRef fuel_total, max_fuel, batteries, gpu, gpu_available, apu;
    XPLMDataRef generators, engines, fuel, tank_ratio, doors;
    XPLMDataRef oat, qnh, wind_direction, wind_speed;
    XPLMDataRef com1, com1_standby, transponder, transponder_mode;
    XPLMDataRef avionics, seatbelts, beacon, gear, speedbrake, parking_brake;
    XPLMDataRef failures[17];
} EfbRefs;

static EfbRefs refs;
static const char *failure_names[17] = {
    "ENGINE 1", "ENGINE 2", "ENGINE 3", "ENGINE 4",
    "GENERATOR 1", "GENERATOR 2", "GENERATOR 3", "GENERATOR 4",
    "BATTERY 1", "BATTERY 2", "FLAP ACTUATOR", "GEAR ACTUATOR",
    "HYDRAULIC 1", "HYDRAULIC 2", "APU", "PITOT 1", "PITOT 2"
};
static const char *failure_refs[17] = {
    "sim/operation/failures/rel_engfai0", "sim/operation/failures/rel_engfai1",
    "sim/operation/failures/rel_engfai2", "sim/operation/failures/rel_engfai3",
    "sim/operation/failures/rel_genera0", "sim/operation/failures/rel_genera1",
    "sim/operation/failures/rel_genera2", "sim/operation/failures/rel_genera3",
    "sim/operation/failures/rel_batter0", "sim/operation/failures/rel_batter1",
    "sim/operation/failures/rel_flap_act", "sim/operation/failures/rel_gear_act",
    "sim/operation/failures/rel_hydpmp", "sim/operation/failures/rel_hydpmp2",
    "sim/operation/failures/rel_apu", "sim/operation/failures/rel_pitot",
    "sim/operation/failures/rel_pitot2"
};

static const char *checklist_names[10] = {
    "COCKPIT PREPARATION", "BEFORE START", "AFTER START", "TAXI", "LINE-UP",
    "DEPARTURE CHANGE", "APPROACH", "LANDING", "PARKING", "SECURING AIRCRAFT"
};
static const unsigned checklist_item_counts[10] = {4, 3, 3, 8, 3, 4, 4, 4, 4, 4};
typedef enum {
    CHECK_MANUAL, CHECK_SEATBELTS, CHECK_PARKING_BRAKE, CHECK_BEACON,
    CHECK_GEAR_DOWN, CHECK_SPOILERS_ARMED, CHECK_ENGINES_OFF, CHECK_BATTERIES_OFF
} CheckCondition;
typedef struct { const char *name, *response; CheckCondition condition; } CheckItem;
static const CheckItem checklist_items[10][8] = {
    {{"GEAR PINS & COVERS", "REMOVED", CHECK_MANUAL}, {"FUEL QTY", "CHECKED", CHECK_MANUAL},
     {"SEAT BELTS", "ON", CHECK_SEATBELTS}, {"BARO REF", "SET", CHECK_MANUAL}},
    {{"PARKING BRAKE", "SET", CHECK_PARKING_BRAKE}, {"T.O SPEEDS & THRUST", "CHECKED", CHECK_MANUAL},
     {"BEACON", "ON", CHECK_BEACON}},
    {{"ANTI ICE", "AS RQRD", CHECK_MANUAL}, {"PITCH TRIM", "T.O", CHECK_MANUAL},
     {"RUDDER TRIM", "NEUTRAL", CHECK_MANUAL}},
    {{"FLIGHT CONTROLS", "CHECKED", CHECK_MANUAL}, {"FLAPS SETTING", "CONF", CHECK_MANUAL},
     {"RADAR", "ON", CHECK_MANUAL}, {"T.O SEAT BELTS", "ON", CHECK_SEATBELTS},
     {"T.O GND SPLRS", "ARMED", CHECK_SPOILERS_ARMED}, {"T.O FLAPS", "SET", CHECK_MANUAL},
     {"T.O AUTO BRAKE", "RTO", CHECK_MANUAL}, {"T.O CONFIG", "TESTED", CHECK_MANUAL}},
    {{"T.O RWY", "CONFIRMED", CHECK_MANUAL}, {"CABIN CREW", "ADVISED", CHECK_MANUAL},
     {"PACKS 1+2", "ON", CHECK_MANUAL}},
    {{"RWY & SID", "CONFIRMED", CHECK_MANUAL}, {"FLAPS SETTING", "CONFIRMED", CHECK_MANUAL},
     {"T.O SPEEDS & THRUST", "CHECKED", CHECK_MANUAL}, {"FCU ALT", "SET", CHECK_MANUAL}},
    {{"BARO REF", "SET", CHECK_MANUAL}, {"SEAT BELTS", "ON", CHECK_SEATBELTS},
     {"MINIMUM", "SET", CHECK_MANUAL}, {"AUTO BRAKE", "SET", CHECK_MANUAL}},
    {{"LDG SEAT BELTS", "ON", CHECK_SEATBELTS}, {"LDG GEAR", "DOWN", CHECK_GEAR_DOWN},
     {"GND SPLRS", "ARM", CHECK_SPOILERS_ARMED}, {"FLAPS", "LDG", CHECK_MANUAL}},
    {{"PARK BRAKE OR CHOCKS", "AS RQRD", CHECK_MANUAL}, {"ENGINES", "OFF", CHECK_ENGINES_OFF},
     {"WING LIGHTS", "OFF", CHECK_MANUAL}, {"FUEL PUMPS", "OFF", CHECK_MANUAL}},
    {{"OXYGEN", "OFF", CHECK_MANUAL}, {"EMER EXIT LT", "OFF", CHECK_MANUAL},
     {"EFBs", "OFF", CHECK_MANUAL}, {"BATTERIES", "OFF", CHECK_BATTERIES_OFF}}
};

typedef enum { COMMAND_PAGE, COMMAND_ACTION, COMMAND_POWER, COMMAND_NEXT, COMMAND_SLIDES, COMMAND_POPUP } CommandKind;
typedef struct {
    const char *name;
    const char *description;
    CommandKind kind;
    int value;
    XPLMCommandRef ref;
} EfbCommand;

static EfbCommand commands[PAGE_COUNT + 13];
static int command_count;
static EfbPage active_page = PAGE_DASHBOARD;
static int powered = 1;
static int metric_units = 1;
static int automatic_checklists = 1;
static int battery_simulation = 1;
static int brightness = 85;
static double battery_level = 100.0;
static int target_altitude_ft = 3000;
static double target_angle_deg = 3.0;
static int selected_failure;
static int selected_checklist;
static unsigned checklist_bits[10];
static int native_available = 1;
static XPLMDataRef native_available_ref;
static char status_message[160] = "A380 control panel ready";
/* Escape slides of the 16 passenger doors (door_open_ratio order). Native state owned here; the web
 * EFB (a380x_efb_web.inc), the door command veto, the SD DOOR page and presets all use it. */
static unsigned char slide_armed[A380X_EFB_SLIDES];
static void efb_changed(void);
static void efb_preset_slides(int preset);
static void efb_tablets_follow_page(int page);
static void efb_slides_set_all(int arm);
static void efb_toggle_popup(int tablet);

static XPLMDataRef optional_ref(const char *name)
{
    XPLMDataRef ref = XPLMFindDataRef(name);
    if (ref == NULL) {
        XPLMDebugString("A380X flyPad: optional dataref unavailable ");
        XPLMDebugString(name);
        XPLMDebugString("\n");
    }
    return ref;
}

static float get_float(XPLMDataRef ref)
{
    return ref != NULL ? XPLMGetDataf(ref) : 0.0f;
}

static double get_double(XPLMDataRef ref)
{
    return ref != NULL ? XPLMGetDatad(ref) : 0.0;
}

static int get_int(XPLMDataRef ref)
{
    return ref != NULL ? XPLMGetDatai(ref) : 0;
}

static void set_status(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(status_message, sizeof(status_message), format, args);
    va_end(args);
}

static int read_native_available(void *refcon)
{
    (void)refcon;
    return native_available;
}

static int on_ground(void)
{
    return get_int(refs.on_ground) != 0;
}

static int engines_off(void)
{
    int values[4] = {0};
    int i;
    if (refs.engines == NULL) return 0;
    XPLMGetDatavi(refs.engines, values, 0, 4);
    for (i = 0; i < 4; ++i) if (values[i] != 0) return 0;
    return 1;
}

/* Native A380 network (a380x_elec_adapter.inc, other translation unit): looked up lazily
 * because it registers after the flyPad. */
static XPLMDataRef native_lookup(XPLMDataRef *cache, const char *name)
{
    if (*cache == NULL) *cache = XPLMFindDataRef(name);
    return *cache;
}
static int elec_network_active(void)
{
    static XPLMDataRef ref;
    XPLMDataRef handle = native_lookup(&ref, "a380x/xp12/elec/network_active");
    return handle != NULL && XPLMGetDatai(handle) != 0;
}
static int elec_native_int(const char *name, int fallback)
{
    static struct { const char *name; XPLMDataRef ref; } cache[8];
    int i;
    for (i = 0; i < 8; ++i) {
        if (cache[i].name == NULL) cache[i].name = name;
        if (cache[i].name == name || strcmp(cache[i].name, name) == 0) {
            XPLMDataRef handle = native_lookup(&cache[i].ref, name);
            return handle != NULL ? XPLMGetDatai(handle) : fallback;
        }
    }
    return fallback;
}
static int gpu_connected(void)
{
    return elec_network_active() ? elec_native_int("a380x/xp12/elec/ext/connected", 0) : get_int(refs.gpu);
}
static void gpu_toggle(void)
{
    if (elec_network_active() && XPLMFindCommand("a380x/xp12/elec/gpu_connect_toggle") != NULL)
        XPLMCommandOnce(XPLMFindCommand("a380x/xp12/elec/gpu_connect_toggle"));
    else {
        XPLMCommandRef command = XPLMFindCommand("sim/ground_ops/toggle_gpu_request");
        if (command != NULL) XPLMCommandOnce(command);
    }
}

/* Tablet charging: aircraft power = any AC main bus of the A380 network (stock switches
 * without it). */
static int electrical_power_available(void)
{
    int values[4] = {0};
    int i;
    if (elec_network_active()) return elec_native_int("a380x/xp12/elec/ac_main_powered", 0) != 0;
    if (refs.batteries != NULL) {
        XPLMGetDatavi(refs.batteries, values, 0, 2);
        if (values[0] || values[1]) return 1;
    }
    if (get_int(refs.gpu) || get_int(refs.apu)) return 1;
    memset(values, 0, sizeof(values));
    if (refs.generators != NULL) {
        XPLMGetDatavi(refs.generators, values, 0, 4);
        for (i = 0; i < 4; ++i) if (values[i]) return 1;
    }
    return 0;
}

static void run_command(const char *name)
{
    XPLMCommandRef command = XPLMFindCommand(name);
    if (command != NULL) XPLMCommandOnce(command);
    else set_status("Native command unavailable: %s", name);
}

static int loading_allowed(void)
{
    if (!on_ground() || !engines_off()) {
        set_status("Load changes require on ground with all engines off");
        return 0;
    }
    return 1;
}

static void set_payload(float kilograms)
{
    if (!loading_allowed() || refs.payload == NULL) return;
    if (kilograms < 0.0f) kilograms = 0.0f;
    if (kilograms > MAX_PAYLOAD_KG) kilograms = MAX_PAYLOAD_KG;
    XPLMSetDataf(refs.payload, kilograms);
    set_status("Payload set to %.0f kg", kilograms);
}

static void set_fuel(float target)
{
    float ratios[9] = {0.30f, 0.078f, 0.22f, 0.22f, 0.078f, 0.023f, 0.023f, 0.058f, 0.0f};
    float values[9] = {0};
    float total = 0.0f, written = 0.0f;
    int i;
    if (!loading_allowed() || refs.fuel == NULL) return;
    if (target < 0.0f) target = 0.0f;
    if (target > MAX_FUEL_KG) target = MAX_FUEL_KG;
    if (refs.tank_ratio != NULL) XPLMGetDatavf(refs.tank_ratio, ratios, 0, 9);
    for (i = 0; i < 9; ++i) {
        if (ratios[i] < 0.0f) ratios[i] = 0.0f;
        total += ratios[i];
    }
    if (total <= 0.0f) {
        const float fallback[9] = {0.30f, 0.078f, 0.22f, 0.22f, 0.078f,
                                   0.023f, 0.023f, 0.058f, 0.0f};
        memcpy(ratios, fallback, sizeof(ratios));
        total = 1.0f;
    }
    for (i = 0; i < 9; ++i) {
        values[i] = ratios[i] > 0.0f ? target * ratios[i] / total : 0.0f;
        written += values[i];
    }
    if (written > 0.0f && fabsf(written - target) > 0.1f) values[0] += target - written;
    XPLMSetDatavf(refs.fuel, values, 0, 9);
    set_status("Fuel set to %.0f kg across nine native tanks", target);
}

static void set_page(int page)
{
    if (page >= 0 && page < PAGE_COUNT) active_page = (EfbPage)page;
}

/* FBW flyPad aircraft-state presets 1-5 run natively in the A380 network
 * (a380x_elec_presets.c, a380x/xp12/elec/preset/<n>). The stock fallbacks below remain
 * only for a plugin without the network. */
static int native_preset(int preset)
{
    char name[48];
    XPLMCommandRef command;
    if (!elec_network_active()) return 0;
    snprintf(name, sizeof(name), "a380x/xp12/elec/preset/%d", preset);
    command = XPLMFindCommand(name);
    if (command == NULL) return 0;
    XPLMCommandOnce(command);
    if (elec_native_int("a380x/xp12/elec/preset/active", 0) == preset)
        set_status("%s preset running", preset == 1 ? "Cold & Dark" : preset == 2 ? "Turnaround" :
                   preset == 3 ? "Ready for Pushback" : preset == 4 ? "Ready for Taxi" : "Ready for Takeoff");
    else set_status("Preset stopped or not available (ground only; engines off for 1-2)");
    return 1;
}

static void preset_cold_dark(void)
{
    if (native_preset(1)) return;
    if (!on_ground()) { set_status("Aircraft presets are ground-only"); return; }
    run_command("sim/autopilot/servos_off_any");
    run_command("sim/systems/avionics_off");
    run_command("sim/electrical/APU_off");
    run_command("sim/electrical/GPU_off");
    run_command("sim/electrical/battery_1_off");
    run_command("sim/electrical/battery_2_off");
    run_command("sim/fuel/fuel_pumps_off");
    efb_preset_slides(1);
    set_status("Cold & Dark native preset applied");
}

static void preset_turnaround(void)
{
    if (native_preset(2)) return;
    if (!on_ground()) { set_status("Aircraft presets are ground-only"); return; }
    run_command("sim/electrical/battery_1_on");
    run_command("sim/electrical/battery_2_on");
    run_command("sim/ground_ops/toggle_gpu_request");
    run_command("sim/systems/avionics_on");
    efb_preset_slides(2);
    set_status("Turnaround native preset applied");
}

static void preset_ready(void)
{
    if (native_preset(4)) return;
    if (!on_ground()) { set_status("Aircraft presets are ground-only"); return; }
    run_command("sim/operation/quick_start");
    run_command("sim/systems/avionics_on");
    run_command("sim/fuel/fuel_pumps_on");
    efb_preset_slides(4);
    set_status("Ready for Taxi native preset applied");
}

static void toggle_all_doors(void)
{
    float doors[20] = {0};
    int i, open_all = 0, kept = 0;
    if (refs.doors != NULL) {
        XPLMGetDatavf(refs.doors, doors, 0, 20);
        for (i = 0; i < 20; ++i) if (doors[i] < 0.5f && !(i < A380X_EFB_SLIDES && slide_armed[i])) open_all = 1;
    }
    for (i = 0; i < 20; ++i) {
        char name[64];
        /* An armed escape slide keeps its door closed (a380x_efb_web.inc). */
        if (open_all && doors[i] < 0.5f && i < A380X_EFB_SLIDES && slide_armed[i]) { ++kept; continue; }
        if ((open_all && doors[i] < 0.5f) || (!open_all && doors[i] >= 0.5f)) {
            snprintf(name, sizeof(name), "sim/flight_controls/door_toggle_%d", i + 1);
            run_command(name);
        }
    }
    if (kept) set_status("Doors commanded open; %d with armed slides stayed closed", kept);
    else set_status("All 20 doors commanded %s", open_all ? "open" : "closed");
    efb_changed();
}

static void action_dashboard(int action)
{
    static const int pages[6] = {PAGE_DISPATCH, PAGE_GROUND, PAGE_PERFORMANCE,
                                 PAGE_NAVIGATION, PAGE_CHECKLISTS, PAGE_SETTINGS};
    set_page(pages[action]);
}

static void action_dispatch(int action)
{
    switch (action) {
    case 0: set_payload(242.0f * 104.0f + 10000.0f); break;
    case 1: set_payload(484.0f * 104.0f + 20000.0f); break;
    case 2: set_fuel(MAX_FUEL_KG * 0.5f); break;
    case 3: set_fuel(MAX_FUEL_KG); break;
    case 4: run_command("sim/FMS/CDU_popup"); break;
    case 5: set_payload(0.0f); set_fuel(0.0f); break;
    default: break;
    }
}

static void action_ground(int action)
{
    switch (action) {
    case 0: run_command("sim/ground_ops/jetway"); break;
    case 1: run_command("sim/ground_ops/service_plane"); break;
    case 2: gpu_toggle(); break;
    case 3: run_command("sim/flight_controls/door_toggle_1"); break;
    case 4: toggle_all_doors(); break;
    case 5: run_command("sim/ground_ops/pushback_straight"); break;
    default: break;
    }
}

static void action_performance(int action)
{
    switch (action) {
    case 0: target_altitude_ft -= 1000; break;
    case 1: target_altitude_ft += 1000; break;
    case 2: target_angle_deg -= 0.5; break;
    case 3: target_angle_deg += 0.5; break;
    case 4: target_altitude_ft = 3000; target_angle_deg = 3.0; break;
    case 5: set_page(PAGE_NAVIGATION); break;
    default: break;
    }
    if (target_altitude_ft < 0) target_altitude_ft = 0;
    if (target_altitude_ft > 50000) target_altitude_ft = 50000;
    if (target_angle_deg < 1.0) target_angle_deg = 1.0;
    if (target_angle_deg > 6.0) target_angle_deg = 6.0;
}

static void action_navigation(int action)
{
    static const char *names[6] = {
        "sim/map/show_current", "sim/map/show_low_enroute", "sim/map/show_high_enroute",
        "sim/instruments/map_zoom_in", "sim/instruments/map_zoom_out", "sim/instruments/EFIS_apt"
    };
    run_command(names[action]);
}

static void action_atc(int action)
{
    switch (action) {
    case 0: run_command("sim/radios/com1_standy_flip"); break;
    case 1: if (refs.transponder_mode != NULL) XPLMSetDatai(refs.transponder_mode, 1); break;
    case 2: if (refs.transponder_mode != NULL) XPLMSetDatai(refs.transponder_mode, 3); break;
    case 3: if (refs.transponder_mode != NULL) XPLMSetDatai(refs.transponder_mode, 7); break;
    case 4: if (refs.com1_standby != NULL) XPLMSetDatai(refs.com1_standby, 121500); break;
    case 5: if (refs.transponder != NULL) XPLMSetDatai(refs.transponder, 2000); break;
    default: break;
    }
    set_status("Native radio/transponder action applied");
}

static void action_failures(int action)
{
    if (action == 0) selected_failure = (selected_failure + 16) % 17;
    else if (action == 1) selected_failure = (selected_failure + 1) % 17;
    else if (action == 2 && refs.failures[selected_failure] != NULL) {
        int failed = XPLMGetDatai(refs.failures[selected_failure]) >= 6;
        XPLMSetDatai(refs.failures[selected_failure], failed ? 0 : 6);
        set_status("%s %s", failure_names[selected_failure], failed ? "repaired" : "failed");
    } else if (action == 3) run_command("sim/operation/fix_all_systems");
    else if (action == 4) selected_failure = 0;
    else if (action == 5) set_page(PAGE_CHECKLISTS);
}

static int checklist_condition_met(CheckCondition condition)
{
    int batteries[2] = {0};
    float gear[5] = {0};
    int i;
    switch (condition) {
    case CHECK_SEATBELTS: return refs.seatbelts && get_int(refs.seatbelts) > 0;
    case CHECK_PARKING_BRAKE: return refs.parking_brake && get_float(refs.parking_brake) > 0.8f;
    case CHECK_BEACON: return refs.beacon && get_int(refs.beacon) > 0;
    case CHECK_GEAR_DOWN:
        if (!refs.gear || XPLMGetDatavf(refs.gear, gear, 0, 5) < 5) return 0;
        for (i = 0; i < 5; ++i) if (gear[i] < 0.99f) return 0;
        return 1;
    case CHECK_SPOILERS_ARMED: {
        /* SPEED BRAKE module (other translation unit, registers after the flyPad): the lever's ARM; X-Plane's
         * handle carries the surface order there (docs/SPEED_BRAKE.md). */
        static XPLMDataRef armed;
        XPLMDataRef handle = native_lookup(&armed, "a380x/xp12/speedbrake/ground_spoilers_armed");
        if (handle != NULL) return XPLMGetDatai(handle) != 0;
        return refs.speedbrake && get_float(refs.speedbrake) < -0.1f;
    }
    case CHECK_ENGINES_OFF: return engines_off();
    case CHECK_BATTERIES_OFF:
        if (elec_network_active())
            return !elec_native_int("a380x/xp12/elec/pb/bat1", 1) && !elec_native_int("a380x/xp12/elec/pb/bat2", 1) &&
                   !elec_native_int("a380x/xp12/elec/pb/bat_ess", 1) && !elec_native_int("a380x/xp12/elec/pb/bat_apu", 1);
        if (!refs.batteries || XPLMGetDatavi(refs.batteries, batteries, 0, 2) < 2) return 0;
        return !batteries[0] && !batteries[1];
    default: return 0;
    }
}

static unsigned checklist_completed_bits(int checklist)
{
    unsigned bits = checklist_bits[checklist];
    unsigned i;
    if (automatic_checklists) {
        for (i = 0; i < checklist_item_counts[checklist]; ++i)
            if (checklist_condition_met(checklist_items[checklist][i].condition)) bits |= 1u << i;
    }
    return bits;
}

static void action_checklists(int action)
{
    unsigned full = (1u << checklist_item_counts[selected_checklist]) - 1u;
    if (action == 0) selected_checklist = (selected_checklist + 9) % 10;
    else if (action == 1) selected_checklist = (selected_checklist + 1) % 10;
    else if (action == 2) {
        unsigned bit;
        for (bit = 1u; bit <= full; bit <<= 1u) {
            if ((checklist_completed_bits(selected_checklist) & bit) == 0u) {
                checklist_bits[selected_checklist] |= bit;
                break;
            }
        }
    } else if (action == 3) checklist_bits[selected_checklist] = 0;
    else if (action == 4) memset(checklist_bits, 0, sizeof(checklist_bits));
    else if (action == 5) checklist_bits[selected_checklist] = full;
}

static void action_presets(int action)
{
    if (elec_network_active()) {
        /* FBW flyPad: 1 Cold & Dark, 2 Turnaround, 3 Pushback, 4 Taxi, 5 Takeoff; 6th: expedite. */
        if (action < 5) native_preset(action + 1);
        else {
            XPLMCommandRef command = XPLMFindCommand("a380x/xp12/elec/preset/expedite_toggle");
            if (command != NULL) XPLMCommandOnce(command);
            set_status("Expedited presets %s", elec_native_int("a380x/xp12/elec/preset/expedited", 0) ? "on" : "off");
        }
        return;
    }
    if (action == 0) preset_cold_dark();
    else if (action == 1) preset_turnaround();
    else if (action == 2) preset_ready();
    else if (action == 3) run_command("sim/operation/fix_all_systems");
    else if (action == 4) set_page(PAGE_GROUND);
    else if (action == 5) set_page(PAGE_CHECKLISTS);
}

static void action_settings(int action)
{
    if (action == 0) metric_units = !metric_units;
    else if (action == 1) automatic_checklists = !automatic_checklists;
    else if (action == 2) battery_simulation = !battery_simulation;
    else if (action == 3) brightness -= 10;
    else if (action == 4) brightness += 10;
    else if (action == 5) powered = !powered;
    if (brightness < 20) brightness = 20;
    if (brightness > 100) brightness = 100;
}

static void execute_action(int action)
{
    if (action < 0 || action >= 6) return;
    switch (active_page) {
    case PAGE_DASHBOARD: action_dashboard(action); break;
    case PAGE_DISPATCH: action_dispatch(action); break;
    case PAGE_GROUND: action_ground(action); break;
    case PAGE_PERFORMANCE: action_performance(action); break;
    case PAGE_NAVIGATION: action_navigation(action); break;
    case PAGE_ATC: action_atc(action); break;
    case PAGE_FAILURES: action_failures(action); break;
    case PAGE_CHECKLISTS: action_checklists(action); break;
    case PAGE_PRESETS: action_presets(action); break;
    case PAGE_SETTINGS: action_settings(action); break;
    default: break;
    }
}

static int command_handler(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    EfbCommand *entry = (EfbCommand *)refcon;
    (void)command;
    if (phase != xplm_CommandBegin || entry == NULL) return 1;
    if (entry->kind == COMMAND_PAGE) { set_page(entry->value); efb_tablets_follow_page(entry->value); }
    else if (entry->kind == COMMAND_ACTION) execute_action(entry->value);
    else if (entry->kind == COMMAND_POWER) powered = !powered;
    else if (entry->kind == COMMAND_NEXT) { set_page(((int)active_page + 1) % PAGE_COUNT); efb_tablets_follow_page(active_page); }
    else if (entry->kind == COMMAND_SLIDES) efb_slides_set_all(entry->value);
    else if (entry->kind == COMMAND_POPUP) efb_toggle_popup(entry->value);
    efb_changed();
    return 1;
}

static int add_command(const char *name, const char *description, CommandKind kind, int value)
{
    EfbCommand *entry;
    if (command_count >= ARRAY_LEN(commands)) return 0;
    entry = &commands[command_count++];
    entry->name = name;
    entry->description = description;
    entry->kind = kind;
    entry->value = value;
    /* X-Plane retains created command names across an aircraft reload. Reuse
     * our command and attach the new plugin instance's handler. */
    entry->ref = XPLMFindCommand(name);
    if (entry->ref == NULL) entry->ref = XPLMCreateCommand(name, description);
    if (entry->ref == NULL) return 0;
    XPLMRegisterCommandHandler(entry->ref, command_handler, 1, entry);
    return 1;
}

/* The previous sidebar renderer is retained solely as a migration reference;
 * only the shared control-panel renderer below is compiled. */
#if 0
static void set_efb_color(EfbColor color)
{
    float scale = brightness / 100.0f;
    glColor3f(color.r * scale, color.g * scale, color.b * scale);
}

static void efb_rectangle(float left, float bottom, float right, float top, EfbColor color)
{
    set_efb_color(color);
    glBegin(GL_QUADS);
    glVertex2f(left, bottom); glVertex2f(right, bottom);
    glVertex2f(right, top); glVertex2f(left, top);
    glEnd();
}

static void efb_outline(float left, float bottom, float right, float top, EfbColor color)
{
    set_efb_color(color);
    glBegin(GL_LINE_LOOP);
    glVertex2f(left, bottom); glVertex2f(right, bottom);
    glVertex2f(right, top); glVertex2f(left, top);
    glEnd();
}

static void efb_line(float x0, float y0, float x1, float y1, EfbColor color, float width)
{
    set_efb_color(color);
    glLineWidth(width);
    glBegin(GL_LINES);
    glVertex2f(x0, y0); glVertex2f(x1, y1);
    glEnd();
    glLineWidth(1.0f);
}

static void efb_circle(float cx, float cy, float radius, EfbColor color, int filled)
{
    int i;
    set_efb_color(color);
    glBegin(filled ? GL_POLYGON : GL_LINE_LOOP);
    for (i = 0; i < 24; ++i) {
        double angle = 6.283185307179586 * i / 24.0;
        glVertex2f(cx + radius * (float)cos(angle), cy + radius * (float)sin(angle));
    }
    glEnd();
}

static void efb_arc(float cx, float cy, float radius, float start, float end,
                    EfbColor color)
{
    int i;
    set_efb_color(color);
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (i = 0; i <= 12; ++i) {
        float angle = start + (end - start) * i / 12.0f;
        glVertex2f(cx + radius * cosf(angle), cy + radius * sinf(angle));
    }
    glEnd();
    glLineWidth(1.0f);
}

static void efb_text(float x, float y, const char *value, EfbColor color)
{
    float scale = brightness / 100.0f;
    float rgb[3] = {color.r * scale, color.g * scale, color.b * scale};
    XPLMDrawString(rgb, (int)lroundf(x), (int)lroundf(y), value, NULL, xplmFont_Proportional);
    /* XPLMDrawString may change OpenGL state; later panel primitives must
     * restore the documented gauge-drawing state before using immediate GL. */
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
}

static void draw_button(float left, float bottom, float right, float top,
                        const char *label, int selected)
{
    efb_rectangle(left, bottom, right, top, selected ? EFB_HIGHLIGHT : EFB_ACCENT);
    efb_outline(left, bottom, right, top, selected ? EFB_HIGHLIGHT : EFB_UNSELECTED);
    efb_text(left + 8.0f, bottom + 17.0f, label, selected ? EFB_BODY : EFB_WHITE);
}

static const char *action_label(EfbPage page, int action)
{
    static const char *labels[PAGE_COUNT][6] = {
        {"Dispatch", "Ground", "Performance", "Navigation", "Checklists", "Settings"},
        {"242 passengers", "484 passengers", "Fuel 50%", "Fuel 100%", "Open FMS", "Clear load"},
        {"Jetway", "Services", "Ground power", "Door 1", "All doors", "Pushback"},
        {"Target -1000", "Target +1000", "Angle -0.5", "Angle +0.5", "Reset", "Navigation"},
        {"Current map", "Low enroute", "High enroute", "Zoom +", "Zoom -", "Airports"},
        {"COM1 swap", "XPDR standby", "XPDR altitude", "XPDR TA/RA", "COM1 guard", "XPDR 2000"},
        {"Previous", "Next", "Fail / repair", "Repair all", "First failure", "Checklists"},
        {"Previous", "Next", "Check item", "Reset page", "Reset all", "Complete"},
        {"Cold & dark", "Turnaround", "Ready for taxi", "Repair all", "Ground", "Checklists"},
        {"Metric units", "Auto check", "Battery sim", "Brightness -", "Brightness +", "Power"},
    };
    return labels[page][action];
}

static void draw_card(float left, float bottom, float right, float top, const char *title)
{
    efb_rectangle(left, bottom, right, top, EFB_SECONDARY);
    efb_outline(left, bottom, right, top, EFB_ACCENT);
    efb_text(left + 12.0f, top - 22.0f, title, EFB_WHITE);
}

static void draw_tab(float left, float right, const char *label, int selected)
{
    efb_rectangle(left, 940, right, 970, selected ? EFB_HIGHLIGHT : EFB_ACCENT);
    efb_text(left + 7.0f, 951, label, selected ? EFB_BODY : EFB_WHITE);
}

static void draw_tabs(const char *a, const char *b, const char *c, const char *d)
{
    const char *tabs[4] = {a, b, c, d};
    int count = 0, i;
    float width, left;
    while (count < 4 && tabs[count] != NULL) ++count;
    if (count == 0) return;
    width = count == 4 ? 78.0f : 92.0f;
    left = 858.0f - width * count;
    for (i = 0; i < count; ++i) draw_tab(left + i * width, left + (i + 1) * width - 4.0f, tabs[i], i == 0);
}

static void draw_toolbar_icon(int page, float cx, float cy, int active)
{
    EfbColor color = active ? EFB_WHITE : EFB_UNSELECTED;
    float s = 11.0f;
    if (active) efb_rectangle(cx - 25, cy - 19, cx + 25, cy + 19, EFB_ACCENT);
    set_efb_color(color);
    glLineWidth(2.0f);
    switch (page) {
    case PAGE_DASHBOARD: /* Exact FBW tail-mark silhouette, simplified to its SVG polygon. */
        set_efb_color(EFB_HIGHLIGHT);
        glBegin(GL_POLYGON);
        glVertex2f(cx + 15, cy + 15); glVertex2f(cx + 4, cy + 15);
        glVertex2f(cx - 7, cy - 6); glVertex2f(cx - 14, cy - 15);
        glVertex2f(cx + 8, cy - 15);
        glEnd();
        break;
    case PAGE_DISPATCH:
        efb_outline(cx - 9, cy - 12, cx + 9, cy + 12, color);
        efb_outline(cx - 4, cy + 8, cx + 4, cy + 14, color);
        efb_line(cx - 5, cy + 3, cx + 5, cy + 3, color, 2);
        efb_line(cx - 5, cy - 3, cx + 5, cy - 3, color, 2);
        break;
    case PAGE_GROUND:
        efb_outline(cx - 13, cy - 7, cx + 4, cy + 7, color);
        efb_outline(cx + 4, cy - 7, cx + 12, cy + 2, color);
        efb_circle(cx - 7, cy - 9, 3, color, 0); efb_circle(cx + 8, cy - 9, 3, color, 0);
        break;
    case PAGE_PERFORMANCE:
        efb_outline(cx - 10, cy - 13, cx + 10, cy + 13, color);
        efb_outline(cx - 6, cy + 5, cx + 6, cy + 9, color);
        efb_line(cx - 6, cy, cx + 6, cy, color, 2);
        efb_line(cx, cy - 8, cx, cy + 3, color, 2);
        break;
    case PAGE_NAVIGATION:
        efb_circle(cx, cy, 13, color, 0);
        efb_line(cx - 6, cy - 8, cx + 5, cy + 9, color, 2);
        efb_line(cx + 5, cy + 9, cx + 2, cy - 3, color, 2);
        break;
    case PAGE_ATC:
        efb_circle(cx, cy - 7, 2.5f, color, 1);
        efb_line(cx, cy - 5, cx, cy + 12, color, 2);
        efb_arc(cx, cy, 8, 0.7f, 2.44f, color);
        efb_arc(cx, cy, 14, 0.7f, 2.44f, color);
        break;
    case PAGE_FAILURES:
        glBegin(GL_LINE_LOOP);
        glVertex2f(cx, cy + 14); glVertex2f(cx + 14, cy);
        glVertex2f(cx, cy - 14); glVertex2f(cx - 14, cy);
        glEnd();
        efb_text(cx - 2, cy - 5, "!", color);
        break;
    case PAGE_CHECKLISTS:
        efb_outline(cx - 10, cy - 13, cx + 10, cy + 13, color);
        efb_line(cx - 6, cy + 5, cx - 2, cy + 1, color, 2);
        efb_line(cx - 2, cy + 1, cx + 6, cy + 8, color, 2);
        efb_line(cx - 5, cy - 5, cx + 6, cy - 5, color, 2);
        break;
    case PAGE_PRESETS:
        efb_line(cx - s, cy + 8, cx + s, cy + 8, color, 2);
        efb_line(cx - s, cy, cx + s, cy, color, 2);
        efb_line(cx - s, cy - 8, cx + s, cy - 8, color, 2);
        efb_circle(cx - 4, cy + 8, 3, color, 0);
        efb_circle(cx + 5, cy, 3, color, 0);
        efb_circle(cx - 1, cy - 8, 3, color, 0);
        break;
    case PAGE_SETTINGS:
        efb_circle(cx, cy, 10, color, 0); efb_circle(cx, cy, 3, color, 0);
        efb_line(cx, cy + 10, cx, cy + 15, color, 2);
        efb_line(cx, cy - 10, cx, cy - 15, color, 2);
        efb_line(cx - 10, cy, cx - 15, cy, color, 2);
        efb_line(cx + 10, cy, cx + 15, cy, color, 2);
        break;
    default: break;
    }
    glLineWidth(1.0f);
}

static void draw_toolbar(void)
{
    int i;
    efb_rectangle(152, 516, 226, 986, EFB_BODY);
    for (i = 0; i < PAGE_SETTINGS; ++i) {
        draw_toolbar_icon(i, 189.0f, 948.0f - i * 42.0f, i == (int)active_page);
    }
    efb_line(169, 579, 209, 579, EFB_ACCENT, 3);
    draw_toolbar_icon(PAGE_SETTINGS, 189, 547, active_page == PAGE_SETTINGS);
}

static void draw_status_bar(void)
{
    char text[64];
    int zulu = (int)get_float(refs.zulu) % 86400;
    float battery_right;
    efb_rectangle(152, 986, 872, 1016, EFB_STATUS);
    efb_text(166, 997, "A380-842", EFB_WHITE);
    snprintf(text, sizeof(text), "%02d:%02dZ", zulu / 3600, (zulu % 3600) / 60);
    efb_text(493, 997, text, EFB_WHITE);
    efb_text(760, 997, "WiFi", EFB_UNSELECTED);
    snprintf(text, sizeof(text), "%.0f%%", battery_level);
    efb_text(800, 997, text, EFB_WHITE);
    efb_outline(837, 994, 860, 1008, EFB_WHITE);
    efb_rectangle(860, 998, 863, 1004, EFB_WHITE);
    battery_right = 839.0f + 19.0f * (float)(battery_level / 100.0);
    if (battery_right > 858.0f) battery_right = 858.0f;
    efb_rectangle(839, 996, battery_right, 1006,
                  battery_level < 15.0 ? EFB_RED : (electrical_power_available() ? EFB_GREEN : EFB_WHITE));
}

static void draw_action_buttons(void)
{
    int i;
    for (i = 0; i < 6; ++i) {
        int column = i % 3, row = i / 3;
        float left = 320.0f + column * 182.0f;
        float bottom = row == 0 ? 598.0f : 538.0f;
        draw_button(left, bottom, left + 170.0f, bottom + 48.0f,
                    action_label(active_page, i), 0);
    }
}

static void draw_dashboard(void)
{
    char text[128];
    draw_card(238, 688, 548, 925, "Your Flight");
    efb_text(250, 875, "No SimBrief flight loaded", EFB_UNSELECTED);
    efb_text(250, 850, "External SimBrief client required", EFB_UNSELECTED);
    snprintf(text, sizeof(text), "Altitude  %.0f ft", get_float(refs.altitude));
    efb_text(250, 806, text, EFB_WHITE);
    snprintf(text, sizeof(text), "Ground speed  %.0f kt", get_float(refs.ground_speed));
    efb_text(250, 782, text, EFB_WHITE);
    snprintf(text, sizeof(text), "Fuel  %.0f kg", get_float(refs.fuel_total));
    efb_text(250, 758, text, EFB_HIGHLIGHT);
    snprintf(text, sizeof(text), "Gross weight  %.0f kg", get_float(refs.gross_weight));
    efb_text(250, 734, text, EFB_HIGHLIGHT);

    draw_card(558, 688, 860, 925, "Important Information");
    efb_text(570, 875, "Weather", EFB_WHITE);
    snprintf(text, sizeof(text), "QNH %.0f hPa   Wind %03.0f / %.0f kt",
             get_float(refs.qnh) / 100.0f, get_float(refs.wind_direction), get_float(refs.wind_speed));
    efb_text(570, 850, text, EFB_UNSELECTED);
    efb_line(570, 832, 848, 832, EFB_ACCENT, 2);
    efb_text(570, 806, "Maintenance", EFB_WHITE);
    efb_text(570, 781, "No active flyPad notices", EFB_GREEN);
    efb_line(570, 763, 848, 763, EFB_ACCENT, 2);
    efb_text(570, 737, "Checklists", EFB_WHITE);
    efb_text(570, 712, "Open the current aircraft checklist", EFB_UNSELECTED);
}

static void draw_dispatch(void)
{
    char text[128];
    float fuel = get_float(refs.fuel_total), gross = get_float(refs.gross_weight);
    draw_tabs("OFP", "Overview", NULL, NULL);
    draw_card(238, 688, 860, 925, "Operational Flight Plan");
    efb_text(252, 878, "Live Loads", EFB_WHITE);
    snprintf(text, sizeof(text), "Zero fuel weight                 %.0f kg", gross - fuel);
    efb_text(252, 844, text, EFB_WHITE);
    snprintf(text, sizeof(text), "Block fuel                       %.0f / %.0f kg", fuel, MAX_FUEL_KG);
    efb_text(252, 816, text, EFB_HIGHLIGHT);
    snprintf(text, sizeof(text), "Payload                          %.0f kg", get_float(refs.payload));
    efb_text(252, 788, text, EFB_HIGHLIGHT);
    snprintf(text, sizeof(text), "Gross weight                     %.0f / 510000 kg", gross);
    efb_text(252, 760, text, gross > 510000.0f ? EFB_AMBER : EFB_GREEN);
    efb_line(252, 741, 846, 741, EFB_ACCENT, 2);
    efb_text(252, 714, "SimBrief import requires an external client", EFB_UNSELECTED);
}

static void draw_ground(void)
{
    float doors[20] = {0};
    char text[160];
    int i, open = 0;
    if (refs.doors != NULL) XPLMGetDatavf(refs.doors, doors, 0, 20);
    for (i = 0; i < 20; ++i) if (doors[i] > 0.5f) ++open;
    draw_tabs("Services", "Fuel", "Payload", "Pushback");
    draw_card(238, 688, 860, 925, "Ground Services");
    /* Compact top view in the same central-aircraft style as FBW's A380 service page. */
    efb_line(505, 735, 505, 884, EFB_UNSELECTED, 2);
    efb_line(505, 854, 430, 784, EFB_UNSELECTED, 2);
    efb_line(505, 854, 580, 784, EFB_UNSELECTED, 2);
    efb_line(505, 768, 476, 738, EFB_UNSELECTED, 2);
    efb_line(505, 768, 534, 738, EFB_UNSELECTED, 2);
    snprintf(text, sizeof(text), "GPU  %s", gpu_connected() ? "Connected" : "Disconnected");
    efb_text(264, 864, text, gpu_connected() ? EFB_GREEN : EFB_UNSELECTED);
    snprintf(text, sizeof(text), "Doors open  %d / 17", open);
    efb_text(264, 830, text, open ? EFB_HIGHLIGHT : EFB_UNSELECTED);
    snprintf(text, sizeof(text), "Fuel  %.0f kg", get_float(refs.fuel_total));
    efb_text(650, 864, text, EFB_WHITE);
    snprintf(text, sizeof(text), "Payload  %.0f kg", get_float(refs.payload));
    efb_text(650, 830, text, EFB_WHITE);
    efb_text(264, 712, "Loading is available on ground with engines off", EFB_AMBER);
}

static void draw_performance(void)
{
    char text[160];
    double current = get_float(refs.altitude);
    double speed = a380x_efb_average_ground_speed(current, target_altitude_ft,
                                                   fmax(1.0, get_float(refs.ground_speed)));
    double distance = a380x_efb_tod_distance_nm(current, target_altitude_ft, target_angle_deg);
    draw_tabs("Top of Descent", "Temp correction", NULL, NULL);
    draw_card(238, 688, 860, 925, "Top of Descent Calculator");
    efb_text(258, 874, "Current altitude", EFB_UNSELECTED);
    snprintf(text, sizeof(text), "%.0f ft", current);
    efb_text(258, 846, text, EFB_WHITE);
    efb_text(448, 874, "Target altitude", EFB_UNSELECTED);
    snprintf(text, sizeof(text), "%d ft", target_altitude_ft);
    efb_text(448, 846, text, EFB_WHITE);
    efb_text(650, 874, "Flight path angle", EFB_UNSELECTED);
    snprintf(text, sizeof(text), "%.1f deg", target_angle_deg);
    efb_text(650, 846, text, EFB_WHITE);
    efb_line(258, 817, 840, 817, EFB_ACCENT, 2);
    snprintf(text, sizeof(text), "Start descent approximately %.0f NM before target", distance);
    efb_text(258, 780, text, EFB_HIGHLIGHT);
    snprintf(text, sizeof(text), "Calculated average ground speed  %.0f kt", speed);
    efb_text(258, 748, text, EFB_WHITE);
    efb_text(258, 714, "A380 takeoff and landing calculators are not supplied upstream", EFB_AMBER);
}

static void draw_navigation(void)
{
    int x, y;
    draw_tabs("Navigraph", "Local files", "Pinned", NULL);
    draw_card(238, 688, 860, 925, "Charts and Map");
    for (x = 260; x <= 838; x += 48) efb_line((float)x, 716, (float)x, 884, EFB_ACCENT, 1);
    for (y = 716; y <= 884; y += 42) efb_line(260, (float)y, 838, (float)y, EFB_ACCENT, 1);
    efb_circle(548, 800, 7, EFB_HIGHLIGHT, 1);
    efb_line(548, 800, 610, 845, EFB_HIGHLIGHT, 2);
    efb_text(272, 898, "Native X-Plane map boundary", EFB_UNSELECTED);
    efb_text(560, 715, "Navigraph / ChartFox authentication unavailable", EFB_UNSELECTED);
}

static void draw_atc(void)
{
    char text[128];
    draw_tabs("ATIS", "AOC", "CPDLC", NULL);
    draw_card(238, 688, 860, 925, "Air Traffic Control");
    snprintf(text, sizeof(text), "COM1 active       %.3f", get_int(refs.com1) / 1000.0);
    efb_text(260, 866, text, EFB_WHITE);
    snprintf(text, sizeof(text), "COM1 standby      %.3f", get_int(refs.com1_standby) / 1000.0);
    efb_text(260, 834, text, EFB_WHITE);
    snprintf(text, sizeof(text), "Transponder       %04d      Mode %d",
             get_int(refs.transponder), get_int(refs.transponder_mode));
    efb_text(260, 802, text, EFB_HIGHLIGHT);
    efb_line(260, 776, 838, 776, EFB_ACCENT, 2);
    efb_text(260, 744, "No online ATC messages", EFB_UNSELECTED);
    efb_text(260, 716, "Hoppie connectivity requires an external client", EFB_UNSELECTED);
}

static void draw_failures(void)
{
    char text[128];
    int failed = refs.failures[selected_failure] != NULL
        && XPLMGetDatai(refs.failures[selected_failure]) >= 6;
    draw_tabs("Comfort", "Compact", NULL, NULL);
    draw_card(238, 688, 860, 925, "Failures");
    snprintf(text, sizeof(text), "Selected failure  %d / 17", selected_failure + 1);
    efb_text(260, 870, text, EFB_UNSELECTED);
    snprintf(text, sizeof(text), "%s", 
             failure_names[selected_failure]);
    efb_text(260, 838, text, EFB_WHITE);
    snprintf(text, sizeof(text), "State  %s", failed ? "FAILED" : "OPERATIONAL");
    efb_text(260, 800, text, failed ? EFB_RED : EFB_GREEN);
    efb_line(260, 775, 838, 775, EFB_ACCENT, 2);
    efb_text(260, 742, "Use Fail / repair to change the selected X-Plane failure", EFB_UNSELECTED);
}

static void draw_checklists(void)
{
    char text[160];
    unsigned full = (1u << checklist_item_counts[selected_checklist]) - 1u;
    unsigned complete = checklist_bits[selected_checklist];
    unsigned count = 0, bit;
    for (bit = 1u; bit <= full; bit <<= 1u) if (complete & bit) ++count;
    draw_card(238, 688, 860, 925, "Aircraft Checklists");
    snprintf(text, sizeof(text), "%d / 10    %s", selected_checklist + 1,
             checklist_names[selected_checklist]);
    efb_text(260, 870, text, EFB_WHITE);
    efb_line(260, 846, 838, 846, EFB_ACCENT, 2);
    snprintf(text, sizeof(text), "Completed  %u / %u", count,
             checklist_item_counts[selected_checklist]);
    efb_text(260, 812, text, complete == full ? EFB_GREEN : EFB_HIGHLIGHT);
    snprintf(text, sizeof(text), "Automatic item completion  %s", automatic_checklists ? "On" : "Off");
    efb_text(260, 780, text, EFB_WHITE);
    efb_text(260, 742, "Checklist definitions: pinned FlyByWire A380X", EFB_UNSELECTED);
}

static void draw_presets(void)
{
    draw_tabs("Interior lights", "Aircraft states", "?", NULL);
    draw_card(238, 688, 860, 925, "Aircraft States");
    efb_text(260, 868, "Cold & Dark", EFB_WHITE);
    efb_text(260, 834, "Turnaround", EFB_WHITE);
    efb_text(260, 800, "Ready for Taxi", EFB_WHITE);
    efb_line(260, 776, 838, 776, EFB_ACCENT, 2);
    efb_text(260, 742, "Aircraft state presets are available while parked", EFB_AMBER);
    efb_text(260, 714, "Translated to guarded native X-Plane commands", EFB_UNSELECTED);
}

static void draw_settings(void)
{
    char text[160];
    draw_tabs("flyPad", "Aircraft", "Realism", "About");
    draw_card(238, 688, 860, 925, "flyPad Settings");
    snprintf(text, sizeof(text), "Units                              %s",
             metric_units ? "Metric" : "Imperial");
    efb_text(260, 870, text, EFB_WHITE);
    snprintf(text, sizeof(text), "Automatic checklists               %s", automatic_checklists ? "On" : "Off");
    efb_text(260, 834, text, EFB_WHITE);
    snprintf(text, sizeof(text), "Battery simulation                 %s", battery_simulation ? "On" : "Off");
    efb_text(260, 798, text, EFB_WHITE);
    snprintf(text, sizeof(text), "Brightness                         %d%%", brightness);
    efb_text(260, 762, text, EFB_HIGHLIGHT);
    efb_line(260, 740, 838, 740, EFB_ACCENT, 2);
    efb_text(260, 712, "A380X FBW Avionics + flyPad", EFB_GREEN);
}

void a380x_efb_draw(void)
{
    static void (*draw_pages[PAGE_COUNT])(void) = {
        draw_dashboard, draw_dispatch, draw_ground, draw_performance, draw_navigation,
        draw_atc, draw_failures, draw_checklists, draw_presets, draw_settings
    };
    efb_rectangle(EFB_LEFT, EFB_BOTTOM, EFB_RIGHT, EFB_TOP,
                  powered ? EFB_BODY : (EfbColor){0.0f, 0.0f, 0.0f});
    if (!powered || (battery_simulation && battery_level <= 0.0 && !electrical_power_available())) {
        /* The upstream SHUTOFF/STANDBY state is intentionally blank. */
        return;
    }
    draw_status_bar();
    draw_toolbar();
    efb_text(238, 954, page_titles[active_page], EFB_WHITE);
    draw_pages[active_page]();
    draw_action_buttons();
    efb_text(238, 660, status_message, EFB_AMBER);
}
#endif

#include "a380x_efb_render.inc"
#include "a380x_iscs.inc"
#ifndef A380X_EFB_TEST
#include "a380x_efb_window.inc"
#endif
#include "a380x_efb_web.inc"
#include "a380x_efb_device.inc"

void a380x_efb_update(float dt_seconds)
{
    int was_blank = efb_tablet_blank();
    efb_web_update(dt_seconds);
    if (battery_simulation && dt_seconds > 0.0f) {
        battery_level = a380x_efb_battery_level(
            battery_level, electrical_power_available(), dt_seconds
        );
        if (battery_level <= 0.0 && !electrical_power_available()) powered = 0;
    }
    if (was_blank != efb_tablet_blank()) efb_changed();
    efb_devices_update(dt_seconds);
}

int a380x_efb_init(void)
{
    int i;
    memset(&refs, 0, sizeof(refs));
    command_count = 0;
    refs.zulu = optional_ref("sim/time/zulu_time_sec");
    refs.on_ground = optional_ref("sim/flightmodel/failures/onground_any");
    refs.latitude = optional_ref("sim/flightmodel/position/latitude");
    refs.longitude = optional_ref("sim/flightmodel/position/longitude");
    refs.altitude = optional_ref("sim/cockpit2/gauges/indicators/altitude_ft_pilot");
    refs.vertical_speed = optional_ref("sim/cockpit2/gauges/indicators/vvi_fpm_pilot");
    refs.ground_speed = optional_ref("sim/cockpit2/gauges/indicators/ground_speed_kt");
    refs.gps_distance = optional_ref("sim/cockpit2/radios/indicators/gps_dme_distance_nm");
    refs.gps_time = optional_ref("sim/cockpit2/radios/indicators/gps_dme_time_min");
    refs.payload = optional_ref("sim/flightmodel/weight/m_fixed");
    refs.gross_weight = optional_ref("sim/flightmodel/weight/m_total");
    refs.fuel_total = optional_ref("sim/flightmodel/weight/m_fuel_total");
    refs.max_fuel = optional_ref("sim/aircraft/weight/acf_m_fuel_tot");
    refs.batteries = optional_ref("sim/cockpit2/electrical/battery_on");
    refs.gpu = optional_ref("sim/cockpit2/electrical/GPU_generator_on");
    refs.gpu_available = optional_ref("sim/cockpit2/electrical/GPU_can_be_called_for");
    refs.apu = optional_ref("sim/cockpit2/electrical/APU_running");
    refs.generators = optional_ref("sim/cockpit2/electrical/generator_on");
    refs.engines = optional_ref("sim/flightmodel2/engines/engine_is_burning_fuel");
    refs.fuel = optional_ref("sim/flightmodel/weight/m_fuel");
    refs.tank_ratio = optional_ref("sim/aircraft/overflow/acf_tank_rat");
    refs.doors = optional_ref("sim/flightmodel2/misc/door_open_ratio");
    refs.oat = optional_ref("sim/weather/aircraft/temperature_ambient_deg_c");
    refs.qnh = optional_ref("sim/weather/aircraft/qnh_pas");
    refs.wind_direction = optional_ref("sim/weather/aircraft/wind_direction_degt");
    refs.wind_speed = optional_ref("sim/weather/aircraft/wind_speed_kts");
    refs.com1 = optional_ref("sim/cockpit2/radios/actuators/com1_frequency_hz_833");
    refs.com1_standby = optional_ref("sim/cockpit2/radios/actuators/com1_standby_frequency_hz_833");
    refs.transponder = optional_ref("sim/cockpit2/radios/actuators/transponder_code");
    refs.transponder_mode = optional_ref("sim/cockpit2/radios/actuators/transponder_mode");
    refs.avionics = optional_ref("sim/cockpit2/switches/avionics_power_on");
    refs.seatbelts = optional_ref("sim/cockpit2/switches/fasten_seat_belts");
    refs.beacon = optional_ref("sim/cockpit2/switches/beacon_on");
    refs.gear = optional_ref("sim/flightmodel2/gear/deploy_ratio");
    refs.speedbrake = optional_ref("sim/cockpit2/controls/speedbrake_ratio");
    refs.parking_brake = optional_ref("sim/flightmodel/controls/parkbrake");
    for (i = 0; i < ARRAY_LEN(refs.failures); ++i) refs.failures[i] = optional_ref(failure_refs[i]);
    efb_web_init();

    if (XPLMFindDataRef("a380x/xp12/efb/native_available") != NULL) return 0;
    native_available_ref = XPLMRegisterDataAccessor(
        "a380x/xp12/efb/native_available", xplmType_Int, 0,
        read_native_available, NULL, NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL
    );
    if (native_available_ref == NULL) return 0;
    for (i = 0; i < PAGE_COUNT; ++i) {
        char description[96];
        snprintf(description, sizeof(description), "A380X flyPad: show %s page", page_names[i]);
        if (!add_command(page_command_names[i], description, COMMAND_PAGE, i)) return 0;
    }
    for (i = 0; i < 6; ++i) {
        static char names[6][40];
        static char descriptions[6][64];
        snprintf(names[i], sizeof(names[i]), "a380x/xp12/efb/action_%d", i + 1);
        snprintf(descriptions[i], sizeof(descriptions[i]), "A380X flyPad: on-screen action %d", i + 1);
        if (!add_command(names[i], descriptions[i], COMMAND_ACTION, i)) return 0;
    }
    if (!add_command("a380x/xp12/efb/power", "A380X flyPad: power / standby", COMMAND_POWER, 0)
            || !add_command("a380x/xp12/efb/toggle", "A380X flyPad: power / standby", COMMAND_POWER, 0)
            || !add_command("a380x/xp12/efb/next_page", "A380X flyPad: next page", COMMAND_NEXT, 0)
            || !add_command("a380x/xp12/efb/slides_arm_all", "A380X EFB: arm the escape slides of all closed passenger doors", COMMAND_SLIDES, 1)
            || !add_command("a380x/xp12/efb/slides_disarm_all", "A380X EFB: disarm all escape slides", COMMAND_SLIDES, 0)
            || !add_command("a380x/xp12/efb/captain/popup", "A380X EFB: captain tablet pop-up", COMMAND_POPUP, 0)
            || !add_command("a380x/xp12/efb/first_officer/popup", "A380X EFB: first officer tablet pop-up", COMMAND_POPUP, 1)) {
        return 0;
    }
    if (!efb_datarefs_register()) return 0;
    if (!efb_window_init()) return 0;
    /* Before X-Plane loads the cockpit OBJs that reference the tablets (ATTR_cockpit_device). */
    efb_devices_create();
    XPLMDebugString("A380X control panel: native window and tablet touch commands registered.\n");
    return 1;
}

void a380x_efb_shutdown(void)
{
    int i;
    efb_devices_destroy();
    efb_datarefs_unregister();
    efb_web_shutdown();
    efb_window_shutdown();
    memset(iscs_sound_refs, 0, sizeof(iscs_sound_refs));
    iscs_sound_refs_ready = 0;
    for (i = 0; i < command_count; ++i) {
        if (commands[i].ref != NULL) {
            XPLMUnregisterCommandHandler(commands[i].ref, command_handler, 1, &commands[i]);
        }
    }
    command_count = 0;
    if (native_available_ref != NULL) {
        XPLMUnregisterDataAccessor(native_available_ref);
        native_available_ref = NULL;
    }
}
