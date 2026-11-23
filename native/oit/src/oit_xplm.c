/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380X OIT plugin, X-Plane side: the two OIT cockpit devices (a380x.xp12.oit.captain / .first_officer, mapped
 * in a380x_mfd_screens.obj over the OIT glass), the KCCU/OIT keyboard and cursor keys
 * (a380x/xp12/oit/<side>/{kbd,acd}/KEY, created by the avionics plugin's cockpit feedback, which passes the
 * presses on), the OIT SIDE switch and brightness knob datarefs, and a 10 Hz snapshot of the simulator for the
 * pages (oit_core.c). The devices can also be popped out (window) with a380x/xp12/oit/<side>/popup.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMNavigation.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include "oit_core.h"
#include "oit_draw.h"

OitBackend *oit_gl_backend(void);
void oit_gl_release(void);

static const char *const side_name[2] = {"captain", "first_officer"};

/* ---------------------------------------------------------------- datarefs */
typedef struct { const char *name; XPLMDataRef ref; } Ref;
enum {
    R_BUS_VOLTS, R_AV_POWER, R_ZULU, R_DAY, R_RUN_TIME, R_TAIL, R_ICAO, R_LAT, R_LON, R_ELEV, R_IAS, R_GS, R_VS,
    R_HDG, R_MACH, R_OAT, R_TAT, R_ONGROUND, R_PARKBRAKE, R_PAUSED, R_ENG_RUN, R_N1, R_FF, R_M_TOTAL, R_M_FUEL,
    R_M_EMPTY, R_M_FIXED, R_M_TANKS, R_NUM_TANKS, R_FLAPS, R_GEAR, R_WIND_DIR, R_WIND_KT, R_DOORS,
    R_FMS_V1, R_FMS_VR, R_FMS_V2, R_FMS_FLEX, R_FMS_THS, R_FMS_ZFW, R_FMS_ZFWCG, R_FMS_GW, R_FMS_CRZ, R_FMS_VAPP,
    R_FMS_VLS, R_FMS_VREF, R_FMS_LDG_ELEV, R_FMS_DH, R_FMS_MDA, R_FMS_TRANS, R_FMS_TO_FLAPS, R_FMS_PHASE,
    R_FMS_TO_POWER, R_FMS_APP_F, R_FMS_APP_S, R_FMS_APP_GD, R_FMS_HEADWIND, R_FMS_CONF3, R_M_MAX,
    R_SIDE_C, R_SIDE_F, R_BRT_C, R_BRT_F, R_LAPTOP_C, R_LAPTOP_F, R_DEST_LAT, R_DEST_LON, R_DEST_VALID, R_COUNT
};
static Ref refs[R_COUNT] = {
    {"sim/cockpit2/electrical/bus_volts"}, {"sim/cockpit2/switches/avionics_power_on"}, {"sim/time/zulu_time_sec"},
    {"sim/time/local_date_days"}, {"sim/time/total_running_time_sec"}, {"sim/aircraft/view/acf_tailnum"},
    {"sim/aircraft/view/acf_ICAO"}, {"sim/flightmodel/position/latitude"}, {"sim/flightmodel/position/longitude"},
    {"sim/flightmodel/position/elevation"}, {"sim/flightmodel/position/indicated_airspeed"},
    {"sim/flightmodel/position/groundspeed"}, {"sim/flightmodel/position/vh_ind_fpm"},
    {"sim/flightmodel/position/mag_psi"}, {"sim/flightmodel/misc/machno"},
    {"sim/cockpit2/temperature/outside_air_temp_degc"}, {"sim/cockpit2/temperature/outside_air_LE_temp_degc"},
    {"sim/flightmodel/failures/onground_any"}, {"sim/flightmodel/controls/parkbrake"}, {"sim/time/paused"},
    {"sim/flightmodel/engine/ENGN_running"}, {"sim/cockpit2/engine/indicators/N1_percent"},
    {"sim/cockpit2/engine/indicators/fuel_flow_kg_sec"}, {"sim/flightmodel/weight/m_total"},
    {"sim/flightmodel/weight/m_fuel_total"}, {"sim/aircraft/weight/acf_m_empty"}, {"sim/flightmodel/weight/m_fixed"},
    {"sim/flightmodel/weight/m_fuel"}, {"sim/aircraft/overflow/acf_num_tanks"},
    {"sim/cockpit2/controls/flap_handle_deploy_ratio"}, {"sim/cockpit2/controls/gear_handle_down"},
    {"sim/cockpit2/gauges/indicators/wind_heading_deg_mag"}, {"sim/cockpit2/gauges/indicators/wind_speed_kts"},
    {"sim/flightmodel2/misc/door_open_ratio"},
    {"a380x/xp12/fms/v1_knots"}, {"a380x/xp12/fms/vr_knots"}, {"a380x/xp12/fms/v2_knots"},
    {"a380x/xp12/fms/to_flex_temp_c"}, {"a380x/xp12/fms/to_ths_for_pct"}, {"a380x/xp12/fms/zfw_kg"},
    {"a380x/xp12/fms/zfw_cg_pct"}, {"a380x/xp12/fms/gross_weight_kg"}, {"a380x/xp12/fms/cruise_altitude_ft"},
    {"a380x/xp12/fms/approach_vapp_knots"}, {"a380x/xp12/fms/approach_vls_knots"},
    {"a380x/xp12/fms/approach_vref_knots"}, {"a380x/xp12/fms/landing_elevation_ft"}, {"a380x/xp12/fms/dh_ft"},
    {"a380x/xp12/fms/mda_ft"}, {"a380x/xp12/fms/trans_alt_ft"}, {"a380x/xp12/fms/to_flaps"},
    {"a380x/xp12/fms/flight_phase"}, {"a380x/xp12/fms/to_power_setting"},
    {"a380x/xp12/fms/approach_f_knots"}, {"a380x/xp12/fms/approach_s_knots"}, {"a380x/xp12/fms/approach_gd_knots"},
    {"a380x/xp12/fms/approach_headwind_knots"}, {"a380x/xp12/fms/landing_conf3"}, {"sim/aircraft/weight/acf_m_max"},
    {"a380x/xp12/oit/captain_side"}, {"a380x/xp12/oit/first_officer_side"},
    {"a380x/xp12/oit/captain_brightness"}, {"a380x/xp12/oit/first_officer_brightness"},
    {"a380x/xp12/cockpit/captain_laptop_power"}, {"a380x/xp12/cockpit/first_officer_laptop_power"},
    {"a380x/xp12/fms/dest_lat_deg"}, {"a380x/xp12/fms/dest_lon_deg"}, {"a380x/xp12/fms/dest_pos_valid"},
};

static void find_refs(void) { int i; for (i = 0; i < R_COUNT; ++i) if (!refs[i].ref) refs[i].ref = XPLMFindDataRef(refs[i].name); }

/* Typed reads that tolerate missing datarefs and either int or float/double storage. */
static double num(int k)
{
    XPLMDataRef r = refs[k].ref;
    int t;
    if (!r) return 0.0;
    t = XPLMGetDataRefTypes(r);
    if (t & xplmType_Double) return XPLMGetDatad(r);
    if (t & xplmType_Float) return XPLMGetDataf(r);
    if (t & xplmType_Int) return XPLMGetDatai(r);
    if (t & xplmType_IntArray) { int v = 0; XPLMGetDatavi(r, &v, 0, 1); return v; }
    if (t & xplmType_FloatArray) { float v = 0; XPLMGetDatavf(r, &v, 0, 1); return v; }
    return 0.0;
}
static int vec(int k, float *out, int n)
{
    XPLMDataRef r = refs[k].ref;
    int t, got = 0, i;
    memset(out, 0, sizeof(float) * (size_t)n);
    if (!r) return 0;
    t = XPLMGetDataRefTypes(r);
    if (t & xplmType_FloatArray) got = XPLMGetDatavf(r, out, 0, n);
    else if (t & xplmType_IntArray) { int iv[32] = {0}; got = XPLMGetDatavi(r, iv, 0, n < 32 ? n : 32); for (i = 0; i < got; ++i) out[i] = (float)iv[i]; }
    return got;
}
static void text(int k, char *out, int n)
{
    XPLMDataRef r = refs[k].ref;
    int got;
    out[0] = 0;
    if (!r || !(XPLMGetDataRefTypes(r) & xplmType_Data)) return;
    got = XPLMGetDatab(r, out, 0, n - 1);
    out[got > 0 ? got : 0] = 0;
}

static OitSim sim;

/* CMS current faults: X-Plane failure datarefs, 6 = inoperative (read at 1 Hz). */
static XPLMDataRef fault_ref[64];
static unsigned char fault_state[64];
static double fault_read_at = -1.0;
static void read_faults(void)
{
    int i;
    double now = XPLMGetElapsedTime();
    if (fault_read_at >= 0.0 && now - fault_read_at < 1.0) { memcpy(sim.fault, fault_state, sizeof(sim.fault)); return; }
    fault_read_at = now;
    for (i = 0; i < oit_fault_count && i < 64; ++i) {
        char name[96];
        if (!fault_ref[i]) { snprintf(name, sizeof(name), "sim/operation/failures/%s", oit_faults[i].ref); fault_ref[i] = XPLMFindDataRef(name); }
        fault_state[i] = fault_ref[i] && XPLMGetDatai(fault_ref[i]) == 6;
    }
    memcpy(sim.fault, fault_state, sizeof(sim.fault));
}

static void snapshot(void)
{
    float v[32];
    int i, n;
    struct tm now_tm;
    time_t now = time(NULL);
    find_refs();
    memset(&sim, 0, sizeof(sim));
    n = vec(R_BUS_VOLTS, v, 6);
    for (i = 0; i < n; ++i) if (v[i] > 18.0f) sim.powered = 1;
    sim.zulu_s = num(R_ZULU); sim.day_of_year = (int)num(R_DAY); sim.sim_time_s = num(R_RUN_TIME);
#if IBM
    gmtime_s(&now_tm, &now);
#else
    gmtime_r(&now, &now_tm);
#endif
    sim.year = now_tm.tm_year + 1900;
    text(R_TAIL, sim.tail, sizeof(sim.tail)); text(R_ICAO, sim.icao, sizeof(sim.icao));
    sim.lat = num(R_LAT); sim.lon = num(R_LON); sim.alt_ft = (float)(num(R_ELEV) * 3.28084);
    sim.ias_kt = (float)num(R_IAS); sim.gs_kt = (float)(num(R_GS) * 1.943844); sim.vs_fpm = (float)num(R_VS);
    sim.hdg_mag = (float)num(R_HDG); sim.mach = (float)num(R_MACH); sim.oat_c = (float)num(R_OAT); sim.tat_c = (float)num(R_TAT);
    sim.on_ground = num(R_ONGROUND) != 0; sim.parking_brake = num(R_PARKBRAKE) > 0.5; sim.paused = num(R_PAUSED) != 0;
    vec(R_ENG_RUN, v, 4); for (i = 0; i < 4; ++i) sim.eng_running[i] = v[i] != 0;
    vec(R_N1, sim.n1, 4);
    vec(R_FF, v, 4); for (i = 0; i < 4; ++i) sim.fuel_flow_kgh[i] = v[i] * 3600.0f;
    sim.gw_kg = (float)num(R_M_TOTAL); sim.fuel_kg = (float)num(R_M_FUEL); sim.empty_kg = (float)num(R_M_EMPTY);
    sim.payload_kg = (float)num(R_M_FIXED); sim.cg_mac = -999.0f;
    sim.fuel_tanks = (int)num(R_NUM_TANKS); if (sim.fuel_tanks > 11) sim.fuel_tanks = 11;
    vec(R_M_TANKS, sim.fuel_tank_kg, 11);
    sim.flaps_handle = (float)num(R_FLAPS); sim.gear_down = num(R_GEAR) != 0;
    sim.wind_dir = (float)num(R_WIND_DIR); sim.wind_kt = (float)num(R_WIND_KT);
    n = vec(R_DOORS, sim.door_ratio, 20); for (i = 0; i < n; ++i) if (sim.door_ratio[i] > 0.02f) ++sim.doors_open;
    sim.app_f = (float)num(R_FMS_APP_F); sim.app_s = (float)num(R_FMS_APP_S); sim.app_gd = (float)num(R_FMS_APP_GD);
    sim.app_headwind = (float)num(R_FMS_HEADWIND); sim.ldg_conf3 = num(R_FMS_CONF3) != 0; sim.mtow_kg = (float)num(R_M_MAX);
    read_faults();
    sim.fms_ok = refs[R_FMS_V1].ref != NULL;
    sim.v1 = (float)num(R_FMS_V1); sim.vr = (float)num(R_FMS_VR); sim.v2 = (float)num(R_FMS_V2);
    sim.flex_c = (float)num(R_FMS_FLEX); sim.ths = (float)num(R_FMS_THS); sim.zfw_kg = (float)num(R_FMS_ZFW);
    sim.zfw_cg = (float)num(R_FMS_ZFWCG); sim.fms_gw_kg = (float)num(R_FMS_GW); sim.crz_alt_ft = (float)num(R_FMS_CRZ);
    sim.vapp = (float)num(R_FMS_VAPP); sim.vls = (float)num(R_FMS_VLS); sim.vref = (float)num(R_FMS_VREF);
    sim.ldg_elev_ft = (float)num(R_FMS_LDG_ELEV); sim.dh_ft = (float)num(R_FMS_DH); sim.mda_ft = (float)num(R_FMS_MDA);
    sim.trans_alt_ft = (float)num(R_FMS_TRANS); sim.to_flaps = (int)num(R_FMS_TO_FLAPS);
    sim.flight_phase = (int)num(R_FMS_PHASE); sim.to_power = (int)num(R_FMS_TO_POWER);
}

/* Nearest airport to a position (X-Plane navigation database), refreshed every 5 s. */
static void nearest_airport(double lat, double lon, char *out, int n)
{
    float la = (float)lat, lo = (float)lon;
    XPLMNavRef r = XPLMFindNavAid(NULL, NULL, &la, &lo, NULL, xplm_Nav_Airport);
    char id[32] = "";
    out[0] = 0;
    if (r == XPLM_NAV_NOT_FOUND) return;
    XPLMGetNavAidInfo(r, NULL, NULL, NULL, NULL, NULL, NULL, id, NULL, NULL);
    snprintf(out, (size_t)n, "%s", id);
}
static char near_apt[8], dest_apt[8];
static double apt_at = -100.0;
static void update_airports(void)
{
    double now = XPLMGetElapsedTime();
    if (now - apt_at < 5.0) return;
    apt_at = now;
    nearest_airport(sim.lat, sim.lon, near_apt, sizeof(near_apt));
    if (refs[R_DEST_VALID].ref && num(R_DEST_VALID) != 0) nearest_airport(num(R_DEST_LAT), num(R_DEST_LON), dest_apt, sizeof(dest_apt));
    else dest_apt[0] = 0;
}

/* FLT OPS laptops: the laptop POWER switches; on a start with the engines running (runway / in-flight spawn) both
 * laptops are on, as in the FBW A380X, until the crew switches one off. */
static int laptop_auto[2] = {-1, -1};
static double laptop_prev[2];
static int laptop_on(int side)
{
    int k = side == OIT_CAPT ? R_LAPTOP_C : R_LAPTOP_F;
    double v;
    if (!refs[k].ref) return 1;
    v = num(k);
    if (laptop_auto[side] < 0) laptop_auto[side] = sim.eng_running[0] || sim.eng_running[1] || sim.eng_running[2] || sim.eng_running[3];
    if (laptop_prev[side] >= 0.5 && v < 0.5) laptop_auto[side] = 0;   /* switched off by the crew */
    laptop_prev[side] = v;
    return v >= 0.5 || laptop_auto[side] == 1;
}

/* OIT knob: 0 = OIT off. On a start with the engines running the OITs are on (knob treated as 80 %) until the
 * crew turns the knob. */
static int knob_auto[2] = {-1, -1};
static float knob_seen[2];
static float knob(int side)
{
    int k = side == OIT_CAPT ? R_BRT_C : R_BRT_F;
    float v;
    if (!refs[k].ref) return 0.8f;
    v = (float)num(k);
    if (knob_auto[side] < 0) { knob_auto[side] = sim.eng_running[0] || sim.eng_running[1] || sim.eng_running[2] || sim.eng_running[3]; knob_seen[side] = v; }
    if (fabsf(v - knob_seen[side]) > 0.001f) knob_auto[side] = 0;
    return knob_auto[side] == 1 && v < 0.02f ? 0.8f : v;
}

/* The view of one OIT: its side switch, knob (0 = OIT off) and laptop. */
static OitSim view_for(int side)
{
    OitSim v = sim;
    v.avncs = (side == OIT_CAPT ? num(R_SIDE_C) : num(R_SIDE_F)) >= 0.5;
    v.brightness = knob(side);
    v.powered = sim.powered && v.brightness > 0.02f;
    v.laptop_on = laptop_on(side);
    snprintf(v.near_apt, sizeof(v.near_apt), "%s", near_apt);
    snprintf(v.dest_apt, sizeof(v.dest_apt), "%s", dest_apt);
    return v;
}

/* ---------------------------------------------------------------- devices */
typedef struct {
    int side;
    XPLMAvionicsID dev;
    OitTerminal *term;
    double last_second;
    char id[64];
    OitSim view;     /* snapshot used by the last tick, also drawn */
} Screen;
static Screen screens[2];

static void draw_screen(void *refcon)
{
    Screen *s = (Screen *)refcon;
    oit_set_backend(oit_gl_backend());
    oit_render(s->term, &s->view);
}

static int touch_screen(int x, int y, XPLMMouseStatus status, void *refcon)
{
    Screen *s = (Screen *)refcon;
    if (status == xplm_MouseDown) {
        oit_touch(s->term, (float)x, (float)(OIT_H - y));
        XPLMAvionicsNeedsDrawing(s->dev);
    }
    return 1;
}

static int scroll_screen(int x, int y, int wheel, int clicks, void *refcon)
{
    Screen *s = (Screen *)refcon;
    (void)x; (void)y;
    if (wheel == 0) { oit_scroll(s->term, clicks); XPLMAvionicsNeedsDrawing(s->dev); }
    return 1;
}

static XPLMCursorStatus cursor_screen(int x, int y, void *refcon) { (void)x; (void)y; (void)refcon; return xplm_CursorArrow; }

static int key_popup(char key, XPLMKeyFlags flags, char vkey, void *refcon, int losing)
{
    Screen *s = (Screen *)refcon;
    if (losing || !(flags & xplm_DownFlag)) return 1;
    switch ((unsigned char)vkey) {
    case XPLM_VK_UP: oit_key(s->term, OIT_K_UP, 0); break;
    case XPLM_VK_DOWN: oit_key(s->term, OIT_K_DOWN, 0); break;
    case XPLM_VK_LEFT: oit_key(s->term, OIT_K_LEFT, 0); break;
    case XPLM_VK_RIGHT: oit_key(s->term, OIT_K_RIGHT, 0); break;
    case XPLM_VK_RETURN: case XPLM_VK_ENTER: oit_key(s->term, OIT_K_ENTER, 0); break;
    case XPLM_VK_ESCAPE: oit_key(s->term, OIT_K_ESC, 0); break;
    case XPLM_VK_BACK: oit_key(s->term, OIT_K_BACKSPACE, 0); break;
    case XPLM_VK_DELETE: oit_key(s->term, OIT_K_DEL, 0); break;
    case XPLM_VK_TAB: oit_key(s->term, OIT_K_TAB, 0); break;
    case XPLM_VK_PRIOR: oit_key(s->term, OIT_K_PGUP, 0); break;
    case XPLM_VK_NEXT: oit_key(s->term, OIT_K_PGDN, 0); break;
    default:
        if ((unsigned char)key >= 32 && (unsigned char)key < 127) oit_key(s->term, OIT_K_CHAR, key);
        break;
    }
    XPLMAvionicsNeedsDrawing(s->dev);
    return 1;
}

static float brightness_screen(float rheo, float ambient, float bus_ratio, void *refcon)
{
    Screen *s = (Screen *)refcon;
    float k = s->view.brightness;
    (void)rheo; (void)ambient; (void)bus_ratio;
    if (!s->view.powered) return 0.0f;      /* knob at 0 or no power: OIT off */
    return 0.15f + 0.85f * (k < 0 ? 0 : k > 1 ? 1 : k);
}

static void create_devices(void)
{
    int k;
    for (k = 0; k < 2; ++k) {
        XPLMCreateAvionics_t p;
        char name[64];
        Screen *s = &screens[k];
        if (s->dev) continue;
        s->side = k;
        s->term = oit_terminal(k);
        snprintf(s->id, sizeof(s->id), "a380x.xp12.oit.%s", side_name[k]);
        snprintf(name, sizeof(name), "A380 OIT %s", k == OIT_CAPT ? "CAPT" : "F/O");
        memset(&p, 0, sizeof(p));
        p.structSize = sizeof(p);
        p.screenWidth = OIT_W; p.screenHeight = OIT_H;
        p.bezelWidth = OIT_W + 40; p.bezelHeight = OIT_H + 40; p.screenOffsetX = 20; p.screenOffsetY = 20;
        p.drawOnDemand = 1;
        p.drawCallback = draw_screen;
        p.screenTouchCallback = touch_screen;
        p.screenScrollCallback = scroll_screen;
        p.screenCursorCallback = cursor_screen;
        p.keyboardCallback = key_popup;
        p.brightnessCallback = brightness_screen;
        p.deviceID = s->id;
        p.deviceName = name;
        p.refcon = s;
        s->dev = XPLMCreateAvionicsEx(&p);
        if (!s->dev) XPLMDebugString("A380X OIT: XPLMCreateAvionicsEx failed (X-Plane 12.1 or later is needed)\n");
    }
}

static void destroy_devices(void)
{
    int k;
    for (k = 0; k < 2; ++k) if (screens[k].dev) { XPLMDestroyAvionics(screens[k].dev); screens[k].dev = NULL; }
}

/* ---------------------------------------------------------------- keys */
typedef struct { const char *suffix; int key; char ch; } KeyMap;
static const KeyMap acd_keys[] = {
    {"acd/up", OIT_K_UP}, {"acd/down", OIT_K_DOWN}, {"acd/left", OIT_K_LEFT}, {"acd/right", OIT_K_RIGHT},
    {"acd/enter", OIT_K_ENTER}, {"acd/esc", OIT_K_ESC}, {"acd/ois_menu", OIT_K_MENU}, {"acd/ois_xfr", OIT_K_XFR},
    {"kbd/arrow_up", OIT_K_UP}, {"kbd/arrow_down", OIT_K_DOWN}, {"kbd/arrow_left", OIT_K_LEFT},
    {"kbd/arrow_right", OIT_K_RIGHT}, {"kbd/enter", OIT_K_ENTER}, {"kbd/esc", OIT_K_ESC},
    {"kbd/ois_menu", OIT_K_MENU}, {"kbd/ois_xfr", OIT_K_XFR}, {"kbd/tab", OIT_K_TAB},
    {"kbd/backspace", OIT_K_BACKSPACE}, {"kbd/del", OIT_K_DEL}, {"kbd/skip_up", OIT_K_PGUP},
    {"kbd/skip_down", OIT_K_PGDN}, {"kbd/space_lh", OIT_K_CHAR, ' '}, {"kbd/space_rh", OIT_K_CHAR, ' '},
    {"kbd/minus", OIT_K_CHAR, '-'}, {"kbd/plus", OIT_K_CHAR, '+'}, {"kbd/slash", OIT_K_CHAR, '/'},
    {"kbd/fullstop", OIT_K_CHAR, '.'}, {"kbd/comma", OIT_K_CHAR, ','}, {"kbd/semicolon", OIT_K_CHAR, ';'},
    {"kbd/backslash", OIT_K_CHAR, '\\'}, {"kbd/swung_dash", OIT_K_CHAR, '~'},
};
typedef struct { XPLMCommandRef cmd; int side, key; char ch; } Hook;
static Hook hooks[2 * (36 + 12 + sizeof(acd_keys) / sizeof(acd_keys[0]))];
static int hook_count;

static int key_command(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
    Hook *h = (Hook *)refcon;
    (void)cmd;
    if (phase == xplm_CommandBegin) {
        oit_key(screens[h->side].term, h->key, h->ch);
        if (screens[h->side].dev) XPLMAvionicsNeedsDrawing(screens[h->side].dev);
    }
    return 1;   /* the cockpit feedback (key animation) still runs */
}

static void hook(int side, const char *suffix, int key, char ch)
{
    char name[128];
    Hook *h;
    if (hook_count >= (int)(sizeof(hooks) / sizeof(hooks[0]))) return;
    snprintf(name, sizeof(name), "a380x/xp12/oit/%s/%s", side_name[side], suffix);
    h = &hooks[hook_count];
    h->cmd = XPLMCreateCommand(name, "A380 OIT key");
    if (!h->cmd) return;
    h->side = side; h->key = key; h->ch = ch;
    XPLMRegisterCommandHandler(h->cmd, key_command, 1, h);
    ++hook_count;
}

static void hook_keys(void)
{
    int side, i;
    char suffix[32];
    for (side = 0; side < 2; ++side) {
        for (i = 0; i < (int)(sizeof(acd_keys) / sizeof(acd_keys[0])); ++i) hook(side, acd_keys[i].suffix, acd_keys[i].key, acd_keys[i].ch);
        for (i = 0; i < 26; ++i) { snprintf(suffix, sizeof(suffix), "kbd/%c", 'a' + i); hook(side, suffix, OIT_K_CHAR, (char)('A' + i)); }
        for (i = 0; i < 10; ++i) { snprintf(suffix, sizeof(suffix), "kbd/%d", i); hook(side, suffix, OIT_K_CHAR, (char)('0' + i)); }
        for (i = 0; i < 12; ++i) { snprintf(suffix, sizeof(suffix), "kbd/f%d", i + 1); hook(side, suffix, OIT_K_F1 + i, 0); }
    }
}

static void unhook_keys(void)
{
    int i;
    for (i = 0; i < hook_count; ++i) XPLMUnregisterCommandHandler(hooks[i].cmd, key_command, 1, &hooks[i]);
    hook_count = 0;
}

/* Pop-out windows: a380x/xp12/oit/<side>/popup (also usable for a second monitor). */
static XPLMCommandRef popup_cmd[2];
static int popup_command(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
    Screen *s = &screens[(int)(intptr_t)refcon];
    (void)cmd;
    if (phase == xplm_CommandBegin && s->dev) XPLMSetAvionicsPopupVisible(s->dev, !XPLMIsAvionicsPopupVisible(s->dev));
    return 0;
}

/* ---------------------------------------------------------------- loop */
static float flight_loop(float since_call, float since_loop, int counter, void *refcon)
{
    int k;
    (void)since_loop; (void)counter; (void)refcon;
    snapshot();
    update_airports();
    for (k = 0; k < 2; ++k) {
        Screen *s = &screens[k];
        if (!s->dev || !s->term) continue;
        s->view = view_for(k);
        oit_tick(s->term, &s->view, since_call);
        if (oit_take_dirty(s->term) || floor(sim.zulu_s) != s->last_second) {
            s->last_second = floor(sim.zulu_s);
            XPLMAvionicsNeedsDrawing(s->dev);
        }
    }
    return 0.1f;
}

PLUGIN_API int XPluginStart(char *name, char *sig, char *desc)
{
    int k;
    strcpy(name, "A380X OIT");
    strcpy(sig, "org.a380x.xp12.oit");
    strcpy(desc, "A380 Onboard Information Terminals (captain and F/O side-console displays).");
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
    for (k = 0; k < 2; ++k) {
        char cmd[96];
        snprintf(cmd, sizeof(cmd), "a380x/xp12/oit/%s/popup", side_name[k]);
        popup_cmd[k] = XPLMCreateCommand(cmd, k == OIT_CAPT ? "Show/hide the captain OIT window" : "Show/hide the F/O OIT window");
        XPLMRegisterCommandHandler(popup_cmd[k], popup_command, 1, (void *)(intptr_t)k);
    }
    XPLMDebugString("A380X OIT: captain and F/O OIT devices (a380x.xp12.oit.captain / .first_officer) ready\n");
    return 1;
}

PLUGIN_API int XPluginEnable(void)
{
    create_devices();
    hook_keys();
    XPLMRegisterFlightLoopCallback(flight_loop, -1.0f, NULL);
    return 1;
}

PLUGIN_API void XPluginDisable(void)
{
    XPLMUnregisterFlightLoopCallback(flight_loop, NULL);
    unhook_keys();
    destroy_devices();
    oit_gl_release();
}

PLUGIN_API void XPluginStop(void)
{
    int k;
    for (k = 0; k < 2; ++k) if (popup_cmd[k]) XPLMUnregisterCommandHandler(popup_cmd[k], popup_command, 1, (void *)(intptr_t)k);
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID from, int msg, void *param)
{
    int k;
    (void)from; (void)param;
    if (msg == XPLM_MSG_PLANE_LOADED) {
        for (k = 0; k < 2; ++k) if (screens[k].term) oit_reset(screens[k].term);
        for (k = 0; k < R_COUNT; ++k) refs[k].ref = NULL;   /* the A380 datarefs may appear after load */
        laptop_auto[0] = laptop_auto[1] = -1; knob_auto[0] = knob_auto[1] = -1; apt_at = -100.0;
    }
}
