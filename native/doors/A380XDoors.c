/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380X door timing: X-Plane animates every door (sim/flightmodel2/misc/door_open_ratio) toward its
 * switch over sim/flightmodel2/misc/door_cycle_time seconds, the way Laminar's own A330 sets it at
 * aircraft start. The A380X never set it, so its doors jumped open/closed. This aircraft-local plugin
 * writes the cycle times from door_timing.ini (next to this plugin's platform folders) at load and
 * re-applies them every 2 s if anything else resets them. It does nothing else.
 *
 * Door indices (from the aircraft objects): 0-9 main deck (M1R, M1L ... M5R, M5L),
 * 10-15 upper deck (U1R, U1L ... U3R, U3L), 16-19 cargo / bulk doors.
 *
 * Cabin window shades and overhead bins (2026-11-12): the cabin objects animate each shade / bin door with
 * a380x/xp12/cabin/shade_ratio[i] / bin_ratio[j] (0 up / closed .. 1 down / open); the invisible click spots in the
 * cockpit object toggle a380x/xp12/cabin/shade[i] / bin[j] (0 or 1); this plugin moves each ratio toward its
 * target (shades 1.5 s, bins 1.2 s). Commands a380x/xp12/cabin/{shades_close_all, shades_open_all, bins_open_all,
 * bins_close_all}. See Output/A380X-XP12 backups/cabin-bins-shades.
 * 2026-11-14 (cabin v2): 199 shades on all decks (economy, first class, upper deck), so the arrays hold 256. See
 * Output/A380X-XP12 backups/cabin-v2-20261114.
 *
 * Spoiler and aileron visuals (2026-11-14): the wing objects animate the 8 spoiler panels per wing from
 * a380x/xp12/surfaces/spoiler_deg[16] (left 0-7 = panels 1-8 inboard to outboard, right 8-15) and add
 * a380x/xp12/surfaces/aileron_ground_up (0..1 -> ailerons up 40 deg, the ACF's ground speedbrake aileron deflection)
 * on top of the existing aileron animation. Each panel shows the A380 PRIM order of the avionics speedbrake module
 * (a380x/xp12/speedbrake/spoiler_N_deg: speed brake, ground spoilers, 5 deg spoiler lift on the ground) or, on
 * panels 2-8, the roll spoiler deflection of its wing, whichever is larger; the ailerons rise with the ground spoiler
 * command (lift dumping). Surfaces move only with hydraulic pressure (otherwise they hold), rate limited.
 * See Output/A380X-XP12 backups/spoilers-ailerons-v2.
 */
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

#define DOORS 20
#define SHADES 256
#define BINS 160
#define SHADE_TRAVEL_S 1.5f
#define BIN_TRAVEL_S 1.2f
static XPLMDataRef cycle_ref;
static float shade_target[SHADES], shade_ratio[SHADES], bin_target[BINS], bin_ratio[BINS];
static XPLMDataRef cabin_refs[4];
/* ---- spoiler / aileron visuals ---- */
#define SPOILER_RATE_DPS 50.0f
#define AIL_UP_RATE 1.0f
static float spoiler_deg[16], ail_up;
static XPLMDataRef surf_refs[2];
static XPLMDataRef in_order[8], in_gs_cmd, in_xp_spoiler2, in_xp_sb_ratio, in_xp_sb1, in_ground, in_hyd[3], in_paused;
static int hyd_ok = 1;
static double inputs_search_t = -10.0;
static XPLMCommandRef cabin_cmds[4];
static float door_timer;
static float wanted[DOORS];
static int configured[DOORS];

static void set_range(int first, int last, float seconds)
{
    int i;
    for (i = first; i <= last && i < DOORS; ++i) { wanted[i] = seconds; configured[i] = 1; }
}

static void defaults(void)
{
    memset(configured, 0, sizeof(configured));
    set_range(0, 9, 8.0f);      /* main deck passenger doors */
    set_range(10, 15, 8.0f);    /* upper deck passenger doors */
    set_range(16, 19, 20.0f);   /* cargo / bulk doors */
}

/* door_timing.ini: "main_deck = 8", "upper_deck = 8", "cargo = 20", "door_N = seconds"; # comments. */
static void load_ini(void)
{
    char path[1024], line[256];
    FILE *f;
    char *cut;
    int k;
    path[0] = 0;
    XPLMGetPluginInfo(XPLMGetMyID(), NULL, path, NULL, NULL);
    for (k = 0; k < 2; ++k) {          /* .../A380XDoors/<platform>/A380XDoors.xpl -> .../A380XDoors */
        cut = strrchr(path, '/');
        if (!cut) cut = strrchr(path, '\\');
        if (!cut) return;
        *cut = 0;
    }
    strncat(path, "/door_timing.ini", sizeof(path) - strlen(path) - 1);
    f = fopen(path, "r");
    if (!f) { XPLMDebugString("A380X doors: door_timing.ini not found, using 8 s passenger / 20 s cargo\n"); return; }
    while (fgets(line, sizeof(line), f)) {
        char key[64];
        float seconds;
        char *hash = strchr(line, '#');
        if (hash) *hash = 0;
        if (sscanf(line, " %63[^= \t] = %f", key, &seconds) != 2) continue;
        for (k = 0; key[k]; ++k) key[k] = (char)tolower((unsigned char)key[k]);
        if (!(seconds >= 0.0f && seconds <= 600.0f)) continue;
        if (!strcmp(key, "main_deck")) set_range(0, 9, seconds);
        else if (!strcmp(key, "upper_deck")) set_range(10, 15, seconds);
        else if (!strcmp(key, "cargo")) set_range(16, 19, seconds);
        else if (!strncmp(key, "door_", 5)) {
            int n = atoi(key + 5);
            if (n >= 0 && n < DOORS) set_range(n, n, seconds);
        }
    }
    fclose(f);
}

static void apply(void)
{
    float now[DOORS];
    int i, n, change = 0;
    if (!cycle_ref) return;
    n = XPLMGetDatavf(cycle_ref, now, 0, DOORS);
    if (n > DOORS) n = DOORS;
    for (i = 0; i < n; ++i)
        if (configured[i] && now[i] != wanted[i]) { now[i] = wanted[i]; change = 1; }
    if (change) XPLMSetDatavf(cycle_ref, now, 0, n);
}

/* ---- cabin shades and bins: float array datarefs, targets writable (click spots), ratios animated ---- */
static int get_array(void *refcon, float *out, int offset, int count)
{
    float *a; int n;
    switch ((int)(intptr_t)refcon) {
    case 0: a = shade_target; n = SHADES; break;
    case 1: a = shade_ratio; n = SHADES; break;
    case 2: a = bin_target; n = BINS; break;
    default: a = bin_ratio; n = BINS; break;
    }
    if (!out) return n;
    if (offset < 0 || offset >= n || count <= 0) return 0;
    if (count > n - offset) count = n - offset;
    memcpy(out, a + offset, (size_t)count * sizeof(float));
    return count;
}
static void set_array(void *refcon, float *in, int offset, int count)
{
    float *a; int n, i;
    int k = (int)(intptr_t)refcon;
    if (k != 0 && k != 2) return;          /* ratios are read-only */
    a = k == 0 ? shade_target : bin_target; n = k == 0 ? SHADES : BINS;
    if (!in || offset < 0 || offset >= n) return;
    if (count > n - offset) count = n - offset;
    for (i = 0; i < count; ++i) a[offset + i] = in[i] >= 0.5f ? 1.0f : 0.0f;
}
static int cabin_command(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
    int k = (int)(intptr_t)refcon, i;
    (void)cmd;
    if (phase != xplm_CommandBegin) return 0;
    if (k < 2) for (i = 0; i < SHADES; ++i) shade_target[i] = k == 0 ? 1.0f : 0.0f;
    else for (i = 0; i < BINS; ++i) bin_target[i] = k == 2 ? 1.0f : 0.0f;
    return 0;
}
static void step(float *ratio, const float *target, int n, float rate)
{
    int i;
    for (i = 0; i < n; ++i) {
        float d = target[i] - ratio[i];
        if (d > rate) d = rate; else if (d < -rate) d = -rate;
        ratio[i] += d;
    }
}
static int get_spoiler_deg(void *refcon, float *out, int offset, int count)
{
    (void)refcon;
    if (!out) return 16;
    if (offset < 0 || offset >= 16 || count <= 0) return 0;
    if (count > 16 - offset) count = 16 - offset;
    memcpy(out, spoiler_deg + offset, (size_t)count * sizeof(float));
    return count;
}
static float get_ail_up(void *refcon) { (void)refcon; return ail_up; }

static void find_surface_inputs(void)
{
    char name[96];
    int k;
    for (k = 0; k < 8; ++k) if (!in_order[k]) { snprintf(name, sizeof(name), "a380x/xp12/speedbrake/spoiler_%d_deg", k + 1); in_order[k] = XPLMFindDataRef(name); }
    if (!in_gs_cmd) in_gs_cmd = XPLMFindDataRef("a380x/xp12/speedbrake/ground_spoiler_command_deg");
    if (!in_xp_spoiler2) in_xp_spoiler2 = XPLMFindDataRef("sim/flightmodel2/wing/spoiler2_deg");
    if (!in_xp_sb_ratio) in_xp_sb_ratio = XPLMFindDataRef("sim/flightmodel2/controls/speedbrake_ratio");
    if (!in_xp_sb1) in_xp_sb1 = XPLMFindDataRef("sim/flightmodel2/wing/speedbrake1_deg");
    if (!in_ground) in_ground = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    if (!in_paused) in_paused = XPLMFindDataRef("sim/time/paused");
    for (k = 0; k < 3; ++k) if (!in_hyd[k]) { snprintf(name, sizeof(name), "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_%d", k + 1); in_hyd[k] = XPLMFindDataRef(name); }
}

static float rate(float cur, float target, float max_step)
{
    float d = target - cur;
    if (d > max_step) d = max_step; else if (d < -max_step) d = -max_step;
    return cur + d;
}

/* One visual surface step: PRIM panel orders (or X-Plane's speedbrake deflection), roll spoilers, ground aileron
 * up, hydraulics, rate limits. */
static void surfaces_step(float dt)
{
    float order[8], xp_sp2[6] = {0}, roll[2] = {0, 0}, sb_ratio = 0.0f, gs = -1.0f, p = 0.0f, ail_target;
    int k, side, ground = 0, have_orders = 1;
    if (in_paused && XPLMGetDatai(in_paused)) return;
    if (XPLMGetElapsedTime() - inputs_search_t > 2.0) { find_surface_inputs(); inputs_search_t = XPLMGetElapsedTime(); }
    for (k = 0; k < 8; ++k) if (!in_order[k]) have_orders = 0;
    if (in_ground) ground = XPLMGetDatai(in_ground) != 0;
    if (in_xp_sb_ratio) sb_ratio = XPLMGetDataf(in_xp_sb_ratio);
    if (have_orders) for (k = 0; k < 8; ++k) order[k] = XPLMGetDataf(in_order[k]);
    else {
        float sb1 = 0.0f;
        if (in_xp_sb1) XPLMGetDatavf(in_xp_sb1, &sb1, 0, 1);
        for (k = 0; k < 8; ++k) order[k] = fabsf(sb1);
    }
    /* roll spoilers: X-Plane's wing 4/5 roll spoiler deflection minus its speed brake share (ACF spoiler 2 with
     * speed brake: 25 deg in flight, 50 on the ground) */
    if (in_xp_spoiler2) XPLMGetDatavf(in_xp_spoiler2, xp_sp2, 0, 6);
    for (side = 0; side < 2; ++side) {
        float share = (ground ? 50.0f : 25.0f) * (sb_ratio > 0.0f ? sb_ratio : 0.0f);
        roll[side] = fabsf(xp_sp2[4 + side]) - share;   /* magnitude: X-Plane's sign convention for raised spoilers */
        if (!(roll[side] > 0.0f)) roll[side] = 0.0f;
    }
    for (k = 0; k < 3; ++k) if (in_hyd[k]) { float v = XPLMGetDataf(in_hyd[k]); if (v > p) p = v; }
    if (!in_hyd[0]) p = 3000.0f;
    if (p >= 900.0f) hyd_ok = 1; else if (p <= 450.0f) hyd_ok = 0;
    if (!hyd_ok) return;     /* no hydraulic power: every surface holds its position */
    for (side = 0; side < 2; ++side)
        for (k = 0; k < 8; ++k) {
            float target = order[k];
            if (k >= 1 && roll[side] > target) target = roll[side];
            if (target < 0.0f) target = 0.0f;
            spoiler_deg[side * 8 + k] = rate(spoiler_deg[side * 8 + k], target, SPOILER_RATE_DPS * dt);
        }
    if (in_gs_cmd) gs = XPLMGetDataf(in_gs_cmd);
    if (gs >= 0.0f) ail_target = gs / 50.0f;
    else ail_target = ground ? sb_ratio : 0.0f;       /* no avionics: X-Plane's ground speed brake */
    if (ail_target < 0.0f) ail_target = 0.0f; else if (ail_target > 1.0f) ail_target = 1.0f;
    ail_up = rate(ail_up, ail_target, AIL_UP_RATE * dt);
}

static void cabin_reset(void)
{
    memset(shade_target, 0, sizeof(shade_target)); memset(shade_ratio, 0, sizeof(shade_ratio));
    memset(bin_target, 0, sizeof(bin_target)); memset(bin_ratio, 0, sizeof(bin_ratio));
}
static void cabin_start(void)
{
    static const char *const names[4] = {"a380x/xp12/cabin/shade", "a380x/xp12/cabin/shade_ratio", "a380x/xp12/cabin/bin",
                                         "a380x/xp12/cabin/bin_ratio"};
    static const char *const cmd_names[4][2] = {
        {"a380x/xp12/cabin/shades_close_all", "Cabin: lower all window shades"},
        {"a380x/xp12/cabin/shades_open_all", "Cabin: raise all window shades"},
        {"a380x/xp12/cabin/bins_open_all", "Cabin: open all overhead bins"},
        {"a380x/xp12/cabin/bins_close_all", "Cabin: close all overhead bins"}};
    int k;
    for (k = 0; k < 4; ++k) {
        cabin_refs[k] = XPLMRegisterDataAccessor(names[k], xplmType_FloatArray, k == 0 || k == 2, NULL, NULL, NULL, NULL,
                                                 NULL, NULL, NULL, NULL, get_array, k == 0 || k == 2 ? set_array : NULL,
                                                 NULL, NULL, (void *)(intptr_t)k, (void *)(intptr_t)k);
        cabin_cmds[k] = XPLMCreateCommand(cmd_names[k][0], cmd_names[k][1]);
        XPLMRegisterCommandHandler(cabin_cmds[k], cabin_command, 1, (void *)(intptr_t)k);
    }
}
static void cabin_stop(void)
{
    int k;
    for (k = 0; k < 4; ++k) {
        if (cabin_refs[k]) XPLMUnregisterDataAccessor(cabin_refs[k]);
        if (cabin_cmds[k]) XPLMUnregisterCommandHandler(cabin_cmds[k], cabin_command, 1, (void *)(intptr_t)k);
        cabin_refs[k] = NULL; cabin_cmds[k] = NULL;
    }
}

static float flight_loop(float since_call, float since_loop, int counter, void *refcon)
{
    float dt = since_call;
    (void)since_loop; (void)counter; (void)refcon;
    if (!(dt > 0.0f) || dt > 0.5f) dt = 0.0f;
    step(shade_ratio, shade_target, SHADES, dt / SHADE_TRAVEL_S);
    step(bin_ratio, bin_target, BINS, dt / BIN_TRAVEL_S);
    if (dt > 0.0f) surfaces_step(dt);
    door_timer -= since_call;
    if (door_timer <= 0.0f) { apply(); door_timer = 2.0f; }
    return -1.0f;   /* every frame: smooth shade / bin motion */
}

PLUGIN_API int XPluginStart(char *name, char *signature, char *description)
{
    char line[160];
    strcpy(name, "A380X Door Timing");
    strcpy(signature, "org.a380x.xp12.door-timing");
    strcpy(description, "Door cycle times (door_timing.ini), cabin window shades and overhead bins.");
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
    cycle_ref = XPLMFindDataRef("sim/flightmodel2/misc/door_cycle_time");
    if (!cycle_ref) { XPLMDebugString("A380X doors: sim/flightmodel2/misc/door_cycle_time not found\n"); return 0; }
    defaults();
    load_ini();
    snprintf(line, sizeof(line), "A380X doors: cycle times main deck %.1f s, upper deck %.1f s, cargo %.1f s\n",
             wanted[0], wanted[10], wanted[16]);
    XPLMDebugString(line);
    cabin_start();
    surf_refs[0] = XPLMRegisterDataAccessor("a380x/xp12/surfaces/spoiler_deg", xplmType_FloatArray, 0, NULL, NULL, NULL, NULL,
                                            NULL, NULL, NULL, NULL, get_spoiler_deg, NULL, NULL, NULL, NULL, NULL);
    surf_refs[1] = XPLMRegisterDataAccessor("a380x/xp12/surfaces/aileron_ground_up", xplmType_Float, 0, NULL, NULL, get_ail_up,
                                            NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    XPLMDebugString("A380X surfaces: spoiler panels (a380x/xp12/surfaces/spoiler_deg[16]) and ground aileron up ready\n");
    XPLMDebugString("A380X cabin: window shades (a380x/xp12/cabin/shade[]) and overhead bins (bin[]) ready\n");
    XPLMRegisterFlightLoopCallback(flight_loop, -1.0f, NULL);
    return 1;
}

PLUGIN_API void XPluginStop(void)
{
    int k;
    XPLMUnregisterFlightLoopCallback(flight_loop, NULL);
    cabin_stop();
    for (k = 0; k < 2; ++k) if (surf_refs[k]) { XPLMUnregisterDataAccessor(surf_refs[k]); surf_refs[k] = NULL; }
}
PLUGIN_API int XPluginEnable(void) { apply(); return 1; }
PLUGIN_API void XPluginDisable(void) {}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID from, int message, void *param)
{
    (void)from; (void)param;
    if (message == XPLM_MSG_PLANE_LOADED) { apply(); cabin_reset(); }
}
