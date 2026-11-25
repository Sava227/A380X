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
 *
 * Escape slides (2026-11-15): the slides object (objects/exterior/a380x_slides.obj) unfolds and inflates each
 * passenger door's slide with a380x/xp12/doors/slide_ratio[16] (0 stowed .. 1 deployed; same order as
 * door_open_ratio 0-15). A slide deploys when its door opens while the slide was armed: the arming state is the
 * avionics/EFB one (a380x/xp12/efb/slide_armed[16], flyPad, a380x/xp12/efb/slides_arm_all), latched while the door is
 * shut, because the avionics disarms a slide once its door opens. Deployment takes SLIDE_INFLATE_S; closing the door
 * again (or a380x/xp12/doors/slides_stow_all) stows it. See Output/A380X-XP12 backups/slides-20261115.
 *
 * Landing gear steering visuals (2026-11-24): the gear objects turn the nose gear's lower strut and wheels and the
 * body gears' aft axles with a380x/xp12/gear/steer_vis_deg[5] (0 nose, 1/2 wing gears (always 0), 3/4 body gears;
 * degrees, + right). It is X-Plane's enacted steering (sim/flightmodel2/gear/tire_steer_actual_deg) while that gear is
 * on the ground; a gear that leaves the ground centres its wheels over STEER_CENTRE_S, as the A380's nose and body
 * wheel steering centre in flight, so wheels never turn in the air or in the gear bay. The wheels spin with
 * sim/flightmodel2/gear/tire_rotation_angle_deg directly. See Output/A380X-XP12 backups/wheels-20261124.
 *
 * Main gear bogie tilt (2026-11-24): the wing and body gear bogies pitch about their pivot pins with
 * a380x/xp12/gear/bogie_tilt_deg[5] (degrees relative to the airframe, + = rear wheels down; nose gear 0). In flight
 * with the gear down a bogie hangs at BOGIE_TILT_MAX_DEG (reached at BOGIE_TILT_RATE_DPS after lift-off); near the
 * ground it is limited so its rear wheels never go below the runway, from that gear's height (y_agl, attitude and
 * X-Plane's wheel position, calibrated on the ground against the strut deflection): on approach the rear wheels
 * touch first and the bogie levels as the aircraft settles, and on the ground it follows the runway (the bogie
 * stays flat when the aircraft rotates). It is faded out over the last 15 % of gear travel, so the gear stows as
 * modelled. a380x/xp12/gear/height_m[5] shows the computed wheel heights (diagnostics).
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
/* ---- escape slides ---- */
#define SLIDES 16
#define SLIDE_INFLATE_S 6.0f
#define SLIDE_STOW_S 3.0f
#define SLIDE_DOOR_SHUT 0.005f     /* below this a door counts as shut: the arming state is latched */
#define SLIDE_START_MAIN 0.25f     /* door ratio at which the slide pack leaves (after the handle phase) */
#define SLIDE_START_UPPER 0.35f
static float slide_ratio[SLIDES];
static int slide_latched[SLIDES], slide_out[SLIDES];
static XPLMDataRef slide_ref, slide_armed_in, door_ratio_in;
static XPLMCommandRef slide_stow_cmd;
static double slide_search_t = -10.0;
/* ---- landing gear steering visuals ---- */
#define GEAR_N 5
#define STEER_CENTRE_S 1.0f        /* a gear that leaves the ground centres its wheels over this time */
#define STEER_LIMIT_DEG 90.0f
static float steer_vis[GEAR_N], steer_gate[GEAR_N];
static XPLMDataRef steer_ref, steer_act_in, gear_ground_in;
static double gear_search_t = -10.0;
/* ---- main gear bogie tilt ---- */
#define BOGIE_TILT_MAX_DEG 10.0f   /* in flight, gear down: rear wheels down */
#define BOGIE_TILT_MIN_DEG -15.0f  /* on the ground with the nose up the bogie follows the runway */
#define BOGIE_TILT_RATE_DPS 8.0f   /* pitch trimmer after lift-off */
#define BOGIE_GATE_FROM 0.85f      /* deploy ratio below which the bogie is in its modelled stowing pose */
#define GEAR_BIAS_TAU_S 1.0f
static const float bogie_arm_m[GEAR_N] = {0.0f, 0.867f, 0.867f, 1.66f, 1.66f};   /* pivot -> rear axle */
static float bogie_tilt[GEAR_N], bogie_vis[GEAR_N], gear_height[GEAR_N], gear_bias[GEAR_N];
static int gear_bias_ok[GEAR_N];
static XPLMDataRef tilt_ref, height_ref, tire_in[3], agl_in, theta_in, phi_in, deploy_in, defl_in;
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

static int get_slide_ratio(void *refcon, float *out, int offset, int count)
{
    (void)refcon;
    if (!out) return SLIDES;
    if (offset < 0 || offset >= SLIDES || count <= 0) return 0;
    if (count > SLIDES - offset) count = SLIDES - offset;
    memcpy(out, slide_ratio + offset, (size_t)count * sizeof(float));
    return count;
}
static int slide_stow_command(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
    int k;
    (void)cmd; (void)refcon;
    if (phase == xplm_CommandBegin) for (k = 0; k < SLIDES; ++k) slide_out[k] = 0;
    return 0;
}
static void slides_reset(void)
{
    memset(slide_ratio, 0, sizeof(slide_ratio)); memset(slide_latched, 0, sizeof(slide_latched));
    memset(slide_out, 0, sizeof(slide_out));
}
static void slides_step(float dt)
{
    float door[SLIDES];
    int armed[SLIDES], k, have_armed = 0;
    if (XPLMGetElapsedTime() - slide_search_t > 2.0) {
        if (!slide_armed_in) slide_armed_in = XPLMFindDataRef("a380x/xp12/efb/slide_armed");
        if (!door_ratio_in) door_ratio_in = XPLMFindDataRef("sim/flightmodel2/misc/door_open_ratio");
        slide_search_t = XPLMGetElapsedTime();
    }
    if (!door_ratio_in || XPLMGetDatavf(door_ratio_in, door, 0, SLIDES) < SLIDES) return;
    memset(armed, 0, sizeof(armed));
    if (slide_armed_in && XPLMGetDatavi(slide_armed_in, armed, 0, SLIDES) >= SLIDES) have_armed = 1;
    for (k = 0; k < SLIDES; ++k) {
        float start = k >= 10 ? SLIDE_START_UPPER : SLIDE_START_MAIN, target;
        if (!(door[k] > SLIDE_DOOR_SHUT)) {
            if (have_armed) slide_latched[k] = armed[k] != 0;
            slide_out[k] = 0;                         /* door shut: the slide is (re)packed */
        } else if (slide_latched[k] && door[k] >= start) {
            char line[96];
            slide_latched[k] = 0;
            slide_out[k] = 1;
            snprintf(line, sizeof(line), "A380X slides: door %d opened with its slide armed - slide deploying\n", k);
            XPLMDebugString(line);
        }
        target = slide_out[k] ? 1.0f : 0.0f;
        slide_ratio[k] = rate(slide_ratio[k], target, dt / (slide_out[k] ? SLIDE_INFLATE_S : SLIDE_STOW_S));
    }
}

static int get_steer_vis(void *refcon, float *out, int offset, int count)
{
    (void)refcon;
    if (!out) return GEAR_N;
    if (offset < 0 || offset >= GEAR_N || count <= 0) return 0;
    if (count > GEAR_N - offset) count = GEAR_N - offset;
    memcpy(out, steer_vis + offset, (size_t)count * sizeof(float));
    return count;
}
static int get_bogie_tilt(void *refcon, float *out, int offset, int count)
{
    const float *src = refcon ? gear_height : bogie_vis;
    if (!out) return GEAR_N;
    if (offset < 0 || offset >= GEAR_N || count <= 0) return 0;
    if (count > GEAR_N - offset) count = GEAR_N - offset;
    memcpy(out, src + offset, (size_t)count * sizeof(float));
    return count;
}
static void gear_reset(void)
{
    memset(steer_vis, 0, sizeof(steer_vis)); memset(steer_gate, 0, sizeof(steer_gate));
    memset(bogie_tilt, 0, sizeof(bogie_tilt)); memset(bogie_vis, 0, sizeof(bogie_vis));
    memset(gear_height, 0, sizeof(gear_height)); memset(gear_bias, 0, sizeof(gear_bias));
    memset(gear_bias_ok, 0, sizeof(gear_bias_ok));
}
static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
/* Bogie tilt of the main gears from each wheel's height above the ground (see the header). */
static void bogie_step(float dt, const int *ground)
{
    float tx[GEAR_N], ty[GEAR_N], tz[GEAR_N], dep[GEAR_N], defl[GEAR_N], agl, th, ph;
    int k;
    if (!tire_in[0] || !tire_in[1] || !tire_in[2] || !agl_in || !theta_in || !phi_in || !deploy_in) return;
    memset(defl, 0, sizeof(defl));
    if (XPLMGetDatavf(tire_in[0], tx, 0, GEAR_N) < GEAR_N || XPLMGetDatavf(tire_in[1], ty, 0, GEAR_N) < GEAR_N ||
        XPLMGetDatavf(tire_in[2], tz, 0, GEAR_N) < GEAR_N || XPLMGetDatavf(deploy_in, dep, 0, GEAR_N) < GEAR_N) return;
    if (defl_in) XPLMGetDatavf(defl_in, defl, 0, GEAR_N);
    agl = XPLMGetDataf(agl_in); th = XPLMGetDataf(theta_in); ph = XPLMGetDataf(phi_in);
    if (!isfinite(agl) || !isfinite(th) || !isfinite(ph)) return;
    for (k = 1; k < GEAR_N; ++k) {
        float tr = th * 0.0174533f, pr = ph * 0.0174533f, h_raw, h, allowed, cmd, gate;
        /* airplane coordinates (x right, y up, z aft) -> height: roll about z, then pitch about x */
        h_raw = agl + (-tx[k] * sinf(pr) + ty[k] * cosf(pr)) * cosf(tr) - tz[k] * sinf(tr);
        if (!isfinite(h_raw)) continue;
        if (ground[k]) {
            float b = h_raw + (isfinite(defl[k]) ? defl[k] : 0.0f);
            if (!gear_bias_ok[k]) { gear_bias[k] = b; gear_bias_ok[k] = 1; }
            else gear_bias[k] += (b - gear_bias[k]) * clampf(dt / GEAR_BIAS_TAU_S, 0.0f, 1.0f);
            h = 0.0f;
        } else {
            h = h_raw - gear_bias[k];
        }
        gear_height[k] = h;
        /* bogie angle to the ground that keeps the rear wheels at or above it */
        allowed = h <= 0.0f ? 0.0f : asinf(clampf(h / bogie_arm_m[k], 0.0f, 1.0f)) * 57.29578f;
        cmd = clampf(allowed - th, BOGIE_TILT_MIN_DEG, BOGIE_TILT_MAX_DEG);
        if (cmd > bogie_tilt[k]) bogie_tilt[k] = fminf(cmd, bogie_tilt[k] + BOGIE_TILT_RATE_DPS * dt);
        else bogie_tilt[k] = cmd;                                   /* the ground pushes it level at once */
        gate = isfinite(dep[k]) ? clampf((dep[k] - BOGIE_GATE_FROM) / (1.0f - BOGIE_GATE_FROM), 0.0f, 1.0f) : 0.0f;
        bogie_vis[k] = bogie_tilt[k] * gate;
    }
}
/* Visual steering: X-Plane's enacted steering on the ground, centred once a gear is airborne. */
static void gear_step(float dt)
{
    float act[GEAR_N];
    int ground[GEAR_N], k;
    if (XPLMGetElapsedTime() - gear_search_t > 2.0) {
        if (!steer_act_in) steer_act_in = XPLMFindDataRef("sim/flightmodel2/gear/tire_steer_actual_deg");
        if (!gear_ground_in) gear_ground_in = XPLMFindDataRef("sim/flightmodel2/gear/on_ground");
        if (!tire_in[0]) tire_in[0] = XPLMFindDataRef("sim/flightmodel/parts/tire_x_no_deflection");
        if (!tire_in[1]) tire_in[1] = XPLMFindDataRef("sim/flightmodel/parts/tire_y_no_deflection");
        if (!tire_in[2]) tire_in[2] = XPLMFindDataRef("sim/flightmodel/parts/tire_z_no_deflection");
        if (!agl_in) agl_in = XPLMFindDataRef("sim/flightmodel/position/y_agl");
        if (!theta_in) theta_in = XPLMFindDataRef("sim/flightmodel/position/theta");
        if (!phi_in) phi_in = XPLMFindDataRef("sim/flightmodel/position/phi");
        if (!deploy_in) deploy_in = XPLMFindDataRef("sim/flightmodel2/gear/deploy_ratio");
        if (!defl_in) defl_in = XPLMFindDataRef("sim/flightmodel2/gear/tire_vertical_deflection_mtr");
        gear_search_t = XPLMGetElapsedTime();
    }
    if (!steer_act_in || !gear_ground_in) return;
    memset(act, 0, sizeof(act)); memset(ground, 0, sizeof(ground));
    if (XPLMGetDatavf(steer_act_in, act, 0, GEAR_N) < GEAR_N) return;
    if (XPLMGetDatavi(gear_ground_in, ground, 0, GEAR_N) < GEAR_N) return;
    for (k = 0; k < GEAR_N; ++k) {
        float a = isfinite(act[k]) ? act[k] : 0.0f;
        if (a > STEER_LIMIT_DEG) a = STEER_LIMIT_DEG; else if (a < -STEER_LIMIT_DEG) a = -STEER_LIMIT_DEG;
        steer_gate[k] = rate(steer_gate[k], ground[k] ? 1.0f : 0.0f, dt / STEER_CENTRE_S);
        steer_vis[k] = a * steer_gate[k];
    }
    bogie_step(dt, ground);
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
    if (dt > 0.0f) slides_step(dt);
    if (dt > 0.0f) gear_step(dt);
    door_timer -= since_call;
    if (door_timer <= 0.0f) { apply(); door_timer = 2.0f; }
    return -1.0f;   /* every frame: smooth shade / bin motion */
}

PLUGIN_API int XPluginStart(char *name, char *signature, char *description)
{
    char line[160];
    strcpy(name, "A380X Door Timing");
    strcpy(signature, "org.a380x.xp12.door-timing");
    strcpy(description, "Door cycle times (door_timing.ini), cabin shades and bins, surface visuals, escape slides.");
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
    slide_ref = XPLMRegisterDataAccessor("a380x/xp12/doors/slide_ratio", xplmType_FloatArray, 0, NULL, NULL, NULL, NULL,
                                         NULL, NULL, NULL, NULL, get_slide_ratio, NULL, NULL, NULL, NULL, NULL);
    slide_stow_cmd = XPLMCreateCommand("a380x/xp12/doors/slides_stow_all", "Escape slides: stow all deployed slides");
    XPLMRegisterCommandHandler(slide_stow_cmd, slide_stow_command, 1, NULL);
    XPLMDebugString("A380X slides: escape slides (a380x/xp12/doors/slide_ratio[16]) deploy when an armed door opens\n");
    steer_ref = XPLMRegisterDataAccessor("a380x/xp12/gear/steer_vis_deg", xplmType_FloatArray, 0, NULL, NULL, NULL, NULL,
                                         NULL, NULL, NULL, NULL, get_steer_vis, NULL, NULL, NULL, NULL, NULL);
    XPLMDebugString("A380X gear: wheel steering visuals (a380x/xp12/gear/steer_vis_deg[5]) ready\n");
    tilt_ref = XPLMRegisterDataAccessor("a380x/xp12/gear/bogie_tilt_deg", xplmType_FloatArray, 0, NULL, NULL, NULL, NULL,
                                        NULL, NULL, NULL, NULL, get_bogie_tilt, NULL, NULL, NULL, NULL, NULL);
    height_ref = XPLMRegisterDataAccessor("a380x/xp12/gear/height_m", xplmType_FloatArray, 0, NULL, NULL, NULL, NULL,
                                          NULL, NULL, NULL, NULL, get_bogie_tilt, NULL, NULL, NULL, (void *)1, NULL);
    XPLMDebugString("A380X gear: main gear bogie tilt (a380x/xp12/gear/bogie_tilt_deg[5], height_m[5]) ready\n");
    XPLMRegisterFlightLoopCallback(flight_loop, -1.0f, NULL);
    return 1;
}

PLUGIN_API void XPluginStop(void)
{
    int k;
    XPLMUnregisterFlightLoopCallback(flight_loop, NULL);
    cabin_stop();
    for (k = 0; k < 2; ++k) if (surf_refs[k]) { XPLMUnregisterDataAccessor(surf_refs[k]); surf_refs[k] = NULL; }
    if (slide_ref) { XPLMUnregisterDataAccessor(slide_ref); slide_ref = NULL; }
    if (steer_ref) { XPLMUnregisterDataAccessor(steer_ref); steer_ref = NULL; }
    if (tilt_ref) { XPLMUnregisterDataAccessor(tilt_ref); tilt_ref = NULL; }
    if (height_ref) { XPLMUnregisterDataAccessor(height_ref); height_ref = NULL; }
    if (slide_stow_cmd) { XPLMUnregisterCommandHandler(slide_stow_cmd, slide_stow_command, 1, NULL); slide_stow_cmd = NULL; }
}
PLUGIN_API int XPluginEnable(void) { apply(); return 1; }
PLUGIN_API void XPluginDisable(void) {}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID from, int message, void *param)
{
    (void)from; (void)param;
    if (message == XPLM_MSG_PLANE_LOADED) { apply(); cabin_reset(); slides_reset(); gear_reset(); }
}
