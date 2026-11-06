/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * TERRAIN on ND, X-Plane boundary (docs/ND_TERRAIN.md). Core: a380x_terrain.h.
 *
 * Every flight loop (terrain_update, before the xp-kit publication):
 *   - per ND: TERR selected on its EFIS CP (overlay TERR, exclusive with WX, a380x_efis.inc), ARC or
 *     ROSE (not PLAN, not the ZOOM/OANS range), ND unit on, ADIRS position and heading valid, TAWS
 *     available (WXR/TAWS SYS 1 or 2, TERR SYS not OFF, a terrain probe) -> its TerrDisplay
 *     (nd_data.terrain[side]) is swept from the elevation level of its range;
 *   - the levels (one per selected range, at most TERRAIN_LEVELS) follow the aircraft and are filled
 *     with XPLMProbeTerrainXYZ at XPLMWorldToLocal of each node, the hit converted back with
 *     XPLMLocalToWorld (feet MSL, is_wet). A probe that misses is "no data" (magenta);
 *   - the ND pointer (cursor over the ND device) gets one direct probe when it moves (at most every
 *     0.15 s, and again every 0.5 s) for the simulator-aid elevation readout;
 *   - the frame stays within TERRAIN_FRAME_LIMIT_S of main-thread time: the displays and the pointer
 *     first (bounded), probing with what is left (at most TERRAIN_BUDGET_S).
 *   - TAWS look-ahead alerting (a380x_taws.h, docs/ND_TERRAIN.md "Terrain alerts"): whenever TAWS is
 *     available and the position valid, the 10 NM level is kept and probed (first in the round robin)
 *     whatever the NDs show; 5 times a second the caution / warning envelopes are evaluated on it. The
 *     threat points go to both TerrDisplays (solid yellow / red cells), the start of an alert selects
 *     TERR on both EFIS CPs (pop-up), and nd_data.taws_caution / taws_warning feed the aurals, the
 *     native PFD and the FBW PFD's EGPWS discrete word (a380x_xpkit_terrain.inc).
 * Diagnostics (read-only): a380x/xp12/terrain/... (probe cost, frame time, coverage, pointer, TAWS).
 */
#include "a380x_terrain.h"
#include "a380x_taws.h"

#define TERRAIN_LEVELS 4                 /* two ND ranges, the TAWS 10 NM level, one spare */
#define TERRAIN_FRAME_LIMIT_S 0.0005     /* target: terrain work per simulator frame */
#define TERRAIN_BUDGET_S 0.0004          /* probing share of it */
#define TERRAIN_MAX_NODES 512            /* per frame, whatever the clock says */
#define TERRAIN_BUILD_CELLS 2048         /* raster cells built per ND per frame */

/* The ND cockpit device of a side (a380x_native_displays.inc: native or xp-kit device). */
static XPLMAvionicsID terrain_nd_device(int side);

static struct {
    XPLMProbeRef probe;
    XPLMDataRef vs_fpm;
    TerrLevel *levels[TERRAIN_LEVELS];
    int active[2], taws_sys, terr_off, failed;
    /* runway cut-off */
    char airport[16];
    double airport_lat, airport_lon, cutoff_at;
    float airport_elev_ft, cutoff_ft;
    int airport_elev_valid;
    /* diagnostics */
    float probe_us, frame_us, frame_us_max, window_max, probes_per_frame, coverage[2], pointer_ft[2];
    double window_start;
    int probes_total, hits_total, misses_total, active_out[2], levels_kb;
    /* nearest airport (runway cut-off, TAWS phase and runway environment) */
    float airport_nm;
    double airport_probe_at;
    /* TAWS look-ahead */
    XPLMDataRef elevation_m, groundspeed, hpath, onground;
    TawsState taws;
    TerrThreat taws_shown[TAWS_THREATS];
    int taws_shown_count, taws_alert, taws_was_ground, taws_caution_out, taws_warning_out, taws_phase_out, taws_inhibit_out;
    double taws_at, takeoff_at;
    float taws_track, taws_turn_dps, taws_eval_us, taws_clearance_out, taws_ttc_out, taws_rtc_out, taws_lookahead_out;
    XPLMDataRef refs[32];
} terrain;

static double terrain_clock(void)
{
#if IBM
    LARGE_INTEGER counter, frequency;
    QueryPerformanceCounter(&counter); QueryPerformanceFrequency(&frequency);
    return (double)counter.QuadPart / (double)frequency.QuadPart;
#else
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + t.tv_nsec * 1e-9;
#endif
}
static double terrain_clock_cb(void *ctx) { (void)ctx; return terrain_clock(); }
/* Display time (sweep cycle, pointer refresh): the monotonic clock; tests substitute a simulated one. */
#ifndef TERRAIN_NOW
#define TERRAIN_NOW() terrain_clock()
#endif

/* One terrain probe: 1 with feet MSL and the water flag, 0 when there is no terrain under the point
 * (scenery not loaded) or the probe failed. */
static int terrain_probe(void *ctx, double lat, double lon, float *elev_ft, int *wet)
{
    double x, y, z, hit_lat, hit_lon, hit_alt;
    XPLMProbeInfo_t hit;
    (void)ctx;
    if (!terrain.probe) return 0;
    XPLMWorldToLocal(lat, lon, 0.0, &x, &y, &z);
    memset(&hit, 0, sizeof(hit));
    hit.structSize = sizeof(hit);
    if (XPLMProbeTerrainXYZ(terrain.probe, (float)x, (float)y, (float)z, &hit) != xplm_ProbeHitTerrain) return 0;
    XPLMLocalToWorld(hit.locationX, hit.locationY, hit.locationZ, &hit_lat, &hit_lon, &hit_alt);
    if (!isfinite(hit_alt)) return 0;
    *elev_ft = (float)(hit_alt * 3.280839895);
    *wet = hit.is_wet != 0;
    return 1;
}

/* Nearest airport of the cached navigation data (ND nav scan) to (lat, lon): terrain.airport*, its
 * distance in terrain.airport_nm and its elevation, probed once per airport (again every second while
 * the probe finds no scenery there). 1 when its elevation is known. */
static int terrain_nearest_airport(double lat, double lon, double now)
{
    const EfisNdNavPoint *best = NULL;
    float best_nm = 1e9f;
    int n;
    for (n = 0; n < nd_data.nav_count[0]; ++n) {
        float east, north;
        const EfisNdNavPoint *p = &nd_data.nav[0][n];
        if (p->id[0] && efis_nd_project(lat, lon, p->lat, p->lon, 0, &east, &north) && hypotf(east, north) < best_nm) {
            best = p; best_nm = hypotf(east, north);
        }
    }
    if (!best) return 0;
    if (strcmp(best->id, terrain.airport) || best->lat != terrain.airport_lat || best->lon != terrain.airport_lon) {
        snprintf(terrain.airport, sizeof(terrain.airport), "%s", best->id);
        terrain.airport_lat = best->lat; terrain.airport_lon = best->lon;
        terrain.airport_elev_valid = 0; terrain.airport_probe_at = -1e9;
    }
    if (!terrain.airport_elev_valid && now - terrain.airport_probe_at >= 1.0) {
        int wet = 0;
        terrain.airport_probe_at = now;
        terrain.airport_elev_valid = terrain_probe(NULL, best->lat, best->lon, &terrain.airport_elev_ft, &wet);
    }
    terrain.airport_nm = best_nm;
    return terrain.airport_elev_valid;
}

/* Honeywell: terrain within 400 ft of the nearest runway elevation is not displayed. Nearest airport of
 * the cached navigation data, its elevation probed once; FBW's refinement near the runway: 400 ft
 * down to 200 ft between 4 and 1 NM on a glide flatter than 3 degrees. -500 (none) without one. */
static float terrain_cutoff(const TerrAircraft *ac, double now)
{
    float best_nm, cutoff;
    if (now - terrain.cutoff_at < 0.5) return terrain.cutoff_ft;
    terrain.cutoff_at = now;
    if (!terrain_nearest_airport(ac->lat, ac->lon, now)) { terrain.cutoff_ft = -500.0f; return terrain.cutoff_ft; }
    best_nm = terrain.airport_nm;
    cutoff = 400.0f;
    if (best_nm <= 4.0f) {
        float opposite = ac->altitude_ft - terrain.airport_elev_ft, glide = 0.0f;
        if (opposite > 0.0f && best_nm > 0.0f) glide = atanf(opposite / (best_nm * 6076.12f));
        if (glide < 0.0523599f)
            cutoff = best_nm <= 1.0f || glide == 0.0f ? 200.0f
                   : fminf(400.0f, fmaxf(200.0f, 400.0f - 200.0f * (4.0f - best_nm) / 3.0f));
    }
    terrain.cutoff_ft = terrain.airport_elev_ft + cutoff;
    return terrain.cutoff_ft;
}

/* The level of a range (shared by both NDs), following the aircraft; a free or least recently used
 * slot for a new range. NULL if no memory. */
static TerrLevel *terrain_level(float range_nm, const TerrAircraft *ac, double now, const TerrLevel *keep)
{
    int k, slot = -1;
    for (k = 0; k < TERRAIN_LEVELS; ++k) {
        TerrLevel *L = terrain.levels[k];
        if (L && L->used && L->range_nm == range_nm) { terr_level_follow(L, ac->lat, ac->lon, now); return L; }
    }
    for (k = 0; k < TERRAIN_LEVELS; ++k) {
        TerrLevel *L = terrain.levels[k];
        if (!L || !L->used) { slot = k; break; }
        if (L != keep && (slot < 0 || L->last_used < terrain.levels[slot]->last_used)) slot = k;
    }
    if (slot < 0) return NULL;
    if (!terrain.levels[slot] && !(terrain.levels[slot] = malloc(sizeof(TerrLevel)))) return NULL;
    terr_level_init(terrain.levels[slot], range_nm, ac->lat, ac->lon, now);
    return terrain.levels[slot];
}

static int terrain_taws_available(void)
{
    terrain.taws_sys = survh_state[SURV_WXR_TAWS_SYS];
    terrain.terr_off = survh_state[SURV_TERR_OFF] != 0;
    terrain.failed = terrain.probe == NULL;
    return !terrain.failed && terrain.taws_sys != 0 && !terrain.terr_off;
}

static int terrain_side_active(int side, int taws)
{
    const EfisSide *e = &efis[side];
    const Side *s = &refs.side[side];
    return plugin_enabled && taws && e->terr && !e->zoom && e->mode >= 0 && e->mode <= 3 && s->nd_valid &&
           s->navigation_valid && s->heading_valid && nd_data.position_valid;
}

/* Pointer over the ND: the terrain under it (one probe when it moved, refreshed every 0.5 s). */
static int terrain_pointer_update(int side, const TerrAircraft *ac, double now)
{
    TerrPointer *p = &nd_data.terrain_pointer[side];
    XPLMAvionicsID device = terrain_nd_device(side);
    const TerrDisplay *d = &nd_data.terrain[side];
    char text[48] = "";
    int x, y, probes = 0, was_shown = p->shown;
    float px = p->x, py = p->y;
    double lat, lon;
    p->inside = device && mfda_av.loaded && mfda_av.cursor_over(device, &x, &y);
    if (p->inside) {
        p->x = A380X_DU_LOGICAL(x, A380X_ND_SCALE);
        p->y = 1023.0f - A380X_DU_LOGICAL(y, A380X_ND_SCALE);
    }
    p->shown = p->inside && d->on && terr_pointer_target(&d->geo, ac->lat, ac->lon, ac->heading_true, p->x, p->y, &lat, &lon);
    if (p->shown) {
        int moved = fabsf(p->x - p->probed_x) > 0.75f || fabsf(p->y - p->probed_y) > 0.75f;
        if ((moved && now - p->probed_at >= 0.15) || now - p->probed_at >= 0.5) {
            float elev = 0.0f;
            int wet = 0, hit = terrain_probe(NULL, lat, lon, &elev, &wet);
            p->kind = terr_probe_kind(hit, wet, elev);
            p->elev_ft = hit ? elev : 0.0f;
            p->probed_x = p->x; p->probed_y = p->y; p->probed_at = now;
            probes = 1;
            terrain.hits_total += hit; terrain.misses_total += !hit;
        }
        if (p->kind == TERR_NODE_LAND) snprintf(text, sizeof(text), "SIM ELEV %d FT", (int)lroundf(p->elev_ft / 10.0f) * 10);
        else if (p->kind == TERR_NODE_WATER) snprintf(text, sizeof(text), "SIM ELEV %d FT SEA", (int)lroundf(p->elev_ft));
        else snprintf(text, sizeof(text), "SIM ELEV NO DATA");
    }
    terrain.pointer_ft[side] = p->shown && p->kind != TERR_NODE_MISSED ? p->elev_ft : -9999.0f;
    if (device && mfda_av.loaded && (strcmp(text, p->text) || was_shown != p->shown || (p->shown && (px != p->x || py != p->y))))
        mfda_av.needs_drawing(device);
    snprintf(p->text, sizeof(p->text), "%s", text);
    return probes;
}

/* TAWS look-ahead, 5 times a second on the 10 NM level L (NULL: none). Geometric altitude, ground
 * speed, true track, inertial vertical speed and ground contact from the flight model (the EGPWS uses
 * GPS geometric altitude and position); radio altitude from the RA; turn rate from the track change. */
static void terrain_taws_update(double now, int available, const TerrLevel *L)
{
    TawsInputs in;
    double dt = now - terrain.taws_at, t0;
    int on_ground, alert, k, side;
    if (dt < 0.2 - 1e-6) return;
    terrain.taws_at = now;
    if (dt > 0.5) dt = 0.2;   /* first evaluation, or after a stall of the loop */
    memset(&in, 0, sizeof(in));
    on_ground = terrain.onground && XPLMGetDatai(terrain.onground) != 0;
    if (terrain.taws_was_ground == 1 && !on_ground) terrain.takeoff_at = now;
    terrain.taws_was_ground = on_ground;
    in.available = available && L && nd_data.position_valid;
    in.lat = nd_data.lat; in.lon = nd_data.lon;
    in.alt_ft = terrain.elevation_m ? (float)(XPLMGetDatad(terrain.elevation_m) * 3.280839895) : nd_data.altitude_ft;
    in.gs_kt = terrain.groundspeed ? XPLMGetDataf(terrain.groundspeed) * 1.94384449f : NAN;
    in.track_true = terrain.hpath ? XPLMGetDataf(terrain.hpath) : NAN;
    in.vs_fpm = terrain.vs_fpm ? XPLMGetDataf(terrain.vs_fpm) : 0.0f;
    in.ra_valid = refs.side[0].radio_altitude_valid; in.ra_ft = refs.side[0].radio_altitude_value;
    in.on_ground = on_ground;
    in.since_takeoff_s = now - terrain.takeoff_at;
    if (in.available && terrain_nearest_airport(in.lat, in.lon, now)) {
        in.airport_valid = 1;
        in.apt_lat = terrain.airport_lat; in.apt_lon = terrain.airport_lon; in.apt_elev_ft = terrain.airport_elev_ft;
    }
    if (isfinite(in.track_true) && isfinite(terrain.taws_track) && isfinite(in.gs_kt) && in.gs_kt >= 50.0f) {
        float rate = (float)(terr_wrap180(in.track_true - terrain.taws_track) / dt);
        terrain.taws_turn_dps = 0.6f * terrain.taws_turn_dps + 0.4f * rate;
    } else terrain.taws_turn_dps = 0.0f;
    terrain.taws_track = in.track_true;
    in.turn_dps = terrain.taws_turn_dps;
    t0 = terrain_clock();
    taws_evaluate(&terrain.taws, &in, L, dt);
    terrain.taws_eval_us = (float)((terrain_clock() - t0) * 1e6);
    /* Threat areas while an alert lasts: the latest points (a warning point is red only once the
     * warning is on), kept through the alert's off delay. */
    alert = terrain.taws.caution || terrain.taws.warning;
    if (!alert) terrain.taws_shown_count = 0;
    else if (terrain.taws.threat_count) {
        for (k = 0; k < terrain.taws.threat_count; ++k) {
            terrain.taws_shown[k] = terrain.taws.threats[k];
            terrain.taws_shown[k].level = terrain.taws.threats[k].level >= 2 && terrain.taws.warning ? 2 : 1;
        }
        terrain.taws_shown_count = terrain.taws.threat_count;
    }
    /* Pop-up: the start of an alert selects TERR (replacing WX) on both EFIS CPs. */
    if (alert && !terrain.taws_alert)
        for (side = 0; side < 2; ++side)
            if (!efis[side].terr) { efis[side].terr = 1; efis[side].wx_requested = 0; efis_sync_side(side); }
    terrain.taws_alert = alert;
    nd_data.taws_caution = terrain.taws.caution;
    nd_data.taws_warning = terrain.taws.warning;
    terrain.taws_caution_out = terrain.taws.caution; terrain.taws_warning_out = terrain.taws.warning;
    terrain.taws_phase_out = terrain.taws.phase; terrain.taws_inhibit_out = terrain.taws.inhibit;
    terrain.taws_clearance_out = terrain.taws.inhibit == TAWS_ACTIVE && terrain.taws.samples ? terrain.taws.min_clearance_ft : 99999.0f;
    terrain.taws_ttc_out = terrain.taws.time_to_conflict_s;
    terrain.taws_rtc_out = terrain.taws.rtc_ft; terrain.taws_lookahead_out = terrain.taws.lookahead_nm;
}

static void terrain_update(void)
{
    const double start = terrain_clock(), now = TERRAIN_NOW();
    TerrAircraft ac;
    TerrLevel *side_level[2] = {NULL, NULL}, *probe_levels[3], *taws_level = NULL;
    TerrProbeStats st;
    int side, count = 0, taws, probes = 0, k;
    double elapsed, budget;
    if (!plugin_enabled) { nd_data.taws_caution = nd_data.taws_warning = 0; return; }
    memset(&ac, 0, sizeof(ac));
    ac.lat = nd_data.lat; ac.lon = nd_data.lon;
    ac.heading_true = nd_data.true_heading; ac.altitude_ft = nd_data.altitude_ft;
    ac.vs_fpm = terrain.vs_fpm ? XPLMGetDataf(terrain.vs_fpm) : 0.0f;
    ac.gear_down = gear_values[0] >= 0.99f;
    taws = terrain_taws_available();
    /* The TAWS level first: never evicted by an ND range change in this frame, probed first. */
    if (taws && nd_data.position_valid && isfinite(ac.lat) && isfinite(ac.lon) && (taws_level = terrain_level(TAWS_RANGE_NM, &ac, now, NULL)))
        probe_levels[count++] = taws_level;
    terrain_taws_update(now, taws, taws_level);
    for (side = 0; side < 2; ++side) terrain.active[side] = terrain_side_active(side, taws);
    ac.cutoff_ft = terrain.active[0] || terrain.active[1] ? terrain_cutoff(&ac, now) : -500.0f;
    for (side = 0; side < 2; ++side) {
        int seen = 0;
        if (!terrain.active[side]) continue;
        side_level[side] = terrain_level(efis[side].range_nm, &ac, now, side ? side_level[0] : taws_level);
        for (k = 0; k < count; ++k) seen |= probe_levels[k] == side_level[side];
        if (side_level[side] && !seen) probe_levels[count++] = side_level[side];
    }
    for (side = 0; side < 2; ++side) {
        TerrDisplay *d = &nd_data.terrain[side];
        d->threats = terrain.taws_shown; d->threat_count = terrain.taws_shown_count;
        terr_display_update(d, terrain.active[side], efis[side].mode, efis[side].range_nm, &ac, side_level[side], now,
                            TERRAIN_BUILD_CELLS);
        terrain.coverage[side] = d->on && d->visible_cells ? (float)d->data_cells / (float)d->visible_cells : 0.0f;
        terrain.active_out[side] = terrain.active[side];
        probes += terrain_pointer_update(side, &ac, now);
    }
    elapsed = terrain_clock() - start;
    budget = fmin(TERRAIN_BUDGET_S, TERRAIN_FRAME_LIMIT_S - elapsed);
    memset(&st, 0, sizeof(st));
    if (count && budget > 0.0)
        terr_probe_levels(probe_levels, count, &ac, now, budget, TERRAIN_MAX_NODES, terrain_probe, terrain_clock_cb, NULL, &st);
    probes += st.probes;
    terrain.probes_total += probes;
    terrain.hits_total += st.hits;
    terrain.misses_total += st.probes - st.hits;
    if (st.probes > 0) {
        float per_probe = (float)(st.elapsed_s * 1e6 / st.probes);
        terrain.probe_us = terrain.probe_us > 0.0f ? terrain.probe_us * 0.95f + per_probe * 0.05f : per_probe;
    }
    terrain.probes_per_frame = terrain.probes_per_frame * 0.95f + (float)probes * 0.05f;
    elapsed = (terrain_clock() - start) * 1e6;
    terrain.frame_us = terrain.frame_us * 0.95f + (float)elapsed * 0.05f;
    if (now - terrain.window_start > 5.0) { terrain.frame_us_max = terrain.window_max; terrain.window_max = 0; terrain.window_start = now; }
    if (elapsed > terrain.window_max) terrain.window_max = (float)elapsed;
    if (elapsed > terrain.frame_us_max) terrain.frame_us_max = (float)elapsed;
    {
        int k, kb = 0;
        for (k = 0; k < TERRAIN_LEVELS; ++k) kb += terrain.levels[k] ? (int)(sizeof(TerrLevel) / 1024) : 0;
        terrain.levels_kb = kb;
    }
}

static float terrain_get_float(void *refcon) { return *(float *)refcon; }
static int terrain_get_int(void *refcon) { return *(int *)refcon; }

static void terrain_init(void)
{
    static const struct { const char *name; int is_int; } outputs[] = {
        {"a380x/xp12/terrain/probe_us", 0}, {"a380x/xp12/terrain/frame_us", 0}, {"a380x/xp12/terrain/frame_us_max", 0},
        {"a380x/xp12/terrain/probes_per_frame", 0}, {"a380x/xp12/terrain/probes_total", 1},
        {"a380x/xp12/terrain/hits_total", 1}, {"a380x/xp12/terrain/misses_total", 1},
        {"a380x/xp12/terrain/captain_active", 1}, {"a380x/xp12/terrain/first_officer_active", 1},
        {"a380x/xp12/terrain/captain_coverage", 0}, {"a380x/xp12/terrain/first_officer_coverage", 0},
        {"a380x/xp12/terrain/captain_pointer_elevation_ft", 0}, {"a380x/xp12/terrain/first_officer_pointer_elevation_ft", 0},
        {"a380x/xp12/terrain/cache_kb", 1},
        {"a380x/xp12/terrain/taws_caution", 1}, {"a380x/xp12/terrain/taws_warning", 1},
        {"a380x/xp12/terrain/taws_phase", 1}, {"a380x/xp12/terrain/taws_inhibit", 1},
        {"a380x/xp12/terrain/taws_min_clearance_ft", 0}, {"a380x/xp12/terrain/taws_time_to_conflict_s", 0},
        {"a380x/xp12/terrain/taws_required_clearance_ft", 0}, {"a380x/xp12/terrain/taws_lookahead_nm", 0},
        {"a380x/xp12/terrain/taws_eval_us", 0}
    };
    void *fields[] = {&terrain.probe_us, &terrain.frame_us, &terrain.frame_us_max, &terrain.probes_per_frame,
                      &terrain.probes_total, &terrain.hits_total, &terrain.misses_total, &terrain.active_out[0],
                      &terrain.active_out[1], &terrain.coverage[0], &terrain.coverage[1], &terrain.pointer_ft[0],
                      &terrain.pointer_ft[1], &terrain.levels_kb,
                      &terrain.taws_caution_out, &terrain.taws_warning_out, &terrain.taws_phase_out, &terrain.taws_inhibit_out,
                      &terrain.taws_clearance_out, &terrain.taws_ttc_out, &terrain.taws_rtc_out, &terrain.taws_lookahead_out,
                      &terrain.taws_eval_us};
    int k;
    memset(&terrain, 0, sizeof(terrain));
    terrain.probe = XPLMCreateProbe(xplm_ProbeY);
    terrain.vs_fpm = XPLMFindDataRef("sim/flightmodel/position/vh_ind_fpm");
    terrain.elevation_m = XPLMFindDataRef("sim/flightmodel/position/elevation");
    terrain.groundspeed = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    terrain.hpath = XPLMFindDataRef("sim/flightmodel/position/hpath");
    terrain.onground = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    terrain.cutoff_ft = -500.0f; terrain.cutoff_at = -1e9;
    taws_init(&terrain.taws);
    terrain.taws_at = terrain.takeoff_at = terrain.airport_probe_at = -1e9;
    terrain.taws_was_ground = -1; terrain.taws_track = NAN;
    terrain.taws_clearance_out = 99999.0f; terrain.taws_ttc_out = -1.0f;
    terrain.taws_inhibit_out = TAWS_INHIBIT_OFF;
    terrain.pointer_ft[0] = terrain.pointer_ft[1] = -9999.0f;
    for (k = 0; k < (int)(sizeof(fields) / sizeof(fields[0])) && k < (int)(sizeof(terrain.refs) / sizeof(terrain.refs[0])); ++k) {
        if (XPLMFindDataRef(outputs[k].name)) continue;
        terrain.refs[k] = XPLMRegisterDataAccessor(outputs[k].name, outputs[k].is_int ? xplmType_Int : xplmType_Float, 0,
            outputs[k].is_int ? terrain_get_int : NULL, NULL, outputs[k].is_int ? NULL : terrain_get_float, NULL,
            NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, fields[k], NULL);
    }
    for (k = 0; k < 2; ++k) terr_display_reset(&nd_data.terrain[k]);
}

static void terrain_shutdown(void)
{
    int k;
    for (k = 0; k < (int)(sizeof(terrain.refs) / sizeof(terrain.refs[0])); ++k)
        if (terrain.refs[k]) XPLMUnregisterDataAccessor(terrain.refs[k]);
    for (k = 0; k < TERRAIN_LEVELS; ++k) free(terrain.levels[k]);
    if (terrain.probe) XPLMDestroyProbe(terrain.probe);
    memset(&terrain, 0, sizeof(terrain));
    for (k = 0; k < 2; ++k) terr_display_reset(&nd_data.terrain[k]);
    nd_data.taws_caution = nd_data.taws_warning = 0;
}
