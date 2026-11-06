/* SPDX-License-Identifier: GPL-3.0-or-later
 * TAWS terrain look-ahead (a380x_taws.h) flown through synthetic terrain: level / descending / climbing
 * towards a ridge, reduced clearance, valley (straight and turning), runway approach and departure
 * without nuisance alerts, inhibits, missing data, on/off delays, threat cells on the ND raster, and
 * the evaluation cost. Simulated at 50 Hz, the look-ahead evaluated 5 times a second as in the plugin.
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "a380x_taws.h"

static const double LAT0 = 45.0, LON0 = 7.0;
typedef float (*World)(double x, double y, int *wet);   /* NM east / north of (LAT0, LON0) -> ft MSL */

static double wall(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static void to_xy(double lat, double lon, double *x, double *y)
{
    *y = (lat - LAT0) * 60.0; *x = terr_wrap180(lon - LON0) * 60.0 * cos(LAT0 * TERR_PI / 180.0);
}
static void to_latlon(double x, double y, double *lat, double *lon)
{
    *lat = LAT0 + y / 60.0; *lon = LON0 + x / (60.0 * cos(LAT0 * TERR_PI / 180.0));
}

/* Store every unsampled node of the level from the world (the plugin probes them instead). */
static void fill(TerrLevel *L, World w)
{
    int i, j;
    for (j = 0; j < TERR_GRID_N; ++j) for (i = 0; i < TERR_GRID_N; ++i) {
        double lat, lon, x, y;
        int wet = 0;
        float e;
        if (L->kind[j * TERR_GRID_N + i] != TERR_NODE_UNSAMPLED) continue;
        terr_level_node(L, i, j, &lat, &lon);
        to_xy(lat, lon, &x, &y);
        e = w(x, y, &wet);
        terr_level_store(L, i, j, terr_probe_kind(1, wet, e), e, 0);
    }
}

/* Terrain */
static float ridge_north(double x, double y, int *wet)   /* 6000 ft ridge 12-13 NM north, 1000 ft elsewhere */
{
    (void)x; *wet = 0; return y >= 12.0 && y <= 13.0 ? 6000.0f : 1000.0f;
}
static float plateau(double x, double y, int *wet)        /* 1000 ft, rising to a 5000 ft plateau from 6 NM north */
{
    (void)x; *wet = 0; return y < 6.0 ? 1000.0f : 5000.0f;
}
static float valley(double x, double y, int *wet)         /* N-S valley floor 1500 ft, |x| < 1.2 NM; walls 7000 ft */
{
    (void)y; *wet = 0; return fabs(x) < 1.2 ? 1500.0f : 7000.0f;
}
static float airfield(double x, double y, int *wet)       /* flat 400 ft; a 600 ft hill 1 NM east of final */
{
    *wet = 0;
    return hypot(x - 1.0, y + 4.0) < 0.4 ? 1000.0f : 400.0f;
}
static float airfield_hill_beyond(double x, double y, int *wet)   /* + a 600 ft hill 0.5 NM past the far end */
{
    *wet = 0;
    return hypot(x, y - 1.6) < 0.25 ? 1000.0f : 400.0f;
}
static float airfield_ridge_beyond(double x, double y, int *wet)  /* + a 3000 ft ridge from 4 NM north */
{
    (void)x; *wet = 0;
    return y > 4.0 ? 3000.0f : 400.0f;
}
static float departure_hills(double x, double y, int *wet)/* flat 400 ft; 2500 ft hills from 8 NM north */
{
    (void)x; *wet = 0; return y > 8.0 ? 2500.0f : 400.0f;
}
static float sea_level(double x, double y, int *wet) { (void)x; (void)y; *wet = 1; return 0.0f; }

typedef struct {
    double x, y; float alt, gs, track, vs, turn;
    int on_ground, available;
    int airport; double apt_x, apt_y; float apt_elev;
    double since_takeoff;
    int ra_valid; float ra;
} Flight;

typedef struct {
    double first_caution, first_warning;   /* s of flight time, -1 never */
    double caution_ahead, warning_ahead;    /* s before reaching y_conflict, at the onset */
    int max_threats, warning_threats;
    float min_clearance;
} Result;

static TawsInputs inputs(const Flight *f, World w)
{
    TawsInputs in;
    int wet;
    memset(&in, 0, sizeof(in));
    to_latlon(f->x, f->y, &in.lat, &in.lon);
    in.available = f->available; in.alt_ft = f->alt; in.gs_kt = f->gs; in.track_true = f->track; in.vs_fpm = f->vs;
    in.turn_dps = f->turn; in.on_ground = f->on_ground;
    in.airport_valid = f->airport;
    if (f->airport) to_latlon(f->apt_x, f->apt_y, &in.apt_lat, &in.apt_lon);
    in.apt_elev_ft = f->apt_elev; in.since_takeoff_s = f->since_takeoff;
    in.ra_valid = 1; in.ra_ft = f->ra_valid ? f->ra : f->alt - w(f->x, f->y, &wet);
    return in;
}

/* Fly `seconds` (50 Hz); evaluate at 5 Hz. y_conflict: the ridge's near edge for the "seconds ahead" figures.
 * stop_alt: stop when the altitude reaches it (approaches). */
static Result fly(TerrLevel *L, World w, Flight f, double seconds, double y_conflict, float stop_alt)
{
    TawsState s;
    Result r = {-1, -1, -1, -1, 0, 0, 1e9f};
    double t = 0, since_eval = 0;
    taws_init(&s);
    terr_level_init(L, TAWS_RANGE_NM, LAT0 + f.y / 60.0, LON0, 0);
    fill(L, w);
    while (t < seconds) {
        const double dt = 0.02;
        double lat, lon;
        t += dt; since_eval += dt;
        f.track += f.turn * (float)dt;
        f.x += sin(f.track * TERR_PI / 180.0) * f.gs / 3600.0 * dt;
        f.y += cos(f.track * TERR_PI / 180.0) * f.gs / 3600.0 * dt;
        f.alt += f.vs / 60.0f * (float)dt;
        f.since_takeoff += dt;
        if (stop_alt > -1e8f && f.alt <= stop_alt) break;
        to_latlon(f.x, f.y, &lat, &lon);
        if (terr_level_follow(L, lat, lon, t) || L->sampled < TERR_GRID_NODES) fill(L, w);
        if (since_eval >= 0.2 - 1e-9) {
            TawsInputs in = inputs(&f, w);
            taws_evaluate(&s, &in, L, since_eval);
            since_eval = 0;
            if (s.min_clearance_ft < r.min_clearance) r.min_clearance = s.min_clearance_ft;
            if (s.caution && r.first_caution < 0) { r.first_caution = t; r.caution_ahead = (y_conflict - f.y) / (f.gs / 3600.0); }
            if (s.warning && r.first_warning < 0) { r.first_warning = t; r.warning_ahead = (y_conflict - f.y) / (f.gs / 3600.0); }
            if (s.threat_count > r.max_threats) r.max_threats = s.threat_count;
            if (s.warning) { int k; for (k = 0; k < s.threat_count; ++k) r.warning_threats += s.threats[k].level == 2; }
        }
    }
    return r;
}

static Flight cruise(float alt, float gs)
{
    Flight f;
    memset(&f, 0, sizeof(f));
    f.alt = alt; f.gs = gs; f.available = 1; f.since_takeoff = 1e9;
    return f;
}

int main(void)
{
    TerrLevel *L = calloc(1, sizeof(TerrLevel));
    Result r;
    Flight f;
    assert(L);

    /* Required clearance by phase (CS ACNS.E.TAWS.015 Table 1). */
    {
        int phase;
        assert(taws_required_clearance(0, 0, 0, 0, &phase) == 700 && phase == TAWS_PHASE_ENROUTE);
        assert(taws_required_clearance(0, 0, 0, 1, &phase) == 500);
        assert(taws_required_clearance(1, 10, 0, 0, &phase) == 350 && phase == TAWS_PHASE_TERMINAL);
        assert(taws_required_clearance(1, 10, 0, 1, &phase) == 300);
        assert(taws_required_clearance(1, 3, 0, 0, &phase) == 150 && phase == TAWS_PHASE_APPROACH);
        assert(taws_required_clearance(1, 3, 0, 1, &phase) == 100);
        assert(taws_required_clearance(1, 3, 1, 0, &phase) == 100 && phase == TAWS_PHASE_DEPARTURE);
        assert(fabsf(taws_required_clearance(1, 5.0, 0, 0, &phase) - 250) < .01f);    /* linear 4.5 .. 5.5 NM */
        assert(fabsf(taws_required_clearance(1, 15.0, 0, 0, &phase) - 525) < .01f);   /* linear 14 .. 16 NM */
    }
    /* On / off delays. */
    {
        int out = 0;
        double on = 0, off = 0;
        taws_hysteresis(1, 0.2, &out, &on, &off); assert(!out);
        taws_hysteresis(1, 0.2, &out, &on, &off); assert(out);
        taws_hysteresis(0, 1.0, &out, &on, &off); taws_hysteresis(0, 0.8, &out, &on, &off); assert(out);
        taws_hysteresis(0, 0.2, &out, &on, &off); assert(!out);
        taws_hysteresis(1, 0.2, &out, &on, &off); taws_hysteresis(0, 0.2, &out, &on, &off);
        taws_hysteresis(1, 0.2, &out, &on, &off); assert(!out);   /* a flickering conflict never latched */
    }

    /* A. Level at 5000 ft, 250 kt, towards a 6000 ft ridge 12 NM ahead (en route, 700 ft clearance):
     * caution about 60 s and warning about 30 s before the ridge, threat cells marked as warnings. */
    f = cruise(5000, 250);
    r = fly(L, ridge_north, f, 172, 12.0, -1e9f);
    printf("TAWS level 5000 ft -> 6000 ft ridge, 250 kt: caution %.1f s and warning %.1f s before it, %d threat points\n",
           r.caution_ahead, r.warning_ahead, r.max_threats);
    assert(r.first_caution > 0 && r.caution_ahead > 55 && r.caution_ahead < 61);
    assert(r.first_warning > r.first_caution && r.warning_ahead > 26 && r.warning_ahead < 31);
    assert(r.max_threats > 0 && r.warning_threats > 0);
    /* Same at 480 kt: the look-ahead stops at 8 NM (60 s would be 8 NM): caution 60 s, warning 30 s. */
    f = cruise(5000, 480);
    r = fly(L, ridge_north, f, 89, 12.0, -1e9f);
    printf("TAWS level 5000 ft -> ridge, 480 kt: caution %.1f s, warning %.1f s before it\n", r.caution_ahead, r.warning_ahead);
    assert(r.caution_ahead > 55 && r.caution_ahead <= 60.5 && r.warning_ahead > 26 && r.warning_ahead < 31);

    /* B. 1500 ft above the ridge: no alert. C. 500 ft above it (less than 700): reduced-clearance alerts. */
    f = cruise(7500, 250);
    r = fly(L, ridge_north, f, 200, 12.0, -1e9f);
    assert(r.first_caution < 0 && r.first_warning < 0);
    f = cruise(6500, 250);
    r = fly(L, ridge_north, f, 172, 12.0, -1e9f);
    assert(r.first_caution > 0 && r.first_warning > 0);
    printf("TAWS ridge: 1500 ft above no alert; 500 ft above (< 700 ft) caution %.1f s, warning %.1f s ahead\n",
           r.caution_ahead, r.warning_ahead);

    /* D. Climbing 1500 fpm from 3000 ft at 250 kt: the climb clears the ridge, no alert; level at the
     * altitude the climb has when the ridge comes within 60 s: alerts. */
    f = cruise(3000, 250); f.vs = 1500;
    r = fly(L, ridge_north, f, 172, 12.0, -1e9f);
    assert(r.first_caution < 0 && r.first_warning < 0);
    f = cruise(5800, 250);
    r = fly(L, ridge_north, f, 172, 12.0, -1e9f);
    assert(r.first_caution > 0);
    puts("TAWS climbing 1500 fpm clears the ridge: no alert; level at the same height: alert");

    /* E. Descending 2500 fpm from 9000 ft towards a 5000 ft plateau (6 NM ahead, 250 kt). */
    f = cruise(9000, 250); f.vs = -2500;
    r = fly(L, plateau, f, 140, 6.0, 1200);
    assert(r.first_caution > 0 && r.first_warning > r.first_caution);
    printf("TAWS descent 2500 fpm towards a plateau: caution at %.1f s, warning at %.1f s of flight\n", r.first_caution, r.first_warning);

    /* F. Valley at 3000 ft (walls 4000 ft above, 1.2 NM either side), 200 kt: no alert straight;
     * turning 3 deg/s towards a wall: alert. */
    {
        float straight_clearance;
        f = cruise(3000, 200);
        r = fly(L, valley, f, 300, 1e9, -1e9f);
        assert(r.first_caution < 0 && r.first_warning < 0);
        straight_clearance = r.min_clearance;
        f = cruise(3000, 200); f.turn = 3.0f;
        r = fly(L, valley, f, 20, 1e9, -1e9f);
        assert(r.first_caution > 0);
        printf("TAWS valley: straight no alert (min clearance in the ribbon %.0f ft); turning 3 deg/s into a wall: "
               "caution after %.1f s\n", straight_clearance, r.first_caution);
    }

    /* G. 3-degree ILS approach to a 400 ft airport (runway centre at the origin, threshold 1 NM south),
     * 140 kt, gear down, from 10 NM down to 50 ft: no alert; nor with a 600 ft hill 1 NM beside final. */
    memset(&f, 0, sizeof(f));
    f.y = -11.0; f.gs = 140; f.available = 1; f.since_takeoff = 1e9;
    f.airport = 1; f.apt_elev = 400;
    f.alt = 400 + 10.0f * 318.4f; f.vs = -(float)(140.0 * 6076.12 / 60.0 * tan(3.0 * TERR_PI / 180.0));
    r = fly(L, airfield, f, 400, 0, 450);
    printf("TAWS 3-degree approach to 50 ft (vs %.0f fpm): caution %s, warning %s, min clearance %.0f ft\n",
           f.vs, r.first_caution < 0 ? "none" : "YES", r.first_warning < 0 ? "none" : "YES", r.min_clearance);
    assert(r.first_caution < 0 && r.first_warning < 0);
    /* ... the same approach 1000 ft low (descending into the ground 5 NM short): alerts. */
    f.alt -= 1100;
    r = fly(L, airfield, f, 400, 0, 450);
    assert(r.first_caution > 0);
    printf("TAWS same approach 1100 ft low: caution at %.1f s, warning at %.1f s\n", r.first_caution, r.first_warning);

    /* ... a 600 ft hill 0.5 NM past the far runway end (1.5 NM past the reference point), inside the
     * runway area but above the runway margin: still no alert on the approach (look-ahead ends 0.5 NM
     * past the reference point while descending towards it). */
    f.alt += 1100;
    r = fly(L, airfield_hill_beyond, f, 400, 0, 450);
    assert(r.first_caution < 0 && r.first_warning < 0);
    puts("TAWS same approach, 600 ft hill 0.5 NM past the runway end: no alert");
    /* ... but descending past the airport (overflight, 1500 ft, 1000 fpm, 200 kt) towards that hill and
     * a 3000 ft ridge 4 NM beyond: the full look-ahead again once the airport is behind -> alerts. */
    memset(&f, 0, sizeof(f));
    f.y = -1.0; f.gs = 200; f.alt = 1900; f.vs = -1000; f.available = 1; f.since_takeoff = 1e9; f.airport = 1; f.apt_elev = 400;
    r = fly(L, airfield_ridge_beyond, f, 120, 4.0, 450);
    assert(r.first_caution > 0 && r.caution_ahead > 30.0);
    printf("TAWS descending overflight of the airport towards a ridge 4 NM beyond: caution %.1f s before it\n", r.caution_ahead);

    /* H. Departure: lift-off at the airport, 160 kt, 2000 fpm; 2500 ft hills from 8 NM: no alert. */
    memset(&f, 0, sizeof(f));
    f.gs = 160; f.vs = 2000; f.alt = 450; f.available = 1; f.airport = 1; f.apt_elev = 400; f.since_takeoff = 0;
    r = fly(L, departure_hills, f, 240, 8.0, -1e9f);
    assert(r.first_caution < 0 && r.first_warning < 0);
    puts("TAWS departure climbing 2000 fpm towards 2500 ft hills: no alert");

    /* I. Inhibits: on the ground, TAWS unavailable (TERR SYS OFF), below 30 ft RA, below 50 kt. */
    {
        Flight g[4];
        int k;
        for (k = 0; k < 4; ++k) g[k] = cruise(5000, 250);
        g[0].on_ground = 1; g[1].available = 0; g[2].ra_valid = 1; g[2].ra = 20; g[3].gs = 40;
        for (k = 0; k < 4; ++k) { r = fly(L, ridge_north, g[k], k == 3 ? 900 : 172, 12.0, -1e9f); assert(r.first_caution < 0); }
        puts("TAWS inhibits: ground, TERR SYS OFF / no TAWS, RA < 30 ft, < 50 kt");
    }

    /* J. Sea at sea level counts as terrain at 0 ft: descending into it at 1500 fpm from 1500 ft alerts;
     * level at 1500 ft over it does not (approach phase unknown: en route 700 ft). */
    f = cruise(1500, 200); f.vs = -1500;
    r = fly(L, sea_level, f, 50, 1e9, 100);
    assert(r.first_warning > 0);
    f = cruise(1500, 200);
    r = fly(L, sea_level, f, 60, 1e9, -1e9f);
    assert(r.first_caution < 0);

    /* K. No terrain data (unsampled level): no alert. */
    {
        TawsState s;
        TawsInputs in;
        Flight g = cruise(5000, 250);
        int k;
        taws_init(&s);
        terr_level_init(L, TAWS_RANGE_NM, LAT0, LON0, 0);
        in = inputs(&g, ridge_north);
        for (k = 0; k < 20; ++k) taws_evaluate(&s, &in, L, 0.2);
        assert(!s.caution && !s.warning && s.samples == 0 && s.inhibit == TAWS_ACTIVE);
    }

    /* L. Threat cells on the ND raster: solid yellow / red cells at the threat points (and around them). */
    {
        TerrDisplay *d = calloc(1, sizeof(TerrDisplay));
        TerrAircraft ac = {LAT0, LON0, 0.0f, 5000, 0, -500, 0};
        TerrThreat threats[2];
        int caution = 0, warning = 0, row, col;
        assert(d);
        terr_level_init(L, 40, LAT0, LON0, 0); fill(L, ridge_north);
        to_latlon(0.0, 12.5, &threats[0].lat, &threats[0].lon); threats[0].level = 2;
        to_latlon(5.0, 12.5, &threats[1].lat, &threats[1].lon); threats[1].level = 1;
        terr_display_reset(d);
        d->threats = threats; d->threat_count = 2;
        terr_display_update(d, 1, 3, 40, &ac, L, 0.0, 100000);
        terr_display_update(d, 1, 3, 40, &ac, L, 0.7, 100000);
        for (row = 0; row < d->geo.rows; ++row) for (col = 0; col < d->geo.cols; ++col) {
            caution += d->cls[row][col] == TERR_CAUTION; warning += d->cls[row][col] == TERR_WARNING;
        }
        assert(warning == 9 && caution == 9);
        /* the warning cell is where 12.5 NM north lies on an ARC 40 NM map: 155.6 px above (384, 626) */
        row = (int)floorf((626.0f - 12.5f * 498.0f / 40.0f - 128.0f) / 8.0f); col = (int)floorf((384.0f - 6.0f) / 8.0f);
        assert(d->cls[row][col] == TERR_WARNING);
        free(d);
    }

    /* M. Cost of one evaluation (500 kt: 8 NM ahead, 100 steps). */
    {
        TawsState s;
        TawsInputs in;
        Flight g = cruise(5000, 500);
        double t0;
        int k, n = 200;
        taws_init(&s);
        terr_level_init(L, TAWS_RANGE_NM, LAT0, LON0, 0); fill(L, ridge_north);
        in = inputs(&g, ridge_north);
        t0 = wall();
        for (k = 0; k < n; ++k) taws_evaluate(&s, &in, L, 0.2);
        printf("TAWS evaluation at 500 kt: %d terrain samples, %.1f us per evaluation (5 per second in the plugin)\n",
               s.samples, (wall() - t0) / n * 1e6);
        assert((wall() - t0) / n < 2e-3);
    }
    free(L);
    puts("TAWS look-ahead: clearances, delays, ridge level/descending/climbing, valley, approach/departure without "
         "nuisance alerts, inhibits, sea, missing data, threat cells and cost passed");
    return 0;
}
