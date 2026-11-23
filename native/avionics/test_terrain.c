/* SPDX-License-Identifier: GPL-3.0-or-later
 * TERRAIN on ND core (a380x_terrain.h): dot patterns, clip geometry, EGPWS bands, peaks mode, TERR
 * box values and text, run-length string, probe scheduler and frame budget (simulated and real-time
 * probe cost), cache shifts, the scan-line sweep and the pointer target. No XPLM.
 *   test_terrain [--emit <out.ppm>]   (--emit writes an ARC raster of a synthetic level, for review)
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "a380x_terrain.h"

static int near(double a, double b, double tolerance) { return fabs(a - b) <= tolerance; }

static double wall_clock(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + t.tv_nsec * 1e-9;
}

/* Synthetic world: a 9000 ft cone 20 NM east-north-east of (45 N, 7 E), sea west of 6.7 E, no scenery
 * (probe miss) north of 46.5 N. */
static float world_elevation(double lat, double lon, int *wet)
{
    double dn = (lat - 45.10) * 60.0, de = (lon - 7.45) * 60.0 * cos(45.0 * TERR_PI / 180.0);
    double d = hypot(de, dn);
    *wet = lon < 6.7;
    if (*wet) return 0.0f;
    return (float)(500.0 + fmax(0.0, 8500.0 * (1.0 - d / 15.0)));
}

typedef struct { double now, cost_s, real_cost_s; int probes; } MockCtx;
static int mock_probe(void *ctx, double lat, double lon, float *elev, int *wet)
{
    MockCtx *m = ctx;
    ++m->probes;
    m->now += m->cost_s;
    if (m->real_cost_s > 0) { double until = wall_clock() + m->real_cost_s; while (wall_clock() < until) {} }
    if (lat > 46.5) return 0;
    *elev = world_elevation(lat, lon, wet);
    return 1;
}
static double mock_clock(void *ctx) { MockCtx *m = ctx; return m->real_cost_s > 0 ? wall_clock() : m->now; }

static void test_patterns(void)
{
    int pattern, x, y;
    const int tiles[4][2] = {{TERR_PAT_LOW, 13}, {TERR_PAT_HIGH, 6}, {TERR_PAT_WATER, 9}, {TERR_PAT_SOLID, 1}};
    for (pattern = 0; pattern < 4; ++pattern) {
        int p = tiles[pattern][1], on = 0;
        for (y = 0; y < p; ++y) for (x = 0; x < p; ++x) {
            on += terr_pattern_on(tiles[pattern][0], x, y);
            /* periodic */
            assert(terr_pattern_on(tiles[pattern][0], x, y) == terr_pattern_on(tiles[pattern][0], x + 3 * p, y + 5 * p));
        }
        assert(on == (p == 1 ? 1 : 18));
    }
    /* FBW scanlinemode.ts tiles: high "..###." / "##...#", low ".....###....." / "##..........#",
     * water "....###.." / "##......#". */
    assert(!terr_pattern_on(TERR_PAT_HIGH, 0, 0) && terr_pattern_on(TERR_PAT_HIGH, 2, 0) && terr_pattern_on(TERR_PAT_HIGH, 4, 1));
    assert(terr_pattern_on(TERR_PAT_HIGH, 0, 2) && terr_pattern_on(TERR_PAT_HIGH, 5, 4) && !terr_pattern_on(TERR_PAT_HIGH, 2, 3));
    assert(terr_pattern_on(TERR_PAT_LOW, 5, 0) && terr_pattern_on(TERR_PAT_LOW, 7, 12) && !terr_pattern_on(TERR_PAT_LOW, 5, 2));
    assert(terr_pattern_on(TERR_PAT_LOW, 0, 5) && terr_pattern_on(TERR_PAT_LOW, 12, 7) && !terr_pattern_on(TERR_PAT_LOW, 6, 6));
    assert(terr_pattern_on(TERR_PAT_WATER, 4, 0) && terr_pattern_on(TERR_PAT_WATER, 8, 4) && !terr_pattern_on(TERR_PAT_WATER, 4, 2));
    assert(!terr_pattern_on(TERR_PAT_NONE, 3, 3));
    assert(terr_class_pattern(TERR_UNKNOWN) == TERR_PAT_HIGH && terr_class_rgb(TERR_UNKNOWN) == 0xff94ffu);
    assert(terr_class_pattern(TERR_GREEN_SOLID) == TERR_PAT_SOLID && terr_class_rgb(TERR_YELLOW_LOW) == 0xffff32u);
}

static void test_geometry(void)
{
    TerrGeometry g;
    float x, y;
    double lat, lon;
    assert(!terr_geometry(4, 40, &g) && !terr_geometry(3, 0, &g) && !terr_geometry(3, NAN, &g));
    assert(terr_geometry(3, 40, &g) && g.arc && g.cols == 95 && g.rows == 63 && near(g.px_per_nm, 12.45, .01));
    terr_cell_center(&g, 0, 0, &x, &y); assert(x == 10 && y == 132);
    assert(terr_in_clip(1, 384, 300) && terr_in_clip(1, 384, 130) && !terr_in_clip(1, 384, 126));
    assert(!terr_in_clip(1, 10, 200) && !terr_in_clip(1, 700, 600) && terr_in_clip(1, 700, 560) && !terr_in_clip(1, 384, 630));
    assert(terr_in_clip(1, 600, 590) && !terr_in_clip(1, 640, 600));
    assert(terr_geometry(2, 40, &g) && !g.arc && g.cols == 85 && g.rows == 32 && near(g.px_per_nm, 12.5, 1e-4));
    assert(terr_in_clip(0, 384, 140) && !terr_in_clip(0, 100, 140) && terr_in_clip(0, 100, 200) && !terr_in_clip(0, 384, 390));
    assert(!terr_in_clip(0, 40, 300) && !terr_in_clip(0, 730, 300));
    /* Pointer: ARC 40 NM, heading 090 at 0 N 0 E; 249 px above the aircraft = 20 NM east. */
    assert(terr_geometry(3, 40, &g));
    assert(terr_pointer_target(&g, 0, 0, 90, 384, 626 - 249, &lat, &lon));
    assert(near(lat, 0, 1e-4) && near(lon, 20.0 / 60.0, 2e-3));
    assert(!terr_pointer_target(&g, 0, 0, 90, 384, 700, &lat, &lon));
    /* ROSE 40 NM: 250 px = 20 NM. North-up, 125 px left of the aircraft = 10 NM west. */
    assert(terr_geometry(0, 40, &g));
    assert(terr_pointer_target(&g, 10, 20, 0, 384 - 125, 383, &lat, &lon) && near(lon, 20 - 10.0 / 60 / cos(10 * TERR_PI / 180), 2e-3));
}

static void histogram(int *hist, const float *elevations, int count)
{
    int k;
    memset(hist, 0, TERR_HIST_BINS * sizeof(int));
    for (k = 0; k < count; ++k) ++hist[terr_hist_bin(elevations[k])];
}

static void test_bands(void)
{
    int hist[TERR_HIST_BINS], k;
    float uniform[1001];
    TerrBands b;
    TerrThresholds t;
    char upper[16], lower[16];
    for (k = 0; k <= 1000; ++k) uniform[k] = 9.0f * k;   /* 0 .. 9000 ft */
    histogram(hist, uniform, 1001);
    /* Normal mode at 5000 ft, gear up (FBW warning/green thresholds). */
    terr_bands(hist, 5000, 500, -500, &b);
    assert(!b.peaks && !b.empty && b.min_elev == 0 && b.max_elev == 9100);
    assert(b.ldy == 4500 && b.hdy == 6000 && b.hdr == 7000 && b.ldg == 3000 && b.hdg == 4000);
    assert(terr_classify(TERR_NODE_LAND, 7100, &b) == TERR_RED_HIGH);
    assert(terr_classify(TERR_NODE_LAND, 7000, &b) == TERR_RED_HIGH);
    assert(terr_classify(TERR_NODE_LAND, 6200, &b) == TERR_YELLOW_HIGH);
    assert(terr_classify(TERR_NODE_LAND, 4600, &b) == TERR_YELLOW_LOW);
    assert(terr_classify(TERR_NODE_LAND, 4200, &b) == TERR_GREEN_HIGH);
    assert(terr_classify(TERR_NODE_LAND, 3500, &b) == TERR_GREEN_LOW);
    assert(terr_classify(TERR_NODE_LAND, 2500, &b) == TERR_NONE);
    assert(terr_classify(TERR_NODE_WATER, 0, &b) == TERR_WATER && terr_classify(TERR_NODE_MISSED, 0, &b) == TERR_UNKNOWN);
    assert(terr_classify(TERR_NODE_UNSAMPLED, 9000, &b) == TERR_NONE);
    terr_thresholds(&b, &t);
    assert(!t.peaks && t.min_elev == 3000 && t.min_mode == TERR_LEVEL_GREEN && t.max_elev == 9100 && t.max_mode == TERR_LEVEL_RED);
    assert(terr_threshold_text(&t, upper, lower) && !strcmp(upper, " 92") && !strcmp(lower, " 30"));
    /* Gear down: the yellow band starts 250 ft below the aircraft. */
    terr_bands(hist, 5000, 250, -500, &b);
    assert(b.ldy == 4750 && terr_classify(TERR_NODE_LAND, 4600, &b) == TERR_GREEN_HIGH);
    /* Descent look-ahead: 2000 ft/min lowers the reference by 1000 ft (30 s). */
    assert(terr_reference_altitude(5000, -2000) == 4000 && terr_reference_altitude(5000, -900) == 5000);
    terr_bands(hist, terr_reference_altitude(5000, -2000), 500, -500, &b);
    assert(terr_classify(TERR_NODE_LAND, 6200, &b) == TERR_RED_HIGH);
    /* Runway cut-off: black below it; the statistics start at it, so the low-density green band begins
     * 200 ft above the lowest terrain counted (FBW), and the box shows the higher of the two. */
    terr_bands(hist, 5000, 500, 3600, &b);
    assert(b.min_elev == 3600 && b.ldg == 3800);
    assert(terr_classify(TERR_NODE_LAND, 3500, &b) == TERR_NONE && terr_classify(TERR_NODE_LAND, 3700, &b) == TERR_NONE);
    assert(terr_classify(TERR_NODE_LAND, 3900, &b) == TERR_GREEN_LOW);
    terr_thresholds(&b, &t); assert(t.min_elev == 3800);
    terr_bands(hist, 9000, 500, 6000, &b); terr_thresholds(&b, &t);
    assert(!b.peaks && b.ldg == 7000 && t.min_elev == 7000);
    terr_bands(hist, 5000, 500, 2000, &b); terr_thresholds(&b, &t);
    assert(b.ldg == 3000 && t.min_elev == 3000 && terr_classify(TERR_NODE_LAND, 1900, &b) == TERR_NONE);
    /* Highest terrain only in the yellow band: yellow maximum. */
    for (k = 0; k <= 1000; ++k) uniform[k] = 6.5f * k;   /* 0 .. 6500 */
    histogram(hist, uniform, 1001);
    terr_bands(hist, 5000, 500, -500, &b); terr_thresholds(&b, &t);
    assert(!b.peaks && t.max_mode == TERR_LEVEL_YELLOW && t.max_elev == 6600);
    /* Terrain all near the aircraft: the green bands are squeezed out, the lowest value is yellow. */
    for (k = 0; k <= 1000; ++k) uniform[k] = 4800.0f + 2.2f * k;   /* 4800 .. 7000 */
    histogram(hist, uniform, 1001);
    terr_bands(hist, 5000, 500, -500, &b); terr_thresholds(&b, &t);
    assert(!b.peaks && t.min_mode == TERR_LEVEL_YELLOW && t.max_mode == TERR_LEVEL_RED);
    /* Peaks mode: far above the terrain, bands from the histogram. */
    for (k = 0; k <= 1000; ++k) uniform[k] = 10.0f * k;   /* 0 .. 10000 */
    histogram(hist, uniform, 1001);
    terr_bands(hist, 35000, 500, -500, &b);
    assert(b.peaks && b.min_elev == 0 && b.max_elev == 10100);
    assert(near(b.p_low, 5050, .5) && near(b.p_high, 6565, .5) && near(b.p_solid, 9595, .5));
    assert(terr_classify(TERR_NODE_LAND, 9800, &b) == TERR_GREEN_SOLID);
    assert(terr_classify(TERR_NODE_LAND, 8000, &b) == TERR_GREEN_HIGH);
    assert(terr_classify(TERR_NODE_LAND, 6000, &b) == TERR_GREEN_LOW);
    assert(terr_classify(TERR_NODE_LAND, 4000, &b) == TERR_NONE);
    assert(terr_classify(TERR_NODE_LAND, 4000, &b) == TERR_NONE);
    terr_thresholds(&b, &t);
    assert(t.peaks && t.min_elev == 5050 && t.max_elev == 10100 && t.min_mode == TERR_LEVEL_GREEN && t.max_mode == TERR_LEVEL_GREEN);
    assert(terr_threshold_text(&t, upper, lower) && !strcmp(upper, "102") && !strcmp(lower, " 50"));
    /* Peaks mode switches to normal within 500 ft (250 gear down) of the highest terrain in view. */
    terr_bands(hist, 10100 + 499, 500, -500, &b); assert(!b.peaks);
    terr_bands(hist, 10100 + 501, 500, -500, &b); assert(b.peaks);
    terr_bands(hist, 10100 + 300, 250, -500, &b); assert(b.peaks);
    /* Nothing in view: the TERR box is hidden. */
    memset(hist, 0, sizeof(hist));
    terr_bands(hist, 5000, 500, -500, &b); terr_thresholds(&b, &t);
    assert(b.empty && t.min_elev == -1 && t.max_elev == -1 && !terr_threshold_text(&t, upper, lower));
}

static void fill_level(TerrLevel *L, double lat, double lon)
{
    MockCtx m = {0};
    TerrAircraft ac = {lat, lon, 90, 5000, 0, -500, 0};
    TerrProbeStats st;
    TerrLevel *levels[1] = {L};
    int frames = 0;
    do {
        terr_probe_levels(levels, 1, &ac, 0.0, 1e9, 100000, mock_probe, mock_clock, &m, &st);
        ++frames;
    } while (st.nodes > 0 && frames < 100);
}

static void test_scheduler(void)
{
    TerrLevel *L = calloc(1, sizeof(TerrLevel));
    const TerrSpiral *s = terr_spiral();
    MockCtx m = {0};
    TerrAircraft ac = {45.0, 7.0, 90.0f, 5000, 0, -500, 0};
    TerrLevel *levels[1] = {L};
    TerrProbeStats st;
    int i, j, k, first_i = -1, first_j = -1, count = 0, behind_seen = 0;
    double fi, fj, previous = 0, lat, lon, lat2, lon2;
    float e;
    assert(L);
    assert(s->count > 15000 && s->di[0] == 0 && s->dj[0] == 0 && s->ring_start[0] == 0);
    for (k = 1; k < s->count; ++k)
        assert(s->di[k] * s->di[k] + s->dj[k] * s->dj[k] >= s->di[k - 1] * s->di[k - 1] + s->dj[k - 1] * s->dj[k - 1]);
    terr_level_init(L, 40, ac.lat, ac.lon, 0);
    assert(near(L->spacing_nm, 0.64, 1e-9) && L->sub == 1);
    terr_level_fraction(L, ac.lat, ac.lon, &fi, &fj);
    assert(fabs(fi - TERR_GRID_N / 2) < 1.01 && fabs(fj - TERR_GRID_N / 2) < 1.01);
    /* Forward half first, nearest first; then the rest; the whole spiral exactly once. */
    while (terr_level_next(L, ac.lat, ac.lon, ac.heading_true, 0, 100000, &i, &j)) {
        double ahead = (i - floor(fi + .5)) * 1.0, d = hypot(i - floor(fi + .5), j - floor(fj + .5));
        if (count == 0) { first_i = i; first_j = j; }
        if (ahead < -1.5) behind_seen = 1;
        else assert(!behind_seen);
        if (!behind_seen) { assert(d >= previous - 1e-9); previous = d; }
        terr_level_store(L, i, j, TERR_NODE_LAND, 100, 0);
        ++count;
    }
    assert(first_i == (int)floor(fi + .5) && first_j == (int)floor(fj + .5) && behind_seen);
    assert(count == s->count && L->sampled == count);
    /* Done: nothing until a rescan; a missed node is probed again after TERR_RETRY_S. */
    assert(!terr_level_next(L, ac.lat, ac.lon, ac.heading_true, 5, 100000, &i, &j));
    terr_level_store(L, first_i + 3, first_j, TERR_NODE_MISSED, 0, 5);
    assert(L->missed == 1 && !terr_level_next(L, ac.lat, ac.lon, ac.heading_true, 16, 100000, &i, &j));
    /* (the scan restarted at 16 s and found it not yet due; it is due at 20 s, rescanned after 26 s) */
    assert(!terr_level_next(L, ac.lat, ac.lon, ac.heading_true, 21, 100000, &i, &j));
    assert(terr_level_next(L, ac.lat, ac.lon, ac.heading_true, 27, 100000, &i, &j) && i == first_i + 3 && j == first_j);
    /* Cache follows the aircraft by integer shifts and keeps samples. */
    terr_level_node(L, first_i + 10, first_j + 10, &lat, &lon);
    terr_level_store(L, first_i + 10, first_j + 10, TERR_NODE_LAND, 4321, 27);
    assert(!terr_level_follow(L, ac.lat + 30 * L->dlat, ac.lon, 28));
    assert(terr_level_sample(L, lat + 0.01 * L->dlat, lon + 0.01 * L->dlon, &e) == TERR_NODE_LAND && e == 4321);
    terr_level_fraction(L, ac.lat + 30 * L->dlat, ac.lon, &fi, &fj);
    assert(fabs(fj - TERR_GRID_N / 2) < 1.01);
    terr_level_node(L, (int)fi, TERR_GRID_N - 1, &lat2, &lon2);
    assert(terr_level_sample(L, lat2 + 0.01 * L->dlat, lon2 + 0.01 * L->dlon, &e) == TERR_NODE_UNSAMPLED);
    /* A jump far away rebuilds the grid. */
    assert(terr_level_follow(L, ac.lat + 4.0, ac.lon, 29) && L->sampled == 0);

    /* Frame budget with a simulated probe cost: never more than one node beyond the budget. */
    {
        const double costs[] = {1e-6, 5e-6, 20e-6, 100e-6};
        int c;
        for (c = 0; c < 4; ++c) {
            terr_level_init(L, 40, ac.lat, ac.lon, 0);
            m.now = 0; m.cost_s = costs[c]; m.real_cost_s = 0; m.probes = 0;
            terr_probe_levels(levels, 1, &ac, 0, 400e-6, 100000, mock_probe, mock_clock, &m, &st);
            assert(st.elapsed_s <= 400e-6 + costs[c] + 1e-12 && st.elapsed_s >= 400e-6 - 1e-12);
            assert(st.nodes == (int)ceil(400e-6 / costs[c] - 1e-9) && st.probes == st.nodes);
            printf("terrain budget: simulated probe %5.0f us -> %3d probes in a 0.4 ms frame, frame %.0f us\n",
                   costs[c] * 1e6, st.probes, st.elapsed_s * 1e6);
        }
        /* Coarse level (160 NM, spacing 2.56 NM): four probes per node. */
        terr_level_init(L, 160, ac.lat, ac.lon, 0);
        m.now = 0; m.cost_s = 5e-6; m.probes = 0;
        terr_probe_levels(levels, 1, &ac, 0, 400e-6, 100000, mock_probe, mock_clock, &m, &st);
        assert(L->sub == 4 && st.probes == 4 * st.nodes && st.elapsed_s <= 400e-6 + 4 * 5e-6 + 1e-12);
    }
    /* Real time: a busy-waiting 5 us probe, then the scheduler's own overhead with a free probe. */
    {
        int frame;
        double worst = 0, own = 0;
        terr_level_init(L, 40, ac.lat, ac.lon, 0);
        m.real_cost_s = 5e-6; m.cost_s = 0;
        for (frame = 0; frame < 20; ++frame) {
            terr_probe_levels(levels, 1, &ac, 0, 400e-6, 100000, mock_probe, mock_clock, &m, &st);
            if (st.elapsed_s > worst) worst = st.elapsed_s;
        }
        assert(worst < 400e-6 + 200e-6);   /* one probe of overrun, plus scheduling noise */
        terr_level_init(L, 40, ac.lat, ac.lon, 0);
        m.real_cost_s = 1e-12;
        {
            double t0 = wall_clock();
            int total = 0;
            for (frame = 0; frame < 4; ++frame) {
                terr_probe_levels(levels, 1, &ac, 0, 1.0, 4000, mock_probe, mock_clock, &m, &st);
                total += st.nodes;
            }
            own = (wall_clock() - t0) / (total > 0 ? total : 1);
        }
        printf("terrain budget: real 5 us probe -> worst frame %.0f us of a 400 us budget; "
               "scheduler + store + synthetic terrain %.2f us per node\n", worst * 1e6, own * 1e6);
    }
    free(L);
}

static void test_display(void)
{
    TerrLevel *L = calloc(1, sizeof(TerrLevel));
    TerrDisplay *d = calloc(1, sizeof(TerrDisplay));
    TerrAircraft ac = {45.0, 7.0, 90.0f, 5000, 0, -500, 0};
    int row, col, counts[TERR_CLASS_COUNT] = {0}, revealed;
    double t;
    char *rle = malloc(65536);
    uint8_t (*decoded)[TERR_COLS_MAX] = calloc(TERR_ROWS_MAX, TERR_COLS_MAX);
    assert(L && d && rle && decoded);
    terr_display_reset(d);
    terr_level_init(L, 40, ac.lat, ac.lon, 0);
    fill_level(L, ac.lat, ac.lon);
    assert(L->sampled > 15000);
    /* Inactive: nothing. */
    assert(!terr_display_update(d, 0, 3, 40, &ac, L, 0, 100000) && !d->on);
    /* Activation: geometry, frame built and classified at once (large budget), TERR box shown. */
    assert(terr_display_update(d, 1, 3, 40, &ac, L, 10.0, 100000) && d->on && d->phase == TERR_TRANSITION);
    assert(d->shown.max_elev >= 9000 && d->shown.max_mode == TERR_LEVEL_RED && d->visible_cells > 3000);
    assert(d->data_cells == d->visible_cells);
    /* Scan line: bottom rows first (39 px per 40 ms from y 720), top row at 0.6 s. */
    for (row = 0; row < d->geo.rows; ++row) for (col = 0; col < d->geo.cols; ++col) assert(d->cls[row][col] == TERR_NONE);
    terr_display_update(d, 1, 3, 40, &ac, L, 10.10, 100000);
    revealed = 0;
    for (row = 0; row < d->geo.rows; ++row) if (memcmp(d->cls[row], d->next[row], d->geo.cols) == 0) ++revealed;
    {   /* ticks 3 -> border 603: rows with centre >= 603 */
        int expected = 0;
        for (row = 0; row < d->geo.rows; ++row) expected += 128 + 8 * row + 4 >= 720 - 39 * 3 || !memcmp(d->next[row], d->cls[row], d->geo.cols);
        assert(revealed == expected && revealed < d->geo.rows);
    }
    for (t = 10.12; t < 10.6; t += 0.04) terr_display_update(d, 1, 3, 40, &ac, L, t, 100000);
    assert(d->phase == TERR_TRANSITION);
    terr_display_update(d, 1, 3, 40, &ac, L, 10.605, 100000);
    assert(d->phase == TERR_IDLE && !memcmp(d->cls, d->next, sizeof(d->cls)));
    for (row = 0; row < d->geo.rows; ++row) for (col = 0; col < d->geo.cols; ++col) ++counts[d->cls[row][col]];
    assert(counts[TERR_RED_HIGH] > 0 && counts[TERR_YELLOW_HIGH] > 0 && counts[TERR_YELLOW_LOW] > 0);
    assert(counts[TERR_GREEN_HIGH] > 0 && counts[TERR_GREEN_LOW] > 0);
    /* Next cycle 1.1 s after the previous start. */
    terr_display_update(d, 1, 3, 40, &ac, L, 11.05, 100000); assert(d->phase == TERR_IDLE);
    terr_display_update(d, 1, 3, 40, &ac, L, 11.11, 10); assert(d->phase == TERR_BUILD);   /* budgeted build */
    while (d->phase == TERR_BUILD) terr_display_update(d, 1, 3, 40, &ac, L, 11.12, 1000);
    /* Run-length string round trip. */
    {
        int length = terr_rle_encode(d, rle, 65536);
        assert(length > 0 && terr_rle_decode(rle, decoded, d->geo.rows, d->geo.cols) == d->geo.rows * d->geo.cols);
        for (row = 0; row < d->geo.rows; ++row) assert(!memcmp(decoded[row], d->cls[row], d->geo.cols));
        printf("terrain display: ARC 40 NM raster %dx%d, %d cells in view, RLE %d bytes\n", d->geo.cols, d->geo.rows,
               d->visible_cells, length);
    }
    assert(terr_rle_encode(d, rle, 8) == -1 && terr_rle_decode("z", decoded, 2, 2) == -1 && terr_rle_decode("a9", decoded, 2, 2) == -1);
    /* Range / mode change: cleared, TERR box hidden until the new frame. */
    assert(terr_display_update(d, 1, 2, 40, &ac, L, 12.0, 0) && d->on && d->geo.mode == 2 && d->shown.max_elev == -1);
    for (row = 0; row < TERR_ROWS_MAX; ++row) for (col = 0; col < TERR_COLS_MAX; ++col) assert(!d->cls[row][col]);
    /* Peaks mode at FL350 over the same terrain: greens only, box green. */
    ac.altitude_ft = 35000;
    terr_display_update(d, 1, 3, 40, &ac, L, 20.0, 100000);
    terr_display_update(d, 1, 3, 40, &ac, L, 20.7, 100000);
    memset(counts, 0, sizeof(counts));
    for (row = 0; row < d->geo.rows; ++row) for (col = 0; col < d->geo.cols; ++col) ++counts[d->cls[row][col]];
    assert(d->shown.peaks && d->shown.max_mode == TERR_LEVEL_GREEN && counts[TERR_GREEN_SOLID] > 0);
    assert(!counts[TERR_RED_HIGH] && !counts[TERR_YELLOW_HIGH] && !counts[TERR_YELLOW_LOW]);
    /* Water west and missing scenery north (ROSE, heading north, 160 NM). */
    {
        TerrLevel *W = calloc(1, sizeof(TerrLevel));
        TerrAircraft north = {46.0, 6.8, 0.0f, 5000, 0, -500, 0};
        assert(W);
        terr_display_reset(d);
        terr_level_init(W, 160, north.lat, north.lon, 0);
        fill_level(W, north.lat, north.lon);
        terr_display_update(d, 1, 2, 160, &north, W, 30.0, 100000);
        terr_display_update(d, 1, 2, 160, &north, W, 30.7, 100000);
        memset(counts, 0, sizeof(counts));
        for (row = 0; row < d->geo.rows; ++row) for (col = 0; col < d->geo.cols; ++col) ++counts[d->cls[row][col]];
        assert(counts[TERR_WATER] > 0 && counts[TERR_UNKNOWN] > 0);
        free(W);
    }
    /* Pixel rows: pattern anchored at the image origin and clipped. */
    {
        unsigned char *rgba = malloc((size_t)TERR_COLS_MAX * TERR_CELL_PX * 4 * TERR_CELL_PX);
        assert(rgba);
        memset(d->cls, 0, sizeof(d->cls));
        d->cls[10][20] = TERR_GREEN_SOLID; d->cls[10][21] = TERR_RED_HIGH; d->cls[0][0] = TERR_GREEN_SOLID;
        terr_render_row(d, 0, rgba, (size_t)d->geo.cols * TERR_CELL_PX * 4);
        assert(rgba[3] == 0);                                                                  /* outside the ROSE clip */
        terr_render_row(d, 10, rgba, (size_t)d->geo.cols * TERR_CELL_PX * 4);
        assert(rgba[(20 * 8) * 4 + 1] == 255 && rgba[(20 * 8) * 4 + 3] == 255);              /* solid green */
        /* high density, image row 80 = tile row 2 "##...#": x 168 (tile 0) on, x 170 (tile 2) off */
        assert(rgba[(21 * 8) * 4 + 0] == 255 && rgba[(21 * 8) * 4 + 3] == 255 && rgba[(21 * 8 + 2) * 4 + 3] == 0);
        free(rgba);
    }
    free(decoded); free(rle); free(d); free(L);
}

static void test_build_cost(void)
{
    TerrLevel *L = calloc(1, sizeof(TerrLevel));
    TerrDisplay *d = calloc(1, sizeof(TerrDisplay));
    TerrAircraft ac = {45.0, 7.0, 90.0f, 5000, 0, -500, 0};
    double t0, build;
    int k;
    assert(L && d);
    terr_level_init(L, 40, ac.lat, ac.lon, 0);
    fill_level(L, ac.lat, ac.lon);
    terr_display_reset(d);
    t0 = wall_clock();
    for (k = 0; k < 20; ++k) { terr_display_reset(d); terr_display_update(d, 1, 3, 40, &ac, L, k * 2.0, 100000); }
    build = (wall_clock() - t0) / 20;
    printf("terrain display: whole ARC frame (lattice, %d cells, classes, box) %.0f us; built over frames of 2048 cells\n",
           d->geo.rows * d->geo.cols, build * 1e6);
    assert(build < 5e-3);
    free(d); free(L);
}

static void emit(const char *path)
{
    TerrLevel *L = calloc(1, sizeof(TerrLevel));
    TerrDisplay *d = calloc(1, sizeof(TerrDisplay));
    TerrAircraft ac = {45.0, 7.0, 70.0f, 5000, 0, -500, 0};
    unsigned char *image = calloc((size_t)768 * 768, 3), *row = malloc((size_t)TERR_COLS_MAX * 8 * 4 * 8);
    FILE *f;
    int r, y, x;
    assert(L && d && image && row);
    terr_level_init(L, 40, ac.lat, ac.lon, 0);
    fill_level(L, ac.lat, ac.lon);
    terr_display_reset(d);
    terr_display_update(d, 1, 3, 40, &ac, L, 0, 100000);
    terr_display_update(d, 1, 3, 40, &ac, L, 0.7, 100000);
    for (r = 0; r < d->geo.rows; ++r) {
        terr_render_row(d, r, row, (size_t)d->geo.cols * 8 * 4);
        for (y = 0; y < 8; ++y) for (x = 0; x < d->geo.cols * 8; ++x) {
            int X = d->geo.x0 + x, Y = d->geo.y0 + r * 8 + y;
            const unsigned char *p = row + ((size_t)y * d->geo.cols * 8 + x) * 4;
            if (X < 768 && Y < 768 && p[3]) memcpy(image + ((size_t)Y * 768 + X) * 3, p, 3);
        }
    }
    f = fopen(path, "wb");
    assert(f);
    fprintf(f, "P6\n768 768\n255\n");
    fwrite(image, 1, (size_t)768 * 768 * 3, f);
    fclose(f);
    free(row); free(image); free(d); free(L);
}

int main(int argc, char **argv)
{
    if (argc == 3 && !strcmp(argv[1], "--emit")) { emit(argv[2]); return 0; }
    test_patterns();
    test_geometry();
    test_bands();
    test_scheduler();
    test_display();
    test_build_cost();
    puts("TERRAIN core: FBW dot patterns, clip geometry, EGPWS bands, peaks mode, TERR box, scheduler/budget, "
         "cache shifts, scan-line sweep, RLE and pointer target passed");
    return 0;
}
