/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * TERRAIN on ND (A380 TAWS terrain display), XPLM-free core (docs/ND_TERRAIN.md):
 *   - TerrLevel: a cached latitude/longitude grid of terrain elevations around the aircraft, one per
 *     selected ND range (node spacing = 8 ND pixels), filled by the X-Plane adapter with terrain probes
 *     chosen by a nearest-first, forward-first scheduler (terr_level_next) and kept by integer shifts as
 *     the aircraft moves;
 *   - TerrDisplay: the ND raster (8x8 pixel cells of the 768x768 ND map, FBW A380 image origin) built
 *     from a level, coloured into EGPWS classes and revealed by the A380 scan-line sweep;
 *   - the EGPWS colour bands, peaks mode and TERR box thresholds; dot patterns; a run-length string for
 *     the web ND; pixel rows for the native renderers.
 *
 * Ported (GPL-3.0) from FlyByWire SimBridge, github.com/flybywiresim/simbridge @ f5932323,
 * apps/server/src/terrain:
 *   processing/gpu/rendering/navigationdisplay.ts  - normal-mode band thresholds, peaks thresholds,
 *                                                     the per-pixel colour decision and colours;
 *   processing/navigationdisplayrenderer.ts        - histogram constants, 85th/95th percentiles,
 *                                                     TERR min/max threshold modes (analyzeMetadata),
 *                                                     scan-line transition (A380 "ScanlineMode");
 *   processing/gpu/patterns/scanlinemode.ts        - the dot pattern map, which is exactly the three
 *                                                     periodic 3x3-dot tiles of terr_pattern_on;
 *   processing/generic/constants.ts                - A380X image origin (y 128) and transition times.
 * Colour meanings: Honeywell MK V/MK VII EGPWS Pilot's Guide 060-4241-000 Rev H, TAD colour table.
 * Map geometry: the FlyByWire A380X ND bundle (ARC 498 px = range, aircraft at (384, 626); ROSE 250 px
 * = range / 2 about (384, 384); clip paths arc-mode-wx-terr-clip and rose-mode-wx-terr-clip).
 */
#ifndef A380X_TERRAIN_H
#define A380X_TERRAIN_H
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TERR_PI 3.14159265358979323846

/* ---- classes, patterns, colours ------------------------------------------------------------- */
enum {
    TERR_NONE = 0,                       /* black: no significant terrain, below cut-off, not sampled */
    TERR_GREEN_LOW, TERR_GREEN_HIGH, TERR_GREEN_SOLID,
    TERR_YELLOW_LOW, TERR_YELLOW_HIGH, TERR_RED_HIGH,
    TERR_WATER,                          /* sea level, low-density cyan */
    TERR_UNKNOWN,                        /* no terrain data (probe missed: scenery not loaded), magenta */
    TERR_CAUTION, TERR_WARNING,          /* solid yellow / red threat areas (no TAWS alerting feeds them yet) */
    TERR_CLASS_COUNT
};
enum { TERR_PAT_NONE, TERR_PAT_LOW, TERR_PAT_HIGH, TERR_PAT_WATER, TERR_PAT_SOLID };
/* FBW TerrainLevelMode (PeaksMode 0, Warning 1, Caution 2): the TERR box digit colours green/yellow/red. */
enum { TERR_LEVEL_GREEN = 0, TERR_LEVEL_YELLOW = 1, TERR_LEVEL_RED = 2 };
enum { TERR_NODE_UNSAMPLED = 0, TERR_NODE_LAND, TERR_NODE_WATER, TERR_NODE_MISSED };

static inline int terr_class_pattern(int c)
{
    switch (c) {
    case TERR_GREEN_LOW: case TERR_YELLOW_LOW: return TERR_PAT_LOW;
    case TERR_GREEN_HIGH: case TERR_YELLOW_HIGH: case TERR_RED_HIGH: case TERR_UNKNOWN: return TERR_PAT_HIGH;
    case TERR_WATER: return TERR_PAT_WATER;
    case TERR_GREEN_SOLID: case TERR_CAUTION: case TERR_WARNING: return TERR_PAT_SOLID;
    default: return TERR_PAT_NONE;
    }
}

/* FBW SimBridge colours (0xRRGGBB). */
static inline uint32_t terr_class_rgb(int c)
{
    switch (c) {
    case TERR_GREEN_LOW: case TERR_GREEN_HIGH: case TERR_GREEN_SOLID: return 0x00ff00u;
    case TERR_YELLOW_LOW: case TERR_YELLOW_HIGH: case TERR_CAUTION: return 0xffff32u;
    case TERR_RED_HIGH: case TERR_WARNING: return 0xff0000u;
    case TERR_WATER: return 0x00ffffu;
    case TERR_UNKNOWN: return 0xff94ffu;
    default: return 0u;
    }
}

/* FBW A380 (scan-line) dot patterns, in image pixels from the image origin: period P, two 3x3 dots at
 * (c, P-1) and (P-1, c) with wrap-around. Low density P 13 (18/169 = 10.7 %), high density P 6 (50 %),
 * water P 9 (18/81 = 22.2 %): identical to every pixel of scanlinemode.ts. */
static inline int terr_pattern_on(int pattern, int x, int y)
{
    int p, c;
    switch (pattern) {
    case TERR_PAT_SOLID: return 1;
    case TERR_PAT_LOW: p = 13; c = 5; break;
    case TERR_PAT_HIGH: p = 6; c = 2; break;
    case TERR_PAT_WATER: p = 9; c = 4; break;
    default: return 0;
    }
    x %= p; y %= p;
    if (x < 0) x += p;
    if (y < 0) y += p;
    return ((x - c + p) % p < 3 && (y + 1) % p < 3) || ((x + 1) % p < 3 && (y - c + p) % p < 3);
}

/* ---- ND raster geometry ---------------------------------------------------------------------- */
#define TERR_CELL_PX 8
#define TERR_COLS_MAX 96
#define TERR_ROWS_MAX 64
#define TERR_LATTICE 4                                   /* exact positions every 4 cells, interpolated */
#define TERR_LAT_COLS (TERR_COLS_MAX / TERR_LATTICE + 2)
#define TERR_LAT_ROWS (TERR_ROWS_MAX / TERR_LATTICE + 2)
#define TERR_IMAGE_Y0 128                                /* FBW NavigationDisplayMapStartOffsetY */
#define TERR_IMAGE_BOTTOM 720                            /* 128 + A380X map height 592 */

typedef struct {
    int valid, mode, arc;      /* EFIS ND mode 0 ROSE LS, 1 ROSE VOR, 2 ROSE NAV, 3 ARC */
    float range_nm;            /* selected EFIS range */
    int x0, y0, cols, rows;    /* raster origin (ND pixels, y down) and size in cells */
    float ac_x, ac_y;          /* aircraft reference of the map projection */
    float px_per_nm;
} TerrGeometry;

/* ARC: image x 6..762 (FBW width 756), aircraft (384, 626), 498 px = range (CanvasMap
 * set_map_pixel_radius / set_plane_y); rows to y 632 (the clip ends at 625). ROSE: image x 45..723
 * (width 678), aircraft (384, 384), 250 px = range / 2; rows to y 384 (forward half only). */
static inline int terr_geometry(int mode, float range_nm, TerrGeometry *g)
{
    memset(g, 0, sizeof(*g));
    if (mode < 0 || mode > 3 || !isfinite(range_nm) || !(range_nm > 0.0f)) return 0;
    g->valid = 1; g->mode = mode; g->arc = mode == 3; g->range_nm = range_nm; g->y0 = TERR_IMAGE_Y0;
    if (g->arc) {
        g->x0 = 6; g->cols = 95; g->rows = 63; g->ac_x = 384.0f; g->ac_y = 626.0f; g->px_per_nm = 498.0f / range_nm;
    } else {
        g->x0 = 45; g->cols = 85; g->rows = 32; g->ac_x = 384.0f; g->ac_y = 384.0f; g->px_per_nm = 500.0f / range_nm;
    }
    return 1;
}

/* FBW clip paths (ND pixels, y down). ARC "M0,312 a492,492 0 0 1 768,0 L768,562 L648,562 L591,625
 * L0,625 L0,312": the arc is centred on (384, 619.59). ROSE "M45,155 L282,155 a250,250 0 0 1 204,0
 * L723,155 L723,384 L45,384 L45,155": the arc is the rose ring about (384, 383.2). */
static inline int terr_in_clip(int arc, float x, float y)
{
    if (arc) {
        float dx = x - 384.0f, dy = y - 619.59f;
        if (x < 0.0f || x > 768.0f || y > 625.0f || dx * dx + dy * dy > 492.0f * 492.0f) return 0;
        if (x >= 648.0f) return y <= 562.0f;
        if (x > 591.0f) return y <= 562.0f + (648.0f - x) * (63.0f / 57.0f);
        return 1;
    }
    if (x < 45.0f || x > 723.0f || y > 384.0f) return 0;
    if (y >= 155.0f) return 1;
    if (x > 282.0f && x < 486.0f) {
        float dx = x - 384.0f, dy = y - 383.2f;
        return dx * dx + dy * dy <= 250.0f * 250.0f;
    }
    return 0;
}

static inline void terr_cell_center(const TerrGeometry *g, int row, int col, float *x, float *y)
{
    *x = (float)(g->x0 + TERR_CELL_PX * col) + 0.5f * TERR_CELL_PX;
    *y = (float)(g->y0 + TERR_CELL_PX * row) + 0.5f * TERR_CELL_PX;
}

static inline double terr_wrap180(double a)
{
    a = fmod(a + 180.0, 360.0);
    if (a < 0.0) a += 360.0;
    return a - 180.0;
}

/* Great-circle destination (degrees, nautical miles; the ND bundle's projection is great-circle too). */
static inline void terr_destination(double lat, double lon, double bearing_deg, double dist_nm, double *out_lat, double *out_lon)
{
    const double r = TERR_PI / 180.0;
    double d = dist_nm / 3440.065, p1 = lat * r, b = bearing_deg * r;
    double p2 = asin(fmin(1.0, fmax(-1.0, sin(p1) * cos(d) + cos(p1) * sin(d) * cos(b))));
    *out_lat = p2 / r;
    *out_lon = terr_wrap180(lon + atan2(sin(b) * sin(d) * cos(p1), cos(d) - sin(p1) * sin(p2)) / r);
}

/* ND pixel (y down) -> latitude/longitude for a heading-up map at (lat, lon) with true heading. */
static inline void terr_screen_latlon(const TerrGeometry *g, double lat, double lon, double heading_true,
                                      float x, float y, double *out_lat, double *out_lon)
{
    double e = (x - g->ac_x) / g->px_per_nm, n = (g->ac_y - y) / g->px_per_nm;
    terr_destination(lat, lon, heading_true + atan2(e, n) * 180.0 / TERR_PI, hypot(e, n), out_lat, out_lon);
}

/* ---- elevation cache --------------------------------------------------------------------------- */
typedef struct {
    double lat, lon;                   /* present position */
    float heading_true, altitude_ft, vs_fpm;
    float cutoff_ft;                   /* black below (normal mode): nearest runway elevation + 400 ft */
    int gear_down;
} TerrAircraft;

#define TERR_GRID_N 192
#define TERR_GRID_NODES (TERR_GRID_N * TERR_GRID_N)
#define TERR_NODES_PER_RANGE 62.5      /* node spacing = range / 62.5 NM = 8 ND pixels */
#define TERR_SPIRAL_R 70               /* probing radius in nodes (about 1.1 x range) */
#define TERR_RECENTER_NODES 24         /* shift the grid when the aircraft is this far from its centre */
#define TERR_RETRY_S 15.0              /* a missed node (no scenery) is probed again after this */
#define TERR_RESCAN_S 10.0             /* restart a finished scan this often (retries due) */
#define TERR_WATER_MAX_FT 50.0f        /* wet and lower than this: sea level water */

typedef struct {
    int count;
    int ring_start[TERR_SPIRAL_R + 2];     /* first entry with distance >= r nodes */
    int8_t di[(2 * TERR_SPIRAL_R + 1) * (2 * TERR_SPIRAL_R + 1)];
    int8_t dj[(2 * TERR_SPIRAL_R + 1) * (2 * TERR_SPIRAL_R + 1)];
} TerrSpiral;

/* Node offsets within TERR_SPIRAL_R, nearest first (counting sort on the squared distance). */
static inline const TerrSpiral *terr_spiral(void)
{
    static TerrSpiral s;
    static int counts[TERR_SPIRAL_R * TERR_SPIRAL_R + 2];
    if (!s.count) {
        const int r = TERR_SPIRAL_R, n2 = TERR_SPIRAL_R * TERR_SPIRAL_R;
        int di, dj, k, total = 0, ring = 0;
        memset(counts, 0, sizeof(counts));
        for (dj = -r; dj <= r; ++dj) for (di = -r; di <= r; ++di) if (di * di + dj * dj <= n2) ++counts[di * di + dj * dj + 1];
        for (k = 1; k <= n2 + 1; ++k) counts[k] += counts[k - 1];   /* counts[d2] = first index of d2 */
        for (dj = -r; dj <= r; ++dj) for (di = -r; di <= r; ++di) {
            int d2 = di * di + dj * dj;
            if (d2 > n2) continue;
            k = counts[d2]++;
            s.di[k] = (int8_t)di; s.dj[k] = (int8_t)dj; ++total;
        }
        s.count = total;
        for (k = 0; k < total && ring <= r + 1; ++k) {
            while (ring <= r + 1 && s.di[k] * s.di[k] + s.dj[k] * s.dj[k] >= ring * ring) s.ring_start[ring++] = k;
        }
        while (ring <= r + 1) s.ring_start[ring++] = total;
    }
    return &s;
}

typedef struct {
    int used;
    float range_nm;                    /* EFIS range this level serves */
    double spacing_nm, dlat, dlon, lat_ref, epoch, last_used;
    long base_i, base_j;               /* lattice index of node (0, 0): lat = (base_j + j) dlat */
    int sub;                           /* probes per node: 1, or 4 for spacing above 0.7 NM (max) */
    int sched_k, sched_pass, sched_ci, sched_cj, front_ring;
    float sched_heading;
    double sched_done_at;
    int sampled, missed;               /* nodes with a result / without terrain (diagnostics) */
    int16_t elev[TERR_GRID_NODES];     /* feet MSL */
    uint8_t kind[TERR_GRID_NODES];     /* TERR_NODE_* */
    uint16_t retry[TERR_GRID_NODES];   /* seconds after epoch: probe a missed node again */
} TerrLevel;

static inline void terr_level_init(TerrLevel *L, float range_nm, double lat, double lon, double now)
{
    double c = cos(lat * TERR_PI / 180.0);
    memset(L, 0, sizeof(*L));
    L->used = 1; L->range_nm = range_nm;
    L->spacing_nm = range_nm / TERR_NODES_PER_RANGE;
    L->dlat = L->spacing_nm / 60.0;
    L->lat_ref = lat;
    L->dlon = L->dlat / (c > 0.05 ? c : 0.05);
    L->base_j = (long)floor(lat / L->dlat) - TERR_GRID_N / 2;
    L->base_i = (long)floor(terr_wrap180(lon) / L->dlon) - TERR_GRID_N / 2;
    L->sub = L->spacing_nm > 0.7 ? 4 : 1;
    L->epoch = L->last_used = now;
    L->sched_ci = L->sched_cj = -100000;
}

static inline void terr_level_fraction(const TerrLevel *L, double lat, double lon, double *fi, double *fj)
{
    double centre = (double)(L->base_i + TERR_GRID_N / 2) * L->dlon;
    *fi = terr_wrap180(lon - centre) / L->dlon + TERR_GRID_N / 2;
    *fj = lat / L->dlat - (double)L->base_j;
}

static inline void terr_level_node(const TerrLevel *L, int i, int j, double *lat, double *lon)
{
    *lat = (double)(L->base_j + j) * L->dlat;
    *lon = terr_wrap180((double)(L->base_i + i) * L->dlon);
}

static inline void terr_level_recount(TerrLevel *L)
{
    int n;
    L->sampled = L->missed = 0;
    for (n = 0; n < TERR_GRID_NODES; ++n) {
        L->sampled += L->kind[n] != TERR_NODE_UNSAMPLED;
        L->missed += L->kind[n] == TERR_NODE_MISSED;
    }
}

/* New node (i, j) = old node (i + di, j + dj); nodes that enter the grid are unsampled. */
static inline void terr_level_shift(TerrLevel *L, int di, int dj)
{
    const int N = TERR_GRID_N, keep = N - abs(di), from = di > 0 ? di : 0, to = di < 0 ? -di : 0;
    int k;
    for (k = 0; k < N; ++k) {
        int j = dj >= 0 ? k : N - 1 - k, src = j + dj;
        size_t dst = (size_t)j * N;
        if (src < 0 || src >= N || keep <= 0) {
            memset(&L->elev[dst], 0, N * sizeof(L->elev[0]));
            memset(&L->kind[dst], 0, N * sizeof(L->kind[0]));
            memset(&L->retry[dst], 0, N * sizeof(L->retry[0]));
            continue;
        }
        memmove(&L->elev[dst + to], &L->elev[(size_t)src * N + from], keep * sizeof(L->elev[0]));
        memmove(&L->kind[dst + to], &L->kind[(size_t)src * N + from], keep * sizeof(L->kind[0]));
        memmove(&L->retry[dst + to], &L->retry[(size_t)src * N + from], keep * sizeof(L->retry[0]));
        if (di > 0) {
            memset(&L->elev[dst + keep], 0, di * sizeof(L->elev[0]));
            memset(&L->kind[dst + keep], 0, di * sizeof(L->kind[0]));
            memset(&L->retry[dst + keep], 0, di * sizeof(L->retry[0]));
        } else if (di < 0) {
            memset(&L->elev[dst], 0, -di * sizeof(L->elev[0]));
            memset(&L->kind[dst], 0, -di * sizeof(L->kind[0]));
            memset(&L->retry[dst], 0, -di * sizeof(L->retry[0]));
        }
    }
    L->base_i += di; L->base_j += dj;
    L->sched_ci = L->sched_cj = -100000; L->front_ring = 0;
    terr_level_recount(L);
}

/* Keep the aircraft within TERR_RECENTER_NODES of the grid centre. Returns 1 when the grid was
 * rebuilt (a large jump, or a latitude change that changes the east-west node spacing). */
static inline int terr_level_follow(TerrLevel *L, double lat, double lon, double now)
{
    double fi, fj;
    int di, dj;
    L->last_used = now;
    if (fabs(lat - L->lat_ref) > 3.0) { terr_level_init(L, L->range_nm, lat, lon, now); return 1; }
    terr_level_fraction(L, lat, lon, &fi, &fj);
    di = (int)floor(fi) - TERR_GRID_N / 2;
    dj = (int)floor(fj) - TERR_GRID_N / 2;
    if (abs(di) <= TERR_RECENTER_NODES && abs(dj) <= TERR_RECENTER_NODES) return 0;
    if (abs(di) >= TERR_GRID_N / 2 || abs(dj) >= TERR_GRID_N / 2) { terr_level_init(L, L->range_nm, lat, lon, now); return 1; }
    terr_level_shift(L, di, dj);
    return 0;
}

/* The next node to probe: nearest first, the half-plane ahead of the aircraft (pass 0) before
 * everything else still due (pass 1); unsampled nodes, and missed nodes whose retry time has come. At most max_checks
 * spiral entries are examined per call. A move of one node restarts the scan just inside the reached
 * frontier, a heading change of more than 30 degrees (or a finished scan after TERR_RESCAN_S) from
 * the centre. Returns 1 with (i, j), 0 when nothing is due within this call. */
static inline int terr_level_next(TerrLevel *L, double lat, double lon, float heading_true, double now,
                                  int max_checks, int *out_i, int *out_j)
{
    const TerrSpiral *s = terr_spiral();
    double fi, fj, h = heading_true * TERR_PI / 180.0;
    float sh = (float)sin(h), ch = (float)cos(h);
    int ci, cj;
    terr_level_fraction(L, lat, lon, &fi, &fj);
    ci = (int)floor(fi + 0.5); cj = (int)floor(fj + 0.5);
    if (fabs(terr_wrap180(heading_true - L->sched_heading)) > 30.0 || L->sched_ci < -1000 ||
        (L->sched_pass >= 2 && now - L->sched_done_at > TERR_RESCAN_S)) {
        L->sched_k = 0; L->sched_pass = 0; L->front_ring = 0; L->sched_heading = heading_true;
        L->sched_ci = ci; L->sched_cj = cj;
    } else if (ci != L->sched_ci || cj != L->sched_cj) {
        int back = L->front_ring - 2 - (abs(ci - L->sched_ci) > abs(cj - L->sched_cj) ? abs(ci - L->sched_ci) : abs(cj - L->sched_cj));
        L->sched_pass = 0; L->sched_k = s->ring_start[back > 0 ? back : 0];
        L->front_ring = back > 0 ? back : 0;
        L->sched_ci = ci; L->sched_cj = cj;
    }
    if (L->sched_pass >= 2) return 0;
    while (max_checks-- > 0) {
        int k, i, j, n, ahead;
        if (L->sched_k >= s->count) {
            if (++L->sched_pass >= 2) { L->sched_done_at = now; return 0; }
            L->sched_k = 0;
            continue;
        }
        k = L->sched_k++;
        ahead = s->di[k] * sh + s->dj[k] * ch >= -1.5f;
        if (L->sched_pass == 0) {
            int ring = (int)sqrtf((float)(s->di[k] * s->di[k] + s->dj[k] * s->dj[k]));
            if (ring > L->front_ring) L->front_ring = ring;
        }
        if (L->sched_pass == 0 && !ahead) continue;   /* pass 1 takes every node still due */
        i = ci + s->di[k]; j = cj + s->dj[k];
        if (i < 0 || j < 0 || i >= TERR_GRID_N || j >= TERR_GRID_N) continue;
        n = j * TERR_GRID_N + i;
        if (L->kind[n] == TERR_NODE_UNSAMPLED || (L->kind[n] == TERR_NODE_MISSED && now - L->epoch >= L->retry[n])) {
            *out_i = i; *out_j = j;
            return 1;
        }
    }
    return 0;
}

static inline void terr_level_store(TerrLevel *L, int i, int j, int kind, float elev_ft, double now)
{
    int n;
    double retry;
    if (i < 0 || j < 0 || i >= TERR_GRID_N || j >= TERR_GRID_N) return;
    n = j * TERR_GRID_N + i;
    L->sampled += (L->kind[n] == TERR_NODE_UNSAMPLED && kind != TERR_NODE_UNSAMPLED) -
                  (L->kind[n] != TERR_NODE_UNSAMPLED && kind == TERR_NODE_UNSAMPLED);
    L->missed += (kind == TERR_NODE_MISSED) - (L->kind[n] == TERR_NODE_MISSED);
    L->kind[n] = (uint8_t)kind;
    L->elev[n] = (int16_t)(isfinite(elev_ft) ? fmaxf(-1500.0f, fminf(32000.0f, roundf(elev_ft))) : 0.0f);
    retry = now - L->epoch + TERR_RETRY_S;
    L->retry[n] = (uint16_t)(kind == TERR_NODE_MISSED ? (retry < 65535.0 ? retry : 65535.0) : 0.0);
}

/* Combine one node's probe results: the highest hit; sea level water only if every hit was wet and
 * low (a lake surface above TERR_WATER_MAX_FT is terrain at that height); no hit is "missed". */
static inline int terr_probe_kind(int hits, int wet_hits, float highest_ft)
{
    if (hits <= 0) return TERR_NODE_MISSED;
    return wet_hits == hits && highest_ft < TERR_WATER_MAX_FT ? TERR_NODE_WATER : TERR_NODE_LAND;
}

/* One frame of probing: nodes from each level in turn (round robin) until `budget_s` of `clock` time
 * has elapsed or `max_nodes` nodes were probed. The clock is read before every node, so a frame
 * overruns its budget by at most one node (1 or 4 probes). A level of spacing above 0.7 NM probes four
 * points per node, at +-1/4 spacing, and keeps the highest (terr_probe_kind). */
typedef int (*TerrProbeFn)(void *ctx, double lat, double lon, float *elev_ft, int *wet);
typedef double (*TerrClockFn)(void *ctx);
typedef struct { int probes, hits, nodes; double elapsed_s; } TerrProbeStats;

static inline void terr_probe_levels(TerrLevel *const *levels, int count, const TerrAircraft *ac, double now,
                                     double budget_s, int max_nodes, TerrProbeFn probe, TerrClockFn clock,
                                     void *ctx, TerrProbeStats *st)
{
    const double start = clock(ctx);
    int idle = 0, turn = 0;
    memset(st, 0, sizeof(*st));
    while (count > 0 && idle < count && st->nodes < max_nodes && clock(ctx) - start < budget_s) {
        TerrLevel *L = levels[turn++ % count];
        int i, j, k, hits = 0, wet_hits = 0;
        float highest = -1e9f;
        double lat, lon;
        if (!L || !terr_level_next(L, ac->lat, ac->lon, ac->heading_true, now, 2048, &i, &j)) { ++idle; continue; }
        idle = 0;
        terr_level_node(L, i, j, &lat, &lon);
        for (k = 0; k < L->sub; ++k) {
            double plat = lat, plon = lon;
            float elev = 0.0f;
            int wet = 0;
            if (L->sub > 1) { plat += (k & 1 ? 0.25 : -0.25) * L->dlat; plon = terr_wrap180(plon + (k & 2 ? 0.25 : -0.25) * L->dlon); }
            ++st->probes;
            if (!probe(ctx, plat, plon, &elev, &wet) || !isfinite(elev)) continue;
            ++hits; wet_hits += wet != 0;
            if (elev > highest) highest = elev;
        }
        st->hits += hits;
        ++st->nodes;
        terr_level_store(L, i, j, terr_probe_kind(hits, wet_hits, highest), hits ? highest : 0.0f, now);
    }
    st->elapsed_s = clock(ctx) - start;
}

/* Elevation of the cell around (lat, lon): the four surrounding nodes, as the highest-in-cell value
 * FBW draws (its 8x8 pixel maximum). Unknown (a missed node) dominates, as FBW's UnknownElevation
 * marker does in that maximum; then land (highest), then water; unsampled nodes are ignored, and a
 * cell with only unsampled nodes is TERR_NODE_UNSAMPLED. */
static inline int terr_level_sample(const TerrLevel *L, double lat, double lon, float *elev_ft)
{
    double fi, fj;
    int i0, j0, a, b, land = 0, water = 0, missed = 0;
    float highest = -1e9f;
    terr_level_fraction(L, lat, lon, &fi, &fj);
    i0 = (int)floor(fi); j0 = (int)floor(fj);
    for (b = 0; b < 2; ++b) for (a = 0; a < 2; ++a) {
        int i = i0 + a, j = j0 + b, n;
        if (i < 0 || j < 0 || i >= TERR_GRID_N || j >= TERR_GRID_N) continue;
        n = j * TERR_GRID_N + i;
        switch (L->kind[n]) {
        case TERR_NODE_LAND: ++land; if (L->elev[n] > highest) highest = L->elev[n]; break;
        case TERR_NODE_WATER: ++water; break;
        case TERR_NODE_MISSED: ++missed; break;
        default: break;
        }
    }
    *elev_ft = land ? highest : 0.0f;
    return missed ? TERR_NODE_MISSED : land ? TERR_NODE_LAND : water ? TERR_NODE_WATER : TERR_NODE_UNSAMPLED;
}

/* ---- EGPWS bands (FBW SimBridge navigationdisplay.ts / navigationdisplayrenderer.ts) ---------- */
#define TERR_HIST_MIN_FT (-500)
#define TERR_HIST_BIN_FT 100
#define TERR_HIST_BINS 296            /* ceil((29040 + 500 + 1) / 100) */

typedef struct {
    int peaks, empty;
    float ref_alt, gear_offset, cutoff;
    float min_elev, max_elev, lower_pct, upper_pct;
    float ldy, hdy, hdr, ldg, hdg;     /* normal mode: low/high density yellow, red, low/high density green */
    float p_low, p_high, p_solid;      /* peaks mode: low/high density and solid green */
} TerrBands;

typedef struct { int min_elev, min_mode, max_elev, max_mode, peaks; } TerrThresholds;

static inline int terr_hist_bin(float elev_ft)
{
    int bin = (int)ceilf((elev_ft - (float)TERR_HIST_MIN_FT) / (float)TERR_HIST_BIN_FT);
    return bin < 0 ? 0 : bin >= TERR_HIST_BINS ? TERR_HIST_BINS - 1 : bin;
}

/* Reference altitude: the aircraft altitude, lowered by half the vertical speed (30 s of look-ahead)
 * when descending at 1000 ft/min or more (Honeywell: "biased upward proportional to the descent rate"). */
static inline float terr_reference_altitude(float altitude_ft, float vs_fpm)
{
    return altitude_ft + (isfinite(vs_fpm) && vs_fpm <= -1000.0f ? vs_fpm * 0.5f : 0.0f);
}

static inline void terr_bands(const int *hist, float ref_alt, float gear_offset, float cutoff, TerrBands *b)
{
    int bin, total = 0, lower = -1, upper = -1, min_bin = -1, max_bin = -1;
    int cut = (int)floorf((cutoff - (float)TERR_HIST_MIN_FT) / (float)TERR_HIST_BIN_FT);
    double cumulative = 0.0;
    float flat, half;
    memset(b, 0, sizeof(*b));
    b->ref_alt = ref_alt; b->gear_offset = gear_offset; b->cutoff = cutoff;
    if (cut < 0) cut = 0;
    if (cut > TERR_HIST_BINS - 1) cut = TERR_HIST_BINS - 1;
    for (bin = cut; bin < TERR_HIST_BINS; ++bin) total += hist[bin];
    for (bin = cut; bin < TERR_HIST_BINS; ++bin) {
        if (total > 0) {
            cumulative += (double)hist[bin] / total;
            if (lower < 0 && cumulative >= 0.85) lower = bin;
            if (upper < 0 && cumulative >= 0.95) upper = bin;
        }
        if (hist[bin] > 0) { if (min_bin < 0) min_bin = bin; max_bin = bin; }
    }
    if (upper < 0) upper = TERR_HIST_BINS - 1;
    b->empty = total == 0;
    b->lower_pct = (float)(lower * TERR_HIST_BIN_FT + TERR_HIST_MIN_FT);
    b->upper_pct = (float)(upper * TERR_HIST_BIN_FT + TERR_HIST_MIN_FT);
    b->min_elev = min_bin >= 0 ? (float)(min_bin * TERR_HIST_BIN_FT + TERR_HIST_MIN_FT) : -1.0f;
    b->max_elev = max_bin >= 0 ? (float)((max_bin + 1) * TERR_HIST_BIN_FT + TERR_HIST_MIN_FT) : 0.0f;
    flat = 100.0f - (b->max_elev - b->min_elev);
    half = b->max_elev * 0.5f;
    b->peaks = !(b->max_elev >= ref_alt - gear_offset);
    /* calculateNormalModeWarningThresholds / calculateNormalModeGreenThresholds */
    b->ldy = ref_alt - gear_offset;
    if (b->ldy <= b->min_elev) b->ldy = b->min_elev + 200.0f;
    b->hdy = ref_alt + 1000.0f;
    b->hdr = ref_alt + 2000.0f;
    b->ldg = ref_alt - 2000.0f <= b->min_elev ? b->min_elev + 200.0f : ref_alt - 2000.0f;
    b->hdg = ref_alt - 1000.0f <= b->min_elev ? b->min_elev + 200.0f : ref_alt - 1000.0f;
    if (flat >= 0.0f) {
        if (half <= b->lower_pct && b->ldg > half) b->ldg = half;
        else if (half > b->lower_pct && b->ldg > b->lower_pct) b->ldg = b->lower_pct;
    }
    /* calculatePeaksModeThresholds */
    b->p_low = fminf(b->lower_pct, half);
    b->p_high = fminf(b->upper_pct, (b->max_elev - b->min_elev) * 0.65f + b->min_elev);
    b->p_solid = (b->max_elev - b->min_elev) * 0.95f + b->min_elev;
    if (b->p_low >= b->p_high || b->p_low >= b->p_solid || b->p_high >= b->p_solid ||
        b->lower_pct >= b->upper_pct || b->lower_pct >= b->p_solid || b->upper_pct >= b->p_solid)
        b->p_high = b->p_solid = b->max_elev + 100.0f;
}

/* The class of one cell (renderNormalMode / renderPeaksMode). */
static inline int terr_classify(int kind, float e, const TerrBands *b)
{
    if (kind == TERR_NODE_MISSED) return TERR_UNKNOWN;
    if (kind == TERR_NODE_WATER) return TERR_WATER;
    if (kind != TERR_NODE_LAND) return TERR_NONE;
    if (b->peaks) {
        if (e >= b->p_solid) return TERR_GREEN_SOLID;
        if (e >= b->p_high) return TERR_GREEN_HIGH;
        if (e >= b->p_low) return TERR_GREEN_LOW;
        return TERR_NONE;
    }
    if (e < b->cutoff) return TERR_NONE;
    if (e >= b->hdr) return TERR_RED_HIGH;
    if (e >= b->hdy) return TERR_YELLOW_HIGH;
    if (e >= b->hdg && e < b->ldy) return TERR_GREEN_HIGH;
    if (e >= b->ldy && e < b->hdy) return TERR_YELLOW_LOW;
    if (e >= b->ldg && e < b->hdg) return TERR_GREEN_LOW;
    return TERR_NONE;
}

/* TERR box values (analyzeMetadata), feet; -1 hides the box (no terrain in view). Normal mode: the
 * bottom of the lowest band (cut-off or low-density green) and the highest terrain, green / yellow
 * (lowest band yellow when the green bands are squeezed out) and yellow / red (red when the highest
 * terrain is in the red band). Peaks mode: both green. */
static inline void terr_thresholds(const TerrBands *b, TerrThresholds *t)
{
    memset(t, 0, sizeof(*t));
    t->peaks = b->peaks;
    if (b->empty) { t->min_elev = t->max_elev = -1; return; }
    if (!b->peaks) {
        t->min_elev = (int)lroundf(b->cutoff > b->ldg ? b->cutoff : b->ldg);
        t->min_mode = b->ldy <= b->hdg ? TERR_LEVEL_YELLOW : TERR_LEVEL_GREEN;
        t->max_elev = (int)lroundf(b->max_elev);
        t->max_mode = b->max_elev >= b->hdr ? TERR_LEVEL_RED : TERR_LEVEL_YELLOW;
        return;
    }
    if (b->max_elev < 0.0f) { t->min_elev = -1; t->max_elev = 0; return; }
    t->min_elev = (int)lroundf(b->p_low > b->min_elev ? b->p_low : b->min_elev);
    t->max_elev = (int)lroundf(b->max_elev);
}

/* FBW TerrainMapThresholds text: upper = round(max / 100 + 0.5) ("0" for 0), lower = floor(min / 100)
 * only when min > 0, both padded to three characters. Returns 0 when the box is hidden. */
static inline int terr_threshold_text(const TerrThresholds *t, char upper[16], char lower[16])
{
    upper[0] = lower[0] = 0;
    if (t->min_elev < 0 || t->max_elev < 0 || t->max_elev > 99999 || t->min_elev > 99999) return 0;
    if (t->max_elev != 0) snprintf(upper, 16, "%3d", (int)floor(t->max_elev / 100.0 + 1.0));
    else snprintf(upper, 16, "%3s", "0");
    if (t->min_elev > 0) snprintf(lower, 16, "%3d", (int)floor(t->min_elev / 100.0));
    return 1;
}

/* ---- ND raster and the A380 scan-line sweep ------------------------------------------------- */
#define TERR_CYCLE_S 1.1               /* new frame: 600 ms transition + 500 ms timeout (FBW scan-line) */
#define TERR_TICK_S 0.04               /* RenderingMapTransitionDeltaTime */
#define TERR_SCAN_STEP_PX 39           /* round(592 / 600 * 40) */

enum { TERR_IDLE, TERR_BUILD, TERR_TRANSITION };

/* A TAWS look-ahead conflict point (a380x_taws.h): terrain that intrudes into the caution (level 1) or
 * warning (level 2) ribbon. The display paints its cell and the eight around it solid yellow / red. */
typedef struct { double lat, lon; int level; } TerrThreat;

typedef struct {
    int on;                            /* terrain drawn on this ND */
    int phase, build_index, rev;       /* rev changes with every visible change (classes, box) */
    int visible_cells, data_cells, border_y;
    TerrGeometry geo;
    double cycle_start, transition_start;
    TerrAircraft snap;                 /* aircraft state the frame in build/transition was made for */
    float ref_alt;
    TerrThresholds shown, pending;
    const TerrThreat *threats;         /* set by the caller before each update (NULL: none) */
    int threat_count;
    double lattice_lat[TERR_LAT_ROWS][TERR_LAT_COLS], lattice_dlon[TERR_LAT_ROWS][TERR_LAT_COLS];
    uint8_t cls[TERR_ROWS_MAX][TERR_COLS_MAX];    /* displayed */
    uint8_t next[TERR_ROWS_MAX][TERR_COLS_MAX];   /* frame being revealed */
    uint8_t kind[TERR_ROWS_MAX][TERR_COLS_MAX];   /* frame being built */
    int16_t elev[TERR_ROWS_MAX][TERR_COLS_MAX];
} TerrDisplay;

static inline void terr_display_reset(TerrDisplay *d)
{
    int rev = d->rev + 1;
    memset(d, 0, sizeof(*d));
    d->rev = rev;
    d->shown.min_elev = d->shown.max_elev = d->pending.min_elev = d->pending.max_elev = -1;
}

static inline void terr_display_lattice(TerrDisplay *d)
{
    int r, c;
    for (r = 0; r < TERR_LAT_ROWS; ++r) for (c = 0; c < TERR_LAT_COLS; ++c) {
        double lat, lon;
        float x, y;
        terr_cell_center(&d->geo, r * TERR_LATTICE, c * TERR_LATTICE, &x, &y);
        terr_screen_latlon(&d->geo, d->snap.lat, d->snap.lon, d->snap.heading_true, x, y, &lat, &lon);
        d->lattice_lat[r][c] = lat;
        d->lattice_dlon[r][c] = terr_wrap180(lon - d->snap.lon);
    }
}

static inline void terr_display_cell_latlon(const TerrDisplay *d, int row, int col, double *lat, double *lon)
{
    int r = row / TERR_LATTICE, c = col / TERR_LATTICE;
    double tr = (double)(row - r * TERR_LATTICE) / TERR_LATTICE, tc = (double)(col - c * TERR_LATTICE) / TERR_LATTICE;
#define TERR_BILERP(a) ((a)[r][c] * (1 - tr) * (1 - tc) + (a)[r][c + 1] * (1 - tr) * tc + \
                        (a)[r + 1][c] * tr * (1 - tc) + (a)[r + 1][c + 1] * tr * tc)
    *lat = TERR_BILERP(d->lattice_lat);
    *lon = terr_wrap180(d->snap.lon + TERR_BILERP(d->lattice_dlon));
#undef TERR_BILERP
}

/* Up to `budget` cells of the frame being built. Returns 1 once the whole frame is built. */
static inline int terr_display_build(TerrDisplay *d, const TerrLevel *L, int budget)
{
    const int total = d->geo.rows * d->geo.cols;
    while (budget-- > 0 && d->build_index < total) {
        int row = d->build_index / d->geo.cols, col = d->build_index % d->geo.cols;
        float x, y, elev = 0.0f;
        int kind = TERR_NODE_UNSAMPLED;
        ++d->build_index;
        terr_cell_center(&d->geo, row, col, &x, &y);
        if (L && terr_in_clip(d->geo.arc, x, y)) {
            double lat, lon;
            terr_display_cell_latlon(d, row, col, &lat, &lon);
            kind = terr_level_sample(L, lat, lon, &elev);
        }
        d->kind[row][col] = (uint8_t)kind;
        d->elev[row][col] = (int16_t)elev;
    }
    return d->build_index >= total;
}

/* TAWS threat areas (Honeywell: "the terrain that created the alert is changed to solid yellow / solid
 * red ... the display image (cells) surrounding the target are enlarged"): each threat point's cell of
 * the new frame and its eight neighbours, inside the clip; a warning cell is never lowered to caution.
 * Positions use the frame's snapshot (a local flat projection: threats lie within about 11 NM). */
static inline void terr_display_mark_threats(TerrDisplay *d)
{
    const double h = d->snap.heading_true * TERR_PI / 180.0, ch = cos(h), sh = sin(h);
    const double k = cos(d->snap.lat * TERR_PI / 180.0);
    int i, dr, dc;
    for (i = 0; d->threats && i < d->threat_count; ++i) {
        const TerrThreat *t = &d->threats[i];
        double e = terr_wrap180(t->lon - d->snap.lon) * 60.0 * k, n = (t->lat - d->snap.lat) * 60.0;
        float x = d->geo.ac_x + (float)((e * ch - n * sh) * d->geo.px_per_nm);
        float y = d->geo.ac_y - (float)((e * sh + n * ch) * d->geo.px_per_nm);
        int row = (int)floorf((y - (float)d->geo.y0) / TERR_CELL_PX), col = (int)floorf((x - (float)d->geo.x0) / TERR_CELL_PX);
        int cls = t->level >= 2 ? TERR_WARNING : TERR_CAUTION;
        for (dr = -1; dr <= 1; ++dr) for (dc = -1; dc <= 1; ++dc) {
            int r = row + dr, c = col + dc;
            float cx, cy;
            if (r < 0 || c < 0 || r >= d->geo.rows || c >= d->geo.cols) continue;
            terr_cell_center(&d->geo, r, c, &cx, &cy);
            if (!terr_in_clip(d->geo.arc, cx, cy) || d->next[r][c] == TERR_WARNING) continue;
            d->next[r][c] = (uint8_t)cls;
        }
    }
}

/* Statistics over the cells in view, bands, the classes of the new frame and its TERR box. */
static inline void terr_display_classify(TerrDisplay *d)
{
    int hist[TERR_HIST_BINS], row, col;
    TerrBands b;
    memset(hist, 0, sizeof(hist));
    d->visible_cells = d->data_cells = 0;
    for (row = 0; row < d->geo.rows; ++row) for (col = 0; col < d->geo.cols; ++col) {
        float x, y;
        terr_cell_center(&d->geo, row, col, &x, &y);
        if (!terr_in_clip(d->geo.arc, x, y)) continue;
        ++d->visible_cells;
        d->data_cells += d->kind[row][col] != TERR_NODE_UNSAMPLED;
        if (d->kind[row][col] == TERR_NODE_LAND) ++hist[terr_hist_bin(d->elev[row][col])];
    }
    d->ref_alt = terr_reference_altitude(d->snap.altitude_ft, d->snap.vs_fpm);
    terr_bands(hist, d->ref_alt, d->snap.gear_down ? 250.0f : 500.0f, d->snap.cutoff_ft, &b);
    for (row = 0; row < d->geo.rows; ++row) for (col = 0; col < d->geo.cols; ++col)
        d->next[row][col] = (uint8_t)terr_classify(d->kind[row][col], d->elev[row][col], &b);
    terr_thresholds(&b, &d->pending);
    terr_display_mark_threats(d);
}

/* One simulator frame of an ND's terrain display. active: TERR selected and available for this ND,
 * mode/range: its EFIS selection, L: the elevation level of that range (NULL: none yet), now: monotonic
 * seconds, budget: cells built this frame. A new frame starts every TERR_CYCLE_S; once built it is
 * revealed bottom-up by the scan line (39 px per 40 ms from y 720) and its TERR box shown. Returns 1
 * when something visible changed (rev incremented). */
static inline int terr_display_update(TerrDisplay *d, int active, int mode, float range_nm,
                                      const TerrAircraft *ac, const TerrLevel *L, double now, int budget)
{
    TerrGeometry g;
    const TerrThreat *threats = d->threats;
    const int threat_count = d->threat_count;
    int changed = 0;
    if (!active || !ac || !terr_geometry(mode, range_nm, &g)) {
        if (!d->on) return 0;
        terr_display_reset(d);
        d->threats = threats; d->threat_count = threat_count;
        return 1;
    }
    if (!d->on || d->geo.mode != g.mode || d->geo.range_nm != g.range_nm) {
        terr_display_reset(d);
        d->threats = threats; d->threat_count = threat_count;
        d->on = 1; d->geo = g; d->cycle_start = now - 2.0 * TERR_CYCLE_S;   /* start at once */
        changed = 1;
    }
    if (d->phase == TERR_IDLE && now - d->cycle_start >= TERR_CYCLE_S) {
        d->cycle_start = now; d->snap = *ac;
        terr_display_lattice(d);
        d->phase = TERR_BUILD; d->build_index = 0;
    }
    if (d->phase == TERR_BUILD && terr_display_build(d, L, budget)) {
        terr_display_classify(d);
        if (memcmp(&d->shown, &d->pending, sizeof(d->shown))) changed = 1;
        d->shown = d->pending;
        d->phase = TERR_TRANSITION; d->transition_start = now; d->border_y = TERR_IMAGE_BOTTOM + 1;
    }
    if (d->phase == TERR_TRANSITION) {
        int ticks = (int)floor((now - d->transition_start) / TERR_TICK_S) + 1, row;
        int border = TERR_IMAGE_BOTTOM - TERR_SCAN_STEP_PX * ticks;
        for (row = 0; row < d->geo.rows; ++row) {
            int centre = d->geo.y0 + TERR_CELL_PX * row + TERR_CELL_PX / 2;
            if (centre < border || centre >= d->border_y) continue;
            if (memcmp(d->cls[row], d->next[row], (size_t)d->geo.cols)) {
                memcpy(d->cls[row], d->next[row], (size_t)d->geo.cols);
                changed = 1;
            }
        }
        d->border_y = border;
        if (border <= d->geo.y0) d->phase = TERR_IDLE;
    }
    if (changed) ++d->rev;
    return changed;
}

/* ---- outputs ----------------------------------------------------------------------------------- */
/* Row-major run-length string of the displayed classes: a class letter ('a' + class) followed by the
 * run length in decimal when it is more than 1. Returns the length, or -1 if `cap` is too small. */
static inline int terr_rle_encode(const TerrDisplay *d, char *out, size_t cap)
{
    const int total = d->geo.rows * d->geo.cols;
    size_t length = 0;
    int k = 0;
    if (cap < 1) return -1;
    while (k < total) {
        int c = d->cls[k / d->geo.cols][k % d->geo.cols], run = 1, n;
        while (k + run < total && d->cls[(k + run) / d->geo.cols][(k + run) % d->geo.cols] == c) ++run;
        n = run > 1 ? snprintf(out + length, cap - length, "%c%d", 'a' + c, run)
                    : snprintf(out + length, cap - length, "%c", 'a' + c);
        if (n < 0 || (size_t)n >= cap - length) return -1;
        length += (size_t)n; k += run;
    }
    out[length] = 0;
    return (int)length;
}

/* Inverse of terr_rle_encode into cls[rows][TERR_COLS_MAX]. Returns the cell count, -1 when malformed. */
static inline int terr_rle_decode(const char *s, uint8_t cls[][TERR_COLS_MAX], int rows, int cols)
{
    int k = 0, total = rows * cols;
    while (*s) {
        int c = *s++ - 'a', run = 0;
        if (c < 0 || c >= TERR_CLASS_COUNT) return -1;
        while (*s >= '0' && *s <= '9') { run = run * 10 + (*s++ - '0'); if (run > total) return -1; }
        if (!run) run = 1;
        if (k + run > total) return -1;
        while (run--) { cls[k / cols][k % cols] = (uint8_t)c; ++k; }
    }
    return k;
}

/* RGBA bytes of raster row `row` (TERR_CELL_PX pixel rows of cols * TERR_CELL_PX pixels) at `rgba`
 * with `stride` bytes per pixel row: the class colour where its pattern is on and the pixel is inside
 * the FBW clip, transparent elsewhere. Patterns are anchored at the image origin (x0, y0). */
static inline void terr_render_row(const TerrDisplay *d, int row, unsigned char *rgba, size_t stride)
{
    const int width = d->geo.cols * TERR_CELL_PX;
    int py, px;
    for (py = 0; py < TERR_CELL_PX; ++py) {
        unsigned char *p = rgba + (size_t)py * stride;
        int iy = row * TERR_CELL_PX + py;
        float y = (float)(d->geo.y0 + iy) + 0.5f;
        for (px = 0; px < width; ++px, p += 4) {
            int c = d->cls[row][px / TERR_CELL_PX];
            uint32_t rgb = terr_class_rgb(c);
            int on = c != TERR_NONE && terr_pattern_on(terr_class_pattern(c), px, iy) &&
                     terr_in_clip(d->geo.arc, (float)(d->geo.x0 + px) + 0.5f, y);
            p[0] = (unsigned char)(on ? rgb >> 16 : 0); p[1] = (unsigned char)(on ? rgb >> 8 : 0);
            p[2] = (unsigned char)(on ? rgb : 0); p[3] = (unsigned char)(on ? 255 : 0);
        }
    }
}

/* Simulator-aid readout of the terrain under the ND pointer (the X-Plane adapter fills it, the native
 * overlay draws it). */
typedef struct {
    int inside, shown, kind;           /* pointer over the ND; readout shown; TERR_NODE_* of the probe */
    float x, y;                        /* 768 x 1024 logical ND pixels, y down */
    float probed_x, probed_y, elev_ft;
    double probed_at;
    char text[48];
} TerrPointer;

/* The terrain point under an ND pixel (y down): 1 with its latitude/longitude when the pixel is in the
 * terrain area of `g`, for the live position and true heading. */
static inline int terr_pointer_target(const TerrGeometry *g, double lat, double lon, float heading_true,
                                      float x, float y, double *out_lat, double *out_lon)
{
    if (!g->valid || !terr_in_clip(g->arc, x, y)) return 0;
    terr_screen_latlon(g, lat, lon, heading_true, x, y, out_lat, out_lon);
    return 1;
}
#endif
