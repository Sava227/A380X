/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * TAWS terrain look-ahead alerting (forward-looking terrain avoidance), XPLM-free (docs/ND_TERRAIN.md,
 * "Terrain alerts"). It works on the TERRAIN on ND elevation cache (a380x_terrain.h): the level of the
 * 10 NM range (0.16 NM nodes, probed out to about 11 NM ahead).
 *
 * A model of the published behaviour, not Honeywell's proprietary algorithm:
 *   - Honeywell MK V/MK VII EGPWS Pilot's Guide 060-4241-000 Rev H, "Terrain Look Ahead Alerting":
 *     caution and warning "ribbons" projected forward along the track, 1/4 NM wide at the aircraft and
 *     splayed +-3 degrees ("more if turning"), their vertical path from the flight path angle; caution
 *     typically 60 s before the conflict and repeated every 7 s while it lasts, warning typically 30 s
 *     before it; look-ahead a function of speed and of the distance to the nearest runway (no alerts on
 *     normal take-offs and landings); solid yellow / red threat terrain on the display; pop-up.
 *   - Required terrain clearance by phase of flight: EASA CS ACNS.E.TAWS.015 Table 1 / FAA TSO-C151
 *     (en route 700 ft level, 500 ft descending/climbing; terminal 350 / 300; approach 150 / 100;
 *     departure, above 400 ft, 100 / 100).
 * Model choices (documented, tunable): the phases follow the distance to the nearest airport (approach
 * within 5 NM, terminal within 15 NM, linear between), departure for 3 min after lift-off; the predicted
 * path keeps the current vertical speed (a climb counts at 80 %) and turn rate (at most 90 degrees of
 * turn); terrain within 2.5 NM of the nearest airport and less than 150 ft above it is the runway
 * environment and never alerts; in the approach phase, descending towards the airport (its reference
 * point ahead), the look-ahead ends 0.5 NM past it; at most 8 NM ahead; caution and warning both need a conflict in two consecutive
 * evaluations (0.4 s accumulated) to start and 2 s without conflict to end. Inhibited: TAWS off or TERR SYS OFF, no position, on the ground, below 30 ft
 * radio altitude, below 50 kt, and on departure below 400 ft above the airport.
 */
#ifndef A380X_TAWS_H
#define A380X_TAWS_H
#include "a380x_terrain.h"

#define TAWS_RANGE_NM 10.0f            /* elevation level used: 0.16 NM nodes */
#define TAWS_CAUTION_S 60.0
#define TAWS_WARNING_S 30.0
#define TAWS_MAX_LOOKAHEAD_NM 8.0
#define TAWS_RIBBON_HALF_NM 0.125      /* 1/4 NM wide at the aircraft */
#define TAWS_SPLAY_DEG 3.0
#define TAWS_RUNWAY_AREA_NM 2.5
#define TAWS_RUNWAY_MARGIN_FT 150.0f
#define TAWS_ON_DELAY_S 0.4
#define TAWS_OFF_DELAY_S 2.0
#define TAWS_DEPARTURE_S 180.0
#define TAWS_THREATS 512              /* threat points kept per evaluation (nearest first) */

enum { TAWS_PHASE_ENROUTE, TAWS_PHASE_TERMINAL, TAWS_PHASE_APPROACH, TAWS_PHASE_DEPARTURE };
enum { TAWS_ACTIVE, TAWS_INHIBIT_OFF, TAWS_INHIBIT_GROUND, TAWS_INHIBIT_LOW, TAWS_INHIBIT_SLOW };

typedef struct {
    int available;                     /* TAWS selected and powered, TERR SYS on, position valid */
    double lat, lon;
    float alt_ft;                      /* geometric altitude, feet MSL (the elevations are MSL) */
    float gs_kt, track_true, vs_fpm, turn_dps;
    int ra_valid; float ra_ft;
    int on_ground;
    int airport_valid; double apt_lat, apt_lon; float apt_elev_ft;
    double since_takeoff_s;            /* since lift-off; large when unknown */
} TawsInputs;

typedef struct {
    int caution, warning;              /* alert outputs (on/off delays applied) */
    int raw_caution, raw_warning;      /* this evaluation's conflicts */
    double caution_for, warning_for, caution_clear_for, warning_clear_for;
    int phase, inhibit, samples;
    float rtc_ft, lookahead_nm, min_clearance_ft, time_to_conflict_s;
    int threat_count;
    TerrThreat threats[TAWS_THREATS];
} TawsState;

static inline void taws_init(TawsState *s) { memset(s, 0, sizeof(*s)); s->inhibit = TAWS_INHIBIT_OFF; }

/* Required terrain clearance (CS ACNS.E.TAWS.015 Table 1) by the distance to the nearest airport:
 * approach within 4.5 NM, terminal from 5.5 to 14 NM, en route beyond 16 NM, linear in between;
 * departure for TAWS_DEPARTURE_S after lift-off. `vertical`: descending or climbing faster than 500 fpm. */
static inline float taws_required_clearance(int airport_valid, double airport_nm, int departure, int vertical, int *phase)
{
    const float enroute = vertical ? 500.0f : 700.0f, terminal = vertical ? 300.0f : 350.0f, approach = vertical ? 100.0f : 150.0f;
    double d = airport_valid && isfinite(airport_nm) ? airport_nm : 1e9;
    if (departure) { *phase = TAWS_PHASE_DEPARTURE; return 100.0f; }
    if (d <= 4.5) { *phase = TAWS_PHASE_APPROACH; return approach; }
    if (d < 5.5) { *phase = TAWS_PHASE_APPROACH; return approach + (terminal - approach) * (float)(d - 4.5); }
    if (d <= 14.0) { *phase = TAWS_PHASE_TERMINAL; return terminal; }
    if (d < 16.0) { *phase = TAWS_PHASE_TERMINAL; return terminal + (enroute - terminal) * (float)(d - 14.0) / 2.0f; }
    *phase = TAWS_PHASE_ENROUTE;
    return enroute;
}

/* Local flat offsets (NM east / north) from (lat, lon), adequate within the 8 NM look-ahead. */
static inline void taws_offset(double lat, double lon, double east, double north, double *out_lat, double *out_lon)
{
    *out_lat = lat + north / 60.0;
    *out_lon = terr_wrap180(lon + east / (60.0 * fmax(0.05, cos(lat * TERR_PI / 180.0))));
}

static inline double taws_distance_nm(double lat1, double lon1, double lat2, double lon2)
{
    double n = (lat2 - lat1) * 60.0, e = terr_wrap180(lon2 - lon1) * 60.0 * cos(0.5 * (lat1 + lat2) * TERR_PI / 180.0);
    return hypot(e, n);
}

static inline void taws_hysteresis(int raw, double dt, int *out, double *on_for, double *clear_for)
{
    if (raw) {
        *on_for += dt; *clear_for = 0.0;
        if (*on_for >= TAWS_ON_DELAY_S - 1e-9) *out = 1;
    } else {
        *on_for = 0.0;
        if (*out) { *clear_for += dt; if (*clear_for >= TAWS_OFF_DELAY_S - 1e-9) *out = 0; }
    }
}

/* One evaluation (the adapter runs it 5 times a second; dt = time since the previous one). */
static inline void taws_evaluate(TawsState *s, const TawsInputs *in, const TerrLevel *L, double dt)
{
    double gs_nmps, d_caution, d_warning, step, splay, omega, heading, east = 0.0, north = 0.0, d, turned = 0.0;
    double airport_nm = 1e9, apt_height;
    float rtc, vs_eff;
    int departure, vertical;
    if (!isfinite(dt) || dt < 0.0) dt = 0.0;
    s->raw_caution = s->raw_warning = 0;
    s->threat_count = 0; s->samples = 0;
    s->min_clearance_ft = 1e9f; s->time_to_conflict_s = -1.0f; s->lookahead_nm = 0.0f;
    if (in->airport_valid) airport_nm = taws_distance_nm(in->lat, in->lon, in->apt_lat, in->apt_lon);
    apt_height = in->airport_valid ? in->alt_ft - in->apt_elev_ft : 1e9;
    departure = in->since_takeoff_s < TAWS_DEPARTURE_S;
    if (!in->available || !L || !isfinite(in->alt_ft) || !isfinite(in->lat) || !isfinite(in->lon)) s->inhibit = TAWS_INHIBIT_OFF;
    else if (in->on_ground) s->inhibit = TAWS_INHIBIT_GROUND;
    else if (!isfinite(in->gs_kt) || in->gs_kt < 50.0f || !isfinite(in->track_true)) s->inhibit = TAWS_INHIBIT_SLOW;
    else if ((in->ra_valid && isfinite(in->ra_ft) && in->ra_ft < 30.0f) || (departure && apt_height < 400.0)) s->inhibit = TAWS_INHIBIT_LOW;
    else s->inhibit = TAWS_ACTIVE;
    if (s->inhibit != TAWS_ACTIVE) {
        s->caution = s->warning = 0;
        s->caution_for = s->warning_for = s->caution_clear_for = s->warning_clear_for = 0.0;
        return;
    }
    vertical = isfinite(in->vs_fpm) && fabsf(in->vs_fpm) > 500.0f;
    rtc = taws_required_clearance(in->airport_valid, airport_nm, departure, vertical, &s->phase);
    s->rtc_ft = rtc;
    vs_eff = isfinite(in->vs_fpm) ? fmaxf(-8000.0f, fminf(6000.0f, in->vs_fpm < 0.0f ? in->vs_fpm : 0.8f * in->vs_fpm)) : 0.0f;
    gs_nmps = in->gs_kt / 3600.0;
    d_caution = fmin(gs_nmps * TAWS_CAUTION_S, TAWS_MAX_LOOKAHEAD_NM);
    if (s->phase == TAWS_PHASE_APPROACH && in->vs_fpm < -200.0f && in->airport_valid) {
        /* towards the airport: no look-ahead beyond the runways (the path would continue below them) */
        const double n = (in->apt_lat - in->lat) * 60.0, k = cos(in->lat * TERR_PI / 180.0);
        const double e = terr_wrap180(in->apt_lon - in->lon) * 60.0 * k, h = in->track_true * TERR_PI / 180.0;
        const double ahead = e * sin(h) + n * cos(h);
        if (ahead > 0.0) d_caution = fmin(d_caution, ahead + 0.5);
    }
    d_warning = fmin(gs_nmps * TAWS_WARNING_S, d_caution);
    s->lookahead_nm = (float)d_caution;
    step = fmax(0.04, L->spacing_nm * 0.5);
    omega = isfinite(in->turn_dps) ? fmax(-6.0, fmin(6.0, in->turn_dps)) : 0.0;
    splay = (TAWS_SPLAY_DEG + fmin(7.0, 1.5 * fabs(omega))) * TERR_PI / 180.0;
    heading = in->track_true * TERR_PI / 180.0;
    for (d = step; d <= d_caution + 1e-9; d += step) {
        const double t = d / gs_nmps, pred = in->alt_ft + vs_eff * t / 60.0, half = TAWS_RIBBON_HALF_NM + d * tan(splay);
        const double spacing = L->spacing_nm;
        int m = (int)ceil(half / spacing), k;
        if (m < 1) m = 1;
        if (m > 4) m = 4;
        if (fabs(turned) < 90.0) {   /* constant turn, at most 90 degrees */
            double dturn = omega * step / gs_nmps;
            heading += dturn * TERR_PI / 180.0; turned += dturn;
        }
        east += sin(heading) * step; north += cos(heading) * step;
        for (k = -m; k <= m; ++k) {
            const double off = half * k / m;
            double lat, lon;
            float elev;
            int kind;
            taws_offset(in->lat, in->lon, east + cos(heading) * off, north - sin(heading) * off, &lat, &lon);
            kind = terr_level_sample(L, lat, lon, &elev);
            if (kind != TERR_NODE_LAND && kind != TERR_NODE_WATER) continue;   /* no data: no alert */
            if (kind == TERR_NODE_WATER) elev = 0.0f;
            ++s->samples;
            if (in->airport_valid && elev < in->apt_elev_ft + TAWS_RUNWAY_MARGIN_FT &&
                taws_distance_nm(lat, lon, in->apt_lat, in->apt_lon) < TAWS_RUNWAY_AREA_NM) continue;   /* runway environment */
            if ((float)pred - elev < s->min_clearance_ft) s->min_clearance_ft = (float)pred - elev;
            if (elev <= pred - rtc) continue;
            if (s->time_to_conflict_s < 0.0f) s->time_to_conflict_s = (float)t;
            s->raw_caution = 1;
            if (d <= d_warning + 1e-9) s->raw_warning = 1;
            if (s->threat_count < TAWS_THREATS) {
                TerrThreat *x = &s->threats[s->threat_count++];
                x->lat = lat; x->lon = lon; x->level = d <= d_warning + 1e-9 ? 2 : 1;
            }
        }
    }
    taws_hysteresis(s->raw_warning, dt, &s->warning, &s->warning_for, &s->warning_clear_for);
    taws_hysteresis(s->raw_caution, dt, &s->caution, &s->caution_for, &s->caution_clear_for);
}
#endif
