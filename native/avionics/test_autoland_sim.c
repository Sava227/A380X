/* SPDX-License-Identifier: GPL-3.0-or-later
 * Offline closed-loop ILS approach -> autoland -> roll-out -> stop simulation.
 *
 * The REAL native guidance (a380x_guidance.c + a380x_autoland.inc) and the unmodified FlyByWire
 * A380FgOuterLoops laws fly a simplified A380-like airframe. This airframe is a hand-written bench
 * model (longitudinal point mass + pitch, lateral/directional linear aero, oleo/tyre ground model,
 * MIL-F-8785C style log wind profile and first-order turbulence). It is NOT X-Plane's flight model:
 * the numbers printed here verify the mode sequence, signs and loop closure, not in-sim performance.
 *
 * Runway: LFBO 14R, X-Plane 12 default navdata (read-only, values frozen here):
 *   earth_nav.dat  4 43.614722222 1.376527778 485 11070 18  51263.010 TBS LFBO LF 14R ILS-cat-III
 *                  6 43.642444444 1.349583333 485 11070 18 300143.010 TBS LFBO LF 14R GS
 *                 12 43.642444444 1.349583333 540 11070 25      0.000 TBS LFBO LF BLAGNAC DME-ILS
 *   apt.dat 100    14R 43.6441160 1.3459430 ... 32L 43.6189635 1.3721196 (45 m wide)
 *   CIFP LFBO.dat  RWY:RW14R elevation 488 ft; I14R RW14R crossing 544 ft (TCH 56 ft), GS -3.00
 * The localizer course is 143.010 deg true (magnetic 142); the DME is co-located with the glide antenna.
 */
#include "a380x_guidance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define D2R 0.017453292519943295
#define R2D 57.29577951308232
#define KT 0.514444444
#define FT 0.3048
#define G 9.80665

/* ---- runway geometry (local tangent plane at the 14R threshold) ---- */
typedef struct { double loc_x, loc_y, gs_x, gs_y, length, course_true, magvar, elevation_ft; } Runway;
static void project(double lat, double lon, double lat0, double lon0, double course, double *along, double *right) {
    const double r = 6371008.8;
    double e = (lon - lon0) * D2R * r * cos(lat0 * D2R), n = (lat - lat0) * D2R * r;
    *along = e * sin(course * D2R) + n * cos(course * D2R);
    *right = e * cos(course * D2R) - n * sin(course * D2R);
}
static Runway lfbo14r(void) {
    Runway w; double y;
    w.course_true = 143.010; w.magvar = 1.010; w.elevation_ft = 488;
    project(43.614722222, 1.376527778, 43.6441160, 1.3459430, w.course_true, &w.loc_x, &w.loc_y);
    project(43.642444444, 1.349583333, 43.6441160, 1.3459430, w.course_true, &w.gs_x, &w.gs_y);
    project(43.6189635, 1.3721196, 43.6441160, 1.3459430, w.course_true, &w.length, &y);
    return w;
}

/* ---- deterministic turbulence ---- */
static unsigned long long rng_state;
static double gauss(void) {
    double a, b;
    rng_state ^= rng_state << 13; rng_state ^= rng_state >> 7; rng_state ^= rng_state << 17;
    a = ((rng_state >> 11) + .5) / 9007199254740992.0;
    rng_state ^= rng_state << 13; rng_state ^= rng_state >> 7; rng_state ^= rng_state << 17;
    b = ((rng_state >> 11) + .5) / 9007199254740992.0;
    return sqrt(-2 * log(a)) * cos(6.283185307179586 * b);
}

typedef struct {
    const char *name;
    double mass_kg, vapp_kt, vls_kt;
    double cross_kt, head_kt;      /* at 20 ft; cross > 0 blows from the LEFT (toward +Y) */
    double turbulence;             /* sigma of the horizontal gust, m/s (vertical 70 %) */
    double go_around_ra_ft;        /* > 0: TOGA at this height instead of landing */
    int retard_late;               /* crew sets IDLE 1 s after touchdown instead of at the 10 ft call */
    unsigned long long seed;
} Case;

typedef struct {
    double X, Y, H, u, v, vz, psi, theta, q, phi, p, r;
    double eng, rev, spoiler, gust[3];
} Air;

typedef struct {
    double t_land, ra_land, t_flare, ra_flare, t_retard, ra_retard, t_rollout, t_athr_off;
    double td_t, td_x, td_y, td_sink_fpm, td_ias, td_pitch, td_crab_deg, td_bank, td_tail_margin_m;
    double max_y_rollout, stop_x, stop_y, min_ra_ga, ga_final_alt_ft, ga_max_alt_ft, max_pitch;
    int touched, stopped, ap_lost, sequence_ok, ga_modes_ok;
    int cap_land, cap_alert, cap_touchdown, cap_downgrades, cap_downgrade_prev;   /* A380XLandCapability */
    char sequence[512];
} Result;

/* Airframe constants (A380-800-like; bench values, not certified data). */
static const double S = 845, CHORD = 12.0, SPAN = 79.75;
static const double IYY = 4.0e7, IXX = 3.0e7, IZZ = 6.5e7;
static const double MAIN_X = -2.5, NOSE_X = 31.0, GEAR_Z = -6.2, TAIL_X = -35.0, TAIL_Z = 0.2;

static double wind_scale(double h_m) {   /* MIL-F-8785C log profile, z0 = 0.15 ft, 1.0 at 20 ft */
    double h_ft = fmax(1.0, fmin(1000.0, h_m / FT));
    return log(h_ft / .15) / log(20 / .15);
}
static double point_height(const Air *a, double x, double z) { return a->H + x * sin(a->theta) + z * cos(a->theta); }
static double point_rate(const Air *a, double x, double z) {
    return a->vz + (x * cos(a->theta) - z * sin(a->theta)) * a->q;
}

static void log_event(Result *res, double t, const char *what, double ra_ft) {
    char item[64];
    snprintf(item, sizeof(item), "%s%s@%.0fft/%.1fs", res->sequence[0] ? " > " : "", what, ra_ft, t);
    if (strlen(res->sequence) + strlen(item) + 1 < sizeof(res->sequence)) strcat(res->sequence, item);
}

static Result fly(const Case *c, int verbose) {
    Runway rw = lfbo14r();
    A380XGuidanceState s; A380XGuidanceInputs in; Air a; Result res;
    double t = 0, levers = 25.0 / 45.0, last_vz_main = 0;
    int i, n, reverse = 0, prev_lat = -1, prev_vert = -1, prev_land = 0, prev_flare = 0, prev_ro = 0,
        prev_retard = 0, prev_athr = 0, prev_ga = 0, pilot_disc = 0, toga = 0;
    memset(&res, 0, sizeof(res)); memset(&a, 0, sizeof(a)); memset(&in, 0, sizeof(in)); memset(&s, 0, sizeof(s));
    res.min_ra_ga = 1e9;
    rng_state = c->seed;
    /* Start 8 nm out, level at 2000 ft above the threshold, 700 m left of the centreline, heading
     * 25 deg right of the course: LOC capture from the left, G/S capture from below. */
    a.X = -22000; a.Y = -900; a.H = 2000 * FT; a.psi = 25 * D2R;
    {   /* level-flight trim at VAPP + 5 kt: attitude from the lift curve, engine from the drag polar */
        double v = (c->vapp_kt + 5) * KT, q0 = .5 * 1.208 * v * v, cl = c->mass_kg * G / (q0 * S);
        double cd = .090 + .042 * cl * cl, tmax = 4 * 320000 * (1 - .0006 * v / KT);
        a.theta = (cl - .95) / 5.0;
        a.eng = fmax(0, fmin(1, (q0 * S * cd - 4 * 9000) / (tmax - 4 * 9000)));
    }
    {   /* start at VAPP + 5 kt through the air: ground velocity = air velocity + wind (heading frame) */
        double sc = wind_scale(a.H), wx = -c->head_kt * KT * sc, wy = c->cross_kt * KT * sc;
        a.u = (c->vapp_kt + 5) * KT + wx * cos(a.psi) + wy * sin(a.psi);
        a.v = -wx * sin(a.psi) + wy * cos(a.psi);
    }
    in.powered = in.attitude_valid = in.airdata_valid = in.athr_speed_valid = 1;
    in.throttle_levers_valid = 1; in.idle_detent_ratio = .01; in.cl_detent_ratio = 25.0 / 45.0;
    in.mct_detent_ratio = 35.0 / 45.0; in.toga_detent_ratio = 1; in.detent_tolerance_ratio = .02;
    in.vls_knots = c->vls_kt; in.vmax_knots = 182; in.weight_kg = c->mass_kg;
    in.radio_height_valid = 1; in.flap_handle_index = 4;
    for (n = 0; n < 2; ++n) {
        in.ils[n].gs_angle_deg = 3.0; in.ils[n].course_deg = rw.course_true - rw.magvar; in.ils[n].frequency_mhz = 110.70;
        in.side_radio_height_valid[n] = in.side_attitude_valid[n] = in.side_airdata_valid[n] = 1;
    }
    in.hydraulics_valid = in.rudder_valid = in.yaw_channel_available = 1;   /* full LAND 3 DUAL redundancy */
    for (int step = 0; step < 50 * 600; ++step) {
        const double dt = .02;
        double heading_true, track_true, gs_ms, u_air, v_air, w_air, vair, rho = 1.208, qbar, alpha, beta_std, ra_ft;
        /* ---- sensors -> guidance inputs ---- */
        {
            double wu, wv, wx, wy, sc = wind_scale(point_height(&a, MAIN_X, GEAR_Z));
            wx = -c->head_kt * KT * sc + a.gust[0]; wy = c->cross_kt * KT * sc + a.gust[1];
            wu = wx * cos(a.psi) + wy * sin(a.psi); wv = -wx * sin(a.psi) + wy * cos(a.psi);
            u_air = a.u - wu; v_air = a.v - wv; w_air = a.vz - a.gust[2];
        }
        vair = sqrt(u_air * u_air + v_air * v_air + w_air * w_air);
        qbar = .5 * rho * vair * vair;
        alpha = a.theta - atan2(w_air, u_air); beta_std = atan2(v_air, u_air);
        heading_true = rw.course_true + a.psi * R2D;
        gs_ms = hypot(a.u, a.v);
        track_true = heading_true + atan2(a.v, a.u) * R2D;
        ra_ft = fmax(0, point_height(&a, MAIN_X, GEAR_Z)) / FT;
        in.ias_knots = vair * sqrt(rho / 1.225) / KT; in.tas_knots = vair / KT; in.mach = vair / 340.0;
        in.ground_speed_knots = gs_ms / KT;
        in.heading_deg = fmod(heading_true - rw.magvar + 720, 360); in.true_heading_deg = fmod(heading_true + 720, 360);
        in.track_deg = fmod(track_true - rw.magvar + 720, 360); in.true_track_deg = fmod(track_true + 720, 360);
        in.altitude_ft = in.pressure_altitude_ft = rw.elevation_ft + (a.H + GEAR_Z) / FT;
        in.radio_height_ft = in.side_radio_height_ft[0] = in.side_radio_height_ft[1] = ra_ft;
        in.vertical_speed_fpm = a.vz / FT * 60;
        in.pitch_deg = a.theta * R2D; in.roll_deg = a.phi * R2D;
        in.pitch_rate_deg_s = a.q * R2D; in.roll_rate_deg_s = a.p * R2D; in.yaw_rate_deg_s = a.r * R2D;
        in.alpha_deg = alpha * R2D; in.beta_deg = -beta_std * R2D;   /* X-Plane: heading minus flown path */
        in.on_ground = point_height(&a, MAIN_X, GEAR_Z) <= 0 || point_height(&a, NOSE_X, GEAR_Z) <= 0;
        for (n = 0; n < 2; ++n) {
            A380XIlsInputs *ils = &in.ils[n];
            double dx = rw.loc_x - a.X, dy = a.Y - rw.loc_y, gx = a.X - rw.gs_x, gy = a.Y - rw.gs_y;
            double ground = hypot(gx, gy), elevation = atan2(a.H, fmax(1, -gx)) * R2D;
            double loc_angle = atan2(dy, dx) * R2D;   /* + right of the centreline */
            ils->valid = ils->loc_valid = fabs(loc_angle) < 35 && dx > 0;
            ils->gs_valid = ils->loc_valid && gx < 0 && a.H > 0;
            ils->loc_deviation_dots = -loc_angle / 1.0;      /* port bridge: 1 dot = 1 deg */
            ils->gs_deviation_dots = (3.0 - elevation) / .28; /* + needle up, 0.28 deg/dot */
            ils->distance_nm = hypot(ground, a.H) / 1852.0;
            ils->localizer_distance_valid = 1; ils->localizer_distance_nm = hypot(dx, dy) / 1852.0;
        }
        /* ---- crew ---- */
        if (step == 300) { a380x_guidance_command(&s, &in, A380X_FCU_AP1, 0); }   /* 5 s after "liftoff" */
        if (step == 310) {
            a380x_guidance_command(&s, &in, A380X_FCU_ATHR, 0);
            s.selected_speed_knots = c->vapp_kt; s.selected_altitude_ft = 3000;
            s.selected_heading_deg = in.heading_deg;
            a380x_guidance_command(&s, &in, A380X_FCU_HDG_PULL, 0);
            a380x_guidance_command(&s, &in, A380X_FCU_ALT_HOLD, 0);
            a380x_guidance_command(&s, &in, A380X_FCU_APPR, 0);
            a380x_guidance_command(&s, &in, A380X_FCU_AP2, 0);
        }
        if (c->go_around_ra_ft > 0 && !toga && s.land.land_active && ra_ft < c->go_around_ra_ft) {
            toga = 1; levers = 1.0;
        }
        if (toga && levers > .9 && ra_ft > 1500) levers = 25.0 / 45.0;   /* thrust reduction altitude: CL */
        if (!toga && !res.touched && s.land.land_active && ra_ft <= 10 && !c->retard_late) levers = 0;
        if (!toga && res.touched && t >= res.td_t + 1 && levers > 0) levers = 0;
        if (!toga && res.touched && t >= res.td_t + 2.5 && !reverse && levers <= .011) reverse = 1;
        if (reverse == 1 && in.ground_speed_knots < 70) reverse = 2;    /* idle reverse below 70 kt */
        if (reverse == 2 && in.ground_speed_knots < 25) reverse = 3;    /* stow */
        if (res.touched && !pilot_disc && in.ground_speed_knots < 30 && (s.ap1 || s.ap2)) {
            pilot_disc = 1; a380x_guidance_command(&s, &in, A380X_FCU_AP_DISCONNECT, 0);
        }
        for (n = 0; n < 4; ++n) { in.throttle_lever_ratio[n] = levers; in.reverse_selected[n] = 0; }
        if (reverse == 1 || reverse == 2) in.reverse_selected[1] = in.reverse_selected[2] = 1;
        in.throttle_ratio = levers;
        in.rudder_ratio = 0;
        if (step == 0) a380x_guidance_init(&s, &in);
        a380x_guidance_update(&s, &in, dt);
        t += dt;
        /* ---- mode bookkeeping ---- */
        if (step > 320 && !pilot_disc && !s.ap1 && !s.ap2 && !res.ap_lost) {
            res.ap_lost = 1; log_event(&res, t, "AP OFF(unexpected)", ra_ft);
            if (verbose) printf("  AP lost: IAS %.1f VLS %.1f pitch %.2f roll %.2f valid %d gv %d\n", in.ias_knots, in.vls_knots,
                                in.pitch_deg, in.roll_deg, s.guidance_valid, s.vertical_guidance_selected);
        }
        if ((int)s.lateral_mode != prev_lat) {
            if (s.lateral_mode == A380X_C1_LOC_STAR) log_event(&res, t, "LOC*", ra_ft);
            if (s.lateral_mode == A380X_C1_LOC) log_event(&res, t, "LOC", ra_ft);
            if (s.lateral_mode == A380X_C1_GA_TRK) log_event(&res, t, "GA TRK", ra_ft);
            prev_lat = s.lateral_mode;
        }
        if ((int)s.vertical_mode != prev_vert) {
            if (s.vertical_mode == A380X_B1_GS_STAR) log_event(&res, t, "G/S*", ra_ft);
            if (s.vertical_mode == A380X_B1_GS) log_event(&res, t, "G/S", ra_ft);
            if (s.vertical_mode == A380X_B1_SRS) log_event(&res, t, "SRS", ra_ft);
            if (s.vertical_mode == A380X_B1_ALT_STAR && toga) log_event(&res, t, "ALT*", ra_ft);
            if (s.vertical_mode == A380X_B1_ALT && toga) log_event(&res, t, "ALT", ra_ft);
            prev_vert = s.vertical_mode;
        }
        if (s.land.land_active && !prev_land) {
            res.t_land = t; res.ra_land = ra_ft; res.cap_land = s.land.capability; log_event(&res, t, "LAND", ra_ft);
        }
        if (s.land.capability_frozen && !res.cap_alert) res.cap_alert = s.land.capability;
        if (s.land.capability_downgrade && !res.cap_downgrade_prev) ++res.cap_downgrades;
        res.cap_downgrade_prev = s.land.capability_downgrade;
        if (s.land.flare_active && !prev_flare) { res.t_flare = t; res.ra_flare = ra_ft; log_event(&res, t, "FLARE", ra_ft); }
        if (s.land.retard_active && !prev_retard) { res.t_retard = t; res.ra_retard = ra_ft; log_event(&res, t, "IDLE(RETARD)", ra_ft); }
        if (s.land.rollout_active && !prev_ro) { res.t_rollout = t; log_event(&res, t, "ROLL OUT", ra_ft); }
        if (prev_athr && !s.athr_armed) { res.t_athr_off = t; log_event(&res, t, "A/THR OFF", ra_ft); }
        if (s.land.go_around_active && !prev_ga) log_event(&res, t, "GA", ra_ft);
        prev_land = s.land.land_active; prev_flare = s.land.flare_active; prev_ro = s.land.rollout_active;
        prev_retard = s.land.retard_active; prev_athr = s.athr_armed; prev_ga = s.land.go_around_active;
        if (toga) { res.min_ra_ga = fmin(res.min_ra_ga, ra_ft); res.ga_max_alt_ft = fmax(res.ga_max_alt_ft, in.altitude_ft); }
        /* ---- airframe, 4 substeps ---- */
        for (i = 0; i < 4; ++i) {
            const double h = dt / 4, m = c->mass_kg, W = m * G;
            double cl, cd, cm, lift, drag, thrust, tmax, tidle = 4 * 9000, f_up, f_side, mom, roll_m, yaw_m;
            double nm = 0, nn = 0, hm, hn, fx_ground = 0, fy_ground = 0, yaw_ground = 0, pitch_ground = 0;
            double ge, command, de, sc, wu, wv, wx, wy, ua, va, wa, vv, gam, al, bs;
            sc = wind_scale(point_height(&a, MAIN_X, GEAR_Z));
            for (n = 0; n < 3; ++n) {   /* first-order gusts, length scale 150 m (vertical: height) */
                double L = n == 2 ? fmax(15, fmin(150, a.H)) : 150, sig = c->turbulence * (n == 2 ? .7 : 1);
                double vv0 = fmax(10, hypot(a.u, a.v));
                if (sig > 0) a.gust[n] += -a.gust[n] * vv0 / L * h + sig * sqrt(2 * vv0 / L * h) * gauss();
            }
            wx = -c->head_kt * KT * sc + a.gust[0]; wy = c->cross_kt * KT * sc + a.gust[1];
            wu = wx * cos(a.psi) + wy * sin(a.psi); wv = -wx * sin(a.psi) + wy * cos(a.psi);
            ua = a.u - wu; va = a.v - wv; wa = a.vz - a.gust[2];
            vv = sqrt(ua * ua + va * va + wa * wa); gam = atan2(wa, ua); al = a.theta - gam; bs = atan2(va, ua);
            qbar = .5 * rho * vv * vv;
            hm = point_height(&a, MAIN_X, GEAR_Z); hn = point_height(&a, NOSE_X, GEAR_Z);
            ge = exp(-2.2 * fmax(0, hm + 6) / SPAN);                    /* ground effect */
            if (a.spoiler < 1 && hm <= .05 && res.touched) a.spoiler = fmin(1, a.spoiler + h / .8);
            de = -s.pitch_servo * 25 * D2R;
            cl = (.95 + 5.0 * al - .35 * de) * (1 + .10 * ge) - 1.0 * a.spoiler;
            cl = fmin(cl, 2.6);
            cd = .090 + .042 * cl * cl * (1 - .45 * ge) + .06 * a.spoiler;
            cm = .0 + -1.0 * (al - 5.4 * D2R) + -35 * (a.q * CHORD / (2 * fmax(vv, 30))) + -1.6 * de - .02 * ge;
            lift = qbar * S * cl; drag = qbar * S * cd;
            command = s.athr_active ? s.throttle_command : levers;
            if (reverse == 1 || reverse == 2) command = 0;
            a.eng += (command - a.eng) * h / (command > a.eng ? 3.0 : 2.0);
            a.rev += ((reverse == 1 ? 1.0 : 0.0) - a.rev) * h / 2.5;
            tmax = 4 * 320000 * (1 - .0006 * vv / KT);
            thrust = tidle + a.eng * (tmax - tidle) - (reverse == 1 || reverse == 2 ? 2 * (25000 + 115000 * a.rev) : 0);
            /* oleo/tyre normal loads */
            if (hm < 0) nm = fmax(0, -hm * 1.3e7 - point_rate(&a, MAIN_X, GEAR_Z) * 2.7e6);
            if (hn < 0) nn = fmax(0, -hn * 2.3e6 - point_rate(&a, NOSE_X, GEAR_Z) * 5.0e5);
            if (hm < 0 && !res.touched) {
                res.touched = 1; res.td_t = t; res.td_x = a.X; res.td_y = a.Y; res.cap_touchdown = s.land.capability;
                res.td_sink_fpm = -last_vz_main / FT * 60; res.td_ias = in.ias_knots;
                res.td_pitch = a.theta * R2D; res.td_crab_deg = a.psi * R2D; res.td_bank = a.phi * R2D;
                res.td_tail_margin_m = point_height(&a, TAIL_X, TAIL_Z) - hm;
                log_event(&res, t, "TOUCHDOWN", 0);
            }
            last_vz_main = point_rate(&a, MAIN_X, GEAR_Z);
            if (nm + nn > 0) {
                double ue = fmax(fabs(a.u), 3), vm = a.v + a.r * MAIN_X, vn = a.v + a.r * NOSE_X, steer, fm, fn, lim;
                double brake = 0, decel_other, gsk = hypot(a.u, a.v) / KT;
                /* X-Plane pedal steering of the nose gear (ACF: 90 deg low speed, 7 deg high speed),
                 * assumed blended linearly up to 60 kt. */
                steer = s.land.yaw_active ? s.land.yaw_servo : 0;
                steer *= (7 + (90 - 7) * fmax(0, 1 - gsk / 60)) * D2R;
                fm = -6.0 * atan2(vm, ue) * nm; lim = .6 * nm; fm = fmax(-lim, fmin(lim, fm));
                fn = -5.0 * (atan2(vn, ue) - steer) * nn; lim = .6 * nn; fn = fmax(-lim, fmin(lim, fn));
                fy_ground = fm + fn; yaw_ground = fn * NOSE_X + fm * MAIN_X;
                decel_other = (drag + .015 * (nm + nn) - thrust) / m;
                if (res.touched && t > res.td_t + 2.0) brake = fmax(0, fmin(.5 * nm, m * (2.0 - decel_other)));
                if (a.u < .3) brake = 0;
                fx_ground = -(.015 * (nm + nn) + brake) * (a.u > 0 ? 1 : 0);
                pitch_ground = nn * NOSE_X * cos(a.theta) + nm * MAIN_X * cos(a.theta) - (fx_ground) * GEAR_Z;
            }
            /* longitudinal (vertical plane of the heading) */
            f_up = lift * cos(gam) * cos(a.phi) + drag * -sin(gam) + thrust * sin(a.theta) - W + nm + nn;
            {
                double fx = -drag * cos(gam) - lift * sin(gam) + thrust * cos(a.theta) + fx_ground;
                double ax = fx / m, az = f_up / m;
                /* specific force, body axes, for the accelerometers (bx forward, bz incremental g) */
                in.accel_x_m_s2 = ax * cos(a.theta) + (az + G) * sin(a.theta);
                in.accel_z_m_s2 = -ax * sin(a.theta) + (az + G) * cos(a.theta) - G;
                a.u += (ax + a.v * a.r) * h; a.vz += az * h;
            }
            mom = qbar * S * CHORD * cm + thrust * 2.5 + pitch_ground;
            a.q += mom / IYY * h;
            /* lateral/directional */
            f_side = qbar * S * (-.95 * bs) + lift * sin(a.phi) + fy_ground;
            roll_m = qbar * S * SPAN * (-.10 * bs - .45 * a.p * SPAN / (2 * fmax(vv, 30)) + .10 * a.r * SPAN / (2 * fmax(vv, 30))
                                         + .065 * s.roll_servo * (s.ap1 || s.ap2 ? 1 : 0));
            yaw_m = qbar * S * SPAN * (.11 * bs - .25 * a.r * SPAN / (2 * fmax(vv, 30))
                                        + .044 * (s.land.yaw_active ? s.land.yaw_servo : 0) * 30 * D2R) + yaw_ground;
            in.accel_y_m_s2 = f_side / m;
            a.v += (f_side / m - a.u * a.r) * h;
            a.r += yaw_m / IZZ * h;
            if (nm + nn > 0) { a.p += (-a.phi * 8 - a.p * 4) * h; } else a.p += roll_m / IXX * h;
            a.phi += a.p * h; a.psi += a.r * h; a.theta += a.q * h;
            a.H += a.vz * h;
            a.X += (a.u * cos(a.psi) - a.v * sin(a.psi)) * h;
            a.Y += (a.u * sin(a.psi) + a.v * cos(a.psi)) * h;
            (void)beta_std;
        }
        res.max_pitch = fmax(res.max_pitch, a.theta * R2D);
        if (res.touched && t > res.td_t + 1) res.max_y_rollout = fmax(res.max_y_rollout, fabs(a.Y));
        if (verbose && (step % 250 == 0 || (ra_ft < 120 && !res.touched && step % 10 == 0) || (res.touched && t < res.td_t + 4 && step % 10 == 0) || (toga && step % 50 == 0)))
            printf("  t=%6.1f X=%8.0f Y=%6.1f RA=%7.1f IAS=%5.1f VS=%6.0f th=%5.2f(c%5.2f) phi=%5.2f psi=%5.2f gs=%5.2f loc=%5.2f lat=%d vert=%d land=%d/%d/%d yaw=%.2f thr=%.2f eng=%.2f ps=%.2f\n",
                   t, a.X, a.Y, ra_ft, in.ias_knots, in.vertical_speed_fpm, a.theta * R2D, s.pitch_command_deg, a.phi * R2D, a.psi * R2D,
                   in.ils[0].gs_deviation_dots, in.ils[0].loc_deviation_dots,
                   s.lateral_mode, s.vertical_mode, s.land.land_active, s.land.flare_active, s.land.rollout_active,
                   s.land.yaw_servo, s.throttle_command, a.eng, s.pitch_servo);
        if (toga && t > res.t_land + 120) { res.ga_final_alt_ft = in.altitude_ft; break; }
        if (res.touched && hypot(a.u, a.v) < 1.5) { res.stopped = 1; res.stop_x = a.X; res.stop_y = a.Y; break; }
    }
    res.ga_modes_ok = toga && strstr(res.sequence, "SRS") && strstr(res.sequence, "GA TRK") && !res.touched;
    {
        const char *p = strstr(res.sequence, "LOC");
        const char *q = p ? strstr(p, "G/S") : NULL, *l = q ? strstr(q, "LAND") : NULL;
        const char *f = l ? strstr(l, "FLARE") : NULL, *td = f ? strstr(f, "TOUCHDOWN") : NULL;
        const char *ro = td ? strstr(td, "ROLL OUT") : NULL;
        res.sequence_ok = ro != NULL;
    }
    a380x_guidance_destroy(&s);
    return res;
}

int main(int argc, char **argv) {
    static const Case cases[] = {
        {"calm, 386 t", 386000, 138, 132, 0, 0, 0, 0, 0, 11},
        {"15 kt X-wind from left, 5 kt head, light turb", 386000, 140, 132, 15, 5, .8, 0, 0, 23},
        {"25 kt X-wind from right, 10 kt head, moderate turb", 386000, 143, 132, -25, 10, 1.5, 0, 0, 37},
        {"300 t, 10 kt tailwind, crew retards late", 300000, 128, 120, 0, -10, .5, 0, 1, 41},
        {"go-around (TOGA at 150 ft)", 386000, 138, 132, 8, 5, .5, 150, 0, 53},
    };
    int verbose = argc > 1 && !strcmp(argv[1], "-v"), only = argc > 2 ? atoi(argv[2]) : -1, k, failures = 0;
    Runway rw = lfbo14r();
    printf("LFBO 14R: LOC %.0f m beyond the threshold (runway %.0f m), GS antenna %.0f m past / %.0f m off the centreline (TCH %.1f ft)\n",
           rw.loc_x, rw.length, rw.gs_x, rw.gs_y, tan(3 * D2R) * rw.gs_x / FT);
    for (k = 0; k < (int)(sizeof(cases) / sizeof(cases[0])); ++k) {
        const Case *c = &cases[k];
        Result r;
        if (only >= 0 && k != only) continue;
        r = fly(c, verbose);
        printf("[%s]\n  modes: %s\n", c->name, r.sequence);
        if (c->go_around_ra_ft > 0) {
            printf("  GA: TOGA at %.0f ft, min RA %.0f ft (height loss %.0f ft), FCU 3000 ft: max %.0f ft, after 120 s %.0f ft, AP lost %d\n",
                   c->go_around_ra_ft, r.min_ra_ga, c->go_around_ra_ft - r.min_ra_ga, r.ga_max_alt_ft, r.ga_final_alt_ft, r.ap_lost);
            /* The ALT* level-off itself is the pre-existing port ALT-capture and A/THR regulation (not part of
             * this module): only capture and a bounded overshoot are required here. */
            if (!r.ga_modes_ok || r.ap_lost || r.min_ra_ga < 40 || r.ga_max_alt_ft > 3400 || r.ga_final_alt_ft < 2700 ||
                !strstr(r.sequence, "ALT*")) { puts("  FAIL go-around"); ++failures; }
            continue;
        }
        {
            static const char *const cap[] = {"none", "APPR 1", "LAND 2", "LAND 3 SINGLE", "LAND 3 DUAL"};
            printf("  capability: %s at LAND, %s frozen at the 200 ft alert height, %s at touchdown, %d downgrade(s)\n",
                   cap[r.cap_land], cap[r.cap_alert], cap[r.cap_touchdown], r.cap_downgrades);
        }
        printf("  LAND %.0f ft, FLARE %.0f ft (%.1f s before touchdown), RETARD %.0f ft, ROLL OUT +%.2f s, A/THR off +%.1f s\n",
               r.ra_land, r.ra_flare, r.td_t - r.t_flare, r.ra_retard, r.t_rollout - r.td_t, r.t_athr_off - r.td_t);
        printf("  touchdown: %.0f m past threshold, %.2f m from centreline, sink %.0f fpm, IAS %.1f kt, pitch %.2f deg, "
               "crab %.2f deg, bank %.2f deg, tail clearance %.2f m\n",
               r.td_x, r.td_y, r.td_sink_fpm, r.td_ias, r.td_pitch, r.td_crab_deg, r.td_bank, r.td_tail_margin_m);
        printf("  roll-out: max |centreline deviation| %.2f m, stop %.0f m past threshold, %.2f m from centreline, AP lost early %d\n",
               r.max_y_rollout, r.stop_x, r.stop_y, r.ap_lost);
        if (!r.sequence_ok || !r.stopped || r.ap_lost ||
            r.ra_land < 360 || r.ra_land > 400 || r.ra_flare < 30 || r.ra_flare > 80 ||
            r.td_sink_fpm < 30 || r.td_sink_fpm > 480 || r.td_x < 150 || r.td_x > 1000 ||
            fabs(r.td_y) > 6 || r.max_y_rollout > 10 || r.td_tail_margin_m < .5 || r.stop_x > rw.length ||
            r.cap_land != A380X_LAND_CAPABILITY_LAND3_DUAL || r.cap_alert != A380X_LAND_CAPABILITY_LAND3_DUAL ||
            r.cap_touchdown != A380X_LAND_CAPABILITY_LAND3_DUAL || r.cap_downgrades ||
            (c->retard_late ? 0 : r.t_retard <= 0)) {
            puts("  FAIL: outside the bench acceptance window"); ++failures;
        }
    }
    if (failures) { printf("autoland closed-loop bench: %d case(s) failed\n", failures); return 1; }
    puts("autoland closed-loop bench (LFBO 14R, bench airframe, not X-Plane): all cases inside the acceptance window");
    return 0;
}
