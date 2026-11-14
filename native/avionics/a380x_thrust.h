/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 thrust levers, reverse levers, FADEC thrust rating / FLEX and A/THR instinctive disconnect:
 * XPLM-free core (docs/THRUST_LEVERS.md). The X-Plane adapter is a380x_thrust.inc.
 *
 * Translated from the pinned FlyByWire A380X sources (revision 2baa2b35):
 *   Part_Interior_Cockpit/model/behaviour/legacy/generated/A32NX_Interior_Engine.xml
 *     FBW_ENGINE_Lever_Throttle_SubTemplate2: detents IDLE 0 / CL 25 / FLX-MCT 35 / TOGA 45 deg TLA,
 *     MAX REV -20; the reverse lever animation is (1 - min(position, 1)) * 100 (engines 2/3 only).
 *   wasm/fbw_a380/src/FlyByWireInterface.cpp: 3D lever position table TLA -20/0/25/35/45 ->
 *     0/0/55/78/100 %, reversers disabled on engines 1 and 4, FADEC inputs (TAT, ATHR disconnect
 *     held, ATHR reset disable).
 *   wasm/fbw_a380/src/model/A380FadecComputer.cpp (+ _data.cpp):
 *     FLEX latch: set when flex temp valid > TAT, on ground and TLA == 35; cleared at TLA 25 or 45;
 *     FLEX rating = (candidate && on ground) || (in flight && latch);
 *     N1 command: TLA 0..25 IDLE->CLB, 25..35 CLB->FLEX|MCT, 35..45 FLEX|MCT->TOGA, reverse
 *     |TLA| 6..20 -> IDLE+1..REV (TLA floored at 0 in flight); IAS < 60 kt keep-out 62.5..73.5 % N1,
 *     IAS < 35 kt N1 <= 76.5 %;
 *     thrust_limit_type (TOGA/FLX/MCT/CLB/REV) with the higher detent's rating between detents;
 *     ATHR_disabled latched when the disconnect has been held >= 15 s (CompareToConstant_const),
 *     reset by ATHR_RESET_DISABLE (FMC initialisation, FlightManagementComputer.ts initSimVars).
 *   fadec_a380x/src/Fadec/EngineControl_A380X.cpp updateThrustLimits: FLEX temperature latched
 *     unless FLX is selected with every TLA at 35; after a FLEX take-off CLB = min(CLB, FLEX)
 *     until 10 s (TRANSITION_WAIT_TIME) after CLB is selected and CLB exceeds FLEX.
 *
 * The N1 numbers themselves come from the vendored FBW tables (a380x_ewd_fadec.cpp); this core
 * only selects and interpolates them. How the N1 command reaches X-Plane's engines is the adapter's
 * documented choice (see a380x_thrust_engine_command and docs/THRUST_LEVERS.md).
 */
#ifndef A380X_THRUST_H
#define A380X_THRUST_H
#include "a380x_detent_lever.h"

#define A380X_THRUST_ENGINES 4
#define A380X_TLA_IDLE 0.0
#define A380X_TLA_CL 25.0
#define A380X_TLA_FLX 35.0
#define A380X_TLA_TOGA 45.0
#define A380X_TLA_MAX_REV (-20.0)
#define A380X_TLA_REV_IDLE (-6.0)        /* FBW: reverse N1 is IDLE+1 down to |TLA| 6 */
#define A380X_REV_IDLE_POSITION 0.3       /* reverse lever travel (0..1) of IDLE REV = 6/20 */
#define A380X_ATHR_DISABLE_HOLD_S 15.0
#define A380X_FLEX_TRANSITION_WAIT_S 10.0
/* Stickiness (TLA degrees): CL and FLX/MCT capture within 1 deg and hold until the hand is 3.5 deg
 * away; IDLE and TOGA are end stops. Click bands: entry within 0.5 deg, re-armed beyond 1.5 deg. */
#define A380X_DETENT_CAPTURE_DEG 1.0
#define A380X_DETENT_ESCAPE_DEG 3.5
#define A380X_CLICK_IN_DEG 0.5
#define A380X_CLICK_OUT_DEG 1.5
/* Reverse lever (0 stowed .. 1 MAX REV): IDLE REV sticky, stowed and MAX REV end stops. */
#define A380X_REV_CAPTURE 0.04
#define A380X_REV_ESCAPE 0.10
#define A380X_REV_CLICK_IN 0.02
#define A380X_REV_CLICK_OUT 0.06
#define A380X_DISC_PRESS_S 0.08f
#define A380X_DISC_RELEASE_S 0.10f

enum { A380X_LIMIT_NONE = 0, A380X_LIMIT_CLB = 1, A380X_LIMIT_MCT = 2, A380X_LIMIT_FLX = 3,
       A380X_LIMIT_TOGA = 4, A380X_LIMIT_MREV = 5 };
/* How the N1 command reaches X-Plane: 0 raw pilot lever (no override), 1 scheduled N1 fraction
 * (default), 2 closed loop on X-Plane N1 (assumes X-Plane's N1 is on the FBW scale). */
enum { A380X_FADEC_RAW = 0, A380X_FADEC_SCHEDULED = 1, A380X_FADEC_N1_LOOP = 2 };
/* Thrust lever detents (lever 0..45) and reverse lever detents (0..1), indices of the click bits. */
enum { A380X_DETENT_IDLE, A380X_DETENT_CL, A380X_DETENT_FLX, A380X_DETENT_TOGA };
enum { A380X_REV_DETENT_STOWED, A380X_REV_DETENT_IDLE, A380X_REV_DETENT_MAX };

/* Pilot-lever calibration (fcu-calibration.ini ratios of sim/cockpit2/engine/actuators/throttle_ratio). */
typedef struct { double idle, cl, mct, toga, tolerance; } A380XThrustCalibration;

/* FADEC thrust limits (N1 %) at the current conditions, from the FBW tables. */
typedef struct {
    int valid;
    double idle, clb, mct, toga, rev;
    int flex_valid;    /* a FLEX limit exists (latched FLEX temperature > 0) */
    double flex;
} A380XThrustLimits;

/* X-Plane cockpit actuator state of one engine: reverse = prop_mode 3, ratio = throttle_ratio. */
typedef struct { int reverse; double ratio; } A380XXpLever;

typedef struct {
    int on_ground;
    int engine_running[A380X_THRUST_ENGINES];
    double n1[A380X_THRUST_ENGINES];          /* X-Plane N1 %, for the closed-loop mode only */
    double ias_knots;                         /* FBW V_ias_kn (low-speed N1 rules) */
    int flex_temp_valid;                      /* FMGC FLEX temperature valid (raw FMS value != 0) */
    double flex_temp_raw;                     /* L:AIRLINER_TO_FLEX_TEMP encoding: 0.1 = FLEX 0 C */
    int tat_valid;
    double tat_c;
    int athr_disconnect_held;                 /* instinctive disconnect (any source) held */
    int fmgc_powered;                         /* at least one FMC healthy */
    A380XThrustLimits limits;
} A380XThrustInputs;

typedef struct {
    A380XDetentLever lever[A380X_THRUST_ENGINES];    /* thrust lever TLA 0..45 */
    A380XDetentLever reverse[A380X_THRUST_ENGINES];  /* reverse lever 0..1 (engines 2 and 3 only) */
    A380XThrustCalibration cal;
    A380XXpLever written[A380X_THRUST_ENGINES];      /* X-Plane state last written or adopted */
    unsigned char dirty[A380X_THRUST_ENGINES];       /* the lever must be (re)written to X-Plane */
    int written_valid, synced, reverse_inhibited;
    /* FADEC */
    double tla[A380X_THRUST_ENGINES];                /* FBW TLA, reverse negative */
    unsigned char flex_latch[A380X_THRUST_ENGINES];
    unsigned char flex_rating[A380X_THRUST_ENGINES]; /* FBW rtb_y_mv: FLEX anchors the N1 law */
    int limit_type;                                  /* engine 1 FADEC (EWD) thrust_limit_type */
    double limit_n1;                                 /* its N1 (FBW thrust_limit_percent) */
    double n1_lever[A380X_THRUST_ENGINES];           /* N1 of the lever (FBW N1_TLA, pre low-speed rules) */
    double n1_command[A380X_THRUST_ENGINES];         /* N1 command after the low-speed rules (N1_c) */
    double latched_flex_temp;                        /* EngineControl latchedFlexTemperature (raw encoding) */
    int was_flex, transition, previous_type;
    double transition_start_s, time_s, flex_clb;
    int fadec_mode;
    double loop_trim[A380X_THRUST_ENGINES];
    /* Instinctive disconnect: held time (simulated) and the 15 s A/THR disable latch */
    double disc_held_s;
    int athr_disabled, previous_fmgc_powered;
    unsigned char disc_pressed[2];
    float disc_anim[2];
} A380XThrust;

static inline double a380x_thrust_lerp(double x, double x0, double x1, double y0, double y1)
{
    return x1 == x0 ? y1 : y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

static inline A380XThrustCalibration a380x_thrust_default_calibration(void)
{
    A380XThrustCalibration c = { 0.01, 25.0 / 45.0, 35.0 / 45.0, 1.0, 0.02 };
    return c;
}

/* Pilot-lever ratio -> TLA, as a380x_ewd_data.inc ewd_lever_tla: inside the calibrated tolerance
 * a lever IS in its detent (FBW's exact comparisons TLA == 25/35/45 then hold). */
static inline double a380x_thrust_ratio_to_tla(const A380XThrustCalibration *c, double ratio)
{
    if (!isfinite(ratio)) return 0.0;
    if (ratio <= c->idle + c->tolerance) return 0.0;
    if (fabs(ratio - c->cl) <= c->tolerance) return A380X_TLA_CL;
    if (fabs(ratio - c->mct) <= c->tolerance) return A380X_TLA_FLX;
    if (ratio >= c->toga - c->tolerance) return A380X_TLA_TOGA;
    if (ratio < c->cl) return a380x_thrust_lerp(ratio, c->idle, c->cl, 0.0, A380X_TLA_CL);
    if (ratio < c->mct) return a380x_thrust_lerp(ratio, c->cl, c->mct, A380X_TLA_CL, A380X_TLA_FLX);
    return a380x_thrust_lerp(ratio, c->mct, c->toga, A380X_TLA_FLX, A380X_TLA_TOGA);
}

/* Forward TLA -> pilot-lever ratio written to X-Plane (IDLE writes a true 0). */
static inline double a380x_thrust_tla_to_ratio(const A380XThrustCalibration *c, double tla)
{
    if (!(tla > 0.0)) return 0.0;
    if (tla <= A380X_TLA_CL) return a380x_thrust_lerp(tla, 0.0, A380X_TLA_CL, c->idle, c->cl);
    if (tla <= A380X_TLA_FLX) return a380x_thrust_lerp(tla, A380X_TLA_CL, A380X_TLA_FLX, c->cl, c->mct);
    return a380x_detent_clamp(a380x_thrust_lerp(tla, A380X_TLA_FLX, A380X_TLA_TOGA, c->mct, c->toga), 0.0, 1.0);
}

/* FlyByWireInterface mappingTable3d: TLA -> 3D thrust lever position, percent. */
static inline double a380x_thrust_lever_anim_percent(double tla)
{
    if (!(tla > 0.0)) return 0.0;
    if (tla <= 25.0) return a380x_thrust_lerp(tla, 0.0, 25.0, 0.0, 55.0);
    if (tla <= 35.0) return a380x_thrust_lerp(tla, 25.0, 35.0, 55.0, 78.0);
    return a380x_detent_clamp(a380x_thrust_lerp(tla, 35.0, 45.0, 78.0, 100.0), 0.0, 100.0);
}

static inline int a380x_thrust_has_reverser(int engine) { return engine == 1 || engine == 2; }

/* Reverse lever travel (0..1) -> TLA: FBW position 0..1 maps MAX REV..IDLE, i.e. TLA = -20 * travel. */
static inline double a380x_thrust_reverse_tla(double travel) { return A380X_TLA_MAX_REV * a380x_detent_clamp(travel, 0.0, 1.0); }

/* X-Plane reverse handle magnitude (throttle_ratio while prop_mode is 3): 0 at IDLE REV .. 1 at MAX REV. */
static inline double a380x_thrust_reverse_ratio(double travel)
{
    return a380x_detent_clamp((travel - A380X_REV_IDLE_POSITION) / (1.0 - A380X_REV_IDLE_POSITION), 0.0, 1.0);
}

static inline void a380x_thrust_init(A380XThrust *t, const A380XThrustCalibration *cal)
{
    static const A380XDetent lever[] = {
        {A380X_TLA_IDLE, 0.0, 0.0, A380X_CLICK_IN_DEG, A380X_CLICK_OUT_DEG},
        {A380X_TLA_CL, A380X_DETENT_CAPTURE_DEG, A380X_DETENT_ESCAPE_DEG, A380X_CLICK_IN_DEG, A380X_CLICK_OUT_DEG},
        {A380X_TLA_FLX, A380X_DETENT_CAPTURE_DEG, A380X_DETENT_ESCAPE_DEG, A380X_CLICK_IN_DEG, A380X_CLICK_OUT_DEG},
        {A380X_TLA_TOGA, 0.0, 0.0, A380X_CLICK_IN_DEG, A380X_CLICK_OUT_DEG},
    };
    static const A380XDetent reverse[] = {
        {0.0, 0.0, 0.0, A380X_REV_CLICK_IN, A380X_REV_CLICK_OUT},
        {A380X_REV_IDLE_POSITION, A380X_REV_CAPTURE, A380X_REV_ESCAPE, A380X_REV_CLICK_IN, A380X_REV_CLICK_OUT},
        {1.0, 0.0, 0.0, A380X_REV_CLICK_IN, A380X_REV_CLICK_OUT},
    };
    int i;
    memset(t, 0, sizeof(*t));
    t->cal = cal ? *cal : a380x_thrust_default_calibration();
    for (i = 0; i < A380X_THRUST_ENGINES; ++i) {
        a380x_detent_lever_init(&t->lever[i], A380X_TLA_IDLE, A380X_TLA_TOGA, lever, 4, 0.0);
        a380x_detent_lever_init(&t->reverse[i], 0.0, 1.0, reverse, 3, 0.0);
    }
    t->limit_type = A380X_LIMIT_NONE;
    t->previous_type = A380X_LIMIT_NONE;
    t->fadec_mode = A380X_FADEC_SCHEDULED;
}

/* Interlocks of the pedestal: a reverse lever lifts only with its thrust lever at IDLE, and a
 * thrust lever leaves IDLE only with its reverse lever stowed. */
static inline int a380x_thrust_lever_locked(const A380XThrust *t, int i) { return t->reverse[i].value > 0.0; }
static inline int a380x_thrust_reverse_locked(const A380XThrust *t, int i)
{
    return !a380x_thrust_has_reverser(i) || (t->lever[i].value > 0.0 && t->reverse[i].value <= 0.0);
}

/* Cockpit manipulator writes (lever TLA 0..45, reverse travel 0..1). */
static inline void a380x_thrust_drag_lever(A380XThrust *t, int i, double tla)
{
    if (i < 0 || i >= A380X_THRUST_ENGINES) return;
    if (a380x_thrust_lever_locked(t, i)) a380x_detent_lever_drag_locked(&t->lever[i], tla);
    else a380x_detent_lever_drag(&t->lever[i], tla);
    t->dirty[i] = 1;
}
static inline void a380x_thrust_drag_reverse(A380XThrust *t, int i, double travel)
{
    if (i < 0 || i >= A380X_THRUST_ENGINES) return;
    if (a380x_thrust_reverse_locked(t, i)) a380x_detent_lever_drag_locked(&t->reverse[i], travel);
    else a380x_detent_lever_drag(&t->reverse[i], travel);
    t->dirty[i] = 1;
}

/* What the plugin keeps in X-Plane's cockpit actuators for engine i. */
static inline A380XXpLever a380x_thrust_xp_lever(const A380XThrust *t, int i)
{
    A380XXpLever x;
    x.reverse = a380x_thrust_has_reverser(i) && t->reverse[i].value > 0.0 && !t->reverse_inhibited;
    x.ratio = x.reverse ? a380x_thrust_reverse_ratio(t->reverse[i].value)
                        : a380x_thrust_tla_to_ratio(&t->cal, t->lever[i].value);
    return x;
}

static inline int a380x_thrust_xp_differs(A380XXpLever a, A380XXpLever b)
{
    return a.reverse != b.reverse || !(fabs(a.ratio - b.ratio) <= 1e-5);
}

/* Foreign changes of X-Plane's actuators (hardware axis, keyboard/joystick commands, other
 * plugins) are adopted as lever positions, without stickiness, and become the new baseline (an
 * adopted value is not written back, so a hardware axis is never fought). The A380 refuses reverse
 * on the outer engines, in flight, and on an inner engine whose thrust lever is not at IDLE; while
 * an inner engine reverses, outer levers are held at IDLE (a single hardware axis then drives the
 * reverse amount). Refusals mark the engine dirty so the adapter restores the lever in X-Plane. */
static inline void a380x_thrust_sync_external(A380XThrust *t, const A380XXpLever read[A380X_THRUST_ENGINES])
{
    int i, inner_reverse = 0, first = !t->synced;
    for (i = 0; i < A380X_THRUST_ENGINES; ++i) {
        A380XXpLever r = read[i];
        if (!isfinite(r.ratio)) continue;
        r.ratio = a380x_detent_clamp(r.ratio, 0.0, 1.0);
        if (t->written_valid && !a380x_thrust_xp_differs(r, t->written[i])) continue;
        if (r.reverse) {
            if (!a380x_thrust_has_reverser(i) || t->reverse_inhibited
                || (t->lever[i].value > 0.0 && t->reverse[i].value <= 0.0)) {
                t->dirty[i] = 1;          /* no reverser / in flight / thrust lever not at IDLE: refused */
                continue;
            }
            a380x_detent_lever_set(&t->lever[i], 0.0);
            a380x_detent_lever_set(&t->reverse[i], A380X_REV_IDLE_POSITION + (1.0 - A380X_REV_IDLE_POSITION) * r.ratio);
        } else {
            if (t->reverse[i].value > 0.0) a380x_detent_lever_set(&t->reverse[i], 0.0);
            a380x_detent_lever_set(&t->lever[i], a380x_thrust_ratio_to_tla(&t->cal, r.ratio));
        }
        t->written[i] = r;
        t->dirty[i] = 0;
    }
    t->written_valid = 1;
    for (i = 1; i <= 2; ++i) inner_reverse = inner_reverse || t->reverse[i].value > 0.0;
    if (inner_reverse) for (i = 0; i < A380X_THRUST_ENGINES; i += 3) if (t->lever[i].value > 0.0) {
        a380x_detent_lever_set(&t->lever[i], 0.0);
        t->dirty[i] = 1;
    }
    if (first) {   /* the levers found at load are not detent entries */
        for (i = 0; i < A380X_THRUST_ENGINES; ++i) {
            a380x_detent_lever_rearm(&t->lever[i]);
            a380x_detent_lever_rearm(&t->reverse[i]);
        }
        t->synced = 1;
    }
}

/* Keyboard / joystick-button throttle commands (sim/engines/throttle_up, throttle_down and their
 * per-engine forms) step X-Plane's handle without the cockpit lever's magnetic detents, so CL (the top
 * of the A/THR active range) and FLX/MCT were hard to hit. While such a command is held, a lever that
 * reaches (within the calibration tolerance) or crosses CL or FLX/MCT stops there and stays (*hold,
 * < 0 = none) until the command is released; the next press moves it on. previous = the ratio last
 * written or adopted, next = X-Plane's new ratio. Returns the ratio to adopt. */
static inline double a380x_thrust_key_detent(const A380XThrustCalibration *c, double previous, double next,
                                             double *hold)
{
    double d[2], band;
    int k, up;
    if (*hold >= 0.0) return *hold;
    if (!isfinite(previous) || !isfinite(next) || next == previous) return next;
    band = c->tolerance > 0.0 ? c->tolerance : 0.0;
    up = next > previous;
    d[0] = up ? c->cl : c->mct;
    d[1] = up ? c->mct : c->cl;
    for (k = 0; k < 2; ++k)
        if ((up && previous < d[k] - band && next >= d[k] - band)
            || (!up && previous > d[k] + band && next <= d[k] + band)) {
            *hold = d[k];
            return d[k];
        }
    return next;
}

/* Called by the adapter after it wrote engine i's actuators to X-Plane. */
static inline void a380x_thrust_mark_written(A380XThrust *t, int i, A380XXpLever written)
{
    t->written[i] = written;
    t->dirty[i] = 0;
}

/* Detent click events of all levers since the last call (bit per lever and detent); any != 0. */
static inline unsigned a380x_thrust_take_clicks(A380XThrust *t)
{
    unsigned clicks = 0;
    int i;
    for (i = 0; i < A380X_THRUST_ENGINES; ++i) {
        clicks |= a380x_detent_lever_take_clicks(&t->lever[i]) << (8 * i);
        clicks |= a380x_detent_lever_take_clicks(&t->reverse[i]) << (8 * i + 4);
    }
    return clicks;
}

static inline void a380x_thrust_tick(A380XThrust *t, double wall_dt)
{
    int i;
    for (i = 0; i < A380X_THRUST_ENGINES; ++i) {
        a380x_detent_lever_tick(&t->lever[i], wall_dt);
        a380x_detent_lever_tick(&t->reverse[i], wall_dt);
    }
}

/* EngineControl_A380X::updateThrustLimits: the FLEX temperature used for the limits, latched while
 * FLX is selected with every TLA at 35 and during the FLEX-to-CLB transition. Call before the
 * adapter computes the limits; uses the previous frame's thrust limit type. */
static inline double a380x_thrust_latch_flex_temp(A380XThrust *t, const A380XThrustInputs *in)
{
    int i, all_flx = 1;
    for (i = 0; i < A380X_THRUST_ENGINES; ++i) all_flx = all_flx && t->tla[i] == A380X_TLA_FLX;
    if (!t->transition && (t->limit_type != A380X_LIMIT_FLX || !all_flx))
        t->latched_flex_temp = in->flex_temp_valid && isfinite(in->flex_temp_raw) ? in->flex_temp_raw : 0.0;
    return t->latched_flex_temp;
}

/* FBW thrust_limit_type of one FADEC; *n1 = the rating's limit (FBW thrust_limit_percent). */
static inline int a380x_thrust_limit_type(double tla, int on_ground, int engine_operative, int flex,
                                          const A380XThrustLimits *l, double clb, double *n1)
{
    double angle = on_ground ? tla : fmax(0.0, tla);
    int type;
    if (!on_ground || !engine_operative) {
        if (angle > 35.0) type = A380X_LIMIT_TOGA;
        else if (angle > 25.0) type = flex ? A380X_LIMIT_FLX : A380X_LIMIT_MCT;
        else if (angle >= 0.0) type = A380X_LIMIT_CLB;
        else type = A380X_LIMIT_MREV;
    } else if (angle >= 0.0) {
        type = (!flex || angle > 35.0) ? A380X_LIMIT_TOGA : A380X_LIMIT_FLX;
    } else {
        type = A380X_LIMIT_MREV;
    }
    switch (type) {
    case A380X_LIMIT_TOGA: *n1 = l->toga; break;
    case A380X_LIMIT_FLX: *n1 = l->flex; break;
    case A380X_LIMIT_MCT: *n1 = l->mct; break;
    case A380X_LIMIT_CLB: *n1 = clb; break;
    default: *n1 = l->rev; break;
    }
    return type;
}

/* FBW N1 law of the lever (rtb_N1c before the low-speed rules). */
static inline double a380x_thrust_lever_n1(double tla, int on_ground, int flex, const A380XThrustLimits *l, double clb)
{
    double angle = on_ground ? tla : fmax(0.0, tla), anchor = flex ? l->flex : l->mct;
    if (angle < 0.0) {
        angle = fmax(fabs(angle), 6.0);
        return a380x_thrust_lerp(angle, 6.0, 20.0, fabs(l->idle + 1.0), fabs(l->rev));
    }
    if (angle <= 25.0) return a380x_thrust_lerp(angle, 0.0, 25.0, l->idle, clb);
    if (angle <= 35.0) return a380x_thrust_lerp(angle, 25.0, 35.0, clb, anchor);
    return a380x_thrust_lerp(angle, 35.0, 45.0, anchor, l->toga);
}

/* FBW FADEC low-speed rules applied to N1_end (IAS < 60 kt keep-out band, IAS < 35 kt cap). */
static inline double a380x_thrust_low_speed_n1(double n1, double ias)
{
    if (!isfinite(ias)) ias = 0.0;
    if (ias < 60.0 && n1 > 62.5 && n1 < 73.5) n1 = n1 < 68.0 ? 62.5 : 73.5;
    if (ias < 35.0) n1 = fmin(n1, 76.5);
    return n1;
}

/* One FADEC step after the limits are known (simulated dt; 0 while paused). */
static inline void a380x_thrust_update(A380XThrust *t, const A380XThrustInputs *in, double sim_dt)
{
    const A380XThrustLimits *l = &in->limits;
    int i, candidate;
    double clb = l->clb;
    if (!isfinite(sim_dt) || sim_dt < 0.0) sim_dt = 0.0;
    t->time_s += sim_dt;
    /* FBW floors TLA at 0 in flight (no reverse); X-Plane is then kept in forward idle. */
    if (t->reverse_inhibited != !in->on_ground) {
        t->reverse_inhibited = !in->on_ground;
        for (i = 1; i <= 2; ++i) if (t->reverse[i].value > 0.0) t->dirty[i] = 1;
    }
    for (i = 0; i < A380X_THRUST_ENGINES; ++i)
        t->tla[i] = t->reverse[i].value > 0.0 ? a380x_thrust_reverse_tla(t->reverse[i].value) : t->lever[i].value;
    /* FLEX candidate: a valid FLEX temperature (raw 0.1 = FLEX 0 C) above TAT. */
    candidate = in->flex_temp_valid && in->tat_valid && isfinite(in->flex_temp_raw) && in->flex_temp_raw != 0.0
        && (fabs(in->flex_temp_raw - 0.1) < 1e-6 ? 0.0 : in->flex_temp_raw) > in->tat_c;
    for (i = 0; i < A380X_THRUST_ENGINES; ++i) {
        t->flex_latch[i] = (unsigned char)((candidate && in->on_ground && t->tla[i] == A380X_TLA_FLX) || t->flex_latch[i]);
        t->flex_latch[i] = (unsigned char)(t->flex_latch[i] && t->tla[i] != A380X_TLA_CL && t->tla[i] != A380X_TLA_TOGA);
        t->flex_rating[i] = (unsigned char)(((candidate && in->on_ground) || (!in->on_ground && t->flex_latch[i]))
                                            && l->flex_valid);
    }
    /* EngineControl FLEX -> CLB transition: CLB limited to FLEX for 10 s after CLB, then while CLB <= FLEX. */
    if (t->previous_type != A380X_LIMIT_FLX && t->limit_type == A380X_LIMIT_FLX) t->was_flex = 1;
    else if (t->limit_type == A380X_LIMIT_TOGA) t->was_flex = 0;
    if (t->was_flex && !t->transition && t->limit_type == A380X_LIMIT_CLB) {
        t->transition = 1;
        t->transition_start_s = t->time_s;
    } else if (!t->was_flex) {
        t->transition = 0;
        t->transition_start_s = 0.0;
    }
    if (t->transition && t->time_s - t->transition_start_s - A380X_FLEX_TRANSITION_WAIT_S > 0.0 && clb > l->flex)
        t->was_flex = 0;
    if (t->was_flex && l->flex_valid) clb = fmin(clb, l->flex);
    t->flex_clb = clb;
    t->previous_type = t->limit_type;
    if (!l->valid) {
        t->limit_type = A380X_LIMIT_NONE;
        t->limit_n1 = 0.0;
        for (i = 0; i < A380X_THRUST_ENGINES; ++i) t->n1_lever[i] = t->n1_command[i] = 0.0;
    } else {
        for (i = 0; i < A380X_THRUST_ENGINES; ++i) {
            double limit;
            int type = a380x_thrust_limit_type(t->tla[i], in->on_ground, in->engine_running[i], t->flex_rating[i], l, clb, &limit);
            if (i == 0) { t->limit_type = type; t->limit_n1 = limit; }
            t->n1_lever[i] = a380x_thrust_lever_n1(t->tla[i], in->on_ground, t->flex_rating[i], l, clb);
            t->n1_command[i] = a380x_thrust_low_speed_n1(t->n1_lever[i], in->ias_knots);
        }
    }
    /* Instinctive disconnect held >= 15 s (simulated time): A/THR lost, alpha floor included. */
    t->disc_held_s = in->athr_disconnect_held ? t->disc_held_s + sim_dt : 0.0;
    if (t->disc_held_s >= A380X_ATHR_DISABLE_HOLD_S) t->athr_disabled = 1;
    /* ATHR_RESET_DISABLE: FMGC (FMC) power-up on the ground. */
    if (in->fmgc_powered && !t->previous_fmgc_powered && in->on_ground) t->athr_disabled = 0;
    t->previous_fmgc_powered = in->fmgc_powered;
}

/* X-Plane engine command (normalised throttle 0..1, magnitude in reverse) for the FBW N1 command.
 * SCHEDULED: the N1 command as a fraction of the IDLE..TOGA N1 range (reverse: IDLE+1..REV), so
 * TOGA keeps X-Plane's full rated throttle and FLEX/MCT/CLB are the FBW fractions below it,
 * whatever X-Plane's own N1 scale is. N1_LOOP adds an integral trim on the measured X-Plane N1. */
static inline double a380x_thrust_schedule(const A380XThrust *t, int i, double n1, const A380XThrustLimits *l)
{
    if (!l->valid) return 0.0;
    if (t->tla[i] < 0.0 && t->reverse[i].value > 0.0)
        return a380x_detent_clamp((n1 - (l->idle + 1.0)) / (l->rev - (l->idle + 1.0)), 0.0, 1.0);
    if (!(l->toga > l->idle)) return 0.0;
    return a380x_detent_clamp((n1 - l->idle) / (l->toga - l->idle), 0.0, 1.0);
}

static inline double a380x_thrust_engine_command(A380XThrust *t, int i, const A380XThrustInputs *in, double sim_dt)
{
    double u = a380x_thrust_schedule(t, i, t->n1_command[i], &in->limits);
    if (t->fadec_mode == A380X_FADEC_N1_LOOP && in->engine_running[i] && isfinite(in->n1[i]) && in->limits.valid
        && t->tla[i] >= 0.0 && isfinite(sim_dt) && sim_dt > 0.0) {
        t->loop_trim[i] = a380x_detent_clamp(t->loop_trim[i] + 0.01 * (t->n1_command[i] - in->n1[i]) * sim_dt, -0.5, 0.5);
        u = a380x_detent_clamp(u + t->loop_trim[i], 0.0, 1.0);
    } else if (t->fadec_mode != A380X_FADEC_N1_LOOP || !in->engine_running[i]) {
        t->loop_trim[i] = 0.0;
    }
    return u;
}

/* A/THR authority: the engine command of the lever's own N1 (FBW clamps A/THR to [IDLE, N1_TLA]). */
static inline double a380x_thrust_athr_ceiling(const A380XThrust *t, int i, const A380XThrustLimits *l)
{
    return a380x_thrust_schedule(t, i, t->n1_lever[i], l);
}

/* Instinctive-disconnect buttons (0 left of lever 1, 1 right of lever 4): wall-time push travel. */
static inline void a380x_thrust_disc_buttons(A380XThrust *t, float wall_dt)
{
    int k;
    if (!isfinite(wall_dt) || wall_dt < 0.0f) wall_dt = 0.0f;
    for (k = 0; k < 2; ++k) {
        if (t->disc_pressed[k]) t->disc_anim[k] = fminf(1.0f, t->disc_anim[k] + wall_dt / A380X_DISC_PRESS_S);
        else t->disc_anim[k] = fmaxf(0.0f, t->disc_anim[k] - wall_dt / A380X_DISC_RELEASE_S);
    }
}

#endif
