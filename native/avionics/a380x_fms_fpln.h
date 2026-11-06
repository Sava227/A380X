/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * A380X flight-plan model and operations (WP-D).
 * Spec: docs/mfd-spec/05-fms-fpln.md §2 (model, predictions feed, naming, operations) and the page
 * behaviours §3-§14; 07-fmc-logic.md §2.3 (plans); 12-architecture.md §3/§7.
 * Translation of the FlyByWire flight-plan manager (@fmgc/flightplanning, pinned 2baa2b35):
 * FlightPlanManager indices, BaseFlightPlan segments, FlightPlanService operations, A380 FpmConfig
 * (every lateral revision of ACTIVE goes to TMPY, 200 legs, 3 SEC plans).
 *
 * Simulator independent C11, no heap use after a380x_fpln_create (all storage is fixed-size).
 * Not thread safe. Uses the navigation database (a380x_navdata.h) for airports, runways,
 * procedures and airways.
 *
 * ============================================================================================
 * QUICK GUIDE
 * ============================================================================================
 *  Store:   A380XFlightPlans *fp = a380x_fpln_create(nav);   ... a380x_fpln_destroy(fp);
 *  Plans:   index 0 ACTIVE, 1 TMPY, 2 UPLINK (unused), 3..5 SEC1..SEC3 (A380XPlanIndex, = FBW).
 *           const A380XFplnPlan *p = a380x_fpln_plan(fp, idx);    NULL when it does not exist.
 *           Each plan has a primary route `pri` and an alternate route `alt` ("altn" flag = 1).
 *  Page:    plan shown for category active = a380x_fpln_loaded_index(fp, 0) (TMPY if it exists,
 *           else ACTIVE); for SECn = a380x_fpln_loaded_index(fp, n). Rebuild display lines when
 *           (plan->version, plan->active_leg_index) change; iterate with A380XFplnLineIter.
 *  Edits:   every operation takes (plan index, altn, element index). Called on ACTIVE it creates
 *           TMPY (copy of ACTIVE) and edits TMPY; on TMPY/SEC it edits in place. Operations are
 *           transactional: on error nothing changes (not even TMPY creation). Return value 0 =
 *           A380X_FPLN_OK, < 0 = A380XFplnError (map to FMS messages, see enum).
 *  TMPY:    a380x_fpln_tmpy_insert(fp, &aircraft) / a380x_fpln_tmpy_erase(fp).
 *  FMS:     call a380x_fpln_update(fp, &aircraft) every FMS cycle (sequencing of ACTIVE).
 *  ND:      for each element of a route, a380x_fpln_leg_path() gives drawable geometry.
 *
 * Element indices are "flat" indices into route->el[] (legs and discontinuities). Segment order
 * (A380XFplnSegment) is always non-decreasing along el[].
 */
#ifndef A380X_FMS_FPLN_H
#define A380X_FMS_FPLN_H

#include <stdint.h>
#include "a380x_navdata.h"

#ifdef __cplusplus
extern "C" {
#endif

#define A380X_FPLN_MAX_ELEMENTS 200   /* FpmConfigs.A380.MAX_NUM_LEGS, per route */
#define A380X_FPLN_IDENT_LEN     12   /* "EGLL27R", "(SPDLIM)", "INTCPT", "ABC/12" + NUL */
#define A380X_FPLN_ANNOT_LEN     16   /* "C090\xc2\xb0", "10 ABC", "HOLD R", "UMBO1A" + NUL (UTF-8) */
#define A380X_FPLN_SEC_COUNT      3
#define A380X_FPLN_FIX_INFOS      4
#define A380X_FPLN_AWY_LINES     11   /* AIRWAYS page lines (spec 05 §9.1) */
#define A380X_FPLN_MAX_PWP       16   /* pseudo waypoints accepted from a prediction feed */

typedef enum {                       /* FlightPlanIndex (FlightPlanManager.ts:32-37) */
    A380X_PLAN_ACTIVE = 0, A380X_PLAN_TEMPORARY = 1, A380X_PLAN_UPLINK = 2,
    A380X_PLAN_SEC1 = 3, A380X_PLAN_SEC2 = 4, A380X_PLAN_SEC3 = 5,
    A380X_PLAN_COUNT = 6
} A380XPlanIndex;

typedef enum {                       /* allLegs order (BaseFlightPlan.ts:737-755) */
    A380X_SEG_ORIGIN = 0, A380X_SEG_DEP_RWY_TRANS, A380X_SEG_SID, A380X_SEG_SID_TRANS,
    A380X_SEG_ENROUTE,
    A380X_SEG_STAR_TRANS, A380X_SEG_STAR, A380X_SEG_STAR_RWY_TRANS, A380X_SEG_APPR_VIA,
    A380X_SEG_APPROACH, A380X_SEG_DESTINATION, A380X_SEG_MISSED,
    A380X_SEG_COUNT
} A380XFplnSegment;

typedef enum {                       /* SegmentClass: <= SID_TRANS / ENROUTE / >= STAR_TRANS */
    A380X_SEGCLASS_DEPARTURE = 0, A380X_SEGCLASS_ENROUTE, A380X_SEGCLASS_ARRIVAL
} A380XFplnSegmentClass;

typedef enum { A380X_WPDESC_NONE = 0, A380X_WPDESC_AIRPORT, A380X_WPDESC_RUNWAY, A380X_WPDESC_OTHER } A380XFplnWpDesc;
typedef enum { A380X_CSTR_UNKNOWN = 0, A380X_CSTR_CLB, A380X_CSTR_DES } A380XFplnCstrType;
typedef enum { A380X_TURN_EITHER = 0, A380X_TURN_LEFT, A380X_TURN_RIGHT } A380XFplnTurn;
typedef enum { A380X_HOLD_COMPUTED = 0, A380X_HOLD_DATABASE = 1, A380X_HOLD_PILOT = 2 } A380XFplnHoldType;

typedef enum {                       /* operation results; page maps them to FMS messages */
    A380X_FPLN_OK               =  0,
    A380X_FPLN_E_ARG            = -1, /* bad plan/index/argument (programming error, no message) */
    A380X_FPLN_E_NO_PLAN        = -2, /* plan does not exist */
    A380X_FPLN_E_NOT_IN_DB      = -3, /* NOT IN DATABASE */
    A380X_FPLN_E_NOT_ALLOWED    = -4, /* NOT ALLOWED */
    A380X_FPLN_E_NO_INTERSECTION= -5, /* NO INTERSECTION FOUND */
    A380X_FPLN_E_FULL           = -6, /* 200 elements reached (FBW: silently refused) */
    A380X_FPLN_E_TMPY_EXISTS    = -7  /* SEC activate/swap refused while TMPY exists */
} A380XFplnError;

/* ------------------------------------------------------------------------------------------ */
/* Leg                                                                                         */
/* ------------------------------------------------------------------------------------------ */

typedef struct {                     /* AltitudeConstraint */
    char  desc;                      /* 0 none, '@' at, '+' at/above, '-' at/below, 'B' between (alt1 high,
                                        alt2 low), 'C','G','H','I','J','V','X','Y' (ProcedureLeg.ts:123-175) */
    float alt1_ft, alt2_ft;          /* NAN if absent */
} A380XFplnAltCstr;

typedef struct {                     /* SpeedConstraint */
    char  desc;                      /* 0 none, '@', '+', '-'; pilot entries are always '-' */
    float speed;                     /* kt (Mach if < 2.0) */
} A380XFplnSpdCstr;

typedef struct {                     /* HoldData; dist and time mutually exclusive (other NAN) */
    int   valid;
    float inbound_crs_mag;
    float dist_nm, time_min;
    A380XFplnTurn turn;
    A380XFplnHoldType type;
} A380XFplnHold;

typedef struct { int valid; float to_alt_ft, dist_before_term_nm; int ignored; } A380XFplnCruiseStep;

typedef struct {                     /* spec 05 §2.3 mfd_wpt_pred_t; valid = 0 until VNAV exists */
    int   valid;
    float seconds_from_present;
    float altitude_ft;               /* NAN = none */
    float speed;                     /* kt, or Mach if < 2 */
    int   alt_cstr_met, spd_cstr_met;
    float efob_kg;                   /* NAN if none (FBW lb; port stores kg like the FMS) */
    float dist_from_aircraft_nm;
    uint8_t wind_kind;               /* 0 none, 1 vector, 2 tail/head component */
    float wind_dir_true_deg, wind_speed_kt, tailwind_kt;
} A380XFplnWptPred;

enum {                               /* A380XFplnLeg.flags */
    A380X_LEGF_OVERFLY          = 1u << 0,  /* definition.overfly */
    A380X_LEGF_ORIGIN           = 1u << 1,  /* FlightPlanLegFlags.Origin (origin airport/runway leg) */
    A380X_LEGF_DIRTO_TP         = 1u << 2,  /* DIR TO turning point ("T-P") */
    A380X_LEGF_PENDING_DIRTO_TP = 1u << 3,  /* T-P in TMPY, moved to PPOS at INSERT */
    A380X_LEGF_HOLD_IMM_EXIT    = 1u << 4,  /* holdImmExit */
    A380X_LEGF_CALC_VALID       = 1u << 5,  /* geometry computed (leg_dist_nm, cum_* valid) */
    A380X_LEGF_TOO_STEEP_AFTER  = 1u << 6,  /* calculated.endsInTooSteepPath (VNAV, not set yet) */
    A380X_LEGF_DISCONTINUITY    = 1u << 7,  /* element is a discontinuity; other fields ignored */
    A380X_LEGF_AUTO_DISCO       = 1u << 8,  /* discontinuity created by procedure stringing */
    A380X_LEGF_DESTINATION      = 1u << 9,  /* destination airport/runway leg */
    A380X_LEGF_AIRWAY           = 1u << 10, /* inserted from an airway (annotation = airway) */
    A380X_LEGF_PILOT            = 1u << 11, /* inserted by the pilot (NEXT WPT, DIR TO fix, AIRWAYS DCT) */
    A380X_LEGF_ABEAM            = 1u << 12, /* DIR TO WITH ABEAM point */
    A380X_LEGF_MISSED_START     = 1u << 13, /* first leg of the missed approach */
    A380X_LEGF_EST_GEOMETRY     = 1u << 14, /* termination estimated (CA/FA/VA/HA altitude legs) */
    A380X_LEGF_FIX_UNRESOLVED   = 1u << 15, /* procedure fix not found in the database */
    A380X_LEGF_DIRTO_FIX        = 1u << 16  /* the DF leg created by DIR TO */
};

typedef struct {
    uint32_t         flags;
    A380XLegType     type;            /* effective type (an edited hold becomes HM) */
    A380XFplnSegment segment;
    A380XFplnWpDesc  wp_desc;
    char   ident[A380X_FPLN_IDENT_LEN];       /* display ident (spec 05 §2.4) */
    char   annotation[A380X_FPLN_ANNOT_LEN];  /* upper-left text (spec 05 §2.4) */
    char   procedure_ident[A380X_NAV_IDENT_LEN]; /* SID/STAR/approach ident, "" */
    char   airway_ident[A380X_NAV_IDENT_LEN];    /* airway the leg was inserted from, "" */
    A380XNavRef fix;                  /* termination fix (FA/FC/FD/FM: start fix); lat NAN if none */
    A380XNavRef navaid;               /* recommended navaid */
    A380XNavRef arc_center;           /* RF centre */
    char   desc[5];                   /* database waypoint description code, "" for pilot legs */
    float  course_deg;                /* definition.course (magnetic unless course_true), NAN */
    int    course_true;
    float  theta_deg, rho_nm, arc_radius_nm, rnp_nm;   /* NAN if absent */
    A380XFplnTurn turn;               /* definition.turnDirection */
    float  length_nm, length_min;     /* definition.length / lengthTime, NAN */
    float  vertical_angle_deg;        /* FPA, NAN if none */
    float  magvar_deg;                /* variation used for this leg (east +) */
    A380XFplnAltCstr alt_db, alt_pilot;   /* effective = pilot if set, else db (a380x_fpln_eff_alt) */
    A380XFplnSpdCstr spd_db, spd_pilot;
    float  trans_alt_ft;              /* procedure transition altitude from the database, NAN */
    A380XFplnCstrType cstr_type;      /* leg.constraintType */
    A380XFplnHold hold_default, hold_modified;   /* HX legs */
    A380XFplnCruiseStep cruise_step;
    /* calculated geometry (straight segments between termination points; no turn transitions) */
    double term_lat, term_lon;        /* termination point (= fix for XF), NAN if indeterminate */
    double start_lat, start_lon;      /* path start (previous termination / F-type fix), NAN */
    float  leg_dist_nm;               /* path length of this leg, NAN */
    float  path_true_crs;             /* initial true course of the path, NAN */
    float  cum_dist_nm;               /* calculated.cumulativeDistance (restarts at MISSED and ALTN) */
    float  cum_dist_with_trans_nm;    /* = cum_dist_nm (no turn transitions modelled yet) */
    A380XFplnWptPred pred;            /* valid = 0 (no VNAV yet) */
} A380XFplnLeg;

/* ------------------------------------------------------------------------------------------ */
/* Route (primary or alternate) and plan                                                       */
/* ------------------------------------------------------------------------------------------ */

typedef struct {
    int  count;                                /* elements incl. discontinuities */
    A380XFplnLeg el[A380X_FPLN_MAX_ELEMENTS];
    int  first_missed_appr_index;              /* == count if no missed approach */
    int  last_enroute_index;                   /* last element with segment <= ENROUTE, -1 */
    int  destination_leg_index;                /* -1 if none (MAP / dest runway / dest airport leg) */
    char origin_icao[A380X_NAV_ICAO_LEN], dest_icao[A380X_NAV_ICAO_LEN];   /* "" if none */
    char origin_rwy[A380X_NAV_IDENT_LEN];      /* CIFP runway ident "RW27R", "" */
    char dest_rwy[A380X_NAV_IDENT_LEN];
    char sid[A380X_NAV_IDENT_LEN], sid_trans[A380X_NAV_IDENT_LEN];
    char star[A380X_NAV_IDENT_LEN], star_trans[A380X_NAV_IDENT_LEN];
    char approach[A380X_NAV_IDENT_LEN];        /* CIFP ident "I27R"; display name via a380x_fpln_approach_name */
    char appr_via[A380X_NAV_IDENT_LEN];
    int  sid_none, star_none, appr_none;       /* "NONE" explicitly selected (pages show NONE) */
    /* database defaults for the perf store (WP-C owns the effective TA/TL): NAN if unknown */
    float db_trans_alt_ft;                     /* origin airport transition altitude */
    float db_trans_level_ft;                   /* destination transition level, feet (FL x 100) */
    float origin_elev_ft, dest_elev_ft;        /* NAN if unknown */
} A380XFplnRoute;

typedef struct {                     /* FIX INFO entry (FixInfoEntry) */
    int   valid;
    A380XNavFix fix;
    int   radial_count;              /* 0..2 */
    float radial_mag[2], radial_true[2];
    int   radius_valid;
    float radius_nm;
} A380XFplnFixInfo;

typedef struct {                     /* AIRWAYS page line (spec 05 §9.2) */
    char via[A380X_NAV_IDENT_LEN];   /* airway ident, "DCT", "" */
    int  via_locked, to_locked;
    A380XNavRef from;                /* line 0: revised fix; line k: TO of line k-1 */
    A380XNavRef to;                  /* valid when to_locked */
} A380XFplnAwyLine;

typedef struct {                     /* pending airway entry (LocalPendingAirways) */
    int active;
    int altn;
    int at_index;                    /* revised element index the airways start from */
    int line_count;                  /* lines shown (>= 1 while active) */
    A380XFplnAwyLine line[A380X_FPLN_AWY_LINES];
} A380XFplnAirwayEntry;

enum {                               /* A380XFplnPlan.flags (FlightPlanFlags) */
    A380X_PLANF_COPIED_FROM_ACTIVE = 1u << 0,
    A380X_PLANF_MANUAL_CREATION    = 1u << 1,
    A380X_PLANF_DIRTO_PENDING      = 1u << 2  /* TMPY created by DIR TO */
};

typedef struct {
    int      exists;
    uint32_t version;                /* increments on every structural / constraint change */
    uint32_t flags;
    int      active_leg_index;       /* TO leg of the primary route; -1 if none */
    A380XFplnRoute pri, alt;
    A380XFplnAirwayEntry awy;
    A380XFplnFixInfo fix_info[A380X_FPLN_FIX_INFOS];
} A380XFplnPlan;

/* Aircraft state used by DIR TO, TMPY INSERT and sequencing. */
typedef struct {
    int    valid;                    /* position valid (IRS/GPS) */
    double lat, lon;
    float  alt_ft;                   /* indicated altitude (CA/FA/VA/HA termination) */
    float  true_track_deg;           /* NAN if invalid (FBW then uses 0) */
    float  gs_kt;
    int    on_ground;
    int    nav_mode;                 /* 1 = lateral NAV engaged (sequencing of manual legs) */
} A380XFplnAircraft;

typedef struct A380XFlightPlans {
    A380XNavdata *nav;
    A380XFplnPlan plan[A380X_PLAN_COUNT];
    int  pilot_wpt_seq;              /* numbering of LLxx / PBDxx / PBXxx */
    /* private work areas (transactional edit buffer, procedure building) */
    A380XFplnPlan  scratch;
    A380XFplnRoute work[4];
} A380XFlightPlans;

/* Store: ~1.5 MB (sizeof(A380XFlightPlans)); allocate with create (or embed and call init). */
A380XFlightPlans *a380x_fpln_create(A380XNavdata *nav);
void a380x_fpln_destroy(A380XFlightPlans *fp);
void a380x_fpln_init(A380XFlightPlans *fp, A380XNavdata *nav);
/* FBW reset (DATA/STATUS SWAP, DONE): deletes every plan. */
void a380x_fpln_reset(A380XFlightPlans *fp);

const A380XFplnPlan  *a380x_fpln_plan(const A380XFlightPlans *fp, int plan_index);   /* NULL if absent */
const A380XFplnRoute *a380x_fpln_route(const A380XFlightPlans *fp, int plan_index, int altn);
int  a380x_fpln_has_tmpy(const A380XFlightPlans *fp);
/* Plan the F-PLN family shows: sec = 0 -> TMPY if it exists else ACTIVE; sec = 1..3 -> SECn. */
int  a380x_fpln_loaded_index(const A380XFlightPlans *fp, int sec);
/* fromLegIndex: last non-discontinuity element before the active leg, -1. */
int  a380x_fpln_from_leg_index(const A380XFplnPlan *p);

/* ---------------- leg helpers (pure) ---------------- */
A380XFplnSegmentClass a380x_fpln_segment_class(A380XFplnSegment s);
int  a380x_fpln_is_disco(const A380XFplnLeg *l);
/* revision-menu "dbFix": a leg, XF type, not the DIR TO T-P (spec 05 §4). */
int  a380x_fpln_is_db_fix(const A380XFplnLeg *l);
/* isConstraintRevisionAllowed (spec 05 §3.9): leg, not runway, XF, not T-P. */
int  a380x_fpln_constraint_revision_allowed(const A380XFplnLeg *l);
/* Effective constraints (spec 05 §2.2): pilot if set, else database (alt: unless desc G/H). */
A380XFplnAltCstr a380x_fpln_eff_alt(const A380XFplnLeg *l);
A380XFplnSpdCstr a380x_fpln_eff_spd(const A380XFplnLeg *l);
/* Approach display name of a route ("ILS27R", "RNV22L-X"); "" if none. */
void a380x_fpln_approach_name(A380XFlightPlans *fp, const A380XFplnRoute *r, char *out, size_t n);
/* Display runway ident "EGLL27R" for origin (dest = 0) or destination (dest = 1); "" if none. */
void a380x_fpln_runway_display(const A380XFplnRoute *r, int dest, char *out, size_t n);

/* ------------------------------------------------------------------------------------------ */
/* Plan-level operations                                                                       */
/* ------------------------------------------------------------------------------------------ */

/* INIT FROM/TO(/ALTN): deletes TMPY (when plan == ACTIVE) and the plan, recreates it with
 * ORIGIN airport leg, discontinuity, destination airport leg (+ alternate if altn != NULL/"").
 * Edits in place (FBW newCityPair does not go through TMPY). NOT_IN_DB if an airport is unknown. */
int a380x_fpln_new_city_pair(A380XFlightPlans *fp, int plan, const char *from, const char *to,
                             const char *altn);
/* INIT ALTN: rebuilds the alternate route (origin = destination, then DISCONTINUITY, ALTN airport);
 * edits the given plan in place (no TMPY), NULL/"" removes the alternate. */
int a380x_fpln_set_alternate(A380XFlightPlans *fp, int plan, const char *altn_or_null);
int a380x_fpln_delete_plan(A380XFlightPlans *fp, int plan);

/* TMPY (spec 05 §3.12). INSERT finalises pending airways, moves a pending DIR TO T-P to the
 * aircraft position/track, copies TMPY (incl. FIX INFOs) to ACTIVE and deletes TMPY. */
int a380x_fpln_tmpy_insert(A380XFlightPlans *fp, const A380XFplnAircraft *ac);
int a380x_fpln_tmpy_erase(A380XFlightPlans *fp);

/* SEC plans (07 §2.3). n = 1..3. copy_from_active copies ACTIVE (not TMPY), sets COPIED_FROM_ACTIVE.
 * activate copies SECn -> ACTIVE (SECn kept, FIX INFOs of ACTIVE preserved); swap exchanges them.
 * Both refuse with TMPY_EXISTS while TMPY exists and with NOT_ALLOWED when SECn lacks origin or
 * destination. The extra FBW conditions (HDG/TRK, phase) are the FMS's job:
 * a380x_fpln_sec_can_activate() implements canActivateOrSwapSecondary with them as inputs. */
int a380x_fpln_sec_copy_from_active(A380XFlightPlans *fp, int n);
int a380x_fpln_sec_copy(A380XFlightPlans *fp, int src_plan, int n);
int a380x_fpln_sec_delete(A380XFlightPlans *fp, int n);
int a380x_fpln_sec_activate(A380XFlightPlans *fp, int n);
int a380x_fpln_sec_swap(A380XFlightPlans *fp, int n);
int a380x_fpln_sec_can_activate(const A380XFlightPlans *fp, int n, int hdg_or_trk_engaged, int phase_preflight);

/* ---------------- departure / arrival (spec 05 §7, §8; all -> TMPY on ACTIVE) ---------------
 * id NULL or "" clears the selection; "NONE" selects NONE explicitly. Runway idents accept "27R"
 * or "RW27R". Effects (FBW): origin runway clears SID + trans; SID clears trans; dest runway
 * clears approach + via; approach sets the dest runway to its runway and clears via; STAR clears
 * its transition. Each rebuilds the procedure legs from CIFP and re-strings them with the enroute
 * legs (duplicate fix -> legs between removed; otherwise a discontinuity). */
int a380x_fpln_set_origin_runway(A380XFlightPlans *fp, int plan, int altn, const char *rwy);
int a380x_fpln_set_sid(A380XFlightPlans *fp, int plan, int altn, const char *sid);
int a380x_fpln_set_sid_trans(A380XFlightPlans *fp, int plan, int altn, const char *trans);
int a380x_fpln_set_dest_runway(A380XFlightPlans *fp, int plan, int altn, const char *rwy);
int a380x_fpln_set_approach(A380XFlightPlans *fp, int plan, int altn, const char *appr);
int a380x_fpln_set_appr_via(A380XFlightPlans *fp, int plan, int altn, const char *via);
int a380x_fpln_set_star(A380XFlightPlans *fp, int plan, int altn, const char *star);
int a380x_fpln_set_star_trans(A380XFlightPlans *fp, int plan, int altn, const char *trans);

/* ---------------- lateral revisions (spec 05 §4-§6, §10, §11) ---------------- */
/* DELETE: removeElementAt(idx). Refused (NOT_ALLOWED) for the FROM leg and for a discontinuity
 * after an FM/VM leg. Deleting a leg followed by its HM hold deletes both; deleting a disco between
 * two copies of the same fix merges them; the next XF leg becomes TF. */
int a380x_fpln_delete_element(A380XFlightPlans *fp, int plan, int altn, int idx);
/* INSERT NEXT WPT: f downstream (same ident and position) -> elements in between removed; else a
 * TF leg (IF after a discontinuity) + discontinuity are inserted after idx. */
int a380x_fpln_next_waypoint(A380XFlightPlans *fp, int plan, int altn, int after_idx, const A380XNavFix *f);
/* NEW DEST: clears arrival procedures, sets the destination (no runway), removes everything after
 * at_idx, appends DISCONTINUITY + destination airport. */
int a380x_fpln_new_dest(A380XFlightPlans *fp, int plan, int altn, int at_idx, const char *icao);
int a380x_fpln_toggle_overfly(A380XFlightPlans *fp, int plan, int altn, int idx);
/* ENABLE ALTN: primary = legs up to at_idx + DISCONTINUITY + alternate route; destination = ALTN.
 * *out_crz_fl (may be NULL) = 310 / 220 / 100 by DEST-ALTN distance (FMC:1052-1072). */
int a380x_fpln_enable_altn(A380XFlightPlans *fp, int plan, int at_idx, int *out_crz_fl);

typedef enum {
    A380X_DIRTO_DIRECT = 0,          /* T-P -> DF(target) */
    A380X_DIRTO_ABEAM,               /* + abeam points "ABxxx" of bypassed fixes */
    A380X_DIRTO_RADIAL_IN,           /* T-P -> CF(target, course) */
    A380X_DIRTO_RADIAL_OUT           /* T-P -> DF(target) -> FM(target, course) -> DISCONTINUITY */
} A380XDirToMode;
/* DIR TO a leg of `plan` (must be ACTIVE; index into ACTIVE). The result replaces any existing TMPY
 * (FBW's page erases it first) and is flagged PENDING DIR TO; ACTIVE changes only at TMPY INSERT.
 * target must be XF/HX, at/after the FROM leg and before the missed approach (else NOT_ALLOWED).
 * Result: T-P (FROM, index 0) -> DF target (active, index 1) -> rest; everything before is removed.
 * course_mag: RADIAL IN/OUT course (NAN for DIRECT/ABEAM). The DIR TO page DIST = TMPY el[1].cum_dist_nm. */
int a380x_fpln_direct_to_leg(A380XFlightPlans *fp, int plan, const A380XFplnAircraft *ac, int target_idx,
                             A380XDirToMode mode, float course_mag);
/* DIR TO a fix: if it is in the plan (active..missed) == direct_to_leg; otherwise
 * T-P -> DF(fix) -> DISCONTINUITY -> rest of the plan from the active leg. */
int a380x_fpln_direct_to_fix(A380XFlightPlans *fp, int plan, const A380XFplnAircraft *ac, const A380XNavFix *f,
                             A380XDirToMode mode, float course_mag);

/* HOLD (spec 05 §4 HOLD action and §11). dflt helper: database hold when use_database and one is
 * published at the fix, else computed: turn R, 1.0 min (<= 14000 ft) / 1.5 min, inbound = course
 * from the previous XF fix (else 100). */
int a380x_fpln_default_hold(A380XFlightPlans *fp, int plan, int altn, int idx, float alt_ft,
                            int use_database, A380XFplnHold *out);
/* Edits the HX leg at at_idx, or inserts an HM leg after at_idx. Returns the HM element index
 * (>= 0) or an error. modified may be NULL. */
int a380x_fpln_add_or_edit_hold(A380XFlightPlans *fp, int plan, int altn, int at_idx,
                                const A380XFplnHold *desired, const A380XFplnHold *modified,
                                const A380XFplnHold *dflt);
int a380x_fpln_revert_hold_to_computed(A380XFlightPlans *fp, int plan, int altn, int hm_idx);
/* IMMEDIATE EXIT / RESUME HOLD on the ACTIVE plan (no TMPY). */
int a380x_fpln_set_hold_imm_exit(A380XFlightPlans *fp, int hm_idx, int exit);

/* AIRWAYS (spec 05 §9). start creates TMPY (ACTIVE) and the pending entry at at_idx. via/to act
 * on the last line. via: NOT_IN_DB (unknown airway), NOT_ALLOWED (from fix not on it; also when the
 * previous line has an airway and no TO and no intersection exists -> NO_INTERSECTION).
 * to_ident: TO on an airway line (NOT_IN_DB if the ident is not on the airway). to_fix with dct = 1:
 * DCT line. finalise inserts the legs after at_idx (duplicate downstream fix -> legs in between
 * removed, else + DISCONTINUITY) and ends the entry. */
int a380x_fpln_airway_start(A380XFlightPlans *fp, int plan, int altn, int at_idx);
int a380x_fpln_airway_via(A380XFlightPlans *fp, int plan, const char *awy);
/* With VIA = DCT/empty this looks the ident up; more than one match returns NOT_ALLOWED: run the
 * DUPLICATE NAMES dialog (a380x_nav_find_near) and call airway_to_fix(..., dct = 1) instead. */
int a380x_fpln_airway_to_ident(A380XFlightPlans *fp, int plan, const char *ident);
int a380x_fpln_airway_to_fix(A380XFlightPlans *fp, int plan, const A380XNavFix *f, int dct);
int a380x_fpln_airway_finalise(A380XFlightPlans *fp, int plan);
void a380x_fpln_airway_cancel(A380XFlightPlans *fp, int plan);

/* ---------------- vertical revisions (spec 05 §12) ---------------- */
/* c == NULL deletes (clears database and pilot constraints). Setting updates cstr_type: a CLB
 * constraint makes every earlier UNKNOWN leg CLB, a DES constraint every later UNKNOWN leg DES. */
int a380x_fpln_set_alt_cstr(A380XFlightPlans *fp, int plan, int altn, int idx, int is_des,
                            const A380XFplnAltCstr *c);
/* kt == 0 deletes (clears database and pilot). Pilot speed constraints are '-' (at or below). */
int a380x_fpln_set_spd_cstr(A380XFlightPlans *fp, int plan, int altn, int idx, int is_des, float kt);
/* CLB/DES/UNKNOWN for the VERT REV page: leg type, else from the segment class. */
A380XFplnCstrType a380x_fpln_cstr_type_for(const A380XFplnLeg *l);
int a380x_fpln_add_cruise_step(A380XFlightPlans *fp, int plan, int idx, float alt_ft);
int a380x_fpln_remove_cruise_step(A380XFlightPlans *fp, int plan, int idx);

/* FIX INFO n = 1..4 on ACTIVE (FIX INFOs are not lateral revisions; no TMPY). fi NULL deletes. */
int a380x_fpln_set_fix_info(A380XFlightPlans *fp, int n, const A380XFplnFixInfo *fi);

/* Pilot waypoint ident "LL01" / "PBD01" / "PBX01" (prefix "LL", "PBD", "PBX"). */
void a380x_fpln_next_pilot_ident(A380XFlightPlans *fp, const char *prefix, char out[A380X_NAV_IDENT_LEN]);

/* ------------------------------------------------------------------------------------------ */
/* Sequencing and geometry                                                                     */
/* ------------------------------------------------------------------------------------------ */

enum {                               /* a380x_fpln_update() result bits */
    A380X_FPLN_EVT_SEQUENCED     = 1u << 0,  /* active leg advanced (one or more) */
    A380X_FPLN_EVT_DISCO_AHEAD   = 1u << 1,  /* TO leg is followed by a discontinuity */
    A380X_FPLN_EVT_AT_DISCO      = 1u << 2,  /* reached the end before a discontinuity (no sequencing) */
    A380X_FPLN_EVT_DEST_REACHED  = 1u << 3,  /* active leg is past the destination leg */
    A380X_FPLN_EVT_MANUAL_LEG    = 1u << 4   /* TO leg is FM/VM (never sequenced automatically) */
};
/* Sequences the ACTIVE plan (and TMPY in lockstep) from the aircraft state. No sequencing on the
 * ground. Fly-by anticipation uses a 25 deg bank turn at ground speed; overfly legs sequence when
 * passed abeam. Altitude legs (CA/FA/VA/HA) sequence at the altitude; DME/radial legs on crossing. */
uint32_t a380x_fpln_update(A380XFlightPlans *fp, const A380XFplnAircraft *ac);
/* Advances the ACTIVE plan's TO leg to idx (current < idx < count, not a discontinuity), TMPY in
 * lockstep as a380x_fpln_update does. Used by the FMGC LNAV sequencer (a380x_fmgc_guidance.c),
 * which owns the leg termination logic consistent with the path it flies. Returns 1 if sequenced. */
int a380x_fpln_sequence_to(A380XFlightPlans *fp, int idx);
/* Recomputes leg geometry/naming/indices of a plan (operations do this themselves). */
void a380x_fpln_recompute(A380XFlightPlans *fp, int plan);
/* Along-track distance from the aircraft to the destination leg of a plan's primary route, NAN
 * when unknown (no destination, no active leg, or no position). */
float a380x_fpln_dist_to_dest(const A380XFlightPlans *fp, int plan, const A380XFplnAircraft *ac);
/* Distance from the aircraft to the active leg's termination, NAN. */
float a380x_fpln_dist_to_active(const A380XFlightPlans *fp, int plan, const A380XFplnAircraft *ac);

typedef enum { A380X_PATH_NONE = 0, A380X_PATH_LINE, A380X_PATH_ARC, A380X_PATH_HOLD } A380XFplnPathKind;
typedef struct {                     /* drawable leg geometry for the ND */
    A380XFplnPathKind kind;
    double start_lat, start_lon, end_lat, end_lon;
    double center_lat, center_lon;   /* ARC / HOLD fix */
    float  radius_nm;                /* ARC */
    int    clockwise;                /* ARC/HOLD turn direction (1 = right) */
    float  inbound_true;             /* HOLD inbound course (true) */
    float  hold_leg_nm;              /* HOLD straight leg length */
    int    is_missed, is_altn, is_tmpy;
} A380XFplnLegPath;
/* Returns 1 when element idx of the route has a drawable path. */
int a380x_fpln_leg_path(const A380XFlightPlans *fp, int plan, int altn, int idx, A380XFplnLegPath *out);

/* ------------------------------------------------------------------------------------------ */
/* F-PLN page line list (spec 05 §3.3, with the [PORT] fixes)                                  */
/* ------------------------------------------------------------------------------------------ */

typedef struct {                     /* pseudo waypoint from VNAV (spec 05 §2.3 mfd_pwp_t) */
    char  ident[10];
    int   along_leg_index;           /* joint index (primary count + alt index for ALTN) */
    float dist_from_start_nm;        /* same reference as cum_dist_with_trans_nm */
    A380XFplnWptPred pred;
} A380XFplnPwp;
typedef struct {                     /* optional prediction feed; NULL = phase 1 (all dashes) */
    int pwp_count;
    A380XFplnPwp pwp[A380X_FPLN_MAX_PWP];
    const A380XFplnWptPred *leg_pred; /* indexed by joint index, NULL = none */
    int leg_pred_count;
    float hold_speed_kt;             /* NAN */
    int   hold_leg_index;            /* -1 */
    int   hold_decel_reached;
} A380XFplnPredFeed;

typedef enum { A380X_FLINE_WAYPOINT = 0, A380X_FLINE_HOLD_HM, A380X_FLINE_SPECIAL } A380XFplnLineKind;
typedef enum {
    A380X_FSPEC_NONE = 0, A380X_FSPEC_DISCONTINUITY, A380X_FSPEC_END_OF_FPLN, A380X_FSPEC_NO_ALTN_FPLN,
    A380X_FSPEC_END_OF_ALTN_FPLN, A380X_FSPEC_TOO_STEEP_PATH
} A380XFplnSpecial;

typedef struct {
    A380XFplnLineKind kind;
    A380XFplnSpecial  special;        /* SPECIAL lines */
    const char *special_text;         /* "DISCONTINUITY", ... (static string) */
    int   leg_index;                  /* index in its own route (primary or ALTN); -1 = not clickable */
    int   joint_index;                /* index in primary+ALTN joint list, -1 */
    int   is_altn, is_missed, is_pseudo;
    int   is_active_leg;              /* [PORT] own leg index == active (primary) */
    int   is_before_active;           /* own leg index == active - 1 (the FROM line) */
    int   is_active_hold_fix;         /* HM leg that is the active leg */
    int   use_trans_level;            /* leg index >= last_enroute_index: format with TL, else TA */
    char  ident[A380X_FPLN_IDENT_LEN];
    char  annotation[A380X_FPLN_ANNOT_LEN];
    int   overfly;
    float trk_true_deg, trk_mag_deg;  /* NAN = blank */
    float dist_nm;                    /* NAN = blank */
    float fpa_deg;                    /* NAN = blank */
    A380XFplnAltCstr alt_cstr;        /* effective constraints */
    A380XFplnSpdCstr spd_cstr;
    A380XFplnWptPred pred;            /* valid = 0 without a feed */
    /* HOLD_HM lines */
    A380XFplnTurn hold_turn;
    float hold_speed_kt;              /* NAN */
    int   hold_imm_exit, hold_decel_sequenced;
    const A380XFplnLeg *leg;          /* the leg, NULL for PWP/SPECIAL-without-leg */
} A380XFplnLine;

typedef struct {                      /* iterator; ~3 KB, keep on the stack */
    const A380XFlightPlans *fp;
    const A380XFplnPlan *plan;
    const A380XFplnPredFeed *feed;
    int  plan_index;
    int  i;                           /* next joint index to expand */
    int  n_pri, n_alt;
    int  empty_stage;                 /* empty-plan sequence */
    int  line_no;                     /* index of the next line returned */
    double last;                      /* cumulative distance reference */
    double last_lat, last_lon;
    int  have_last_pos;
    double last_d;                    /* lastD of the PWP split */
    int  q_head, q_count;
    A380XFplnLine q[A380X_FPLN_MAX_PWP + 6];
} A380XFplnLineIter;

void a380x_fpln_lines_begin(A380XFplnLineIter *it, const A380XFlightPlans *fp, int plan_index,
                            const A380XFplnPredFeed *feed);
/* Returns 1 and the next line, or 0 at the end. */
int  a380x_fpln_lines_next(A380XFplnLineIter *it, A380XFplnLine *out);
int  a380x_fpln_line_count(const A380XFlightPlans *fp, int plan_index, const A380XFplnPredFeed *feed);
/* Line index of the WAYPOINT line of a leg (primary when altn = 0), -1 if not found. */
int  a380x_fpln_line_of_leg(const A380XFlightPlans *fp, int plan_index, const A380XFplnPredFeed *feed,
                            int leg_index, int altn);

#ifdef __cplusplus
}
#endif

#endif
