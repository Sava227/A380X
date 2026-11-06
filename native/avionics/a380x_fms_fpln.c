/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * A380X flight-plan model and operations. See a380x_fms_fpln.h and docs/mfd-spec/05-fms-fpln.md §2.
 * No heap use after a380x_fpln_create. Every edit runs on a scratch copy and is committed only on
 * success (transactional, so a refused edit never leaves a TMPY behind).
 */
#include "a380x_fms_fpln.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEG_SIGN "\xc2\xb0"
#define NM_PER_FT (1.0 / 6076.115)
#define EST_FT_PER_NM 500.0      /* nominal climb gradient for altitude-terminated legs */

/* ============================================================================================ */
/* Small helpers                                                                                */
/* ============================================================================================ */

static void scopy(char *dst, size_t cap, const char *src)
{
    if (!cap) return;
    if (!src) src = "";
    size_t n = strlen(src);
    if (n >= cap) n = cap - 1;
    memcpy(dst, src, n);
    dst[n] = 0;
}
static int streq(const char *a, const char *b) { return !strcmp(a ? a : "", b ? b : ""); }
static int finite2(double a, double b) { return isfinite(a) && isfinite(b); }

static void leg_init(A380XFplnLeg *l)
{
    memset(l, 0, sizeof *l);
    a380x_nav_ref_clear(&l->fix);
    a380x_nav_ref_clear(&l->navaid);
    a380x_nav_ref_clear(&l->arc_center);
    l->course_deg = l->theta_deg = l->rho_nm = l->arc_radius_nm = l->rnp_nm = NAN;
    l->length_nm = l->length_min = l->vertical_angle_deg = NAN;
    l->alt_db.alt1_ft = l->alt_db.alt2_ft = NAN;
    l->alt_pilot.alt1_ft = l->alt_pilot.alt2_ft = NAN;
    l->trans_alt_ft = NAN;
    l->hold_default.dist_nm = l->hold_default.time_min = NAN;
    l->hold_modified.dist_nm = l->hold_modified.time_min = NAN;
    l->hold_default.inbound_crs_mag = l->hold_modified.inbound_crs_mag = NAN;
    l->term_lat = l->term_lon = l->start_lat = l->start_lon = NAN;
    l->leg_dist_nm = l->path_true_crs = NAN;
    l->cum_dist_nm = l->cum_dist_with_trans_nm = NAN;
    l->pred.altitude_ft = l->pred.efob_kg = NAN;
}
static void disco_init(A380XFplnLeg *l, A380XFplnSegment seg, int automatic)
{
    leg_init(l);
    l->flags = A380X_LEGF_DISCONTINUITY | (automatic ? A380X_LEGF_AUTO_DISCO : 0);
    l->segment = seg;
}
static void route_init(A380XFplnRoute *r)
{
    memset(r, 0, sizeof *r);
    r->first_missed_appr_index = 0;
    r->last_enroute_index = -1;
    r->destination_leg_index = -1;
    r->db_trans_alt_ft = r->db_trans_level_ft = NAN;
    r->origin_elev_ft = r->dest_elev_ft = NAN;
}
static void plan_init(A380XFplnPlan *p)
{
    uint32_t v = p->version;
    memset(p, 0, sizeof *p);
    p->version = v;
    p->active_leg_index = -1;
    route_init(&p->pri);
    route_init(&p->alt);
}

int a380x_fpln_is_disco(const A380XFplnLeg *l) { return l && (l->flags & A380X_LEGF_DISCONTINUITY); }
static int is_leg(const A380XFplnLeg *l) { return l && !(l->flags & A380X_LEGF_DISCONTINUITY); }
static int has_fix(const A380XFplnLeg *l) { return is_leg(l) && l->fix.ident[0] && finite2(l->fix.lat, l->fix.lon); }
/* the fix is the termination of the leg (not the start of an F-type leg) */
static int ends_at_fix(const A380XFplnLeg *l)
{
    return has_fix(l) && !a380x_leg_starts_at_fix(l->type);
}
static int same_fix(const A380XNavRef *a, const A380XNavRef *b)
{
    if (!a->ident[0] || strcmp(a->ident, b->ident)) return 0;
    if (!finite2(a->lat, a->lon) || !finite2(b->lat, b->lon)) return 1;
    return a380x_geo_distance_nm(a->lat, a->lon, b->lat, b->lon) < 1.0;
}

A380XFplnSegmentClass a380x_fpln_segment_class(A380XFplnSegment s)
{
    if (s <= A380X_SEG_SID_TRANS) return A380X_SEGCLASS_DEPARTURE;
    if (s == A380X_SEG_ENROUTE) return A380X_SEGCLASS_ENROUTE;
    return A380X_SEGCLASS_ARRIVAL;
}
int a380x_fpln_is_db_fix(const A380XFplnLeg *l)
{
    return is_leg(l) && a380x_leg_is_xf(l->type) && !(l->flags & A380X_LEGF_DIRTO_TP);
}
int a380x_fpln_constraint_revision_allowed(const A380XFplnLeg *l)
{
    return a380x_fpln_is_db_fix(l) && l->wp_desc != A380X_WPDESC_RUNWAY;
}
A380XFplnAltCstr a380x_fpln_eff_alt(const A380XFplnLeg *l)
{
    A380XFplnAltCstr none = { 0, NAN, NAN };
    if (!is_leg(l)) return none;
    if (l->alt_pilot.desc) return l->alt_pilot;
    if (l->alt_db.desc && l->alt_db.desc != 'G' && l->alt_db.desc != 'H') return l->alt_db;
    return none;
}
A380XFplnSpdCstr a380x_fpln_eff_spd(const A380XFplnLeg *l)
{
    A380XFplnSpdCstr none = { 0, NAN };
    if (!is_leg(l)) return none;
    if (l->spd_pilot.desc) return l->spd_pilot;
    if (l->spd_db.desc) return l->spd_db;
    return none;
}
A380XFplnCstrType a380x_fpln_cstr_type_for(const A380XFplnLeg *l)
{
    if (!is_leg(l)) return A380X_CSTR_UNKNOWN;
    if (l->cstr_type != A380X_CSTR_UNKNOWN) return l->cstr_type;
    switch (a380x_fpln_segment_class(l->segment)) {
    case A380X_SEGCLASS_DEPARTURE: return A380X_CSTR_CLB;
    case A380X_SEGCLASS_ARRIVAL: return A380X_CSTR_DES;
    default: return A380X_CSTR_UNKNOWN;
    }
}

/* ============================================================================================ */
/* Store                                                                                        */
/* ============================================================================================ */

void a380x_fpln_init(A380XFlightPlans *fp, A380XNavdata *nav)
{
    memset(fp, 0, sizeof *fp);
    fp->nav = nav;
    for (int i = 0; i < A380X_PLAN_COUNT; i++) plan_init(&fp->plan[i]);
}
A380XFlightPlans *a380x_fpln_create(A380XNavdata *nav)
{
    A380XFlightPlans *fp = (A380XFlightPlans *)malloc(sizeof *fp);
    if (fp) a380x_fpln_init(fp, nav);
    return fp;
}
void a380x_fpln_destroy(A380XFlightPlans *fp) { free(fp); }
void a380x_fpln_reset(A380XFlightPlans *fp)
{
    for (int i = 0; i < A380X_PLAN_COUNT; i++) {
        uint32_t v = fp->plan[i].version + 1;
        plan_init(&fp->plan[i]);
        fp->plan[i].version = v;
    }
    fp->pilot_wpt_seq = 0;
}

static int valid_plan(int p) { return p >= 0 && p < A380X_PLAN_COUNT && p != A380X_PLAN_UPLINK; }
const A380XFplnPlan *a380x_fpln_plan(const A380XFlightPlans *fp, int i)
{
    if (!fp || i < 0 || i >= A380X_PLAN_COUNT || !fp->plan[i].exists) return NULL;
    return &fp->plan[i];
}
const A380XFplnRoute *a380x_fpln_route(const A380XFlightPlans *fp, int i, int altn)
{
    const A380XFplnPlan *p = a380x_fpln_plan(fp, i);
    return p ? (altn ? &p->alt : &p->pri) : NULL;
}
int a380x_fpln_has_tmpy(const A380XFlightPlans *fp) { return fp && fp->plan[A380X_PLAN_TEMPORARY].exists; }
int a380x_fpln_loaded_index(const A380XFlightPlans *fp, int sec)
{
    if (sec >= 1 && sec <= A380X_FPLN_SEC_COUNT) return A380X_PLAN_SEC1 + sec - 1;
    return a380x_fpln_has_tmpy(fp) ? A380X_PLAN_TEMPORARY : A380X_PLAN_ACTIVE;
}
int a380x_fpln_from_leg_index(const A380XFplnPlan *p)
{
    if (!p) return -1;
    for (int i = p->active_leg_index - 1; i >= 0 && i < p->pri.count; i--)
        if (is_leg(&p->pri.el[i])) return i;
    return -1;
}

void a380x_fpln_next_pilot_ident(A380XFlightPlans *fp, const char *prefix, char out[A380X_NAV_IDENT_LEN])
{
    fp->pilot_wpt_seq = fp->pilot_wpt_seq % 99 + 1;
    snprintf(out, A380X_NAV_IDENT_LEN, "%.4s%02d", prefix ? prefix : "WP", fp->pilot_wpt_seq);
}

/* ---------------- transactions ---------------- */

/* Copies the plan an edit applies to into fp->scratch. ACTIVE edits go to TMPY (existing TMPY, or a
 * new copy of ACTIVE). */
static A380XFplnPlan *begin_edit(A380XFlightPlans *fp, int plan, int *err)
{
    if (!fp || !valid_plan(plan)) { *err = A380X_FPLN_E_ARG; return NULL; }
    const A380XFplnPlan *src;
    if (plan == A380X_PLAN_ACTIVE && fp->plan[A380X_PLAN_TEMPORARY].exists) src = &fp->plan[A380X_PLAN_TEMPORARY];
    else src = &fp->plan[plan];
    if (!src->exists) { *err = A380X_FPLN_E_NO_PLAN; return NULL; }
    memcpy(&fp->scratch, src, sizeof fp->scratch);
    if (plan == A380X_PLAN_ACTIVE && !fp->plan[A380X_PLAN_TEMPORARY].exists) {
        fp->scratch.flags = (fp->scratch.flags & ~(uint32_t)A380X_PLANF_DIRTO_PENDING) | A380X_PLANF_COPIED_FROM_ACTIVE;
        if (fp->scratch.version < fp->plan[A380X_PLAN_TEMPORARY].version) fp->scratch.version = fp->plan[A380X_PLAN_TEMPORARY].version;
    }
    *err = 0;
    return &fp->scratch;
}
static int target_of(const A380XFlightPlans *fp, int plan)
{
    (void)fp;
    return plan == A380X_PLAN_ACTIVE ? A380X_PLAN_TEMPORARY : plan;
}
static void commit_edit(A380XFlightPlans *fp, int plan)
{
    int t = target_of(fp, plan);
    fp->scratch.exists = 1;
    if (fp->scratch.version < fp->plan[t].version) fp->scratch.version = fp->plan[t].version;
    fp->scratch.version++;
    memcpy(&fp->plan[t], &fp->scratch, sizeof fp->scratch);
    a380x_fpln_recompute(fp, t);
}
static A380XFplnRoute *route_of(A380XFplnPlan *p, int altn) { return altn ? &p->alt : &p->pri; }

/* ---------------- element insert / remove with active-leg bookkeeping ---------------- */

static int el_insert(A380XFplnPlan *p, int altn, int idx, const A380XFplnLeg *leg)
{
    A380XFplnRoute *r = route_of(p, altn);
    if (r->count >= A380X_FPLN_MAX_ELEMENTS) return A380X_FPLN_E_FULL;
    if (idx < 0) idx = 0;
    if (idx > r->count) idx = r->count;
    memmove(&r->el[idx + 1], &r->el[idx], (size_t)(r->count - idx) * sizeof r->el[0]);
    r->el[idx] = *leg;
    r->count++;
    if (!altn && p->active_leg_index >= 0 && idx <= p->active_leg_index && p->active_leg_index > 0) p->active_leg_index++;
    return 0;
}
static void el_remove(A380XFplnPlan *p, int altn, int idx, int n)
{
    A380XFplnRoute *r = route_of(p, altn);
    if (idx < 0 || n <= 0 || idx >= r->count) return;
    if (idx + n > r->count) n = r->count - idx;
    memmove(&r->el[idx], &r->el[idx + n], (size_t)(r->count - idx - n) * sizeof r->el[0]);
    r->count -= n;
    if (!altn && p->active_leg_index >= 0) {
        if (p->active_leg_index >= idx + n) p->active_leg_index -= n;
        else if (p->active_leg_index >= idx) p->active_leg_index = idx;
        if (p->active_leg_index >= r->count) p->active_leg_index = r->count - 1;
    }
}
/* removes consecutive discontinuities */
static void normalize(A380XFplnPlan *p, int altn)
{
    A380XFplnRoute *r = route_of(p, altn);
    for (int i = 1; i < r->count;) {
        if (a380x_fpln_is_disco(&r->el[i]) && a380x_fpln_is_disco(&r->el[i - 1])) {
            /* keep a pilot disco over an automatic one */
            if (r->el[i - 1].flags & A380X_LEGF_AUTO_DISCO) el_remove(p, altn, i - 1, 1);
            else el_remove(p, altn, i, 1);
        } else i++;
    }
    if (r->count && a380x_fpln_is_disco(&r->el[0]) && r->count > 1) el_remove(p, altn, 0, 1);
}

/* ============================================================================================ */
/* Leg construction                                                                             */
/* ============================================================================================ */

static A380XFplnTurn turn_of(char c) { return c == 'L' ? A380X_TURN_LEFT : (c == 'R' ? A380X_TURN_RIGHT : A380X_TURN_EITHER); }

static A380XFplnWpDesc wpdesc_of(const A380XNavRef *f)
{
    if (!f->ident[0]) return A380X_WPDESC_NONE;
    if (f->kind == A380X_NAV_RUNWAY) return A380X_WPDESC_RUNWAY;
    if (f->kind == A380X_NAV_AIRPORT) return A380X_WPDESC_AIRPORT;
    return A380X_WPDESC_OTHER;
}

static void leg_from_proc(A380XFplnLeg *l, const A380XNavProcLeg *pl, A380XFplnSegment seg, const char *proc, float var)
{
    leg_init(l);
    l->type = pl->type;
    l->segment = seg;
    l->fix = pl->fix;
    l->navaid = pl->navaid;
    l->arc_center = pl->center;
    memcpy(l->desc, pl->desc, sizeof l->desc);
    scopy(l->procedure_ident, sizeof l->procedure_ident, proc);
    l->course_deg = pl->course_deg;
    l->course_true = pl->course_true;
    l->theta_deg = pl->theta_deg;
    l->rho_nm = pl->rho_nm;
    l->arc_radius_nm = pl->arc_radius_nm;
    l->rnp_nm = pl->rnp_nm;
    l->turn = turn_of(pl->turn);
    l->length_nm = pl->dist_nm;
    l->length_min = pl->time_min;
    l->vertical_angle_deg = pl->vertical_angle_deg;
    l->magvar_deg = var;
    l->alt_db.desc = pl->alt_desc;
    l->alt_db.alt1_ft = pl->alt1_ft;
    l->alt_db.alt2_ft = pl->alt2_ft;
    l->spd_db.desc = pl->spd_desc;
    l->spd_db.speed = isfinite(pl->speed_kt) ? pl->speed_kt : NAN;
    if (!l->spd_db.desc) l->spd_db.speed = NAN;
    l->trans_alt_ft = pl->trans_alt_ft;
    if (pl->overfly) l->flags |= A380X_LEGF_OVERFLY;
    if (pl->missed_start) l->flags |= A380X_LEGF_MISSED_START;
    if (pl->fix_unresolved) l->flags |= A380X_LEGF_FIX_UNRESOLVED;
    l->wp_desc = wpdesc_of(&l->fix);
    if (a380x_leg_is_hx(l->type)) {
        l->hold_default.valid = 1;
        l->hold_default.type = A380X_HOLD_DATABASE;
        l->hold_default.inbound_crs_mag = pl->course_deg;
        l->hold_default.dist_nm = pl->dist_nm;
        l->hold_default.time_min = pl->time_min;
        if (!isfinite(pl->dist_nm) && !isfinite(pl->time_min)) l->hold_default.time_min = 1.0f;
        l->hold_default.turn = l->turn == A380X_TURN_LEFT ? A380X_TURN_LEFT : A380X_TURN_RIGHT;
    }
    switch (a380x_fpln_segment_class(seg)) {
    case A380X_SEGCLASS_DEPARTURE: l->cstr_type = A380X_CSTR_CLB; break;
    case A380X_SEGCLASS_ARRIVAL: l->cstr_type = A380X_CSTR_DES; break;
    default: break;
    }
}

static void leg_from_fix(A380XFplnLeg *l, const A380XNavFix *f, A380XLegType type, A380XFplnSegment seg)
{
    leg_init(l);
    l->type = type;
    l->segment = seg;
    a380x_nav_ref_from_fix(&l->fix, f);
    l->wp_desc = wpdesc_of(&l->fix);
}
static void leg_from_ref(A380XFplnLeg *l, const A380XNavRef *f, A380XLegType type, A380XFplnSegment seg)
{
    leg_init(l);
    l->type = type;
    l->segment = seg;
    l->fix = *f;
    l->wp_desc = wpdesc_of(&l->fix);
}

/* origin / destination airport or runway leg */
static int airport_leg(A380XFlightPlans *fp, A380XFplnLeg *l, const char *icao, const char *rwy, A380XFplnSegment seg,
                       const char *proc)
{
    leg_init(l);
    l->type = A380X_LEG_IF;
    l->segment = seg;
    scopy(l->procedure_ident, sizeof l->procedure_ident, proc);
    if (rwy && rwy[0]) {
        const A380XNavAirport *apt = a380x_nav_airport(fp->nav, icao);
        const A380XNavRunway *rw = a380x_nav_runway(apt, rwy);
        if (rw && isfinite(rw->thr_lat)) {
            scopy(l->fix.ident, sizeof l->fix.ident, rw->ident);
            scopy(l->fix.region, sizeof l->fix.region, apt->info.region);
            l->fix.kind = A380X_NAV_RUNWAY;
            l->fix.lat = rw->thr_lat;
            l->fix.lon = rw->thr_lon;
            l->wp_desc = A380X_WPDESC_RUNWAY;
            l->magvar_deg = apt->info.magvar_deg;
            l->course_deg = rw->mag_brg;
            return 1;
        }
    }
    A380XNavAirportInfo ai;
    if (!a380x_nav_airport_info(fp->nav, icao, &ai)) return 0;
    scopy(l->fix.ident, sizeof l->fix.ident, icao);
    scopy(l->fix.region, sizeof l->fix.region, ai.region);
    l->fix.kind = A380X_NAV_AIRPORT;
    l->fix.lat = ai.lat;
    l->fix.lon = ai.lon;
    l->wp_desc = A380X_WPDESC_AIRPORT;
    l->magvar_deg = ai.magvar_deg;
    return 1;
}

/* ---------------- procedure building ---------------- */

typedef struct { A380XFplnLeg *el; int n, cap; } LegBuf;

static int buf_push(LegBuf *b, const A380XFplnLeg *l)
{
    if (b->n >= b->cap) return 0;
    b->el[b->n++] = *l;
    return 1;
}
static A380XFplnLeg *buf_last_leg(LegBuf *b) { return b->n ? &b->el[b->n - 1] : NULL; }

/* Appends a procedure leg, merging it into the previous leg when both describe the same fix
 * (procedure part junctions: runway transition -> common route -> enroute transition, STAR -> VIA). */
static int buf_append_merge(LegBuf *b, const A380XFplnLeg *l)
{
    A380XFplnLeg *last = buf_last_leg(b);
    if (last && is_leg(last) && ends_at_fix(last) && has_fix(l) && !a380x_leg_starts_at_fix(l->type)
        && !a380x_leg_is_hx(l->type) && same_fix(&last->fix, &l->fix)
        && (l->type == A380X_LEG_IF || l->type == A380X_LEG_TF || l->type == A380X_LEG_CF || l->type == A380X_LEG_DF)) {
        if (l->alt_db.desc) last->alt_db = l->alt_db;
        if (l->spd_db.desc) last->spd_db = l->spd_db;
        if (l->flags & A380X_LEGF_OVERFLY) last->flags |= A380X_LEGF_OVERFLY;
        return 1;
    }
    return buf_push(b, l);
}

static void append_part(LegBuf *b, const A380XNavAirport *apt, const A380XNavProcTrans *t, A380XFplnSegment seg,
                        const char *proc, int missed_split, LegBuf *missed)
{
    int in_missed = 0;
    for (int i = 0; i < t->leg_count; i++) {
        const A380XNavProcLeg *pl = &apt->legs[t->first_leg + i];
        if (missed_split && pl->missed_start) in_missed = 1;
        A380XFplnLeg l;
        leg_from_proc(&l, pl, in_missed ? A380X_SEG_MISSED : seg, proc, apt->info.magvar_deg);
        if (in_missed) buf_push(missed, &l);
        else buf_append_merge(b, &l);
    }
}

static const A380XNavProcTrans *find_trans(const A380XNavAirport *apt, const A380XNavProcedure *p,
                                           A380XProcTransRole role, const char *ident)
{
    for (int i = 0; i < p->trans_count; i++) {
        const A380XNavProcTrans *t = &apt->trans[p->first_trans + i];
        if (t->role == role && streq(t->ident, ident)) return t;
    }
    return NULL;
}
static int rwy_ident(const char *s) { return s && s[0] == 'R' && s[1] == 'W' && s[2] >= '0' && s[2] <= '9'; }

/* Departure block (ORIGIN..SID_TRANS) of route r into b. */
static int build_departure(A380XFlightPlans *fp, const A380XFplnRoute *r, LegBuf *b)
{
    b->n = 0;
    if (!r->origin_icao[0]) return 1;
    A380XFplnLeg l;
    const char *sid_disp = (r->sid[0] && !r->sid_none) ? r->sid : "";
    if (!airport_leg(fp, &l, r->origin_icao, r->origin_rwy, A380X_SEG_ORIGIN, sid_disp)) return 0;
    l.flags |= A380X_LEGF_ORIGIN;
    buf_push(b, &l);
    if (!sid_disp[0]) return 1;
    const A380XNavAirport *apt = a380x_nav_airport(fp->nav, r->origin_icao);
    const A380XNavProcedure *p = a380x_nav_procedure(apt, A380X_PROC_SID, r->sid);
    if (!p) return 1;
    /* runway transition */
    for (int i = 0; i < p->trans_count; i++) {
        const A380XNavProcTrans *t = &apt->trans[p->first_trans + i];
        if (t->role == A380X_TRANS_RUNWAY && a380x_nav_trans_matches_runway(t->ident, r->origin_rwy)) {
            append_part(b, apt, t, A380X_SEG_DEP_RWY_TRANS, p->ident, 0, NULL);
            break;
        }
    }
    /* common route(s): runway specific first match, else generic */
    const A380XNavProcTrans *common = NULL;
    for (int i = 0; i < p->trans_count && !common; i++) {
        const A380XNavProcTrans *t = &apt->trans[p->first_trans + i];
        if (t->role == A380X_TRANS_COMMON && rwy_ident(t->ident) && a380x_nav_trans_matches_runway(t->ident, r->origin_rwy)) common = t;
    }
    for (int i = 0; i < p->trans_count && !common; i++) {
        const A380XNavProcTrans *t = &apt->trans[p->first_trans + i];
        if (t->role == A380X_TRANS_COMMON && !rwy_ident(t->ident)) common = t;
    }
    if (common) append_part(b, apt, common, A380X_SEG_SID, p->ident, 0, NULL);
    if (r->sid_trans[0]) {
        const A380XNavProcTrans *t = find_trans(apt, p, A380X_TRANS_ENROUTE, r->sid_trans);
        if (t) append_part(b, apt, t, A380X_SEG_SID_TRANS, p->ident, 0, NULL);
    }
    /* a first SID leg that re-states the runway (IF/TF at the threshold) duplicates the origin leg */
    if (b->n > 1 && b->el[0].wp_desc == A380X_WPDESC_RUNWAY && has_fix(&b->el[1]) && !a380x_leg_starts_at_fix(b->el[1].type)
        && (b->el[1].type == A380X_LEG_IF || b->el[1].type == A380X_LEG_TF) && same_fix(&b->el[0].fix, &b->el[1].fix)) {
        memmove(&b->el[1], &b->el[2], (size_t)(b->n - 2) * sizeof b->el[0]);
        b->n--;
    }
    return 1;
}

/* Arrival block (STAR_TRANS..MISSED) of route r into b. */
static int build_arrival(A380XFlightPlans *fp, const A380XFplnRoute *r, LegBuf *b, A380XFplnLeg *missed_store, int missed_cap)
{
    b->n = 0;
    if (!r->dest_icao[0]) return 1;
    const A380XNavAirport *apt = a380x_nav_airport(fp->nav, r->dest_icao);
    const A380XNavProcedure *star = (r->star[0] && !r->star_none) ? a380x_nav_procedure(apt, A380X_PROC_STAR, r->star) : NULL;
    const A380XNavProcedure *appr = (r->approach[0] && !r->appr_none) ? a380x_nav_procedure(apt, A380X_PROC_APPROACH, r->approach) : NULL;
    if (star) {
        if (r->star_trans[0]) {
            const A380XNavProcTrans *t = find_trans(apt, star, A380X_TRANS_ENROUTE, r->star_trans);
            if (t) append_part(b, apt, t, A380X_SEG_STAR_TRANS, star->ident, 0, NULL);
        }
        const A380XNavProcTrans *common = NULL;
        for (int i = 0; i < star->trans_count && !common; i++) {
            const A380XNavProcTrans *t = &apt->trans[star->first_trans + i];
            if (t->role == A380X_TRANS_COMMON && (!rwy_ident(t->ident) || a380x_nav_trans_matches_runway(t->ident, r->dest_rwy))) common = t;
        }
        if (common) append_part(b, apt, common, A380X_SEG_STAR, star->ident, 0, NULL);
        const char *rwy = appr && appr->runway[0] ? appr->runway : r->dest_rwy;
        for (int i = 0; i < star->trans_count; i++) {
            const A380XNavProcTrans *t = &apt->trans[star->first_trans + i];
            if (t->role == A380X_TRANS_RUNWAY && rwy[0] && a380x_nav_trans_matches_runway(t->ident, rwy)) {
                append_part(b, apt, t, A380X_SEG_STAR_RWY_TRANS, star->ident, 0, NULL);
                break;
            }
        }
    }
    if (appr) {
        char name[24];
        a380x_nav_approach_name(appr, 0, name, sizeof name);
        int star_end = b->n;
        LegBuf ab = { b->el + b->n, 0, b->cap - b->n };
        LegBuf mb = { missed_store, 0, missed_cap };
        if (r->appr_via[0]) {
            const A380XNavProcTrans *t = find_trans(apt, appr, A380X_TRANS_APPR_VIA, r->appr_via);
            if (t) append_part(&ab, apt, t, A380X_SEG_APPR_VIA, name, 0, NULL);
        }
        for (int i = 0; i < appr->trans_count; i++) {
            const A380XNavProcTrans *t = &apt->trans[appr->first_trans + i];
            if (t->role == A380X_TRANS_APPR_FINAL) { append_part(&ab, apt, t, A380X_SEG_APPROACH, name, 1, &mb); break; }
        }
        /* STAR -> approach junction: same fix = merge, F-type leg from the STAR end = connected,
         * otherwise a discontinuity */
        if (star_end > 0 && ab.n > 0) {
            A380XFplnLeg *last = &b->el[star_end - 1];
            A380XFplnLeg *first = &ab.el[0];
            if (ends_at_fix(last) && has_fix(first) && same_fix(&last->fix, &first->fix)) {
                if (!a380x_leg_starts_at_fix(first->type) && !a380x_leg_is_hx(first->type)) {
                    if (first->alt_db.desc) last->alt_db = first->alt_db;
                    if (first->spd_db.desc) last->spd_db = first->spd_db;
                    memmove(&ab.el[0], &ab.el[1], (size_t)(ab.n - 1) * sizeof ab.el[0]);
                    ab.n--;
                }
            } else {
                if (b->n + ab.n + 1 > b->cap) return 0;
                memmove(&ab.el[1], &ab.el[0], (size_t)ab.n * sizeof ab.el[0]);
                disco_init(&ab.el[0], A380X_SEG_APPR_VIA, 1);
                ab.n++;
            }
        }
        b->n += ab.n;
        for (int i = 0; i < mb.n; i++) buf_push(b, &mb.el[i]);
        /* the approach runway / MAP leg is the destination */
        return 1;
    }
    A380XFplnLeg l;
    if (!airport_leg(fp, &l, r->dest_icao, r->dest_rwy, A380X_SEG_DESTINATION, "")) return 0;
    l.flags |= A380X_LEGF_DESTINATION;
    l.cstr_type = A380X_CSTR_DES;
    if (b->n > 0 && ends_at_fix(&b->el[b->n - 1]) && same_fix(&b->el[b->n - 1].fix, &l.fix)) {
        b->el[b->n - 1].flags |= A380X_LEGF_DESTINATION;
        return 1;
    }
    if (l.type == A380X_LEG_IF && b->n > 0) l.type = A380X_LEG_TF;
    buf_push(b, &l);
    return 1;
}

/* Rebuilds the departure and/or arrival block of a route and re-strings it with the enroute legs. */
static int rebuild(A380XFlightPlans *fp, A380XFplnPlan *p, int altn, int do_dep, int do_arr)
{
    A380XFplnRoute *r = route_of(p, altn);
    A380XFplnLeg *dep = fp->work[0].el, *enr = fp->work[1].el, *arr = fp->work[2].el;
    A380XFplnLeg *missed_tmp = fp->work[3].el;
    int nd = 0, ne = 0, na = 0;
    /* remember the active leg for re-mapping */
    int old_active = altn ? -1 : p->active_leg_index;
    A380XFplnLeg old_act;
    int have_old = old_active > 1 && old_active < r->count;
    if (have_old) old_act = r->el[old_active];
    for (int i = 0; i < r->count; i++) {
        const A380XFplnLeg *e = &r->el[i];
        if (e->segment < A380X_SEG_ENROUTE) dep[nd++] = *e;
        else if (e->segment == A380X_SEG_ENROUTE) enr[ne++] = *e;
        else arr[na++] = *e;
    }
    if (do_dep) {
        LegBuf b = { dep, 0, A380X_FPLN_MAX_ELEMENTS };
        if (!build_departure(fp, r, &b)) return A380X_FPLN_E_NOT_IN_DB;
        nd = b.n;
        if (ne && (enr[0].flags & A380X_LEGF_AUTO_DISCO)) { memmove(enr, enr + 1, (size_t)(ne - 1) * sizeof *enr); ne--; }
    }
    if (do_arr) {
        LegBuf b = { arr, 0, A380X_FPLN_MAX_ELEMENTS };
        if (!build_arrival(fp, r, &b, missed_tmp, A380X_FPLN_MAX_ELEMENTS)) return A380X_FPLN_E_NOT_IN_DB;
        na = b.n;
        if (ne && (enr[ne - 1].flags & A380X_LEGF_AUTO_DISCO)) ne--;
    }
    if (do_dep && nd > 1) {   /* departure end vs enroute */
        const A380XFplnLeg *last = &dep[nd - 1];
        int j = -1;
        if (ends_at_fix(last))
            for (int k = 0; k < ne; k++) if (is_leg(&enr[k]) && same_fix(&enr[k].fix, &last->fix)) { j = k; break; }
        if (j >= 0) {
            dep[nd - 1].alt_pilot = enr[j].alt_pilot.desc ? enr[j].alt_pilot : dep[nd - 1].alt_pilot;
            memmove(enr, enr + j + 1, (size_t)(ne - j - 1) * sizeof *enr);
            ne -= j + 1;
        } else if (ne && !a380x_fpln_is_disco(&enr[0])) {
            memmove(enr + 1, enr, (size_t)ne * sizeof *enr);
            disco_init(&enr[0], A380X_SEG_ENROUTE, 1);
            ne++;
        }
    }
    if (do_arr && na > 0) {   /* enroute vs arrival start */
        const A380XFplnLeg *first = &arr[0];
        int j = -1;
        if (has_fix(first) && !a380x_leg_starts_at_fix(first->type))
            for (int k = 0; k < ne; k++) if (is_leg(&enr[k]) && same_fix(&enr[k].fix, &first->fix)) { j = k; break; }
        if (j >= 0) ne = j;
        else if (ne && !a380x_fpln_is_disco(&enr[ne - 1])) disco_init(&enr[ne++], A380X_SEG_ENROUTE, 1);
    }
    if (ne == 0 && nd > 0 && na > 0 && (do_dep || do_arr)) {
        const A380XFplnLeg *last = &dep[nd - 1], *first = &arr[0];
        if (ends_at_fix(last) && has_fix(first) && same_fix(&last->fix, &first->fix) && !a380x_leg_starts_at_fix(first->type)
            && nd > 1) {
            if (first->alt_db.desc) dep[nd - 1].alt_db = first->alt_db;
            memmove(arr, arr + 1, (size_t)(na - 1) * sizeof *arr);
            na--;
        } else if (!(a380x_leg_starts_at_fix(first->type) && ends_at_fix(last) && same_fix(&last->fix, &first->fix))) {
            disco_init(&enr[ne++], A380X_SEG_ENROUTE, 1);
        }
    }
    if (nd + ne + na > A380X_FPLN_MAX_ELEMENTS) return A380X_FPLN_E_FULL;
    r->count = 0;
    for (int i = 0; i < nd; i++) r->el[r->count++] = dep[i];
    for (int i = 0; i < ne; i++) r->el[r->count++] = enr[i];
    for (int i = 0; i < na; i++) r->el[r->count++] = arr[i];
    if (!altn) {
        if (p->active_leg_index <= 1) p->active_leg_index = r->count > 1 ? 1 : (r->count ? 0 : -1);
        else if (have_old) {
            int found = -1;
            for (int i = 0; i < r->count; i++)
                if (r->el[i].segment == old_act.segment && is_leg(&r->el[i]) && streq(r->el[i].fix.ident, old_act.fix.ident)) { found = i; break; }
            p->active_leg_index = found >= 0 ? found : (old_active < r->count ? old_active : r->count - 1);
        }
    }
    normalize(p, altn);
    return 0;
}

/* ============================================================================================ */
/* Geometry, naming, indices                                                                    */
/* ============================================================================================ */

static double crs_true(const A380XFplnLeg *l)
{
    if (!isfinite(l->course_deg)) return NAN;
    return a380x_geo_norm360(l->course_true ? l->course_deg : l->course_deg + l->magvar_deg);
}

/* first point along course from (lat,lon) at which the distance to (clat,clon) crosses `target` */
static int dme_point(double lat, double lon, double crs, double clat, double clon, double target, double *olat, double *olon)
{
    double d0 = a380x_geo_distance_nm(lat, lon, clat, clon) - target;
    double prev_s = 0, prev_d = d0;
    for (double s = 0.2; s <= 200.0; s += 0.2) {
        double la, lo;
        a380x_geo_destination(lat, lon, crs, s, &la, &lo);
        double d = a380x_geo_distance_nm(la, lo, clat, clon) - target;
        if ((d <= 0) != (prev_d <= 0)) {
            double a = prev_s, b = s;
            for (int k = 0; k < 30; k++) {
                double m = 0.5 * (a + b), ml, mo;
                a380x_geo_destination(lat, lon, crs, m, &ml, &mo);
                double dm = a380x_geo_distance_nm(ml, mo, clat, clon) - target;
                if ((dm <= 0) == (prev_d <= 0)) a = m; else b = m;
            }
            a380x_geo_destination(lat, lon, crs, 0.5 * (a + b), olat, olon);
            return 1;
        }
        prev_s = s;
        prev_d = d;
    }
    /* already beyond the DME distance and moving away: the leg terminates immediately */
    *olat = lat;
    *olon = lon;
    return 1;
}

static void pad3(char *out, size_t n, double v)
{
    int d = (int)lround(a380x_geo_norm360(v));
    if (d == 0) d = 360;
    snprintf(out, n, "%03d", d);
}

static void name_leg(A380XFlightPlans *fp, const A380XFplnRoute *r, A380XFplnLeg *l)
{
    char ccc[8] = "---";
    const char *mag = l->course_true ? "T" : DEG_SIGN;
    char ch = a380x_leg_is_heading(l->type) ? 'H' : 'C';
    if (isfinite(l->course_deg)) pad3(ccc, sizeof ccc, l->course_deg);
    l->annotation[0] = 0;
    if (l->flags & A380X_LEGF_DIRTO_TP) { scopy(l->ident, sizeof l->ident, "T-P"); return; }
    const char *proc = l->airway_ident[0] ? l->airway_ident : l->procedure_ident;
    char nav3[4];
    scopy(nav3, sizeof nav3, l->navaid.ident[0] ? l->navaid.ident : l->fix.ident);
    if (l->wp_desc == A380X_WPDESC_RUNWAY && l->fix.kind == A380X_NAV_RUNWAY &&
        (l->type == A380X_LEG_IF || l->type == A380X_LEG_TF || l->type == A380X_LEG_CF || l->type == A380X_LEG_DF)) {
        const char *icao = (l->flags & A380X_LEGF_ORIGIN) ? r->origin_icao : r->dest_icao;
        snprintf(l->ident, sizeof l->ident, "%.4s%.4s", icao, l->fix.ident + 2);
        scopy(l->annotation, sizeof l->annotation, proc);
        (void)fp;
        return;
    }
    switch (l->type) {
    case A380X_LEG_IF: case A380X_LEG_TF: case A380X_LEG_DF:
        scopy(l->ident, sizeof l->ident, l->fix.ident);
        scopy(l->annotation, sizeof l->annotation, proc);
        break;
    case A380X_LEG_CF:
        scopy(l->ident, sizeof l->ident, l->fix.ident);
        snprintf(l->annotation, sizeof l->annotation, "C%s%s", ccc, mag);
        break;
    case A380X_LEG_AF:
        scopy(l->ident, sizeof l->ident, l->fix.ident);
        snprintf(l->annotation, sizeof l->annotation, "%2d %.3s", isfinite(l->rho_nm) ? (int)lround(l->rho_nm) : 0, l->navaid.ident);
        break;
    case A380X_LEG_RF:
        scopy(l->ident, sizeof l->ident, l->fix.ident);
        snprintf(l->annotation, sizeof l->annotation, "%2d ARC", isfinite(l->length_nm) ? (int)lround(l->length_nm) : 0);
        break;
    case A380X_LEG_CA: case A380X_LEG_FA: case A380X_LEG_VA:
        snprintf(l->ident, sizeof l->ident, "%d", isfinite(l->alt_db.alt1_ft) ? (int)lround(l->alt_db.alt1_ft) : 0);
        snprintf(l->annotation, sizeof l->annotation, "%c%s%s", ch, ccc, mag);
        break;
    case A380X_LEG_CD: case A380X_LEG_FC: case A380X_LEG_FD: case A380X_LEG_VD: {
        const char *src = l->type == A380X_LEG_FC ? l->fix.ident : (l->navaid.ident[0] ? l->navaid.ident : l->fix.ident);
        snprintf(l->ident, sizeof l->ident, "%.3s/%02d", src, isfinite(l->length_nm) ? (int)lround(l->length_nm) : 0);
        snprintf(l->annotation, sizeof l->annotation, "%c%s%s", ch, ccc, mag);
        break;
    }
    case A380X_LEG_CI: case A380X_LEG_VI:
        scopy(l->ident, sizeof l->ident, "INTCPT");
        snprintf(l->annotation, sizeof l->annotation, "%c%s%s", ch, ccc, mag);
        break;
    case A380X_LEG_CR: case A380X_LEG_VR: {
        char t3[8] = "000";
        if (isfinite(l->theta_deg)) snprintf(t3, sizeof t3, "%03d", (int)lround(l->theta_deg) % 360);
        snprintf(l->ident, sizeof l->ident, "%.3s%s", l->navaid.ident, t3);
        snprintf(l->annotation, sizeof l->annotation, "%c%s%s", ch, ccc, mag);
        break;
    }
    case A380X_LEG_HA:
        snprintf(l->ident, sizeof l->ident, "%d", isfinite(l->alt_db.alt1_ft) ? (int)lround(l->alt_db.alt1_ft) : 0);
        snprintf(l->annotation, sizeof l->annotation, "HOLD %c", l->turn == A380X_TURN_LEFT ? 'L' : 'R');
        break;
    case A380X_LEG_HF: case A380X_LEG_HM:
        scopy(l->ident, sizeof l->ident, l->fix.ident);
        snprintf(l->annotation, sizeof l->annotation, "HOLD %c", l->turn == A380X_TURN_LEFT ? 'L' : 'R');
        break;
    case A380X_LEG_PI:
        scopy(l->ident, sizeof l->ident, "INTCPT");
        snprintf(l->annotation, sizeof l->annotation, "PROC %c", l->turn == A380X_TURN_LEFT ? 'L' : 'R');
        break;
    case A380X_LEG_FM: case A380X_LEG_VM:
        scopy(l->ident, sizeof l->ident, "MANUAL");
        snprintf(l->annotation, sizeof l->annotation, "%c%s%s", ch, ccc, mag);
        break;
    default:
        snprintf(l->ident, sizeof l->ident, "(UNKN %s)", a380x_leg_type_name(l->type));
        scopy(l->annotation, sizeof l->annotation, "UNKNOWN");
        break;
    }
}

static void apply_hold(A380XFplnLeg *l)
{
    if (!a380x_leg_is_hx(l->type)) return;
    const A380XFplnHold *h = l->hold_modified.valid ? &l->hold_modified : (l->hold_default.valid ? &l->hold_default : NULL);
    if (!h) return;
    l->course_deg = h->inbound_crs_mag;
    l->course_true = 0;
    l->turn = h->turn;
    l->length_nm = h->dist_nm;
    l->length_min = h->time_min;
}

static void compute_route(A380XFlightPlans *fp, A380XFplnRoute *r, float origin_elev)
{
    double plat = NAN, plon = NAN;
    double cum = 0.0;
    double est_alt = isfinite(origin_elev) ? origin_elev : 0.0;
    r->first_missed_appr_index = r->count;
    r->last_enroute_index = -1;
    for (int i = 0; i < r->count; i++) {
        if (r->el[i].segment == A380X_SEG_MISSED && r->first_missed_appr_index == r->count) r->first_missed_appr_index = i;
        if (r->el[i].segment <= A380X_SEG_ENROUTE) r->last_enroute_index = i;
    }
    for (int i = 0; i < r->count; i++) {
        A380XFplnLeg *l = &r->el[i];
        if (i == r->first_missed_appr_index) cum = 0.0;
        if (a380x_fpln_is_disco(l)) { plat = plon = NAN; continue; }
        if (has_fix(l)) l->magvar_deg = (float)a380x_nav_magvar(fp->nav, l->fix.lat, l->fix.lon);
        else if (finite2(plat, plon)) l->magvar_deg = (float)a380x_nav_magvar(fp->nav, plat, plon);
        apply_hold(l);
        name_leg(fp, r, l);
        l->flags &= ~(uint32_t)(A380X_LEGF_CALC_VALID | A380X_LEGF_EST_GEOMETRY);
        double sl = plat, so = plon;
        if (a380x_leg_starts_at_fix(l->type) && has_fix(l)) { sl = l->fix.lat; so = l->fix.lon; }
        double tl = NAN, to = NAN, dist = NAN;
        double crs = crs_true(l);
        switch (l->type) {
        case A380X_LEG_IF: case A380X_LEG_TF: case A380X_LEG_DF: case A380X_LEG_CF:
        case A380X_LEG_HF: case A380X_LEG_HA: case A380X_LEG_HM:
            if (has_fix(l)) {
                tl = l->fix.lat; to = l->fix.lon;
                dist = finite2(sl, so) ? a380x_geo_distance_nm(sl, so, tl, to) : (i == 0 || l->type == A380X_LEG_IF ? 0.0 : NAN);
                if (l->type == A380X_LEG_IF && !finite2(sl, so)) dist = 0.0;
            }
            if (l->type == A380X_LEG_HA) l->flags |= A380X_LEGF_EST_GEOMETRY;
            break;
        case A380X_LEG_RF: case A380X_LEG_AF:
            if (has_fix(l)) {
                tl = l->fix.lat; to = l->fix.lon;
                const A380XNavRef *c = l->type == A380X_LEG_RF ? &l->arc_center : &l->navaid;
                if (l->type == A380X_LEG_RF && isfinite(l->length_nm)) dist = l->length_nm;
                else if (finite2(sl, so) && finite2(c->lat, c->lon)) {
                    double rad = l->type == A380X_LEG_RF ? (isfinite(l->arc_radius_nm) ? l->arc_radius_nm : a380x_geo_distance_nm(c->lat, c->lon, tl, to))
                                                         : (isfinite(l->rho_nm) ? l->rho_nm : a380x_geo_distance_nm(c->lat, c->lon, tl, to));
                    double b1 = a380x_geo_bearing_deg(c->lat, c->lon, sl, so), b2 = a380x_geo_bearing_deg(c->lat, c->lon, tl, to);
                    double sweep = a380x_geo_angle_diff(b1, b2);
                    if (l->turn == A380X_TURN_RIGHT && sweep < 0) sweep += 360;
                    if (l->turn == A380X_TURN_LEFT && sweep > 0) sweep -= 360;
                    dist = fabs(sweep) * A380X_NAV_PI / 180.0 * rad;
                } else if (finite2(sl, so)) dist = a380x_geo_distance_nm(sl, so, tl, to);
            }
            break;
        case A380X_LEG_PI:
            if (has_fix(l)) {
                tl = l->fix.lat; to = l->fix.lon;
                double ex = isfinite(l->length_nm) ? l->length_nm : 10.0;
                dist = (finite2(sl, so) ? a380x_geo_distance_nm(sl, so, tl, to) : 0.0) + 2.0 * ex;
            }
            break;
        case A380X_LEG_CA: case A380X_LEG_FA: case A380X_LEG_VA:
            if (finite2(sl, so) && isfinite(crs)) {
                double a1 = isfinite(l->alt_db.alt1_ft) ? l->alt_db.alt1_ft : est_alt + 1000.0;
                double d = (a1 - est_alt) / EST_FT_PER_NM;
                if (d < 0.5) d = 0.5;
                if (d > 30.0) d = 30.0;
                a380x_geo_destination(sl, so, crs, d, &tl, &to);
                dist = d;
                l->flags |= A380X_LEGF_EST_GEOMETRY;
            }
            break;
        case A380X_LEG_FC:
            if (finite2(sl, so) && isfinite(crs) && isfinite(l->length_nm)) {
                a380x_geo_destination(sl, so, crs, l->length_nm, &tl, &to);
                dist = l->length_nm;
            }
            break;
        case A380X_LEG_CD: case A380X_LEG_FD: case A380X_LEG_VD: {
            const A380XNavRef *c = l->navaid.ident[0] ? &l->navaid : &l->fix;
            if (finite2(sl, so) && isfinite(crs) && finite2(c->lat, c->lon) && isfinite(l->length_nm)
                && dme_point(sl, so, crs, c->lat, c->lon, l->length_nm, &tl, &to))
                dist = a380x_geo_distance_nm(sl, so, tl, to);
            break;
        }
        case A380X_LEG_CR: case A380X_LEG_VR:
            if (finite2(sl, so) && isfinite(crs) && finite2(l->navaid.lat, l->navaid.lon) && isfinite(l->theta_deg)) {
                double rad = a380x_geo_norm360(l->theta_deg + l->magvar_deg);
                if (a380x_geo_intersect(sl, so, crs, l->navaid.lat, l->navaid.lon, rad, &tl, &to))
                    dist = a380x_geo_distance_nm(sl, so, tl, to);
            }
            break;
        case A380X_LEG_CI: case A380X_LEG_VI: {
            const A380XFplnLeg *nx = i + 1 < r->count ? &r->el[i + 1] : NULL;
            if (finite2(sl, so) && isfinite(crs) && nx && has_fix(nx)) {
                double ncrs = crs_true(nx);
                if (!isfinite(ncrs)) ncrs = a380x_geo_bearing_deg(sl, so, nx->fix.lat, nx->fix.lon);
                if (a380x_geo_intersect(sl, so, crs, nx->fix.lat, nx->fix.lon, ncrs, &tl, &to)
                    && a380x_geo_distance_nm(sl, so, tl, to) < 100.0)
                    dist = a380x_geo_distance_nm(sl, so, tl, to);
                else { tl = to = NAN; }
            }
            break;
        }
        case A380X_LEG_FM: case A380X_LEG_VM:
        default:
            break;
        }
        l->start_lat = sl; l->start_lon = so;
        l->term_lat = tl; l->term_lon = to;
        l->leg_dist_nm = (float)dist;
        if (finite2(sl, so) && finite2(tl, to) && dist > 0.01) l->path_true_crs = (float)a380x_geo_bearing_deg(sl, so, tl, to);
        else l->path_true_crs = (float)crs;
        if (isfinite(dist)) {
            cum += dist;
            l->flags |= A380X_LEGF_CALC_VALID;
            l->cum_dist_nm = l->cum_dist_with_trans_nm = (float)cum;
        } else {
            l->cum_dist_nm = l->cum_dist_with_trans_nm = NAN;
        }
        A380XFplnAltCstr a = a380x_fpln_eff_alt(l);
        if (l->wp_desc == A380X_WPDESC_RUNWAY && isfinite(a.alt1_ft)) est_alt = a.alt1_ft;
        else if (a.desc && isfinite(a.alt1_ft) && a.alt1_ft > est_alt) est_alt = a.alt1_ft;
        else if ((l->type == A380X_LEG_CA || l->type == A380X_LEG_FA || l->type == A380X_LEG_VA) && isfinite(l->alt_db.alt1_ft))
            est_alt = l->alt_db.alt1_ft;
        plat = tl; plon = to;
        if (a380x_leg_is_vectors(l->type)) plat = plon = NAN;
    }
    /* destination leg */
    r->destination_leg_index = -1;
    for (int i = r->first_missed_appr_index - 1; i >= 0; i--) {
        const A380XFplnLeg *l = &r->el[i];
        if (!is_leg(l)) continue;
        if ((l->flags & A380X_LEGF_DESTINATION) || (l->fix.kind == A380X_NAV_RUNWAY && l->segment > A380X_SEG_ENROUTE)
            || l->desc[3] == 'M' || (l->fix.kind == A380X_NAV_AIRPORT && streq(l->fix.ident, r->dest_icao) && l->segment > A380X_SEG_ENROUTE)) {
            r->destination_leg_index = i;
            break;
        }
    }
}

static void route_db_info(A380XFlightPlans *fp, A380XFplnRoute *r)
{
    A380XNavAirportInfo a;
    r->db_trans_alt_ft = r->db_trans_level_ft = NAN;
    r->origin_elev_ft = r->dest_elev_ft = NAN;
    if (r->origin_icao[0] && a380x_nav_airport_info(fp->nav, r->origin_icao, &a)) { r->db_trans_alt_ft = a.trans_alt_ft; r->origin_elev_ft = a.elev_ft; }
    if (r->dest_icao[0] && a380x_nav_airport_info(fp->nav, r->dest_icao, &a)) { r->db_trans_level_ft = a.trans_level_ft; r->dest_elev_ft = a.elev_ft; }
}

void a380x_fpln_recompute(A380XFlightPlans *fp, int plan)
{
    if (!fp || plan < 0 || plan >= A380X_PLAN_COUNT) return;
    A380XFplnPlan *p = &fp->plan[plan];
    if (!p->exists) return;
    route_db_info(fp, &p->pri);
    route_db_info(fp, &p->alt);
    float oe = p->pri.origin_elev_ft;
    if (p->pri.count && p->pri.el[0].wp_desc == A380X_WPDESC_RUNWAY) {
        const A380XNavRunway *rw = a380x_nav_runway(a380x_nav_airport(fp->nav, p->pri.origin_icao), p->pri.origin_rwy);
        if (rw && isfinite(rw->thr_elev_ft)) oe = rw->thr_elev_ft;
    }
    compute_route(fp, &p->pri, oe);
    compute_route(fp, &p->alt, p->alt.origin_elev_ft);
    if (p->pri.count == 0) p->active_leg_index = -1;
    else if (p->active_leg_index >= p->pri.count) p->active_leg_index = p->pri.count - 1;
    else if (p->active_leg_index < 0) p->active_leg_index = p->pri.count > 1 ? 1 : 0;
}

/* ============================================================================================ */
/* Plan-level operations                                                                        */
/* ============================================================================================ */

static void set_alt_route(A380XFlightPlans *fp, A380XFplnPlan *p, const char *altn)
{
    route_init(&p->alt);
    if (!altn || !altn[0] || !p->pri.dest_icao[0]) return;
    scopy(p->alt.origin_icao, sizeof p->alt.origin_icao, p->pri.dest_icao);
    scopy(p->alt.dest_icao, sizeof p->alt.dest_icao, altn);
    rebuild(fp, p, 1, 1, 1);
}

int a380x_fpln_new_city_pair(A380XFlightPlans *fp, int plan, const char *from, const char *to, const char *altn)
{
    if (!fp || !valid_plan(plan) || plan == A380X_PLAN_TEMPORARY || !from || !to) return A380X_FPLN_E_ARG;
    A380XNavAirportInfo ai;
    if (!a380x_nav_airport_info(fp->nav, from, &ai) || !a380x_nav_airport_info(fp->nav, to, &ai)) return A380X_FPLN_E_NOT_IN_DB;
    if (altn && altn[0] && !a380x_nav_airport_info(fp->nav, altn, &ai)) return A380X_FPLN_E_NOT_IN_DB;
    if (plan == A380X_PLAN_ACTIVE && fp->plan[A380X_PLAN_TEMPORARY].exists) {
        fp->plan[A380X_PLAN_TEMPORARY].version++;
        plan_init(&fp->plan[A380X_PLAN_TEMPORARY]);
    }
    A380XFplnPlan *p = &fp->scratch;
    p->version = fp->plan[plan].version;
    plan_init(p);
    p->flags = A380X_PLANF_MANUAL_CREATION;
    scopy(p->pri.origin_icao, sizeof p->pri.origin_icao, from);
    scopy(p->pri.dest_icao, sizeof p->pri.dest_icao, to);
    p->active_leg_index = 1;
    int e = rebuild(fp, p, 0, 1, 1);
    if (e) return e;
    set_alt_route(fp, p, altn);
    p->exists = 1;
    p->version++;
    memcpy(&fp->plan[plan], p, sizeof *p);
    a380x_fpln_recompute(fp, plan);
    return 0;
}

int a380x_fpln_set_alternate(A380XFlightPlans *fp, int plan, const char *altn)
{
    if (!fp || !valid_plan(plan)) return A380X_FPLN_E_ARG;
    A380XFplnPlan *p = &fp->plan[plan];
    if (!p->exists) return A380X_FPLN_E_NO_PLAN;
    A380XNavAirportInfo ai;
    if (altn && altn[0] && !a380x_nav_airport_info(fp->nav, altn, &ai)) return A380X_FPLN_E_NOT_IN_DB;
    memcpy(&fp->scratch, p, sizeof *p);
    set_alt_route(fp, &fp->scratch, altn);
    fp->scratch.version++;
    memcpy(p, &fp->scratch, sizeof *p);
    a380x_fpln_recompute(fp, plan);
    return 0;
}

int a380x_fpln_delete_plan(A380XFlightPlans *fp, int plan)
{
    if (!fp || plan < 0 || plan >= A380X_PLAN_COUNT) return A380X_FPLN_E_ARG;
    uint32_t v = fp->plan[plan].version + 1;
    plan_init(&fp->plan[plan]);
    fp->plan[plan].version = v;
    return 0;
}

/* ---------------- TMPY ---------------- */

static int airway_finalise_in(A380XFlightPlans *fp, A380XFplnPlan *p);

int a380x_fpln_tmpy_insert(A380XFlightPlans *fp, const A380XFplnAircraft *ac)
{
    if (!fp) return A380X_FPLN_E_ARG;
    A380XFplnPlan *t = &fp->plan[A380X_PLAN_TEMPORARY];
    if (!t->exists) return A380X_FPLN_E_NO_PLAN;
    memcpy(&fp->scratch, t, sizeof *t);
    A380XFplnPlan *s = &fp->scratch;
    if (s->awy.active) airway_finalise_in(fp, s);
    for (int i = 0; i < s->pri.count; i++) {
        A380XFplnLeg *l = &s->pri.el[i];
        if (!(l->flags & A380X_LEGF_PENDING_DIRTO_TP)) continue;
        if (ac && ac->valid) {
            l->fix.lat = ac->lat;
            l->fix.lon = ac->lon;
            double var = a380x_nav_magvar(fp->nav, ac->lat, ac->lon);
            double trk = isfinite(ac->true_track_deg) ? ac->true_track_deg : 0.0;
            l->course_deg = (float)a380x_geo_norm360(trk - var);
            l->magvar_deg = (float)var;
        }
        l->flags &= ~(uint32_t)A380X_LEGF_PENDING_DIRTO_TP;
    }
    s->flags &= ~(uint32_t)(A380X_PLANF_DIRTO_PENDING | A380X_PLANF_COPIED_FROM_ACTIVE);
    A380XFplnPlan *a = &fp->plan[A380X_PLAN_ACTIVE];
    uint32_t v = (a->version > s->version ? a->version : s->version) + 1;
    memcpy(a, s, sizeof *s);
    a->version = v;
    a->exists = 1;
    uint32_t tv = t->version + 1;
    plan_init(t);
    t->version = tv;
    a380x_fpln_recompute(fp, A380X_PLAN_ACTIVE);
    return 0;
}

int a380x_fpln_tmpy_erase(A380XFlightPlans *fp)
{
    if (!fp) return A380X_FPLN_E_ARG;
    A380XFplnPlan *t = &fp->plan[A380X_PLAN_TEMPORARY];
    if (!t->exists) return A380X_FPLN_E_NO_PLAN;
    uint32_t v = t->version + 1;
    plan_init(t);
    t->version = v;
    return 0;
}

/* ---------------- SEC ---------------- */

static int sec_index(int n) { return (n >= 1 && n <= A380X_FPLN_SEC_COUNT) ? A380X_PLAN_SEC1 + n - 1 : -1; }

int a380x_fpln_sec_copy(A380XFlightPlans *fp, int src, int n)
{
    int d = sec_index(n);
    if (!fp || d < 0 || src < 0 || src >= A380X_PLAN_COUNT || src == d) return A380X_FPLN_E_ARG;
    if (!fp->plan[src].exists) return A380X_FPLN_E_NO_PLAN;
    uint32_t v = fp->plan[d].version + 1;
    memcpy(&fp->plan[d], &fp->plan[src], sizeof fp->plan[d]);
    fp->plan[d].version = v;
    fp->plan[d].flags &= ~(uint32_t)(A380X_PLANF_COPIED_FROM_ACTIVE | A380X_PLANF_DIRTO_PENDING);
    if (src == A380X_PLAN_ACTIVE) fp->plan[d].flags |= A380X_PLANF_COPIED_FROM_ACTIVE;
    memset(&fp->plan[d].awy, 0, sizeof fp->plan[d].awy);
    for (int i = 0; i < fp->plan[d].pri.count; i++) fp->plan[d].pri.el[i].pred.valid = 0;
    return 0;
}
int a380x_fpln_sec_copy_from_active(A380XFlightPlans *fp, int n) { return a380x_fpln_sec_copy(fp, A380X_PLAN_ACTIVE, n); }
int a380x_fpln_sec_delete(A380XFlightPlans *fp, int n)
{
    int d = sec_index(n);
    if (!fp || d < 0) return A380X_FPLN_E_ARG;
    return a380x_fpln_delete_plan(fp, d);
}
static int sec_check(const A380XFlightPlans *fp, int d)
{
    if (fp->plan[A380X_PLAN_TEMPORARY].exists) return A380X_FPLN_E_TMPY_EXISTS;
    if (!fp->plan[d].exists) return A380X_FPLN_E_NO_PLAN;
    if (!fp->plan[d].pri.origin_icao[0] || !fp->plan[d].pri.dest_icao[0]) return A380X_FPLN_E_NOT_ALLOWED;
    return 0;
}
int a380x_fpln_sec_activate(A380XFlightPlans *fp, int n)
{
    int d = sec_index(n);
    if (!fp || d < 0) return A380X_FPLN_E_ARG;
    int e = sec_check(fp, d);
    if (e) return e;
    A380XFplnPlan *a = &fp->plan[A380X_PLAN_ACTIVE];
    A380XFplnFixInfo fi[A380X_FPLN_FIX_INFOS];
    memcpy(fi, a->fix_info, sizeof fi);
    uint32_t v = (a->version > fp->plan[d].version ? a->version : fp->plan[d].version) + 1;
    memcpy(a, &fp->plan[d], sizeof *a);
    memcpy(a->fix_info, fi, sizeof fi);
    a->version = v;
    a->flags &= ~(uint32_t)A380X_PLANF_COPIED_FROM_ACTIVE;
    a380x_fpln_recompute(fp, A380X_PLAN_ACTIVE);
    return 0;
}
int a380x_fpln_sec_swap(A380XFlightPlans *fp, int n)
{
    int d = sec_index(n);
    if (!fp || d < 0) return A380X_FPLN_E_ARG;
    int e = sec_check(fp, d);
    if (e) return e;
    A380XFplnPlan *a = &fp->plan[A380X_PLAN_ACTIVE], *s = &fp->plan[d];
    memcpy(&fp->scratch, a, sizeof *a);
    uint32_t va = a->version, vs = s->version;
    memcpy(a, s, sizeof *a);
    memcpy(s, &fp->scratch, sizeof *s);
    memcpy(a->fix_info, s->fix_info, sizeof a->fix_info);
    a->version = va + 1;
    s->version = vs + 1;
    a->flags &= ~(uint32_t)A380X_PLANF_COPIED_FROM_ACTIVE;
    a380x_fpln_recompute(fp, A380X_PLAN_ACTIVE);
    a380x_fpln_recompute(fp, d);
    return 0;
}
int a380x_fpln_sec_can_activate(const A380XFlightPlans *fp, int n, int hdg_trk, int preflight)
{
    int d = sec_index(n);
    if (!fp || d < 0 || sec_check(fp, d)) return 0;
    const A380XFplnPlan *a = &fp->plan[A380X_PLAN_ACTIVE], *s = &fp->plan[d];
    if (hdg_trk || preflight) return 1;
    if (!a->exists || (a->active_leg_index < 0 && s->active_leg_index < 0)) return 1;
    if (a->active_leg_index == s->active_leg_index && a->active_leg_index >= 0 && a->active_leg_index < a->pri.count
        && a->active_leg_index < s->pri.count) {
        const A380XFplnLeg *x = &a->pri.el[a->active_leg_index], *y = &s->pri.el[s->active_leg_index];
        if (x->type == y->type && streq(x->fix.ident, y->fix.ident)) return 1;
    }
    return 0;
}

/* ---------------- departure / arrival selections ---------------- */

static void norm_rwy(char *out, size_t n, const char *in)
{
    if (!in || !in[0]) { out[0] = 0; return; }
    if (!strncmp(in, "RW", 2)) scopy(out, n, in);
    else snprintf(out, n, "RW%s", in);
}
static int is_none(const char *s) { return s && !strcmp(s, "NONE"); }

typedef enum { SEL_ORIGIN_RWY, SEL_SID, SEL_SID_TRANS, SEL_DEST_RWY, SEL_APPR, SEL_VIA, SEL_STAR, SEL_STAR_TRANS } SelKind;

static int set_selection(A380XFlightPlans *fp, int plan, int altn, SelKind k, const char *id)
{
    int err;
    A380XFplnPlan *p = begin_edit(fp, plan, &err);
    if (!p) return err;
    A380XFplnRoute *r = route_of(p, altn);
    const char *v = id ? id : "";
    int none = is_none(v);
    const A380XNavAirport *oa = r->origin_icao[0] ? a380x_nav_airport(fp->nav, r->origin_icao) : NULL;
    switch (k) {
    case SEL_ORIGIN_RWY: {
        char rw[A380X_NAV_IDENT_LEN];
        norm_rwy(rw, sizeof rw, v);
        if (rw[0] && !a380x_nav_runway(oa, rw)) return A380X_FPLN_E_NOT_IN_DB;
        scopy(r->origin_rwy, sizeof r->origin_rwy, rw);
        r->sid[0] = r->sid_trans[0] = 0;
        r->sid_none = 0;
        break;
    }
    case SEL_SID:
        if (v[0] && !none) {
            const A380XNavProcedure *s = a380x_nav_procedure(oa, A380X_PROC_SID, v);
            if (!s) return A380X_FPLN_E_NOT_IN_DB;
            if (!a380x_nav_sid_serves_runway(oa, s, r->origin_rwy)) return A380X_FPLN_E_NOT_ALLOWED;
        }
        scopy(r->sid, sizeof r->sid, none ? "" : v);
        r->sid_none = none;
        r->sid_trans[0] = 0;
        break;
    case SEL_SID_TRANS: {
        if (v[0] && !none) {
            const A380XNavProcedure *s = a380x_nav_procedure(oa, A380X_PROC_SID, r->sid);
            if (!s || !find_trans(oa, s, A380X_TRANS_ENROUTE, v)) return A380X_FPLN_E_NOT_IN_DB;
        }
        scopy(r->sid_trans, sizeof r->sid_trans, none ? "" : v);
        break;
    }
    default: {
        const A380XNavAirport *da = r->dest_icao[0] ? a380x_nav_airport(fp->nav, r->dest_icao) : NULL;
        if (k == SEL_DEST_RWY) {
            char rw[A380X_NAV_IDENT_LEN];
            norm_rwy(rw, sizeof rw, v);
            if (rw[0] && !a380x_nav_runway(da, rw)) return A380X_FPLN_E_NOT_IN_DB;
            scopy(r->dest_rwy, sizeof r->dest_rwy, rw);
            r->approach[0] = r->appr_via[0] = 0;
            r->appr_none = 0;
        } else if (k == SEL_APPR) {
            if (v[0] && !none) {
                const A380XNavProcedure *a = a380x_nav_procedure(da, A380X_PROC_APPROACH, v);
                if (!a) return A380X_FPLN_E_NOT_IN_DB;
                if (a->runway[0]) scopy(r->dest_rwy, sizeof r->dest_rwy, a->runway);
            }
            scopy(r->approach, sizeof r->approach, none ? "" : v);
            r->appr_none = none;
            r->appr_via[0] = 0;
        } else if (k == SEL_VIA) {
            if (v[0] && !none) {
                const A380XNavProcedure *a = a380x_nav_procedure(da, A380X_PROC_APPROACH, r->approach);
                if (!a || !find_trans(da, a, A380X_TRANS_APPR_VIA, v)) return A380X_FPLN_E_NOT_IN_DB;
            }
            scopy(r->appr_via, sizeof r->appr_via, none ? "" : v);
        } else if (k == SEL_STAR) {
            if (v[0] && !none && !a380x_nav_procedure(da, A380X_PROC_STAR, v)) return A380X_FPLN_E_NOT_IN_DB;
            scopy(r->star, sizeof r->star, none ? "" : v);
            r->star_none = none;
            r->star_trans[0] = 0;
        } else {
            if (v[0] && !none) {
                const A380XNavProcedure *s = a380x_nav_procedure(da, A380X_PROC_STAR, r->star);
                if (!s || !find_trans(da, s, A380X_TRANS_ENROUTE, v)) return A380X_FPLN_E_NOT_IN_DB;
            }
            scopy(r->star_trans, sizeof r->star_trans, none ? "" : v);
        }
        break;
    }
    }
    int dep = k <= SEL_SID_TRANS;
    err = rebuild(fp, p, altn, dep, !dep);
    if (err) return err;
    commit_edit(fp, plan);
    return 0;
}

int a380x_fpln_set_origin_runway(A380XFlightPlans *fp, int plan, int altn, const char *v) { return set_selection(fp, plan, altn, SEL_ORIGIN_RWY, v); }
int a380x_fpln_set_sid(A380XFlightPlans *fp, int plan, int altn, const char *v) { return set_selection(fp, plan, altn, SEL_SID, v); }
int a380x_fpln_set_sid_trans(A380XFlightPlans *fp, int plan, int altn, const char *v) { return set_selection(fp, plan, altn, SEL_SID_TRANS, v); }
int a380x_fpln_set_dest_runway(A380XFlightPlans *fp, int plan, int altn, const char *v) { return set_selection(fp, plan, altn, SEL_DEST_RWY, v); }
int a380x_fpln_set_approach(A380XFlightPlans *fp, int plan, int altn, const char *v) { return set_selection(fp, plan, altn, SEL_APPR, v); }
int a380x_fpln_set_appr_via(A380XFlightPlans *fp, int plan, int altn, const char *v) { return set_selection(fp, plan, altn, SEL_VIA, v); }
int a380x_fpln_set_star(A380XFlightPlans *fp, int plan, int altn, const char *v) { return set_selection(fp, plan, altn, SEL_STAR, v); }
int a380x_fpln_set_star_trans(A380XFlightPlans *fp, int plan, int altn, const char *v) { return set_selection(fp, plan, altn, SEL_STAR_TRANS, v); }

void a380x_fpln_approach_name(A380XFlightPlans *fp, const A380XFplnRoute *r, char *out, size_t n)
{
    if (!out || !n) return;
    out[0] = 0;
    if (!fp || !r || !r->approach[0]) return;
    const A380XNavProcedure *a = a380x_nav_procedure(a380x_nav_airport(fp->nav, r->dest_icao), A380X_PROC_APPROACH, r->approach);
    if (a) a380x_nav_approach_name(a, 0, out, n);
    else scopy(out, n, r->approach);
}
void a380x_fpln_runway_display(const A380XFplnRoute *r, int dest, char *out, size_t n)
{
    if (!out || !n) return;
    out[0] = 0;
    if (!r) return;
    const char *icao = dest ? r->dest_icao : r->origin_icao, *rw = dest ? r->dest_rwy : r->origin_rwy;
    if (!rw[0]) return;
    snprintf(out, n, "%.4s%.4s", icao, rw + 2);
}

/* ---------------- lateral revisions ---------------- */

static int check_idx(const A380XFplnRoute *r, int idx) { return idx >= 0 && idx < r->count; }

/* segment for an element inserted after `at` (keeps segment order non-decreasing) */
static A380XFplnSegment seg_after(const A380XFplnRoute *r, int at)
{
    A380XFplnSegment sa = r->count ? r->el[at].segment : A380X_SEG_ENROUTE;
    A380XFplnSegment sn = at + 1 < r->count ? r->el[at + 1].segment : A380X_SEG_COUNT;
    if (sa == sn) return sa;
    if (sa <= A380X_SEG_ENROUTE && sn >= A380X_SEG_ENROUTE) return A380X_SEG_ENROUTE;
    if (sa == A380X_SEG_ORIGIN) return sn < A380X_SEG_ENROUTE ? sn : A380X_SEG_ENROUTE;
    return sa;
}

static void to_tf(A380XFplnLeg *l)
{
    if (!is_leg(l)) return;
    if (l->type == A380X_LEG_IF || l->type == A380X_LEG_CF || l->type == A380X_LEG_DF || l->type == A380X_LEG_AF
        || l->type == A380X_LEG_RF)
        if (has_fix(l)) l->type = A380X_LEG_TF;
}

int a380x_fpln_delete_element(A380XFlightPlans *fp, int plan, int altn, int idx)
{
    int err;
    A380XFplnPlan *p = begin_edit(fp, plan, &err);
    if (!p) return err;
    A380XFplnRoute *r = route_of(p, altn);
    if (!check_idx(r, idx)) return A380X_FPLN_E_ARG;
    A380XFplnLeg *e = &r->el[idx];
    if (!altn && idx == a380x_fpln_from_leg_index(p)) return A380X_FPLN_E_NOT_ALLOWED;
    if (a380x_fpln_is_disco(e)) {
        if (idx > 0 && is_leg(&r->el[idx - 1]) && a380x_leg_is_vectors(r->el[idx - 1].type)) return A380X_FPLN_E_NOT_ALLOWED;
        el_remove(p, altn, idx, 1);
        if (idx > 0 && idx < r->count && ends_at_fix(&r->el[idx - 1]) && has_fix(&r->el[idx])
            && same_fix(&r->el[idx - 1].fix, &r->el[idx].fix) && !a380x_leg_is_hx(r->el[idx].type)) {
            A380XFplnLeg *a = &r->el[idx - 1], *b = &r->el[idx];
            if (b->alt_db.desc && !a->alt_db.desc) a->alt_db = b->alt_db;
            if (b->spd_db.desc && !a->spd_db.desc) a->spd_db = b->spd_db;
            el_remove(p, altn, idx, 1);
        } else if (idx < r->count) to_tf(&r->el[idx]);
    } else {
        int n = 1;
        if (idx + 1 < r->count && is_leg(&r->el[idx + 1]) && r->el[idx + 1].type == A380X_LEG_HM
            && same_fix(&r->el[idx + 1].fix, &e->fix) && !a380x_leg_is_hx(e->type)) n = 2;
        el_remove(p, altn, idx, n);
        if (idx < r->count) {
            if (idx == 0 || a380x_fpln_is_disco(&r->el[idx - 1])) {
                if (has_fix(&r->el[idx]) && a380x_leg_is_xf(r->el[idx].type)) r->el[idx].type = A380X_LEG_IF;
            } else to_tf(&r->el[idx]);
        }
    }
    normalize(p, altn);
    commit_edit(fp, plan);
    return 0;
}

static int find_downstream(const A380XFplnRoute *r, int after, const A380XNavRef *f)
{
    for (int k = after + 1; k < r->count && k < r->first_missed_appr_index; k++)
        if (is_leg(&r->el[k]) && ends_at_fix(&r->el[k]) && same_fix(&r->el[k].fix, f)) return k;
    return -1;
}

/* inserts legs[] after `at` with the FBW duplicate rule; returns 0 or an error */
static int insert_sequence(A380XFplnPlan *p, int altn, int at, A380XFplnLeg *legs, int n, int add_disco)
{
    A380XFplnRoute *r = route_of(p, altn);
    if (n <= 0) return 0;
    int dup = find_downstream(r, at, &legs[n - 1].fix);
    if (dup >= 0) {
        /* keep the downstream copy (with its constraints): drop our last leg, remove in between */
        if (n == 1) { el_remove(p, altn, at + 1, dup - at - 1); return 0; }
        el_remove(p, altn, at + 1, dup - at - 1);
        n--;
        add_disco = 0;
    }
    if (r->count + n + (add_disco ? 1 : 0) > A380X_FPLN_MAX_ELEMENTS) return A380X_FPLN_E_FULL;
    A380XFplnSegment seg = seg_after(r, at);
    int prev_disco = a380x_fpln_is_disco(&r->el[at]);
    for (int i = 0; i < n; i++) {
        legs[i].segment = seg;
        if (i == 0) legs[i].type = prev_disco ? A380X_LEG_IF : A380X_LEG_TF;
        int e = el_insert(p, altn, at + 1 + i, &legs[i]);
        if (e) return e;
    }
    int after = at + 1 + n;
    if (dup >= 0) {
        if (after < r->count) to_tf(&r->el[after]);
    } else if (add_disco && !(after < r->count && a380x_fpln_is_disco(&r->el[after]))) {
        A380XFplnLeg d;
        disco_init(&d, seg, 0);
        int e = el_insert(p, altn, after, &d);
        if (e) return e;
    }
    return 0;
}

int a380x_fpln_next_waypoint(A380XFlightPlans *fp, int plan, int altn, int after, const A380XNavFix *f)
{
    if (!f || !isfinite(f->lat)) return A380X_FPLN_E_ARG;
    int err;
    A380XFplnPlan *p = begin_edit(fp, plan, &err);
    if (!p) return err;
    A380XFplnRoute *r = route_of(p, altn);
    if (!check_idx(r, after)) return A380X_FPLN_E_ARG;
    if (is_leg(&r->el[after]) && a380x_leg_is_vectors(r->el[after].type)) return A380X_FPLN_E_NOT_ALLOWED;
    A380XFplnLeg l;
    leg_from_fix(&l, f, A380X_LEG_TF, A380X_SEG_ENROUTE);
    l.flags |= A380X_LEGF_PILOT;
    err = insert_sequence(p, altn, after, &l, 1, 1);
    if (err) return err;
    normalize(p, altn);
    commit_edit(fp, plan);
    return 0;
}

int a380x_fpln_new_dest(A380XFlightPlans *fp, int plan, int altn, int at, const char *icao)
{
    A380XNavAirportInfo ai;
    if (!icao || !icao[0]) return A380X_FPLN_E_ARG;
    if (!a380x_nav_airport_info(fp->nav, icao, &ai)) return A380X_FPLN_E_NOT_IN_DB;
    int err;
    A380XFplnPlan *p = begin_edit(fp, plan, &err);
    if (!p) return err;
    A380XFplnRoute *r = route_of(p, altn);
    if (!check_idx(r, at)) return A380X_FPLN_E_ARG;
    A380XFplnSegment seg = r->el[at].segment < A380X_SEG_ENROUTE ? A380X_SEG_ENROUTE : r->el[at].segment;
    if (seg > A380X_SEG_ENROUTE) seg = A380X_SEG_ENROUTE;
    el_remove(p, altn, at + 1, r->count - at - 1);
    /* everything kept becomes departure/enroute so the arrival rebuild does not drop it */
    for (int i = 0; i < r->count; i++) if (r->el[i].segment > A380X_SEG_ENROUTE) r->el[i].segment = A380X_SEG_ENROUTE;
    A380XFplnLeg d;
    disco_init(&d, seg, 0);
    el_insert(p, altn, r->count, &d);
    scopy(r->dest_icao, sizeof r->dest_icao, icao);
    r->dest_rwy[0] = r->star[0] = r->star_trans[0] = r->approach[0] = r->appr_via[0] = 0;
    r->star_none = r->appr_none = 0;
    /* append the destination airport leg directly (the pilot disco stays) */
    A380XFplnLeg l;
    if (!airport_leg(fp, &l, icao, "", A380X_SEG_DESTINATION, "")) return A380X_FPLN_E_NOT_IN_DB;
    l.flags |= A380X_LEGF_DESTINATION;
    if (el_insert(p, altn, r->count, &l)) return A380X_FPLN_E_FULL;
    if (!altn && p->alt.count) {   /* alternate starts at the new destination */
        char altn_icao[A380X_NAV_ICAO_LEN];
        scopy(altn_icao, sizeof altn_icao, p->alt.dest_icao);
        set_alt_route(fp, p, altn_icao);
    }
    commit_edit(fp, plan);
    return 0;
}

int a380x_fpln_toggle_overfly(A380XFlightPlans *fp, int plan, int altn, int idx)
{
    int err;
    A380XFplnPlan *p = begin_edit(fp, plan, &err);
    if (!p) return err;
    A380XFplnRoute *r = route_of(p, altn);
    if (!check_idx(r, idx)) return A380X_FPLN_E_ARG;
    if (!a380x_fpln_is_db_fix(&r->el[idx]) || (!altn && idx == a380x_fpln_from_leg_index(p))) return A380X_FPLN_E_NOT_ALLOWED;
    r->el[idx].flags ^= A380X_LEGF_OVERFLY;
    commit_edit(fp, plan);
    return 0;
}

int a380x_fpln_enable_altn(A380XFlightPlans *fp, int plan, int at, int *out_crz_fl)
{
    int err;
    A380XFplnPlan *p = begin_edit(fp, plan, &err);
    if (!p) return err;
    A380XFplnRoute *r = &p->pri, *a = &p->alt;
    if (!check_idx(r, at) || a380x_fpln_is_disco(&r->el[at])) return A380X_FPLN_E_ARG;
    if (!a->dest_icao[0]) return A380X_FPLN_E_NOT_ALLOWED;
    A380XNavAirportInfo d1, d2;
    int crz = 100;
    if (a380x_nav_airport_info(fp->nav, r->dest_icao, &d1) && a380x_nav_airport_info(fp->nav, a->dest_icao, &d2)) {
        double dist = a380x_geo_distance_nm(d1.lat, d1.lon, d2.lat, d2.lon);
        crz = dist > 300 ? 310 : (dist > 150 ? 220 : 100);
    }
    el_remove(p, 0, at + 1, r->count - at - 1);
    for (int i = 0; i < r->count; i++) if (r->el[i].segment > A380X_SEG_ENROUTE) r->el[i].segment = A380X_SEG_ENROUTE;
    A380XFplnLeg d;
    disco_init(&d, A380X_SEG_ENROUTE, 0);
    if (el_insert(p, 0, r->count, &d)) return A380X_FPLN_E_FULL;
    int k = 0;
    while (k < a->count && (a->el[k].segment == A380X_SEG_ORIGIN || a380x_fpln_is_disco(&a->el[k]))) k++;
    for (; k < a->count; k++) {
        A380XFplnLeg l = a->el[k];
        if (l.segment < A380X_SEG_ENROUTE) l.segment = A380X_SEG_ENROUTE;
        if (el_insert(p, 0, r->count, &l)) return A380X_FPLN_E_FULL;
    }
    if (r->count > at + 2) { A380XFplnLeg *f = &r->el[at + 2]; if (a380x_leg_is_xf(f->type) && has_fix(f)) f->type = A380X_LEG_IF; }
    scopy(r->dest_icao, sizeof r->dest_icao, a->dest_icao);
    scopy(r->dest_rwy, sizeof r->dest_rwy, a->dest_rwy);
    scopy(r->star, sizeof r->star, a->star);
    scopy(r->star_trans, sizeof r->star_trans, a->star_trans);
    scopy(r->approach, sizeof r->approach, a->approach);
    scopy(r->appr_via, sizeof r->appr_via, a->appr_via);
    route_init(a);
    normalize(p, 0);
    if (out_crz_fl) *out_crz_fl = crz;
    commit_edit(fp, plan);
    return 0;
}

/* ---------------- DIR TO ---------------- */

static void make_tp(A380XFlightPlans *fp, A380XFplnLeg *tp, const A380XFplnAircraft *ac, A380XFplnSegment seg)
{
    leg_init(tp);
    tp->type = A380X_LEG_CF;
    tp->segment = seg;
    tp->flags = A380X_LEGF_DIRTO_TP | A380X_LEGF_PENDING_DIRTO_TP;
    scopy(tp->fix.ident, sizeof tp->fix.ident, "T-P");
    tp->fix.kind = A380X_NAV_USER;
    tp->fix.lat = ac->lat;
    tp->fix.lon = ac->lon;
    double var = a380x_nav_magvar(fp->nav, ac->lat, ac->lon);
    tp->magvar_deg = (float)var;
    tp->course_deg = (float)a380x_geo_norm360((isfinite(ac->true_track_deg) ? ac->true_track_deg : 0.0) - var);
    tp->wp_desc = A380X_WPDESC_OTHER;
}

/* Builds the DIR TO result into fp->scratch from ACTIVE. target leg `tgt` (copy) at index k of
 * ACTIVE; rest = elements of ACTIVE to append after the new legs (start index rest_from). */
static int dirto_build(A380XFlightPlans *fp, const A380XFplnAircraft *ac, const A380XFplnLeg *tgt, int rest_from,
                       int bypass_from, int bypass_to, A380XDirToMode mode, float crs, int with_disco)
{
    const A380XFplnPlan *a = &fp->plan[A380X_PLAN_ACTIVE];
    A380XFplnPlan *s = &fp->scratch;
    memcpy(s, a, sizeof *s);
    s->flags = (s->flags & ~(uint32_t)A380X_PLANF_COPIED_FROM_ACTIVE) | A380X_PLANF_COPIED_FROM_ACTIVE | A380X_PLANF_DIRTO_PENDING;
    memset(&s->awy, 0, sizeof s->awy);
    A380XFplnRoute *r = &s->pri;
    A380XFplnLeg *buf = fp->work[0].el;
    int n = 0;
    A380XFplnSegment seg = tgt->segment;
    if (rest_from < a->pri.count && a->pri.el[rest_from].segment < seg && !with_disco) seg = a->pri.el[rest_from].segment;
    make_tp(fp, &buf[n++], ac, seg);
    if (mode == A380X_DIRTO_ABEAM) {
        /* abeam points of bypassed XF fixes, in along-track order */
        for (int k = bypass_from; k < bypass_to && n < 20; k++) {
            const A380XFplnLeg *b = &a->pri.el[k];
            if (!ends_at_fix(b) || !a380x_leg_is_xf(b->type)) continue;
            double along, cross;
            a380x_geo_along_cross(ac->lat, ac->lon, tgt->fix.lat, tgt->fix.lon, b->fix.lat, b->fix.lon, &along, &cross);
            double total = a380x_geo_distance_nm(ac->lat, ac->lon, tgt->fix.lat, tgt->fix.lon);
            if (along <= 0.5 || along >= total - 0.5) continue;
            A380XFplnLeg l;
            leg_init(&l);
            l.type = n == 1 ? A380X_LEG_DF : A380X_LEG_TF;
            l.segment = seg;
            l.flags = A380X_LEGF_ABEAM;
            snprintf(l.fix.ident, sizeof l.fix.ident, "AB%.3s", b->fix.ident);
            l.fix.kind = A380X_NAV_USER;
            a380x_geo_destination(ac->lat, ac->lon, a380x_geo_bearing_deg(ac->lat, ac->lon, tgt->fix.lat, tgt->fix.lon), along,
                                  &l.fix.lat, &l.fix.lon);
            l.wp_desc = A380X_WPDESC_OTHER;
            buf[n++] = l;
        }
    }
    A380XFplnLeg t = *tgt;
    t.segment = seg < t.segment ? t.segment : seg;
    t.flags &= ~(uint32_t)(A380X_LEGF_ORIGIN);
    t.flags |= A380X_LEGF_DIRTO_FIX;
    int keep_hx = a380x_leg_is_hx(tgt->type);
    if (mode == A380X_DIRTO_RADIAL_IN) {
        t.type = A380X_LEG_CF;
        t.course_deg = crs;
        t.course_true = 0;
    } else {
        t.type = n > 1 ? A380X_LEG_TF : A380X_LEG_DF;
    }
    if (keep_hx) { t.hold_default.valid = 0; t.hold_modified.valid = 0; }
    buf[n++] = t;
    if (keep_hx) buf[n++] = *tgt;
    if (mode == A380X_DIRTO_RADIAL_OUT) {
        A380XFplnLeg fm;
        leg_init(&fm);
        fm.type = A380X_LEG_FM;
        fm.segment = t.segment;
        fm.fix = t.fix;
        fm.course_deg = crs;
        fm.flags = A380X_LEGF_PILOT;
        buf[n++] = fm;
        with_disco = 1;
    }
    if (with_disco) disco_init(&buf[n++], t.segment, 0);
    for (int k = rest_from; k < a->pri.count && n < A380X_FPLN_MAX_ELEMENTS; k++) buf[n++] = a->pri.el[k];
    if (n > A380X_FPLN_MAX_ELEMENTS) return A380X_FPLN_E_FULL;
    /* keep segment order non-decreasing */
    for (int k = 1; k < n; k++) if (buf[k].segment < buf[k - 1].segment) buf[k].segment = buf[k - 1].segment;
    memcpy(r->el, buf, (size_t)n * sizeof *buf);
    r->count = n;
    s->active_leg_index = 1;
    normalize(s, 0);
    return 0;
}

static void erase_tmpy_silent(A380XFlightPlans *fp)
{
    if (fp->plan[A380X_PLAN_TEMPORARY].exists) a380x_fpln_tmpy_erase(fp);
}

int a380x_fpln_direct_to_leg(A380XFlightPlans *fp, int plan, const A380XFplnAircraft *ac, int k, A380XDirToMode mode, float crs)
{
    if (!fp || plan != A380X_PLAN_ACTIVE || !ac || !ac->valid) return A380X_FPLN_E_ARG;
    const A380XFplnPlan *a = &fp->plan[A380X_PLAN_ACTIVE];
    if (!a->exists) return A380X_FPLN_E_NO_PLAN;
    const A380XFplnRoute *r = &a->pri;
    if (!check_idx(r, k)) return A380X_FPLN_E_ARG;
    const A380XFplnLeg *t = &r->el[k];
    if (k >= r->first_missed_appr_index || !has_fix(t) || !(a380x_leg_is_xf(t->type) || a380x_leg_is_hx(t->type))
        || (t->flags & A380X_LEGF_DIRTO_TP) || k < (a->active_leg_index < 0 ? 0 : a->active_leg_index - 1))
        return A380X_FPLN_E_NOT_ALLOWED;
    if ((mode == A380X_DIRTO_RADIAL_IN || mode == A380X_DIRTO_RADIAL_OUT) && !isfinite(crs)) return A380X_FPLN_E_ARG;
    A380XFplnLeg tgt = *t;
    int rest = k + 1;
    if (a380x_leg_is_hx(t->type)) rest = k + 1;   /* hold kept after the DF (dirto_build) */
    int e = dirto_build(fp, ac, &tgt, rest, a->active_leg_index < 0 ? 0 : a->active_leg_index, k, mode, crs, 0);
    if (e) return e;
    erase_tmpy_silent(fp);
    commit_edit(fp, A380X_PLAN_ACTIVE);
    return 0;
}

int a380x_fpln_direct_to_fix(A380XFlightPlans *fp, int plan, const A380XFplnAircraft *ac, const A380XNavFix *f,
                             A380XDirToMode mode, float crs)
{
    if (!fp || plan != A380X_PLAN_ACTIVE || !ac || !ac->valid || !f || !isfinite(f->lat)) return A380X_FPLN_E_ARG;
    const A380XFplnPlan *a = &fp->plan[A380X_PLAN_ACTIVE];
    if (!a->exists) return A380X_FPLN_E_NO_PLAN;
    A380XNavRef ref;
    a380x_nav_ref_from_fix(&ref, f);
    int act = a->active_leg_index < 0 ? 0 : a->active_leg_index;
    for (int k = act; k < a->pri.first_missed_appr_index; k++)
        if (ends_at_fix(&a->pri.el[k]) && same_fix(&a->pri.el[k].fix, &ref)
            && (a380x_leg_is_xf(a->pri.el[k].type) || a380x_leg_is_hx(a->pri.el[k].type)))
            return a380x_fpln_direct_to_leg(fp, plan, ac, k, mode, crs);
    if ((mode == A380X_DIRTO_RADIAL_IN || mode == A380X_DIRTO_RADIAL_OUT) && !isfinite(crs)) return A380X_FPLN_E_ARG;
    A380XFplnLeg tgt;
    A380XFplnSegment seg = act < a->pri.count ? a->pri.el[act].segment : A380X_SEG_ENROUTE;
    leg_from_fix(&tgt, f, A380X_LEG_DF, seg);
    tgt.flags |= A380X_LEGF_PILOT;
    /* the rest from the active leg: drop leading non-ground-referenced legs, first XF becomes IF */
    int rest = act;
    while (rest < a->pri.count && !(a380x_leg_is_xf(a->pri.el[rest].type) && has_fix(&a->pri.el[rest]))) rest++;
    int e = dirto_build(fp, ac, &tgt, rest, act, act, mode == A380X_DIRTO_ABEAM ? A380X_DIRTO_DIRECT : mode, crs, 1);
    if (e) return e;
    A380XFplnRoute *r = &fp->scratch.pri;
    for (int i = 0; i < r->count; i++)
        if (a380x_fpln_is_disco(&r->el[i])) {
            if (i + 1 < r->count && a380x_leg_is_xf(r->el[i + 1].type) && has_fix(&r->el[i + 1])) r->el[i + 1].type = A380X_LEG_IF;
            break;
        }
    erase_tmpy_silent(fp);
    commit_edit(fp, A380X_PLAN_ACTIVE);
    return 0;
}

/* ---------------- HOLD ---------------- */

int a380x_fpln_default_hold(A380XFlightPlans *fp, int plan, int altn, int idx, float alt_ft, int use_db, A380XFplnHold *out)
{
    const A380XFplnRoute *r = a380x_fpln_route(fp, plan, altn);
    if (!r || !out) return A380X_FPLN_E_NO_PLAN;
    if (!check_idx(r, idx) || !has_fix(&r->el[idx])) return A380X_FPLN_E_ARG;
    const A380XFplnLeg *l = &r->el[idx];
    memset(out, 0, sizeof *out);
    out->valid = 1;
    out->dist_nm = NAN;
    if (a380x_leg_is_hx(l->type)) {
        *out = l->hold_modified.valid ? l->hold_modified : l->hold_default;
        return 0;
    }
    if (use_db) {
        A380XNavHold h[4];
        int n = a380x_nav_find_holds(fp->nav, l->fix.ident, l->fix.region, h, 4);
        for (int i = 0; i < n && i < 4; i++) {
            if (finite2(h[i].fix.lat, h[i].fix.lon) && a380x_geo_distance_nm(h[i].fix.lat, h[i].fix.lon, l->fix.lat, l->fix.lon) > 1.0) continue;
            if (isfinite(alt_ft) && h[i].max_alt_ft > 0 && alt_ft > h[i].max_alt_ft) continue;
            if (isfinite(alt_ft) && h[i].min_alt_ft > 0 && alt_ft < h[i].min_alt_ft) continue;
            out->type = A380X_HOLD_DATABASE;
            out->inbound_crs_mag = h[i].inbound_mag;
            out->turn = h[i].turn == 'L' ? A380X_TURN_LEFT : A380X_TURN_RIGHT;
            out->time_min = h[i].leg_time_min > 0 ? h[i].leg_time_min : NAN;
            out->dist_nm = h[i].leg_dist_nm > 0 ? h[i].leg_dist_nm : NAN;
            if (!isfinite(out->time_min) && !isfinite(out->dist_nm)) out->time_min = 1.0f;
            return 0;
        }
    }
    out->type = A380X_HOLD_COMPUTED;
    out->turn = A380X_TURN_RIGHT;
    out->time_min = (isfinite(alt_ft) && alt_ft > 14000.0f) ? 1.5f : 1.0f;
    out->inbound_crs_mag = 100.0f;
    if (idx > 0 && is_leg(&r->el[idx - 1]) && a380x_leg_is_xf(r->el[idx - 1].type) && has_fix(&r->el[idx - 1])) {
        double brg = a380x_geo_bearing_deg(r->el[idx - 1].fix.lat, r->el[idx - 1].fix.lon, l->fix.lat, l->fix.lon);
        out->inbound_crs_mag = (float)a380x_geo_norm360(brg - a380x_nav_magvar(fp->nav, l->fix.lat, l->fix.lon));
    }
    return 0;
}

int a380x_fpln_add_or_edit_hold(A380XFlightPlans *fp, int plan, int altn, int at, const A380XFplnHold *desired,
                                const A380XFplnHold *modified, const A380XFplnHold *dflt)
{
    if (!desired || !dflt) return A380X_FPLN_E_ARG;
    int err;
    A380XFplnPlan *p = begin_edit(fp, plan, &err);
    if (!p) return err;
    A380XFplnRoute *r = route_of(p, altn);
    if (!check_idx(r, at) || !has_fix(&r->el[at])) return A380X_FPLN_E_ARG;
    A380XFplnLeg *l = &r->el[at];
    int result = at;
    if (a380x_leg_is_hx(l->type)) {
        if (modified) { l->hold_modified = *modified; l->hold_modified.valid = 1; }
        else l->hold_modified.valid = 0;
        if (!l->hold_default.valid) l->hold_default = *dflt;
        l->type = A380X_LEG_HM;
    } else {
        if (!a380x_fpln_is_db_fix(l)) return A380X_FPLN_E_NOT_ALLOWED;
        A380XFplnLeg h;
        leg_from_ref(&h, &l->fix, A380X_LEG_HM, l->segment);
        h.hold_default = *dflt;
        h.hold_default.valid = 1;
        if (modified) { h.hold_modified = *modified; h.hold_modified.valid = 1; }
        h.magvar_deg = l->magvar_deg;
        h.cstr_type = l->cstr_type;
        err = el_insert(p, altn, at + 1, &h);
        if (err) return err;
        result = at + 1;
    }
    commit_edit(fp, plan);
    return result;
}

int a380x_fpln_revert_hold_to_computed(A380XFlightPlans *fp, int plan, int altn, int hm)
{
    int err;
    A380XFplnPlan *p = begin_edit(fp, plan, &err);
    if (!p) return err;
    A380XFplnRoute *r = route_of(p, altn);
    if (!check_idx(r, hm) || !a380x_leg_is_hx(r->el[hm].type)) return A380X_FPLN_E_ARG;
    r->el[hm].hold_modified.valid = 0;
    commit_edit(fp, plan);
    return 0;
}

int a380x_fpln_set_hold_imm_exit(A380XFlightPlans *fp, int hm, int exit)
{
    if (!fp) return A380X_FPLN_E_ARG;
    A380XFplnPlan *p = &fp->plan[A380X_PLAN_ACTIVE];
    if (!p->exists) return A380X_FPLN_E_NO_PLAN;
    if (!check_idx(&p->pri, hm) || !a380x_leg_is_hx(p->pri.el[hm].type)) return A380X_FPLN_E_ARG;
    if (exit) p->pri.el[hm].flags |= A380X_LEGF_HOLD_IMM_EXIT;
    else p->pri.el[hm].flags &= ~(uint32_t)A380X_LEGF_HOLD_IMM_EXIT;
    p->version++;
    return 0;
}

/* ---------------- AIRWAYS ---------------- */

int a380x_fpln_airway_start(A380XFlightPlans *fp, int plan, int altn, int at)
{
    int err;
    A380XFplnPlan *p = begin_edit(fp, plan, &err);
    if (!p) return err;
    A380XFplnRoute *r = route_of(p, altn);
    if (!check_idx(r, at)) return A380X_FPLN_E_ARG;
    const A380XFplnLeg *l = &r->el[at];
    if (!a380x_fpln_is_db_fix(l) || !has_fix(l) || l->wp_desc == A380X_WPDESC_AIRPORT || l->wp_desc == A380X_WPDESC_RUNWAY)
        return A380X_FPLN_E_NOT_ALLOWED;
    memset(&p->awy, 0, sizeof p->awy);
    p->awy.active = 1;
    p->awy.altn = altn;
    p->awy.at_index = at;
    p->awy.line_count = 1;
    p->awy.line[0].from = l->fix;
    a380x_nav_ref_clear(&p->awy.line[0].to);
    commit_edit(fp, plan);
    return 0;
}

static A380XFplnPlan *awy_begin(A380XFlightPlans *fp, int plan, int *err)
{
    A380XFplnPlan *p = begin_edit(fp, plan, err);
    if (!p) return NULL;
    if (!p->awy.active) { *err = A380X_FPLN_E_NOT_ALLOWED; return NULL; }
    return p;
}
static void awy_append_line(A380XFplnAirwayEntry *e, const A380XNavRef *from)
{
    if (e->line_count >= A380X_FPLN_AWY_LINES) return;
    A380XFplnAwyLine *n = &e->line[e->line_count++];
    memset(n, 0, sizeof *n);
    n->from = *from;
    a380x_nav_ref_clear(&n->to);
}
static int is_dct(const char *v) { return !strcmp(v, "DCT"); }

int a380x_fpln_airway_via(A380XFlightPlans *fp, int plan, const char *awy)
{
    if (!awy || !awy[0]) return A380X_FPLN_E_ARG;
    int err;
    A380XFplnPlan *p = awy_begin(fp, plan, &err);
    if (!p) return err;
    A380XFplnAirwayEntry *e = &p->awy;
    A380XFplnAwyLine *L = &e->line[e->line_count - 1];
    if (is_dct(awy)) {
        if (L->via_locked) return A380X_FPLN_E_NOT_ALLOWED;
        scopy(L->via, sizeof L->via, "DCT");
        L->via_locked = e->line_count > 1;
        commit_edit(fp, plan);
        return 0;
    }
    if (!a380x_nav_airway_exists(fp->nav, awy)) return A380X_FPLN_E_NOT_IN_DB;
    if (L->via_locked && !L->to_locked && L->via[0] && !is_dct(L->via)) {
        /* [PORT] automatic airway-airway intersection */
        if (e->line_count >= A380X_FPLN_AWY_LINES) return A380X_FPLN_E_NOT_ALLOWED;
        A380XNavRef x;
        if (!a380x_nav_airway_intersection(fp->nav, L->via, &L->from, awy, &x)) return A380X_FPLN_E_NO_INTERSECTION;
        L->to = x;
        L->to_locked = 1;
        awy_append_line(e, &x);
        L = &e->line[e->line_count - 1];
    } else if (L->via_locked && !is_dct(L->via)) return A380X_FPLN_E_NOT_ALLOWED;
    if (!a380x_nav_airway_contains(fp->nav, awy, &L->from)) return A380X_FPLN_E_NOT_ALLOWED;
    scopy(L->via, sizeof L->via, awy);
    L->via_locked = 1;
    commit_edit(fp, plan);
    return 0;
}

int a380x_fpln_airway_to_ident(A380XFlightPlans *fp, int plan, const char *ident)
{
    if (!ident || !ident[0]) return A380X_FPLN_E_ARG;
    int err;
    A380XFplnPlan *p = awy_begin(fp, plan, &err);
    if (!p) return err;
    A380XFplnAirwayEntry *e = &p->awy;
    A380XFplnAwyLine *L = &e->line[e->line_count - 1];
    if (L->to_locked) return A380X_FPLN_E_NOT_ALLOWED;
    if (!L->via[0] || is_dct(L->via)) {
        A380XNavFix f;
        int n = a380x_nav_find(fp->nav, ident, A380X_NAVMASK_PLACE, &f, 1);
        if (n == 0) return A380X_FPLN_E_NOT_IN_DB;
        if (n > 1) return A380X_FPLN_E_NOT_ALLOWED;   /* page must resolve duplicates and use to_fix */
        return a380x_fpln_airway_to_fix(fp, plan, &f, 1);
    }
    A380XNavRef path[64];
    int n = a380x_nav_airway_path(fp->nav, L->via, &L->from, ident, path, 64);
    if (n == -3) return A380X_FPLN_E_NOT_IN_DB;
    if (n <= 0) return A380X_FPLN_E_NO_INTERSECTION;
    L->to = path[n - 1];
    L->to_locked = 1;
    awy_append_line(e, &L->to);
    commit_edit(fp, plan);
    return 0;
}

int a380x_fpln_airway_to_fix(A380XFlightPlans *fp, int plan, const A380XNavFix *f, int dct)
{
    if (!f || !isfinite(f->lat)) return A380X_FPLN_E_ARG;
    int err;
    A380XFplnPlan *p = awy_begin(fp, plan, &err);
    if (!p) return err;
    A380XFplnAirwayEntry *e = &p->awy;
    A380XFplnAwyLine *L = &e->line[e->line_count - 1];
    if (L->to_locked) return A380X_FPLN_E_NOT_ALLOWED;
    A380XNavRef ref;
    a380x_nav_ref_from_fix(&ref, f);
    if (dct || !L->via[0] || is_dct(L->via)) {
        scopy(L->via, sizeof L->via, "DCT");
        L->via_locked = 1;
        L->to = ref;
    } else {
        A380XNavRef path[64];
        int n = a380x_nav_airway_path(fp->nav, L->via, &L->from, f->ident, path, 64);
        if (n <= 0 || !same_fix(&path[n - 1], &ref)) return A380X_FPLN_E_NO_INTERSECTION;
        L->to = path[n - 1];
    }
    L->to_locked = 1;
    awy_append_line(e, &L->to);
    commit_edit(fp, plan);
    return 0;
}

static int airway_finalise_in(A380XFlightPlans *fp, A380XFplnPlan *p)
{
    A380XFplnAirwayEntry *e = &p->awy;
    if (!e->active) return 0;
    A380XFplnLeg *buf = fp->work[2].el;
    int n = 0;
    for (int i = 0; i < e->line_count; i++) {
        const A380XFplnAwyLine *L = &e->line[i];
        if (!L->to_locked) break;
        if (is_dct(L->via) || !L->via[0]) {
            if (n >= A380X_FPLN_MAX_ELEMENTS) break;
            leg_from_ref(&buf[n], &L->to, A380X_LEG_TF, A380X_SEG_ENROUTE);
            buf[n++].flags |= A380X_LEGF_PILOT;
        } else {
            A380XNavRef path[64];
            int k = a380x_nav_airway_path(fp->nav, L->via, &L->from, L->to.ident, path, 64);
            for (int j = 0; j < k && n < A380X_FPLN_MAX_ELEMENTS; j++) {
                leg_from_ref(&buf[n], &path[j], A380X_LEG_TF, A380X_SEG_ENROUTE);
                scopy(buf[n].airway_ident, sizeof buf[n].airway_ident, L->via);
                buf[n++].flags |= A380X_LEGF_AIRWAY;
            }
        }
    }
    int altn = e->altn, at = e->at_index;
    memset(e, 0, sizeof *e);
    if (!n) return 0;
    int err = insert_sequence(p, altn, at, buf, n, 1);
    normalize(p, altn);
    return err;
}

int a380x_fpln_airway_finalise(A380XFlightPlans *fp, int plan)
{
    int err;
    A380XFplnPlan *p = begin_edit(fp, plan, &err);
    if (!p) return err;
    if (!p->awy.active) return 0;
    err = airway_finalise_in(fp, p);
    if (err) return err;
    commit_edit(fp, plan);
    return 0;
}

void a380x_fpln_airway_cancel(A380XFlightPlans *fp, int plan)
{
    if (!fp || !valid_plan(plan)) return;
    int t = plan == A380X_PLAN_ACTIVE ? A380X_PLAN_TEMPORARY : plan;
    if (fp->plan[t].exists && fp->plan[t].awy.active) {
        memset(&fp->plan[t].awy, 0, sizeof fp->plan[t].awy);
        fp->plan[t].version++;
    }
}

/* ---------------- constraints, steps, fix info ---------------- */

static void propagate_cstr_type(A380XFplnRoute *r, int idx, int is_des)
{
    if (is_des) { for (int i = idx + 1; i < r->count; i++) if (is_leg(&r->el[i]) && r->el[i].cstr_type == A380X_CSTR_UNKNOWN) r->el[i].cstr_type = A380X_CSTR_DES; }
    else { for (int i = idx - 1; i >= 0; i--) if (is_leg(&r->el[i]) && r->el[i].cstr_type == A380X_CSTR_UNKNOWN) r->el[i].cstr_type = A380X_CSTR_CLB; }
}

int a380x_fpln_set_alt_cstr(A380XFlightPlans *fp, int plan, int altn, int idx, int is_des, const A380XFplnAltCstr *c)
{
    int err;
    A380XFplnPlan *p = begin_edit(fp, plan, &err);
    if (!p) return err;
    A380XFplnRoute *r = route_of(p, altn);
    if (!check_idx(r, idx)) return A380X_FPLN_E_ARG;
    A380XFplnLeg *l = &r->el[idx];
    if (!a380x_fpln_constraint_revision_allowed(l)) return A380X_FPLN_E_NOT_ALLOWED;
    if (!c || !c->desc) {
        l->alt_db.desc = 0; l->alt_db.alt1_ft = l->alt_db.alt2_ft = NAN;
        l->alt_pilot.desc = 0; l->alt_pilot.alt1_ft = l->alt_pilot.alt2_ft = NAN;
    } else {
        if (!isfinite(c->alt1_ft)) return A380X_FPLN_E_ARG;
        l->alt_pilot = *c;
        l->cstr_type = is_des ? A380X_CSTR_DES : A380X_CSTR_CLB;
        propagate_cstr_type(r, idx, is_des);
    }
    commit_edit(fp, plan);
    return 0;
}

int a380x_fpln_set_spd_cstr(A380XFlightPlans *fp, int plan, int altn, int idx, int is_des, float kt)
{
    int err;
    A380XFplnPlan *p = begin_edit(fp, plan, &err);
    if (!p) return err;
    A380XFplnRoute *r = route_of(p, altn);
    if (!check_idx(r, idx)) return A380X_FPLN_E_ARG;
    A380XFplnLeg *l = &r->el[idx];
    if (!a380x_fpln_constraint_revision_allowed(l)) return A380X_FPLN_E_NOT_ALLOWED;
    if (!(kt > 0)) {
        l->spd_db.desc = 0; l->spd_db.speed = NAN;
        l->spd_pilot.desc = 0; l->spd_pilot.speed = NAN;
    } else {
        l->spd_pilot.desc = '-';
        l->spd_pilot.speed = kt;
        l->cstr_type = is_des ? A380X_CSTR_DES : A380X_CSTR_CLB;
        propagate_cstr_type(r, idx, is_des);
    }
    commit_edit(fp, plan);
    return 0;
}

int a380x_fpln_add_cruise_step(A380XFlightPlans *fp, int plan, int idx, float alt_ft)
{
    int err;
    A380XFplnPlan *p = begin_edit(fp, plan, &err);
    if (!p) return err;
    if (!check_idx(&p->pri, idx) || !is_leg(&p->pri.el[idx]) || !isfinite(alt_ft)) return A380X_FPLN_E_ARG;
    A380XFplnCruiseStep *s = &p->pri.el[idx].cruise_step;
    s->valid = 1; s->to_alt_ft = alt_ft; s->dist_before_term_nm = 0; s->ignored = 0;
    for (int i = 0; i < p->pri.count; i++) p->pri.el[i].cruise_step.ignored = 0;
    commit_edit(fp, plan);
    return 0;
}
int a380x_fpln_remove_cruise_step(A380XFlightPlans *fp, int plan, int idx)
{
    int err;
    A380XFplnPlan *p = begin_edit(fp, plan, &err);
    if (!p) return err;
    if (!check_idx(&p->pri, idx)) return A380X_FPLN_E_ARG;
    memset(&p->pri.el[idx].cruise_step, 0, sizeof p->pri.el[idx].cruise_step);
    commit_edit(fp, plan);
    return 0;
}

int a380x_fpln_set_fix_info(A380XFlightPlans *fp, int n, const A380XFplnFixInfo *fi)
{
    if (!fp || n < 1 || n > A380X_FPLN_FIX_INFOS) return A380X_FPLN_E_ARG;
    A380XFplnPlan *a = &fp->plan[A380X_PLAN_ACTIVE];
    if (!a->exists) return A380X_FPLN_E_NO_PLAN;
    A380XFplnFixInfo v;
    memset(&v, 0, sizeof v);
    if (fi && fi->valid) {
        v = *fi;
        double var = a380x_nav_magvar(fp->nav, v.fix.lat, v.fix.lon);
        for (int i = 0; i < v.radial_count && i < 2; i++) v.radial_true[i] = (float)a380x_geo_norm360(v.radial_mag[i] + var);
    }
    a->fix_info[n - 1] = v;
    a->version++;
    if (fp->plan[A380X_PLAN_TEMPORARY].exists) { fp->plan[A380X_PLAN_TEMPORARY].fix_info[n - 1] = v; fp->plan[A380X_PLAN_TEMPORARY].version++; }
    return 0;
}

/* ============================================================================================ */
/* Sequencing                                                                                   */
/* ============================================================================================ */

static double turn_anticipation(const A380XFplnRoute *r, int i, double gs)
{
    const A380XFplnLeg *l = &r->el[i];
    if (l->flags & A380X_LEGF_OVERFLY) return 0.0;
    if (i + 1 >= r->count || !is_leg(&r->el[i + 1])) return 0.0;
    double in_crs = (finite2(l->start_lat, l->start_lon) && finite2(l->term_lat, l->term_lon))
                    ? a380x_geo_norm360(a380x_geo_bearing_deg(l->term_lat, l->term_lon, l->start_lat, l->start_lon) + 180.0)
                    : l->path_true_crs;
    double out_crs = r->el[i + 1].path_true_crs;
    if (!isfinite(in_crs) || !isfinite(out_crs)) return 0.0;
    double d = fabs(a380x_geo_angle_diff(in_crs, out_crs));
    if (d > 150.0) d = 150.0;
    if (gs < 100.0) gs = 100.0;
    double radius_nm = (gs * gs) / (11.26 * tan(25.0 * A380X_NAV_PI / 180.0)) * NM_PER_FT;
    double a = radius_nm * tan(d * A380X_NAV_PI / 360.0);
    return a > 10.0 ? 10.0 : a;
}

/* 1 when the TO leg i is complete */
static int leg_done(const A380XFplnRoute *r, int i, const A380XFplnAircraft *ac)
{
    const A380XFplnLeg *l = &r->el[i];
    switch (l->type) {
    case A380X_LEG_CA: case A380X_LEG_FA: case A380X_LEG_VA:
        return isfinite(l->alt_db.alt1_ft) && ac->alt_ft >= l->alt_db.alt1_ft - 50.0f;
    case A380X_LEG_HA:
        return isfinite(l->alt_db.alt1_ft) && ac->alt_ft >= l->alt_db.alt1_ft - 50.0f
               && a380x_geo_distance_nm(ac->lat, ac->lon, l->fix.lat, l->fix.lon) < 1.5;
    case A380X_LEG_FM: case A380X_LEG_VM:
        return 0;
    case A380X_LEG_HM: case A380X_LEG_HF:
        return (l->flags & A380X_LEGF_HOLD_IMM_EXIT) && has_fix(l)
               && a380x_geo_distance_nm(ac->lat, ac->lon, l->fix.lat, l->fix.lon) < 1.0;
    default:
        break;
    }
    if (!finite2(l->term_lat, l->term_lon)) return 0;
    double rem;
    if (finite2(l->start_lat, l->start_lon) && a380x_geo_distance_nm(l->start_lat, l->start_lon, l->term_lat, l->term_lon) > 0.05) {
        double along, cross;
        a380x_geo_along_cross(l->start_lat, l->start_lon, l->term_lat, l->term_lon, ac->lat, ac->lon, &along, &cross);
        rem = a380x_geo_distance_nm(l->start_lat, l->start_lon, l->term_lat, l->term_lon) - along;
    } else {
        rem = a380x_geo_distance_nm(ac->lat, ac->lon, l->term_lat, l->term_lon);
        if (rem > 0.3) return 0;
        return 1;
    }
    return rem <= turn_anticipation(r, i, ac->gs_kt) + 0.05;
}

uint32_t a380x_fpln_update(A380XFlightPlans *fp, const A380XFplnAircraft *ac)
{
    if (!fp || !ac) return 0;
    A380XFplnPlan *p = &fp->plan[A380X_PLAN_ACTIVE];
    if (!p->exists || p->active_leg_index < 0 || !ac->valid) return 0;
    A380XFplnRoute *r = &p->pri;
    uint32_t ev = 0;
    int old = p->active_leg_index;
    for (int guard = 0; guard < 5 && !ac->on_ground; guard++) {
        int i = p->active_leg_index;
        if (i >= r->count || !is_leg(&r->el[i])) break;
        if (r->destination_leg_index >= 0 && i >= r->destination_leg_index && i < r->first_missed_appr_index) break;
        if (!leg_done(r, i, ac)) break;
        if (i + 1 >= r->count) break;
        if (a380x_fpln_is_disco(&r->el[i + 1])) { ev |= A380X_FPLN_EVT_AT_DISCO; break; }
        p->active_leg_index = i + 1;
        ev |= A380X_FPLN_EVT_SEQUENCED;
    }
    int a = p->active_leg_index;
    if (a >= 0 && a < r->count) {
        if (a + 1 < r->count && a380x_fpln_is_disco(&r->el[a + 1])) ev |= A380X_FPLN_EVT_DISCO_AHEAD;
        if (is_leg(&r->el[a]) && a380x_leg_is_vectors(r->el[a].type)) ev |= A380X_FPLN_EVT_MANUAL_LEG;
        if (r->destination_leg_index >= 0 && a > r->destination_leg_index) ev |= A380X_FPLN_EVT_DEST_REACHED;
    }
    if (ev & A380X_FPLN_EVT_SEQUENCED) {
        /* sequencing removes the passed waypoints from display through active/from; the version
         * changes so pages rebuild */
        p->version++;
        A380XFplnPlan *t = &fp->plan[A380X_PLAN_TEMPORARY];
        if (t->exists && t->active_leg_index == old && p->active_leg_index < t->pri.count) {
            t->active_leg_index = p->active_leg_index;
            t->version++;
        }
    }
    return ev;
}

int a380x_fpln_sequence_to(A380XFlightPlans *fp, int idx)
{
    if (!fp) return 0;
    A380XFplnPlan *p = &fp->plan[A380X_PLAN_ACTIVE];
    if (!p->exists || idx <= p->active_leg_index || idx >= p->pri.count || !is_leg(&p->pri.el[idx])) return 0;
    int old = p->active_leg_index;
    p->active_leg_index = idx;
    p->version++;
    A380XFplnPlan *t = &fp->plan[A380X_PLAN_TEMPORARY];
    if (t->exists && t->active_leg_index == old && idx < t->pri.count) {
        t->active_leg_index = idx;
        t->version++;
    }
    return 1;
}

float a380x_fpln_dist_to_active(const A380XFlightPlans *fp, int plan, const A380XFplnAircraft *ac)
{
    const A380XFplnPlan *p = a380x_fpln_plan(fp, plan);
    if (!p || !ac || !ac->valid || p->active_leg_index < 0 || p->active_leg_index >= p->pri.count) return NAN;
    const A380XFplnLeg *l = &p->pri.el[p->active_leg_index];
    if (!finite2(l->term_lat, l->term_lon)) return NAN;
    return (float)a380x_geo_distance_nm(ac->lat, ac->lon, l->term_lat, l->term_lon);
}

float a380x_fpln_dist_to_dest(const A380XFlightPlans *fp, int plan, const A380XFplnAircraft *ac)
{
    const A380XFplnPlan *p = a380x_fpln_plan(fp, plan);
    if (!p) return NAN;
    int d = p->pri.destination_leg_index, a = p->active_leg_index;
    if (d < 0 || a < 0 || a > d) return NAN;
    float sum = a380x_fpln_dist_to_active(fp, plan, ac);
    if (!isfinite(sum)) return NAN;
    double llat = p->pri.el[a].term_lat, llon = p->pri.el[a].term_lon;
    for (int k = a + 1; k <= d; k++) {
        const A380XFplnLeg *l = &p->pri.el[k];
        if (!is_leg(l)) continue;
        /* across a discontinuity count the direct distance to the next fix */
        int after_disco = a380x_fpln_is_disco(&p->pri.el[k - 1]);
        if (after_disco && finite2(llat, llon) && finite2(l->term_lat, l->term_lon))
            sum += (float)a380x_geo_distance_nm(llat, llon, l->term_lat, l->term_lon);
        else if (isfinite(l->leg_dist_nm)) sum += l->leg_dist_nm;
        if (finite2(l->term_lat, l->term_lon)) { llat = l->term_lat; llon = l->term_lon; }
    }
    return sum;
}

int a380x_fpln_leg_path(const A380XFlightPlans *fp, int plan, int altn, int idx, A380XFplnLegPath *out)
{
    const A380XFplnRoute *r = a380x_fpln_route(fp, plan, altn);
    if (!r || !out || !check_idx(r, idx)) return 0;
    memset(out, 0, sizeof *out);
    const A380XFplnLeg *l = &r->el[idx];
    out->is_altn = altn;
    out->is_missed = idx >= r->first_missed_appr_index;
    out->is_tmpy = plan == A380X_PLAN_TEMPORARY;
    out->start_lat = l->start_lat; out->start_lon = l->start_lon;
    out->end_lat = l->term_lat; out->end_lon = l->term_lon;
    out->clockwise = l->turn == A380X_TURN_RIGHT;
    if (!is_leg(l)) return 0;
    if (a380x_leg_is_hx(l->type) && has_fix(l)) {
        out->kind = A380X_PATH_HOLD;
        out->center_lat = l->fix.lat; out->center_lon = l->fix.lon;
        out->inbound_true = (float)crs_true(l);
        out->hold_leg_nm = isfinite(l->length_nm) ? l->length_nm : (float)((isfinite(l->length_min) ? l->length_min : 1.0) * 3.5);
        out->clockwise = l->turn != A380X_TURN_LEFT;
        return 1;
    }
    if (!finite2(l->start_lat, l->start_lon) || !finite2(l->term_lat, l->term_lon)) return 0;
    if ((l->type == A380X_LEG_RF || l->type == A380X_LEG_AF)) {
        const A380XNavRef *c = l->type == A380X_LEG_RF ? &l->arc_center : &l->navaid;
        if (finite2(c->lat, c->lon)) {
            out->kind = A380X_PATH_ARC;
            out->center_lat = c->lat; out->center_lon = c->lon;
            out->radius_nm = (float)((l->type == A380X_LEG_RF && isfinite(l->arc_radius_nm)) ? l->arc_radius_nm
                                     : (l->type == A380X_LEG_AF && isfinite(l->rho_nm)) ? l->rho_nm
                                     : a380x_geo_distance_nm(c->lat, c->lon, l->term_lat, l->term_lon));
            return 1;
        }
    }
    out->kind = A380X_PATH_LINE;
    return 1;
}

/* ============================================================================================ */
/* F-PLN line list                                                                              */
/* ============================================================================================ */

static void line_init(A380XFplnLine *ln)
{
    memset(ln, 0, sizeof *ln);
    ln->leg_index = ln->joint_index = -1;
    ln->trk_true_deg = ln->trk_mag_deg = ln->dist_nm = ln->fpa_deg = NAN;
    ln->alt_cstr.alt1_ft = ln->alt_cstr.alt2_ft = NAN;
    ln->spd_cstr.speed = NAN;
    ln->hold_speed_kt = NAN;
    ln->pred.altitude_ft = ln->pred.efob_kg = NAN;
}
static A380XFplnLine *q_push(A380XFplnLineIter *it)
{
    int cap = (int)(sizeof it->q / sizeof it->q[0]);
    if (it->q_count >= cap) return NULL;
    A380XFplnLine *ln = &it->q[(it->q_head + it->q_count) % cap];
    it->q_count++;
    line_init(ln);
    return ln;
}
static void push_special(A380XFplnLineIter *it, A380XFplnSpecial s, int leg_index, int joint, int altn)
{
    static const char *const TXT[] = { "", "DISCONTINUITY", "END OF F-PLN", "NO ALTN F-PLN", "END OF ALTN F-PLN", "TOO STEEP PATH" };
    A380XFplnLine *ln = q_push(it);
    if (!ln) return;
    ln->kind = A380X_FLINE_SPECIAL;
    ln->special = s;
    ln->special_text = TXT[s];
    ln->leg_index = leg_index;
    ln->joint_index = joint;
    ln->is_altn = altn;
}

void a380x_fpln_lines_begin(A380XFplnLineIter *it, const A380XFlightPlans *fp, int plan_index, const A380XFplnPredFeed *feed)
{
    memset(it, 0, sizeof *it);
    it->fp = fp;
    it->plan_index = plan_index;
    it->plan = a380x_fpln_plan(fp, plan_index);
    it->feed = feed;
    it->n_pri = it->plan ? it->plan->pri.count : 0;
    it->n_alt = it->plan ? it->plan->alt.count : 0;
    it->last_lat = it->last_lon = NAN;
}

static void expand(A380XFplnLineIter *it)
{
    const A380XFplnPlan *p = it->plan;
    if (!p) return;
    if (it->n_pri == 0) {
        if (it->empty_stage) return;
        it->empty_stage = 1;
        A380XFplnLine *ln = q_push(it);
        if (ln) { ln->kind = A380X_FLINE_WAYPOINT; scopy(ln->ident, sizeof ln->ident, "P.POS"); }
        push_special(it, A380X_FSPEC_DISCONTINUITY, -1, -1, 0);
        push_special(it, A380X_FSPEC_END_OF_FPLN, -1, -1, 0);
        push_special(it, A380X_FSPEC_NO_ALTN_FPLN, -1, -1, 0);
        return;
    }
    int total = it->n_pri + it->n_alt;
    while (it->q_count == 0 && it->i < total) {
        int i = it->i++;
        int altn = i >= it->n_pri;
        const A380XFplnRoute *r = altn ? &p->alt : &p->pri;
        int li = altn ? i - it->n_pri : i;
        const A380XFplnLeg *l = &r->el[li];
        if (i == it->n_pri || i == p->pri.first_missed_appr_index) { it->last = 0.0; }
        if (i == it->n_pri) it->have_last_pos = 0;
        double dist = NAN, trk = NAN;
        if (is_leg(l)) {
            int calc = (l->flags & A380X_LEGF_CALC_VALID) != 0;
            int term_fix = ends_at_fix(l);
            if (i != 0 && calc) {
                dist = l->cum_dist_with_trans_nm - it->last;
                if (term_fix && it->have_last_pos) trk = a380x_geo_bearing_deg(it->last_lat, it->last_lon, l->fix.lat, l->fix.lon);
            }
            if (calc) {
                it->last = l->cum_dist_with_trans_nm;
                if (term_fix) { it->last_lat = l->fix.lat; it->last_lon = l->fix.lon; it->have_last_pos = 1; }
            }
        }
        double reduce = 0.0;
        const A380XFplnPredFeed *f = it->feed;
        if (f && it->plan_index == A380X_PLAN_ACTIVE) {
            for (int k = 0; k < f->pwp_count && k < A380X_FPLN_MAX_PWP; k++) {
                const A380XFplnPwp *w = &f->pwp[k];
                if (w->along_leg_index != i) continue;
                A380XFplnLine *ln = q_push(it);
                if (!ln) break;
                ln->kind = A380X_FLINE_WAYPOINT;
                ln->is_pseudo = 1;
                ln->joint_index = i;
                scopy(ln->ident, sizeof ln->ident, w->ident);
                ln->pred = w->pred;
                ln->trk_true_deg = (float)trk;
                ln->trk_mag_deg = isfinite(trk) ? (float)a380x_geo_norm360(trk - l->magvar_deg) : NAN;
                reduce = w->dist_from_start_nm - it->last_d;
                ln->dist_nm = (float)reduce;
                ln->is_altn = altn;
                it->last_d = w->dist_from_start_nm;
            }
        }
        if (is_leg(l)) {
            int missed = li >= r->first_missed_appr_index;
            int active = !altn && li == p->active_leg_index;
            if (l->type == A380X_LEG_HM) {
                A380XFplnLine *h = q_push(it);
                if (h) {
                    h->kind = A380X_FLINE_HOLD_HM;
                    h->leg_index = li; h->joint_index = i; h->is_altn = altn; h->is_missed = missed;
                    snprintf(h->ident, sizeof h->ident, "HOLD %c", l->turn == A380X_TURN_LEFT ? 'L' : 'R');
                    h->hold_turn = l->turn;
                    h->hold_imm_exit = (l->flags & A380X_LEGF_HOLD_IMM_EXIT) != 0;
                    h->is_active_leg = active;
                    if (f && f->hold_leg_index == i) {
                        h->hold_speed_kt = f->hold_speed_kt;
                        h->hold_decel_sequenced = it->plan_index == A380X_PLAN_ACTIVE && li == p->active_leg_index + 1 && f->hold_decel_reached;
                    }
                    h->leg = l;
                }
            }
            A380XFplnLine *ln = q_push(it);
            if (ln) {
                ln->kind = A380X_FLINE_WAYPOINT;
                ln->leg_index = li; ln->joint_index = i; ln->is_altn = altn; ln->is_missed = missed;
                ln->is_active_leg = active;
                ln->is_before_active = !altn && li == p->active_leg_index - 1;
                ln->is_active_hold_fix = l->type == A380X_LEG_HM && active;
                ln->use_trans_level = i >= p->pri.last_enroute_index;
                scopy(ln->ident, sizeof ln->ident, l->ident);
                if (l->type == A380X_LEG_HM) {
                    const A380XFplnHold *h = l->hold_modified.valid ? &l->hold_modified : &l->hold_default;
                    char c3[8] = "---";
                    if (isfinite(h->inbound_crs_mag)) pad3(c3, sizeof c3, h->inbound_crs_mag);
                    snprintf(ln->annotation, sizeof ln->annotation, "C%s" DEG_SIGN "\xc2\xa0\xc2\xa0", c3);
                } else scopy(ln->annotation, sizeof ln->annotation, l->annotation);
                ln->overfly = (l->flags & A380X_LEGF_OVERFLY) != 0;
                ln->trk_true_deg = (float)trk;
                ln->trk_mag_deg = isfinite(trk) ? (float)a380x_geo_norm360(trk - (li > 0 ? r->el[li - 1].magvar_deg : l->magvar_deg)) : NAN;
                ln->dist_nm = isfinite(dist) ? (float)(dist - reduce) : NAN;
                ln->fpa_deg = l->vertical_angle_deg;
                ln->alt_cstr = a380x_fpln_eff_alt(l);
                ln->spd_cstr = a380x_fpln_eff_spd(l);
                if (f && it->plan_index == A380X_PLAN_ACTIVE && f->leg_pred && i < f->leg_pred_count) ln->pred = f->leg_pred[i];
                else { ln->pred.alt_cstr_met = ln->pred.spd_cstr_met = 1; }
                ln->leg = l;
            }
            if (l->flags & A380X_LEGF_CALC_VALID) it->last_d = l->cum_dist_with_trans_nm;
            else if (isfinite(dist)) it->last_d = it->last_d + dist - reduce;
            if (l->flags & A380X_LEGF_TOO_STEEP_AFTER) push_special(it, A380X_FSPEC_TOO_STEEP_PATH, -1, i, altn);
        } else {
            push_special(it, A380X_FSPEC_DISCONTINUITY, li, i, altn);
        }
        if (i == it->n_pri - 1) {
            push_special(it, A380X_FSPEC_END_OF_FPLN, -1, -1, 0);
            if (it->n_alt == 0) push_special(it, A380X_FSPEC_NO_ALTN_FPLN, -1, -1, 0);
        }
        if (it->n_alt > 0 && i == total - 1) push_special(it, A380X_FSPEC_END_OF_ALTN_FPLN, -1, -1, 1);
    }
}

int a380x_fpln_lines_next(A380XFplnLineIter *it, A380XFplnLine *out)
{
    if (!it) return 0;
    if (it->q_count == 0) expand(it);
    if (it->q_count == 0) return 0;
    int cap = (int)(sizeof it->q / sizeof it->q[0]);
    if (out) *out = it->q[it->q_head];
    it->q_head = (it->q_head + 1) % cap;
    it->q_count--;
    it->line_no++;
    return 1;
}

int a380x_fpln_line_count(const A380XFlightPlans *fp, int plan_index, const A380XFplnPredFeed *feed)
{
    A380XFplnLineIter it;
    a380x_fpln_lines_begin(&it, fp, plan_index, feed);
    int n = 0;
    while (a380x_fpln_lines_next(&it, NULL)) n++;
    return n;
}

int a380x_fpln_line_of_leg(const A380XFlightPlans *fp, int plan_index, const A380XFplnPredFeed *feed, int leg_index, int altn)
{
    A380XFplnLineIter it;
    A380XFplnLine ln;
    a380x_fpln_lines_begin(&it, fp, plan_index, feed);
    int n = 0;
    while (a380x_fpln_lines_next(&it, &ln)) {
        if (ln.kind == A380X_FLINE_WAYPOINT && !ln.is_pseudo && ln.leg_index == leg_index && ln.is_altn == altn) return n;
        n++;
    }
    return -1;
}
