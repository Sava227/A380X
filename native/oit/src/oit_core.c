/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380X OIT core: the Onboard Information Terminal of each pilot's side console.
 *
 * Modelled on the FlyByWire A380X OIT (fbw-a380x/src/systems/instruments/src/OIT) and the Airbus OIS
 * description (EASA A380 EFB/OIS evaluation, ATA 46):
 *  - the OIT brightness knob switches the terminal on (0 = off); after power-up a self test (black screen);
 *  - the OIT SIDE switch selects the domain: FLT OPS (flight operations laptop) or NSS AVNCS (avionics server);
 *  - each domain boots (FLT OPS needs this side's laptop), then a PILOT login opens a session;
 *  - FLT OPS: MENU / FUNCTIONS dropdowns, message box, four-column FLT OPS MENU, FLT OPS STS (flight number,
 *    FROM / TO, SYNCHRO AVIONICS), T.O PERF, LOADSHEET, LDG PERF, IN-FLT PERF, switching bar; OIS XFR mirrors the
 *    other pilot's FLT OPS display (X-OIS);
 *  - NSS AVNCS: folder-tree menu, COMPANY COM (Inbox, Flight Log with OUT/OFF/ON/IN), UTILITIES > A380X SYSTEMS
 *    (CMS current faults, doors, engines).
 * The performance pages cross-check the data entered on the MFD (FMS datarefs); this port has no Airbus
 * performance database, so nothing is computed that the real application would compute.
 * Input: OIT keyboard and ACD keys, the mouse (touch) and, in the pop-up window, the PC keyboard.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "oit_core.h"
#include "oit_draw.h"
#include "oit_font.h"

/* ---------------------------------------------------------------- theme (FBW definitions.scss) */
#define C_BG       oit_rgb(0x000000)
#define C_WHITE    oit_rgb(0xffffff)
#define C_CYAN     oit_rgb(0x00ffff)
#define C_GREEN    oit_rgb(0x00ff00)
#define C_AMBER    oit_rgb(0xe68000)
#define C_RED      oit_rgb(0xff3030)
#define C_MAGENTA  oit_rgb(0xff94ff)
#define C_BORDER   oit_rgb(0xc4c6cf)
#define C_SHADOW   oit_rgb(0x5d5f68)
#define C_DISABLED oit_rgb(0x868a9c)
#define C_THEAD    oit_rgb(0x4f5467)
#define C_BTN      oit_rgb(0x272525)
#define C_PROGRESS oit_rgb(0x00e0fe)
#define C_ROW      oit_rgb(0x101114)

#define SELFTEST_S 10.0
#define FLTOPS_LOAD_S 20.0
#define AVNCS_LOAD_S 15.0
#define MLW_KG 394000.0f
#define MZFW_KG 369000.0f
#define OIS_VERSION "12-NOV 26 V1.0"

/* ---------------------------------------------------------------- faults (CMS) */
const OitFault oit_faults[] = {
    {"rel_engfir0", 26, "ENG 1 FIRE"}, {"rel_engfir1", 26, "ENG 2 FIRE"}, {"rel_engfir2", 26, "ENG 3 FIRE"},
    {"rel_engfir3", 26, "ENG 4 FIRE"}, {"rel_apu_fire", 26, "APU FIRE"},
    {"rel_engfai0", 72, "ENG 1 FAILURE"}, {"rel_engfai1", 72, "ENG 2 FAILURE"}, {"rel_engfai2", 72, "ENG 3 FAILURE"},
    {"rel_engfai3", 72, "ENG 4 FAILURE"},
    {"rel_engfla0", 73, "ENG 1 FLAME OUT"}, {"rel_engfla1", 73, "ENG 2 FLAME OUT"}, {"rel_engfla2", 73, "ENG 3 FLAME OUT"},
    {"rel_engfla3", 73, "ENG 4 FLAME OUT"},
    {"rel_genera0", 24, "ELEC GEN 1 FAULT"}, {"rel_genera1", 24, "ELEC GEN 2 FAULT"}, {"rel_genera2", 24, "ELEC GEN 3 FAULT"},
    {"rel_genera3", 24, "ELEC GEN 4 FAULT"}, {"rel_batter0", 24, "ELEC BAT 1 FAULT"}, {"rel_batter1", 24, "ELEC BAT 2 FAULT"},
    {"rel_hydpmp", 29, "HYD ENG 1 PMP FAULT"}, {"rel_hydpmp2", 29, "HYD ENG 2 PMP FAULT"},
    {"rel_hydpmp3", 29, "HYD ENG 3 PMP FAULT"}, {"rel_hydpmp4", 29, "HYD ENG 4 PMP FAULT"},
    {"rel_hydpmp_ele", 29, "HYD ELEC PMP FAULT"}, {"rel_hydleak", 29, "HYD G SYS LEAK"}, {"rel_hydleak2", 29, "HYD Y SYS LEAK"},
    {"rel_fuepmp0", 28, "FUEL FEED TK 1 PMP FAULT"}, {"rel_fuepmp1", 28, "FUEL FEED TK 2 PMP FAULT"},
    {"rel_fuepmp2", 28, "FUEL FEED TK 3 PMP FAULT"}, {"rel_fuepmp3", 28, "FUEL FEED TK 4 PMP FAULT"},
    {"rel_apu", 49, "APU FAULT"}, {"rel_bleed_air_lft", 36, "AIR BLEED L FAULT"}, {"rel_bleed_air_rgt", 36, "AIR BLEED R FAULT"},
    {"rel_depres_slow", 21, "CAB PRESS SLOW DEPRESSURIZATION"}, {"rel_depres_fast", 21, "CAB PRESS RAPID DEPRESSURIZATION"},
    {"rel_pitot", 34, "NAV PITOT 1 BLOCKED"}, {"rel_pitot2", 34, "NAV PITOT 2 BLOCKED"},
    {"rel_static", 34, "NAV STATIC 1 BLOCKED"}, {"rel_static2", 34, "NAV STATIC 2 BLOCKED"},
    {"rel_ice_pitot_heat1", 30, "ANTI ICE PROBES 1 HEAT"}, {"rel_ice_pitot_heat2", 30, "ANTI ICE PROBES 2 HEAT"},
    {"rel_ice_window_heat", 30, "ANTI ICE WINDSHIELD HEAT"},
    {"rel_gps", 34, "NAV GPS 1 FAULT"}, {"rel_gps2", 34, "NAV GPS 2 FAULT"}, {"rel_nav1", 34, "NAV VOR/ILS 1 FAULT"},
    {"rel_nav2", 34, "NAV VOR/ILS 2 FAULT"},
    {"rel_flap_act", 27, "F/CTL FLAP ACTUATOR"}, {"rel_fc_slt", 27, "F/CTL SLAT ACTUATOR"}, {"rel_otto", 22, "AUTO FLT AP FAULT"},
    {"rel_gear_act", 32, "L/G RETRACTION ACTUATOR"}, {"rel_tire1", 32, "WHEEL TIRE 1 BURST"}, {"rel_tire2", 32, "WHEEL TIRE 2 BURST"},
    {"rel_tire3", 32, "WHEEL TIRE 3 BURST"}, {"rel_tire4", 32, "WHEEL TIRE 4 BURST"}, {"rel_tire5", 32, "WHEEL TIRE 5 BURST"},
};
const int oit_fault_count = (int)(sizeof(oit_faults) / sizeof(oit_faults[0]));

static const char *const door_names[20] = {
    "M1R", "M1L", "M2R", "M2L", "M3R", "M3L", "M4R", "M4L", "M5R", "M5L",
    "U1R", "U1L", "U2R", "U2L", "U3R", "U3L", "FWD CARGO", "AFT CARGO", "BULK CARGO", "AVIONICS",
};

/* ---------------------------------------------------------------- pages */
enum {
    PG_FO_MENU, PG_STS, PG_TO, PG_LOAD, PG_LDG, PG_INFLT,                    /* FLT OPS */
    PG_AV_MENU, PG_CCOM, PG_SYSTEMS, PG_FAULTS, PG_DOORS, PG_ENGINES,         /* NSS AVNCS */
    PG_COUNT
};
static const char *const page_title[PG_COUNT] = {
    "FLT OPS MENU", "STATUS PAGE", "T.O PERF", "LOADSHEET", "LDG PERF", "IN-FLT PERF",
    "NSS AVNCS MENU", "COMPANY COM", "A380X SYSTEMS", "CURRENT FAULTS", "DOORS", "ENGINES",
};
enum { D_FLTOPS = 0, D_AVNCS = 1 };
enum { DS_LOADING, DS_LOGIN, DS_SESSION };

/* ---------------------------------------------------------------- shared aircraft data (both OITs) */
typedef struct { double t; float fob; } Oooi;
typedef struct { char flt[12], from[8], to[8]; int doy; Oooi out, off, on, in; } FlightLog;
typedef struct { char subject[48], body[160]; int doy; double t; int read; } Message;
static struct {
    char flt[12], from[8], to[8];
    FlightLog cur, prev;
    int have_prev;
    Message inbox[16];
    int inbox_n;
    /* OOOI detection (FBW AnsuOps: OFF / ON confirmed after 10 s, IN with the park brake and a door open) */
    int init, ground;
    double ground_since, air_since, done_since, last_t;
    double touchdown_t, liftoff_t;
} ac;

/* ---------------------------------------------------------------- terminal */
typedef struct { float x, y, w, h; int action, arg, focusable; } Hit;
enum {
    A_NONE, A_PAGE, A_BACK, A_FWD, A_HOME, A_LOGIN, A_EXIT, A_DROP, A_DD_ITEM, A_EDIT, A_SYNCHRO, A_CLEAR,
    A_SCROLL_UP, A_SCROLL_DN, A_CCOM_VIEW, A_INBOX_ROW, A_HIDE_BAR
};
enum { F_FLT = 1, F_FROM, F_TO };
typedef struct { int state; double load_t; int page; int back[16], nback, fwd[16], nfwd; } Domain;

struct OitTerminal {
    int side;
    int on;                  /* powered and knob above 0 */
    double selftest_t;
    int avncs;
    Domain dom[2];
    int focus, nfocus;
    int dropdown, dd_sel;    /* FLT OPS header dropdown: 1 MENU, 2 FUNCTIONS */
    int edit_field;
    char edit[12];
    int x_ois;
    int hide_bar;
    int scroll;
    int ccom_view;           /* 0 inbox, 1 flight log */
    int inbox_sel;
    char msg[64];
    int msg_count;
    int dirty;
    Hit hits[96];
    int nhits;
};
static OitTerminal terms[2];

OitTerminal *oit_terminal(int side) { OitTerminal *t = &terms[side ? 1 : 0]; t->side = side ? 1 : 0; return t; }
void oit_reset(OitTerminal *t) { int side = t->side; memset(t, 0, sizeof(*t)); t->side = side; t->inbox_sel = -1; t->dirty = 1; }
int oit_take_dirty(OitTerminal *t) { int d = t->dirty; t->dirty = 0; return d; }

static Domain *dom(OitTerminal *t) { return &t->dom[t->avncs]; }
static void message(OitTerminal *t, const char *m) { snprintf(t->msg, sizeof(t->msg), "%s", m); ++t->msg_count; t->dirty = 1; }
static void message_both(const char *m) { message(&terms[0], m); message(&terms[1], m); }

static void go(OitTerminal *t, int page)
{
    Domain *d = dom(t);
    if (d->page != page) {
        if (d->nback < 16) d->back[d->nback++] = d->page; else { memmove(d->back, d->back + 1, 15 * sizeof(int)); d->back[15] = d->page; }
        d->nfwd = 0;
    }
    d->page = page; t->scroll = 0; t->focus = 0; t->dropdown = 0; t->edit_field = 0; t->dirty = 1;
}
static void back(OitTerminal *t)
{
    Domain *d = dom(t);
    if (!d->nback) return;
    if (d->nfwd < 16) d->fwd[d->nfwd++] = d->page;
    d->page = d->back[--d->nback]; t->scroll = 0; t->focus = 0; t->dirty = 1;
}
static void forward(OitTerminal *t)
{
    Domain *d = dom(t);
    if (!d->nfwd) return;
    if (d->nback < 16) d->back[d->nback++] = d->page;
    d->page = d->fwd[--d->nfwd]; t->scroll = 0; t->focus = 0; t->dirty = 1;
}
static void home(OitTerminal *t) { go(t, t->avncs ? PG_AV_MENU : PG_FO_MENU); }

/* ---------------------------------------------------------------- formatting */
static void fmt_hhmm(char *b, size_t n, double z) { int s = (int)fmod(z + 86400.0, 86400.0); snprintf(b, n, "%02d:%02d", s / 3600, (s / 60) % 60); }
static void doy_to_md(int doy, int year, int *m, int *d)
{
    int days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}, k = 0;
    if ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0) days[1] = 29;
    while (k < 11 && doy >= days[k]) doy -= days[k++];
    *m = k; *d = doy + 1;
}
static const char *const MON[12] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
static void fmt_dmy(char *b, size_t n, int doy, int year) { int m, d; doy_to_md(doy, year, &m, &d); snprintf(b, n, "%02d-%s-%04d", d, MON[m], year); }
static void fmt_dd_hhmm(char *b, size_t n, int doy, int year, double z)
{
    int m, d; char hm[8];
    doy_to_md(doy, year, &m, &d); fmt_hhmm(hm, sizeof(hm), z);
    snprintf(b, n, "%02d-%s %s", d, MON[m], hm);
}
static void fmt_dur(char *b, size_t n, double a, double c) { double s = c - a; int m; if (s < 0) s += 86400; m = (int)(s / 60 + 0.5); snprintf(b, n, "%02d:%02d", m / 60, m % 60); }
static void fmt_t(char *b, size_t n, float kg) { snprintf(b, n, "%.1f T", kg / 1000.0f); }
static void fmt_latlon(char *b, size_t n, double lat, double lon)
{
    double a = fabs(lat), o = fabs(lon);
    snprintf(b, n, "%c%02d\x80%04.1f' %c%03d\x80%04.1f'", lat >= 0 ? 'N' : 'S', (int)a, (a - (int)a) * 60,
             lon >= 0 ? 'E' : 'W', (int)o, (o - (int)o) * 60);
}

/* ---------------------------------------------------------------- flight log (OUT / OFF / ON / IN) */
static void post(const OitSim *s, const char *subject, const char *body)
{
    Message *m;
    if (ac.inbox_n == 16) { memmove(ac.inbox, ac.inbox + 1, 15 * sizeof(Message)); ac.inbox_n = 15; }
    m = &ac.inbox[ac.inbox_n++];
    memset(m, 0, sizeof(*m));
    snprintf(m->subject, sizeof(m->subject), "%s", subject);
    snprintf(m->body, sizeof(m->body), "%s", body);
    m->doy = s->day_of_year; m->t = s->zulu_s;
}

static void oooi_event(const OitSim *s, Oooi *o, double t, const char *what)
{
    char subj[48], body[160], hm[8];
    o->t = t; o->fob = s->fuel_kg;
    if (o == &ac.cur.out) {
        snprintf(ac.cur.flt, sizeof(ac.cur.flt), "%s", ac.flt); snprintf(ac.cur.from, 8, "%s", ac.from);
        snprintf(ac.cur.to, 8, "%s", ac.to); ac.cur.doy = s->day_of_year;
    }
    fmt_hhmm(hm, sizeof(hm), t);
    snprintf(subj, sizeof(subj), "MVT %s %sZ", what, hm);
    snprintf(body, sizeof(body), "%s %s %s-%s  %s %sZ  FOB %.1f T", ac.flt[0] ? ac.flt : "----", s->tail,
             ac.from[0] ? ac.from : "----", ac.to[0] ? ac.to : "----", what, hm, s->fuel_kg / 1000.0f);
    post(s, subj, body);
    snprintf(subj, sizeof(subj), "%s RECORDED %sZ", what, hm);
    message_both(subj);
}

static void flightlog_update(const OitSim *s)
{
    double now = s->sim_time_s, dt;
    float door = 0;
    int i;
    for (i = 0; i < 16; ++i) if (s->door_ratio[i] > door) door = s->door_ratio[i];
    if (!ac.init) { ac.init = 1; ac.ground = s->on_ground; ac.ground_since = ac.air_since = now; ac.last_t = now; return; }
    dt = now - ac.last_t; ac.last_t = now;
    if (dt < 0 || dt > 5) { ac.ground = s->on_ground; ac.ground_since = ac.air_since = now; return; }
    if (s->on_ground != ac.ground) {
        ac.ground = s->on_ground;
        if (s->on_ground) { ac.ground_since = now; ac.touchdown_t = s->zulu_s; } else { ac.air_since = now; ac.liftoff_t = s->zulu_s; }
    }
    if (ac.cur.out.t == 0 && s->on_ground && !s->parking_brake && door < 0.25f && s->gs_kt > 1.0f)
        oooi_event(s, &ac.cur.out, s->zulu_s, "OUT");
    if (ac.cur.out.t != 0 && ac.cur.off.t == 0 && !s->on_ground && now - ac.air_since >= 10.0)
        oooi_event(s, &ac.cur.off, ac.liftoff_t, "OFF");
    if (ac.cur.off.t != 0 && ac.cur.on.t == 0 && s->on_ground && now - ac.ground_since >= 10.0 && s->ias_kt < 60.0f)
        oooi_event(s, &ac.cur.on, ac.touchdown_t, "ON");
    if (ac.cur.on.t != 0 && ac.cur.in.t == 0 && s->on_ground && s->parking_brake && s->gs_kt < 1.0f &&
        (door > 0.25f || !(s->eng_running[0] || s->eng_running[1] || s->eng_running[2] || s->eng_running[3]))) {
        oooi_event(s, &ac.cur.in, s->zulu_s, "IN");
        ac.done_since = now;
    }
    /* 15 min after IN the log starts a new flight (FBW). */
    if (ac.cur.in.t != 0 && now - ac.done_since > 900.0) { ac.prev = ac.cur; ac.have_prev = 1; memset(&ac.cur, 0, sizeof(ac.cur)); }
}

static const char *flight_state(void)
{
    if (ac.cur.in.t != 0) return "IN";
    if (ac.cur.on.t != 0) return "POST-FLIGHT";
    if (ac.cur.off.t != 0) return "IN-FLIGHT";
    if (ac.cur.out.t != 0) return "PRE-FLIGHT";
    return "INITIAL";
}

/* ---------------------------------------------------------------- tick */
void oit_tick(OitTerminal *t, const OitSim *s, double dt)
{
    int k;
    if (t->side == OIT_CAPT) flightlog_update(s);
    if (s->powered && !t->on) {
        t->on = 1; t->selftest_t = 0; t->inbox_sel = -1;
        for (k = 0; k < 2; ++k) { t->dom[k].state = DS_LOADING; t->dom[k].load_t = 0; }
        t->dirty = 1;
    }
    if (!s->powered && t->on) oit_reset(t);
    if (!t->on) return;
    if (t->selftest_t < SELFTEST_S) { t->selftest_t += dt; t->dirty = 1; }
    else {
        /* the NSS server boots with the terminal; the FLT OPS laptop only while it is switched on */
        for (k = 0; k < 2; ++k) {
            Domain *d = &t->dom[k];
            if (k == D_FLTOPS && !s->laptop_on) {
                if (d->state != DS_LOADING || d->load_t > 0) { d->state = DS_LOADING; d->load_t = 0; t->dirty = 1; }
                continue;
            }
            if (d->state == DS_LOADING) {
                d->load_t += dt; t->dirty = 1;
                if (d->load_t >= (k == D_FLTOPS ? FLTOPS_LOAD_S : AVNCS_LOAD_S)) d->state = DS_LOGIN;
            }
        }
    }
    if (s->avncs != t->avncs) { t->avncs = s->avncs; t->dropdown = 0; t->edit_field = 0; t->x_ois = 0; t->focus = 0; t->dirty = 1; }
}

/* ---------------------------------------------------------------- input */
static void commit_edit(OitTerminal *t)
{
    char *dst = t->edit_field == F_FLT ? ac.flt : t->edit_field == F_FROM ? ac.from : ac.to;
    size_t cap = t->edit_field == F_FLT ? sizeof(ac.flt) : 8;
    snprintf(dst, cap, "%.*s", (int)cap - 1, t->edit);
    t->edit_field = 0;
}
static void begin_edit(OitTerminal *t, int field)
{
    const char *src = field == F_FLT ? ac.flt : field == F_FROM ? ac.from : ac.to;
    t->edit_field = field;
    snprintf(t->edit, sizeof(t->edit), "%s", src);
    t->dirty = 1;
}

static const OitSim *last_sim;   /* the snapshot of the last render, for actions triggered by keys */

static void synchro(OitTerminal *t)
{
    const OitSim *s = last_sim;
    int got = 0;
    if (s && s->on_ground && s->near_apt[0]) { snprintf(ac.from, sizeof(ac.from), "%s", s->near_apt); ++got; }
    if (s && s->dest_apt[0]) { snprintf(ac.to, sizeof(ac.to), "%s", s->dest_apt); ++got; }
    message(t, got ? "SYNCHRO AVIONICS COMPLETED" : "NO AVIONICS FLIGHT DATA");
}

/* FLT OPS header dropdowns */
typedef struct { const char *label; int enabled, action, arg; int sep_before; } DdItem;
static const DdItem dd_menu[] = {
    {"FLT OPS MENU", 1, A_PAGE, PG_FO_MENU, 0}, {"FLT FOLDER", 0, A_NONE, 0, 1}, {"TERML CHART", 0, A_NONE, 0, 0},
    {"OPS LIBRARY", 0, A_NONE, 0, 1}, {"T.O PERF", 1, A_PAGE, PG_TO, 1}, {"LOADSHEET", 1, A_PAGE, PG_LOAD, 0},
    {"LDG PERF", 1, A_PAGE, PG_LDG, 0}, {"IN-FLT PERF", 1, A_PAGE, PG_INFLT, 0}, {"FLT OPS STS", 1, A_PAGE, PG_STS, 1},
    {"LOAD BOX", 0, A_NONE, 0, 0}, {"EXPORT BOX", 0, A_NONE, 0, 0}, {"EXIT SESSION", 1, A_EXIT, 0, 1},
};
static const DdItem dd_functions[] = {
    {"HOME", 1, A_HOME, 0, 0}, {"PREVIOUS", 1, A_BACK, 0, 0}, {"NEXT", 1, A_FWD, 0, 0}, {"PRINT", 0, A_NONE, 0, 1},
    {"STORE", 0, A_NONE, 0, 0}, {"UPDATE", 0, A_NONE, 0, 0}, {"UNDO", 0, A_NONE, 0, 1}, {"REDO", 0, A_NONE, 0, 0},
    {"HELP", 0, A_NONE, 0, 1}, {"HIDE SWITCHING BAR", 1, A_HIDE_BAR, 0, 1}, {"CLOSE APPLICATION", 1, A_HOME, 0, 0},
};
#define DD_N(a) ((int)(sizeof(a) / sizeof(a[0])))

static void do_action(OitTerminal *t, int action, int arg)
{
    Domain *d = dom(t);
    t->dirty = 1;
    switch (action) {
    case A_PAGE: go(t, arg); break;
    case A_BACK: back(t); t->dropdown = 0; break;
    case A_FWD: forward(t); t->dropdown = 0; break;
    case A_HOME: home(t); break;
    case A_LOGIN:
        d->state = DS_SESSION; d->nback = d->nfwd = 0; d->page = t->avncs ? PG_AV_MENU : PG_FO_MENU; t->focus = 0;
        if (t->avncs && last_sim) {
            char body[160], hm[8];
            fmt_hhmm(hm, sizeof(hm), last_sim->zulu_s);
            snprintf(body, sizeof(body), "PILOT SESSION OPENED ON %s OIT AT %sZ.", t->side == OIT_CAPT ? "CAPT" : "F/O", hm);
            post(last_sim, "NSS AVNCS SESSION OPENED", body);
        }
        break;
    case A_EXIT: d->state = DS_LOGIN; t->dropdown = 0; t->x_ois = 0; t->focus = 0; break;
    case A_DROP: t->dropdown = t->dropdown == arg ? 0 : arg; t->dd_sel = 0; break;
    case A_DD_ITEM: {
        const DdItem *it = t->dropdown == 1 ? &dd_menu[arg] : &dd_functions[arg];
        t->dropdown = 0;
        if (it->enabled) do_action(t, it->action, it->arg);
        break;
    }
    case A_EDIT: begin_edit(t, arg); break;
    case A_SYNCHRO: synchro(t); break;
    case A_CLEAR: t->msg[0] = 0; t->msg_count = 0; break;
    case A_SCROLL_UP: if (t->scroll > 0) --t->scroll; break;
    case A_SCROLL_DN: ++t->scroll; break;
    case A_CCOM_VIEW: t->ccom_view = arg; break;
    case A_INBOX_ROW: t->inbox_sel = arg; if (arg >= 0 && arg < ac.inbox_n) ac.inbox[arg].read = 1; break;
    case A_HIDE_BAR: t->hide_bar = !t->hide_bar; break;
    default: break;
    }
}

void oit_key(OitTerminal *t, int key, char ch)
{
    Domain *d;
    if (!t->on || t->selftest_t < SELFTEST_S) return;
    d = dom(t);
    t->dirty = 1;
    if (t->x_ois) { if (key == OIT_K_XFR || key == OIT_K_ESC) t->x_ois = 0; return; }
    if (t->edit_field) {
        size_t n = strlen(t->edit), max = t->edit_field == F_FLT ? 8 : 4;
        if (key == OIT_K_CHAR) {
            char c = (ch >= 'a' && ch <= 'z') ? (char)(ch - 32) : ch;
            if (n < max && ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) { t->edit[n] = c; t->edit[n + 1] = 0; }
        } else if (key == OIT_K_BACKSPACE || key == OIT_K_DEL) { if (n) t->edit[n - 1] = 0; }
        else if (key == OIT_K_ENTER) commit_edit(t);
        else if (key == OIT_K_TAB) { int next = t->edit_field == F_TO ? F_FLT : t->edit_field + 1; commit_edit(t); begin_edit(t, next); }
        else if (key == OIT_K_ESC) t->edit_field = 0;
        return;
    }
    if (d->state != DS_SESSION) {
        if (d->state == DS_LOGIN && (key == OIT_K_ENTER || (key == OIT_K_CHAR && (ch == 'P' || ch == 'p')))) do_action(t, A_LOGIN, 0);
        return;
    }
    if (t->dropdown) {
        int n = t->dropdown == 1 ? DD_N(dd_menu) : DD_N(dd_functions);
        if (key == OIT_K_UP) t->dd_sel = (t->dd_sel + n - 1) % n;
        else if (key == OIT_K_DOWN) t->dd_sel = (t->dd_sel + 1) % n;
        else if (key == OIT_K_ENTER) do_action(t, A_DD_ITEM, t->dd_sel);
        else if (key == OIT_K_ESC) t->dropdown = 0;
        return;
    }
    switch (key) {
    case OIT_K_MENU: home(t); return;
    case OIT_K_ESC: back(t); return;
    case OIT_K_XFR: {
        OitTerminal *o = &terms[!t->side];
        if (!t->avncs && o->on && !o->avncs && o->dom[D_FLTOPS].state == DS_SESSION) t->x_ois = 1;
        else message(t, "X-OIS NOT AVAILABLE");
        return;
    }
    case OIT_K_UP: case OIT_K_LEFT: if (t->nfocus) t->focus = (t->focus + t->nfocus - 1) % t->nfocus; return;
    case OIT_K_DOWN: case OIT_K_RIGHT: case OIT_K_TAB: if (t->nfocus) t->focus = (t->focus + 1) % t->nfocus; return;
    case OIT_K_PGUP: do_action(t, A_SCROLL_UP, 0); return;
    case OIT_K_PGDN: do_action(t, A_SCROLL_DN, 0); return;
    case OIT_K_ENTER: {
        int i, k = 0;
        for (i = 0; i < t->nhits; ++i) if (t->hits[i].focusable && k++ == t->focus) { do_action(t, t->hits[i].action, t->hits[i].arg); return; }
        return;
    }
    default: return;
    }
}

void oit_scroll(OitTerminal *t, int clicks)
{
    int i;
    for (i = 0; i < (clicks < 0 ? -clicks : clicks); ++i) oit_key(t, clicks > 0 ? OIT_K_PGUP : OIT_K_PGDN, 0);
}

void oit_touch(OitTerminal *t, float x, float y)
{
    int i;
    if (!t->on || t->selftest_t < SELFTEST_S) return;
    t->dirty = 1;
    if (t->x_ois) { t->x_ois = 0; return; }
    if (t->edit_field) commit_edit(t);
    for (i = t->nhits - 1; i >= 0; --i) {
        Hit *h = &t->hits[i];
        if (x < h->x || y < h->y || x > h->x + h->w || y > h->y + h->h) continue;
        if (h->focusable) { int k, j = 0; for (k = 0; k < i; ++k) if (t->hits[k].focusable) ++j; t->focus = j; }
        do_action(t, h->action, h->arg);
        return;
    }
    t->dropdown = 0;
}

/* ---------------------------------------------------------------- widgets */
static int add_hit(OitTerminal *t, float x, float y, float w, float h, int action, int arg, int focusable)
{
    int focused = 0;
    if (t->nhits < (int)(sizeof(t->hits) / sizeof(t->hits[0]))) {
        Hit *e = &t->hits[t->nhits++];
        e->x = x; e->y = y; e->w = w; e->h = h; e->action = action; e->arg = arg; e->focusable = focusable;
    }
    if (focusable) focused = t->nfocus++ == t->focus;
    return focused;
}

static void bevel(float x, float y, float w, float h, int inset, OitColor fill)
{
    OitColor hi = inset ? C_SHADOW : C_BORDER, lo = inset ? C_BORDER : C_SHADOW;
    oit_rect(x, y, w, h, fill);
    oit_rect(x, y, w, 2, hi); oit_rect(x, y, 2, h, hi);
    oit_rect(x, y + h - 2, w, 2, lo); oit_rect(x + w - 2, y, 2, h, lo);
}

/* FBW OIT button: #272525, outset light-grey border, white label; cyan border when focused (hover); selected
 * buttons inset with the light fill; disabled labels grey. */
static void button(OitTerminal *t, float x, float y, float w, float h, const char *label, int enabled, int selected,
                   int action, int arg, int face)
{
    int focused = add_hit(t, x, y, w, h, enabled ? action : A_NONE, arg, enabled);
    bevel(x, y, w, h, selected, selected ? C_DISABLED : C_BTN);
    if (focused) oit_frame(x, y, w, h, 2, C_CYAN);
    oit_text(face, x + w / 2, y + h / 2 - oit_face_height(face) / 2.0f - 1, label,
             enabled ? (selected ? C_BG : C_WHITE) : C_DISABLED, OIT_CENTER);
}

static void field(OitTerminal *t, float x, float y, float w, int id, const char *value)
{
    int focused = add_hit(t, x, y, w, 40, A_EDIT, id, 1), editing = t->edit_field == id;
    char b[24];
    bevel(x, y, w, 40, 1, C_BG);
    if (focused || editing) oit_frame(x, y, w, 40, 2, C_CYAN);
    if (editing) { snprintf(b, sizeof(b), "%s_", t->edit); oit_text(OIT_F_MONO20, x + 10, y + 8, b, C_CYAN, OIT_LEFT); }
    else if (value[0]) oit_text(OIT_F_MONO20, x + 10, y + 8, value, C_CYAN, OIT_LEFT);
    else oit_text(OIT_F_MONO20, x + 10, y + 8, "[]", C_AMBER, OIT_LEFT);
}

static void kv(float x, float y, float colw, const char *label, const char *value, OitColor vc)
{
    oit_text(OIT_F_UI18, x, y + 2, label, C_WHITE, OIT_LEFT);
    oit_text(OIT_F_MONO20, x + colw, y, value, vc, OIT_LEFT);
}
static void heading(float x, float y, const char *t) { oit_text(OIT_F_BOLD20, x, y, t, C_CYAN, OIT_LEFT); }
static void frame_box(float x, float y, float w, float h) { oit_frame(x, y, w, h, 2, C_BORDER); }
static void check(float x, float y, int ok, const char *bad)
{
    if (ok) oit_text(OIT_F_BOLD18, x, y + 2, "\x85 OK", C_GREEN, OIT_LEFT);
    else oit_text(OIT_F_BOLD18, x, y + 2, bad, C_AMBER, OIT_LEFT);
}
static void progress(float cx, float y, float w, float frac)
{
    oit_frame(cx - w / 2, y, w, 22, 2, C_BORDER);
    oit_rect(cx - w / 2 + 4, y + 4, (w - 8) * (frac < 0 ? 0 : frac > 1 ? 1 : frac), 14, C_PROGRESS);
}
static void msg_box(OitTerminal *t, float x, float y, float w)
{
    char cnt[16];
    snprintf(cnt, sizeof(cnt), "%d MSG", t->msg_count);
    oit_text(OIT_F_UI18, x, y + 12, cnt, t->msg_count ? C_WHITE : C_DISABLED, OIT_LEFT);
    x += 70;
    bevel(x, y, w, 44, 0, C_BG);
    if (t->msg[0]) oit_text(OIT_F_UI16, x + 10, y + 13, t->msg, C_WHITE, OIT_LEFT);
    button(t, x + w + 4, y, 40, 44, "\x82", t->msg_count > 0, 0, A_NONE, 0, OIT_F_UI18);
    button(t, x + w + 48, y, 84, 44, "CLEAR", t->msg_count > 0, 0, A_CLEAR, 0, OIT_F_UI16);
}

/* ---------------------------------------------------------------- FLT OPS pages */
static void pg_fo_menu(OitTerminal *t, const OitSim *s)
{
    static const char *const col_title[4] = {"MISSION", "DOCUMENTATION", "PERFORMANCE", "UTILITIES"};
    static const struct { int col; const char *label; int page; } items[] = {
        {0, "FLT FOLDER", -1}, {0, "CHARTS", -1}, {1, "OPS LIBRARY", -1},
        {2, "T.O PERF", PG_TO}, {2, "LOADSHEET", PG_LOAD}, {2, "LDG PERF", PG_LDG}, {2, "IN-FLT PERF", PG_INFLT},
        {3, "FLT OPS STS", PG_STS}, {3, "LOAD BOX", -1}, {3, "EXPORT BOX", -1},
    };
    int i, row[4] = {0};
    (void)s;
    for (i = 0; i < 4; ++i) oit_text(OIT_F_BOLD24, 20 + i * 252 + 115, 96, col_title[i], C_WHITE, OIT_CENTER);
    for (i = 0; i < (int)(sizeof(items) / sizeof(items[0])); ++i) {
        int c = items[i].col;
        button(t, 20 + c * 252, 150 + row[c]++ * 64, 230, 50, items[i].label, items[i].page >= 0, 0, A_PAGE, items[i].page, OIT_F_BOLD18);
    }
    button(t, OIT_W - 250, 660, 230, 50, "EXIT SESSION", 1, 0, A_EXIT, 0, OIT_F_BOLD18);
}

static void pg_sts(OitTerminal *t, const OitSim *s)
{
    char b[64];
    float x = 40, y = 92, cw = 280;
    frame_box(20, 80, OIT_W - 40, 650);
    kv(x, y + 10, cw, "ACFT REGISTRATION", s->tail[0] ? s->tail : "----", C_GREEN);
    button(t, OIT_W - 290, y, 240, 44, "SYNCHRO AVIONICS", 1, 0, A_SYNCHRO, 0, OIT_F_BOLD18);
    kv(x, y + 64, cw, "ACFT TYPE", s->icao[0] ? s->icao : "A388", C_GREEN);
    kv(x, y + 110, cw, "OIS VERSION", OIS_VERSION, C_GREEN);
    fmt_dmy(b, sizeof(b), s->day_of_year, s->year);
    kv(x, y + 156, cw, "DATE", b, C_GREEN);
    fmt_latlon(b, sizeof(b), s->lat, s->lon);
    kv(x, y + 202, cw, "POSITION", b, C_GREEN);
    oit_rect(30, y + 260, OIT_W - 60, 2, C_BORDER);
    y += 290;
    oit_text(OIT_F_UI18, x, y + 10, "FLT NUMBER", C_WHITE, OIT_LEFT); field(t, x + cw, y, 220, F_FLT, ac.flt);
    oit_text(OIT_F_UI18, x, y + 70, "FROM", C_WHITE, OIT_LEFT); field(t, x + cw, y + 60, 140, F_FROM, ac.from);
    oit_text(OIT_F_UI18, x + cw + 170, y + 70, "TO", C_WHITE, OIT_LEFT); field(t, x + cw + 210, y + 60, 140, F_TO, ac.to);
    oit_text(OIT_F_UI16, x, y + 140, t->edit_field ? "TYPE ON THE KEYBOARD - ENTER TO CONFIRM, TAB NEXT FIELD, ESC CANCEL"
                                                   : "SELECT A FIELD AND PRESS ENTER (OR CLICK IT) TO MODIFY", C_DISABLED, OIT_LEFT);
    oit_text(OIT_F_UI16, x, y + 166, "SYNCHRO AVIONICS: FROM = NEAREST AIRPORT ON GROUND, TO = FMS DESTINATION", C_DISABLED, OIT_LEFT);
    if (s->fms_ok && s->crz_alt_ft > 0) { snprintf(b, sizeof(b), "FL%03.0f", s->crz_alt_ft / 100.0f); kv(x, y + 216, cw, "CRZ FL (FMS)", b, C_GREEN); }
}

static void pg_to(OitTerminal *t, const OitSim *s)
{
    char b[64];
    float x = 40, y = 92, cw = 230, c2 = 540;
    int entered = s->fms_ok && s->v1 > 0 && s->vr > 0 && s->v2 > 0;
    (void)t;
    frame_box(20, 80, OIT_W - 40, 336);
    heading(x, y, "T.O DATA - MFD PERF");
    if (s->to_flaps > 0) snprintf(b, sizeof(b), "%d", s->to_flaps); else snprintf(b, sizeof(b), "---");
    kv(x, y + 46, cw, "CONF", b, s->to_flaps > 0 ? C_GREEN : C_DISABLED);
    if (s->flex_c != 0) snprintf(b, sizeof(b), "FLEX %.0f\x80" "C", s->flex_c); else snprintf(b, sizeof(b), "TOGA");
    kv(x, y + 86, cw, "THRUST", b, C_GREEN);
    if (s->ths != 0) snprintf(b, sizeof(b), "%.1f", s->ths); else snprintf(b, sizeof(b), "---");
    kv(x, y + 126, cw, "THS FOR", b, s->ths != 0 ? C_GREEN : C_DISABLED);
    if (entered) {
        snprintf(b, sizeof(b), "%.0f KT", s->v1); kv(x + c2, y + 46, 80, "V1", b, C_GREEN);
        snprintf(b, sizeof(b), "%.0f KT", s->vr); kv(x + c2, y + 86, 80, "VR", b, C_GREEN);
        snprintf(b, sizeof(b), "%.0f KT", s->v2); kv(x + c2, y + 126, 80, "V2", b, C_GREEN);
    } else oit_text(OIT_F_BOLD18, x + c2, y + 48, "V SPEEDS NOT ENTERED ON MFD", C_AMBER, OIT_LEFT);
    oit_rect(30, y + 172, OIT_W - 60, 2, C_THEAD);
    heading(x, y + 186, "CHECKS");
    oit_text(OIT_F_UI18, x, y + 228, "V1 / VR / V2 ORDER", C_WHITE, OIT_LEFT);
    if (entered) check(x + 340, y + 226, s->v1 <= s->vr && s->vr <= s->v2, "SPEEDS OUT OF ORDER");
    else oit_text(OIT_F_UI18, x + 340, y + 228, "---", C_DISABLED, OIT_LEFT);
    oit_text(OIT_F_UI18, x, y + 262, "FLEX TEMP ABOVE OAT", C_WHITE, OIT_LEFT);
    if (s->flex_c != 0) check(x + 340, y + 260, s->flex_c > s->oat_c, "FLEX TEMP BELOW OAT");
    else oit_text(OIT_F_UI18, x + 340, y + 262, "TOGA", C_DISABLED, OIT_LEFT);
    oit_text(OIT_F_UI18, x, y + 296, "FMS GW / ACFT GW", C_WHITE, OIT_LEFT);
    if (s->fms_gw_kg > 1000) check(x + 340, y + 294, fabsf(s->fms_gw_kg - s->gw_kg) < 2000, "DIFFERENCE ABOVE 2 T");
    else oit_text(OIT_F_UI18, x + 340, y + 296, "---", C_DISABLED, OIT_LEFT);

    frame_box(20, 432, OIT_W - 40, 236);
    y = 444;
    heading(x, y, "CONDITIONS");
    if (ac.from[0]) kv(x, y + 46, cw, "AIRPORT", ac.from, C_GREEN); else kv(x, y + 46, cw, "AIRPORT", "[] SEE FLT OPS STS", C_AMBER);
    snprintf(b, sizeof(b), "%.0f\x80" "C", s->oat_c); kv(x, y + 86, cw, "OAT", b, C_GREEN);
    snprintf(b, sizeof(b), "%03.0f/%.0f KT", s->wind_dir, s->wind_kt); kv(x, y + 126, cw, "WIND", b, C_GREEN);
    fmt_t(b, sizeof(b), s->gw_kg); kv(x + c2, y + 46, 120, "TOW", b, C_GREEN);
    snprintf(b, sizeof(b), "%.0f FT", s->alt_ft); kv(x + c2, y + 86, 120, "ELEV", b, C_GREEN);
    snprintf(b, sizeof(b), "%03.0f\x80", s->hdg_mag); kv(x + c2, y + 126, 120, "HDG", b, C_GREEN);
    oit_text(OIT_F_UI16, x, y + 184, "No performance database in this port: compute T.O data on the MFD and cross-check here.", C_DISABLED, OIT_LEFT);
}

static void pg_load(OitTerminal *t, const OitSim *s)
{
    char b[64];
    float x = 40, y = 92, cw = 260, c2 = 560;
    float zfw = s->gw_kg - s->fuel_kg, mtow = s->mtow_kg > 100000 ? s->mtow_kg : 575000.0f;
    int fms = s->fms_ok && s->zfw_kg > 1000;
    (void)t;
    frame_box(20, 80, OIT_W - 40, 300);
    heading(x, y, "WEIGHTS");
    oit_text(OIT_F_BOLD16, x + cw, y + 34, "FMS", C_DISABLED, OIT_LEFT); oit_text(OIT_F_BOLD16, x + c2, y + 34, "ACFT", C_DISABLED, OIT_LEFT);
    oit_text(OIT_F_UI18, x, y + 64, "ZFW", C_WHITE, OIT_LEFT);
    if (fms) { fmt_t(b, sizeof(b), s->zfw_kg); oit_text(OIT_F_MONO20, x + cw, y + 62, b, C_GREEN, OIT_LEFT); }
    else oit_text(OIT_F_MONO20, x + cw, y + 62, "[]", C_AMBER, OIT_LEFT);
    fmt_t(b, sizeof(b), zfw); oit_text(OIT_F_MONO20, x + c2, y + 62, b, C_GREEN, OIT_LEFT);
    oit_text(OIT_F_UI18, x, y + 104, "ZFWCG", C_WHITE, OIT_LEFT);
    if (fms) { snprintf(b, sizeof(b), "%.1f %%", s->zfw_cg); oit_text(OIT_F_MONO20, x + cw, y + 102, b, C_GREEN, OIT_LEFT); }
    else oit_text(OIT_F_MONO20, x + cw, y + 102, "[]", C_AMBER, OIT_LEFT);
    oit_text(OIT_F_UI18, x, y + 144, "FUEL ON BOARD", C_WHITE, OIT_LEFT);
    fmt_t(b, sizeof(b), s->fuel_kg); oit_text(OIT_F_MONO20, x + c2, y + 142, b, C_GREEN, OIT_LEFT);
    oit_text(OIT_F_UI18, x, y + 184, "GROSS WEIGHT", C_WHITE, OIT_LEFT);
    if (s->fms_gw_kg > 1000) { fmt_t(b, sizeof(b), s->fms_gw_kg); oit_text(OIT_F_MONO20, x + cw, y + 182, b, C_GREEN, OIT_LEFT); }
    fmt_t(b, sizeof(b), s->gw_kg); oit_text(OIT_F_MONO20, x + c2, y + 182, b, C_GREEN, OIT_LEFT);
    oit_text(OIT_F_UI18, x, y + 224, "PAYLOAD / EMPTY WEIGHT", C_WHITE, OIT_LEFT);
    snprintf(b, sizeof(b), "%.1f / %.1f T", s->payload_kg / 1000, s->empty_kg / 1000);
    oit_text(OIT_F_MONO20, x + c2, y + 222, b, C_GREEN, OIT_LEFT);

    frame_box(20, 396, OIT_W - 40, 190);
    y = 408;
    heading(x, y, "LIMITATIONS");
    fmt_t(b, sizeof(b), mtow); kv(x, y + 44, cw, "MTOW", b, C_WHITE); check(x + c2, y + 44, s->gw_kg <= mtow, "GW ABOVE MTOW");
    fmt_t(b, sizeof(b), MLW_KG); kv(x, y + 84, cw, "MLW", b, C_WHITE); check(x + c2, y + 84, s->on_ground || s->gw_kg <= MLW_KG, "GW ABOVE MLW");
    fmt_t(b, sizeof(b), MZFW_KG); kv(x, y + 124, cw, "MZFW", b, C_WHITE); check(x + c2, y + 124, zfw <= MZFW_KG, "ZFW ABOVE MZFW");

    frame_box(20, 602, OIT_W - 40, 110);
    y = 614;
    heading(x, y, "CROSS-CHECK");
    if (fms) {
        float d = s->zfw_kg - zfw;
        snprintf(b, sizeof(b), "%+.1f T", d / 1000);
        kv(x, y + 44, cw, "FMS ZFW - ACFT ZFW", b, fabsf(d) <= 1000 ? C_GREEN : C_AMBER);
        if (fabsf(d) > 1000) oit_text(OIT_F_BOLD18, x + c2, y + 46, "CHECK ZFW ON MFD FUEL&LOAD", C_AMBER, OIT_LEFT);
        else check(x + c2, y + 44, 1, "");
    } else oit_text(OIT_F_BOLD18, x, y + 46, "INSERT ZFW / ZFWCG ON THE MFD FUEL&LOAD PAGE", C_AMBER, OIT_LEFT);
}

static void pg_ldg(OitTerminal *t, const OitSim *s)
{
    char b[64];
    float x = 40, y = 92, cw = 230, c2 = 540;
    (void)t;
#define SPD(v) (snprintf(b, sizeof(b), (v) > 0 ? "%.0f KT" : "---", (v)), b)
    frame_box(20, 80, OIT_W - 40, 330);
    heading(x, y, "APPROACH - MFD PERF");
    kv(x, y + 46, cw, "LDG CONF", s->fms_ok ? (s->ldg_conf3 ? "CONF 3" : "CONF FULL") : "---", C_GREEN);
    kv(x, y + 86, cw, "VAPP", SPD(s->vapp), s->vapp > 0 ? C_GREEN : C_DISABLED);
    kv(x, y + 126, cw, "VLS", SPD(s->vls), s->vls > 0 ? C_GREEN : C_DISABLED);
    kv(x, y + 166, cw, "VREF", SPD(s->vref), s->vref > 0 ? C_GREEN : C_DISABLED);
    kv(x + c2, y + 46, 150, "F", SPD(s->app_f), s->app_f > 0 ? C_GREEN : C_DISABLED);
    kv(x + c2, y + 86, 150, "S", SPD(s->app_s), s->app_s > 0 ? C_GREEN : C_DISABLED);
    kv(x + c2, y + 126, 150, "GREEN DOT", SPD(s->app_gd), s->app_gd > 0 ? C_GREEN : C_DISABLED);
    snprintf(b, sizeof(b), "%+.0f KT", s->app_headwind); kv(x + c2, y + 166, 150, "HEADWIND", b, C_GREEN);
#undef SPD
    if (s->dh_ft > 0) snprintf(b, sizeof(b), "DH %.0f FT", s->dh_ft);
    else if (s->mda_ft > 0) snprintf(b, sizeof(b), "MDA %.0f FT", s->mda_ft);
    else snprintf(b, sizeof(b), "---");
    kv(x, y + 206, cw, "MINIMUM", b, (s->dh_ft > 0 || s->mda_ft > 0) ? C_GREEN : C_DISABLED);
    if (s->ldg_elev_ft != 0) snprintf(b, sizeof(b), "%.0f FT", s->ldg_elev_ft); else snprintf(b, sizeof(b), "---");
    kv(x, y + 246, cw, "LDG ELEV", b, s->ldg_elev_ft != 0 ? C_GREEN : C_DISABLED);
    if (s->trans_alt_ft > 0) snprintf(b, sizeof(b), "%.0f FT", s->trans_alt_ft); else snprintf(b, sizeof(b), "---");
    kv(x + c2, y + 206, 150, "TRANS ALT", b, s->trans_alt_ft > 0 ? C_GREEN : C_DISABLED);
    kv(x + c2, y + 246, 150, "DEST", ac.to[0] ? ac.to : "----", ac.to[0] ? C_GREEN : C_DISABLED);

    frame_box(20, 426, OIT_W - 40, 180);
    y = 438;
    heading(x, y, "LANDING WEIGHT");
    fmt_t(b, sizeof(b), s->gw_kg); kv(x, y + 46, cw, "CURRENT GW", b, C_GREEN);
    fmt_t(b, sizeof(b), MLW_KG); kv(x, y + 86, cw, "MLW", b, C_WHITE);
    check(x + c2, y + 46, s->gw_kg <= MLW_KG, "ABOVE MLW: OVERWEIGHT LDG");
    oit_text(OIT_F_UI16, x, y + 136, "No performance database in this port: landing distance not computed.", C_DISABLED, OIT_LEFT);
}

static void pg_inflt(OitTerminal *t, const OitSim *s)
{
    static const float ratio[9] = {0.300f, 0.078f, 0.220f, 0.220f, 0.078f, 0.023f, 0.023f, 0.058f, 0.0f};
    char b[64];
    float x = 40, y = 92, ff = 0;
    int i, n = s->fuel_tanks > 0 && s->fuel_tanks <= 9 ? s->fuel_tanks : 9;
    (void)t;
    for (i = 0; i < 4; ++i) ff += s->fuel_flow_kgh[i];
    frame_box(20, 80, OIT_W - 40, 400);
    heading(x, y, "FUEL ON BOARD");
    for (i = 0; i < n; ++i) {
        float cap = ratio[i] * 255500.0f, kg = s->fuel_tank_kg[i], f, ry = y + 44 + i * 40;
        if (cap <= 0) continue;
        f = kg / cap; f = f < 0 ? 0 : f > 1 ? 1 : f;
        oit_textf(OIT_F_UI18, x, ry + 2, C_WHITE, OIT_LEFT, "TANK %d", i + 1);
        oit_frame(x + 110, ry + 4, 520, 22, 2, C_BORDER);
        oit_rect(x + 114, ry + 8, 512 * f, 14, f < 0.05f ? C_AMBER : C_GREEN);
        snprintf(b, sizeof(b), "%6.1f / %.0f T", kg / 1000, cap / 1000);
        oit_text(OIT_F_MONO20, x + 660, ry, b, C_GREEN, OIT_LEFT);
    }
    frame_box(20, 496, OIT_W - 40, 210);
    y = 508;
    heading(x, y, "ENDURANCE");
    fmt_t(b, sizeof(b), s->fuel_kg); kv(x, y + 44, 260, "FOB", b, C_GREEN);
    snprintf(b, sizeof(b), "%.0f KG/H", ff); kv(x, y + 84, 260, "TOTAL FUEL FLOW", b, C_GREEN);
    if (ff > 100) { int m = (int)(s->fuel_kg / ff * 60); snprintf(b, sizeof(b), "%d:%02d", m / 60, m % 60); } else snprintf(b, sizeof(b), "---");
    kv(x, y + 124, 260, "ENDURANCE", b, ff > 100 ? C_GREEN : C_DISABLED);
    if (s->gw_kg > MLW_KG && ff > 100) {
        int m = (int)((s->gw_kg - MLW_KG) / ff * 60);
        snprintf(b, sizeof(b), "%d:%02d", m / 60, m % 60);
        kv(x + 520, y + 44, 190, "TIME TO MLW", b, C_AMBER);
    }
    if (ac.cur.out.t != 0) { snprintf(b, sizeof(b), "%.1f T", (ac.cur.out.fob - s->fuel_kg) / 1000); kv(x + 520, y + 84, 190, "USED SINCE OUT", b, C_GREEN); }
}

/* ---------------------------------------------------------------- NSS AVNCS pages */
static void tree_item(OitTerminal *t, float x, float y, int depth, const char *label, int folder, int enabled, int action, int arg, int selected)
{
    float ix = x + depth * 22, w = 290 - depth * 22 - 12;
    int clickable = enabled && action != A_NONE;
    int focused = add_hit(t, ix, y, w, 30, clickable ? action : A_NONE, arg, clickable);
    if (selected) oit_rect(ix, y, w, 30, C_BORDER);
    if (focused) oit_frame(ix, y, w, 30, 2, C_CYAN);
    if (folder) {
        oit_text(OIT_F_MONO16, ix + 2, y + 5, "-", C_WHITE, OIT_LEFT);
        oit_rect(ix + 16, y + 6, 8, 4, oit_rgb(0xd8b04a));
        oit_rect(ix + 16, y + 9, 18, 13, oit_rgb(0xd8b04a));
    }
    oit_text(OIT_F_UI16, ix + (folder ? 42 : 12), y + 6, label, enabled ? (selected ? C_BG : C_WHITE) : C_DISABLED, OIT_LEFT);
}

static void pg_av_menu(OitTerminal *t, const OitSim *s)
{
    float x = 16, y = 128;
    (void)s;
    oit_frame(10, 116, 300, 590, 3, C_BORDER);
    tree_item(t, x, y, 0, "OIS MENU", 1, 1, A_NONE, 0, 0);
    tree_item(t, x, y + 34, 1, "NSS AVNCS MENU", 1, 1, A_NONE, 0, 0);
    tree_item(t, x, y + 68, 2, "COMPANY COM", 0, 1, A_PAGE, PG_CCOM, 0);
    tree_item(t, x, y + 102, 2, "REFUEL", 0, 0, A_NONE, 0, 0);
    tree_item(t, x, y + 136, 1, "UTILITIES", 1, 1, A_NONE, 0, 0);
    tree_item(t, x, y + 170, 2, "A380X SYSTEMS", 1, 1, A_PAGE, PG_SYSTEMS, 0);
    tree_item(t, x, y + 204, 3, "CURRENT FAULTS", 0, 1, A_PAGE, PG_FAULTS, 0);
    tree_item(t, x, y + 238, 3, "DOORS", 0, 1, A_PAGE, PG_DOORS, 0);
    tree_item(t, x, y + 272, 3, "ENGINES", 0, 1, A_PAGE, PG_ENGINES, 0);
    button(t, OIT_W - 190, 124, 170, 50, "EXIT OIS", 1, 0, A_EXIT, 0, OIT_F_BOLD18);
    button(t, 520, 300, 300, 56, "COMPANY COM", 1, 0, A_PAGE, PG_CCOM, OIT_F_BOLD20);
    button(t, 520, 376, 300, 56, "REFUEL", 0, 0, A_NONE, 0, OIT_F_BOLD20);
}

static void sub_header(OitTerminal *t, const char *title)
{
    static const char *const names[7] = {"HOME", "BACK", "FORWARD", "PRINT", "UPDATE", "HELP", "CLOSE"};
    static const int acts[7] = {A_HOME, A_BACK, A_FWD, A_NONE, A_NONE, A_NONE, A_HOME};
    int i;
    oit_text(OIT_F_BOLD24, 14, 124, title, C_CYAN, OIT_LEFT);
    for (i = 0; i < 7; ++i) button(t, 296 + i * 104, 116, 100, 42, names[i], acts[i] != A_NONE, 0, acts[i], 0, OIT_F_UI16);
    oit_rect(10, 166, OIT_W - 20, 2, C_BORDER);
}

static void pg_ccom(OitTerminal *t, const OitSim *s)
{
    char b[64];
    int i, unread = 0;
    float x = 330, y = 180;
    for (i = 0; i < ac.inbox_n; ++i) if (!ac.inbox[i].read) ++unread;
    sub_header(t, "COMPANY COM");
    oit_frame(10, 176, 300, 530, 3, C_BORDER);
    snprintf(b, sizeof(b), "Inbox (%d)", unread);
    tree_item(t, 16, 188, 0, b, 0, 1, A_CCOM_VIEW, 0, t->ccom_view == 0);
    tree_item(t, 16, 222, 0, "Outbox", 0, 0, A_NONE, 0, 0);
    tree_item(t, 16, 256, 0, "Sent", 0, 0, A_NONE, 0, 0);
    tree_item(t, 16, 290, 0, "Pre-flight", 1, 1, A_NONE, 0, 0);
    tree_item(t, 16, 324, 1, "INIT Data", 0, 0, A_NONE, 0, 0);
    tree_item(t, 16, 358, 1, "Flight Log", 0, 1, A_CCOM_VIEW, 1, t->ccom_view == 1);
    tree_item(t, 16, 392, 0, "In-flight", 1, 1, A_NONE, 0, 0);
    tree_item(t, 16, 426, 1, "Flight Log", 0, 1, A_CCOM_VIEW, 1, 0);
    tree_item(t, 16, 460, 0, "Post-flight", 1, 1, A_NONE, 0, 0);
    tree_item(t, 16, 494, 1, "Flight Log", 0, 1, A_CCOM_VIEW, 1, 0);
    if (t->ccom_view == 0) {
        int first, rows = 6;
        oit_rect(x, y, OIT_W - x - 10, 34, C_THEAD);
        oit_text(OIT_F_BOLD16, x + 50, y + 8, "Subject", C_WHITE, OIT_LEFT);
        oit_text(OIT_F_BOLD16, OIT_W - 180, y + 8, "Date", C_WHITE, OIT_LEFT);
        if (!ac.inbox_n) oit_text(OIT_F_UI18, x + 10, y + 50, "No message.", C_DISABLED, OIT_LEFT);
        if (t->scroll > (ac.inbox_n > rows ? ac.inbox_n - rows : 0)) t->scroll = ac.inbox_n > rows ? ac.inbox_n - rows : 0;
        first = t->scroll;
        for (i = 0; i < rows && first + i < ac.inbox_n; ++i) {
            int k = ac.inbox_n - 1 - (first + i);    /* newest first */
            const Message *m = &ac.inbox[k];
            float ry = y + 38 + i * 42;
            int sel = t->inbox_sel == k, focused = add_hit(t, x, ry, OIT_W - x - 10, 40, A_INBOX_ROW, k, 1);
            OitColor tc = sel ? C_BG : C_GREEN;
            oit_rect(x, ry, OIT_W - x - 10, 40, sel ? C_WHITE : C_ROW);
            if (focused) oit_frame(x, ry, OIT_W - x - 10, 40, 2, C_CYAN);
            oit_text(OIT_F_BOLD18, x + 16, ry + 9, m->read ? "\x89" : "\x86", tc, OIT_LEFT);
            oit_text(OIT_F_UI18, x + 50, ry + 9, m->subject, tc, OIT_LEFT);
            fmt_dd_hhmm(b, sizeof(b), m->doy, s->year, m->t);
            oit_text(OIT_F_MONO18, OIT_W - 180, ry + 9, b, tc, OIT_LEFT);
        }
        if (t->inbox_sel >= 0 && t->inbox_sel < ac.inbox_n) {
            const Message *m = &ac.inbox[t->inbox_sel];
            float dy = y + 300;
            oit_text(OIT_F_UI16, x, dy, m->read ? "Message is read" : "Message is unread", C_DISABLED, OIT_LEFT);
            snprintf(b, sizeof(b), "To : %s", ac.flt[0] ? ac.flt : s->tail);
            oit_text(OIT_F_UI18, x, dy + 26, b, C_WHITE, OIT_LEFT);
            snprintf(b, sizeof(b), "Subject : %s", m->subject);
            oit_text(OIT_F_UI18, x, dy + 54, b, C_WHITE, OIT_LEFT);
            bevel(x, dy + 88, OIT_W - x - 10, 120, 1, C_BG);
            oit_text(OIT_F_MONO18, x + 12, dy + 100, m->body, C_GREEN, OIT_LEFT);
        }
    } else {
        static const char *const state[4] = {"OUT", "OFF", "ON", "IN"};
        const Oooi *o[4] = {&ac.cur.out, &ac.cur.off, &ac.cur.on, &ac.cur.in};
        frame_box(x, y, OIT_W - x - 10, 160);
        oit_text(OIT_F_BOLD18, x + 12, y + 10, "Flight", C_WHITE, OIT_LEFT);
        oit_frame(x + 14, y + 44, 18, 18, 2, C_BORDER); oit_rect(x + 19, y + 49, 8, 8, C_CYAN);
        oit_text(OIT_F_UI18, x + 42, y + 42, "Current FLT", C_WHITE, OIT_LEFT);
        kv(x + 14, y + 78, 120, "From", ac.from[0] ? ac.from : "----", C_GREEN);
        kv(x + 250, y + 78, 60, "To", ac.to[0] ? ac.to : "----", C_GREEN);
        kv(x + 14, y + 114, 190, "Flight Identifier", ac.flt[0] ? ac.flt : "----", C_MAGENTA);
        fmt_dmy(b, sizeof(b), s->day_of_year, s->year); kv(x + 400, y + 114, 70, "Date", b, C_GREEN);
        y += 176;
        oit_rect(x, y, OIT_W - x - 10, 34, C_THEAD);
        oit_text(OIT_F_BOLD16, x + 14, y + 8, "State", C_WHITE, OIT_LEFT);
        oit_text(OIT_F_BOLD16, x + 160, y + 8, "Time (UTC)", C_WHITE, OIT_LEFT);
        oit_text(OIT_F_BOLD16, x + 400, y + 8, "FOB", C_WHITE, OIT_LEFT);
        for (i = 0; i < 4; ++i) {
            float ry = y + 38 + i * 40;
            oit_rect(x, ry, OIT_W - x - 10, 38, C_ROW);
            oit_text(OIT_F_BOLD18, x + 14, ry + 9, state[i], C_WHITE, OIT_LEFT);
            if (o[i]->t != 0) {
                fmt_dd_hhmm(b, sizeof(b), ac.cur.doy ? ac.cur.doy : s->day_of_year, s->year, o[i]->t);
                oit_text(OIT_F_MONO18, x + 160, ry + 9, b, C_GREEN, OIT_LEFT);
                fmt_t(b, sizeof(b), o[i]->fob); oit_text(OIT_F_MONO18, x + 400, ry + 9, b, C_GREEN, OIT_LEFT);
            } else {
                oit_text(OIT_F_MONO18, x + 160, ry + 9, "--", C_DISABLED, OIT_LEFT);
                oit_text(OIT_F_MONO18, x + 400, ry + 9, "--", C_DISABLED, OIT_LEFT);
            }
        }
        y += 210;
        kv(x, y, 190, "Current State", flight_state(), C_GREEN);
        if (ac.cur.off.t != 0) fmt_dur(b, sizeof(b), ac.cur.off.t, ac.cur.on.t != 0 ? ac.cur.on.t : s->zulu_s); else snprintf(b, sizeof(b), "--:--");
        kv(x, y + 36, 190, "Flight Time", b, C_GREEN);
        if (ac.cur.out.t != 0) fmt_dur(b, sizeof(b), ac.cur.out.t, ac.cur.in.t != 0 ? ac.cur.in.t : s->zulu_s); else snprintf(b, sizeof(b), "--:--");
        kv(x, y + 72, 190, "Block Time", b, C_GREEN);
    }
}

static void pg_systems(OitTerminal *t, const OitSim *s)
{
    int n = 0, i;
    char b[48];
    for (i = 0; i < oit_fault_count; ++i) n += s->fault[i] != 0;
    sub_header(t, "A380X SYSTEMS");
    button(t, 330, 220, 360, 56, "CURRENT FAULTS", 1, 0, A_PAGE, PG_FAULTS, OIT_F_BOLD20);
    if (n) snprintf(b, sizeof(b), "%d FAULT%s", n, n == 1 ? "" : "S"); else snprintf(b, sizeof(b), "NO FAULT");
    oit_text(OIT_F_BOLD18, 710, 236, b, n ? C_AMBER : C_GREEN, OIT_LEFT);
    button(t, 330, 296, 360, 56, "DOORS", 1, 0, A_PAGE, PG_DOORS, OIT_F_BOLD20);
    if (s->doors_open) snprintf(b, sizeof(b), "%d NOT CLOSED", s->doors_open); else snprintf(b, sizeof(b), "ALL CLOSED");
    oit_text(OIT_F_BOLD18, 710, 312, b, s->doors_open ? C_AMBER : C_GREEN, OIT_LEFT);
    button(t, 330, 372, 360, 56, "ENGINES", 1, 0, A_PAGE, PG_ENGINES, OIT_F_BOLD20);
}

static void pg_faults(OitTerminal *t, const OitSim *s)
{
    int idx[64], n = 0, i, rows = 11, first;
    float x = 20, y = 180;
    for (i = 0; i < oit_fault_count && i < 64; ++i) if (s->fault[i]) idx[n++] = i;
    sub_header(t, "CURRENT FAULTS");
    oit_rect(x, y, OIT_W - 40, 34, C_THEAD);
    oit_text(OIT_F_BOLD16, x + 14, y + 8, "ATA", C_WHITE, OIT_LEFT);
    oit_text(OIT_F_BOLD16, x + 100, y + 8, "Fault", C_WHITE, OIT_LEFT);
    oit_text(OIT_F_BOLD16, OIT_W - 40, y + 8, "Class", C_WHITE, OIT_RIGHT);
    if (!n) { oit_text(OIT_F_BOLD24, OIT_W / 2.0f, 380, "NO CURRENT FAULT", C_GREEN, OIT_CENTER); t->scroll = 0; return; }
    if (t->scroll > (n > rows ? n - rows : 0)) t->scroll = n > rows ? n - rows : 0;
    first = t->scroll;
    for (i = first; i < n && i < first + rows; ++i) {
        const OitFault *f = &oit_faults[idx[i]];
        float ry = y + 38 + (i - first) * 42;
        int warn = f->ata == 26;
        oit_rect(x, ry, OIT_W - 40, 40, C_ROW);
        oit_textf(OIT_F_MONO20, x + 14, ry + 8, C_WHITE, OIT_LEFT, "%02d", f->ata);
        oit_text(OIT_F_BOLD18, x + 100, ry + 10, f->text, warn ? C_RED : C_AMBER, OIT_LEFT);
        oit_text(OIT_F_MONO18, OIT_W - 40, ry + 10, warn ? "WARNING" : "FAULT", warn ? C_RED : C_AMBER, OIT_RIGHT);
    }
    if (n > rows) {
        button(t, OIT_W - 200, 690, 80, 40, "\x81", 1, 0, A_SCROLL_UP, 0, OIT_F_UI18);
        button(t, OIT_W - 110, 690, 80, 40, "\x82", 1, 0, A_SCROLL_DN, 0, OIT_F_UI18);
    }
}

static void pg_doors(OitTerminal *t, const OitSim *s)
{
    int i;
    float x = 30, y = 186;
    sub_header(t, "DOORS");
    heading(x, y, "PASSENGER DOORS");
    for (i = 0; i < 16; ++i) {
        float r = s->door_ratio[i], dx = x + (i % 2) * 480, dy = y + 40 + (i / 2) * 40;
        char b[24];
        oit_text(OIT_F_MONO20, dx, dy, door_names[i], C_WHITE, OIT_LEFT);
        if (r < 0.02f) oit_text(OIT_F_BOLD18, dx + 110, dy + 2, "CLOSED", C_GREEN, OIT_LEFT);
        else {
            if (r > 0.98f) snprintf(b, sizeof(b), "OPEN"); else snprintf(b, sizeof(b), "MOVING %d %%", (int)(r * 100));
            oit_text(OIT_F_BOLD18, dx + 110, dy + 2, b, C_AMBER, OIT_LEFT);
        }
    }
    y += 40 + 8 * 40 + 14;
    heading(x, y, "CARGO DOORS");
    for (i = 16; i < 19; ++i) {
        float r = s->door_ratio[i], dy = y + 40 + (i - 16) * 36;
        oit_text(OIT_F_MONO20, x, dy, door_names[i], C_WHITE, OIT_LEFT);
        oit_text(OIT_F_BOLD18, x + 200, dy + 2, r < 0.02f ? "CLOSED" : r > 0.98f ? "OPEN" : "MOVING", r < 0.02f ? C_GREEN : C_AMBER, OIT_LEFT);
    }
}

static void pg_engines(OitTerminal *t, const OitSim *s)
{
    int i;
    float x = 20, y = 186;
    sub_header(t, "ENGINES");
    oit_rect(x, y, OIT_W - 40, 34, C_THEAD);
    oit_text(OIT_F_BOLD16, x + 14, y + 8, "Engine", C_WHITE, OIT_LEFT);
    oit_text(OIT_F_BOLD16, x + 200, y + 8, "Status", C_WHITE, OIT_LEFT);
    oit_text(OIT_F_BOLD16, x + 440, y + 8, "N1 %", C_WHITE, OIT_LEFT);
    oit_text(OIT_F_BOLD16, x + 640, y + 8, "Fuel flow KG/H", C_WHITE, OIT_LEFT);
    for (i = 0; i < 4; ++i) {
        float ry = y + 38 + i * 44;
        char b[24];
        oit_rect(x, ry, OIT_W - 40, 42, C_ROW);
        oit_textf(OIT_F_BOLD18, x + 14, ry + 10, C_WHITE, OIT_LEFT, "ENG %d", i + 1);
        oit_text(OIT_F_BOLD18, x + 200, ry + 10, s->eng_running[i] ? "RUNNING" : "SHUT DOWN", s->eng_running[i] ? C_GREEN : C_DISABLED, OIT_LEFT);
        snprintf(b, sizeof(b), "%5.1f", s->n1[i]); oit_text(OIT_F_MONO20, x + 440, ry + 9, b, C_GREEN, OIT_LEFT);
        snprintf(b, sizeof(b), "%7.0f", s->fuel_flow_kgh[i]); oit_text(OIT_F_MONO20, x + 640, ry + 9, b, C_GREEN, OIT_LEFT);
    }
}

/* ---------------------------------------------------------------- frames */
static void fo_header(OitTerminal *t)
{
    Domain *d = dom(t);
    button(t, 10, 10, 136, 44, "MENU \x82", 1, t->dropdown == 1, A_DROP, 1, OIT_F_BOLD18);
    oit_text(OIT_F_BOLD24, 270, 18, page_title[d->page], C_CYAN, OIT_CENTER);
    button(t, 396, 10, 166, 44, "FUNCTIONS \x82", 1, t->dropdown == 2, A_DROP, 2, OIT_F_BOLD18);
    msg_box(t, 572, 10, 230);
}

static void fo_footer(OitTerminal *t)
{
    static const struct { const char *label; int page; } apps[4] = {
        {"FLT OPS STS", PG_STS}, {"T.O PERF", PG_TO}, {"LOADSHEET", PG_LOAD}, {"LDG PERF", PG_LDG}};
    Domain *d = dom(t);
    int i;
    if (t->hide_bar) return;
    oit_rect(10, 742, OIT_W - 20, 2, C_BORDER);
    button(t, 10, 750, 44, 42, "\x83", d->nback > 0, 0, A_BACK, 0, OIT_F_UI18);
    for (i = 0; i < 4; ++i) button(t, 62 + i * 228, 750, 220, 42, apps[i].label, 1, d->page == apps[i].page, A_PAGE, apps[i].page, OIT_F_BOLD16);
    button(t, OIT_W - 54, 750, 44, 42, "\x84", d->nfwd > 0, 0, A_FWD, 0, OIT_F_UI18);
}

static void fo_dropdown(OitTerminal *t)
{
    const DdItem *items = t->dropdown == 1 ? dd_menu : dd_functions;
    int n = t->dropdown == 1 ? DD_N(dd_menu) : DD_N(dd_functions), i;
    float x = t->dropdown == 1 ? 10 : 396, y = 56, w = 280, ih = 38, h = 0;
    for (i = 0; i < n; ++i) h += ih + (items[i].sep_before ? 6 : 0);
    bevel(x, y, w, h + 8, 0, C_BTN);
    y += 4;
    for (i = 0; i < n; ++i) {
        if (items[i].sep_before) { oit_rect(x + 8, y + 2, w - 16, 1, C_SHADOW); y += 6; }
        if (i == t->dd_sel) oit_rect(x + 4, y, w - 8, ih, C_DISABLED);
        oit_text(OIT_F_UI18, x + 16, y + 9, items[i].label, items[i].enabled ? (i == t->dd_sel ? C_BG : C_WHITE) : C_DISABLED, OIT_LEFT);
        add_hit(t, x, y, w, ih, A_DD_ITEM, i, 0);
        y += ih;
    }
}

static void av_header(OitTerminal *t)
{
    Domain *d = dom(t);
    button(t, 10, 10, 210, 44, "NSS AVNCS MENU", 1, d->page == PG_AV_MENU, A_HOME, 0, OIT_F_BOLD18);
    button(t, 226, 10, 140, 44, "MAXIMIZE", 0, 0, A_NONE, 0, OIT_F_BOLD18);
    msg_box(t, 378, 10, 260);
    oit_text(OIT_F_BOLD20, OIT_W - 14, 20, ac.flt, C_MAGENTA, OIT_RIGHT);
    oit_rect(10, 62, OIT_W - 20, 2, C_BORDER);
}

static void av_footer(OitTerminal *t)
{
    oit_rect(10, 742, OIT_W - 20, 2, C_BORDER);
    button(t, 10, 750, 220, 42, "COMPANY COM", 1, dom(t)->page == PG_CCOM, A_PAGE, PG_CCOM, OIT_F_BOLD16);
}

static void loading_screen(OitTerminal *t, const OitSim *s, int domain)
{
    Domain *d = &t->dom[domain];
    float total = domain == D_FLTOPS ? (float)FLTOPS_LOAD_S : (float)AVNCS_LOAD_S;
    if (domain == D_FLTOPS && !s->laptop_on) {
        oit_text(OIT_F_BOLD30, OIT_W / 2.0f, OIT_H / 2.0f - 30, "Not available.", C_AMBER, OIT_CENTER);
        oit_text(OIT_F_UI18, OIT_W / 2.0f, OIT_H / 2.0f + 24, t->side == OIT_CAPT ? "CAPT LAPTOP OFF" : "F/O LAPTOP OFF", C_DISABLED, OIT_CENTER);
        return;
    }
    if (domain == D_AVNCS) oit_text(OIT_F_UI20, 20, 20, "The system is loading, please wait...", C_WHITE, OIT_LEFT);
    oit_text(OIT_F_BOLD40, OIT_W / 2.0f, 280, "A380", C_WHITE, OIT_CENTER);
    oit_text(OIT_F_UI20, OIT_W / 2.0f, 340, domain == D_FLTOPS ? "FLT OPS" : "NSS AVNCS", C_DISABLED, OIT_CENTER);
    progress(OIT_W / 2.0f, 460, OIT_W * 0.4f, (float)(d->load_t / total));
}

static void login_screen(OitTerminal *t, int domain)
{
    if (domain == D_FLTOPS) {
        oit_text(OIT_F_BOLD30, OIT_W / 2.0f, 190, "FLT OPS Domain", C_WHITE, OIT_CENTER);
        oit_text(OIT_F_UI24, OIT_W / 2.0f, 250, "Login Page", C_WHITE, OIT_CENTER);
        oit_text(OIT_F_BOLD24, OIT_W / 2.0f, 310, t->side == OIT_CAPT ? "CAPTAIN" : "FIRST OFFICER", C_CYAN, OIT_CENTER);
        button(t, OIT_W / 2.0f - 120, 390, 240, 56, "PILOT", 1, 0, A_LOGIN, 0, OIT_F_BOLD20);
        button(t, OIT_W / 2.0f - 120, 466, 240, 56, "MAINTAINER", 0, 0, A_NONE, 0, OIT_F_BOLD20);
    } else {
        oit_text(OIT_F_BOLD30, OIT_W / 2.0f, 190, "NSS AVNCS DOMAIN", C_WHITE, OIT_CENTER);
        oit_text(OIT_F_UI24, OIT_W / 2.0f, 250, "LOGIN PAGE", C_WHITE, OIT_CENTER);
        button(t, OIT_W / 2.0f - 120, 350, 240, 56, "PILOT", 1, 0, A_LOGIN, 0, OIT_F_BOLD20);
        button(t, OIT_W / 2.0f - 120, 426, 240, 56, "MAINTAINER", 0, 0, A_NONE, 0, OIT_F_BOLD20);
        button(t, OIT_W / 2.0f - 120, 502, 240, 56, "ADMINISTRATOR", 0, 0, A_NONE, 0, OIT_F_BOLD20);
        oit_text(OIT_F_UI18, 20, OIT_H - 40, "Connected.", C_WHITE, OIT_LEFT);
    }
}

static void session(OitTerminal *t, const OitSim *s)
{
    Domain *d = dom(t);
    if (!t->avncs) {
        fo_header(t);
        switch (d->page) {
        case PG_STS: pg_sts(t, s); break;
        case PG_TO: pg_to(t, s); break;
        case PG_LOAD: pg_load(t, s); break;
        case PG_LDG: pg_ldg(t, s); break;
        case PG_INFLT: pg_inflt(t, s); break;
        default: pg_fo_menu(t, s); break;
        }
        fo_footer(t);
        if (t->dropdown) fo_dropdown(t);
    } else {
        av_header(t);
        switch (d->page) {
        case PG_CCOM: pg_ccom(t, s); break;
        case PG_SYSTEMS: pg_systems(t, s); break;
        case PG_FAULTS: pg_faults(t, s); break;
        case PG_DOORS: pg_doors(t, s); break;
        case PG_ENGINES: pg_engines(t, s); break;
        default: pg_av_menu(t, s); break;
        }
        av_footer(t);
    }
}

void oit_render(OitTerminal *t, const OitSim *s)
{
    Domain *d;
    last_sim = s;
    t->nhits = 0; t->nfocus = 0;
    oit_frame_begin();
    oit_rect(0, 0, OIT_W, OIT_H, C_BG);
    if (!t->on || t->selftest_t < SELFTEST_S) { oit_frame_end(); return; }   /* off / self test: black */
    if (t->x_ois) {
        OitTerminal *o = &terms[!t->side];
        if (o->on && !o->avncs && o->dom[D_FLTOPS].state == DS_SESSION) {
            static Hit saved[96];
            int nh = o->nhits, nf = o->nfocus;
            memcpy(saved, o->hits, sizeof(saved));
            o->nhits = 0; o->nfocus = 0;
            session(o, s);
            memcpy(o->hits, saved, sizeof(saved)); o->nhits = nh; o->nfocus = nf;
            oit_rect(OIT_W / 2.0f - 170, OIT_H / 2.0f - 30, 340, 60, C_BG);
            oit_frame(OIT_W / 2.0f - 170, OIT_H / 2.0f - 30, 340, 60, 2, C_AMBER);
            oit_text(OIT_F_BOLD24, OIT_W / 2.0f, OIT_H / 2.0f - 14, "X-OIS Activated", C_AMBER, OIT_CENTER);
            oit_frame_end();
            return;
        }
        t->x_ois = 0;
    }
    d = dom(t);
    if (d->state == DS_LOADING) loading_screen(t, s, t->avncs);
    else if (d->state == DS_LOGIN) login_screen(t, t->avncs);
    else session(t, s);
    if (t->nfocus && t->focus >= t->nfocus) t->focus = 0;
    oit_frame_end();
}

/* ---------------------------------------------------------------- test hooks (offline renderer) */
void oit_test_open(OitTerminal *t, int avncs, int page)
{
    t->on = 1; t->selftest_t = SELFTEST_S; t->avncs = avncs;
    t->dom[avncs].state = DS_SESSION; t->dom[avncs].page = page; t->dropdown = 0; t->x_ois = 0; t->focus = 0; t->dirty = 1;
}
void oit_test_flight(const OitSim *s)
{
    snprintf(ac.flt, sizeof(ac.flt), "AFR123"); snprintf(ac.from, 8, "LFPG"); snprintf(ac.to, 8, "EGLL");
    snprintf(ac.cur.flt, sizeof(ac.cur.flt), "AFR123"); ac.cur.doy = s->day_of_year;
    ac.cur.out.t = s->zulu_s - 3600; ac.cur.out.fob = 158900; ac.cur.off.t = s->zulu_s - 3000; ac.cur.off.fob = 157700;
    post(s, "MVT OUT 08:41Z", "AFR123 F-WWOW LFPG-EGLL  OUT 08:41Z  FOB 158.9 T");
    post(s, "MVT OFF 08:51Z", "AFR123 F-WWOW LFPG-EGLL  OFF 08:51Z  FOB 157.7 T");
}
void oit_test_dropdown(OitTerminal *t, int which) { t->dropdown = which; t->dd_sel = 4; }
void oit_test_set_ccom(OitTerminal *t, int view, int sel) { t->ccom_view = view; t->inbox_sel = sel; }
const char *oit_test_flight_no(void) { return ac.flt; }
const char *oit_test_from(void) { return ac.from; }
const char *oit_test_to(void) { return ac.to; }
int oit_test_state(OitTerminal *t, int domain) { return t->dom[domain].state; }
int oit_test_page(OitTerminal *t) { return t->dom[t->avncs].page; }
int oit_test_x_ois(OitTerminal *t) { return t->x_ois; }
int oit_test_inbox_n(void) { return ac.inbox_n; }
