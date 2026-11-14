/* Offline test of the OIT core: power / self test / domain loading / login, navigation by touch, keys and
 * dropdowns, FLT NUMBER / FROM / TO entry, SYNCHRO AVIONICS, X-OIS, domain switching, the OUT/OFF/ON/IN flight
 * log, laptop off, and a render of every page through the software backend (PNG via build.sh). No X-Plane. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "oit_core.h"
#include "oit_draw.h"

OitBackend *oit_soft_backend(void);
void oit_soft_clear(void);
int oit_soft_write_ppm(const char *path);
void oit_test_open(OitTerminal *t, int avncs, int page);
void oit_test_flight(const OitSim *s);
void oit_test_dropdown(OitTerminal *t, int which);
void oit_test_set_ccom(OitTerminal *t, int view, int sel);
const char *oit_test_flight_no(void);
const char *oit_test_from(void);
const char *oit_test_to(void);
int oit_test_state(OitTerminal *t, int domain);
int oit_test_page(OitTerminal *t);
int oit_test_x_ois(OitTerminal *t);
int oit_test_inbox_n(void);

enum { PG_FO_MENU, PG_STS, PG_TO, PG_LOAD, PG_LDG, PG_INFLT, PG_AV_MENU, PG_CCOM, PG_SYSTEMS, PG_FAULTS, PG_DOORS, PG_ENGINES };
enum { DS_LOADING, DS_LOGIN, DS_SESSION };

static void sim_defaults(OitSim *s)
{
    int i;
    static const float kg[9] = {58200, 9800, 33100, 33100, 9800, 3100, 3100, 2400, 0};
    memset(s, 0, sizeof(*s));
    s->powered = 1; s->brightness = 0.8f; s->laptop_on = 1; s->zulu_s = 9 * 3600 + 41 * 60 + 27; s->day_of_year = 315; s->year = 2026;
    s->sim_time_s = 100; strcpy(s->tail, "F-WWOW"); strcpy(s->icao, "A388");
    s->lat = 51.4775; s->lon = -0.4614; s->alt_ft = 83; s->on_ground = 1; s->parking_brake = 1;
    s->oat_c = 14; s->wind_dir = 270; s->wind_kt = 9; s->hdg_mag = 271;
    for (i = 0; i < 4; ++i) { s->eng_running[i] = 1; s->n1[i] = 19.8f + i * 0.2f; s->fuel_flow_kgh[i] = 1150 + i * 20; }
    s->gw_kg = 512300; s->fuel_kg = 152600; s->empty_kg = 277000; s->payload_kg = 82700; s->mtow_kg = 575150;
    s->fuel_tanks = 9; for (i = 0; i < 9; ++i) s->fuel_tank_kg[i] = kg[i];
    s->fms_ok = 1; s->v1 = 148; s->vr = 153; s->v2 = 160; s->flex_c = 46; s->ths = 1.4f; s->zfw_kg = 359200; s->zfw_cg = 34.5f;
    s->fms_gw_kg = 511800; s->crz_alt_ft = 38000; s->vapp = 141; s->vls = 136; s->vref = 136; s->ldg_elev_ft = 13;
    s->dh_ft = 200; s->trans_alt_ft = 6000; s->to_flaps = 2; s->app_f = 158; s->app_s = 189; s->app_gd = 228; s->app_headwind = 8;
    s->door_ratio[1] = 1.0f; s->doors_open = 1;
    s->fault[21] = 1; s->fault[36] = 1;      /* HYD ENG 3 PMP, NAV PITOT 2 */
    strcpy(s->near_apt, "EGLL"); strcpy(s->dest_apt, "LFPG");
}

static void render(OitTerminal *t, const OitSim *s, const char *name)
{
    char path[256];
    oit_soft_clear();
    oit_render(t, s);
    snprintf(path, sizeof(path), "out/%s.ppm", name);
    assert(oit_soft_write_ppm(path));
}

/* touch as in X-Plane: the screen is redrawn after every input, so the targets are those of the current page */
static void touch(OitTerminal *t, const OitSim *s, float x, float y)
{
    oit_soft_clear(); oit_render(t, s);
    oit_touch(t, x, y);
}

/* advance the simulation: time moves, the captain OIT's tick runs the flight log */
static void run(OitTerminal *t, OitSim *s, double seconds)
{
    double step = 0.5;
    for (; seconds > 0; seconds -= step) { s->sim_time_s += step; s->zulu_s += step; oit_tick(t, s, step); }
}

int main(void)
{
    static const struct { int avncs, page; const char *name; } pages[] = {
        {0, PG_FO_MENU, "p01_fltops_menu"}, {0, PG_STS, "p02_fltops_sts"}, {0, PG_TO, "p03_to_perf"},
        {0, PG_LOAD, "p04_loadsheet"}, {0, PG_LDG, "p05_ldg_perf"}, {0, PG_INFLT, "p06_inflt_perf"},
        {1, PG_AV_MENU, "p07_avncs_menu"}, {1, PG_CCOM, "p08_company_com_inbox"}, {1, PG_SYSTEMS, "p10_a380x_systems"},
        {1, PG_FAULTS, "p11_current_faults"}, {1, PG_DOORS, "p12_doors"}, {1, PG_ENGINES, "p13_engines"},
    };
    OitSim s;
    OitTerminal *t = oit_terminal(OIT_CAPT), *f = oit_terminal(OIT_FO);
    int i, n0;
    oit_set_backend(oit_soft_backend());
    sim_defaults(&s);

    /* power: off -> self test (black, keys ignored) -> FLT OPS loading -> login -> session */
    s.powered = 0; run(t, &s, 1); render(t, &s, "a1_off");
    s.powered = 1; run(t, &s, 5); oit_key(t, OIT_K_ENTER, 0); render(t, &s, "a2_selftest");
    assert(oit_test_state(t, 0) == DS_LOADING);
    run(t, &s, 5 + 8); render(t, &s, "a3_fltops_loading");
    assert(oit_test_state(t, 0) == DS_LOADING);
    run(t, &s, 13); render(t, &s, "a4_fltops_login");
    assert(oit_test_state(t, 0) == DS_LOGIN && oit_test_state(t, 1) == DS_LOGIN);
    touch(t, &s, 512, 418);                                         /* PILOT */
    assert(oit_test_state(t, 0) == DS_SESSION && oit_test_page(t) == PG_FO_MENU);
    render(t, &s, "a5_fltops_menu");

    /* touch a menu button, the MENU dropdown by keys, FUNCTIONS > PREVIOUS, ESC */
    touch(t, &s, 600, 170); assert(oit_test_page(t) == PG_TO);        /* PERFORMANCE column: T.O PERF */
    render(t, &s, "b1_to_perf");
    touch(t, &s, 70, 30); render(t, &s, "b2_menu_dropdown");          /* MENU v */
    for (i = 0; i < 5; ++i) oit_key(t, OIT_K_DOWN, 0);
    oit_key(t, OIT_K_ENTER, 0); assert(oit_test_page(t) == PG_LOAD);
    touch(t, &s, 470, 30); oit_key(t, OIT_K_DOWN, 0); oit_key(t, OIT_K_ENTER, 0);   /* FUNCTIONS > PREVIOUS */
    assert(oit_test_page(t) == PG_TO);
    oit_key(t, OIT_K_ESC, 0); assert(oit_test_page(t) == PG_FO_MENU);

    /* FLT OPS STS from the switching bar; FLT NUMBER typed on the keyboard; TAB to FROM; SYNCHRO AVIONICS */
    touch(t, &s, 150, 770); assert(oit_test_page(t) == PG_STS);
    touch(t, &s, 400, 400);                                         /* FLT NUMBER field */
    oit_key(t, OIT_K_CHAR, 'a'); oit_key(t, OIT_K_CHAR, 'f'); oit_key(t, OIT_K_CHAR, 'r'); oit_key(t, OIT_K_CHAR, '1');
    oit_key(t, OIT_K_CHAR, '2'); oit_key(t, OIT_K_CHAR, '9'); oit_key(t, OIT_K_BACKSPACE, 0); oit_key(t, OIT_K_CHAR, '3');
    render(t, &s, "c1_sts_typing");
    oit_key(t, OIT_K_TAB, 0); oit_key(t, OIT_K_CHAR, 'x'); oit_key(t, OIT_K_ESC, 0);
    assert(!strcmp(oit_test_flight_no(), "AFR123") && !strcmp(oit_test_from(), ""));
    touch(t, &s, 850, 114);                                         /* SYNCHRO AVIONICS */
    assert(!strcmp(oit_test_from(), "EGLL") && !strcmp(oit_test_to(), "LFPG"));
    render(t, &s, "c2_sts_synchro");

    /* X-OIS: the F/O OIT is in a FLT OPS session on LOADSHEET; OIS XFR on the captain OIT mirrors it */
    run(f, &s, 0.5); oit_test_open(f, 0, PG_LOAD);
    oit_key(t, OIT_K_XFR, 0); assert(oit_test_x_ois(t));
    render(t, &s, "d1_x_ois");
    oit_key(t, OIT_K_XFR, 0); assert(!oit_test_x_ois(t));

    /* NSS AVNCS: switch, login (posts a message), COMPANY COM */
    s.avncs = 1; run(t, &s, 0.5); assert(oit_test_state(t, 1) == DS_LOGIN);
    render(t, &s, "e1_avncs_login");
    n0 = oit_test_inbox_n();
    oit_key(t, OIT_K_ENTER, 0); assert(oit_test_page(t) == PG_AV_MENU && oit_test_inbox_n() == n0 + 1);
    touch(t, &s, 600, 328); assert(oit_test_page(t) == PG_CCOM);

    /* OUT / OFF / ON / IN */
    s.parking_brake = 0; s.door_ratio[1] = 0; s.gs_kt = 8; run(t, &s, 2);
    s.on_ground = 0; s.ias_kt = 170; run(t, &s, 12);
    s.fuel_kg -= 9000; s.on_ground = 1; s.ias_kt = 40; run(t, &s, 12);
    s.gs_kt = 0; s.parking_brake = 1; s.door_ratio[1] = 1; run(t, &s, 2);
    assert(oit_test_inbox_n() == n0 + 5);
    oit_test_set_ccom(t, 1, -1); render(t, &s, "p09_company_com_flight_log");
    oit_test_set_ccom(t, 0, oit_test_inbox_n() - 1); render(t, &s, "e2_inbox_message");

    /* laptop off: FLT OPS not available */
    s.avncs = 0; s.laptop_on = 0; run(t, &s, 0.5); render(t, &s, "f1_laptop_off");
    assert(oit_test_state(t, 0) == DS_LOADING);
    s.laptop_on = 1;

    /* every page */
    oit_test_flight(&s);
    for (i = 0; i < (int)(sizeof(pages) / sizeof(pages[0])); ++i) { oit_test_open(t, pages[i].avncs, pages[i].page); render(t, &s, pages[i].name); }
    oit_test_open(t, 0, PG_LOAD); oit_test_dropdown(t, 1); render(t, &s, "p14_menu_dropdown");
    printf("OIT core tests passed\n");
    return 0;
}
