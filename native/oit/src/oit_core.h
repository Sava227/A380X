/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380X OIT core: terminal state, pages and input handling, independent of X-Plane (tested offline).
 */
#pragma once

enum { OIT_CAPT = 0, OIT_FO = 1 };

/* Keys from the KCCU / OIT keyboard (a380x/xp12/oit/<side>/kbd/...) and the cursor control (acd/...). */
enum {
    OIT_K_NONE, OIT_K_UP, OIT_K_DOWN, OIT_K_LEFT, OIT_K_RIGHT, OIT_K_ENTER, OIT_K_ESC, OIT_K_MENU, OIT_K_XFR,
    OIT_K_TAB, OIT_K_BACKSPACE, OIT_K_DEL, OIT_K_PGUP, OIT_K_PGDN, OIT_K_F1, OIT_K_F12 = OIT_K_F1 + 11,
    OIT_K_CHAR   /* printable character in the ch argument */
};

typedef struct {
    /* power and controls */
    int powered;              /* AC power on the OIT bus (stock bus voltage, slaved by the A380 ELEC) */
    float brightness;         /* side knob 0..1 */
    int avncs;                /* side switch: 0 FLT OPS, 1 NSS AVNCS */
    /* time */
    double zulu_s;            /* seconds since midnight UTC */
    int day_of_year;          /* 0-based */
    int year;
    double sim_time_s;        /* total running sim time */
    /* aircraft */
    char tail[32];
    char icao[8];
    double lat, lon;
    float alt_ft, ias_kt, gs_kt, vs_fpm, hdg_mag, mach, oat_c, tat_c;
    int on_ground, parking_brake, paused;
    int eng_running[4];
    float n1[4], fuel_flow_kgh[4];
    float gw_kg, fuel_kg, empty_kg, payload_kg, cg_mac;  /* cg_mac: % MAC, < -900 = unknown */
    float fuel_tank_kg[11]; int fuel_tanks;
    float flaps_handle;       /* 0..1 */
    int gear_down;
    float wind_dir, wind_kt;
    int doors_open;           /* passenger/cargo doors not closed (count) */
    /* A380 FMS (a380x/xp12/fms/..., published by the avionics plugin; 0 = not entered / not available) */
    int fms_ok;
    float v1, vr, v2, flex_c, ths, zfw_kg, zfw_cg, fms_gw_kg, crz_alt_ft, vapp, vls, vref, ldg_elev_ft, dh_ft, mda_ft,
          trans_alt_ft;
    int to_flaps, flight_phase, to_power;
    float app_f, app_s, app_gd, app_headwind; int ldg_conf3;
    float mtow_kg;            /* ACF maximum take-off weight */
    float door_ratio[20];     /* 0-9 main deck M1R..M5L, 10-15 upper deck U1R..U3L, 16-19 cargo */
    int laptop_on;            /* this side's FLT OPS laptop (a380x/xp12/cockpit/<side>_laptop_power) */
    char near_apt[8];         /* nearest airport (X-Plane navigation database) */
    char dest_apt[8];         /* airport at the FMS destination position, if any */
    unsigned char fault[64];  /* 1 = failed, indexed like oit_faults[] */
} OitSim;

/* X-Plane failures shown on the CMS CURRENT FAULTS page (sim/operation/failures/<ref> == 6: inoperative). */
typedef struct { const char *ref; int ata; const char *text; } OitFault;
extern const OitFault oit_faults[];
extern const int oit_fault_count;

typedef struct OitTerminal OitTerminal;

OitTerminal *oit_terminal(int side);
void oit_reset(OitTerminal *t);
void oit_tick(OitTerminal *t, const OitSim *sim, double dt);
void oit_key(OitTerminal *t, int key, char ch);
void oit_touch(OitTerminal *t, float x, float y);       /* screen pixels, origin top-left */
void oit_scroll(OitTerminal *t, int clicks);
void oit_render(OitTerminal *t, const OitSim *sim);    /* draws with the current oit_draw backend */
int oit_take_dirty(OitTerminal *t);                     /* 1 if the screen changed since the last call */
