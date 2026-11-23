/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * XPLM-free core of the A380 EFB (docs/EFB_DEVELOPMENT.md): the ten sections and their tabs,
 * door / escape-slide rules, typed payload and radio entries, the FQMS tank view, a bounded JSON
 * writer for the tablet frame and the page -> plugin action grammar. The X-Plane boundary is
 * a380x_efb.c; this file never touches the simulator, so test_efb_core.c covers it directly.
 */
#ifndef A380X_EFB_CORE_H
#define A380X_EFB_CORE_H

#include <stddef.h>

/* ---- Sections and tabs (the side rail of the approved design) -------------------------------- */
enum {
    A380X_EFB_HOME, A380X_EFB_OFP, A380X_EFB_GROUND, A380X_EFB_LOADING, A380X_EFB_CABIN,
    A380X_EFB_FLIGHT, A380X_EFB_OPS, A380X_EFB_FAILURES, A380X_EFB_AIRCRAFT, A380X_EFB_SETTINGS,
    A380X_EFB_SECTIONS
};
#define A380X_EFB_MAX_TABS 3
const char *a380x_efb_section_name(int section);           /* "home" ... "settings"; NULL out of range */
int a380x_efb_section_find(const char *name);               /* -1 when unknown */
int a380x_efb_tab_count(int section);                       /* 0 for sections without tabs */
const char *a380x_efb_tab_name(int section, int tab);       /* NULL out of range */
int a380x_efb_tab_find(int section, const char *name);      /* -1 when unknown */
/* The ten pre-existing flyPad pages (a380x/xp12/efb/page/<name>, index = native PAGE_*) in the new
 * layout, and the closest legacy page of a section/tab (keeps action_1..6 meaningful). */
void a380x_efb_legacy_page_target(int page, int *section, int *tab);
int a380x_efb_section_legacy_page(int section, int tab);

/* ---- Doors and escape slides ----------------------------------------------------------------- */
/* sim/flightmodel2/misc/door_open_ratio order: 0..9 main deck M1L M1R .. M5L M5R, 10..15 upper deck
 * U1L .. U3R, 16 cockpit, 17..19 FWD / AFT / BULK cargo. The 16 passenger doors carry slides. */
#define A380X_EFB_DOORS 20
#define A380X_EFB_SLIDES 16
#define A380X_EFB_DOOR_COCKPIT 16
#define A380X_EFB_DOOR_CLOSED_RATIO 0.01f   /* above this a door is open (or moving) */
const char *a380x_efb_door_name(int door);
int a380x_efb_door_has_slide(int door);
int a380x_efb_door_find(const char *name);                  /* "M1L" ... "BULK"; -1 when unknown */
typedef enum { A380X_EFB_DOOR_OK = 0, A380X_EFB_DOOR_ARMED, A380X_EFB_DOOR_INVALID } A380XEfbDoorRule;
/* Toggling a closed door opens it: refused while its slide is armed. Closing is always allowed. */
A380XEfbDoorRule a380x_efb_door_toggle_rule(int door, float open_ratio, int slide_armed);
typedef enum {
    A380X_EFB_SLIDE_OK = 0, A380X_EFB_SLIDE_NO_SLIDE, A380X_EFB_SLIDE_DOOR_OPEN, A380X_EFB_SLIDE_INVALID
} A380XEfbSlideRule;
/* Arming needs the door closed; disarming is always allowed on a slide door. */
A380XEfbSlideRule a380x_efb_slide_rule(int door, float open_ratio, int arm);
/* Arms every closed passenger door; returns how many stayed disarmed because the door is open. */
int a380x_efb_arm_all(const float open_ratio[A380X_EFB_DOORS], unsigned char slides[A380X_EFB_SLIDES]);
/* A door opened by a path the EFB cannot veto (dataref write, other plugin) disarms its slide so the
 * model never shows "open and armed". Returns how many slides were disarmed. */
int a380x_efb_slides_follow_doors(const float open_ratio[A380X_EFB_DOORS], unsigned char slides[A380X_EFB_SLIDES]);
int a380x_efb_slides_armed(const unsigned char slides[A380X_EFB_SLIDES]);
int a380x_efb_doors_open(const float open_ratio[A380X_EFB_DOORS]);

/* ---- Payload (FBW A380-842 cabin.json5 / payload/mod.rs, per-pax mass of the old tablet) ------- */
#define A380X_EFB_CLASSES 4
#define A380X_EFB_HOLDS 3
#define A380X_EFB_PAX_KG 104.0                  /* 84 kg passenger + 20 kg baggage */
#define A380X_EFB_MAX_PAYLOAD_KG 101736.0       /* existing native clamp (MZFW - OEW) */
#define A380X_EFB_LB_PER_KG 2.20462262
extern const int a380x_efb_class_seats[A380X_EFB_CLASSES];   /* F 14, J 76, W 56, Y 338 = 484 */
extern const int a380x_efb_hold_max_kg[A380X_EFB_HOLDS];     /* FWD 28577, AFT 20310, BULK 2513 */
extern const char *const a380x_efb_class_names[A380X_EFB_CLASSES];
extern const char *const a380x_efb_hold_names[A380X_EFB_HOLDS];
typedef struct { int pax[A380X_EFB_CLASSES]; int cargo_kg[A380X_EFB_HOLDS]; } A380XEfbLoad;
typedef enum {
    A380X_EFB_ENTRY_OK = 0, A380X_EFB_ENTRY_EMPTY, A380X_EFB_ENTRY_NOT_A_NUMBER, A380X_EFB_ENTRY_TOO_LARGE
} A380XEfbEntry;
/* Unsigned whole number typed on the keypad: digits only (no sign, no exponent, at most 7). */
A380XEfbEntry a380x_efb_parse_count(const char *text, long maximum, long *value);
/* Cargo typed in the display unit; result in whole kilograms, limited to the hold. */
A380XEfbEntry a380x_efb_parse_cargo(const char *text, int hold, int pounds, int *kilograms);
int a380x_efb_load_pax(const A380XEfbLoad *load);
double a380x_efb_load_kg(const A380XEfbLoad *load);

/* ---- Radios ---------------------------------------------------------------------------------- */
/* "121.9", "121.900" or "121900" -> kHz; 118.000-136.990 MHz on a 25 / 8.33 kHz channel name. */
int a380x_efb_parse_com_khz(const char *text, int *khz);
int a380x_efb_parse_squawk(const char *text, int *code);   /* four octal digits */

/* ---- Fuel: the FQMS tank view of the nine X-Plane tanks (same derivation as the SD FUEL page) - */
#define A380X_EFB_TANKS 11
#define A380X_EFB_FEED23_KG (7753.2 * 3.0387)
extern const char *const a380x_efb_tank_names[A380X_EFB_TANKS];
/* Returns 0 (all outputs NaN) when fewer than 8 X-Plane tanks are known. */
int a380x_efb_fqms_tanks(const float xp_kg[9], int count, double out_kg[A380X_EFB_TANKS]);

/* ---- Bounded JSON writer (never emits NaN/Inf: non-finite numbers become null) ---------------- */
typedef struct {
    char *data;
    size_t cap, len;
    int failed;
    unsigned depth;
    unsigned char comma[24];
} A380XEfbJson;
void a380x_efbj_init(A380XEfbJson *j, char *buffer, size_t cap);
void a380x_efbj_object(A380XEfbJson *j, const char *key);   /* key NULL inside arrays / at the root */
void a380x_efbj_array(A380XEfbJson *j, const char *key);
void a380x_efbj_end(A380XEfbJson *j, char closer);           /* '}' or ']' */
void a380x_efbj_number(A380XEfbJson *j, const char *key, double value, int decimals);
void a380x_efbj_int(A380XEfbJson *j, const char *key, long value);
void a380x_efbj_bool(A380XEfbJson *j, const char *key, int value);
void a380x_efbj_null(A380XEfbJson *j, const char *key);
void a380x_efbj_string(A380XEfbJson *j, const char *key, const char *value);
/* Returns the length (NUL-terminated) or -1 when the buffer overflowed or nesting is unbalanced. */
long a380x_efbj_finish(A380XEfbJson *j);

/* ---- Page -> plugin actions: "<id> <verb> [arg ...]" ------------------------------------------ */
#define A380X_EFB_ACTION_TEXT 128
#define A380X_EFB_ACTION_ARGS 4
typedef struct {
    unsigned long id;
    int argc;
    char verb[24];
    char arg[A380X_EFB_ACTION_ARGS][40];
} A380XEfbAction;
/* Printable ASCII only, a positive id, a lower-case verb and at most four arguments. 0 = rejected. */
int a380x_efb_parse_action(const char *text, A380XEfbAction *out);

#endif
