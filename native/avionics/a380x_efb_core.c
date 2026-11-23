/* SPDX-License-Identifier: GPL-3.0-or-later
 * XPLM-free A380 EFB core (a380x_efb_core.h, docs/EFB_DEVELOPMENT.md).
 */
#include "a380x_efb_core.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define EFB_COUNT(a) ((int)(sizeof(a) / sizeof((a)[0])))

/* ---- Sections ---------------------------------------------------------------------------------- */
static const char *const section_names[A380X_EFB_SECTIONS] = {
    "home", "ofp", "ground", "loading", "cabin", "flight", "ops", "failures", "aircraft", "settings"
};
static const char *const section_tabs[A380X_EFB_SECTIONS][A380X_EFB_MAX_TABS] = {
    {NULL}, {NULL}, {"doors", "services", "push"}, {"payload", "fuel", "wb"}, {NULL},
    {"perf", "nav", "comms"}, {"checklists", "presets"}, {NULL}, {NULL}, {"general", "sounds"}
};

const char *a380x_efb_section_name(int section)
{
    return section >= 0 && section < A380X_EFB_SECTIONS ? section_names[section] : NULL;
}

int a380x_efb_section_find(const char *name)
{
    int i;
    if (name == NULL) return -1;
    for (i = 0; i < A380X_EFB_SECTIONS; ++i) if (!strcmp(name, section_names[i])) return i;
    return -1;
}

int a380x_efb_tab_count(int section)
{
    int n = 0;
    if (section < 0 || section >= A380X_EFB_SECTIONS) return 0;
    while (n < A380X_EFB_MAX_TABS && section_tabs[section][n] != NULL) ++n;
    return n;
}

const char *a380x_efb_tab_name(int section, int tab)
{
    return tab >= 0 && tab < a380x_efb_tab_count(section) ? section_tabs[section][tab] : NULL;
}

int a380x_efb_tab_find(int section, const char *name)
{
    int i, n = a380x_efb_tab_count(section);
    if (name == NULL) return -1;
    for (i = 0; i < n; ++i) if (!strcmp(name, section_tabs[section][i])) return i;
    return -1;
}

/* Legacy page order: DASHBOARD DISPATCH GROUND PERFORMANCE NAVIGATION ATC FAILURES CHECKLISTS
 * PRESETS SETTINGS (a380x_efb.c PAGE_*). */
void a380x_efb_legacy_page_target(int page, int *section, int *tab)
{
    static const int target[10][2] = {
        {A380X_EFB_HOME, 0}, {A380X_EFB_LOADING, 0}, {A380X_EFB_GROUND, 0}, {A380X_EFB_FLIGHT, 0},
        {A380X_EFB_FLIGHT, 1}, {A380X_EFB_FLIGHT, 2}, {A380X_EFB_FAILURES, 0}, {A380X_EFB_OPS, 0},
        {A380X_EFB_OPS, 1}, {A380X_EFB_SETTINGS, 0}
    };
    if (page < 0 || page >= 10) page = 0;
    if (section) *section = target[page][0];
    if (tab) *tab = target[page][1];
}

int a380x_efb_section_legacy_page(int section, int tab)
{
    switch (section) {
    case A380X_EFB_OFP: case A380X_EFB_LOADING: case A380X_EFB_CABIN: return 1;
    case A380X_EFB_GROUND: return 2;
    case A380X_EFB_FLIGHT: return tab == 1 ? 4 : tab == 2 ? 5 : 3;
    case A380X_EFB_FAILURES: return 6;
    case A380X_EFB_OPS: return tab == 1 ? 8 : 7;
    case A380X_EFB_AIRCRAFT: case A380X_EFB_SETTINGS: return 9;
    default: return 0;
    }
}

/* ---- Doors ------------------------------------------------------------------------------------- */
static const char *const door_names[A380X_EFB_DOORS] = {
    "M1L", "M1R", "M2L", "M2R", "M3L", "M3R", "M4L", "M4R", "M5L", "M5R",
    "U1L", "U1R", "U2L", "U2R", "U3L", "U3R", "CKPT", "FWD", "AFT", "BULK"
};

const char *a380x_efb_door_name(int door)
{
    return door >= 0 && door < A380X_EFB_DOORS ? door_names[door] : NULL;
}

int a380x_efb_door_has_slide(int door) { return door >= 0 && door < A380X_EFB_SLIDES; }

int a380x_efb_door_find(const char *name)
{
    int i;
    if (name == NULL) return -1;
    for (i = 0; i < A380X_EFB_DOORS; ++i) if (!strcmp(name, door_names[i])) return i;
    return -1;
}

static int door_closed(float ratio) { return isfinite(ratio) && ratio <= A380X_EFB_DOOR_CLOSED_RATIO; }

A380XEfbDoorRule a380x_efb_door_toggle_rule(int door, float open_ratio, int slide_armed)
{
    if (door < 0 || door >= A380X_EFB_DOORS) return A380X_EFB_DOOR_INVALID;
    if (a380x_efb_door_has_slide(door) && slide_armed && door_closed(open_ratio)) return A380X_EFB_DOOR_ARMED;
    return A380X_EFB_DOOR_OK;
}

A380XEfbSlideRule a380x_efb_slide_rule(int door, float open_ratio, int arm)
{
    if (door < 0 || door >= A380X_EFB_DOORS) return A380X_EFB_SLIDE_INVALID;
    if (!a380x_efb_door_has_slide(door)) return A380X_EFB_SLIDE_NO_SLIDE;
    if (arm && !door_closed(open_ratio)) return A380X_EFB_SLIDE_DOOR_OPEN;
    return A380X_EFB_SLIDE_OK;
}

int a380x_efb_arm_all(const float open_ratio[A380X_EFB_DOORS], unsigned char slides[A380X_EFB_SLIDES])
{
    int i, left = 0;
    for (i = 0; i < A380X_EFB_SLIDES; ++i) {
        if (door_closed(open_ratio[i])) slides[i] = 1;
        else { slides[i] = 0; ++left; }
    }
    return left;
}

int a380x_efb_slides_follow_doors(const float open_ratio[A380X_EFB_DOORS], unsigned char slides[A380X_EFB_SLIDES])
{
    int i, changed = 0;
    for (i = 0; i < A380X_EFB_SLIDES; ++i)
        if (slides[i] && !door_closed(open_ratio[i])) { slides[i] = 0; ++changed; }
    return changed;
}

int a380x_efb_slides_armed(const unsigned char slides[A380X_EFB_SLIDES])
{
    int i, n = 0;
    for (i = 0; i < A380X_EFB_SLIDES; ++i) n += slides[i] != 0;
    return n;
}

int a380x_efb_doors_open(const float open_ratio[A380X_EFB_DOORS])
{
    int i, n = 0;
    for (i = 0; i < A380X_EFB_DOORS; ++i) n += !door_closed(open_ratio[i]);
    return n;
}

/* ---- Payload ----------------------------------------------------------------------------------- */
const int a380x_efb_class_seats[A380X_EFB_CLASSES] = {14, 76, 56, 338};
const int a380x_efb_hold_max_kg[A380X_EFB_HOLDS] = {28577, 20310, 2513};
const char *const a380x_efb_class_names[A380X_EFB_CLASSES] = {"First", "Business", "Premium economy", "Economy"};
const char *const a380x_efb_hold_names[A380X_EFB_HOLDS] = {"Forward hold", "Aft hold", "Bulk hold"};

A380XEfbEntry a380x_efb_parse_count(const char *text, long maximum, long *value)
{
    long result = 0;
    int digits = 0;
    if (text == NULL || *text == 0) return A380X_EFB_ENTRY_EMPTY;
    for (; *text; ++text) {
        if (*text < '0' || *text > '9') return A380X_EFB_ENTRY_NOT_A_NUMBER;
        if (++digits > 7) return A380X_EFB_ENTRY_TOO_LARGE;
        result = result * 10 + (*text - '0');
    }
    if (result > maximum) return A380X_EFB_ENTRY_TOO_LARGE;
    if (value) *value = result;
    return A380X_EFB_ENTRY_OK;
}

A380XEfbEntry a380x_efb_parse_cargo(const char *text, int hold, int pounds, int *kilograms)
{
    long typed = 0, limit;
    A380XEfbEntry result;
    if (hold < 0 || hold >= A380X_EFB_HOLDS) return A380X_EFB_ENTRY_NOT_A_NUMBER;
    limit = pounds ? (long)floor(a380x_efb_hold_max_kg[hold] * A380X_EFB_LB_PER_KG + 1e-6) : a380x_efb_hold_max_kg[hold];
    result = a380x_efb_parse_count(text, limit, &typed);
    if (result != A380X_EFB_ENTRY_OK) return result;
    if (kilograms) {
        long kg = pounds ? lround(typed / A380X_EFB_LB_PER_KG) : typed;
        *kilograms = (int)(kg > a380x_efb_hold_max_kg[hold] ? a380x_efb_hold_max_kg[hold] : kg);
    }
    return A380X_EFB_ENTRY_OK;
}

int a380x_efb_load_pax(const A380XEfbLoad *load)
{
    int i, n = 0;
    for (i = 0; i < A380X_EFB_CLASSES; ++i) n += load->pax[i];
    return n;
}

double a380x_efb_load_kg(const A380XEfbLoad *load)
{
    double kg = a380x_efb_load_pax(load) * A380X_EFB_PAX_KG;
    int i;
    for (i = 0; i < A380X_EFB_HOLDS; ++i) kg += load->cargo_kg[i];
    return kg;
}

/* ---- Radios ------------------------------------------------------------------------------------ */
int a380x_efb_parse_com_khz(const char *text, int *khz)
{
    long whole = 0, fraction = 0;
    int whole_digits = 0, fraction_digits = 0, dot = 0, value, last;
    if (text == NULL || *text == 0) return 0;
    for (; *text; ++text) {
        if (*text == '.') { if (dot++) return 0; continue; }
        if (*text < '0' || *text > '9') return 0;
        if (!dot) { if (++whole_digits > 6) return 0; whole = whole * 10 + (*text - '0'); }
        else { if (++fraction_digits > 3) return 0; fraction = fraction * 10 + (*text - '0'); }
    }
    if (!dot && whole_digits == 6) value = (int)whole;                   /* 121900 */
    else if (whole_digits == 3) {
        while (fraction_digits < 3) { fraction *= 10; ++fraction_digits; }
        value = (int)(whole * 1000 + fraction);
    } else return 0;
    if (value < 118000 || value > 136990 || value % 5) return 0;
    last = value % 100;   /* 8.33 kHz channel names never end in 20, 45, 70 or 95 */
    if (last == 20 || last == 45 || last == 70 || last == 95) return 0;
    if (khz) *khz = value;
    return 1;
}

int a380x_efb_parse_squawk(const char *text, int *code)
{
    int value = 0, digits = 0;
    if (text == NULL) return 0;
    for (; *text; ++text) {
        if (*text < '0' || *text > '7' || ++digits > 4) return 0;
        value = value * 10 + (*text - '0');
    }
    if (digits != 4) return 0;
    if (code) *code = value;
    return 1;
}

/* ---- Fuel -------------------------------------------------------------------------------------- */
const char *const a380x_efb_tank_names[A380X_EFB_TANKS] = {
    "L OUTR", "FEED 1", "L MID", "L INR", "FEED 2", "FEED 3", "R INR", "R MID", "FEED 4", "R OUTR", "TRIM"
};

/* ACF tanks: 0 centre wing, 1..4 main 1..4, 5/6 outboard reserves, 7 horizontal stabiliser. Feed 2/3
 * hold at most the FBW feed capacity of their X-Plane main tank, the rest is the inner tank; the
 * centre wing tank is split between L/R MID (a380x_sd_data.inc, FUEL). */
int a380x_efb_fqms_tanks(const float xp_kg[9], int count, double out_kg[A380X_EFB_TANKS])
{
    double m[8];
    int i;
    if (xp_kg == NULL || count < 8) {
        for (i = 0; i < A380X_EFB_TANKS; ++i) out_kg[i] = NAN;
        return 0;
    }
    for (i = 0; i < 8; ++i) m[i] = isfinite(xp_kg[i]) && xp_kg[i] > 0.0f ? xp_kg[i] : 0.0;
    out_kg[0] = m[5];
    out_kg[1] = m[1];
    out_kg[2] = 0.5 * m[0];
    out_kg[3] = fmax(0.0, m[2] - A380X_EFB_FEED23_KG);
    out_kg[4] = fmin(m[2], A380X_EFB_FEED23_KG);
    out_kg[5] = fmin(m[3], A380X_EFB_FEED23_KG);
    out_kg[6] = fmax(0.0, m[3] - A380X_EFB_FEED23_KG);
    out_kg[7] = 0.5 * m[0];
    out_kg[8] = m[4];
    out_kg[9] = m[6];
    out_kg[10] = m[7];
    return 1;
}

/* ---- JSON -------------------------------------------------------------------------------------- */
void a380x_efbj_init(A380XEfbJson *j, char *buffer, size_t cap)
{
    memset(j, 0, sizeof(*j));
    j->data = buffer;
    j->cap = cap;
    if (buffer == NULL || cap == 0) j->failed = 1;
    else buffer[0] = 0;
}

static void efbj_raw(A380XEfbJson *j, const char *text, size_t n)
{
    if (j->failed) return;
    if (j->len + n + 1 > j->cap) { j->failed = 1; return; }
    memcpy(j->data + j->len, text, n);
    j->len += n;
    j->data[j->len] = 0;
}

static void efbj_text(A380XEfbJson *j, const char *text) { efbj_raw(j, text, strlen(text)); }

static void efbj_quoted(A380XEfbJson *j, const char *text)
{
    const unsigned char *p;
    efbj_raw(j, "\"", 1);
    for (p = (const unsigned char *)(text ? text : ""); *p; ++p) {
        char escape[8];
        if (*p == '"' || *p == '\\') { escape[0] = '\\'; escape[1] = (char)*p; efbj_raw(j, escape, 2); }
        else if (*p < 32) { snprintf(escape, sizeof(escape), "\\u%04x", *p); efbj_text(j, escape); }
        else efbj_raw(j, (const char *)p, 1);
    }
    efbj_raw(j, "\"", 1);
}

static void efbj_member(A380XEfbJson *j, const char *key)
{
    if (j->failed) return;
    if (j->depth > 0) {
        if (j->comma[j->depth - 1]) efbj_raw(j, ",", 1);
        j->comma[j->depth - 1] = 1;
    }
    if (key != NULL) { efbj_quoted(j, key); efbj_raw(j, ":", 1); }
}

static void efbj_open(A380XEfbJson *j, const char *key, const char *opener)
{
    efbj_member(j, key);
    efbj_text(j, opener);
    if (j->depth >= sizeof(j->comma)) { j->failed = 1; return; }
    j->comma[j->depth++] = 0;
}

void a380x_efbj_object(A380XEfbJson *j, const char *key) { efbj_open(j, key, "{"); }
void a380x_efbj_array(A380XEfbJson *j, const char *key) { efbj_open(j, key, "["); }

void a380x_efbj_end(A380XEfbJson *j, char closer)
{
    char text[2] = {closer, 0};
    if (j->depth == 0) { j->failed = 1; return; }
    --j->depth;
    efbj_text(j, text);
}

void a380x_efbj_number(A380XEfbJson *j, const char *key, double value, int decimals)
{
    char text[48];
    efbj_member(j, key);
    if (!isfinite(value)) { efbj_text(j, "null"); return; }
    if (decimals < 0) decimals = 0;
    if (decimals > 6) decimals = 6;
    if (fabs(value) >= 1e15) { efbj_text(j, "null"); return; }
    snprintf(text, sizeof(text), "%.*f", decimals, value);
    if (!strcmp(text, "-0") || !strcmp(text, "-0.0") || !strcmp(text, "-0.00")) snprintf(text, sizeof(text), "%.*f", decimals, 0.0);
    efbj_text(j, text);
}

void a380x_efbj_int(A380XEfbJson *j, const char *key, long value)
{
    char text[32];
    efbj_member(j, key);
    snprintf(text, sizeof(text), "%ld", value);
    efbj_text(j, text);
}

void a380x_efbj_bool(A380XEfbJson *j, const char *key, int value)
{
    efbj_member(j, key);
    efbj_text(j, value ? "true" : "false");
}

void a380x_efbj_null(A380XEfbJson *j, const char *key)
{
    efbj_member(j, key);
    efbj_text(j, "null");
}

void a380x_efbj_string(A380XEfbJson *j, const char *key, const char *value)
{
    efbj_member(j, key);
    efbj_quoted(j, value);
}

long a380x_efbj_finish(A380XEfbJson *j)
{
    if (j->failed || j->depth != 0) {
        if (j->data != NULL && j->cap > 0) j->data[0] = 0;
        return -1;
    }
    return (long)j->len;
}

/* ---- Actions ----------------------------------------------------------------------------------- */
int a380x_efb_parse_action(const char *text, A380XEfbAction *out)
{
    const char *p = text;
    unsigned long id = 0;
    int digits = 0, field = -1;
    if (text == NULL || out == NULL) return 0;
    memset(out, 0, sizeof(*out));
    if (strlen(text) >= A380X_EFB_ACTION_TEXT) return 0;
    for (; *p; ++p) if ((unsigned char)*p < 32 || (unsigned char)*p > 126) return 0;
    for (p = text; *p >= '0' && *p <= '9'; ++p) {
        if (++digits > 9) return 0;
        id = id * 10 + (unsigned long)(*p - '0');
    }
    if (!digits || id == 0 || *p != ' ') return 0;
    out->id = id;
    while (*p) {
        size_t n = 0;
        char *target;
        size_t limit;
        while (*p == ' ') ++p;
        if (!*p) break;
        if (field >= A380X_EFB_ACTION_ARGS) return 0;
        target = field < 0 ? out->verb : out->arg[field];
        limit = field < 0 ? sizeof(out->verb) : sizeof(out->arg[0]);
        while (*p && *p != ' ') {
            if (n + 1 >= limit) return 0;
            target[n++] = *p++;
        }
        target[n] = 0;
        ++field;
    }
    if (field < 0) return 0;
    for (p = out->verb; *p; ++p) if (!((*p >= 'a' && *p <= 'z') || *p == '_')) return 0;
    out->argc = field;
    return 1;
}
