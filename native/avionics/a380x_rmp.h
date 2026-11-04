/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * A380 Radio Management Panels (RMP 1-3): XPLM-free logic ported from FlyByWire A380X
 * (fbw-a380x/src/systems/instruments/src/RMP: VhfComController.ts, VhfComPage.tsx, TransponderController.ts,
 * RmpPageStack.tsx, RmpStateController.ts; fbw-common RadioUtils.ts). GPL-3.0.
 *
 * Frequencies are MSFS BCD32 as in FBW (0x1183000 = 118.300 MHz). Three RMPs share the radios; each RMP has
 * its own page, selected row and standby entry. Pages: VHF (three rows VHF1/2/3) and SQWK; the other page
 * keys (HF, TEL, MENU, NAV) are not implemented by FBW either and leave the page unchanged.
 *
 * VHF page: ADKn / LSKn select row n; on the selected row ADK swaps active and standby, LSK enters the typed
 * standby frequency. Digits type into the selected row (FBW auto-completion: 8 -> 118.000, 875 -> 118.750,
 * 3x -> 13x.___), CLR deletes the last digit (held 1 s: the whole entry), UP / DOWN change the VHF 3 standby
 * mode (DATA / EMER / frequency). SQWK page: digits type the code (8 / 9 invalid), it validates itself 1 s
 * after the fourth digit or on any LSK; ADK3 transmits IDENT for 5 s.
 */
#ifndef A380X_RMP_H
#define A380X_RMP_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define A380X_RMP_COUNT 3
#define A380X_RMP_VHF 3

enum { A380X_RMP_PAGE_VHF, A380X_RMP_PAGE_SQWK };
enum { A380X_RMP_MODE_FREQUENCY, A380X_RMP_MODE_DATA, A380X_RMP_MODE_EMERGENCY };   /* FBW FrequencyMode */
enum { A380X_RMP_OFF_STANDBY, A380X_RMP_ON, A380X_RMP_OFF_FAILED, A380X_RMP_ON_FAILED };  /* FBW RmpState */
#define A380X_RMP_EMERGENCY_FREQ 0x1215000u
#define A380X_RMP_CLR_HELD_S 1.0f
#define A380X_RMP_XPDR_VALIDATE_S 1.0f
#define A380X_RMP_IDENT_S 5.0f

typedef struct {
    char entered[8];      /* typed digits */
    int offset;           /* digits auto-completed in front of the typed ones */
    char displayed[8];    /* six characters shown in the STBY field */
    uint32_t frequency;   /* BCD32, 0 while incomplete */
} A380XRmpEntry;

typedef struct {          /* shared by the three RMPs */
    uint32_t active[A380X_RMP_VHF], standby[A380X_RMP_VHF];
    int active_mode[A380X_RMP_VHF], standby_mode[A380X_RMP_VHF];
    int xpdr_code;        /* four octal digits as a decimal number (2000) */
    int xpdr_auto;
    float ident_left;
} A380XRmpRadios;

typedef struct {
    int page, row, state;
    A380XRmpEntry entry[A380X_RMP_VHF];
    char xpdr_entry[5];   /* "" = no entry; else four characters with '_' */
    int xpdr_invalid;
    float xpdr_validate_left, clr_held;
    int clr_down;
    char message[48];
    int tx;               /* transmitting VHF 1..3 (CALL key green bars), 0 none */
    int rx[A380X_RMP_VHF];
    float brightness;     /* OFF / BRT knob 0..1 */
} A380XRmp;

/* ---------------------------------------------------------------- RadioUtils (VhfCom8_33_25) */
static const uint32_t a380x_rmp_channels[16] = {0, 0x50, 0x100, 0x150, 0x250, 0x300, 0x350, 0x400, 0x500,
                                                0x550, 0x600, 0x650, 0x750, 0x800, 0x850, 0x900};
static inline int a380x_rmp_valid_frequency(uint32_t bcd32)
{
    if (bcd32 < 0x1180000u || bcd32 > 0x1369750u) return 0;
    for (int i = 0; i < 16; ++i) if ((bcd32 & 0xfffu) == a380x_rmp_channels[i]) return 1;
    return 0;
}
/* "118.300" (separator '.') or "118300" (separator 0). */
static inline void a380x_rmp_format(uint32_t bcd32, char sep, char *out, size_t n)
{
    char d[16];
    snprintf(d, sizeof(d), "%06x", (unsigned)((bcd32 >> 4) & 0xffffffu));
    if (sep) snprintf(out, n, "%.3s%c%.3s", d, sep, d + 3);
    else snprintf(out, n, "%s", d);
}
/* BCD32 <-> kHz (X-Plane com*_frequency_hz_833 are kHz: 118300). */
static inline int a380x_rmp_bcd_to_khz(uint32_t bcd32)
{
    int khz = 0;
    for (int shift = 24; shift >= 4; shift -= 4) khz = khz * 10 + (int)((bcd32 >> shift) & 0xfu);
    return khz;
}
static inline uint32_t a380x_rmp_khz_to_bcd(int khz)
{
    uint32_t bcd = 0;
    for (int shift = 4; shift <= 24; shift += 4) { bcd |= (uint32_t)(khz % 10) << shift; khz /= 10; }
    return bcd;
}
static inline uint32_t a380x_rmp_hex(const char *s) { uint32_t v = 0; for (; *s; ++s) v = (v << 4) | (uint32_t)(*s - '0'); return v; }

/* ---------------------------------------------------------------- messages */
static inline void a380x_rmp_message(A380XRmp *r, const char *radio, const char *text)
{
    if (radio) snprintf(r->message, sizeof(r->message), "%s %s", radio, text);
    else snprintf(r->message, sizeof(r->message), "%s", text);
}

/* ---------------------------------------------------------------- VHF standby entry (VhfComController) */
static inline void a380x_rmp_entry_clear(A380XRmpEntry *e) { memset(e, 0, sizeof(*e)); }
static inline int a380x_rmp_entry_invalid(const A380XRmpEntry *e)
{
    return e->frequency != 0 && e->frequency != 0x1100000u && !a380x_rmp_valid_frequency(e->frequency);
}
static inline void a380x_rmp_set_entry(A380XRmpEntry *e, const char *entered, int offset, const char *displayed, uint32_t f)
{
    size_t n = strlen(entered), m = strlen(displayed);
    if (n > 7) n = 7;
    if (m > 7) m = 7;
    memcpy(e->entered, entered, n); e->entered[n] = 0;
    e->offset = offset;
    memcpy(e->displayed, displayed, m); e->displayed[m] = 0;
    e->frequency = f;
}
static inline void a380x_rmp_pad(char *out, const char *s, size_t len, char pad)
{
    size_t n = strlen(s);
    for (size_t i = 0; i < len; ++i) out[i] = i < n ? s[i] : pad;
    out[len] = 0;
}

static inline void a380x_rmp_digit(A380XRmp *r, const A380XRmpRadios *radios, int row, int digit)
{
    A380XRmpEntry *e = &r->entry[row];
    char entered[16], shown[16], fmt[16];
    uint32_t f;
    size_t len;
    if (radios->standby_mode[row] != A380X_RMP_MODE_FREQUENCY) return;
    if (!e->entered[0] && digit == 1) { a380x_rmp_set_entry(e, "1", 0, "1_____", 0); return; }
    if (strlen(e->entered) >= 6 || a380x_rmp_entry_invalid(e)) return;
    len = strlen(e->entered);
    memcpy(entered, e->entered, len);
    entered[len++] = (char)('0' + (digit % 10));
    entered[len] = 0;
    f = a380x_rmp_hex(entered) << (4 + 4 * (6 - len));
    if (a380x_rmp_valid_frequency(f) || len == 6) {
        a380x_rmp_pad(shown, entered, 6, '0');
        a380x_rmp_set_entry(e, entered, 0, shown, f);
        return;
    }
    if (entered[0] == '3') {              /* 13x.xxx: no auto-fill of the leading 13 */
        char tail[16];
        f = 0x1000000u | (f >> 4);
        if (len < 5 && f < 0x1369750u) f = 0;
        a380x_rmp_pad(tail, entered, 5, '_');
        shown[0] = '1';
        memcpy(shown + 1, tail, 6);   /* tail is 5 characters + NUL */
        a380x_rmp_set_entry(e, entered, 1, shown, f);
        return;
    }
    for (int i = 0; i < 3; ++i) {         /* shift right, filling leading 1s, until valid */
        if (i > 0) {
            f = (f >> 4) | 0x1000000u;
            if (a380x_rmp_valid_frequency(f)) {
                a380x_rmp_format(f, 0, fmt, sizeof(fmt));
                a380x_rmp_set_entry(e, entered, i, fmt, f);
                return;
            }
        }
        if ((size_t)e->offset + len == 5) {  /* valid with the channel spacing applied */
            for (int c = 0; c < 16; ++c) {
                uint32_t chan = a380x_rmp_channels[c], g;
                if ((chan & 0xf00u) != (f & 0xf00u)) continue;
                g = (f & ~0xffu) | chan;
                if (a380x_rmp_valid_frequency(g)) {
                    a380x_rmp_format(g, 0, fmt, sizeof(fmt));
                    a380x_rmp_set_entry(e, entered, i, fmt, g);
                    return;
                }
            }
        }
    }
    f = a380x_rmp_hex(entered) << (4 + 4 * (6 - len));
    a380x_rmp_format(f, 0, fmt, sizeof(fmt));
    fmt[len] = 0;
    a380x_rmp_pad(shown, fmt, 6, '_');
    a380x_rmp_set_entry(e, entered, 0, shown, f);
}

static inline void a380x_rmp_vhf_clear(A380XRmp *r, const A380XRmpRadios *radios, int row, int single)
{
    A380XRmpEntry *e = &r->entry[row];
    char old[16], name[16];
    size_t n = strlen(e->entered);
    snprintf(name, sizeof(name), "VHF%d", row + 1);
    if (!n) return;
    snprintf(old, sizeof(old), "%s", e->entered);
    a380x_rmp_entry_clear(e);
    if (single && n > 1) {
        char shown[16];
        for (size_t i = 0; i + 1 < n; ++i) a380x_rmp_digit(r, radios, row, old[i] - '0');
        memcpy(shown, e->displayed, sizeof(shown));
        shown[e->offset + (int)strlen(e->entered) < 7 ? e->offset + (int)strlen(e->entered) : 6] = 0;
        a380x_rmp_pad(e->displayed, shown, 6, '_');
    } else if (n == 1) a380x_rmp_message(r, name, "STBY REVERTED TO PREV ENTRY");
}

/* Swap active and standby of a VHF (ADK on the selected row). */
static inline void a380x_rmp_swap(A380XRmpRadios *radios, int row)
{
    uint32_t f = radios->active[row];
    int m = radios->active_mode[row];
    radios->active[row] = radios->standby[row]; radios->active_mode[row] = radios->standby_mode[row];
    radios->standby[row] = f; radios->standby_mode[row] = m;
}

static inline void a380x_rmp_lsk_vhf(A380XRmp *r, A380XRmpRadios *radios, int row)
{
    A380XRmpEntry *e = &r->entry[row];
    char name[16];
    snprintf(name, sizeof(name), "VHF%d", row + 1);
    if (!e->entered[0]) { a380x_rmp_message(r, name, "FREQ NOT COMPLETED"); return; }
    if (a380x_rmp_valid_frequency(e->frequency)) radios->standby[row] = e->frequency;
    else a380x_rmp_message(r, name, "STBY REVERTED TO PREV ENTRY");
    a380x_rmp_entry_clear(e);
}

static inline void a380x_rmp_mode_step(A380XRmp *r, A380XRmpRadios *radios, int row, int up)
{
    int max = row == 2 ? 2 : 0, m = radios->standby_mode[row] + (up ? 1 : -1);
    if (m < 0 || m > max) return;
    a380x_rmp_entry_clear(&r->entry[row]);
    radios->standby_mode[row] = m;
    if (m == A380X_RMP_MODE_EMERGENCY) radios->standby[row] = A380X_RMP_EMERGENCY_FREQ;
}

/* ---------------------------------------------------------------- transponder (TransponderController) */
static inline void a380x_rmp_xpdr_validate(A380XRmp *r, A380XRmpRadios *radios)
{
    r->xpdr_validate_left = 0;
    if (!r->xpdr_entry[0]) return;
    if (r->xpdr_invalid || strchr(r->xpdr_entry, '_')) {
        a380x_rmp_message(r, NULL, "SQUAWK CODE REVERTED TO PREV ENTRY");
    } else {
        int code = 0;
        for (int i = 0; i < 4; ++i) code = code * 10 + (r->xpdr_entry[i] - '0');
        radios->xpdr_code = code;
    }
    r->xpdr_entry[0] = 0; r->xpdr_invalid = 0;
}
static inline void a380x_rmp_xpdr_digit(A380XRmp *r, int digit)
{
    size_t n;
    r->xpdr_validate_left = 0;
    if (r->xpdr_invalid || (r->xpdr_entry[0] && !strchr(r->xpdr_entry, '_'))) return;
    if (!r->xpdr_entry[0]) memcpy(r->xpdr_entry, (char[5]){(char)('0' + digit % 10), '_', '_', '_', 0}, 5);
    else {
        n = (size_t)(strchr(r->xpdr_entry, '_') - r->xpdr_entry);
        r->xpdr_entry[n] = (char)('0' + digit);
        if (n == 3) r->xpdr_validate_left = A380X_RMP_XPDR_VALIDATE_S;
    }
    if (digit >= 8) { r->xpdr_invalid = 1; a380x_rmp_message(r, NULL, "SQUAWK CODE NOT VALID"); }
}
static inline void a380x_rmp_xpdr_clear(A380XRmp *r, int single)
{
    char *u;
    r->xpdr_validate_left = 0;
    if (!r->xpdr_entry[0]) return;
    r->xpdr_invalid = 0;
    u = strchr(r->xpdr_entry, '_');
    if (!single) r->xpdr_entry[0] = 0;
    else if (!u || u - r->xpdr_entry > 1) { int last = u ? (int)(u - r->xpdr_entry) - 1 : 3; r->xpdr_entry[last] = '_'; }
    else { r->xpdr_entry[0] = 0; a380x_rmp_message(r, NULL, "SQUAWK CODE REVERTED TO PREV ENTRY"); }
}

/* ---------------------------------------------------------------- keys */
enum {
    A380X_RMP_KEY_0, A380X_RMP_KEY_1, A380X_RMP_KEY_2, A380X_RMP_KEY_3, A380X_RMP_KEY_4, A380X_RMP_KEY_5,
    A380X_RMP_KEY_6, A380X_RMP_KEY_7, A380X_RMP_KEY_8, A380X_RMP_KEY_9, A380X_RMP_KEY_DOT, A380X_RMP_KEY_CLR,
    A380X_RMP_KEY_UP, A380X_RMP_KEY_DOWN, A380X_RMP_KEY_ADK1, A380X_RMP_KEY_ADK2, A380X_RMP_KEY_ADK3,
    A380X_RMP_KEY_LSK1, A380X_RMP_KEY_LSK2, A380X_RMP_KEY_LSK3, A380X_RMP_KEY_VHF, A380X_RMP_KEY_HF,
    A380X_RMP_KEY_TEL, A380X_RMP_KEY_SQWK, A380X_RMP_KEY_MENU, A380X_RMP_KEY_NAV, A380X_RMP_KEY_MSG_CLR,
    A380X_RMP_KEY_RST, A380X_RMP_KEY_VHF1_CALL, A380X_RMP_KEY_VHF2_CALL, A380X_RMP_KEY_VHF3_CALL,
    A380X_RMP_KEY_OTHER, A380X_RMP_KEYS
};

/* A key press (KeypadController: ignored unless the RMP is ON). */
static inline void a380x_rmp_key(A380XRmp *r, A380XRmpRadios *radios, int key)
{
    if (r->state != A380X_RMP_ON) return;
    if (key >= A380X_RMP_KEY_VHF1_CALL && key <= A380X_RMP_KEY_VHF3_CALL) {   /* transmit select */
        int vhf = key - A380X_RMP_KEY_VHF1_CALL + 1;
        r->tx = r->tx == vhf ? 0 : vhf;
        return;
    }
    switch (key) {
    case A380X_RMP_KEY_VHF: r->page = A380X_RMP_PAGE_VHF; return;
    case A380X_RMP_KEY_SQWK:
        if (r->page != A380X_RMP_PAGE_SQWK) { r->xpdr_entry[0] = 0; r->xpdr_invalid = 0; }
        r->page = A380X_RMP_PAGE_SQWK; return;
    case A380X_RMP_KEY_MSG_CLR: r->message[0] = 0; return;
    default: break;
    }
    if (r->page == A380X_RMP_PAGE_SQWK) {
        if (key <= A380X_RMP_KEY_9) a380x_rmp_xpdr_digit(r, key);
        else if (key == A380X_RMP_KEY_CLR) { a380x_rmp_xpdr_clear(r, 1); r->clr_down = 1; r->clr_held = 0; }
        else if (key >= A380X_RMP_KEY_LSK1 && key <= A380X_RMP_KEY_LSK3) a380x_rmp_xpdr_validate(r, radios);
        else if (key == A380X_RMP_KEY_ADK3 && radios->ident_left <= 0) { radios->ident_left = A380X_RMP_IDENT_S;
            a380x_rmp_message(r, NULL, "TRANSMITTING IDENT"); }
        return;
    }
    if (key <= A380X_RMP_KEY_9) a380x_rmp_digit(r, radios, r->row, key);
    else if (key == A380X_RMP_KEY_CLR) { a380x_rmp_vhf_clear(r, radios, r->row, 1); r->clr_down = 1; r->clr_held = 0; }
    else if (key == A380X_RMP_KEY_UP || key == A380X_RMP_KEY_DOWN) a380x_rmp_mode_step(r, radios, r->row, key == A380X_RMP_KEY_UP);
    else if (key >= A380X_RMP_KEY_ADK1 && key <= A380X_RMP_KEY_ADK3) {
        int row = key - A380X_RMP_KEY_ADK1;
        if (row == r->row) a380x_rmp_swap(radios, row);
        else { a380x_rmp_entry_clear(&r->entry[r->row]); r->row = row; }
    } else if (key >= A380X_RMP_KEY_LSK1 && key <= A380X_RMP_KEY_LSK3) {
        int row = key - A380X_RMP_KEY_LSK1;
        if (row == r->row) a380x_rmp_lsk_vhf(r, radios, row);
        else { a380x_rmp_entry_clear(&r->entry[r->row]); r->row = row; }
    }
}
static inline void a380x_rmp_key_up(A380XRmp *r, int key) { if (key == A380X_RMP_KEY_CLR) r->clr_down = 0; }

/* Power (RMP 1/2 on DC ESS, RMP 3 on DC 1) and the OFF / BRT knob: RmpStateController. */
static inline void a380x_rmp_update(A380XRmp *r, A380XRmpRadios *radios, int powered, int failed, float dt)
{
    int on = r->brightness > 0.0f;
    if (dt < 0 || dt != dt) dt = 0;
    r->state = powered ? (failed ? (on ? A380X_RMP_ON_FAILED : A380X_RMP_OFF_FAILED) : (on ? A380X_RMP_ON : A380X_RMP_OFF_STANDBY))
                       : (on ? A380X_RMP_ON_FAILED : A380X_RMP_OFF_FAILED);
    if (r->clr_down) {
        r->clr_held += dt;
        if (r->clr_held >= A380X_RMP_CLR_HELD_S) {
            r->clr_down = 0;
            if (r->page == A380X_RMP_PAGE_SQWK) a380x_rmp_xpdr_clear(r, 0);
            else a380x_rmp_vhf_clear(r, radios, r->row, 0);
        }
    }
    if (r->xpdr_validate_left > 0 && (r->xpdr_validate_left -= dt) <= 0) a380x_rmp_xpdr_validate(r, radios);
}

static inline void a380x_rmp_radios_tick(A380XRmpRadios *radios, A380XRmp *rmps, int count, float dt)
{
    if (radios->ident_left > 0 && (radios->ident_left -= dt) <= 0) {
        radios->ident_left = 0;
        for (int i = 0; i < count; ++i)
            if (!strcmp(rmps[i].message, "TRANSMITTING IDENT")) rmps[i].message[0] = 0;
    }
}

static inline void a380x_rmp_init(A380XRmp *r, int index)
{
    memset(r, 0, sizeof(*r));
    r->tx = index == 2 ? 0 : 1;         /* RMP 1 / 2 transmit on VHF 1 */
    r->rx[0] = 1;
    r->brightness = 1.0f;
}
static inline void a380x_rmp_radios_init(A380XRmpRadios *radios)
{
    memset(radios, 0, sizeof(*radios));
    radios->active[0] = 0x1183000u; radios->standby[0] = 0x1219000u;
    radios->active[1] = 0x1215000u; radios->standby[1] = 0x1235000u;
    radios->active[2] = 0x1310250u; radios->standby[2] = 0x1225000u;
    radios->active_mode[2] = radios->standby_mode[2] = A380X_RMP_MODE_DATA;
    radios->xpdr_code = 2000; radios->xpdr_auto = 1;
}
#endif
