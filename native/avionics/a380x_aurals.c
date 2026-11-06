/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 cockpit aurals decision core (see a380x_aurals.h, docs/COCKPIT_AURALS.md). XPLM-free.
 */
#include "a380x_aurals.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

const int aur_ra_threshold_ft[AUR_RA_COUNT] = {2500, 2000, 1000, 500, 400, 300, 200, 100, 90, 80,
                                               70, 60, 50, 40, 30, 20, 10, 5};

#define AUR_FAR 1e9          /* invalid altitude: treated as above every threshold (arms them) */
#define AUR_QUEUE_LIMIT 1.0  /* s a waiting callout may wait for the voice before it is dropped */

/* A threshold re-arms only after climbing this far above it (no repeats from noise or bounces). */
static double ra_rearm(int threshold)
{
    return threshold >= 1000 ? 100.0 : threshold >= 100 ? 20.0 : 3.0 + 0.1 * threshold;
}

static int voice_priority(int s)
{
    switch (s) {
    case AUR_STALL: return 6;
    case AUR_MINIMUM: return 5;
    case AUR_HUNDRED_ABOVE: return 4;
    case AUR_RETARD: return 3;
    case AUR_V1: return 1;
    default: return s >= AUR_RA_2500 && s <= AUR_RA_5 ? 2 : 0;
    }
}

int aurals_is_voice(AurSample s) { return s >= AUR_RA_2500 && s <= AUR_V1; }

void aurals_init(Aurals *a)
{
    memset(a, 0, sizeof(*a));
    a->voice = -1;
    a->pending = -1;
    a->prev_seatbelt = -1;
    a->last_ra = a->last_baro = AUR_FAR;
}

void aurals_set_length(Aurals *a, AurSample s, double seconds)
{
    if ((int)s >= 0 && s < AUR_COUNT) a->seconds[s] = isfinite(seconds) && seconds > 0.0 ? seconds : 0.0;
}

void aurals_voice_silenced(Aurals *a)
{
    a->voice = -1;
    a->pending = -1;
}

static int voice_busy(const Aurals *a) { return a->voice >= 0 && a->clock < a->voice_until; }

static void start_voice(Aurals *a, int s, AurOutputs *out)
{
    out->start[s] = 1;
    a->voice = s;
    a->voice_until = a->clock + a->seconds[s] + 0.05;
}

/* Ask for a callout on the single voice channel. */
static void request_voice(Aurals *a, int s, AurOutputs *out)
{
    int p = voice_priority(s);
    if (a->seconds[s] <= 0.0) return;
    if (!voice_busy(a)) { start_voice(a, s, out); return; }
    if (p >= voice_priority(AUR_MINIMUM) && p > voice_priority(a->voice)) {
        if (out->start[a->voice]) out->start[a->voice] = 0;   /* requested this cycle, never started */
        else out->stop_voice = 1;
        start_voice(a, s, out);
        return;
    }
    if (a->pending < 0 || p >= voice_priority(a->pending)) {
        a->pending = s;
        a->pending_since = a->clock;
    }
}

/* silent: thresholds are still consumed, nothing is called (stall, or a minimums callout now). */
static void radio_altitude_callouts(Aurals *a, const AurInputs *in, int ra_ok, double ra, int silent,
                                    AurOutputs *out)
{
    double retard_height = in->ap_engaged ? 10.0 : 20.0;
    int i, fire = -1, descending = ra_ok && ra < a->last_ra - 1e-6;
    for (i = 0; i < AUR_RA_COUNT; ++i) {
        int t = aur_ra_threshold_ft[i];
        if (ra > t + ra_rearm(t)) a->ra_armed[i] = 1;
        else if (ra_ok && a->ra_armed[i] && ra <= t) {
            a->ra_armed[i] = 0;
            /* Several thresholds crossed in one step: only the lowest is called. RETARD replaces the
             * numeric callouts below its height for the rest of the approach. */
            if (descending && !in->on_ground && !(a->retard_called && t <= retard_height)) fire = i;
        }
    }
    if (fire >= 0 && !silent) request_voice(a, AUR_RA_2500 + fire, out);
}

/* Returns 1 when HUNDRED ABOVE or MINIMUM was called: it replaces that cycle's altitude callout. */
static int minimums_callouts(Aurals *a, const AurInputs *in, int ra_ok, int silent, AurOutputs *out)
{
    int called = 0;
    int kind = 0, alt_ok = 0;
    double ref = 0.0, alt = AUR_FAR, last = AUR_FAR;
    if (in->dh_valid && isfinite(in->dh_ft) && in->dh_ft > 0.0) { kind = 1; ref = in->dh_ft; }
    else if (in->mda_valid && isfinite(in->mda_ft)) { kind = 2; ref = in->mda_ft; }
    if (kind != a->min_kind || fabs(ref - a->min_ref) > 0.5) {
        a->min_kind = kind; a->min_ref = ref;
        a->hundred_armed = a->minimum_armed = 0;
    }
    if (kind == 1) { alt_ok = ra_ok; alt = in->ra_ft; last = a->last_ra; }
    else if (kind == 2) { alt_ok = in->baro_valid && isfinite(in->baro_ft); alt = in->baro_ft; last = a->last_baro; }
    if (!kind || !alt_ok) return 0;
    if (alt > ref + 150.0) a->hundred_armed = 1;
    if (alt > ref + 50.0) a->minimum_armed = 1;
    if (a->hundred_armed && alt <= ref + 100.0) {
        a->hundred_armed = 0;
        if (alt < last && !in->on_ground && !silent) { request_voice(a, AUR_HUNDRED_ABOVE, out); called = 1; }
    }
    if (a->minimum_armed && alt <= ref) {
        a->minimum_armed = 0;
        if (alt < last && !in->on_ground && !silent) { request_voice(a, AUR_MINIMUM, out); called = 1; }
    }
    return called;
}

/* RETARD at 20 ft (manual landing) or 10 ft (A/P engaged) while a thrust lever is above IDLE,
 * repeated until the levers reach IDLE (at most six calls). */
static void retard_callout(Aurals *a, const AurInputs *in, int ra_ok, double ra, int silent, AurOutputs *out)
{
    double height = in->ap_engaged ? 10.0 : 20.0;
    if (ra > height + 10.0) a->retard_armed = 1;
    if (ra > height + 30.0) a->retard_called = 0;
    if (a->retard_armed && ra_ok && ra <= height && ra < a->last_ra && !in->on_ground) {
        a->retard_armed = 0;
        if (in->thrust_above_idle) {
            a->retard_active = a->retard_called = 1;
            a->retard_count = 0; a->retard_next = a->clock;
        }
    }
    if (!a->retard_active) return;
    if (!in->thrust_above_idle || a->retard_count >= 6 || (ra_ok && ra > height + 30.0)) {
        a->retard_active = 0;
        return;
    }
    if (a->clock >= a->retard_next) {
        if (!silent) request_voice(a, AUR_RETARD, out);
        ++a->retard_count;
        a->retard_next = a->clock + a->seconds[AUR_RETARD] + 0.6;
    }
}

void aurals_update(Aurals *a, const AurInputs *in, AurOutputs *out)
{
    int ra_ok = in->ra_valid && isfinite(in->ra_ft);
    double ra = ra_ok ? in->ra_ft : AUR_FAR;
    int stall, seatbelt, minimums;
    memset(out, 0, sizeof(*out));
    if (isfinite(in->dt) && in->dt > 0.0 && in->dt < 1.0) a->clock += in->dt;
    if (!in->powered) {
        /* Unpowered FWS: no aurals; edges are consumed so nothing fires at power-up. */
        a->was_powered = 0; a->powered_for = 0.0;
        a->voice = a->pending = -1;
        a->retard_active = 0;
        a->prev_v1 = (unsigned char)(in->v1 != 0);
        a->prev_cchord_once = (unsigned char)(in->cchord_once != 0);
        a->prev_reversion = (unsigned char)(in->mode_reversion != 0);
        a->prev_seatbelt = -1;
        a->last_ra = ra;
        a->last_baro = in->baro_valid && isfinite(in->baro_ft) ? in->baro_ft : AUR_FAR;
        return;
    }
    if (!a->was_powered) { a->was_powered = 1; a->powered_for = 0.0; }
    else if (isfinite(in->dt) && in->dt > 0.0 && in->dt < 1.0) a->powered_for += in->dt;

    /* Stall: looped, silences the synthetic voice while it lasts. */
    stall = in->stall && a->seconds[AUR_STALL] > 0.0;
    if (stall) {
        if (voice_busy(a)) out->stop_voice = 1;
        a->voice = a->pending = -1;
        out->loop[AUR_STALL] = 1;
    }

    minimums = minimums_callouts(a, in, ra_ok, stall, out);
    radio_altitude_callouts(a, in, ra_ok, ra, stall || minimums, out);
    retard_callout(a, in, ra_ok, ra, stall, out);
    if (in->v1 && !a->prev_v1 && !stall) request_voice(a, AUR_V1, out);

    /* A waiting callout starts when the voice is free, unless it has become stale. */
    if (a->pending >= 0 && !voice_busy(a)) {
        int s = a->pending;
        a->pending = -1;
        if (!stall && a->clock - a->pending_since <= AUR_QUEUE_LIMIT) start_voice(a, s, out);
    }

    /* Attention getters (independent channels). */
    if (in->crc && a->seconds[AUR_CRC] > 0.0) out->loop[AUR_CRC] = 1;
    if (in->cavalry && a->seconds[AUR_CAVALRY] > 0.0) out->loop[AUR_CAVALRY] = 1;
    if (in->cchord_cont && a->seconds[AUR_CCHORD] > 0.0) out->loop[AUR_CCHORD] = 1;
    else if (in->cchord_once && !a->prev_cchord_once && a->seconds[AUR_CCHORD] > 0.0) out->start[AUR_CCHORD] = 1;
    if (in->mode_reversion && !a->prev_reversion && a->seconds[AUR_TRIPLE_CLICK] > 0.0)
        out->start[AUR_TRIPLE_CLICK] = 1;
    if (in->seatbelt_valid) {
        seatbelt = in->seatbelt_sign > 0;
        if (a->prev_seatbelt >= 0 && seatbelt != a->prev_seatbelt && a->powered_for >= 5.0 &&
            a->seconds[AUR_SEATBELT] > 0.0)
            out->start[AUR_SEATBELT] = 1;
        a->prev_seatbelt = seatbelt;
    }

    a->prev_v1 = (unsigned char)(in->v1 != 0);
    a->prev_cchord_once = (unsigned char)(in->cchord_once != 0);
    a->prev_reversion = (unsigned char)(in->mode_reversion != 0);
    a->last_ra = ra;
    a->last_baro = in->baro_valid && isfinite(in->baro_ft) ? in->baro_ft : AUR_FAR;
}

/* Strict RIFF/WAVE: PCM 16-bit mono, 8-48 kHz, at most 12 s; the samples are copied (malloc). */
int aurals_parse_wav(const unsigned char *b, size_t n, short **data, unsigned *bytes, int *rate)
{
    size_t at = 12;
    int have_format = 0;
    *data = NULL; *bytes = 0; *rate = 0;
    if (n < 44 || memcmp(b, "RIFF", 4) || memcmp(b + 8, "WAVE", 4)) return 0;
    while (at + 8 <= n) {
        uint32_t size = (uint32_t)b[at + 4] | (uint32_t)b[at + 5] << 8 | (uint32_t)b[at + 6] << 16 | (uint32_t)b[at + 7] << 24;
        const unsigned char *body = b + at + 8;
        if (size > n - at - 8) return 0;
        if (!memcmp(b + at, "fmt ", 4) && size >= 16) {
            int format = body[0] | body[1] << 8, channels = body[2] | body[3] << 8, bits = body[14] | body[15] << 8;
            *rate = (int)((uint32_t)body[4] | (uint32_t)body[5] << 8 | (uint32_t)body[6] << 16 | (uint32_t)body[7] << 24);
            if (format != 1 || channels != 1 || bits != 16 || *rate < 8000 || *rate > 48000) return 0;
            have_format = 1;
        } else if (!memcmp(b + at, "data", 4) && have_format) {
            if (size < 2 || size > (uint32_t)*rate * 2u * 12u) return 0;
            *data = (short *)malloc(size);
            if (!*data) return 0;
            memcpy(*data, body, size);
            *bytes = size & ~1u;
            return 1;
        }
        at += 8 + size + (size & 1u);
    }
    return 0;
}
