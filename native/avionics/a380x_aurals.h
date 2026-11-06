/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * A380 cockpit aurals: radio-altitude auto callouts, minimums, RETARD, V1, stall, CRC, C-chord,
 * triple click and the seat-belt chime. XPLM-free decision core (docs/COCKPIT_AURALS.md); the
 * X-Plane adapter is a380x_aurals.inc, which loads the samples and plays them on the interior bus.
 *
 * Sources of the requests:
 *   FWS (a380x_fws*.c, ported FwsCore): CRC (auralCrcActive), C-chord once / continuous (altitude
 *   alert), the V1 callout pulse and the stall warning. The radio-altitude callouts, the minimums
 *   callouts and RETARD are decided here from the captain's radio altimeter, the FMS minimums
 *   (PERF APPR BARO / RADIO), A/P engagement and the thrust lever angles. The triple click follows
 *   the FMA mode reversion; the chime follows the FASTEN SEAT BELTS sign. The A/P disconnect cavalry
 *   charge follows the FWS request (one 1.2 s cycle for a voluntary disconnect, repeated until
 *   acknowledged for an involuntary one). GPWS modes 1 (excessive descent rate) and 2 (excessive terrain
 *   closure rate), ported from FBW A380 LegacyGpws.ts, loop the TERRAIN / PULL UP voice (the EFB
 *   chooses the new or the old A380 recording; the adapter plays the chosen one).
 *
 * The synthetic voice is one channel: one callout at a time, by priority (STALL > PULL UP > MINIMUM >
 * HUNDRED ABOVE > RETARD > altitude callouts > V1). STALL and MINIMUM interrupt a lower callout;
 * the others wait for the current one and are dropped if they cannot start within 0.5 s.
 */
#ifndef A380X_AURALS_H
#define A380X_AURALS_H

#include <stddef.h>

typedef enum {
    AUR_RA_2500, AUR_RA_2000, AUR_RA_1000, AUR_RA_500, AUR_RA_400, AUR_RA_300, AUR_RA_200, AUR_RA_100,
    AUR_RA_90, AUR_RA_80, AUR_RA_70, AUR_RA_60, AUR_RA_50, AUR_RA_40, AUR_RA_30, AUR_RA_20, AUR_RA_10, AUR_RA_5,
    AUR_HUNDRED_ABOVE, AUR_MINIMUM, AUR_RETARD, AUR_V1,   /* synthetic voice, one at a time */
    AUR_STALL,                                             /* voice + cricket, looped */
    AUR_PULL_UP,                                           /* GPWS terrain / PULL UP voice, looped */
    AUR_CRC, AUR_CCHORD, AUR_TRIPLE_CLICK, AUR_SEATBELT,   /* attention getters */
    AUR_CAVALRY,                                           /* A/P disconnect cavalry charge, looped */
    AUR_COUNT
} AurSample;

#define AUR_RA_COUNT 18
extern const int aur_ra_threshold_ft[AUR_RA_COUNT];

typedef struct {
    double dt;                       /* s since the last update */
    int powered;                     /* FWS powered: every aural is an FWS output */
    int ra_valid; double ra_ft;      /* captain radio altitude */
    int on_ground;
    int baro_valid; double baro_ft;  /* captain baro-corrected altitude (BARO minimum) */
    int dh_valid; double dh_ft;      /* RADIO minimum (> 0) */
    int mda_valid; double mda_ft;    /* BARO minimum */
    int ap_engaged;
    int thrust_above_idle;           /* any thrust lever above IDLE */
    int crc, cchord_once, cchord_cont, v1, stall;   /* FWS requests; v1 and cchord_once are pulses */
    int cavalry;                                    /* FWS cavalry charge (A/P OFF): level */
    /* Port stall warning (no alpha protection in this port, so the FWS normal-law gate never warns):
     * airborne above 60 kt and below VSW (3 kt hysteresis), or the flight model's stall-warning AoA. */
    int cas_valid; double cas_kt, vsw_kt;
    int aoa_stall_warning;
    /* GPWS modes 1 and 2 (FBW A380 LegacyGpws.ts): inertial vertical speed, landing flaps, gear. */
    double vs_fpm;
    int landing_flaps, gear_down;
    int mode_reversion;
    int seatbelt_valid, seatbelt_sign;              /* sign 0 off, > 0 on */
} AurInputs;

typedef struct {
    unsigned char start[AUR_COUNT];  /* one-shot samples to start now */
    unsigned char loop[AUR_COUNT];   /* looped samples that must be playing (CRC, C-chord, stall) */
    unsigned char stop_voice;        /* stop the voice sample that is playing (interrupted) */
} AurOutputs;

typedef struct {
    double clock;
    double seconds[AUR_COUNT];       /* sample lengths; 0 = sample missing, never requested */
    int was_powered;
    double powered_for;              /* s since power-up (seat-belt chime settle) */
    unsigned char ra_armed[AUR_RA_COUNT];
    int last_alt_valid; double last_ra, last_baro;
    int min_kind; double min_ref;    /* 0 none, 1 RADIO (dh), 2 BARO (mda) */
    unsigned char hundred_armed, minimum_armed;
    unsigned char retard_armed, retard_called; int retard_count; double retard_next; int retard_active;
    int voice; double voice_until;   /* sample playing, -1 none */
    int pending; double pending_since;
    unsigned char prev_v1, prev_cchord_once, prev_reversion;
    int prev_seatbelt;
    int port_stall;
    double gpws_ra_rate, gpws_last_ra, gpws_mode2_time; int gpws_last_ra_valid, gpws_pull_up;
} Aurals;

void aurals_init(Aurals *a);
/* Sample length in seconds (0 = not loaded). */
void aurals_set_length(Aurals *a, AurSample s, double seconds);
void aurals_update(Aurals *a, const AurInputs *in, AurOutputs *out);
/* The adapter calls this when it could not start or had to stop the voice (paused, outside view). */
void aurals_voice_silenced(Aurals *a);
int aurals_is_voice(AurSample s);
/* Strict RIFF/WAVE reader (PCM 16-bit mono, 8-48 kHz, <= 12 s); *data is malloc'd. 1 = ok. */
int aurals_parse_wav(const unsigned char *bytes, size_t length, short **data, unsigned *data_bytes, int *rate);

#endif
