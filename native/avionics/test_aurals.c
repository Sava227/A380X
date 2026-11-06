/* SPDX-License-Identifier: GPL-3.0-or-later
 * Scenario tests of the cockpit aurals core (a380x_aurals.c): approaches, takeoff, go-around,
 * minimums, RETARD, stall, FWS chimes, mode reversion, seat belts, power.
 */
#include "a380x_aurals.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *const names[AUR_COUNT] = {
    "2500", "2000", "1000", "500", "400", "300", "200", "100", "90", "80", "70", "60", "50", "40", "30",
    "20", "10", "5", "HUNDRED ABOVE", "MINIMUM", "RETARD", "V1", "STALL", "CRC", "CCHORD", "TRIPLE CLICK",
    "SEATBELT", "CAVALRY"};
static const double lengths[AUR_COUNT] = {
    1.35, 0.72, 0.80, 0.75, 0.75, 0.74, 0.70, 0.65, 0.60, 0.53, 0.80, 0.65, 0.55, 0.55, 0.57, 0.49, 0.59, 0.75,
    0.89, 0.65, 0.77, 0.85, 2.85, 5.00, 1.50, 0.55, 0.99, 1.20};

static Aurals core;
static char said[2048];
static int loops[AUR_COUNT], starts[AUR_COUNT];

static void reset(void)
{
    aurals_init(&core);
    for (int i = 0; i < AUR_COUNT; ++i) aurals_set_length(&core, (AurSample)i, lengths[i]);
    said[0] = 0;
    memset(loops, 0, sizeof(loops)); memset(starts, 0, sizeof(starts));
}

static void step(AurInputs *in)
{
    AurOutputs out;
    aurals_update(&core, in, &out);
    for (int i = 0; i < AUR_COUNT; ++i) {
        if (out.start[i]) {
            ++starts[i];
            if (aurals_is_voice((AurSample)i)) {
                if (said[0]) strcat(said, ",");
                strcat(said, names[i]);
            }
        }
        if (out.loop[i]) ++loops[i];
    }
}

static AurInputs base(void)
{
    AurInputs in;
    memset(&in, 0, sizeof(in));
    in.dt = 1.0 / 30.0; in.powered = 1; in.ra_valid = 1; in.baro_valid = 1; in.seatbelt_valid = 1;
    return in;
}

/* Descend at rate ft/s from `from` to the ground; thrust goes to IDLE at idle_at ft (or never: -1). */
static void approach(AurInputs *in, double from, double rate, double idle_at)
{
    double ra = from;
    in->thrust_above_idle = 1;
    while (ra > 0.0) {
        in->ra_ft = ra; in->baro_ft = ra + 400.0; in->on_ground = 0;
        if (idle_at >= 0.0 && ra <= idle_at) in->thrust_above_idle = 0;
        step(in);
        ra -= rate * in->dt;
    }
    in->ra_ft = 0.0; in->on_ground = 1;
    for (int i = 0; i < 90; ++i) step(in);   /* 3 s of rollout */
}

static void expect(const char *scenario, const char *wanted)
{
    printf("%-34s %s\n", scenario, said);
    if (strcmp(said, wanted)) { printf("  expected %s\n", wanted); assert(0); }
}

int main(void)
{
    AurInputs in;

    /* Manual landing, no minimums, thrust to IDLE at 12 ft: RETARD at 20 ft until IDLE. */
    reset(); in = base();
    approach(&in, 3000.0, 12.0, 12.0);
    expect("manual landing", "2500,2000,1000,500,400,300,200,100,90,80,70,60,50,40,30,20,RETARD");

    /* Autoland (A/P engaged), RADIO minimum 200 ft: HUNDRED ABOVE replaces 300, MINIMUM replaces 200;
     * RETARD at 10 ft replaces "5"; thrust reaches IDLE at 4 ft. */
    reset(); in = base(); in.ap_engaged = 1; in.dh_valid = 1; in.dh_ft = 200.0;
    approach(&in, 3000.0, 12.0, 4.0);
    expect("autoland DH 200", "2500,2000,1000,500,400,HUNDRED ABOVE,MINIMUM,100,90,80,70,60,50,40,30,20,10,RETARD");

    /* BARO minimum (MDA) 900 ft baro = 500 ft RA here: MINIMUM replaces 500 (HUNDRED ABOVE at 600 baro). */
    reset(); in = base(); in.mda_valid = 1; in.mda_ft = 900.0;
    approach(&in, 1500.0, 12.0, 15.0);
    expect("MDA approach", "1000,HUNDRED ABOVE,MINIMUM,400,300,200,100,90,80,70,60,50,40,30,20,RETARD");

    /* Takeoff: climb to 3000 ft, no callouts; V1 pulse gives V1. */
    reset(); in = base(); in.on_ground = 1; in.ra_ft = 0.0;
    for (int i = 0; i < 30; ++i) step(&in);
    in.v1 = 1; step(&in); in.v1 = 0; step(&in);
    in.on_ground = 0;
    for (double ra = 0.0; ra < 3000.0; ra += 25.0 * in.dt) { in.ra_ft = ra; step(&in); }
    expect("takeoff", "V1");

    /* Go-around at 150 ft, climb to 1500, second approach: callouts again from 1000 ft. */
    reset(); in = base(); in.thrust_above_idle = 1;
    for (double ra = 1800.0; ra > 150.0; ra -= 15.0 * in.dt) { in.ra_ft = ra; step(&in); }
    for (double ra = 150.0; ra < 1500.0; ra += 20.0 * in.dt) { in.ra_ft = ra; step(&in); }
    approach(&in, 1500.0, 12.0, 12.0);
    expect("go-around", "1000,500,400,300,200,1000,500,400,300,200,100,90,80,70,60,50,40,30,20,RETARD");

    /* RA jump through several thresholds in one step: only the lowest is called. */
    reset(); in = base(); in.ra_ft = 2600.0; step(&in); in.ra_ft = 1900.0; step(&in);
    for (int i = 0; i < 60; ++i) step(&in);
    expect("RA jump", "2000");

    /* RA invalid above its range, valid again at 2450 while descending: 2500. */
    reset(); in = base(); in.ra_valid = 0; in.ra_ft = 0.0; step(&in);
    in.ra_valid = 1; in.ra_ft = 2450.0; step(&in); in.ra_ft = 2440.0; step(&in);
    expect("RA becomes valid", "2500");

    /* Stall silences the voice and loops; a bounce near the ground does not repeat callouts. */
    reset(); in = base(); in.ra_ft = 700.0; step(&in); in.ra_ft = 450.0; step(&in);
    in.stall = 1; for (int i = 0; i < 30; ++i) { in.ra_ft -= 5.0; step(&in); }
    in.stall = 0; step(&in);
    assert(loops[AUR_STALL] == 30);
    expect("stall", "500");

    /* FWS attention getters: CRC loops, C-chord once / continuous, triple click on reversion. */
    reset(); in = base(); in.ra_ft = 5000.0;
    in.crc = 1; for (int i = 0; i < 10; ++i) step(&in); in.crc = 0; step(&in);
    in.cchord_once = 1; step(&in); step(&in); in.cchord_once = 0; step(&in);
    in.cchord_cont = 1; for (int i = 0; i < 5; ++i) step(&in); in.cchord_cont = 0; step(&in);
    in.mode_reversion = 1; for (int i = 0; i < 5; ++i) step(&in); in.mode_reversion = 0; step(&in);
    in.cavalry = 1; for (int i = 0; i < 36; ++i) step(&in); in.cavalry = 0; step(&in);   /* 1.2 s A/P OFF */
    assert(loops[AUR_CRC] == 10 && starts[AUR_CCHORD] == 1 && loops[AUR_CCHORD] == 5 && starts[AUR_TRIPLE_CLICK] == 1);
    assert(loops[AUR_CAVALRY] == 36);
    printf("%-34s CRC loop %d, C-chord once %d + loop %d, triple click %d, cavalry charge loop %d\n",
           "attention getters", loops[AUR_CRC], starts[AUR_CCHORD], loops[AUR_CCHORD], starts[AUR_TRIPLE_CLICK],
           loops[AUR_CAVALRY]);

    /* Seat belts: no chime in the first 5 s after power-up (state settles), then on each change. */
    reset(); in = base(); in.ra_ft = 0.0; in.on_ground = 1; in.seatbelt_sign = 0;
    step(&in); in.seatbelt_sign = 2; step(&in);                       /* during settle: silent */
    for (int i = 0; i < 160; ++i) step(&in);
    in.seatbelt_sign = 0; step(&in); in.seatbelt_sign = 2; step(&in); in.seatbelt_sign = 1; step(&in);
    assert(starts[AUR_SEATBELT] == 2);
    printf("%-34s %d chimes (off, on; AUTO<->ON is not a change of the sign)\n", "seat belts", starts[AUR_SEATBELT]);

    /* Unpowered FWS: nothing at all. At power-up the pulses/edges that were already high do not fire;
     * a CRC request (a level) sounds as soon as the FWS is powered. */
    reset(); in = base(); in.powered = 0; in.crc = 1; in.v1 = 1; in.mode_reversion = 1;
    approach(&in, 1200.0, 12.0, -1.0);
    assert(!loops[AUR_CRC] && !starts[AUR_V1] && !starts[AUR_TRIPLE_CLICK]);
    in.powered = 1; step(&in);
    assert(loops[AUR_CRC] == 1 && !starts[AUR_V1] && !starts[AUR_TRIPLE_CLICK]);
    expect("unpowered", "");

    /* Missing samples are never requested. */
    reset(); in = base(); aurals_set_length(&core, AUR_RA_500, 0.0);
    approach(&in, 600.0, 12.0, 12.0);
    expect("missing 500 sample", "400,300,200,100,90,80,70,60,50,40,30,20,RETARD");

    /* The packaged samples: every file the adapter loads must pass its reader. */
    {
        static const char *const files[AUR_COUNT] = {
            "callout_2500", "callout_2000", "callout_1000", "callout_500", "callout_400", "callout_300",
            "callout_200", "callout_100", "callout_90", "callout_80", "callout_70", "callout_60", "callout_50",
            "callout_40", "callout_30", "callout_20", "callout_10", "callout_5", "hundred_above", "minimum",
            "retard", "v1", "stall", "crc_master_warning", "cchord", "triple_click", "seatbelt_chime",
            "cavalry_charge"};
        const char *dir = getenv("A380X_AURALS_DIR");
        int ok = 0;
        for (int i = 0; dir && i < AUR_COUNT; ++i) {
            char path[1024]; FILE *f; long n; unsigned char *b; short *d; unsigned bytes; int rate;
            snprintf(path, sizeof(path), "%s/%s.wav", dir, files[i]);
            f = fopen(path, "rb"); assert(f);
            fseek(f, 0, SEEK_END); n = ftell(f); fseek(f, 0, SEEK_SET);
            b = malloc((size_t)n); assert(b && fread(b, 1, (size_t)n, f) == (size_t)n); fclose(f);
            assert(aurals_parse_wav(b, (size_t)n, &d, &bytes, &rate) && bytes > 0 && rate >= 8000);
            free(d); free(b); ++ok;
        }
        if (dir) printf("%-34s %d/%d packaged samples accepted by the reader\n", "samples", ok, AUR_COUNT);
    }
    printf("aurals: all scenarios passed\n");
    return 0;
}
