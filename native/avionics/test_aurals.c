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
    "20", "10", "5", "HUNDRED ABOVE", "MINIMUM", "RETARD", "V1", "TERRAIN AHEAD", "STALL", "PULL UP", "CRC", "CCHORD", "TRIPLE CLICK",
    "SEATBELT", "CAVALRY"};
static const double lengths[AUR_COUNT] = {
    1.35, 0.72, 0.80, 0.75, 0.75, 0.74, 0.70, 0.65, 0.60, 0.53, 0.80, 0.65, 0.55, 0.55, 0.57, 0.49, 0.59, 0.75,
    0.89, 0.65, 0.77, 0.85, 1.60, 2.85, 2.37, 5.00, 1.50, 0.55, 0.99, 1.20};

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

    /* Port stall warning: below VSW airborne (3 kt hysteresis), the flight-model AoA warning, never on
     * the ground or below 60 kt; it silences the voice like the FWS stall warning. */
    reset(); in = base(); in.ra_ft = 3000.0; in.cas_valid = 1; in.vsw_kt = 140.0;
    in.cas_kt = 150.0; step(&in); assert(!loops[AUR_STALL]);
    in.cas_kt = 139.0; for (int i = 0; i < 10; ++i) step(&in); assert(loops[AUR_STALL] == 10);
    in.cas_kt = 142.0; step(&in); assert(loops[AUR_STALL] == 11);          /* still inside the hysteresis */
    in.cas_kt = 143.5; step(&in); assert(loops[AUR_STALL] == 11);          /* cleared above VSW + 3 kt */
    in.aoa_stall_warning = 1; step(&in); assert(loops[AUR_STALL] == 12);   /* flight-model AoA warning */
    in.on_ground = 1; step(&in); assert(loops[AUR_STALL] == 12);           /* never on the ground */
    in.on_ground = 0; in.aoa_stall_warning = 0; in.cas_kt = 50.0; step(&in); assert(loops[AUR_STALL] == 12);
    printf("%-34s below VSW, hysteresis, AoA warning, ground and 60 kt inhibits\n", "port stall warning");

    /* GPWS mode 1: 3000 fpm sink, PULL UP below 0.4*3000-410 = 790 ft RA (landing flaps: mode 2's
     * landing envelope only starts at 775 ft, so this isolates mode 1); nothing at 800 fpm; it silences
     * the altitude callouts. Mode 2: 6000 fpm terrain closure (rising terrain, aircraft level) at
     * 2000 ft RA, clean: PULL UP after 0.2 s. */
    reset(); in = base(); in.vs_fpm = -3000.0; in.cas_valid = 1; in.cas_kt = 250.0; in.landing_flaps = 1;
    for (double ra = 1500.0; ra > 795.0; ra -= 50.0 * in.dt) { in.ra_ft = ra; step(&in); }
    assert(!loops[AUR_PULL_UP]);
    for (double ra = 789.0; ra > 600.0; ra -= 50.0 * in.dt) { in.ra_ft = ra; step(&in); }
    assert(loops[AUR_PULL_UP] > 100 && !strstr(said, "500"));
    printf("%-34s mode 1 PULL UP from 790 ft at 3000 fpm (%d frames), callouts silenced: '%s'\n", "GPWS", loops[AUR_PULL_UP], said);
    reset(); in = base(); in.vs_fpm = -800.0; in.cas_valid = 1; in.cas_kt = 160.0;
    for (double ra = 1500.0; ra > 100.0; ra -= 13.3 * in.dt) { in.ra_ft = ra; step(&in); }
    assert(!loops[AUR_PULL_UP]);
    reset(); in = base(); in.vs_fpm = 0.0; in.cas_valid = 1; in.cas_kt = 250.0;
    for (int i = 0; i < 30; ++i) { in.ra_ft = 2000.0; step(&in); }
    for (double ra = 2000.0; ra > 900.0; ra -= 100.0 * in.dt) { in.ra_ft = ra; step(&in); }
    assert(loops[AUR_PULL_UP] > 0);
    printf("%-34s mode 2 terrain closure 6000 fpm: PULL UP (%d frames); no warning on a normal approach\n", "GPWS", loops[AUR_PULL_UP]);

    /* TAWS look-ahead warning ("TERRAIN AHEAD, PULL UP"): the PULL UP loop while it lasts, above the
     * callouts; the stall keeps priority. GPWS SYS OFF inhibits modes 1/2, not the TAWS warning. */
    reset(); in = base(); in.ra_ft = 3000.0; in.cas_valid = 1; in.cas_kt = 250.0;
    in.taws_warning = 1; for (int i = 0; i < 60; ++i) step(&in);
    assert(loops[AUR_PULL_UP] == 60 && core.gpws_pull_up && !core.gpws_mode_pull_up);
    in.stall = 1; for (int i = 0; i < 10; ++i) step(&in);
    assert(loops[AUR_PULL_UP] == 60 && loops[AUR_STALL] == 10);
    in.stall = 0; in.taws_warning = 0; step(&in); assert(loops[AUR_PULL_UP] == 60 && !core.gpws_pull_up);
    reset(); in = base(); in.vs_fpm = -3000.0; in.cas_valid = 1; in.cas_kt = 250.0; in.landing_flaps = 1; in.gpws_inhibit = 1;
    for (double ra = 1000.0; ra > 600.0; ra -= 50.0 * in.dt) { in.ra_ft = ra; step(&in); }
    assert(!loops[AUR_PULL_UP]);
    in.taws_warning = 1; step(&in); assert(loops[AUR_PULL_UP] == 1);
    printf("%-34s TAWS warning loops PULL UP; stall first; GPWS SYS OFF inhibits modes 1/2 only\n", "TAWS warning");

    /* TAWS caution: TERRAIN AHEAD at the start, then every 7 s while it lasts; it interrupts an altitude
     * callout; nothing during the warning; nothing at all without the (optional) recording. */
    reset(); in = base(); in.ra_ft = 3000.0; step(&in);
    in.ra_ft = 2490.0; step(&in); step(&in); assert(starts[0] == 1);  /* "2500" is playing */
    in.taws_caution = 1;
    for (int i = 0; i < 30 * 15; ++i) step(&in);                      /* 15 s of caution */
    assert(starts[AUR_TERRAIN_AHEAD] == 3);                           /* 0, 7, 14 s */
    in.taws_caution = 0; for (int i = 0; i < 30 * 8; ++i) step(&in);
    assert(starts[AUR_TERRAIN_AHEAD] == 3);
    in.taws_caution = 1; in.taws_warning = 1; for (int i = 0; i < 30 * 8; ++i) step(&in);
    assert(starts[AUR_TERRAIN_AHEAD] == 3 && loops[AUR_PULL_UP] == 240);
    expect("TAWS caution (with recording)", "2500,TERRAIN AHEAD,TERRAIN AHEAD,TERRAIN AHEAD");
    reset(); aurals_set_length(&core, AUR_TERRAIN_AHEAD, 0.0); in = base(); in.ra_ft = 5000.0; in.taws_caution = 1;
    for (int i = 0; i < 30 * 15; ++i) step(&in);
    assert(!starts[AUR_TERRAIN_AHEAD]);
    expect("TAWS caution (no recording)", "");

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

    /* Landing capability downgrade (a380x_autoland.inc level): a rising edge clicks once, the held
     * level does not repeat, a new downgrade clicks again; with a reversion in the same frame: one. */
    reset(); in = base(); in.ra_ft = 1500.0;
    in.capability_downgrade = 1; for (int i = 0; i < 90; ++i) step(&in); in.capability_downgrade = 0; step(&in);
    assert(starts[AUR_TRIPLE_CLICK] == 1);
    in.capability_downgrade = 1; step(&in); in.capability_downgrade = 0; step(&in);
    assert(starts[AUR_TRIPLE_CLICK] == 2);
    in.capability_downgrade = 1; in.mode_reversion = 1; step(&in);
    assert(starts[AUR_TRIPLE_CLICK] == 3);
    in.capability_downgrade = 0; in.mode_reversion = 0; step(&in);
    printf("%-34s triple click on each capability downgrade edge (%d), one for a coincident reversion\n",
           "capability downgrade", starts[AUR_TRIPLE_CLICK]);

    /* Seat belts: no chime in the first 5 s after power-up (state settles), then on each change. */
    reset(); in = base(); in.ra_ft = 0.0; in.on_ground = 1; in.seatbelt_sign = 0;
    step(&in); in.seatbelt_sign = 2; step(&in);                       /* during settle: silent */
    for (int i = 0; i < 160; ++i) step(&in);
    in.seatbelt_sign = 0; step(&in); in.seatbelt_sign = 2; step(&in); in.seatbelt_sign = 1; step(&in);
    assert(starts[AUR_SEATBELT] == 2);
    printf("%-34s %d chimes (off, on; AUTO<->ON is not a change of the sign)\n", "seat belts", starts[AUR_SEATBELT]);

    /* Unpowered FWS: nothing at all. At power-up the pulses/edges that were already high do not fire;
     * a CRC request (a level) sounds as soon as the FWS is powered. */
    reset(); in = base(); in.powered = 0; in.crc = 1; in.v1 = 1; in.mode_reversion = 1; in.capability_downgrade = 1;
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
            "retard", "v1", NULL /* terrain_ahead: optional, user-supplied */, "stall", "pull_up_new", "crc_master_warning", "cchord", "triple_click", "seatbelt_chime",
            "cavalry_charge"};
        const char *dir = getenv("A380X_AURALS_DIR");
        int ok = 0;
        for (int i = 0; dir && i < AUR_COUNT; ++i) {
            char path[1024]; FILE *f; long n; unsigned char *b; short *d; unsigned bytes; int rate;
            if (!files[i]) continue;
            snprintf(path, sizeof(path), "%s/%s.wav", dir, files[i]);
            f = fopen(path, "rb"); assert(f);
            fseek(f, 0, SEEK_END); n = ftell(f); fseek(f, 0, SEEK_SET);
            b = malloc((size_t)n); assert(b && fread(b, 1, (size_t)n, f) == (size_t)n); fclose(f);
            assert(aurals_parse_wav(b, (size_t)n, &d, &bytes, &rate) && bytes > 0 && rate >= 8000);
            free(d); free(b); ++ok;
        }
        if (dir) printf("%-34s %d/%d packaged samples accepted by the reader (terrain_ahead optional)\n", "samples", ok, AUR_COUNT - 1);
    }
    printf("aurals: all scenarios passed\n");
    return 0;
}
