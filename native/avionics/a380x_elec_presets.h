/* SPDX-License-Identifier: GPL-3.0-or-later
 * flyPad aircraft-state presets (FBW aircraft_preset_procedures.xml, A380X): simulator-
 * independent step tables and runner.
 *
 * Every step drives one aircraft "item" (a native pushbutton, switch or simulator control,
 * see A380XPresetItem) to a value, or waits until an item reports a value. The host (the
 * X-Plane adapter, or a unit-test mock) implements get/set for the items, so the tables
 * act on native ELEC pb states and native modules, never on stock X-Plane quick commands.
 *
 * Step types follow FBW: STEP (normal + expedited), PROC (normal only), NOEX (keeps its
 * delay in expedited mode), EXON (expedited only), COND (wait, both modes), NCON (wait,
 * normal only), ECON (wait, expedited only). An action step whose item already has the
 * target value is skipped without its delay (FBW "condition already true").
 */
#ifndef A380X_ELEC_PRESETS_H
#define A380X_ELEC_PRESETS_H
#ifdef __cplusplus
extern "C" {
#endif

typedef enum { A380X_PST_STEP, A380X_PST_PROC, A380X_PST_NOEX, A380X_PST_EXON,
               A380X_PST_COND, A380X_PST_NCON, A380X_PST_ECON } A380XPresetStepType;

typedef enum {
    A380X_PI_BAT,           /* index 0 BAT 1, 1 BAT 2, 2 ESS, 3 APU; value 1 AUTO */
    A380X_PI_EXT_PB,        /* index 0..3: EXT n ON */
    A380X_PI_GPU,           /* ground power connected */
    A380X_PI_APU_MASTER,
    A380X_PI_APU_START,     /* set 1 = press START; get 1 = START latched or APU AVAIL */
    A380X_PI_APU_AVAIL,     /* read-only */
    A380X_PI_APU_BLEED,
    A380X_PI_AC1_POWERED,   /* read-only */
    A380X_PI_ADIRS_MODE,    /* index 0..2; value 0 OFF, 1 NAV, 2 ATT */
    A380X_PI_ADIRS_ALIGNED, /* read-only, index 0..2 */
    A380X_PI_ADIRS_ALIGN_NOW,
    A380X_PI_CREW_OXY,
    A380X_PI_STROBE,        /* 0 OFF, 1 AUTO, 2 ON */
    A380X_PI_NAV_LOGO,
    A380X_PI_BEACON,
    A380X_PI_SEATBELTS,
    A380X_PI_NO_SMOKING,    /* 0 OFF, 1 AUTO, 2 ON */
    A380X_PI_FUEL_PUMP,     /* index 0..3: feed tank n main pump */
    A380X_PI_COCKPIT_DOOR,  /* 1 locked */
    A380X_PI_ENG_MASTER,    /* index 0..3: ENG MASTER (the FADEC starts it with IGN START selected) */
    A380X_PI_ENG_RUNNING,   /* read-only, index 0..3 */
    A380X_PI_ENG_STOPPED,   /* read-only, index 0..3: N1 < 3 % */
    A380X_PI_FLAPS,         /* detent index (0 UP, 1 = 1+F) */
    A380X_PI_FLAPS_AT,      /* read-only: flaps/slats at the detent */
    A380X_PI_SPOILER_ARM,
    A380X_PI_RUDDER_TRIM_ZERO,
    A380X_PI_NOSE_LIGHT,    /* nose switch at TAXI or T.O (set 0: OFF, set 1: TAXI unless at T.O) */
    A380X_PI_NOSE_TO_LIGHT, /* nose switch at T.O (set 1: T.O, set 0: back to TAXI) */
    A380X_PI_RWY_TURNOFF,
    A380X_PI_LANDING_LIGHTS,
    A380X_PI_AUTOBRAKE_RTO,
    A380X_PI_TERR_ND,
    A380X_PI_TRANSPONDER,   /* 0 standby, 1 auto/on */
    A380X_PI_ENG_MODE_SEL,  /* ENG START selector: 0 CRANK, 1 NORM, 2 IGN START */
    A380X_PI_TO_CONFIG,     /* ECP T.O CONFIG held (1) / released (0) */
    A380X_PI_COUNT
} A380XPresetItem;

typedef struct {
    const char *name;
    unsigned char type, item, index;
    signed char value;
    unsigned short delay_ms;
} A380XPresetStep;

typedef enum {
    A380X_PROC_POWERUP_ON, A380X_PROC_POWERUP_OFF, A380X_PROC_PUSHBACK_ON, A380X_PROC_PUSHBACK_OFF,
    A380X_PROC_TAXI_ON, A380X_PROC_TAXI_OFF, A380X_PROC_TAKEOFF_ON, A380X_PROC_TAKEOFF_OFF,
    A380X_PROC_COUNT
} A380XPresetProcedure;

/* FBW preset ids 1..5. */
typedef enum {
    A380X_PRESET_NONE = 0, A380X_PRESET_COLD_DARK = 1, A380X_PRESET_TURNAROUND = 2,
    A380X_PRESET_PUSHBACK = 3, A380X_PRESET_TAXI = 4, A380X_PRESET_TAKEOFF = 5
} A380XPresetId;
extern const char *const a380x_preset_names[6];

typedef struct {
    int (*get)(void *ctx, int item, int index);
    void (*set)(void *ctx, int item, int index, int value);
    void *ctx;
} A380XPresetHost;

typedef struct {
    int preset, expedited, running, done;
    int proc_list[4], proc_count, proc_pos, step;
    double wait_s;        /* remaining delay before the next step */
    int steps_total, steps_done;
    const char *current;  /* name of the step in progress */
} A380XPresetRunner;

const A380XPresetStep *a380x_preset_procedure(int proc, int *count);
/* Ordered procedures of a preset (FBW AircraftPresets): returns the count (<= 4). */
int a380x_preset_procedures(int preset, int out[4]);
void a380x_preset_start(A380XPresetRunner *r, int preset, int expedited);
void a380x_preset_cancel(A380XPresetRunner *r);
/* Advances by dt simulator seconds; returns 1 while running. */
int a380x_preset_update(A380XPresetRunner *r, const A380XPresetHost *host, double dt);

#ifdef __cplusplus
}
#endif
#endif
