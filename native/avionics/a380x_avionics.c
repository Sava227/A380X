/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Aircraft-local FBW-derived avionics adapter for X-Plane 12.
 *
 * The display state machine and FMA priority are translated from the pinned
 * FlyByWire A380 source.  X-Plane supplies raw air/attitude/engine/failure
 * inputs only; stock EFIS selector positions are used as physical ND controls,
 * while stock map, flight-director, VNAV and autopilot mode datarefs are not
 * consumed. The plugin draws both PFDs and both NDs directly into
 * the aircraft's 2048x2048 cockpit panel texture.  The PFD geometry and
 * symbology are a native OpenGL translation of the pinned open-source
 * FlyByWire A380 PFD; no FlightFactor code, binary, texture, font, or dataref
 * is linked or packaged.
 * Existing displays occupy the left 1024-pixel atlas column. The EWD has a
 * full-resolution 768 x 1024 region in the right column.
 * The two MFDs are plugin cockpit devices (768 x 1024 each, X-Plane >= 12.1,
 * a380x_mfd_adapter.inc) driven by the native FMS (a380x_fms_adapter.inc).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#if APL
#include <OpenGL/gl.h>
#elif IBM
#include <windows.h>
#include <GL/gl.h>
#else
#include <GL/gl.h>
#endif

#define XPLM301 1
#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMSound.h"
#include "XPLMUtilities.h"
#include "XPLMNavigation.h"
#include "XPLMScenery.h"

#include "a380x_avionics_core.h"
#include "a380x_efb.h"
#include "a380x_guidance.h"
#include "a380x_pfd_data.h"
#include "a380x_mfd.h"
#include "a380x_mfd_page_data.h"
#include "a380x_fms.h"
#include "a380x_navdata.h"
#include "a380x_fms_fpln.h"
#include "a380x_elec_power.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ARRAY_LEN(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define PANEL_W 2048
#define PANEL_H 2048

typedef struct { float r, g, b; } Color;
static const Color WHITE = {0.92f, 0.96f, 1.0f};
static const Color GREEN = {0.24f, 0.95f, 0.36f};
static const Color CYAN = {0.10f, 0.88f, 0.96f};
static const Color AMBER = {1.0f, 0.65f, 0.0f};
static const Color RED = {1.0f, 0.12f, 0.08f};
static const Color YELLOW = {1.0f, 0.82f, 0.0f};
static const Color GREY = {0.48f, 0.55f, 0.60f};
static const Color BLACK = {0.004f, 0.008f, 0.012f};
/* Pinned A380 PFD style.scss: #4f5467, #005bc4, #65290a. */
static const Color TAPE_BACKGROUND = {79.0f/255.0f,84.0f/255.0f,103.0f/255.0f};
static const Color SKY = {0.0f,91.0f/255.0f,196.0f/255.0f};
static const Color EARTH = {101.0f/255.0f,41.0f/255.0f,10.0f/255.0f};

typedef struct {
    XPLMDataRef ias, altitude, vertical_speed, pitch, roll, heading, radio_altitude, mach, decision_height;
    XPLMDataRef fail_ias, fail_altitude, fail_vertical_speed, fail_attitude;
    XPLMDataRef fail_heading, fail_radio_altitude, fail_efis;
    XPLMDataRef baro, baro_standard, nav_type, nav_horizontal, nav_vertical;
    XPLMDataRef efis_mode, efis_range_nm;
    XPLMDataRef nav_loc, nav_gs, nav_course, nav_distance;
    XPLMDataRef nav_frequency, nav_identifier, nav_has_dme;
    char nav_ident[16];
    float nav_frequency_mhz, nav_dme_nm;
    int nav_dme_valid;
    float ias_trend_value, previous_ias_value, baro_hpa_value, loc_dots, gs_dots;
    int ias_trend_visible, trend_initialized, baro_std, loc_valid, gs_valid, ls_visible;
    float ias_value, altitude_value, vertical_speed_value, pitch_value, roll_value;
    float heading_value, radio_altitude_value, mach_value, decision_height_value;
    float nd_range_nm;
    int nd_mode;
    int air_valid, altitude_valid, vertical_speed_valid, attitude_valid;
    int heading_valid, radio_altitude_valid, display_valid, navigation_valid;
    int nd_valid; /* ND display unit On (the PFD uses display_valid) */
} Side;
/* draw_nd follows the ND display unit, draw_pfd the PFD unit (a380x_pfd_render.inc). */
#define SIDE_ND_VALID(side) ((side)->nd_valid)

typedef struct {
    XPLMDataRef avionics_power, on_ground, agl_m, ground_speed, ground_track;
    XPLMDataRef throttle, n1, gear, flap_handle;
    Side side[2];
} Inputs;

typedef struct {
    int flight_phase;
    int fma_a1a2, fma_a3, fma_b1, fma_c1, fma_d1d2;
    int captain_valid, first_officer_valid;
} Published;

typedef struct { const char *name; int *value; XPLMDataRef handle; } PublishedRef;

static Inputs refs;
static Published published;
static A380XFlightPhaseState phase_state;
static A380XGuidanceState guidance;
static A380XGuidanceInputs guidance_inputs;
static A380XPfdGuidance pfd_guidance;
static A380XFmsFeed fms_feed; /* native FMS outputs for the FWS/PFD/guidance (a380x_fms_adapter.inc) */
static int plugin_enabled;
static float throttle_values[4], n1_values[4], gear_values[5];
static float flap_value, ground_speed_knots, ground_track_degrees;

#include "a380x_lighting.inc"
#include "a380x_du.inc"

static PublishedRef outputs[] = {
    {"a380x/xp12/avionics/flight_phase", &published.flight_phase, NULL},
    {"a380x/xp12/avionics/fma_a1a2", &published.fma_a1a2, NULL},
    {"a380x/xp12/avionics/fma_a3", &published.fma_a3, NULL},
    {"a380x/xp12/avionics/fma_b1", &published.fma_b1, NULL},
    {"a380x/xp12/avionics/fma_c1", &published.fma_c1, NULL},
    {"a380x/xp12/avionics/fma_d1d2", &published.fma_d1d2, NULL},
    {"a380x/xp12/avionics/pfd_captain_valid", &published.captain_valid, NULL},
    {"a380x/xp12/avionics/pfd_first_officer_valid", &published.first_officer_valid, NULL},
};

static int get_published(void *refcon)
{
    PublishedRef *ref = (PublishedRef *)refcon;
    return ref != NULL && ref->value != NULL ? *ref->value : 0;
}

static int failed(XPLMDataRef ref)
{
    /* X-Plane's failure_enum uses 6 for failed.  Armed mean-time states below
     * that value must not blank a display before the actual failure occurs. */
    return ref != NULL && XPLMGetDatai(ref) >= 6;
}

static XPLMDataRef find_required(const char *name, int *ok)
{
    XPLMDataRef ref = XPLMFindDataRef(name);
    if (ref == NULL) {
        XPLMDebugString("A380X Avionics: missing required raw input ");
        XPLMDebugString(name);
        XPLMDebugString("\n");
        *ok = 0;
    }
    return ref;
}

static void find_side(Side *side, const char *suffix, const char *failure_prefix, int *ok)
{
    char name[160];
#define SIDE_REF(field, stem) do { \
    snprintf(name, sizeof(name), "sim/cockpit2/gauges/indicators/%s_%s", stem, suffix); \
    side->field = find_required(name, ok); \
} while (0)
    SIDE_REF(ias, "airspeed_kts");
    SIDE_REF(altitude, "altitude_ft");
    SIDE_REF(vertical_speed, "vvi_fpm");
    SIDE_REF(pitch, "pitch_electric_deg");
    SIDE_REF(roll, "roll_electric_deg");
    SIDE_REF(heading, "heading_AHARS_deg_mag");
    SIDE_REF(radio_altitude, "radio_altimeter_height_ft");
    SIDE_REF(mach, "mach");
#undef SIDE_REF
    snprintf(name,sizeof(name),"sim/cockpit2/gauges/actuators/radio_altimeter_bug_ft_%s",suffix);
    side->decision_height=find_required(name,ok);
    snprintf(name, sizeof(name), "sim/operation/failures/%s_asi", failure_prefix);
    side->fail_ias = find_required(name, ok);
    snprintf(name, sizeof(name), "sim/operation/failures/%s_alt", failure_prefix);
    side->fail_altitude = find_required(name, ok);
    snprintf(name, sizeof(name), "sim/operation/failures/%s_vvi", failure_prefix);
    side->fail_vertical_speed = find_required(name, ok);
    snprintf(name, sizeof(name), "sim/operation/failures/%s_ahz", failure_prefix);
    side->fail_attitude = find_required(name, ok);
    snprintf(name, sizeof(name), "sim/operation/failures/%s_dgy", failure_prefix);
    side->fail_heading = find_required(name, ok);
}

static int find_inputs(void)
{
    int ok = 1;
    memset(&refs, 0, sizeof(refs));
    refs.avionics_power = find_required("sim/cockpit2/switches/avionics_power_on", &ok);
    refs.on_ground = find_required("sim/flightmodel/failures/onground_any", &ok);
    refs.agl_m = find_required("sim/flightmodel/position/y_agl", &ok);
    refs.ground_speed = find_required("sim/flightmodel/position/groundspeed", &ok);
    refs.ground_track = find_required("sim/flightmodel/position/hpath", &ok);
    refs.throttle = find_required("sim/cockpit2/engine/actuators/throttle_ratio", &ok);
    refs.n1 = find_required("sim/cockpit2/engine/indicators/N1_percent", &ok);
    refs.gear = find_required("sim/flightmodel2/gear/deploy_ratio", &ok);
    refs.flap_handle = find_required("sim/cockpit2/controls/flap_handle_request_ratio", &ok);
    find_side(&refs.side[0], "pilot", "rel_ss", &ok);
    find_side(&refs.side[1], "copilot", "rel_cop", &ok);
    refs.side[0].efis_mode = find_required("sim/cockpit2/EFIS/map_mode", &ok);
    refs.side[1].efis_mode = find_required("sim/cockpit2/EFIS/map_mode_copilot", &ok);
    refs.side[0].efis_range_nm = find_required("sim/cockpit2/EFIS/map_range_nm", &ok);
    refs.side[1].efis_range_nm = find_required("sim/cockpit2/EFIS/map_range_nm_copilot", &ok);
    refs.side[0].fail_radio_altitude = find_required("sim/operation/failures/rel_pil_radalt", &ok);
    refs.side[1].fail_radio_altitude = find_required("sim/operation/failures/rel_cop_radalt", &ok);
    refs.side[0].fail_efis = find_required("sim/operation/failures/rel_efis_1", &ok);
    refs.side[1].fail_efis = find_required("sim/operation/failures/rel_efis_2", &ok);
    return ok;
}

static void read_side(Side *side, int powered)
{
    side->ias_value = XPLMGetDataf(side->ias);
    side->altitude_value = XPLMGetDataf(side->altitude);
    side->vertical_speed_value = XPLMGetDataf(side->vertical_speed);
    side->pitch_value = XPLMGetDataf(side->pitch);
    side->roll_value = XPLMGetDataf(side->roll);
    side->heading_value = XPLMGetDataf(side->heading);
    side->radio_altitude_value = XPLMGetDataf(side->radio_altitude);
    side->mach_value = XPLMGetDataf(side->mach);
    side->decision_height_value = XPLMGetDataf(side->decision_height);
    side->nd_mode = XPLMGetDatai(side->efis_mode);
    side->nd_range_nm = XPLMGetDataf(side->efis_range_nm);
    if (side->nd_mode < 0 || side->nd_mode > 4 ||
        !isfinite(side->nd_range_nm) || side->nd_range_nm <= 0.0f) {
        side->nd_mode = 2;
        side->nd_range_nm = 80.0f;
    }
    side->air_valid = !failed(side->fail_ias);
    side->altitude_valid = !failed(side->fail_altitude);
    side->vertical_speed_valid = !failed(side->fail_vertical_speed);
    side->attitude_valid = !failed(side->fail_attitude);
    side->heading_valid = !failed(side->fail_heading);
    side->radio_altitude_valid = !failed(side->fail_radio_altitude);
    side->air_valid = side->air_valid && isfinite(side->ias_value) && isfinite(side->mach_value);
    side->altitude_valid = side->altitude_valid && isfinite(side->altitude_value);
    side->vertical_speed_valid = side->vertical_speed_valid && isfinite(side->vertical_speed_value);
    side->attitude_valid = side->attitude_valid && isfinite(side->pitch_value) && isfinite(side->roll_value);
    side->heading_valid = side->heading_valid && isfinite(side->heading_value);
    side->radio_altitude_valid = side->radio_altitude_valid && isfinite(side->radio_altitude_value);
    side->display_valid = powered && !failed(side->fail_efis);
}

#include "a380x_adirs_adapter.inc"
#include "a380x_adirs_window.inc"
#include "a380x_fcu_adapter.inc"
#include "a380x_efis.inc"
#include "a380x_efis_nd.inc"
#include "a380x_efis_airport.inc"
#include "a380x_glareshield.inc"
#include "a380x_clock.inc"
#include "a380x_center_right.inc"
#include "a380x_lg_grvty.inc"     /* L/G GRVTY panel: owns the crew lever while the hydraulics are isolated */
#include "a380x_cockpit_door.inc"
#include "a380x_elec_panel.inc"
#include "a380x_elec_adapter.inc"
#include "a380x_ovhd_lamps.inc"
#include "a380x_ewd_data.inc"
#include "a380x_thrust.inc"      /* thrust/reverse levers, FADEC rating + FLEX, instinctive disconnect */
#include "a380x_flaps.inc"       /* FLAPS lever (pull-out, gates 1/3), FBW SFCC, flap/slat FPPU drives */
#include "a380x_speedbrake.inc"  /* SPEED BRAKE lever (push down, 1/2 notch, ARM), FBW speed brake + ground spoilers */
#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F /* OpenGL 1.2; absent from the Windows GL 1.1 header */
#endif
#include "a380x_mfd_render.inc"
static int xpkit_draw_mfd(int side,float x,float y,float scale);
static int xpkit_draw_sd(void);
#define MFDA_CDS_OVERLAY(side) cds_overlay((side) ? A380X_ELN_DU_FO_MFD : A380X_ELN_DU_CAPT_MFD, 0, 0, 768, 1024)
#define SDA_CDS_OVERLAY() cds_overlay(A380X_ELN_DU_SD, 0, 0, 768, 1024)
#include "a380x_mfd_adapter.inc"
#include "a380x_engine_indications.inc"
#include "a380x_sd_adapter.inc"
#include "a380x_fms_adapter.inc"
#include "a380x_isis_adapter.inc"
#include "a380x_fws_adapter.inc"
#define XPKIT_ND_OVERLAY(side) (plugin_enabled && cds_overlay((side) ? A380X_ELN_DU_FO_ND : A380X_ELN_DU_CAPT_ND, 0, 0, 768, 1024))
#include "a380x_xpkit_adapter.inc"

static float flight_loop(float elapsed_since_last_call, float elapsed_since_last_loop,
                         int counter, void *refcon)
{
    int i, running = 0, takeoff = 0;
    int powered;
    float dt = elapsed_since_last_loop > 0.0f ? elapsed_since_last_loop : elapsed_since_last_call;
    A380XFlightPhaseInputs phase_inputs;
    (void)counter;
    (void)refcon;
    if (!plugin_enabled) return -1.0f;
    /* The A380 network runs first (spec 9.5.6): every consumer below reads its per-bus power.
     * The ELEC pbs (wall-time press animation) are stepped just before it. */
    elec_update(elapsed_since_last_call > 0.0f ? elapsed_since_last_call : dt);
    {
        float du_brightness[A380X_ELN_DU_COUNT];
        du_brightness[A380X_ELN_DU_CAPT_PFD] = du_channels[DU_PFD].level;
        du_brightness[A380X_ELN_DU_CAPT_ND] = du_channels[DU_ND].level;
        du_brightness[A380X_ELN_DU_CAPT_MFD] = mfda_du_knob(0);
        du_brightness[A380X_ELN_DU_FO_PFD] = du_channels[DU_CHANNELS_PER_SIDE + DU_PFD].level;
        du_brightness[A380X_ELN_DU_FO_ND] = du_channels[DU_CHANNELS_PER_SIDE + DU_ND].level;
        du_brightness[A380X_ELN_DU_FO_MFD] = mfda_du_knob(1);
        du_brightness[A380X_ELN_DU_EWD] = sda.initialized ? sda.ewd_brightness : 1.0f;
        du_brightness[A380X_ELN_DU_SD] = sda.initialized ? sda.brightness : 1.0f;
        du_brightness[A380X_ELN_DU_CAPT_OIT] = du_brightness[A380X_ELN_DU_FO_OIT] = 1.0f;
        eln_update(elapsed_since_last_call > 0.0f ? elapsed_since_last_call : dt, du_brightness);
    }
    /* Thrust levers first: cockpit drags, hardware adoption and interlocks are in X-Plane's
     * handles before anything below reads them (docs/THRUST_LEVERS.md). */
    thrust_levers_update(elapsed_since_last_call > 0.0f ? elapsed_since_last_call : dt);
    /* FLAPS lever and SFCC: X-Plane's flap handle carries the configuration, the feed (PFD, FWS, xp-kit)
     * the lever and the FBW surface positions (docs/FLAPS_LEVER.md). */
    flaps_update(elapsed_since_last_call > 0.0f ? elapsed_since_last_call : dt);
    /* SPEED BRAKE lever and FBW PRIM speed brake / ground spoilers (TLA and flap index from the two above):
     * X-Plane's speedbrake handle carries the surface order, the feed the lever (docs/SPEED_BRAKE.md). */
    speedbrake_update(elapsed_since_last_call > 0.0f ? elapsed_since_last_call : dt);
    /* Legacy single criterion, used only while the native network is unavailable. */
    powered = XPLMGetDatai(refs.avionics_power) != 0;
    XPLMGetDatavf(refs.throttle, throttle_values, 0, 4);
    XPLMGetDatavf(refs.n1, n1_values, 0, 4);
    XPLMGetDatavf(refs.gear, gear_values, 0, 5);
    /* Lever position as a 0..1 handle (X-Plane's handle now carries the SFCC configuration). */
    flap_value = flaps_feed.ready ? (float)flaps_feed.handle_index / 4.0f : XPLMGetDataf(refs.flap_handle);
    ground_speed_knots = XPLMGetDataf(refs.ground_speed) * 1.94384449f;
    ground_track_degrees = XPLMGetDataf(refs.ground_track);
    for (i = 0; i < 4; ++i) {
        if (n1_values[i] > 15.0f) ++running;
        if (throttle_values[i] >= 0.85f || thrust_takeoff_power(i)) ++takeoff;
    }
    read_side(&refs.side[0], a380x_power_du_on(A380X_ELN_DU_CAPT_PFD, powered));
    read_side(&refs.side[1], a380x_power_du_on(A380X_ELN_DU_FO_PFD, powered));
    refs.side[0].nd_valid = a380x_power_du_on(A380X_ELN_DU_CAPT_ND, powered) && !failed(refs.side[0].fail_efis);
    refs.side[1].nd_valid = a380x_power_du_on(A380X_ELN_DU_FO_ND, powered) && !failed(refs.side[1].fail_efis);
    adirs_update(dt);
    ovhd_update(dt); /* overhead annunciators (after the network power and the ADIRS lamps) */
    /* FMC: any CDS supply (assumption, docs/ELEC_SYSTEM.md). */
    fmsa_update(dt, a380x_power_any(A380X_ELN_BIT(A380X_ELN_DC_ESS) | A380X_ELN_BIT(A380X_ELN_DC1) |
                                    A380X_ELN_BIT(A380X_ELN_DC2), powered)); /* native FMS, before guidance */
    memset(&phase_inputs, 0, sizeof(phase_inputs));
    phase_inputs.on_ground = XPLMGetDatai(refs.on_ground) != 0;
    phase_inputs.running_engines = running;
    phase_inputs.engines_at_takeoff_power = takeoff >= 2;
    phase_inputs.indicated_airspeed = refs.side[0].ias_value;
    phase_inputs.radio_height_ft = fmax(0.0, XPLMGetDataf(refs.agl_m) * 3.280839895);
    /* FMS V1 of the active plan; 0 = none (never "above V1", FBW FwsFlightPhases). */
    phase_inputs.configured_v1 = fms_feed.valid ? fms_feed.v1_knots : 0.0;
    published.flight_phase = a380x_flight_phase_update(&phase_state, &phase_inputs, dt);
    published.captain_valid = refs.side[0].display_valid;
    published.first_officer_valid = refs.side[1].display_valid;
    update_guidance(dt);
    /* FADEC N1 command to the engines unless A/THR (just decided above) drives them. */
    thrust_engines_update(elapsed_since_last_call > 0.0f ? elapsed_since_last_call : dt);
    efis_update(dt);
    efis_nd_update(dt);
    efis_airport_update(dt);
    glare_update(dt);
    clock_update(dt);
    /* L/G GRVTY handle and X-Plane gear-handle ownership, before the lever is read. The FBW handle
     * timing (7.2 s per turn) uses this callback's own elapsed wall time (see sda_update below). */
    lgg_update(elapsed_since_last_call > 0.0f ? elapsed_since_last_call : dt);
    center_right_update(dt);
    ewd_update(dt, a380x_power_du_on(A380X_ELN_DU_EWD, powered));
    if(adirs.on_batt && ewd.powered) ewd_add_message(ewd.memo,&ewd.memo_count,"ADIRS ON BATT",EWD_COLOR_AMBER);
    /* EWD.tsx: A.FLOOR for A/THR modes A.FLOOR and TOGA LK; THR LK for the thrust-lock message. */
    if(ewd.powered) {
        ewd.a_floor=guidance.thrust_mode==A380X_A1A2_A_FLOOR || guidance.thrust_mode==A380X_A1A2_TOGA_LK;
        ewd.thr_lk=published.fma_a3==A380X_A3_THR_LK;
    }
    /* Native FWS: alerts, memos, ECL and master lights; it replaces the legacy EWD lists. */
    fwsa_update(dt, a380x_power_du_on(A380X_ELN_DU_EWD, powered));
    fwsa_publish_ewd();
    cockpit_door_update();
    mfda_update(dt, powered);
    /* SDK elapsedSinceLastLoop is since ANY plugin callback, not this one.
     * Mechanical ENG motion uses our own callback's elapsed wall time. */
    sda_update(elapsed_since_last_call, a380x_power_du_on(A380X_ELN_DU_SD, powered), &ewd);
    isis_adapter_update(dt);
    a380x_efb_update(dt);
    xpkit_update();
    return -1.0f;
}

#include "a380x_pfd_render.inc"
#include "a380x_ewd_font.inc"
#include "a380x_ewd_render.inc"
#include "a380x_fcu_render.inc"
#include "a380x_fcu_font.inc"
#include "a380x_efis_render.inc"
#include "a380x_clock_render.inc"

/* FBW CdsDisplayUnit boot screens (CDS:260-276), drawn over a DU area in its 768x1024
 * layout: Thales boot = white bar with a blue square, self-test = green text. Returns 1
 * when the unit is booting or self-testing. */
static int cds_overlay(int du, float x, float y, float w, float h)
{
    static const Color boot_blue = {77.0f / 255.0f, 77.0f / 255.0f, 1.0f};
    int state = a380x_power_du_state(du);
    float saved = ewd_brightness;
    EwdCanvas p = {0.0f, 0.0f};
    if (state != A380X_ELN_DU_BOOT && state != A380X_ELN_DU_SELFTEST) return 0;
    glPushMatrix();
    glTranslatef(x, y, 0.0f);
    glScalef(w / 768.0f, h / 1024.0f, 1.0f);
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    ewd_brightness = 1.0f;
    ewd_rect(&p, 0, 0, 768, 1024, BLACK);
    if (state == A380X_ELN_DU_BOOT) {
        ewd_rect(&p, 84, 880, 600, 70, WHITE);
        ewd_rect(&p, 89, 885, 35, 60, boot_blue);
    } else {
        ewd_text(&p, 384, 512, "SAFETY TEST IN PROGRESS", 28, GREEN, 1);
        ewd_text(&p, 384, 553, "(MAX 30 SECONDS)", 28, GREEN, 1);
    }
    ewd_brightness = saved;
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    glPopMatrix();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    return 1;
}

/* Late READY spawn (engines found running after the load message): re-initialise the
 * consumers that depend on the spawn type. */
static void elec_spawn_changed(void)
{
    adirs_reset();
    isis_adapter_plane_loaded();
    fmsa_cold_and_dark = 0;
    survh_defaults(0);
}

#include "a380x_native_displays.inc"

static void du_dim_display(int side, int unit, float x, float y, float width, float height)
{
    float darkness = 1.0f - du_channels[side * DU_CHANNELS_PER_SIDE + unit].level;
    if (darkness <= 0.0f) return;
    glColor4f(0.0f, 0.0f, 0.0f, darkness);
    glBegin(GL_QUADS);
    glVertex2f(x, y); glVertex2f(x + width, y);
    glVertex2f(x + width, y + height); glVertex2f(x, y + height);
    glEnd();
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
}

static int draw_panel(XPLMDrawingPhase phase, int before, void *refcon)
{
    int side_index;
    GLboolean cull_was_enabled;
    (void)phase;
    (void)before;
    (void)refcon;
    if (!plugin_enabled) return 1;
    /* X-Plane enters with clockwise front faces and culling enabled. Our
     * 2D quads/fans use counterclockwise winding. Preserve this unmanaged GL
     * state so filled screens and digit segments survive the panel pass. */
    cull_was_enabled = glIsEnabled(GL_CULL_FACE);
    glDisable(GL_CULL_FACE);
    pfd_glare.land_mode=glare.land_mode;
    pfd_glare.autoland_warning=glare.autoland_warning;
    pfd_glare.autoland_mask=glare.autoland_mask;
    pfd_glare.flash_time=glare.flash_time;
    for (side_index=0;side_index<2;++side_index) {
        pfd_glare.chrono_mode[side_index]=glare.chrono_mode[side_index];
        pfd_glare.chrono_seconds[side_index]=glare.chrono_seconds[side_index];
        pfd_glare.chrono_frozen[side_index]=glare.chrono_frozen[side_index];
    }
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    glLineWidth(1.0f);
    /* With a bound 768x1024 PFD cockpit device (xp-kit builds) the device
     * presents the PFD and covers this atlas region: skip the atlas work. */
    if (xpkit_pfd_atlas_needed(0)) {
        if (!xpkit_draw(0, 0.0f, 32.0f)) draw_pfd(&refs.side[0], 0.0f, 32.0f, "CAPT");
        cds_overlay(A380X_ELN_DU_CAPT_PFD, 0.0f, 32.0f, 320.0f, 416.0f);
        du_dim_display(0, DU_PFD, 0.0f, 32.0f, 320.0f, 416.0f);
    }
    if (xpkit_pfd_atlas_needed(1)) {
        if (!xpkit_draw(1, 704.0f, 32.0f)) draw_pfd(&refs.side[1], 704.0f, 32.0f, "F/O");
        cds_overlay(A380X_ELN_DU_FO_PFD, 704.0f, 32.0f, 320.0f, 416.0f);
        du_dim_display(1, DU_PFD, 704.0f, 32.0f, 320.0f, 416.0f);
    }
    if (!xpkit_draw_nd(0,328.0f,230.0f)) draw_nd(&refs.side[0], 328.0f, 230.0f, "CAPT");
    cds_overlay(A380X_ELN_DU_CAPT_ND, 328.0f, 230.0f, 180.0f, 234.0f);
    du_dim_display(0, DU_ND, 328.0f, 230.0f, 180.0f, 234.0f);
    if (!xpkit_draw_nd(1,516.0f,230.0f)) draw_nd(&refs.side[1], 516.0f, 230.0f, "F/O");
    cds_overlay(A380X_ELN_DU_FO_ND, 516.0f, 230.0f, 180.0f, 234.0f);
    du_dim_display(1, DU_ND, 516.0f, 230.0f, 180.0f, 234.0f);
    EwdData filtered_ewd;sda_filter_ewd(&filtered_ewd,&ewd);
    ewd_brightness=sda.ewd_brightness;
    draw_ewd(&filtered_ewd, 1024.0f, 0.0f);
    cds_overlay(A380X_ELN_DU_EWD, 1024.0f, 0.0f, 768.0f, 1024.0f);
    fcu_lcd_level = lighting[2].value;
    draw_fcu();
    draw_efis_cp();
    draw_clock();
    a380x_efb_draw();
    glLineWidth(1.0f);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    if (cull_was_enabled) glEnable(GL_CULL_FACE);
    return 1;
}

static void unregister_outputs(void)
{
    int i;
    for (i = 0; i < ARRAY_LEN(outputs); ++i) {
        if (outputs[i].handle != NULL) {
            XPLMUnregisterDataAccessor(outputs[i].handle);
            outputs[i].handle = NULL;
        }
    }
}

static int register_outputs(void)
{
    int i, j;
    for (i = 0; i < ARRAY_LEN(outputs); ++i) {
        if (XPLMFindDataRef(outputs[i].name) != NULL) {
            XPLMDebugString("A380X Avionics: custom dataref collision\n");
            return 0;
        }
    }
    for (i = 0; i < ARRAY_LEN(outputs); ++i) {
        outputs[i].handle = XPLMRegisterDataAccessor(
            outputs[i].name, xplmType_Int, 0,
            get_published, NULL, NULL, NULL, NULL, NULL,
            NULL, NULL, NULL, NULL, NULL, NULL,
            &outputs[i], NULL);
        if (outputs[i].handle == NULL) {
            for (j = 0; j < i; ++j) {
                XPLMUnregisterDataAccessor(outputs[j].handle);
                outputs[j].handle = NULL;
            }
            return 0;
        }
    }
    return 1;
}

PLUGIN_API int XPluginStart(char *name, char *signature, char *description)
{
#if APL
    /* fopen requires POSIX paths; the macOS SDK otherwise returns legacy HFS paths. */
    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
#endif
    strcpy(name, "A380X FBW Avionics + flyPad");
    strcpy(signature, "org.a380x.xp12.fbw-avionics.v1");
    strcpy(description, "FlyByWire A380 guidance, PFD/FCU and touch-enabled cockpit flyPad.");
    memset(&published, 0, sizeof(published));
    a380x_flight_phase_init(&phase_state);
    if (!find_inputs() || !register_outputs()) return 0;
    if (!fcu_init()) { unregister_outputs(); return 0; }
    if (!lighting_init()) { fcu_shutdown(); unregister_outputs(); return 0; }
    if (!du_init()) { lighting_shutdown(); fcu_shutdown(); unregister_outputs(); return 0; }
    if (!glare_init()) { du_shutdown(); lighting_shutdown(); fcu_shutdown(); unregister_outputs(); return 0; }
    if (!clock_init()) { glare_shutdown(); du_shutdown(); lighting_shutdown(); fcu_shutdown(); unregister_outputs(); return 0; }
    if (!center_right_init()) { clock_shutdown(); glare_shutdown(); du_shutdown(); lighting_shutdown(); fcu_shutdown(); unregister_outputs(); return 0; }
    if (!cockpit_door_init()) { center_right_shutdown(); clock_shutdown(); glare_shutdown(); du_shutdown(); lighting_shutdown(); fcu_shutdown(); unregister_outputs(); return 0; }
    if (!efis_init()) {
        cockpit_door_shutdown(); center_right_shutdown(); clock_shutdown();
        glare_shutdown(); du_shutdown(); lighting_shutdown(); fcu_shutdown();
        unregister_outputs(); return 0;
    }
    if (!adirs_init() || !adirs_window_init()) {
        adirs_window_shutdown(); adirs_shutdown(); efis_shutdown();
        cockpit_door_shutdown(); center_right_shutdown(); clock_shutdown();
        glare_shutdown(); du_shutdown(); lighting_shutdown(); fcu_shutdown();
        unregister_outputs(); return 0;
    }
    efis_nd_init();
    efis_airport_init();
    ewd_init();
    if (!a380x_efb_init()) {
        adirs_window_shutdown(); adirs_shutdown();
        a380x_efb_shutdown();
        efis_airport_shutdown();
        efis_nd_shutdown();
        efis_shutdown();
        cockpit_door_shutdown();
        center_right_shutdown();
        clock_shutdown();
        glare_shutdown();
        du_shutdown();
        lighting_shutdown();
        fcu_shutdown();
        unregister_outputs();
        return 0;
    }
    if (!fmsa_init() || !mfda_init(&fmsa_fms, &fmsa_fms.msgs, &fmsa_host)) {
        mfda_shutdown(); fmsa_shutdown();
        adirs_window_shutdown(); adirs_shutdown();
        a380x_efb_shutdown();
        efis_airport_shutdown();
        efis_nd_shutdown();
        efis_shutdown();
        cockpit_door_shutdown();
        center_right_shutdown();
        clock_shutdown();
        glare_shutdown();
        du_shutdown();
        lighting_shutdown();
        fcu_shutdown();
        unregister_outputs();
        return 0;
    }
    /* Create the MFD cockpit devices now, before X-Plane loads the aircraft OBJs that
     * reference them (ATTR_cockpit_device / ATTR_manip_device). */
    mfda_devices_enable();
    isis_adapter_init(); /* ISIS 1 cockpit device, also before the aircraft OBJs */
    if (!elec_init()) XPLMDebugString("A380X ELEC panel: registration failed; existing avionics remain available\n");
    else if (!eln_init()) XPLMDebugString("A380X ELEC network: unavailable; displays keep the stock avionics-master power\n");
    /* Overhead annunciator lamp datarefs before the aircraft OBJs bind them (ATTR_light_level). */
    if (!ovhd_init()) XPLMDebugString("A380X OVHD lamps: registration failed; overhead legends stay dark\n");
    if (fwsa_init()) {
        char fws_line[160];
        snprintf(fws_line, sizeof(fws_line), "A380X FWS: native flight warning system ready (%d alert definitions, "
                 "%d ECAM procedures)\n", fws_signals_alert_count(), FWS_ABNORMAL_PROCEDURE_COUNT);
        XPLMDebugString(fws_line);
    }
    engi_init();
    if (!sda_init()) XPLMDebugString("A380X ECAM SD: registration failed; existing avionics remain available\n");
    /* L/G GRVTY panel datarefs before the aircraft OBJs bind them (cover, guards, switch). */
    if (!lgg_init()) XPLMDebugString("A380X L/G GRVTY: registration failed; gravity extension unavailable, the lever drives the gear\n");
    /* Thrust-lever manipulator datarefs before the aircraft OBJs bind them; needs the FCU commands. */
    if (!thrust_init()) XPLMDebugString("A380X thrust levers: registration failed; X-Plane's handles drive the engines directly\n");
    /* FLAPS lever manipulator / animation datarefs before the aircraft OBJs bind them. */
    if (!flaps_init()) XPLMDebugString("A380X flaps lever: registration failed; X-Plane's flap handle drives the flaps directly\n");
    /* SPEED BRAKE lever manipulator / animation datarefs before the aircraft OBJs bind them. */
    if (!speedbrake_init()) XPLMDebugString("A380X speed brake lever: registration failed; X-Plane's speedbrake handle drives the spoilers directly\n");
    if (!XPLMRegisterDrawCallback(draw_panel, xplm_Phase_Gauges, 0, NULL)) {
        speedbrake_shutdown();
        flaps_shutdown();
        thrust_shutdown();
        lgg_shutdown();
        isis_adapter_shutdown();
        sda_shutdown();
        engi_shutdown();
        mfda_shutdown(); fmsa_shutdown();
        adirs_window_shutdown(); adirs_shutdown();
        a380x_efb_shutdown();
        efis_airport_shutdown();
        efis_nd_shutdown();
        efis_shutdown();
        cockpit_door_shutdown();
        center_right_shutdown();
        clock_shutdown();
        glare_shutdown();
        du_shutdown();
        lighting_shutdown();
        fcu_shutdown();
        unregister_outputs();
        return 0;
    }
    XPLMRegisterFlightLoopCallback(flight_loop, -1.0f, NULL);
    xpkit_init();
    native_displays_init();
    XPLMDebugString("A380X Avionics: Imported FBW A380 guidance, PFD/FCU, MFD/FMS and physical flyPad registered.\n");
    return 1;
}

PLUGIN_API void XPluginStop(void)
{
    plugin_enabled = 0;
    xpkit_shutdown();
    native_displays_shutdown();
    XPLMUnregisterFlightLoopCallback(flight_loop, NULL);
    ovhd_shutdown();
    fwsa_shutdown();
    eln_shutdown();
    elec_shutdown();
    XPLMUnregisterDrawCallback(draw_panel, xplm_Phase_Gauges, 0, NULL);
    isis_adapter_shutdown();
    sda_shutdown();
    engi_shutdown();
    mfda_shutdown(); fmsa_shutdown();
    adirs_window_shutdown(); adirs_shutdown();
    ewd_font_shutdown();
    fcu_font_shutdown();
    mfd_gl_shutdown();
    a380x_efb_shutdown();
    efis_airport_shutdown();
    efis_nd_shutdown();
    efis_shutdown();
    cockpit_door_shutdown();
    speedbrake_shutdown();
    flaps_shutdown();
    thrust_shutdown();
    lgg_shutdown();
    center_right_shutdown();
    clock_shutdown();
    glare_shutdown();
    du_shutdown();
    lighting_shutdown();
    fcu_shutdown();
    unregister_outputs();
}

PLUGIN_API int XPluginEnable(void)
{
    plugin_enabled = 1;
    fmsa_enable();
    mfda_devices_enable();
    sda.force = 1;
    xpkit_start();
    return 1;
}

PLUGIN_API void XPluginDisable(void)
{
    xpkit_stop();
    isis_adapter_disable();
    sda_disable();
    mfda_devices_disable();
    if ((glare.ptt[0] || glare.ptt[1]) && glare.radio_ptt) XPLMCommandEnd(glare.radio_ptt);
    glare.ptt[0]=glare.ptt[1]=0;
    adirs_window_set_visible(0); adirs_release_controls();
    glare_stop_audio();
    eln_disable();
    elec_release_controls();
    lgg_release_controls();   /* X-Plane's gear handle back at the crew lever, override released */
    thrust_release_controls(); /* engines back on X-Plane's handles, disconnect buttons released */
    flaps_release_controls();  /* a lever drag ends; X-Plane keeps the configuration in its handle */
    speedbrake_release_controls(); /* a lever drag ends; X-Plane's auto speedbrake setting restored */
    center_right_release_controls();
    clock_release_controls();
    release_guidance_controls();
    guidance.ap1 = guidance.ap2 = guidance.athr_armed = guidance.athr_active = 0;
    guidance.fd1 = guidance.fd2 = guidance.guidance_valid = 0;
    guidance.vertical_guidance_selected = 0;
    guidance.loc_armed = guidance.gs_armed = guidance.approach_selected = 0;
    guidance.lateral_mode = A380X_C1_NONE;
    guidance.vertical_mode = A380X_B1_NONE;
    guidance.thrust_mode = A380X_A1A2_NONE;
    publish_guidance();
    plugin_enabled = 0;
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID from, int message, void *param)
{
    (void)from;
    if (message == XPLM_MSG_PLANE_LOADED && (intptr_t)param == 0) {
        /* Panel reset, then the network decides the spawn (cold & dark / ready) before the
         * consumers re-initialise from it. */
        elec_plane_loaded();
        eln_plane_loaded();
        ovhd_plane_loaded();
        fwsa_plane_loaded();
        fmsa_plane_loaded();
        mfda_plane_loaded();
        sda_plane_loaded();
        isis_adapter_plane_loaded();
        lgg_plane_loaded();   /* L/G GRVTY cold & dark: covers/guards closed, switch OFF, handle at rest */
        thrust_plane_loaded(); /* levers re-read from X-Plane, FADEC reset, A/THR disable cleared */
        flaps_plane_loaded();  /* lever, SFCC and surfaces re-read from X-Plane's flap handle */
        speedbrake_plane_loaded(); /* lever re-read from X-Plane's speedbrake handle (ARMED = ARM) */
    }
    if (message == XPLM_MSG_PLANE_LOADED && (intptr_t)param == 0 && guidance.initialized) {
        adirs_reset();
        release_guidance_controls();
        fcu_thrust_init(); /* Re-read the actual aircraft's detent calibration. */
        a380x_guidance_destroy(&guidance);
        read_side(&refs.side[0], a380x_power_du_on(A380X_ELN_DU_CAPT_PFD, XPLMGetDatai(refs.avionics_power)));
        read_side(&refs.side[1], a380x_power_du_on(A380X_ELN_DU_FO_PFD, XPLMGetDatai(refs.avionics_power)));
        adirs_update(0);
        read_guidance_inputs(0);
        a380x_guidance_init(&guidance, &guidance_inputs);
        publish_guidance();
    }
}

#if IBM
BOOL APIENTRY DllMain(HINSTANCE instance, DWORD reason, LPVOID reserved)
{
    (void)instance;
    (void)reason;
    (void)reserved;
    return TRUE;
}
#endif
