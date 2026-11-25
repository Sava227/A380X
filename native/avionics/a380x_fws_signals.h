/* SPDX-License-Identifier: GPL-3.0-or-later
 * Native Flight Warning System (FWS) "signal" side for the X-Plane 12 A380X port.
 *
 * Translation of the FlyByWire A380X FWS signal computations (pinned upstream tree,
 * fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/):
 *   FwsCore.ts (update() signal part, lines ~2850-5470; NOT the alert life cycle 5470-6050),
 *   FwsFlightPhases.ts, FwsMemos.ts, FwsAbnormalSensed.ts (ewdAbnormalSensed + ewdDeferredProcs),
 *   FwsNormalChecklists.ts (sensedItems), FwsLimitations.ts, FwsInopSys.ts, FwsInformation.ts,
 *   FwsSystemDisplayLogic.ts (page-when-unselected logic).
 *
 * Scope: flight phases, memos (FBW codes, FBW order and priority), per-alert "simVarIsActive",
 * whichItemsToShow / whichItemsChecked / whichItemsActive / whichItemsTimer, the deprecated per-alert
 * INFO / INOP SYS / LIMITATIONS lists, the FwsInopSys / FwsInformation / FwsLimitations dictionaries,
 * normal checklist sensed items, attention getters, SD auto page, aural requests.
 * NOT in scope (owned by the integrator): the alert life cycle (presented / cleared / recalled lists,
 * master caution / warning latching from faults, CRC / SC scheduling), ECP handling, text and
 * rendering. The descriptor table gives that life cycle everything FwsCore reads from EwdAbnormalItem.
 *
 * Every alert, memo and list id is the FBW key as an unsigned number (leading zeros dropped, the same
 * convention as a380x_fws_tables.h: "000017001" -> 17001u, limitation "1" -> 1u).
 *
 * Plain C11, XPLM-free. Times come from the caller (clock_s monotonic seconds, dt seconds).
 * Every input defaults to a HEALTHY value (fws_inputs_defaults); systems the port does not simulate
 * are never allowed to raise an alert. See docs/FWS_SIGNALS.md for the input contract.
 */
#ifndef A380X_FWS_SIGNALS_H
#define A380X_FWS_SIGNALS_H

#include "a380x_fws_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------------------------------------- */
/* Enumerations (FBW values)                                                                        */
/* ----------------------------------------------------------------------------------------------- */

/* @shared/EcamSystemPages SdPages */
enum {
    FWS_SD_NONE = -1, FWS_SD_ENG = 0, FWS_SD_APU, FWS_SD_BLEED, FWS_SD_COND, FWS_SD_PRESS, FWS_SD_DOOR,
    FWS_SD_ELEC_AC, FWS_SD_ELEC_DC, FWS_SD_FUEL, FWS_SD_WHEEL, FWS_SD_HYD, FWS_SD_FCTL, FWS_SD_CB,
    FWS_SD_CRZ, FWS_SD_STATUS, FWS_SD_VIDEO
};

/* FwsCore.ts engineState (L:A32NX_ENGINE_STATE) */
enum { FWS_ENG_OFF = 0, FWS_ENG_ON = 1, FWS_ENG_STARTING = 2, FWS_ENG_SHUTTING = 3 };

/* FwcAuralWarning (FwsCore.ts) */
enum { FWS_AURAL_NONE = 0, FWS_AURAL_SINGLE_CHIME = 1, FWS_AURAL_CRC = 2, FWS_AURAL_CAVALRY_CHARGE = 3 };

/* Radio altimeter ARINC SSM */
enum { FWS_SSM_NORMAL = 0, FWS_SSM_NCD = 1, FWS_SSM_FAILURE = 2 };

/* a380x_avionics_core.h A380XA3Message subset used by A/THR LIMITED / ENGINE THRUST LOCKED */
enum { FWS_A3_NONE = 0, FWS_A3_THR_LK = 1, FWS_A3_LVR_TOGA = 2, FWS_A3_LVR_CLB = 3, FWS_A3_LVR_MCT = 4 };

#define FWS_PHASE_BIT(p) (1u << (p))   /* flightPhaseInhib bit for FBW phase 1..12 */

/* ----------------------------------------------------------------------------------------------- */
/* Inputs. Fill with fws_inputs_defaults() first, then overwrite what the port really has.          */
/* The *_valid flags default to 0 = "not simulated": the module then uses the healthy value.        */
/* Units: ft, kt, deg, degC, kg, psi, seconds. Booleans are 0/1 (unsigned char).                    */
/* ----------------------------------------------------------------------------------------------- */
typedef struct {
    /* ---- ELEC (native network a380x_elec, eln.st / elec panel). elec_valid = network running ---- */
    unsigned char elec_valid;
    unsigned char ac_powered[4];               /* AC 1..4 */
    unsigned char ac_ess_powered, ac_emer_powered;
    unsigned char dc1_powered, dc2_powered, dc_ess_powered;
    unsigned char dc_ess_inflt_powered;        /* 108PH (DC ESS INFLT) */
    unsigned char dc_eha_powered;              /* 247PP (DC EHA) */
    unsigned char gen_pb_on[4];                /* GEN 1..4 pb not OFF */
    unsigned char gen_fault[4];                /* GEN failure (eln.in.fail.gen[] | rel_genera0-3), not the lamp */
    unsigned char gen_potential_normal[4];     /* gen_avail (L:A32NX_ELEC_ENG_GEN_n_POTENTIAL_NORMAL) */
    unsigned char apu_gen_pb_on[2], apu_gen_fault[2]; /* fault = eln.in.fail.apu_gen[] */
    unsigned char bat_pb_auto[4];              /* BAT 1, BAT 2, BAT ESS, BAT APU */
    unsigned char bat_fault[4];                /* eln.in.fail.bat[] | rel_batter0-3 (= FAULT lamp) */
    unsigned char ext_connected;               /* GPU plugged in */
    unsigned char ext_avail[4], ext_pb_on[4];
    unsigned char ext_contactor_closed[4];     /* 990XG1..4 (FBW extPwrConnected) */
    unsigned char drive_disc[4];               /* IDG/VFG drive disconnected (latched) */
    unsigned char bus_tie_auto, ac_ess_feed_normal;
    unsigned char comm_on[2];                  /* COMMERCIAL 1/2 pb ON */
    unsigned char galley_auto;
    float rat_stow_ratio;                      /* 0 stowed .. 1 deployed (L:A32NX_RAT_STOW_POSITION) */
    unsigned char rat_man_on, rat_fault;       /* rat_fault = elec/fail/rat_fault */
    unsigned char tr_fault[4];                 /* TR 1, TR 2, TR ESS, TR APU failure inputs */
    unsigned char stat_inv_fault;

    /* ---- ENGINES (stock X-Plane + a380x_ewd_fadec). HP spool = X-Plane N2 (FBW N3) ---- */
    unsigned char eng_valid;                   /* 0: engines treated as OFF/healthy */
    float n1_pct[4], n3_pct[4], egt_c[4];
    float n1_idle_pct;                         /* L:A32NX_ENGINE_IDLE_N1 (FADEC idle N1) */
    unsigned char eng_state[4];                /* FWS_ENG_* */
    unsigned char eng_master_on[4];            /* ENG MASTER ON (fuel_valve_switch / engine_master) */
    unsigned char eng_limits_valid;
    float egt_red_c, n1_red_pct, n3_red_pct;   /* red-line limits for the EGT / N1/N2 OVER LIMIT ext. */
    unsigned char oil_press_low[4];            /* sim/cockpit2/annunciators/oil_pressure_low */
    unsigned char eng_start_fault[4];          /* adapter: STARTING && stock hung/hot start failures */
    unsigned char eng_anti_ice_on[4], wing_anti_ice_on;
    unsigned char eng_bleed_pb_auto[4];        /* ENG n BLEED pb AUTO */
    unsigned char eng_mode_selector;           /* L:XMLVAR_ENG_MODE_SEL: 0 CRANK, 1 NORM, 2 IGN/START */
    unsigned char tla_valid;
    float tla_deg[4];                          /* L:A32NX_AUTOTHRUST_TLA (IDLE 0, CL 25, MCT/FLX 35, TOGA 45) */
    unsigned char flex_temp_set;               /* L:A32NX_AIRLINER_TO_FLEX_TEMP != 0 */
    unsigned char thrust_limit_flex;           /* L:A32NX_AUTOTHRUST_THRUST_LIMIT_TYPE == 3 */

    /* ---- FIRE ---- */
    unsigned char eng_fire_detected[4], apu_fire_detected;
    unsigned char eng_fire_pb_released[4], apu_fire_pb_released;
    unsigned char eng_agent_discharged[4][2], apu_agent_discharged;
    unsigned char fire_test_pb, evac_command;

    /* ---- APU / BLEED / AIR ---- */
    unsigned char apu_master_on, apu_avail, apu_bleed_pb_on, apu_bleed_valve_open;
    unsigned char apu_n_valid;
    float apu_n_pct;
    unsigned char pack_pb_on[2];
    unsigned char ram_air_on, cabin_air_extract_on;
    unsigned char pack_flow_lo;                /* pack flow selector == LO (FBW flowSelectorKnob == 1) */
    unsigned char man_cab_alt_mode;            /* PRESS MAN ALT pb not AUTO */
    unsigned char cabin_alt_valid;
    float cabin_alt_ft, cabin_diff_psi;
    unsigned char pax_oxy_masks_deployed;

    /* ---- AIR DATA / ADIRS (native a380x_adirs, stock air data) ---- */
    unsigned char adr_cas_valid[3], adr_mach_valid[3], adr_alt_valid[3];
    float adr_cas_kt[3], adr_mach[3], adr_alt_ft[3];
    unsigned char adr_fault[3];                /* ADR FAULT (discrete word FW or fault bit) */
    unsigned char adr_overspeed[3];            /* ADR discrete word 1 bit 9 */
    unsigned char adr_pb_on[3], ir_pb_on[3];
    unsigned char ir_fault[3];                 /* IR FAULT (pitch FW or maint word bit 9) */
    unsigned char ir_in_align[3];              /* maint word bits 16..18 */
    unsigned char ir_mode[3];                  /* mode selector 0 OFF, 1 NAV, 2 ATT */
    float ir_align_remaining_s;                /* L:A32NX_ADIRS_REMAINING_IR_ALIGNMENT_TIME */
    unsigned char air_data_knob, att_hdg_knob; /* 0 CAPT ON 3, 1 NORM, 2 F/O ON 3 */
    unsigned char ir_gs_valid[3];
    float ir_gs_kt[3];
    float ra_ft[3];
    unsigned char ra_ssm[3];                   /* FWS_SSM_*; NCD above 8192 ft (ALA-52B) */
    float indicated_alt_ft;                    /* Simplane.getAltitude() (altitude alert) */
    unsigned char true_north_ref;
    unsigned char stall_warning_raw;           /* stall_warning_on */
    unsigned char normal_law;                  /* FCDC word 1 bit 11 (port: always 1) */

    /* ---- GROUND / GEAR / BRAKES ---- */
    unsigned char sim_on_ground;               /* Simplane.getIsGrounded() (flight phases only) */
    unsigned char main_gear_compressed;        /* LGCIU left MLG compressed (FWS on-ground logic) */
    unsigned char gear_lever_down;             /* GEAR HANDLE POSITION */
    unsigned char lg_grvty_switch_pos;         /* L:A32NX_LG_GRVTY_SWITCH_POS 0 RESET, 1 OFF, 2 DOWN */
    unsigned char gear_valid;
    unsigned char nose_downlocked, main_left_downlocked, main_right_downlocked; /* LGCIU bits 25/23/24 */
    unsigned char all_gear_uplocked;
    float gear_extended_ratio;                 /* GEAR TOTAL PCT EXTENDED / 100 */
    float nose_gear_ratio;                     /* GEAR POSITION:0 */
    unsigned char park_brake_set;              /* L:A32NX_PARK_BRAKE_LEVER_POS == 1 */
    unsigned char antiskid_on;                 /* ANTISKID BRAKES ACTIVE */
    unsigned char autobrake_active;            /* L:A32NX_AUTOBRAKES_ACTIVE */
    unsigned char autobrake_mode;              /* A380AutobrakeMode: 0 DISARM,1 BTV,2 LO,3 L2,4 L3,5 HI,6 RTO */
    unsigned char brakes_valid;
    unsigned char autobrake_fault, norm_brake_fault, antiskid_fault; /* port extensions */
    unsigned char nw_steering_disc;

    /* ---- FLIGHT CONTROLS ---- */
    unsigned char flaps_handle_index;          /* L:A32NX_FLAPS_HANDLE_INDEX 0,1(1+F),2,3,4(FULL) */
    signed char flaps_conf_index;              /* L:A32NX_FLAPS_CONF_INDEX 0..6, -1 = derive from angles */
    unsigned char flap_slat_valid;
    float flap_angle_deg, slat_angle_deg;      /* surface angles (flaps 0..33, slats 0..23) */
    unsigned char flap_sys_fault[2], slat_sys_fault[2], flaps_jammed, slats_jammed;
    unsigned char gnd_splr_armed;              /* FCDC word 4 bit 27 */
    unsigned char speedbrake_lever_out;        /* FCDC word 4 bit 28 (speed brake command) */
    unsigned char pitch_trim_valid, rudder_trim_valid;
    float pitch_trim_deg;                      /* ELEVATOR TRIM POSITION (THS, deg, nose up +) */
    float rudder_trim_deg;                     /* SEC rudder trim actual position (deg) */
    unsigned char controls_moved;              /* sidestick |x|,|y| > 0.05 or rudder > 0.2 (SD logic) */

    /* ---- AUTO FLIGHT (a380x_guidance) ---- */
    unsigned char ap_engaged, fd1_on, fd2_on;
    unsigned ap_instinctive_disc_count;        /* AP instinctive disc presses since the last update */
    unsigned char athr_instinctive_disc;       /* A/THR instinctive disc pressed since the last update */
    unsigned char athr_status;                 /* L:A32NX_AUTOTHRUST_STATUS 0 off, 1 armed, 2 active */
    unsigned char athr_mode_active;            /* L:A32NX_AUTOTHRUST_MODE != 0 */
    unsigned char afs_a3;                      /* FWS_A3_* (L:A32NX_AUTOTHRUST_MODE_MESSAGE) */
    unsigned char afs_a1a2;                    /* A380XA1A2Message (pass-through: A.FLOOR / TOGA LK) */
    float fcu_alt_ft;                          /* AUTOPILOT ALTITUDE LOCK VAR:3 */
    float fg_alt_constraint_ft;                /* L:A32NX_FG_ALTITUDE_CONSTRAINT, 0 = none */
    unsigned char alt_constraint_is_target;    /* managed alt && L:AP_CURRENT_TARGET_ALTITUDE_IS_CONSTRAINT */
    unsigned char gs_captured;                 /* FMA vertical G/S (30..34) */
    unsigned char tcas_ra_active;              /* L:A32NX_TCAS_STATE > 1 */

    /* ---- FMS (native a380x_fms published outputs) ---- */
    unsigned char fms_valid;
    float v1_kt;                               /* L:AIRLINER_V1_SPEED, 0 = none */
    unsigned char fms_to_speeds_not_inserted, fms_to_speeds_too_low, fms_v_disagree; /* FM word 3 */
    signed char fms_to_flaps;                  /* 0..3 FM word 2 bits 13..16, -1 = none */
    unsigned char fms_to_pitch_trim_valid;
    float fms_to_pitch_trim_cg;                /* L:A32NX_FM1_TO_PITCH_TRIM (%MAC) */
    unsigned char gw_cg_valid;
    float gw_cg_pct;                           /* FQMS gross weight CG %MAC */
    unsigned char fms_landing_conf3;           /* L:A380X_FM_LANDING_CONF3 */
    unsigned char fms_zfw_zfwcg_set;           /* FQMS status bit 12 inverted (valid with fms_valid) */
    unsigned char fms_dest_efob_below_min;
    unsigned char landing_elev_valid, departure_elev_valid;
    float landing_elev_ft, departure_elev_ft;

    /* ---- FUEL (stock tanks: feed tank n = m_fuel[n]) ---- */
    unsigned char fuel_valid;
    float feed_tank_kg[4];
    float fuel_on_board_kg;
    unsigned char feed_pump_main_on[4], feed_pump_stby_on[4], feed_pump_stby_valid;
    unsigned char feed_pump_main_fault[4], feed_pump_stby_fault[4];
    unsigned char crossfeed_open[4];
    unsigned char refuel_panel_open, refuel_in_progress, defuel_in_progress;

    /* ---- HYDRAULICS (stock, 1 = green, 2 = yellow) ---- */
    unsigned char hyd_valid;
    float hyd_green_psi, hyd_yellow_psi;
    unsigned char eng_pump_pb_auto[4][2];      /* ENG n PMP A/B pb AUTO */
    unsigned char eng_pump_section_low[4][2];  /* pump section pressure switch < 2900 psi */
    unsigned char eng_pump_disc[4];
    unsigned char green_rsv_level_low, yellow_rsv_level_low;
    unsigned char elec_pump_active[4];         /* G A, G B, Y A, Y B (memos) */
    /* A380 hydraulic model (a380x_hyd.inc) */
    unsigned char elec_pump_fault[4];          /* G A, G B, Y A, Y B: ELEC PMP FAULT */
    unsigned char elec_pump_off[4];            /* ELEC PMP FAULT/OFF pb released (OFF) */
    unsigned char green_rsv_air_low, yellow_rsv_air_low, green_rsv_ovht, yellow_rsv_ovht;

    /* ---- CABIN / LIGHTS / DOORS / SURVEILLANCE ---- */
    unsigned char seat_belts_on, no_mobile_on, strobe_off, beacon_on;
    unsigned char crew_oxy_off, emer_exit_light_off;
    unsigned char main_door_open[10];          /* M1L, M1R, M2L, ... M5R (door_open_ratio[0..9] > 0) */
    unsigned char cargo_fwd_locked, cargo_aft_locked;
    unsigned char brakes_hot;
    unsigned char xpdr_stby, xpdr_alt_reporting;
    unsigned char tcas_mode;                   /* L:A32NX_TCAS_MODE 0 STBY, 1 TA, 2 TA/RA */
    unsigned char tcas_fault;
    unsigned char vhf3_voice;
    unsigned char ice_valid, in_cloud;
    float ice_ratio, tat_c;                    /* STRUCTURAL ICE PCT (0..1), TAT */

    /* ---- crew buttons (raw level, any side) ---- */
    unsigned char to_config_pb, master_warning_pb, master_caution_pb, rcl_pb;

    /* ---- ECAM SD (from the integrator's life cycle / SD module) ---- */
    signed char sd_fail_page;                  /* L:A32NX_ECAM_SFAIL of the previous cycle, -1 none */
} FwsInputs;

/* ----------------------------------------------------------------------------------------------- */
/* Alert descriptor table                                                                           */
/* ----------------------------------------------------------------------------------------------- */
#define FWS_SIG_MAX_SUPPRESS 4

typedef struct {
    unsigned id;
    const char *title;                         /* plain ASCII, debug / documentation only */
    signed char failure;                       /* 3 warning, 2 caution, 1 level-1, 0 deferred, -1 none */
    unsigned short phase_inhib;                /* FWS_PHASE_BIT(phase) set = inhibited when new */
    float monitor_confirm_s;                   /* default 0.6 */
    unsigned char cancel;                      /* 0: only EMER CANC cancels the level-3 audio */
    signed char sys_page;                      /* FWS_SD_* */
    unsigned char n_items;                     /* procedure item count (EcamMessages text) */
    unsigned char non_procedure;               /* nonProcedureKey */
    unsigned char has_aural;                   /* auralWarning override defined (see eval.aural) */
    unsigned char has_items_active, has_items_timer;
    unsigned char recommendation;              /* 0 none, 1 LAND ASAP, 2 LAND ANSA (from the text) */
    unsigned char port_extension;              /* 1: not in FBW FwsAbnormalSensed (port logic) */
    unsigned char deferred;                    /* 1: ewdDeferredProcs entry */
    unsigned not_active_when[FWS_SIG_MAX_SUPPRESS]; /* notActiveWhenItemActive, 0-terminated */
} FwsAlertDesc;

/* Per-alert evaluation (FwsCore reads these from EwdAbnormalItem every cycle). */
typedef struct {
    unsigned char active;                      /* simVarIsActive */
    unsigned char aural;                       /* FWS_AURAL_* of the auralWarning override */
    unsigned char n_items;
    unsigned char items_to_show[FWS_MAX_ITEMS];
    unsigned char items_checked[FWS_MAX_ITEMS];/* raw whichItemsChecked (no sensed mask applied) */
    unsigned char has_items_active;
    unsigned char items_active[FWS_MAX_ITEMS];
    unsigned char has_items_timer;
    unsigned char timer_valid[FWS_MAX_ITEMS];
    double items_timer_s[FWS_MAX_ITEMS];       /* clock_s at which the timed condition started */
} FwsAlertEval;

/* Deprecated per-alert STS lists (EwdAbnormalItem.info / inopSys* / limitations* / redundLoss). */
typedef enum {
    FWS_LIST_INFO, FWS_LIST_INOP_ALL, FWS_LIST_INOP_APPR_LDG, FWS_LIST_REDUND_LOSS,
    FWS_LIST_LIM_ALL, FWS_LIST_LIM_APPR_LDG, FWS_LIST_LIM_PFD, FWS_LIST_KIND_COUNT
} FwsListKind;

#define FWS_SIG_LIST_MAX 48
typedef struct {
    unsigned n;
    unsigned id[FWS_SIG_LIST_MAX];
} FwsIdList;

/* STS / EWD lists as FwsCore publishes them */
typedef struct {
    FwsIdList limitations_all_phases;          /* fws_limitations_all_phases (priority sorted) */
    FwsIdList limitations_appr_ldg;            /* fws_limitations_appr_ldg */
    FwsIdList limitations_pfd;                 /* PFD limitation lines (max 8 used) */
    FwsIdList inop_sys_all_phases;             /* fws_inop_sys_all_phases */
    FwsIdList inop_sys_appr_ldg;               /* fws_inop_sys_appr_ldg */
    FwsIdList inop_sys_redundancy_loss;        /* fws_inop_sys_redundancy_loss */
    FwsIdList information;                     /* fws_information */
    FwsIdList alerts_impacting_ldg_perf;       /* fws_alerts_impacting_ldg_perf (FBW: always empty) */
    unsigned char ecam_status_normal;
} FwsStatusLists;

/* Alert the life cycle considers active for the STS lists (FwsCore lines 5784-5806). */
typedef struct {
    unsigned id;
    const unsigned char *checked;              /* presented/cleared itemsChecked, NULL = undefined */
    unsigned n_checked;
} FwsActiveAlertRef;

/* ----------------------------------------------------------------------------------------------- */
/* Internal logic node types (FBW NXLogic). Exposed only so FwsSignals can be a plain struct.       */
/* ----------------------------------------------------------------------------------------------- */
typedef struct { double t, timer; unsigned char rising, prev_in, prev_out; } FwsConfNode;
typedef struct { double t, timer; unsigned char rising, retrig, prev, out; } FwsMtrigNode;
typedef struct { unsigned char set_star, value; } FwsMemNode;
typedef struct { unsigned char rising, last, out; } FwsPulseNode;

#define FWS_SIG_MAX_ALERTS 224
#define FWS_SIG_MAX_SUPPRESSABLE 352

/* Signal state. Treat as opaque; allocate statically (it is a few tens of KB). */
typedef struct {
    FwsInputs in;                              /* copy of the last inputs */
    double clock_s, dt;
    int initialized;

    /* ---------- flight phases (FwsFlightPhases) ---------- */
    int flight_phase;                          /* 1..12 */
    int prev_flight_phase;
    FwsConfNode fp_gnd_memo, fp_one_eng_running_conf, fp_mct_memo, fp_fire_pb_out_conf;
    FwsMemNode fp_speed_above_80kts, fp_speed_above_v1, fp_fire_pb_clear12, fp_phase9_nvm;
    FwsMtrigNode fp_fire_pb_out_memo, fp_phase112_memo, fp_phase10_ground_memo, fp_ac80kts_memo;
    FwsMtrigNode fp_prev_phase11_invert_memo, fp_two_eng_to_power_invert_memo, fp_ground_immediate_memo;
    FwsMtrigNode fp_phase6_memo, fp_phase7_memo, fp_phase89_memo;
    unsigned char fp_prev_phase11;
    FwsConfNode memo_to_conf01, memo_ldg_conf01, memo_ldg_conf02;
    FwsMemNode memo_to_memo, memo_ldg_inhibit, memo_ldg_below2000;
    unsigned char to_memo, ldg_memo;
    /* altitude alert */
    double alt_prev_target;
    unsigned char alt_prev_target_valid, alt_was_below, alt_was_above, alt_was_in_range, alt_was_reach200;
    unsigned char alt_cchord_short_triggered, aural_cchord_cont, aural_cchord_once;

    /* ---------- FwsCore derived signals ---------- */
    FwsPulseNode to_config_pulse, rcl_pulse, phase3_pulse, phase_change_pulse, mw_pulse, mc_pulse;
    FwsMtrigNode to_config_trigger, to_config_held_1s5;
    unsigned char to_config_test, to_config_test_held;
    FwsMemNode flight_phase_inhibit_override;
    FwsConfNode to_inhibit_timer, ldg_inhibit_timer;
    unsigned char show_takeoff_inhibit, show_landing_inhibit;
    FwsConfNode phase8_conf60, phase8_conf180, phase815min, phase1112_1min, phase10_90s;
    FwsMemNode phase12_shutdown_memory;
    FwsConfNode shutdown_50min_conf;
    unsigned char checklist_reset_50min, phase1112_more_than_1min;
    FwsConfNode v1_conf;
    unsigned char aural_v1;

    /* on ground */
    FwsMtrigNode ignore_ra_on_ground_trigger;
    FwsMemNode ra_on_ground_mem[3];
    FwsConfNode on_ground_conf;
    unsigned char on_ground_immediate, aircraft_on_ground;

    /* engines */
    unsigned char eng_running[4], one_eng_running;
    FwsConfNode eng_not_starting_conf[4];
    FwsMemNode eng_was_running_mem[4], eng_fail_mem[4];
    FwsPulseNode eng_master_on_pulse[4];
    unsigned char eng_fail[4], eng_shutdown[4], eng_out[4], all_engines_failure;
    unsigned char eng_start_or_crank[4], eng_primary_abnormal[4];
    FwsConfNode eng12_to_power_conf, eng34_to_power_conf;
    unsigned char eng12_to_power, eng34_to_power;
    FwsConfNode engines_off_on_ground_conf;

    /* AP / A/THR */
    FwsPulseNode ap_disengaged_pulse, ap_disc_pressed_pulse, ap_vol_disc_pulse, ap_vol_first_cc_ended_pulse;
    FwsMtrigNode ap_disc_in_1p9, ap_disc_twice_in_1p9, ap_first_cc_0p3, ap_vol_end_1p9, ap_vol_first_cc_active;
    FwsMemNode ap_off_vol_mem, ap_off_invol_mem, ap_off_unack;
    unsigned char ap_off_involuntary, ap_off_show_memo, mw_from_ap_off, aural_cav_charge;
    FwsPulseNode athr_disengaged_pulse, athr_disc_pressed_pulse, phase1_pulse;
    FwsPulseNode athr_vol_mem_down_pulse, athr_off_vol_cond_pulse;
    FwsMtrigNode athr_idle_mtrig, athr_vol_off_mtrig, athr_vol_caution_mtrig, athr_vol_memo_mtrig;
    FwsMemNode athr_vol_disc_mem, athr_vol_caution_mem, athr_vol_memo_mem;
    FwsMemNode athr_invol_warning_mem, athr_invol_caution_mem, athr_invol_pfd_memo_mem;
    unsigned char athr_engaged_or_armed, vol_athr_off_caution, vol_athr_off_memo;
    unsigned char athr_off_invol_warning, athr_off_invol_caution;
    FwsConfNode athr_limited_conf, eng_thrust_locked_conf;
    FwsMtrigNode athr_limited_mtrig, eng_thrust_locked_mtrig;
    unsigned char athr_limited_delay, athr_limited, athr_limited_clb, athr_limited_mct;
    unsigned char eng_thrust_locked_delay, eng_thrust_locked, eng_thrust_locked_alert;
    /* autobrake */
    FwsConfNode abrk_off_conf;
    FwsPulseNode abrk_deactivated_pulse;
    FwsMtrigNode abrk_athr_disc_trig, abrk_deactivated_node, abrk_off_mc_node;
    FwsMemNode abrk_off_mem, abrk_off_mc_mem;
    unsigned char autobrake_off_memo, autobrake_off_mc;

    /* configuration / T.O */
    FwsMemNode flap_config_sr, slat_config_sr, spdbrk_config_sr, pitch_config_sr, rudder_config_sr;
    unsigned char flaps_not_to, slats_not_to, spdbrk_not_to, pitch_trim_not_to, rudder_trim_not_to;
    unsigned char flap_config_warning, flap_config_aural, slat_config_warning, slat_config_aural;
    unsigned char spdbrk_config_warning, spdbrk_config_aural;
    unsigned char pitch_trim_warning, pitch_trim_audio, rudder_trim_warning, rudder_trim_audio;
    unsigned char flaps_inf8, flaps_sup8, flaps_sup17, flaps_sup26, slats_inf20, flaps_conf3_full;
    unsigned char flaps_not_in_to_pos, slats_not_in_to_pos;
    unsigned char flaps_mcdu_disagree, flaps_pitch_mcdu_enable, pitch_trim_mcdu_disagree;
    FwsConfNode trim_disagree_conf;
    unsigned char to_config_checked_in_23, to_speeds_not_inserted_rs;
    FwsPulseNode to_config_no_speeds_pulse;
    unsigned char to_speeds_not_inserted, to_speeds_too_low, to_v_disagree;
    FwsConfNode to_config_normal_conf;
    unsigned char to_config_normal;
    unsigned char flaps_lever_not_zero;
    FwsConfNode spdbrk_5s, spdbrk_50s, eng_above_idle_spdbrk, spdbrk_caution1_conf;
    FwsPulseNode spdbrk_caution1_pulse, spdbrk_caution2_pulse;
    unsigned char speed_brakes_still_extended;
    FwsMtrigNode phase10_4s5;
    FwsConfNode gnd_splr_5s, spdbrk_5s_delayed;
    unsigned char gnd_splr_not_armed;
    unsigned char overspeed_vmo, overspeed_vle, overspeed_vfe[5];
    FwsMemNode adr3_overspeed_mem;
    unsigned char overspeed_occurred, load_analysis_required;
    unsigned char config_park_brk, lg_park_brk_on;
    FwsConfNode park_brake_2s;
    FwsMemNode config_park_brk_mem;

    /* gear */
    unsigned char main_gear_downlocked, all_gear_downlocked;
    FwsPulseNode lg_not_down_pulse1, lg_not_down_pulse2;
    unsigned char lg_not_down, lg_not_down_no_cancel, lg_red_arrow;
    FwsConfNode gear_not_locked_down_conf, gear_not_locked_up_conf; /* port extensions */
    unsigned char gear_not_locked_down, gear_not_locked_up;

    /* air / press */
    unsigned char pack_off[2], pack12_fault, apu_bleed_outside_envelope, eng_bleed_abnormal_off[4];
    unsigned char excess_cab_alt, excess_diff_press, excess_residual_diff_press, high_landing_field;
    unsigned char inhibited_by_doors, brakes_hot;
    int flight_level;
    double adr_pressure_alt;
    unsigned char adr_pressure_alt_valid;

    /* ADIRS / RA / surveillance */
    unsigned char adr_faulty[3], ir_fault[3];
    unsigned char ra_failed[3];
    FwsConfNode tcas_stby_conf, tcas_stby_memo_conf, tcas1_adr_ir_conf, tcas2_adr_ir_conf;
    FwsConfNode tcas1_fault_conf, tcas2_fault_conf;
    unsigned char tcas_standby, tcas_standby_memo, tcas1_fault, tcas2_fault, tcas12_fault;
    unsigned char stall_warning;

    /* fuel */
    FwsConfNode feed_low_conf[4];
    unsigned char feed_tank_low[4], crossfeed_open_memo, all_crossfeed_open, all_feed_pumps_on;
    unsigned char all_feed_pumps_off, all_fuel_pumps_off;

    /* hydraulics */
    unsigned char green_press_sw, yellow_press_sw;   /* pressurised switch with hysteresis */
    FwsConfNode eng12_running_phase_conf, eng34_running_phase_conf;
    unsigned char green_lo, yellow_lo, green_yellow_lo, eng12_34_running_phase;
    FwsConfNode eng_pump_off_conf[4][2];
    unsigned char eng_pump_fault[4][2], three_yellow_pumps_failed;

    /* fire timers */
    unsigned char fire_clock10[4], fire_clock30[4], apu_fire_clock10;
    double fire_ts10[4], fire_ts30[4], apu_fire_ts10;

    /* ice */
    FwsConfNode ice_det1, ice_det2, ice_severe, ice_not_det1, ice_not_det2;
    unsigned char ice_not_detected, ice_detected;

    /* SD auto page */
    double sd_crz_timer, sd_fctl_timer, sd_eng_timer, sd_apu_timer;
    signed char sd_page_when_unselected;

    /* ---------- memos ---------- */
    unsigned memo_left[10], memo_right[10], pfd_memo[3];
    int n_memo_left, n_memo_right, n_pfd_memo;

    /* ---------- alerts ---------- */
    unsigned char alert_active[FWS_SIG_MAX_ALERTS];
    unsigned char alert_aural[FWS_SIG_MAX_ALERTS];
    /* activation time of every suppressable item (alerts, INOP SYS, INFO, LIMITATIONS) */
    unsigned char sup_active[FWS_SIG_MAX_SUPPRESSABLE];
    double sup_since[FWS_SIG_MAX_SUPPRESSABLE];

    FwsStatusLists lists;                      /* computed in update() from confirmed alerts */
    unsigned char tla_idle_all_cache;
    /* scratch for the default STS list assembly in update() */
    FwsActiveAlertRef scratch_refs[FWS_SIG_MAX_ALERTS];
    unsigned char scratch_checked[FWS_SIG_MAX_ALERTS][FWS_MAX_ITEMS];
} FwsSignals;

/* Outputs that are not alerts, read with fws_signals_outputs(). */
typedef struct {
    int flight_phase;
    unsigned char to_memo, ldg_memo, on_ground;
    unsigned char show_takeoff_inhibit, show_landing_inhibit;
    unsigned char to_config_normal, to_config_test;
    unsigned char normal_attention_getter_eng[4];          /* fws_normal_attention_getter_eng */
    unsigned char abnormal_primary_eng_attention_getter[4];/* fws_abnormal_primary_engine_parameters_attention_getter */
    signed char sd_page_when_unselected;                   /* FwsSystemDisplayLogic pageWhenUnselected */
    unsigned char master_warning_from_ap_off;              /* requestMasterWarningFromApOff */
    unsigned char aural_cavalry_charge;                    /* 'cavalryChargeCont' queued (AP OFF) */
    unsigned char aural_cchord_once, aural_cchord_cont;    /* altitude alert */
    unsigned char aural_v1;                                /* 'v1' callout pulse */
    unsigned char stall_warning;
    unsigned char land_asap;
    unsigned char lg_lever_red_arrow;
    unsigned char checklist_reset_50min;                   /* shutDownFor50MinutesCheckListReset */
    unsigned char autothrust_involuntary_pfd_memo;         /* A/THR OFF PFD-only memo */
    unsigned char afs_alpha_floor, afs_toga_lk, afs_thr_lk;/* pass-through FMA flags */
} FwsSignalOutputs;

/* ----------------------------------------------------------------------------------------------- */
/* API                                                                                             */
/* ----------------------------------------------------------------------------------------------- */
void fws_inputs_defaults(FwsInputs *in);
void fws_signals_init(FwsSignals *s);
/* clock_s: monotonic seconds (e.g. sim elapsed time); dt: seconds since the previous update. */
void fws_signals_update(FwsSignals *s, const FwsInputs *in, double clock_s, float dt);
void fws_signals_outputs(const FwsSignals *s, FwsSignalOutputs *out);

/* Alert table (implemented alerts in FBW Object.entries order, deferred procedures last). */
int fws_signals_alert_count(void);
const FwsAlertDesc *fws_signals_alert_desc(int index);
int fws_signals_alert_index(unsigned id);                  /* -1 when not implemented */
/* simVarIsActive of the last update */
int fws_signals_alert_active(const FwsSignals *s, int index);
/* itemIsActiveConsideringFaultSuppression(item, key, 0.6): active, confirmed for monitor_confirm_s
 * and not suppressed by a confirmed notActiveWhenItemActive entry (FwsCore lines 5568-5601). */
int fws_signals_alert_confirmed(const FwsSignals *s, int index);
/* Item vectors, aural override and timers for the alert, computed from the last update. */
void fws_signals_alert_eval(const FwsSignals *s, int index, FwsAlertEval *out);
/* Deprecated per-alert list (checked = presented itemsChecked or NULL). Returns the count. */
int fws_signals_alert_list(const FwsSignals *s, int index, FwsListKind kind,
                           const unsigned char *checked, unsigned n_checked, unsigned *out, int max);
/* FBW STS list assembly for the alerts the life cycle reports (in FBW order = ascending id);
 * alerts == NULL uses every confirmed, non phase-inhibited procedure with its sensed checked state. */
void fws_signals_status_lists(const FwsSignals *s, const FwsActiveAlertRef *alerts, int n_alerts,
                              FwsStatusLists *out);

/* Memos (FBW codes, display order). Return the count, ids written to out. */
int fws_signals_memos_left(const FwsSignals *s, unsigned *out, int max);
int fws_signals_memos_right(const FwsSignals *s, unsigned *out, int max);
int fws_signals_memos_pfd(const FwsSignals *s, unsigned *out, int max);

/* Normal checklist sensed items (FwsNormalChecklists.sensedItems). out[i]: -1 null (not sensed),
 * 0 / 1 sensed state. Returns the item count, -1 for an unknown procedure id. */
int fws_signals_normal_sensed(const FwsSignals *s, unsigned proc_id, signed char *out, int max);

/* Utilities */
int fws_signals_flaps_conf_from_angles(float flap_deg, float slat_deg);
const char *fws_signals_code_str(unsigned id, char buf[16]); /* "%09u" (memo / alert keys) */

#ifdef __cplusplus
}
#endif
#endif
