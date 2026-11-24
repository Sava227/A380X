/* SPDX-License-Identifier: GPL-3.0-or-later
 * X-Plane boundary of the native FWS (a380x_fws.h): fills FwsInputs from the native A380 systems
 * (electrical network, ELEC panel, APU ECB, ADIRS, center-right brakes, FCU guidance, FMS) and
 * stock X-Plane datarefs following docs/FWS_SIGNALS.md, samples the ECP keys and MASTER WARN/CAUT
 * through pass-through command handlers, and publishes the WD view to the EWD and the master
 * lights to the glareshield. Unity-build include of a380x_avionics.c after the EWD data adapter,
 * the electrical, ADIRS, center-right, guidance and FMS adapters.
 */
#include "a380x_fws.h"

enum {
    FR_N1, FR_N2, FR_EGT, FR_BURNING, FR_STARTER, FR_MIXTURE, FR_OIL_LOW, FR_RED_EGT, FR_RED_N1, FR_RED_N2,
    FR_ENG_AI, FR_WING_AI, FR_BLEED_SOV, FR_MODE_SEL, FR_ENG_FIRE, FR_APU_FIRE, FR_FIRE_PB,
    FR_APU_BLEED, FR_APU_N1, FR_PACK_L, FR_PACK_R, FR_CAB_ALT, FR_CAB_DP,
    FR_ADR_IAS, FR_ADR_MACH, FR_ADR_ALT, FR_GROUNDSPEED, FR_RA_PILOT, FR_RA_COPILOT, FR_RA_FAIL_P, FR_RA_FAIL_C,
    FR_ALT_PILOT, FR_STALL, FR_ON_GROUND_ANY, FR_GEAR_ON_GROUND, FR_GEAR_HANDLE, FR_GEAR_DEPLOY, FR_PARK_BRAKE,
    FR_FLAP_HANDLE, FR_FLAP1, FR_SLAT1, FR_FAIL_FLAP, FR_FAIL_SLAT, FR_SPEEDBRAKE, FR_STAB, FR_RUDDER_TRIM,
    FR_YOKE_PITCH, FR_YOKE_ROLL, FR_YOKE_HEADING,
    FR_FUEL_TANKS, FR_FUEL_TOTAL, FR_FUEL_PUMP, FR_CROSSFEED,
    FR_HYD_1, FR_HYD_2, FR_HYD_FLUID_1, FR_HYD_FLUID_2, FR_FAIL_HYD_1, FR_FAIL_HYD_2,
    FR_SEAT_BELTS, FR_NO_SMOKING, FR_STROBE, FR_BEACON, FR_O2, FR_DOORS,
    FR_XPDR_MODE, FR_FAIL_XPDR, FR_ICE, FR_TAT,
    FR_COUNT
};
static const char *const fwsa_names[FR_COUNT] = {
    "sim/cockpit2/engine/indicators/N1_percent", "sim/cockpit2/engine/indicators/N2_percent",
    "sim/flightmodel2/engines/EGT_deg_cel", "sim/flightmodel2/engines/engine_is_burning_fuel",
    "sim/flightmodel2/engines/starter_is_running", "sim/cockpit2/engine/actuators/mixture_ratio",
    "sim/cockpit2/annunciators/oil_pressure_low", "sim/aircraft/limits/red_hi_EGT",
    "sim/aircraft/limits/red_hi_N1", "sim/aircraft/limits/red_hi_N2",
    "sim/cockpit2/ice/ice_inlet_heat_on_per_engine", "sim/cockpit2/ice/ice_surface_hot_bleed_air_on",
    "sim/cockpit2/bleedair/actuators/engine_bleed_sov", "sim/cockpit2/engine/actuators/eng_mode_selector",
    "sim/cockpit2/annunciators/engine_fires", "sim/operation/failures/rel_apu_fire",
    "a380x/xp12/engine/fire_pushbutton",
    "sim/cockpit2/bleedair/actuators/apu_bleed", "sim/cockpit2/electrical/APU_N1_percent",
    "sim/cockpit2/bleedair/actuators/pack_left", "sim/cockpit2/bleedair/actuators/pack_right",
    "sim/cockpit2/pressurization/indicators/cabin_altitude_ft",
    "sim/cockpit2/pressurization/indicators/pressure_diffential_psi",
    "sim/cockpit2/gauges/indicators/airspeed_kts_pilot", "sim/flightmodel/misc/machno",
    "sim/flightmodel2/position/pressure_altitude", "sim/flightmodel/position/groundspeed",
    "sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot",
    "sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_copilot",
    "sim/operation/failures/rel_pil_radalt", "sim/operation/failures/rel_cop_radalt",
    "sim/cockpit2/gauges/indicators/altitude_ft_pilot", "sim/cockpit2/annunciators/stall_warning",
    "sim/flightmodel/failures/onground_any", "sim/flightmodel2/gear/on_ground",
    "sim/cockpit2/controls/gear_handle_down", "sim/flightmodel2/gear/deploy_ratio",
    "sim/flightmodel/controls/parkbrake",
    "sim/cockpit2/controls/flap_handle_request_ratio", "sim/flightmodel2/controls/flap1_deploy_ratio",
    "sim/flightmodel2/controls/slat1_deploy_ratio", "sim/operation/failures/rel_flap_act",
    "sim/operation/failures/rel_fc_slt", "sim/cockpit2/controls/speedbrake_ratio",
    "sim/flightmodel2/controls/stabilizer_deflection_degrees", "sim/cockpit2/controls/rudder_trim",
    "sim/cockpit2/controls/yoke_pitch_ratio", "sim/cockpit2/controls/yoke_roll_ratio",
    "sim/cockpit2/controls/yoke_heading_ratio",
    "sim/flightmodel/weight/m_fuel", "sim/flightmodel/weight/m_fuel_total",
    "sim/cockpit2/engine/actuators/fuel_pump_on", "sim/cockpit2/annunciators/crossfeed_on",
    "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1", "sim/cockpit2/hydraulics/indicators/hydraulic_pressure_2",
    "sim/cockpit2/hydraulics/indicators/hydraulic_fluid_ratio_1",
    "sim/cockpit2/hydraulics/indicators/hydraulic_fluid_ratio_2",
    "sim/operation/failures/rel_hydpmp", "sim/operation/failures/rel_hydpmp2",
    "sim/cockpit2/switches/fasten_seat_belts", "sim/cockpit2/switches/no_smoking",
    "sim/cockpit2/switches/strobe_lights_on", "sim/cockpit2/switches/beacon_on",
    "sim/cockpit2/oxygen/actuators/o2_valve_on", "sim/flightmodel2/misc/door_open_ratio",
    "sim/cockpit2/radios/actuators/transponder_mode", "sim/operation/failures/rel_xpndr",
    "sim/flightmodel/failures/frm_ice", "sim/cockpit2/temperature/outside_air_LE_temp_degc",
};

/* Pass-through command taps: level while held + a latch so a press inside one frame is not lost. */
enum {
    FC_CLR, FC_CLR2, FC_RCL, FC_CL, FC_CHECK_LH, FC_CHECK_RH, FC_UP, FC_DOWN, FC_ABN, FC_TO_CONFIG,
    FC_MW_L, FC_MW_R, FC_MC_L, FC_MC_R, FC_AP_DISC, FC_ATHR_DISC, FC_FIRE_TEST, FC_COUNT
};
static const char *const fwsa_command_names[FC_COUNT] = {
    "a380x/xp12/ecam/sd/clear_left", "a380x/xp12/ecam/sd/clear_right", "a380x/xp12/ecam/sd/recall",
    "a380x/xp12/ecam/sd/checklist", "a380x/xp12/ecam/sd/check_left", "a380x/xp12/ecam/sd/check_right",
    "a380x/xp12/ecam/sd/up", "a380x/xp12/ecam/sd/down", "a380x/xp12/ecam/sd/abnormal_procedures",
    "a380x/xp12/ecam/sd/to_config",
    "a380x/xp12/glareshield/master_warning_left", "a380x/xp12/glareshield/master_warning_right",
    "a380x/xp12/glareshield/master_caution_left", "a380x/xp12/glareshield/master_caution_right",
    "a380x/xp12/fcu/ap_disconnect", "a380x/xp12/fcu/athr_disconnect", "a380x/xp12/fire/test",
};

static struct {
    XPLMDataRef ref[FR_COUNT];
    XPLMCommandRef command[FC_COUNT];
    unsigned char held[FC_COUNT], latched[FC_COUNT];
    unsigned presses[FC_COUNT];
    double bind_timer, clock;
    int ready, active;
    FwsInputs in;
} fwsa;
static Fws fwsa_fws;   /* large: static storage */

static int fwsa_command(XPLMCommandRef cmd, XPLMCommandPhase phase, void *refcon)
{
    int i = (int)(intptr_t)refcon;
    (void)cmd;
    if (i < 0 || i >= FC_COUNT) return 1;
    if (phase == xplm_CommandBegin) { fwsa.held[i] = 1; fwsa.latched[i] = 1; ++fwsa.presses[i]; }
    else if (phase == xplm_CommandEnd) fwsa.held[i] = 0;
    return 1;   /* never consume: the owning module handles the command as before */
}
static int fwsa_level(int i) { int v = fwsa.held[i] || fwsa.latched[i]; fwsa.latched[i] = 0; return v; }

static void fwsa_bind(void)
{
    int i;
    for (i = 0; i < FR_COUNT; ++i) if (!fwsa.ref[i]) fwsa.ref[i] = XPLMFindDataRef(fwsa_names[i]);
    for (i = 0; i < FC_COUNT; ++i) if (!fwsa.command[i]) {
        /* find or create: the FIRE TEST command belongs to the feedback module, registered later */
        fwsa.command[i] = XPLMCreateCommand(fwsa_command_names[i], fwsa_command_names[i]);
        if (fwsa.command[i]) XPLMRegisterCommandHandler(fwsa.command[i], fwsa_command, 1, (void *)(intptr_t)i);
    }
}

/* Typed reads: return 1 and the value only when the dataref exists with a finite value. */
static int fwsa_f(int r, float *out)
{
    XPLMDataRef ref = fwsa.ref[r];
    XPLMDataTypeID t;
    float v;
    if (!ref) return 0;
    t = XPLMGetDataRefTypes(ref);
    if (t & xplmType_Float) v = XPLMGetDataf(ref);
    else if (t & xplmType_Double) v = (float)XPLMGetDatad(ref);
    else if (t & xplmType_Int) v = (float)XPLMGetDatai(ref);
    else return 0;
    if (!isfinite(v)) return 0;
    *out = v;
    return 1;
}
static int fwsa_i(int r, int *out) { float v; if (!fwsa_f(r, &v)) return 0; *out = (int)lroundf(v); return 1; }
static int fwsa_vf(int r, float *out, int count)
{
    XPLMDataRef ref = fwsa.ref[r];
    XPLMDataTypeID t;
    int i, n;
    if (!ref) return 0;
    t = XPLMGetDataRefTypes(ref);
    if (t & xplmType_FloatArray) n = XPLMGetDatavf(ref, out, 0, count);
    else if (t & xplmType_IntArray) {
        int v[32];
        if (count > 32) count = 32;
        n = XPLMGetDatavi(ref, v, 0, count);
        for (i = 0; i < n; ++i) out[i] = (float)v[i];
    } else return 0;
    for (i = 0; i < n; ++i) if (!isfinite(out[i])) out[i] = 0.0f;
    return n >= count;
}

static void fwsa_read_inputs(FwsInputs *in)
{
    float v4[4], v8[8], v10[20], f;
    int i, k, iv;
    fws_inputs_defaults(in);

    /* ---- ELEC: native A380 network (a380x_elec_adapter.inc) ---- */
    in->elec_valid = (unsigned char)(eln.ready && eln.active && a380x_power.available);
    if (in->elec_valid) {
        const A380XElnState *st = &eln.st;
        for (k = 0; k < 4; ++k) in->ac_powered[k] = (unsigned char)a380x_eln_powered(st, A380X_ELN_AC1 + k);
        in->ac_ess_powered = (unsigned char)a380x_eln_powered(st, A380X_ELN_AC_ESS);
        in->ac_emer_powered = (unsigned char)a380x_eln_powered(st, A380X_ELN_AC_EMER);
        in->dc1_powered = (unsigned char)a380x_eln_powered(st, A380X_ELN_DC1);
        in->dc2_powered = (unsigned char)a380x_eln_powered(st, A380X_ELN_DC2);
        in->dc_ess_powered = (unsigned char)a380x_eln_powered(st, A380X_ELN_DC_ESS);
        in->dc_ess_inflt_powered = (unsigned char)a380x_eln_powered(st, A380X_ELN_DC_ESS_INFLT);
        in->dc_eha_powered = (unsigned char)a380x_eln_powered(st, A380X_ELN_DC_EHA);
        for (k = 0; k < 4; ++k) {
            in->gen_pb_on[k] = eln.ctl.gen_on[k];
            in->gen_fault[k] = eln.in.fail.gen[k];
            in->gen_potential_normal[k] = (unsigned char)(st->gen_avail[k] != 0);
            in->bat_pb_auto[k] = eln.ctl.bat_auto[k];
            in->bat_fault[k] = st->lamps.bat_fault[k];
            in->ext_avail[k] = (unsigned char)(st->ext_avail[k] != 0);
            in->ext_pb_on[k] = eln.ctl.ext_on[k];
            in->ext_contactor_closed[k] = (unsigned char)a380x_eln_contactor(st, A380X_ELN_K_990XG1 + k);
            in->drive_disc[k] = eln.ctl.drive_disc[k];
            in->tr_fault[k] = eln.in.fail.tr[k];
        }
        for (k = 0; k < 2; ++k) {
            in->apu_gen_pb_on[k] = eln.ctl.apu_gen_on[k];
            in->apu_gen_fault[k] = eln.in.fail.apu_gen[k];
            in->comm_on[k] = eln.ctl.comm_on[k];
        }
        in->ext_connected = eln.in.ext_connected[0];
        in->bus_tie_auto = eln.ctl.bus_tie_auto;
        in->ac_ess_feed_normal = eln.ctl.ac_ess_feed_normal;
        in->galley_auto = eln.ctl.galley_auto;
        in->rat_stow_ratio = (float)st->rat_stow;
        in->rat_man_on = eln.ctl.rat_man_on;
        in->rat_fault = st->lamps.rat_fault;
        in->stat_inv_fault = eln.in.fail.stat_inv;
        in->apu_master_on = (unsigned char)(st->apu_master != 0);
        in->apu_avail = (unsigned char)(st->apu_avail != 0);
    }

    /* ---- ENGINES ---- */
    if (fwsa_vf(FR_N1, v4, 4)) {
        float n2[4] = {0}, egt[4] = {0}, burning[4] = {0}, starter[4] = {0}, mixture[4] = {0};
        in->eng_valid = (unsigned char)(fwsa_vf(FR_N2, n2, 4) && fwsa_vf(FR_EGT, egt, 4) &&
                                        fwsa_vf(FR_BURNING, burning, 4) && fwsa_vf(FR_STARTER, starter, 4) &&
                                        fwsa_vf(FR_MIXTURE, mixture, 4));
        for (k = 0; k < 4 && in->eng_valid; ++k) {
            int running = burning[k] > 0.5f && starter[k] < 0.5f;
            in->n1_pct[k] = v4[k]; in->n3_pct[k] = n2[k]; in->egt_c[k] = egt[k];
            in->eng_master_on[k] = mixture[k] > 0.5f;
            in->eng_state[k] = running ? FWS_ENG_ON
                             : (starter[k] > 0.5f || (in->eng_master_on[k] && n2[k] > 5.0f && n2[k] < 50.0f)) ? FWS_ENG_STARTING
                             : (!in->eng_master_on[k] && n2[k] > 5.0f) ? FWS_ENG_SHUTTING : FWS_ENG_OFF;
        }
        if (ewd.engine[0].idle_n1_valid) in->n1_idle_pct = ewd.engine[0].idle_n1;
    }
    {
        float egt_red, n1_red, n2_red;
        in->eng_limits_valid = (unsigned char)(fwsa_f(FR_RED_EGT, &egt_red) && fwsa_f(FR_RED_N1, &n1_red) &&
                                               fwsa_f(FR_RED_N2, &n2_red) && egt_red > 0 && n1_red > 0 && n2_red > 0);
        if (in->eng_limits_valid) { in->egt_red_c = egt_red; in->n1_red_pct = n1_red; in->n3_red_pct = n2_red; }
    }
    if (fwsa_vf(FR_OIL_LOW, v4, 4)) for (k = 0; k < 4; ++k) in->oil_press_low[k] = v4[k] > 0.5f && in->eng_state[k] == FWS_ENG_ON;
    if (fwsa_vf(FR_ENG_AI, v4, 4)) for (k = 0; k < 4; ++k) in->eng_anti_ice_on[k] = v4[k] > 0.5f;
    if (fwsa_i(FR_WING_AI, &iv)) in->wing_anti_ice_on = iv != 0;
    if (fwsa_vf(FR_BLEED_SOV, v4, 4)) for (k = 0; k < 4; ++k) in->eng_bleed_pb_auto[k] = v4[k] > 0.5f;
    if (fwsa_i(FR_MODE_SEL, &iv)) in->eng_mode_selector = (unsigned char)(iv < 0 ? 0 : iv == 0 ? 1 : 2);
    /* TLA on FBW's scale from the calibrated levers (a380x_ewd_data.inc ewd_lever_tla). */
    if (guidance_inputs.throttle_levers_valid && guidance_inputs.cl_detent_ratio > guidance_inputs.idle_detent_ratio &&
        guidance_inputs.mct_detent_ratio > guidance_inputs.cl_detent_ratio &&
        guidance_inputs.toga_detent_ratio > guidance_inputs.mct_detent_ratio) {
        in->tla_valid = 1;
        for (k = 0; k < 4; ++k)
            in->tla_deg[k] = (float)ewd_lever_tla(guidance_inputs.throttle_lever_ratio[k], guidance_inputs.idle_detent_ratio,
                guidance_inputs.cl_detent_ratio, guidance_inputs.mct_detent_ratio, guidance_inputs.toga_detent_ratio,
                guidance_inputs.detent_tolerance_ratio, guidance_inputs.reverse_selected[k]);
    }
    in->flex_temp_set = (unsigned char)(fmsa_fms.out.valid && fmsa_fms.out.to_flex_temp_c != 0.0);
    in->thrust_limit_flex = (unsigned char)(!strcmp(ewd.thrust_mode, "FLX"));
    /* ---- FIRE ---- */
    if (fwsa_vf(FR_ENG_FIRE, v4, 4)) for (k = 0; k < 4; ++k) in->eng_fire_detected[k] = v4[k] > 0.5f;
    if (fwsa_i(FR_APU_FIRE, &iv)) in->apu_fire_detected = iv == 6;
    if (fwsa_vf(FR_FIRE_PB, v4, 4)) for (k = 0; k < 4; ++k) in->eng_fire_pb_released[k] = v4[k] > 0.5f;

    /* ---- APU / BLEED / AIR ---- */
    if (fwsa_i(FR_APU_BLEED, &iv)) in->apu_bleed_pb_on = in->apu_bleed_valve_open = iv != 0;
    if (fwsa_f(FR_APU_N1, &f)) { in->apu_n_valid = 1; in->apu_n_pct = f; }
    if (fwsa_i(FR_PACK_L, &iv)) in->pack_pb_on[0] = iv != 0;
    if (fwsa_i(FR_PACK_R, &iv)) in->pack_pb_on[1] = iv != 0;
    {
        float alt, dp;
        if (fwsa_f(FR_CAB_ALT, &alt) && fwsa_f(FR_CAB_DP, &dp)) { in->cabin_alt_valid = 1; in->cabin_alt_ft = alt; in->cabin_diff_psi = dp; }
    }

    /* ---- AIR DATA / ADIRS ---- */
    {
        float ias = 0, mach = 0, alt = 0, gs = 0;
        int have_air = fwsa_f(FR_ADR_IAS, &ias) && fwsa_f(FR_ADR_MACH, &mach) && fwsa_f(FR_ADR_ALT, &alt);
        int have_gs = fwsa_f(FR_GROUNDSPEED, &gs);
        double remaining = 0;
        for (k = 0; k < 3; ++k) {
            const A380XAdirsUnitState *u = &adirs.unit[k];
            in->adr_cas_valid[k] = in->adr_mach_valid[k] = in->adr_alt_valid[k] = (unsigned char)(have_air && u->adr_valid);
            in->adr_cas_kt[k] = ias; in->adr_mach[k] = mach; in->adr_alt_ft[k] = alt;
            in->adr_fault[k] = (unsigned char)(u->adr_fault_lit != 0);
            in->adr_pb_on[k] = (unsigned char)!u->adr_off;
            in->ir_pb_on[k] = (unsigned char)!u->ir_off;
            in->ir_fault[k] = (unsigned char)(u->ir_fault_lit != 0);
            in->ir_in_align[k] = (unsigned char)(u->aligning != 0);
            in->ir_mode[k] = (unsigned char)(u->mode < 0 ? 0 : u->mode > 2 ? 2 : u->mode);
            if (u->aligning && u->alignment_remaining > remaining) remaining = u->alignment_remaining;
            in->ir_gs_valid[k] = (unsigned char)(have_gs && u->nav_valid);
            in->ir_gs_kt[k] = gs * 1.94384f;
        }
        in->ir_align_remaining_s = (float)remaining;
    }
    {
        float ra_p = 0, ra_c = 0;
        int fail_p = 0, fail_c = 0, ok_p = fwsa_f(FR_RA_PILOT, &ra_p), ok_c = fwsa_f(FR_RA_COPILOT, &ra_c);
        fwsa_i(FR_RA_FAIL_P, &fail_p); fwsa_i(FR_RA_FAIL_C, &fail_c);
        in->ra_ft[0] = in->ra_ft[2] = ra_p; in->ra_ft[1] = ra_c;
        in->ra_ssm[0] = in->ra_ssm[2] = (unsigned char)(!ok_p || fail_p >= 6 ? FWS_SSM_FAILURE : ra_p > 8192.0f ? FWS_SSM_NCD : FWS_SSM_NORMAL);
        in->ra_ssm[1] = (unsigned char)(!ok_c || fail_c >= 6 ? FWS_SSM_FAILURE : ra_c > 8192.0f ? FWS_SSM_NCD : FWS_SSM_NORMAL);
    }
    if (fwsa_f(FR_ALT_PILOT, &f)) in->indicated_alt_ft = f;
    if (fwsa_i(FR_STALL, &iv)) in->stall_warning_raw = iv != 0;

    /* ---- GROUND / GEAR / BRAKES ---- */
    if (fwsa_i(FR_ON_GROUND_ANY, &iv)) in->sim_on_ground = iv != 0;
    if (fwsa_vf(FR_GEAR_ON_GROUND, v10, 5)) {
        in->main_gear_compressed = 0;
        for (k = 1; k < 5; ++k) if (v10[k] > 0.5f) in->main_gear_compressed = 1;
    }
    /* The physical lever (FBW GEAR_LEVER_POSITION_REQUEST): center-right reads X-Plane's handle,
     * or the crew lever while L/G GRVTY holds that handle (a380x_lg_grvty.inc). */
    if (center_right.lever_ref) in->gear_lever_down = (unsigned char)(center_right.gear_lever != 0);
    else if (fwsa_i(FR_GEAR_HANDLE, &iv)) in->gear_lever_down = iv != 0;
    in->lg_grvty_switch_pos = (unsigned char)lgg_switch_position();
    if (fwsa_vf(FR_GEAR_DEPLOY, v10, 5)) {
        float sum = 0;
        in->gear_valid = 1;
        in->nose_downlocked = v10[0] >= 0.99f;
        in->main_left_downlocked = v10[1] >= 0.99f && v10[3] >= 0.99f;
        in->main_right_downlocked = v10[2] >= 0.99f && v10[4] >= 0.99f;
        in->all_gear_uplocked = 1;
        for (k = 0; k < 5; ++k) { sum += v10[k]; if (v10[k] > 0.01f) in->all_gear_uplocked = 0; }
        in->gear_extended_ratio = sum / 5.0f;
        in->nose_gear_ratio = v10[0];
    }
    if (fwsa_f(FR_PARK_BRAKE, &f)) in->park_brake_set = f >= 0.99f;
    in->brakes_valid = 1;
    in->antiskid_on = (unsigned char)(center_right.antiskid_on != 0);
    in->autobrake_active = (unsigned char)(center_right.autobrake_active != 0);
    in->autobrake_mode = (unsigned char)(center_right.rto_arm ? 6 :
        center_right.autobrake_selected < 0 ? 0 : center_right.autobrake_selected > 5 ? 5 : center_right.autobrake_selected);
    in->autobrake_fault = (unsigned char)!center_right.autobrake_available;
    in->norm_brake_fault = (unsigned char)!center_right.normal_brakes_available;
    in->antiskid_fault = (unsigned char)!center_right.antiskid_available;

    /* ---- FLIGHT CONTROLS ---- */
    if (flaps_feed.ready) {
        /* FLAPS module (a380x_flaps.inc): CSU handle index, FBW SFCC configuration, FPPU surface angles. */
        in->flaps_handle_index = (unsigned char)flaps_feed.handle_index;
        in->flaps_conf_index = (signed char)flaps_feed.conf;
        in->flap_slat_valid = 1;
        in->flap_angle_deg = (float)flaps_feed.flap_deg;
        in->slat_angle_deg = (float)flaps_feed.slat_deg;
    } else {
        if (fwsa_f(FR_FLAP_HANDLE, &f)) in->flaps_handle_index = (unsigned char)lroundf(fminf(4.0f, fmaxf(0.0f, f * 4.0f)));
        {
            float flap, slat;
            if (fwsa_f(FR_FLAP1, &flap) && fwsa_f(FR_SLAT1, &slat)) {
                in->flap_slat_valid = 1; in->flap_angle_deg = flap * 33.0f; in->slat_angle_deg = slat * 23.0f;
            }
        }
    }
    if (fwsa_i(FR_FAIL_FLAP, &iv) && iv >= 6) in->flap_sys_fault[0] = in->flap_sys_fault[1] = in->flaps_jammed = 1;
    if (fwsa_i(FR_FAIL_SLAT, &iv) && iv >= 6) in->slat_sys_fault[0] = in->slat_sys_fault[1] = in->slats_jammed = 1;
    /* SPEED BRAKE module (docs/SPEED_BRAKE.md): the lever, since X-Plane's handle carries the surface order. */
    if (speedbrake_feed.ready) {
        in->gnd_splr_armed = (unsigned char)(speedbrake_feed.armed != 0);
        in->speedbrake_lever_out = (unsigned char)(speedbrake_feed.lever_out != 0);
    } else if (fwsa_f(FR_SPEEDBRAKE, &f)) { in->gnd_splr_armed = f < -0.01f; in->speedbrake_lever_out = f > 0.05f; }
    if (fwsa_f(FR_STAB, &f)) { in->pitch_trim_valid = 1; in->pitch_trim_deg = f; }
    if (fwsa_f(FR_RUDDER_TRIM, &f)) { in->rudder_trim_valid = 1; in->rudder_trim_deg = f * 25.0f; }
    {
        float p = 0, r = 0, h = 0;
        if (fwsa_f(FR_YOKE_PITCH, &p) && fwsa_f(FR_YOKE_ROLL, &r) && fwsa_f(FR_YOKE_HEADING, &h))
            in->controls_moved = fabsf(p) > 0.05f || fabsf(r) > 0.05f || fabsf(h) > 0.2f;
    }

    /* ---- AUTO FLIGHT (guidance) ---- */
    in->ap_engaged = (unsigned char)(guidance.ap1 || guidance.ap2);
    in->fd1_on = (unsigned char)(guidance.fd1 != 0); in->fd2_on = (unsigned char)(guidance.fd2 != 0);
    in->ap_instinctive_disc_count = fwsa.presses[FC_AP_DISC]; fwsa.presses[FC_AP_DISC] = 0;
    in->athr_instinctive_disc = (unsigned char)(fwsa.presses[FC_ATHR_DISC] > 0); fwsa.presses[FC_ATHR_DISC] = 0;
    in->athr_status = (unsigned char)(guidance.athr_active ? 2 : guidance.athr_armed ? 1 : 0);
    in->athr_mode_active = (unsigned char)(guidance.thrust_mode != A380X_A1A2_NONE);
    in->afs_a3 = (unsigned char)published.fma_a3;
    in->afs_a1a2 = (unsigned char)guidance.thrust_mode;
    in->fcu_alt_ft = (float)guidance.selected_altitude_ft;
    in->gs_captured = (unsigned char)(guidance.vertical_mode == A380X_B1_GS || guidance.vertical_mode == A380X_B1_F_GS ||
                                      guidance.vertical_mode == A380X_B1_GS_STAR || guidance.vertical_mode == A380X_B1_F_GS_STAR);

    /* ---- FMS (native) ---- */
    {
        const A380XFmsOutputs *o = &fmsa_fms.out;
        in->fms_valid = (unsigned char)(o->valid != 0);
        if (in->fms_valid) {
            in->v1_kt = (float)o->v1_knots;
            in->fms_to_speeds_not_inserted = (unsigned char)(o->to_speeds_not_inserted != 0);
            in->fms_to_speeds_too_low = (unsigned char)(o->to_speeds_too_low != 0);
            in->fms_v_disagree = (unsigned char)(o->vspeeds_disagree != 0);
            in->fms_to_flaps = (signed char)(o->to_flaps >= 1 && o->to_flaps <= 3 ? o->to_flaps : -1);
            in->fms_to_pitch_trim_valid = (unsigned char)!isnan(o->to_ths_for_pct);
            in->fms_to_pitch_trim_cg = isnan(o->to_ths_for_pct) ? 0.0f : (float)o->to_ths_for_pct;
            in->fms_landing_conf3 = (unsigned char)(o->landing_conf3 != 0);
            in->fms_zfw_zfwcg_set = (unsigned char)(!isnan(o->zfw_kg) && !isnan(o->zfw_cg_pct));
            in->fms_dest_efob_below_min = (unsigned char)(o->dest_efob_below_min != 0);
            in->landing_elev_valid = (unsigned char)(o->landing_elevation_valid != 0);
            in->landing_elev_ft = (float)o->landing_elevation_ft;
            in->fg_alt_constraint_ft = (float)o->fg_altitude_constraint_ft;
        }
    }

    /* ---- FUEL (feed tanks 1..4 = m_fuel[1..4]) ---- */
    if (fwsa_vf(FR_FUEL_TANKS, v10, 9) && fwsa_f(FR_FUEL_TOTAL, &f)) {
        in->fuel_valid = 1;
        for (k = 0; k < 4; ++k) in->feed_tank_kg[k] = v10[k + 1];
        in->fuel_on_board_kg = f;
    }
    if (fwsa_vf(FR_FUEL_PUMP, v8, 4)) for (k = 0; k < 4; ++k) in->feed_pump_main_on[k] = v8[k] > 0.5f;
    if (fwsa_i(FR_CROSSFEED, &iv)) for (k = 0; k < 4; ++k) in->crossfeed_open[k] = iv != 0;

    /* ---- HYDRAULICS (1 = green, 2 = yellow) ---- */
    {
        float g, y;
        if (fwsa_f(FR_HYD_1, &g) && fwsa_f(FR_HYD_2, &y)) {
            int fail_g = 0, fail_y = 0;
            in->hyd_valid = 1; in->hyd_green_psi = g; in->hyd_yellow_psi = y;
            fwsa_i(FR_FAIL_HYD_1, &fail_g); fwsa_i(FR_FAIL_HYD_2, &fail_y);
            /* green: engines 1+2, yellow: engines 3+4; a section is low unless its engine runs. */
            for (k = 0; k < 4; ++k) {
                int sys_failed = k < 2 ? fail_g >= 6 : fail_y >= 6;
                in->eng_pump_section_low[k][0] = in->eng_pump_section_low[k][1] =
                    (unsigned char)!(in->eng_state[k] == FWS_ENG_ON && !sys_failed);
            }
        }
        if (fwsa_f(FR_HYD_FLUID_1, &g)) in->green_rsv_level_low = g < 0.3f;
        if (fwsa_f(FR_HYD_FLUID_2, &y)) in->yellow_rsv_level_low = y < 0.3f;
    }

    /* ---- CABIN / LIGHTS / DOORS / SURVEILLANCE ---- */
    if (fwsa_i(FR_SEAT_BELTS, &iv)) in->seat_belts_on = iv != 0;
    if (fwsa_i(FR_NO_SMOKING, &iv)) in->no_mobile_on = iv != 0;
    if (fwsa_i(FR_STROBE, &iv)) in->strobe_off = iv == 0;
    if (fwsa_i(FR_BEACON, &iv)) in->beacon_on = iv != 0;
    if (fwsa_i(FR_O2, &iv)) in->crew_oxy_off = iv == 0;
    if (fwsa_vf(FR_DOORS, v10, 19)) {
        for (k = 0; k < 10; ++k) in->main_door_open[k] = v10[k] > 0.01f;
        in->cargo_fwd_locked = v10[17] <= 0.01f;
        in->cargo_aft_locked = v10[18] <= 0.01f;
    }
    if (fwsa_i(FR_XPDR_MODE, &iv)) {
        in->xpdr_stby = iv == 1;
        in->xpdr_alt_reporting = (unsigned char)(iv == 3 || iv == 5 || iv == 6 || iv == 7 || in->sim_on_ground);
        in->tcas_mode = (unsigned char)(iv == 6 ? 1 : iv == 7 ? 2 : iv >= 3 ? 2 : 0);
    }
    if (fwsa_i(FR_FAIL_XPDR, &iv)) in->tcas_fault = iv >= 6;
    {
        float ice, tat;
        if (fwsa_f(FR_ICE, &ice) && fwsa_f(FR_TAT, &tat)) { in->ice_valid = 1; in->ice_ratio = ice; in->tat_c = tat; }
    }

    /* ---- crew buttons ---- */
    in->to_config_pb = (unsigned char)fwsa_level(FC_TO_CONFIG);
    in->fire_test_pb = (unsigned char)fwsa_level(FC_FIRE_TEST);
    in->rcl_pb = (unsigned char)(fwsa.held[FC_RCL] || fwsa.latched[FC_RCL]);
    in->master_warning_pb = (unsigned char)(fwsa_level(FC_MW_L) | fwsa_level(FC_MW_R));
    in->master_caution_pb = (unsigned char)(fwsa_level(FC_MC_L) | fwsa_level(FC_MC_R));
    (void)i;
}

static void fwsa_update(float dt, int powered)
{
    FwsCycleInput cycle;
    if (!fwsa.ready) return;
    if (isfinite(dt) && dt > 0.0f) { fwsa.bind_timer += dt; fwsa.clock += dt; } else dt = 0.0f;
    if (fwsa.bind_timer >= 1.0) { fwsa_bind(); fwsa.bind_timer = 0.0; }
    fwsa_read_inputs(&fwsa.in);
    memset(&cycle, 0, sizeof(cycle));
    cycle.ecp.clr = (unsigned char)fwsa_level(FC_CLR);
    cycle.ecp.clr2 = (unsigned char)fwsa_level(FC_CLR2);
    cycle.ecp.rcl = (unsigned char)fwsa_level(FC_RCL);
    cycle.ecp.cl = (unsigned char)fwsa_level(FC_CL);
    cycle.ecp.check_lh = (unsigned char)fwsa_level(FC_CHECK_LH);
    cycle.ecp.check_rh = (unsigned char)fwsa_level(FC_CHECK_RH);
    cycle.ecp.up = (unsigned char)fwsa_level(FC_UP);
    cycle.ecp.down = (unsigned char)fwsa_level(FC_DOWN);
    cycle.ecp.abn_proc = (unsigned char)fwsa_level(FC_ABN);
    cycle.sd_status_shown = (unsigned char)(sda.page == SD_STS);
    cycle.baro_std = (unsigned char)(efis[0].baro_std != 0);
    fws_update(&fwsa_fws, &fwsa.in, &cycle, powered, fwsa.clock, dt);
    fwsa.active = powered != 0;
}

/* After ewd_update(): the FWS replaces the legacy memo / warning lists and the attention getters. */
static void fwsa_publish_ewd(void)
{
    int k;
    if (!fwsa.ready || !fwsa.active || !ewd.powered) return;
    fws_ewd_view(&fwsa_fws, &ewd.wd);
    ewd.attention_valid = 1;
    for (k = 0; k < 4; ++k) {
        ewd.attention_normal[k] = fwsa_fws.out.normal_attention_getter_eng[k];
        ewd.attention_abnormal[k] = fwsa_fws.out.abnormal_primary_eng_attention_getter[k];
    }
}

static int fwsa_master_warning(void) { return fwsa.ready && fwsa.active && fwsa_fws.master_warning; }
static int fwsa_master_caution(void) { return fwsa.ready && fwsa.active && fwsa_fws.master_caution; }
static int fwsa_available(void) { return fwsa.ready && fwsa.active; }

static int fwsa_init(void)
{
    memset(&fwsa, 0, sizeof(fwsa));
    fws_init(&fwsa_fws);
    fwsa_bind();
    fwsa.ready = 1;
    return 1;
}

static void fwsa_plane_loaded(void)
{
    if (!fwsa.ready) return;
    fws_init(&fwsa_fws);
    fwsa.clock = 0.0;
}

static void fwsa_shutdown(void)
{
    int i;
    for (i = 0; i < FC_COUNT; ++i) if (fwsa.command[i]) {
        XPLMUnregisterCommandHandler(fwsa.command[i], fwsa_command, 1, (void *)(intptr_t)i);
        fwsa.command[i] = NULL;
    }
    fwsa.ready = 0;
}
