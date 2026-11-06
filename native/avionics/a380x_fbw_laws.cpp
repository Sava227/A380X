/* SPDX-License-Identifier: GPL-3.0-or-later
 * Native bridge to unmodified FlyByWire A380FgOuterLoops, pinned in vendor/fbw.
 * Units and law selectors follow A380PrimComputerFg.cpp lines 3817-3871 and
 * 4450-4590 at revision 2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2.
 * FCU state selection and the actuator servo are the X-Plane port, not PRIM.
 * LAND/FLARE/ROLL OUT/SRS GA law selection follows A380PrimComputerFg.cpp lines 3815-3870
 * (active_lateral_law / active_longitudinal_law) via the sub-state of a380x_autoland.inc.
 */
#include "a380x_fbw_laws.h"
#include "vendor/fbw/A380FgOuterLoops.h"
#include <cmath>
#include <new>

static double bound(double x, double lo, double hi) { return std::fmax(lo, std::fmin(x, hi)); }
struct FbwLawContext {
    A380FgOuterLoops law;
    bool initialized=false;
    double altitude_hold_target=0;
};
extern "C" void *a380x_fbw_laws_create(void) {
    auto *context = new (std::nothrow) FbwLawContext;
    if (context) context->law.init();
    return context;
}
extern "C" void a380x_fbw_laws_destroy(void *p) { delete static_cast<FbwLawContext *>(p); }
extern "C" void a380x_fbw_laws_reset(void *p) { if (p) { auto *c=static_cast<FbwLawContext *>(p); c->law.reset(); c->initialized=false; } }
extern "C" int a380x_fbw_laws_step(void *p, const A380XGuidanceInputs *i, A380XGuidanceState *s, double dt) {
    if (!p || dt <= 0) return 0;
    ap_laws_input in{};
    ap_raw_output out{};
    const A380XIlsInputs &ils = i->ils[s->source_side];
    in.time.dt = dt;
    in.time.simulation_time = s->simulation_seconds;
    auto &d = in.data;
    d.Theta_deg=i->pitch_deg; d.Phi_deg=i->roll_deg;
    d.qk_deg_s=i->pitch_rate_deg_s; d.pk_deg_s=i->roll_rate_deg_s; d.rk_deg_s=i->yaw_rate_deg_s;
    d.V_ias_kn=i->ias_knots; d.V_tas_kn=std::fmax(i->tas_knots, 60.0);
    d.V_mach=std::fmax(i->mach, 0.1); d.V_gnd_kn=std::fmax(i->ground_speed_knots, 30.0);
    /* FBW sideslip convention (positive nose-left of the flown path) is opposite to X-Plane's
     * position/beta; derivation in a380x_autoland.inc (FBW_BETA_FROM_XPLANE). */
    d.alpha_deg=i->alpha_deg; d.beta_deg=-i->beta_deg;
    d.H_ft=i->pressure_altitude_ft; d.H_ind_ft=i->altitude_ft;
    d.H_radio_ft=i->radio_height_ft; d.H_dot_ft_min=i->vertical_speed_fpm;
    d.Psi_magnetic_deg=i->heading_deg; d.Psi_magnetic_track_deg=i->track_deg;
    d.Psi_true_deg=i->true_heading_deg; d.Chi_true_deg=i->true_track_deg;
    d.bx_m_s2=i->accel_x_m_s2; d.by_m_s2=i->accel_y_m_s2; d.bz_m_s2=i->accel_z_m_s2;
    const bool loc_valid=ils.valid && ils.loc_valid && std::isfinite(ils.course_deg) && std::isfinite(ils.loc_deviation_dots);
    const bool gs_valid=ils.valid && ils.gs_valid && std::isfinite(ils.gs_deviation_dots) &&
        std::isfinite(ils.gs_angle_deg) && ils.gs_angle_deg>0 && ils.gs_angle_deg<10;
    const A380XLandingState &land=s->land;
    /* In LAND a lost localizer keeps the memorized course (no reversion below 400 ft; AUTOLAND light). */
    d.nav_loc_deg=loc_valid?ils.course_deg:(land.land_active && land.course_valid?land.course_deg:i->heading_deg);
    /* FBW glide path angle is negative for a descending path: the GS TRACK law saturates the stored
     * angle to [-4,-1] deg (A380FgOuterLoops Saturation_*_dj/bn). +3 deg would become -1 deg there. */
    d.nav_gs_deg=-(gs_valid?ils.gs_angle_deg:3.0);
    d.nav_dme_nmi=bound(ils.distance_nm,0.1,100.0);
    /* LAND/ROLL OUT convert the angular deviation with the distance to the localizer antenna when the
     * adapter supplies it (the ILS DME is often co-located with the glide antenna, abeam touchdown). */
    if (land.land_active && ils.localizer_distance_valid && std::isfinite(ils.localizer_distance_nm))
        d.nav_dme_nmi=bound(ils.localizer_distance_nm,0.1,100.0);
    /* FBW subtracts magnetic variation from LOC course to obtain true. */
    d.nav_loc_magvar_deg=i->heading_deg-i->true_heading_deg;
    /* ILS angular half-width: LOC 2.5 deg / 2.5 dots; GS .7 deg / 2.5 dots.
     * LOC sign is needle displacement, verified against the imported law. */
    d.nav_loc_error_deg=loc_valid?ils.loc_deviation_dots:0;
    d.nav_gs_valid=gs_valid;
    d.nav_gs_error_deg=gs_valid?-ils.gs_deviation_dots*0.28:0;
    d.fms_xtk_nmi=i->route_xtk_nm; d.fms_tae_deg=i->route_track_error_deg;
    d.fms_phi_deg=i->route_bank_deg; d.fms_phi_limit_deg=25;
    d.fms_H_c_profile_ft=s->altitude_target_ft; d.fms_H_dot_c_profile_ft_min=s->vs_target_fpm;
    /* Missing envelope data disables those protections: zero VLS and a generous
     * VMAX are numerical placeholders, never displayed as aircraft speeds. */
    d.VLS_kn=i->vls_knots>0?i->vls_knots:0;
    d.VMAX_kn=i->vmax_knots>0?i->vmax_knots:450;
    d.on_ground=i->on_ground; d.total_weight_kg=bound(i->weight_kg,250000,650000);
    /* Rudder position (FBW zeta, degrees, positive nose-left): ROLL OUT stores it at engagement. */
    d.zeta_deg=-30.0*bound(land.yaw_active?land.yaw_servo:(std::isfinite(i->rudder_ratio)?i->rudder_ratio:0.0),-1.0,1.0);
    auto &u=in.input;
    u.ap_engaged=s->ap1 || s->ap2;
    switch (s->lateral_mode) {
        case A380X_C1_HDG: u.lateral_law=1; break;
        case A380X_C1_TRACK: u.lateral_law=2; break;
        case A380X_C1_NAV: u.lateral_law=3; break;
        case A380X_C1_LOC_STAR: u.lateral_law=4; break;
        case A380X_C1_LOC: u.lateral_law=5; break;
        case A380X_C1_GA_TRK: u.lateral_law=2; break;
        default: u.lateral_law=0;
    }
    if (land.land_active && s->lateral_mode==A380X_C1_LOC) u.lateral_law=land.rollout_active?6:5;
    switch (s->vertical_mode) {
        case A380X_B1_ALT: u.vertical_law=1; break;
        case A380X_B1_ALT_STAR: u.vertical_law=2; break;
        case A380X_B1_OP_CLB: case A380X_B1_OP_DES: u.vertical_law=3; break;
        case A380X_B1_VS: u.vertical_law=4; break;
        case A380X_B1_FPA: u.vertical_law=5; break;
        case A380X_B1_GS: case A380X_B1_GS_STAR: u.vertical_law=6; break;
        case A380X_B1_CLB: case A380X_B1_DES: u.vertical_law=9; break;
        case A380X_B1_SRS: u.vertical_law=8; break;
        default: u.vertical_law=0;
    }
    if (land.land_active && land.flare_active) u.vertical_law=7;
    /* Heading law uses magnetic heading; TRUE selection is converted at this boundary. */
    u.Psi_c_deg=s->selected_heading_deg-(s->true_heading?(i->true_heading_deg-i->heading_deg):0);
    u.Chi_c_deg=s->lateral_mode==A380X_C1_GA_TRK?land.ga_track_deg:u.Psi_c_deg;
    u.H_c_ft=s->altitude_target_ft; u.H_dot_c_fpm=s->vs_target_fpm;
    u.FPA_c_deg=s->selected_fpa_deg; u.V_c_kn=s->speed_target_knots;
    u.GS_track_mode=s->vertical_mode==A380X_B1_GS;
    auto *context=static_cast<FbwLawContext *>(p);
    /* ALT_HOLD latches pressure-corrected target while its law is inactive.
     * PRIM normally provides this transition. Seed it when starting in flight
     * or when ALT pb requests a new hold altitude, including ALT-to-ALT. */
    if (u.vertical_law==1 && (!context->initialized || context->altitude_hold_target!=s->altitude_target_ft)) {
        ap_laws_input seed=in; seed.input.vertical_law=0;
        context->law.step(&seed,&out);
    }
    context->initialized=true; context->altitude_hold_target=s->altitude_target_ft;
    context->law.step(&in,&out);
    if (!std::isfinite(out.autopilot.Theta_c_deg) || !std::isfinite(out.autopilot.Phi_c_deg) ||
        !std::isfinite(out.flight_director.Theta_c_deg) || !std::isfinite(out.flight_director.Phi_c_deg)) return 0;
    s->pitch_command_deg=bound(out.autopilot.Theta_c_deg,-15,25);
    s->roll_command_deg=bound(out.autopilot.Phi_c_deg,-30,30);
    /* FBW pitch FD command uses screen coordinates (negative means fly up).
     * The C API normalizes it to positive nose up; roll is positive right. */
    s->fd_pitch_error_deg=-out.flight_director.Theta_c_deg;
    s->fd_roll_error_deg=out.flight_director.Phi_c_deg;
    s->land.law_flare_condition=out.flare_law.condition_Flare;
    s->land.law_beta_c_deg=std::isfinite(out.autopilot.Beta_c_deg)?out.autopilot.Beta_c_deg:0;
    s->land.law_nosewheel_c=std::isfinite(out.Nosewheel_c)?out.Nosewheel_c:0;
    s->land.law_rudder_storage_deg=d.zeta_deg;
    return 1;
}
