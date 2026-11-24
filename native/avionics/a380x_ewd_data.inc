/* SPDX-License-Identifier: GPL-3.0-or-later
 * Read-only X-Plane EWD adapter. Standard datarefs verified against the
 * X-Plane 12 Resources/plugins/DataRefs.txt catalog. FBW sources:
 * EWD/elements/{ThrustGauge,ThrustRatingMode,Idle,BleedSupply}.tsx,
 * FlightWarningSystem/FwsMemos.ts and MsfsAvionicsCommon/EcamMessages.
 *
 * Deliberately bounded: this is not the A380 FWS, ECL or FADEC. No fabricated
 * checklist, limitation, thrust rating, or healthy-system messages are
 * generated. Unsupported producers are represented by invalid fields.
 * Native thrust in newtons is NOT Airbus indicated THR. THR requires valid
 * idle/max N1 references, from the source FADEC equations or a live producer.
 * Optional a380x/xp12/engine inputs below are consumer contracts for a future
 * FADEC; this adapter never registers them or substitutes nominal constants.
 * Include after the plugin's Published published declaration.
 */
#include "a380x_ewd_data.h"
#include "a380x_ewd_fadec.h"

static EwdData ewd;

enum {
    ER_N1, ER_EGT, ER_FADEC, ER_BURNING, ER_STARTER, ER_REV_MODE,
    ER_REV_DEPLOY, ER_FIRE, ER_ANTI_ICE, ER_N1_HIGH,
    ER_N1_LIMIT, ER_EGT_LIMIT, ER_N1_FAIL_1, ER_N1_FAIL_2,
    ER_EGT_FAIL_1, ER_EGT_FAIL_2,
    ER_APU_RUNNING, ER_APU_N1, ER_APU_BLEED, ER_SEAT_BELTS,
    ER_PARK_BRAKE, ER_EXT_PWR, ER_WING_ICE, ER_ON_GROUND,
    ER_PACK_L, ER_PACK_R, ER_GEAR_NOT_DOWN, ER_GEAR_NOT_LOCKED_DOWN,
    ER_GEAR_NOT_LOCKED_UP,
    ER_IDLE_N1, ER_MAX_N1, ER_REV_N1, ER_COMMANDED_N1,
    ER_THROTTLE_N1, ER_RATING_N1, ER_RATING_MODE, ER_FLEX,
    ER_ATHR_STATUS, ER_SAT, ER_PRESSURE, ER_PRESSURE_ALT, ER_MACH,
    ER_CAS, ER_CAS_FAIL, ER_ENG_SELECTOR, ER_FIRE_PB, ER_TAT, ER_REF_COUNT
};

typedef struct {
    const char *name;
    int type;
    XPLMDataRef ref;
} EwdInputRef;

#define EWD_INPUT(name, type) {name, type, NULL}
static EwdInputRef ewd_inputs[ER_REF_COUNT] = {
    EWD_INPUT("sim/cockpit2/engine/indicators/N1_percent", xplmType_FloatArray),
    /* Unlike cockpit2 EGT, this flight-model input is always Celsius. */
    EWD_INPUT("sim/flightmodel2/engines/EGT_deg_cel", xplmType_FloatArray),
    EWD_INPUT("sim/cockpit2/engine/actuators/fadec_on", xplmType_IntArray),
    EWD_INPUT("sim/flightmodel2/engines/engine_is_burning_fuel", xplmType_IntArray),
    EWD_INPUT("sim/flightmodel2/engines/starter_is_running", xplmType_IntArray),
    EWD_INPUT("sim/cockpit2/engine/actuators/prop_mode", xplmType_IntArray),
    EWD_INPUT("sim/flightmodel2/engines/thrust_reverser_deploy_ratio", xplmType_FloatArray),
    EWD_INPUT("sim/cockpit2/annunciators/engine_fires", xplmType_IntArray),
    EWD_INPUT("sim/cockpit2/ice/cowling_thermal_anti_ice_per_engine", xplmType_IntArray),
    EWD_INPUT("sim/cockpit2/annunciators/N1_high", xplmType_IntArray),
    EWD_INPUT("sim/aircraft/limits/red_hi_N1", xplmType_Float),
    /* Catalog explicitly documents this threshold in Celsius. The package
     * also sets acf/_EGT_is_C=1; raw EGT above is always Celsius. */
    EWD_INPUT("sim/aircraft/limits/red_hi_EGT", xplmType_Float),
    EWD_INPUT("sim/operation/failures/rel_N1_ind0", xplmType_Int),
    EWD_INPUT("sim/operation/failures/rel_N1_ind1", xplmType_Int),
    EWD_INPUT("sim/operation/failures/rel_EGT_ind_0", xplmType_Int),
    EWD_INPUT("sim/operation/failures/rel_EGT_ind_1", xplmType_Int),
    EWD_INPUT("sim/cockpit2/electrical/APU_running", xplmType_Int),
    EWD_INPUT("a380x/xp12/apu/n_percent", xplmType_Float),
    EWD_INPUT("sim/cockpit2/bleedair/actuators/apu_bleed", xplmType_Int),
    EWD_INPUT("sim/cockpit2/switches/fasten_seat_belts", xplmType_Int),
    EWD_INPUT("sim/flightmodel/controls/parkbrake", xplmType_Float),
    EWD_INPUT("sim/cockpit2/annunciators/external_power_on", xplmType_Int),
    EWD_INPUT("sim/cockpit2/ice/ice_surface_hot_bleed_air_on", xplmType_Int),
    EWD_INPUT("sim/flightmodel/failures/onground_any", xplmType_Int),
    EWD_INPUT("sim/cockpit2/bleedair/actuators/pack_left", xplmType_Int),
    EWD_INPUT("sim/cockpit2/bleedair/actuators/pack_right", xplmType_Int),
    EWD_INPUT("a380x/xp12/center_right/gear_not_down_warning", xplmType_Int),
    EWD_INPUT("a380x/xp12/center_right/gear_not_locked_down_warning", xplmType_Int),
    EWD_INPUT("a380x/xp12/center_right/gear_not_locked_up_warning", xplmType_Int),
    /* N1 percent references, not normalized THR percent. max includes the
     * actual pack correction. Reverse reference is a POSITIVE N1 limit. */
    EWD_INPUT("a380x/xp12/engine/thrust_limit_idle_n1", xplmType_Float),
    EWD_INPUT("a380x/xp12/engine/thrust_limit_max_n1", xplmType_Float),
    EWD_INPUT("a380x/xp12/engine/thrust_limit_reverse_n1", xplmType_Float),
    EWD_INPUT("a380x/xp12/engine/commanded_n1", xplmType_FloatArray),
    EWD_INPUT("a380x/xp12/engine/throttle_position_n1", xplmType_FloatArray),
    EWD_INPUT("a380x/xp12/engine/thrust_limit_n1", xplmType_Float),
    /* Same enum as FBW: 0 none, 1 CLB, 2 MCT, 3 FLX, 4 TOGA, 5 MREV. */
    EWD_INPUT("a380x/xp12/engine/thrust_limit_type", xplmType_Int),
    EWD_INPUT("a380x/xp12/fms/to_flex_temp_c", xplmType_Float),
    EWD_INPUT("a380x/xp12/engine/autothrust_status", xplmType_Int),
    EWD_INPUT("sim/weather/aircraft/temperature_ambient_deg_c", xplmType_Float),
    EWD_INPUT("sim/weather/aircraft/barometer_current_pas", xplmType_Float),
    EWD_INPUT("sim/flightmodel2/position/pressure_altitude", xplmType_Double),
    EWD_INPUT("sim/flightmodel/misc/machno", xplmType_Float),
    EWD_INPUT("sim/cockpit2/gauges/indicators/airspeed_kts_pilot", xplmType_Float),
    EWD_INPUT("sim/operation/failures/rel_ss_asi", xplmType_Int),
    /* X-Plane: -1 CRANK, 0 NORM, 1 IGN/START, 2 AIR start. */
    EWD_INPUT("sim/cockpit2/engine/actuators/eng_mode_selector", xplmType_Int),
    /* Optional true fire pushbutton state. A fire warning or extinguisher
     * discharge is not an equivalent source and must not blank the gauge. */
    EWD_INPUT("a380x/xp12/engine/fire_pushbutton", xplmType_IntArray),
    /* TAT for the FADEC FLEX condition (flex temperature > TAT). */
    EWD_INPUT("sim/cockpit2/temperature/outside_air_LE_temp_degc", xplmType_Float),
};
#undef EWD_INPUT

static float ewd_rebind_timer;

static void ewd_bind_inputs(void)
{
    int i;
    for (i=0;i<ER_REF_COUNT;++i) {
        XPLMDataRef ref;
        if (ewd_inputs[i].ref) continue;
        ref=XPLMFindDataRef(ewd_inputs[i].name);
        if (ref && (XPLMGetDataRefTypes(ref)&ewd_inputs[i].type))
            ewd_inputs[i].ref=ref;
    }
}

static void ewd_init(void)
{
    int i;
    memset(&ewd,0,sizeof(ewd));
    for (i=0;i<ER_REF_COUNT;++i) ewd_inputs[i].ref=NULL;
    ewd_rebind_timer=1.0f;
    ewd_bind_inputs();
}

static int ewd_float(int input, int index, float low, float high, float *out)
{
    const EwdInputRef *r=&ewd_inputs[input];
    float value;
    if (!r->ref) return 0;
    if (r->type==xplmType_FloatArray) {
        if (XPLMGetDatavf(r->ref,&value,index,1)!=1) return 0;
    } else if (r->type==xplmType_Double) value=(float)XPLMGetDatad(r->ref);
    else value=XPLMGetDataf(r->ref);
    if (!isfinite(value) || value<low || value>high) return 0;
    *out=value;
    return 1;
}

static int ewd_integer(int input, int index, int low, int high, int *out)
{
    const EwdInputRef *r=&ewd_inputs[input];
    int value;
    if (!r->ref) return 0;
    if (r->type==xplmType_IntArray) {
        if (XPLMGetDatavi(r->ref,&value,index,1)!=1) return 0;
    } else value=XPLMGetDatai(r->ref);
    if (value<low || value>high) return 0;
    *out=value;
    return 1;
}

static int ewd_bool(int input, int index)
{
    int value=0;
    return ewd_integer(input,index,0,1,&value) && value;
}

static int ewd_indicator_failed(int input)
{
    int value=0;
    return ewd_integer(input,0,0,6,&value) && value==6;
}

static float ewd_thrust_from_n1(float n1, float idle, float maximum, int running)
{
    float offset=running ? 0.042f : 0.0f;
    return fminf(1.0f,fmaxf(0.0f,(n1-idle)/(maximum-idle))*
                  (1.0f-offset)+offset)*100.0f;
}

static void ewd_add_message(EwdLine *lines, int *count, const char *text, int color)
{
    if (*count>=16) return;
    snprintf(lines[*count].text,sizeof(lines[*count].text),"%s",text);
    lines[*count].color=color;
    ++*count;
}

static void ewd_bleed_append(const char *text)
{
    size_t used=strlen(ewd.bleed);
    snprintf(ewd.bleed+used,sizeof(ewd.bleed)-used,"%s%s",used ? "/" : "",text);
}

/* FBW A380FadecComputer.cpp thrust_limit_type (fbw_a380/src/model, pinned revision, the block
 * after "rtb_y_mv = (rtb_NOT1_f && (rtb_y_pg > ...TAT_degC))"): the EWD shows engine 1's FADEC.
 * Used only while no a380x/xp12/engine/thrust_limit_type producer is registered. TLA scale of the
 * A380 lever: idle 0, CL 25, FLX/MCT 35, TOGA 45 degrees, reverse negative. Detent positions are
 * the aircraft-local lever calibration (a380x_fcu_thrust.inc); inside the calibrated tolerance a
 * lever IS in its detent, so the FBW exact comparisons (TLA == 35 ...) hold. */
static int ewd_flex_latch;
/* Optional native rating source (a380x_thrust.inc): engine 1's FADEC rating and N1 from the FBW
 * FADEC that also drives the engines (FLEX by the FBW limitN1 FLEX method, CLB after a FLEX
 * take-off), so the EWD always shows what the engines get. NULL: the selection below is used. */
typedef struct { int valid, mode; double rating_n1, flex_n1; } EwdRatingSource;
static int (*ewd_rating_source)(EwdRatingSource *out);
static double ewd_lever_tla(double ratio,double idle,double cl,double mct,double toga,double tolerance,int reverse)
{
    if (reverse) return -20.0;
    if (fabs(ratio-idle)<=tolerance) return 0.0;
    if (fabs(ratio-cl)<=tolerance) return 25.0;
    if (fabs(ratio-mct)<=tolerance) return 35.0;
    if (fabs(ratio-toga)<=tolerance) return 45.0;
    if (ratio<cl) return 25.0*(ratio-idle)/(cl-idle);
    if (ratio<mct) return 25.0+10.0*(ratio-cl)/(mct-cl);
    return 35.0+10.0*(ratio-mct)/(toga-mct);
}
/* Returns 0 NONE, 1 CLB, 2 MCT, 3 FLX, 4 TOGA, 5 MREV; updates the FLEX latch. */
static int ewd_thrust_limit_type(double tla,int on_ground,int engine_operative,int flex_candidate,
                                 int ats_toga,int *latch)
{
    int flex;
    double angle=on_ground?tla:fmax(0.0,tla);
    *latch=(flex_candidate && on_ground && tla==35.0) || *latch;
    *latch=(!*latch || (tla!=25.0 && tla!=45.0)) && *latch;
    flex=(flex_candidate && on_ground) || (!on_ground && *latch);
    if (!on_ground || !engine_operative) {
        if (ats_toga || angle>35.0) return 4;
        if (angle>25.0) return flex?3:2;
        if (angle>=0.0) return 1;
        return 5;
    }
    if (angle>=0.0) return (!flex || angle>35.0)?4:3;
    return 5;
}

static void ewd_update(float dt, int powered)
{
    static const char *modes[]={"","CLB","MCT","FLX","TOGA","MREV"};
    float idle=0.0f, maximum=0.0f, reverse_limit=0.0f, value, sat;
    int reference_valid, idle_valid, maximum_valid, reverse_limit_valid, i, running_count=0;
    int running_known=0, running[4]={0}, reverse_any=0, nai=0;
    int wai=ewd_bool(ER_WING_ICE,0), ground=0, mode=0, mode_valid;
    int idle_engines=0, athr=0, phase=published.flight_phase;
    int native_limits_valid=0;
    int selector=0, selector_valid, aggregate_active=0, aggregate_valid;
    int state_known[4], starter_known[4], starter[4]={0}, all_states_known=1;
    int cas_valid;
    float cas=0.0f;
    A380xEwdFadecLimits native_limits;
    float native_rating_n1=0.0f;
    int native_rating_n1_valid=0;
    char text[64];

    if (isfinite(dt) && dt>0.0f) ewd_rebind_timer+=dt;
    if (ewd_rebind_timer>=1.0f) {
        ewd_bind_inputs();
        ewd_rebind_timer=0.0f;
    }
    memset(&ewd,0,sizeof(ewd));
    ewd.powered=powered!=0;
    if (!ewd.powered) return;
    selector_valid=ewd_integer(ER_ENG_SELECTOR,0,-1,2,&selector);
    if (selector_valid && selector>=1) aggregate_active=1;
    for (i=0;i<4;++i) {
        int combustion=0;
        int combustion_valid=ewd_integer(ER_BURNING,i,0,1,&combustion);
        starter_known[i]=ewd_integer(ER_STARTER,i,0,1,&starter[i]);
        state_known[i]=combustion_valid && starter_known[i];
        all_states_known=all_states_known && state_known[i];
        running[i]=state_known[i] && combustion && !starter[i];
        running_known+=state_known[i];
        running_count+=running[i];
        if ((combustion_valid && combustion) || (starter_known[i] && starter[i]))
            aggregate_active=1;
    }
    aggregate_valid=aggregate_active || (all_states_known && selector_valid);
    cas_valid=ewd_float(ER_CAS,0,0.0f,600.0f,&cas) && !ewd_indicator_failed(ER_CAS_FAIL);
    idle_valid=ewd_float(ER_IDLE_N1,0,0.0f,120.0f,&idle);
    maximum_valid=ewd_float(ER_MAX_N1,0,0.0f,120.0f,&maximum);
    reference_valid=idle_valid && maximum_valid && maximum>idle;
    /* The exact FBW numerical engine model supplies its own environmental
     * limits when no external FADEC producer exists. Require every raw
     * environmental/configuration input rather than guessing ISA/pack state. */
    if (!ewd_inputs[ER_IDLE_N1].ref && !ewd_inputs[ER_MAX_N1].ref) {
        float temperature, pressure, altitude, mach;
        int pack_l, pack_r, wing, engine_ice=0, anti_ice_known=1;
        for (i=0;i<4;++i) {
            int ai=0;
            anti_ice_known=ewd_integer(ER_ANTI_ICE,i,0,1,&ai) && anti_ice_known;
            engine_ice=engine_ice || ai;
        }
        if (anti_ice_known &&
            ewd_float(ER_SAT,0,-100.0f,100.0f,&temperature) &&
            ewd_float(ER_PRESSURE,0,1.0f,120000.0f,&pressure) &&
            ewd_float(ER_PRESSURE_ALT,0,-2000.0f,60000.0f,&altitude) &&
            ewd_float(ER_MACH,0,0.0f,1.0f,&mach) &&
            ewd_integer(ER_PACK_L,0,0,1,&pack_l) &&
            ewd_integer(ER_PACK_R,0,0,1,&pack_r) &&
            ewd_integer(ER_WING_ICE,0,0,1,&wing)) {
            native_limits_valid=a380x_ewd_fadec_limits(temperature,pressure*0.01,
                altitude,mach,pack_l+pack_r,engine_ice,wing,&native_limits);
            if (native_limits_valid) {
                idle=(float)native_limits.idle_n1;
                maximum=(float)native_limits.max_n1;
                idle_valid=isfinite(idle) && idle>=0.0f && idle<=120.0f;
                maximum_valid=isfinite(maximum) && maximum>=0.0f && maximum<=120.0f;
                reference_valid=idle_valid && maximum_valid && maximum>idle;
            }
        }
    }
    reverse_limit_valid=ewd_float(ER_REV_N1,0,0.0f,120.0f,&reverse_limit) &&
        idle_valid && reverse_limit>idle;
    if (!ewd_inputs[ER_REV_N1].ref && native_limits_valid) {
        reverse_limit=(float)native_limits.reverse_n1;
        reverse_limit_valid=isfinite(reverse_limit) && reverse_limit>idle;
    }
    mode_valid=ewd_integer(ER_RATING_MODE,0,1,5,&mode);
    native_rating_n1_valid=0;
    if (!ewd_inputs[ER_RATING_MODE].ref && native_limits_valid && ewd_rating_source) {
        EwdRatingSource source;
        if (ewd_rating_source(&source) && source.valid && source.mode >= 1 && source.mode <= 5
            && isfinite(source.rating_n1)) {
            mode = source.mode;
            mode_valid = 1;
            native_rating_n1 = (float)source.rating_n1;
            native_rating_n1_valid = 1;
        }
    } else if (!ewd_inputs[ER_RATING_MODE].ref && native_limits_valid && guidance_inputs.throttle_levers_valid) {
        const A380XGuidanceInputs *g=&guidance_inputs;
        int on_ground=0, combustion=0, flex_candidate=0;
        float flex=0.0f, tat=0.0f;
        if (ewd_integer(ER_ON_GROUND,0,0,1,&on_ground) && ewd_integer(ER_BURNING,0,0,1,&combustion)
            && g->cl_detent_ratio>g->idle_detent_ratio && g->mct_detent_ratio>g->cl_detent_ratio
            && g->toga_detent_ratio>g->mct_detent_ratio) {
            double tla=ewd_lever_tla(g->throttle_lever_ratio[0],g->idle_detent_ratio,g->cl_detent_ratio,
                g->mct_detent_ratio,g->toga_detent_ratio,g->detent_tolerance_ratio,g->reverse_selected[0]);
            /* FLEX: a valid FMS FLEX temperature above TAT (0 = none, 0.1 = FLEX 0 degC). */
            if (ewd_float(ER_FLEX,0,-100.0f,100.0f,&flex) && flex!=0.0f
                && ewd_float(ER_TAT,0,-100.0f,100.0f,&tat)) {
                if (fabsf(flex-0.1f)<0.001f) flex=0.0f;
                flex_candidate=flex>tat;
            }
            mode=ewd_thrust_limit_type(tla,on_ground,combustion,flex_candidate,0,&ewd_flex_latch);
            mode_valid=mode>=1 && mode<=5;
            switch (mode) {
            case 1: native_rating_n1=(float)native_limits.climb_n1; break;
            case 2: native_rating_n1=(float)native_limits.mct_n1; break;
            case 4: native_rating_n1=(float)native_limits.toga_n1; break;
            case 5: native_rating_n1=(float)native_limits.reverse_n1; break;
            case 3: {
                /* Port assumption: the FLEX N1 is the TOGA limit at the assumed temperature
                 * (the vendored FADEC equations have no FLEX table), never above TOGA. */
                A380xEwdFadecLimits assumed;
                float pressure=0.0f, altitude=0.0f, mach=0.0f;
                if (ewd_float(ER_PRESSURE,0,1.0f,120000.0f,&pressure)
                    && ewd_float(ER_PRESSURE_ALT,0,-2000.0f,60000.0f,&altitude)
                    && ewd_float(ER_MACH,0,0.0f,1.0f,&mach)
                    && a380x_ewd_fadec_limits(flex,pressure*0.01,altitude,mach,
                           ewd_bool(ER_PACK_L,0)+ewd_bool(ER_PACK_R,0),0,ewd_bool(ER_WING_ICE,0),&assumed))
                    native_rating_n1=(float)fmin(assumed.toga_n1,native_limits.toga_n1);
                else native_rating_n1=(float)native_limits.toga_n1;
                break;
            }
            default: mode_valid=0; break;
            }
            native_rating_n1_valid=mode_valid && isfinite(native_rating_n1);
        }
    }
    ewd.athr_valid=ewd_integer(ER_ATHR_STATUS,0,0,2,&athr);
    ewd.athr_engaged=ewd.athr_valid && athr!=0;

    for (i=0;i<4;++i) {
        EwdEngine *engine=&ewd.engine[i];
        int enabled=0, prop_mode=0, burning_valid=state_known[i], fire_pb=0;
        float deploy=0.0f;
        engine->active_valid=ewd_integer(ER_FADEC,i,0,1,&enabled) && aggregate_valid;
        /* FBW activates all four channels when any engine runs/starts or
         * IGN/START is selected. The native FADEC switch adds the observed
         * channel power gate; an engine fire alone never blanks parameters. */
        if (ewd_inputs[ER_FIRE_PB].ref &&
            !ewd_integer(ER_FIRE_PB,i,0,1,&fire_pb)) engine->active_valid=0;
        engine->active=engine->active_valid && enabled && aggregate_active && !fire_pb;
        engine->idle_n1=idle;
        engine->idle_n1_valid=idle_valid;
        /* Raw N1 targets remain useful in degraded-N1 mode even when THR
         * normalization is unavailable. Preserve their own validity. */
        engine->commanded_n1_valid=ewd_float(ER_COMMANDED_N1,i,0.0f,150.0f,&engine->commanded_n1);
        engine->throttle_n1_valid=ewd_float(ER_THROTTLE_N1,i,0.0f,150.0f,&engine->throttle_n1);
        engine->n1_valid=ewd_float(ER_N1,i,0.0f,150.0f,&engine->n1);
        engine->egt_valid=ewd_float(ER_EGT,i,-273.15f,2000.0f,&engine->egt);
        if (i<2) {
            if (ewd_indicator_failed(ER_N1_FAIL_1+i)) engine->n1_valid=0;
            if (ewd_indicator_failed(ER_EGT_FAIL_1+i)) engine->egt_valid=0;
        }
        engine->n1_limit_valid=ewd_float(ER_N1_LIMIT,0,1.0f,150.0f,&engine->n1_limit);
        engine->egt_limit_valid=ewd_float(ER_EGT_LIMIT,0,1.0f,2000.0f,&engine->egt_limit);
        engine->starting_valid=starter_known[i];
        engine->starting=starter[i];
        engine->running=running[i];
        engine->running_valid=state_known[i];
        engine->fire_valid=ewd_integer(ER_FIRE,i,0,1,&engine->fire);
        engine->anti_ice_valid=ewd_integer(ER_ANTI_ICE,i,0,1,&engine->anti_ice);
        nai=nai || (engine->anti_ice_valid && engine->anti_ice);
        /* The A380 has reverse only on engines 2 and 3. Never draw REV
         * on an outer engine even if the generic simulator reports it. */
        if (i==1 || i==2) {
            int mode_known=ewd_integer(ER_REV_MODE,i,0,3,&prop_mode);
            int deploy_known=ewd_float(ER_REV_DEPLOY,i,0.0f,1.0f,&deploy);
            engine->reverse_valid=mode_known && deploy_known;
            engine->reverse=(mode_known && prop_mode==3) || (deploy_known && deploy>0.0f);
            engine->reverse_deployed=deploy_known && deploy>=0.99f;
        } else engine->reverse_valid=1;
        reverse_any=reverse_any || engine->reverse;
        if (idle_valid && engine->n1_valid && burning_valid) {
            if (engine->reverse && reverse_limit_valid) {
                engine->thrust=ewd_thrust_from_n1(engine->n1,idle,reverse_limit,0);
                engine->thrust_valid=1;
            } else if (!engine->reverse && engine->reverse_valid && reference_valid) {
                engine->thrust=ewd_thrust_from_n1(engine->n1,idle,maximum,running[i]);
                engine->thrust_valid=1;
            }
        }
        engine->n1_degraded=engine->active && !engine->thrust_valid;
        engine->avail_valid=idle_valid && engine->starting_valid && engine->n1_valid;
        engine->avail=engine->avail_valid && engine->starting && engine->n1>floorf(idle);
        if (reference_valid && burning_valid && cas_valid) {
            engine->available_thrust=cas<35.0f ?
                ewd_thrust_from_n1(76.5f,idle,maximum,running[i]) : 100.0f;
            engine->available_valid=1;
        }
        /* Donut and demand are N1 targets normalized to indicated THR,
         * never an assumed linear conversion from lever travel. A/THR
         * engagement gates only the transient cue, not raw demand validity. */
        if (reference_valid && burning_valid && !engine->reverse) {
            if (engine->commanded_n1_valid) {
                engine->commanded_thrust=ewd_thrust_from_n1(engine->commanded_n1,idle,maximum,running[i]);
                engine->commanded_valid=ewd.athr_engaged;
            }
            if (engine->throttle_n1_valid) {
                engine->throttle=ewd_thrust_from_n1(engine->throttle_n1,idle,maximum,running[i]);
                engine->throttle_valid=1;
            }
        }
        /* Native reverse references and N1 demand are positive magnitudes.
         * The pinned display negates its reverse-limit variable, whereas
         * its producer supplies positive TOGA*.813. Resolve that sign at
         * this native boundary; 0%=idle and 100%=the actual reverse limit. */
        if (engine->reverse && engine->reverse_valid && reverse_limit_valid &&
            engine->throttle_n1_valid) {
            engine->throttle=ewd_thrust_from_n1(engine->throttle_n1,idle,reverse_limit,0);
            engine->throttle_valid=1;
        }
        if (idle_valid && engine->n1_valid && engine->n1<=idle+2.0f) ++idle_engines;
        if (engine->fire_valid && engine->fire) {
            snprintf(text,sizeof(text),"ENG %d FIRE",i+1);
            ewd_add_message(ewd.warning,&ewd.warning_count,text,EWD_COLOR_RED);
        }
    }
    if (ewd_bool(ER_GEAR_NOT_DOWN,0))
        ewd_add_message(ewd.warning,&ewd.warning_count,"L/G GEAR NOT DOWN",EWD_COLOR_RED);
    if (ewd_bool(ER_GEAR_NOT_LOCKED_DOWN,0))
        ewd_add_message(ewd.warning,&ewd.warning_count,"L/G GEAR NOT LOCKED DOWN",EWD_COLOR_RED);
    /* Cautions follow warnings. Thresholds come from the loaded aircraft,
     * not a copied nominal A380 EGT or N1 redline. */
    for (i=0;i<4;++i) {
        const EwdEngine *engine=&ewd.engine[i];
        if (engine->egt_valid && engine->egt_limit_valid && engine->egt>engine->egt_limit) {
            snprintf(text,sizeof(text),"ENG %d EGT OVER LIMIT",i+1);
            ewd_add_message(ewd.warning,&ewd.warning_count,text,EWD_COLOR_AMBER);
        }
        if (ewd_bool(ER_N1_HIGH,i)) {
            snprintf(text,sizeof(text),"ENG %d N1/N2 OVER LIMIT",i+1);
            ewd_add_message(ewd.warning,&ewd.warning_count,text,EWD_COLOR_AMBER);
        }
    }
    if (ewd_bool(ER_GEAR_NOT_LOCKED_UP,0))
        ewd_add_message(ewd.warning,&ewd.warning_count,"L/G GEAR NOT LOCKED UP",EWD_COLOR_AMBER);

    /* Rating: the external producer when registered, else the native FADEC selection above. */
    if (native_rating_n1_valid) { value=native_rating_n1; }
    if (reference_valid && mode_valid && !reverse_any &&
        (native_rating_n1_valid || ewd_float(ER_RATING_N1,0,0.0f,150.0f,&value))) {
        ewd.thrust_limit=ewd_thrust_from_n1(value,idle,maximum,1);
        ewd.thrust_limit_valid=1;
        snprintf(ewd.thrust_mode,sizeof(ewd.thrust_mode),"%s",modes[mode]);
    }
    /* FLEX comes from the native FMS PERF T.O entry with the FBW encoding of
     * L:A32NX_AIRLINER_TO_FLEX_TEMP: 0 = none, 0.1 = FLEX 0 degC. */
    if (ewd.thrust_limit_valid && mode==3 &&
        ewd_float(ER_FLEX,0,-100.0f,100.0f,&value) && value!=0.0f) {
        if (fabsf(value-0.1f)<0.001f) value=0.0f;
        if (ewd_float(ER_SAT,0,-100.0f,100.0f,&sat) && value>=sat-10.0f) {
            ewd.flex_valid=1;
            ewd.flex_temp=value;
        }
    }
    ewd.idle=idle_valid && idle_engines>=3 && phase>=6 && phase<=9 && ewd.athr_engaged;
    if (!reverse_any && ewd_integer(ER_ON_GROUND,0,0,1,&ground) &&
        ((ground && running_count>0) || (!ground && mode_valid && mode>=2))) {
        if (ewd_bool(ER_PACK_L,0) || ewd_bool(ER_PACK_R,0)) ewd_bleed_append("PACKS");
        if (nai) ewd_bleed_append("NAI");
        if (wai) ewd_bleed_append("WAI");
    }
    /* Source wording and phase inhibition. Full T.O/LDG memo/checklist
     * replacement is unavailable until the native FWS is ported. */
    if (ewd_bool(ER_APU_RUNNING,0) && ewd_float(ER_APU_N1,0,0.0f,110.0f,&value) && value>=95.0f) {
        int bleed=0;
        if (ewd_integer(ER_APU_BLEED,0,0,1,&bleed))
            ewd_add_message(ewd.memo,&ewd.memo_count,bleed ? "APU BLEED" : "APU AVAIL",EWD_COLOR_GREEN);
    }
    if (ewd_bool(ER_SEAT_BELTS,0))
        ewd_add_message(ewd.memo,&ewd.memo_count,"SEAT BELTS",EWD_COLOR_GREEN);
    if ((phase==1 || phase==2 || phase==11 || phase==12) &&
        ewd_float(ER_PARK_BRAKE,0,0.0f,1.0f,&value) && value>0.0f)
        ewd_add_message(ewd.memo,&ewd.memo_count,"PARK BRK ON",EWD_COLOR_GREEN);
    if (phase>=1 && phase<=12 && !(phase>=3 && phase<=8) &&
        running_known==4 && ewd_bool(ER_EXT_PWR,0))
        ewd_add_message(ewd.memo,&ewd.memo_count,"ELEC EXT PWR",
                 running_count>1 ? EWD_COLOR_AMBER : EWD_COLOR_GREEN);
    if (nai) ewd_add_message(ewd.memo,&ewd.memo_count,"ENG A-ICE",EWD_COLOR_GREEN);
    if (wai) ewd_add_message(ewd.memo,&ewd.memo_count,"WING A-ICE",EWD_COLOR_GREEN);
}
