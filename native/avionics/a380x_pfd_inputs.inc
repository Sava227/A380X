/* SPDX-License-Identifier: GPL-3.0-or-later
 * PFD aircraft-state boundary. Raw X-Plane surface positions are measurements;
 * they are not the A380 SFCC/PRIM network. In particular the current aircraft
 * has six native flap detents, collapsed into the four extended lever labels.
 * A validated aircraft envelope provider may supply the missing computed
 * speeds. Its validity flag is required; absent data never becomes a band.
 * Constants and GREEN_DOT table: pinned FBW PerformanceConstants.ts and
 * OperatingSpeeds.tsx (systems/shared/src), GPL-3.0.
 */
#include <stddef.h>
#include "a380x_flaps.h"
#include "a380x_fms_speeds.h"
/* FLAPS lever / FBW SFCC feed, filled by a380x_flaps.inc every frame (ready = 0: the legacy X-Plane
 * handle interpretation below applies). Also read by the FWS adapter and the xp-kit SFCC words. */
static A380XFlapsFeed flaps_feed;
#include "a380x_speedbrake.h"
/* SPEED BRAKE lever / FBW ground spoiler feed, filled by a380x_speedbrake.inc every frame (ready = 0: X-Plane's
 * speedbrake handle is read as the lever below). X-Plane's handle then carries the FBW surface order, not the
 * lever. Also read by the FWS adapter and the xp-kit PFD variables. */
static A380XSpeedbrakeFeed speedbrake_feed;
enum {
    PI_FLAP_HANDLE, PI_FLAP_ACTUAL, PI_SLAT_ACTUAL, PI_SPOILER_ACTUAL,
    PI_SPOILER_HANDLE, PI_GEAR, PI_FLAP_FAILURE, PI_SLAT_FAILURE, PI_STALL,
    PI_ENVELOPE_VALID, PI_VLS, PI_ALPHA_PROT, PI_ALPHA_LIMIT, PI_STALL_SPEED,
    PI_F_SPEED, PI_S_SPEED, PI_GD, PI_V1, PI_VR, PI_VMAX,
    PI_FLAP_DETENTS, PI_FLAP_ANGLES, PI_COUNT
};
static const char *pfd_input_names[PI_COUNT] = {
    "sim/cockpit2/controls/flap_handle_request_ratio",
    "sim/flightmodel2/controls/flap1_deploy_ratio",
    "sim/flightmodel2/controls/slat1_deploy_ratio",
    "sim/flightmodel2/controls/speedbrake_ratio",
    "sim/cockpit2/controls/speedbrake_ratio",
    "sim/flightmodel2/gear/deploy_ratio",
    "sim/operation/failures/rel_flap_act",
    "a380x/xp12/systems/pfd/slat_fault",
    "sim/cockpit2/annunciators/stall_warning",
    "a380x/xp12/systems/pfd/envelope_valid",
    "a380x/xp12/systems/pfd/vls_knots",
    "a380x/xp12/systems/pfd/alpha_prot_knots",
    "a380x/xp12/systems/pfd/alpha_limit_knots",
    "a380x/xp12/systems/pfd/stall_warning_knots",
    "a380x/xp12/systems/pfd/f_speed_knots",
    "a380x/xp12/systems/pfd/s_speed_knots",
    "a380x/xp12/systems/pfd/green_dot_knots",
    "a380x/xp12/systems/pfd/v1_knots",
    "a380x/xp12/systems/pfd/vr_knots",
    "a380x/xp12/systems/pfd/vmax_knots",
    "sim/aircraft/controls/acf_flap_detents",
    "sim/aircraft/controls/acf_flap_dn"
};
static XPLMDataRef pfd_input_refs[PI_COUNT];
static A380XPfdGuidance pfd_aircraft;
static float pfd_input_retry_time;

typedef struct { const char *name; size_t offset; int integer; XPLMDataRef handle; } PfdOutput;
#define PFD_INT(field) {"a380x/xp12/pfd/" #field, offsetof(A380XPfdGuidance,field),1,NULL}
#define PFD_NUM(field) {"a380x/xp12/pfd/" #field, offsetof(A380XPfdGuidance,field),0,NULL}
static PfdOutput pfd_outputs[] = {
    PFD_INT(on_ground), PFD_INT(flap_handle_index), PFD_INT(flap_config),
    PFD_INT(flap_valid), PFD_INT(slat_valid), PFD_INT(speedbrake_valid),
    PFD_INT(spoilers_armed), PFD_INT(gear_down), PFD_INT(flap_fault), PFD_INT(slat_fault),
    PFD_INT(stall_warning), PFD_INT(envelope_valid), PFD_INT(clean_performance_valid),
    PFD_NUM(flap_ratio), PFD_NUM(slat_ratio), PFD_NUM(spoiler_ratio),
    PFD_NUM(flap_target_ratio), PFD_NUM(slat_target_ratio),
    PFD_NUM(vmax_knots), PFD_NUM(vfe_next_knots), PFD_NUM(vls_knots),
    PFD_NUM(alpha_prot_knots), PFD_NUM(alpha_limit_knots), PFD_NUM(stall_warning_knots),
    PFD_NUM(f_speed_knots), PFD_NUM(s_speed_knots), PFD_NUM(green_dot_knots),
    PFD_NUM(v1_knots), PFD_NUM(vr_knots), PFD_NUM(v2_knots)
};
#undef PFD_INT
#undef PFD_NUM
static float pfd_get_output_float(void *refcon)
{
    const PfdOutput *p=(const PfdOutput *)refcon;
    const char *data=(const char *)&pfd_guidance+p->offset;
    return p->integer?(float)*(const int *)data:(float)*(const double *)data;
}
static int pfd_get_output_int(void *refcon) { return (int)pfd_get_output_float(refcon); }
static void pfd_inputs_find(void)
{
    int i;
    memset(pfd_input_refs,0,sizeof(pfd_input_refs));
    memset(&pfd_aircraft,0,sizeof(pfd_aircraft));
    for(i=0;i<PI_COUNT;++i) pfd_input_refs[i]=XPLMFindDataRef(pfd_input_names[i]);
    pfd_input_retry_time=XPLMGetElapsedTime()+1;
}
static int pfd_input_number(int index,double minimum,double maximum,double *out)
{
    double value;
    if(!pfd_input_refs[index]) return 0;
    value=XPLMGetDataf(pfd_input_refs[index]);
    if(!isfinite(value)||value<minimum||value>maximum) return 0;
    *out=value;return 1;
}
static double pfd_green_dot(double weight,double altitude)
{
    static const double rows[4][8]={
        {160,175,190,205,216,228,239,252},
        {161,178,196,219,237,256,267,277},
        {186,208,224,241,256,272,284,292},
        {203,220,236,252,252,260,260,260}
    };
    double w,a,fw,fa,low,high;int wi,ai;
    if(!isfinite(weight)||weight<250000||weight>600000||!isfinite(altitude)||altitude>43000) return 0;
    w=(weight-250000)/50000;wi=(int)fmin(6,floor(w));fw=w-wi;
    a=fmax(0,fmin(3,(altitude-10000)/10000));ai=(int)fmin(2,floor(a));fa=a-ai;
    low=rows[ai][wi]+fw*(rows[ai][wi+1]-rows[ai][wi]);
    high=rows[ai+1][wi]+fw*(rows[ai+1][wi+1]-rows[ai+1][wi]);
    return low+fa*(high-low);
}
static double pfd_clean_vls(double weight,double altitude)
{
    /* Exact pinned A380PrimComputerFe_data.cpp uDLookupTable1 table, revision
     * 2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2. Do not substitute the similar
     * OperatingSpeeds.tsx table: its 15000/38000ft rows differ. The PRIM clean
     * configuration uses Vs1g=table/1.23 and VLS=max(Vmcl,1.23*Vs1g).
     * With speedbrakes stowed the additional VLS term is zero. This returns
     * that steady-state performance target, not the full PRIM rate-limited
     * state or a native aerodynamic stall/protection threshold. */
    static const double altitudes[12]={0,10000,15000,20000,25000,30000,32000,34000,36000,38000,40000,43000};
    static const double rows[12][8]={
        {149,163,176,188,199,210,220,231},
        {149,163,176,189,202,216,231,243},
        {149,163,178,193,209,222,234,246},
        {149,165,183,199,212,226,239,252},
        {151,171,187,202,217,231,246,260},
        {158,174,191,207,224,239,256,270},
        {158,176,193,210,227,244,259,270},
        {159,178,196,213,231,247,259,270},
        {161,180,198,217,235,247,259,275},
        {162,182,202,222,235,247,262,280},
        {164,185,206,222,235,250,265,270},
        {168,191,208,223,235,250,250,250}
    };
    double w,fw,fa,low,high;int wi,ai=0;
    if(!isfinite(weight)||weight<250000||weight>600000||
       !isfinite(altitude)||altitude<0||altitude>43000)return 0;
    w=(weight-250000)/50000;wi=(int)fmin(6,floor(w));fw=w-wi;
    while(ai<10&&altitude>altitudes[ai+1])++ai;
    fa=(altitude-altitudes[ai])/(altitudes[ai+1]-altitudes[ai]);
    low=rows[ai][wi]+fw*(rows[ai][wi+1]-rows[ai][wi]);
    high=rows[ai+1][wi]+fw*(rows[ai+1][wi+1]-rows[ai+1][wi]);
    return fmax(123,low+fa*(high-low));
}
static double pfd_mach_limit(double mach)
{
    double ias=guidance_inputs.ias_knots,m=guidance_inputs.mach;
    double impact,pressure;
    if(!guidance_inputs.airdata_valid||!isfinite(ias)||ias<=0||!isfinite(m)||m<=.1) return 0;
    impact=pow(1+.2*pow(ias/661.4788,2),3.5)-1;
    pressure=impact/(pow(1+.2*m*m,3.5)-1);
    return 661.4788*sqrt(5*(pow(1+pressure*(pow(1+.2*mach*mach,3.5)-1),2.0/7.0)-1));
}
static int pfd_flap_geometry(double handle,double actual,double *target,int *actual_index)
{
    float angles[10];int count,detents,i,lower,native_index=0;double position,angle;
    if(!pfd_input_refs[PI_FLAP_DETENTS]||!pfd_input_refs[PI_FLAP_ANGLES])return 0;
    detents=XPLMGetDatai(pfd_input_refs[PI_FLAP_DETENTS]);
    if(detents<1||detents>=ARRAY_LEN(angles))return 0;
    count=XPLMGetDatavf(pfd_input_refs[PI_FLAP_ANGLES],angles,0,ARRAY_LEN(angles));
    if(count<=detents||!isfinite(angles[0])||fabs(angles[0])>.01)return 0;
    for(i=1;i<=detents;++i)
        if(!isfinite(angles[i])||angles[i]<angles[i-1]||angles[i]>90)return 0;
    if(angles[detents]<=.01)return 0;
    /* Handle is travel through the detents; flap1_deploy_ratio is surface
     * angle divided by the maximum flap angle. Live X-Plane confirmed that
     * handle .5 with [0,1,5,10,20,25,40] degrees gives actual ratio10/40=.25. */
    position=handle*detents;lower=(int)fmin(detents-1,floor(position));
    *target=(angles[lower]+(position-lower)*(angles[lower+1]-angles[lower]))/angles[detents];
    angle=actual*angles[detents];
    if(angle>.01){
        /* Use the next enclosing physical detent during transit, retaining
         * its restrictive limit until the surface reaches the lower angle. */
        native_index=detents;
        for(i=1;i<=detents;++i)if(angle<=angles[i]+.01){native_index=i;break;}
    }
    *actual_index=(int)lround((double)native_index/detents*4);
    if(native_index>0&&*actual_index==0)*actual_index=1;
    return 1;
}
static void read_pfd_aircraft(void)
{
    static const double vfe[6]={340,263,222,220,196,182};
    static const double next[6]={263,222,220,196,182,0};
    A380XPfdGuidance *p=&pfd_aircraft;
    float gear[5]={0};double handle=0,spoiler_handle=0,gear_mach,max_gear=0;
    int i,actual_index=0,limit_config,gear_count=0;
    float now=XPLMGetElapsedTime();
    if(now>=pfd_input_retry_time){
        for(i=0;i<PI_COUNT;++i)
            if(!pfd_input_refs[i])pfd_input_refs[i]=XPLMFindDataRef(pfd_input_names[i]);
        pfd_input_retry_time=now+1;
    }
    memset(p,0,sizeof(*p));
    p->on_ground=guidance_inputs.on_ground;
    p->flap_valid=pfd_input_number(PI_FLAP_HANDLE,0,1,&handle)&&
        pfd_input_number(PI_FLAP_ACTUAL,0,1,&p->flap_ratio);
    p->slat_valid=pfd_input_number(PI_SLAT_ACTUAL,0,1,&p->slat_ratio);
    p->speedbrake_valid=pfd_input_number(PI_SPOILER_ACTUAL,0,1.5,&p->spoiler_ratio)&&
        pfd_input_number(PI_SPOILER_HANDLE,-1,1.5,&spoiler_handle);
    p->spoiler_ratio=fmin(1,p->spoiler_ratio);
    p->spoilers_armed=p->speedbrake_valid&&spoiler_handle<0;
    p->flap_fault=pfd_input_refs[PI_FLAP_FAILURE]&&XPLMGetDatai(pfd_input_refs[PI_FLAP_FAILURE])>=6;
    /* X-Plane's rel_autoslats is not an A380 slat-system fault. A fault
     * annunciation requires an explicit aircraft systems provider. */
    p->slat_fault=pfd_input_refs[PI_SLAT_FAILURE]&&XPLMGetDatai(pfd_input_refs[PI_SLAT_FAILURE])==1;
    p->stall_warning=pfd_input_refs[PI_STALL]&&XPLMGetDatai(pfd_input_refs[PI_STALL])!=0&&!p->on_ground;
    if(p->flap_valid)
        p->flap_valid=pfd_flap_geometry(handle,p->flap_ratio,&p->flap_target_ratio,&actual_index);
    /* Current ACF has slats at every extended detent; do not describe this
     * native schedule as the Airbus alpha-lock/automatic-retraction logic. */
    p->slat_target_ratio=handle>.001?1:0;
    p->flap_handle_index=(int)lround(handle*4);
    if(handle>.001&&p->flap_handle_index==0)p->flap_handle_index=1;
    p->flap_config=p->flap_handle_index?p->flap_handle_index+1:0;
    if(p->flap_handle_index==1&&p->flap_valid&&p->slat_valid&&p->flap_ratio<.001&&p->slat_ratio>.001)
        p->flap_config=1;
    /* During retraction retain the more restrictive actual-position limit.
     * This is a conservative port limit, not the missing PRIM/SFCC schedule. */
    limit_config=actual_index?actual_index+1:0;
    if(p->slat_valid&&p->slat_ratio>.001&&limit_config==0)limit_config=1;
    if(p->flap_config>limit_config)limit_config=p->flap_config;
    p->vmax_knots=guidance_inputs.vmax_knots;
    if(p->flap_valid&&p->slat_valid&&limit_config>=0&&limit_config<=5){
        p->vmax_knots=fmin(p->vmax_knots,vfe[limit_config]);
        p->vfe_next_knots=next[p->flap_config];
    }
    if(flaps_feed.ready){
        /* FBW SFCC (a380x_flaps.inc): lever, configuration text, FPPU surfaces and VFE; X-Plane's handle
         * carries the SFCC configuration, so the lever replaces it for the clean-performance gate. */
        handle=flaps_feed.handle_index/4.0;
        p->flap_valid=p->slat_valid=p->sfcc_valid=1;
        p->flap_handle_index=flaps_feed.handle_index;
        p->flap_config=flaps_feed.display_config;
        p->flap_ratio=flaps_feed.flap_deg/A380X_FLAPS_MAX_DEG;
        p->slat_ratio=flaps_feed.slat_deg/A380X_SLATS_MAX_DEG;
        p->flap_target_ratio=flaps_feed.flap_demand_deg/A380X_FLAPS_MAX_DEG;
        p->slat_target_ratio=flaps_feed.slat_demand_deg/A380X_SLATS_MAX_DEG;
        p->flap_fppu=flaps_feed.flap_fppu;p->slat_fppu=flaps_feed.slat_fppu;
        p->flap_relief=flaps_feed.relief;p->alpha_lock=flaps_feed.alpha_lock;
        p->flap_target_index=flaps_feed.flap_target;p->slat_target_index=flaps_feed.slat_target;
        p->vmax_knots=guidance_inputs.vmax_knots;
        if(flaps_feed.vfe_knots>0)p->vmax_knots=fmin(p->vmax_knots,flaps_feed.vfe_knots);
        p->vfe_next_knots=flaps_feed.vfe_next_knots;
    }
    if(speedbrake_feed.ready){
        /* SPEED BRAKE lever + FBW ground spoilers (a380x_speedbrake.inc): armed from the lever, the extension
         * as a fraction of its maximum (VLS increment, PFD symbol); X-Plane's handle carries the surface order. */
        p->speedbrake_valid=1;
        p->spoilers_armed=speedbrake_feed.armed;
        p->spoiler_ratio=speedbrake_feed.extension_ratio;
        spoiler_handle=speedbrake_feed.lever_pos;
    }
    if(pfd_input_refs[PI_GEAR])gear_count=XPLMGetDatavf(pfd_input_refs[PI_GEAR],gear,0,5);
    p->gear_down=gear_count==5;
    for(i=0;i<gear_count&&i<5;++i){
        if(!isfinite(gear[i])||gear[i]<.99)p->gear_down=0;
        if(isfinite(gear[i]))max_gear=fmax(max_gear,gear[i]);
    }
    if(max_gear>.01){
        p->vmax_knots=fmin(p->vmax_knots,250);
        gear_mach=pfd_mach_limit(.55);
        if(gear_mach>0)p->vmax_knots=fmin(p->vmax_knots,gear_mach);
    }
    p->green_dot_knots=pfd_green_dot(guidance_inputs.weight_kg,guidance_inputs.pressure_altitude_ft);
    p->envelope_valid=pfd_input_refs[PI_ENVELOPE_VALID]&&XPLMGetDatai(pfd_input_refs[PI_ENVELOPE_VALID])==1;
    if(p->envelope_valid){
#define PFD_SPEED(index,field) (void)pfd_input_number(index,40,450,&p->field)
        PFD_SPEED(PI_VLS,vls_knots);PFD_SPEED(PI_ALPHA_PROT,alpha_prot_knots);
        PFD_SPEED(PI_ALPHA_LIMIT,alpha_limit_knots);PFD_SPEED(PI_STALL_SPEED,stall_warning_knots);
        PFD_SPEED(PI_F_SPEED,f_speed_knots);PFD_SPEED(PI_S_SPEED,s_speed_knots);
        PFD_SPEED(PI_GD,green_dot_knots);PFD_SPEED(PI_V1,v1_knots);PFD_SPEED(PI_VR,vr_knots);
#undef PFD_SPEED
        { double provided; if(pfd_input_number(PI_VMAX,40,450,&provided))p->vmax_knots=fmin(p->vmax_knots,provided); }
        /* Reject inconsistent alpha bands rather than draw an inverted envelope. */
        if(p->alpha_limit_knots>p->alpha_prot_knots){p->alpha_limit_knots=0;p->alpha_prot_knots=0;}
    }
    /* T.O speeds are FMS data, not envelope data (spec 07 section 10.2): the native FMS
     * supplies them regardless of envelope_valid; a provider value above stays an override
     * only while the FMS has none. */
    if(fms_feed.valid){
        if(fms_feed.v1_knots>=40&&fms_feed.v1_knots<=450)p->v1_knots=fms_feed.v1_knots;
        if(fms_feed.vr_knots>=40&&fms_feed.vr_knots<=450)p->vr_knots=fms_feed.vr_knots;
        if(fms_feed.v2_knots>=40&&fms_feed.v2_knots<=450)p->v2_knots=fms_feed.v2_knots;
    }
    a380x_pfd_apply_fms_flap_speeds(p,&fms_feed);
    /* Clean performance requires positively known retracted surfaces, no
     * speedbrake request/deployment, valid air data and bounded table inputs.
     * Arming the stowed spoilers is permitted; no extension penalty applies.
     * The complete aircraft-system provider takes precedence when present. */
    if(!p->envelope_valid&&guidance_inputs.airdata_valid&&
       p->flap_valid&&p->slat_valid&&p->speedbrake_valid&&!p->flap_fault&&!p->slat_fault&&
       handle<=.0001&&p->flap_ratio<=.0001&&p->slat_ratio<=.0001&&
       spoiler_handle<=0&&p->spoiler_ratio<=.0001){
        p->vls_knots=pfd_clean_vls(guidance_inputs.weight_kg,guidance_inputs.pressure_altitude_ft);
        p->clean_performance_valid=p->vls_knots>0;
    }
    /* Stall warning speed without an aircraft-systems provider (docs/PFD_IMPLEMENTATION.md, "Stall warning
     * speed"). This port has no Airbus normal-law alpha protection, so the speed scale shows VSW, the
     * red/black barber pole of the laws without protection, instead of alpha prot / alpha max, and the
     * stall warning aural uses the same speed (a380x_aurals). VS1g: FBW A380 operating-speed tables
     * (a380x_operating_speeds) at the aircraft's actual mass and flap lever (CG 30 %MAC when the FMS has
     * none); VSW = max(1.05 VS1g, VS1g + 5 kt) * sqrt(nz): the CS 25.207(c) stall-warning margin, raised
     * with the load factor. */
    if(!p->envelope_valid&&guidance_inputs.airdata_valid&&guidance_inputs.weight_kg>200000.0&&
       guidance_inputs.weight_kg<700000.0&&p->flap_handle_index>=0&&p->flap_handle_index<=4){
        static XPLMDataRef g_normal;
        A380XOperatingSpeeds ops;
        double nz=1.0;
        if(!g_normal)g_normal=XPLMFindDataRef("sim/flightmodel/forces/g_nrml");
        if(g_normal){double g=XPLMGetDataf(g_normal);if(isfinite(g))nz=fmin(2.5,fmax(0.5,g));}
        if(p->on_ground)nz=1.0;
        a380x_operating_speeds(guidance_inputs.weight_kg,30.0,guidance_inputs.ias_knots,p->flap_handle_index,
                               A380X_SPEEDS_PHASE_APPROACH,0.0,guidance_inputs.pressure_altitude_ft,0.0,1,0.0,
                               1013.25,&ops);
        if(isfinite(ops.vs1g)&&ops.vs1g>60.0&&ops.vs1g<300.0)
            p->stall_warning_knots=fmax(1.05*ops.vs1g,ops.vs1g+5.0)*sqrt(nz);
    }
}
static void publish_pfd_aircraft(void)
{
#define PFD_COPY(field) pfd_guidance.field=pfd_aircraft.field
    PFD_COPY(on_ground);PFD_COPY(flap_handle_index);PFD_COPY(flap_config);
    PFD_COPY(flap_valid);PFD_COPY(slat_valid);PFD_COPY(speedbrake_valid);
    PFD_COPY(spoilers_armed);PFD_COPY(gear_down);PFD_COPY(flap_fault);PFD_COPY(slat_fault);
    PFD_COPY(stall_warning);PFD_COPY(envelope_valid);PFD_COPY(clean_performance_valid);
    PFD_COPY(flap_ratio);PFD_COPY(slat_ratio);PFD_COPY(spoiler_ratio);
    PFD_COPY(flap_target_ratio);PFD_COPY(slat_target_ratio);
    PFD_COPY(vmax_knots);PFD_COPY(vfe_next_knots);PFD_COPY(vls_knots);
    PFD_COPY(alpha_prot_knots);PFD_COPY(alpha_limit_knots);PFD_COPY(stall_warning_knots);
    PFD_COPY(f_speed_knots);PFD_COPY(s_speed_knots);PFD_COPY(green_dot_knots);
    PFD_COPY(v1_knots);PFD_COPY(vr_knots);PFD_COPY(v2_knots);
    PFD_COPY(sfcc_valid);PFD_COPY(flap_relief);PFD_COPY(alpha_lock);
    PFD_COPY(slat_target_index);PFD_COPY(flap_target_index);PFD_COPY(flap_fppu);PFD_COPY(slat_fppu);
#undef PFD_COPY
}
static void pfd_outputs_shutdown(void)
{
    int i;for(i=0;i<ARRAY_LEN(pfd_outputs);++i)if(pfd_outputs[i].handle){
        XPLMUnregisterDataAccessor(pfd_outputs[i].handle);pfd_outputs[i].handle=NULL;
    }
}
static int pfd_outputs_init(void)
{
    int i;for(i=0;i<ARRAY_LEN(pfd_outputs);++i){
        if(XPLMFindDataRef(pfd_outputs[i].name)){pfd_outputs_shutdown();return 0;}
        pfd_outputs[i].handle=XPLMRegisterDataAccessor(pfd_outputs[i].name,xplmType_Int|xplmType_Float,0,
            pfd_get_output_int,NULL,pfd_get_output_float,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,&pfd_outputs[i],NULL);
        if(!pfd_outputs[i].handle){pfd_outputs_shutdown();return 0;}
    }
    return 1;
}
