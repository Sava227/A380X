/* SPDX-License-Identifier: GPL-3.0-or-later
 * Read-only live input transport to the aircraft-local SD renderer.
 * No preview scenarios, estimates, or SDK writes are used here.
 * FBW page variables (exact SimVar names, SDv2 bus topics as "ev:<topic>") come from
 * a380x_sd_data.inc; see docs/SD_PAGES.md for the per-page census and every derivation.
 */
#include "a380x_sd_data.inc"
static struct { const char *name; XPLMDataRef ref; } xpkit_sd_inputs[]={
#define SDRAW(n) {n,NULL}
 SDRAW("sim/flightmodel/failures/onground_any"),SDRAW("sim/flightmodel/misc/h_ind"),
 SDRAW("sim/flightmodel/weight/m_total"),SDRAW("sim/flightmodel/weight/m_fuel_total"),
 SDRAW("sim/time/zulu_time_sec"),SDRAW("sim/cockpit2/temperature/outside_air_temp_degc"),SDRAW("sim/cockpit2/temperature/outside_air_LE_temp_degc"),
 SDRAW("sim/cockpit2/engine/indicators/N2_percent"),SDRAW("sim/cockpit2/engine/indicators/fuel_flow_kg_sec"),
 SDRAW("sim/cockpit2/engine/indicators/oil_temperature_deg_C"),SDRAW("sim/cockpit2/engine/indicators/oil_pressure_psi"),
 SDRAW("sim/cockpit2/engine/indicators/engine_running"),SDRAW("sim/aircraft/engine/acf_oilT_is_C"),
 SDRAW("a380x/xp12/apu/n_percent"),SDRAW("sim/cockpit2/electrical/APU_EGT_c"),
 SDRAW("sim/cockpit2/electrical/APU_running"),SDRAW("sim/cockpit2/electrical/APU_starter_switch"),
 SDRAW("sim/cockpit2/electrical/bus_volts"),SDRAW("sim/cockpit2/electrical/generator_volts"),SDRAW("sim/cockpit2/electrical/generator_on"),
 SDRAW("sim/cockpit2/electrical/battery_voltage_actual_volts"),SDRAW("sim/cockpit2/electrical/battery_amps"),SDRAW("sim/cockpit2/electrical/battery_on"),
 SDRAW("sim/cockpit2/bleedair/actuators/apu_bleed"),SDRAW("sim/cockpit2/bleedair/actuators/engine_bleed_sov"),
 SDRAW("sim/cockpit2/bleedair/actuators/pack_left"),SDRAW("sim/cockpit2/bleedair/actuators/pack_right"),
 SDRAW("sim/cockpit2/pressurization/indicators/cabin_altitude_ft"),SDRAW("sim/cockpit2/pressurization/indicators/cabin_vvi_fpm"),
 SDRAW("sim/cockpit2/pressurization/indicators/pressure_diffential_psi"),SDRAW("sim/cockpit2/pressurization/indicators/outflow_valve"),
 SDRAW("sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1"),SDRAW("sim/cockpit2/hydraulics/indicators/hydraulic_pressure_2"),
 SDRAW("sim/cockpit2/hydraulics/indicators/hydraulic_fluid_ratio_1"),SDRAW("sim/cockpit2/hydraulics/indicators/hydraulic_fluid_ratio_2"),
 SDRAW("sim/cockpit2/controls/gear_handle_down"),SDRAW("sim/flightmodel2/gear/deploy_ratio"),
 SDRAW("a380x/xp12/cockpit_door/position"),
 SDRAW("a380x/xp12/engine/n3_percent"),SDRAW("a380x/xp12/engine/oil_quantity_quarts"),
 SDRAW("a380x/xp12/apu/n2_percent"),SDRAW("a380x/xp12/apu/bleed_pressure_psi"),SDRAW("a380x/xp12/apu/fuel_used_kg")
#undef SDRAW
};
static uint64_t xpkit_sd_hash;
static double xpkit_sd_read_time,xpkit_sd_find_time;
static int xpkit_sd_action=-1;
static void xpkit_sd_publish(void) {
    double now=xpkit_now();
    if(now-xpkit_sd_read_time<.25 && xpkit_sd_action==sda.action_count)return;
    xpkit_sd_read_time=now;xpkit_sd_action=sda.action_count;
    if(now-xpkit_sd_find_time>=1) {
        for(int i=0;i<ARRAY_LEN(xpkit_sd_inputs);++i)xpkit_sd_inputs[i].ref=XPLMFindDataRef(xpkit_sd_inputs[i].name);
        xpkit_sd_find_time=now;
    }
    static XpKitJson j;j.size=0;j.failed=0;
    xpkit_append(&j,"{\"page\":%d,\"powered\":%s,\"more\":%s,\"manual\":%s,\"action\":%d,\"vars\":{",sda.page,sda.powered?"true":"false",sda.more?"true":"false",sda.manual?"true":"false",sda.action_count);
    for(int i=0;i<ARRAY_LEN(xpkit_sd_inputs);++i) {
        XPLMDataRef ref=xpkit_sd_inputs[i].ref;if(!ref)continue;
        XPLMDataTypeID type=XPLMGetDataRefTypes(ref);
        if(type&(xplmType_FloatArray|xplmType_IntArray)) {
            float f[32];int ints[32],count;
            if(type&xplmType_FloatArray)count=XPLMGetDatavf(ref,f,0,32);
            else {count=XPLMGetDatavi(ref,ints,0,32);for(int k=0;k<count&&k<32;++k)f[k]=(float)ints[k];}
            if(count<1||count>32)continue;
            xpkit_append(&j,"\"%s\":[",xpkit_sd_inputs[i].name);
            for(int k=0;k<count;++k) {if(k)xpkit_append(&j,",");if(isfinite(f[k]))xpkit_append(&j,"%.7g",f[k]);else xpkit_append(&j,"null");}
            xpkit_append(&j,"],");
        } else {double v=type&xplmType_Float?XPLMGetDataf(ref):type&xplmType_Int?XPLMGetDatai(ref):NAN;xpkit_number(&j,xpkit_sd_inputs[i].name,v);}
    }
    {
        /* STATUS lists from the native FWS (a380x_fws_adapter.inc, included before this file). */
        static SddStatus status;
        const FwsStatusLists *lists=&fwsa_fws.status;
        memset(&status,0,sizeof(status));
        status.available=fwsa_available();
        for(unsigned k=0;k<5&&k<lists->information.n;++k)status.info[k]=lists->information.id[k];
        for(unsigned k=0;k<10&&k<lists->inop_sys_all_phases.n;++k)status.inop_all[k]=lists->inop_sys_all_phases.id[k];
        for(unsigned k=0;k<10&&k<lists->inop_sys_appr_ldg.n;++k)status.inop_ldg[k]=lists->inop_sys_appr_ldg.id[k];
        status.limitation_all=lists->limitations_all_phases.n?lists->limitations_all_phases.id[0]:0;
        status.limitation_ldg=lists->limitations_appr_ldg.n?lists->limitations_appr_ldg.id[0]:0;
        status.limitation_pfd=lists->limitations_pfd.n?lists->limitations_pfd.id[0]:0;
        SddContext context={now,sda.warnings.engine[0].idle_n1,sda.warnings.engine[0].idle_n1_valid,&status};
        sdd_emit(&j,&context);
    }
    xpkit_append(&j,"\"_end\":0},\"warnings\":[");
    EwdData filtered;sda_filter_ewd(&filtered,&sda.warnings);
    for(int i=0;i<filtered.warning_count;++i) {
        if(i)xpkit_append(&j,",");
        xpkit_append(&j,"\"");
        for(const unsigned char *p=(const unsigned char*)filtered.warning[i].text;*p;++p) {
            if(*p=='"'||*p=='\\')xpkit_append(&j,"\\%c",*p);else if(*p>=32)xpkit_append(&j,"%c",*p);
        }xpkit_append(&j,"\"");
    }
    xpkit_append(&j,"],\"fwsComplete\":false}");
    if(j.failed||j.size>=XPKIT_SD_CAP){++xpkit_health[3];return;}
    uint64_t hash=1469598103934665603ULL;for(size_t i=0;i<j.size;++i)hash=(hash^(unsigned char)j.data[i])*1099511628211ULL;
    if(hash==xpkit_sd_hash&&atomic_load(&xpkit_shared->sd.sequence))return;
    A380XXpKitSd *out=&xpkit_shared->sd;unsigned seq=atomic_load(&out->sequence);
    atomic_store_explicit(&out->sequence,seq+1,memory_order_release);
    memcpy(out->json,j.data,j.size+1);out->size=(uint32_t)j.size;
    atomic_store_explicit(&out->sequence,seq+2,memory_order_release);xpkit_sd_hash=hash;
    if(!sda.manual&&!sda.all&&sda_auto_page<0) {   /* automatic APU / ENGINE start pages win */
        int page=SD_ENGINE;XPLMDataRef ground=xpkit_sd_inputs[0].ref,alt=xpkit_sd_inputs[1].ref;
        if(ground&&alt&&!XPLMGetDatai(ground)&&XPLMGetDataf(alt)>20000)page=SD_CRUISE;
        if(sda.page!=page){sda.page=page;sda.force=1;xpkit_sd_action=-1;}
    }
}
