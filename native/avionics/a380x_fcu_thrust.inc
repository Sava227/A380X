/* SPDX-License-Identifier: GPL-3.0-or-later
 * Raw pilot lever / calibrated detent boundary. Never read ENGN_thro_use here:
 * that is the A/THR actuator output and would feed our own command back as TLA.
 * A missing or invalid aircraft-local calibration disables automatic detents.
 */
#include <ctype.h>
#include <errno.h>
#include "XPLMPlanes.h"

typedef struct {
    double idle, cl, mct, toga, tolerance;
    int calibrated, raw_warning;
    XPLMDataRef levers, prop_mode;
    XPLMDataRef envelope_valid, alpha_floor_request;
    XPLMDataRef flex_mode, flex_temperature, fmc_healthy[3];
    char aircraft_path[1024], calibration_path[1200];
} FcuThrustBoundary;
static FcuThrustBoundary fcu_thrust;

static char *fcu_thrust_trim(char *text)
{
    char *end;
    while (isspace((unsigned char)*text)) ++text;
    end=text+strlen(text);
    while (end>text && isspace((unsigned char)end[-1])) --end;
    *end=0;
    return text;
}

/* Strict, bounded INI: no sections, duplicate/unknown keys, non-finite values,
 * trailing tokens or partially specified calibration are accepted. */
static int fcu_thrust_parse(FILE *file, FcuThrustBoundary *out)
{
    static const char *const names[]={"version","idle_detent_ratio","cl_detent_ratio",
        "mct_detent_ratio","toga_detent_ratio","detent_tolerance_ratio"};
    double values[6]={0};
    unsigned seen=0;
    char source[4097],*cursor;
    size_t total=fread(source,1,sizeof(source),file);
    if (ferror(file) || total>4096 || memchr(source,0,total)) return 0;
    source[total]=0; cursor=source;
    while (*cursor) {
        char *key,*eq,*value,*tail;
        double number;
        int index;
        char *end=strchr(cursor,'\n');
        size_t length=end?(size_t)(end-cursor):strlen(cursor);
        if (length>=256) return 0;
        key=cursor;
        if(end) { *end=0; cursor=end+1; } else cursor+=length;
        key=fcu_thrust_trim(key);
        if (!*key || *key=='#' || *key==';') continue;
        eq=strchr(key,'=');
        if (!eq) return 0;
        *eq=0; key=fcu_thrust_trim(key); value=fcu_thrust_trim(eq+1);
        for (index=0;index<6 && strcmp(key,names[index]);++index) {}
        if (index==6 || (seen&(1u<<index)) || !*value) return 0;
        errno=0; number=strtod(value,&tail);
        if (errno || tail==value || *fcu_thrust_trim(tail) || !isfinite(number)) return 0;
        values[index]=number; seen|=1u<<index;
    }
    if (ferror(file) || seen!=63u || values[0]!=1 || values[1]<0 || values[4]>1
        || !(values[1]<values[2] && values[2]<values[3] && values[3]<values[4])
        || !(values[5]>0 && values[5]<=.1)
        || values[2]-values[1]<=2*values[5] || values[3]-values[2]<=2*values[5]
        || values[4]-values[3]<=2*values[5]) return 0;
    out->idle=values[1]; out->cl=values[2]; out->mct=values[3];
    out->toga=values[4]; out->tolerance=values[5];
    return 1;
}

static int fcu_thrust_config_path(void)
{
    char filename[256]={0},pathname[1024]={0};
    const char *slash,*backslash,*base;
    size_t prefix;
    XPLMGetNthAircraftModel(0,filename,pathname);
    if (!filename[0] || !pathname[0]) return 0;
    slash=strrchr(pathname,'/'); backslash=strrchr(pathname,'\\');
    if (backslash && (!slash || backslash>slash)) slash=backslash;
    if (!slash) return 0;
    base=slash+1;
    if (strcmp(base,filename) || strlen(filename)<5) return 0;
    {
        const char *ext=filename+strlen(filename)-4;
        if (ext[0]!='.' || tolower((unsigned char)ext[1])!='a'
            || tolower((unsigned char)ext[2])!='c' || tolower((unsigned char)ext[3])!='f') return 0;
    }
    prefix=(size_t)(base-pathname);
    if (prefix+strlen("fcu-calibration.ini")>=sizeof(fcu_thrust.calibration_path)) return 0;
    memcpy(fcu_thrust.calibration_path,pathname,prefix);
    strcpy(fcu_thrust.calibration_path+prefix,"fcu-calibration.ini");
    snprintf(fcu_thrust.aircraft_path,sizeof(fcu_thrust.aircraft_path),"%s",pathname);
    return 1;
}

static void fcu_thrust_shutdown(void) { memset(&fcu_thrust,0,sizeof(fcu_thrust)); }

static int fcu_thrust_init(void)
{
    FILE *file=NULL;
    fcu_thrust_shutdown();
    fcu_thrust.levers=XPLMFindDataRef("sim/cockpit2/engine/actuators/throttle_ratio");
    fcu_thrust.prop_mode=XPLMFindDataRef("sim/cockpit2/engine/actuators/prop_mode");
    fcu_thrust.envelope_valid=XPLMFindDataRef("a380x/xp12/flight_envelope/valid");
    fcu_thrust.alpha_floor_request=XPLMFindDataRef("a380x/xp12/flight_envelope/alpha_floor_request");
    if (fcu_thrust_config_path()) file=fopen(fcu_thrust.calibration_path,"rb");
    if (file) {
        fcu_thrust.calibrated=fcu_thrust_parse(file,&fcu_thrust);
        fclose(file);
    }
    if (!fcu_thrust.calibrated)
        XPLMDebugString("A380X FCU: missing/invalid aircraft-local fcu-calibration.ini; automatic thrust detents disabled.\n");
    else
        XPLMDebugString("A380X FCU: aircraft-local pilot-lever calibration loaded; no actuator feedback used as TLA.\n");
    /* This optional capability must never prevent the rest of the FCU loading. */
    return 1;
}

static void fcu_thrust_optional_refs(void)
{
    int i;
    static const char *const healthy[]={"a380x/xp12/fms/fmc_a_healthy",
        "a380x/xp12/fms/fmc_b_healthy","a380x/xp12/fms/fmc_c_healthy"};
    /* FMS and optional envelope providers may register after this adapter. */
    if (!fcu_thrust.envelope_valid)
        fcu_thrust.envelope_valid=XPLMFindDataRef("a380x/xp12/flight_envelope/valid");
    if (!fcu_thrust.alpha_floor_request)
        fcu_thrust.alpha_floor_request=XPLMFindDataRef("a380x/xp12/flight_envelope/alpha_floor_request");
    if (!fcu_thrust.flex_mode) fcu_thrust.flex_mode=XPLMFindDataRef("a380x/xp12/fms/to_power_setting");
    if (!fcu_thrust.flex_temperature) fcu_thrust.flex_temperature=XPLMFindDataRef("a380x/xp12/fms/to_flex_temp_c");
    for (i=0;i<3;++i) if (!fcu_thrust.fmc_healthy[i]) fcu_thrust.fmc_healthy[i]=XPLMFindDataRef(healthy[i]);
}

/* The native flight-control laws (a380x_fctl.inc): α floor for the A/THR, high-AoA protection for the
 * speed brake. ready = the laws run; the dataref provider path below takes precedence. */
static struct { int ready, alpha_floor, alpha_prot; } fctl_link;

static void fcu_thrust_read_into(A380XGuidanceInputs *in)
{
    float levers[4]={0};
    int modes[4]={0},i,raw_valid=0,healthy=0;
    if (!in) return;
    in->throttle_levers_valid=0;
    in->flex_valid=in->alpha_floor_valid=in->alpha_floor_request=0;
    in->athr_inhibited=in->athr_ceiling_valid=0; in->athr_ceiling=0;
    for (i=0;i<4;++i) { in->throttle_lever_ratio[i]=0; in->reverse_selected[i]=0; }
    in->idle_detent_ratio=fcu_thrust.calibrated?fcu_thrust.idle:0;
    in->cl_detent_ratio=fcu_thrust.calibrated?fcu_thrust.cl:0;
    in->mct_detent_ratio=fcu_thrust.calibrated?fcu_thrust.mct:0;
    in->toga_detent_ratio=fcu_thrust.calibrated?fcu_thrust.toga:0;
    in->detent_tolerance_ratio=fcu_thrust.calibrated?fcu_thrust.tolerance:0;
    if (fcu_thrust.levers && fcu_thrust.prop_mode
        && (XPLMGetDataRefTypes(fcu_thrust.levers)&xplmType_FloatArray)
        && (XPLMGetDataRefTypes(fcu_thrust.prop_mode)&xplmType_IntArray)
        && XPLMGetDatavf(fcu_thrust.levers,levers,0,4)==4
        && XPLMGetDatavi(fcu_thrust.prop_mode,modes,0,4)==4) {
        raw_valid=1;
        for (i=0;i<4;++i) {
            if (!isfinite(levers[i]) || levers[i]<0 || levers[i]>1 || modes[i]<0 || modes[i]>3) raw_valid=0;
        }
        if (raw_valid) for (i=0;i<4;++i) {
            in->throttle_lever_ratio[i]=levers[i]; in->reverse_selected[i]=modes[i]==3;
        }
    }
    in->throttle_levers_valid=fcu_thrust.calibrated && raw_valid;
    if (fcu_thrust.calibrated && !raw_valid && !fcu_thrust.raw_warning) {
        XPLMDebugString("A380X FCU: invalid pilot throttle/reverse input; automatic thrust detents disabled until valid.\n");
        fcu_thrust.raw_warning=1;
    } else if (raw_valid && fcu_thrust.raw_warning) {
        XPLMDebugString("A380X FCU: pilot throttle/reverse input valid again.\n");
        fcu_thrust.raw_warning=0;
    }
    fcu_thrust_optional_refs();
    if (fcu_thrust.envelope_valid && fcu_thrust.alpha_floor_request
        && (XPLMGetDataRefTypes(fcu_thrust.envelope_valid)&xplmType_Int)
        && (XPLMGetDataRefTypes(fcu_thrust.alpha_floor_request)&xplmType_Int)) {
        int valid=XPLMGetDatai(fcu_thrust.envelope_valid),request=XPLMGetDatai(fcu_thrust.alpha_floor_request);
        if (valid==1 && (request==0 || request==1)) {
            in->alpha_floor_valid=1; in->alpha_floor_request=request;
        }
    }
    if (!in->alpha_floor_valid && fctl_link.ready) {
        in->alpha_floor_valid=1; in->alpha_floor_request=fctl_link.alpha_floor!=0;
    }
    for (i=0;i<3;++i) if (fcu_thrust.fmc_healthy[i]
        && (XPLMGetDataRefTypes(fcu_thrust.fmc_healthy[i])&xplmType_Int)
        && XPLMGetDatai(fcu_thrust.fmc_healthy[i])==1) healthy=1;
    if (healthy && fcu_thrust.flex_mode && fcu_thrust.flex_temperature
        && (XPLMGetDataRefTypes(fcu_thrust.flex_mode)&xplmType_Int)
        && (XPLMGetDataRefTypes(fcu_thrust.flex_temperature)&xplmType_Float)
        && XPLMGetDatai(fcu_thrust.flex_mode)==1) {
        double temp=XPLMGetDataf(fcu_thrust.flex_temperature);
        /* The native FMS emits 0 for no entry and 0.1 for a parsed FLEX 0 C
         * entry. No assumption is made from AoA, throttle position or SAT. */
        in->flex_valid=isfinite(temp) && temp>0 && temp<=99;
    }
    /* The thrust-lever / FADEC module owns the FBW FLEX rating (TAT, latch after take-off), the
     * 15 s instinctive-disconnect inhibit and the A/THR ceiling (docs/THRUST_LEVERS.md). */
    if (fcu_thrust_link.ready) {
        in->flex_valid=fcu_thrust_link.flex_rating!=0;
        in->athr_inhibited=fcu_thrust_link.athr_inhibited!=0;
        if (fcu_thrust_link.ceiling_valid) {
            double ceiling=0;
            for (i=0;i<4;++i) if (isfinite(fcu_thrust_link.ceiling[i])) ceiling=fmax(ceiling,fcu_thrust_link.ceiling[i]);
            in->athr_ceiling_valid=1;
            in->athr_ceiling=fmin(1,ceiling);
        }
    }
}
