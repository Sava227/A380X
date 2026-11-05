/* Aircraft-scoped OBJ animation datarefs and their native visual driver.
 * Register in XPluginStart, before X-Plane resolves attached OBJ datarefs:
 * https://developer.x-plane.com/article/deferredinitialization/
 *
 * The plugin owns 22 cached Floats (the exterior aileron, elevator, rudder and spoiler animations)
 * and, from XPluginEnable, drives them from a flight loop with a C port of the two FlyWithLua visual
 * drivers (a380x_surface_motion.c), so the surfaces move whatever the aircraft folder is called and
 * without FlyWithLua. It READS simulator state only: no flight control, override or flight-model
 * dataref is ever written. a380x/xp12/hydraulics/native_driver (Int, read-only) is 1 while the native
 * driver owns these writes; the global FlyWithLua drivers stand down when it is 1.
 */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

#include "a380x_surface_motion.h"

typedef struct {
    const char *name;
    float minimum;
    float maximum;
    XPLMDataRef handle;
} VisualRef;

#define REF(prefix) {"a380x/xp12/visual/aileron_" prefix "_ratio", -1.0f, 1.0f, NULL}
#define HYD_REF(name) {"a380x/xp12/hydraulics/" name "_ratio", -1.0f, 1.0f, NULL}
#define SPOILER_REF(name) {"a380x/xp12/hydraulics/spoiler_" name "_deg", 0.0f, 65.0f, NULL}
static VisualRef refs[] = {
    REF("l_inboard"), REF("l_midboard"), REF("l_outboard"),
    REF("r_inboard"), REF("r_midboard"), REF("r_outboard"),
    HYD_REF("aileron_l_inboard"), HYD_REF("aileron_l_midboard"), HYD_REF("aileron_l_outboard"),
    HYD_REF("aileron_r_inboard"), HYD_REF("aileron_r_midboard"), HYD_REF("aileron_r_outboard"),
    HYD_REF("elevator"), HYD_REF("rudder"),
    SPOILER_REF("l_inboard"), SPOILER_REF("r_inboard"),
    SPOILER_REF("l_inner"), SPOILER_REF("r_inner"),
    SPOILER_REF("l_middle"), SPOILER_REF("r_middle"),
    SPOILER_REF("l_outer"), SPOILER_REF("r_outer")
};
#undef REF
#undef HYD_REF
#undef SPOILER_REF
enum { REF_COUNT = sizeof(refs) / sizeof(refs[0]) };
#define NATIVE_DRIVER_NAME "a380x/xp12/hydraulics/native_driver"

/* Simulator inputs of the two FlyWithLua drivers (read only). */
enum {
    IN_ROLL, IN_PITCH, IN_YAW, IN_GROUNDED, IN_PAUSED, IN_DT, IN_HEADING, IN_WIND_DIR, IN_WIND_SPEED,
    IN_AIRSPEED, IN_P1, IN_P2, IN_P3, IN_SPEEDBRAKE1, IN_SPOILER1, IN_SPOILER2, IN_COUNT
};
typedef struct {
    const char *name;
    XPLMDataTypeID type;
    XPLMDataRef ref;
} SourceRef;
static SourceRef sources[IN_COUNT] = {
    {"sim/flightmodel2/controls/roll_ratio", xplmType_Float, NULL},
    {"sim/flightmodel2/controls/pitch_ratio", xplmType_Float, NULL},
    {"sim/flightmodel2/controls/heading_ratio", xplmType_Float, NULL},
    {"sim/flightmodel/failures/onground_any", xplmType_Int, NULL},
    {"sim/time/paused", xplmType_Int, NULL},
    {"sim/operation/misc/frame_rate_period", xplmType_Float, NULL},
    {"sim/flightmodel/position/psi", xplmType_Float, NULL},
    {"sim/weather/aircraft/wind_now_direction_degt", xplmType_Float, NULL},
    {"sim/weather/aircraft/wind_now_speed_msc", xplmType_Float, NULL},
    {"sim/flightmodel/position/true_airspeed", xplmType_Float, NULL},
    {"sim/cockpit2/hydraulics/indicators/hydraulic_pressure_1", xplmType_Float, NULL},
    {"sim/cockpit2/hydraulics/indicators/hydraulic_pressure_2", xplmType_Float, NULL},
    {"sim/cockpit2/hydraulics/indicators/hydraulic_pressure_3", xplmType_Float, NULL},
    {"sim/flightmodel2/wing/speedbrake1_deg", xplmType_FloatArray, NULL},
    {"sim/flightmodel2/wing/spoiler1_deg", xplmType_FloatArray, NULL},
    {"sim/flightmodel2/wing/spoiler2_deg", xplmType_FloatArray, NULL},
};

static A380XSurfaceDriver driver;
static XPLMDataRef native_flag_handle;
static XPLMFlightLoopID driver_loop;
static int started = 0, running = 0, logged_nonzero = 0;

static float read_value(void *refcon)
{
    const VisualRef *ref = (const VisualRef *)refcon;
    return ref != NULL ? driver.pub[ref - refs] : 0.0f;
}

static void write_value(void *refcon, float value)
{
    VisualRef *ref = (VisualRef *)refcon;
    /* Reject NaN/infinity and retain the last valid pose. Never export NaN. */
    if (ref == NULL)
        return;
    a380x_surface_store(&driver.pub[ref - refs], value, ref->minimum, ref->maximum);
}

static int read_native_flag(void *refcon)
{
    (void)refcon;
    return running;
}

static void clear_values(void)
{
    memset(driver.pub, 0, sizeof(driver.pub));
}

static void unregister_owned(void)
{
    int i;
    for (i = 0; i < REF_COUNT; ++i) {
        if (refs[i].handle != NULL) {
            XPLMUnregisterDataAccessor(refs[i].handle);
            refs[i].handle = NULL;
        }
    }
    if (native_flag_handle != NULL) {
        XPLMUnregisterDataAccessor(native_flag_handle);
        native_flag_handle = NULL;
    }
    clear_values();
    started = 0;
}

/* Every input must exist with its type; otherwise the driver stays off (refs hold their pose). */
static int bind_sources(void)
{
    int i, ok = 1;
    for (i = 0; i < IN_COUNT; ++i) {
        sources[i].ref = XPLMFindDataRef(sources[i].name);
        if (sources[i].ref == NULL || (XPLMGetDataRefTypes(sources[i].ref) & sources[i].type) == 0) {
            XPLMDebugString("A380X surface driver: missing or mistyped input ");
            XPLMDebugString(sources[i].name);
            XPLMDebugString("\n");
            sources[i].ref = NULL;
            ok = 0;
        }
    }
    return ok;
}

static void read_array(int index, float *out, int offset, int count)
{
    int i, got = XPLMGetDatavf(sources[index].ref, out, offset, count);
    for (i = got < 0 ? 0 : got; i < count; ++i)
        out[i] = NAN;   /* a short array is invalid input: the hydraulic driver holds */
}

static void read_sources(A380XSurfaceSources *s)
{
    s->roll = XPLMGetDataf(sources[IN_ROLL].ref);
    s->pitch = XPLMGetDataf(sources[IN_PITCH].ref);
    s->yaw = XPLMGetDataf(sources[IN_YAW].ref);
    s->grounded = XPLMGetDatai(sources[IN_GROUNDED].ref);
    s->paused = XPLMGetDatai(sources[IN_PAUSED].ref);
    s->dt = XPLMGetDataf(sources[IN_DT].ref);
    s->heading = XPLMGetDataf(sources[IN_HEADING].ref);
    s->wind_direction = XPLMGetDataf(sources[IN_WIND_DIR].ref);
    s->wind_speed = XPLMGetDataf(sources[IN_WIND_SPEED].ref);
    s->airspeed = XPLMGetDataf(sources[IN_AIRSPEED].ref);
    s->pressures[0] = XPLMGetDataf(sources[IN_P1].ref);
    s->pressures[1] = XPLMGetDataf(sources[IN_P2].ref);
    s->pressures[2] = XPLMGetDataf(sources[IN_P3].ref);
    read_array(IN_SPEEDBRAKE1, s->speedbrake1, 0, 2);
    read_array(IN_SPOILER1, s->spoiler1, 0, 4);
    read_array(IN_SPOILER2, s->spoiler2, 4, 2);
}

static float driver_callback(float since_last_call, float since_last_loop, int counter, void *refcon)
{
    A380XSurfaceSources input;
    (void)since_last_call;
    (void)since_last_loop;
    (void)counter;
    (void)refcon;
    if (!running)
        return 0.0f;
    read_sources(&input);
    a380x_surface_driver_step(&driver, &input);
    if (driver.dance_reported_nonzero && !logged_nonzero) {
        char line[256];
        snprintf(line, sizeof(line), "A380X surface driver: first nonzero aileron publication "
                 "(L in/mid/out, R in/mid/out) %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                 driver.pub[0], driver.pub[1], driver.pub[2], driver.pub[3], driver.pub[4], driver.pub[5]);
        XPLMDebugString(line);
        logged_nonzero = 1;
    }
    return -1.0f;   /* every frame */
}

PLUGIN_API int XPluginStart(char *name, char *signature, char *description)
{
    int i;
    int collision = 0;
    strcpy(name, "A380X Control Surface Visual Datarefs");
    strcpy(signature, "org.a380x.xp12.aileron-visual-datarefs.v1");
    strcpy(description, "Early aircraft OBJ visual datarefs and their native visual driver.");
    if (started) {
        XPLMDebugString("A380X Aileron Datarefs: duplicate start refused.\n");
        return 0;
    }

    /* Preflight ALL names before registering ANY, regardless of owner/type.
     * Do not borrow/unregister another provider's references. A late Lua
     * provider must bind to this complete set, not create a competing set.
     */
    for (i = 0; i < REF_COUNT; ++i) {
        if (XPLMFindDataRef(refs[i].name) != NULL) {
            XPLMDebugString("A380X Aileron Datarefs: existing dataref collision: ");
            XPLMDebugString(refs[i].name);
            XPLMDebugString("\n");
            collision = 1;
        }
    }
    if (XPLMFindDataRef(NATIVE_DRIVER_NAME) != NULL) {
        XPLMDebugString("A380X Aileron Datarefs: existing dataref collision: " NATIVE_DRIVER_NAME "\n");
        collision = 1;
    }
    if (collision)
        return 0;

    memset(&driver, 0, sizeof(driver));
    a380x_surface_driver_reset(&driver);
    running = 0;
    for (i = 0; i < REF_COUNT; ++i) {
        refs[i].handle = XPLMRegisterDataAccessor(
            refs[i].name, xplmType_Float, 1,
            NULL, NULL, read_value, write_value,
            NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
            &refs[i], &refs[i]);
        if (refs[i].handle == NULL) {
            XPLMDebugString("A380X Aileron Datarefs: registration failed; rolling back.\n");
            unregister_owned();
            return 0;
        }
    }
    native_flag_handle = XPLMRegisterDataAccessor(
        NATIVE_DRIVER_NAME, xplmType_Int, 0,
        read_native_flag, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL, NULL);
    if (native_flag_handle == NULL) {
        XPLMDebugString("A380X Aileron Datarefs: registration failed; rolling back.\n");
        unregister_owned();
        return 0;
    }
    started = 1;
    XPLMDebugString("A380X Aileron Datarefs: 22 writable Floats and the native-driver flag registered in XPluginStart.\n");
    return 1;
}

PLUGIN_API void XPluginStop(void)
{
    running = 0;
    if (driver_loop != NULL) {
        XPLMDestroyFlightLoop(driver_loop);
        driver_loop = NULL;
    }
    unregister_owned();
}

PLUGIN_API int XPluginEnable(void)
{
    XPLMCreateFlightLoop_t loop;
    if (!started)
        return 0;
    /* A fresh FlyWithLua-style load on every enable: neutral cores, first frame seeds them. */
    a380x_surface_driver_reset(&driver);
    logged_nonzero = 0;
    running = 0;
    if (!bind_sources()) {
        XPLMDebugString("A380X surface driver: inputs unavailable; exterior surfaces hold their pose.\n");
        return 1;
    }
    if (driver_loop == NULL) {
        memset(&loop, 0, sizeof(loop));
        loop.structSize = (int)sizeof(loop);
        loop.phase = xplm_FlightLoop_Phase_AfterFlightModel;
        loop.callbackFunc = driver_callback;
        loop.refcon = NULL;
        driver_loop = XPLMCreateFlightLoop(&loop);
    }
    if (driver_loop == NULL) {
        XPLMDebugString("A380X surface driver: flight loop unavailable; exterior surfaces hold their pose.\n");
        return 1;
    }
    running = 1;
    XPLMScheduleFlightLoop(driver_loop, -1.0f, 1);
    XPLMDebugString("A380X surface driver: native aileron and hydraulic surface driver running "
                    "(" NATIVE_DRIVER_NAME " = 1); simulator flight controls unchanged.\n");
    return 1;
}

PLUGIN_API void XPluginDisable(void)
{
    running = 0;
    if (driver_loop != NULL) {
        XPLMDestroyFlightLoop(driver_loop);
        driver_loop = NULL;
    }
    /* Keep ownership until Stop; resume from neutral, not an old visual pose. */
    clear_values();
    a380x_surface_driver_reset(&driver);
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID from, int message, void *param)
{
    (void)from;
    /* A (re)loaded user aircraft or a new location is a time discontinuity: seed again from the
     * current inputs on the next frame, as FlyWithLua's script reload does. */
    if ((message == XPLM_MSG_PLANE_LOADED && (intptr_t)param == 0) || message == XPLM_MSG_AIRPORT_LOADED)
        a380x_surface_driver_reset(&driver);
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
