/* SPDX-License-Identifier: GPL-3.0-or-later
 * Production flight-control adapter (a380x_fctl.inc) against a mock XPLM: registry contract, gear
 * identification (nose / body), steering through override_wheel_steer (hardware and keyboard tiller),
 * BetterPushback and foreign-override stand-down, the artstab channel in the flight law, the AP bypass,
 * auto-trim range conversion, the A.FLOOR link, the crew toggles, replay, disable and shutdown.
 *
 *   ./test_fctl_adapter              run the tests
 *   ./test_fctl_adapter --contracts  C commands, D datarefs, R raw datarefs
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "a380x_guidance.h"
#define ARRAY_LEN(a) ((int)(sizeof(a)/sizeof((a)[0])))

/* ---------------------------------------------------------------- mock XPLM */
typedef struct {
    char name[160];
    int registered, type, writes;
    double value;
    float af[128];
    int ai[128];
    XPLMGetDatai_f get_i; XPLMGetDataf_f get_f; XPLMGetDatad_f get_d;
    void *refcon;
} MockRef;
typedef struct { char name[160]; XPLMCommandCallback_f callback; void *refcon; } MockCommand;
static MockRef refs_mock[96];
static MockCommand commands_mock[16];
static int ref_count, command_count;

static MockRef *new_ref(const char *name, int type)
{
    MockRef *r;
    assert(ref_count < ARRAY_LEN(refs_mock));
    r = &refs_mock[ref_count++];
    memset(r, 0, sizeof(*r));
    snprintf(r->name, sizeof(r->name), "%s", name);
    r->registered = 1; r->type = type;
    return r;
}
static int sim_type(const char *name)
{
    if (strstr(name, "joy_mapped_axis_avail") || strstr(name, "acf_gear_type")) return xplmType_IntArray;
    if (strstr(name, "joy_mapped_axis_value") || strstr(name, "tire_steer") || strstr(name, "gear_xnodef")
        || strstr(name, "gear_znodef")) return xplmType_FloatArray;
    if (strstr(name, "override") || strstr(name, "paused") || strstr(name, "replay") || strstr(name, "onground"))
        return xplmType_Int;
    return xplmType_Float;
}
XPLMDataRef XPLMFindDataRef(const char *name)
{
    int i;
    for (i = 0; i < ref_count; ++i) if (refs_mock[i].registered && !strcmp(refs_mock[i].name, name)) return &refs_mock[i];
    return !strncmp(name, "sim/", 4) ? new_ref(name, sim_type(name)) : NULL;
}
static MockRef *ref(const char *name) { MockRef *r = XPLMFindDataRef(name); assert(r); return r; }
XPLMDataTypeID XPLMGetDataRefTypes(XPLMDataRef h) { return ((MockRef *)h)->type; }
int XPLMGetDatai(XPLMDataRef h) { MockRef *r = h; assert(r->registered); return r->get_i ? r->get_i(r->refcon) : (int)r->value; }
float XPLMGetDataf(XPLMDataRef h) { MockRef *r = h; assert(r->registered); return r->get_f ? r->get_f(r->refcon) : (float)r->value; }
void XPLMSetDatai(XPLMDataRef h, int v) { MockRef *r = h; assert(r->registered && !r->get_i && r->type == xplmType_Int); r->value = v; ++r->writes; }
void XPLMSetDataf(XPLMDataRef h, float v) { MockRef *r = h; assert(r->registered && !r->get_f && r->type == xplmType_Float); r->value = v; ++r->writes; }
int XPLMGetDatavf(XPLMDataRef h, float *v, int o, int n)
{
    MockRef *r = h;
    assert(r->registered && r->type == xplmType_FloatArray);
    if (!v) return 10;
    memcpy(v, r->af + o, (size_t)n * sizeof(*v));
    return n;
}
int XPLMGetDatavi(XPLMDataRef h, int *v, int o, int n)
{
    MockRef *r = h;
    assert(r->registered && r->type == xplmType_IntArray);
    memcpy(v, r->ai + o, (size_t)n * sizeof(*v));
    return n;
}
void XPLMSetDatavf(XPLMDataRef h, float *v, int o, int n)
{
    MockRef *r = h;
    assert(r->registered && r->type == xplmType_FloatArray && o + n <= 128);
    memcpy(r->af + o, v, (size_t)n * sizeof(*v)); ++r->writes;
}
XPLMDataRef XPLMRegisterDataAccessor(const char *name, XPLMDataTypeID type, int writable,
    XPLMGetDatai_f ri, XPLMSetDatai_f wi, XPLMGetDataf_f rf, XPLMSetDataf_f wf, XPLMGetDatad_f rd, XPLMSetDatad_f wd,
    XPLMGetDatavi_f rvi, XPLMSetDatavi_f wvi, XPLMGetDatavf_f rvf, XPLMSetDatavf_f wvf, XPLMGetDatab_f rb,
    XPLMSetDatab_f wb, void *rr, void *wr)
{
    MockRef *r;
    assert(!writable && !wi && !wf && !wd && !rvi && !wvi && !rvf && !wvf && !rb && !wb && !wr);
    assert(type == xplmType_Int ? (ri && !rf && !rd) : (type == (xplmType_Float | xplmType_Double) && rf && rd && !ri));
    assert(!XPLMFindDataRef(name));
    r = new_ref(name, type);
    r->get_i = ri; r->get_f = rf; r->get_d = rd; r->refcon = rr;
    return r;
}
void XPLMUnregisterDataAccessor(XPLMDataRef h) { MockRef *r = h; assert(r->registered); r->registered = 0; }
XPLMCommandRef XPLMCreateCommand(const char *name, const char *description)
{
    MockCommand *c;
    assert(description && *description && command_count < ARRAY_LEN(commands_mock));
    c = &commands_mock[command_count++];
    memset(c, 0, sizeof(*c));
    snprintf(c->name, sizeof(c->name), "%s", name);
    return c;
}
void XPLMRegisterCommandHandler(XPLMCommandRef h, XPLMCommandCallback_f cb, int before, void *refcon)
{ MockCommand *c = h; assert(before && !c->callback); c->callback = cb; c->refcon = refcon; }
void XPLMUnregisterCommandHandler(XPLMCommandRef h, XPLMCommandCallback_f cb, int before, void *refcon)
{ MockCommand *c = h; assert(before && c->callback == cb && c->refcon == refcon); c->callback = NULL; }
static char debug_text[8192];
void XPLMDebugString(const char *s) { size_t n = strlen(debug_text); snprintf(debug_text + n, sizeof(debug_text) - n, "%s", s); }
static MockCommand *command(const char *name)
{
    int i;
    for (i = 0; i < command_count; ++i) if (!strcmp(commands_mock[i].name, name)) return &commands_mock[i];
    assert(!"command");
    return NULL;
}
static void press(const char *name, XPLMCommandPhase phase) { MockCommand *c = command(name); c->callback(c, phase, c->refcon); }

/* ---------------------------------------------------------------- plugin context stand-ins */
static int plugin_enabled = 1;
static A380XGuidanceState guidance;
static A380XGuidanceInputs guidance_inputs;
static struct { int own_pitch, own_roll, own_yaw; } fcu;
typedef struct { int air_valid, attitude_valid, radio_altitude_valid; float radio_altitude_value; } Side;
static struct { Side side[2]; } refs;
static struct { int flap_handle_index; } pfd_aircraft;
static struct { int ready, alpha_floor, alpha_prot; } fctl_link;
static char afs_text[4096];
static void afs_log(const char *format, ...)
{
    size_t n = strlen(afs_text);
    snprintf(afs_text + n, sizeof(afs_text) - n, "%s\n", format);
}

#include "a380x_fctl.inc"

static void tick(int frames) { int i; for (i = 0; i < frames; ++i) fctl_update(0.05f); }
static float steer(int i) { return ref("sim/flightmodel2/gear/tire_steer_actual_deg")->af[i]; }
static int ovr(const char *what) { return (int)ref(what)->value; }
#define OVR_STEER "sim/operation/override/override_wheel_steer"
#define OVR_ART "sim/operation/override/override_artstab"

static void setup_sim(void)
{
    static const float x[5] = {0.0f, -6.17f, 6.17f, -2.63f, 2.63f}, z[5] = {-30.0f, 3.0f, 3.0f, 6.5f, 6.5f};
    int i;
    for (i = 0; i < 5; ++i) {
        ref("sim/aircraft/parts/acf_gear_xnodef")->af[i] = x[i];
        ref("sim/aircraft/parts/acf_gear_znodef")->af[i] = z[i];
        ref("sim/aircraft/parts/acf_gear_type")->ai[i] = i == 0 ? 3 : 5;
    }
    ref("sim/aircraft/controls/acf_hstb_trim_up")->value = 2.0;
    ref("sim/aircraft/controls/acf_hstb_trim_dn")->value = 12.0;
    ref("sim/aircraft/view/acf_Vne")->value = 340; ref("sim/aircraft/view/acf_Mmo")->value = 0.89;
    ref("sim/flightmodel/failures/onground_any")->value = 1;
    ref("sim/flightmodel/forces/g_nrml")->value = 1;
    guidance_inputs.powered = 1;
    refs.side[0].air_valid = refs.side[0].attitude_valid = refs.side[0].radio_altitude_valid = 1;
    refs.side[0].radio_altitude_value = 0;
}

static void test_registry_and_gears(void)
{
    int i;
    for (i = 0; i < FCTLD_COUNT; ++i) assert(ref(fctl_dataref_names[i]));
    for (i = 0; i < FCTLC_COUNT; ++i) assert(command(fctl_command_names[i]));
    assert(XPLMGetDatai(ref("a380x/xp12/fbw/enabled")) == 1);
    assert(fctl.gears_known && fctl.nose == 0 && fctl.body_count == 2 && fctl.body[0] == 3 && fctl.body[1] == 4);
}

static void test_steering(void)
{
    ref("sim/flightmodel/position/groundspeed")->value = 10 / 1.9438444924;
    ref("sim/joystick/joy_mapped_axis_avail")->ai[37] = 1;
    ref("sim/joystick/joy_mapped_axis_value")->af[37] = 1.0f;
    tick(100);
    assert(ovr(OVR_STEER) == 1 && fctl.own_steer);
    assert(fabsf(steer(0) - 70.0f) < 1e-3f && fabsf(steer(3) + 15.0f) < 1e-3f && fabsf(steer(4) + 15.0f) < 1e-3f);
    assert(steer(1) == 0.0f && steer(2) == 0.0f);
    assert(fabsf(XPLMGetDataf(ref("a380x/xp12/steering/nose_deg")) - 70.0f) < 1e-3f);
    /* keyboard tiller, no hardware */
    ref("sim/joystick/joy_mapped_axis_avail")->ai[37] = 0;
    press("a380x/xp12/steering/tiller_left", xplm_CommandBegin); tick(20);
    assert(fctl.tiller_key_value < -0.99);                    /* 1.5/s to full */
    tick(140);                                                /* the nose gear slews at 20°/s */
    assert(fabsf(steer(0) + 70.0f) < 1e-3f && fabsf(steer(3) - 15.0f) < 1e-3f);
    press("a380x/xp12/steering/tiller_left", xplm_CommandEnd); tick(20);
    assert(fabs(fctl.tiller_key_value) < 1e-9);
    /* BetterPushback connected: stand down */
    {
        MockRef *bp = new_ref("bp/connected", xplmType_Int);
        bp->value = 1; tick(1);
        assert(ovr(OVR_STEER) == 0 && !fctl.own_steer);
        bp->value = 0; tick(1);
        assert(ovr(OVR_STEER) == 1 && fctl.own_steer);
    }
    /* another plugin owns the steering override: never taken over */
    fctl_release_steering(); ref(OVR_STEER)->value = 1; tick(5);
    assert(!fctl.own_steer);
    ref(OVR_STEER)->value = 0; tick(1);
    assert(fctl.own_steer);
    press("a380x/xp12/steering/toggle", xplm_CommandBegin); tick(1);
    assert(ovr(OVR_STEER) == 0 && !fctl.own_steer && strstr(afs_text, "steering"));
    press("a380x/xp12/steering/toggle", xplm_CommandBegin); tick(1);
    assert(fctl.own_steer);
}

static void test_flight_law_channel(void)
{
    /* ground mode: the artstab channel is owned but carries no correction */
    tick(2);
    assert(ovr(OVR_ART) == 1 && XPLMGetDataf(ref("sim/joystick/artstab_pitch_ratio")) == 0.0f);
    /* airborne, 250 kt: the law corrects the pilot's stick through artstab */
    ref("sim/flightmodel/failures/onground_any")->value = 0;
    ref("sim/flightmodel/position/indicated_airspeed")->value = 250;
    ref("sim/flightmodel/position/true_airspeed")->value = 250 / 1.9438444924;
    ref("sim/flightmodel/position/groundspeed")->value = 250 / 1.9438444924;
    refs.side[0].radio_altitude_value = 5000;
    ref("sim/joystick/yoke_pitch_ratio")->value = 0.5;
    tick(150);
    assert(XPLMGetDatai(ref("a380x/xp12/fbw/mode")) == A380X_FCTL_FLIGHT && XPLMGetDatai(ref("a380x/xp12/fbw/normal_law")) == 1);
    assert(fabsf(XPLMGetDataf(ref("sim/joystick/artstab_pitch_ratio"))) > 0.05f);
    assert(fabs(fctl.out.pitch - (0.5 + XPLMGetDataf(ref("sim/joystick/artstab_pitch_ratio")))) < 1e-6);
    assert(steer(0) == 0.0f);                                   /* nose wheel centred in flight */
    /* the AP flies: no pitch/roll correction */
    guidance.ap1 = 1; fcu.own_pitch = fcu.own_roll = 1; guidance.pitch_servo = 0.1;
    tick(1);
    assert(XPLMGetDataf(ref("sim/joystick/artstab_pitch_ratio")) == 0.0f && XPLMGetDataf(ref("sim/joystick/artstab_roll_ratio")) == 0.0f);
    guidance.ap1 = 0; fcu.own_pitch = fcu.own_roll = 0;
    ref("sim/joystick/yoke_pitch_ratio")->value = 0;
    /* auto-trim: nose-up elevator unloads into nose-up trim within the 2° up range */
    {
        float before = XPLMGetDataf(ref("sim/cockpit2/controls/elevator_trim"));
        fctl.core.pitch_int = 0.1; tick(1);
        assert(XPLMGetDatai(ref("a380x/xp12/fbw/auto_trim")) == 1);
        assert(XPLMGetDataf(ref("sim/cockpit2/controls/elevator_trim")) > before);
    }
    /* high AoA: protection and the A.FLOOR link */
    ref("sim/flightmodel/position/indicated_airspeed")->value = 150;
    ref("sim/flightmodel/misc/machno")->value = 0.25;
    ref("sim/flightmodel/position/alpha")->value = 13;
    tick(40);
    assert(fctl_link.ready && fctl_link.alpha_floor && fctl_link.alpha_prot);
    assert(XPLMGetDatai(ref("a380x/xp12/fbw/protections")) & A380X_FCTL_PROT_ALPHA);
    assert(strstr(afs_text, "angle-of-attack"));
    ref("sim/flightmodel/position/alpha")->value = 3;
    ref("sim/flightmodel/position/indicated_airspeed")->value = 250;
    ref("sim/joystick/yoke_pitch_ratio")->value = -0.8; tick(5);
    ref("sim/joystick/yoke_pitch_ratio")->value = 0; tick(20);
    assert(!fctl_link.alpha_prot);
    /* IR lost: direct law, channel released */
    refs.side[0].attitude_valid = 0; tick(1);
    assert(XPLMGetDatai(ref("a380x/xp12/fbw/law")) == A380X_FCTL_LAW_DIRECT && ovr(OVR_ART) == 0
           && XPLMGetDatai(ref("a380x/xp12/fbw/normal_law")) == 0);
    refs.side[0].attitude_valid = 1; tick(1);
    assert(ovr(OVR_ART) == 1);
    /* a foreign artstab owner is never overridden */
    fctl_release_artstab(); ref(OVR_ART)->value = 1; tick(3);
    assert(!fctl.own_artstab);
    ref(OVR_ART)->value = 0; tick(1);
    assert(fctl.own_artstab);
    /* crew toggle */
    press("a380x/xp12/fbw/toggle", xplm_CommandBegin); tick(1);
    assert(ovr(OVR_ART) == 0 && XPLMGetDatai(ref("a380x/xp12/fbw/law")) == A380X_FCTL_LAW_OFF);
    press("a380x/xp12/fbw/toggle", xplm_CommandBegin); tick(1);
    assert(ovr(OVR_ART) == 1);
}

static void test_replay_disable_shutdown(void)
{
    ref("sim/time/is_in_replay")->value = 1; tick(1);
    assert(ovr(OVR_ART) == 0 && ovr(OVR_STEER) == 0);
    ref("sim/time/is_in_replay")->value = 0; tick(1);
    assert(ovr(OVR_ART) == 1);
    fctl_release_controls();
    assert(ovr(OVR_ART) == 0 && ovr(OVR_STEER) == 0 && !fctl_link.alpha_floor);
    fctl_plane_loaded(); assert(fctl.gears_known && fctl.nose == 0);
    tick(1);
    fctl_shutdown();
    assert(ovr(OVR_ART) == 0 && ovr(OVR_STEER) == 0 && !XPLMFindDataRef("a380x/xp12/fbw/enabled") && !fctl.ready);
}

int main(int argc, char **argv)
{
    int i;
    setup_sim();
    assert(fctl_init());
    if (argc > 1 && !strcmp(argv[1], "--contracts")) {
        for (i = 0; i < FCTLC_COUNT; ++i) printf("C %s\n", fctl_command_names[i]);
        for (i = 0; i < FCTLD_COUNT; ++i) printf("D %s\n", fctl_dataref_names[i]);
        for (i = 0; i < FCTLR_COUNT; ++i) printf("R %s\n", fctl_raw_names[i]);
        fctl_shutdown();
        return 0;
    }
    test_registry_and_gears();
    test_steering();
    test_flight_law_channel();
    test_replay_disable_shutdown();
    puts("Flight-control adapter: registry, nose/body gear identification, tiller + pedal steering, keyboard tiller, "
         "BetterPushback / foreign-override stand-down, artstab law channel, AP bypass, auto-trim, A.FLOOR link, "
         "direct law, toggles, replay, disable and shutdown passed");
    return 0;
}
