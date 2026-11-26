/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ENG START selector (CRANK / NORM / IGN START) and the engine start it commands, following the A380 FADEC
 * start logic (FBW A380X: L:A32NX_ENGINE_STATE / ENG START selector, overhead MAN START pushbuttons):
 *
 *   IGN START + ENG MASTER ON           automatic start: after FBW's 1.7 s delay the starter engages with
 *                                       ignition until the engine reaches idle (FBW EngineControl_A380X: the
 *                                       starter valve is open while STARTING, until N3 >= idle N3 - 0.1).
 *   IGN START + MAN START, MASTER OFF   motoring (starter, no ignition); MASTER ON then lights it.
 *   CRANK + MAN START                   dry crank: starter only, never ignition.
 *   NORM                                no starter; a start in progress is aborted.
 *   IGN START with an engine running    continuous ignition.
 *   A start that has not reached idle after 120 s is aborted until MASTER or the selector is cycled (port
 *   safety; FBW has no abort). Idle = running, N2 >= 50 % and N2 settled (|rate| < 0.1 percentage point/s for 1 sim second): X-Plane's idle N2
 *   depends on the flight model, so FBW's idle N3 table is not used. Each start logs simulated MASTER ON -> idle time.
 *   The starters are air turbines: they turn only with bleed air, from the APU (APU AVAIL, APU BLEED ON and native pneumatic air established)
 *   or from an engine already running (crossbleed). With APU bleed selected, the port queues starter requests
 *   one at a time: the native duct could not sustain the measured paired start.
 *
 * The selector state is the cockpit-feedback selector a380x/xp12/eng_start/selector (0 CRANK, 1 NORM, 2 IGN
 * START; tools/cockpit_feedback_contract.py) and MAN START is the overhead pushbutton position
 * a380x/xp12/ovhd/pb/man_start<n>. X-Plane does the rest: sim/starters/engage_starter_<n> held while the
 * starter runs, sim/cockpit2/engine/actuators/igniter_on, fuel by ENG MASTER (mixture_ratio).
 * The flyPad READY FOR TAXI preset starts the engines the same way (selector IGN START, then ENG MASTER ON).
 * Unity-build include of a380x_avionics.c after a380x_cockpit_feedback.inc and the ELEC adapter.
 */
#define ENGS_CUTOUT_N2 50.0f
#define ENGS_TIMEOUT_S 120.0f
#define ENGS_MASTER_DELAY_S 1.7f   /* FBW engineStartProcedure: master-switch to start delay */
#define ENGS_IDLE_RATE 0.1f        /* N2 percentage points per simulated second */
#define ENGS_IDLE_DWELL_S 1.0f
#define ENGS_MAX_STEP_S 1.0f       /* longer jumps are not contiguous samples */
#define ENGS_APU_N_READY 99.5f     /* XP's native APU delivers rated bleed around 100% N1 */
#define ENGS_DUCT_PRESENT 0.05f    /* distinguish some actual native flow from an empty duct */
enum { ENGS_CRANK, ENGS_NORM, ENGS_IGN_START };
enum { ENGS_N2, ENGS_RUNNING, ENGS_MIXTURE, ENGS_IGNITER, ENGS_SIM_TIME, ENGS_PAUSED, ENGS_REPLAY, ENGS_APU_N1, ENGS_DUCT, ENGS_REFS };

static struct {
    int initialized, selector_index, man_start_pb[4];
    int starter_on[4], igniter_set[4], aborted[4], last_master[4], last_selector, no_air_logged;
    int at_idle[4], primed, clock_valid;
    float start_time[4], master_time[4], prev_n2[4], idle_dwell[4], now, last_sim;
    XPLMCommandRef starter[4];
    XPLMDataRef refs[ENGS_REFS];
} engs;

static int engs_selector(void)
{
    return engs.selector_index >= 0 ? cfb.position[engs.selector_index].position : ENGS_NORM;
}

static void engs_starter(int i, int on)
{
    if (on == engs.starter_on[i] || !engs.starter[i]) return;
    if (on) XPLMCommandBegin(engs.starter[i]); else XPLMCommandEnd(engs.starter[i]);
    engs.starter_on[i] = on;
}

static void engs_release(void)
{
    int off = 0;
    for (int i = 0; i < 4; ++i) {
        engs_starter(i, 0);
        if (engs.igniter_set[i] && engs.refs[ENGS_IGNITER])
            XPLMSetDatavi(engs.refs[ENGS_IGNITER], &off, i, 1);
        engs.igniter_set[i] = engs.aborted[i] = engs.last_master[i] = engs.at_idle[i] = 0;
        engs.idle_dwell[i] = 0;
    }
    engs.primed = engs.clock_valid = engs.no_air_logged = 0;
    engs.last_selector = ENGS_NORM;
    engs.now = 0;
}

static int engs_init(void)
{
    static const char *const names[ENGS_REFS] = {
        "sim/cockpit2/engine/indicators/N2_percent", "sim/flightmodel/engine/ENGN_running",
        "sim/cockpit2/engine/actuators/mixture_ratio", "sim/cockpit2/engine/actuators/igniter_on",
        "sim/time/total_flight_time_sec", "sim/time/paused", "sim/time/is_in_replay",
        "sim/cockpit2/electrical/APU_N1_percent", "sim/cockpit2/bleedair/indicators/bleed_available_center"};
    char name[64];
    memset(&engs, 0, sizeof(engs));
    for (int r = 0; r < ENGS_REFS; ++r) engs.refs[r] = XPLMFindDataRef(names[r]);
    for (int i = 0; i < 4; ++i) {
        snprintf(name, sizeof(name), "sim/starters/engage_starter_%d", i + 1);
        engs.starter[i] = XPLMFindCommand(name);
        snprintf(name, sizeof(name), "man_start%d", i + 1);
        engs.man_start_pb[i] = a380x_ovhd_find(name);
    }
    engs.selector_index = cfb_index("eng_start/selector");
    engs.last_selector = ENGS_NORM;
    engs.initialized = engs.selector_index >= 0;
    for (int r = 0; r < ENGS_REFS; ++r) if (!engs.refs[r]) engs.initialized = 0;
    XPLMDebugString(engs.initialized ? "A380X ENG START: CRANK / NORM / IGN START selector and FADEC start ready\n"
                                     : "A380X ENG START: selector not registered; engine start stays manual\n");
    return engs.initialized;
}

static void engs_update(void)
{
    float n2[4], mixture[4], sim, dt = 0;
    int running[4], igniter[4], sel;
    if (!engs.initialized) return;
    if (!plugin_enabled || XPLMGetDatai(engs.refs[ENGS_REPLAY])) { engs_release(); return; }
    sim = XPLMGetDataf(engs.refs[ENGS_SIM_TIME]);
    if (XPLMGetDatai(engs.refs[ENGS_PAUSED]) || !isfinite(sim) || sim < 0) return;
    if (engs.clock_valid) {
        float step = sim - engs.last_sim;
        if (step == 0) return;
        if (step < 0) { engs_release(); return; }
        if (step <= ENGS_MAX_STEP_S) dt = step;
    }
    engs.clock_valid = 1;
    engs.last_sim = sim;
    engs.now += dt;
    float now = engs.now;
    sel = engs_selector();
    if (XPLMGetDatavf(engs.refs[ENGS_N2], n2, 0, 4) != 4 ||
        XPLMGetDatavf(engs.refs[ENGS_MIXTURE], mixture, 0, 4) != 4 ||
        XPLMGetDatavi(engs.refs[ENGS_RUNNING], running, 0, 4) != 4 ||
        XPLMGetDatavi(engs.refs[ENGS_IGNITER], igniter, 0, 4) != 4) { engs_release(); return; }
    /* FBW's APU AVAIL and native XP APU N1 are independent. A cold trace can have the
     * annunciator live with native N1 near 65%; asking for both starters then can
     * collapse the native duct. Check the duct on entry only (under-load pulses do not
     * override the physical starter); keep requiring APU speed and the valve once engaged. */
    float apu_n1 = XPLMGetDataf(engs.refs[ENGS_APU_N1]);
    float duct = XPLMGetDataf(engs.refs[ENGS_DUCT]);
    int apu_selected = ovhd_in.apu_avail && ovhd_in.apu_bleed;
    int apu_air = apu_selected && isfinite(apu_n1) && apu_n1 >= ENGS_APU_N_READY;
    int apu_starter = -1;
    if (apu_selected)
        for (int i = 0; i < 4; ++i)
            if (engs.starter_on[i]) { apu_starter = i; break; }
    int duct_ready = isfinite(duct) && duct >= ENGS_DUCT_PRESENT;
    unsigned engine_air = 0;
    for (int i = 0; i < 4; ++i)
        if (running[i] && isfinite(n2[i]) && n2[i] >= ENGS_CUTOUT_N2) engine_air |= 1u << i;
    for (int i = 0; i < 4; ++i) {
        int master = isfinite(mixture[i]) && mixture[i] > 0.5f;
        int man = engs.man_start_pb[i] >= 0 && ovhd_in.pb[engs.man_start_pb[i]];
        int lit = (engine_air & (1u << i)) != 0;
        int air = (apu_air && (duct_ready || engs.starter_on[i])) || (engine_air & ~(1u << i));
        int settled = dt > 0 && isfinite(engs.prev_n2[i]) && isfinite(n2[i]) &&
            fabsf(n2[i] - engs.prev_n2[i]) < ENGS_IDLE_RATE * dt;
        engs.prev_n2[i] = n2[i];
        if (!lit) engs.at_idle[i] = 0;
        if (lit && settled) engs.idle_dwell[i] += dt;
        else engs.idle_dwell[i] = 0;
        if (lit && !engs.at_idle[i] && (!engs.primed || engs.idle_dwell[i] >= ENGS_IDLE_DWELL_S)) {
            char line[140];
            engs.at_idle[i] = 1;
            if (engs.starter_on[i]) {
                snprintf(line, sizeof(line), "A380X ENG START: engine %d at idle (N2 %.1f %%) %.1f sim s after MASTER ON\n",
                         i + 1, n2[i], now - engs.master_time[i]);
                XPLMDebugString(line);
            }
        }
        int started = engs.at_idle[i];
        int want_starter = 0, want_igniter = 0;
        if (master != engs.last_master[i] || sel != engs.last_selector) engs.aborted[i] = 0;   /* cycled: retry */
        if (master && !engs.last_master[i]) engs.master_time[i] = now;
        engs.last_master[i] = master;
        if (sel == ENGS_IGN_START) {
            if (started) want_igniter = 1;                                   /* continuous ignition */
            else if (master) {                                               /* automatic (or manual) start */
                want_starter = want_igniter = now - engs.master_time[i] >= ENGS_MASTER_DELAY_S || man;
            }
            else if (man) want_starter = 1;                                  /* motoring before fuel */
        } else if (sel == ENGS_CRANK && man && !started) {
            want_starter = 1;                                                /* dry crank: no ignition */
        }
        if (want_starter && !air) {                                         /* no bleed air: no rotation */
            if (!engs.no_air_logged) XPLMDebugString("A380X ENG START: no bleed air - APU BLEED ON with the APU "
                                                     "available and native air ready, or another running engine, is needed to start\n");
            engs.no_air_logged = 1;
            want_starter = 0;
            if (!started) want_igniter = 0;
        }
        if (want_starter && engs.starter_on[i] && now - engs.start_time[i] > ENGS_TIMEOUT_S) engs.aborted[i] = 1;
        if (engs.aborted[i]) want_starter = want_igniter = 0;
        /* A fully spun-up native APU supplied a single starter but not two on the
         * matched cold test. Keep ownership with the current starter until it
         * releases. A queued request cannot spend its engagement timeout yet. */
        if (apu_selected && want_starter) {
            if (apu_starter < 0) apu_starter = i;
            else if (apu_starter != i) want_starter = want_igniter = 0;
        }
        if (want_starter && !engs.starter_on[i]) engs.start_time[i] = now;
        engs_starter(i, want_starter);
        if (want_igniter && !igniter[i]) {
            int on = 1;
            XPLMSetDatavi(engs.refs[ENGS_IGNITER], &on, i, 1);
            engs.igniter_set[i] = 1;
        } else if (!want_igniter && engs.igniter_set[i]) {
            int off = 0;
            XPLMSetDatavi(engs.refs[ENGS_IGNITER], &off, i, 1);
            engs.igniter_set[i] = 0;
        }
    }
    engs.primed = 1;
    engs.last_selector = sel;
}

/* ENG MASTER FAULT light (a380x_cockpit_feedback.inc): the start was aborted (timeout). */
static int engs_start_fault(int engine)
{
    return engs.initialized && engine >= 0 && engine < 4 && engs.aborted[engine];
}

static void engs_shutdown(void) { if (engs.initialized) engs_release(); memset(&engs, 0, sizeof(engs)); }
