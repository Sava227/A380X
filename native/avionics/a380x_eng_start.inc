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
 *   safety; FBW has no abort). Idle = running, N2 >= 50 % and N2 settled (< 0.1 %/s): X-Plane's idle N2
 *   depends on the flight model, so FBW's idle N3 table is not used. Each start logs MASTER ON -> idle time.
 *   The starters are air turbines: they turn only with bleed air, from the APU (APU AVAIL and APU BLEED ON)
 *   or from an engine already running (crossbleed).
 *
 * The selector state is the cockpit-feedback selector a380x/xp12/eng_start/selector (0 CRANK, 1 NORM, 2 IGN
 * START; tools/cockpit_feedback_contract.py) and MAN START is the overhead pushbutton position
 * a380x/xp12/ovhd/pb/man_start<n>. X-Plane does the rest: sim/starters/engage_starter_<n> held while the
 * starter runs, sim/cockpit2/engine/actuators/igniter_on, fuel by ENG MASTER (mixture_ratio).
 * Engines being started by a flyPad preset (eln.starting) are left to the preset.
 * Unity-build include of a380x_avionics.c after a380x_cockpit_feedback.inc and the ELEC adapter.
 */
#define ENGS_CUTOUT_N2 50.0f
#define ENGS_TIMEOUT_S 120.0f
#define ENGS_MASTER_DELAY_S 1.7f   /* FBW engineStartProcedure: master-switch to start delay */
#define ENGS_IDLE_RATE 0.1f        /* N2 %/s: settled at idle */
enum { ENGS_CRANK, ENGS_NORM, ENGS_IGN_START };
enum { ENGS_N2, ENGS_RUNNING, ENGS_MIXTURE, ENGS_IGNITER, ENGS_REFS };

static struct {
    int initialized, selector_index, man_start_pb[4];
    int starter_on[4], igniter_set[4], aborted[4], last_master[4], last_selector, no_air_logged;
    int at_idle[4];
    float start_time[4], master_time[4], prev_n2[4], now;
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
    int off[4] = {0, 0, 0, 0}, any = 0;
    for (int i = 0; i < 4; ++i) { engs_starter(i, 0); any |= engs.igniter_set[i]; engs.igniter_set[i] = 0; }
    if (any && engs.refs[ENGS_IGNITER]) XPLMSetDatavi(engs.refs[ENGS_IGNITER], off, 0, 4);
}

static int engs_init(void)
{
    static const char *const names[ENGS_REFS] = {
        "sim/cockpit2/engine/indicators/N2_percent", "sim/flightmodel/engine/ENGN_running",
        "sim/cockpit2/engine/actuators/mixture_ratio", "sim/cockpit2/engine/actuators/igniter_on"};
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
    engs.now = XPLMGetElapsedTime();
    engs.initialized = engs.selector_index >= 0;
    XPLMDebugString(engs.initialized ? "A380X ENG START: CRANK / NORM / IGN START selector and FADEC start ready\n"
                                     : "A380X ENG START: selector not registered; engine start stays manual\n");
    return engs.initialized;
}

static void engs_update(void)
{
    float n2[4] = {0}, mixture[4] = {0}, now, dt;
    int running[4] = {0}, igniter[8] = {0}, write_igniter = 0, sel;
    if (!engs.initialized) return;
    now = XPLMGetElapsedTime();
    dt = now - engs.now;
    engs.now = now;
    if (!plugin_enabled) { engs_release(); return; }
    sel = engs_selector();
    if (engs.refs[ENGS_N2]) XPLMGetDatavf(engs.refs[ENGS_N2], n2, 0, 4);
    if (engs.refs[ENGS_MIXTURE]) XPLMGetDatavf(engs.refs[ENGS_MIXTURE], mixture, 0, 4);
    if (engs.refs[ENGS_RUNNING]) XPLMGetDatavi(engs.refs[ENGS_RUNNING], running, 0, 4);
    if (engs.refs[ENGS_IGNITER]) XPLMGetDatavi(engs.refs[ENGS_IGNITER], igniter, 0, 8);
    int air = ovhd_in.apu_avail && ovhd_in.apu_bleed;
    for (int i = 0; i < 4; ++i) air |= running[i] && isfinite(n2[i]) && n2[i] >= ENGS_CUTOUT_N2;
    for (int i = 0; i < 4; ++i) {
        int master = isfinite(mixture[i]) && mixture[i] > 0.5f;
        int man = engs.man_start_pb[i] >= 0 && ovhd_in.pb[engs.man_start_pb[i]];
        int lit = running[i] && isfinite(n2[i]) && n2[i] >= ENGS_CUTOUT_N2;
        int settled = dt > 0.0f && (n2[i] - engs.prev_n2[i]) / dt < ENGS_IDLE_RATE;
        engs.prev_n2[i] = isfinite(n2[i]) ? n2[i] : 0.0f;
        if (!lit) engs.at_idle[i] = 0;
        else if (!engs.at_idle[i] && settled) {
            char line[128];
            engs.at_idle[i] = 1;
            if (engs.starter_on[i]) {
                snprintf(line, sizeof(line), "A380X ENG START: engine %d at idle (N2 %.1f %%) %.1f s after MASTER ON\n",
                         i + 1, n2[i], now - engs.master_time[i]);
                XPLMDebugString(line);
            }
        }
        int started = engs.at_idle[i];
        int want_starter = 0, want_igniter = 0;
        if (eln.starting[i]) { engs.starter_on[i] = 0; continue; }       /* the flyPad preset owns this one */
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
                                                     "available, or a running engine, is needed to start\n");
            engs.no_air_logged = 1;
            want_starter = 0;
            if (!started) want_igniter = 0;
        }
        if (want_starter && !engs.starter_on[i]) engs.start_time[i] = now;
        if (want_starter && now - engs.start_time[i] > ENGS_TIMEOUT_S) engs.aborted[i] = 1;
        if (engs.aborted[i]) want_starter = want_igniter = 0;
        engs_starter(i, want_starter);
        if (want_igniter != (igniter[i] != 0) && (want_igniter || engs.igniter_set[i])) {
            igniter[i] = want_igniter; write_igniter = 1;
        }
        engs.igniter_set[i] = want_igniter;
    }
    engs.last_selector = sel;
    if (write_igniter && engs.refs[ENGS_IGNITER]) XPLMSetDatavi(engs.refs[ENGS_IGNITER], igniter, 0, 4);
}

/* ENG MASTER FAULT light (a380x_cockpit_feedback.inc): the start was aborted (timeout). */
static int engs_start_fault(int engine)
{
    return engs.initialized && engine >= 0 && engine < 4 && engs.aborted[engine];
}

static void engs_shutdown(void) { if (engs.initialized) engs_release(); memset(&engs, 0, sizeof(engs)); }
