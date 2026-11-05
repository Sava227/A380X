/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Native state, page actions and the tablet JSON frame of the web EFB (docs/EFB_DEVELOPMENT.md).
 * Included by a380x_efb.c after the legacy flyPad functions it reuses (payload/fuel loading, presets,
 * checklists, failures, radios). The WebKit page is presentation and touch only: every value below
 * comes from X-Plane or the native A380 modules, every action is validated here, and anything
 * without a backing model is sent as null (the page shows it as unavailable).
 */

#define EFB_TOUCH_EVENTS 32
#define EFB_SOUNDS 10
#define EFB_FAILURE_COUNT 17

typedef struct { unsigned seq; int type, x, y; } EfbTouchEvent;   /* type 1 down, 0 move, 2 up, 3 cancel */
typedef struct {
    int section, tab[A380X_EFB_SECTIONS], group;
    unsigned long ack;
    unsigned touch_serial;
    EfbTouchEvent events[EFB_TOUCH_EVENTS];
    int last_x, last_y, pressed, draws, web_draws, fallback_draws, bound, web_live, blank_drawn;
    void *device;
    XPLMCommandRef popup_command;
} EfbTablet;

static EfbTablet tablets[2];
static A380XEfbLoad load_plan;
static int load_plan_set;          /* plan typed or applied in this session */
static float load_applied_kg = -1; /* m_fixed written by the last EFB apply, -1 none */
static int fuel_target_kg = -1;
static int refuel_active, refuel_rate = 2; /* 0 real, 1 fast, 2 instant */
static int push_steer = 1;                 /* 0 nose left, 1 straight, 2 nose right */
static int option_engine, option_brake_fans, option_hud = 1;   /* Phase 3: stored, not applied */
static int cabin_layout[3][5] = {{0, 0, 56, 338, 0}, {14, 76, 0, 0, 0}, {0, 0, 0, 0, 0}}; /* main, upper, lower: F J W Y fill */
static int hold_pallets[2];
static unsigned efb_change_serial = 1, efb_epoch;
static unsigned long efb_actions_processed, efb_actions_rejected, efb_touch_count;
static int efb_preset_seen_done = -1, efb_preset_seen_active = -1;
static float efb_sound_load[EFB_SOUNDS];
static int efb_sound_load_valid;
static XPLMDataRef efb_sound_refs[EFB_SOUNDS];
static XPLMCommandRef door_intercepts[A380X_EFB_SLIDES];
static const char *const efb_sound_ids[EFB_SOUNDS] = {
    "master", "engines", "exterior", "interior", "ui", "environment", "weather", "ground", "warnings", "radios"
};
static const char *const efb_sound_refs_names[EFB_SOUNDS] = {
    "sim/operation/sound/master_volume_ratio", "sim/operation/sound/engine_volume_ratio",
    "sim/operation/sound/exterior_volume_ratio", "sim/operation/sound/interior_volume_ratio",
    "sim/operation/sound/ui_volume_ratio", "sim/operation/sound/enviro_volume_ratio",
    "sim/operation/sound/weather_volume_ratio", "sim/operation/sound/ground_volume_ratio",
    "sim/operation/sound/warning_volume_ratio", "sim/operation/sound/radio_volume_ratio"
};
enum { EX_LOCAL_TIME, EX_COM2, EX_COM2_STBY, EX_CHOCKS, EX_PUSHBACK, EX_COUNT };
static const char *const efb_extra_names[EX_COUNT] = {
    "sim/time/local_time_sec", "sim/cockpit2/radios/actuators/com2_frequency_hz_833",
    "sim/cockpit2/radios/actuators/com2_standby_frequency_hz_833", "sim/flightmodel2/gear/is_chocked",
    "sim/aircraft/overflow/pushback_attached"
};
static XPLMDataRef efb_extra_refs[EX_COUNT];
static const struct { const char *system, *ata; } failure_systems[EFB_FAILURE_COUNT] = {
    {"ENG", "72"}, {"ENG", "72"}, {"ENG", "72"}, {"ENG", "72"},
    {"ELEC", "24"}, {"ELEC", "24"}, {"ELEC", "24"}, {"ELEC", "24"}, {"ELEC", "24"}, {"ELEC", "24"},
    {"F/CTL", "27"}, {"L/G", "32"}, {"HYD", "29"}, {"HYD", "29"}, {"APU", "49"}, {"NAV", "34"}, {"NAV", "34"}
};
/* Approximate refuel rates (not Airbus figures): real ~3 t/min, fast 5x; instant = the old tablet. */
static const float refuel_kg_per_s[2] = {50.0f, 250.0f};

static void efb_changed(void) { if (++efb_change_serial == 0) efb_change_serial = 1; }

static float efb_door_ratio(int door)
{
    float values[A380X_EFB_DOORS] = {0};
    if (refs.doors == NULL || door < 0 || door >= A380X_EFB_DOORS) return 0.0f;
    if (XPLMGetDatavf(refs.doors, values, 0, A380X_EFB_DOORS) < door + 1) return 0.0f;
    return isfinite(values[door]) ? values[door] : 0.0f;
}

static int efb_read_doors(float out[A380X_EFB_DOORS])
{
    int i, n;
    memset(out, 0, sizeof(float) * A380X_EFB_DOORS);
    if (refs.doors == NULL) return 0;
    n = XPLMGetDatavf(refs.doors, out, 0, A380X_EFB_DOORS);
    for (i = 0; i < A380X_EFB_DOORS; ++i) if (!isfinite(out[i])) out[i] = 0.0f;
    return n;
}

static int efb_tablet_blank(void)
{
    return !powered || (battery_simulation && battery_level <= 0.0 && !electrical_power_available());
}

/* ---- Slides -------------------------------------------------------------------------------------- */
static void efb_slides_set_all(int arm)
{
    float doors[A380X_EFB_DOORS];
    int left;
    efb_read_doors(doors);
    if (!arm) {
        memset(slide_armed, 0, sizeof(slide_armed));
        set_status("All 16 escape slides disarmed");
    } else {
        left = a380x_efb_arm_all(doors, slide_armed);
        if (left) set_status("%d slides armed; %d open doors left disarmed (close them first)", 16 - left, left);
        else set_status("All 16 escape slides armed");
    }
    efb_changed();
}

static void efb_slide_set(int door, int arm)
{
    const char *name = a380x_efb_door_name(door);
    switch (a380x_efb_slide_rule(door, efb_door_ratio(door), arm)) {
    case A380X_EFB_SLIDE_OK:
        slide_armed[door] = (unsigned char)(arm != 0);
        set_status("%s slide %s", name, arm ? "armed" : "disarmed");
        break;
    case A380X_EFB_SLIDE_NO_SLIDE: set_status("%s has no escape slide", name ? name : "Door"); break;
    case A380X_EFB_SLIDE_DOOR_OPEN: set_status("Close %s before arming its slide", name); break;
    default: set_status("Unknown door"); break;
    }
    efb_changed();
}

/* After an aircraft-state preset: parked states disarm, departure states arm every closed door. */
static void efb_preset_slides(int preset)
{
    if (preset <= 0) return;
    if (preset <= 2) memset(slide_armed, 0, sizeof(slide_armed));
    else {
        float doors[A380X_EFB_DOORS];
        efb_read_doors(doors);
        a380x_efb_arm_all(doors, slide_armed);
    }
    efb_changed();
}

/* An armed passenger door cannot be opened, whichever path asks for it. */
static int efb_door_intercept(XPLMCommandRef command, XPLMCommandPhase phase, void *refcon)
{
    int door = (int)(intptr_t)refcon;
    (void)command;
    if (a380x_efb_door_toggle_rule(door, efb_door_ratio(door), slide_armed[door]) == A380X_EFB_DOOR_ARMED) {
        if (phase == xplm_CommandBegin) {
            set_status("Disarm the %s slide before opening the door", a380x_efb_door_name(door));
            efb_changed();
        }
        return 0;
    }
    return 1;
}

static void efb_door_toggle(int door)
{
    char name[64];
    const char *label = a380x_efb_door_name(door);
    if (label == NULL) return;
    if (!on_ground()) { set_status("Door controls are ground-only"); efb_changed(); return; }
    if (a380x_efb_door_toggle_rule(door, efb_door_ratio(door), slide_armed[door]) == A380X_EFB_DOOR_ARMED) {
        set_status("Disarm the %s slide before opening the door", label);
        efb_changed();
        return;
    }
    snprintf(name, sizeof(name), "sim/flight_controls/door_toggle_%d", door + 1);
    run_command(name);
    set_status("%s door %s", label, efb_door_ratio(door) > A380X_EFB_DOOR_CLOSED_RATIO ? "closing" : "opening");
    efb_changed();
}

static void efb_doors_all(int open)
{
    float doors[A380X_EFB_DOORS];
    int i, moved = 0, kept = 0;
    char name[64];
    if (!on_ground()) { set_status("Door controls are ground-only"); efb_changed(); return; }
    efb_read_doors(doors);
    for (i = 0; i < A380X_EFB_DOORS; ++i) {
        int is_open = doors[i] > A380X_EFB_DOOR_CLOSED_RATIO;
        if (open == is_open) continue;
        if (open && a380x_efb_door_has_slide(i) && slide_armed[i]) { ++kept; continue; }
        snprintf(name, sizeof(name), "sim/flight_controls/door_toggle_%d", i + 1);
        run_command(name);
        ++moved;
    }
    if (open && kept) set_status("%d doors opening; %d with armed slides stayed closed", moved, kept);
    else set_status("%d doors %s", moved, open ? "opening" : "closing");
    efb_changed();
}

/* ---- Payload / fuel ------------------------------------------------------------------------------- */
static const char *efb_entry_error(A380XEfbEntry entry)
{
    switch (entry) {
    case A380X_EFB_ENTRY_EMPTY: return "empty entry";
    case A380X_EFB_ENTRY_NOT_A_NUMBER: return "digits only";
    case A380X_EFB_ENTRY_TOO_LARGE: return "above the limit";
    default: return "ok";
    }
}

static void efb_load_apply(void)
{
    double kg = a380x_efb_load_kg(&load_plan);
    if (!loading_allowed()) { efb_changed(); return; }
    if (kg > A380X_EFB_MAX_PAYLOAD_KG) {
        set_status("Payload %.0f kg exceeds the %.0f kg maximum", kg, A380X_EFB_MAX_PAYLOAD_KG);
        efb_changed();
        return;
    }
    set_payload((float)kg);
    load_applied_kg = (float)kg;
    load_plan_set = 1;
    set_status("Payload loaded: %d passengers, %.0f kg", a380x_efb_load_pax(&load_plan), kg);
    efb_changed();
}

static void efb_refuel_start(void)
{
    if (fuel_target_kg < 0) { set_status("Enter a block fuel first"); efb_changed(); return; }
    if (!loading_allowed()) { refuel_active = 0; efb_changed(); return; }
    if (refuel_rate == 2) { set_fuel((float)fuel_target_kg); refuel_active = 0; }
    else { refuel_active = 1; set_status("Refuelling to %d kg (%s rate)", fuel_target_kg, refuel_rate ? "fast" : "real"); }
    efb_changed();
}

static void efb_refuel_update(float dt)
{
    float total, step, next;
    if (!refuel_active || dt <= 0.0f) return;
    if (!on_ground() || !engines_off() || fuel_target_kg < 0) {
        refuel_active = 0;
        set_status("Refuel stopped: ground only, engines off");
        efb_changed();
        return;
    }
    total = get_float(refs.fuel_total);
    step = refuel_kg_per_s[refuel_rate ? 1 : 0] * dt;
    if (fabsf(fuel_target_kg - total) <= step) { next = (float)fuel_target_kg; refuel_active = 0; }
    else next = total + (fuel_target_kg > total ? step : -step);
    set_fuel(next);
    if (!refuel_active) set_status("Refuel complete: %d kg", fuel_target_kg);
    efb_changed();
}

/* ---- Sounds --------------------------------------------------------------------------------------- */
static void efb_sounds_bind(void)
{
    int i;
    for (i = 0; i < EFB_SOUNDS; ++i) {
        if (efb_sound_refs[i] == NULL) efb_sound_refs[i] = XPLMFindDataRef(efb_sound_refs_names[i]);
        if (!efb_sound_load_valid) efb_sound_load[i] = efb_sound_refs[i] ? get_float(efb_sound_refs[i]) : NAN;
    }
    efb_sound_load_valid = 1;
}

/* ---- Navigation state ------------------------------------------------------------------------------ */
static void efb_tablet_show(EfbTablet *t, int section, int tab)
{
    if (section < 0 || section >= A380X_EFB_SECTIONS) return;
    t->section = section;
    if (tab >= 0 && tab < a380x_efb_tab_count(section)) t->tab[section] = tab;
    active_page = (EfbPage)a380x_efb_section_legacy_page(section, t->tab[section]);
}

/* a380x/xp12/efb/page/<legacy page>: both tablets follow, as both showed the one shared page before. */
static void efb_tablets_follow_page(int page)
{
    int section, tab, i;
    a380x_efb_legacy_page_target(page, &section, &tab);
    for (i = 0; i < 2; ++i) {
        tablets[i].section = section;
        if (tab < a380x_efb_tab_count(section)) tablets[i].tab[section] = tab;
    }
    efb_changed();
}

/* ---- Actions --------------------------------------------------------------------------------------- */
static int efb_arg_int(const A380XEfbAction *a, int index, int lo, int hi, int *out)
{
    long value;
    if (index >= a->argc || a380x_efb_parse_count(a->arg[index], hi, &value) != A380X_EFB_ENTRY_OK || value < lo) return 0;
    *out = (int)value;
    return 1;
}

static int efb_execute(EfbTablet *t, const A380XEfbAction *a)
{
    const char *v = a->verb;
    int n = 0, m = 0;
    if (!strcmp(v, "nav")) {
        int section = a->argc > 0 ? a380x_efb_section_find(a->arg[0]) : -1;
        if (section < 0) return 0;
        efb_tablet_show(t, section, a->argc > 1 ? a380x_efb_tab_find(section, a->arg[1]) : -1);
    } else if (!strcmp(v, "tab")) {
        int section = a->argc > 1 ? a380x_efb_section_find(a->arg[0]) : -1;
        int tab = section >= 0 ? a380x_efb_tab_find(section, a->arg[1]) : -1;
        if (tab < 0) return 0;
        efb_tablet_show(t, section, tab);
    } else if (!strcmp(v, "power")) {
        powered = !powered;
        set_status("Tablet power %s", powered ? "on" : "off");
    } else if (!strcmp(v, "set")) {
        if (a->argc < 2 || !efb_arg_int(a, 1, 0, 100, &n)) return 0;
        if (!strcmp(a->arg[0], "metric")) metric_units = n != 0;
        else if (!strcmp(a->arg[0], "autocheck")) automatic_checklists = n != 0;
        else if (!strcmp(a->arg[0], "battery")) battery_simulation = n != 0;
        else if (!strcmp(a->arg[0], "brightness")) brightness = n < 20 ? 20 : n;
        else return 0;
    } else if (!strcmp(v, "door")) {
        int door = a->argc > 0 ? a380x_efb_door_find(a->arg[0]) : -1;
        if (door < 0) return 0;
        efb_door_toggle(door);
    } else if (!strcmp(v, "doors")) {
        if (a->argc < 1 || (strcmp(a->arg[0], "open") && strcmp(a->arg[0], "close"))) return 0;
        efb_doors_all(!strcmp(a->arg[0], "open"));
    } else if (!strcmp(v, "slide")) {
        int door = a->argc > 1 ? a380x_efb_door_find(a->arg[0]) : -1;
        if (door < 0 || !efb_arg_int(a, 1, 0, 1, &n)) return 0;
        efb_slide_set(door, n);
    } else if (!strcmp(v, "slides")) {
        if (a->argc < 1 || (strcmp(a->arg[0], "arm") && strcmp(a->arg[0], "disarm"))) return 0;
        efb_slides_set_all(!strcmp(a->arg[0], "arm"));
    } else if (!strcmp(v, "pax") || !strcmp(v, "cargo")) {
        int cargo = v[0] == 'c', limit = cargo ? A380X_EFB_HOLDS : A380X_EFB_CLASSES, kg = 0;
        long typed = 0;
        A380XEfbEntry entry;
        if (a->argc < 2 || !efb_arg_int(a, 0, 0, limit - 1, &n)) return 0;
        entry = cargo ? a380x_efb_parse_cargo(a->arg[1], n, !metric_units, &kg)
                      : a380x_efb_parse_count(a->arg[1], a380x_efb_class_seats[n], &typed);
        if (entry != A380X_EFB_ENTRY_OK) {
            if (cargo) set_status("%s: %s (max %d kg)", a380x_efb_hold_names[n], efb_entry_error(entry), a380x_efb_hold_max_kg[n]);
            else set_status("%s: %s (%d seats)", a380x_efb_class_names[n], efb_entry_error(entry), a380x_efb_class_seats[n]);
            efb_changed();
            return 1;   /* a valid request whose value the rule refused */
        }
        if (cargo) load_plan.cargo_kg[n] = kg; else load_plan.pax[n] = (int)typed;
        load_plan_set = 1;
        set_status("%s set to %ld%s; press Load to apply", cargo ? a380x_efb_hold_names[n] : a380x_efb_class_names[n],
                   cargo ? (long)kg : typed, cargo ? " kg" : "");
    } else if (!strcmp(v, "load")) {
        if (a->argc < 1) return 0;
        if (!strcmp(a->arg[0], "apply")) efb_load_apply();
        else if (!strcmp(a->arg[0], "clear")) {
            memset(&load_plan, 0, sizeof(load_plan));
            load_plan_set = 1;
            efb_load_apply();
        } else if (!strcmp(a->arg[0], "full")) {
            for (n = 0; n < A380X_EFB_CLASSES; ++n) load_plan.pax[n] = a380x_efb_class_seats[n];
            load_plan_set = 1;
            set_status("All 484 seats planned; press Load to apply");
        } else return 0;
    } else if (!strcmp(v, "fuel")) {
        long typed = 0;
        if (a->argc < 1) return 0;
        if (!strcmp(a->arg[0], "start")) efb_refuel_start();
        else if (!strcmp(a->arg[0], "stop")) { refuel_active = 0; set_status("Refuel stopped"); }
        else if (!strcmp(a->arg[0], "rate")) {
            if (a->argc < 2) return 0;
            refuel_rate = !strcmp(a->arg[1], "real") ? 0 : !strcmp(a->arg[1], "fast") ? 1 : !strcmp(a->arg[1], "instant") ? 2 : -1;
            if (refuel_rate < 0) { refuel_rate = 2; return 0; }
        } else if (!strcmp(a->arg[0], "target")) {
            double limit = metric_units ? MAX_FUEL_KG : floor(MAX_FUEL_KG * A380X_EFB_LB_PER_KG);
            A380XEfbEntry entry = a->argc > 1 ? a380x_efb_parse_count(a->arg[1], (long)limit, &typed) : A380X_EFB_ENTRY_EMPTY;
            if (entry != A380X_EFB_ENTRY_OK) set_status("Block fuel: %s (max %.0f %s)", efb_entry_error(entry), limit, metric_units ? "kg" : "lb");
            else {
                fuel_target_kg = metric_units ? (int)typed : (int)lround(typed / A380X_EFB_LB_PER_KG);
                if (fuel_target_kg > (int)MAX_FUEL_KG) fuel_target_kg = (int)MAX_FUEL_KG;
                set_status("Block fuel target %d kg; press Start", fuel_target_kg);
            }
        } else return 0;
    } else if (!strcmp(v, "service")) {
        if (a->argc < 1) return 0;
        if (!on_ground()) { set_status("Ground services are ground-only"); }
        else if (!strcmp(a->arg[0], "gpu")) gpu_toggle();
        else if (!strcmp(a->arg[0], "jetway")) run_command("sim/ground_ops/jetway");
        else if (!strcmp(a->arg[0], "trucks")) run_command("sim/ground_ops/service_plane");
        else if (!strcmp(a->arg[0], "chocks")) run_command("sim/flight_controls/toggle_chocks");
        else return 0;
    } else if (!strcmp(v, "push")) {
        static const char *const pushes[3] = {"sim/ground_ops/pushback_left", "sim/ground_ops/pushback_straight",
                                              "sim/ground_ops/pushback_right"};
        if (a->argc < 1) return 0;
        if (!strcmp(a->arg[0], "steer")) {
            if (!efb_arg_int(a, 1, 0, 2, &n)) return 0;
            push_steer = n;
        } else if (!strcmp(a->arg[0], "start")) {
            if (!on_ground()) set_status("Pushback is ground-only");
            else { run_command(pushes[push_steer]); set_status("Pushback requested"); }
        } else if (!strcmp(a->arg[0], "stop")) run_command("sim/ground_ops/pushback_stop");
        else return 0;
    } else if (!strcmp(v, "cl_group")) {
        if (!efb_arg_int(a, 0, 0, 9, &n)) return 0;
        t->group = selected_checklist = n;
    } else if (!strcmp(v, "cl_item")) {
        if (!efb_arg_int(a, 0, 0, 9, &n) || !efb_arg_int(a, 1, 0, (int)checklist_item_counts[n] - 1, &m)) return 0;
        checklist_bits[n] ^= 1u << m;
        t->group = selected_checklist = n;
    } else if (!strcmp(v, "cl_next") || !strcmp(v, "cl_reset")) {
        if (!efb_arg_int(a, 0, 0, 9, &n)) return 0;
        t->group = selected_checklist = n;
        if (v[3] == 'n' && checklist_completed_bits(n) == (1u << checklist_item_counts[n]) - 1u && n < 9)
            t->group = selected_checklist = n + 1;   /* complete: CHECK ITEM moves on to the next group */
        else action_checklists(v[3] == 'n' ? 2 : 3);
    } else if (!strcmp(v, "preset")) {
        if (a->argc < 1) return 0;
        if (!strcmp(a->arg[0], "expedite")) action_presets(5);
        else {
            if (!efb_arg_int(a, 0, 1, 5, &n)) return 0;
            if (elec_network_active()) native_preset(n);
            else if (n == 1) { preset_cold_dark(); efb_preset_slides(1); }
            else if (n == 2) { preset_turnaround(); efb_preset_slides(2); }
            else if (n == 4) { preset_ready(); efb_preset_slides(4); }
            else set_status("This preset needs the native A380 electrical network");
        }
    } else if (!strcmp(v, "fail")) {
        if (!efb_arg_int(a, 0, 0, EFB_FAILURE_COUNT - 1, &n) || !efb_arg_int(a, 1, 0, 1, &m)) return 0;
        if (refs.failures[n] == NULL) set_status("%s failure unavailable", failure_names[n]);
        else { XPLMSetDatai(refs.failures[n], m ? 6 : 0); set_status("%s %s", failure_names[n], m ? "failed" : "repaired"); }
        selected_failure = n;
    } else if (!strcmp(v, "fail_clear")) {
        run_command("sim/operation/fix_all_systems");
        set_status("All X-Plane failures repaired");
    } else if (!strcmp(v, "radio")) {
        int khz = 0;
        if (a->argc < 1) return 0;
        if (!strcmp(a->arg[0], "swap1")) run_command("sim/radios/com1_standy_flip");
        else if (!strcmp(a->arg[0], "swap2")) run_command("sim/radios/com2_standy_flip");
        else if (!strcmp(a->arg[0], "guard")) { if (refs.com1_standby) XPLMSetDatai(refs.com1_standby, 121500); }
        else if (!strcmp(a->arg[0], "ident")) run_command("sim/transponder/transponder_ident");
        else if (!strcmp(a->arg[0], "stby1") || !strcmp(a->arg[0], "stby2")) {
            XPLMDataRef target = a->arg[0][4] == '1' ? refs.com1_standby : efb_extra_refs[EX_COM2_STBY];
            if (a->argc < 2 || !a380x_efb_parse_com_khz(a->arg[1], &khz)) set_status("COM frequency must be 118.000-136.990 on a valid channel");
            else if (target) { XPLMSetDatai(target, khz); set_status("COM %c standby %d.%03d", a->arg[0][4], khz / 1000, khz % 1000); }
        } else if (!strcmp(a->arg[0], "squawk")) {
            if (a->argc < 2 || !a380x_efb_parse_squawk(a->arg[1], &khz)) set_status("Squawk: four digits 0-7");
            else if (refs.transponder) { XPLMSetDatai(refs.transponder, khz); set_status("Squawk %04d", khz); }
        } else if (!strcmp(a->arg[0], "mode")) {
            if (!efb_arg_int(a, 1, 1, 7, &n) || (n != 1 && n != 3 && n != 6 && n != 7)) return 0;
            if (refs.transponder_mode) XPLMSetDatai(refs.transponder_mode, n);
        } else return 0;
    } else if (!strcmp(v, "tod")) {
        if (a->argc < 1) return 0;
        if (!strcmp(a->arg[0], "target")) {
            if (!efb_arg_int(a, 1, 0, metric_units ? 15240 : 50000, &n)) {
                set_status("Target altitude %s", metric_units ? "0-15240 m" : "0-50000 ft");
                efb_changed();
                return 1;
            }
            target_altitude_ft = metric_units ? (int)lround(n / 0.3048) : n;
            if (target_altitude_ft > 50000) target_altitude_ft = 50000;
        } else if (!strcmp(a->arg[0], "up")) action_performance(1);
        else if (!strcmp(a->arg[0], "down")) action_performance(0);
        else if (!strcmp(a->arg[0], "steeper")) action_performance(3);
        else if (!strcmp(a->arg[0], "shallower")) action_performance(2);
        else if (!strcmp(a->arg[0], "reset")) action_performance(4);
        else return 0;
    } else if (!strcmp(v, "map")) {
        static const char *const names[6] = {"current", "low", "high", "zoom_in", "zoom_out", "airports"};
        for (n = 0; n < 6; ++n) if (a->argc > 0 && !strcmp(a->arg[0], names[n])) break;
        if (n == 6) return 0;
        action_navigation(n);
    } else if (!strcmp(v, "vol")) {
        for (n = 0; n < EFB_SOUNDS; ++n) if (a->argc > 0 && !strcmp(a->arg[0], efb_sound_ids[n])) break;
        if (a->argc > 0 && !strcmp(a->arg[0], "restore")) {
            efb_sounds_bind();
            for (n = 0; n < EFB_SOUNDS; ++n)
                if (efb_sound_refs[n] && isfinite(efb_sound_load[n])) XPLMSetDataf(efb_sound_refs[n], efb_sound_load[n]);
            set_status("Volumes restored to their values at aircraft load");
        } else {
            if (n == EFB_SOUNDS || !efb_arg_int(a, 1, 0, 100, &m)) return 0;
            efb_sounds_bind();
            if (efb_sound_refs[n]) XPLMSetDataf(efb_sound_refs[n], m / 100.0f);
        }
    } else if (!strcmp(v, "option")) {
        if (a->argc < 2 || !efb_arg_int(a, 1, 0, 1, &n)) return 0;
        if (!strcmp(a->arg[0], "engine")) option_engine = n;
        else if (!strcmp(a->arg[0], "brakefans")) option_brake_fans = n;
        else if (!strcmp(a->arg[0], "hud")) option_hud = n;
        else return 0;
        set_status("Aircraft option stored; it applies in a later update");
    } else if (!strcmp(v, "cabin")) {
        if (a->argc > 0 && !strcmp(a->arg[0], "reset")) {
            static const int defaults[3][5] = {{0, 0, 56, 338, 0}, {14, 76, 0, 0, 0}, {0, 0, 0, 0, 0}};
            memcpy(cabin_layout, defaults, sizeof(cabin_layout));
            hold_pallets[0] = hold_pallets[1] = 0;
        } else if (a->argc > 0 && !strcmp(a->arg[0], "hold")) {
            if (!efb_arg_int(a, 1, 0, 1, &n) || !efb_arg_int(a, 2, 0, 1, &m)) return 0;
            hold_pallets[n] = m;
        } else {
            if (!efb_arg_int(a, 0, 0, 2, &n) || !efb_arg_int(a, 1, 0, 4, &m) || a->argc < 3) return 0;
            if (m == 4) { if (!efb_arg_int(a, 2, 0, 1, &m)) return 0; cabin_layout[n][4] = m; }
            else {
                long seats = 0;
                if (a380x_efb_parse_count(a->arg[2], 900, &seats) != A380X_EFB_ENTRY_OK) {
                    set_status("Cabin seats: whole number up to 900");
                    efb_changed();
                    return 1;
                }
                cabin_layout[n][m] = (int)seats;
            }
        }
        set_status("Cabin layout stored as a preview; it applies in a later update");
    } else return 0;
    efb_changed();
    return 1;
}

void a380x_efb_web_action(int tablet, const char *text)
{
    A380XEfbAction action;
    EfbTablet *t;
    if (tablet < 0 || tablet > 1) return;
    t = &tablets[tablet];
    if (!a380x_efb_parse_action(text, &action)) { ++efb_actions_rejected; return; }
    if (action.id > t->ack) t->ack = action.id;
    if (efb_execute(t, &action)) ++efb_actions_processed;
    else { ++efb_actions_rejected; set_status("Unknown tablet request ignored"); }
    efb_changed();
}

/* ---- Frame ---------------------------------------------------------------------------------------- */
static double efb_round(double value, double step) { return isfinite(value) ? floor(value / step + 0.5) * step : NAN; }

static double efb_ref_or_nan(XPLMDataRef ref, int is_int)
{
    if (ref == NULL) return NAN;
    return is_int ? (double)XPLMGetDatai(ref) : (double)XPLMGetDataf(ref);
}

static void efb_json_frame(A380XEfbJson *j, int index)
{
    EfbTablet *t = &tablets[index];
    float doors[A380X_EFB_DOORS], xp_fuel[9] = {0}, ratios[9] = {0};
    double tanks[A380X_EFB_TANKS], caps[A380X_EFB_TANKS];
    int i, k, tank_count = 0, chocks[10] = {0}, chock_count = 0;
    double gross = get_float(refs.gross_weight), fuel = get_float(refs.fuel_total), payload = get_float(refs.payload);
    double zulu = get_float(refs.zulu), local = efb_ref_or_nan(efb_extra_refs[EX_LOCAL_TIME], 0);
    double altitude = get_float(refs.altitude), ground_speed = get_float(refs.ground_speed);
    int network = elec_network_active();
    unsigned first;

    efb_read_doors(doors);
    if (refs.fuel != NULL) tank_count = XPLMGetDatavf(refs.fuel, xp_fuel, 0, 9);
    if (refs.tank_ratio != NULL) XPLMGetDatavf(refs.tank_ratio, ratios, 0, 9);
    a380x_efb_fqms_tanks(xp_fuel, tank_count, tanks);
    {
        float cap_kg[9];
        double total_ratio = 0.0;
        for (i = 0; i < 9; ++i) total_ratio += isfinite(ratios[i]) && ratios[i] > 0 ? ratios[i] : 0;
        for (i = 0; i < 9; ++i)
            cap_kg[i] = total_ratio > 0 && isfinite(ratios[i]) && ratios[i] > 0 ? (float)(MAX_FUEL_KG * ratios[i] / total_ratio) : 0.0f;
        if (!a380x_efb_fqms_tanks(cap_kg, total_ratio > 0 ? 9 : 0, caps)) for (i = 0; i < A380X_EFB_TANKS; ++i) caps[i] = NAN;
    }
    if (efb_extra_refs[EX_CHOCKS] != NULL) chock_count = XPLMGetDatavi(efb_extra_refs[EX_CHOCKS], chocks, 0, 10);

    a380x_efbj_object(j, NULL);
    a380x_efbj_int(j, "v", 1);
    a380x_efbj_int(j, "tablet", index);
    a380x_efbj_int(j, "epoch", (long)efb_epoch);
    a380x_efbj_int(j, "ack", (long)t->ack);

    a380x_efbj_object(j, "power");
    a380x_efbj_bool(j, "blank", efb_tablet_blank());
    a380x_efbj_bool(j, "on", powered);
    a380x_efbj_number(j, "battery", battery_level, 0);
    a380x_efbj_bool(j, "charging", electrical_power_available());
    a380x_efbj_bool(j, "sim", battery_simulation);
    a380x_efbj_end(j, '}');

    a380x_efbj_object(j, "settings");
    a380x_efbj_bool(j, "metric", metric_units);
    a380x_efbj_bool(j, "autoCheck", automatic_checklists);
    a380x_efbj_int(j, "brightness", brightness);
    a380x_efbj_end(j, '}');

    a380x_efbj_object(j, "clock");
    a380x_efbj_number(j, "zulu", refs.zulu ? efb_round(fmod(fmod(zulu, 86400.0) + 86400.0, 86400.0) / 60.0 - 0.5, 1) : NAN, 0);
    a380x_efbj_number(j, "local", isfinite(local) ? efb_round(fmod(fmod(local, 86400.0) + 86400.0, 86400.0) / 60.0 - 0.5, 1) : NAN, 0);
    a380x_efbj_end(j, '}');

    a380x_efbj_object(j, "ui");
    a380x_efbj_string(j, "section", a380x_efb_section_name(t->section));
    a380x_efbj_object(j, "tabs");
    for (i = 0; i < A380X_EFB_SECTIONS; ++i)
        if (a380x_efb_tab_count(i)) a380x_efbj_string(j, a380x_efb_section_name(i), a380x_efb_tab_name(i, t->tab[i]));
    a380x_efbj_end(j, '}');
    a380x_efbj_int(j, "group", t->group);
    a380x_efbj_end(j, '}');
    a380x_efbj_string(j, "status", status_message);

    a380x_efbj_object(j, "touch");
    a380x_efbj_int(j, "serial", (long)t->touch_serial);
    a380x_efbj_array(j, "events");
    first = t->touch_serial > EFB_TOUCH_EVENTS ? t->touch_serial - EFB_TOUCH_EVENTS + 1 : 1;
    for (; t->touch_serial && first <= t->touch_serial; ++first) {
        const EfbTouchEvent *e = &t->events[(first - 1) % EFB_TOUCH_EVENTS];
        a380x_efbj_array(j, NULL);
        a380x_efbj_int(j, NULL, (long)e->seq); a380x_efbj_int(j, NULL, e->type);
        a380x_efbj_int(j, NULL, e->x); a380x_efbj_int(j, NULL, e->y);
        a380x_efbj_end(j, ']');
    }
    a380x_efbj_end(j, ']');
    a380x_efbj_end(j, '}');

    a380x_efbj_object(j, "ac");
    a380x_efbj_string(j, "variant", "A380-842");
    a380x_efbj_string(j, "engines", "Trent 972B-84");
    a380x_efbj_bool(j, "onGround", on_ground());
    a380x_efbj_bool(j, "enginesOff", engines_off());
    a380x_efbj_bool(j, "loadingAllowed", on_ground() && engines_off());
    a380x_efbj_bool(j, "elecNetwork", network);
    a380x_efbj_bool(j, "aircraftPower", electrical_power_available());
    {
        static XPLMDataRef phase_ref;
        XPLMDataRef phase = native_lookup(&phase_ref, "a380x/xp12/avionics/flight_phase");
        if (phase) a380x_efbj_int(j, "phase", XPLMGetDatai(phase)); else a380x_efbj_null(j, "phase");
    }
    a380x_efbj_number(j, "altFt", refs.altitude ? efb_round(altitude, 10) : NAN, 0);
    a380x_efbj_number(j, "gsKt", refs.ground_speed ? efb_round(ground_speed, 1) : NAN, 0);
    a380x_efbj_number(j, "lat", refs.latitude ? get_double(refs.latitude) : NAN, 4);
    a380x_efbj_number(j, "lon", refs.longitude ? get_double(refs.longitude) : NAN, 4);
    a380x_efbj_number(j, "qnh", refs.qnh ? efb_round(get_float(refs.qnh) / 100.0, 1) : NAN, 0);
    a380x_efbj_number(j, "windDir", refs.wind_direction ? efb_round(get_float(refs.wind_direction), 1) : NAN, 0);
    a380x_efbj_number(j, "windKt", refs.wind_speed ? efb_round(get_float(refs.wind_speed), 1) : NAN, 0);
    a380x_efbj_number(j, "oatC", refs.oat ? efb_round(get_float(refs.oat), 1) : NAN, 0);
    a380x_efbj_number(j, "parkBrake", refs.parking_brake ? get_float(refs.parking_brake) : NAN, 2);
    a380x_efbj_bool(j, "beacon", refs.beacon && get_int(refs.beacon) > 0);
    a380x_efbj_end(j, '}');

    a380x_efbj_object(j, "weights");
    a380x_efbj_number(j, "gw", refs.gross_weight ? efb_round(gross, 10) : NAN, 0);
    a380x_efbj_number(j, "fob", refs.fuel_total ? efb_round(fuel, 10) : NAN, 0);
    a380x_efbj_number(j, "payload", refs.payload ? efb_round(payload, 10) : NAN, 0);
    a380x_efbj_number(j, "zfw", refs.gross_weight && refs.fuel_total ? efb_round(gross - fuel, 10) : NAN, 0);
    a380x_efbj_number(j, "oew", refs.gross_weight && refs.fuel_total && refs.payload ? efb_round(gross - fuel - payload, 10) : NAN, 0);
    a380x_efbj_int(j, "mtow", 510000); a380x_efbj_int(j, "mlw", 395000); a380x_efbj_int(j, "mzfw", 373000);
    a380x_efbj_number(j, "maxFuel", MAX_FUEL_KG, 0);
    a380x_efbj_number(j, "maxPayload", A380X_EFB_MAX_PAYLOAD_KG, 0);
    a380x_efbj_null(j, "cg");   /* Phase 2: real weight & balance */
    a380x_efbj_end(j, '}');

    a380x_efbj_object(j, "load");
    a380x_efbj_array(j, "pax"); for (i = 0; i < A380X_EFB_CLASSES; ++i) a380x_efbj_int(j, NULL, load_plan.pax[i]); a380x_efbj_end(j, ']');
    a380x_efbj_array(j, "cargo"); for (i = 0; i < A380X_EFB_HOLDS; ++i) a380x_efbj_int(j, NULL, load_plan.cargo_kg[i]); a380x_efbj_end(j, ']');
    a380x_efbj_array(j, "seats"); for (i = 0; i < A380X_EFB_CLASSES; ++i) a380x_efbj_int(j, NULL, a380x_efb_class_seats[i]); a380x_efbj_end(j, ']');
    a380x_efbj_array(j, "holdMax"); for (i = 0; i < A380X_EFB_HOLDS; ++i) a380x_efbj_int(j, NULL, a380x_efb_hold_max_kg[i]); a380x_efbj_end(j, ']');
    a380x_efbj_number(j, "paxKg", A380X_EFB_PAX_KG, 0);
    a380x_efbj_number(j, "planKg", a380x_efb_load_kg(&load_plan), 0);
    a380x_efbj_bool(j, "planSet", load_plan_set);
    a380x_efbj_bool(j, "liveMatches", load_applied_kg >= 0 && refs.payload && fabs(payload - load_applied_kg) < 1.0);
    a380x_efbj_end(j, '}');

    a380x_efbj_object(j, "fuel");
    a380x_efbj_array(j, "tanks");
    for (i = 0; i < A380X_EFB_TANKS; ++i) {
        a380x_efbj_array(j, NULL);
        a380x_efbj_string(j, NULL, a380x_efb_tank_names[i]);
        a380x_efbj_number(j, NULL, efb_round(tanks[i], 10), 0);
        a380x_efbj_number(j, NULL, efb_round(caps[i], 10), 0);
        a380x_efbj_end(j, ']');
    }
    a380x_efbj_end(j, ']');
    if (fuel_target_kg >= 0) a380x_efbj_int(j, "target", fuel_target_kg); else a380x_efbj_null(j, "target");
    a380x_efbj_bool(j, "refuelling", refuel_active);
    a380x_efbj_string(j, "rate", refuel_rate == 0 ? "real" : refuel_rate == 1 ? "fast" : "instant");
    a380x_efbj_end(j, '}');

    a380x_efbj_object(j, "doors");
    a380x_efbj_array(j, "ratio"); for (i = 0; i < A380X_EFB_DOORS; ++i) a380x_efbj_number(j, NULL, refs.doors ? doors[i] : NAN, 2); a380x_efbj_end(j, ']');
    a380x_efbj_array(j, "slides"); for (i = 0; i < A380X_EFB_SLIDES; ++i) a380x_efbj_int(j, NULL, slide_armed[i]); a380x_efbj_end(j, ']');
    a380x_efbj_int(j, "open", a380x_efb_doors_open(doors));
    a380x_efbj_int(j, "armed", a380x_efb_slides_armed(slide_armed));
    a380x_efbj_end(j, '}');

    a380x_efbj_object(j, "ground");
    a380x_efbj_bool(j, "gpu", gpu_connected());
    if (chock_count > 0) { for (k = 0, i = 0; i < chock_count; ++i) k |= chocks[i] != 0; a380x_efbj_bool(j, "chocks", k); }
    else a380x_efbj_null(j, "chocks");
    if (efb_extra_refs[EX_PUSHBACK]) a380x_efbj_bool(j, "tug", XPLMGetDatai(efb_extra_refs[EX_PUSHBACK]) != 0);
    else a380x_efbj_null(j, "tug");
    a380x_efbj_null(j, "jetway");   /* X-Plane exposes no jet-bridge connection state */
    a380x_efbj_int(j, "steer", push_steer);
    a380x_efbj_end(j, '}');

    a380x_efbj_array(j, "checklists");
    for (i = 0; i < 10; ++i) {
        unsigned automatic = 0, b;
        for (b = 0; b < checklist_item_counts[i]; ++b)
            if (automatic_checklists && checklist_condition_met(checklist_items[i][b].condition)) automatic |= 1u << b;
        a380x_efbj_object(j, NULL);
        a380x_efbj_string(j, "name", checklist_names[i]);
        a380x_efbj_int(j, "done", (long)(checklist_bits[i] | automatic));
        a380x_efbj_int(j, "auto", (long)automatic);
        a380x_efbj_array(j, "items");
        for (b = 0; b < checklist_item_counts[i]; ++b) {
            a380x_efbj_array(j, NULL);
            a380x_efbj_string(j, NULL, checklist_items[i][b].name);
            a380x_efbj_string(j, NULL, checklist_items[i][b].response);
            a380x_efbj_int(j, NULL, checklist_items[i][b].condition != CHECK_MANUAL);
            a380x_efbj_end(j, ']');
        }
        a380x_efbj_end(j, ']');
        a380x_efbj_end(j, '}');
    }
    a380x_efbj_end(j, ']');

    a380x_efbj_array(j, "failures");
    for (i = 0; i < EFB_FAILURE_COUNT; ++i) {
        a380x_efbj_array(j, NULL);
        a380x_efbj_string(j, NULL, failure_names[i]);
        a380x_efbj_string(j, NULL, failure_systems[i].system);
        a380x_efbj_string(j, NULL, failure_systems[i].ata);
        if (refs.failures[i]) a380x_efbj_int(j, NULL, XPLMGetDatai(refs.failures[i])); else a380x_efbj_null(j, NULL);
        a380x_efbj_end(j, ']');
    }
    a380x_efbj_end(j, ']');

    a380x_efbj_object(j, "radios");
    a380x_efbj_array(j, "com1");
    a380x_efbj_number(j, NULL, efb_ref_or_nan(refs.com1, 1), 0); a380x_efbj_number(j, NULL, efb_ref_or_nan(refs.com1_standby, 1), 0);
    a380x_efbj_end(j, ']');
    a380x_efbj_array(j, "com2");
    a380x_efbj_number(j, NULL, efb_ref_or_nan(efb_extra_refs[EX_COM2], 1), 0);
    a380x_efbj_number(j, NULL, efb_ref_or_nan(efb_extra_refs[EX_COM2_STBY], 1), 0);
    a380x_efbj_end(j, ']');
    a380x_efbj_number(j, "squawk", efb_ref_or_nan(refs.transponder, 1), 0);
    a380x_efbj_number(j, "mode", efb_ref_or_nan(refs.transponder_mode, 1), 0);
    a380x_efbj_end(j, '}');

    a380x_efbj_object(j, "tod");
    a380x_efbj_number(j, "altFt", refs.altitude ? efb_round(altitude, 10) : NAN, 0);
    a380x_efbj_int(j, "targetFt", target_altitude_ft);
    a380x_efbj_number(j, "angle", target_angle_deg, 1);
    a380x_efbj_number(j, "distanceNm", refs.altitude ? efb_round(a380x_efb_tod_distance_nm(altitude, target_altitude_ft, target_angle_deg), 1) : NAN, 0);
    a380x_efbj_number(j, "gsKt", refs.altitude && refs.ground_speed ? efb_round(a380x_efb_average_ground_speed(altitude, target_altitude_ft, fmax(1.0, ground_speed)), 1) : NAN, 0);
    a380x_efbj_end(j, '}');

    a380x_efbj_object(j, "presets");
    a380x_efbj_bool(j, "native", network);
    if (network) {
        char step[64];
        XPLMDataRef text = XPLMFindDataRef("a380x/xp12/elec/preset/step_name");
        step[0] = 0;
        if (text != NULL) { int n = XPLMGetDatab(text, step, 0, (int)sizeof(step) - 1); step[n > 0 ? n : 0] = 0; }
        a380x_efbj_int(j, "active", elec_native_int("a380x/xp12/elec/preset/active", 0));
        a380x_efbj_int(j, "done", elec_native_int("a380x/xp12/elec/preset/done", 0));
        a380x_efbj_int(j, "step", elec_native_int("a380x/xp12/elec/preset/step", 0));
        a380x_efbj_int(j, "count", elec_native_int("a380x/xp12/elec/preset/step_count", 0));
        a380x_efbj_bool(j, "expedited", elec_native_int("a380x/xp12/elec/preset/expedited", 0));
        a380x_efbj_string(j, "stepName", step);
    }
    a380x_efbj_end(j, '}');

    a380x_efbj_array(j, "sounds");
    for (i = 0; i < EFB_SOUNDS; ++i) {
        a380x_efbj_array(j, NULL);
        a380x_efbj_string(j, NULL, efb_sound_ids[i]);
        a380x_efbj_number(j, NULL, efb_sound_refs[i] ? efb_round(get_float(efb_sound_refs[i]) * 100.0, 1) : NAN, 0);
        a380x_efbj_end(j, ']');
    }
    a380x_efbj_end(j, ']');

    a380x_efbj_object(j, "options");
    a380x_efbj_int(j, "engine", option_engine);
    a380x_efbj_bool(j, "brakeFans", option_brake_fans);
    a380x_efbj_bool(j, "hud", option_hud);
    a380x_efbj_end(j, '}');

    a380x_efbj_object(j, "cabin");
    a380x_efbj_array(j, "decks");
    for (i = 0; i < 3; ++i) {
        a380x_efbj_array(j, NULL);
        for (k = 0; k < 5; ++k) a380x_efbj_int(j, NULL, cabin_layout[i][k]);
        a380x_efbj_end(j, ']');
    }
    a380x_efbj_end(j, ']');
    a380x_efbj_array(j, "pallets"); a380x_efbj_int(j, NULL, hold_pallets[0]); a380x_efbj_int(j, NULL, hold_pallets[1]); a380x_efbj_end(j, ']');
    a380x_efbj_end(j, '}');

    a380x_efbj_end(j, '}');
}

long a380x_efb_frame_json(int tablet, char *out, size_t cap)
{
    A380XEfbJson j;
    if (tablet < 0 || tablet > 1 || out == NULL) return -1;
    efb_sounds_bind();
    a380x_efbj_init(&j, out, cap);
    efb_json_frame(&j, tablet);
    return a380x_efbj_finish(&j);
}

unsigned a380x_efb_change_serial(void) { return efb_change_serial; }

/* ---- Periodic work (called from a380x_efb_update) ------------------------------------------------ */
static void efb_web_update(float dt)
{
    float doors[A380X_EFB_DOORS];
    int changed;
    efb_read_doors(doors);
    changed = a380x_efb_slides_follow_doors(doors, slide_armed);
    if (changed) { set_status("%d slide%s disarmed: door opened", changed, changed > 1 ? "s" : ""); efb_changed(); }
    if (elec_network_active()) {
        int done = elec_native_int("a380x/xp12/elec/preset/done", 0);
        int active = elec_native_int("a380x/xp12/elec/preset/active", 0);
        if (efb_preset_seen_done >= 0 && done != efb_preset_seen_done && !active) efb_preset_slides(done);
        if (active != efb_preset_seen_active) efb_changed();
        efb_preset_seen_done = done;
        efb_preset_seen_active = active;
    }
    efb_refuel_update(dt);
}

static void efb_web_init(void)
{
    int i;
    memset(tablets, 0, sizeof(tablets));
    for (i = 0; i < 2; ++i) tablets[i].last_x = tablets[i].last_y = -1;
    memset(slide_armed, 0, sizeof(slide_armed));
    memset(efb_sound_refs, 0, sizeof(efb_sound_refs));
    efb_sound_load_valid = 0;
    for (i = 0; i < EX_COUNT; ++i) efb_extra_refs[i] = optional_ref(efb_extra_names[i]);
    efb_epoch = (unsigned)(efb_epoch * 1103515245u + 12345u + (unsigned)command_count) | 1u;
    efb_preset_seen_done = efb_preset_seen_active = -1;
    refuel_active = 0;
    efb_changed();
    for (i = 0; i < A380X_EFB_SLIDES; ++i) {
        char name[64];
        snprintf(name, sizeof(name), "sim/flight_controls/door_toggle_%d", i + 1);
        door_intercepts[i] = XPLMFindCommand(name);
        if (door_intercepts[i]) XPLMRegisterCommandHandler(door_intercepts[i], efb_door_intercept, 1, (void *)(intptr_t)i);
    }
}

static void efb_web_shutdown(void)
{
    int i;
    for (i = 0; i < A380X_EFB_SLIDES; ++i) {
        if (door_intercepts[i]) XPLMUnregisterCommandHandler(door_intercepts[i], efb_door_intercept, 1, (void *)(intptr_t)i);
        door_intercepts[i] = NULL;
    }
    refuel_active = 0;
}
