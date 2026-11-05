/* SPDX-License-Identifier: GPL-3.0-or-later
 * One atlas-coordinate renderer for both cockpit tablets and the native menu
 * window. The UI is an original, unbranded A380 control panel.
 */
static int efb_host_left = 152, efb_host_bottom = 516;
static int efb_host_right = 872, efb_host_top = 1016;
static int efb_in_window;

static float efb_screen_x(float x)
{
    return efb_host_left + (x - EFB_LEFT) * (efb_host_right - efb_host_left) / 720.0f;
}

static float efb_screen_y(float y)
{
    return efb_host_bottom + (y - EFB_BOTTOM) * (efb_host_top - efb_host_bottom) / 500.0f;
}

static void efb_ink(EfbColor color)
{
    float light = (efb_in_window ? 1.0f : brightness / 100.0f);
    glColor3f(color.r * light, color.g * light, color.b * light);
}

static void efb_vertex(float x, float y)
{
    glVertex2f(efb_screen_x(x), efb_screen_y(y));
}

static void efb_box(float left, float bottom, float right, float top, EfbColor color)
{
    efb_ink(color);
    glBegin(GL_QUADS);
    efb_vertex(left, bottom); efb_vertex(right, bottom);
    efb_vertex(right, top); efb_vertex(left, top);
    glEnd();
}

static void efb_rule(float x0, float y0, float x1, float y1, EfbColor color)
{
    efb_ink(color);
    glBegin(GL_LINES);
    efb_vertex(x0, y0); efb_vertex(x1, y1);
    glEnd();
}

static void efb_label(float x, float y, const char *value, EfbColor color)
{
    float light = efb_in_window ? 1.0f : brightness / 100.0f;
    float rgb[3] = {color.r * light, color.g * light, color.b * light};
    XPLMDrawString(rgb, (int)lroundf(efb_screen_x(x)),
                   (int)lroundf(efb_screen_y(y)), value, NULL, xplmFont_Basic);
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
}

static void efb_pair(float x, float y, const char *name, const char *value)
{
    efb_label(x + 8, y, name, EFB_UNSELECTED);
    efb_label(x + 150, y, value, EFB_WHITE);
    efb_rule(x + 8, y - 8, x + 322, y - 8, EFB_STATUS);
}

static void efb_group(float x, const char *title)
{
    efb_label(x + 8, 881, title, EFB_HIGHLIGHT);
    efb_rule(x + 8, 869, x + 322, 869, EFB_ACCENT);
}

static void efb_note(float y, const char *message)
{
    efb_label(178, y, message, EFB_UNSELECTED);
}

static const char *efb_tab_names[PAGE_COUNT] = {
    "AIRCRAFT", "LOADING", "GROUND", "PERFO", "NAV", "RADIOS", "FAULTS",
    "CHECKS", "STATES", "SETTINGS"
};

static const char *efb_action_names[PAGE_COUNT][6] = {
    {"LOADING", "GROUND SERVICES", "PERFORMANCE", "NAVIGATION", "CHECKLISTS", "SETTINGS"},
    {"242 PASSENGERS", "484 PASSENGERS", "FUEL 50%", "FUEL 100%", "OPEN FMS", "CLEAR LOAD"},
    {"JETWAY", "SERVICES", "GROUND POWER", "DOOR 1", "ALL 20 DOORS", "PUSHBACK"},
    {"ALTITUDE -1000", "ALTITUDE +1000", "ANGLE -0.5", "ANGLE +0.5", "RESET", "NAVIGATION"},
    {"CURRENT MAP", "LOW ENROUTE", "HIGH ENROUTE", "ZOOM IN", "ZOOM OUT", "AIRPORTS"},
    {"COM1 SWAP", "XPDR STANDBY", "XPDR ALTITUDE", "XPDR TA/RA", "COM1 GUARD", "XPDR 2000"},
    {"PREVIOUS", "NEXT", "FAIL / REPAIR", "REPAIR ALL", "FIRST FAILURE", "CHECKLISTS"},
    {"PREVIOUS", "NEXT", "CHECK ITEM", "RESET PAGE", "RESET ALL", "COMPLETE"},
    {"COLD & DARK", "TURNAROUND", "READY FOR TAXI", "REPAIR ALL", "GROUND", "CHECKLISTS"},
    {"METRIC UNITS", "AUTO CHECK", "BATTERY SIM", "BRIGHTNESS -", "BRIGHTNESS +", "TABLET POWER"}
};

static const char *efb_native_preset_names[6] = {
    "COLD & DARK", "TURNAROUND", "READY PUSHBACK", "READY TAXI", "READY TAKEOFF", "EXPEDITE ON/OFF"
};

static void efb_draw_aircraft(void)
{
    char value[80];
    efb_group(170, "AIRCRAFT DATA");
    if (metric_units) snprintf(value, sizeof(value), "%.0f M", get_float(refs.altitude) * 0.3048f);
    else snprintf(value, sizeof(value), "%.0f FT", get_float(refs.altitude));
    efb_pair(170, 843, "ALTITUDE", value);
    if (metric_units) snprintf(value, sizeof(value), "%.0f KM/H", get_float(refs.ground_speed) * 1.852f);
    else snprintf(value, sizeof(value), "%.0f KT", get_float(refs.ground_speed));
    efb_pair(170, 811, "GROUND SPEED", value);
    if (metric_units) snprintf(value, sizeof(value), "%.0f KG", get_float(refs.gross_weight));
    else snprintf(value, sizeof(value), "%.0f LB", get_float(refs.gross_weight) * 2.20462262f);
    efb_pair(170, 779, "GROSS WEIGHT", value);
    if (metric_units) snprintf(value, sizeof(value), "%.0f KG", get_float(refs.fuel_total));
    else snprintf(value, sizeof(value), "%.0f LB", get_float(refs.fuel_total) * 2.20462262f);
    efb_pair(170, 747, "FUEL ON BOARD", value);
    efb_group(524, "SIMULATOR STATUS");
    efb_pair(524, 843, "AIRCRAFT", "A380-842");
    efb_pair(524, 811, "POSITION", on_ground() ? "ON GROUND" : "AIRBORNE");
    snprintf(value, sizeof(value), "%.0f HPA", get_float(refs.qnh) / 100.0f);
    efb_pair(524, 779, "QNH", value);
    snprintf(value, sizeof(value), "%03.0f / %.0f KT",
             get_float(refs.wind_direction), get_float(refs.wind_speed));
    efb_pair(524, 747, "WIND", value);
    efb_note(706, "FLIGHT PLAN IMPORT REQUIRES AN EXTERNAL CLIENT");
}

static void efb_draw_loading(void)
{
    char value[80];
    float fuel = get_float(refs.fuel_total);
    float factor = metric_units ? 1.0f : 2.20462262f;
    const char *unit = metric_units ? "KG" : "LB";
    efb_group(170, "WEIGHT AND BALANCE");
    snprintf(value, sizeof(value), "%.0f %s", get_float(refs.payload) * factor, unit);
    efb_pair(170, 843, "PAYLOAD", value);
    snprintf(value, sizeof(value), "%.0f %s", (get_float(refs.gross_weight) - fuel) * factor, unit);
    efb_pair(170, 811, "ZERO FUEL WT", value);
    snprintf(value, sizeof(value), "%.0f %s", get_float(refs.gross_weight) * factor, unit);
    efb_pair(170, 779, "GROSS WEIGHT", value);
    snprintf(value, sizeof(value), "%.0f %s", 510000.0f * factor, unit);
    efb_pair(170, 747, "MAX GROSS WT", value);
    efb_group(524, "FUEL AND PAYLOAD");
    snprintf(value, sizeof(value), "%.0f %s", fuel * factor, unit);
    efb_pair(524, 843, "FUEL ON BOARD", value);
    snprintf(value, sizeof(value), "%.0f %s", MAX_FUEL_KG * factor, unit);
    efb_pair(524, 811, "MAX FUEL", value);
    efb_pair(524, 779, "LOAD CHANGES", on_ground() && engines_off() ? "AVAILABLE" : "BLOCKED");
    efb_pair(524, 747, "CONDITION", "PARKED / ENG OFF");
    efb_note(706, "SIMBRIEF IMPORT REQUIRES AN EXTERNAL CLIENT");
}

static void efb_draw_ground(void)
{
    float doors[20] = {0};
    int i, open = 0;
    char value[80];
    if (refs.doors) XPLMGetDatavf(refs.doors, doors, 0, 20);
    for (i = 0; i < 20; ++i) if (doors[i] > 0.5f) ++open;
    efb_group(170, "GROUND SERVICES");
    efb_pair(170, 843, "AIRCRAFT", on_ground() ? "ON GROUND" : "AIRBORNE");
    efb_pair(170, 811, "GPU", gpu_connected() ? "CONNECTED" : "DISCONNECTED");
    efb_pair(170, 779, "ENGINES", engines_off() ? "OFF" : "RUNNING");
    efb_pair(170, 747, "LOADING", on_ground() && engines_off() ? "AVAILABLE" : "BLOCKED");
    efb_group(524, "DOORS AND LOAD");
    snprintf(value, sizeof(value), "%d / 20", open);
    efb_pair(524, 843, "DOORS OPEN", value);
    snprintf(value, sizeof(value), "%.0f %s", get_float(refs.fuel_total) * (metric_units ? 1.0f : 2.20462262f), metric_units ? "KG" : "LB");
    efb_pair(524, 811, "FUEL", value);
    snprintf(value, sizeof(value), "%.0f %s", get_float(refs.payload) * (metric_units ? 1.0f : 2.20462262f), metric_units ? "KG" : "LB");
    efb_pair(524, 779, "PAYLOAD", value);
    efb_pair(524, 747, "COCKPIT DOOR", doors[16] > 0.5f ? "OPEN" : "CLOSED");
    efb_note(706, "ALL DOORS INCLUDES MAIN, UPPER, COCKPIT AND CARGO");
}

static void efb_draw_performance(void)
{
    char value[100];
    double current = get_float(refs.altitude);
    double speed = a380x_efb_average_ground_speed(current, target_altitude_ft,
                                                   fmax(1.0, get_float(refs.ground_speed)));
    double distance = a380x_efb_tod_distance_nm(current, target_altitude_ft, target_angle_deg);
    efb_group(170, "DESCENT INPUTS");
    if (metric_units) snprintf(value, sizeof(value), "%.0f M", current * 0.3048);
    else snprintf(value, sizeof(value), "%.0f FT", current);
    efb_pair(170, 843, "CURRENT ALT", value);
    if (metric_units) snprintf(value, sizeof(value), "%.0f M", target_altitude_ft * 0.3048);
    else snprintf(value, sizeof(value), "%d FT", target_altitude_ft);
    efb_pair(170, 811, "TARGET ALT", value);
    snprintf(value, sizeof(value), "%.1f DEG", target_angle_deg);
    efb_pair(170, 779, "FLIGHT PATH", value);
    efb_group(524, "CALCULATED RESULT");
    if (metric_units) snprintf(value, sizeof(value), "%.0f KM", distance * 1.852);
    else snprintf(value, sizeof(value), "%.0f NM", distance);
    efb_pair(524, 843, "START DESCENT", value);
    if (metric_units) snprintf(value, sizeof(value), "%.0f KM/H", speed * 1.852);
    else snprintf(value, sizeof(value), "%.0f KT", speed);
    efb_pair(524, 811, "AVG GROUND SPD", value);
    efb_pair(524, 779, "METHOD", "GEOMETRIC TOD");
    efb_note(706, "ALTITUDE BUTTONS CHANGE BY 1000 FT; T.O / LDG DATA UNAVAILABLE");
}

static void efb_draw_navigation(void)
{
    char value[100];
    efb_group(170, "PRESENT POSITION");
    snprintf(value, sizeof(value), "%.4f", get_double(refs.latitude));
    efb_pair(170, 843, "LATITUDE", value);
    snprintf(value, sizeof(value), "%.4f", get_double(refs.longitude));
    efb_pair(170, 811, "LONGITUDE", value);
    if (metric_units) snprintf(value, sizeof(value), "%.0f M", get_float(refs.altitude) * 0.3048f);
    else snprintf(value, sizeof(value), "%.0f FT", get_float(refs.altitude));
    efb_pair(170, 779, "ALTITUDE", value);
    efb_group(524, "NATIVE MAP");
    if (metric_units) snprintf(value, sizeof(value), "%.0f KM", get_float(refs.gps_distance) * 1.852f);
    else snprintf(value, sizeof(value), "%.0f NM", get_float(refs.gps_distance));
    efb_pair(524, 843, "GPS DISTANCE", value);
    snprintf(value, sizeof(value), "%.0f MIN", get_float(refs.gps_time));
    efb_pair(524, 811, "GPS TIME", value);
    efb_pair(524, 779, "DISPLAY", "X-PLANE MAP");
    efb_note(706, "CHART AUTHENTICATION REQUIRES AN EXTERNAL CLIENT");
}

static void efb_draw_radios(void)
{
    char value[100];
    efb_group(170, "COM 1");
    snprintf(value, sizeof(value), "%.3f MHZ", get_int(refs.com1) / 1000.0);
    efb_pair(170, 843, "ACTIVE", value);
    snprintf(value, sizeof(value), "%.3f MHZ", get_int(refs.com1_standby) / 1000.0);
    efb_pair(170, 811, "STANDBY", value);
    efb_pair(170, 779, "GUARD", "121.500 MHZ");
    efb_group(524, "TRANSPONDER");
    snprintf(value, sizeof(value), "%04d", get_int(refs.transponder));
    efb_pair(524, 843, "CODE", value);
    snprintf(value, sizeof(value), "%d", get_int(refs.transponder_mode));
    efb_pair(524, 811, "MODE", value);
    efb_pair(524, 779, "ATC LINK", "LOCAL ONLY");
    efb_note(706, "ONLINE ATC REQUIRES AN EXTERNAL CLIENT");
}

static void efb_draw_faults(void)
{
    char value[100];
    int failed = refs.failures[selected_failure]
        && XPLMGetDatai(refs.failures[selected_failure]) >= 6;
    efb_group(170, "SELECTED SYSTEM");
    snprintf(value, sizeof(value), "%d / 20", selected_failure + 1);
    efb_pair(170, 843, "INDEX", value);
    efb_pair(170, 811, "SYSTEM", failure_names[selected_failure]);
    efb_pair(170, 779, "STATE", failed ? "FAILED" : "OPERATIONAL");
    efb_group(524, "FAULT CONTROL");
    efb_pair(524, 843, "SOURCE", "X-PLANE FAILURES");
    efb_pair(524, 811, "ACTION", "FAIL / REPAIR");
    efb_pair(524, 779, "REPAIR ALL", "AVAILABLE");
    efb_note(706, "SELECT A SYSTEM, THEN APPLY OR CLEAR ITS FAILURE");
}

static void efb_draw_checks(void)
{
    char value[128];
    unsigned full = (1u << checklist_item_counts[selected_checklist]) - 1u;
    unsigned completed = checklist_completed_bits(selected_checklist);
    unsigned count = 0, bit, i;
    for (bit = 1u; bit <= full; bit <<= 1u) if (completed & bit) ++count;
    snprintf(value, sizeof(value), "%d/10  %s", selected_checklist + 1,
             checklist_names[selected_checklist]);
    efb_label(178, 879, value, EFB_HIGHLIGHT);
    snprintf(value, sizeof(value), "%u/%u  AUTO %s", count,
             checklist_item_counts[selected_checklist], automatic_checklists ? "ON" : "OFF");
    efb_label(700, 879, value, EFB_WHITE);
    for (i = 0; i < checklist_item_counts[selected_checklist]; ++i) {
        const CheckItem *item = &checklist_items[selected_checklist][i];
        float x = i < 4 ? 178.0f : 532.0f;
        float y = 842.0f - (float)(i % 4) * 32.0f;
        snprintf(value, sizeof(value), "%s %-20.20s %s",
                 completed & (1u << i) ? "[X]" : "[ ]", item->name, item->response);
        efb_label(x, y, value, completed & (1u << i) ? EFB_GREEN : EFB_WHITE);
    }
    efb_note(699, "TAP AN ITEM TO CHECK IT; AUTO ITEMS FOLLOW AIRCRAFT STATE");
}

static void efb_draw_states(void)
{
    if (elec_network_active()) {
        static const char *const names[6] = {"NONE", "COLD & DARK", "TURNAROUND", "READY FOR PUSHBACK",
                                             "READY FOR TAXI", "READY FOR TAKEOFF"};
        char value[80], step[64];
        int active = elec_native_int("a380x/xp12/elec/preset/active", 0);
        int done = elec_native_int("a380x/xp12/elec/preset/done", 0);
        int count = elec_native_int("a380x/xp12/elec/preset/step_count", 0);
        int dc = elec_native_int("a380x/xp12/elec/bus/dc_ess_powered", 0);
        XPLMDataRef text = XPLMFindDataRef("a380x/xp12/elec/preset/step_name");
        step[0] = 0;
        if (text != NULL) { int n = XPLMGetDatab(text, step, 0, (int)sizeof(step) - 1); step[n > 0 ? n : 0] = 0; }
        efb_group(170, "AIRCRAFT STATE PRESET");
        efb_pair(170, 843, "PRESET", names[active >= 1 && active <= 5 ? active : (done >= 1 && done <= 5 ? done : 0)]);
        if (active) snprintf(value, sizeof(value), "%d / %d", elec_native_int("a380x/xp12/elec/preset/step", 0), count);
        else snprintf(value, sizeof(value), "%s", done ? "COMPLETE" : "IDLE");
        efb_pair(170, 811, "PROGRESS", value);
        efb_pair(170, 779, "STEP", step[0] ? step : "-");
        efb_group(524, "CURRENT AIRCRAFT");
        efb_pair(524, 843, "POSITION", on_ground() ? "ON GROUND" : "AIRBORNE");
        efb_pair(524, 811, "ELECTRICAL", electrical_power_available() ? "AC POWERED" : dc ? "BATTERIES" : "COLD & DARK");
        efb_pair(524, 779, "MODE", elec_native_int("a380x/xp12/elec/preset/expedited", 0) ? "EXPEDITED" : "NORMAL");
        efb_note(706, "TAP A RUNNING PRESET AGAIN TO STOP IT. GROUND ONLY.");
        return;
    }
    efb_group(170, "AIRCRAFT SITUATIONS");
    efb_pair(170, 843, "COLD & DARK", "PARKED");
    efb_pair(170, 811, "TURNAROUND", "PARKED");
    efb_pair(170, 779, "READY FOR TAXI", "PARKED");
    efb_group(524, "CURRENT AIRCRAFT");
    efb_pair(524, 843, "POSITION", on_ground() ? "ON GROUND" : "AIRBORNE");
    efb_pair(524, 811, "ELECTRICAL", electrical_power_available() ? "POWERED" : "OFF");
    efb_pair(524, 779, "ENGINES", engines_off() ? "STOPPED" : "RUNNING");
    efb_note(706, "STATE PRESETS ARE AVAILABLE WHILE PARKED");
}

static void efb_draw_settings(void)
{
    char value[80];
    efb_group(170, "DISPLAY SETTINGS");
    efb_pair(170, 843, "UNITS", metric_units ? "METRIC" : "IMPERIAL");
    snprintf(value, sizeof(value), "%d%%", brightness);
    efb_pair(170, 811, "BRIGHTNESS", value);
    efb_pair(170, 779, "TABLET POWER", powered ? "ON" : "OFF");
    efb_group(524, "SIMULATION OPTIONS");
    efb_pair(524, 843, "AUTO CHECK", automatic_checklists ? "ON" : "OFF");
    efb_pair(524, 811, "BATTERY SIM", battery_simulation ? "ON" : "OFF");
    snprintf(value, sizeof(value), "%.0f%%", battery_level);
    efb_pair(524, 779, "BATTERY LEVEL", value);
    efb_note(706, "A380 CONTROL PANEL | NATIVE X-PLANE INTERFACE");
}

static void efb_render_shared(void)
{
    static void (*const draw_pages[PAGE_COUNT])(void) = {
        efb_draw_aircraft, efb_draw_loading, efb_draw_ground, efb_draw_performance,
        efb_draw_navigation, efb_draw_radios, efb_draw_faults, efb_draw_checks,
        efb_draw_states, efb_draw_settings
    };
    char value[96];
    int i, zulu;
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    efb_box(EFB_LEFT, EFB_BOTTOM, EFB_RIGHT, EFB_TOP,
            !efb_in_window && !powered ? (EfbColor){0, 0, 0} : EFB_BODY);
    if (!efb_in_window && (!powered || (battery_simulation && battery_level <= 0.0
                                && !electrical_power_available()))) return;
    efb_box(152, 986, 872, 1016, EFB_STATUS);
    efb_label(166, 998, "A380 CONTROL PANEL", EFB_WHITE);
    zulu = (int)get_float(refs.zulu) % 86400;
    if (zulu < 0) zulu += 86400;
    snprintf(value, sizeof(value), "%02d:%02dZ   %s", zulu / 3600,
             (zulu % 3600) / 60, on_ground() ? "ON GROUND" : "AIRBORNE");
    efb_label(584, 998, value, EFB_UNSELECTED);
    efb_box(825, 988, 858, 1013, powered ? EFB_HIGHLIGHT : EFB_SECONDARY);
    efb_label(830, 996, powered ? "ON" : "OFF", powered ? EFB_BODY : EFB_HIGHLIGHT);
    for (i = 0; i < PAGE_COUNT; ++i) {
        float x = 160.0f + 70.0f * i;
        if ((int)active_page == i) {
            efb_box(x, 940, x + 68, 985, EFB_SECONDARY);
            efb_rule(x + 2, 940, x + 66, 940, EFB_WHITE);
        }
        efb_label(x + 5, 956, efb_tab_names[i], (int)active_page == i ? EFB_WHITE : EFB_HIGHLIGHT);
    }
    efb_rule(160, 936, 858, 936, EFB_ACCENT);
    efb_label(170, 908, page_titles[active_page], EFB_WHITE);
    efb_label(726, 908, "A380-842", EFB_UNSELECTED);
    efb_box(170, 728, 508, 896, EFB_SECONDARY);
    efb_box(524, 728, 858, 896, EFB_SECONDARY);
    draw_pages[active_page]();
    efb_rule(170, 684, 858, 684, EFB_ACCENT);
    snprintf(value, sizeof(value), "STATUS: %.75s", status_message);
    efb_label(170, 658, value, EFB_AMBER);
    for (i = 0; i < 6; ++i) {
        float x = 170.0f + (i % 3) * 230.0f;
        float y = i < 3 ? 594.0f : 538.0f;
        efb_box(x, y, x + 220, y + 44, EFB_HIGHLIGHT);
        efb_label(x + 12, y + 16, active_page == PAGE_PRESETS && elec_network_active()
                  ? efb_native_preset_names[i] : efb_action_names[active_page][i], EFB_BODY);
    }
    efb_rule(170, 526, 858, 526, EFB_STATUS);
    efb_label(170, 518, "NATIVE A380 / X-PLANE 12", EFB_GREEN);
    if (active_page == PAGE_FAILURES && refs.failures[selected_failure]
            && get_int(refs.failures[selected_failure]) >= 6)
        efb_label(750, 518, "FAULT ACTIVE", EFB_RED);
}

void a380x_efb_draw(void)
{
    /* Both tablets are cockpit devices once the OBJ binds them (a380x_efb_device.inc). */
    if (!a380x_efb_atlas_needed()) return;
    efb_host_left = 152; efb_host_bottom = 516;
    efb_host_right = 872; efb_host_top = 1016;
    efb_in_window = 0;
    efb_render_shared();
}

void a380x_efb_draw_window(int left, int bottom, int right, int top)
{
    efb_host_left = left; efb_host_bottom = bottom;
    efb_host_right = right; efb_host_top = top;
    efb_in_window = 1;
    efb_render_shared();
    efb_in_window = 0;
}

void a380x_efb_click(float x, float y)
{
    int i;
    if (x >= 825 && x <= 858 && y >= 986 && y <= 1013) {
        powered = !powered;
        set_status("Tablet power %s", powered ? "on" : "off");
        return;
    }
    for (i = 0; i < PAGE_COUNT; ++i) {
        float left = 160.0f + 70.0f * i;
        if (x >= left && x <= left + 68 && y >= 940 && y <= 986) {
            set_page((EfbPage)i);
            return;
        }
    }
    if (active_page == PAGE_CHECKLISTS && x >= 170 && x <= 858 && y >= 731 && y <= 858) {
        int column = x >= 524 ? 1 : 0;
        int row = (int)((858 - y) / 32);
        int item = column * 4 + row;
        if (item < (int)checklist_item_counts[selected_checklist]) {
            checklist_bits[selected_checklist] ^= 1u << item;
            set_status("%s: %s", checklist_items[selected_checklist][item].name,
                       checklist_bits[selected_checklist] & (1u << item) ? "checked" : "unchecked");
        }
        return;
    }
    for (i = 0; i < 6; ++i) {
        float left = 170.0f + (i % 3) * 230.0f;
        float bottom = i < 3 ? 594.0f : 538.0f;
        if (x >= left && x <= left + 220 && y >= bottom && y <= bottom + 44) {
            execute_action(i);
            return;
        }
    }
}
