"""Single source of the overhead annunciator lamp contract (every overhead pushbutton legend
outside the ELEC / EMER ELEC PWR panel and the ADIRS panel, which have their own modules).

Shared by the native table generator (native/avionics/a380x_ovhd_lamps_table.h, checked by
native/avionics/build.py), the avionics provider manifest and the cockpit OBJ patcher
(tools/ovhd_cockpit_lamps.py).  docs/OVERHEAD_LAMPS.md explains every row.

Sources (FlyByWire A380X, GPL-3.0, upstream-aircraft/fbw-a380x/.../Part_Interior_Cockpit/model):
  * legend faces: the authored <node>_SEQ1 (upper) / _SEQ2 (lower) children of each PUSH_OVHD_*
    node in A380_COCKPIT_LOD00.gltf; the legend text/colour is the art those faces sample in
    A380X_PUSH_TEXT_EMIS (identified per face, see docs/OVERHEAD_LAMPS.md);
  * lamp supply: the SEQ_POWERED parameter of each pushbutton in A380_Cockpit_Behavior.xml
    (FBW names: AC_ESS_SHED = real AC ESS 400XP, 309PP = DC APU; spec docs/ELEC_SYSTEM_SPEC.md 0.3);
    no SEQ_POWERED (FBW default "always") -> any of DC ESS / DC 1 / DC 2 here;
  * conditions: the SEQ1_CODE / SEQ2_CODE of the FBW templates, mapped to the port's native
    state or the X-Plane dataref the pushbutton drives (config/cockpit_controls.json).
"""
from __future__ import annotations

PREFIX = "a380x/xp12/ovhd/"

# Legend art: key -> (text, RGBA, atlas cell of cockpit_3d/fcu_labels*.png).
# Cells 47/48/59-62 are the existing ADIRS/ELEC cells (identical text and colour, reused);
# the others use cells 0-13, the unused FCU daylight-label cells (no drawn geometry samples
# them, see tools/ovhd_cockpit_lamps.py atlas_usage()).
WHITE, AMBER, GREEN, BLUE, RED = ((240, 245, 255, 255), (255, 180, 45, 255), (65, 255, 95, 255),
                                  (80, 175, 255, 255), (255, 55, 45, 255))
LEGENDS = {
    "fault": ("FAULT", AMBER, 47), "off": ("OFF", WHITE, 48), "altn": ("ALTN", WHITE, 59),
    "disc": ("DISC", WHITE, 60), "avail": ("AVAIL", GREEN, 61), "on": ("ON", BLUE, 62),
    "on_white": ("ON", WHITE, 0), "open": ("OPEN", GREEN, 1), "man": ("MAN", WHITE, 2),
    "ovrd": ("OVRD", WHITE, 3), "smoke": ("SMOKE", RED, 4), "disch": ("DISCH", WHITE, 5),
    "squib": ("SQUIB", WHITE, 6), "disch_amber": ("DISCH", AMBER, 7), "call": ("CALL", AMBER, 8),
    "evac": ("EVAC", RED, 9), "btl1": ("BTL 1", AMBER, 10), "btl2": ("BTL 2", AMBER, 11),
    "fault_white": ("FAULT", WHITE, 12), "off_red": ("OFF", RED, 13),
}
NEW_LEGEND_CELLS = {key: cell for key, (_text, _rgba, cell) in LEGENDS.items() if cell < 16}

# Lamp supply tokens -> native buses (a380x_elec.h).  "A|B" any of, "A&B" all of.
BUSES = {"DC_ESS": "A380X_ELN_DC_ESS", "DC1": "A380X_ELN_DC1", "DC2": "A380X_ELN_DC2",
         "AC1": "A380X_ELN_AC1", "AC2": "A380X_ELN_AC2", "AC_ESS": "A380X_ELN_AC_ESS",
         "DC_APU": "A380X_ELN_DC_APU"}
ANY_DC = "DC_ESS|DC1|DC2"

# Lamp conditions (native lamp logic a380x_ovhd_lamps.h).  None: ANN LT TEST only (the system
# behind the pushbutton is not simulated, the FBW normal position shows no light).
#   "test"            ANN LT TEST only
#   "fault"           writable fail/<id> input only (no X-Plane failure exists)
#   "<kind>:<n>"      indexed kinds below
SOURCES = (
    "test", "fault", "apu_master_on", "apu_master_fault", "apu_start_on", "apu_avail", "apu_bleed_on",
    "eng_bleed_off", "eng_bleed_fault", "pack_off", "eng_ai_on", "eng_ai_fault", "wing_ai_on",
    "wing_ai_fault", "window_heat_on", "feed_pump_off", "crew_oxy_off", "pb_off", "pb_on",
)

# id, panel caption, FBW pushbutton node, upper (SEQ1) legend, lower (SEQ2) legend,
# supply, upper condition, lower condition.  None legend: no authored face (or blank art).
PUSHBUTTONS = (
    # APU
    ("apu_master", "APU MASTER SW", "PUSH_OVHD_APU_MASTERSW", "fault", "on", "DC_APU", "apu_master_fault", "apu_master_on"),
    ("apu_start", "APU START", "PUSH_OVHD_APU_START", "avail", "on", "AC_ESS|DC_APU", "apu_avail", "apu_start_on"),
    # AIR COND / bleed
    ("eng1_bleed", "ENG 1 BLEED", "PUSH_OVHD_AIRCOND_ENG1BLEED", "fault", "off", "DC2", "eng_bleed_fault:0", "eng_bleed_off:0"),
    ("eng2_bleed", "ENG 2 BLEED", "PUSH_OVHD_AIRCOND_ENG2BLEED", "fault", "off", "DC_ESS", "eng_bleed_fault:1", "eng_bleed_off:1"),
    ("eng3_bleed", "ENG 3 BLEED", "PUSH_OVHD_AIRCOND_ENG3BLEED", "fault", "off", "DC_ESS", "eng_bleed_fault:2", "eng_bleed_off:2"),
    ("eng4_bleed", "ENG 4 BLEED", "PUSH_OVHD_AIRCOND_ENG4BLEED", "fault", "off", "DC2", "eng_bleed_fault:3", "eng_bleed_off:3"),
    ("apu_bleed", "APU BLEED", "PUSH_OVHD_AIRCOND_APUBLEED", "fault", "on", "DC_ESS", "fault", "apu_bleed_on"),
    ("pack1", "PACK 1", "PUSH_OVHD_AIRCOND_PACK1", "fault", "off", "DC1&DC_ESS", "fault", "pack_off:0"),
    ("pack2", "PACK 2", "PUSH_OVHD_AIRCOND_PACK2", "fault", "off", "DC2", "fault", "pack_off:1"),
    ("hot_air1", "HOT AIR 1", "PUSH_OVHD_AIRCOND_HOTAIR", "fault", "off", "DC1&DC_ESS", "fault", "test"),
    ("hot_air2", "HOT AIR 2", "PUSH_OVHD_AIRCOND_HOTAIR2", "fault", "off", "DC1&DC_ESS", "fault", "test"),
    ("ram_air", "RAM AIR", "PUSH_OVHD_AIRCOND_RAMAIR", None, "on_white", "DC_ESS", None, "test"),
    ("cargo_fwd_isol", "FWD CARGO ISOL VALVES", "PUSH_OVHD_AIRCOND_FWDCARGO", "fault", "off", "AC_ESS", "fault", "test"),
    ("cargo_bulk_isol", "BULK CARGO ISOL VALVES", "PUSH_OVHD_AIRCOND_BULKCARGO", "fault", "off", "AC_ESS", "fault", "test"),
    ("cargo_heater", "CARGO HEATER", "PUSH_OVHD_AIRCOND_CARGOHOT", "fault", "off", "AC_ESS", "fault", "test"),
    ("cab_air_extract", "CAB AIR EXTRACT", "PUSH_OVHD_CABAIR_EXTRACT", "fault", "ovrd", "AC_ESS", "fault", "test"),
    ("vent_extract", "VENT EXTRACT", "PUSH_OVHD_VENTILATION_EXTRACT", "fault", "ovrd", "AC_ESS", "fault", "test"),
    ("vent_smoke", "VENT SMOKE", "PUSH_OVHD_VENTILATION_SMOKE", "smoke", None, "AC_ESS", "test", None),
    ("cab_fans", "CAB FANS", "PUSH_OVHD_VENTILATION_CABFANS", None, "off", "AC1", None, "test"),
    ("vent_coolg", "VENT COOLG", "PUSH_OVHD_VENTILATION_COOLG", "fault", "off", "AC1", "fault", "test"),
    ("avncs_gnd_coolg", "AVNCS GND COOLG", "PUSH_OVHD_AVNCS_GND_COOLG", None, "on_white", ANY_DC, None, "test"),
    ("ovht_cond_fans", "OVHT COND FANS", "PUSH_OVHD_OVHT_COND_FANS", None, "on_white", ANY_DC, None, "test"),
    # CABIN PRESS
    ("press_man_alt", "CABIN PRESS MAN ALT", "PUSH_OVHD_CABINPRESS_MODESEL", None, "man", "AC_ESS", None, "test"),
    ("press_man_vs", "CABIN PRESS MAN V/S", "PUSH_OVHD_CABINPRESS_MODESELVS", None, "man", "AC_ESS", None, "test"),
    ("ditching", "DITCHING", "PUSH_OVHD_CABINPRESS_DITCHING", None, "on_white", "AC_ESS", None, "test"),
    # ANTI ICE
    ("eng1_anti_ice", "ENG 1 ANTI ICE", "PUSH_OVHD_ANTIICE_ENG1", "fault", "on", "AC1", "eng_ai_fault:0", "eng_ai_on:0"),
    ("eng2_anti_ice", "ENG 2 ANTI ICE", "PUSH_OVHD_ANTIICE_ENG2", "fault", "on", "AC2", "eng_ai_fault:1", "eng_ai_on:1"),
    ("eng3_anti_ice", "ENG 3 ANTI ICE", "PUSH_OVHD_ANTIICE_ENG3", "fault", "on", "AC1", "eng_ai_fault:2", "eng_ai_on:2"),
    ("eng4_anti_ice", "ENG 4 ANTI ICE", "PUSH_OVHD_ANTIICE_ENG4", "fault", "on", "AC2", "eng_ai_fault:3", "eng_ai_on:3"),
    ("wing_anti_ice", "WING ANTI ICE", "PUSH_OVHD_ANTIICE_WING", "fault", "on", "AC_ESS", "wing_ai_fault", "wing_ai_on"),
    ("probes_window_heat", "PROBES & WINDOW HEAT", "PUSH_OVHD_PROBESWINDOW", None, "on", "AC2", None, "window_heat_on"),
    # FUEL
    ("feed1_main", "FEED TK 1 MAIN", "PUSH_OVHD_FUEL_FEEDTK1_MAIN", "fault", "off", "AC_ESS", "fault", "feed_pump_off:0"),
    ("feed2_main", "FEED TK 2 MAIN", "PUSH_OVHD_FUEL_FEEDTK2_MAIN", "fault", "off", "AC2", "fault", "feed_pump_off:1"),
    ("feed3_main", "FEED TK 3 MAIN", "PUSH_OVHD_FUEL_FEEDTK3_MAIN", "fault", "off", "AC2", "fault", "feed_pump_off:2"),
    ("feed4_main", "FEED TK 4 MAIN", "PUSH_OVHD_FUEL_FEEDTK4_MAIN", "fault", "off", "AC2", "fault", "feed_pump_off:3"),
    ("feed1_stby", "FEED TK 1 STBY", "PUSH_OVHD_FUEL_FEEDTK1_STBY", "fault", "off", "AC2", "fault", "test"),
    ("feed2_stby", "FEED TK 2 STBY", "PUSH_OVHD_FUEL_FEEDTK2_STBY", "fault", "off", "AC1", "fault", "test"),
    ("feed3_stby", "FEED TK 3 STBY", "PUSH_OVHD_FUEL_FEEDTK3_STBY", "fault", "off", "AC1", "fault", "test"),
    ("feed4_stby", "FEED TK 4 STBY", "PUSH_OVHD_FUEL_FEEDTK4_STBY", "fault", "off", "AC1", "fault", "test"),
    ("outer_tk_left", "L OUTR TK PUMP", "PUSH_OVHD_FUEL_LOUTR_PMP", "fault", "off", "AC_ESS", "fault", "test"),
    ("mid_tk_left_fwd", "L MID TK FWD", "PUSH_OVHD_FUEL_LMIDTK_FWD", "fault", "off", "AC2", "fault", "test"),
    ("mid_tk_left_aft", "L MID TK AFT", "PUSH_OVHD_FUEL_LMIDTK_AFT", "fault", "off", "AC2", "fault", "test"),
    ("inr_tk_left_fwd", "L INR TK FWD", "PUSH_OVHD_FUEL_LINRTK_FWD", "fault", "off", "AC2", "fault", "test"),
    ("inr_tk_left_aft", "L INR TK AFT", "PUSH_OVHD_FUEL_LINRTK_AFT", "fault", "off", "AC2", "fault", "test"),
    ("inr_tk_right_fwd", "R INR TK FWD", "PUSH_OVHD_FUEL_RINRTK_FWD", "fault", "off", "AC2", "fault", "test"),
    ("inr_tk_right_aft", "R INR TK AFT", "PUSH_OVHD_FUEL_RINRTK_AFT", "fault", "off", "AC2", "fault", "test"),
    ("mid_tk_right_fwd", "R MID TK FWD", "PUSH_OVHD_FUEL_RMIDTK_FWD", "fault", "off", "AC2", "fault", "test"),
    ("mid_tk_right_aft", "R MID TK AFT", "PUSH_OVHD_FUEL_RMIDTK_AFT", "fault", "off", "AC2", "fault", "test"),
    ("outer_tk_right", "R OUTR TK PUMP", "PUSH_OVHD_FUEL_ROUTR_PMP", "fault", "off", "AC_ESS", "fault", "test"),
    ("trim_tk_left", "TRIM TK L", "PUSH_OVHD_FUEL_TRIMTK_L", "fault", "off", "AC2", "fault", "test"),
    ("trim_tk_right", "TRIM TK R", "PUSH_OVHD_FUEL_TRIMTK_R", "fault", "off", "AC2", "fault", "test"),
    ("xfeed1", "X FEED 1", "PUSH_OVHD_FUEL_XFEED", "open", "on_white", "AC_ESS", "test", "test"),
    ("xfeed2", "X FEED 2", "PUSH_OVHD_FUEL_XFEED2", "open", "on_white", "AC_ESS", "test", "test"),
    ("xfeed3", "X FEED 3", "PUSH_OVHD_FUEL_XFEED3", "open", "on_white", "AC_ESS", "test", "test"),
    ("xfeed4", "X FEED 4", "PUSH_OVHD_FUEL_XFEED4", "open", "on_white", "AC_ESS", "test", "test"),
    ("outer_tk_xfr", "OUTR TK XFR", "PUSH_OVHD_FUEL_OUTRTK_XFR", "fault", "man", "AC_ESS", "fault", "test"),
    ("mid_tk_xfr", "MID TK XFR", "PUSH_OVHD_FUEL_MIDTK_XFR", "fault", "man", "AC_ESS", "fault", "test"),
    ("inr_tk_xfr", "INR TK XFR", "PUSH_OVHD_FUEL_INRTK_XFR", "fault", "man", "AC_ESS", "fault", "test"),
    ("trim_tk_xfr", "TRIM TK XFR", "PUSH_OVHD_FUEL_TRIMTK_XFR", "fault", "man", "AC_ESS", "fault", "test"),
    ("emer_outr_xfr", "EMER OUTR TK XFR", "PUSH_OVHD_EMER_OUTR_XFR", None, "on_white", "AC1", None, "test"),
    ("jettison_arm", "JETTISON ARM", "PUSH_OVHD_JETTISON_ARM", None, "on_white", "AC1", None, "test"),
    ("jettison_active", "JETTISON ACTIVE", "PUSH_OVHD_JETTISON_ACTIVE", "open", "on_white", "AC1", "test", "test"),
    ("refuel", "REFUEL", "PUSH_OVHD_REFUEL", None, "on_white", ANY_DC, None, "test"),
    ("auto_gnd_xfr", "AUTO GND XFR", "PUSH_OVHD_AUTO_GND_XFR", None, "on_white", ANY_DC, None, "test"),
    # HYD
    ("hyd_eng1a", "ENG 1 PUMP A", "PUSH_OVHD_HYD_ENG1PUMP", "fault", "off", "AC1", "fault", "test"),
    ("hyd_eng1b", "ENG 1 PUMP B", "PUSH_OVHD_HYD_ENG1PUMPB", "fault", "off", "AC1", "fault", "test"),
    ("hyd_eng2a", "ENG 2 PUMP A", "PUSH_OVHD_HYD_ENG2PUMP", "fault", "off", "AC2", "fault", "test"),
    ("hyd_eng2b", "ENG 2 PUMP B", "PUSH_OVHD_HYD_ENG2PUMPB", "fault", "off", "AC2", "fault", "test"),
    ("hyd_eng3a", "ENG 3 PUMP A", "PUSH_OVHD_HYD_ENG3PUMP", "fault", "off", "AC1", "fault", "test"),
    ("hyd_eng3b", "ENG 3 PUMP B", "PUSH_OVHD_HYD_ENG3PUMPB", "fault", "off", "AC1", "fault", "test"),
    ("hyd_eng4a", "ENG 4 PUMP A", "PUSH_OVHD_HYD_ENG4PUMP", "fault", "off", "AC2", "fault", "test"),
    ("hyd_eng4b", "ENG 4 PUMP B", "PUSH_OVHD_HYD_ENG4PUMPB", "fault", "off", "AC2", "fault", "test"),
    ("hyd_eng1_disc", "ENG 1 PUMP DISC", "PUSH_OVHD_HYD_ENG1_AB", "fault", "disc", "AC2", "fault", "test"),
    ("hyd_eng2_disc", "ENG 2 PUMP DISC", "PUSH_OVHD_HYD_ENG2_AB", "fault", "disc", "AC2", "fault", "test"),
    ("hyd_eng3_disc", "ENG 3 PUMP DISC", "PUSH_OVHD_HYD_ENG3_AB", "fault", "disc", "AC2", "fault", "test"),
    ("hyd_eng4_disc", "ENG 4 PUMP DISC", "PUSH_OVHD_HYD_ENG4_AB", "fault", "disc", "AC2", "fault", "test"),
    ("hyd_g_elec_a_on", "GREEN ELEC PUMP A ON", "PUSH_OVHD_HYD_GPUMPA", None, "on", "AC_ESS", None, "test"),
    ("hyd_g_elec_b_on", "GREEN ELEC PUMP B ON", "PUSH_OVHD_HYD_GPUMPB", None, "on", "AC_ESS", None, "test"),
    ("hyd_y_elec_a_on", "YELLOW ELEC PUMP A ON", "PUSH_OVHD_HYD_YPUMPA", None, "on", "AC_ESS", None, "test"),
    ("hyd_y_elec_b_on", "YELLOW ELEC PUMP B ON", "PUSH_OVHD_HYD_YPUMPB", None, "on", "AC_ESS", None, "test"),
    ("hyd_g_elec_a", "GREEN ELEC PUMP A", "PUSH_OVHD_HYD_G_A", "fault", "off", "AC2", "fault", "test"),
    ("hyd_g_elec_b", "GREEN ELEC PUMP B", "PUSH_OVHD_HYD_G_B", "fault", "off", "AC2", "fault", "test"),
    ("hyd_y_elec_a", "YELLOW ELEC PUMP A", "PUSH_OVHD_HYD_Y_A", "fault", "off", "AC2", "fault", "test"),
    ("hyd_y_elec_b", "YELLOW ELEC PUMP B", "PUSH_OVHD_HYD_Y_B", "fault", "off", "AC2", "fault", "test"),
    # FIRE
    ("fire_eng1_agent1", "ENG 1 AGENT 1", "PUSH_OVHD_FIRE_ENG1_AGENT1", "squib", "disch_amber", "AC_ESS", "test", "test"),
    ("fire_eng1_agent2", "ENG 1 AGENT 2", "PUSH_OVHD_FIRE_ENG1_AGENT2", "squib", "disch_amber", "AC2", "test", "test"),
    ("fire_eng2_agent1", "ENG 2 AGENT 1", "PUSH_OVHD_FIRE_ENG2_AGENT1", "squib", "disch_amber", "AC2", "test", "test"),
    ("fire_eng2_agent2", "ENG 2 AGENT 2", "PUSH_OVHD_FIRE_ENG2_AGENT2", "squib", "disch_amber", "AC_ESS", "test", "test"),
    ("fire_eng3_agent1", "ENG 3 AGENT 1", "PUSH_OVHD_FIRE_ENG3_AGENT1", "squib", "disch_amber", "AC_ESS", "test", "test"),
    ("fire_eng3_agent2", "ENG 3 AGENT 2", "PUSH_OVHD_FIRE_ENG3_AGENT2", "squib", "disch_amber", "AC2", "test", "test"),
    ("fire_eng4_agent1", "ENG 4 AGENT 1", "PUSH_OVHD_FIRE_ENG4_AGENT1", "squib", "disch_amber", "AC2", "test", "test"),
    ("fire_eng4_agent2", "ENG 4 AGENT 2", "PUSH_OVHD_FIRE_ENG4_AGENT2", "squib", "disch_amber", "AC_ESS", "test", "test"),
    ("fire_apu_agent", "APU AGENT", "PUSH_OVHD_FIRE_AGENT", "squib", "disch_amber", "AC_ESS", "test", "test"),
    ("cargo_smoke_fwd", "CARGO SMOKE FWD DISCH", "PUSH_OVHD_CARGOSMOKE_FWD", "smoke", "disch", "AC_ESS", "test", "test"),
    ("cargo_smoke_aft", "CARGO SMOKE AFT DISCH", "PUSH_OVHD_CARGOSMOKE_AFT", "smoke", "disch", "AC_ESS", "test", "test"),
    ("cargo_btl1", "CARGO SMOKE BTL 1", "PUSH_OVHD_CARGOSMOKE_FWD2", "btl1", None, ANY_DC, "test", None),
    ("cargo_btl2", "CARGO SMOKE BTL 2", "PUSH_OVHD_CARGOSMOKE_FWD1", "btl2", None, ANY_DC, "test", None),
    # ENGINE
    ("fadec1", "FADEC GND PWR 1", "PUSH_OVHD_ENG_FADEC1", None, "on_white", "DC_ESS", None, "test"),
    ("fadec2", "FADEC GND PWR 2", "PUSH_OVHD_ENG_FADEC2", None, "on_white", "DC_ESS", None, "test"),
    ("fadec3", "FADEC GND PWR 3", "PUSH_OVHD_ENG_FADEC3", None, "on_white", "DC_ESS", None, "test"),
    ("fadec4", "FADEC GND PWR 4", "PUSH_OVHD_ENG_FADEC4", None, "on_white", "DC_ESS", None, "test"),
    ("man_start1", "ENG 1 MAN START", "PUSH_OVHD_ENGMANSTART_1", None, "on", "AC_ESS", None, "test"),
    ("man_start2", "ENG 2 MAN START", "PUSH_OVHD_ENGMANSTART_2", None, "on", "AC_ESS", None, "test"),
    ("man_start3", "ENG 3 MAN START", "PUSH_OVHD_ENGMANSTART_3", None, "on", "AC_ESS", None, "test"),
    ("man_start4", "ENG 4 MAN START", "PUSH_OVHD_ENGMANSTART_4", None, "on", "AC_ESS", None, "test"),
    ("man_start_altn", "ENG MAN START ALTN", "PUSH_OVHD_ENGMANSTARTALTN", None, "on", "AC_ESS", None, "test"),
    # FLT CTL (FBW: no SEQ_POWERED)
    ("prim1", "PRIM 1", "PUSH_OVHD_FLTCTL_PRIM1", "fault", "off", ANY_DC, "fault", "test"),
    ("prim2", "PRIM 2", "PUSH_OVHD_FLTCTL_PRIM2", "fault", "off", ANY_DC, "fault", "test"),
    ("prim3", "PRIM 3", "PUSH_OVHD_FLTCTL_PRIM3", "fault", "off", ANY_DC, "fault", "test"),
    ("sec1", "SEC 1", "PUSH_OVHD_FLTCTL_SEC1", "fault", "off", ANY_DC, "fault", "test"),
    ("sec2", "SEC 2", "PUSH_OVHD_FLTCTL_SEC2", "fault", "off", ANY_DC, "fault", "test"),
    ("sec3", "SEC 3", "PUSH_OVHD_FLTCTL_SEC3", "fault", "off", ANY_DC, "fault", "test"),
    # CALLS, EVAC, OXYGEN, RCDR, signs, cabin / NSS
    ("calls_emer", "CALLS EMER", "PUSH_OVHD_CALLS_EMER", "call", "on_white", "AC_ESS", "test", "test"),
    ("evac_command", "EVAC COMMAND", "PUSH_OVHD_EVAC_COMMAND", "evac", "on_white", "AC_ESS", "test", "test"),
    ("crew_oxy", "CREW SUPPLY (OXYGEN)", "PUSH_OVHD_OXYGEN_CREW", None, "off", "AC_ESS", None, "crew_oxy_off"),
    ("pax_oxy", "PAX OXY MASKS", "PUSH_OVHD_OXYGEN_PASSENGER", "fault_white", "on_white", "AC_ESS", "test", "test"),
    ("rcdr_gnd_ctl", "RCDR GND CTL", "PUSH_OVHD_RCDR_GNDCTL", None, "on", "DC_ESS", None, "test"),
    ("remote_cb_ctl", "REMOTE C/B CTL", "PUSH_OVHD_REMO_GNDCTL", None, "on_white", "AC1", None, "test"),
    ("svge_int_ovrd", "SVGE INT OVRD", "PUSH_OVHD_SVGEINT", None, "on_white", "AC1", None, "test"),
    ("emer_exit_lt_off", "EMER EXIT LT OFF", "PUSH_OVHD_SIGNS", None, "off_red", "AC_ESS", None, "test"),
    ("cockpit_door_video", "COCKPIT DOOR VIDEO", "PUSH_OVHD_COCKPITDOORVIDEO", None, "off", "AC1", None, "test"),
    ("nss_avncs", "NSS DATA TO AVNCS", "PUSH_OVHD_NSSAVNCS", None, "off", "AC1", None, "test"),
    ("nss_master", "NSS MASTER", "PUSH_OVHD_NSS_MASTER", "smoke", "off", "AC1", "test", "test"),
    ("ent_cws", "CWS", "PUSH_OVHD_ENT_CWS", "smoke", "off", "AC1", "test", "test"),
    ("ent_ifec", "IFEC", "PUSH_OVHD_ENT_IFEC", "smoke", "off", "AC1", "test", "test"),
    ("gatelink", "GATELINK", "PUSH_OVHD_GATELINK", None, "on_white", ANY_DC, None, "test"),
    ("cab_data_to_nss", "CABIN DATA TO NSS", "PUSH_OVHD_CAB_DATA_TO_NSS", None, "on_white", ANY_DC, None, "test"),
)
SEQ_POSITION = {1: "upper", 2: "lower"}

# Pushbutton position (dark cockpit, 2026-11-03). A legend whose system is not simulated used to light in
# ANN LT TEST only, so nothing happened when the pushbutton was used. Such a pushbutton now has its own
# latched position (a380x/xp12/ovhd/pb/<id>, toggled by the same-named command, 1 = pressed in) and its
# legends follow it like FBW's FBW_Push_Toggle: an OFF legend lights while the pushbutton is released,
# an ON / MAN / OVRD / DISC / DISCH legend while it is pressed in, an X FEED / JETTISON OPEN legend while
# it is pressed in. FAULT, SMOKE, SQUIB, CALL, EVAC and BTL legends keep their conditions. Defaults are the
# Airbus normal positions, so a normal cockpit is dark: OFF-type pushbuttons in, ON-type out.
PB_OFF_LEGENDS = ("off", "off_red")
PB_UPPER_ON_LEGENDS = ("open",)


def effective_sources(row: tuple) -> tuple[str | None, str | None]:
    _pid, _caption, _node, upper, lower, _supply, src1, src2 = row
    if src2 == "test" and lower:
        src2 = "pb_off" if lower in PB_OFF_LEGENDS else "pb_on"
    if src1 == "test" and upper in PB_UPPER_ON_LEGENDS:
        src1 = "pb_on"
    return src1, src2


def pb_state_ids() -> tuple[str, ...]:
    return tuple(r[0] for r in PUSHBUTTONS if any(s in ("pb_off", "pb_on") for s in effective_sources(r)))


def pb_default(pid: str) -> int:
    return 1 if effective_sources(row(pid))[1] == "pb_off" else 0
# Click targets added with the lamps (pushbuttons the port had no 3D click for).
CLICKS = (("crew_oxy", "sim/oxy/crew_valve_toggle", "CREW OXY SUPPLY - ON / OFF"),)
ANN_LT_NODE = "SWITCH_OVHD_INTLT_ANNLT"


def row(pid: str) -> tuple:
    return next(r for r in PUSHBUTTONS if r[0] == pid)


def seq_node(node: str, sequence: int) -> str:
    return f"{node}_SEQ{sequence}"


def lamps() -> tuple[tuple[str, str, int, str, str], ...]:
    """(lamp dataref, pushbutton id, SEQ number, legend, authored legend node)."""
    rows = []
    for pid, _caption, node, upper, lower, *_rest in PUSHBUTTONS:
        for sequence, legend in ((1, upper), (2, lower)):
            if legend:
                rows.append((f"{PREFIX}lamp/{pid}_{SEQ_POSITION[sequence]}", pid, sequence, legend,
                             seq_node(node, sequence)))
    return tuple(rows)


def fault_ids() -> tuple[str, ...]:
    """Pushbuttons with a FAULT legend: writable fail/<id> input."""
    return tuple(r[0] for r in PUSHBUTTONS if r[3] in ("fault", "fault_white"))


def parse_supply(token: str) -> tuple[tuple[str, ...], tuple[str, ...]]:
    """'A|B' -> any of (A, B); 'A&B' -> all of (A, B)."""
    if "&" in token:
        names = tuple(token.split("&"))
        return (), names
    return tuple(token.split("|")), ()


def parse_source(token: str | None) -> tuple[str, int]:
    if token is None:
        return "none", 0
    kind, _sep, index = token.partition(":")
    if kind not in SOURCES:
        raise ValueError(f"unknown lamp condition {token}")
    return kind, int(index or 0)


COMMANDS = (PREFIX + "ann_lt_up", PREFIX + "ann_lt_down", *(PREFIX + f"pb/{pid}" for pid in pb_state_ids()))
DATAREFS = (
    *(row_[0] for row_ in lamps()),
    *(PREFIX + f"fail/{pid}" for pid in fault_ids()),
    PREFIX + "ann_lt", PREFIX + "ann_lt_switch", PREFIX + "settings/lamp_supply", PREFIX + "lamp_count",
    *(PREFIX + f"pb/{pid}" for pid in pb_state_ids()),
)


def emit_c_table() -> str:
    """Native table a380x_ovhd_lamps_table.h (generated, checked by native/avionics/build.py)."""
    legends = list(LEGENDS)
    out = ["/* SPDX-License-Identifier: GPL-3.0-or-later",
           " * GENERATED by tools/ovhd_lamps_contract.py emit_c_table(); do not edit by hand.",
           " * native/avionics/build.py fails when this file differs from the contract. */",
           "#ifndef A380X_OVHD_LAMPS_TABLE_H", "#define A380X_OVHD_LAMPS_TABLE_H", "",
           "enum {", "    A380X_OVHD_LEGEND_NONE,"]
    out += [f"    A380X_OVHD_LEGEND_{key.upper()}," for key in legends]
    out += ["    A380X_OVHD_LEGEND_COUNT", "};",
            "static const char *const a380x_ovhd_legend_text[A380X_OVHD_LEGEND_COUNT] = {",
            "    \"\", " + ", ".join(f'"{LEGENDS[k][0]}"' for k in legends), "};", "", "enum {"]
    out += [f"    A380X_OVHD_SRC_{s.upper()}," for s in ("none", *SOURCES)]
    out += ["    A380X_OVHD_SRC_COUNT", "};", "", "enum {"]
    out += [f"    A380X_OVHD_PB_{r[0].upper()}," for r in PUSHBUTTONS]
    out += ["    A380X_OVHD_PB_COUNT", "};", "",
            "static const A380XOvhdLampInfo a380x_ovhd_pbs[A380X_OVHD_PB_COUNT] = {"]

    def mask(names):
        return " | ".join(f"A380X_ELN_BIT({BUSES[n]})" for n in names) or "0u"

    for pid, _caption, node, upper, lower, supply, src1, src2 in PUSHBUTTONS:
        src1, src2 = effective_sources((pid, _caption, node, upper, lower, supply, src1, src2))
        any_of, all_of = parse_supply(supply)
        k1, i1 = parse_source(src1)
        k2, i2 = parse_source(src2)
        leg = lambda key: f"A380X_OVHD_LEGEND_{key.upper()}" if key else "A380X_OVHD_LEGEND_NONE"
        out.append(f'    {{"{pid}", "{node}", {{{leg(upper)}, {leg(lower)}}}, '
                   f'{{A380X_OVHD_SRC_{k1.upper()}, A380X_OVHD_SRC_{k2.upper()}}}, {{{i1}, {i2}}}, '
                   f'{mask(any_of)}, {mask(all_of)}}},')
    out += ["};", "#endif", ""]
    return "\n".join(out)


if __name__ == "__main__":
    import sys
    if sys.argv[1:] == ["--emit-c"]:
        sys.stdout.write(emit_c_table())
    else:
        print(f"{len(PUSHBUTTONS)} pushbuttons, {len(lamps())} lamps, {len(DATAREFS)} datarefs, "
              f"{len(COMMANDS)} commands")
