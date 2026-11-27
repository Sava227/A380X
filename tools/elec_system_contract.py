"""Single source of the A380 electrical network (phase 2) dataref / command contract.

The native adapter (native/avionics/a380x_elec_adapter.inc) registers exactly these names;
native/avionics/build.py compares its mock-registered contract (test_elec_adapter --contracts)
against this module, and tools/avionics_provider.py lists them in the provider manifest.
The ELEC panel names (pb/, lamp/, ext/connected, fail/<pb id>, ...) stay in
tools/elec_panel_contract.py. See docs/ELEC_SYSTEM.md.
"""
from __future__ import annotations

PREFIX = "a380x/xp12/elec/"
APU_PREFIX = "a380x/xp12/apu/"

BUSES = ("ac1", "ac2", "ac3", "ac4", "ac_ess", "ac_emer", "ac_eha", "ac_gnd_svc",
         "dc1", "dc2", "dc_ess", "dc_ess_inflt", "dc_eha", "dc_apu", "apu_start",
         "hot1", "hot2", "hot_ess", "hot_apu", "dc_gnd_svc")
CONTACTORS = ("990XU1", "990XU2", "990XU3", "990XU4", "990XG1", "990XG2", "990XG3", "990XG4",
              "990XS1", "990XS2", "980XU1", "980XU2", "980XU3", "980XU4", "980XU5", "980XU6",
              "900XU", "3XC1", "3XC2", "5XE", "3XB_1", "3XB_2", "911XN", "911XH", "991PU6", "991PU2",
              "990PU1", "990PU2", "6PE", "7PU", "990PB1", "990PB2", "6PC1", "6PB3", "5PB",
              "6PC2", "980PC", "7XB", "970PN2", "14PH", "990PX", "970PN", "10PH", "10KA_5KA")
DISPLAY_UNITS = ("capt_pfd", "capt_nd", "capt_mfd", "fo_pfd", "fo_nd", "fo_mfd", "ewd", "sd",
                 "capt_oit", "fo_oit")
FAILURES = ("tr1", "tr2", "tr_ess", "tr_apu", "stat_inv", "ac_bus1", "ac_bus2", "ac_bus3", "ac_bus4",
            "dc_bus1", "dc_bus2", "dc_ess_bus", "ext1", "ext2", "ext3", "ext4", "galley")

DATAREFS = (
    *(PREFIX + f"bus/{bus}_{kind}" for bus in BUSES for kind in ("powered", "volts")),
    *(PREFIX + f"contactor/{contactor}" for contactor in CONTACTORS),
    *(PREFIX + name for name in (
        "gen/potential", "gen/freq", "gen/load_pct", "gen/line_closed", "drive/connected",
        "apu_gen/potential", "apu_gen/freq", "apu_gen/load_pct", "ext/potential", "ext/freq",
        "emer_gen/potential", "emer_gen/load_pct", "rat/stow_ratio", "stat_inv/potential", "stat_inv/freq",
        "bat/potential", "bat/current", "bat/charge_ah", "bat/selector", "lights/nose_switch", "bat/display_volts",
        "tr/potential", "tr/current", "emer/active", "emer/tefo", "emer/config", "galley_shed")),
    *(PREFIX + f"du/{unit}_state" for unit in DISPLAY_UNITS),
    *(PREFIX + name for name in (
        "du/isis_powered", "spawn_mode", "settings/default_spawn", "settings/du_selftest_s",
        "network_active", "ac_main_powered")),
    *(PREFIX + f"fail/{failure}" for failure in FAILURES),
    *(PREFIX + f"preset/{name}" for name in ("active", "step", "step_count", "step_name", "expedited", "done")),
    *(APU_PREFIX + name for name in ("master_on", "start_on", "avail", "flap_open_ratio", "ecb_powered",
                                     "generator_volts", "generator_hz", "generator_load_percent",
                                     "n_percent", "n2_percent")),   # FBW PW980 N / N2 (a380x_apu_pw980.h)
)
COMMANDS = (
    *(PREFIX + f"preset/{n}" for n in range(1, 6)),
    PREFIX + "preset/cancel", PREFIX + "preset/expedite_toggle",
    PREFIX + "bat_selector_left", PREFIX + "bat_selector_right",
    PREFIX + "lights/nose_up", PREFIX + "lights/nose_down",   # NOSE switch OFF / TAXI / T.O.
    APU_PREFIX + "start",
)
# Stock commands consumed while the network is active (hardware bindings act on native pbs).
INTERCEPTED_COMMANDS = (
    *(f"sim/electrical/battery_{n}_{action}" for n in (1, 2) for action in ("on", "off", "toggle")),
    *(f"sim/electrical/generator_{n}_{action}" for n in range(1, 5) for action in ("on", "off", "toggle")),
    "sim/electrical/generators_toggle", "sim/electrical/APU_generator_on", "sim/electrical/APU_generator_off",
    "sim/electrical/GPU_on", "sim/electrical/GPU_off", "sim/electrical/GPU_toggle", "sim/electrical/APU_start",
    "sim/systems/avionics_on", "sim/systems/avionics_off", "sim/systems/avionics_toggle",
)
