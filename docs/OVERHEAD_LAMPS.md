# Overhead annunciator lamps

> **Pushbutton positions / dark cockpit (2026-11-03):** the 104 pushbuttons whose system is not simulated
> used to light only in ANN LT TEST, so nothing happened when they were used. Each now has a latched
> position `a380x/xp12/ovhd/pb/<id>` (1 = pressed in), toggled by the command of the same name and clicked
> on its 3D cap (press travel from `tools/cockpit_feedback.py`). Their legends follow it as in FBW's
> FBW_Push_Toggle: OFF lights while released; ON / MAN / OVRD / DISC / DISCH and X FEED / JETTISON OPEN
> while pressed in. FAULT, SMOKE, SQUIB, CALL, EVAC and BTL keep their conditions. Defaults are the Airbus
> normal positions (OFF-type pressed in, ON-type released), and on aircraft load the fuel pumps, engine
> bleeds and packs are set ON, so a normal cockpit is dark instead of showing OFF on every pump and bleed.
> In the tables below, "ANN LT TEST only" lower legends now read "pushbutton position".

> **Default supply (user decision, 2026-10-21):** `a380x/xp12/ovhd/settings/lamp_supply` defaults to **1**:
> every overhead legend (ELEC included) lights as soon as any battery is AUTO or any DC bus is live,
> so OFF legends show with BAT 1 + BAT 2. Set it to 0 for FBW's per-legend supply buses.

Every authored legend of every overhead pushbutton is now a lamp: the ELEC / EMER ELEC PWR panel
([ELEC_PANEL.md](ELEC_PANEL.md)), the ADIRS panel ([ADIRS.md](ADIRS.md)) and, new, the 127 other
overhead pushbuttons (217 legends): APU, air conditioning and bleed, cabin pressure, anti-ice,
fuel, hydraulics, fire agents and cargo smoke, engine FADEC / manual start, flight controls
PRIM / SEC, calls, evacuation, oxygen, recorders, cabin/NSS. Each legend follows FlyByWire's
A380X cockpit behaviour: lit when its condition is true (or ANN LT is in TEST) **and** its own
supply bus is powered, at full brightness with ANN LT BRT and 0.3 with DIM.

| Part | File |
| --- | --- |
| Contract (rows, legends, supply, conditions, names) | `tools/ovhd_lamps_contract.py` |
| Lamp logic (XPLM-free) and generated table | `native/avionics/a380x_ovhd_lamps.h`, `a380x_ovhd_lamps_table.h` (`python3 tools/ovhd_lamps_contract.py --emit-c`) |
| X-Plane adapter | `native/avionics/a380x_ovhd_lamps.inc` |
| Cockpit patch (lamp quads, click targets, ANN LT lever, atlas cells) | `tools/ovhd_cockpit_lamps.py` |
| Tests | `native/avionics/test_ovhd_lamps.c`, `test_ovhd_lamps_adapter.c`, `tools/test_ovhd_cockpit_lamps.py` |
| Offline previews | `tools/preview_ovhd_lamps.py` |

## Lit legends still invisible: the decals are drawn after the lamps (2026-11-04)

Second in-sim check with the Log.txt diagnostics: the lamp datarefs were 1.0 (PACK 1 OFF, APU MASTER
ON) yet nothing glowed. The quads sit 0.4 mm in front of their decal with nothing drawn in front of them
(checked with every animation applied), the DDS atlas keeps its alpha, and the UVs cover the legend cell.
What differs from the FCU lamps that work is draw order: every legend decal object (`decal_*.obj`,
_obja 120-125) is drawn after `a380x_cockpit_controls.obj` (119), and the lamp quads carry
`ATTR_poly_os`, so the later decal still painted over them (the 2026-10-22 alpha-test fix did not
change this in the sim).

`tools/hide_lit_legend_decals.py` wraps each decal face that has a lamp quad in `ANIM_hide` on that
lamp's dataref above its dark threshold: while lit, the quad (legend art plus glow) replaces the decal;
while dark the decal shows as before. 360 of 364 lamp faces (overhead, ELEC, ADIRS, glareshield / gear,
cockpit-feedback lamps; faces exported in node-local frames matched at their rest pose); the ENG 2
BLEED upper face, which covers the whole cap, hides for either legend. Run it after any tool that adds
lamp quads. Idempotent.

## Why the lamps were still invisible in the sim (2026-10-22, X-Plane 12.4.4)

First in-simulator check (cold & dark, BAT 1/2/ESS/APU AUTO, DC ESS live): the lamp datarefs were
right (`ovhd/lamp/eng1_bleed_lower` = `pack1_lower` = 1.0, ANN LT TEST lit every legend in the
datarefs), yet no ELEC, overhead or ADIRS legend was drawn; the decal's grey "OFF" stayed visible.
Every invisible quad sits 0.4 mm in front of `decal_a380_pushbutton_decal.obj` (_obja 121), which X-Plane
draws after `a380x_cockpit_controls.obj` (_obja 119); the quads were alpha-blended and blended geometry
writes no depth, so the later opaque decal painted over them. The FCU lamps that work (AP1/AP2/FD/A/THR,
LOC, ALT, APPR, LS/VV) are backed by `mip_knob01`/`mip03`, drawn earlier.

`tools/patch_lamp_depth.py` (applied in place, idempotent, `# A380X LAMP DEPTH V1`) draws the 297
decal-backed quads (217 overhead, 45 ELEC, 12 ADIRS, 23 glareshield/gear: MASTER WARN/CAUT, AUTOLAND,
ATC MSG, SIDESTICK PRIORITY, gear lights, AUTO BRK, RTO) alpha-tested opaque (`ATTR_no_blend 0.5`), so
they write depth and the decal is rejected behind the glyphs. It also animates the APU MASTER SW decal
faces with the cap travel (`# A380X APU MASTER DECAL MOTION V1`): the static decal hid the ON lamp as
soon as the pushbutton latched. `elec_cockpit_panel.py` / `ovhd_cockpit_lamps.py` now generate the opaque
form. Test: `tools/test_patch_lamp_depth.py`.

## Why no OFF light was visible (2026-10-21 report)

*"The lights on the overhead buttons don't work, it doesn't say OFF. When 2 batteries are on, OFF
should display; with the APU the same."*

1. **Only the ELEC (and ADIRS) pushbuttons had lamps.** APU MASTER SW, APU START, APU BLEED,
   the bleeds, packs, anti-ice, fuel pumps and every other overhead pushbutton had no lamp
   geometry and no lamp dataref at all, so they could never show OFF, ON or AVAIL, whatever the
   power. This is fixed by this module.
2. **The ELEC legends follow FBW's per-legend supply** (spec 4.2): BAT 1, ESS, GEN 1/2, BUS TIE,
   AC ESS FEED, PAX SYS, EXT ON and DRIVE are on DC ESS; BAT 2, APU BAT, GEN 3/4, APU GEN B,
   GALLEY, COMMERCIAL 2 and ELMU on DC 2. On batteries the only DC bus is DC ESS, and FBW's
   emergency line contactor needs **BAT 1 and ESS** (the two left battery pushbuttons):
   * BAT 1 + BAT 2 only: DC ESS stays dead (FBW `direct_current.rs`: `bat_is_auto(1) && bat_is_auto(3)`),
     so every legend is dark, in FBW as in the port.
   * BAT 1 + ESS: DC ESS is live; GEN 1/2 FAULT light and the OFF of any DC ESS pushbutton that is
     OFF, but the BAT 2 / APU BAT OFF legends are on DC 2, which needs AC (FBW
     `A380_Cockpit_Behavior.xml` `SEQ_POWERED` of `PUSH_OVHD_ELEC_BAT2`/`_BATAPU`), so they stay dark.
   * APU MASTER SW ON is on DC APU (309PP, FBW): it needs the APU BAT pushbutton.

   The FBW template (`FBW_Push_Toggle`, `A32NX_Interior_Generics.xml`) is
   `SEQn_EMISSIVE = (code || ANN LT TEST) && SEQn_POWERED && SEQ_POWERED`; pushbuttons without
   `SEQ_POWERED` default to "always powered" in FBW and to any of DC ESS / DC 1 / DC 2 here.
   The rendering path (`ATTR_light_level 0 1 <float dataref>`, `ANIM_hide` while dark, the
   `fcu_labels_LIT.png` cell, datarefs registered in `XPluginStart` before the aircraft OBJs
   load) is the one the FCU / glareshield / ADIRS lamps use; the lit cells contain the legend
   pixels in both the albedo and the LIT atlas (checked), so a lit legend shows its coloured
   text in daylight (albedo, lit by the cockpit light) plus the additive glow.

   For users who expect the A320-style "OFF lights with any battery on", an opt-in setting
   `a380x/xp12/ovhd/settings/lamp_supply = 1` (not FBW) powers every overhead legend, ELEC panel
   included, whenever any DC bus is live or any battery pushbutton is AUTO (BAT 1 + BAT 2 then show
   ESS / APU BAT OFF, fuel pump OFF, ...). The default 0 is FBW.

### Which legends can light on each power state (FBW supply)

| Power | Lamp buses live | Overhead legends that can light |
| --- | --- | --- |
| Cold & dark (hot buses) | none | none, not even in ANN LT TEST |
| BAT 1 + BAT 2 | none | none (DC ESS needs BAT 1 **and** ESS) |
| BAT 1 + ESS (+ BAT 2) | DC ESS | ELEC DC ESS legends (GEN 1/2 FAULT, BAT 1/ESS, BUS TIE, AC ESS FEED, PAX SYS, EXT ON, DRIVE, RAT & EMER GEN), ENG 2/3 BLEED, APU BLEED, RAM AIR, FADEC GND PWR, RCDR GND CTL, PRIM/SEC, the "any DC" pushbuttons |
| all four BAT | DC ESS, DC APU | the above + APU MASTER SW, APU START (DC APU) |
| EXT 2 (or APU GEN, engine GEN) | AC 1-2/AC ESS/DC 1-2/DC ESS (AC 3-4 with EXT 3) | everything: fuel pumps, anti-ice, hydraulics, PACKs, fire agents, ... |

## ANN LT (TEST / BRT / DIM)

The overhead INT LT ANN LT switch is now a working control: click its upper half (or the
`a380x/xp12/ovhd/ann_lt_up` command) toward TEST, the lower half (`ann_lt_down`) toward DIM; the
lever rotates about its authored FBW hinge (`a380x/xp12/ovhd/ann_lt_switch`, 0 TEST, 0.5 BRT,
1 DIM, 0.12 s full travel). `a380x/xp12/ovhd/ann_lt` (writable int, 0 TEST, 1 BRT, 2 DIM) is the same
state as `a380x/xp12/elec/ann_lt`. TEST lights every authored legend whose bus is powered (FBW),
including the ELEC panel and the ADIRS IR/ADR OFF and FAULT legends of a powered ADIRU. DIM = 0.3.
Aircraft (re)load sets BRT.

## Lamps

Names start with `a380x/xp12/ovhd/`. `lamp/<id>_upper` / `lamp/<id>_lower` are the SEQ1 / SEQ2
legends (float 0..1). `fail/<id>` (writable int) injects the FAULT of a pushbutton with a FAULT
legend. `lamp_count` is the number of lit overhead legends (diagnostic, excludes ELEC/ADIRS).
"ANN LT TEST only" means the system behind the pushbutton is not simulated: in its FBW normal
position the legend is dark, and only the test lights it. Conditions use the port's native state
or the X-Plane dataref the pushbutton drives (`config/cockpit_controls.json`). The X-Plane failure
mapping of the FAULT legends (APU, bleed, inlet / surface heat) is a port choice; FBW sets those
from its own system models.

| Pushbutton | id | Upper (SEQ1) | condition | Lower (SEQ2) | condition | Lamp bus |
| --- | --- | --- | --- | --- | --- | --- |
| APU MASTER SW | `apu_master` | FAULT (amber) | X-Plane `rel_apu`/`rel_apu_fire` with MASTER on, or `fail/<id>` | ON (blue) | MASTER SW on (`APU_starter_switch` ≥ 1) | DC APU |
| APU START | `apu_start` | AVAIL (green) | native ECB AVAIL | ON (blue) | native ECB start latched, not AVAIL | AC ESS or DC APU |
| ENG 1 BLEED | `eng1_bleed` | FAULT (amber) | `rel_bleed_air_lft/rgt` with the valve selected, or `fail/<id>` | OFF (white) | `engine_bleed_sov[0]` = 0 | DC 2 |
| ENG 2 BLEED | `eng2_bleed` | FAULT (amber) | `rel_bleed_air_lft/rgt` with the valve selected, or `fail/<id>` | OFF (white) | `engine_bleed_sov[1]` = 0 | DC ESS |
| ENG 3 BLEED | `eng3_bleed` | FAULT (amber) | `rel_bleed_air_lft/rgt` with the valve selected, or `fail/<id>` | OFF (white) | `engine_bleed_sov[2]` = 0 | DC ESS |
| ENG 4 BLEED | `eng4_bleed` | FAULT (amber) | `rel_bleed_air_lft/rgt` with the valve selected, or `fail/<id>` | OFF (white) | `engine_bleed_sov[3]` = 0 | DC 2 |
| APU BLEED | `apu_bleed` | FAULT (amber) | `fail/<id>` input | ON (blue) | `bleedair/actuators/apu_bleed` | DC ESS |
| PACK 1 | `pack1` | FAULT (amber) | `fail/<id>` input | OFF (white) | `pack_left/right` = 0 | DC 1 and DC ESS |
| PACK 2 | `pack2` | FAULT (amber) | `fail/<id>` input | OFF (white) | `pack_left/right` = 0 | DC 2 |
| HOT AIR 1 | `hot_air1` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | DC 1 and DC ESS |
| HOT AIR 2 | `hot_air2` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | DC 1 and DC ESS |
| RAM AIR | `ram_air` | — |  | ON (white) | ANN LT TEST only | DC ESS |
| FWD CARGO ISOL VALVES | `cargo_fwd_isol` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC ESS |
| BULK CARGO ISOL VALVES | `cargo_bulk_isol` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC ESS |
| CARGO HEATER | `cargo_heater` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC ESS |
| CAB AIR EXTRACT | `cab_air_extract` | FAULT (amber) | `fail/<id>` input | OVRD (white) | ANN LT TEST only | AC ESS |
| VENT EXTRACT | `vent_extract` | FAULT (amber) | `fail/<id>` input | OVRD (white) | ANN LT TEST only | AC ESS |
| VENT SMOKE | `vent_smoke` | SMOKE (red) | ANN LT TEST only | — |  | AC ESS |
| CAB FANS | `cab_fans` | — |  | OFF (white) | ANN LT TEST only | AC 1 |
| VENT COOLG | `vent_coolg` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 1 |
| AVNCS GND COOLG | `avncs_gnd_coolg` | — |  | ON (white) | ANN LT TEST only | DC ESS or DC 1 or DC 2 |
| OVHT COND FANS | `ovht_cond_fans` | — |  | ON (white) | ANN LT TEST only | DC ESS or DC 1 or DC 2 |
| CABIN PRESS MAN ALT | `press_man_alt` | — |  | MAN (white) | ANN LT TEST only | AC ESS |
| CABIN PRESS MAN V/S | `press_man_vs` | — |  | MAN (white) | ANN LT TEST only | AC ESS |
| DITCHING | `ditching` | — |  | ON (white) | ANN LT TEST only | AC ESS |
| ENG 1 ANTI ICE | `eng1_anti_ice` | FAULT (amber) | `rel_ice_inlet_heat1` with ENG ANTI ICE on, or `fail/<id>` | ON (blue) | `ice_inlet_heat_on_per_engine[0]` | AC 1 |
| ENG 2 ANTI ICE | `eng2_anti_ice` | FAULT (amber) | `rel_ice_inlet_heat2` with ENG ANTI ICE on, or `fail/<id>` | ON (blue) | `ice_inlet_heat_on_per_engine[1]` | AC 2 |
| ENG 3 ANTI ICE | `eng3_anti_ice` | FAULT (amber) | `rel_ice_inlet_heat3` with ENG ANTI ICE on, or `fail/<id>` | ON (blue) | `ice_inlet_heat_on_per_engine[2]` | AC 1 |
| ENG 4 ANTI ICE | `eng4_anti_ice` | FAULT (amber) | `rel_ice_inlet_heat4` with ENG ANTI ICE on, or `fail/<id>` | ON (blue) | `ice_inlet_heat_on_per_engine[3]` | AC 2 |
| WING ANTI ICE | `wing_anti_ice` | FAULT (amber) | `rel_ice_surf_heat` with WING ANTI ICE on, or `fail/<id>` | ON (blue) | `ice_surface_hot_bleed_air_on` | AC ESS |
| PROBES & WINDOW HEAT | `probes_window_heat` | — |  | ON (blue) | `ice_window_heat_on` | AC 2 |
| FEED TK 1 MAIN | `feed1_main` | FAULT (amber) | `fail/<id>` input | OFF (white) | `fuel_pump_on[0]` = 0 | AC ESS |
| FEED TK 2 MAIN | `feed2_main` | FAULT (amber) | `fail/<id>` input | OFF (white) | `fuel_pump_on[1]` = 0 | AC 2 |
| FEED TK 3 MAIN | `feed3_main` | FAULT (amber) | `fail/<id>` input | OFF (white) | `fuel_pump_on[2]` = 0 | AC 2 |
| FEED TK 4 MAIN | `feed4_main` | FAULT (amber) | `fail/<id>` input | OFF (white) | `fuel_pump_on[3]` = 0 | AC 2 |
| FEED TK 1 STBY | `feed1_stby` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| FEED TK 2 STBY | `feed2_stby` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 1 |
| FEED TK 3 STBY | `feed3_stby` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 1 |
| FEED TK 4 STBY | `feed4_stby` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 1 |
| L OUTR TK PUMP | `outer_tk_left` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC ESS |
| L MID TK FWD | `mid_tk_left_fwd` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| L MID TK AFT | `mid_tk_left_aft` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| L INR TK FWD | `inr_tk_left_fwd` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| L INR TK AFT | `inr_tk_left_aft` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| R INR TK FWD | `inr_tk_right_fwd` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| R INR TK AFT | `inr_tk_right_aft` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| R MID TK FWD | `mid_tk_right_fwd` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| R MID TK AFT | `mid_tk_right_aft` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| R OUTR TK PUMP | `outer_tk_right` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC ESS |
| TRIM TK L | `trim_tk_left` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| TRIM TK R | `trim_tk_right` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| X FEED 1 | `xfeed1` | OPEN (green) | ANN LT TEST only | ON (white) | ANN LT TEST only | AC ESS |
| X FEED 2 | `xfeed2` | OPEN (green) | ANN LT TEST only | ON (white) | ANN LT TEST only | AC ESS |
| X FEED 3 | `xfeed3` | OPEN (green) | ANN LT TEST only | ON (white) | ANN LT TEST only | AC ESS |
| X FEED 4 | `xfeed4` | OPEN (green) | ANN LT TEST only | ON (white) | ANN LT TEST only | AC ESS |
| OUTR TK XFR | `outer_tk_xfr` | FAULT (amber) | `fail/<id>` input | MAN (white) | ANN LT TEST only | AC ESS |
| MID TK XFR | `mid_tk_xfr` | FAULT (amber) | `fail/<id>` input | MAN (white) | ANN LT TEST only | AC ESS |
| INR TK XFR | `inr_tk_xfr` | FAULT (amber) | `fail/<id>` input | MAN (white) | ANN LT TEST only | AC ESS |
| TRIM TK XFR | `trim_tk_xfr` | FAULT (amber) | `fail/<id>` input | MAN (white) | ANN LT TEST only | AC ESS |
| EMER OUTR TK XFR | `emer_outr_xfr` | — |  | ON (white) | ANN LT TEST only | AC 1 |
| JETTISON ARM | `jettison_arm` | — |  | ON (white) | ANN LT TEST only | AC 1 |
| JETTISON ACTIVE | `jettison_active` | OPEN (green) | ANN LT TEST only | ON (white) | ANN LT TEST only | AC 1 |
| REFUEL | `refuel` | — |  | ON (white) | ANN LT TEST only | DC ESS or DC 1 or DC 2 |
| AUTO GND XFR | `auto_gnd_xfr` | — |  | ON (white) | ANN LT TEST only | DC ESS or DC 1 or DC 2 |
| ENG 1 PUMP A | `hyd_eng1a` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 1 |
| ENG 1 PUMP B | `hyd_eng1b` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 1 |
| ENG 2 PUMP A | `hyd_eng2a` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| ENG 2 PUMP B | `hyd_eng2b` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| ENG 3 PUMP A | `hyd_eng3a` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 1 |
| ENG 3 PUMP B | `hyd_eng3b` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 1 |
| ENG 4 PUMP A | `hyd_eng4a` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| ENG 4 PUMP B | `hyd_eng4b` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| ENG 1 PUMP DISC | `hyd_eng1_disc` | FAULT (amber) | `fail/<id>` input | DISC (white) | ANN LT TEST only | AC 2 |
| ENG 2 PUMP DISC | `hyd_eng2_disc` | FAULT (amber) | `fail/<id>` input | DISC (white) | ANN LT TEST only | AC 2 |
| ENG 3 PUMP DISC | `hyd_eng3_disc` | FAULT (amber) | `fail/<id>` input | DISC (white) | ANN LT TEST only | AC 2 |
| ENG 4 PUMP DISC | `hyd_eng4_disc` | FAULT (amber) | `fail/<id>` input | DISC (white) | ANN LT TEST only | AC 2 |
| GREEN ELEC PUMP A ON | `hyd_g_elec_a_on` | — |  | ON (blue) | ANN LT TEST only | AC ESS |
| GREEN ELEC PUMP B ON | `hyd_g_elec_b_on` | — |  | ON (blue) | ANN LT TEST only | AC ESS |
| YELLOW ELEC PUMP A ON | `hyd_y_elec_a_on` | — |  | ON (blue) | ANN LT TEST only | AC ESS |
| YELLOW ELEC PUMP B ON | `hyd_y_elec_b_on` | — |  | ON (blue) | ANN LT TEST only | AC ESS |
| GREEN ELEC PUMP A | `hyd_g_elec_a` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| GREEN ELEC PUMP B | `hyd_g_elec_b` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| YELLOW ELEC PUMP A | `hyd_y_elec_a` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| YELLOW ELEC PUMP B | `hyd_y_elec_b` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | AC 2 |
| ENG 1 AGENT 1 | `fire_eng1_agent1` | SQUIB (white) | ANN LT TEST only | DISCH (amber) | ANN LT TEST only | AC ESS |
| ENG 1 AGENT 2 | `fire_eng1_agent2` | SQUIB (white) | ANN LT TEST only | DISCH (amber) | ANN LT TEST only | AC 2 |
| ENG 2 AGENT 1 | `fire_eng2_agent1` | SQUIB (white) | ANN LT TEST only | DISCH (amber) | ANN LT TEST only | AC 2 |
| ENG 2 AGENT 2 | `fire_eng2_agent2` | SQUIB (white) | ANN LT TEST only | DISCH (amber) | ANN LT TEST only | AC ESS |
| ENG 3 AGENT 1 | `fire_eng3_agent1` | SQUIB (white) | ANN LT TEST only | DISCH (amber) | ANN LT TEST only | AC ESS |
| ENG 3 AGENT 2 | `fire_eng3_agent2` | SQUIB (white) | ANN LT TEST only | DISCH (amber) | ANN LT TEST only | AC 2 |
| ENG 4 AGENT 1 | `fire_eng4_agent1` | SQUIB (white) | ANN LT TEST only | DISCH (amber) | ANN LT TEST only | AC 2 |
| ENG 4 AGENT 2 | `fire_eng4_agent2` | SQUIB (white) | ANN LT TEST only | DISCH (amber) | ANN LT TEST only | AC ESS |
| APU AGENT | `fire_apu_agent` | SQUIB (white) | ANN LT TEST only | DISCH (amber) | ANN LT TEST only | AC ESS |
| CARGO SMOKE FWD DISCH | `cargo_smoke_fwd` | SMOKE (red) | ANN LT TEST only | DISCH (white) | ANN LT TEST only | AC ESS |
| CARGO SMOKE AFT DISCH | `cargo_smoke_aft` | SMOKE (red) | ANN LT TEST only | DISCH (white) | ANN LT TEST only | AC ESS |
| CARGO SMOKE BTL 1 | `cargo_btl1` | BTL 1 (amber) | ANN LT TEST only | — |  | DC ESS or DC 1 or DC 2 |
| CARGO SMOKE BTL 2 | `cargo_btl2` | BTL 2 (amber) | ANN LT TEST only | — |  | DC ESS or DC 1 or DC 2 |
| FADEC GND PWR 1 | `fadec1` | — |  | ON (white) | ANN LT TEST only | DC ESS |
| FADEC GND PWR 2 | `fadec2` | — |  | ON (white) | ANN LT TEST only | DC ESS |
| FADEC GND PWR 3 | `fadec3` | — |  | ON (white) | ANN LT TEST only | DC ESS |
| FADEC GND PWR 4 | `fadec4` | — |  | ON (white) | ANN LT TEST only | DC ESS |
| ENG 1 MAN START | `man_start1` | — |  | ON (blue) | ANN LT TEST only | AC ESS |
| ENG 2 MAN START | `man_start2` | — |  | ON (blue) | ANN LT TEST only | AC ESS |
| ENG 3 MAN START | `man_start3` | — |  | ON (blue) | ANN LT TEST only | AC ESS |
| ENG 4 MAN START | `man_start4` | — |  | ON (blue) | ANN LT TEST only | AC ESS |
| ENG MAN START ALTN | `man_start_altn` | — |  | ON (blue) | ANN LT TEST only | AC ESS |
| PRIM 1 | `prim1` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | DC ESS or DC 1 or DC 2 |
| PRIM 2 | `prim2` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | DC ESS or DC 1 or DC 2 |
| PRIM 3 | `prim3` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | DC ESS or DC 1 or DC 2 |
| SEC 1 | `sec1` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | DC ESS or DC 1 or DC 2 |
| SEC 2 | `sec2` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | DC ESS or DC 1 or DC 2 |
| SEC 3 | `sec3` | FAULT (amber) | `fail/<id>` input | OFF (white) | ANN LT TEST only | DC ESS or DC 1 or DC 2 |
| CALLS EMER | `calls_emer` | CALL (amber) | ANN LT TEST only | ON (white) | ANN LT TEST only | AC ESS |
| EVAC COMMAND | `evac_command` | EVAC (red) | ANN LT TEST only | ON (white) | ANN LT TEST only | AC ESS |
| CREW SUPPLY (OXYGEN) | `crew_oxy` | — |  | OFF (white) | `oxygen/actuators/o2_valve_on` = 0 | AC ESS |
| PAX OXY MASKS | `pax_oxy` | FAULT (white) | ANN LT TEST only | ON (white) | ANN LT TEST only | AC ESS |
| RCDR GND CTL | `rcdr_gnd_ctl` | — |  | ON (blue) | ANN LT TEST only | DC ESS |
| REMOTE C/B CTL | `remote_cb_ctl` | — |  | ON (white) | ANN LT TEST only | AC 1 |
| SVGE INT OVRD | `svge_int_ovrd` | — |  | ON (white) | ANN LT TEST only | AC 1 |
| EMER EXIT LT OFF | `emer_exit_lt_off` | — |  | OFF (red) | ANN LT TEST only | AC ESS |
| COCKPIT DOOR VIDEO | `cockpit_door_video` | — |  | OFF (white) | ANN LT TEST only | AC 1 |
| NSS DATA TO AVNCS | `nss_avncs` | — |  | OFF (white) | ANN LT TEST only | AC 1 |
| NSS MASTER | `nss_master` | SMOKE (red) | ANN LT TEST only | OFF (white) | ANN LT TEST only | AC 1 |
| CWS | `ent_cws` | SMOKE (red) | ANN LT TEST only | OFF (white) | ANN LT TEST only | AC 1 |
| IFEC | `ent_ifec` | SMOKE (red) | ANN LT TEST only | OFF (white) | ANN LT TEST only | AC 1 |
| GATELINK | `gatelink` | — |  | ON (white) | ANN LT TEST only | DC ESS or DC 1 or DC 2 |
| CABIN DATA TO NSS | `cab_data_to_nss` | — |  | ON (white) | ANN LT TEST only | DC ESS or DC 1 or DC 2 |


ELEC and ADIRS: [ELEC_PANEL.md](ELEC_PANEL.md) (45 legends), [ADIRS.md](ADIRS.md) (12 legends).
Not lamps: MAINT GND CONNECTION (FBW disables both legends), the blank upper faces (VENT SMOKE
lower, CABIN PRESS MAN upper, ...), fire handles and the radio panels.

## Cockpit geometry

`tools/ovhd_cockpit_lamps.py` patched the installed objects in place (report
`build/reports/ovhd-lamps-20261021.json`; backups in
`native/avionics/.install-backups/before-overhead-lamps-20261021/objects/`):

* `a380x_cockpit_controls.obj`, after `# A380X OVHD LAMPS V1`: 217 lamp quads on the authored
  `<node>_SEQ1` / `_SEQ2` faces of the FBW glTF, lifted 0.4 mm, `ATTR_light_level 0 1` by the lamp
  dataref, hidden while dark. 37 quads move with their pushbutton: the transform of the installed
  legend decal (`decal_a380_pushbutton_decal.obj`) is evaluated at every key of its latched stock
  dataref and replayed (APU START, APU BLEED, ENG BLEED 1-4, PACK 1/2, ENG/WING ANTI ICE, PROBES,
  FEED MAIN 1-4); the APU MASTER SW legend decal is not animated by the legacy export, so its two
  quads follow the cap's key table (1.10 mm in at MASTER ON). FBW's ENG 2 BLEED upper face covers
  the whole cap (16 vertices, 14 x 14 mm, and UVs over the whole atlas); its quad uses the upper
  half like every other 14 x 7 mm legend. Click targets (draw-disabled):
  CREW SUPPLY oxygen (`sim/oxy/crew_valve_toggle`, it had none and its OFF legend would stay lit)
  and the ANN LT switch (`ATTR_manip_command_switch_up_down`).
* `panel_a380_cockpit_ovhd_knobs_4k.obj`: the ANN LT lever (1716 indices) wrapped in the authored
  rotation (0°, 22.1°, 47.6°); geometry/UV payload byte-identical.
* `cockpit_3d/fcu_labels.png` / `_LIT.png`: cells 0-13 (the FCU daylight-label cells, which no
  drawn geometry samples) now hold ON (white), OPEN (green), MAN, OVRD, SMOKE (red), DISCH,
  SQUIB, DISCH (amber), CALL (amber), EVAC (red), BTL 1, BTL 2 (amber), FAULT (white), OFF (red);
  FAULT (47), OFF (48), ALTN, DISC, AVAIL and ON blue (59-62) are the ADIRS/ELEC cells, identical
  text and colour. Every other pixel is unchanged (checked); `tools/build_fcu_panel.py` draws the
  same cells when the atlas is regenerated.

Previews (offline, no X-Plane): `docs/control-anim-previews/ovhd-lamps-*.png` (all legends in ANN
LT TEST; APU MASTER ON / START AVAIL / APU BLEED ON as in the FBW reference; batteries only) and
`ovhd-ann-lt-lever-test-brt-dim.png` (lever up in TEST, centred in BRT, down in DIM).

## In-simulator checklist

2026-10-22: lamp datarefs verified live in X-Plane 12.4.4 (values above); the visual check after
`patch_lamp_depth.py` is still to be run.

1. Cold & dark: overhead dark; ANN LT TEST: still dark (no bus).
2. BAT 1 + ESS: GEN 1/2 FAULT; set ENG 2 BLEED off: its OFF lights (DC ESS); ENG 1 BLEED OFF stays
   dark (DC 2). ANN LT up to TEST: DC ESS legends light (APU BLEED FAULT/ON, RCDR, FADEC, PRIM/SEC...).
3. APU BAT + APU MASTER: blue ON on MASTER SW; APU START: blue ON, then green AVAIL; APU BLEED: blue ON.
4. Ground power, EXT 2: fuel pump OFF, crew oxygen OFF, PACK OFF, ENG 1/4 BLEED OFF light when off;
   click CREW SUPPLY: OFF goes out. ANN LT TEST / DIM: all legends lit / dimmed; lever moves.
5. Default `lamp_supply = 1`: cold & dark, BAT 1 + BAT 2: every OFF lights (set 0 for FBW supply).
6. Daylight: legends readable against the sunlit overhead; night: glow without the dull decal text.
