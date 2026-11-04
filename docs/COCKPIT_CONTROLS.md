# Cockpit control interaction

The interaction pass covers 34 physical flyPad touch
regions across both tablets and the configured pushbuttons, selector knobs,
switches, levers, and the aft cockpit door handle, plus both KCCUs and the
two MFD touch screens (see "MFDs and KCCUs" below). It reuses each named
FlyByWire button, switch, lever, reverse-handle, or door-click mesh as an
invisible X-Plane mouse target, so the interactive area follows the visible
control rather than a large panel-wide rectangle. The original texture decals
remain the visible panel signs. The `label` field in
`config/cockpit_controls.json` becomes the mouse-hover sign shown by X-Plane.
The aft cockpit door handle calls native `sim/flight_controls/door_toggle_17`.
The pedestal `CKPT DOOR` switch is a separate up/down mouse target: UNLOCK
temporarily releases the electric lock and opens a closed door, while LOCK
forces the lock while held. Both positions spring back to NORM on release;
the closed door then locks automatically. The aircraft-local commands are
`a380x/xp12/cockpit_door/unlock` and `a380x/xp12/cockpit_door/lock`, with
read-only `position` and `locked` datarefs under the same prefix. The modeled
three-frame `SWITCH_DOOR_LOCK` animation follows `position` (-1, 0, +1), and
its invisible mouse target is placed at the NORM pose. The keypad, buzzer,
operator lockout timing, dead bolt, and CDLS failures do not yet
have simulator-side controls. The flyPad
regions call aircraft-local page, action, and power commands. See
`EFB_DEVELOPMENT.md` for the flyPad runtime and rendering boundary.

The same generated cockpit object also contains ten main-deck passenger-door
click targets from `config/cabin_door_controls.json` and six upper-deck targets
from `config/upper_deck_door_controls.json`. Keeping all manipulators in one OBJ
is required because X-Plane only treats the attachment marked as the cockpit
object as interactive. See `CABIN_INTERIOR.md` for the door channel mapping and
visual-animation coverage.

| Visible legend | Position | Native development action |
| --- | --- | --- |
| CHRONO | Captain and first officer | Start, stop, or reset the timer |
| Clock MODE | Center right panel | Select GPS, INT, or SET clock mode |
| Clock ET | Center right panel | Run, stop, or momentarily reset elapsed time |
| Clock CHR / RST / DATE | Center right panel | Control chronometer and date display; edit time in SET |
| GEAR lever | Center right panel | Select native gear UP or DOWN |
| AUTO BRK | Center right panel | Select DISARM, BTV, LO, L2, L3, or HI |
| RTO | Center right panel | Arm or disarm native RTO autobraking on ground |
| A-SKID | Center right panel | Toggle the aircraft's native anti-lock brake setting |
| SIDE STICK PRIORITY | Captain and first officer | Select left or right stick priority |
| MASTER CAUT | Captain and first officer | Acknowledge master caution |
| MASTER WARN | Captain and first officer | Acknowledge master warning |
| ALT | FCU | Select or hold altitude |
| LOC | FCU | Arm native NAV/localizer mode |
| AP1 / AP2 | FCU | Toggle native autopilot channel 1 or 2 |
| FD | FCU | Toggle the native flight director |
| A/THR | FCU | Toggle native autothrottle |
| APPR | FCU | Arm native approach mode |
| SPD/MACH | FCU | Toggle the native selected-speed unit |
| HDG/TRK | FCU | Toggle native HDG/VS and TRK/FPA presentation |
| SPD selector | FCU | Increase or decrease selected airspeed/Mach |
| HDG selector | FCU | Increase or decrease selected magnetic heading |
| ALT selector | FCU | Increase or decrease selected altitude |
| ALT 100/1000 selector | FCU | Toggle the native altitude increment between 100 and 1000 feet |
| V/S selector | FCU | Increase or decrease selected vertical speed/FPA |

The four original FBW glareshield underside meshes are now rotary targets.
From captain to first officer, they control captain flood, FCU LCD, integral
legends, and first officer flood. The upstream FBW cockpit behavior assigns
LCD to source knob 2 and integral lighting to source knob 3. Each turn changes
its aircraft-local lighting dataref by 5%. The outer knobs drive separate
X-Plane panel spotlights, while the center knobs dim the native FCU digits and
labels independently. Their command/dataref prefix is `a380x/xp12/lighting/`.

The captain and first officer Display Unit Control Panels use the original
`KNOB_EFIS_*` meshes. PFD and ND inner knobs dim their respective native
displays and turn them fully off at zero. The ND outer knob has an independent
WX/TERR brightness channel, ready for a weather/terrain layer. The MFD knobs
dim and turn off their own side's MFD (labels `CAPT MFD DU - brightness and
off` and `F/O MFD DU - brightness and off`). Each
detent changes brightness by 5%. All eight active DU knobs rotate with their
own brightness levels, using the original FBW cockpit animation clips. The
active commands and read-only 0–100
datarefs use the `a380x/xp12/du/` prefix, with names such as
`captain_pfd_up` and `first_officer_nd`. OIT, PFD/ND exchange, and RECONF
remain inactive because the referenced A380X implementation does not provide
those controls yet. The MSFS lighting potentiometer indices are 88/89/94/98
for captain PFD/ND/WX/MFD and 90/91/95/99 for the first officer; this X-Plane
port exposes equivalent local state rather than MSFS SimConnect variables.

| EFIS control | Side | Native development action |
| --- | --- | --- |
| WX; TERR; TRAF | Captain and first officer | Toggle the side-specific weather, terrain, or traffic selection |
| CSTR; WPT; VOR D; NDB; ARPT | Captain and first officer | Toggle the side-specific constraint or navigation-symbol selection |
| RANGE | Captain and first officer | Increase or decrease the side-specific map range |
| MODE | Captain and first officer | Rotate through the side-specific native map modes |
| BARO | Captain and first officer | Increase or decrease the side-specific altimeter setting |

| Overhead group | Interactive controls | Native development action |
| --- | --- | --- |
| ELEC + EMER ELEC PWR | All 26 pushbuttons and 6 guards | Native ELEC panel module (`a380x/xp12/elec/...`); see [ELEC_PANEL.md](ELEC_PANEL.md) |
| APU | MASTER; START | Switch native APU power on/off, then start the native APU |
| ANTI ICE | ENG 1-4; WING; PROBES/WINDOW | Toggle native engine, wing bleed-air, or window heat |
| AIR | ENG 1-4 BLEED; PACK 1/2; APU BLEED | Toggle the corresponding native bleed or pack |
| FUEL | FEED 1-4 MAIN | Toggle native fuel pumps 1-4 |
| EXT LT | NAV; BEACON; STROBE; NOSE; LAND | Toggle native navigation, beacon, strobe, taxi, or landing lights |
| SIGNS | SEAT BELTS; NO SMOKING | Toggle the corresponding native passenger sign |

Each of these 32 overhead targets also drives the original FBW source control
animation from the corresponding native X-Plane state. Pushbuttons depress,
two-position rockers move between their end positions, and the source
three-position light/sign switches use their two end positions for X-Plane's
boolean controls. The APU START button follows the native OFF/ON/START state.
This gives an immediate in-cockpit response while keeping the simulator state,
rather than a separate cosmetic variable, authoritative.

| Throttle control | Native development action |
| --- | --- |
| THRUST LEVER 1-4 | Superseded by [THRUST_LEVERS.md](THRUST_LEVERS.md): animated click-hold-drag levers with sticky CL/FLX detents |
| REVERSE 2/3 | Superseded by [THRUST_LEVERS.md](THRUST_LEVERS.md): animated reverse levers, IDLE REV / MAX REV, interlocked with IDLE |
| A/THR DISCONNECT, left/right | Superseded by [THRUST_LEVERS.md](THRUST_LEVERS.md): animated instinctive-disconnect buttons, 15 s inhibit |
| FLAPS lever | [FLAPS_LEVER.md](FLAPS_LEVER.md): pull-out lift-detent lever, gates at 1 and 3, FBW SFCC (1 / 1+F, relief, baulk, alpha lock) |
| ENG MASTER 1-4 | Toggle the corresponding native engine fuel cutoff between OFF and RUN |

The ten FCU and EFIS rotary selector meshes use X-Plane command-knob manipulators, so mouse-wheel
or knob-rotation input changes the same native targets used by X-Plane commands and hardware bindings.
The A380 altitude-increment mesh uses a two-value X-Plane toggle manipulator on
`sim/aircraft/autopilot/alt_step_ft`. Managed/selected push-pull actions are implemented
in the optional FCU window and exposed as bindable FlyWithLua commands. Independent
push/pull mouse zones and selector animation on the physical 3D knobs remain unfinished.

The EFIS buttons and selectors update the captain/copilot native X-Plane states
and their source control animations. The aircraft-local ND intentionally remains
`FMS MAP PARTIAL`: the FBW navigation database and route/symbol rendering are not
ported, so these controls do not claim to add missing FMS layers to that display.

These are X-Plane-native development mappings, not a port of the FBW flight
guidance, electrical, pneumatic, fuel, warning, or display systems. A click
changes the native simulator state and the mapped control now moves, but
annunciator lamps, Airbus mode logic, display feedback, and failure behavior
are not implemented by this pass (the ELEC panel is the exception, see ELEC_PANEL.md). APU MASTER, transfer
and standby pumps, and the remaining light controls stay non-interactive until
they have unambiguous simulator-side behavior. The generated thrust and reverse
drag axes listed in `config/cockpit_controls.json` are retired in the installed
object by `tools/thrust_lever_cockpit.py`, which adds the animated levers, Airbus
detents and the disconnect buttons ([THRUST_LEVERS.md](THRUST_LEVERS.md)).

`tools/build_local_mac.sh` regenerates the click object from the pinned cockpit,
LOD00 cabin, and exterior fuselage glTFs and validates every configured command against the installed X-Plane
`Commands.txt` when that catalogue is available. The generated object and JSON
report live under the external build directory and are copied into the aircraft
package as `objects/interior/cockpit-controls/a380x_cockpit_controls.obj`.

For an existing complete package, `--refresh-group cockpit-controls` is a
transactional focused refresh: it preflights only the regenerated control OBJ
and its coupled native panel assets, preserves every unselected attachment from
the installed ACF, and does not inspect or recopy unrelated cabin, seat,
exterior, Lua, airfoil, or license inputs. The command requires
`--cockpit-controls-object`, `--fcu-panel-assets` and `--mfd-screens-object`,
preventing an updated cockpit-region mesh from drifting out of sync with its
embedded panel definition, and the MFD touch quads from drifting out of sync
with the visible MFD screens. A full build retains the broader all-input preflight.

Static package validation confirms the object attachment, the OBJ8 manipulators, index
bounds, clockwise winding, and command catalogue entries. For an A388 package
it also requires the packaged 1024x1024 RGBA panel, the aircraft-local
`a380x-fbw-avionics-v1` provider, all four bilateral PFD/ND targets, the absence
of stock X-Plane EFIS/map/FD/VNAV/GPSS instruments, and matching cockpit-region
UV quads. These checks read the finished package rather than trusting the
external build directory. The focused panel/control tests separately check the
exact `SCREEN_DU_PFDL`/`SCREEN_DU_PFDR`, `SCREEN_DU_NDL`/`SCREEN_DU_NDR`, and
both physical flyPad bounds, all six display UV quads, and all 34 tablet touch
zones. Native mouse hit testing, GPU panel rendering,
and command behavior still require an in-simulator cockpit check.

## MFDs and KCCUs

Both lower-centre MFDs are X-Plane plugin cockpit devices (`XPLMCreateAvionicsEx`,
X-Plane 12.1 or later), not regions of the 2048x2048 panel texture. The native
plugin creates `a380x.xp12.mfd.captain` ("A380X MFD CAPT") and
`a380x.xp12.mfd.first_officer` ("A380X MFD F/O"), each 768x1024 texels. The
geometry is generated from the authored `SCREEN_DU_MFDL`/`SCREEN_DU_MFDR` corners
(`MFD_SCREEN_CORNERS` in `tools/avionics_panel_layout.py`) in two places:

* `objects/interior/cockpit-panels/a380x_mfd_screens.obj` holds the visible
  screens. It is attached directly after `screen_powered_off_display_faces.obj`
  with the same internal flags and transform. Its albedo is the opaque 8x8
  `textures/interior/cockpit-panels/mfd_screen_black.png`; the device image
  replaces only the lit texture. Each quad is
  `ATTR_cockpit_device <id> 1 <14|15> 1` (bus 0, instrument-brightness slots 14
  and 15, auto-adjust on), lifted 2 mm over the dark face with
  `ATTR_poly_os 2`.
* `a380x_cockpit_controls.obj` carries a draw-disabled touch twin of each screen,
  1 mm above the face: `ATTR_cockpit_device`, `ATTR_manip_device arrow <id>
  <CAPT|F/O> MFD`, two triangles, `ATTR_manip_none`, `ATTR_no_cockpit`. Clicks,
  drags, the mouse wheel and hover go to the plugin, which hit-tests the MFD
  widgets itself. The touch quads are never hidden; the plugin ignores input
  while a side's DU is off.

UV (0,0) is the device's bottom-left texel as seen from the seat, so the image is
upright and unmirrored. `tools/build_cockpit_controls.py --mfd-screens-output`
writes the visible object and its black texture next to the controls object, and
the `cockpit-controls` focused refresh copies and attaches both.
`tools/validate_aircraft.py` requires one full-UV quad per device in each object,
matching `ATTR_cockpit_device` values, and device IDs equal to the avionics
provider's `cockpit_devices`.

The two KCCUs reuse the named FBW `PUSH_KCCUL_*` (captain) and `PUSH_KCCUR_*`
(first officer) meshes as 136 click targets. Every target sends an
aircraft-local command to the MFD on its own side only. The hover label reads
`CAPT KCCU - <legend>` or `F/O KCCU - <legend>`.

| KCCU control | Command (`<side>` = `captain` or `first_officer`) | MFD action |
| --- | --- | --- |
| A-Z, 0-9 | `a380x/xp12/kccu/<side>/<a..z, 0..9>` | Type into the focused field |
| `.` and `+/-` | `.../dot`, `.../plusminus` | Decimal point; sign toggle |
| ENT; BACKSPACE; SP; `/` | `.../ent`, `.../backspace`, `.../sp`, `.../slash` | Validate field; delete; space; slash |
| ESC (keyboard and cursor-control) | `.../esc`, `.../esc2` | Cancel the edit |
| Cursor UP / DOWN / LEFT / RIGHT | `.../up`, `.../down`, `.../left`, `.../right` | No field action (as in FBW) |
| DIR; PERF; INIT; F-PLN; DEST | `.../dir`, `.../perf`, `.../init`, `.../fpln`, `.../dest` | FMS ACTIVE F-PLN direct-to, PERF, INIT, F-PLN, F-PLN at destination |
| NAVAID; SEC INDEX; SURV; ATC COM | `.../navaid`, `.../secindex`, `.../surv`, `.../atccom` | POSITION navaids, SEC index, SURV controls, ATC COM connect |
| CLR INFO | `.../clrinfo` | Clear the latest FMS message |
| MAILBOX; ND; KBD | `.../mailbox`, `.../nd`, `.../kbd` | No MFD function |
| REWIND; FORWARD | `.../rewind`, `.../forward` | Inoperative (cursor to the left/right display) |
| Scroll wheel | `.../wheel_up`, `.../wheel_down` (command knob) | Scroll at the MFD cursor |
| Cursor-select button (outboard side) | `.../click` | Primary click at the MFD cursor while the cursor is shown |
| Trackball | `a380x/xp12/mfd/<side>/popup` | Open or close that side's MFD pop-up; point at the 3-D MFD with the mouse to move the cursor |
| KBD switch; CCD switch | `.../kbd_switch_toggle`, `.../ccd_switch_toggle` | Flip `a380x/xp12/kccu/<side>/kbd_on` / `ccd_on` |

The six blank caps per side, the `_SEQ1` legend decals, the KCCU body and the
trackball cover are not mapped. The KBD/CCD levers and key caps do not animate.
`a380x/xp12/mfd/<side>/popout` is bindable but has no cockpit target. The
pull-out OIT keyboards are not mapped to the MFD.

## Control motion (FCU, EFIS, ISIS, ECAM brightness)

Every clickable FCU, EFIS and ISIS button and knob now moves with its authored FBW clip
(pivot, axis, travel, and the cap's legend decal). Momentary keys use eased ~80 ms travel
datarefs from `native/avionics/a380x_control_motion.h`: they go in on command Begin (mouse
down), stay in while held, and spring back on End; a click shorter than a frame still shows
a full press. Knob rotations accumulate detents. The panel OBJs are patched in place by
`tools/patch_fcu_animations.py` (FCU, earlier), `tools/patch_efis_animations.py` (EFIS
selectors, earlier) and `tools/patch_controls_animations.py` (this pass): only draw commands
change, vertex/UV/texture payloads are byte-identical, reruns are no-ops.
`tools/preview_control_motion.py` evaluates the installed OBJs offline and measures every
moving draw (see `docs/control-anim-previews/`).

| Panel | Control | Motion | Dataref |
| --- | --- | --- | --- |
| FCU | AP1, AP2, FD, A/THR, LOC, ALT, APPR, SPD/MACH, HDG/TRK, TRUE/MAG, METER (+ legends) | 1.35 mm press | `a380x/xp12/fcu/<key>_pressed` |
| FCU | SPD, HDG, ALT, V/S knobs | push 0.8–1.4 mm / pull, 10° per detent | `fcu/<knob>_knob_axial`, `fcu/<knob>_knob_rotation` |
| FCU | ALT 100/1000 ring (+ light) | rotates to its two stops | `fcu/altitude_selector` |
| EFIS (both) | CSTR, WPT, VOR.D, NDB, ARPT, VOR1, VOR2, WX, TERR, TRAF, TAXI, LS, VV (+ legends) | 0.61 mm momentary press | `efis/<side>/<key>_pressed` |
| EFIS (both) | BARO knob | rotates with the preset; push 1.3 mm (CAPT) / 0.9 mm (F/O) for STD, pull for QNH | `efis/<side>/baro_preset_hpa`, `efis/<side>/baro_knob_axial` |
| EFIS (both) | hPa/inHg ring (knob + light) | rotates between units | `efis/<side>/baro_inhg` |
| EFIS (both) | ND mode and range selectors | rotate to the selected stop | `efis/<side>/mode`, `efis/<side>/range_index` |
| EFIS (both) | PFD, ND, WX/TERR, MFD brightness knobs | rotate with the DU level | `du/<side>_<unit>` |
| ISIS 1 / 2 | MODE, LS/D, +, −, MENU (+ legends) | 1.0 mm momentary press | `isis[2]/<key>_pressed` |
| ISIS 1 / 2 | SET/SEL | cap push 1.5 mm; knob 10° clockwise per detent, endless | `isis[2]/set_sel_axial`, `isis[2]/set_sel_rotation` |
| ECAM CP | SD and EWD brightness knobs | rotate 0–270° with the brightness | `ecam/sd/brightness`, `ecam/sd/ewd_brightness` |

The EFIS keys CSTR…TRAF used to move with the latched filter state (in while ON); they are now
momentary like the real keys. The PFD/RECONF pushbuttons, the blank caps and the OIT knobs have
no function or click target in this port and stay still. FBW does not model an EXPED key.

## Mouse cursors (October 21 audit)

X-Plane shows the manipulator's cursor word: `button` is the pointing index finger,
`hand` the grabbing hand. Every pushbutton, pb-switch, key and knob PUSH zone now uses
`button` (FCU, EFIS, glareshield, clock, RTO, ECAM control panel, ISIS 1/2, anti-ice, bleed,
packs, fuel pumps, APU START, A/THR disconnect, the whole ELEC panel). `hand` remains on
levers and toggle switches (gear, anti-skid, exterior lights, signs, ALT increment), doors,
knob PULL zones, the KCCU trackball, screen pop-up regions, and the six ELEC guards, which
the user grabs and moves. Knobs keep their rotate cursors and the thrust/reverse levers
their drag manipulators. `tools/patch_cockpit_cursors.py` applies the classification to the
installed `a380x_cockpit_controls.obj` (120 existing `hand` click targets changed) and to
`config/cockpit_controls.json` (73 entries); the ISIS and ECAM generators emit `button` for
keys. Report: `build/reports/cursor-audit-20261021.json`.

## No exponent notation in OBJ files (November 3 fix)

X-Plane's OBJ8 reader stops a number at its exponent: `4.694689366e-07` is read as 4.69 and
`3.187809668e-10` as 3.19. The patch tools wrote tiny values with Python's `:g` / `:.9g` / `:.10g`
formats, which use exponents, so 28,194 numbers in 17 OBJs were misread in the sim:

| Symptom | Misread number |
| --- | --- |
| DU and AUTO BRK knobs tip over as they turn; ND inner knob vanishes | rotation axis x component `4.69e-07` read as 4.69 |
| A-SKID toggle swings sideways | axis `2.7e-06, 1.9e-06` |
| Flap and speed-brake levers vanish when lifted | lift translation `-7.07e-06` / `-1.16e-06` read as metres |
| ELEC pushbuttons fly about 3 m when pressed; their click targets sit off the panel | press translation `3.19e-10` |
| ISIS legends blank, SET/SEL knob cap missing | rest offset `1.73e-18` / `-4.1e-15` |
| Triple brake-pressure needles, clock, GRVTY, ANN LT, FCU ALT ring, EWD/SD knobs, fans 2/3 | rotation axes |

`tools/obj8_numbers.py` is the shared formatter (`fmt`) every OBJ writer now uses, and its CLI
repairs existing files (`--fix`) or checks them (`--check`); only whole numeric tokens with an
exponent change. `tools/validate_aircraft.py` reports any remaining exponent as an error.
