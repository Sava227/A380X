# A380 EFB: cockpit tablets and the ISCS window

This document covers **Phase 1** of the new A380 EFB. It is a web page shown on both cockpit tablets.
It follows the approved design in [efb-design/EFB_MOCKUP.dc.html](efb-design/EFB_MOCKUP.dc.html).

The **ISCS floating window** (native, eight tabs) is unchanged. It is described at the end.

## 1. Phase plan

| Phase | Content | State |
| --- | --- | --- |
| 1 | Web EFB on both tablets, with touch. Every existing tablet function is mapped onto the new sections. Typed payload, door escape slides, a Failures section, sound volumes. SimBrief, Aircraft options and Cabin as UI only. | **This change** |
| 2 | Real take-off / landing performance, real CG and weight & balance, charts (airport diagram from apt.dat) | Next task |
| 3 | Aircraft options take effect: engine type, brake fans, HUD that disappears when not installed | Later |
| 4 | Cabin layout with weight changes and 3D seats, including lower-deck seating | Last |

The page shows **no sample data**. Every value comes from X-Plane or the native A380 modules. Anything
without a backing model arrives as `null` and is shown as unavailable (a grey "—", "NOT MODELLED",
"NEXT UPDATE" or "NO EFFECT YET").

## 2. Architecture

```
 cockpit tablet glass (OBJ: ATTR_cockpit_device + ATTR_manip_device, 1440 x 1000 texels)
      |  touch: down / drag / up (device texels, bottom-left)        ^ texture
      v                                                              |
 a380x_efb.c  (EFB translation unit)                                 |
   a380x_efb_device.inc  cockpit devices, touch ring, web or native fallback drawing
   a380x_efb_web.inc     native state, action validation, JSON frame per tablet
   a380x_efb_core.c      XPLM-free rules: sections, doors/slides, entries, FQMS, JSON, grammar
   (legacy) a380x_efb_render.inc / a380x_iscs.inc / a380x_efb_window.inc
      |  a380x_efb_frame_json()        ^ a380x_efb_web_action()        ^ host.draw()/live()
      v                                |                               |
 a380x_avionics.c  -> a380x_xpkit_efb.inc  (publish frames, drain action rings, texture hook)
      |  shared memory, protocol v8 (a380x_xpkit_shared.h): efb[2] = JSON + action ring
      v                                ^
 A380XDisplayHost (macOS WebKit / Windows WebView2), views 7 and 8: xp-kit/efb.html?tablet=N
      |  __a380Efb.apply(frame) -> HTML; capture only on change    postMessage("a380x-efb-action")
```

Key points:

* **The native side owns all state.** Payload/fuel loading, doors and slides, presets, checklists,
  failures, radios, sound volumes and the per-tablet navigation all live in the plugin. The page is
  presentation and touch only. Every page action is parsed and validated natively again.
* **Two independent tablets.** Each tablet has its own section, tabs, checklist group and touch
  stream. The aircraft state is shared. The legacy `a380x/xp12/efb/page/<page>` commands move both
  tablets, as both showed the one shared page before.
* **Touch path.** X-Plane calls the device touch callback. The plugin records each sample in order
  (type, x, y in page pixels, origin top-left) in a 32-event ring per tablet and publishes it at
  once. The page replays only the new events, hit-tests them with `elementFromPoint`, and posts
  `<id> <verb> [args]` back. The helper writes those strings into the tablet's action ring (single
  producer, single consumer). The plugin drains the ring every flight loop and acknowledges the
  highest id in the next frame. Taps and slider drags (with throttled updates) both work.
* **Positionless releases.** X-Plane 12.4 sends drags/releases at (-16,-16) or (0,0). The plugin
  finishes the gesture at the last position, the same handling as the MFD/ND.
* **Renderer fallback.** When the WebKit page is not live (helper missing, crashed or stale, or a
  Windows build without the WebView2 host), the device draws the native flyPad renderer
  (`a380x_efb_render.inc`) at twice its atlas size. A press is then mapped back to the legacy atlas
  hit test. A tablet never freezes or goes blank because of the helper.
* **Capture cost.** Frames are rebuilt on any EFB change (touch, action, state) and at least every
  250 ms. They are published only when their bytes differ. The helper captures a new 1440 x 1000
  texture only when the page reports a visual change; otherwise it renews a heartbeat. An idle
  tablet costs no WebKit snapshot and no GPU upload.
* **Brightness and power.** The device's brightness callback returns the tablet brightness (20-100 %).
  It returns 0 when the tablet is off or its simulated battery is empty without aircraft power. The
  tablet is not gated by a cockpit bus. A tap on a dark tablet wakes it (power on), unless the
  battery is empty and not charging.
* **Old panel-atlas tablet.** It is drawn only while the OBJ has not bound both devices. Aircraft
  packages from before the device geometry keep working with the new plugin.

## 3. Files

| Area | Files |
| --- | --- |
| Native core (XPLM-free) | `native/avionics/a380x_efb_core.h/.c`, test `test_efb_core.c` |
| Native adapter | `a380x_efb.c` (+ `a380x_efb_web.inc`, `a380x_efb_device.inc`), header `a380x_efb.h`, test `test_efb_adapter.c` (mock XPLM with the cockpit-device API; `--contracts`, `--frame`) |
| Legacy renderer / ISCS | `a380x_efb_render.inc`, `a380x_iscs.inc`, `a380x_efb_window.inc`, test `test_efb_render.c` |
| Transport | `a380x_xpkit_shared.h` (v8: GitHub's v7 per-kind capture scale plus the EFB views), `a380x_xpkit_efb.inc`, `a380x_xpkit_adapter.inc`, helpers `a380x_xpkit_host.m` (macOS) and `a380x_xpkit_host_win.cpp` (Windows), probe `test_xpkit_host.c` |
| Page | `native/avionics/xp-kit-efb/efb.html` (template), `efb.css`, `efb.js`, `Fonts/efb/*` (+ `OFL.txt`) |
| Packaging | `tools/package_xp_kit_cockpit.py` (`package_efb`; the page is always rebuilt from source, never reused) |
| Geometry | `tools/efb_cockpit_devices.py` (applied by `tools/install_xp_kit_cockpit.py`) |
| SD DOOR page | `a380x_sd_data.inc` (per-door slide flags), `tools/patch_xp_kit_sd.py` (version 2 slides edit) |
| Validation | `tools/validate_aircraft.py`, `tools/avionics_provider.py`, `tools/xp_kit_transport.py` |
| Tests (Python / JS) | `tools/test_efb_page.py` + `tools/efb_page_check.js`, `tools/test_efb_cockpit_devices.py`, `tools/test_xp_kit_transport.py`, `tools/test_patch_xp_kit_sd.py`, `tools/test_reuse_xp_kit_assets.py` |
| Previews | `tools/render_efb_page.m` (offline WebKit renderer of the packaged page) |

## 4. The tablet frame (plugin -> page)

The plugin builds one JSON object per tablet with `a380x_efb_frame_json()`. The size cap is 49,152
bytes; a realistic frame is about 4 KB. The writer never emits NaN/Inf: a non-finite value becomes
`null`. If the frame does not fit, it is not published at all.

| Key | Content |
| --- | --- |
| `v`, `tablet`, `epoch`, `ack` | Format 1, tablet 0/1, plugin-load epoch (the page resyncs touch history on change), highest page action id processed |
| `power` | `blank` (dark screen), `on`, `battery` (%), `charging` (aircraft power available), `sim` (battery simulation) |
| `settings` | `metric`, `autoCheck`, `brightness` |
| `clock` | `zulu`, `local`: minutes of the day (whole minutes keep the frame stable) |
| `ui` | `section`, `tabs` (selected tab per section), `group` (checklist group) |
| `status` | The native status line. The page shows a change as a toast for 5 s |
| `touch` | `serial`, `events`: up to 32 `[seq, type, x, y]` (type 1 down, 0 move, 2 up; page pixels) |
| `ac` | Variant, engines, `onGround`, `enginesOff`, `loadingAllowed`, `elecNetwork`, `aircraftPower`, FWS `phase`, altitude, ground speed, lat/lon, QNH, wind, OAT, parking brake, beacon |
| `weights` | `gw`, `fob`, `payload`, `zfw`, `oew` (all live, rounded to 10 kg), limits, `cg: null` (Phase 2) |
| `load` | Planned `pax[4]` and `cargo[3]` (kg), `seats`, `holdMax`, `paxKg` 104, `planKg`, `planSet`, `liveMatches` |
| `fuel` | 11 FQMS tanks `[name, kg, capacity kg]`, block-fuel `target`, `refuelling`, `rate` |
| `doors` | `ratio[20]` (door_open_ratio), `slides[16]`, counts `open`, `armed` |
| `ground` | `gpu`, `chocks`, `tug` (X-Plane pushback attached), `jetway: null`, pushback `steer` |
| `checklists` | 10 groups: name, `done` bits (manual + automatic), `auto` bits, items `[name, response, auto-capable]` |
| `failures` | 17 `[name, system, ATA, X-Plane failure enum or null]` |
| `radios` | `com1`/`com2` `[active, standby]` (kHz, 8.33), `squawk`, `mode` (X-Plane transponder enum) |
| `tod` | Altitude, target, angle, distance (nm) and average ground speed of the existing TOD calculator |
| `presets` | `native` (A380 network present), `active`, `done`, `step`, `count`, `stepName`, `expedited` |
| `sounds` | 10 `[id, percent]` of X-Plane's writable volume datarefs |
| `options`, `cabin` | Stored Phase 3 / Phase 4 settings (engine, brake fans, HUD; deck seat counts, fill flags, hold types) |

## 5. Page actions (page -> plugin)

Grammar: `<id> <verb> [arg ...]`. The id has 1 to 9 digits and must be positive. The verb is
lower-case. There are at most four arguments, printable ASCII only, and the line is under 128 bytes.
Anything else is rejected and counted in `a380x/xp12/efb/actions_rejected`.

| Verb | Arguments | Native effect (rule) |
| --- | --- | --- |
| `nav` / `tab` | section [tab] | Tablet navigation; also sets the legacy page for `action_1..6` |
| `door` | M1L..U3R, CKPT, FWD, AFT, BULK | `door_toggle_N` (ground only; refused when its slide is armed) |
| `doors` | open / close | All doors; armed doors stay closed |
| `slide` | door 0/1 | Arm needs the door closed; cargo and cockpit doors have no slide |
| `slides` | arm / disarm | Arm the slides of all closed passenger doors / disarm all |
| `pax` / `cargo` | class or hold, typed digits | Validated against seats / hold limit (cargo typed in kg or lb) |
| `load` | apply / clear / full | Writes `m_fixed` (ground, engines off, at most 101,736 kg) |
| `fuel` | target N / rate real\|fast\|instant / start / stop | Ground, engines off; instant = the old tablet |
| `service` | gpu / jetway / trucks / chocks | Ground only |
| `push` | steer 0-2 / start / stop | X-Plane pushback left / straight / right / stop |
| `cl_group`, `cl_item`, `cl_next`, `cl_reset` | group [item] | Checklists. CHECK ITEM on a complete group moves on to the next group |
| `preset` | 1-5 / expedite | Native A380 presets (a second tap stops a running one); without the network: 1, 2, 4 |
| `fail` / `fail_clear` | index 0/1 | X-Plane failure 6 (failed now) or 0; `fix_all_systems` |
| `radio` | swap1/2, guard, ident, stby1/2 freq, squawk code, mode 1\|3\|6\|7 | 118.000-136.990 MHz on a valid 25/8.33 kHz channel; squawk with four octal digits |
| `tod` | target N / up / down / steeper / shallower / reset | Metres when metric is on |
| `map` | current / low / high / zoom_in / zoom_out / airports | X-Plane map commands (the old Nav page) |
| `vol` | id 0-100 / restore | X-Plane volume datarefs; restore = the values at aircraft load |
| `set` | metric / autocheck / battery 0/1, brightness 20-100 | Tablet settings |
| `power` | | Tablet standby |
| `option`, `cabin` | ... | Stored only (Phase 3 / Phase 4); the status line says so |

## 6. Sections: what is real and what is not

| Section / tab | Real (from the aircraft) | Unavailable or planned |
| --- | --- | --- |
| Home | GW, FOB, ZFW, door and slide counts, next checklist, ZFW/GW limit bars, presets, FWS phase, altitude | Origin/destination (OFP), CG (Phase 2), LW (needs OFP trip fuel) |
| OFP | — | The whole SimBrief page: **PLANNED · NOT ACTIVE YET**. Nothing is fetched and nothing is invented. |
| Ground › Doors & slides | 20 doors, 16 slides, rules, open/close all, arm/disarm all | — |
| Ground › Services | GPU (A380 EXT or X-Plane GPU), wheel chocks, jet bridge and service-truck requests | Jet-bridge state (X-Plane does not report it), PCA, ASU |
| Ground › Pushback | Tug attached, steering choice, start/stop, before-pushback checks | Tug speed (fixed by X-Plane) |
| Loading › Payload | Typed passengers per class, cargo per hold, apply/unload/all seats, live payload, OEW, planned ZFW | Per-class split of a payload set outside the EFB (shown as unknown) |
| Loading › Fuel | 11 FQMS tanks (the SD FUEL split), block fuel typed, real/fast/instant refuel | Real refuel rates are approximations (≈3 t/min, fast ≈15 t/min) |
| Loading › W&B | Live GW / ZFW / FOB | CG envelope and CG values (Phase 2) |
| Cabin | Stored layout (seat counts per deck, fill flag, hold types) | **PREVIEW · applies in a later update**: no 3D seats and no weight change (Phase 4) |
| Flight › Performance | TOD calculator (target altitude typed or ±1000 ft, angle ±0.5°) | T.O / LDG performance (Phase 2) |
| Flight › Charts & nav | Present position, wind, QNH, X-Plane map buttons | Chart viewer (Phase 2: apt.dat airport diagram) |
| Flight › Comms | COM 1/2 active/standby (typed), swap, guard, squawk (typed), IDENT, mode | Datalink (Hoppie/CPDLC) |
| Ops › Checklists | 10 groups, manual ticks, CHECK ITEM, auto items (seat belts, parking brake, beacon, gear, spoilers, engines, batteries) | — |
| Ops › Presets | Native A380 presets with step progress, expedite | — |
| Failures | 17 selectable X-Plane failures, system filters, clear all | Armed/random failures (an X-Plane-scheduled failure shows its X-Plane mode) |
| Aircraft | Stored engine / brake fans / HUD choice | **NO EFFECT YET** (Phase 3) |
| Settings › General | Units, clock (UTC/local, per tablet), auto check, battery simulation, brightness, tablet off, battery state | — |
| Settings › Sounds | 10 X-Plane volume sliders: master, engines, exterior, interior, clicks (UI), environment, weather, ground, warnings, radios | No new sound assets |

## 7. Doors and escape slides

* **Native state.** Slide state for the 16 passenger doors (`door_open_ratio` order: M1L M1R … M5R,
  U1L … U3R) lives in `slide_armed[16]`. It is published as `a380x/xp12/efb/slide_armed` (int[16])
  and `slides_armed_count`.
* **An armed door cannot be opened, by any path.** The plugin registers a "before" handler on
  `sim/flight_controls/door_toggle_1..16`. While a door's slide is armed and the door is closed, the
  handler vetoes the command (status: "Disarm the … slide before opening the door"). This covers the
  EFB, the ISCS window, the legacy ALL DOORS button and key bindings.
* **An open door cannot be armed.** Arming needs `door_open_ratio <= 0.01`, so a moving door counts
  as open. If a door is opened by a path that cannot be vetoed (a direct dataref write, another
  plugin), its slide is disarmed at the next flight loop, so the state never shows "open and armed".
* **Cargo and cockpit doors have no slides.**
* **Arm all / Disarm all** exist on the page and as the commands `a380x/xp12/efb/slides_arm_all` /
  `slides_disarm_all`. Arm all leaves open doors disarmed and says how many.
* **Presets.**
  * Cold & dark and Turnaround disarm all slides.
  * Ready for pushback, taxi and take-off arm the slides of every closed door.
  * With the A380 network this happens when `a380x/xp12/elec/preset/done` changes while no preset
    is active. Without it, the legacy X-Plane command presets apply it directly.
* **SD DOOR page.**
  * FBW's DoorPage passes `LIGHT BEACON ON` to every door as a TODO proxy for "slide armed".
  * The SD frame now publishes `L:A380X_SLIDE_ARMED:<n>` per door.
  * The version 2 bundle edit makes each CabinDoor prefer that per-door value. When it is absent,
    the FBW proxy still applies. The door shows "S" only when the slide is armed and the door is
    not open, exactly as in FBW.
* **FWS.** The native FWS has no slide variables. FBW's A380 FWS defines no slide alert, so nothing
  is fed there. The door-open alerts (MAIN nL/R NOT CLOSED) keep using `door_open_ratio`.

## 8. Commands and datarefs

* **Kept for existing bindings.**
  * `a380x/xp12/efb/page/<dashboard|dispatch|ground|performance|navigation|atc|failures|checklists|presets|settings>`.
    Both tablets follow: dashboard → Home, dispatch → Loading › Payload, ground → Ground › Doors,
    performance → Flight › Performance, navigation → Flight › Charts & nav, atc → Flight › Comms,
    failures → Failures, checklists → Ops › Checklists, presets → Ops › Presets, settings →
    Settings › General.
  * `a380x/xp12/efb/action_1..6`: the legacy contextual actions of the current legacy page.
  * `power`, `toggle`, `next_page`, `window_toggle`, `window_open`, `window_close`.
* **New commands:** `a380x/xp12/efb/slides_arm_all`, `slides_disarm_all`, `captain/popup`,
  `first_officer/popup` (the X-Plane 2D pop-up of a tablet).
* **New datarefs (read-only):**
  * `a380x/xp12/efb/slide_armed` (int[16]), `slides_armed_count`, `doors_open_count`.
  * `devices_active`, `captain/device_bound`, `first_officer/device_bound`, `captain/web_live`,
    `first_officer/web_live`.
  * `actions_processed`, `actions_rejected`, `touch_events`.
  * `a380x/xp12/efb/native_available` is unchanged.
* **xp-kit health:** `a380x/xp12/xpkit/captain_efb_active`, `first_officer_efb_active`,
  `captain_efb_presented_frames`, `first_officer_efb_presented_frames`.
* The cockpit-device contract (devices, commands, datarefs) is stored as `efb_devices` in the avionics
  provider manifest. `build.py` checks the native registry against `tools/efb_cockpit_devices.py`
  and `avionics_provider.EFB_COMMANDS`.

## 9. Geometry and installation

* **Devices.** `a380x.xp12.efb.captain` and `.first_officer`.
  * Each is 1440 x 1000 texels, twice the old 720 x 500 atlas region. The aspect is 1.44; the
    measured glass is 1.427.
  * Lighting channels 24 and 25 (14-23 are taken).
* **Quads.** They use the measured `EFB_SCREEN_SURFACES` corners, with UV (0,0) at the device's
  bottom-left, the same convention as the PFD/ND/MFD devices.
  * The visible quad goes in `a380x_mfd_screens.obj`, 3 mm in front of the glass (like the old
    atlas face).
  * The draw-disabled `ATTR_manip_device` (cursor `button`) touch quad goes in
    `a380x_cockpit_controls.obj`, 4 mm in front.
* **Retired.** The two cockpit-region atlas EFB faces and the 34 legacy atlas touch regions (10 tabs,
  6 actions and power per tablet). They are commented out with a `# A380X EFB DEVICE V1 RETIRED`
  prefix. No vertex, index or other draw moves; new geometry is appended. The edit is idempotent
  (`# A380X NATIVE EFB DEVICE QUADS V1`). It refuses an object that does not contain exactly two
  atlas faces and 34 legacy regions.
* **Install.** `tools/install_xp_kit_cockpit.py` applies the geometry together with the plugin, like
  the PFD/ND/SD/ISIS device quads. It verifies the EFB contract and the page hash, and writes the
  two objects all-or-nothing: if a later write fails, the first is restored. `--check-only` reports
  the objects that would change and their sha256 after the change.
* **Validator.** `tools/validate_aircraft.py` expects 0 atlas EFB faces when the provider declares
  `efb_devices` (2 without it), plus one touch quad and one visible quad per EFB device with the
  declared bus/channel.
* **Dry run.** `install_xp_kit_cockpit.py <build> --check-only --stage-dir <new dir>` writes the
  objects and provider manifest an installation would write, and prints their before/after
  sha256. `tools/validate_aircraft_overlay.py <aircraft> --stage-dir <dir> --file <rel>=<staged>`
  then validates the live tree with those files overlaid in-process, without copying the aircraft.

### Install order

1. Quit X-Plane.
2. Run `python3 tools/install_xp_kit_cockpit.py <build dir>`. This one step installs:
   * the macOS plugin and the `A380XDisplayHost` helper;
   * the xp-kit assets, including `efb.html`, its fonts and the SD bundle with the slides edit;
   * both cockpit objects, with the EFB geometry;
   * the provider manifest (`efb_devices`) and the build report.

   Backups go to `build/backups/xp-kit-cockpit-<timestamp>`.
3. Windows only: a Windows kit built with `--xpkit-windows` and packaged by
   `tools/package_windows_avionics.py` / `install_windows_avionics.py`. It carries the EFB devices
   and the WebView2 EFB views. Without it, Windows tablets have no device (see section 12).

## 10. Fonts

The approved design uses **Barlow** and **IBM Plex Mono**. Neither is available offline in this
repository, so the page uses FlyByWire flyPad's own fonts as the documented fallback:

* **Manrope** (Regular/Medium/SemiBold/Bold) stands in for Barlow.
* **JetBrains Mono** (Regular) stands in for IBM Plex Mono.

Both are SIL OFL 1.1, and their license texts are in `xp-kit/Fonts/efb/OFL.txt`. The CSS families
are `EfbSans` and `EfbMono`. To switch to Barlow / IBM Plex Mono later, replace the font files and
the `@font-face` sources in `efb.css`.

## 11. Verification (offline)

| Check | Where |
| --- | --- |
| Core rules, entries, FQMS view, JSON writer (NaN → null, overflow, nesting), action grammar | `test_efb_core.c` (clang + MinGW `-std=c11 -O2 -Wall -Wextra -Werror`) |
| Devices, touch (ordered, positionless release, wake), web/native drawing, door veto, slides, presets, typed payload/fuel/radios, failures, sounds, checklists, navigation/ack, frames (realistic, NaN inputs, cold & dark, standby tablet, cap), reload | `test_efb_adapter.c` (mock XPLM incl. `XPLMCreateAvionicsEx`) |
| Legacy flyPad renderer and ISCS window unchanged | `test_efb_render.c` |
| Page logic on 18 real native frames: every section, unavailable values, keypad, optimistic navigation, door/slide mode, filters, units, toast, blank/stale screens; every posted verb is a native verb; offline package | `tools/test_efb_page.py` + `tools/efb_page_check.js` (JavaScriptCore) |
| Geometry, touch mapping to the old regions, native constants | `tools/test_efb_cockpit_devices.py` |
| Shared-memory ABI (C vs Python, v8, including every view's scaled size), Windows C/C++ ABI | `tools/test_xp_kit_transport.py`, `tools/build_windows_display_host.py` |
| SD slides edit (anchors, idempotent, JS lookup, whole-bundle parse), SD frame slide flags | `tools/test_patch_xp_kit_sd.py`, `test_sd_data.c` |
| Real page in WebKit at 1440 x 1000, including touches replayed at device coordinates (tap, keypad entry, slider drag) | `tools/render_efb_page.m`; previews `docs/control-anim-previews/efb-home.png`, `efb-ground-doors-slides.png`, `efb-loading-payload-keypad.png`, `efb-ops-checklists.png`, `efb-settings-sounds.png` |
| The real display helper (`A380XDisplayHost`, views 7/8) applies native frames, captures 1440 x 1000 textures, and returns a keypad entry made by replayed touches through the action ring | `test_xpkit_host.c` with `A380X_EFB_JSON_0/1` fixtures |

## 12. Not verified in the simulator

Nothing in this phase has run inside X-Plane yet. Check in the simulator:

1. **Load.** `Log.txt` shows no EFB device error. `a380x/xp12/efb/devices_active` = 1. Both
   `…/device_bound` = 1 after the cockpit loads. `a380x/xp12/xpkit/captain_efb_active` = 1.
2. **Picture.**
   * Both tablets show the web EFB upright and unmirrored, filling the glass.
   * The old atlas picture is gone.
   * Brightness follows Settings and the tablet stays lit cold & dark.
   * If a tablet is mirrored or upside down, the device UV convention differs from the
     PFD/MFD screens.
3. **Touch.**
   * Taps land where the finger points (rail, tabs, door tiles, keypad keys).
   * Slider drags follow the mouse.
   * Releases outside the glass still complete a tap.
   * The cursor is the pointing finger.
4. **Latency.** A tap to a visible change takes about 100-250 ms (touch → frame → page → action →
   frame → capture). Check it feels acceptable.
5. **Doors and slides.**
   * Arm M1L, then try to open it from the tablet, the ISCS window and a `door_toggle_1` key
     binding: it stays closed.
   * The SD DOOR page shows "S" on armed, closed doors.
6. **Fallback.** Kill `A380XDisplayHost`: both tablets switch to the native flyPad renderer within
   about 1 s and taps work there. The helper restarts with the plugin.
7. **Windows.**
   * `install_xp_kit_cockpit.py` installs only the macOS plugin, but the cockpit objects are shared.
   * The Windows `A380XAvionics.xpl` installed today predates this change and creates no EFB
     devices. After the geometry install, Windows tablets therefore stay blank until a matching
     Windows plugin is installed. The PFD/ND devices already have the same dependency.
   * A Windows plugin from this source creates the EFB devices and, without the WebView2 host,
     draws the native fallback.
   * The WebView2 host has the EFB views and compiles (C/C++ ABI checked), but has never run.

## 13. ISCS floating window (unchanged)

Open the window from the top-level **ISCS > Open / Close** menu, or bind `a380x/xp12/efb/window_toggle`.
It has eight tabs:

* Startup/Situations
* A/C + Sim Config
* Loading/Perfo
* Ground Services
* Fault Scenarios
* Sounds/Addons
* Joystick/Actions
* Settings/Accounts

It shares the native aircraft data and commands with the tablets, including the slide rule: its door
buttons and ALL 20 DOORS keep armed doors closed. CG position, deicing, ADIRU alignment,
random-fault scheduling, external account logins and a full take-off calculator lack a backing model
there and are labelled unavailable. Simulator-configuration switches hold session-local UI state.

## 14. Honest limitations

* The MSFS React/Coherent flyPad does not run in X-Plane. The page is an original port of the
  approved design, on the FBW flyPad page model.
* Navigraph, ChartFox, SimBrief, GSX, Hoppie/online ATC and account authentication are not emulated.
* EFB state is session-local: slides, the payload plan, options, the cabin preview and settings
  reset on aircraft reload. Saving them is part of Phase 3.
* Payload is X-Plane's single `m_fixed` value: the ACF has no payload stations. The per-class split
  exists only in the EFB plan.

## Port onto GitHub main (fc361ec, "A380X-XP12 Latest") — 2026-11-05

Phase 1 exactly as built in `build/a380x-build-efb-20261103-175125` (every native source matches that
build report's sha256; Phase 2 is not included), merged into GitHub main:

* **Protocol v8.** GitHub main had its own v7 (every PFD / ND / MFD / SD view captured at its kind's
  scale, `a380x_du_scale.h`); Phase 1's v7 added the two EFB views. v8 is both: views 7/8 are the EFB
  tablets at 1440 x 1000 and scale 1, `XPKIT_MAX_VIEW_BYTES` covers the larger 7/4-scale MFD/SD.
  `xpkit_view_css_width/height()` give each view's layout size (768 x 1024, or 1440 x 1000 for the
  EFB); the macOS helper sizes its web views and snapshot rectangles with them, the Windows helper
  keeps GitHub's `RasterizationScale = xpkit_view_scale()` (1 for the EFB). `tools/xp_kit_transport.py`
  reads the scales from `a380x_du_scale.h` and its test compares every offset and every view size with
  the compiled C header.
* **Geometry** is written with `tools/obj8_numbers.fmt` (fixed point, no exponent), like every other
  OBJ writer on main.
* **Windows.** The `win_x64` plugin is built as main builds it (`--xpkit --xpkit-windows`, MinGW,
  `A380X_XPKIT`), so its PFD / ND devices are unchanged. It now also creates the two EFB devices; with
  no WebView2 host installed (main ships none) the xp-kit helper is absent, `efb_web_live()` is false
  and both tablets draw the native flyPad renderer. The WebView2 host is still built (ABI check) but
  not shipped. Not run on Windows.
* The SD DOOR slide edit is applied over the shipped bundle (`60189976…`, v1-patched) and gives the
  same bytes as Phase 1 (`69937b96…`).
