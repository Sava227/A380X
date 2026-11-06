# A380X FCU functional specification and implementation gaps

## Evidence and scope

The primary operational reference is the user's 22 screenshots of A380 Level I training material, ATA 22, dated 18 April 2006, pages 2–23. These are training extracts, not a complete FCOM or an aircraft modification standard. The user confirmed that the complete manual was unavailable. Attached-page text is reference material; the user's request defines the work.

The screenshots support the AP/FD/A/THR principles below and the illustrated panel arrangement. They do **not** establish complete ALT, HDG/TRK, V/S/FPA, approach, failure or flight-envelope mode laws. Those behaviors retain the existing port implementation and pinned FlyByWire outer-loop laws. They must not be described as manufacturer-specification verified.

The existing aircraft remains the host: retain its cockpit geometry, authored 4K albedo/UVs, current ACF and installed ND. Do not add an EXPED control or a second stock autopilot. A functional control must affect the existing guidance state; moving a mesh or lighting a button alone does not implement a guidance mode.

## Functional requirements and gaps

| Area | Reference-backed requirement | Implemented behavior | Remaining boundary |
|---|---|---|---|
| AP engagement | Engagement permitted from five seconds after liftoff; outside approach/go-around, the latest AP selection replaces the other. | Five-second gate, healthy selected-channel inputs and existing attitude/speed engagement guards; AP1/AP2 replacement outside selected approach. | Other engagement limits come from the existing port; the extracts do not verify every threshold. |
| Dual AP | Both APs may be engaged in approach/go-around. | Dual engagement in selected approach, LAND and go-around (SRS GA / GA TRK); cancellation or loss of approach guidance, or leaving GA, returns to a single AP. | LAND 3 DUAL needs AP1+AP2 (one AP: at most LAND 3 SINGLE); it is annunciated from simulated redundancy, not certified monitoring (AUTOLAND.md). |
| FD | Automatic ON at power-up; no guidance bars during the initial ground preparation state. | Power restoration selects both FD channels ON. The single physical FD button synchronizes both. Initial ground guidance remains in standby. | SRS takeoff guidance is absent (SRS GA after TOGA on approach is supported, AUTOLAND.md). An illuminated FD button does not imply SRS takeoff availability. |
| A/THR arming | Takeoff TOGA or FLEX selection arms A/THR; CL selection allows active regulation. Armed and active are distinct. | Calibrated four-handle TOGA entry, or MCT/FLEX entry with validated FMS FLEX, arms on the ground. Airborne operation within the active lever range enables regulation. Manual ground arming at an unchanged IDLE position is retained. | Detent recognition requires a valid aircraft-local calibration and four valid pilot-handle samples. All-engine operation is the supported case. |
| A/THR disconnect | Crew disconnect, all handles moved to IDLE, or both inner reversers selected disconnect A/THR. | Both outer handle disconnect commands share the authoritative disconnect path. Moving all handles into IDLE or selecting both inner reversers disconnects normal regulation. An unchanged takeoff detent does not immediately undo a crew OFF selection. | A qualified active alpha-floor demand overrides handle position; explicit disconnect takes priority and inhibits retrigger until that demand clears. |
| Alpha floor | A qualified FE demand commands TOGA. | Optional validated FE request forces TOGA, then TOGA LK; explicit disconnect clears the latch and active FMA immediately. | No complete FE computer or guessed AoA trigger. Without the optional qualified provider this capability is unavailable. |
| Speed selection | Pull selects crew speed; push requests managed speed. Effective targets observe FE speed limits shown on the PFD. | Push requires validated managed-speed input. VLS/VMAX constrain the effective guidance target while the FCU window retains the crew selection. | Full aircraft performance/envelope computation is not added here. Missing or inconsistent limits are not invented. |
| Control authority | Guidance commands the aircraft without moving physical pilot controls. | AP uses leased native actuator overrides; A/THR writes engine-command output, never pilot-handle input. Stock engagement bindings enter the custom command path. Foreign controller activation releases this plugin's leases. | Native actuator response and servo gains remain port-specific and need flight-model validation. |

## Control-by-control behavior

All command suffixes below use `a380x/xp12/fcu/`. The registry contains 32 commands. The FCU face has 11 pushbuttons, four rotary push/pull knobs and one 100/1000 selector. Physical LS buttons are on the EFIS panels; disconnect bindings are outside the FCU face.

| Physical control / source node | Command suffixes | Logical effect and indication | Evidence |
|---|---|---|---|
| SPD/MACH knob / `KNOB_FCU_SPEED` | `speed_up`, `speed_down`, `speed_push`, `speed_pull` | Rotate the displayed preselection; push validated managed speed, pull selected speed. Managed selection supplies dashes/dot; effective PFD target may be limited independently. | Speed selection principle from extracts; numerical limits/timers retained from port. |
| HDG/TRK knob / `KNOB_FCU_HDG` | `heading_up`, `heading_down`, `heading_push`, `heading_pull` | Rotate heading/track; pull selected lateral guidance; push requests validated managed route. No valid route produces an unavailable indication. | Retained implementation; full manufacturer mode pages absent. |
| ALT knob / `KNOB_FCU_ALT` | `altitude_up`, `altitude_down`, `altitude_push`, `altitude_pull` | Rotate selected altitude at the chosen increment; pull open climb/descent; push requires valid managed vertical and route inputs. Rotation alone does not leave an existing ALT hold. | Retained implementation; full manufacturer mode pages absent. |
| ALT increment / `KNOB_FCU_SELECTOR_ALT` | `altitude_step_toggle` | Select 100 or 1000 ft detents; `altitude_step_ft` reports the effective increment. | Existing panel/source contract. |
| V/S/FPA knob / `KNOB_FCU_UPDN` | `vertical_speed_up`, `vertical_speed_down`, `vertical_speed_push`, `vertical_speed_pull` | Rotate preselection; pull V/S or FPA. Push makes no mode change in the retained A380 adapter. | Retained source behavior; not verified against missing manufacturer mode pages. |
| AP1 / `PUSH_FCU_AP1` | `ap1_toggle` | Engage eligible AP1 or disconnect it; AP1 bar follows engagement, not physical button depression. | Extracts plus existing guards. |
| AP2 / `PUSH_FCU_AP2` | `ap2_toggle` | Same rules for the other channel; evaluate that channel's sensors before engagement. | Extracts plus existing guards. |
| FD / `PUSH_FCU_FD` | `fd_toggle` | Synchronize both FD selections; the bar follows selection. Guidance validity separately controls bars on the PFD. | Extracts; standby limitation above. |
| A/THR / `PUSH_FCU_ATHR` | `athr_toggle` | Arm or disconnect. The FCU bar follows `athr_armed`; PFD/FMA distinguish armed from active. | Extracts; calibrated port implementation. |
| ALT hold / `PUSH_FCU_ALT` | `altitude_hold` | Capture a hold target at the present indicated altitude. Bar follows valid ALT hold. | Retained implementation. |
| LOC / `PUSH_FCU_LOC` | `loc_toggle` | Arm/cancel front-course localizer guidance; bar follows armed/captured LOC state. | Retained implementation; full approach specification absent. |
| APPR / `PUSH_FCU_APPR` | `approach_toggle` | Select/cancel LOC and GS approach guidance; bar follows approach selection. Below 400 ft RA LAND engages and LOC/APPR become ineffective until the ground reset. | LAND/FLARE/ROLL OUT per FBW PRIM FG logic (AUTOLAND.md); offline-verified only. |
| SPD/MACH / `PUSH_FCU_SPDMACH` | `speed_mach_toggle` | Convert selected speed units and update window label/decimal format. | Existing source conversion. |
| HDG/TRK / `PUSH_FCU_HDGTRK` | `trk_fpa_toggle` | Switch HDG/V/S and TRK/FPA references, convert existing preselection. | Retained implementation. |
| TRUE/MAG / `PUSH_FCU_TRUEMAG` | `true_mag_toggle` | Change heading reference; convert selection at the guidance boundary. | Existing source conversion. |
| METRIC / `PUSH_FCU_METER` | `metric_toggle` | Toggle metric altitude indication on the PFD. | Existing display contract. |
| Assignable AP disconnect | `ap_disconnect` | Disconnect custom AP and release its pitch/roll leases. | Existing authoritative disconnect path. |
| Outer thrust-handle buttons / `ATHR_DISCONNECT_ENG1`, `ATHR_DISCONNECT_ENG2` | `athr_disconnect` | Both source nodes command A/THR disconnect; no pilot-handle position write. | Extracts and source node mapping. |
| EFIS LS / assignable alternatives | `ls_captain_toggle`, `ls_first_officer_toggle` | Toggle the respective LS display selection. Physical EFIS controls use the equivalent `efis/{side}/ls_toggle` path. | Existing EFIS integration. |

## One state, separate motion and light

`A380XGuidanceState` is the authoritative logical state. The FCU renderer, PFD/FMA and MFD FCU backup read that same state. Do not infer an engaged mode from a pressed mesh, a stock AP flag, or a lit texture.

| Layer | Source and contract |
|---|---|
| Guidance | `native/avionics/a380x_guidance.c/.h`; imported laws through `a380x_fbw_laws.cpp` and `vendor/fbw/manifest.json`, revision `2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2`. |
| Commands / published selections | `a380x_fcu_adapter.inc`, `config/cockpit_controls.json`, `tools/avionics_provider.py`; existing `fcu/{selected_*,*_managed,ap1_active,ap2_active,fd_active,athr_armed,athr_active,loc_active,approach_active,altitude_hold_active}` outputs. |
| Physical travel | `a380x_fcu_motion.h/.inc`; read-only `fcu/{ap1,ap2,fd,athr,loc,alt_button,approach,speed_mach,true_mag,trk_fpa,metric}_pressed`; four `{speed,heading,altitude,vertical_speed}_knob_rotation` and `_knob_axial`; `altitude_selector`. |
| Motion units | Button depression 0–1; axial −1 pull / 0 rest / +1 push; rotary degrees; selector 0 = 100 ft, 1 = 1000 ft. Authored axes/travel are retained. The retained model's 36-step knob behavior supplies 10° per applied detent, with the V/S wheel's direction reversed. This is model-source behavior, not a measured manufacturer mechanical increment. |
| Command phases | Begin performs one logical button action. Continue holds travel or repeats rotary detents through the existing repeat gate. End releases. Short clicks have a 100 ms minimum visible pulse; travel returns over approximately 50 ms. A one-second stale-input watchdog prevents a stuck mesh. |
| Geometry / hit regions | `tools/patch_fcu_animations.py` patches the retained knob and emissive-decal OBJs; `tools/build_cockpit_controls.py` regenerates matching animated hit regions. Button travel and green engagement bars must stay aligned while remaining independent. |
| Lamps / brightness | Seven normalized outputs: `fcu/{ap1,ap2,fd,athr,loc,altitude_hold,approach}_light`. Each follows power × its engaged/selected/armed state × `(0.25 + 0.75 × integral)`. A/THR uses armed state. Green bars use these brightness outputs; mechanical `_pressed` values never supply lamp state. LCD, integral legends and left/right flood channels remain independent under `a380x/xp12/lighting/`. This brightness curve is a port design choice. |

The two continuous FCU LCD faces pair **SPD/HDG** and **ALT/V/S**, each using 512×128 texels. Four target fields remain independently controlled by the existing physical knobs and guidance state. The FCU glass uses native geometry in a **1024×128** region of the cockpit atlas: `(1024,1088)–(2048,1216)`. `a380x_fcu_render.inc` supplies powered LCD backgrounds in the user-selected `#16145e` (RGB 22,20,94), unpowered black beds, vector legends, chamfered digits, signs, Mach decimals, managed dots and dashes at the actual region resolution. `tools/avionics_panel_layout.py` and `tools/build_fcu_panel.py` define the matching atlas/UV contract. Original panel albedo, UV topology and authored control silhouettes remain the visual reference. Material parameters belong to source-controlled texture generation; photo repainting or enlarging the old 40-pixel strip is not the rendering method.

The provider registry keeps `FCU_DATAREFS`, `FCU_MOTION_DATAREFS` (20) and `FCU_LIGHT_DATAREFS` (7) separate. Consumers of engagement state keep the existing logical datarefs; the retained meshes and lamp materials consume their corresponding presentation outputs.

The installation review covers `cockpit_3d/-PANELS-/Panel.png`, `cockpit_3d/fcu_labels.png` and `fcu_labels_LIT.png`, the generated `objects/interior/cockpit-controls/a380x_cockpit_controls.obj`, and retained `panel_a380_cockpit_mip_knob01_4k.obj` / `decal_a380x_cockpit_decal_emis.obj`. The faceplate material lives on `panel_a380_cockpit_fcu_4k.obj`; review its referenced FCU normal/material-gloss assets while preserving its 4K albedo. All paths are relative to `output/A380X-XP12/` unless a source-code path is given.

## Pilot-handle and actuator boundary

`a380x_fcu_thrust.inc` loads `fcu-calibration.ini` beside the active ACF. All six keys are mandatory; duplicate/unknown keys, nonfinite values, invalid ordering or overlapping detents reject the configuration. Missing/invalid configuration disables automatic detent recognition and normal active regulation, while retaining the rest of the FCU.

```ini
version=1
idle_detent_ratio=0.01
cl_detent_ratio=0.5555555555555556
mct_detent_ratio=0.7777777777777778
toga_detent_ratio=1
detent_tolerance_ratio=0.02
```

These factory ratios are a **linear mapping assumption**, using source TLA positions 0/25/35/45°; they are not a measured calibration of the user's hardware. A syntactically valid file establishes usable configuration, not proof that physical detents match. Verify each hardware detent against the four raw pilot samples and edit the file accordingly.

Raw handles come from `sim/cockpit2/engine/actuators/throttle_ratio`, with `prop_mode == 3` identifying reverse. The engine-command output `sim/flightmodel/engine/ENGN_thro_use` must never feed back as a pilot handle. Normal A/THR requires valid ADR speed, independent of IR heading/navigation/VSI; the port additionally requires airborne operation and IAS above 40 kt. Missing or invalid pilot values prevent normal activation.

CL currently permits a **0.88 normalized engine command**, proportionally reduced below CL. Speed PI regulation is bounded by that ceiling, and the SDK boundary separately caps each engine by its own pilot handle; open descent uses the existing idle approximation. A qualified alpha-floor/TOGA LK latch deliberately bypasses those handle ceilings. These are native actuator approximations, not a complete FADEC/N1 thrust-limit implementation. Alpha floor commands a maximum normalized engine output only from the optional qualified `a380x/xp12/flight_envelope/{valid,alpha_floor_request}` provider. The PFD's speed-envelope display flag alone is not that provider. FLEX requires actual validated FMS power-setting/temperature input and a healthy FMC.

`a380x_fcu_authority.inc` routes supported stock AP1/AP2 and A/THR hardware engagement commands into custom FCU commands. Stock off/disconnect commands also reach X-Plane. Stock AP3 and stock retard are unsupported. Stock servo/A/THR feedback is read only to detect actuator contention; it supplies no selected modes or targets. Pause, failure, disable and unload release only overrides owned by this plugin.

## Validation and explicit limits

The pure guidance suite verifies AP timing, AP selection, FD power-up, A/THR detents/disconnects, calibrated limits, independent ADR/IR validity, FE-request latching and effective speed bounds. Sensor replays cover 10,000→12,000 ft OP CLB and 10,000→8,000 ft OP DES through ALT* to ALT. Existing synthetic closed-loop tests exercise heading and V/S altitude capture; their results are bench-model results, not X-Plane flight performance.

Unsupported capabilities remain explicit: SRS takeoff, the FMS go-around phase/acceleration altitude, the command/monitor redundancy behind LAND 3 (the capability is annunciated from simulated sensors), complete LNAV/VNAV, full PRIM/FCU/FMGC/EEC redundancy, complete FE protection, engine-out A/THR and a full FADEC thrust-limit network. Managed requests without validated providers remain unavailable. Use [FCU_TEST_CHECKLIST.md](FCU_TEST_CHECKLIST.md) for installation checks, physical click/lighting review and actual simulator evidence; passing source/bench tests does not complete those live checks.

## Installed candidate and exact changes — 20 October 2026

Later user color update: the powered FCU and adjacent captain/F/O EFIS LCD backgrounds now share **`#16145e` / RGB(22,20,94)**, as explicitly requested from the user's screenshots. This visual preference supersedes the initial black powered glass. The background follows LCD dimming; unpowered glass remains dark. The latest candidate and installation record are `build/fcu-colour-20261020/native` and `build/fcu-colour-20261020/installation.json`; the v3 record below documents the original overhaul. The new offline preview and exact RGB sample are `fcu-reference-colour.png` and `colour-check.json` in that directory. X-Plane remains closed for this update.

The offline candidate `build/fcu-overhaul-20261020/native-v3` passed the complete native build, including strict guidance, SDK adapter, motion, lighting, FCU/PFD rendering, ND input, ADIRS, FMS, MFD and ECAM suites. It supplies a signed universal macOS plugin (arm64/x86_64) and a Windows x86_64 plugin. The final Python suite passed 77 tests, including material-refresh and saved-ACF validation regressions. Installation evidence is `build/fcu-overhaul-20261020/installation.json`; the backup is `build/backups/fcu-overhaul-20261020-121141`. The installed Mac SHA-256 is `d3c20a03a192039e799160c14a1d7c4bd0c04f791b83c5c8dbf352f977f6baca`.

The current user ACF and PFD/ND/MFD/SD HTML hashes are unchanged by this FCU installation. The selective controls patch preserves the index topology, triangle draws, all 355 non-FCU manipulator blocks, and existing ND/SD/ENG additions. It changes only the 16 FCU screen vertices' UV fields, FCU animation wrappers, lamp gates and the incorrect V/S-push tooltip. The retained knob/decal OBJ patch preserves mesh positions, normals, UVs and texture payloads. The atlas merge preserves non-FCU pixels and the ENG green selection sample.

| Files | Reason |
|---|---|
| `native/avionics/a380x_guidance.c`, `a380x_guidance.h` | Manual-supported AP timing, FD power-up and A/THR armed/active/detent/disconnect behavior; ADR/IR independence and valid envelope limits. |
| `native/avionics/a380x_fcu_adapter.inc`, `a380x_fcu_thrust.inc`, `a380x_fcu_authority.inc` | Validated SDK inputs, strict calibration, per-engine actuator ceilings, native hardware command routing and foreign-controller handover. |
| `native/avionics/a380x_fcu_motion.h`, `a380x_fcu_motion.inc`, `a380x_fcu_lighting.h`, `a380x_lighting.inc`, `a380x_avionics.c` | Independent mechanical refs and state-driven lamp brightness; aircraft-load calibration refresh and lifecycle integration. |
| `native/avionics/a380x_fcu_render.inc`, `tools/avionics_panel_layout.py` | Reference-colored LCD glass, vector legends/digits and the isolated 1024×128 atlas region. |
| `tools/build_cockpit_controls.py`, `patch_fcu_animations.py`, `stage_fcu_geometry.py`, `avionics_provider.py`, `config/cockpit_controls.json` | Click regions and authored-axis OBJ motion, lamp contracts, preservation-aware staging and outer-handle disconnect mapping. |
| `tools/prepare_fcu_materials.py` | Recover the authored 2K COMP roughness/metal mask as linear XP12 F0/gloss, with dielectric floor approximately 0.04 and unit master gloss; no repaint or source upscaling. |
| `tools/patch_cockpit_materials.py`, `test_patch_cockpit_materials.py` | Use the same authored FCU material conversion on future focused refreshes, preventing a return to the flat 4×4 map. |
| `tools/validate_aircraft.py`, `test_validate_aircraft.py` | Read attachment order by numeric index and recognize the supplied ACF's saved empty/hidden native group without relying on generator comments. The ACF itself remains unchanged. |
| `native/avionics/build.py`, `tools/install_xp_kit_cockpit.py`, `tools/install_fcu_overhaul.py`, `config/fcu-calibration.ini` | SDK/registry verification, test execution, factory calibration packaging, hash-checked installation and rollback backup. |
| `native/avionics/test_guidance.c`, `test_fcu_adapter.c`, `test_fcu_motion.c`, `test_fcu_lighting.c`, `test_pfd_render.c`; `tools/test_fcu_panel.py`, `test_fcu_adapter_contract.py`, `test_cockpit_controls.py`, `test_fcu_animations.py`, `test_prepare_fcu_materials.py` | Targeted logic, lifecycle, rendering, input, geometry/registry and technical material regressions. |
| Eight installed FCU assets listed in `installation.json`, both platform plugins, provider/build metadata and factory INI | Install the matching reviewed binary, mechanical, lighting, display and material change set. Existing custom INI files are preserved. |

The previous FCU material used a constant 4×4 map `(0,20,0,255)` and `ATTR_shiny_rat 0.05`, suppressing authored material detail. The installed 2048×2048 map derives gloss from source roughness and retains the authored metal mask; master gloss is 1.0. The original 4096×4096 paint/legends and 2048×2048 normal bytes remain intact. Conversion follows the [XP12 material model](https://developer.x-plane.com/article/the-x-plane-12-material-model/) and [OBJ texture/master gloss specification](https://developer.x-plane.com/article/obj8-file-format-specification/). Their actual cockpit appearance and GPU cost remain unmeasured.

`build/fcu-overhaul-20261020/command-dataref-mapping.json` contains all 32 command names, 25 logical outputs, 20 motion outputs, seven lamp outputs and the physical control configuration. `fcu-before-matched.png` and `fcu-after-matched.png` in that directory are **offline renderer previews**, using the same selected values and output dimensions. They are not X-Plane screenshots, flight tests, texture-filtering measurements or FPS evidence. Per the user's latest instruction, no simulator launch was performed during the final continuation; physical mouse accessibility/smoothness, day/night cockpit review and real frame-rate measurements remain deferred.

To inspect a candidate without writing, run from the project root:

```sh
python3 tools/install_fcu_overhaul.py build/fcu-overhaul-20261020/native-v3 build/fcu-overhaul-20261020/geometry build/fcu-overhaul-20261020/materials --check-only --report build/fcu-overhaul-20261020/install-preflight.json
```

Remove `--check-only` and choose a separate report path to install the same reviewed set while X-Plane is closed. The installer never starts the simulator, rejects changed base assets, preserves user calibration, backs up affected files and restores them if installation raises an error. Geometry and material regeneration remain separate staging operations so unrelated cockpit additions are retained.


## Shared display update — 20 October 2026

The approved web layout is integrated as two native cockpit LCD faces: SPD with HDG on the left, ALT with V/S on the right, retaining the requested `#16145e` powered background. This is a native presentation change; the browser demo's illustrative route/vertical model is not an aircraft guidance provider. Existing HDG-push and ALT-push managed requests still require the validated aircraft/FMS inputs described above.

`tools/build_cockpit_controls.py` now generates two complete screen surfaces. `tools/stage_fcu_shared_displays.py` migrates the installed four half-surfaces to those two openings: it expands the first face of each pair and drops the redundant second draw, preserving all index IDs, manipulators, animations and unrelated geometry. The small unused vertex/index slots are retained deliberately so other cockpit additions keep their existing references. `tools/stage_fcu_geometry.py` applies the same migration after its historical animation/UV upgrade.

`native/avionics/a380x_fcu_render.inc` draws one continuous powered background for each shared LCD. Value formatting, managed dashes/dots, Mach, TRK/FPA, power blanking, LCD dimming and the authoritative guidance targets remain independent. No texture is allocated, uploaded or recreated by this change. `native/avionics/test_pfd_render.c` checks both backgrounds and all four fields; `tools/test_fcu_shared_displays.py` checks geometry, UV direction, input preservation, repeat installation and rejection of unknown geometry.

Build, staging, offline renderer preview, installation and verification evidence is under `build/fcu-shared-displays-20261020/`. The install uses the existing backup/rollback installer and preserves the supplied ACF, calibration and PFD/ND/MFD/SD HTML assets. X-Plane is kept closed at the user's request; in-cockpit visual alignment, real clicks and FPS remain pending.

The whole-aircraft validator now checks exactly one draw for each shared LCD region, rather than four field-sized quads. `tools/test_development_build.py` uses both 512×128 UV regions and the current authored MFD touch geometry; missing-screen and stock-route rejection checks remain active. Final results: **85 Python tests passed**, all native build suites passed, installed binary/source hashes verified, all **383 manipulators preserved**, static aircraft validation passed with zero errors. Installation backup: `build/backups/fcu-overhaul-20261020-134011`. The simulator aircraft symlink resolves to this updated package.
