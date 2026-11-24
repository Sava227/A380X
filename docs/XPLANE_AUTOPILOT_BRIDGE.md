# A380X autopilot fix (installed 2026-11-15)

`plugins/A380XAvionics/mac_x64/A380XAvionics.xpl` was rebuilt from source. The source is the local git worktree `../avionics-src-20261115`, branch `ap-fix-20261115`, commit `404817f`, on top of `latest-sync-20261114`. It hasn't been pushed to GitHub.

## The problem

On the 2026-11-14 flight, AP1 engaged and disconnected in the same frame 8 times. The log said only `AP disconnected by the guidance: mode or sensor limit`:

- The engagement check reported no problem.
- Flight data was valid.
- The guidance dropped the AP because it saw its own output as not valid.

The same guidance code, run offline, never drops the AP:

- the flight conditions from the log;
- 3,000 random in-flight states;
- every input set non-finite one at a time.

The only input that reproduces the drop is a non-finite law input, for example the selected heading. So the cause in the sim is still unconfirmed.

## Changes

- **Guidance (`a380x_guidance.c`):**
  - An engaged AP with valid flight data no longer disconnects the moment the guidance becomes invalid. It holds, with neutral stick orders, so the FBW normal law keeps the flight path, while the FBW law restarts.
  - It disconnects only if the fault lasts 3 s.
  - Every disconnect records a reason (`A380X_DROP_*`).
  - An engaged AP always has a vertical mode: ALT hold if none was selected.
- **FBW law bridge (`a380x_fbw_laws.cpp`):** every input is checked. A non-finite one is replaced by a safe value and named. A non-finite output restarts the law and is reported.
- **AFS log (`Output/A380X-XP12_AFS.log` and Log.txt):**
  - **Fault:** `AP guidance fault (…): AP kept, law restarting`.
  - **Disconnect:** `AP disconnected by the guidance: <reason> - <law fault>`, followed by `AP drop snapshot: IAS … ALT … pitch … sel HDG … lat/vert modes … dt`.
  - **If the AP still drops:** these lines name the cause.
- **Binary patches now in the source** (`docs/BINARY_PATCHES.md`):
  - joystick axes centred at 0 (`pilot_override`);
  - FCU/EFIS LCD colour #16145E (`a380x_fcu_render.inc`);
  - no door-command veto for armed slides (`efb_door_intercept`), so the slide deploys.
- **APU BLEED / PACK FAULT legends:**
  - **Behaviour:** they light on X-Plane's `rel_APU_press` / `rel_HVAC` failures.
  - **Why:** the installed plugin, a build from 2026-11-10 whose source wasn't kept, already did this. The source didn't, so it was recreated: `tools/ovhd_lamps_contract.py` (regenerated `a380x_ovhd_lamps_table.h`), `a380x_ovhd_lamps.h` and `.inc`.
  - **Other differences:** apart from those three overhead-lamp functions, the installed plugin's code was identical to this source.

## Checks

- **Avionics tests:** the build's avionics + flyPad core tests pass. The test for the door veto was updated: an armed door now opens.
- **Guidance tests:** `test_guidance`, `test_autoland` and the closed-loop autoland bench pass.
- **Engagement harness:** the offline harness engages and keeps the AP for every non-finite input, including the selected heading that used to drop it.
- **Function comparison:** compared with the previous plugin, only the intended functions differ.
- **Not yet flown in X-Plane.**

## Not covered

- **Windows plugin:** not rebuilt; that needs the WebView2 SDK on the external drive. It has none of these changes.
- **Upstream FWS table check:** skipped, because the drive isn't mounted.

# Update 2: AP and A/THR through X-Plane's autopilot, Laminar A330 style (installed 2026-11-15 13:05)

## The 12:28 flight

- **AP1:** every engagement was cut by the joystick check. The rebuild had remapped the axis to -1..1, and the user's centred stick read -1.02.
- **A/THR:** armed but never active ("conditions met, activation pending"). It could not be reproduced offline with the same code and values.

## Changes (source commit `20541ce`, branch `ap-fix-20261115`)

**X-Plane AP/A/THR bridge (`a380x_xpap_bridge.inc`):** the FCU drives X-Plane's own autopilot and autothrottle, as Laminar's A330 does. This is our own implementation of that approach; no Laminar code was copied.

- **AP1/AP2:** X-Plane servos. Disconnect is `servos_off_any`.
- **A/THR:**
  - The pushbutton arms it, or turns it hard off.
  - Active (SPEED/MACH) in flight with the levers above IDLE and below FLX/MCT.
  - Armed only on the ground or at FLX/MCT and TOGA.
  - Off at IDLE.
  - Takeoff levers arm it and give the FD X-Plane TO/GA guidance (SRS). TOGA in flight with flaps out is a go-around.
- **SPD:** turn sets the target. Pull gives selected speed and SPEED. Push gives managed speed: the native FMS speed is written to X-Plane's target.
- **HDG:** pull gives HDG/TRK. Push gives NAV: X-Plane's heading mode steering the native FMS managed heading.
- **ALT:** turn sets the altitude, in 100 or 1000 ft steps. Pull gives OP CLB/DES (X-Plane level change). Push gives CLB/DES (level change at managed speed).
- **V/S:** pull gives V/S or FPA. Push levels off.
- **LOC / APPR:** X-Plane NAV / approach on NAV1.
- **Displays:** X-Plane's modes and targets drive the PFD FMA, FD bars and FCU windows.
- **Ownership:**
  - The FBW law stands down while X-Plane's servos fly.
  - The thrust-lever module hands the engines to X-Plane while its A/THR drives.
  - The authority guard lets X-Plane's commands through.
- **Logging:** the AFS log records "X-Plane AFS modes" changes and an "X-Plane AFS status" line every 10 s.
- **Switching back:** `a380x/xp12/afs/xplane_autopilot` = 0 returns to the native guidance (DataRefTool).

**Stick check:** the stick counts as deflected only when it is more than 20 % off both possible centres, 0.5 and 0.

**ACF (`acf_xpap.py`), as in the A330:**

| Setting | Value |
|---|---|
| `_autopilot_logic` | 12 (Airbus) |
| A/THR PID `_ott_atr_Kp` / `Ki` / `Kd` | 9 / 2.5 / 5.75 (were 0) |
| `_ott_asi_thro_per_sec` | 0.1 |
| `_alt_always_armed` | 1 |
| `_load_presel_on_vvi_hit` | 1 |

**Tests:** the build's avionics + flyPad tests pass. They run with the bridge off, as native guidance. **Not yet flown in X-Plane.**
