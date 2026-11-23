# Binary patches in the shipped Mac avionics plugin

`output/A380X-XP12/plugins/A380XAvionics/mac_x64/A380XAvionics.xpl` carries two small patches that were made directly in
the binary (arm64 slice only, the slice X-Plane loads on Apple Silicon), because no source rebuild was possible at the
time. The x86_64 slice and the Windows plugin do not have them. **Make the same changes in the source before the next
rebuild**, or they will be lost.

## 1. Autopilot stick-override check (2026-11-12)

- **Symptom:** AP1/AP2 engaged and disconnected in the same frame:
  `AP1 disconnected: joystick axis 0 (pitch/roll/yaw) at -0.01, more than 20% off centre`.
- **Cause:** `pilot_override()` in `native/avionics/a380x_fcu_adapter.inc` tests `|axis - 0.5| > 0.2`, assuming axes
  from 0 to 1 centred at 0.5. X-Plane 12 reports the pitch/roll/yaw axes centred at 0 (-1..1), so a centred stick read
  as fully deflected.
- **Patch:** `_pilot_override+0x16c` (file offset `0x6b3a0` in the thin arm64 slice):
  `1e3c1001 fmov s1, #-0.5` → `1e2703e1 fmov s1, wzr`. The test becomes `|axis| > 0.2`, so the AP still disconnects
  when the stick or rudder moves more than 20 %.
- **Source fix to make:** in `pilot_override()`, read `sim/joystick/joy_mapped_axis_value[1..3]` (-1..1) instead.

## 2. FCU / EFIS LCD colour #16145E (2026-11-12)

In `_draw_panel` (draws the FCU strip and the 8 FCU/EFIS LCD windows into the panel texture):

- **Backlight colour** (× brightness): (0.247, 0.137, 0.953) → (0.0863, 0.0784, 0.3686) = `#16145E`.
  `mov`/`movk` at `0xfa60`/`0xfa70`/`0xfa80` (FCU strip) and `0x10648`/`0x10660`/`0x10670` (other windows).
- **Inactive-label "ghost" colour:** (0.212, 0.125, 0.839) → (0.0739, 0.0717, 0.3246), the same ratio to the backlight.
  `mov`/`movk` at `0x10130`/`0x10140`/`0x10150` and `0x10680`/`0x10690`/`0x106a4`.
- The unlit (power off) colour is unchanged.
- **Source fix to make:** change the same constants in the FCU/EFIS panel renderer.

After both patches the binary was re-signed ad hoc (`codesign --force --sign -`).
