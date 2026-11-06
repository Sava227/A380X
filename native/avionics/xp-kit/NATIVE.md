# FlyByWire PFD library in the A380 3D cockpit

The macOS `--xpkit` build replaces the native-translated PFD presentation with
the original xp-kit-compiled FlyByWire PFD bundle. Since 2026-10-20 both PFDs are
captured at their full 768×1024 logical size (shared-memory v6, 30 Hz helper
timer) and presented 1:1 on dedicated X-Plane cockpit devices
`a380x.xp12.pfd.captain` / `a380x.xp12.pfd.first_officer` (lighting channels 19/20,
`tools/pfd_cockpit_devices.py`). Devices redraw on demand (new frame or power/health
change). Clicking a PFD in the 3D cockpit toggles its X-Plane pop-up; commands
`a380x/xp12/pfd/<captain|first_officer>/popup` and `.../popout` are bindable.
While a device is bound the legacy atlas regions captain `(0,32,320,448)` and
first officer `(704,32,1024,448)` are not drawn; they remain the fallback when the
device API is unavailable (`a380x/xp12/xpkit/pfd_devices_active` hides the device
quads). No separate panel, browser tab or server is needed.

`A380XDisplayHost` is a separate, aircraft-local macOS process using system
WebKit. Six views (PFD, ND/OANS and MFD pairs) run at 768×1024. PFD/ND snapshots
are 384×512; MFD snapshots preserve 768×1024 for dense page text, at up to 20 Hz.
X-Plane uploads complete, sequence-checked frames on its draw callbacks.
The offscreen views receive an explicit animation clock, since WebKit suspends
normal requestAnimationFrame callbacks in hidden windows. The PFD's CSS animations
(FBW blinking warnings) are advanced on the same clock (`animation-clock.js`,
inserted by `tools/patch_xp_kit_pfd.py`).

The native plugin remains the source of truth for all sensors, ADIRS alignment,
source switching, failure validity, power, individual DU brightness, EFIS baro
and supported FCU/guidance modes. Its guidance state is encoded into the library's
documented ARINC labels and discrete bits. The three PRIM label sets describe
the same native controller; this does **not** simulate three redundant PRIMs.
Unsupported flight-control laws, autoland, complete FWS and navigation functions
are not synthesized. In particular the PFD's lower FWS memo/limitations regions
continue to indicate unavailability; the existing native EWD keeps its supported
engine alerts/memos. NDs now use the [native ND/OANS extension](../xp-kit-nd/NATIVE.md);
MFDs now use the [MFD renderer extension](../xp-kit-mfd/README.md), retaining their
separate native FMS/UI controller and the translated FlyByWire page modules.

If helper startup fails, a renderer dies or its frames are more than one second
old, the plugin uses the existing native PFD renderer. It never freezes the last
library frame as if it were current. Plugin disable/unload terminates the helper
and removes only its own temporary shared-memory file. Helper exit also follows
parent-process death.

Read-only health datarefs:

- `a380x/xp12/xpkit/helper_running`
- `a380x/xp12/xpkit/captain_active`
- `a380x/xp12/xpkit/first_officer_active`
- `a380x/xp12/xpkit/errors`
- `a380x/xp12/xpkit/captain_presented_frames`
- `a380x/xp12/xpkit/first_officer_presented_frames`

Build and install while X-Plane is closed:

```sh
python3 native/avionics/build.py --sdk build/sdk/SDK --xpkit --output-dir build/new-xpkit-build
python3 tools/install_xp_kit_cockpit.py build/new-xpkit-build
```

The installer verifies hashes/signatures and backs up the previous Mac plugin
and current simulator/aircraft preferences under `build/backups`. It changes
only this project's `output/A380X-XP12/plugins/A380XAvionics/mac_x64` package;
the Windows binary and other installed aircraft are untouched.

The packaging source is `package_xp_kit_cockpit.py`; it bundles the previously
validated PFD build, runtime, font and images into one local HTML asset and
retains source provenance plus GPL licensing. The browser demo's synthetic
scenarios are never linked into the aircraft plugin.

## Simulator verification — 2026-10-15

The installed universal Mac build is `build/xp-kit-cockpit-20261015-c`.
`build/xp-kit-cockpit-validation-4/native-validation.json` records six passing
live checks on X-Plane 12.4.3 with this exact aircraft: native channel health,
actual panel-texture presentation in 3D cockpit view, paused rendering, physical
FCU speed command propagation, independent captain DU blanking, and automatic
stale-frame fallback/recovery. All moved test controls were restored, with zero
renderer errors. Native-renderer screenshots are saved beside that report.

The full existing native core/ADIRS/FCU/EFIS/EWD/MFD/FMS test suite also passes.
`tools/test_xp_kit_bridge.cjs` checks the new native clock, local-file loading,
ARINC encoding, side routing, brightness, pause and lost-feed behavior.

Earlier simulator launches stalled in the Mac graphics subsystem, including a
comparison with the new helper disabled. The final normal-app launch passed the
integration checks, including after closing the old browser demo/server. The
root cause of those earlier stalls is not established. The internal disk had
only roughly 260–320 MiB free; long-duration flight stability is not certified.
