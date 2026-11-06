# FlyByWire ND/OANS in the A380X X-Plane cockpit

The approved ND/OANS mockup is now packaged as a native aircraft integration,
not as a browser page or a localhost service. The original FlyByWire ND and OANC
components are compiled through xp-kit, with explicit local-data/input adapters.
The existing PFDs keep their proven rendering and transport path.

## Pilot controls

- Use each physical EFIS mode/range selector for ARC, ROSE NAV, PLAN, VOR and LS.
- Select the minimum EFIS range (**ZOOM**) in ARC, ROSE NAV or PLAN for OANS.
- Airport zoom defaults to **0.1 NM**, ten times closer than the former 1 NM
  default. Mouse wheel or zoom commands select **0.05, 0.1, 0.2, 0.5, 1, 2,
  or 5 NM**. These values describe the 500-pixel compass diameter; the PLAN
  range ring shows half that value. The full 768-pixel view spans about 284 m
  at the new default and 142 m at the closest setting.
- Move the mouse over an ND to position the original **yellow cross cursor**.
  Drag the map to pan. Double-click for ADD CROSS, ADD FLAG, MAP DATA, erase and
  center-on-aircraft actions. A yellow pointer is distinct from an added map marker.
- The lower-right arrow opens the original airport panel. Mouse wheel zooms
  OANS independently on each side; no simulator camera zoom is consumed off-map.
- Commands under `a380x/xp12/nd/captain/` and `.../first_officer/` provide `popup`,
  `map_data`, `center`, `zoom_in` and `zoom_out` for bindings and diagnostics.
  Right-drag remains available for camera look.

At 0.2 NM and closer, airport geometry is drawn directly into a bounded
768×768 viewport from cached source paths, keeping taxiway and parking-stand
detail sharp during close inspection. Geometry redraws follow view/data
changes, not pointer movement. Labels preserve valid source taxiway IDs and
stand names; wider views retain their original detail thresholds.

The native cockpit draws the original yellow 80×80 cursor shape as geometry over
the retained ND texture, with the original first-officer rotation. Pointer movement
requests a device redraw without taking a WebKit snapshot or uploading map pixels.
The DOM cursor is hidden in native views; ordered mouse events still reach the
original map and panel controls. Loss of a live ND feed restores the simulator
cursor. The browser preview retains the corrected upstream cursor implementation.
PLAN waits for valid map parameters before positioning its ownship symbol.
Source adaptations use exact-match guards and leave the user's FlyByWire checkout
unchanged.

## Runtime and sources

`A380XDisplayHost` renders seven separate system-WebKit views: PFD slots 0/1,
ND slots 2/3, native-port MFD slots 4/5 and ECAM SD slot 6. Native sensor data, selected ADIRS sources and validity, power,
DU brightness, EFIS selections and the aircraft's own active FMS route arrive
through version-5 shared memory. State and pixel frames use sequence guards.
Stale data blanks the library view, while stale/dead renderers fall back to the
existing native cockpit rendering. A stale ND input channel cannot capture the
simulator's pointer; held map drags are cancelled on loss of the live feed.
Helper exit explicitly repaints both on-demand ND devices with the native
fallback and clears held pointer state. Dedicated device presentation is tracked
separately from the legacy atlas's shared texture uploads.

While aircraft input is fresh, the offscreen helper holds a standard macOS
user-initiated activity token to prevent background App Nap throttling. The token
allows ordinary system sleep and is released when input becomes stale. This is
renderer scheduling protection; it does not identify the cause of any previous
simulator or helper failure.

Both NDs have 768×1024 X-Plane cockpit-device surfaces at the authored NDL/NDR
locations, now fed by independent 768×1024 RGBA buffers. WebKit renders the
original vector text, SVG symbols and Canvas map geometry at their 768×1024
logical display size. The prior 384×512 downsample followed by 2× enlargement
has been removed for ND slots. PFD capture remains 384×512; MFD and SD remain
768×1024. A larger Retina snapshot is resolved once into the ND buffer; a 1:1
snapshot is copied without another interpolation pass.

The helper retains bitmap storage and its Core Graphics context. The plugin
allocates an RGBA8 ND texture once, then uses `glTexSubImage2D` for completed
frames. Minification and magnification use `GL_LINEAR`, with edge clamping and
no generated mipmaps. This preserves the cockpit device's full display area
without adding a sharpening filter or stretching a smaller source image.

The helper samples inputs at 20 Hz. ND content changes, map transitions and UI
interaction invalidate the capture; unchanged views retain their texture and
renew a liveness heartbeat. A 500 ms safety refresh covers asynchronous display
indications. Navigation changes still capture at the available helper update
rate. Native device redraws follow new frames, power/health transitions and
pointer movement. The `nd_profile` counters report capture/evaluation latency,
capture skips, uploaded bytes and device draws. These counters are transport
diagnostics, not simulator FPS or GPU timings.

Waypoint text uses measured font widths, symbol reservations and prioritized
screen-space placement. Airport labels account for runway label transforms and
preserve runway, selected and marker indications while optional labels yield
when crowded. Canvas clip state is restored after each frame so changing modes
does not accumulate previous clipping regions.

Their full-UV touch twins live in the cockpit control object. The old atlas
surfaces remain available as a fallback. The geometry
generator appends vertices/indices without changing existing draw ranges; the normal
cockpit builder can reproduce it with `--native-nd`. ND device quads hide when the
native device provider is unavailable.

OANS uses enabled local X-Plane `apt.dat` scenery, in scenery-pack priority order.
A one-shot Python reader runs on a serial background queue in the helper process,
never in X-Plane's flight/render callback. It is cached for four airports. This Mac
installation provides `/usr/bin/python3`; if it is unavailable or the scenery is
missing, OANS cannot load that airport rather than inventing a map. The aircraft
does not start a web server or contact Navigraph. Its native HTML blocks network
connections. Nearby-airport selection comes from X-Plane's navigation catalogue;
the loaded airport's geometry/name comes from its scenery block.

## Explicit limits

- Scenery is not an operational AMDB. No buildings or hold-short geometry are
  invented. Runway distances are geometric, not published declared distances.
  Taxi-edge lines are background geometry, not clearances or A380 wingspan approval.
- The native A380 FMS supplies its own active plan, not X-Plane's stock FMS route.
  Only its calculated straight TF/DF/CF legs are drawn as lines; discontinuities
  are not bridged, and AF/RF/hold turn paths are not invented. TO distance/bearing
  and ETA are geometric estimates, not full FMS performance predictions.
- Weather image layers, VNAV profiles and TCAS encounter classification are not ported; the original
  INOP/standby indications remain (a selected WX shows WXR INOP). TERRAIN on ND is provided by the
  plugin from X-Plane terrain probes and drawn by `terrain-layer.js` under the FBW symbology
  ([ND_TERRAIN.md](../../../docs/ND_TERRAIN.md)). The available received
  VOR fields are mapped; this is not a complete radio-navigation implementation.
- OANS BTV selections do not command X-Plane brakes. No stopping-distance guarantee
  is provided. The airport catalogue is nearby rather than the full Navigraph AMDB;
  virtual-keyboard text entry is not a fully ported KCCU input stack.
- This is the macOS xp-kit integration. The Windows binary is not replaced by
  this installer. Native fallback geometry remains in the aircraft.

## Build, installation and checks

```sh
ND_NATIVE=1 node tools/build_xp_kit_nd.cjs
python3 native/avionics/build.py --sdk build/sdk/SDK --xpkit --output-dir build/fresh-native-build
python3 tools/install_xp_kit_cockpit.py build/fresh-native-build
```

Quit X-Plane before installation. The installer checks signatures and every packaged
asset hash, backs up the prior Mac plugin, cockpit objects, provider manifest and
preferences, then updates only this aircraft. `package_xp_kit_cockpit.py --refresh-report`
supports asset-only staging refreshes and refuses changed native runtime sources.

The 16 October installation used `build/xp-kit-nd-cockpit-20261016-d`; its recoverable
backup is `build/backups/xp-kit-cockpit-20261016-100234`. Those historical paths do
not establish that a later candidate has been installed or verified in X-Plane.

Focused checks include the native PFD and ND bridge tests, C/Python shared-memory
ABI checks, native pointer/cursor tests, airport-parser and label-layout tests,
full-UV geometry/package tests and the existing cockpit/build validation suites.
The native helper probe captures real WebKit-rendered pixels from isolated inputs.
Its snapshots exclude the native cursor overlay, which is tested separately in
the production input/draw code. `tools/probe_nd_quality.py` records fixed-input
capture comparisons and transport measurements; its direct helper CPU samples
exclude WebKit's separate WebContent/GPU processes.

In-simulator verification is a separate step through `tools/check_xp_kit_nd.py`,
which gates on the exact stationary project aircraft and restores any selectors
it moves. Off-simulator images are not cockpit screenshots, and helper timings
do not establish in-simulator FPS, cursor smoothness or flight validation.

The original upstream ND TypeScript project is attempted with `FBW_TYPECHECK=1`;
the checkout still lacks the full MSFS SDK typing/dependency environment, so that
upstream check remains blocked. The focused bundle and native C/Objective-C builds
are separate checks. No full FlyByWire aircraft-system equivalence is claimed.

## Live verification — 16 October 2026

The installed aircraft was loaded in X-Plane 12.4.3 for a stationary, powered
ground check at UAAA. All four native views were ready and advancing with zero
host errors; both ND devices were bound to this aircraft's cockpit.

`build/xp-kit-nd-live-validation/native-validation.json` records seven passing
checks: cockpit bindings, physical EFIS ARC/40 NM selection, local UAAA OANS
geometry on both sides (191 features each), independent zoom, the original
airport panel, and captures from both live ND render channels.

`build/xp-kit-nd-pfd-regression/native-validation.json` records six passing PFD
regression checks: the exact project aircraft, actual 3D-panel texture uploads,
updates while paused, physical FCU input, independent DU blanking and automatic
renderer fallback/recovery. Both reports confirm that changed test selectors,
brightness and pause state were restored. This is a ground integration check,
not full flight validation. The yellow cursor and ordered map-pointer events
were additionally checked in the native WebKit host and focused input tests;
these are not a claim of an automated cockpit-mouse end-to-end test.

After these checks, the simulator log records a user-confirmed quit at 10:13.
The preserved `build/xp-kit-nd-live-validation/xplane-after-user-quit.log` contains
duplicate avionics-device cleanup warnings (also present in pre-ND backup logs
for this aircraft and another plugin) and an application-crash marker during
shutdown. No current macOS X-Plane crash report established its cause. A clean
shutdown is therefore **not** certified by the passing ground checks. No
speculative device-lifecycle change or simulator restart was made after the quit.
