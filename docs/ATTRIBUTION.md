# Attribution and redistribution notice

This experimental port uses work from FlyByWire Simulations' `aircraft` and `aircraft-large-files` repositories.

- FlyByWire source code: GNU General Public License v3.0.
- Original FlyByWire 3D assets: Creative Commons Attribution-NonCommercial 4.0 International.
- Other included assets may be subject to Microsoft's Game Content Usage Rules or their own notices.

Upstream project: https://github.com/flybywiresim/aircraft

Pinned FlyByWire revisions:

- `aircraft`: `2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2`
- `aircraft-large-files`: `5f07408bfec09c66dd6b5c5e484027f64664fb43`

The development `.acf` and airfoil starting structure comes from the open-source `mSparks43/747-400` project at revision `041c9ffa4b2b0c72a4aad36643759b7cace836c0`, licensed under CC BY-NC-SA 4.0. Its aircraft identity, original object attachments, and 3D panel were removed; it is used only as a technical X-Plane file-format and aerodynamic scaffold.

Reference project: https://github.com/mSparks43/747-400

No affiliation with or endorsement by FlyByWire Simulations, Laminar Research, Airbus, or Microsoft is implied. Airbus A380 is a trademark of Airbus. X-Plane is a trademark of Laminar Research.

## Imported A380 flight-guidance laws

`native/avionics/vendor/fbw` includes the unchanged FlyByWire A380
`A380FgOuterLoops` C++ implementation and its required support headers/functions
from `flybywiresim/aircraft` revision
`2baa2b35eadaf4c78e172ce41bbe6b40b4aeafb2`. Original relative paths and SHA256
hashes are in `vendor/fbw/manifest.json`; the upstream GPL-3.0 license is
retained in `vendor/fbw/LICENSE`. The C bridge, FCU mode adapter and X-Plane
actuator boundary are separate port code. The original files remain unchanged.

The LAND / FLARE / ROLL OUT / RETARD / SRS GA mode transitions in
`native/avionics/a380x_autoland.inc` are a hand port (GPL-3.0) of the landing
mode logic of `fbw-a380x/src/wasm/fbw_a380/src/model/A380PrimComputerFg.cpp`
and its parameters in `A380PrimComputerFg_data.cpp` at the same revision; each
threshold names its Simulink parameter. The FMA bit layouts follow
`fbw-a380x/src/systems/shared/src/publishers/PrimFgPublisher.ts` /
`FcdcPublisher.ts` and the PFD `FMA.tsx`. These files are not vendored.

The PFD/native FCU presentation also follows the pinned A380 instrument sources
and FlyByWire's public A380 AFS/EFIS documentation. No proprietary Airbus
manual, FlightFactor implementation or other aircraft vendor's assets were
imported for this update.

## TERRAIN on ND

`native/avionics/a380x_terrain.h` ports the terrain-on-ND colour bands, peaks mode, histogram statistics,
TERR threshold modes, A380 scan-line transition timing and dot patterns from FlyByWire SimBridge
(`flybywiresim/simbridge` revision `f5932323e2f21dd44a5fcfa3e3aab0cdb35b2f36`, GPL-3.0,
`apps/server/src/terrain`). The elevations come from X-Plane's own scenery through terrain probes; no
SimBridge terrain database, Honeywell or Airbus data is included. See `docs/ND_TERRAIN.md`.
