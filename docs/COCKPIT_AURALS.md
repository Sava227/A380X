# Cockpit aurals

The avionics plugin plays the A380 cockpit aurals from the user-supplied sample set
(`native/avionics/sounds/cockpit/`, installed as `plugins/A380XAvionics/sounds/cockpit/`;
origin and processing in `provenance.json`).

* Decision core: `native/avionics/a380x_aurals.c` (XPLM-free, scenario tests in `test_aurals.c`).
* X-Plane adapter: `native/avionics/a380x_aurals.inc` (loads the samples, reads the inputs, plays).

## Cockpit only

Every sample plays on X-Plane's **interior** bus and only while the camera is on the flight deck
(inside view, within 4 m of the pilot's eye point). Nothing plays in an outside view, in the cabin,
while paused or in replay. Loops stop when the view leaves the cockpit and resume when it comes back
while still requested; a one-shot callout that fell outside the cockpit is not replayed.

## What plays when

| Sample | Trigger |
|---|---|
| `callout_2500` … `callout_5` (2500, 2000, 1000, 500, 400, 300, 200, 100, 90 … 10, 5) | Captain radio altitude crossing the height while descending in flight. Only the lowest of several heights crossed in one step is called. A height re-arms after climbing above it (100 ft for 1000 ft and up, 20 ft for 100–500, 3 ft + 10 % below 100), so a go-around gives a fresh set and noise or a bounce does not repeat callouts. |
| `hundred_above`, `minimum` | FMS PERF APPR minimum: RADIO (DH, radio altitude) or else BARO (MDA, baro altitude), 100 ft above and at the minimum, descending. They replace the altitude callout of that moment. |
| `retard` | 20 ft (manual landing) or 10 ft (A/P engaged) while a thrust lever is above IDLE (TLA > 2°); repeated every ~1.4 s until IDLE (at most six). After RETARD the numeric callouts below that height are not called. |
| `v1` | FWS V1 callout pulse (takeoff, V1 reached). |
| `stall` | Looped ("STALL STALL" + cricket); silences the synthetic voice. Sounds for the FWS stall warning and for the port stall warning: airborne above 60 kt and below VSW (the red/black barber pole on the PFD; clears 3 kt above it), or at the flight model's stall-warning angle of attack (15° in the .acf). The FWS warning alone never sounds here because it requires alternate/direct law and this port always reports normal law without implementing alpha protection. |
| `pull_up_new` / `pull_up_old` | GPWS terrain warning (TERRAIN / PULL UP), looped while active, 10–2450 ft RA airborne. Mode 1 (excessive descent rate): RA ≤ 1.3 × sink − 1940 ft below 1700 fpm sink, else ≤ 0.4 × sink − 410 ft (sink > 1000 fpm). Mode 2 (excessive terrain closure rate): the RA rate (0.5 s low-pass) inside the FBW mode-2 envelope (airspeed-dependent upper limit 1650–2450 ft; 775 ft with landing flaps) for more than 0.2 s. Port of FBW A380 `LegacyGpws.ts`; FBW keeps mode 2 off because of an MSFS terrain artefact, X-Plane's radio altitude is the real terrain so it is on here. Silences the callouts; the stall warning has priority. The recording is chosen on the EFB (Settings > Sounds > GPWS / TAWS voice: New A380 / Old A380, saved in X-Plane's preferences folder as `A380X-XP12_efb.prf`) or with the command `a380x/xp12/aurals/gpws_voice_toggle`; switching while the warning sounds restarts it with the chosen recording. User-supplied recordings (`~/Downloads/airbus-a380-pull-up.mp3`, `old-airbus-a380-pull-up.mp3`). GPWS SYS OFF (SURV panel) inhibits modes 1 and 2. |
| `pull_up_new` / `pull_up_old` (TAWS) | TAWS look-ahead **warning** (terrain inside the warning envelope ahead, about 30 s; see `docs/ND_TERRAIN.md`, "Terrain alerts"): the same looped PULL UP recording, OR'ed with the GPWS modes, as long as the warning lasts. The stall warning keeps priority. Not inhibited by GPWS SYS OFF (TERR SYS OFF or WXR/TAWS SYS OFF inhibit the look-ahead). |
| `terrain_ahead` (optional) | TAWS look-ahead **caution** (about 60 s ahead): "TERRAIN AHEAD" once at its start, then every 7 s while the caution lasts (EGPWS behaviour); not during the warning or the stall. **No recording is supplied**: the caution is silent unless you add one (below). |
| `crc_master_warning` | FWS CRC (continuous repetitive chime) while requested — it replaces the glareshield's old synthetic 640 Hz master-warning tone for those warnings. |
| `cchord` | FWS altitude alert: once (C-chord) or looped (continuous C-chord). |
| `triple_click` | FMA mode reversion, or a landing capability downgrade (`capability_downgrade` from the guidance, docs/AUTOLAND.md); rising edges, one click if both rise together. |
| `cavalry_charge` | A/P disconnect (FWS cavalry charge): one 1.2 s cycle for a voluntary disconnect with the sidestick button, repeated until acknowledged for an involuntary one; it replaces the glareshield tone for the A/P OFF master warning. Original synthesized sample (not from the supplied set). |
| `seatbelt_chime` | FASTEN SEAT BELTS sign switched on or off (`sim/cockpit2/switches/fasten_seat_belts`, not within 5 s of power-up). |

All aurals are FWS outputs: nothing plays while the FWS is unpowered, and requests that were already
active at power-up do not fire as edges (a CRC request does sound once powered).

The synthetic voice is one channel. Priority: STALL > PULL UP (GPWS / TAWS warning) > TERRAIN AHEAD >
MINIMUM > HUNDRED ABOVE > RETARD > altitude callouts > V1. STALL and PULL UP are loops that silence the
voice; TERRAIN AHEAD and MINIMUM interrupt a lower callout; the others wait for the current one and are
dropped if they cannot start within 1 s.

## Adding the TERRAIN AHEAD recording

The TAWS caution has a sample slot but no recording (none is synthesized or downloaded). To hear
"TERRAIN AHEAD", put your own recording at

    plugins/A380XAvionics/sounds/cockpit/terrain_ahead.wav

in the installed aircraft: **mono, 16-bit PCM WAV**, 8–48 kHz, at most about 12 s (2 MB), ideally
about 1.5 s with no leading silence. It is read when the plugin starts (reload the aircraft or the
plugins after adding it). X-Plane's `Log.txt` line "A380X cockpit aurals: N/30 samples loaded" says
"optional terrain_ahead.wav not installed (TAWS caution silent)" while it is missing. Without it the
caution still shows on the PFD (TERRAIN, amber) and the NDs (solid yellow terrain, TERR pop-up).

## Not covered by the supplied set

The single chime (master caution) keeps the glareshield's existing tone; there is no sample for it.

## Packaged for later

`plugins/A380XAvionics/sounds/systems/ptu.wav` (user-supplied `airbus-ptu-sound.mp3`, mono 16-bit,
7.7 s) is installed for the future PTU simulation; nothing plays it yet.
