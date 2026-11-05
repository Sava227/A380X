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
| `stall` | FWS stall warning; looped ("STALL STALL" + cricket); silences the synthetic voice. |
| `crc_master_warning` | FWS CRC (continuous repetitive chime) while requested — it replaces the glareshield's old synthetic 640 Hz master-warning tone for those warnings. |
| `cchord` | FWS altitude alert: once (C-chord) or looped (continuous C-chord). |
| `triple_click` | FMA mode reversion (rising edge). |
| `seatbelt_chime` | FASTEN SEAT BELTS sign switched on or off (`sim/cockpit2/switches/fasten_seat_belts`, not within 5 s of power-up). |

All aurals are FWS outputs: nothing plays while the FWS is unpowered, and requests that were already
active at power-up do not fire as edges (a CRC request does sound once powered).

The synthetic voice is one channel. Priority: STALL > MINIMUM > HUNDRED ABOVE > RETARD > altitude
callouts > V1. STALL and MINIMUM interrupt a lower callout; the others wait for the current one and
are dropped if they cannot start within 1 s.

## Not covered by the supplied set

Single chime (master caution) and cavalry charge (A/P disconnect) keep the glareshield's existing
tones; there are no samples for them in the set.
