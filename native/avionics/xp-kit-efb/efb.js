/* SPDX-License-Identifier: GPL-3.0-or-later
 * A380 EFB tablet page (docs/EFB_DEVELOPMENT.md). Presentation and touch only: the native EFB
 * (native/avionics/a380x_efb.c) sends one JSON frame per tablet through the aircraft-local WebKit
 * helper and validates every action this page posts back. Values the aircraft does not model arrive
 * as null and are shown as unavailable; nothing here invents data. Ported from the approved design
 * docs/efb-design/EFB_MOCKUP.dc.html (no DC runtime, no network, no external resources).
 *
 * Bridge: window.__a380Efb.apply(frame, inputValid) / .idle() both return true when the page
 * changed and needs a new texture capture; window.__a380EfbCore exposes the pure parts for tests.
 */
(function (global) {
  'use strict';
  var SCALE = 1440 / 1160;
  var LB = 2.20462262;
  var ACC = '#4FC3E6';
  var SECTIONS = [
    ['home', 'Home', 'Dashboard'], ['ofp', 'OFP', 'SimBrief OFP'], ['ground', 'Ground', 'Ground'],
    ['loading', 'Loading', 'Loading'], ['cabin', 'Cabin', 'Cabin layout'], ['flight', 'Flight', 'Flight'],
    ['ops', 'Ops', 'Operations'], ['failures', 'Failures', 'Failures'], ['aircraft', 'Aircraft', 'Aircraft options'],
    ['settings', 'Settings', 'Settings']
  ];
  var TABS = {
    ground: [['doors', 'Doors & slides'], ['services', 'Services'], ['push', 'Pushback']],
    loading: [['payload', 'Payload'], ['fuel', 'Fuel'], ['wb', 'Weight & balance']],
    flight: [['perf', 'Performance'], ['nav', 'Charts & nav'], ['comms', 'Comms']],
    ops: [['checklists', 'Checklists'], ['presets', 'Presets']],
    settings: [['general', 'General'], ['sounds', 'Sounds']]
  };
  var ICONS = {
    home: '<path d="M3 11l9-7 9 7"/><path d="M5 10v10h14V10"/><path d="M10 20v-5h4v5"/>',
    ofp: '<path d="M6 3h9l3 3v15H6z"/><path d="M9 10h6M9 14h6M9 18h4"/>',
    ground: '<rect x="6" y="3" width="12" height="18" rx="1.5"/><circle cx="14.5" cy="12" r="1"/>',
    loading: '<path d="M6 9h12l2 11H4z"/><circle cx="12" cy="5.5" r="2.5"/>',
    cabin: '<path d="M8 3v10h8"/><path d="M8 13l-2 8M16 13l2 8M6 17h12"/>',
    flight: '<path d="M21 16v-2l-8-5V3.5a1.5 1.5 0 0 0-3 0V9l-8 5v2l8-2.5V19l-2 1.5V22l3.5-1 3.5 1v-1.5L13 19v-5.5z"/>',
    ops: '<rect x="5" y="3" width="14" height="18" rx="2"/><path d="M9 8h6M9 12h6M9 16h4"/>',
    failures: '<path d="M12 3l10 18H2z"/><path d="M12 10v5M12 18v.5"/>',
    aircraft: '<circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="2.5"/><path d="M12 4v5.5M12 14.5V20M4 12h5.5M14.5 12H20"/>',
    settings: '<path d="M4 6h10M18 6h2M4 12h4M12 12h8M4 18h12"/><circle cx="16" cy="6" r="2"/><circle cx="10" cy="12" r="2"/><circle cx="18" cy="18" r="2"/>'
  };
  var DOOR_ROWS = {
    main: [['M1L', 0], ['M2L', 2], ['M3L', 4], ['M4L', 6], ['M5L', 8], ['M1R', 1], ['M2R', 3], ['M3R', 5], ['M4R', 7], ['M5R', 9]],
    upper: [['U1L', 10], ['U2L', 12], ['U3L', 14], ['U1R', 11], ['U2R', 13], ['U3R', 15]],
    other: [['FWD', 17], ['AFT', 18], ['BULK', 19], ['CKPT', 16]]
  };
  var CLASSES = [['First', '#E8B54A'], ['Business', '#4FC3E6'], ['Premium economy', '#A78BFA'], ['Economy', '#46C38B']];
  var HOLDS = ['Forward hold', 'Aft hold', 'Bulk hold'];
  var TANK_LEFT = [0, 2, 3, 1, 4], TANK_RIGHT = [5, 8, 6, 7, 9];
  var PRESETS = [
    [1, 'Cold & dark', 'Everything off, doors as they are, slides disarmed.'],
    [2, 'Turnaround', 'External power and services; slides disarmed.'],
    [3, 'Ready for pushback', 'APU running, beacon on; slides of closed doors armed.'],
    [4, 'Ready for taxi', 'Engines running; slides of closed doors armed.'],
    [5, 'Ready for take-off', 'Take-off configuration; slides of closed doors armed.']
  ];
  var SOUNDS = {
    master: ['Master', 'All simulator sounds'], engines: ['Engines', 'Engine sound effects'],
    exterior: ['Exterior', 'External effects heard outside'], interior: ['Interior', 'Cockpit and cabin effects'],
    ui: ['Clicks', 'Switch, button and UI clicks'], environment: ['Environment', 'Wind, rain and thunder'],
    weather: ['Weather', 'Weather sound effects'], ground: ['Ground', 'Ground contact and rumble'],
    warnings: ['Warnings', 'Warning systems and aurals'], radios: ['Radios', 'COM radio audio']
  };
  var PHASES = ['', 'Preflight', 'Taxi out', 'Take-off power', 'Take-off roll', 'Rotation', 'Lift-off',
    'Initial climb', 'In flight', 'Approach', 'Landing roll', 'Taxi in', 'Parked'];
  var FILTERS = ['All', 'ENG', 'ELEC', 'F/CTL', 'L/G', 'HYD', 'APU', 'NAV'];
  var CABIN_UNITS = [4.0, 2.4, 1.3, 1.0], CABIN_CAP = [420, 300, 160];

  /* ---- State ------------------------------------------------------------------------------------- */
  var F = null, valid = true;
  var L = {
    doorMode: 'doors', filter: 'All', clock: 'UTC', keypad: null, pressed: null, drag: null,
    toast: null, toastUntil: 0, lastStatus: null, optimistic: null, sliders: {}, epoch: null, lastSeq: 0,
    nextId: 1, perf: 'tod'
  };
  var dirty = true, lastHtml = '', sent = [];

  /* ---- Formatting -------------------------------------------------------------------------------- */
  function isNum(v) { return typeof v === 'number' && isFinite(v); }
  function esc(s) {
    return String(s == null ? '' : s).replace(/[&<>"']/g, function (c) {
      return {'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[c];
    });
  }
  function pad(n) { return (n < 10 ? '0' : '') + n; }
  function metric() { return !F || !F.settings || F.settings.metric !== false; }
  function unit() { return metric() ? 'kg' : 'lb'; }
  function mass(kg) {
    if (!isNum(kg)) return '<span class="na">—</span>';
    return metric() ? (kg / 1000).toFixed(1) + ' t' : (kg * LB / 1000).toFixed(1) + ' klb';
  }
  function toUnit(kg) { return isNum(kg) ? Math.round(metric() ? kg : kg * LB) : null; }
  function group3(n) { return String(n).replace(/\B(?=(\d{3})+(?!\d))/g, ','); }
  function qty(kg) { var v = toUnit(kg); return v == null ? '<span class="na">—</span>' : group3(v) + ' ' + unit(); }
  function na(v, text) { return v == null || (typeof v === 'number' && !isFinite(v)) ? '<span class="na">' + (text || '—') + '</span>' : v; }
  function freq(khz) { return isNum(khz) ? (khz / 1000).toFixed(3) : null; }
  function clockText() {
    var m = F && F.clock ? (L.clock === 'UTC' ? F.clock.zulu : F.clock.local) : null;
    if (!isNum(m)) return '--:--';
    m = ((Math.floor(m) % 1440) + 1440) % 1440;
    return pad(Math.floor(m / 60)) + ':' + pad(m % 60) + (L.clock === 'UTC' ? 'Z' : ' LT');
  }
  function pct(v, limit) { return Math.max(0, Math.min(100, v / limit * 100)).toFixed(1); }
  function bar(v, limit) {
    if (!isNum(v)) return '<div class="track"></div>';
    return '<div class="track"><div class="' + (v > limit ? 'over' : '') + '" style="width:' + pct(v, limit) + '%"></div></div>';
  }

  /* ---- Elements ---------------------------------------------------------------------------------- */
  function pressed(key) { return L.pressed === key ? ' pressed' : ''; }
  function act(key) { return key ? ' data-a="' + esc(key) + '"' : ''; }
  function btn(key, label, cls, disabled) {
    return '<button class="btn ' + (cls || '') + (disabled ? ' disabled' : pressed(key)) + '"' + (disabled ? '' : act(key)) + '>' + label + '</button>';
  }
  function seg(options, current, prefix, disabled) {
    return '<div class="seg">' + options.map(function (o) {
      var key = prefix + ' ' + o[0];
      return '<button class="' + (o[0] === current ? 'on' : '') + (disabled ? '' : pressed(key)) + '"' + (disabled ? ' disabled' : act(key)) + '>' + esc(o[1]) + '</button>';
    }).join('') + '</div>';
  }
  function sw(on, key, disabled) {
    return '<button class="sw' + (on ? ' on' : '') + (disabled ? '' : pressed(key)) + '"' + (disabled ? ' disabled' : act(key)) + '></button>';
  }
  function field(key, text, opts) {
    opts = opts || {};
    return '<div class="field' + (opts.warn ? ' warn' : '') + (opts.disabled ? ' disabled' : pressed(key)) + '"' + (opts.disabled ? '' : act(key)) +
      (opts.style ? ' style="' + opts.style + '"' : '') + '>' + text + '</div>';
  }
  function kv(label, value) { return '<div class="kv"><span>' + label + '</span><span>' + value + '</span></div>'; }
  function card(cls, style, inner) { return '<section class="card ' + (cls || '') + '"' + (style ? ' style="' + style + '"' : '') + '>' + inner + '</section>'; }
  function lbl(text, style) { return '<div class="lbl"' + (style ? ' style="' + style + '"' : '') + '>' + text + '</div>'; }
  function slider(key, value, min, max) {
    var local = L.sliders[key];
    if (local && (L.drag && L.drag.key === key || Date.now() < local.until)) value = local.value;
    var ok = isNum(value), p = ok ? Math.max(0, Math.min(1, (value - min) / (max - min))) * 100 : 0;
    return '<div class="slider' + (ok ? '' : ' na') + '" data-s="' + esc(key) + '" data-min="' + min + '" data-max="' + max + '">' +
      '<div class="rail2"></div><div class="fill" style="width:' + p.toFixed(1) + '%"></div><div class="knob" style="left:' + p.toFixed(1) + '%"></div></div>';
  }
  function sliderValue(key, value) {
    var local = L.sliders[key];
    return local && (L.drag && L.drag.key === key || Date.now() < local.until) ? local.value : value;
  }

  /* ---- Derived frame values --------------------------------------------------------------------- */
  function ui() {
    var section = F.ui.section, tabs = {}, k;
    for (k in F.ui.tabs) tabs[k] = F.ui.tabs[k];
    if (L.optimistic) {
      section = L.optimistic.section || section;
      if (L.optimistic.tab) tabs[section] = L.optimistic.tab;
    }
    return {section: section, tab: tabs[section], tabs: tabs};
  }
  function doorOpen(i) { var r = F.doors.ratio[i]; return isNum(r) && r > 0.01; }
  function doorState(i) {
    var r = F.doors.ratio[i];
    if (!isNum(r)) return '—';
    return r > 0.99 ? 'OPEN' : r > 0.01 ? 'MOVING' : 'CLOSED';
  }
  function nextGroup() {
    for (var g = 0; g < F.checklists.length; ++g) if (doneCount(g) < F.checklists[g].items.length) return g;
    return F.checklists.length - 1;
  }
  function doneCount(g) {
    var c = F.checklists[g], n = 0;
    for (var i = 0; i < c.items.length; ++i) if (c.done & (1 << i)) ++n;
    return n;
  }
  function loadingLock() {
    return F.ac.loadingAllowed ? '' : '<span class="chip warn">LOCKED · ON GROUND, ENGINES OFF</span>';
  }
  function zfwPlan() { return isNum(F.weights.oew) ? F.weights.oew + F.load.planKg : null; }

  /* ---- Sections ---------------------------------------------------------------------------------- */
  function viewHome() {
    var w = F.weights, g = nextGroup(), c = F.checklists[g], done = doneCount(g);
    var phase = isNum(F.ac.phase) ? (PHASES[F.ac.phase] || 'Phase ' + F.ac.phase) : null;
    var presets = PRESETS.map(function (p) {
      var available = F.presets.native || p[0] === 1 || p[0] === 2 || p[0] === 4;
      return btn('preset ' + p[0], esc(p[1]), F.presets.active === p[0] ? 'primary' : '', !available);
    }).join('');
    return '<div class="grid" style="grid-template-columns:repeat(3,minmax(0,1fr))">' +
      card('col', 'grid-column:span 2;gap:18px', lbl('Flight') +
        '<div style="display:flex;align-items:center;gap:28px"><div><div class="mono na" style="font-size:40px;font-weight:600;line-height:1">----</div>' +
        '<div class="muted" style="font-size:15px;margin-top:6px">Origin</div></div>' +
        '<svg width="64" height="24" viewBox="0 0 64 24" fill="none" stroke="' + ACC + '" stroke-width="2"><path d="M2 12h58M52 5l8 7-8 7"/></svg>' +
        '<div><div class="mono na" style="font-size:40px;font-weight:600;line-height:1">----</div><div class="muted" style="font-size:15px;margin-top:6px">Destination</div></div>' +
        '<span class="chip plan" style="margin-left:auto">OFP IMPORT PLANNED</span></div>' +
        '<div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px">' +
        '<div>' + lbl('Phase') + '<div class="val" style="font-size:19px">' + na(phase) + '</div></div>' +
        '<div>' + lbl('Altitude') + '<div class="val">' + (isNum(F.ac.altFt) ? group3(Math.round(F.ac.altFt)) + ' ft' : na(null)) + '</div></div>' +
        '<div>' + lbl('Seats') + '<div class="val">' + F.load.seats.reduce(function (a, b) { return a + b; }, 0) + '</div></div>' +
        '<div>' + lbl('Engines') + '<div class="val" style="color:' + ACC + ';font-size:17px;white-space:nowrap">' + esc(F.ac.engines) + '</div></div></div>') +
      card('', 'display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px 12px;align-content:start',
        lbl('Aircraft', 'grid-column:span 2') +
        '<div>' + lbl('Gross wt') + '<div class="val">' + mass(w.gw) + '</div></div>' +
        '<div>' + lbl('Fuel on board') + '<div class="val">' + mass(w.fob) + '</div></div>' +
        '<div>' + lbl('Zero-fuel wt') + '<div class="val">' + mass(w.zfw) + '</div></div>' +
        '<div>' + lbl('CG') + '<div class="val na">—</div></div>') +
      card('col', 'gap:12px', lbl('Doors &amp; slides') +
        '<div style="display:flex;align-items:baseline;gap:10px"><span class="mono" style="font-size:34px;font-weight:600;color:#F2A93B">' + F.doors.open + '</span><span class="muted">of 20 doors open</span></div>' +
        '<div style="display:flex;align-items:baseline;gap:10px"><span class="mono" style="font-size:34px;font-weight:600;color:' + ACC + '">' + F.doors.armed + '</span><span class="muted">of 16 slides armed</span></div>' +
        '<div>' + btn('nav ground doors', 'Open Ground') + '</div>') +
      card('col', 'gap:12px', lbl('Next checklist') +
        '<div style="font:700 20px/1.2 var(--sans)">' + esc(c.name) + '</div>' +
        '<div style="display:flex;align-items:center;gap:12px"><div style="flex:1">' + bar(done, c.items.length) + '</div><span class="mono muted">' + done + '/' + c.items.length + '</span></div>' +
        '<div>' + btn('openchecklist ' + g, 'Open checklist') + '</div>') +
      card('col', 'gap:12px', lbl('Weight limits') + limits()) +
      card('col', 'grid-column:span 3;gap:12px',
        '<div style="display:flex;justify-content:space-between;align-items:center">' + lbl('Quick presets') +
        '<span class="muted" style="font-size:13px">Ground only' + (F.presets.native ? '' : ' · 3 of 5 without the A380 electrical network') + '</span></div>' +
        '<div style="display:flex;gap:10px;flex-wrap:wrap">' + presets + '</div>' + presetProgress()) +
      '</div>';
  }
  function limits() {
    var w = F.weights;
    function row(label, value, limit, note) {
      return '<div class="col" style="gap:6px"><div style="display:flex;justify-content:space-between;font-size:14px"><span style="font-weight:600">' + label +
        '</span><span class="mono muted">' + (note || mass(value) + ' / ' + mass(limit)) + '</span></div>' + bar(value, limit) + '</div>';
    }
    return row('ZFW', w.zfw, w.mzfw) + row('GW', w.gw, w.mtow) + row('LW', null, w.mlw, 'needs OFP trip fuel');
  }
  function presetProgress() {
    var p = F.presets;
    if (!p.native || (!p.active && !p.done)) return '';
    var name = (PRESETS[(p.active || p.done) - 1] || [0, ''])[1];
    return '<div class="oktext">' + (p.active ? esc(name) + ' running · step ' + p.step + ' / ' + p.count + (p.stepName ? ' · ' + esc(p.stepName) : '') :
      esc(name) + ' complete') + '</div>';
  }

  function viewOfp() {
    var dash = '<span class="mono na">—</span>';
    function rows(list) { return list.map(function (r) { return kv(r, dash); }).join(''); }
    return '<div class="grid" style="grid-template-columns:repeat(3,minmax(0,1fr))">' +
      card('', 'grid-column:span 3;display:flex;align-items:flex-end;gap:12px',
        '<label style="flex:1;display:flex;flex-direction:column;gap:6px"><span class="lbl">SimBrief pilot ID</span>' +
        field(null, '<span class="na">e.g. 123456</span>', {disabled: true}) + '</label>' + btn(null, 'Fetch latest OFP', '', true) +
        '<span class="chip plan" style="height:44px">PLANNED · NOT ACTIVE YET</span>') +
      card('col', 'gap:10px', lbl('Summary') + rows(['Route', 'Alternate', 'Cruise / CI', 'Air distance', 'Trip time'])) +
      card('col', 'gap:10px', lbl('Fuel plan') + rows(['Taxi', 'Trip', 'Contingency', 'Alternate', 'Final reserve', 'Extra', 'Block'])) +
      card('col', 'gap:10px', lbl('Planned weights') + rows(['ZFW', 'TOW', 'LW', 'Passengers']) +
        '<div style="display:flex;gap:8px;margin-top:6px">' + btn(null, 'Load into FMS', '', true) + btn(null, 'Load payload &amp; fuel', '', true) + '</div>') +
      card('col', 'grid-column:span 3;gap:8px', lbl('Route string') +
        '<div class="mono muted" style="font-size:14px;line-height:1.6">No OFP loaded. SimBrief import is planned for a later update and is not active: nothing on this page is fetched or invented.</div>') +
      '</div>';
  }

  function doorTile(entry) {
    var name = entry[0], i = entry[1], open = doorOpen(i), slide = i < 16, armed = slide && F.doors.slides[i];
    var key = L.doorMode === 'doors' ? 'door ' + name : slide ? 'slide ' + name + ' ' + (armed ? 0 : 1) : 'slide ' + name + ' 1';
    return '<button class="tile' + (open ? ' open' : armed ? ' armed' : '') + pressed(key) + '"' + act(key) + '><span>' + (name === 'CKPT' ? 'Cockpit' : name) +
      '</span><span class="st">' + doorState(i) + '</span><span class="sl' + (armed ? ' armed' : '') + '">' + (slide ? (armed ? 'ARMED' : 'DISARMED') : 'NO SLIDE') + '</span></button>';
  }
  function viewDoors() {
    var d = F.doors;
    return '<div class="grid" style="grid-template-columns:minmax(0,1.65fr) minmax(0,1fr)">' +
      card('col', 'gap:12px', '<div style="display:flex;justify-content:space-between;align-items:center;gap:12px"><div style="width:260px">' +
        seg([['doors', 'Doors'], ['slides', 'Slides']], L.doorMode, '~doormode') + '</div><span class="muted" style="font-size:13px">' +
        (L.doorMode === 'doors' ? 'Tap a door to open or close it' : 'Tap a door to arm or disarm its slide') + '</span></div>' +
        '<div class="deck">MAIN DECK</div><div style="display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:8px">' + DOOR_ROWS.main.map(doorTile).join('') + '</div>' +
        '<div class="deck">UPPER DECK</div><div style="display:grid;grid-template-columns:repeat(6,minmax(0,1fr));gap:8px">' + DOOR_ROWS.upper.map(doorTile).join('') + '</div>' +
        '<div class="deck">CARGO &amp; COCKPIT</div><div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px">' + DOOR_ROWS.other.map(doorTile).join('') + '</div>') +
      card('col', 'gap:12px', lbl('Doors') + kv('Open', '<span style="color:#F2A93B">' + d.open + ' / 20</span>') +
        '<div style="display:flex;gap:8px">' + btn('doors open', 'Open all') + btn('doors close', 'Close all') + '</div><div class="hr"></div>' +
        lbl('Escape slides') + kv('Armed', '<span style="color:' + ACC + '">' + d.armed + ' / 16</span>') +
        '<div style="display:flex;gap:8px">' + btn('slides arm', 'Arm all slides', 'primary') + btn('slides disarm', 'Disarm all') + '</div>' +
        '<div class="note">A door must be closed to arm its slide, and disarmed before it can be opened. Cargo and cockpit doors have no slides. Doors are ground-only.</div>' +
        (F.ac.onGround ? '' : '<span class="chip warn">AIRBORNE · DOORS LOCKED</span>')) +
      '</div>';
  }
  function viewServices() {
    var g = F.ground, connected = (g.gpu ? 1 : 0) + (g.chocks ? 1 : 0);
    function row(label, control, note) {
      return '<div class="list-row"><div class="col" style="gap:4px"><span style="font-size:15px;font-weight:500">' + label + '</span>' +
        (note ? '<span class="note">' + note + '</span>' : '') + '</div>' + control + '</div>';
    }
    var unavailable = '<span class="chip">NOT MODELLED</span>';
    return card('', 'display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:4px 32px;align-content:start',
      lbl('Ground services · ' + connected + ' connected', 'grid-column:span 2;margin-bottom:8px') +
      row('Ground power (EXT)', sw(g.gpu, 'service gpu', !F.ac.onGround), F.ac.elecNetwork ? 'A380 electrical network EXT' : 'X-Plane ground power unit') +
      row('Wheel chocks', g.chocks == null ? unavailable : sw(g.chocks, 'service chocks', !F.ac.onGround), 'X-Plane tire chocks') +
      row('Jet bridge', btn('service jetway', 'Request', '', !F.ac.onGround), 'X-Plane jetway service; connection state is not reported') +
      row('Ground service trucks', btn('service trucks', 'Request', '', !F.ac.onGround), 'Catering, fuel and baggage trucks of X-Plane') +
      row('Pre-conditioned air', unavailable, 'No low-pressure air model yet') +
      row('Air start unit', unavailable, 'No high-pressure air model yet'));
  }
  function viewPush() {
    var g = F.ground, a = F.ac;
    function row(label, ok, text) {
      return '<div style="display:flex;justify-content:space-between;min-height:40px;align-items:center;border-bottom:1px solid var(--line)"><span>' + label +
        '</span><span class="mono" style="color:' + (ok == null ? '#6B7A8A' : ok ? '#7FD8A9' : '#F2A93B') + '">' + text + '</span></div>';
    }
    return '<div class="grid" style="grid-template-columns:minmax(0,1fr) minmax(0,1fr)">' +
      card('col', 'gap:18px', lbl('Pushback') +
        '<div style="display:flex;align-items:center;gap:12px"><span style="width:10px;height:10px;border-radius:5px;background:' + (g.tug ? ACC : '#3A4656') + '"></span>' +
        '<span style="font:600 18px/1.2 var(--sans)">' + (g.tug == null ? 'Tug state unavailable' : g.tug ? 'Tug attached' : 'No tug attached') + '</span></div>' +
        '<div class="col" style="gap:8px"><span class="lbl">Steering</span>' + seg([['0', 'Nose left'], ['1', 'Straight'], ['2', 'Nose right']], String(g.steer), 'push steer') + '</div>' +
        '<div class="col" style="gap:8px"><span class="lbl">Tug speed</span><span class="note">Set by X-Plane&#39;s pushback; not adjustable here.</span></div>' +
        '<div style="display:flex;gap:10px">' + btn('push start', 'Start pushback', 'primary', !a.onGround) + btn('push stop', 'Stop') + '</div>') +
      card('col', 'gap:12px', lbl('Before pushback') +
        row('Doors closed', F.doors.open === 0, F.doors.open === 0 ? 'YES' : F.doors.open + ' OPEN') +
        row('Slides armed', F.doors.armed === 16, F.doors.armed === 16 ? 'ALL 16' : F.doors.armed + ' / 16') +
        row('Beacon', a.beacon, a.beacon ? 'ON' : 'OFF') +
        row('Parking brake', isNum(a.parkBrake) ? a.parkBrake > 0.5 : null, isNum(a.parkBrake) ? (a.parkBrake > 0.5 ? 'SET' : 'RELEASED') : '—') +
        row('Jet bridge', null, 'not reported')) +
      '</div>';
  }

  function viewPayload() {
    var ld = F.load, pax = 0, warn = '', zfw = zfwPlan();
    var cols = 'grid-template-columns:minmax(0,1.5fr) 80px 150px minmax(0,1fr)';
    var rows = CLASSES.map(function (c, i) {
      pax += ld.pax[i];
      return '<div class="trow" style="' + cols + ';min-height:54px"><span style="font-weight:600">' + c[0] + '</span><span class="mono muted">' + ld.seats[i] +
        '</span>' + field('~kp pax ' + i, String(ld.pax[i]), {disabled: !F.ac.loadingAllowed}) + '<span class="mono" style="text-align:right">' +
        mass(ld.pax[i] * ld.paxKg) + '</span></div>';
    }).join('') + HOLDS.map(function (h, i) {
      return '<div class="trow" style="' + cols + ';min-height:54px"><span style="font-weight:600">' + h + '</span><span class="mono muted">' + unit() +
        '</span>' + field('~kp cargo ' + i, String(toUnit(ld.cargo[i])), {disabled: !F.ac.loadingAllowed}) + '<span class="mono" style="text-align:right">' +
        mass(ld.cargo[i]) + '</span></div>';
    }).join('');
    if (isNum(zfw) && zfw > F.weights.mzfw) warn = 'Planned ZFW ' + mass(zfw) + ' exceeds MZFW ' + mass(F.weights.mzfw) + '.';
    else if (ld.planKg > F.weights.maxPayload) warn = 'Planned payload exceeds the ' + mass(F.weights.maxPayload) + ' maximum.';
    var live = ld.liveMatches ? '<span class="oktext" style="font-size:13px">Live payload matches this plan.</span>' :
      '<span class="note">Live payload ' + mass(F.weights.payload) + ' was not set from this plan; its per-class split is unknown.</span>';
    return '<div class="grid" style="grid-template-columns:minmax(0,1.5fr) minmax(0,1fr)">' +
      card('col', 'gap:4px', '<div class="trow thead" style="' + cols + '"><span class="lbl">Cabin / hold</span><span class="lbl">Seats</span><span class="lbl">Load</span><span class="lbl" style="text-align:right">Mass</span></div>' +
        rows + '<div class="warntext" style="margin-top:6px">' + warn + '</div><div class="note">Tap a value to type it. ' + ld.paxKg + ' kg per passenger incl. baggage.</div>') +
      card('col', 'gap:14px', '<div style="display:flex;justify-content:space-between;align-items:center">' + lbl('Totals') + loadingLock() + '</div>' +
        kv('Passengers', pax + ' / ' + ld.seats.reduce(function (a, b) { return a + b; }, 0)) + kv('Planned payload', mass(ld.planKg)) +
        kv('Live payload', mass(F.weights.payload)) + kv('Operating empty wt', mass(F.weights.oew)) + '<div class="hr"></div>' +
        '<div class="col" style="gap:6px"><div style="display:flex;justify-content:space-between;font-size:14px"><span style="font-weight:600">Planned ZFW</span><span class="mono muted">' +
        mass(zfw) + ' / ' + mass(F.weights.mzfw) + '</span></div>' + bar(zfw, F.weights.mzfw) + '</div>' +
        '<div style="display:flex;gap:10px;margin-top:6px">' + btn('load apply', 'Load payload', 'primary', !F.ac.loadingAllowed) +
        btn('load full', 'All seats', '', !F.ac.loadingAllowed) + btn('load clear', 'Unload all', '', !F.ac.loadingAllowed) + '</div>' + live) +
      '</div>';
  }
  function tankCell(i) {
    var t = F.fuel.tanks[i], q = t[1], c = t[2];
    return '<div class="tank"><span class="lbl">' + esc(t[0]) + '</span><span class="mono" style="font-size:15px">' + mass(q) + '</span>' +
      (isNum(q) && isNum(c) && c > 0 ? bar(q, c) : '<div class="track"></div>') + '</div>';
  }
  function viewFuel() {
    var fu = F.fuel, target = isNum(fu.target) ? group3(toUnit(fu.target)) : '<span class="na">enter</span>';
    return '<div class="grid" style="grid-template-columns:minmax(0,1.6fr) minmax(0,1fr)">' +
      card('col', 'gap:14px', lbl('Tanks · ' + mass(F.weights.fob)) +
        '<div class="deck">LEFT WING</div><div style="display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:8px">' + TANK_LEFT.map(tankCell).join('') + '</div>' +
        '<div class="deck">RIGHT WING</div><div style="display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:8px">' + TANK_RIGHT.map(tankCell).join('') + '</div>' +
        '<div style="display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:8px"><div style="grid-column:3">' + tankCell(10) + '</div></div>' +
        '<div class="note">FQMS view of the nine X-Plane tanks (the same split as the SD FUEL page).</div>') +
      card('col', 'gap:16px', '<div style="display:flex;justify-content:space-between;align-items:center">' + lbl('Refuel') + loadingLock() + '</div>' +
        '<label class="col" style="gap:6px"><span class="lbl">Block fuel (' + unit() + ')</span>' +
        field('~kp fuel', target, {disabled: !F.ac.loadingAllowed, style: 'font-size:24px;height:52px;line-height:50px'}) + '</label>' +
        '<div class="col" style="gap:8px"><span class="lbl">Rate</span>' + seg([['real', 'Real'], ['fast', 'Fast'], ['instant', 'Instant']], fu.rate, 'fuel rate') + '</div>' +
        '<div style="display:flex;gap:10px">' + btn('fuel start', 'Start refuel', 'primary', !F.ac.loadingAllowed || !isNum(fu.target)) + btn('fuel stop', 'Stop') + '</div>' +
        '<div class="note">' + (fu.refuelling ? 'Refuelling to ' + mass(fu.target) + ' (' + fu.rate + ' rate)' : 'Refuel idle') +
        '. Real ≈ 3 t/min and Fast ≈ 15 t/min are approximations; Instant sets the tanks at once.</div>') +
      '</div>';
  }
  function viewWb() {
    return '<div class="grid" style="grid-template-columns:minmax(0,1.5fr) minmax(0,1fr)">' +
      card('col', 'gap:10px', lbl('CG envelope') + '<div class="placeholder" style="min-height:360px"><div style="font-weight:600">Weight &amp; balance arrives in the next update</div>' +
        '<div class="muted" style="font-size:14px;max-width:380px">The CG envelope, ZFW / TOW CG and trim settings need the real weight-and-balance model (Phase 2). Nothing is estimated here.</div></div>') +
      card('col', 'gap:12px', lbl('Live weights') + kv('Gross weight', mass(F.weights.gw)) + kv('Zero-fuel weight', mass(F.weights.zfw)) +
        kv('Fuel on board', mass(F.weights.fob)) + kv('ZFW CG', '<span class="na">—</span>') + kv('TOW CG', '<span class="na">—</span>') +
        kv('THS for take-off', '<span class="na">—</span>')) +
      '</div>';
  }

  function viewCabin() {
    var c = F.cabin, names = ['Main deck', 'Upper deck', 'Lower deck seating'], totals = [0, 0, 0, 0];
    var decks = [1, 0, 2].map(function (d) {
      var row = c.decks[d], used = 0, zones = '';
      var counts = row.slice(0, 4);
      if (row[4]) {
        var other = counts[0] * CABIN_UNITS[0] + counts[1] * CABIN_UNITS[1] + counts[2] * CABIN_UNITS[2];
        counts[3] = Math.max(0, Math.floor(CABIN_CAP[d] - other));
      }
      counts.forEach(function (n, k) { used += n * CABIN_UNITS[k]; totals[k] += n; });
      counts.forEach(function (n, k) {
        if (n > 0) zones += '<div style="flex:' + (n * CABIN_UNITS[k]).toFixed(1) + ' 0 0;background:' + CLASSES[k][1] + '">' + 'FJWY'[k] + ' ' + n + '</div>';
      });
      if (CABIN_CAP[d] - used > 0.5) zones += '<div class="free" style="flex:' + (CABIN_CAP[d] - used).toFixed(1) + ' 0 0">' + (CABIN_CAP[d] - used > 30 ? 'free' : '') + '</div>';
      var over = used > CABIN_CAP[d] + 0.01;
      return card('col', 'gap:10px;padding:14px', '<div style="display:flex;justify-content:space-between;align-items:center"><span style="font:700 16px/1 var(--sans)">' + names[d] +
        '</span><span class="mono" style="font-size:13px;color:' + (over ? '#F06A6A' : '#A3B1BF') + '">' + (over ? 'OVER CAPACITY' : Math.round(used / CABIN_CAP[d] * 100) + '% of floor (estimate)') + '</span></div>' +
        '<div class="zones">' + zones + '</div>' +
        '<div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr)) auto;gap:8px;align-items:end">' +
        ['First', 'Business', 'Prem. eco', 'Economy'].map(function (label, k) {
          var auto = k === 3 && row[4];
          return '<label class="col" style="gap:4px"><span class="lbl">' + label + '</span>' + field(auto ? null : '~kp cabin ' + d + ' ' + k,
            auto ? '<span style="color:#46C38B">' + counts[3] + '</span>' : String(row[k]), {disabled: auto}) + '</label>';
        }).join('') +
        '<button class="btn' + (row[4] ? ' primary' : '') + pressed('cabin ' + d + ' 4 ' + (row[4] ? 0 : 1)) + '"' + act('cabin ' + d + ' 4 ' + (row[4] ? 0 : 1)) + ' style="font-size:13px">Fill rest economy</button></div>');
    }).join('');
    var total = totals.reduce(function (a, b) { return a + b; }, 0);
    return '<div class="grid" style="grid-template-columns:minmax(0,1.9fr) minmax(0,1fr)">' +
      '<div class="col" style="gap:8px;min-width:0">' + decks +
      card('col', 'gap:10px;padding:14px', '<div style="display:flex;justify-content:space-between;align-items:center"><span style="font:700 16px/1 var(--sans)">Lower deck · cargo holds</span></div>' +
        '<div style="display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px"><div class="col" style="gap:6px"><span class="lbl">Forward hold</span>' +
        seg([['0', 'Containers'], ['1', 'Pallets']], String(c.pallets[0]), 'cabin hold 0') + '</div><div class="col" style="gap:6px"><span class="lbl">Aft hold</span>' +
        seg([['0', 'Containers'], ['1', 'Pallets']], String(c.pallets[1]), 'cabin hold 1') + '</div></div>') + '</div>' +
      card('col', 'gap:12px', '<div class="banner">PREVIEW · applies in a later update</div>' + lbl('Layout summary') +
        CLASSES.map(function (cl, k) {
          return '<div style="display:flex;align-items:center;gap:10px"><span style="width:14px;height:14px;border-radius:4px;background:' + cl[1] + '"></span><span style="flex:1">' + cl[0] +
            '</span><span class="mono">' + totals[k] + '</span></div>';
        }).join('') + '<div class="hr"></div>' + kv('Total seats', '<span style="color:' + (total > 853 ? '#F06A6A' : '#E6EDF3') + '">' + total + ' / 853</span>') +
        kv('Cabin weight change', '<span class="na">not modelled yet</span>') +
        '<div style="display:flex;gap:8px;margin-top:4px">' + btn(null, 'Apply to aircraft', 'primary', true) + btn('cabin reset', 'Default') + '</div>' +
        '<div class="note">The layout is stored now. 3D seats, lower-deck seating and the empty-weight change come in a later update (Phase 4); the aircraft is unchanged.</div>') +
      '</div>';
  }

  function viewPerf() {
    var t = F.tod, m = metric();
    var alt = function (ft) { return isNum(ft) ? (m ? group3(Math.round(ft * 0.3048)) + ' m' : group3(Math.round(ft)) + ' ft') : '<span class="na">—</span>'; };
    var dist = isNum(t.distanceNm) ? (m ? group3(Math.round(t.distanceNm * 1.852)) + ' km' : group3(Math.round(t.distanceNm)) + ' nm') : '<span class="na">—</span>';
    var speed = isNum(t.gsKt) ? (m ? Math.round(t.gsKt * 1.852) + ' km/h' : Math.round(t.gsKt) + ' kt') : '<span class="na">—</span>';
    return '<div class="grid" style="grid-template-columns:minmax(0,1.3fr) minmax(0,1fr)">' +
      card('col', 'gap:14px', lbl('Top of descent') +
        '<div style="display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px 16px">' +
        '<div class="col" style="gap:6px"><span class="lbl">Current altitude</span><div class="val">' + alt(t.altFt) + '</div></div>' +
        '<div class="col" style="gap:6px"><span class="lbl">Target altitude</span>' + field('~kp tod', alt(t.targetFt)) + '</div>' +
        '<div class="col" style="gap:6px"><span class="lbl">Flight path angle</span><div class="val">' + t.angle.toFixed(1) + '°</div></div>' +
        '<div style="display:flex;gap:8px;align-items:flex-end">' + btn('tod down', '−1000 ft') + btn('tod up', '+1000 ft') + '</div></div>' +
        '<div style="display:flex;gap:8px">' + btn('tod shallower', 'Angle −0.5°') + btn('tod steeper', 'Angle +0.5°') + btn('tod reset', 'Reset') + '</div>' +
        '<div class="hr"></div><div style="display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px">' +
        '<div>' + lbl('Start descent') + '<div class="val" style="font-size:30px;color:' + ACC + '">' + dist + '</div><div class="note">before the target</div></div>' +
        '<div>' + lbl('Average ground speed') + '<div class="val" style="font-size:30px">' + speed + '</div></div></div>') +
      '<div class="col" style="gap:16px">' +
      card('col', 'gap:10px', '<div style="display:flex;justify-content:space-between;align-items:center">' + lbl('Take-off performance') + '<span class="chip plan">NEXT UPDATE</span></div>' +
        '<div class="note">V1 / VR / V2, FLEX temperature and CONF from the real A380 performance model arrive in Phase 2. No speeds are shown until then.</div>') +
      card('col', 'gap:10px', '<div style="display:flex;justify-content:space-between;align-items:center">' + lbl('Landing performance') + '<span class="chip plan">NEXT UPDATE</span></div>' +
        '<div class="note">VAPP and landing distance arrive with the take-off calculator (Phase 2).</div>') + '</div>' +
      '</div>';
  }
  function viewNav() {
    var a = F.ac, m = metric();
    return '<div class="grid" style="grid-template-columns:minmax(0,1fr) minmax(0,1.4fr)">' +
      card('col', 'gap:12px', lbl('Present position') +
        kv('Latitude', isNum(a.lat) ? a.lat.toFixed(4) + '°' : '<span class="na">—</span>') + kv('Longitude', isNum(a.lon) ? a.lon.toFixed(4) + '°' : '<span class="na">—</span>') +
        kv('Altitude', isNum(a.altFt) ? (m ? group3(Math.round(a.altFt * 0.3048)) + ' m' : group3(Math.round(a.altFt)) + ' ft') : '<span class="na">—</span>') +
        kv('Ground speed', isNum(a.gsKt) ? (m ? Math.round(a.gsKt * 1.852) + ' km/h' : Math.round(a.gsKt) + ' kt') : '<span class="na">—</span>') +
        kv('Wind', isNum(a.windDir) && isNum(a.windKt) ? pad3(a.windDir) + '° / ' + Math.round(a.windKt) + ' kt' : '<span class="na">—</span>') +
        kv('QNH', isNum(a.qnh) ? Math.round(a.qnh) + ' hPa' : '<span class="na">—</span>') + '<div class="hr"></div>' + lbl('X-Plane map') +
        '<div style="display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px">' + btn('map current', 'Current') + btn('map low', 'Low enroute') + btn('map high', 'High enroute') +
        btn('map zoom_in', 'Zoom in') + btn('map zoom_out', 'Zoom out') + btn('map airports', 'Airports') + '</div>') +
      card('col', 'gap:12px', '<div style="display:flex;justify-content:space-between;align-items:center">' + lbl('Charts') + '<span class="chip plan">NEXT UPDATE</span></div>' +
        '<div class="placeholder" style="flex:1;min-height:360px"><svg width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="#A3B1BF" stroke-width="1.5"><path d="M3 6l6-2 6 2 6-2v14l-6 2-6-2-6 2z"/><path d="M9 4v14M15 6v14"/></svg>' +
        '<div style="font-weight:600">Chart viewer</div><div class="muted" style="font-size:14px;max-width:380px">An airport diagram drawn from X-Plane&#39;s own apt.dat, with your aircraft position, arrives in Phase 2.</div></div>') +
      '</div>';
  }
  function pad3(v) { v = Math.round(v) % 360; return (v < 10 ? '00' : v < 100 ? '0' : '') + v; }
  function viewComms() {
    var r = F.radios;
    function com(n, pair) {
      return card('col', 'gap:12px', lbl('COM ' + n) +
        '<div style="display:flex;align-items:center;gap:16px"><div style="flex:1">' + lbl('Active') + '<div class="val" style="font-size:30px;color:#7FD8A9">' + na(freq(pair[0])) + '</div></div>' +
        btn('radio swap' + n, '⇄ Swap') + '<div style="flex:1;text-align:right">' + lbl('Standby') + field('~kp com' + n, na(freq(pair[1])), {style: 'font-size:24px;height:50px;line-height:48px;text-align:right;margin-top:4px'}) + '</div></div>' +
        (n === 1 ? '<div>' + btn('radio guard', 'Standby 121.500') + '</div>' : ''));
    }
    var modes = [['1', 'STBY'], ['3', 'ALT'], ['6', 'TA ONLY'], ['7', 'TA/RA']];
    return '<div class="grid" style="grid-template-columns:repeat(2,minmax(0,1fr))">' + com(1, r.com1) + com(2, r.com2) +
      card('col', 'gap:14px', lbl('Transponder') + '<div style="display:flex;align-items:center;justify-content:space-between;gap:16px"><div style="width:180px">' +
        field('~kp squawk', isNum(r.squawk) ? ('000' + r.squawk).slice(-4) : '<span class="na">—</span>', {style: 'font-size:30px;height:56px;line-height:54px'}) + '</div>' + btn('radio ident', 'IDENT') + '</div>' +
        seg(modes, isNum(r.mode) ? String(r.mode) : '', 'radio mode') + (isNum(r.mode) && ['1', '3', '6', '7'].indexOf(String(r.mode)) < 0 ? '<div class="note">X-Plane mode ' + r.mode + '</div>' : '')) +
      card('col', 'gap:14px', lbl('Datalink') + '<div class="note">Hoppie / ACARS and CPDLC are not available in X-Plane yet. The A380 SURV panel on the MFD keeps its own transponder and TCAS controls.</div>') +
      '</div>';
  }

  function viewChecklists() {
    var g = Math.max(0, Math.min(F.checklists.length - 1, F.ui.group)), c = F.checklists[g];
    var groups = F.checklists.map(function (cl, i) {
      var done = doneCount(i), full = done === cl.items.length, key = 'cl_group ' + i;
      return '<button class="group' + (i === g ? ' on' : '') + (full ? ' full' : '') + pressed(key) + '"' + act(key) + '><span>' + esc(cl.name) + '</span><span class="mono" style="font-size:13px">' + done + '/' + cl.items.length + '</span></button>';
    }).join('');
    var items = c.items.map(function (item, i) {
      var done = !!(c.done & (1 << i)), automatic = !!(c.auto & (1 << i)), key = 'cl_item ' + g + ' ' + i;
      return '<div class="citem' + (done ? ' done' : '') + pressed(key) + '"' + act(key) + '><span class="box' + (done ? ' on' : '') + '"></span><span class="name"><span>' + esc(item[0]) + '</span>' +
        (item[2] ? '<span class="chip" style="margin-left:10px;' + (automatic ? 'border-color:#46C38B;color:#7FD8A9' : '') + '">AUTO</span>' : '') + '<span class="dots"></span><span class="mono">' + esc(item[1]) + '</span></span></div>';
    }).join('');
    return '<div class="grid" style="grid-template-columns:260px minmax(0,1fr)">' +
      card('col', 'gap:2px;padding:10px', groups) +
      card('col', 'gap:4px', '<div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px"><div style="font:700 20px/1 var(--sans)">' + esc(c.name) +
        '</div><div style="display:flex;gap:8px">' + btn('cl_reset ' + g, 'Reset') + btn('cl_next ' + g, 'Check item', 'primary') + '</div></div>' + items +
        '<div class="note" style="margin-top:8px">' + (F.settings.autoCheck ? 'Auto check is on: items marked AUTO tick themselves when the aircraft confirms them.' :
          'Auto check is off: tick every item by hand.') + '</div>') +
      '</div>';
  }
  function viewPresets() {
    var p = F.presets;
    var cards = PRESETS.map(function (pr) {
      var available = p.native || pr[0] === 1 || pr[0] === 2 || pr[0] === 4;
      return card('col', 'gap:12px;min-height:150px', '<div style="font:700 19px/1.2 var(--sans)">' + esc(pr[1]) + '</div><div class="muted" style="font-size:14px;flex:1">' + esc(pr[2]) +
        '</div><div>' + btn('preset ' + pr[0], p.active === pr[0] ? 'Stop' : 'Apply', 'primary', !available) + '</div>');
    }).join('');
    var status = !p.native ? 'Native A380 electrical network unavailable: X-Plane command presets only.' :
      p.active ? 'Running · step ' + p.step + ' / ' + p.count + (p.stepName ? ' · ' + esc(p.stepName) : '') :
      p.done ? esc((PRESETS[p.done - 1] || [0, ''])[1]) + ' complete' : 'No preset applied this session';
    return '<div class="grid" style="grid-template-columns:repeat(3,minmax(0,1fr))">' + cards +
      card('col', 'justify-content:center;gap:8px;min-height:150px', lbl('Status') + '<div style="font-size:15px;color:#7FD8A9">' + status + '</div>' +
        (p.native ? '<div style="display:flex;align-items:center;justify-content:space-between;gap:12px"><span>Expedited</span>' + sw(p.expedited, 'preset expedite') + '</div>' : '') +
        '<div class="note">Presets run on the ground only. Parked presets disarm the slides; departure presets arm the slides of closed doors.</div>') +
      '</div>';
  }
  function viewFailures() {
    var active = 0, rows = '';
    F.failures.forEach(function (f, i) {
      if (f[3] === 6) ++active;
      if (L.filter !== 'All' && f[1] !== L.filter) return;
      var state = f[3], on = state === 6, other = isNum(state) && state !== 0 && state !== 6;
      var key = 'fail ' + i + ' ' + (on ? 0 : 1);
      rows += '<div class="trow" style="grid-template-columns:60px 80px minmax(0,1fr) 140px;min-height:28px"><span class="mono muted">' + esc(f[2]) + '</span><span style="font-weight:600">' + esc(f[1]) +
        '</span><span class="mono">' + esc(f[0]) + '</span>' + (state == null ? '<span class="chip">UNAVAILABLE</span>' :
        '<button class="fstate' + (on ? ' active' : other ? ' other' : '') + pressed(key) + '"' + act(key) + '>' + (on ? 'Active' : other ? 'X-Plane ' + state : 'Inactive') + '</button>') + '</div>';
    });
    return card('col', 'gap:0', '<div style="display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:8px"><div style="display:flex;gap:6px;flex-wrap:wrap">' +
      FILTERS.map(function (k) { var key = '~filter ' + k; return '<button class="chipbtn' + (k === L.filter ? ' on' : '') + pressed(key) + '"' + act(key) + '>' + esc(k) + '</button>'; }).join('') +
      '</div>' + btn('fail_clear', 'Clear all') + '</div>' + lbl(active + ' active', 'margin-bottom:6px') +
      '<div class="trow thead" style="grid-template-columns:60px 80px minmax(0,1fr) 140px"><span class="lbl">ATA</span><span class="lbl">System</span><span class="lbl">Failure</span><span class="lbl">State</span></div>' +
      rows + '<div class="note" style="margin-top:8px">X-Plane system failures. Tap a state to fail or repair it now; scheduled failures set in X-Plane show their X-Plane mode.</div>');
  }
  function viewAircraft() {
    var o = F.options;
    function engine(k, name) {
      var on = o.engine === k, key = 'option engine ' + k;
      return '<button class="tile' + (on ? ' armed' : '') + pressed(key) + '"' + act(key) + ' style="height:auto;padding:16px;gap:8px;text-align:left"><span style="font:700 18px/1.2 var(--sans)">' + name +
        '</span><span class="muted" style="font-size:14px;font-weight:500">' + (on ? 'Selected · applies in a later update' : 'Tap to select') + '</span></button>';
    }
    function option(label, on, key, desc) {
      return card('col', 'gap:10px', '<div style="display:flex;justify-content:space-between;align-items:center;gap:12px"><span style="font:700 18px/1.2 var(--sans)">' + label + '</span>' +
        sw(on, key) + '</div><div class="muted" style="font-size:14px">' + desc + '</div><div class="mono" style="font-size:13px;color:#F7C77A">' +
        (on ? 'SELECTED' : 'NOT SELECTED') + ' · NO EFFECT YET</div>');
    }
    return '<div class="grid" style="grid-template-columns:repeat(2,minmax(0,1fr))">' +
      '<div class="banner" style="grid-column:span 2">These options are stored now and apply in a later update (Phase 3). The aircraft, its engines and its cockpit are unchanged.</div>' +
      card('col', 'grid-column:span 2;gap:12px', '<div style="display:flex;justify-content:space-between;align-items:center">' + lbl('Engines') + '<span class="mono muted">A380-842 · Trent 972B-84 installed</span></div>' +
        '<div style="display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px">' + engine(0, 'Rolls-Royce Trent 970') + engine(1, 'Engine Alliance GP7270') + '</div>') +
      option('Brake cooling fans', o.brakeFans, 'option brakefans ' + (o.brakeFans ? 0 : 1), 'Fans on the main-gear wheels: visible on the 3D gear and faster brake cooling on the ground.') +
      option('Head-up display', o.hud, 'option hud ' + (o.hud ? 0 : 1), 'HUD combiners for both pilots. When not installed, the HUD units disappear from the cockpit.') +
      '</div>';
  }
  function viewGeneral() {
    var s = F.settings, p = F.power;
    return '<div class="grid" style="grid-template-columns:repeat(2,minmax(0,1fr))">' +
      card('col', 'gap:16px', lbl('Units &amp; time') +
        '<div style="display:flex;align-items:center;justify-content:space-between;gap:16px"><span>Mass</span><div style="width:180px">' + seg([['1', 'kg'], ['0', 'lb']], s.metric ? '1' : '0', 'set metric') + '</div></div>' +
        '<div style="display:flex;align-items:center;justify-content:space-between;gap:16px"><span>Clock</span><div style="width:180px">' + seg([['UTC', 'UTC'], ['Local', 'Local']], L.clock, '~clock') + '</div></div>') +
      card('col', 'gap:6px', lbl('Behaviour', 'margin-bottom:8px') +
        '<div class="list-row"><span>Auto check checklists</span>' + sw(s.autoCheck, 'set autocheck ' + (s.autoCheck ? 0 : 1)) + '</div>' +
        '<div class="list-row"><span>Simulate tablet battery</span>' + sw(p.sim, 'set battery ' + (p.sim ? 0 : 1)) + '</div>' +
        '<div class="list-row"><span>Tablet power</span>' + btn('power', 'Turn off') + '</div>') +
      card('col', 'gap:12px', '<div class="lbl">Screen brightness · ' + Math.round(sliderValue('set brightness', s.brightness)) + ' %</div>' + slider('set brightness', s.brightness, 20, 100) +
        '<div class="note">Battery ' + (isNum(p.battery) ? Math.round(p.battery) + ' %' : '—') + (p.charging ? ' · charging from aircraft power' : '') + '</div>') +
      card('col', 'gap:8px', lbl('About') + '<div style="font-size:15px">A380 EFB for the X-Plane 12 A380X port</div>' +
        '<div class="muted" style="font-size:13px">Built on the FlyByWire flyPad page model; every value comes from the aircraft&#39;s own systems. Fonts: Manrope and JetBrains Mono (SIL OFL 1.1).</div>') +
      '</div>';
  }
  function viewSounds() {
    var items = F.sounds.map(function (s) {
      var meta = SOUNDS[s[0]] || [s[0], ''], key = 'vol ' + s[0], v = sliderValue(key, s[1]);
      return '<div class="col" style="gap:6px"><div style="display:flex;justify-content:space-between"><span style="font-weight:600;font-size:15px">' + meta[0] +
        '</span><span class="mono muted">' + (isNum(v) ? Math.round(v) + ' %' : '—') + '</span></div>' + slider(key, s[1], 0, 100) + '<span class="muted" style="font-size:12px">' + meta[1] + '</span></div>';
    }).join('');
    return card('', 'display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px 40px;align-content:start', lbl('Volume · X-Plane sound settings', 'grid-column:span 2') + items +
      '<div style="grid-column:span 2;display:flex;gap:10px;align-items:center">' + btn('vol restore', 'Restore values from aircraft load') +
      '<span class="note">Sliders write X-Plane&#39;s own volume settings.</span></div>') +
      card('col', 'gap:10px;margin-top:14px', lbl('Cockpit aurals') +
        '<div class="list-row"><span>GPWS / TAWS voice (PULL UP)</span>' +
        seg([['new', 'New A380'], ['old', 'Old A380']], F.gpwsVoice === 'old' ? 'old' : 'new', 'gpws voice') + '</div>' +
        '<span class="note">Plays only in the cockpit view. Saved for the next flights.</span>');
  }

  /* ---- Keypad ------------------------------------------------------------------------------------ */
  var KEYPADS = {
    pax: function (i) { return {title: CLASSES[i][0] + ' passengers', action: 'pax ' + i, max: 3, dot: false, hint: '0 – ' + F.load.seats[i] + ' seats', value: String(F.load.pax[i])}; },
    cargo: function (i) {
      var limit = toUnit(F.load.holdMax[i]);
      return {title: HOLDS[i] + ' (' + unit() + ')', action: 'cargo ' + i, max: 6, dot: false, hint: '0 – ' + group3(limit) + ' ' + unit(), value: String(toUnit(F.load.cargo[i]))};
    },
    fuel: function () { return {title: 'Block fuel (' + unit() + ')', action: 'fuel target', max: 6, dot: false, hint: '0 – ' + group3(toUnit(F.weights.maxFuel)) + ' ' + unit(), value: isNum(F.fuel.target) ? String(toUnit(F.fuel.target)) : ''}; },
    tod: function () { return {title: 'Target altitude (' + (metric() ? 'm' : 'ft') + ')', action: 'tod target', max: 5, dot: false, hint: metric() ? '0 – 15240 m' : '0 – 50000 ft', value: ''}; },
    com1: function () { return {title: 'COM 1 standby (MHz)', action: 'radio stby1', max: 7, dot: true, hint: '118.000 – 136.990', value: ''}; },
    com2: function () { return {title: 'COM 2 standby (MHz)', action: 'radio stby2', max: 7, dot: true, hint: '118.000 – 136.990', value: ''}; },
    squawk: function () { return {title: 'Squawk', action: 'radio squawk', max: 4, dot: false, hint: 'four digits 0 – 7', value: ''}; },
    cabin: function (d, k) { return {title: ['Main', 'Upper', 'Lower'][d] + ' deck · ' + ['First', 'Business', 'Premium economy', 'Economy'][k], action: 'cabin ' + d + ' ' + k, max: 3, dot: false, hint: 'seats, preview only', value: String(F.cabin.decks[d][k])}; }
  };
  function keypadOpen(args) {
    var make = KEYPADS[args[0]];
    if (!make) return;
    var spec = make(Number(args[1]), Number(args[2]));
    spec.text = '';
    spec.placeholder = spec.value;
    L.keypad = spec;
  }
  function keypadKey(k) {
    var kp = L.keypad;
    if (!kp) return;
    if (k === 'back') kp.text = kp.text.slice(0, -1);
    else if (k === 'clear') kp.text = '';
    else if (k === 'cancel') L.keypad = null;
    else if (k === 'enter') {
      if (kp.text !== '') post(kp.action + ' ' + kp.text);
      L.keypad = null;
    } else if (k === '.') { if (kp.dot && kp.text.indexOf('.') < 0 && kp.text.length < kp.max) kp.text += '.'; }
    else if (/^[0-9]$/.test(k) && kp.text.replace('.', '').length < kp.max) kp.text += k;
  }
  function viewKeypad() {
    var kp = L.keypad;
    if (!kp) return '';
    var keys = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '.', '0', 'back'].map(function (k) {
      var key = '~kp key ' + k, disabled = k === '.' && !kp.dot;
      return '<button class="key' + (disabled ? '' : pressed(key)) + '"' + (disabled ? ' disabled' : act(key)) + '>' + (k === 'back' ? '⌫' : k) + '</button>';
    }).join('');
    return '<div class="overlay"><div class="keypad"><div style="display:flex;justify-content:space-between;align-items:baseline"><span style="font:700 18px/1.2 var(--sans)">' + esc(kp.title) +
      '</span><span class="note">' + esc(kp.hint) + '</span></div><div class="display">' + (kp.text ? esc(kp.text) : '<span class="na">' + esc(kp.placeholder || '') + '</span>') + '</div>' +
      '<div class="keys">' + keys + '<button class="key wide' + pressed('~kp key clear') + '"' + act('~kp key clear') + '>Clear</button><button class="key wide' + pressed('~kp key cancel') + '"' +
      act('~kp key cancel') + '>Cancel</button><button class="key wide enter' + pressed('~kp key enter') + '"' + act('~kp key enter') + '>Enter</button></div></div></div>';
  }

  /* ---- Page ------------------------------------------------------------------------------------- */
  var VIEWS = {
    home: viewHome, ofp: viewOfp, cabin: viewCabin, failures: viewFailures, aircraft: viewAircraft,
    ground: {doors: viewDoors, services: viewServices, push: viewPush},
    loading: {payload: viewPayload, fuel: viewFuel, wb: viewWb},
    flight: {perf: viewPerf, nav: viewNav, comms: viewComms},
    ops: {checklists: viewChecklists, presets: viewPresets},
    settings: {general: viewGeneral, sounds: viewSounds}
  };
  function view() {
    if (!F) return '<div class="offscreen"></div>';
    if (F.power && F.power.blank) return '<div class="offscreen"></div>';
    var u = ui(), section = u.section, meta = SECTIONS.filter(function (s) { return s[0] === section; })[0] || SECTIONS[0];
    var tabs = TABS[section] || [];
    var v = VIEWS[section] || viewHome;
    if (typeof v === 'object') v = v[u.tab] || v[tabs[0][0]];
    var phase = isNum(F.ac.phase) ? PHASES[F.ac.phase] || 'Phase ' + F.ac.phase : F.ac.onGround ? 'On ground' : 'Airborne';
    var battery = F.power.sim ? (isNum(F.power.battery) ? Math.round(F.power.battery) + '%' : '—') : 'ext';
    var html = '<div id="screen"><header><div class="brand"><svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="' + ACC + '" stroke-width="1.8" stroke-linejoin="round">' + ICONS.flight +
      '</svg><span>A380 EFB</span><span class="chip ' + (valid ? 'live' : 'warn') + '">' + (valid ? 'LIVE' : 'NO DATA') + '</span></div>' +
      '<div class="mid"><span class="muted" style="font-weight:500">' + esc(F.ac.variant) + ' · ' + esc(phase) + '</span></div>' +
      '<div class="right"><span class="mono">' + clockText() + '</span><span class="battery' + (F.power.sim && F.power.battery < 15 ? ' low' : '') + '">' +
      '<svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.6"><rect x="2" y="7" width="18" height="10" rx="2"/><path d="M22 11v2"/>' +
      '<rect x="4" y="9" width="' + (F.power.sim && isNum(F.power.battery) ? Math.max(1, F.power.battery * 0.14).toFixed(1) : 14) + '" height="6" rx="1" fill="currentColor" stroke="none"/></svg>' +
      battery + (F.power.charging ? ' ⚡' : '') + '</span></div></header><div class="body"><nav class="rail">';
    SECTIONS.forEach(function (s) {
      if (s[0] === 'settings') html += '<div class="spacer"></div>';
      var key = 'nav ' + s[0];
      html += '<button class="' + (s[0] === section ? 'on' : '') + pressed(key) + '"' + act(key) + '><svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round">' +
        ICONS[s[0]] + '</svg>' + s[1] + '</button>';
    });
    html += '</nav><main><div class="titlebar"><h1>' + meta[2] + '</h1>' + (tabs.length ? '<div class="tabs">' + tabs.map(function (t) {
      var key = 'tab ' + section + ' ' + t[0];
      return '<button class="tab' + (t[0] === u.tab ? ' on' : '') + pressed(key) + '"' + act(key) + '>' + esc(t[1]) + '</button>';
    }).join('') + '</div>' : '') + '</div>' + v() + (L.toast ? '<div class="toast">' + esc(L.toast) + '</div>' : '') + '</main></div>' + viewKeypad() +
      (valid ? '' : '<div class="stale">No data from the aircraft<span class="note" style="font-size:15px">The tablet shows the last values it received.</span></div>') + '</div>';
    return html;
  }

  /* ---- Actions ----------------------------------------------------------------------------------- */
  function post(action) {
    var id = ++L.nextId;
    sent.push(id + ' ' + action);
    if (sent.length > 64) sent.shift();
    var handler = global.webkit && global.webkit.messageHandlers && global.webkit.messageHandlers.a380;
    if (handler) handler.postMessage({type: 'a380x-efb-action', id: id, action: action});
    return id;
  }
  function activate(key) {
    var parts = key.split(' ');
    if (parts[0] === '~doormode') L.doorMode = parts[1];
    else if (parts[0] === '~filter') L.filter = parts[1];
    else if (parts[0] === '~clock') L.clock = parts[1];
    else if (parts[0] === '~kp') { if (parts[1] === 'key') keypadKey(parts[2]); else keypadOpen(parts.slice(1)); }
    else if (parts[0] === 'nav' || parts[0] === 'tab') {
      L.optimistic = {section: parts[1], tab: parts[2], id: post(key)};
      L.keypad = null;
    } else if (parts[0] === 'openchecklist') {
      post('cl_group ' + parts[1]);
      L.optimistic = {section: 'ops', tab: 'checklists', id: post('nav ops checklists')};
    } else post(key);
  }

  /* ---- Touch ------------------------------------------------------------------------------------- */
  function hit(x, y) {
    if (typeof document === 'undefined') return null;
    var e = document.elementFromPoint(x, y);
    while (e && e !== document.body) {
      if (e.getAttribute) {
        if (e.getAttribute('data-s')) return {slider: e};
        if (e.getAttribute('data-a')) return {key: e.getAttribute('data-a')};
      }
      e = e.parentNode;
    }
    return null;
  }
  function sliderAt(element, x) {
    var r = element.getBoundingClientRect(), min = Number(element.getAttribute('data-min')), max = Number(element.getAttribute('data-max'));
    var p = r.width > 0 ? Math.max(0, Math.min(1, (x - r.left) / r.width)) : 0;
    return Math.round(min + p * (max - min));
  }
  function dragTo(x, final) {
    var d = L.drag, now = Date.now();
    var value = sliderAt(d.element, x);
    L.sliders[d.key] = {value: value, until: now + 1500};
    if (final || now - d.sentAt > 120 && value !== d.sentValue) {
      post(d.key + ' ' + value);
      d.sentAt = now; d.sentValue = value;
    }
  }
  function touch(type, x, y) {
    if (F && F.power && F.power.blank) return;
    if (type === 1) {
      var target = hit(x, y);
      L.pressed = null; L.drag = null;
      if (!target) return;
      if (target.slider) {
        L.drag = {key: target.slider.getAttribute('data-s'), element: target.slider, sentAt: 0, sentValue: null};
        dragTo(x, false);
      } else {
        L.pressed = target.key;
        L.pressX = x; L.pressY = y;
      }
    } else if (type === 0) {
      if (L.drag) dragTo(x, false);
    } else if (type === 2 || type === 3) {
      if (L.drag) { if (type === 2) dragTo(x, true); L.drag = null; return; }
      var key = L.pressed;
      L.pressed = null;
      if (!key || type === 3) return;
      var over = hit(x, y);
      if ((over && over.key === key) || Math.abs(x - L.pressX) < 24 && Math.abs(y - L.pressY) < 24) activate(key);
    }
  }
  function touches(frame) {
    var t = frame.touch || {serial: 0, events: []};
    if (L.epoch !== frame.epoch || t.serial < L.lastSeq) {   /* first frame or plugin reload: skip history */
      L.epoch = frame.epoch; L.lastSeq = t.serial;
      return;
    }
    t.events.forEach(function (e) {
      if (e[0] <= L.lastSeq) return;
      L.lastSeq = e[0];
      touch(e[1], e[2], e[3]);
    });
  }

  /* ---- Bridge ------------------------------------------------------------------------------------ */
  var root = null;
  function tick() {
    var now = Date.now();
    if (L.toast && now > L.toastUntil) L.toast = null;
    for (var k in L.sliders) if (now > L.sliders[k].until && !(L.drag && L.drag.key === k)) delete L.sliders[k];
  }
  function render() {
    tick();
    var html = view();
    if (html !== lastHtml) {
      lastHtml = html;
      dirty = true;
      if (root) root.innerHTML = html;
    }
  }
  function consume() { var d = dirty; dirty = false; return d; }
  function apply(frame, inputValid) {
    if (!frame || typeof frame !== 'object' || !frame.ui) return consume();
    if (F) touches(frame);
    else { L.epoch = frame.epoch; L.lastSeq = frame.touch ? frame.touch.serial : 0; }
    if (frame.ack > L.nextId) L.nextId = frame.ack;
    if (L.optimistic && frame.ack >= L.optimistic.id) L.optimistic = null;
    if (L.lastStatus !== null && frame.status !== L.lastStatus && frame.status) { L.toast = frame.status; L.toastUntil = Date.now() + 5000; }
    L.lastStatus = frame.status;
    F = frame;
    valid = inputValid !== false;
    render();
    return consume();
  }
  /* Called by the helper on every tick without a new frame: only a toast or a held slider value
   * that has just expired can change the page, so the HTML is rebuilt only then. */
  function idle() {
    var now = Date.now(), due = L.toast && now > L.toastUntil;
    for (var k in L.sliders) if (now > L.sliders[k].until && !(L.drag && L.drag.key === k)) due = true;
    if (F && due) render();
    return consume();
  }
  global.__a380Efb = {apply: apply, idle: idle};
  global.__a380EfbCore = {
    view: view, activate: activate, touch: touch, mass: mass, clockText: clockText, keypad: function () { return L.keypad; },
    local: L, sent: sent, setFrame: function (frame, inputValid) { F = frame; valid = inputValid !== false; }
  };

  if (typeof document !== 'undefined') {
    var report = function (text) {
      var h = global.webkit && global.webkit.messageHandlers && global.webkit.messageHandlers.a380;
      if (h) h.postMessage({type: 'a380x-error', text: String(text)});
    };
    global.addEventListener('error', function (e) { report(e.message); });
    var start = function () {
      root = document.getElementById('efb');
      var tablet = /tablet=1/.test(global.location ? global.location.search : '') ? 1 : 0;
      document.title = 'A380X EFB ' + (tablet ? 'F/O' : 'CAPT');
      var ready = function () {
        var h = global.webkit && global.webkit.messageHandlers && global.webkit.messageHandlers.a380;
        if (h) h.postMessage({type: 'a380x-loaded', ok: true});
      };
      if (document.fonts && document.fonts.ready) document.fonts.ready.then(ready, ready); else ready();
    };
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start); else start();
  }
})(this);
