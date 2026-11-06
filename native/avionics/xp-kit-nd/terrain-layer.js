// SPDX-License-Identifier: GPL-3.0-or-later
// A380X-ND-TERRAIN v1 - TERRAIN on ND for the FlyByWire ND page (docs/ND_TERRAIN.md).
// Loaded after native-bridge.js. In MSFS the FBW terrain image is drawn by the terronnd WASM gauge
// beneath the JS ND; here this layer stands in for it: the plugin sends, per ND, the EGPWS raster it
// swept (state.terrain.nd[side], native/avionics/a380x_xpkit_terrain.inc - 8x8 px cells, one class
// letter per run); it is drawn on a canvas inserted as the FIRST child of the ND map container, so
// every SVG/canvas symbol of the bundle stays above it, clipped by FBW's own arc-mode-wx-terr-clip /
// rose-mode-wx-terr-clip paths, with FBW SimBridge's A380 dot patterns (anchored at the image origin).
// It also feeds the bundle's own TERR box (EgpwcBusPublisher L:vars) and TAWS flags (AesuBusPublisher:
// TERR INOP, NO TERR AND WX DATA AVAILABLE) and the EFIS CP overlay (WX / TERR).
(() => {
  const base = window.__a380xTrial, nd = window.__a380Nd;
  if (!base || !nd) return;
  const side = new URLSearchParams(location.search).get('index') === '2' ? 1 : 0;
  const letter = side ? 'R' : 'L';
  const set = (name, value) => base.vars.set(name.toUpperCase(), value);
  const CELL = 8;
  // a380x_terrain.h classes: 0 none, green low/high/solid, yellow low/high, red high, water, unknown,
  // caution (solid yellow), warning (solid red). Colours and patterns: FBW SimBridge.
  const COLOUR = [null, '#00ff00', '#00ff00', '#00ff00', '#ffff32', '#ffff32', '#ff0000', '#00ffff', '#ff94ff', '#ffff32', '#ff0000'];
  const PATTERN = [0, 1, 2, 4, 1, 2, 2, 3, 2, 4, 4];          // 1 low, 2 high, 3 water density, 4 solid
  const TILE = { 1: [13, 5], 2: [6, 2], 3: [9, 4] };          // period, dot offset (two 3x3 dots)
  const CLIP = {
    arc: 'M0,312 a492,492 0 0 1 768,0 L768,562 L648,562 L591,625 L0,625 L0,312',
    rose: 'M45,155 L282,155 a250,250 0 0 1 204,0 L723,155 L723,384 L45,384 L45,155',
  };
  let canvas = null, context = null, patterns = null, drawnKey = '', revision = 0, capturedRevision = -1, scale = 1;
  let lastCells = null;

  const patternOn = (pattern, x, y) => {
    if (pattern === 4) return true;
    const tile = TILE[pattern];
    if (!tile) return false;
    const [p, c] = tile, mx = ((x % p) + p) % p, my = ((y % p) + p) % p;
    return (((mx - c + p) % p) < 3 && (my + 1) % p < 3) || ((mx + 1) % p < 3 && ((my - c + p) % p) < 3);
  };
  function makePatterns() {
    const made = [];
    for (let c = 1; c < COLOUR.length; c++) {
      if (PATTERN[c] === 4) { made[c] = COLOUR[c]; continue; }
      const [p] = TILE[PATTERN[c]];
      const tile = document.createElement('canvas');
      tile.width = tile.height = p;
      const t = tile.getContext('2d');
      t.fillStyle = COLOUR[c];
      for (let y = 0; y < p; y++) for (let x = 0; x < p; x++) if (patternOn(PATTERN[c], x, y)) t.fillRect(x, y, 1, 1);
      made[c] = context.createPattern(tile, 'repeat');
    }
    return made;
  }
  // "a95b3c" -> runs [[class, length], ...]; null when malformed or not rows x cols cells.
  function decode(cells, rows, cols) {
    const runs = [], re = /([a-k])(\d*)/gy;
    let total = 0, match;
    if (typeof cells !== 'string') return null;
    re.lastIndex = 0;
    while (re.lastIndex < cells.length) {
      match = re.exec(cells);
      if (!match) return null;
      const length = match[2] ? Number(match[2]) : 1;
      if (!(length > 0)) return null;
      runs.push([match[1].charCodeAt(0) - 97, length]);
      total += length;
    }
    return total === rows * cols ? runs : null;
  }
  function ensureCanvas() {
    const map = document.querySelector('canvas.nd-canvas-map');
    const container = map?.parentNode;
    if (!container) return false;
    if (!canvas || canvas.parentNode !== container) {
      // Backing store at the capture scale (the helper's device scale factor): a 1:1 layer to composite,
      // the 1-px pattern dots scaled by nearest-neighbour sampling.
      scale = Math.max(1, Math.min(4, Number(window.devicePixelRatio) || 1));
      canvas = document.createElement('canvas');
      canvas.width = canvas.height = Math.round(768 * scale);
      canvas.className = 'a380x-terrain';
      canvas.style.cssText = 'position:absolute;top:0;left:0;width:768px;height:768px;pointer-events:none';
      container.insertBefore(canvas, container.firstChild);
      context = canvas.getContext('2d');
      patterns = makePatterns();
    }
    return true;
  }
  function draw(t) {
    context.setTransform(scale, 0, 0, scale, 0, 0);
    context.imageSmoothingEnabled = false;
    context.clearRect(0, 0, 768, 768);
    if (!t) return true;
    const runs = decode(t.cells, t.rows, t.cols);
    if (!runs) return !t.cells;
    lastCells = runs;
    context.save();
    context.clip(new Path2D(t.mode === 3 ? CLIP.arc : CLIP.rose));
    context.translate(t.x0, t.y0);
    const rects = [];
    let k = 0;
    for (const [c, length] of runs) {
      let left = length;
      while (left > 0) {
        const row = Math.floor(k / t.cols), col = k % t.cols, span = Math.min(left, t.cols - col);
        if (c) (rects[c] = rects[c] || []).push([col * CELL, row * CELL, span * CELL, CELL]);
        k += span; left -= span;
      }
    }
    for (let c = 1; c < rects.length; c++) {
      if (!rects[c]) continue;
      context.fillStyle = patterns[c];
      context.beginPath();
      for (const [x, y, w, h] of rects[c]) context.rect(x, y, w, h);
      context.fill();
    }
    context.restore();
    return true;
  }
  function update(state) {
    const terrain = (state && state.ok && state.terrain) || {};
    const t = Array.isArray(terrain.nd) ? terrain.nd[side] : null;
    const on = !!(t && t.on);
    // AesuBusPublisher / FcuBusPublisher inputs of the bundle (VD flags, WXR INOP when WX is selected:
    // this port has no weather radar).
    set('L:A32NX_WXR_TAWS_SYS_SELECTED', Number(terrain.sys) || 0);
    set('L:A32NX_GPWS_TERR_OFF', terrain.terrOff ? 1 : 0);
    set('L:A32NX_TERR_1_FAILED', terrain.failed ? 1 : 0);
    set('L:A32NX_TERR_2_FAILED', terrain.failed ? 1 : 0);
    set('L:A32NX_WXR_1_FAILED', 1);
    set('L:A32NX_WXR_2_FAILED', 1);
    set(`L:A380X_EFIS_${letter}_ACTIVE_OVERLAY`, Array.isArray(terrain.overlay) ? Number(terrain.overlay[side]) || 0 : 0);
    // EgpwcBusPublisher -> TerrainMapThresholds (hidden while min or max is negative).
    set(`L:A32NX_EGPWC_ND_${letter}_TERRAIN_ACTIVE`, on ? 1 : 0);
    set(`L:A32NX_EGPWC_ND_${letter}_TERRAIN_MIN_ELEVATION`, on ? t.min : -1);
    set(`L:A32NX_EGPWC_ND_${letter}_TERRAIN_MIN_ELEVATION_MODE`, on ? t.minMode : 0);
    set(`L:A32NX_EGPWC_ND_${letter}_TERRAIN_MAX_ELEVATION`, on ? t.max : -1);
    set(`L:A32NX_EGPWC_ND_${letter}_TERRAIN_MAX_ELEVATION_MODE`, on ? t.maxMode : 0);
    set('L:A32NX_EGPWC_TERRONND_RENDERING_MODE', 3);   // FBW: scan-line sweep | vertical display (A380)
    const key = on ? `${t.rev}|${t.mode}|${t.range}` : 'off';
    if (key !== drawnKey && ensureCanvas() && draw(on ? t : null)) { drawnKey = key; revision++; }
  }
  const apply = base.apply;
  base.apply = (state) => { apply(state); update(state); };
  const needsSnapshot = nd.needsSnapshot;
  // The bundle's own stamp ignores state.terrain: a new raster (sweep step) also needs a capture.
  nd.needsSnapshot = () => {
    const changed = needsSnapshot.call(nd), terrain = revision !== capturedRevision;
    capturedRevision = revision;
    return changed || terrain;
  };
  window.__a380NdTerrain = { decode, patternOn, get revision() { return revision; }, get canvas() { return canvas; },
    get runs() { return lastCells; } };
})();
