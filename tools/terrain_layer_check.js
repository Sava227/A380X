/* SPDX-License-Identifier: GPL-3.0-or-later
 * JavaScriptCore check of the web ND terrain layer (native/avionics/xp-kit-nd/terrain-layer.js) loaded
 * after the real bridge.js and native-bridge.js, fed with terrain states made by the real plugin code
 * (test_terrain_adapter --json <scenario>). Minimal DOM: the ND map container, recording 2D contexts.
 * Run: jsc terrain_layer_check.js -- <bridge.js> <native-bridge.js> <terrain-layer.js> <state.json>...
 */
var g = this;
g.window = g;
g.location = {search: '?index=1&duID=1', href: 'file:///xp-kit/nd.html?index=1&duID=1', origin: 'file://'};
g.URLSearchParams = function (s) { this.s = String(s); };
g.URLSearchParams.prototype.get = function (k) { var m = new RegExp('[?&]' + k + '=([^&]*)').exec(this.s); return m ? m[1] : null; };
g.URL = function (name) { this.pathname = String(name).replace(/^[a-z]+:\/\/[^\/]*/, '').replace(/\?.*$/, ''); };
var clockMs = 1000;
g.performance = {now: function () { return clockMs; }};
g.__A380_NATIVE = true;
g.__a380NdSource = 'bundle';
g.webkit = {messageHandlers: {a380: {postMessage: function () {}}}};
g.addEventListener = function () {};
g.MouseEvent = function () {};
function check(c, message) { if (!c) throw new Error('terrain layer check failed: ' + message); }
function Ctx() { this.ops = []; this.fillStyle = null; }
['clearRect', 'save', 'restore', 'translate', 'beginPath', 'rect', 'clip', 'setTransform'].forEach(function (name) {
  Ctx.prototype[name] = function () { this.ops.push([name].concat(Array.prototype.slice.call(arguments))); };
});
Ctx.prototype.fill = function () { this.ops.push(['fill', this.fillStyle]); };
Ctx.prototype.fillRect = function (x, y, w, h) { this.ops.push(['fillRect', x, y, w, h, this.fillStyle]); };
Ctx.prototype.createPattern = function (tile, repeat) { return {tile: tile, repeat: repeat}; };
var canvases = [];
function Canvas() { this.style = {}; this.ctx = new Ctx(); this.parentNode = null; canvases.push(this); }
Canvas.prototype.getContext = function () { return this.ctx; };
var container = {children: [], insertBefore: function (node, ref) {
  check(ref === this.children[0], 'inserted before the first child');
  node.parentNode = this; this.children.unshift(node);
}};
Object.defineProperty(container, 'firstChild', {get: function () { return this.children[0]; }});
var mapCanvas = new Canvas();
mapCanvas.parentNode = container;
container.children.push({nodeName: 'svg'}, mapCanvas);
g.document = {createElement: function () { return new Canvas(); },
  querySelector: function (s) { return s === 'canvas.nd-canvas-map' ? mapCanvas : null; },
  elementFromPoint: function () { return null; }, documentElement: {setAttribute: function () {}}};
g.Path2D = function (d) { this.d = d; };
var files = arguments;
load(files[0]); load(files[1]); load(files[2]);
var trial = g.__a380xTrial, terrainApi = g.__a380NdTerrain, nd = g.__a380Nd;
check(trial && terrainApi && nd, 'scripts loaded');
function v(name) { return trial.vars.get(name.toUpperCase()); }
function state(terrain) {
  return {ok: true, mode: 'native', values: {}, words: {}, numbers: {},
    nav: {positionValid: true, lat: 45, lon: 7, magVar: 0, controls: [{mode: 3, range: 3}, {mode: 3, range: 3}],
          pointers: [{inside: 0, events: []}, {inside: 0, events: []}], route: [], symbols: []},
    terrain: terrain};
}
var summary = [];
for (var f = 3; f < files.length; ++f) {
  var input = JSON.parse(readFile(files[f])), t = input.terrain.nd[0];
  check(t.on === 1 && typeof t.cells === 'string', files[f] + ': terrain on');
  var before = terrainApi.revision;
  if (terrainApi.canvas) terrainApi.canvas.ctx.ops = [];
  trial.apply(state(input.terrain));
  check(v('L:A32NX_EGPWC_ND_L_TERRAIN_ACTIVE') === 1 && v('L:A32NX_EGPWC_ND_L_TERRAIN_MAX_ELEVATION') === t.max &&
        v('L:A32NX_EGPWC_ND_L_TERRAIN_MIN_ELEVATION') === t.min && v('L:A32NX_EGPWC_ND_L_TERRAIN_MAX_ELEVATION_MODE') === t.maxMode &&
        v('L:A32NX_EGPWC_ND_L_TERRAIN_MIN_ELEVATION_MODE') === t.minMode, 'TERR box vars');
  check(v('L:A32NX_WXR_TAWS_SYS_SELECTED') === input.terrain.sys && v('L:A32NX_TERR_1_FAILED') === 0 &&
        v('L:A32NX_GPWS_TERR_OFF') === 0 && v('L:A380X_EFIS_L_ACTIVE_OVERLAY') === 2 && v('L:A32NX_WXR_1_FAILED') === 1, 'TAWS vars');
  var layer = terrainApi.canvas;
  check(layer && container.firstChild === layer && layer.parentNode === container && layer.width === 768, 'canvas under the symbology');
  check(terrainApi.revision === before + 1, 'redrawn once');
  var ops = layer.ctx.ops, clip = ops.filter(function (o) { return o[0] === 'clip'; });
  check(clip.length === 1 && clip[0][1].d.indexOf(t.mode === 3 ? 'M0,312 a492,492' : 'M45,155 L282,155') === 0, 'FBW clip');
  var translate = ops.filter(function (o) { return o[0] === 'translate'; })[0];
  check(translate[1] === t.x0 && translate[2] === t.y0, 'image origin');
  var runs = terrainApi.decode(t.cells, t.rows, t.cols), area = {}, cells = {}, k = 0, current = null;
  check(runs, 'decodes');
  runs.forEach(function (r) { cells[r[0]] = (cells[r[0]] || 0) + r[1]; });
  ops.forEach(function (o) {
    if (o[0] === 'beginPath') current = [];
    else if (o[0] === 'rect') { current.push(o); check(o[4] === 8 && o[3] % 8 === 0 && o[1] % 8 === 0 && o[2] % 8 === 0, 'cell rects'); }
    else if (o[0] === 'fill') {
      var cls = classOf(o[1]);
      check(cls > 0, 'fill style is a class pattern');
      current.forEach(function (r) { area[cls] = (area[cls] || 0) + r[3] * r[4]; });
    }
  });
  for (var c = 1; c < 11; ++c) check((area[c] || 0) === (cells[c] || 0) * 64, 'class ' + c + ' area');
  // the same state again: no redraw, no capture; then a new revision: one capture
  check(nd.needsSnapshot() === true && nd.needsSnapshot() === false, 'capture on a new raster only');
  trial.apply(state(input.terrain));
  check(terrainApi.revision === before + 1 && nd.needsSnapshot() === false, 'unchanged raster');
  var next = JSON.parse(JSON.stringify(input.terrain)); next.nd[0].rev += 1;
  trial.apply(state(next));
  check(terrainApi.revision === before + 2 && nd.needsSnapshot() === true, 'new revision captured');
  if (input.taws && input.taws.warning) {   // TAWS look-ahead warning state: solid caution (9) and warning (10) cells, the PFD's EGPWS word
    check(cells[9] > 0 && cells[10] > 0 && area[9] > 0 && area[10] > 0, 'TAWS threat cells drawn solid');
    var probe = {ok: true, mode: 'native', values: {}, words: input.words, numbers: {}, nav: state(next).nav, terrain: next};
    var f32 = new Float32Array([2048]), u32 = new Uint32Array(f32.buffer)[0];
    trial.apply(probe);
    check(v('L:A32NX_EGPWS_ALERT_1_DISCRETE_WORD_1') === u32 + 3 * Math.pow(2, 32), 'EGPWS word 1: PULL UP bit 12, normal operation');
    summary.push('TAWS ' + cells[9] + ' caution + ' + cells[10] + ' warning cells');
  }
  summary.push(files[f].replace(/^.*\//, '') + ' ' + runs.length + ' runs');
  clockMs += 1000;
}
// fill style -> class: solid classes fill with their colour; the pattern tiles were created in class
// order 1, 2, 4, 5, 6, 7, 8 (makePatterns skips the solid 3, 9, 10)
function classOf(style) {
  var solid = {'#00ff00': 3, '#ffff32': 9, '#ff0000': 10};
  if (typeof style === 'string') return solid[style] || -1;
  var tiles = canvases.filter(function (x) { return x !== mapCanvas && x !== terrainApi.canvas; });
  var index = tiles.indexOf(style && style.tile);
  return index < 0 ? -1 : [1, 2, 4, 5, 6, 7, 8][index];
}
// pattern tiles: FBW densities (18 dot pixels per tile) and colours
var tiles = canvases.filter(function (x) { return x !== mapCanvas && x !== terrainApi.canvas; });
check(tiles.length === 7, 'seven pattern tiles');
[[0, 13, '#00ff00'], [1, 6, '#00ff00'], [2, 13, '#ffff32'], [3, 6, '#ffff32'], [4, 6, '#ff0000'], [5, 9, '#00ffff'], [6, 6, '#ff94ff']].forEach(function (e) {
  var tile = tiles[e[0]], dots = tile.ctx.ops.filter(function (o) { return o[0] === 'fillRect'; });
  check(tile.width === e[1] && dots.length === 18 && dots.every(function (o) { return o[5] === e[2]; }), 'tile ' + e[0]);
});
// terrain off: cleared, TERR box hidden
var off = JSON.parse(readFile(files[3])).terrain; off.nd[0] = {on: 0, rev: 99};
trial.apply(state(off));
check(v('L:A32NX_EGPWC_ND_L_TERRAIN_ACTIVE') === 0 && v('L:A32NX_EGPWC_ND_L_TERRAIN_MAX_ELEVATION') === -1 &&
      v('L:A32NX_EGPWC_ND_L_TERRAIN_MIN_ELEVATION') === -1, 'off hides the box');
var lastOps = terrainApi.canvas.ctx.ops;
check(lastOps[lastOps.length - 1][0] === 'clearRect', 'off clears the layer');
// TAWS states
off.sys = 0; off.failed = 1; off.terrOff = 1; off.overlay = [1, 0];
trial.apply(state(off));
check(v('L:A32NX_WXR_TAWS_SYS_SELECTED') === 0 && v('L:A32NX_TERR_1_FAILED') === 1 && v('L:A32NX_GPWS_TERR_OFF') === 1 &&
      v('L:A380X_EFIS_L_ACTIVE_OVERLAY') === 1, 'TAWS flags');
// malformed cells are not drawn
check(terrainApi.decode('a3z', 1, 3) === null && terrainApi.decode('a2', 1, 3) === null && terrainApi.decode('a3', 1, 3).length === 1, 'decode guards');
print('TERRAIN LAYER OK: ' + summary.join(', '));
