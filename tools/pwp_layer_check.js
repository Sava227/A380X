/* SPDX-License-Identifier: GPL-3.0-or-later
 * JavaScriptCore check of the web ND pseudo-waypoint layer (native/avionics/xp-kit-nd/fmgc-pwp-layer.js)
 * loaded, as in the page, after the real bridge.js, native-bridge.js and terrain-layer.js, fed with ND
 * states whose nav.route / nav.pseudo come from the closed-loop flight (test_fmgc_flight writes them
 * with the plugin's own serializer a380x_fmgc_nd_pwp_json). It drives the native bridge's onUpdate
 * with a recording bus and checks the 'symbols' messages against the FBW bundle's own filters
 * (MapLayers: PseudoWaypointLayer / WaypointLayer selection).
 * Run: jsc pwp_layer_check.js -- <bridge.js> <native-bridge.js> <terrain-layer.js> <fmgc-pwp-layer.js> <state.json>...
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
var posted = [];
g.webkit = {messageHandlers: {a380: {postMessage: function (m) { posted.push(m); }}}};
g.addEventListener = function () {};
g.MouseEvent = function () {};
function check(c, message) { if (!c) throw new Error('pwp layer check failed: ' + message); }
function Ctx() {}
['clearRect', 'save', 'restore', 'translate', 'beginPath', 'rect', 'clip', 'setTransform', 'fill', 'fillRect'].forEach(function (name) {
  Ctx.prototype[name] = function () {};
});
Ctx.prototype.createPattern = function () { return {}; };
function Canvas() { this.style = {}; this.ctx = new Ctx(); this.parentNode = null; }
Canvas.prototype.getContext = function () { return this.ctx; };
var container = {children: [], insertBefore: function (node) { node.parentNode = this; this.children.unshift(node); }};
Object.defineProperty(container, 'firstChild', {get: function () { return this.children[0]; }});
var mapCanvas = new Canvas();
mapCanvas.parentNode = container;
container.children.push({nodeName: 'svg'}, mapCanvas);
g.document = {createElement: function () { return new Canvas(); },
  querySelector: function (s) { return s === 'canvas.nd-canvas-map' ? mapCanvas : s === 'a380x-nd' ? {frameCount: 1} : null; },
  elementFromPoint: function () { return null; }, documentElement: {setAttribute: function () {}}};
g.Path2D = function (d) { this.d = d; };
var files = arguments;
load(files[0]); load(files[1]); load(files[2]); load(files[3]);
var trial = g.__a380xTrial, nd = g.__a380Nd, pwpApi = g.__a380NdPwp;
check(trial && nd && pwpApi && g.__a380NdTerrain, 'scripts loaded');

// the FBW bundle's MapLayers selection (nd.html PseudoWaypointLayer / WaypointLayer)
function bundlePwp(it) { return (it.type & (8192 | 16384)) || (it.typePwp !== undefined && (it.typePwp & (1 | 64 | 16 | 4 | 32 | 128 | 512 | 1024 | 2 | 256 | 8))); }
function bundleWaypoint(it) { return (it.type & (8 | 4096 | 2048 | 2 | 1 | 512 | 4 | 16)) && !(it.type & 32); }
function colourOf(type) { return type & 131072 ? 'magenta' : type & 65536 ? 'amber' : type & 32768 ? 'cyan' : 'white'; }

var sent = [];
function Bus() {}
Bus.prototype.send = function (topic, data) { sent.push([topic, data]); };
var bus = new Bus();
function sub(get) { return {get: get, set: function () {}}; }
var owner = {
  efisNdMode: 3, cursorVisible: {set: function () {}}, controlPanelVisible: sub(function () { return false; }),
  controlPanelRef: {instance: {activeDatabase: {set: function () {}}, store: {airports: {set: function () {}}},
    navigraphAvailable: {set: function () {}}, manualAirportSelection: false, activeTabIndex: {set: function () {}}}},
  oansRef: {instance: {dataLoading: false, dataAirportIcao: sub(function () { return ''; }), zoomLevelIndex: sub(function () { return 0; }),
    data: null, markerManager: {crosses: {items: function () { return []; }}, flags: {items: function () { return []; }}}}},
};
function state(fixture, pseudo) {
  return {ok: true, mode: 'native', values: {}, words: {}, numbers: {},
    nav: {positionValid: true, lat: fixture.lat, lon: fixture.lon, magVar: fixture.magVar, groundSpeed: fixture.groundSpeed,
          controls: [{mode: 3, range: 3, option: 0}, {mode: 3, range: 3, option: 0}],
          pointers: [{inside: 0, events: []}, {inside: 0, events: []}], route: fixture.route, symbols: [], airports: [],
          to: null, pseudo: pseudo}};
}
function update(st) { trial.apply(st); sent = []; nd.onUpdate(bus, owner); return sent.filter(function (m) { return m[0] === 'symbols'; }); }

var summary = [];
for (var f = 4; f < files.length; ++f) {
  var fixture = JSON.parse(readFile(files[f])), pseudo = fixture.pseudo, name = files[f].replace(/^.*\//, '');
  check(Array.isArray(pseudo), name + ': pseudo array');
  var symbolsMsgs = update(state(fixture, pseudo));
  check(symbolsMsgs.length === 1, name + ': one symbols message (route change)');
  check(!Object.prototype.hasOwnProperty.call(bus, 'send') && bus.send === Bus.prototype.send, name + ': bus.send restored');
  var list = symbolsMsgs[0][1], route = list.filter(function (s) { return /^fms-/.test(s.databaseId); });
  var pwps = list.filter(function (s) { return /^fmgc-pwp-/.test(s.databaseId); });
  check(route.length === fixture.route.length, name + ': route kept');
  check(pwps.length === pseudo.length, name + ': every pseudo waypoint sent');
  pwps.forEach(function (s, i) {
    var p = pseudo[i];
    check(bundlePwp(s) && !bundleWaypoint(s), name + ': ' + s.ident + ' drawn by the PseudoWaypointLayer only');
    check(s.typePwp === p.pwp && s.type === p.type && s.ident === p.ident, name + ': ' + s.ident + ' flags');
    if (typeof p.lat === 'number') check(s.location && s.location.lat === p.lat && s.location.long === p.lon, name + ': located ' + s.ident);
    else check(!s.location && s.distanceFromAirplane === p.dist, name + ': along-track ' + s.ident);
    if (typeof p.alt === 'number') check(s.predictedAltitude === p.alt, name + ': VD altitude ' + s.ident);
  });
  route.forEach(function (s) { check(!bundlePwp(s), name + ': route waypoints are not pseudo waypoints'); });
  // same state: nothing re-sent; the next capture is requested once for the new symbols
  check(update(state(fixture, pseudo)).length === 0, name + ': unchanged state is not re-sent');
  nd.needsSnapshot(); nd.needsSnapshot();
  // pseudo waypoints moved (same set): re-sent with the route, at most every 250 ms
  var moved = JSON.parse(JSON.stringify(pseudo));
  moved.forEach(function (p) { p.dist = Math.round((p.dist - 0.5) * 10) / 10; if (typeof p.lat === 'number') p.lat += 0.001; });
  if (moved.length) {
    clockMs += 100;
    check(update(state(fixture, moved)).length === 0, name + ': moved within 250 ms not re-sent yet');
    clockMs += 200;
    var again = update(state(fixture, moved));
    check(again.length === 1 && again[0][1].filter(function (s) { return /^fmgc-pwp-/.test(s.databaseId); })
      .every(function (s, i) { return s.distanceFromAirplane === (moved[i].dist > 0 ? moved[i].dist : undefined); }), name + ': moved re-sent');
    check(again[0][1].filter(function (s) { return /^fms-/.test(s.databaseId); }).length === route.length, name + ': re-sent with the route');
    check(nd.needsSnapshot() === true, name + ': capture after a re-send');
  }
  // a pseudo waypoint passed (set changed): re-sent immediately; none left: route only
  var fewer = moved.slice(1);
  clockMs += 10;
  var shrink = update(state(fixture, fewer));
  check(shrink.length === (moved.length ? 1 : 0), name + ': set change re-sent at once');
  if (shrink.length) check(shrink[0][1].filter(function (s) { return /^fmgc-pwp-/.test(s.databaseId); }).length === fewer.length, name + ': passed one dropped');
  clockMs += 1000;
  update(state(fixture, []));
  summary.push(name + ' ' + pwps.map(function (s) { return s.ident + '/' + colourOf(s.type) + (s.location ? '' : '@trk'); }).join(' '));
  // next fixture: a new route stamp
  clockMs += 1000;
}
// HDG/TRK: an along-track selected-mode level-off (no location): distanceFromAirplane only
var hdg = JSON.parse(readFile(files[4]));
hdg.route = hdg.route.slice(0, hdg.route.length - 1);
var trackSyms = update(state(hdg, [{ident: 'LEVEL', pwp: 16, type: 32768, dist: 12.5, alt: 10000}]));
check(trackSyms.length === 1, 'along-track state sent');
var lvl = trackSyms[0][1].filter(function (s) { return /^fmgc-pwp-/.test(s.databaseId); });
check(lvl.length === 1 && !lvl[0].location && lvl[0].distanceFromAirplane === 12.5 && lvl[0].typePwp === 16 &&
      colourOf(lvl[0].type) === 'cyan' && bundlePwp(lvl[0]), 'along-track level-off');
summary.push('HDG LEVEL/cyan@trk');
clockMs += 1000;
// a state without nav.pseudo (older plugin): route only, nothing breaks
var bare = JSON.parse(readFile(files[4]));
var noPseudo = state(bare, undefined); delete noPseudo.nav.pseudo; noPseudo.nav.route = bare.route.slice(1);
var msgs = update(noPseudo);
check(msgs.length === 1 && msgs[0][1].every(function (s) { return !/^fmgc-pwp-/.test(s.databaseId); }), 'no pseudo field');
print('PWP LAYER OK: ' + summary.join('; '));
