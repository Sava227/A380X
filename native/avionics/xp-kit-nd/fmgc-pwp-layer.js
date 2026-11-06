// SPDX-License-Identifier: GPL-3.0-or-later
// A380X-ND-PWP v1 - FMGC pseudo waypoints (T/C, T/D, DECEL, level-off / start of climb or descent,
// speed limit, profile intercept) for the FlyByWire ND page (docs/AUTOPILOT_DEVELOPMENT.md).
// Loaded after native-bridge.js. In MSFS the FBW FMGC's EfisSymbols publishes the pseudo waypoints
// with the flight-plan symbols on the 'symbols' bus topic; the bundle's own PseudoWaypointLayer then
// draws them (typePwp symbol, NdSymbolTypeFlags colour, at their location or - without one - along the
// track at distanceFromAirplane; the VD uses distanceFromAirplane + predictedAltitude). Here the
// plugin sends them in state.nav.pseudo (native/avionics/a380x_xpkit_nd_data.inc); this layer appends
// them to every 'symbols' message of the native bridge and re-sends that message when only the
// pseudo waypoints changed (at most 4 Hz while their set stays the same).
(() => {
  const base = window.__a380xTrial, nd = window.__a380Nd;
  if (!base || !nd) return;
  const RESEND_MS = 250;
  let route = null, sentStamp = '', sentKey = '', sentAt = -Infinity, revision = 0, capturedRevision = 0;
  const finite = (value) => typeof value === 'number' && Number.isFinite(value);
  const pseudoOf = () => {
    const list = base.state?.nav?.pseudo;
    return Array.isArray(list) ? list : [];
  };
  function symbols(pseudo) {
    return pseudo.map((p, index) => {
      const symbol = { databaseId: 'fmgc-pwp-' + index, ident: String(p.ident || ''), type: p.type | 0, typePwp: p.pwp | 0 };
      if (finite(p.lat) && finite(p.lon)) symbol.location = { lat: p.lat, long: p.lon };
      if (finite(p.dist) && p.dist > 0) symbol.distanceFromAirplane = p.dist;
      if (finite(p.alt)) symbol.predictedAltitude = p.alt;
      return symbol;
    });
  }
  const keyOf = (pseudo) => pseudo.map((p) => [p.ident, p.pwp, p.type, finite(p.lat)].join(':')).join('|');
  const onUpdate = nd.onUpdate;
  nd.onUpdate = function (bus, owner) {
    const pseudo = pseudoOf(), stamp = JSON.stringify(pseudo), key = keyOf(pseudo);
    const own = Object.prototype.hasOwnProperty.call(bus, 'send'), send = bus.send;
    const mark = () => { sentStamp = stamp; sentKey = key; sentAt = performance.now(); revision++; };
    bus.send = function (topic, data, ...rest) {
      if (topic === 'symbols' && Array.isArray(data)) {
        route = data;
        data = data.concat(symbols(pseudo));
        mark();
      }
      return send.call(this, topic, data, ...rest);
    };
    try {
      return onUpdate.call(this, bus, owner);
    } finally {
      if (own) bus.send = send; else delete bus.send;
      // The bridge sends the route only when it changes: bring changed pseudo waypoints in on their own.
      if (route && stamp !== sentStamp && (key !== sentKey || performance.now() - sentAt >= RESEND_MS)) {
        mark();
        bus.send('symbols', route.concat(symbols(pseudo)));
      }
    }
  };
  const needsSnapshot = nd.needsSnapshot;
  nd.needsSnapshot = () => {
    const changed = needsSnapshot.call(nd), resent = revision !== capturedRevision;
    capturedRevision = revision;
    return changed || resent;
  };
  window.__a380NdPwp = {
    get revision() { return revision; },
    get sent() { return route ? symbols(pseudoOf()) : []; },
    symbols,
  };
})();
