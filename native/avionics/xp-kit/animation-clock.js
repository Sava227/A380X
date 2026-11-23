// SPDX-License-Identifier: GPL-3.0-or-later
// A380X-PFD-ANIMATION-CLOCK v1
// CSS animations on the native frame clock (tools/patch_xp_kit_pfd.py, docs/ND_TERRAIN.md "Terrain alerts").
// The aircraft-local display helper renders the page in an offscreen window, where WebKit suspends the
// document timeline together with requestAnimationFrame (bridge.js supplies the frame clock for the
// latter). The FlyByWire PFD's warnings blink with CSS animations (Blink9Seconds: PULL UP, TERRAIN,
// SINK RATE, W/S AHEAD ...) whose first half-cycle is opacity 0; on a suspended timeline they stay
// there and the warning is never visible. On every native frame each running animation whose time
// did not move by itself is advanced by the frame interval (at most 250 ms); one that runs on its own
// (an ordinary browser) is left alone. Nothing of the page's content or styles is changed.
(() => {
  if (!window.__A380_NATIVE || typeof window.__a380NativeFrame !== 'function' ||
      typeof document.getAnimations !== 'function') return;
  const frame = window.__a380NativeFrame;
  const seen = new WeakMap();
  let last = null;
  const advance = (now) => {
    const dt = last === null ? 0 : Math.min(Math.max(now - last, 0), 250);
    last = now;
    if (dt <= 0) return;
    for (const animation of document.getAnimations()) {
      if (animation.playState !== 'running') continue;
      const time = Number(animation.currentTime) || 0;
      const previous = seen.get(animation);
      const next = previous !== undefined && time - previous < dt * 0.5 ? previous + dt : time;
      if (next !== time) animation.currentTime = next;
      seen.set(animation, next);
    }
  };
  window.__a380NativeFrame = () => {
    try { advance(performance.now()); } catch (error) { /* never break the frame clock */ }
    frame();
  };
  window.__a380AnimationClock = { advance };
})();
