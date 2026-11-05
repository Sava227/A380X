/* SPDX-License-Identifier: GPL-3.0-or-later
 * JavaScriptCore check of the web EFB page logic (native/avionics/xp-kit-efb/efb.js), driven by
 * real native frames from test_efb_adapter --frame. Run: jsc efb_page_check.js -- <efb.js> <frames.json>
 * No DOM: the page renders to an HTML string; actions are captured from the WebKit message bridge.
 */
var posted = [];
this.webkit = {messageHandlers: {a380: {postMessage: function (m) { posted.push(m); }}}};
load(arguments[0]);
var frames = JSON.parse(readFile(arguments[1]));
var core = __a380EfbCore, efb = __a380Efb;
function check(c, message) { if (!c) throw new Error('EFB page check failed: ' + message); }
function attrs(html) {
  var keys = [], re = /data-a="([^"]*)"/g, m;
  while ((m = re.exec(html))) keys.push(m[1].replace(/&amp;/g, '&'));
  return keys;
}
function text(html) { return html.replace(/<[^>]*>/g, ' ').replace(/&[a-z#0-9]+;/g, ' '); }
var allKeys = {}, rendered = 0;
var expected = {
  'home': ['Dashboard', 'of 20 doors open', 'of 16 slides armed', 'Next checklist', 'Quick presets', 'OFP IMPORT PLANNED'],
  'ofp': ['SimBrief OFP', 'PLANNED · NOT ACTIVE YET', 'No OFP loaded'],
  'ground/doors': ['Doors &amp; slides', 'MAIN DECK', 'UPPER DECK', 'M1L', 'U3R', 'BULK', 'Cockpit', 'NO SLIDE', 'ARMED', 'Arm all slides'],
  'ground/services': ['Ground power (EXT)', 'Wheel chocks', 'NOT MODELLED'],
  'ground/push': ['Pushback', 'Nose left', 'Before pushback', '14 / 16', '3 OPEN'],
  'loading/payload': ['Economy', 'Forward hold', 'Load payload', 'Planned ZFW'],
  'loading/fuel': ['LEFT WING', 'FEED 2', 'TRIM', 'Start refuel'],
  'loading/wb': ['Weight &amp; balance arrives in the next update'],
  'cabin': ['Cabin layout', 'PREVIEW · applies in a later update', 'Fill rest economy'],
  'flight/perf': ['Top of descent', 'Take-off performance', 'NEXT UPDATE'],
  'flight/nav': ['Present position', 'X-Plane map', 'Chart viewer'],
  'flight/comms': ['COM 1', 'COM 2', 'Transponder', 'IDENT', '121.900'],
  'ops/checklists': ['BEFORE START', 'Check item', 'AUTO'],
  'ops/presets': ['Ready for pushback', 'Status'],
  'failures': ['Failures', 'GENERATOR 2', 'Active', 'Clear all'],
  'aircraft': ['Aircraft options', 'NO EFFECT YET', 'Engine Alliance GP7270'],
  'settings/general': ['Units &amp; time', 'Screen brightness', 'Simulate tablet battery'],
  'settings/sounds': ['Master', 'Warnings', 'Restore values from aircraft load']
};
Object.keys(frames).forEach(function (name) {
  var frame = frames[name];
  check(efb.apply(frame, true) === true || rendered > 0, name + ': first apply must request a capture');
  var html = core.view(), plain = text(html);
  check(html.indexOf('id="screen"') >= 0, name + ': page root');
  check(!/\bundefined\b|\bNaN\b|\[object/.test(plain), name + ': no undefined / NaN text: ' + plain.slice(0, 400));
  (expected[name] || []).forEach(function (s) { check(html.indexOf(s) >= 0, name + ': expected "' + s + '"'); });
  attrs(html).forEach(function (k) { allKeys[k.split(' ')[0]] = true; });
  ++rendered;
});
check(rendered === Object.keys(expected).length, 'every section and tab rendered (' + rendered + ')');

/* Unavailable values are shown as such, never invented. */
var home = frames['home'];
efb.apply(home, true);
check(core.view().indexOf('lbl">CG</div><div class="val na">—') >= 0, 'CG unavailable on Home');
check(core.view().indexOf('needs OFP trip fuel') >= 0, 'LW needs the OFP');

/* Keypad: typed payload, COM entry with a decimal point, cancel. */
posted.length = 0;
efb.apply(frames['loading/payload'], true);
core.activate('~kp pax 3');
check(core.keypad() && core.keypad().title === 'Economy passengers', 'keypad opens for economy');
check(core.view().indexOf('class="keypad"') >= 0 && core.view().indexOf('0 – 338 seats') >= 0, 'keypad overlay with limits');
['3', '0', '1', '9'].forEach(function (k) { core.activate('~kp key ' + k); });   /* 4th digit refused (max 3) */
core.activate('~kp key back'); core.activate('~kp key 2'); core.activate('~kp key enter');
check(posted.length === 1 && posted[0].type === 'a380x-efb-action' && posted[0].action === 'pax 3 302', 'typed passengers: ' + JSON.stringify(posted));
check(core.keypad() === null, 'keypad closes on Enter');
core.activate('~kp com1');
['1', '2', '1', '.', '9', '.'].forEach(function (k) { core.activate('~kp key ' + k); });
core.activate('~kp key enter');
check(posted[1].action === 'radio stby1 121.9', 'COM entry keeps one decimal point: ' + posted[1].action);
core.activate('~kp cargo 1'); core.activate('~kp key 5'); core.activate('~kp key cancel');
check(posted.length === 2 && core.keypad() === null, 'cancel posts nothing');
core.activate('~kp squawk'); core.activate('~kp key .');
check(core.keypad().text === '', 'no decimal point for a squawk');
core.activate('~kp key enter');
check(posted.length === 2, 'empty entry posts nothing');
check(posted[1].id > posted[0].id, 'action ids increase');

/* Optimistic navigation until the native frame acknowledges it. */
core.activate('nav failures');
check(core.view().indexOf('<h1>Failures</h1>') >= 0, 'optimistic section change');
var ack = JSON.parse(JSON.stringify(frames['loading/payload']));
efb.apply(ack, true);
check(core.view().indexOf('<h1>Failures</h1>') >= 0, 'pending until acknowledged');
ack.ack = posted[posted.length - 1].id; ack.ui.section = 'failures';
efb.apply(ack, true);
check(core.local.optimistic === null && core.view().indexOf('<h1>Failures</h1>') >= 0, 'acknowledged');

/* Door / slide mode, local filters, units, toast, blank and stale screens. */
efb.apply(frames['ground/doors'], true);
core.activate('~doormode slides');
var doors = core.view();
check(doors.indexOf('Tap a door to arm or disarm its slide') >= 0 && doors.indexOf('data-a="slide M2L 0"') >= 0 &&
      doors.indexOf('data-a="slide M1L 1"') >= 0, 'slides mode targets');
core.activate('~doormode doors');
efb.apply(frames['failures'], true);
core.activate('~filter ELEC');
check(core.view().indexOf('ENGINE 1') < 0 && core.view().indexOf('GENERATOR 1') >= 0, 'failure filter');
var lb = JSON.parse(JSON.stringify(frames['home'])); lb.settings.metric = false;
efb.apply(lb, true);
check(core.view().indexOf(' klb') >= 0 && core.view().indexOf(' t<') < 0, 'imperial masses');
var toast = JSON.parse(JSON.stringify(frames['home'])); toast.status = 'Disarm the M1L slide before opening the door';
efb.apply(toast, true);
check(core.view().indexOf('class="toast">Disarm the M1L slide') >= 0, 'native status toast');
var blank = JSON.parse(JSON.stringify(frames['home'])); blank.power.blank = true;
efb.apply(blank, true);
check(core.view() === '<div class="offscreen"></div>', 'blank tablet');
efb.apply(frames['home'], false);
check(core.view().indexOf('No data from the aircraft') >= 0 && core.view().indexOf('NO DATA') >= 0, 'stale input');
efb.apply(frames['home'], true);
check(efb.idle() === false, 'idle page needs no capture');

/* Clock text and every action verb the page can post (checked against the native grammar). */
check(/^\d\d:\d\dZ$/.test(core.clockText()), 'UTC clock ' + core.clockText());
print('VERBS ' + Object.keys(allKeys).sort().join(' '));
print('EFB PAGE OK ' + rendered + ' views');
