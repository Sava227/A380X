"""SD bundle live-data patch: anchors, idempotence, reuse/install hash consistency, JS behaviour.

The installed aircraft bundle is only ever read. The JS checks use macOS JavaScriptCore (`jsc`)
when present; without any JS engine the anchor checks still run (the bundle has no node toolchain).
"""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

import patch_xp_kit_sd as patcher
from package_xp_kit_cockpit import reuse_verified_assets

ROOT = Path(__file__).resolve().parents[1]
INSTALLED = ROOT / 'output/A380X-XP12/plugins/A380XAvionics/mac_x64/xp-kit/sd.html'
JSC = Path('/System/Library/Frameworks/JavaScriptCore.framework/Versions/Current/Helpers/jsc')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def original_bundle():
    """The verified original bundle (the installed one, un-patched if it was already patched)."""
    if not INSTALLED.exists():
        raise unittest.SkipTest('installed SD bundle not present')
    data = INSTALLED.read_bytes()
    if sha(data) not in patcher.KNOWN_INPUT_SHA256:
        raise unittest.SkipTest('installed SD bundle is not the known original (already patched)')
    return data


class AnchorsWithoutJsEngine(unittest.TestCase):
    def test_patch_is_anchored_marked_and_idempotent(self):
        data = original_bundle()
        patched = patcher.patch_bundle(data)
        text = patched.decode()
        for anchor in patcher.PATCHED_ANCHORS:
            self.assertIn(anchor, text)
        for word in patcher.FORBIDDEN:
            self.assertNotIn(word, text)
        self.assertEqual(text.count(patcher.MARKER), 1)
        self.assertEqual(patcher.patch_bundle(patched), patched)          # byte-identical no-op
        self.assertIn('window.__a380Sd={apply:hc}', text)                 # native transport untouched
        self.assertEqual(text.count('<script>'), data.decode().count('<script>'))

    def test_unknown_or_partial_bundles_are_rejected(self):
        data = original_bundle()
        with self.assertRaisesRegex(ValueError, 'Unknown SD bundle'):
            patcher.patch_bundle(data.replace(b'NOT AVAIL', b'NOT AVAJL', 1))
        partial = patcher.MARKER.encode() + data
        with self.assertRaisesRegex(ValueError, 'Partially patched'):
            patcher.patch_bundle(partial)
        with self.assertRaisesRegex(ValueError, 'Not the native ECAM SD'):
            patcher.patch_bundle(b'<html>fixture</html>')

    def test_non_native_fixture_is_left_alone(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'sd.html'
            path.write_bytes(b'native view sd.html')
            self.assertIsNone(patcher.patch_file(path))
            self.assertEqual(path.read_bytes(), b'native view sd.html')


class ReusePathRecordsPatchedHashes(unittest.TestCase):
    """build.py --xpkit-assets -> reuse_verified_assets -> install_xp_kit_cockpit hash checks."""

    def build_installed(self, root, sd_bytes):
        source = root / 'installed' / 'xp-kit'
        source.mkdir(parents=True)
        files = {name: ('native view ' + name).encode() for name in ('pfd.html', 'nd.html', 'mfd.html')}
        files['sd.html'] = sd_bytes
        files['sd-source-manifest.json'] = json.dumps(
            {'native': True, 'scope': 'Native ECAM SD; read-only live values; no preview fixtures'}).encode()
        for name, data in files.items():
            (source / name).write_bytes(data)
        manifest = {'format': 1, 'browser_server_required': False,
                    'views': ['captain_pfd', 'first_officer_pfd', 'captain_nd', 'first_officer_nd',
                              'captain_mfd', 'first_officer_mfd', 'ecam_sd'],
                    'asset_hashes': {name: sha(data) for name, data in files.items()}}
        for name, key in (('pfd.html', 'html_sha256'), ('nd.html', 'nd_html_sha256'),
                          ('mfd.html', 'mfd_html_sha256'), ('sd.html', 'sd_html_sha256')):
            manifest[key] = manifest['asset_hashes'][name]
        (source.parent / 'xp-kit-build-report.json').write_text(json.dumps({'xp_kit_native_pfd': manifest}))
        return source

    def assert_install_consistent(self, staging, manifest):
        # The same per-asset check install_xp_kit_cockpit.py applies to the staged xp-kit directory.
        for name, expected in manifest['asset_hashes'].items():
            self.assertEqual(sha((staging / name).read_bytes()), expected, name)
        self.assertEqual(manifest['sd_html_sha256'], manifest['asset_hashes']['sd.html'])

    def test_reuse_patches_and_records_then_reuses_idempotently(self):
        data = original_bundle()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = self.build_installed(root, data)
            first = root / 'staging1' / 'xp-kit'
            manifest = reuse_verified_assets(source, first)
            patched = (first / 'sd.html').read_bytes()
            self.assertEqual(patched, patcher.patch_bundle(data))
            self.assertEqual(manifest['sd_live_data_patch']['input_sha256'], sha(data))
            self.assertEqual(manifest['sd_live_data_patch']['output_sha256'], sha(patched))
            self.assert_install_consistent(first, manifest)
            self.assertEqual(json.loads((first / 'manifest.json').read_text()), manifest)
            # After installing, the next native-only rebuild reuses the patched bundle unchanged.
            installed = root / 'installed2' / 'xp-kit'
            shutil.copytree(first, installed)
            (installed.parent / 'xp-kit-build-report.json').write_text(json.dumps({'xp_kit_native_pfd': manifest}))
            second = root / 'staging2' / 'xp-kit'
            again = reuse_verified_assets(installed, second)
            self.assertEqual((second / 'sd.html').read_bytes(), patched)
            self.assertEqual(again['sd_html_sha256'], manifest['sd_html_sha256'])
            self.assert_install_consistent(second, again)


def installed_bundle():
    """The installed bundle in whatever verified state it is (original or v1-patched)."""
    if not INSTALLED.exists():
        raise unittest.SkipTest('installed SD bundle not present')
    return INSTALLED.read_bytes()


class SlidesEdit(unittest.TestCase):
    """Version 2: per-door escape slides on the DOOR page, applied over the installed v1 bundle."""

    def test_slides_edit_is_anchored_marked_and_idempotent(self):
        data = installed_bundle()
        patched = patcher.patch_bundle(data)
        text = patched.decode()
        for anchor in (*patcher.PATCHED_ANCHORS, *patcher.SLIDES_ANCHORS):
            self.assertIn(anchor, text)
        self.assertEqual(text.count(patcher.SLIDES_MARKER), 1)
        self.assertEqual(text.count(patcher.MARKER), 1)
        self.assertEqual(patcher.patch_bundle(patched), patched)
        # FBW's beacon proxy remains the fallback when no native slide state is published.
        self.assertIn('E=!u&&(__sv===void 0?o:__sv!==0)', text)

    def test_partial_slides_patch_is_rejected(self):
        text = patcher.patch_bundle(installed_bundle()).decode()
        broken = text.replace('__sv=Ol(`L:A380X_SLIDE_ARMED:${s}`,Fl())', '__sv=0', 1)
        with self.assertRaisesRegex(ValueError, 'Partially patched SD slides'):
            patcher.patch_bundle(broken.encode())

    def test_slides_lookup_in_javascript(self):
        if not JSC.exists():
            self.skipTest('JavaScriptCore shell not available')
        text = patcher.patch_bundle(installed_bundle()).decode()
        source = text[text.index(patcher.MARKER):text.index('var Gu=')] + (
            '\nfunction check(c,m){if(!c)throw new Error(m)}'
            'check(Ol("L:A380X_SLIDE_ARMED:3",{vars:{}})===void 0,"absent keeps the FBW proxy");'
            'check(Ol("L:A380X_SLIDE_ARMED:3",{vars:{"L:A380X_SLIDE_ARMED:3":1}})===1,"armed");'
            'check(Ol("L:A380X_SLIDE_ARMED:3",{vars:{"L:A380X_SLIDE_ARMED:3":0}})===0,"disarmed");print("SLIDES OK")')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'case.js'
            path.write_text(source)
            result = subprocess.run([str(JSC), str(path)], capture_output=True, text=True, timeout=60)
            bundle = Path(directory) / 'bundle.js'
            bundle.write_text(text[text.index('<script>') + len('<script>'):text.rindex('</script>')])
            parse = Path(directory) / 'parse.js'
            parse.write_text('try{new Function(readFile(arguments[0]));print("PARSE OK")}catch(e){print("PARSE "+e)}')
            parsed = subprocess.run([str(JSC), str(parse), '--', str(bundle)], capture_output=True, text=True, timeout=60)
        self.assertIn('SLIDES OK', result.stdout + result.stderr)
        self.assertIn('PARSE OK', parsed.stdout)


JS_HARNESS = r'''
var te=v=>({v,set(x){this.v=x},get(){return this.v}});
var T0={sent:{},send(k,v){this.sent[k]=v}};
var m={createElement:(t,p,...c)=>({t,p,c})};
%s
function check(c,msg){if(!c)throw new Error("JS check failed: "+msg)}
var U=new Uint32Array(1),F=new Float32Array(U.buffer);
function raw(v,s){F[0]=v;return U[0]+s*4294967296}
var f={page:2,vars:{
 "L:A32NX_ELEC_AC_1_BUS_IS_POWERED":1,"L:A32NX_APU_N":raw(99.5,3),"L:A32NX_ENGINE_N3:1":60,
 "sim/cockpit2/engine/indicators/N2_percent":[60,61,62,63],
 "L:A32NX_FM1_LANDING_ELEVATION":raw(1234,3),"L:A32NX_FM2_LANDING_ELEVATION":raw(0,1),
 "L:A32NX_FUEL_USED:1":120,"L:A32NX_FUEL_USED:2":130,"L:A32NX_FUEL_USED:3":140,"L:A32NX_FUEL_USED:4":75,
 "ev:greenPressureSwitch":true,"ev:sec1RudderTrimPosition":raw(-3,3),"ev:fctlInfoValid":1,"ev:bad":"text",
 "L:A32NX_ADIRS_ADR_1_TOTAL_AIR_TEMPERATURE":raw(11,3),"L:A32NX_ADIRS_ADR_1_STATIC_AIR_TEMPERATURE":raw(-9,3),
 "L:A32NX_ADIRS_ADR_1_ALTITUDE":raw(5000,3),"L:A32NX_FCU_EFIS_L_DISCRETE_WORD_2":raw(1024,3),
 "L:A32NX_FQMS_GROSS_WEIGHT":raw(451234,3),"L:A32NX_FQMS_TOTAL_FUEL_ON_BOARD":raw(150049,3),
 "L:A32NX_FQMS_CENTER_OF_GRAVITY_MAC":raw(0,1),"sim/time/zulu_time_sec":45296}};
for(let i=1;i<=8;i++)f.vars["L:A32NX_COND_MAIN_DECK_"+i+"_TEMP"]=21+i/4;
check(Ol("L:A32NX_ELEC_AC_1_BUS_IS_POWERED",f)===1,"direct FBW name");
check(Ol("l:a32nx_elec_ac_1_bus_is_powered",f)===1,"upper-case variant");
check(Ol("L:A32NX_ENGINE_N2:2",f)===61,"legacy X-Plane mapping kept");
check(Ol("L:A32NX_UNKNOWN",f)===void 0,"absent stays absent");
var w=__a380Word(f.vars["L:A32NX_APU_N"]);check(w.isNormalOperation()&&Math.abs(w.value-99.5)<1e-6,"BNR decode");
var d=__a380Word(raw(2048,3));check(d.bitValueOr(12,!1)===!0&&d.bitValueOr(11,!0)===!1,"discrete bit 12");
var n=__a380Word(raw(5,1));check(!n.isNormalOperation()&&n.isNoComputedData()&&n.valueOr(-1)===-1,"NCD");
var t=__a380Word(raw(7,2));check(!t.isNormalOperation()&&t.valueOr(0)===7&&t.bitValueOr(1,!1)===!0,"FT valid for valueOr");
check(__a380Word(0).isFailureWarning(),"raw 0 is FW");
check(__a380Ldg(f)===1250,"LDG ELEV FM1 rounded to 50");
check(__a380Fu(f,0)===100&&__a380FuTot(f)===100+150+150+100,"FU rounding and total");
check(__a380Zone(f,"L:A32NX_COND_MAIN_DECK_",8)==="21 TO 23","main deck min..max");
check(__a380Zone(f,"L:A32NX_COND_UPPER_DECK_",7)==="XX TO XX","absent zones");
__a380Ev(f);
check(T0.sent.greenPressureSwitch===!0&&T0.sent.sec1RudderTrimPosition===raw(-3,3),"bus events");
check(!("bad" in T0.sent)&&__a380FctlValid.get()===!0,"only numbers/booleans; F/CTL info valid");
var texts=[];(function walk(e){if(!e||typeof e!="object")return;if(e.t==="text")texts.push(e.c.join(""));(e.c||[]).forEach(walk)})(__a380Perm(f));
var all=texts.join("|");
["TAT","+11","SAT","-9","ISA","-14","12:34:","56","GPS","GW","451200","GWCG","FOB","150000"].forEach(s=>check(texts.indexOf(s)>=0,"permanent data "+s+" in "+all));
check(texts.indexOf("G LOAD")<0,"no G LOAD without a valid out-of-range word");
check(all.indexOf("--")>=0&&all.indexOf("XX")<0,"NCD CG shows --, nothing XX");
print("SD JS OK");
'''


class PatchedJavaScript(unittest.TestCase):
    def setUp(self):
        if not JSC.exists():
            self.skipTest('JavaScriptCore shell not available')
        self.text = patcher.patch_bundle(original_bundle()).decode()
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)

    def run_js(self, source, *arguments):
        path = Path(self.temporary.name) / 'case.js'
        path.write_text(source)
        result = subprocess.run([str(JSC), str(path), '--', *arguments], capture_output=True, text=True, timeout=60)
        return result.stdout + result.stderr

    def test_whole_bundle_parses(self):
        start = self.text.index('<script>') + len('<script>')
        bundle = Path(self.temporary.name) / 'bundle.js'
        bundle.write_text(self.text[start:self.text.rindex('</script>')])
        output = self.run_js('try{new Function(readFile(arguments[0]));print("PARSE OK")}catch(e){print("PARSE "+e)}',
                             str(bundle))
        self.assertIn('PARSE OK', output)

    def test_helpers_behave_like_fbw(self):
        text = self.text
        parts = (text[text.index(patcher.MARKER):text.index('var Gu=')],
                 text[text.index('var __a380FctlValid='):text.index('var T0=new Mp(')],
                 text[text.index('function __a380W('):text.index('function u2(){')])
        output = self.run_js(JS_HARNESS % '\n'.join(parts))
        self.assertIn('SD JS OK', output, output)


if __name__ == '__main__':
    unittest.main()
