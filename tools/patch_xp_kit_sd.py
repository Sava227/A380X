#!/usr/bin/env python3
"""Deterministic live-data patch of the reused native ECAM SD bundle (sd.html).

The SD renderer is FlyByWire's own React SD pages plus the xp-kit "SDv2" FSComponent pages,
compiled into one minified sd.html. Its SimVar shim (`Ol`) only knows ~25 X-Plane datarefs, so
nearly every FBW variable is "missing" and the pages draw amber XX. The JS build workspace no
longer exists, so this tool makes small, anchored source edits to the verified bundle instead
(docs/SD_PAGES.md, "Bundle patch"):

1. prelude: helpers that read `frame.vars[<exact FBW name>]` (or its upper-case form) and decode
   the FBW raw ARINC 429 encoding (float32 bits + SSM * 2^32, fbw-sdk Arinc429Word);
2. `Ol` (GetSimVarValue / useSimVar shim): the exact FBW name published by the native side wins,
   the existing X-Plane mappings stay as the fallback;
3. `I` (useArinc429Var shim): a directly published word is decoded with its real SSM; the old
   "defined == normal operation" path remains for the legacy mappings;
4. `lf` (SDv2 bus bridge): every `ev:<topic>` frame var is sent on the FSComponent bus, after the
   legacy events; the F/CTL info-valid subjects (FBW: always true, xp-kit: false = "X") follow
   `ev:fctlInfoValid`; the SDv2 ARINC subject shim decodes raw words;
5. the native CRUISE page shows FU, FU TOTAL, LDG ELEVN and cabin zone temperatures from the
   FBW variables (FBW CruisePage rounding);
6. the xp-kit placeholder footer (amber "XX / AMBER: SYSTEM DATA INCOMPLETE" / "FULL FWS NOT
   AVAILABLE", which made every page look inoperative) is replaced by FBW's SDv2 PermanentData
   area (TAT/SAT/ISA, G LOAD, UTC clock, GW/GWCG/FOB) at FBW's coordinates.

Version 2 adds one edit on top (marker SLIDES_MARKER, applied to a v1-patched or an original
bundle): FBW's DOOR page passes "LIGHT BEACON ON" to every CabinDoor as a TODO proxy for "slide
armed". Each door now prefers the native per-door state "L:A380X_SLIDE_ARMED:<interactive point>"
published from the EFB (docs/EFB_DEVELOPMENT.md); without it the FBW proxy stays in effect.

Every anchor must occur exactly once in a known input; the result is marked so that a second
run is a byte-identical no-op. Unknown bundles are rejected, never patched heuristically.
"""
import argparse
import hashlib
from pathlib import Path

MARKER = '/*A380X-SD-LIVE-DATA v1*/'
SLIDES_MARKER = '/*A380X-SD-SLIDES v1*/'
VERSION = 2
# sd.html as shipped by the verified seven-view xp-kit build (sd-source-manifest bundle a80da77a).
KNOWN_INPUT_SHA256 = {
    '369149262b09e84929be720859b808404cb1e30eea109f85ac62c81b233e578c',
}
SIGNATURE = 'window.__a380Sd={apply:hc}'

PRELUDE = MARKER + (
    'var __a380U=new Uint32Array(1),__a380F=new Float32Array(__a380U.buffer);'
    # Exact FBW name (or upper-case variant) published by the native side, finite numbers only.
    'function __a380Raw(f,n){let v=f&&f.vars;if(!v||typeof n!="string")return;let r=v[n];'
    'if(typeof r!="number"||!Number.isFinite(r)){r=v[n.toUpperCase()];'
    'if(typeof r!="number"||!Number.isFinite(r))return}return r}'
    # fbw-sdk Arinc429Word semantics: value = float32(low 32 bits), ssm = bits 32..33.
    'function __a380Word(r){r=typeof r=="number"&&Number.isFinite(r)?r:0;'
    '__a380U[0]=(r&4294967295)>>>0;let v=__a380F[0],s=Math.trunc(r/4294967296)&3,k=s===3||s===2;'
    'return{value:v,ssm:s,rawWord:r,isNormalOperation:()=>s===3,isFailureWarning:()=>s===0,'
    'isNoComputedData:()=>s===1,isFunctionalTest:()=>s===2,isInvalid:()=>!k,valueOr:d=>k?v:d,'
    'bitValue:b=>((v>>(b-1))&1)!==0,bitValueOr:(b,d)=>k?((v>>(b-1))&1)!==0:d}}'
    # CRUISE helpers (FBW CruisePage: FU rounded to 50 kg, LDG ELEV to 50 ft, zone min..max).
    'function __a380Fu(f,i){let v=__a380Raw(f,"L:A32NX_FUEL_USED:"+(i+1));'
    'return v===void 0?void 0:Math.round(v/50)*50}'
    'function __a380FuTot(f){let t=0;for(let i=0;i<4;i++){let v=__a380Fu(f,i);if(v===void 0)return;t+=v}return t}'
    'function __a380Ldg(f){for(let n of["L:A32NX_FM1_LANDING_ELEVATION","L:A32NX_FM2_LANDING_ELEVATION"]){'
    'let r=__a380Raw(f,n);if(r!==void 0){let w=__a380Word(r);if(w.isNormalOperation())return Math.round(w.value/50)*50}}}'
    'function __a380Zone(f,p,c){let a,b;for(let i=1;i<=c;i++){let v=__a380Raw(f,p+i+"_TEMP");'
    'if(v===void 0)return"XX TO XX";a=a===void 0?v:Math.min(a,v);b=b===void 0?v:Math.max(b,v)}'
    'return Math.round(a)+" TO "+Math.round(b)}'
)

# FBW SDv2 PermanentData (StatusArea.tsx), same coordinates and classes; replaces the xp-kit
# placeholder footer (amber "SYSTEM DATA INCOMPLETE" / "FULL FWS NOT AVAILABLE" banner).
PERMANENT_DATA = (
    'function __a380W(f,n){let r=__a380Raw(f,n);return __a380Word(r===void 0?0:r)}'
    'function __a380Pfx(v){return v>=0?"+":""}'
    'function __a380Perm(e){let h=m.createElement,T=(x,y,c,...k)=>h("text",{x,y,className:c},...k),'
    'ok=w=>w.isNormalOperation(),'
    't=__a380W(e,"L:A32NX_ADIRS_ADR_1_TOTAL_AIR_TEMPERATURE"),s=__a380W(e,"L:A32NX_ADIRS_ADR_1_STATIC_AIR_TEMPERATURE"),'
    'z=__a380W(e,"L:A32NX_ADIRS_ADR_1_ALTITUDE"),fl=__a380W(e,"L:A32NX_FCU_EFIS_L_DISCRETE_WORD_2"),'
    'fr=__a380W(e,"L:A32NX_FCU_EFIS_R_DISCRETE_WORD_2"),g=__a380W(e,"L:A32NX_ADIRS_IR_1_BODY_NORMAL_ACC"),'
    'fq=__a380W(e,"L:A32NX_FQMS_TOTAL_FUEL_ON_BOARD"),gw=__a380W(e,"L:A32NX_FQMS_GROSS_WEIGHT"),'
    'cg=__a380W(e,"L:A32NX_FQMS_CENTER_OF_GRAVITY_MAC"),'
    'isa=s.valueOr(0)+Math.min(36089,z.valueOr(0))/500-15,'
    'isaV=(fl.bitValueOr(11,!0)||fr.bitValueOr(11,!0))&&ok(z)&&ok(s),'
    'gV=ok(g)&&(g.value<.7||g.value>1.4),zt=xe(e,"sim/time/zulu_time_sec"),'
    'p=v=>String(v).padStart(2,"0"),hh=zt===void 0?0:Math.floor(zt/3600),mi=zt===void 0?0:Math.floor(zt%3600/60),'
    'ss=zt===void 0?0:Math.floor(zt-hh*3600-mi*60),'
    'wt=(w,c)=>ok(w)?(Math.round(w.value/100)*100).toFixed(0):c&&w.isNoComputedData()?"--\\xa0\\xa0":"XX\\xa0\\xa0",'
    'wc=(w,c)=>"F27 "+(ok(w)?"Green":c&&w.isNoComputedData()?"Cyan":"Amber")+" EndAlign",'
    'tc=w=>"F25 EndAlign "+(ok(w)?"Green":"Amber"),tt=w=>ok(w)?__a380Pfx(w.value)+w.value.toFixed(0):"XX";'
    'return h("svg",{viewBox:"0 0 768 1024",className:"sd-overlay"},'
    'h("path",{className:"SW4 White StrokeRound",d:"M 7,667 l 754,0"}),h("path",{className:"SW4 White",d:"M 0,765 l 768,0"}),'
    'h("path",{className:"SW4 White StrokeRound",d:"M 257,667 l 0,92"}),h("path",{className:"SW4 White StrokeRound",d:"M 512,667 l 0,92"}),'
    'T(34,696,"F26 White LS1","TAT"),T(158,696,tc(t),tt(t)),T(185,696,"F26 Cyan","\\xB0C"),'
    'T(34,725,"F26 White LS1","SAT"),T(158,725,tc(s),tt(s)),T(185,725,"F26 Cyan","\\xB0C"),'
    'isaV&&h("g",null,T(34,754,"F26 White LS1","ISA"),T(158,754,"F25 Green EndAlign",__a380Pfx(isa)+isa.toFixed(0)),'
    'T(185,754,"F26 Cyan","\\xB0C")),'
    'gV&&h("g",null,T(296,702,"F27 Amber","G LOAD"),T(410,702,"F27 Amber",__a380Pfx(g.value)+g.value.toFixed(1))),'
    'zt!==void 0&&h("g",null,T(296,730,"F29 Green LS-1",p(hh)+":"+p(mi)+":"),T(394,730,"F26 Green",p(ss)),'
    'T(434,729,"F22 Green","GPS")),'
    'T(529,696,"F25 White","GW"),T(529,724,"F25 White","GWCG"),T(529,752,"F25 White","FOB"),'
    'T(705,696,wc(gw,1),wt(gw,1)),'
    'T(705,724,wc(cg,1),ok(cg)?cg.value.toFixed(1):cg.isNoComputedData()?"--\\xa0\\xa0":"XX\\xa0\\xa0"),'
    'T(705,752,wc(fq,0),wt(fq,0)),T(711,696,"F22 Cyan","KG"),T(711,724,"F22 Cyan","%"),T(711,752,"F22 Cyan","KG"))}'
)
# The complete xp-kit footer element inside u2 (its "XX / AMBER" text made every page look inoperative).
FOOTER = (
    'm.createElement("svg",{viewBox:"0 0 768 1024",className:"sd-overlay"},m.createElement("rect",{x:"0",y:"820",'
    'width:"768",height:"204",fill:"black"}),m.createElement("path",{d:"M0 864H768 M0 946H768 M254 864V946 M510 864V946",'
    'className:"White SW2"}),m.createElement("text",{x:"15",y:"848",className:"F20 Amber"},e.page===15?"":e.page===14?'
    '"FULL FWS NOT AVAILABLE":"XX / AMBER: SYSTEM DATA INCOMPLETE"),m.createElement("text",{x:"14",y:"895",'
    'className:"F23 White"},"TAT"),m.createElement("text",{x:"232",y:"895",className:"F25 EndAlign Green"},ls(l),'
    '"\\xB0C"),m.createElement("text",{x:"14",y:"931",className:"F23 White"},"SAT"),m.createElement("text",{x:"232",'
    'y:"931",className:"F25 EndAlign Green"},ls(r),"\\xB0C"),m.createElement("text",{x:"272",y:"895",'
    'className:"F23 White"},"FOB"),m.createElement("text",{x:"493",y:"931",className:"F25 EndAlign Green"},ls(o),"KG"),'
    'm.createElement("text",{x:"527",y:"895",className:"F23 White"},"GW"),m.createElement("text",{x:"749",y:"931",'
    'className:"F25 EndAlign Green"},ls(i),"KG"),m.createElement("text",{x:"16",y:"984",className:"F20 Cyan"},a2[e.page],'
    '" \\xB7 AIRCRAFT LIVE INPUTS"))'
)

EVENT_BRIDGE = (
    'var __a380FctlValid=te(!1);'
    'function __a380Ev(f){let v=f&&f.vars;if(!v)return;let g=v["ev:fctlInfoValid"];'
    '__a380FctlValid.set(g===!0||g===1);for(let k in v){if(k.slice(0,3)!=="ev:")continue;let x=v[k];'
    'if(typeof x=="boolean"||typeof x=="number"&&Number.isFinite(x))T0.send(k.slice(3),x,!1,!0)}}'
)

# (anchor, replacement, expected count) - applied in order on the original bundle.
EDITS = (
    ('function xe(e,t,n){', PRELUDE + 'function xe(e,t,n){', 1),
    ('function Ol(e,t){let n=e.toUpperCase(),',
     'function Ol(e,t){let __d=__a380Raw(t,e);if(__d!==void 0)return __d;let n=e.toUpperCase(),', 1),
    ('function I(e){let t=Fl(),n=(0,de.useContext)(mr),s=Ol(e,t),r=s!==void 0;return r||n?.missing.add(e),{value:s??0,',
     'function I(e){let t=Fl(),n=(0,de.useContext)(mr),__w=__a380Raw(t,e);if(__w!==void 0)return __a380Word(__w);'
     'let s=Ol(e,t),r=s!==void 0;return r||n?.missing.add(e),{value:s??0,', 1),
    ('s=r?.handle(l=>n.set(f0(l)))', 's=r?.handle(l=>n.set(__a380Word(l)))', 1),
    ('this.infoAvailable=te(!1)', 'this.infoAvailable=__a380FctlValid', 4),
    ('this.positionInfoValid=te(!1)', 'this.positionInfoValid=__a380FctlValid', 1),
    ('this.deflectionInfoValid=te(!1)', 'this.deflectionInfoValid=__a380FctlValid', 4),
    ('var T0=new Mp(', EVENT_BRIDGE + 'var T0=new Mp(', 1),
    ('function lf(e){nf.set(e.page===11),',
     'function lf(e){__a380LfBase(e),__a380Ev(e)}function __a380LfBase(e){nf.set(e.page===11),', 1),
    ('m.createElement("text",{x:n,y:"227",className:"F29 EndAlign Amber"},"XX")',
     'm.createElement("text",{x:n,y:"227",className:"F29 EndAlign "+(__a380Fu(e,s)===void 0?"Amber":"Green")},'
     'ls(__a380Fu(e,s)))', 1),
    ('m.createElement("text",{x:"383",y:"270",className:"F29 Amber MiddleAlign"},"XX")',
     'm.createElement("text",{x:"383",y:"270",className:(__a380FuTot(e)===void 0?"F29 Amber":"F29 Green")+'
     '" MiddleAlign"},ls(__a380FuTot(e)))', 1),
    ('m.createElement("text",{x:"653",y:"359",className:"F29 Amber EndAlign"},"XX")',
     'm.createElement("text",{x:"653",y:"359",className:(__a380Ldg(e)===void 0?"F29 Amber":"F29 Green")+'
     '" EndAlign"},ls(__a380Ldg(e)))', 1),
    ('m.createElement("text",{x:"175",y:"500",className:"F25 Amber"},"XX TO XX")',
     'm.createElement("text",{x:"175",y:"500",className:__a380Zone(e,"L:A32NX_COND_UPPER_DECK_",7)==="XX TO XX"'
     '?"F25 Amber":"F25 Green"},__a380Zone(e,"L:A32NX_COND_UPPER_DECK_",7))', 1),
    ('m.createElement("text",{x:"175",y:"575",className:"F25 Amber"},"XX TO XX")',
     'm.createElement("text",{x:"175",y:"575",className:__a380Zone(e,"L:A32NX_COND_MAIN_DECK_",8)==="XX TO XX"'
     '?"F25 Amber":"F25 Green"},__a380Zone(e,"L:A32NX_COND_MAIN_DECK_",8))', 1),
    (FOOTER, '__a380Perm(e)', 1),
    ('function u2(){', PERMANENT_DATA + 'function u2(){', 1),
)
# Version 2: per-door escape-slide state for the DOOR page (CabinDoor `IL`). `d` is the bundle's
# useSimVar shim, `Fl` its frame hook and `Ol` the patched name lookup (exact FBW / native names).
SLIDES_EDITS = (
    ('let[a]=d(`INTERACTIVE POINT OPEN:${s}`,"percent",1e3),u=a>20,E=!u&&o,',
     SLIDES_MARKER + 'let[a]=d(`INTERACTIVE POINT OPEN:${s}`,"percent",1e3),__sv=Ol(`L:A380X_SLIDE_ARMED:${s}`,Fl()),'
     'u=a>20,E=!u&&(__sv===void 0?o:__sv!==0),', 1),
)
SLIDES_ANCHORS = (SLIDES_MARKER, '__sv=Ol(`L:A380X_SLIDE_ARMED:${s}`,Fl())', 'E=!u&&(__sv===void 0?o:__sv!==0)')

# Strings the patched bundle must contain (anchor check usable without any JS engine).
PATCHED_ANCHORS = (
    MARKER, 'function __a380Raw(f,n){', 'function __a380Word(r){', 'let __d=__a380Raw(t,e);',
    '__w=__a380Raw(t,e);if(__w!==void 0)return __a380Word(__w);', 'function __a380Ev(f){',
    'function lf(e){__a380LfBase(e),__a380Ev(e)}', 'n.set(__a380Word(l))', 'var __a380FctlValid=te(!1);',
    'this.deflectionInfoValid=__a380FctlValid', 'ls(__a380FuTot(e))', 'function __a380Perm(e){',
    '__a380Perm(e))}var mf=',
    SIGNATURE, 'globalThis.SimVar={GetSimVarValue:e=>Ol(e,Gu)',
)


# Placeholder texts of the xp-kit footer that must be gone after patching.
FORBIDDEN = ('SYSTEM DATA INCOMPLETE', 'FULL FWS NOT AVAILABLE', 'AIRCRAFT LIVE INPUTS')


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def is_native_sd_bundle(data):
    """Only the aircraft-local native SD renderer is ever patched."""
    return SIGNATURE.encode() in data


def is_patched(data):
    return MARKER.encode() in data


def _patch_slides(text):
    """Version 2 edit on a v1-patched text; already-applied text is returned unchanged."""
    if SLIDES_MARKER in text:
        missing = [anchor for anchor in SLIDES_ANCHORS if anchor not in text]
        if missing or text.count(SLIDES_MARKER) != 1:
            raise ValueError('Partially patched SD slides (missing %s)' % (missing[0][:40] if missing else 'marker'))
        return text
    for anchor, _, count in SLIDES_EDITS:
        if text.count(anchor) != count:
            raise ValueError('SD slides anchor count %d != %d: %s' % (text.count(anchor), count, anchor[:60]))
    for anchor, replacement, _ in SLIDES_EDITS:
        text = text.replace(anchor, replacement)
    if any(anchor not in text for anchor in SLIDES_ANCHORS) or text.count(SLIDES_MARKER) != 1:
        raise ValueError('Patched SD slides failed its anchor check')
    return text


def patch_bundle(data):
    """Return the patched bytes of a known sd.html; a fully patched input is returned unchanged."""
    if not is_native_sd_bundle(data):
        raise ValueError('Not the native ECAM SD renderer bundle')
    if is_patched(data):
        text = data.decode('utf-8')
        missing = [anchor for anchor in PATCHED_ANCHORS if anchor not in text]
        if missing:
            raise ValueError('Partially patched SD bundle (missing %s)' % missing[0][:40])
        patched = _patch_slides(text).encode('utf-8')
        return data if patched == data else patched
    if sha256(data) not in KNOWN_INPUT_SHA256:
        raise ValueError('Unknown SD bundle %s; refusing to patch' % sha256(data))
    text = data.decode('utf-8')
    for anchor, _, count in EDITS:
        if text.count(anchor) != count:
            raise ValueError('SD bundle anchor count %d != %d: %s' % (text.count(anchor), count, anchor[:60]))
    for anchor, replacement, count in EDITS:
        text = text.replace(anchor, replacement)
    if '</script' in text[text.find(MARKER):text.find(MARKER) + len(PRELUDE)]:
        raise ValueError('Patch would terminate the inline script')
    missing = [anchor for anchor in PATCHED_ANCHORS if anchor not in text]
    if missing or text.count(MARKER) != 1 or any(word in text for word in FORBIDDEN):
        raise ValueError('Patched SD bundle failed its anchor check')
    return _patch_slides(text).encode('utf-8')


def patch_file(path):
    """Patch sd.html in place. Returns a manifest record, or None for a non-SD fixture."""
    path = Path(path)
    data = path.read_bytes()
    if not is_native_sd_bundle(data):
        return None
    patched = patch_bundle(data)
    if patched != data:
        path.write_bytes(patched)
    return {'tool': 'tools/patch_xp_kit_sd.py', 'version': VERSION, 'marker': MARKER,
            'input_sha256': sha256(data), 'output_sha256': sha256(patched),
            'doc': 'docs/SD_PAGES.md'}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('sd_html', type=Path)
    parser.add_argument('--check', action='store_true', help='Verify the anchors of an already patched bundle')
    args = parser.parse_args()
    data = args.sd_html.read_bytes()
    if args.check:
        text = data.decode('utf-8')
        missing = [anchor for anchor in (*PATCHED_ANCHORS, *SLIDES_ANCHORS) if anchor not in text]
        if missing:
            parser.error('missing anchors: %s' % missing)
        print('patched SD bundle anchors OK', sha256(data))
        return
    record = patch_file(args.sd_html)
    print(record)


if __name__ == '__main__':
    main()
