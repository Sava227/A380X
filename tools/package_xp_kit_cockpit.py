#!/usr/bin/env python3
"""Package the validated library PFD as a self-contained native WebKit view."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import shutil

from patch_xp_kit_sd import patch_file as patch_sd_html
from patch_xp_kit_nd import patch_file as patch_nd_html
from patch_xp_kit_pfd import patch_file as patch_pfd_html

ROOT = Path(__file__).resolve().parents[1]
EFB_SOURCE = ROOT / 'native/avionics/xp-kit-efb'
SEVEN_VIEWS = ['captain_pfd', 'first_officer_pfd', 'captain_nd', 'first_officer_nd',
               'captain_mfd', 'first_officer_mfd', 'ecam_sd']
NINE_VIEWS = [*SEVEN_VIEWS, 'captain_efb', 'first_officer_efb']
EFB_FONTS = ('Manrope-Regular.ttf', 'Manrope-Medium.ttf', 'Manrope-SemiBold.ttf', 'Manrope-Bold.ttf',
             'JetBrainsMono-Regular.ttf', 'OFL.txt')


def package_efb(destination):
    """The web EFB tablet page (views 7/8): one self-contained efb.html plus its OFL fonts.

    Built from the repository sources every time (never reused from an installed package), so
    the staged page always matches the native frame/action contract of the same build.
    """
    destination = Path(destination)
    template = (EFB_SOURCE / 'efb.html').read_text()
    css = (EFB_SOURCE / 'efb.css').read_text()
    script = (EFB_SOURCE / 'efb.js').read_text()
    if template.count('/*EFB_CSS*/') != 1 or template.count('/*EFB_JS*/') != 1:
        raise ValueError('EFB template placeholders missing')
    if '</script' in script or '</style' in css:
        raise ValueError('EFB sources must not terminate their inline element')
    for forbidden in ('http://', 'https://', 'fetch(', 'XMLHttpRequest', 'WebSocket', 'import('):
        if forbidden in script.replace('http://scripts.sil.org', '') or forbidden in css:
            raise ValueError('EFB page must stay offline: ' + forbidden)
    html = template.replace('/*EFB_CSS*/', css).replace('/*EFB_JS*/', script)
    (destination / 'efb.html').write_text(html)
    fonts = destination / 'Fonts/efb'
    fonts.mkdir(parents=True, exist_ok=True)
    for name in EFB_FONTS:
        with (EFB_SOURCE / 'Fonts/efb' / name).open('rb') as incoming, (fonts / name).open('wb') as outgoing:
            shutil.copyfileobj(incoming, outgoing, length=1024 * 1024)
    return hashlib.sha256(html.encode()).hexdigest()


def _with_efb(destination, manifest):
    manifest['views'] = list(NINE_VIEWS)
    manifest['efb_html_sha256'] = package_efb(destination)
    manifest['efb_render_resolution'] = [1440, 1000]
    manifest['asset_hashes'] = {str(p.relative_to(destination)): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in Path(destination).rglob('*')
                                if p.is_file() and p.name != 'manifest.json' and not p.name.startswith('._')}
    return manifest


def reuse_verified_assets(source, destination):
    """Reuse unchanged installed views, never rebuild them with demo fixtures.

    The selected installed build report is the authority for EVERY copied byte.
    This is useful for mechanical-only native plugin changes after a JS build
    workspace has been removed. It does not reuse the old native executable.
    """
    source, destination = Path(source).resolve(), Path(destination)
    report_path=source.parent/'xp-kit-build-report.json'
    previous=json.loads(report_path.read_text())['xp_kit_native_pfd']
    # Seven views (before the web EFB) or nine (EFB installed): the EFB page itself is never
    # reused, it is rebuilt from the repository below.
    if previous.get('views') not in (SEVEN_VIEWS,NINE_VIEWS) or previous.get('browser_server_required') is not False:
        raise ValueError('Installed view contract is not the complete native seven-view package (or the nine-view one with the web EFB)')
    hashes=previous.get('asset_hashes',{})
    if not hashes or not all(name in hashes for name in ('pfd.html','nd.html','mfd.html','sd.html','sd-source-manifest.json')):
        raise ValueError('Installed renderer has no complete asset hash manifest')
    verified=[]
    for name,expected in hashes.items():
        relative=Path(name)
        if name=='efb.html' or name.startswith('Fonts/efb/'):
            continue   # rebuilt from source by package_efb
        candidate=(source/relative).resolve()
        if relative.is_absolute() or '..' in relative.parts or not candidate.is_relative_to(source):
            raise ValueError('Unsafe renderer asset path: '+name)
        if hashlib.sha256(candidate.read_bytes()).hexdigest()!=expected:
            raise ValueError('Installed renderer asset hash mismatch: '+name)
        verified.append((name,candidate))
    sd=json.loads((source/'sd-source-manifest.json').read_text())
    if not sd.get('native') or 'no preview fixtures' not in sd.get('scope',''):
        raise ValueError('Only verified native ECAM views may be reused')
    for name,key in (('pfd.html','html_sha256'),('nd.html','nd_html_sha256'),
                     ('mfd.html','mfd_html_sha256'),('sd.html','sd_html_sha256')):
        if hashes[name]!=previous.get(key):raise ValueError('View hash/report mismatch: '+name)
    destination.mkdir(parents=True,exist_ok=False)
    for name,candidate in verified:
        target=destination/name;target.parent.mkdir(parents=True,exist_ok=True)
        # Avoid macOS fcopyfile/preallocation on the aircraft's ExFAT drive.
        with candidate.open('rb') as incoming,target.open('wb') as outgoing:
            shutil.copyfileobj(incoming,outgoing,length=1024*1024)
    manifest={k:v for k,v in previous.items() if k not in ('helper_sha256','architectures')}
    manifest['reused_assets_from']=str(source)
    manifest['reused_build_report_sha256']=hashlib.sha256(report_path.read_bytes()).hexdigest()
    # Live FBW SD data (docs/SD_PAGES.md): anchored, idempotent patch of the verified native SD
    # bundle; the recorded view/asset hashes describe the bytes actually staged.
    sd_patch=patch_sd_html(destination/'sd.html')
    if sd_patch:
        manifest['asset_hashes']=dict(manifest['asset_hashes'])
        manifest['asset_hashes']['sd.html']=manifest['sd_html_sha256']=sd_patch['output_sha256']
        manifest['sd_live_data_patch']=sd_patch
    # TERRAIN on ND (docs/ND_TERRAIN.md) + FMGC pseudo waypoints: the repository's ND layers, inserted
    # after the native bridge of the verified ND page (or brought up to date in an already patched one).
    nd_patch=patch_nd_html(destination/'nd.html')
    if nd_patch:
        manifest['asset_hashes']=dict(manifest['asset_hashes'])
        manifest['asset_hashes']['nd.html']=manifest['nd_html_sha256']=nd_patch['output_sha256']
        manifest['nd_terrain_patch']=nd_patch
    # PFD CSS animations (FBW blinking warnings: PULL UP, TERRAIN ...) on the native frame clock
    # (docs/ND_TERRAIN.md "Terrain alerts"): the repository's clock after the bridge of the verified page.
    pfd_patch=patch_pfd_html(destination/'pfd.html')
    if pfd_patch:
        manifest['asset_hashes']=dict(manifest['asset_hashes'])
        manifest['asset_hashes']['pfd.html']=manifest['html_sha256']=pfd_patch['output_sha256']
        manifest['pfd_animation_clock_patch']=pfd_patch
    _with_efb(destination,manifest)
    (destination/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


def package(destination):
    source = ROOT / 'native/avionics/xp-kit'
    build = ROOT / 'build/xp-kit-pfd-test'
    destination.mkdir(parents=True, exist_ok=True)
    css = (build / 'PFD/bundle.css').read_text()
    for relative, mime in [('Fonts/fbw-a380x/FBW-Display-EIS-A380.ttf', 'font/ttf'),
                           ('Images/fbw-a380x/Common/thalesTest.svg', 'image/svg+xml'),
                           ('Images/fbw-a380x/TRIM_INDICATOR.png', 'image/png')]:
        uri = 'data:' + mime + ';base64,' + base64.b64encode((build / relative).read_bytes()).decode()
        css = css.replace('/' + relative, uri)
    bundle = (build / 'PFD/bundle.js').read_text()
    image = 'data:image/png;base64,' + base64.b64encode((build / 'Images/fbw-a380x/TRIM_INDICATOR.png').read_bytes()).decode()
    bundle = bundle.replace('/Images/fbw-a380x/TRIM_INDICATOR.png', image)
    runtime = (build / 'runtime/msfs-runtime.js').read_text()
    loader = "const RUNTIME_FILES = ['msfs/coherent.js', 'msfs/simvar.js', 'msfs/environment.js', 'msfs/instrument.js'];"
    if runtime.count(loader) != 1:
        raise ValueError('Native runtime must have exactly one known child-script loader')
    runtime = runtime.replace(loader, 'const RUNTIME_FILES = [];')
    scripts = ['window.__A380_NATIVE=true; window.__a380PfdSource=' + json.dumps(bundle) + ';',
               (source / 'bridge.js').read_text(), (source / 'animation-clock.js').read_text(), runtime]
    scripts.extend((build / 'runtime/msfs' / f).read_text() for f in ('coherent.js', 'simvar.js', 'environment.js', 'instrument.js'))
    html = '<!doctype html><html><head><meta charset="utf-8"><title>A380X cockpit PFD</title><style>' + css + '\nhtml,body{margin:0;width:768px;height:1024px;background:#000}#PFD_CONTENT{width:768px;height:1024px}</style>'
    html += ''.join('<script>' + script.replace('</script', '<\\/script') + '</script>' for script in scripts)
    html += '<script type="text/html" id="A380X_PFD"><div id="Electricity"><div id="PFD_CONTENT"><h1>PFD loading</h1></div></div></script>'
    html += '<script type="text/html" import-script="/PFD/bundle.js"></script></head><body></body></html>'
    (destination / 'pfd.html').write_text(html)
    shutil.copy2(build / 'library/LICENSE', destination / 'LICENSE')
    shutil.copy2(build / 'trial-manifest.json', destination / 'source-manifest.json')
    manifest = {'format': 1, 'renderer': 'macOS WebKit', 'view': 'FlyByWire A380X PFD / xp-kit',
                'transport': 'aircraft-local shared memory', 'browser_server_required': False,
                'html_sha256': hashlib.sha256(html.encode()).hexdigest()}
    package_nd(destination)
    package_mfd(destination)
    package_sd(destination)
    manifest['views']=list(SEVEN_VIEWS)
    manifest['nd_html_sha256']=hashlib.sha256((destination/'nd.html').read_bytes()).hexdigest()
    nd_metadata=json.loads((destination/'nd-source-manifest.json').read_text())
    manifest['nd_render_resolution']=nd_metadata.get('render_resolution',[768,1024])
    for key in ('oans_zoom_levels_nm','oans_default_zoom_nm'):
        if key in nd_metadata:manifest[key]=nd_metadata[key]
    manifest['mfd_html_sha256']=hashlib.sha256((destination/'mfd.html').read_bytes()).hexdigest()
    manifest['sd_html_sha256']=hashlib.sha256((destination/'sd.html').read_bytes()).hexdigest()
    _with_efb(destination, manifest)
    (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


def package_nd(destination):
    source=ROOT/'native/avionics/xp-kit-nd'
    build=ROOT/'build/xp-kit-nd-native'
    metadata=json.loads((build/'manifest.json').read_text())
    if 'Native aircraft' not in metadata['scope']:
        raise ValueError('Only a native, non-demo ND build can be installed')
    bundle=(build/'ND/bundle.js').read_text().replace('/Images/fbw-a380x/','Images/fbw-a380x/')
    if '__ndDemo' in bundle or '/airports?q=' in bundle:
        raise ValueError('Demo ND code must never enter the aircraft package')
    css=(build/'ND/bundle.css').read_text().replace('/Images/fbw-a380x/','Images/fbw-a380x/').replace('/Fonts/fbw-a380x/','Fonts/fbw-a380x/')
    runtime=(build/'runtime/msfs-runtime.js').read_text()
    loader="const RUNTIME_FILES = ['msfs/coherent.js', 'msfs/simvar.js', 'msfs/environment.js', 'msfs/instrument.js'];"
    if runtime.count(loader)!=1: raise ValueError('ND runtime loader mismatch')
    runtime=runtime.replace(loader,'const RUNTIME_FILES = [];')
    scripts=['window.__A380_NATIVE=true; window.__a380NdSource='+json.dumps(bundle)+';',
             (ROOT/'native/avionics/xp-kit/bridge.js').read_text(),(source/'native-bridge.js').read_text(),
             (source/'terrain-layer.js').read_text(),   # TERRAIN on ND (docs/ND_TERRAIN.md)
             (source/'fmgc-pwp-layer.js').read_text(),runtime]   # FMGC pseudo waypoints (tools/patch_xp_kit_nd.py LAYERS)
    scripts.extend((build/'runtime/msfs'/name).read_text() for name in ('coherent.js','simvar.js','environment.js','instrument.js'))
    html='<!doctype html><html><head><meta charset="utf-8"><title>A380X cockpit ND/OANS</title>'
    html+='<meta http-equiv="Content-Security-Policy" content="default-src \'self\' data: file:; script-src \'self\' \'unsafe-inline\' \'unsafe-eval\'; style-src \'self\' \'unsafe-inline\'; connect-src \'none\'">'
    html+='<style>'+css+'\nhtml,body{margin:0;width:768px;height:1024px;background:#000;overflow:hidden}#ND_CONTENT{width:768px;height:1024px}</style>'
    html+=''.join('<script>'+script.replace('</script','<\\/script')+'</script>' for script in scripts)
    html+='<script type="text/html" id="A380X_ND"><div id="Electricity"><div id="ND_CONTENT"><h1>ND loading</h1></div></div></script>'
    html+='<script type="text/html" import-script="/ND/bundle.js"></script></head><body></body></html>'
    (destination/'nd.html').write_text(html)
    for directory in ('Images','Fonts'):
        shutil.copytree(build/directory,destination/directory,dirs_exist_ok=True)
    shutil.copy2(source/'airport_worker.py',destination/'airport_worker.py')
    shutil.copy2(ROOT/'tools/nd_airport_preview.py',destination/'nd_airport_preview.py')
    shutil.copy2(build/'manifest.json',destination/'nd-source-manifest.json')


def refresh_nd(destination):
    """Update ND-only assets while retaining byte-verified PFD/MFD/SD views."""
    destination=Path(destination)
    manifest=json.loads((destination/'manifest.json').read_text())
    for name,expected in manifest['asset_hashes'].items():
        relative=Path(name)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Unsafe renderer asset path: '+name)
        if hashlib.sha256((destination/relative).read_bytes()).hexdigest()!=expected:
            raise ValueError('Cannot refresh modified renderer asset: '+name)
    protected={name:(destination/name).read_bytes() for name in ('pfd.html','mfd.html','sd.html')}
    package_nd(destination)
    for name,data in protected.items():
        if (destination/name).read_bytes()!=data: raise ValueError('ND refresh modified '+name)
    manifest['nd_html_sha256']=hashlib.sha256((destination/'nd.html').read_bytes()).hexdigest()
    manifest['nd_render_resolution']=[768,1024]
    nd_metadata=json.loads((destination/'nd-source-manifest.json').read_text())
    for key in ('oans_zoom_levels_nm','oans_default_zoom_nm'):
        if key in nd_metadata:manifest[key]=nd_metadata[key]
    manifest['asset_hashes']={str(p.relative_to(destination)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in destination.rglob('*') if p.is_file() and p.name!='manifest.json' and not p.name.startswith('._')}
    (destination/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')


def package_mfd(destination):
    build=ROOT/'build/xp-kit-mfd-native'
    metadata=json.loads((build/'manifest.json').read_text())
    if metadata.get('transport_version')!=3:
        raise ValueError('MFD renderer transport does not match the native runtime')
    bundle=(build/'bundle.js').read_text()
    if hashlib.sha256(bundle.encode()).hexdigest()!=metadata['bundle_sha256']:
        raise ValueError('MFD renderer bundle hash mismatch')
    html='<!doctype html><html><head><meta charset="utf-8"><title>A380X native MFD</title>'
    html+='<meta http-equiv="Content-Security-Policy" content="default-src \'self\' data: file:; script-src \'unsafe-inline\'; style-src \'unsafe-inline\'; connect-src \'none\'">'
    html+='<style>@font-face{font-family:A380EIS;src:url(Fonts/fbw-a380x/FBW-Display-EIS-A380.ttf)}'
    html+='@font-face{font-family:A380Slashed;src:url(Fonts/fbw-a380x/FBW-Display-EIS-A380-SlashedZero.ttf)}'
    html+='html,body{margin:0;width:768px;height:1024px;background:#000;overflow:hidden}canvas{display:block}</style></head><body>'
    html+='<script>const report=(text)=>window.webkit?.messageHandlers.a380.postMessage({type:"a380x-error",text:String(text)});window.addEventListener("error",e=>report(e.message));window.addEventListener("unhandledrejection",e=>report(e.reason));</script>'
    html+='<script>'+bundle.replace('</script','<\\/script')+'</script></body></html>'
    (destination/'mfd.html').write_text(html)
    shutil.copy2(build/'manifest.json',destination/'mfd-source-manifest.json')


def package_sd(destination):
    import os
    build=Path(os.environ.get('ECAM_NATIVE_BUILD',str(ROOT/'build/ecam-native')))
    metadata=json.loads((build/'manifest.json').read_text())
    if not metadata.get('native') or 'no preview fixtures' not in metadata['scope']:
        raise ValueError('Native ECAM renderer must not contain preview fixtures')
    bundle=(build/'app.js').read_text()
    if hashlib.sha256(bundle.encode()).hexdigest()!=metadata['bundle_sha256']:
        raise ValueError('Native ECAM renderer hash mismatch')
    bundle=bundle.replace('/Images/fbw-a380x/','Images/fbw-a380x/')
    css=(build/'display.css').read_text().replace('/Fonts/fbw-a380x/','Fonts/fbw-a380x/')
    css+='\nhtml,body{margin:0;width:768px;height:1024px;background:black;overflow:hidden}#native-sd,.sd-page,.sd-overlay,.v2-pages,.v2-pages>svg{position:absolute;inset:0;width:768px;height:1024px}text{font-family:Ecam,monospace}.sd-overlay{pointer-events:none}'
    html='<!doctype html><html><head><meta charset="utf-8"><title>A380X native ECAM SD</title>'
    html+='<meta http-equiv="Content-Security-Policy" content="default-src \'self\' data: file:; script-src \'unsafe-inline\'; style-src \'unsafe-inline\'; connect-src \'none\'">'
    html+='<style>'+css+'</style></head><body><script>'+bundle.replace('</script','<\\/script')+'</script></body></html>'
    (destination/'sd.html').write_text(html)
    shutil.copytree(build/'Images',destination/'Images',dirs_exist_ok=True)
    shutil.copytree(build/'Fonts',destination/'Fonts',dirs_exist_ok=True)
    shutil.copy2(build/'manifest.json',destination/'sd-source-manifest.json')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--nd-only',action='store_true',help='Refresh ND assets only, preserving verified PFD/MFD/SD views')
    parser.add_argument('--refresh-report',type=Path,help='Refresh an existing staging build after asset-only changes')
    args=parser.parse_args()
    if args.refresh_report:
        report=json.loads(args.refresh_report.read_text())
        hashes={row['name']:row['sha256'] for row in report['sources']}
        for name in ('a380x_avionics.c','a380x_mfd_adapter.inc','a380x_xpkit_shared.h','a380x_xpkit_adapter.inc','a380x_xpkit_nd_input.inc','a380x_xpkit_nd_data.inc','a380x_xpkit_mfd.inc','a380x_xpkit_host.m'):
            if hashlib.sha256((ROOT/'native/avionics'/name).read_bytes()).hexdigest()!=hashes.get(name):
                parser.error('Native source changed; a full rebuild is required: '+name)
        previous=report['xp_kit_native_pfd']
        if hashlib.sha256((args.refresh_report.parent/'A380XDisplayHost').read_bytes()).hexdigest()!=previous['helper_sha256']:
            parser.error('Renderer helper differs from the verified build')
    if args.nd_only:refresh_nd(args.destination)
    else:package(args.destination)
    if args.refresh_report:
        manifest=json.loads((args.destination/'manifest.json').read_text())
        manifest.update({key:previous[key] for key in ('helper_sha256','architectures')})
        report['xp_kit_native_pfd']=manifest
        args.refresh_report.write_text(json.dumps(report,indent=2)+'\n')
