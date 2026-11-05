#!/usr/bin/env python3
"""Install a verified xp-kit Mac build in this project's aircraft."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from nd_cockpit_devices import augment, ND_DEVICES, ND_COMMANDS, ND_DATAREFS
from pfd_cockpit_devices import augment as pfd_augment, PFD_DEVICES, PFD_COMMANDS, PFD_DATAREFS
from mfd_cockpit_devices import refresh_touch_geometry
from avionics_provider import MANIFEST, FILES, AVIONICS_DATAREFS, AVIONICS_COMMANDS

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def copy_file(source, destination):
    """Atomic streamed copy; avoid macOS fcopyfile preallocation on this drive."""
    source, destination = Path(source), Path(destination)
    if destination.exists() and sha(source) == sha(destination):
        return str(destination)
    fd, name = tempfile.mkstemp(prefix='.a380x-install-', dir=destination.parent)
    temporary = Path(name)
    try:
        with source.open('rb') as incoming, os.fdopen(fd, 'wb') as outgoing:
            shutil.copyfileobj(incoming, outgoing, length=1024 * 1024)
            outgoing.flush()
            os.fsync(outgoing.fileno())
        shutil.copystat(source, temporary)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)
    return str(destination)


def write_text(path, text):
    # Re-installing a plugin must not rewrite the large, unchanged cockpit OBJ
    # files, especially when the external filesystem has recently stalled.
    if path.exists() and path.read_text() == text:
        return
    fd, name = tempfile.mkstemp(prefix='.a380x-install-', dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, 'w') as output:
            output.write(text)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def provider_document(aircraft, sd, isis, efb, file_sha):
    """The avionics provider manifest exactly as an installation writes it (file_sha(packaged) -> hash)."""
    provider=json.loads((aircraft/MANIFEST).read_text())
    provider['datarefs']=list(AVIONICS_DATAREFS)
    provider['commands']=list(AVIONICS_COMMANDS)
    provider['xpkit_nd']={'cockpit_devices':list(ND_DEVICES),'commands':list(ND_COMMANDS),'datarefs':list(ND_DATAREFS)}
    provider['xpkit_pfd']={'cockpit_devices':list(PFD_DEVICES),'commands':list(PFD_COMMANDS),'datarefs':list(PFD_DATAREFS)}
    if sd:
        provider['ecam_sd']=sd
    if isis:
        from isis_cockpit_device import ISIS_COMMANDS,ISIS_DATAREFS,ISIS_DEVICES
        provider['isis']={'cockpit_devices':list(ISIS_DEVICES),'commands':list(ISIS_COMMANDS),'datarefs':list(ISIS_DATAREFS)}
    if efb:
        from efb_cockpit_devices import EFB_DEVICES,EFB_COMMANDS,EFB_DATAREFS
        provider['efb_devices']={'cockpit_devices':list(EFB_DEVICES),'commands':list(EFB_COMMANDS),'datarefs':list(EFB_DATAREFS)}
    provider['files']=[{'path':packaged,'sha256':file_sha(packaged)} for _,packaged in FILES]
    return json.dumps(provider,indent=2)+'\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('staging', type=Path)
    parser.add_argument('--no-backup', action='store_true', help='Install directly without creating a backup')
    parser.add_argument('--check-only', action='store_true', help='Validate all installation inputs without writing or requiring X-Plane to quit')
    parser.add_argument('--stage-dir', type=Path, help='With --check-only: also write the cockpit objects and provider manifest an '
                        'installation would write into this new directory (aircraft-relative paths), for validation and hashes')
    args = parser.parse_args()
    if args.stage_dir and not args.check_only:
        parser.error('--stage-dir requires --check-only')
    if not args.check_only and subprocess.run(['pgrep', '-x', 'X-Plane'], capture_output=True).returncode == 0:
        parser.error('Quit X-Plane before replacing its aircraft plugin')
    staging = args.staging.resolve()
    report = json.loads((staging / 'build-report.json').read_text())
    manifest = report.get('xp_kit_native_pfd')
    if not manifest:
        parser.error('Not an xp-kit native build')
    expected = next(item['sha256'] for item in report['binaries'] if item['file'] == 'mac.xpl')
    if sha(staging / 'mac.xpl') != expected or sha(staging / 'A380XDisplayHost') != manifest['helper_sha256']:
        parser.error('Build binary hash mismatch')
    if sha(staging / 'xp-kit/pfd.html') != manifest['html_sha256']:
        parser.error('Display assets hash mismatch')
    for name,expected_hash in manifest.get('asset_hashes',{}).items():
        candidate=(staging/'xp-kit'/name).resolve()
        if not candidate.is_relative_to((staging/'xp-kit').resolve()) or sha(candidate)!=expected_hash:
            parser.error('Display asset hash mismatch: '+name)
    for name in ('mac.xpl', 'A380XDisplayHost'):
        subprocess.run(['codesign', '--verify', '--strict', str(staging / name)], check=True)
    target = ROOT / 'output/A380X-XP12/plugins/A380XAvionics/mac_x64'
    backup = None if args.no_backup or args.check_only else ROOT / 'build/backups' / ('xp-kit-cockpit-' + datetime.now().strftime('%Y%m%d-%H%M%S'))
    if backup:
        backup.mkdir(parents=True)
    aircraft=ROOT/'output/A380X-XP12'
    calibration=report.get('fcu_calibration')
    if calibration:
        if calibration.get('file')!='fcu-calibration.ini' or sha(staging/calibration['file'])!=calibration['sha256']:
            parser.error('FCU calibration asset is missing or changed')
    object_paths=('objects/interior/cockpit-controls/a380x_cockpit_controls.obj',
                  'objects/interior/cockpit-panels/a380x_mfd_screens.obj')
    generated=[]
    sd=report.get('ecam_sd')
    sd_patch=None
    if sd:
        from avionics_provider import SD_COMMANDS,SD_DATAREFS,SD_DEVICES
        if (sd.get('commands')!=list(SD_COMMANDS) or sd.get('datarefs')!=list(SD_DATAREFS)
                or sd.get('cockpit_devices')!=list(SD_DEVICES)):
            parser.error('Stale ECAM native registry contract; rebuild before installing')
        from sd_cockpit_device import MARKER, augment_controls as sd_controls, augment_screen as sd_screen
        geometry=sd.get('geometry')
        if geometry:
            patch=(staging/geometry['controls_patch']).resolve()
            if not patch.is_relative_to(staging) or sha(patch)!=geometry['controls_sha256']:
                parser.error('ECAM control patch is missing or changed')
            sd_patch=patch.read_text()
        elif not all(MARKER in (aircraft/p).read_text() for p in object_paths):
            parser.error('ECAM build requires --ecam-source geometry for this aircraft')
    efb=report.get('efb')
    if efb:
        from efb_cockpit_devices import (EFB_DEVICES,EFB_COMMANDS as EFB_DEVICE_COMMANDS,EFB_DATAREFS as EFB_DEVICE_DATAREFS,
                                         augment_controls as efb_controls,augment_screen as efb_screen,check as efb_check)
        if (efb.get('commands')!=list(EFB_DEVICE_COMMANDS) or efb.get('datarefs')!=list(EFB_DEVICE_DATAREFS)
                or efb.get('cockpit_devices')!=list(EFB_DEVICES)):
            parser.error('Stale web EFB native registry contract; rebuild before installing')
        if 'efb_html_sha256' not in manifest or manifest.get('asset_hashes',{}).get('efb.html')!=manifest['efb_html_sha256']:
            parser.error('Web EFB page missing from the verified display assets')
    isis=report.get('isis')
    if isis:
        from isis_cockpit_device import (ISIS_COMMANDS,ISIS_DATAREFS,ISIS_DEVICES,
                                         augment_controls_all as isis_controls, augment_screens_all as isis_screen)
        if (isis.get('commands')!=list(ISIS_COMMANDS) or isis.get('datarefs')!=list(ISIS_DATAREFS)
                or isis.get('cockpit_devices')!=list(ISIS_DEVICES)):
            parser.error('Stale ISIS native registry contract; rebuild before installing')
    if 'nd_html_sha256' in manifest:
        for relative,touch in zip(object_paths,(True,False)):
            original=aircraft/relative
            encoded=pfd_augment(augment(original.read_text(),touch),touch)
            if touch:
                encoded=refresh_touch_geometry(encoded)
            if sd:
                encoded=sd_controls(encoded,sd_patch) if touch and sd_patch is not None else sd_screen(encoded) if not touch else encoded
            if isis:
                # In-place append after every existing patch (FCU/EFIS LCD UVs, PFD/ND/SD quads untouched).
                encoded=isis_controls(encoded) if touch else isis_screen(encoded)
            if efb:
                # Web EFB tablets: retire the two atlas faces and 34 atlas touch regions, append the
                # device quads (idempotent; refuses an unexpected cockpit object).
                try:
                    encoded,_stats=efb_controls(encoded) if touch else efb_screen(encoded)
                except ValueError as error:
                    parser.error('Web EFB geometry cannot be applied: '+str(error))
            generated.append((original,encoded))
            if backup:
                saved=backup/relative;saved.parent.mkdir(parents=True,exist_ok=True);copy_file(original,saved)
        if backup:
            for relative in (MANIFEST,Path(FILES[-1][1])):
                saved=backup/relative;saved.parent.mkdir(parents=True,exist_ok=True);copy_file(aircraft/relative,saved)
    if efb and len(generated)==2 and efb_check(generated[0][1],generated[1][1]):
        parser.error('Web EFB geometry check failed: '+'; '.join(efb_check(generated[0][1],generated[1][1])))
    eng=sd.get('eng_button') if sd else None
    if eng:
        from ecam_button_geometry import ENG_OBJECT_PATHS
        textures=eng.get('textures',[])
        if len(textures)!=2 or {row.get('path') for row in textures}!={
                'cockpit_3d/fcu_labels.png','cockpit_3d/fcu_labels_LIT.png'}:
            parser.error('ENG geometry requires both verified selection atlas textures')
        for row in textures:
            texture=(aircraft/row['path']).resolve()
            if not texture.is_relative_to(aircraft.resolve()) or sha(texture)!=row['sha256']:
                parser.error('ENG selection atlas changed after build')
        files=eng.get('files',[])
        if {row.get('path') for row in files}!=set(ENG_OBJECT_PATHS) or len(files)!=4:
            parser.error('ENG geometry must contain exactly the four inspected cockpit objects')
        updates={}
        for row in files:
            original=(aircraft/row['path']).resolve()
            staged=(staging/row['file']).resolve()
            if not original.is_relative_to(aircraft.resolve()) or not staged.is_relative_to(staging):
                parser.error('Unsafe ENG object path')
            if sha(staged)!=row['sha256'] or sha(original) not in (row['base_sha256'],row['sha256']):
                parser.error('ENG cockpit object changed after build; rebuild before installing: '+row['path'])
            for prior,encoded in generated:
                if prior.resolve()==original and encoded!=prior.read_text():
                    parser.error('Other cockpit geometry needs updating first; rebuild ENG against that version')
            updates[original]=staged.read_text()
            if backup:
                saved=backup/row['path'];saved.parent.mkdir(parents=True,exist_ok=True);copy_file(original,saved)
        generated=[(path,encoded) for path,encoded in generated if path.resolve() not in updates]
        generated.extend(updates.items())
    if args.check_only:
        staged_files={}
        if args.stage_dir:
            stage=args.stage_dir.resolve()
            if stage.exists():
                parser.error('Stage directory already exists; select a fresh path')
            for path,encoded in generated:
                out=stage/path.relative_to(aircraft);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(encoded)
            def staged_sha(packaged):
                if packaged==FILES[0][1]:return sha(staging/'mac.xpl')
                if packaged==FILES[-1][1]:return sha(staging/'build-report.json')
                source=next(src for src,dst in FILES if dst==packaged)
                return sha(ROOT/source) if packaged.startswith('plugins/A380XAvionics/sounds/') else sha(aircraft/packaged)
            if generated:
                out=stage/MANIFEST;out.parent.mkdir(parents=True,exist_ok=True)
                out.write_text(provider_document(aircraft,sd,isis,efb,staged_sha))
            for path in sorted(p for p in stage.rglob('*') if p.is_file() and not p.name.startswith('._')):
                relative=str(path.relative_to(stage));live=aircraft/relative
                staged_files[relative]={'before_sha256':sha(live) if live.is_file() else None,'after_sha256':sha(path)}
            staged_files[FILES[0][1]]={'before_sha256':sha(aircraft/FILES[0][1]),'after_sha256':sha(staging/'mac.xpl'),
                                      'source':str(staging/'mac.xpl')}
            staged_files[FILES[-1][1]]={'before_sha256':sha(aircraft/FILES[-1][1]),'after_sha256':sha(staging/'build-report.json'),
                                       'source':str(staging/'build-report.json')}
        print(json.dumps({'check_only':True,'passed':True,'backup':None,
            'aircraft':str(aircraft),'plugin_sha256':expected,
            'object_updates':[str(path.relative_to(aircraft)) for path,encoded in generated if encoded!=path.read_text()],
            'object_sha256_after':{str(path.relative_to(aircraft)):hashlib.sha256(encoded.encode()).hexdigest() for path,encoded in generated},
            'web_efb':bool(efb),'eng_button':bool(eng),'staged':staged_files or None}))
        return
    if subprocess.run(['pgrep','-x','X-Plane'],capture_output=True).returncode==0:
        parser.error('X-Plane started during preflight; installation has not begun')
    if backup and target.exists():
        shutil.copytree(target, backup / 'mac_x64', copy_function=copy_file)
    xp = Path('/Users/sava/Library/Application Support/Steam/steamapps/common/X-Plane 12')
    for source in (xp / 'Log.txt', xp / 'Output/preferences/X-Plane.prf',
                   ROOT / 'output/A380X-XP12/A380X-XP12_prefs.txt'):
        if backup and source.exists():
            copy_file(source, backup / source.name)
    target.mkdir(parents=True, exist_ok=True)
    if calibration and not (aircraft/'fcu-calibration.ini').exists():
        copy_file(staging/'fcu-calibration.ini',aircraft/'fcu-calibration.ini')
    print('Installing verified plugin and helper (backup '+('disabled' if backup is None else 'complete')+')', flush=True)
    copy_file(staging / 'mac.xpl', target / 'A380XAvionics.xpl')
    copy_file(staging / 'A380XDisplayHost', target / 'A380XDisplayHost')
    shutil.copytree(staging / 'xp-kit', target / 'xp-kit', dirs_exist_ok=True, copy_function=copy_file,
                    ignore=shutil.ignore_patterns('._*'))
    copy_file(staging / 'build-report.json', target / 'xp-kit-build-report.json')
    if generated:
        # All-or-nothing: if a later object write fails, every object written so far is restored.
        previous=[(original,original.read_text()) for original,_ in generated]
        written=[]
        try:
            for original,encoded in generated:
                write_text(original,encoded)
                written.append(original)
        except BaseException:
            for original,text in previous:
                if original in written:
                    write_text(original,text)
            raise
        copy_file(staging/'build-report.json',aircraft/FILES[-1][1])
        # Packaged plugin sounds (FILES order; the build report stays last) are copied from the
        # verified native sources; rows are rebuilt so a newly added sound is listed and hashed.
        for source,packaged in FILES:
            if packaged.startswith('plugins/A380XAvionics/sounds/'):
                (aircraft/packaged).parent.mkdir(parents=True,exist_ok=True)
                copy_file(ROOT/source,aircraft/packaged)
        write_text(aircraft/MANIFEST,provider_document(aircraft,sd,isis,efb,lambda packaged:sha(aircraft/packaged)))
    if sha(target/'A380XAvionics.xpl') != expected or sha(target/'A380XDisplayHost') != manifest['helper_sha256']:
        raise RuntimeError('Installed binary hashes differ from the verified build')
    print(json.dumps({'installed': str(target), 'backup': str(backup) if backup else None,
                      'aircraft': str(ROOT / 'output/A380X-XP12/A380X-XP12.acf'),
                      'plugin_sha256': sha(target / 'A380XAvionics.xpl')}))


if __name__ == '__main__':
    main()
