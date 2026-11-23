#!/usr/bin/env python3
"""Cross-build the Windows display host with pinned official SDK inputs."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import uuid

ROOT=Path(__file__).resolve().parents[1]
NATIVE=ROOT/'native/avionics'
JSON_SHA='aaf127c04cb31c406e5b04a63f1ae89369fccde6d8fa7cdda1ed4f32dfc5de63'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def run(args):return subprocess.run([str(a) for a in args],check=True,text=True,capture_output=True).stdout

def build(sdk,json_header,output,mingw='/opt/homebrew/bin/x86_64-w64-mingw32-gcc',clang='clang'):
    sdk,json_header,output=map(Path,(sdk,json_header,output))
    output.mkdir(parents=True,exist_ok=True)
    header=sdk/'build/native/include/WebView2.h'
    loader=sdk/'build/native/x64/WebView2Loader.dll'
    if not header.is_file() or not loader.is_file():raise ValueError('Incomplete official WebView2 NuGet SDK')
    if sha(json_header)!=JSON_SHA:raise ValueError('Expected unmodified nlohmann/json v3.12.0 header')
    # Microsoft's MIDL UUID attributes need MinGW's generated specializations.
    # Every GUID comes from the actual official SDK header, never a guessed ID.
    declarations=[]
    for guid,name in re.findall(r'MIDL_INTERFACE\("([0-9a-fA-F-]+)"\)\s*(\w+)\s*:',header.read_text()):
        value=uuid.UUID(guid);f=value.fields;tail=value.bytes[8:]
        declarations.append('__CRT_UUID_DECL('+name+','+','.join(hex(n) for n in (f[0],f[1],f[2],*tail))+')')
    if not declarations:raise ValueError('Cannot resolve official WebView2 interface GUIDs')
    (output/'webview2_mingw_guids.h').write_text('\n'.join(declarations)+'\n')
    cc=str(mingw);cxx=cc.removesuffix('gcc')+'g++';objdump=cc.removesuffix('gcc')+'objdump';objcopy=cc.removesuffix('gcc')+'objcopy'
    include=['-I',str(NATIVE),'-I',str(json_header.parent)]
    draw=output/'mfd_draw_win.o'
    run([cc,'-std=c11','-O2','-c',NATIVE/'a380x_mfd_draw.c','-o',draw])
    exe=output/'A380XDisplayHost.exe'
    command=[cxx,'-std=c++17','-O2','-Wall','-Wextra','-Wno-unknown-pragmas','-Wno-misleading-indentation',
        '-Wno-cast-function-type','-D_WIN32_WINNT=0x0A00',*include,'-I',header.parent,'-I',output,
        NATIVE/'a380x_xpkit_host_win.cpp',draw,'-static','-static-libgcc','-static-libstdc++','-municode','-mwindows',
        '-luser32','-lshell32','-lole32','-loleaut32','-lwindowscodecs','-luuid','-lshlwapi','-lwinpthread','-o',exe]
    try:run(command)
    except subprocess.CalledProcessError as error:raise RuntimeError(error.stderr) from error
    # Compare emitted Windows C/C++ layout bytes, without pretending to run PE on macOS.
    layout='''#include "a380x_xpkit_shared.h"
const unsigned long long layout[] __attribute__((section(".a380abi"),used))={
sizeof(A380XXpKitShared),sizeof(A380XXpKitMfd),sizeof(MfdDrawList),
offsetof(A380XXpKitShared,state),offsetof(A380XXpKitShared,ready),
offsetof(A380XXpKitShared,pixels),offsetof(A380XXpKitShared,nd_pixels),
offsetof(A380XXpKitShared,mfd_pixels),offsetof(A380XXpKitShared,mfd),
offsetof(A380XXpKitShared,sd),offsetof(A380XXpKitShared,nd_profile),
offsetof(A380XXpKitShared,efb_pixels),offsetof(A380XXpKitShared,efb),sizeof(A380XXpKitEfb),
offsetof(A380XXpKitShared,surface_id),offsetof(A380XXpKitShared,surface_mode),offsetof(A380XXpKitShared,surface_front),
XPKIT_VERSION};
'''
    signatures=[]
    for extension,compiler in [('c',cc),('cpp',cxx)]:
        source=output/('layout.'+extension);source.write_text(layout)
        obj=output/('layout-'+extension+'.o');binary=output/('layout-'+extension+'.bin')
        run([compiler,'-std='+('c11' if extension=='c' else 'c++17'),*include,'-c',source,'-o',obj])
        run([objcopy,'--only-section=.a380abi','-O','binary',obj,binary]);signatures.append(binary.read_bytes())
    if signatures[0]!=signatures[1]:raise ValueError('Windows plugin/host shared-memory ABI differs')
    native_draw=output/'mfd_draw_test.o';test=output/'test_windows_display_wire'
    run([clang,'-std=c11','-c',NATIVE/'a380x_mfd_draw.c','-o',native_draw])
    run([clang+'++','-std=c++17',*include,NATIVE/'test_windows_display_wire.cpp',native_draw,'-o',test])
    test_result=run([test]).strip()
    inspection=run([objdump,'-p',exe]);dependencies=[line.split(':',1)[1].strip() for line in inspection.splitlines() if 'DLL Name:' in line]
    if 'pei-x86-64' not in run([objdump,'-f',exe]):raise ValueError('Display host is not PE x86-64')
    if any(any(name in dep.lower() for name in ('libgcc','libstdc','libwinpthread','python')) for dep in dependencies):
        raise ValueError('Unbundled compiler dependency: '+str(dependencies))
    shutil.copy2(loader,output/loader.name)
    license=sdk/'LICENSE.txt'
    if not license.exists():license=sdk/'LICENSE'
    if not license.exists():raise ValueError('Official WebView2 redistribution license missing')
    shutil.copy2(license,output/'WebView2-LICENSE.txt')
    manifest={'format':1,'platform':'Windows x86-64','views':9,'transport':'local Windows shared memory',
        'renderer':'Microsoft Edge WebView2 Evergreen','webview2_sdk':'1.0.3537.50','nlohmann_json':'3.12.0',
        'helper_sha256':sha(exe),'loader_sha256':sha(output/loader.name),'dependencies':dependencies,
        'ipc_c_cpp_abi_equal':True,'wire_test_result':test_result,'windows_runtime_tested':False,
        'source_sha256':{p.name:sha(p) for p in (NATIVE/'a380x_xpkit_host_win.cpp',NATIVE/'a380x_xpkit_wire.hpp',header,json_header)}}
    (output/'windows-display-build.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--webview2-sdk',required=True,type=Path);parser.add_argument('--json-header',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path);parser.add_argument('--mingw',default='/opt/homebrew/bin/x86_64-w64-mingw32-gcc')
    parser.add_argument('--update-native-report',type=Path,help='Refresh only the Windows host section of an existing native build report')
    args=parser.parse_args();result=build(args.webview2_sdk,args.json_header,args.output,args.mingw)
    if args.update_native_report:
        report=json.loads(args.update_native_report.read_text())
        expected=next(row['sha256'] for row in report['binaries'] if row['file']=='win.xpl')
        if sha(args.update_native_report.parent/'win.xpl')!=expected:raise ValueError('Native plugin changed; do not relabel its report')
        report['xp_kit_native_windows']=result
        args.update_native_report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(result,indent=2))
