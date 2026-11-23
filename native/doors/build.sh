#!/bin/sh
# Builds the A380XDoors plugin (door cycle times, cabin shades and bins, spoiler/aileron surface values).
# Usage: XPLM_SDK=/path/to/XPSDK ./build.sh      (needs clang on macOS and x86_64-w64-mingw32-gcc for Windows)
# Install: out/mac.xpl -> <aircraft>/plugins/A380XDoors/mac_x64/A380XDoors.xpl,
#          out/win.xpl -> <aircraft>/plugins/A380XDoors/win_x64/A380XDoors.xpl, plus door_timing.ini in plugins/A380XDoors/.
set -e
cd "$(dirname "$0")"
: "${XPLM_SDK:?set XPLM_SDK to the X-Plane SDK folder (with CHeaders/ and Libraries/)}"
DEFS="-DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1 -DXPLM400=1"
mkdir -p out
clang -arch x86_64 -arch arm64 -mmacosx-version-min=10.15 -O2 -Wall -Wextra -Werror -std=c11 -dynamiclib \
    -DAPL=1 -DIBM=0 -DLIN=0 $DEFS -I "$XPLM_SDK/CHeaders/XPLM" -F "$XPLM_SDK/Libraries/Mac" -framework XPLM \
    A380XDoors.c -o out/mac.xpl
codesign -s - -f out/mac.xpl
if command -v x86_64-w64-mingw32-gcc >/dev/null 2>&1; then
    x86_64-w64-mingw32-gcc -O2 -Wall -Wextra -Werror -std=c11 -shared -DIBM=1 -DAPL=0 -DLIN=0 $DEFS \
        -I "$XPLM_SDK/CHeaders/XPLM" A380XDoors.c -L "$XPLM_SDK/Libraries/Win" -lXPLM_64 -static-libgcc -o out/win.xpl
fi
ls -la out
