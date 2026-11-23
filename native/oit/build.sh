#!/bin/zsh
# Build the A380XOIT plugin (macOS universal + Windows x64) and run the offline core tests.
# Usage: ./build.sh            (from this folder)
set -e
HERE=${0:A:h}
cd "$HERE"
SDK="${A380X_XPSDK:-$HERE/../../../Aircraft/A220 FMS Dev/source/A220_FMS/SDK}"
DEFS=(-DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1 -DXPLM400=1 -DXPLM410=1)
SRC=(src/oit_core.c src/oit_draw.c)
INC=(-I src -I gen)
WARN=(-Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -Werror)

python3 tools/make_font.py fonts gen >/dev/null

# offline tests: core + software renderer, no X-Plane
mkdir -p test/out
clang -O1 -g -std=c11 $WARN -fsanitize=address,undefined $INC $SRC src/oit_soft.c test/test_oit.c -lm -o test/test_oit
(cd test && ./test_oit)
python3 - <<'EOF'
import glob, os
from PIL import Image
for p in glob.glob('test/out/*.ppm'):
    Image.open(p).save(p[:-4] + '.png'); os.remove(p)
EOF

mkdir -p out/mac_x64 out/win_x64
clang -arch x86_64 -arch arm64 -mmacosx-version-min=10.15 -O2 -std=c11 $WARN -dynamiclib -DGL_SILENCE_DEPRECATION -DAPL=1 -DIBM=0 -DLIN=0 $DEFS \
  -I "$SDK/CHeaders/XPLM" $INC $SRC src/oit_gl.c src/oit_xplm.c -F "$SDK/Libraries/Mac" -framework XPLM -framework OpenGL \
  -o out/mac_x64/A380XOIT.xpl
codesign -s - -f out/mac_x64/A380XOIT.xpl
x86_64-w64-mingw32-gcc -O2 -std=c11 $WARN -Wno-format-truncation -shared -DIBM=1 -DAPL=0 -DLIN=0 $DEFS -I "$SDK/CHeaders/XPLM" $INC \
  $SRC src/oit_gl.c src/oit_xplm.c -L "$SDK/Libraries/Win" -lXPLM_64 -lopengl32 -static-libgcc -o out/win_x64/A380XOIT.xpl
lipo -info out/mac_x64/A380XOIT.xpl
x86_64-w64-mingw32-objdump -p out/win_x64/A380XOIT.xpl | grep -E "DLL Name" | sort -u
