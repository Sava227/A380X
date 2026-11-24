#!/bin/sh
# Offline tests of A380XDoors against a mock XPLM: escape slides, gear steering visuals, main gear bogie tilt.
# Usage: XPLM_SDK=/path/to/XPSDK ./run_tests.sh
set -e
cd "$(dirname "$0")"
: "${XPLM_SDK:?set XPLM_SDK to the X-Plane SDK folder (with CHeaders/)}"
mkdir -p out
for t in test_slides test_gear_steering test_bogie_tilt; do
    clang -std=c11 -Wall -Wno-unused-parameter -DAPL=1 -DIBM=0 -DLIN=0 -DXPLM200=1 -DXPLM210=1 -DXPLM300=1 \
        -DXPLM301=1 -DXPLM400=1 -I "$XPLM_SDK/CHeaders/XPLM" ../A380XDoors.c mock_xplm.c "$t.c" -o "out/$t"
    "./out/$t" | grep -v '^A380X '
done
