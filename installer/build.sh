#!/bin/bash

mkdir -p ROOT/tmp/NexDomeV3_X2/
cp "../NexDomeV3.ui" ROOT/tmp/NexDomeV3_X2/
cp "../NexDome.png" ROOT/tmp/NexDomeV3_X2/
cp "../domelist NexDomeV3.txt" ROOT/tmp/NexDomeV3_X2/
cp "../build/Release/libNexDomeV3.dylib" ROOT/tmp/NexDomeV3_X2/

if [ ! -z "$installer_signature" ]; then
# signed package using env variable installer_signature
pkgbuild --root ROOT --identifier org.rti-zone.NexDomeV3_X2 --sign "$installer_signature" --scripts Scripts --version 1.0 NexDomeV3_X2.pkg
pkgutil --check-signature ./NexDomeV3_X2.pkg
else
pkgbuild --root ROOT --identifier org.rti-zone.NexDomeV3_X2 --scripts Scripts --version 1.0 NexDomeV3_X2.pkg
fi

rm -rf ROOT
