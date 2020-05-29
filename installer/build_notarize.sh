#!/bin/bash

PACKAGE_NAME="NexDomeV3_X2.pkg"
BUNDLE_NAME="org.rti-zone.NexDomeV3X2"

if [ ! -z "$app_id_signature" ]; then
    codesign -f -s "$app_id_signature" --verbose ../build/Release/libNexDomeV3.dylib
fi

mkdir -p ROOT/tmp/NexDomeV3_X2/
cp "../NexDomeV3.ui" ROOT/tmp/NexDomeV3_X2/
cp "../NexDome.png" ROOT/tmp/NexDomeV3_X2/
cp "../domelist NexDomeV3.txt" ROOT/tmp/NexDomeV3_X2/
cp "../build/Release/libNexDomeV3.dylib" ROOT/tmp/NexDomeV3_X2/


if [ ! -z "$installer_signature" ]; then
	# signed package using env variable installer_signature
	pkgbuild --root ROOT --identifier "$BUNDLE_NAME" --sign "$installer_signature" --scripts Scripts --version 1.0 "$PACKAGE_NAME"
	pkgutil --check-signature "./${PACKAGE_NAME}"
	res=`xcrun altool --notarize-app --primary-bundle-id "$BUNDLE_NAME" --username "$AC_USERNAME" --password "@keychain:AC_PASSWORD" --file $PACKAGE_NAME`
	RequestUUID=`echo $res | grep RequestUUID | cut -d"=" -f 2 | tr -d [:blank:]`
	if [ -z "$RequestUUID" ]; then
		echo "Error notarizing"
		echo "res = $res"
		exit 1
	fi
	echo "Notarization RequestUUID '$RequestUUID'"
	sleep 30
	while true
	echo "Waiting for notarization"
	do
		res=`xcrun altool --notarization-info "$RequestUUID" --username "pineau@rti-zone.org" --password "@keychain:AC_PASSWORD"`
		pkg_ok=`echo $res | grep -i "Package Approved"`
		pkg_in_progress=`echo $res | grep -i "in progress"`
		if [ ! -z "$pkg_ok" ]; then
			break
		elif [ ! -z "$pkg_in_progress" ]; then
			sleep 60
		else
			echo  "Notarization timeout or error"
			echo "res = $res"
			exit 1
		fi
	done
	xcrun stapler staple $PACKAGE_NAME

else
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --scripts Scripts --version 1.0 $PACKAGE_NAME
fi

rm -rf ROOT
