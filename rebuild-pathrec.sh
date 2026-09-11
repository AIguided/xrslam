#!/bin/bash
# Rebuild + sign + install PathRec (minimal RD-VIO walk-path recorder).
# Signing happens in /tmp because this repo lives under iCloud-synced
# ~/Documents, whose FileProvider re-adds xattrs that break codesign.
set -e
cd "$(dirname "$0")"
source build-ios.conf

cmake -S . -B build/iOS -G Xcode \
    -D CMAKE_TOOLCHAIN_FILE=cmake/Modules/Platform/ios.toolchain.cmake \
    -D CMAKE_CONFIGURATION_TYPES=Release \
    -D IOS_PLATFORM=OS64 -D IOS_ARCH=arm64 -D IOS_DEPLOYMENT_TARGET=12.0 \
    -D ENABLE_BITCODE=0 -D ENABLE_ARC=1 -D ENABLE_VISIBILITY=0 \
    -D CMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -D APP_IDENTIFIER_PREFIX="${APP_IDENTIFIER_PREFIX}" \
    -D IOS_DEVELOPMENT_TEAM="${IOS_DEVELOPMENT_TEAM}"

# The in-tree codesign phase fails on iCloud xattrs; compile+link still
# complete, and we re-sign a /tmp staging copy below.
cmake --build build/iOS --config Release --target xrslam-ios-pathrec -- -allowProvisioningUpdates || true

APP="build/iOS/xrslam-ios/pathrec/Release-iphoneos/xrslam-ios-pathrec.app"
STAGE="/tmp/pathrec-staging.app"
XCENT="build/iOS/build/xrslam-ios-pathrec.build/Release-iphoneos/xrslam-ios-pathrec.app.xcent"

if [ ! -d "$APP" ]; then
    echo "error: app not found at $APP (did compile fail?)"; exit 1
fi

# Pin the signing identity this device's provisioning profiles were built with.
SIGN_HASH="${PATHREC_SIGN_HASH:-8320AE9BB245F83F07B430BBCEAB600B95A95DC5}"
rm -rf "$STAGE"
ditto --noextattr --noqtn "$APP" "$STAGE"
xattr -cr "$STAGE"
/usr/bin/codesign --force --sign "$SIGN_HASH" \
    --entitlements "$XCENT" --timestamp=none --generate-entitlement-der "$STAGE"
echo "signed: $STAGE"
codesign -dv "$STAGE" 2>&1 | grep Identifier

DEVICE_ID=$(xcrun devicectl list devices 2>/dev/null | grep AIguidediPhone2 | awk '{print $3}')
echo "installing to device..."
xcrun devicectl device install app --device "$DEVICE_ID" "$STAGE" --timeout 300 2>&1 | grep -E "installationURL|ERROR"
