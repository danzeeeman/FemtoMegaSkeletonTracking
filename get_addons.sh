#!/usr/bin/env bash
# Clones the addons that do not ship with openFrameworks into <openFrameworks>/addons.
# ofxNetwork, ofxOpenCv and ofxOsc come with openFrameworks itself.
# Run from Git Bash (or a macOS shell). Existing addons are left alone (pass --update to git pull them).
# On macOS it also fetches the Orbbec SDK into ofxOrbbec.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
addons="$(cd "$here/../../addons" && pwd)"

repos=(
    "ofxImGui       git@github.com:danzeeeman/ofxImGui.git"
    "ofxOnnx        git@github.com:danzeeeman/ofxOnnx.git"
    "ofxOrbbec      git@github.com:danzeeeman/ofxOrbbec.git"
    "ofxYoloPose    git@github.com:danzeeeman/ofxYoloPose.git"
    "ofxRtmPoseTracker git@github.com:danzeeeman/ofxRtmPoseTracker.git"
)

for entry in "${repos[@]}"; do
    read -r name url <<<"$entry"
    if [ -d "$addons/$name/.git" ]; then
        if [ "${1:-}" = "--update" ]; then
            echo "update  $name"
            git -C "$addons/$name" pull --ff-only
        else
            echo "have    $name"
        fi
    elif [ -e "$addons/$name" ]; then
        echo "skip    $name (exists, not a git checkout)"
    else
        echo "clone   $name"
        git clone "$url" "$addons/$name"
    fi
done

# macOS: the OrbbecSDK_v2 release that ofxOrbbec links (git-ignored in the addon), laid out as its
# addon_config.mk expects: libs/OrbbecSDK/{include,lib/macos,extensions,OrbbecSDKConfig.xml}.
if [ "$(uname)" = "Darwin" ]; then
    sdk="$addons/ofxOrbbec/libs/OrbbecSDK"
    if [ -e "$sdk/lib/macos/libOrbbecSDK.dylib" ]; then
        echo "have    OrbbecSDK"
    elif [ -d "$addons/ofxOrbbec" ]; then
        echo "fetch   OrbbecSDK"
        sdkUrl="https://github.com/orbbec/OrbbecSDK_v2/releases/download/v2.10.6/OrbbecSDK_v2.10.6_202609291804_575347c_macOS.tar.gz"
        tmp="$(mktemp -d)"
        curl -fsSL -o "$tmp/sdk.tar.gz" "$sdkUrl"
        tar -xzf "$tmp/sdk.tar.gz" -C "$tmp"
        src="$(dirname "$(find "$tmp" -name libOrbbecSDK.dylib -path '*/lib/*' | head -1)")/.."
        mkdir -p "$sdk/lib/macos"
        cp -R "$src/include" "$sdk/"
        cp -a "$src/lib/"*.dylib "$sdk/lib/macos/"
        cp -R "$src/lib/extensions" "$sdk/"
        cp "$src/lib/OrbbecSDKConfig.xml" "$src/LICENSE.txt" "$sdk/"
        rm -rf "$tmp"
    fi
fi
