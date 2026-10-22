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

# macOS: the OrbbecSDK_v2 release that ofxOrbbec links (git-ignored in the addon); its own script fetches it.
if [ "$(uname)" = "Darwin" ] && [ -x "$addons/ofxOrbbec/scripts/install_sdk.sh" ]; then
    "$addons/ofxOrbbec/scripts/install_sdk.sh"
fi
