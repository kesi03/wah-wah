#!/bin/sh
set -eu

SDK_DIR="sdk"
SDK_URL="https://github.com/Vibe-FM-SDK/vibe-fm-sdk.git"
SDK_COMMIT="2ac234088ee5e2da4503f380f3a167d253cc75d4"

if [ -d "$SDK_DIR/.git" ]; then
    echo "Updating existing SDK checkout in $SDK_DIR"
    git -C "$SDK_DIR" fetch origin
else
    echo "Cloning $SDK_URL into $SDK_DIR"
    git clone "$SDK_URL" "$SDK_DIR"
fi

git -C "$SDK_DIR" checkout "$SDK_COMMIT"
echo "SDK pinned at $SDK_COMMIT"
