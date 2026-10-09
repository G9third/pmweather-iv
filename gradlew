#!/bin/sh
set -eu

GRADLE_VERSION="8.10.2"
GRADLE_SHA256="31c55713e40233a8303827ceb42ca48a47267a0ad4bab9177123121e71524c26"
GRADLE_URL="https://services.gradle.org/distributions/gradle-${GRADLE_VERSION}-bin.zip"
CACHE_ROOT="${GRADLE_USER_HOME:-$HOME/.gradle}/pmweather-iv-wrapper"
GRADLE_HOME="$CACHE_ROOT/gradle-$GRADLE_VERSION"
ZIP_PATH="$CACHE_ROOT/gradle-$GRADLE_VERSION-bin.zip"

verify_zip() {
    if command -v sha256sum >/dev/null 2>&1; then
        actual=$(sha256sum "$ZIP_PATH" | awk '{print $1}')
    elif command -v shasum >/dev/null 2>&1; then
        actual=$(shasum -a 256 "$ZIP_PATH" | awk '{print $1}')
    else
        echo "ERROR: sha256sum or shasum is required." >&2
        return 1
    fi
    [ "$actual" = "$GRADLE_SHA256" ]
}

if [ ! -x "$GRADLE_HOME/bin/gradle" ]; then
    mkdir -p "$CACHE_ROOT"

    if [ -f "$ZIP_PATH" ] && ! verify_zip; then
        rm -f "$ZIP_PATH"
    fi

    if [ ! -f "$ZIP_PATH" ]; then
        echo "Downloading Gradle $GRADLE_VERSION..."
        if command -v curl >/dev/null 2>&1; then
            curl -fL "$GRADLE_URL" -o "$ZIP_PATH"
        elif command -v wget >/dev/null 2>&1; then
            wget -O "$ZIP_PATH" "$GRADLE_URL"
        else
            echo "ERROR: curl or wget is required to download Gradle." >&2
            exit 1
        fi
    fi

    if ! verify_zip; then
        echo "ERROR: Gradle archive checksum mismatch." >&2
        rm -f "$ZIP_PATH"
        exit 1
    fi

    command -v unzip >/dev/null 2>&1 || {
        echo "ERROR: unzip is required to extract Gradle." >&2
        exit 1
    }

    rm -rf "$GRADLE_HOME"
    unzip -q -o "$ZIP_PATH" -d "$CACHE_ROOT"
fi

exec "$GRADLE_HOME/bin/gradle" "$@"
