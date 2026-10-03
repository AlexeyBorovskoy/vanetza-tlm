#!/bin/sh
# Build the upstream-shaped tree: clean Vanetza (submodule) + overlay files + patches.
# Result in work/vanetza is exactly what an upstream PR series would contain.
set -eu

root=$(cd "$(dirname "$0")/.." && pwd)
src="$root/external/vanetza"
dst="$root/work/vanetza"

if [ ! -f "$src/CMakeLists.txt" ]; then
    echo "submodule missing: git submodule update --init" >&2
    exit 1
fi

rm -rf "$dst"
mkdir -p "$dst"
git -c core.autocrlf=false -C "$src" archive HEAD | tar -x -C "$dst"

# overlay: files at their future Vanetza paths
for dir in vanetza tools docs; do
    if [ -d "$root/$dir" ]; then
        cp -R "$root/$dir" "$dst/"
    fi
done

# patches: edits of existing Vanetza files (CMakeLists, socktap main.cpp, ...)
for patch in "$root"/patches/*.patch; do
    [ -e "$patch" ] || continue
    echo "applying $(basename "$patch")"
    patch -d "$dst" -p1 --forward --quiet < "$patch"
done

echo "overlay ready: $dst"
