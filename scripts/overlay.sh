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

# patch 0002 has two variants: the pinned base needs the MAP and priority
# registration together, current upstream already registers the MAP helpers.
# The helper inspects both facilities CMake lists and fails instead of guessing
# when the registration is inconsistent.
if ! functions_patch=$(sh "$root/scripts/select-functions-patch.sh" "$dst"); then
    echo "cannot select the facilities functions patch" >&2
    exit 1
fi
echo "applying $(basename "$functions_patch")"
patch -d "$dst" -p1 --forward --quiet < "$functions_patch"

# patches: edits of existing Vanetza files (CMakeLists, socktap main.cpp, ...)
for patch in "$root"/patches/*.patch; do
    [ -e "$patch" ] || continue
    name=$(basename "$patch")
    case $name in
        0002-facilities-add-spat-map-priority-functions.patch|0002-facilities-priority-only.patch)
            # variant selected above for this tree
            continue
            ;;
    esac
    echo "applying $name"
    patch -d "$dst" -p1 --forward --quiet < "$patch"
done

echo "overlay ready: $dst"
