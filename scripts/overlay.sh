#!/bin/sh
# Build the upstream-shaped tree: clean Vanetza (submodule) + overlay files + patches.
# Result in work/vanetza is exactly what an upstream PR series would contain.
#
# The facilities helpers exist in three registration states (see
# select-functions-patch.sh): none registered, MAP registered already, or both
# registered.  The overlay files are only copied where the tree still lacks the
# helper; an upstream helper that is already registered stays byte for byte.
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

# keep the upstream helpers the overlay must not replace, then copy the overlay
# wholesale and put the upstream files back
preserve=$(sh "$root/scripts/functions-overlay-files.sh" --preserve "$dst") || exit 1
stash=$(mktemp -d)
trap 'rm -rf "$stash"' EXIT
for file in $preserve; do
    if [ ! -f "$dst/$file" ]; then
        echo "overlay: $file is registered in the tree but missing upstream" >&2
        exit 1
    fi
    mkdir -p "$stash/$(dirname "$file")"
    cp "$dst/$file" "$stash/$file"
done

# overlay: files at their future Vanetza paths
for dir in vanetza tools docs; do
    if [ -d "$root/$dir" ]; then
        cp -R "$root/$dir" "$dst/"
    fi
done

for file in $preserve; do
    cp "$stash/$file" "$dst/$file"
done

# patch 0002 has up to three states: the pinned base needs the MAP and priority
# registration together, current upstream registers MAP already, and a tree that
# registers both needs no registration patch at all (empty output).
functions_patch=$(sh "$root/scripts/select-functions-patch.sh" "$dst") || {
    echo "cannot select the facilities functions patch" >&2
    exit 1
}
if [ -n "$functions_patch" ]; then
    echo "applying $(basename "$functions_patch")"
    patch -d "$dst" -p1 --forward --quiet < "$functions_patch"
else
    echo "facilities functions are registered already, no registration patch"
fi

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
