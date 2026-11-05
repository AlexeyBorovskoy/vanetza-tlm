#!/bin/sh
# Split the overlay into the upstream pull request series, in submission order:
#   1. facilities helpers, 2. fuzzing, 3. fixed-time plan model,
#   4. socktap applications and docs.
# The SPATEM/MAPEM asn1 tests are part of Vanetza since riebl/vanetza#328.
# The SPaT time conversions are part of Vanetza since riebl/vanetza#330.
# The facilities helpers exist in three registration states (see
# select-functions-patch.sh): none registered, MAP registered already, or both
# registered.  The first stage is then empty only when the base already has the
# helpers and the recipe; such a stage is skipped explicitly, so the series has
# fewer than four commits.  Every other difference still fails.
# Result: work/pr/*.patch (git format-patch against the pinned Vanetza commit) and work/pr-repo.
# The script fails if the series does not reproduce the overlay tree exactly.
set -eu

root=$(cd "$(dirname "$0")/.." && pwd)
src="$root/external/vanetza"
repo="$root/work/pr-repo"
out="$root/work/pr"
base=$(git -C "$src" rev-parse HEAD)

rm -rf "$repo" "$out"
git -c core.autocrlf=false clone --quiet --no-checkout "$src" "$repo"
git -C "$repo" config core.autocrlf false
git -C "$repo" checkout --quiet -b tlm "$base"
git -C "$repo" config user.name "${PR_AUTHOR_NAME:-$(git -C "$root" config user.name)}"
git -C "$repo" config user.email "${PR_AUTHOR_EMAIL:-$(git -C "$root" config user.email)}"

# patch 0002 has up to three states: the pinned base needs the MAP and priority
# registration together, current upstream registers MAP already, and a tree that
# registers both needs no registration patch at all.  The selector prints nothing
# in the last state; the copy list from the file helper is empty then as well.
# Select by the base tree; fail instead of guessing on inconsistent registration.
functions_patch=$(sh "$root/scripts/select-functions-patch.sh" "$repo") || {
    echo "cannot select the facilities functions patch" >&2
    exit 1
}
if [ -n "$functions_patch" ]; then
    functions_patch=$(basename "$functions_patch")
fi
# subset of the overlay helpers the base still lacks, may be empty
functions_files=$(sh "$root/scripts/functions-overlay-files.sh" --copy "$repo") || {
    echo "cannot select the facilities functions files" >&2
    exit 1
}

# pr [--allow-empty] <message> <patch>... -- <file>...
# An empty patch or file is skipped; the helper lists are expanded unquoted, so an
# empty list adds no argument.
pr() {
    allow_empty=0
    if [ "$1" = "--allow-empty" ]; then
        allow_empty=1
        shift
    fi
    message=$1
    shift
    while [ "$1" != "--" ]; do
        if [ -n "$1" ]; then
            git -C "$repo" apply --whitespace=error-all "$root/patches/$1"
        fi
        shift
    done
    shift
    for file in "$@"; do
        mkdir -p "$repo/$(dirname "$file")"
        cp "$root/$file" "$repo/$file"
    done
    git -C "$repo" add -A
    if git -C "$repo" diff --cached --quiet; then
        if [ "$allow_empty" -eq 1 ]; then
            echo "note: export stage changes nothing, skipped: $(printf '%s\n' "$message" | head -n 1)" >&2
            return 0
        fi
        echo "export stage changes nothing: $(printf '%s\n' "$message" | head -n 1)" >&2
        exit 1
    fi
    git -C "$repo" commit --quiet -m "$message"
}

# the first stage may legitimately be empty when the base has both helpers and the recipe
pr --allow-empty "facilities: add MAP and priority message helpers

Builders for IntersectionGeometry, lanes, nodes, connections,
SignalRequest and SignalStatus on top of the asn1::append helpers
of Vanetza." \
    "$functions_patch" -- \
    $functions_files \
    docs/recipes/spatem-mapem.md

pr "tools: fuzz SPATEM, MAPEM, SREM and SSEM decoding

Decode arbitrary input, re-encode accepted messages and require an
identical round trip; runs with AFL persistent mode or libFuzzer.
One encoded message of each type seeds the corpus." \
    0005-tools-fuzz-infrastructure-messages.patch -- \
    tools/fuzz-harness/asn1_decoding.hpp tools/fuzz-harness/asn1_decoding.cpp \
    tools/fuzz-harness/asn1_persistent.cpp tools/fuzz-harness/asn1_run.cpp \
    tools/fuzz-harness/asn1_libfuzzer.cpp \
    tools/fuzz-harness/input/spatem.dat tools/fuzz-harness/input/mapem.dat \
    tools/fuzz-harness/input/srem.dat tools/fuzz-harness/input/ssem.dat

pr "facilities: add fixed-time signal plan model

Coordinated fixed-time plans with a weekly schedule fill SPaT movement
states and timing. Protected and permissive states are derived from
conflicting paths of signal groups; timing follows C2C-CC RS 2077
RS_ARSM_55..59 and ISO TS 19091 (MovementEvent, TimeChangeDetails)." \
    0002b-facilities-fixed-time-plan.patch -- \
    vanetza/facilities/fixed_time_plan.hpp vanetza/facilities/fixed_time_plan.cpp \
    vanetza/facilities/tests/fixed_time_plan.cpp

pr "tools: add socktap TLM, RLT and TLC applications

Example road-side and vehicle applications sending SPATEM, MAPEM, SREM
and SSEM on the BTP ports of ETSI TS 103 301 for a synthetic
intersection driven by the fixed-time plan model." \
    0003-tools-socktap-tlm-rlt-applications.patch 0004-docs-socktap-tlm-rlt.patch -- \
    tools/socktap/fixed_time_intersection.hpp tools/socktap/fixed_time_intersection.cpp \
    tools/socktap/tlm_application.hpp tools/socktap/tlm_application.cpp \
    tools/socktap/rlt_application.hpp tools/socktap/rlt_application.cpp \
    tools/socktap/tlc_request_application.hpp tools/socktap/tlc_request_application.cpp \
    tools/socktap/tlc_status_application.hpp tools/socktap/tlc_status_application.cpp \
    tools/socktap/infrastructure_message.hpp \
    tools/socktap/priority_request_table.hpp tools/socktap/priority_request_table.cpp \
    tools/socktap/tests/CMakeLists.txt tools/socktap/tests/priority_request_table.cpp

git -C "$repo" format-patch --quiet -o "$out" "$base"

# every overlay file must be part of exactly the exported series
sh "$root/scripts/overlay.sh" > /dev/null
if ! diff -r -x .git "$repo" "$root/work/vanetza" > "$out/tree.diff"; then
    echo "exported series differs from overlay, see $out/tree.diff" >&2
    exit 1
fi
rm "$out/tree.diff"
ls "$out"
