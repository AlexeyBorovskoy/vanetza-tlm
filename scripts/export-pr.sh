#!/bin/sh
# Split the overlay into the upstream pull request series, in submission order:
#   1. facilities helpers, 2. fuzzing, 3. fixed-time plan model,
#   4. socktap applications and docs, 5. MAPEM/SPATEM validation, 6. timing options.
# The SPATEM/MAPEM asn1 tests are part of Vanetza since riebl/vanetza#328.
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

# pr <message> <patch>... -- <file>...
pr() {
    message=$1
    shift
    while [ "$1" != "--" ]; do
        git -C "$repo" apply --whitespace=error-all "$root/patches/$1"
        shift
    done
    shift
    for file in "$@"; do
        mkdir -p "$repo/$(dirname "$file")"
        cp "$root/$file" "$repo/$file"
    done
    git -C "$repo" add -A
    git -C "$repo" commit --quiet -m "$message"
}

pr "facilities: add SPaT, MAP and priority message helpers

Builders for IntersectionState, MovementState, IntersectionGeometry,
lanes, nodes, connections, SignalRequest and SignalStatus, plus the
SPaT time base: MinuteOfTheYear, DSecond and TimeMark in UTC with the
hour rule of C2C-CC RS 2077 RS_ARSM_54 (ETSI TS 103 301,
ISO TS 19091)." \
    0002-facilities-add-spat-map-priority-functions.patch -- \
    vanetza/facilities/detail/asn1_list.tpp \
    vanetza/facilities/spat_functions.hpp vanetza/facilities/spat_functions.cpp \
    vanetza/facilities/map_functions.hpp vanetza/facilities/map_functions.cpp \
    vanetza/facilities/priority_functions.hpp vanetza/facilities/priority_functions.cpp \
    vanetza/facilities/tests/spat_functions.cpp \
    vanetza/facilities/tests/map_functions.cpp \
    vanetza/facilities/tests/priority_functions.cpp \
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

pr "facilities: validate MAPEM and SPATEM semantics

Checks of MapData and SPAT beyond ASN.1 constraints, and of a SPAT
against the MapData of its intersections: lane and signal group
references, identifiers, timing order and the requirements of
C2C-CC RS 2077 and C-Roads, selected by a validation profile. Every
issue carries its rule, severity and field path; rules without
normative text are warnings only." \
    0006-facilities-map-spat-validation.patch -- \
    vanetza/facilities/validation.hpp vanetza/facilities/validation.cpp \
    vanetza/facilities/map_spat_validation.hpp vanetza/facilities/map_spat_validation.cpp \
    vanetza/facilities/tests/map_spat_validation.cpp

pr "facilities: optionally always send maxEndTime

TimingOptions::always_max_end encodes an unknown latest end, or one
beyond the TimeMark window, as 36000 (C2C-CC RS 2077 RS_ARSM_59).
maxEndTime is mandatory in the C-Roads profile and optional for
fixed-time operation in C2C-CC RS 2077; the default is unchanged.
set_timing() rejects a latest or likely end before the minimum end." \
    0007-facilities-timing-options.patch --

git -C "$repo" format-patch --quiet -o "$out" "$base"

# every overlay file must be part of exactly the exported series
sh "$root/scripts/overlay.sh" > /dev/null
if ! diff -r -x .git "$repo" "$root/work/vanetza" > "$out/tree.diff"; then
    echo "exported series differs from overlay, see $out/tree.diff" >&2
    exit 1
fi
rm "$out/tree.diff"
ls "$out"
