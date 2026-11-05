#!/bin/sh
# Classify the facilities CMake lists and select the matching registration patch.
#
# Three registration states are supported:
#   full           neither helper is registered: the original patch registers the
#                  MAP and priority helpers together (pinned Vanetza base)
#   priority-only  the MAP helpers are registered exactly once and priority is
#                  absent: only the priority registration is added (current upstream)
#   none           the MAP and priority helpers are both registered exactly once:
#                  no registration patch is needed at all
#
# Every other registration - a partial source/test pair, a duplicate entry, or a
# priority registration without the MAP helpers - fails with a diagnostic on stderr
# and an empty stdout.  The variants are never guessed.
#
# Usage: select-functions-patch.sh [--state] <vanetza-tree>
#
# Without options it prints the patch file to apply.  In the "none" state it prints
# nothing and exits successfully, so callers must not hand an empty path to patch.
# With --state it prints one of: full, priority-only, none.
set -eu

root=$(cd "$(dirname "$0")/.." && pwd)

usage() {
    echo "usage: $0 [--state] <vanetza-tree>" >&2
    exit 2
}

mode=patch
case ${1:-} in
    --state)
        mode=state
        shift
        ;;
    -*)
        usage
        ;;
esac

if [ "$#" -ne 1 ]; then
    usage
fi

tree=$1
sources=$tree/vanetza/facilities/CMakeLists.txt
tests=$tree/vanetza/facilities/tests/CMakeLists.txt

if [ ! -f "$sources" ]; then
    echo "select-functions-patch: missing $sources" >&2
    exit 1
fi
if [ ! -f "$tests" ]; then
    echo "select-functions-patch: missing $tests" >&2
    exit 1
fi

# grep -c prints 0 and exits non-zero when nothing matches
count() {
    grep -Ec "$1" "$2" || true
}

map_sources=$(count '^[[:space:]]*map_functions\.cpp[[:space:]]*$' "$sources")
map_tests=$(count '^[[:space:]]*add_gtest\(MapFunctions[[:space:]]' "$tests")
priority_sources=$(count '^[[:space:]]*priority_functions\.cpp[[:space:]]*$' "$sources")
priority_tests=$(count '^[[:space:]]*add_gtest\(PriorityFunctions[[:space:]]' "$tests")

for value in "$map_sources" "$map_tests" "$priority_sources" "$priority_tests"; do
    case $value in
        0|1) ;;
        *)
            echo "select-functions-patch: unexpected match count '$value' in $tree" >&2
            exit 1
            ;;
    esac
done

# A helper is registered only if the source list and the test list agree.
if [ "$map_sources" -ne "$map_tests" ]; then
    echo "select-functions-patch: inconsistent MAP registration in $tree" >&2
    echo "  vanetza/facilities/CMakeLists.txt: $map_sources map_functions.cpp match(es)" >&2
    echo "  vanetza/facilities/tests/CMakeLists.txt: $map_tests add_gtest(MapFunctions ...) match(es)" >&2
    exit 1
fi
if [ "$priority_sources" -ne "$priority_tests" ]; then
    echo "select-functions-patch: inconsistent priority registration in $tree" >&2
    echo "  vanetza/facilities/CMakeLists.txt: $priority_sources priority_functions.cpp match(es)" >&2
    echo "  vanetza/facilities/tests/CMakeLists.txt: $priority_tests add_gtest(PriorityFunctions ...) match(es)" >&2
    exit 1
fi

if [ "$map_sources" -eq 1 ] && [ "$priority_sources" -eq 1 ]; then
    state=none
elif [ "$map_sources" -eq 1 ]; then
    state=priority-only
elif [ "$priority_sources" -eq 0 ]; then
    state=full
else
    # priority is registered but the MAP helpers it builds on are not
    echo "select-functions-patch: priority functions are registered without the MAP helpers in $tree" >&2
    echo "  vanetza/facilities/CMakeLists.txt: $map_sources map_functions.cpp match(es)" >&2
    echo "  vanetza/facilities/tests/CMakeLists.txt: $map_tests add_gtest(MapFunctions ...) match(es)" >&2
    exit 1
fi

case $mode in
    state)
        printf '%s\n' "$state"
        ;;
    patch)
        case $state in
            full)
                patch_file=$root/patches/0002-facilities-add-spat-map-priority-functions.patch
                ;;
            priority-only)
                patch_file=$root/patches/0002-facilities-priority-only.patch
                ;;
            none)
                patch_file=
                ;;
        esac
        if [ -z "$patch_file" ]; then
            echo "select-functions-patch: MAP and priority are both registered, no patch needed" >&2
            exit 0
        fi
        if [ ! -f "$patch_file" ]; then
            echo "select-functions-patch: selected patch is missing: $patch_file" >&2
            exit 1
        fi
        printf '%s\n' "$patch_file"
        ;;
esac
