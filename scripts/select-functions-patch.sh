#!/bin/sh
# Select the variant of patch 0002 for the facilities CMake lists.
#
# The pinned Vanetza base does not know the MAP helpers yet, so the original patch
# registers map_functions.cpp / MapFunctions and priority_functions.cpp /
# PriorityFunctions together.  Current upstream already registers the MAP helpers,
# so only the priority registration may be added there; the original patch fails on
# both facilities CMake lists in that case.
#
# Usage: select-functions-patch.sh <vanetza-tree>
#
# Prints the patch file to apply.  Fails if the MAP registration is inconsistent
# between the source list and the test list, or if priority is already registered;
# never guesses between the variants.
set -eu

root=$(cd "$(dirname "$0")/.." && pwd)

if [ "$#" -ne 1 ]; then
    echo "usage: $0 <vanetza-tree>" >&2
    exit 2
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

if [ "$priority_sources" -ne 0 ] || [ "$priority_tests" -ne 0 ]; then
    echo "select-functions-patch: priority functions are already registered in $tree" >&2
    echo "  sources: $priority_sources match(es), tests: $priority_tests match(es)" >&2
    exit 1
fi

if [ "$map_sources" -eq 1 ] && [ "$map_tests" -eq 1 ]; then
    # current upstream: MAP is registered once already, add priority only
    patch=$root/patches/0002-facilities-priority-only.patch
elif [ "$map_sources" -eq 0 ] && [ "$map_tests" -eq 0 ]; then
    # pinned upstream: register MAP and priority together
    patch=$root/patches/0002-facilities-add-spat-map-priority-functions.patch
else
    echo "select-functions-patch: inconsistent MAP registration in $tree" >&2
    echo "  vanetza/facilities/CMakeLists.txt: $map_sources map_functions.cpp match(es)" >&2
    echo "  vanetza/facilities/tests/CMakeLists.txt: $map_tests add_gtest(MapFunctions ...) match(es)" >&2
    exit 1
fi

if [ ! -f "$patch" ]; then
    echo "select-functions-patch: selected patch is missing: $patch" >&2
    exit 1
fi

printf '%s\n' "$patch"
