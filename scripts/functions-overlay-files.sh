#!/bin/sh
# List the facilities helper files of the overlay for a Vanetza tree.
#
# The overlay ships its own map_functions.hpp/.cpp, priority_functions.hpp/.cpp and
# their tests.  Where the tree already registers and ships a helper, copying the
# overlay file over it would undo the upstream implementation and its fixes, so only
# the helpers the tree still lacks are copied and the upstream ones are preserved.
#
# The registration state is decided once, in select-functions-patch.sh; this script
# only maps that state to file names:
#   full           copy MAP and priority helpers, preserve none
#   priority-only  copy the priority helpers, preserve the MAP helpers
#   none           copy none, preserve MAP and priority helpers
#
# Usage: functions-overlay-files.sh (--copy|--preserve) <vanetza-tree>
#
#   --copy      overlay files to copy into the tree, one path per line
#   --preserve  overlay files that must keep the upstream content, one path per line
#
# The output is empty when no file of that kind applies; the exit status is still 0.
set -eu

root=$(cd "$(dirname "$0")/.." && pwd)

usage() {
    echo "usage: $0 (--copy|--preserve) <vanetza-tree>" >&2
    exit 2
}

case ${1:-} in
    --copy|--preserve)
        mode=$1
        shift
        ;;
    *)
        usage
        ;;
esac

if [ "$#" -ne 1 ]; then
    usage
fi

state=$(sh "$root/scripts/select-functions-patch.sh" --state "$1") || exit 1

list_map() {
    printf '%s\n' \
        vanetza/facilities/map_functions.hpp \
        vanetza/facilities/map_functions.cpp \
        vanetza/facilities/tests/map_functions.cpp
}

list_priority() {
    printf '%s\n' \
        vanetza/facilities/priority_functions.hpp \
        vanetza/facilities/priority_functions.cpp \
        vanetza/facilities/tests/priority_functions.cpp
}

case "$mode:$state" in
    --copy:full)
        list_map
        list_priority
        ;;
    --copy:priority-only)
        list_priority
        ;;
    --copy:none)
        ;;
    --preserve:full)
        ;;
    --preserve:priority-only)
        list_map
        ;;
    --preserve:none)
        list_map
        list_priority
        ;;
esac
