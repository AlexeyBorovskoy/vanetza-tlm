#include <vanetza/asn1/asn1c_wrapper.hpp>
#include <vanetza/asn1/its/ConnectsToList.h>
#include <vanetza/asn1/its/GenericLane.h>
#include <vanetza/asn1/its/IntersectionGeometry.h>
#include <vanetza/asn1/its/IntersectionGeometryList.h>
#include <vanetza/asn1/its/IntersectionState.h>
#include <vanetza/asn1/its/IntersectionStatusObject.h>
#include <vanetza/asn1/its/MapData.h>
#include <vanetza/asn1/its/MovementEvent.h>
#include <vanetza/asn1/its/MovementPhaseState.h>
#include <vanetza/asn1/its/MovementState.h>
#include <vanetza/asn1/its/NodeListXY.h>
#include <vanetza/asn1/its/SPAT.h>
#include <vanetza/asn1/its/TimeChangeDetails.h>
#include <vanetza/facilities/map_spat_validation.hpp>
#include <boost/optional/optional.hpp>
#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace vanetza
{
namespace facilities
{

namespace
{

// TimeMark values with special meaning (ISO TS 19091 TimeMark, C2C-CC RS 2077 pTimeMark*)
const long time_mark_out_of_range = 36000;
const long time_mark_unknown = 36001;
// a resolved TimeMark beyond any value of the next hour
const long time_mark_infinite = 3 * time_mark_out_of_range;

// C2C-CC RS 2077 pMaxNoOfNodesPerLane
const int max_nodes_per_lane = 18;

bool car2car(ValidationProfile profile)
{
    return profile == ValidationProfile::Car2Car || profile == ValidationProfile::Combined;
}

bool croads(ValidationProfile profile)
{
    return profile == ValidationProfile::CRoads || profile == ValidationProfile::Combined;
}

bool any_additional_profile(ValidationProfile profile)
{
    return profile != ValidationProfile::Standard;
}

bool bit(const BIT_STRING_t& bits, int n)
{
    const std::size_t byte = n / 8;
    return bits.buf && byte < bits.size && (bits.buf[byte] & (0x80 >> (n % 8))) != 0;
}

std::string item(const std::string& path, const char* field, int index)
{
    return path + "." + field + "[" + std::to_string(index) + "]";
}

bool check_constraints(asn_TYPE_descriptor_t& type, const void* structure, const std::string& path,
        ValidationResult& result)
{
    std::string error;
    if (!asn1::validate(type, structure, error)) {
        result.add(Severity::Error, "ASN.1", path, error);
        return false;
    }
    return true;
}

// SIZE of a SEQUENCE OF and its elements, which asn1c constraint checking does not cover
template<typename LIST>
bool check_list(const LIST& list, int min, int max, const std::string& path, ValidationResult& result)
{
    bool valid = true;
    if (list.list.count < min || list.list.count > max) {
        result.add(Severity::Error, "ASN.1", path, std::to_string(list.list.count) + " elements, SIZE(" +
            std::to_string(min) + ".." + std::to_string(max) + ")");
        valid = false;
    }
    for (int i = 0; i < list.list.count; ++i) {
        if (!list.list.array[i]) {
            result.add(Severity::Error, "ASN.1", path + "[" + std::to_string(i) + "]", "empty list element");
            valid = false;
        }
    }
    return valid;
}

// ISO TS 19091: IntersectionGeometryList SIZE(1..32), LaneList SIZE(1..255), NodeSetXY SIZE(2..63),
// ConnectsToList SIZE(1..16)
bool check_structure(const MapData& map, ValidationResult& result)
{
    if (!map.intersections) {
        return true;
    }
    bool valid = check_list(*map.intersections, 1, 32, "MapData.intersections", result);
    for (int i = 0; i < map.intersections->list.count; ++i) {
        const IntersectionGeometry* geometry = map.intersections->list.array[i];
        if (!geometry) {
            continue;
        }
        const std::string path = item("MapData", "intersections", i);
        valid = check_list(geometry->laneSet, 1, 255, path + ".laneSet", result) && valid;
        for (int j = 0; j < geometry->laneSet.list.count; ++j) {
            const GenericLane* lane = geometry->laneSet.list.array[j];
            if (!lane) {
                continue;
            }
            const std::string lane_path = item(path, "laneSet", j);
            if (lane->nodeList.present == NodeListXY_PR_nodes) {
                valid = check_list(lane->nodeList.choice.nodes, 2, 63, lane_path + ".nodeList.nodes", result) && valid;
            }
            if (lane->connectsTo) {
                valid = check_list(*lane->connectsTo, 1, 16, lane_path + ".connectsTo", result) && valid;
            }
        }
    }
    return valid;
}

// ISO TS 19091: IntersectionStateList SIZE(1..32), MovementList SIZE(1..255), MovementEventList SIZE(1..16)
bool check_structure(const SPAT& spat, ValidationResult& result)
{
    bool valid = check_list(spat.intersections, 1, 32, "SPAT.intersections", result);
    for (int i = 0; i < spat.intersections.list.count; ++i) {
        const IntersectionState* state = spat.intersections.list.array[i];
        if (!state) {
            continue;
        }
        const std::string path = item("SPAT", "intersections", i);
        valid = check_list(state->states, 1, 255, path + ".states", result) && valid;
        for (int j = 0; j < state->states.list.count; ++j) {
            const MovementState* movement = state->states.list.array[j];
            if (movement) {
                const std::string movement_path = item(path, "states", j) + ".state-time-speed";
                valid = check_list(movement->state_time_speed, 1, 16, movement_path, result) && valid;
            }
        }
    }
    return valid;
}

struct ReferenceId
{
    bool has_region;
    long region;
    long id;

    bool operator==(const ReferenceId& other) const
    {
        return has_region == other.has_region && (!has_region || region == other.region) && id == other.id;
    }
};

ReferenceId reference_id(const IntersectionReferenceID_t& id)
{
    return ReferenceId { id.region != nullptr, id.region ? *id.region : 0, id.id };
}

std::string describe(const ReferenceId& id)
{
    return (id.has_region ? std::to_string(id.region) + "/" : std::string()) + std::to_string(id.id);
}

// RS_ARSM_11 (region present) and RS_ARSM_12 (id tuple unique within the message)
void check_reference(const IntersectionReferenceID_t& id, const std::string& path, std::vector<ReferenceId>& seen,
        ValidationProfile profile, ValidationResult& result)
{
    if (!car2car(profile)) {
        return;
    }
    const ReferenceId reference = reference_id(id);
    if (!reference.has_region) {
        result.add(Severity::Error, "RS_ARSM_11", path, "IntersectionReferenceID without region");
    }
    for (const ReferenceId& other : seen) {
        if (other == reference) {
            result.add(Severity::Error, "RS_ARSM_12", path,
                "IntersectionReferenceID " + describe(reference) + " used twice");
        }
    }
    seen.push_back(reference);
}

struct MapContext
{
    ValidationProfile profile;
    bool fragment; /**< MapData carries layerID, connected lanes may be in another fragment */
};

void check_lane(const GenericLane& lane, const std::string& path, const MapContext& context, ValidationResult& result)
{
    if (lane.laneID == 0 || lane.laneID == 255) {
        result.add(Severity::Warning, "LaneID value", path,
            "LaneID " + std::to_string(lane.laneID) + " means not known (0) or is reserved (255)");
    }
    if (any_additional_profile(context.profile) && lane.nodeList.present == NodeListXY_PR_computed) {
        result.add(Severity::Error, "RS_ARSM_118", path + ".nodeList", "computed lane instead of nodes");
    }
    if (!car2car(context.profile)) {
        return;
    }
    if (lane.maneuvers) {
        result.add(Severity::Error, "RS_ARSM_117", path + ".maneuvers", "maneuvers on lane level");
    }
    if (lane.nodeList.present == NodeListXY_PR_nodes && lane.nodeList.choice.nodes.list.count > max_nodes_per_lane) {
        result.add(Severity::Error, "RS_ARSM_35", path + ".nodeList",
            std::to_string(lane.nodeList.choice.nodes.list.count) + " nodes, at most " +
            std::to_string(max_nodes_per_lane) + " allowed");
    }

    const BIT_STRING_t& direction = lane.laneAttributes.directionalUse;
    const bool ingress = bit(direction, LaneDirection_ingressPath);
    const bool egress = bit(direction, LaneDirection_egressPath);
    const bool both_approaches = lane.ingressApproach && lane.egressApproach;
    const bool one_approach = (lane.ingressApproach != nullptr) != (lane.egressApproach != nullptr);
    if (ingress != egress && !one_approach) {
        result.add(Severity::Error, "RS_ARSM_16", path, "unidirectional lane without exactly one approach");
    }
    const LaneTypeAttributes_t& type = lane.laneAttributes.laneType;
    const bool crossing = type.present == LaneTypeAttributes_PR_crosswalk ||
        type.present == LaneTypeAttributes_PR_bikeLane;
    if (crossing && ingress && egress && !both_approaches) {
        result.add(Severity::Error, "RS_ARSM_17", path,
            "bidirectional crossing lane without ingress and egress approach");
    }
}

void check_target(const GenericLane& lane, long target, const std::string& path, const std::set<long>& lane_ids,
        const MapContext& context, ValidationResult& result)
{
    if (target == 0 || target == 255) {
        result.add(Severity::Warning, "connectingLane", path,
            "target LaneID " + std::to_string(target) + " means not known (0) or is reserved (255)");
    } else if (lane_ids.count(target) == 0) {
        result.add(context.fragment ? Severity::Warning : Severity::Error, "connectingLane", path,
            "target LaneID " + std::to_string(target) + " does not exist" +
            (context.fragment ? std::string(" in this MapData fragment") : std::string()));
    }
    if (target == lane.laneID && lane.laneAttributes.laneType.present == LaneTypeAttributes_PR_crosswalk) {
        result.add(Severity::Warning, "crosswalk self-loop", path,
            "crosswalk connected to itself, a compatibility convention not defined by ISO TS 19091");
    }
}

// direction bits of a maneuver, -1 without maneuver
int directions(const Connection_t& connection)
{
    if (!connection.connectingLane.maneuver) {
        return -1;
    }
    int value = 0;
    for (int n = AllowedManeuvers_maneuverStraightAllowed; n <= AllowedManeuvers_maneuverUTurnAllowed; ++n) {
        value |= bit(*connection.connectingLane.maneuver, n) ? 1 << n : 0;
    }
    return value;
}

void check_maneuver(const Connection_t& connection, const std::string& path, ValidationResult& result)
{
    if (!connection.connectingLane.maneuver) {
        result.add(Severity::Error, "RS_ARSM_21", path + ".connectingLane", "maneuver missing");
        return;
    }
    const AllowedManeuvers_t& maneuver = *connection.connectingLane.maneuver;
    int count = 0;
    for (int n = AllowedManeuvers_maneuverStraightAllowed; n <= AllowedManeuvers_maneuverUTurnAllowed; ++n) {
        count += bit(maneuver, n) ? 1 : 0;
    }
    if (count != 1) {
        result.add(Severity::Error, "RS_ARSM_22", path + ".connectingLane.maneuver",
            std::to_string(count) + " direction bits set instead of exactly one");
    }
    if (bit(maneuver, AllowedManeuvers_maneuverLeftTurnOnRedAllowed) ||
            bit(maneuver, AllowedManeuvers_maneuverRightTurnOnRedAllowed) ||
            bit(maneuver, AllowedManeuvers_maneuverLaneChangeAllowed)) {
        result.add(Severity::Error, "RS_ARSM_24", path + ".connectingLane.maneuver",
            "turn on red or lane change bit used");
    }
}

void check_connections(const GenericLane& lane, const std::string& path, const std::set<long>& lane_ids,
        std::map<long, long>& connection_groups, const MapContext& context, ValidationResult& result)
{
    if (!lane.connectsTo) {
        return;
    }
    std::set<std::pair<long, int>> seen; // local target lane and direction bits
    for (int k = 0; k < lane.connectsTo->list.count; ++k) {
        const std::string connection_path = item(path, "connectsTo", k);
        const Connection_t* connection = lane.connectsTo->list.array[k];
        const long target = connection->connectingLane.lane;
        const bool local = connection->remoteIntersection == nullptr;
        if (local) {
            check_target(lane, target, connection_path + ".connectingLane.lane", lane_ids, context, result);
        }
        if (car2car(context.profile)) {
            check_maneuver(*connection, connection_path, result);
            if (local && directions(*connection) >= 0 && !seen.insert({ target, directions(*connection) }).second) {
                result.add(Severity::Error, "RS_ARSM_20", connection_path,
                    "second connection to LaneID " + std::to_string(target) + " with the same direction");
            }
            if (!connection->signalGroup) {
                result.add(Severity::Warning, "RS_ARSM_48", connection_path, "connection without signalGroup");
            }
        }
        if (croads(context.profile) && !connection->connectionID) {
            result.add(Severity::Error, "C-Roads connectionID", connection_path, "connectionID missing (Annex 7.1)");
        }
        if (connection->connectionID) {
            const long group = connection->signalGroup ? *connection->signalGroup : -1;
            const auto inserted = connection_groups.emplace(*connection->connectionID, group);
            if (!inserted.second && inserted.first->second != group) {
                result.add(Severity::Warning, "connectionID", connection_path,
                    "connectionID " + std::to_string(*connection->connectionID) + " used with different signal groups");
            }
        }
    }
}

void check_geometry(const IntersectionGeometry& geometry, const std::string& path, const MapContext& context,
        ValidationResult& result)
{
    if (car2car(context.profile) && !geometry.laneWidth) {
        result.add(Severity::Error, "RS_ARSM_14", path, "laneWidth missing");
    }

    std::set<long> lane_ids;
    for (int j = 0; j < geometry.laneSet.list.count; ++j) {
        const GenericLane* lane = geometry.laneSet.list.array[j];
        if (!lane_ids.insert(lane->laneID).second) {
            result.add(Severity::Error, "LaneID unique", item(path, "laneSet", j),
                "duplicate LaneID " + std::to_string(lane->laneID));
        }
    }

    std::map<long, long> connection_groups; // connection ID -> signal group of its first use, -1 without
    for (int j = 0; j < geometry.laneSet.list.count; ++j) {
        const GenericLane* lane = geometry.laneSet.list.array[j];
        const std::string lane_path = item(path, "laneSet", j);
        check_lane(*lane, lane_path, context, result);
        check_connections(*lane, lane_path, lane_ids, connection_groups, context, result);
    }
}

// TimeMark relative to the begin of the minute given by moy (C2C-CC RS 2077 RS_ARSM_54)
boost::optional<long> resolve(long mark, const boost::optional<long>& minute_mark)
{
    if (mark == time_mark_unknown) {
        return boost::none;
    }
    if (mark == time_mark_out_of_range) {
        return time_mark_infinite;
    }
    return minute_mark && mark < *minute_mark ? mark + time_mark_out_of_range : mark;
}

boost::optional<long> resolve(const TimeMark_t* mark, const boost::optional<long>& minute_mark)
{
    if (!mark) {
        return boost::none;
    }
    return resolve(*mark, minute_mark);
}

struct StateContext
{
    ValidationProfile profile;
    boost::optional<long> minute_mark; /**< first TimeMark of the minute given by moy */
    bool fixed_time;
    bool traffic_dependent;
};

void check_timing(const TimeChangeDetails_t& timing, const std::string& path, const StateContext& context,
        ValidationResult& result)
{
    if (croads(context.profile) && !timing.maxEndTime) {
        result.add(Severity::Error, "C-Roads maxEndTime", path, "maxEndTime missing (Annex 7.2)");
    } else if (car2car(context.profile) && context.traffic_dependent && !timing.maxEndTime) {
        result.add(Severity::Error, "RS_ARSM_57", path, "maxEndTime missing for traffic dependent operation");
    }
    if (timing.likelyTime && !timing.confidence) {
        if (car2car(context.profile)) {
            result.add(Severity::Error, "RS_ARSM_115", path, "likelyTime without confidence");
        } else if (croads(context.profile)) {
            result.add(Severity::Error, "C-Roads confidence", path, "likelyTime without confidence (Annex 7.2)");
        }
    }
    if (!car2car(context.profile)) {
        return;
    }

    if (context.traffic_dependent && !timing.likelyTime) {
        result.add(Severity::Error, "RS_ARSM_64", path, "likelyTime missing for traffic dependent operation");
    }
    if (timing.minEndTime == time_mark_unknown) {
        result.add(Severity::Error, "RS_ARSM_56", path + ".minEndTime", "minEndTime unknown");
    }
    if (timing.maxEndTime && *timing.maxEndTime == time_mark_unknown) {
        result.add(Severity::Error, "RS_ARSM_60", path + ".maxEndTime", "maxEndTime unknown");
    }
    if (timing.likelyTime && *timing.likelyTime == time_mark_unknown) {
        result.add(Severity::Error, "RS_ARSM_66", path + ".likelyTime", "likelyTime unknown");
    }

    const boost::optional<long> min = resolve(timing.minEndTime, context.minute_mark);
    const boost::optional<long> likely = resolve(timing.likelyTime, context.minute_mark);
    const boost::optional<long> max = resolve(timing.maxEndTime, context.minute_mark);
    if ((min && likely && *min > *likely) || (likely && max && *likely > *max) || (min && max && *min > *max)) {
        result.add(Severity::Error, "RS_ARSM_65", path, "not minEndTime <= likelyTime <= maxEndTime");
    }

    // fixed time operation with a known time of change: the present end times are equal
    if (context.fixed_time && min && *min != time_mark_infinite) {
        const bool likely_differs = likely && *likely != *min;
        const bool max_differs = max && *max != time_mark_infinite && *max != *min;
        if (likely_differs || max_differs) {
            result.add(Severity::Error, "RS_ARSM_61", path,
                "minEndTime, likelyTime and maxEndTime differ in fixed time operation");
        }
    }
}

// MovementPhaseState values strictly allowing or prohibiting to proceed (C2C-CC RS 2077 RS_ARSM_95)
bool phase(long state)
{
    return state == MovementPhaseState_stop_Then_Proceed ||
        state == MovementPhaseState_stop_And_Remain ||
        state == MovementPhaseState_permissive_Movement_Allowed ||
        state == MovementPhaseState_protected_Movement_Allowed;
}

void check_event_order(const std::vector<const MovementEvent_t*>& events, const std::string& path,
        const StateContext& context, ValidationResult& result)
{
    // RS_ARSM_78: chronological order with respect to minEndTime
    boost::optional<long> previous;
    for (std::size_t k = 0; k < events.size(); ++k) {
        if (!events[k]->timing) {
            continue;
        }
        const boost::optional<long> min = resolve(events[k]->timing->minEndTime, context.minute_mark);
        if (min && previous && *min < *previous) {
            result.add(Severity::Error, "RS_ARSM_78", item(path, "state-time-speed", k),
                "minEndTime earlier than the one of a previous event");
        }
        if (min) {
            previous = min;
        }
    }

    // RS_ARSM_120: timing for every event preceding a phase event
    std::size_t last_phase = 0;
    for (std::size_t k = 1; k < events.size(); ++k) {
        last_phase = phase(events[k]->eventState) ? k : last_phase;
    }
    for (std::size_t k = 0; k < last_phase; ++k) {
        if (!events[k]->timing) {
            result.add(Severity::Error, "RS_ARSM_120", item(path, "state-time-speed", k),
                "timing missing for an event preceding a phase");
        }
    }

    // RS_ARSM_79: the next phase is listed, unless the listed events end beyond the TimeMark horizon
    const MovementEvent_t& last = *events.back();
    const bool beyond_horizon = last.timing && last.timing->minEndTime == time_mark_out_of_range;
    if (last_phase == 0 && !beyond_horizon) {
        result.add(Severity::Error, "RS_ARSM_79", path + ".state-time-speed", "events do not reach the next phase");
    }
}

void check_movement(const MovementState& movement, const std::string& path, const StateContext& context,
        ValidationResult& result)
{
    const std::vector<const MovementEvent_t*> events(movement.state_time_speed.list.array,
        movement.state_time_speed.list.array + movement.state_time_speed.list.count);

    for (std::size_t k = 0; k < events.size(); ++k) {
        const MovementEvent_t& event = *events[k];
        const std::string event_path = item(path, "state-time-speed", k);
        const bool allowed = event.eventState == MovementPhaseState_permissive_Movement_Allowed ||
            event.eventState == MovementPhaseState_protected_Movement_Allowed;
        if (movement.signalGroup == 255 && !allowed) {
            result.add(Severity::Error, "SignalGroupID 255", event_path,
                "signal group 255 denotes a permanent green movement state");
        }
        if (k > 0 && events[k - 1]->eventState == event.eventState) {
            result.add(Severity::Warning, "eventState sequence", event_path, "same eventState as the previous event");
        }
        if (car2car(context.profile) && event.eventState == MovementPhaseState_dark) {
            result.add(Severity::Error, "RS_ARSM_72", event_path + ".eventState", "eventState dark");
        }
        if (event.timing) {
            check_timing(*event.timing, event_path + ".timing", context, result);
        }
    }

    if (car2car(context.profile)) {
        check_event_order(events, path, context, result);
    }
}

void check_status(const IntersectionState& state, const std::string& path, ValidationResult& result)
{
    int modes = 0;
    bool other = false;
    for (int n = 0; n < static_cast<int>(state.status.size) * 8; ++n) {
        const bool mode = n >= IntersectionStatusObject_fixedTimeOperation && n <= IntersectionStatusObject_off;
        modes += mode && bit(state.status, n) ? 1 : 0;
        other |= !mode && bit(state.status, n);
    }
    if (other) {
        result.add(Severity::Error, "RS_ARSM_69", path + ".status", "status bits other than 5 to 9 set");
    }
    if (modes != 1) {
        result.add(Severity::Error, "RS_ARSM_70", path + ".status",
            std::to_string(modes) + " operation mode bits set instead of exactly one");
    }
}

void check_state(const IntersectionState& state, const std::string& path, ValidationProfile profile,
        ValidationResult& result)
{
    if (car2car(profile)) {
        check_status(state, path, result);
    }
    if (any_additional_profile(profile) && !state.moy) {
        result.add(Severity::Error, "moy", path, "moy missing (Annex 7.2)");
    }
    if (croads(profile) && !state.timeStamp) {
        result.add(Severity::Error, "C-Roads timeStamp", path, "timeStamp missing (Annex 7.2)");
    }

    StateContext context;
    context.profile = profile;
    context.fixed_time = bit(state.status, IntersectionStatusObject_fixedTimeOperation);
    context.traffic_dependent = bit(state.status, IntersectionStatusObject_trafficDependentOperation);
    if (state.moy) {
        context.minute_mark = (*state.moy % 60) * 600;
    }

    std::set<long> groups;
    for (int j = 0; j < state.states.list.count; ++j) {
        const std::string movement_path = item(path, "states", j);
        const MovementState* movement = state.states.list.array[j];
        if (!groups.insert(movement->signalGroup).second) {
            result.add(Severity::Warning, "SignalGroupID unique", movement_path,
                "second MovementState for signal group " + std::to_string(movement->signalGroup));
        }
        if (movement->signalGroup == 0) {
            result.add(Severity::Warning, "SignalGroupID value", movement_path, "signal group 0 means not known");
        }
        check_movement(*movement, movement_path, context, result);
    }
}

// signal group -> path of the first connection using it
std::map<long, std::string> connection_groups(const IntersectionGeometry& geometry, const std::string& path)
{
    std::map<long, std::string> groups;
    for (int j = 0; j < geometry.laneSet.list.count; ++j) {
        const GenericLane* lane = geometry.laneSet.list.array[j];
        for (int k = 0; lane->connectsTo && k < lane->connectsTo->list.count; ++k) {
            const Connection_t* connection = lane->connectsTo->list.array[k];
            if (connection->signalGroup) {
                const std::string connection_path = item(item(path, "laneSet", j), "connectsTo", k);
                groups.emplace(*connection->signalGroup, connection_path + ".signalGroup");
            }
        }
    }
    return groups;
}

std::set<long> movement_groups(const IntersectionState& state)
{
    std::set<long> groups;
    for (int j = 0; j < state.states.list.count; ++j) {
        groups.insert(state.states.list.array[j]->signalGroup);
    }
    return groups;
}

using GeometryEntry = std::pair<const IntersectionGeometry*, std::string>; // with its path
using StateEntry = std::pair<const IntersectionState*, std::string>; // with its path

// RS identifier in the Car2Car profile, project identifier otherwise
struct PairRule
{
    Severity severity;
    const char* rule;
};

PairRule pair_rule(ValidationProfile profile, const char* requirement, const char* project_rule)
{
    return car2car(profile) ? PairRule { Severity::Error, requirement } : PairRule { Severity::Warning, project_rule };
}

} // namespace

ValidationResult validate_map(const MapData& map, ValidationProfile profile)
{
    ValidationResult result;
    if (!check_constraints(asn_DEF_MapData, &map, "MapData", result) || !check_structure(map, result)) {
        return result;
    }
    if (map.msgIssueRevision != 0) {
        result.add(Severity::Error, "msgIssueRevision", "MapData.msgIssueRevision",
            "shall be 0, revisions are given per intersection");
    }
    if (map.layerType) {
        result.add(Severity::Error, "layerType", "MapData.layerType", "shall not be used");
    }
    if (!map.intersections) {
        if (croads(profile)) {
            result.add(Severity::Error, "C-Roads intersections", "MapData", "intersections missing (Annex 7.1)");
        }
        return result;
    }

    const MapContext context { profile, map.layerID != nullptr };
    std::vector<ReferenceId> seen;
    for (int i = 0; i < map.intersections->list.count; ++i) {
        const std::string path = item("MapData", "intersections", i);
        const IntersectionGeometry* geometry = map.intersections->list.array[i];
        check_reference(geometry->id, path + ".id", seen, profile, result);
        check_geometry(*geometry, path, context, result);
    }
    return result;
}

ValidationResult validate_spat(const SPAT& spat, ValidationProfile profile)
{
    ValidationResult result;
    if (!check_constraints(asn_DEF_SPAT, &spat, "SPAT", result) || !check_structure(spat, result)) {
        return result;
    }

    std::vector<ReferenceId> seen;
    for (int i = 0; i < spat.intersections.list.count; ++i) {
        const std::string path = item("SPAT", "intersections", i);
        const IntersectionState* state = spat.intersections.list.array[i];
        check_reference(state->id, path + ".id", seen, profile, result);
        check_state(*state, path, profile, result);
    }
    return result;
}

ValidationResult validate_map_spat(const MapData& map, const SPAT& spat, ValidationProfile profile)
{
    ValidationResult result;
    ValidationResult structure;
    if (!check_constraints(asn_DEF_MapData, &map, "MapData", structure) || !check_structure(map, structure) ||
            !check_constraints(asn_DEF_SPAT, &spat, "SPAT", structure) || !check_structure(spat, structure)) {
        return result;
    }

    std::vector<GeometryEntry> geometries;
    for (int g = 0; map.intersections && g < map.intersections->list.count; ++g) {
        geometries.emplace_back(map.intersections->list.array[g], item("MapData", "intersections", g));
    }
    std::vector<StateEntry> states;
    for (int i = 0; i < spat.intersections.list.count; ++i) {
        states.emplace_back(spat.intersections.list.array[i], item("SPAT", "intersections", i));
    }

    const bool fragment = map.layerID != nullptr;
    const PairRule state_without_geometry = pair_rule(profile, "RS_ARSM_68", "map/spat intersection");
    const PairRule geometry_without_state = pair_rule(profile, "RS_ARSM_13", "map/spat intersection");
    const PairRule map_group_without_state = pair_rule(profile, "RS_ARSM_49", "map/spat signal group");
    const PairRule state_group_without_map = pair_rule(profile, "RS_ARSM_75", "map/spat signal group");

    for (const auto& state : states) {
        const ReferenceId id = reference_id(state.first->id);
        const auto geometry = std::find_if(geometries.begin(), geometries.end(),
            [&id](const GeometryEntry& g) { return reference_id(g.first->id) == id; });
        if (geometry == geometries.end() && fragment) {
            continue; // the intersection may be described by another fragment
        }
        if (geometry == geometries.end()) {
            result.add(state_without_geometry.severity, state_without_geometry.rule, state.second + ".id",
                "no IntersectionGeometry with IntersectionReferenceID " + describe(id));
            continue;
        }
        if (state.first->revision != geometry->first->revision) {
            result.add(Severity::Error, "revision", state.second + ".revision",
                "revision " + std::to_string(state.first->revision) + " differs from " + geometry->second +
                ".revision " + std::to_string(geometry->first->revision));
        }
        if (fragment) {
            continue; // further signal groups may be described by other fragments
        }
        const std::map<long, std::string> map_groups = connection_groups(*geometry->first, geometry->second);
        for (int j = 0; j < state.first->states.list.count; ++j) {
            const MovementState* movement = state.first->states.list.array[j];
            if (map_groups.count(movement->signalGroup) == 0) {
                result.add(state_group_without_map.severity, state_group_without_map.rule,
                    item(state.second, "states", j), "signal group " + std::to_string(movement->signalGroup) +
                    " is not used by any connection of " + geometry->second);
            }
        }
    }

    for (const auto& geometry : geometries) {
        const ReferenceId id = reference_id(geometry.first->id);
        const auto state = std::find_if(states.begin(), states.end(),
            [&id](const StateEntry& s) { return reference_id(s.first->id) == id; });
        if (state == states.end()) {
            result.add(geometry_without_state.severity, geometry_without_state.rule, geometry.second + ".id",
                "no IntersectionState with IntersectionReferenceID " + describe(id));
            continue;
        }
        const std::set<long> state_groups = movement_groups(*state->first);
        for (const auto& group : connection_groups(*geometry.first, geometry.second)) {
            if (state_groups.count(group.first) == 0) {
                result.add(map_group_without_state.severity, map_group_without_state.rule, group.second,
                    "signal group " + std::to_string(group.first) + " has no MovementState in " + state->second);
            }
        }
    }
    return result;
}

} // namespace facilities
} // namespace vanetza
