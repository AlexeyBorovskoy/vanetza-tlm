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
#include <vanetza/asn1/its/SPAT.h>
#include <vanetza/asn1/its/TimeChangeDetails.h>
#include <vanetza/facilities/map_spat_validation.hpp>
#include <boost/optional/optional.hpp>
#include <map>
#include <set>
#include <string>
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

bool car2car(ValidationProfile profile)
{
    return profile == ValidationProfile::Car2Car || profile == ValidationProfile::Combined;
}

bool croads(ValidationProfile profile)
{
    return profile == ValidationProfile::CRoads || profile == ValidationProfile::Combined;
}

// severity of rules which are only warnings unless a profile makes them mandatory
Severity mandatory_if(bool mandatory)
{
    return mandatory ? Severity::Error : Severity::Warning;
}

bool bit(const BIT_STRING_t& bits, int n)
{
    const std::size_t byte = n / 8;
    return byte < bits.size && (bits.buf[byte] & (0x80 >> (n % 8))) != 0;
}

std::string item(const std::string& path, const char* field, int index)
{
    return path + "." + field + "[" + std::to_string(index) + "]";
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

void check_constraints(asn_TYPE_descriptor_t& type, const void* structure, const std::string& path,
        ValidationResult& result)
{
    std::string error;
    if (!asn1::validate(type, structure, error)) {
        result.add(Severity::Error, "ASN.1", path, error);
    }
}

// RS_ARSM_11 (region present) and RS_ARSM_12 (id tuple unique), Car2Car profile only
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

void check_lane(const GenericLane& lane, const std::string& path, ValidationProfile profile, ValidationResult& result)
{
    if (lane.laneID == 0 || lane.laneID == 255) {
        result.add(Severity::Warning, "LaneID value", path,
            "LaneID " + std::to_string(lane.laneID) + " means not known (0) or is reserved (255)");
    }
    if (!car2car(profile)) {
        return;
    }
    if (lane.maneuvers) {
        result.add(Severity::Error, "RS_ARSM_117", path + ".maneuvers", "maneuvers on lane level");
    }
    const LaneTypeAttributes_t& type = lane.laneAttributes.laneType;
    const bool crossing = type.present == LaneTypeAttributes_PR_crosswalk ||
        type.present == LaneTypeAttributes_PR_bikeLane;
    const BIT_STRING_t& direction = lane.laneAttributes.directionalUse;
    const bool bidirectional = bit(direction, LaneDirection_ingressPath) && bit(direction, LaneDirection_egressPath);
    if (crossing && bidirectional && (!lane.ingressApproach || !lane.egressApproach)) {
        result.add(Severity::Error, "RS_ARSM_17", path,
            "bidirectional crossing lane without ingress and egress approach");
    }
}

void check_connection(const GenericLane& lane, const Connection_t& connection, const std::string& path,
        const std::set<long>& lane_ids, ValidationProfile profile, ValidationResult& result)
{
    const long target = connection.connectingLane.lane;
    const bool local = connection.remoteIntersection == nullptr;
    if (local && lane_ids.count(target) == 0) {
        result.add(Severity::Error, "connectingLane", path + ".connectingLane.lane",
            "target LaneID " + std::to_string(target) + " does not exist");
    }
    if (local && target == lane.laneID && lane.laneAttributes.laneType.present == LaneTypeAttributes_PR_crosswalk) {
        result.add(Severity::Warning, "crosswalk self-loop", path,
            "crosswalk connected to itself, a compatibility convention not defined by ISO TS 19091");
    }
    if (!car2car(profile)) {
        return;
    }
    if (connection.connectingLane.maneuver) {
        int directions = 0;
        for (int n = AllowedManeuvers_maneuverStraightAllowed; n <= AllowedManeuvers_maneuverUTurnAllowed; ++n) {
            directions += bit(*connection.connectingLane.maneuver, n) ? 1 : 0;
        }
        if (directions != 1) {
            result.add(Severity::Error, "RS_ARSM_22", path + ".connectingLane.maneuver",
                std::to_string(directions) + " direction bits set instead of exactly one");
        }
    }
    if (!connection.signalGroup) {
        result.add(Severity::Warning, "RS_ARSM_48", path, "connection without signalGroup");
    }
}

void check_geometry(const IntersectionGeometry& geometry, const std::string& path, ValidationProfile profile,
        ValidationResult& result)
{
    if (car2car(profile) && !geometry.laneWidth) {
        result.add(Severity::Error, "RS_ARSM_14", path, "laneWidth missing");
    }

    std::set<long> lane_ids;
    for (int j = 0; j < geometry.laneSet.list.count; ++j) {
        const long id = geometry.laneSet.list.array[j]->laneID;
        if (!lane_ids.insert(id).second) {
            result.add(Severity::Error, "LaneID unique", item(path, "laneSet", j),
                "duplicate LaneID " + std::to_string(id));
        }
    }

    // connection ID -> signal group of its first use, -1 without signal group
    std::map<long, long> connection_groups;
    for (int j = 0; j < geometry.laneSet.list.count; ++j) {
        const GenericLane& lane = *geometry.laneSet.list.array[j];
        const std::string lane_path = item(path, "laneSet", j);
        check_lane(lane, lane_path, profile, result);
        if (!lane.connectsTo) {
            continue;
        }
        for (int k = 0; k < lane.connectsTo->list.count; ++k) {
            const Connection_t& connection = *lane.connectsTo->list.array[k];
            const std::string connection_path = item(lane_path, "connectsTo", k);
            check_connection(lane, connection, connection_path, lane_ids, profile, result);
            if (!connection.connectionID) {
                continue;
            }
            const long group = connection.signalGroup ? *connection.signalGroup : -1;
            const auto inserted = connection_groups.emplace(*connection.connectionID, group);
            if (!inserted.second && inserted.first->second != group) {
                result.add(Severity::Warning, "connectionID", connection_path,
                    "connectionID " + std::to_string(*connection.connectionID) + " used with different signal groups");
            }
        }
    }
}

// TimeMark relative to the begin of the current minute (C2C-CC RS 2077 RS_ARSM_54), none if unknown
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
        result.add(Severity::Error, "maxEndTime", path, "maxEndTime missing (C-Roads)");
    } else if (car2car(context.profile) && context.traffic_dependent && !timing.maxEndTime) {
        result.add(Severity::Error, "RS_ARSM_57", path, "maxEndTime missing for traffic dependent operation");
    }
    if (!car2car(context.profile)) {
        return;
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
    if (timing.likelyTime && !timing.confidence) {
        result.add(Severity::Error, "RS_ARSM_115", path, "likelyTime without confidence");
    }

    const boost::optional<long> min = resolve(timing.minEndTime, context.minute_mark);
    const boost::optional<long> likely = resolve(timing.likelyTime, context.minute_mark);
    const boost::optional<long> max = resolve(timing.maxEndTime, context.minute_mark);
    if ((min && likely && *min > *likely) || (likely && max && *likely > *max) || (min && max && *min > *max)) {
        result.add(Severity::Error, "RS_ARSM_65", path, "not minEndTime <= likelyTime <= maxEndTime");
    }

    // a known time of change: all present ends are equal for fixed time operation
    const bool known = max && *max != time_mark_infinite;
    if (context.fixed_time && known && min && (*min != *max || (likely && *likely != *max))) {
        result.add(Severity::Error, "RS_ARSM_61", path,
            "minEndTime, likelyTime and maxEndTime differ in fixed time operation");
    }
}

bool transitional(long state)
{
    return state == MovementPhaseState_pre_Movement || state == MovementPhaseState_permissive_clearance ||
        state == MovementPhaseState_protected_clearance;
}

void check_events(const MovementState& movement, const std::string& path, const StateContext& context,
        ValidationResult& result)
{
    const MovementEventList_t& events = movement.state_time_speed;
    bool reaches_next_phase = false;
    for (int k = 0; k < events.list.count; ++k) {
        const MovementEvent_t& event = *events.list.array[k];
        const std::string event_path = item(path, "state-time-speed", k);
        if (k > 0 && events.list.array[k - 1]->eventState == event.eventState) {
            result.add(Severity::Warning, "eventState sequence", event_path, "same eventState as the previous event");
        }
        reaches_next_phase |= k > 0 && !transitional(event.eventState);
        if (event.timing) {
            check_timing(*event.timing, event_path + ".timing", context, result);
        }
    }

    const MovementEvent_t& last = *events.list.array[events.list.count - 1];
    const bool last_end_known = last.timing && last.timing->minEndTime < time_mark_out_of_range;
    if (car2car(context.profile) && !reaches_next_phase && last_end_known) {
        result.add(Severity::Error, "RS_ARSM_79", path + ".state-time-speed", "events do not reach the next phase");
    }
}

void check_state(const IntersectionState& state, const std::string& path, ValidationProfile profile,
        ValidationResult& result)
{
    StateContext context;
    context.profile = profile;
    context.fixed_time = bit(state.status, IntersectionStatusObject_fixedTimeOperation);
    context.traffic_dependent = bit(state.status, IntersectionStatusObject_trafficDependentOperation);
    if (state.moy) {
        context.minute_mark = (*state.moy % 60) * 600;
    }

    std::set<long> groups;
    for (int j = 0; j < state.states.list.count; ++j) {
        const MovementState& movement = *state.states.list.array[j];
        const std::string movement_path = item(path, "states", j);
        if (!groups.insert(movement.signalGroup).second) {
            result.add(Severity::Warning, "SignalGroupID unique", movement_path,
                "second MovementState for signal group " + std::to_string(movement.signalGroup));
        }
        if (movement.signalGroup == 0) {
            result.add(Severity::Warning, "SignalGroupID value", movement_path, "signal group 0 means not known");
        }
        if (movement.state_time_speed.list.count > 0) {
            check_events(movement, movement_path, context, result);
        }
    }
}

// signal group -> path of the first connection using it
std::map<long, std::string> connection_groups(const IntersectionGeometry& geometry, const std::string& path)
{
    std::map<long, std::string> groups;
    for (int j = 0; j < geometry.laneSet.list.count; ++j) {
        const GenericLane& lane = *geometry.laneSet.list.array[j];
        for (int k = 0; lane.connectsTo && k < lane.connectsTo->list.count; ++k) {
            const Connection_t& connection = *lane.connectsTo->list.array[k];
            if (connection.signalGroup) {
                const std::string connection_path = item(item(path, "laneSet", j), "connectsTo", k);
                groups.emplace(*connection.signalGroup, connection_path + ".signalGroup");
            }
        }
    }
    return groups;
}

} // namespace

ValidationResult validate_map(const MapData& map, ValidationProfile profile)
{
    ValidationResult result;
    check_constraints(asn_DEF_MapData, &map, "MapData", result);
    if (!map.intersections) {
        return result;
    }

    std::vector<ReferenceId> seen;
    for (int i = 0; i < map.intersections->list.count; ++i) {
        const IntersectionGeometry& geometry = *map.intersections->list.array[i];
        const std::string path = item("MapData", "intersections", i);
        check_reference(geometry.id, path + ".id", seen, profile, result);
        check_geometry(geometry, path, profile, result);
    }
    return result;
}

ValidationResult validate_spat(const SPAT& spat, ValidationProfile profile)
{
    ValidationResult result;
    check_constraints(asn_DEF_SPAT, &spat, "SPAT", result);

    std::vector<ReferenceId> seen;
    for (int i = 0; i < spat.intersections.list.count; ++i) {
        const IntersectionState& state = *spat.intersections.list.array[i];
        const std::string path = item("SPAT", "intersections", i);
        check_reference(state.id, path + ".id", seen, profile, result);
        check_state(state, path, profile, result);
    }
    return result;
}

ValidationResult validate_map_spat(const MapData& map, const SPAT& spat, ValidationProfile profile)
{
    ValidationResult result;
    const Severity profile_rule = mandatory_if(car2car(profile));

    for (int i = 0; i < spat.intersections.list.count; ++i) {
        const IntersectionState& state = *spat.intersections.list.array[i];
        const std::string state_path = item("SPAT", "intersections", i);
        const ReferenceId id = reference_id(state.id);

        const IntersectionGeometry* geometry = nullptr;
        std::string geometry_path;
        for (int g = 0; map.intersections && g < map.intersections->list.count && !geometry; ++g) {
            if (reference_id(map.intersections->list.array[g]->id) == id) {
                geometry = map.intersections->list.array[g];
                geometry_path = item("MapData", "intersections", g);
            }
        }
        if (!geometry) {
            result.add(profile_rule, "RS_ARSM_13", state_path + ".id",
                "no IntersectionGeometry with IntersectionReferenceID " + describe(id));
            continue;
        }

        if (state.revision != geometry->revision) {
            result.add(Severity::Error, "revision", state_path + ".revision",
                "revision " + std::to_string(state.revision) + " differs from " + geometry_path + ".revision " +
                std::to_string(geometry->revision));
        }

        std::set<long> state_groups;
        for (int j = 0; j < state.states.list.count; ++j) {
            const long group = state.states.list.array[j]->signalGroup;
            state_groups.insert(group);
        }
        const std::map<long, std::string> map_groups = connection_groups(*geometry, geometry_path);
        for (const auto& group : map_groups) {
            if (state_groups.count(group.first) == 0) {
                result.add(profile_rule, "RS_ARSM_49", group.second, "signal group " + std::to_string(group.first) +
                    " has no MovementState in " + state_path);
            }
        }
        for (int j = 0; j < state.states.list.count; ++j) {
            const long group = state.states.list.array[j]->signalGroup;
            if (map_groups.count(group) == 0) {
                result.add(profile_rule, "RS_ARSM_75", item(state_path, "states", j), "signal group " +
                    std::to_string(group) + " is not used by any connection of " + geometry_path);
            }
        }
    }
    return result;
}

} // namespace facilities
} // namespace vanetza
