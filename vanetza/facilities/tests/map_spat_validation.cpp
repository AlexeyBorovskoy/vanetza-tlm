#include <gtest/gtest.h>
#include <vanetza/asn1/mapem.hpp>
#include <vanetza/asn1/spatem.hpp>
#include <vanetza/asn1/its/AllowedManeuvers.h>
#include <vanetza/asn1/its/ComputedLane.h>
#include <vanetza/asn1/its/Connection.h>
#include <vanetza/asn1/its/ConnectsToList.h>
#include <vanetza/asn1/its/DescriptiveName.h>
#include <vanetza/asn1/its/EnabledLaneList.h>
#include <vanetza/asn1/its/GenericLane.h>
#include <vanetza/asn1/its/IntersectionGeometry.h>
#include <vanetza/asn1/its/IntersectionReferenceID.h>
#include <vanetza/asn1/its/IntersectionStatusObject.h>
#include <vanetza/asn1/its/LayerID.h>
#include <vanetza/asn1/its/LayerType.h>
#include <vanetza/asn1/its/MapData.h>
#include <vanetza/asn1/its/MovementEvent.h>
#include <vanetza/asn1/its/MovementPhaseState.h>
#include <vanetza/asn1/its/MovementState.h>
#include <vanetza/asn1/its/NodeAttributeSetXY.h>
#include <vanetza/asn1/its/NodeAttributeXYList.h>
#include <vanetza/asn1/its/NodeListXY.h>
#include <vanetza/asn1/its/NodeXY.h>
#include <vanetza/asn1/its/RoadRegulatorID.h>
#include <vanetza/asn1/its/TimeChangeDetails.h>
#include <vanetza/asn1/its/TimeMark.h>
#include <vanetza/facilities/fixed_time_plan.hpp>
#include <vanetza/facilities/map_functions.hpp>
#include <vanetza/facilities/map_spat_validation.hpp>
#include <vanetza/facilities/spat_functions.hpp>
#include <vanetza/facilities/validation.hpp>
#include <boost/optional/optional.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

using namespace vanetza;
using namespace vanetza::facilities;

namespace
{

// remove an OPTIONAL scalar member allocated by asn1::allocate (calloc); only safe for members
// that own no further allocations themselves (plain long/enum typedefs such as LaneWidth_t,
// ApproachID_t, SignalGroupID_t, LaneConnectionID_t, LayerType_t, DSecond_t, MinuteOfTheYear_t, ...)
template<typename T>
void remove_optional(T*& member)
{
    std::free(member);
    member = nullptr;
}

// remove an OPTIONAL (or list) member that owns further allocations itself (e.g. a BIT STRING
// buffer or a SEQUENCE with its own pointers), freed recursively through its ASN.1 descriptor
template<typename T>
void remove_owned(T*& member, asn_TYPE_descriptor_t& descriptor)
{
    ASN_STRUCT_FREE(descriptor, member);
    member = nullptr;
}

const auto meter = units::si::meter;

const long intersection_id = 7;
const long intersection_region = 1;
const long intersection_revision = 3;

const Clock::time_point reference_time = Clock::at("2026-10-04 12:00:00.000");

const std::array<ValidationProfile, 4> all_profiles {
    ValidationProfile::Standard, ValidationProfile::Car2Car, ValidationProfile::CRoads, ValidationProfile::Combined
};

// an issue without its message: rule, severity and path are asserted exactly, so that extra,
// missing or misplaced issues fail a test
struct Finding
{
    std::string rule;
    Severity severity;
    std::string path;
};

bool operator<(const Finding& a, const Finding& b)
{
    return std::tie(a.rule, a.path, a.severity) < std::tie(b.rule, b.path, b.severity);
}

bool operator==(const Finding& a, const Finding& b)
{
    return a.rule == b.rule && a.severity == b.severity && a.path == b.path;
}

std::ostream& operator<<(std::ostream& os, const Finding& finding)
{
    return os << (finding.severity == Severity::Error ? "error " : "warning ") << finding.rule << " " << finding.path;
}

using Findings = std::multiset<Finding>;

const Findings no_issues {};

Finding error_at(const std::string& rule, const std::string& path)
{
    return Finding { rule, Severity::Error, path };
}

Finding warning_at(const std::string& rule, const std::string& path)
{
    return Finding { rule, Severity::Warning, path };
}

Findings findings(const ValidationResult& result)
{
    Findings found;
    for (const ValidationIssue& issue : result.issues()) {
        found.insert(Finding { issue.rule, issue.severity, issue.path });
    }
    return found;
}

bool has_rule(const ValidationResult& result, const std::string& rule)
{
    return std::any_of(result.issues().begin(), result.issues().end(),
        [&rule](const ValidationIssue& issue) { return issue.rule == rule; });
}

// true if any issue carries a C2C-CC (RS_ARSM_*) or C-Roads identifier; must never happen under
// ValidationProfile::Standard, which checks ETSI TS 103 301 / ISO TS 19091 only
bool has_profile_specific_identifier(const ValidationResult& result)
{
    for (const ValidationIssue& issue : result.issues()) {
        if (issue.rule.find("RS_ARSM_") == 0 || issue.rule.find("C-Roads") != std::string::npos) {
            return true;
        }
    }
    return false;
}

const std::string geometry_path = "MapData.intersections[0]";
const std::string state_path = "SPAT.intersections[0]";

std::string lane_path(int lane)
{
    return geometry_path + ".laneSet[" + std::to_string(lane) + "]";
}

std::string connection_path(int lane, int connection)
{
    return lane_path(lane) + ".connectsTo[" + std::to_string(connection) + "]";
}

std::string movement_path(int movement)
{
    return state_path + ".states[" + std::to_string(movement) + "]";
}

std::string events_path(int movement)
{
    return movement_path(movement) + ".state-time-speed";
}

std::string event_path(int movement, int event)
{
    return events_path(movement) + "[" + std::to_string(event) + "]";
}

void set_region(IntersectionReferenceID_t& id, long region)
{
    id.region = asn1::allocate<RoadRegulatorID_t>();
    *id.region = region;
}

void set_layer_id(MapData& map, long id)
{
    map.layerID = asn1::allocate<LayerID_t>();
    *map.layerID = id;
}

// intersection with region, revision and laneWidth set, no lanes
IntersectionGeometry& build_intersection(asn1::Mapem& mapem, long id = intersection_id,
        long revision = intersection_revision)
{
    IntersectionGeometry& intersection = add_intersection_geometry(mapem->map, id, revision,
        59.0 * units::degree, 30.0 * units::degree, 3.5 * meter);
    set_region(intersection.id, intersection_region);
    return intersection;
}

// one ingress lane without connections, the least an ASN.1-valid geometry needs (LaneList SIZE(1..255))
void add_ingress_lane(IntersectionGeometry& intersection)
{
    GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -20.0 * meter);
    add_node(lane, 0.0 * meter, -10.0 * meter);
}

// intersection with one ingress lane (no connections), otherwise ASN.1-valid and issue-free;
// a minimal base for tests that only care about one unrelated deviation
IntersectionGeometry& build_single_lane_intersection(asn1::Mapem& mapem)
{
    IntersectionGeometry& intersection = build_intersection(mapem);
    add_ingress_lane(intersection);
    return intersection;
}

// intersection state with region and timestamp set, fixedTimeOperation flagged, no movements
IntersectionState& build_intersection_state(asn1::Spatem& spatem, long id, long revision,
        const Clock::time_point& now)
{
    IntersectionState& state = add_intersection_state(spatem->spat, id, revision);
    set_region(state.id, intersection_region);
    set_timestamp(state, now);
    const int bit = IntersectionStatusObject_fixedTimeOperation;
    state.status.buf[bit / 8] |= 0x80 >> (bit % 8);
    return state;
}

// intersection state with region and timestamp set, no status bit, no movements; lets a test
// set up status bits explicitly instead of inheriting fixedTimeOperation
IntersectionState& build_bare_state(asn1::Spatem& spatem, long id, long revision, const Clock::time_point& now)
{
    IntersectionState& state = add_intersection_state(spatem->spat, id, revision);
    set_region(state.id, intersection_region);
    set_timestamp(state, now);
    return state;
}

// event with min_end == max_end == likely (offset from now), confidence 15
MovementEvent& add_known_event(MovementState& movement, long phase_state, const Clock::time_point& now,
        Clock::duration offset)
{
    MovementEvent& event = add_event(movement, phase_state);
    MovementTiming timing;
    timing.min_end = now + offset;
    timing.max_end = timing.min_end;
    timing.likely = timing.min_end;
    timing.confidence = 15;
    set_timing(event, timing, now);
    return event;
}

// event with explicit TimeMark integers, bypassing set_timing's real-world rounding; used where a
// discriminating scenario needs the exact special values 36000/36001 or precise orderings
MovementEvent& add_raw_event(MovementState& movement, long phase_state, long min_end,
        const boost::optional<long>& max_end = boost::none, const boost::optional<long>& likely = boost::none,
        const boost::optional<long>& confidence = boost::none)
{
    MovementEvent& event = add_event(movement, phase_state);
    event.timing = asn1::allocate<TimeChangeDetails_t>();
    event.timing->minEndTime = min_end;
    if (max_end) {
        event.timing->maxEndTime = asn1::allocate<TimeMark_t>();
        *event.timing->maxEndTime = *max_end;
    }
    if (likely) {
        event.timing->likelyTime = asn1::allocate<TimeMark_t>();
        *event.timing->likelyTime = *likely;
    }
    if (confidence) {
        event.timing->confidence = asn1::allocate<TimeIntervalConfidence_t>();
        *event.timing->confidence = *confidence;
    }
    return event;
}

// movement with two fully timed events, permissive green ending in 10 s, then red (a phase)
MovementState& add_timed_movement(IntersectionState& state, long signal_group)
{
    MovementState& movement = add_movement(state, signal_group);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    return movement;
}

// ingress lane 1 connected to egress lane 2 straight ahead with signal group and connection ID 1
GenericLane& build_connected_lanes(IntersectionGeometry& intersection)
{
    GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(ingress, 0.0 * meter, -10.0 * meter);
    add_node(ingress, 0.0 * meter, -5.0 * meter);
    GenericLane& egress = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
    add_node(egress, 0.0 * meter, 5.0 * meter);
    add_node(egress, 0.0 * meter, 10.0 * meter);
    connect(ingress, 2, Maneuver::Straight, 1, 1);
    return ingress;
}

struct Baseline
{
    asn1::Mapem mapem;
    asn1::Spatem spatem;
};

// ASN.1-valid baseline: one intersection, ingress lane 1 -> egress lane 2 via signal group 1,
// bidirectional crosswalk lane 3 (not connected); one SPAT intersection state with signal group 1
// having two fully-timed events reaching a phase. Zero issues are expected under every profile
// for validate_map, validate_spat and validate_map_spat (see the baseline_* tests below).
Baseline build_baseline(const Clock::time_point& now)
{
    Baseline baseline;

    IntersectionGeometry& intersection = build_intersection(baseline.mapem);
    GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(ingress, 0.0 * meter, -20.0 * meter);
    add_node(ingress, 0.0 * meter, -10.0 * meter);
    connect(ingress, 2, Maneuver::Straight, 1, 1);

    GenericLane& egress = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
    add_node(egress, 0.0 * meter, 10.0 * meter);
    add_node(egress, 0.0 * meter, 20.0 * meter);

    GenericLane& crosswalk = add_crosswalk_lane(intersection, 3, 1, 2);
    add_node(crosswalk, -5.0 * meter, 0.0 * meter);
    add_node(crosswalk, 5.0 * meter, 0.0 * meter);

    IntersectionState& state = build_intersection_state(baseline.spatem, intersection_id, intersection_revision, now);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, now, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, now, std::chrono::seconds(30));

    return baseline;
}

} // namespace

// =================================================================================================
// baseline and ValidationResult sanity
// =================================================================================================

TEST(MapSpatValidation, baseline_map_has_no_issues_in_any_profile)
{
    Baseline baseline = build_baseline(reference_time);
    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(no_issues, findings(validate_map(baseline.mapem->map, profile)));
    }
}

TEST(MapSpatValidation, baseline_spat_has_no_issues_in_any_profile)
{
    Baseline baseline = build_baseline(reference_time);
    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(no_issues, findings(validate_spat(baseline.spatem->spat, profile)));
    }
}

TEST(MapSpatValidation, baseline_map_spat_has_no_issues_in_any_profile)
{
    Baseline baseline = build_baseline(reference_time);
    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(no_issues, findings(validate_map_spat(baseline.mapem->map, baseline.spatem->spat, profile)));
    }
}

TEST(MapSpatValidation, baseline_map_has_default_revision_and_no_layer_fields)
{
    // DSRC MapData profile comment: msgIssueRevision shall be 0, layerType shall not be used;
    // the builders must not set these although they may set layerID for a fragment
    Baseline baseline = build_baseline(reference_time);
    EXPECT_EQ(0, baseline.mapem->map.msgIssueRevision);
    EXPECT_EQ(nullptr, baseline.mapem->map.layerType);
    EXPECT_EQ(nullptr, baseline.mapem->map.layerID);
}

TEST(MapSpatValidation, validation_result_valid_and_count_track_issues)
{
    ValidationResult result;
    EXPECT_TRUE(result.valid());
    EXPECT_EQ(0u, result.count(Severity::Error));
    EXPECT_EQ(0u, result.count(Severity::Warning));

    result.add(Severity::Warning, "rule-a", "path.a", "a warning");
    EXPECT_TRUE(result.valid());
    EXPECT_EQ(1u, result.count(Severity::Warning));

    result.add(Severity::Error, "rule-b", "path.b", "an error");
    EXPECT_FALSE(result.valid());
    EXPECT_EQ(1u, result.count(Severity::Error));
    EXPECT_EQ(2u, result.issues().size());

    ValidationResult other;
    other.add(Severity::Warning, "rule-c", "path.c", "another warning");
    result.append(other);
    EXPECT_EQ(3u, result.issues().size());
    EXPECT_EQ(2u, result.count(Severity::Warning));
}

TEST(MapSpatValidation, validation_result_append_self_duplicates_issues)
{
    ValidationResult result;
    result.add(Severity::Warning, "rule-a", "path.a", "a warning");
    result.add(Severity::Error, "rule-b", "path.b", "an error");
    result.append(result); // inserting a range of itself must not be undefined behaviour

    EXPECT_EQ(4u, result.issues().size());
    EXPECT_EQ(2u, result.count(Severity::Error));
    EXPECT_EQ(2u, result.count(Severity::Warning));
}

TEST(MapSpatValidation, validation_issue_stream_operator_formats_as_specified)
{
    ValidationResult error_result;
    error_result.add(Severity::Error, "RS_ARSM_49", "SPAT.intersections[0]", "signal group 5 has no MovementState");
    std::ostringstream error_out;
    error_out << error_result.issues().front();
    EXPECT_EQ("error RS_ARSM_49 SPAT.intersections[0]: signal group 5 has no MovementState", error_out.str());

    ValidationResult warning_result;
    warning_result.add(Severity::Warning, "LaneID value", "MapData.intersections[0].laneSet[0]",
        "LaneID 0 means not known");
    std::ostringstream warning_out;
    warning_out << warning_result.issues().front();
    EXPECT_EQ("warning LaneID value MapData.intersections[0].laneSet[0]: LaneID 0 means not known", warning_out.str());
}

// =================================================================================================
// structure: ASN.1 type checks skip all semantic checks, malformed structures do not crash
// =================================================================================================

TEST(MapSpatValidation, check_asn1_accepts_valid_structure_without_issues)
{
    Baseline baseline = build_baseline(reference_time);
    ValidationResult result;
    EXPECT_TRUE(check_asn1(asn_DEF_MapData, &baseline.mapem->map, "MapData", result));
    EXPECT_EQ(no_issues, findings(result));
}

TEST(MapSpatValidation, check_asn1_reports_absent_structure)
{
    ValidationResult result;
    EXPECT_FALSE(check_asn1(asn_DEF_MapData, nullptr, "MapData", result));
    EXPECT_EQ(Findings { error_at("ASN.1", "MapData") }, findings(result));
}

TEST(MapSpatValidation, map_asn1_constraint_violation_reports_only_one_issue)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -10.0 * meter);
    add_node(lane, 0.0 * meter, -5.0 * meter);
    lane.laneID = 256; // LaneID ::= INTEGER (0..255)

    EXPECT_EQ(Findings { error_at("ASN.1", "MapData") },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, spat_status_wrong_bit_string_size_is_reported_as_asn1_error_without_crash)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    // break IntersectionStatusObject ::= BIT STRING (SIZE(16)): 1 byte (8 bits) instead of 2
    std::free(state.status.buf);
    state.status.size = 1;
    state.status.buf = static_cast<uint8_t*>(asn1::allocate(1));
    state.status.bits_unused = 0;
    add_timed_movement(state, 1);

    EXPECT_EQ(Findings { error_at("ASN.1", "SPAT") },
        findings(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, map_spat_returns_no_issues_when_one_message_violates_constraints)
{
    Baseline baseline = build_baseline(reference_time);
    MovementState& movement = *baseline.spatem->spat.intersections.list.array[0]->states.list.array[0];
    movement.signalGroup = 256; // SignalGroupID ::= INTEGER (0..255), MAP side stays fully valid

    ValidationResult result = validate_map_spat(baseline.mapem->map, baseline.spatem->spat);
    EXPECT_EQ(no_issues, findings(result));
}

TEST(MapSpatValidation, empty_lane_set_element_is_reported_as_single_asn1_issue)
{
    Baseline baseline = build_baseline(reference_time);
    IntersectionGeometry& intersection = *baseline.mapem->map.intersections->list.array[0];
    GenericLane*& slot = intersection.laneSet.list.array[2]; // crosswalk lane, not a connection target
    remove_owned(slot, asn_DEF_GenericLane);

    EXPECT_EQ(Findings { error_at("ASN.1", lane_path(2)) }, findings(validate_map(baseline.mapem->map)));
}

TEST(MapSpatValidation, lane_with_single_node_violates_node_set_size_bound)
{
    // ISO TS 19091 NodeSetXY ::= SEQUENCE (SIZE(2..63)) OF NodeXY
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -10.0 * meter);

    EXPECT_EQ(Findings { error_at("ASN.1", lane_path(0) + ".nodeList.nodes") }, findings(validate_map(mapem->map)));
}

TEST(MapSpatValidation, empty_movement_event_list_violates_size_bound)
{
    // ISO TS 19091 MovementEventList ::= SEQUENCE (SIZE(1..16)) OF MovementEvent
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    add_movement(state, 1); // no events added

    EXPECT_EQ(Findings { error_at("ASN.1", events_path(0)) }, findings(validate_spat(spatem->spat)));
}

TEST(MapSpatValidation, seventeen_connections_violate_connects_to_size_bound)
{
    // ISO TS 19091 ConnectsToList ::= SEQUENCE (SIZE(1..16)) OF Connection
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    GenericLane& ingress = *intersection.laneSet.list.array[0];
    for (long id = 1; id <= 17; ++id) {
        connect(ingress, 1, Maneuver::Straight, 1, id);
    }

    EXPECT_EQ(Findings { error_at("ASN.1", lane_path(0) + ".connectsTo") }, findings(validate_map(mapem->map)));
}

TEST(MapSpatValidation, nested_enabled_lane_list_size_is_checked)
{
    // ISO TS 19091 EnabledLaneList ::= SEQUENCE (SIZE(1..16)) OF LaneID, not checked by asn1c
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    add_timed_movement(state, 1);
    state.enabledLanes = asn1::allocate<EnabledLaneList_t>(); // no lane enabled

    EXPECT_EQ(Findings { error_at("ASN.1", state_path + ".enabledLanes") }, findings(validate_spat(spatem->spat)));
}

TEST(MapSpatValidation, nested_node_attribute_list_size_is_checked)
{
    // ISO TS 19091 NodeAttributeXYList ::= SEQUENCE (SIZE(1..8)) OF NodeAttributeXY
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    NodeXY_t& node = *intersection.laneSet.list.array[0]->nodeList.choice.nodes.list.array[0];
    node.attributes = asn1::allocate<NodeAttributeSetXY_t>();
    node.attributes->localNode = asn1::allocate<NodeAttributeXYList_t>(); // no attribute

    EXPECT_EQ(Findings { error_at("ASN.1", lane_path(0) + ".nodeList.nodes[0].attributes.localNode") },
        findings(validate_map(mapem->map)));
}

TEST(MapSpatValidation, choice_without_alternative_is_reported_without_crash)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1); // no nodes allocated
    lane.nodeList.present = NodeListXY_PR_NOTHING;

    EXPECT_EQ(Findings { error_at("ASN.1", lane_path(0) + ".nodeList") }, findings(validate_map(mapem->map)));
}

TEST(MapSpatValidation, bit_string_without_buffer_is_reported_without_crash)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    add_timed_movement(state, 1);
    std::free(state.status.buf);
    state.status.buf = nullptr; // size stays 2

    EXPECT_EQ(Findings { error_at("ASN.1", state_path + ".status") }, findings(validate_spat(spatem->spat)));
}

TEST(MapSpatValidation, character_string_without_buffer_is_reported_without_crash)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    intersection.name = asn1::allocate<DescriptiveName_t>();
    intersection.name->size = 3; // DescriptiveName ::= IA5String (SIZE(1..63)), no buffer

    EXPECT_EQ(Findings { error_at("ASN.1", geometry_path + ".name") }, findings(validate_map(mapem->map)));
}

TEST(MapSpatValidation, inconsistent_lists_are_reported_without_crash)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    auto& list = intersection.laneSet.list;
    const auto valid = list;
    const Findings expected { error_at("ASN.1", geometry_path + ".laneSet") };

    list.count = list.size + 1; // more elements than allocated
    EXPECT_EQ(expected, findings(validate_map(mapem->map)));
    list = valid;

    list.array = nullptr; // elements without array, asn1c constraint checking would dereference it
    EXPECT_EQ(expected, findings(validate_map(mapem->map)));
    list = valid;

    list.count = -1;
    EXPECT_EQ(expected, findings(validate_map(mapem->map)));
    list = valid;
}

// =================================================================================================
// validate_map: rules without an RS identifier
// =================================================================================================

TEST(MapSpatValidation, msg_issue_revision_nonzero_reports_error_in_every_profile)
{
    asn1::Mapem mapem;
    build_single_lane_intersection(mapem);
    mapem->map.msgIssueRevision = 1;

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(Findings { error_at("msgIssueRevision", "MapData.msgIssueRevision") },
            findings(validate_map(mapem->map, profile)));
    }
}

TEST(MapSpatValidation, layer_type_present_reports_error_in_every_profile)
{
    asn1::Mapem mapem;
    build_single_lane_intersection(mapem);
    mapem->map.layerType = asn1::allocate<LayerType_t>();
    *mapem->map.layerType = LayerType_intersectionData;

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(Findings { error_at("layerType", "MapData.layerType") }, findings(validate_map(mapem->map, profile)));
    }
}

TEST(MapSpatValidation, duplicate_lane_id_reports_error_at_second_occurrence)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& a = add_vehicle_lane(intersection, 5, TravelDirection::Ingress, 1);
    add_node(a, 0.0 * meter, -10.0 * meter);
    add_node(a, 0.0 * meter, -5.0 * meter);
    GenericLane& b = add_vehicle_lane(intersection, 5, TravelDirection::Egress, 2);
    add_node(b, 0.0 * meter, 5.0 * meter);
    add_node(b, 0.0 * meter, 10.0 * meter);

    EXPECT_EQ(Findings { error_at("LaneID unique", lane_path(1)) }, findings(validate_map(mapem->map)));
}

TEST(MapSpatValidation, lane_id_zero_reports_warning_only)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 0, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -10.0 * meter);
    add_node(lane, 0.0 * meter, -5.0 * meter);

    EXPECT_EQ(Findings { warning_at("LaneID value", lane_path(0)) }, findings(validate_map(mapem->map)));
}

TEST(MapSpatValidation, lane_id_reserved_255_reports_warning_only)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 255, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -10.0 * meter);
    add_node(lane, 0.0 * meter, -5.0 * meter);

    EXPECT_EQ(Findings { warning_at("LaneID value", lane_path(0)) }, findings(validate_map(mapem->map)));
}

TEST(MapSpatValidation, dangling_connecting_lane_reports_error_with_full_path)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    GenericLane& ingress = *intersection.laneSet.list.array[0];
    connect(ingress, 99, Maneuver::Straight, 1, 1); // lane 99 does not exist

    EXPECT_EQ(Findings { error_at("connectingLane", connection_path(0, 0) + ".connectingLane.lane") },
        findings(validate_map(mapem->map)));
}

TEST(MapSpatValidation, connecting_lane_target_zero_is_warning_in_every_profile)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    connect(*intersection.laneSet.list.array[0], 0, Maneuver::Straight, 1, 1);

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(Findings { warning_at("connectingLane", connection_path(0, 0) + ".connectingLane.lane") },
            findings(validate_map(mapem->map, profile)));
    }
}

TEST(MapSpatValidation, connecting_lane_target_255_is_warning_in_every_profile)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    connect(*intersection.laneSet.list.array[0], 255, Maneuver::Straight, 1, 1);

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(Findings { warning_at("connectingLane", connection_path(0, 0) + ".connectingLane.lane") },
            findings(validate_map(mapem->map, profile)));
    }
}

TEST(MapSpatValidation, connecting_lane_target_missing_is_error_without_fragment)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    connect(*intersection.laneSet.list.array[0], 9, Maneuver::Straight, 1, 1); // lane 9 does not exist

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(Findings { error_at("connectingLane", connection_path(0, 0) + ".connectingLane.lane") },
            findings(validate_map(mapem->map, profile)));
    }
}

TEST(MapSpatValidation, connecting_lane_target_missing_is_warning_for_a_fragment)
{
    // LayerID: tens give the number of fragments, units the fragment (DSRC LayerID example)
    for (long layer : { 21, 22, 31, 33, 99 }) {
        asn1::Mapem mapem;
        IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
        connect(*intersection.laneSet.list.array[0], 9, Maneuver::Straight, 1, 1); // target may be in another fragment
        set_layer_id(mapem->map, layer);

        for (ValidationProfile profile : all_profiles) {
            EXPECT_EQ(Findings { warning_at("connectingLane", connection_path(0, 0) + ".connectingLane.lane") },
                findings(validate_map(mapem->map, profile))) << "layerID " << layer;
        }
    }
}

TEST(MapSpatValidation, invalid_layer_id_is_error_and_not_treated_as_fragment)
{
    // 1 and 11: no fragmentation (layerID shall not be used), 20: fragment 0, 23: fragment beyond
    // the number of fragments, 100: ten fragments are not representable
    for (long layer : { 1, 11, 20, 23, 100 }) {
        asn1::Mapem mapem;
        IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
        connect(*intersection.laneSet.list.array[0], 9, Maneuver::Straight, 1, 1);
        set_layer_id(mapem->map, layer);

        const Findings expected {
            error_at("layerID", "MapData.layerID"),
            error_at("connectingLane", connection_path(0, 0) + ".connectingLane.lane"),
        };
        EXPECT_EQ(expected, findings(validate_map(mapem->map))) << "layerID " << layer;
    }
}

TEST(MapSpatValidation, connection_id_reused_with_different_signal_groups_warns)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = build_connected_lanes(intersection); // connection ID 1, signal group 1
    GenericLane& egress_b = add_vehicle_lane(intersection, 3, TravelDirection::Egress, 3);
    add_node(egress_b, 5.0 * meter, 0.0 * meter);
    add_node(egress_b, 10.0 * meter, 0.0 * meter);
    connect(ingress, 3, Maneuver::Right, 2, 1); // same connection ID 1, signal group 2

    EXPECT_EQ(Findings { warning_at("connectionID", connection_path(0, 1)) }, findings(validate_map(mapem->map)));
}

TEST(MapSpatValidation, connection_id_reused_with_same_signal_group_is_not_reported)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = build_connected_lanes(intersection); // connection ID 1, signal group 1
    GenericLane& egress_b = add_vehicle_lane(intersection, 3, TravelDirection::Egress, 3);
    add_node(egress_b, 5.0 * meter, 0.0 * meter);
    add_node(egress_b, 10.0 * meter, 0.0 * meter);
    connect(ingress, 3, Maneuver::Right, 1, 1); // same connection ID 1, same signal group 1

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(no_issues, findings(validate_map(mapem->map, profile)));
    }
}

TEST(MapSpatValidation, crosswalk_self_loop_is_warning_not_error_in_every_profile)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& crosswalk = add_crosswalk_lane(intersection, 1, 1, 2);
    add_node(crosswalk, -5.0 * meter, 0.0 * meter);
    add_node(crosswalk, 5.0 * meter, 0.0 * meter);
    connect(crosswalk, 1, Maneuver::Straight, 1, 1); // connects to itself

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(Findings { warning_at("crosswalk self-loop", connection_path(0, 0) + ".connectingLane.lane") },
            findings(validate_map(mapem->map, profile)));
    }
}

// =================================================================================================
// validate_map: C2C-CC RS 2077 rules (Car2Car / Combined)
// =================================================================================================

TEST(MapSpatValidation, rs_arsm_11_missing_region_is_profile_only)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = add_intersection_geometry(mapem->map, intersection_id,
        intersection_revision, 59.0 * units::degree, 30.0 * units::degree, 3.5 * meter); // region left unset
    add_ingress_lane(intersection);

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_11", geometry_path + ".id") },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_12_duplicate_reference_id_reports_error)
{
    asn1::Mapem mapem;
    add_ingress_lane(build_intersection(mapem, 9, 1));
    add_ingress_lane(build_intersection(mapem, 9, 1)); // same id and region

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_12", "MapData.intersections[1].id") },
        findings(validate_map(mapem->map, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_14_missing_lane_width_is_profile_only)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    remove_optional(intersection.laneWidth);

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_14", geometry_path) },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_16_unidirectional_lane_with_both_approaches_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem); // ingress lane, ingressApproach only
    GenericLane& lane = *intersection.laneSet.list.array[0];
    lane.egressApproach = asn1::allocate<ApproachID_t>();
    *lane.egressApproach = 2; // now both approaches set although the lane is unidirectional

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_16", lane_path(0)) },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_16_unidirectional_lane_with_no_approach_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    GenericLane& lane = *intersection.laneSet.list.array[0];
    remove_optional(lane.ingressApproach); // neither approach set now

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_16", lane_path(0)) },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_17_crosswalk_missing_approach_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& crosswalk = add_crosswalk_lane(intersection, 1, 1, 2);
    add_node(crosswalk, -5.0 * meter, 0.0 * meter);
    add_node(crosswalk, 5.0 * meter, 0.0 * meter);
    remove_optional(crosswalk.egressApproach);

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_17", lane_path(0)) },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_20_duplicate_connection_same_direction_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = build_connected_lanes(intersection);
    connect(ingress, 2, Maneuver::Straight, 1, 2); // second connection to the same lane, same direction

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_20", connection_path(0, 1)) },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_20_different_direction_to_same_lane_is_not_reported)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = build_connected_lanes(intersection);
    connect(ingress, 2, Maneuver::Left, 1, 2); // different direction, same target lane

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_20_ignores_unknown_and_reserved_target_lanes)
{
    // LaneID 0 (not known) and 255 (reserved) do not identify a lane, so two such connections
    // are not connections to the same lane
    for (long target : { 0, 255 }) {
        asn1::Mapem mapem;
        IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
        GenericLane& ingress = *intersection.laneSet.list.array[0];
        connect(ingress, target, Maneuver::Straight, 1, 1);
        connect(ingress, target, Maneuver::Straight, 1, 2);

        const Findings expected {
            warning_at("connectingLane", connection_path(0, 0) + ".connectingLane.lane"),
            warning_at("connectingLane", connection_path(0, 1) + ".connectingLane.lane"),
        };
        EXPECT_EQ(expected, findings(validate_map(mapem->map, ValidationProfile::Combined))) << "target " << target;
    }
}

TEST(MapSpatValidation, rs_arsm_21_missing_maneuver_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    GenericLane& ingress = *intersection.laneSet.list.array[0];
    connect(ingress, 1, Maneuver::Straight, 1, 1); // connects to itself, irrelevant here
    Connection_t& connection = *ingress.connectsTo->list.array[0];
    remove_owned(connection.connectingLane.maneuver, asn_DEF_AllowedManeuvers);

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    // no RS_ARSM_22: there are no maneuver bits to miscount
    EXPECT_EQ(Findings { error_at("RS_ARSM_21", connection_path(0, 0) + ".connectingLane") },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_22_connection_with_two_maneuver_bits_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = build_connected_lanes(intersection);
    Connection_t& connection = *ingress.connectsTo->list.array[0];
    ASSERT_NE(nullptr, connection.connectingLane.maneuver);
    connection.connectingLane.maneuver->buf[0] |= 0x80 >> AllowedManeuvers_maneuverRightAllowed;

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_22", connection_path(0, 0) + ".connectingLane.maneuver") },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_22_connection_with_no_maneuver_bits_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = build_connected_lanes(intersection);
    Connection_t& connection = *ingress.connectsTo->list.array[0];
    ASSERT_NE(nullptr, connection.connectingLane.maneuver);
    connection.connectingLane.maneuver->buf[0] = 0; // clear the straight bit, zero bits left

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_22", connection_path(0, 0) + ".connectingLane.maneuver") },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_24_turn_on_red_bit_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    GenericLane& ingress = *intersection.laneSet.list.array[0];
    connect(ingress, 1, Maneuver::Straight, 1, 1);
    Connection_t& connection = *ingress.connectsTo->list.array[0];
    connection.connectingLane.maneuver->buf[0] |= 0x80 >> AllowedManeuvers_maneuverLeftTurnOnRedAllowed;

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    // no RS_ARSM_22: the straight bit is still the only direction bit
    EXPECT_EQ(Findings { error_at("RS_ARSM_24", connection_path(0, 0) + ".connectingLane.maneuver") },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_35_nineteen_nodes_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -20.0 * meter);
    for (int i = 0; i < 18; ++i) {
        add_node(lane, 0.0 * meter, 1.0 * meter); // 19 nodes in total
    }

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_35", lane_path(0) + ".nodeList") },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_35_eighteen_nodes_is_not_reported)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -20.0 * meter);
    for (int i = 0; i < 17; ++i) {
        add_node(lane, 0.0 * meter, 1.0 * meter); // 18 nodes in total
    }

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_48_connection_without_signal_group_is_profile_only_warning)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = build_connected_lanes(intersection);
    remove_optional(ingress.connectsTo->list.array[0]->signalGroup);

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { warning_at("RS_ARSM_48", connection_path(0, 0)) },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_117_lane_level_maneuvers_is_profile_only_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    GenericLane& lane = *intersection.laneSet.list.array[0];
    // AllowedManeuvers ::= BIT STRING (SIZE(12))
    lane.maneuvers = asn1::allocate<AllowedManeuvers_t>();
    lane.maneuvers->buf = static_cast<uint8_t*>(asn1::allocate(2));
    lane.maneuvers->size = 2;
    lane.maneuvers->buf[0] = 0x80 >> AllowedManeuvers_maneuverStraightAllowed;
    lane.maneuvers->bits_unused = 4;

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_117", lane_path(0) + ".maneuvers") },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_118_computed_lane_is_profile_only_error)
{
    // ComputedLane is kept ASN.1-valid (referenceLaneId plus both offset CHOICEs selected) so
    // that only the semantic check under test, not a structural one, is exercised
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    lane.nodeList.present = NodeListXY_PR_computed; // no nodes were ever allocated, nothing to free
    ComputedLane_t& computed = lane.nodeList.choice.computed;
    computed.referenceLaneId = 1;
    computed.offsetXaxis.present = ComputedLane__offsetXaxis_PR_small;
    computed.offsetXaxis.choice.small = 0;
    computed.offsetYaxis.present = ComputedLane__offsetYaxis_PR_small;
    computed.offsetYaxis.choice.small = 0;

    const std::string path = lane_path(0) + ".nodeList";
    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_118", path) },
        findings(validate_map(mapem->map, ValidationProfile::Car2Car)));
    EXPECT_EQ(Findings { error_at("C-Roads computed", path) },
        findings(validate_map(mapem->map, ValidationProfile::CRoads)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_118", path) },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

// =================================================================================================
// validate_map: C-Roads columns of C2C-CC RS 2077 Annex 7.1
// =================================================================================================

TEST(MapSpatValidation, croads_intersections_missing_is_profile_only)
{
    asn1::Mapem mapem; // no intersections at all (optional, absent)

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Car2Car)));
    EXPECT_EQ(Findings { error_at("C-Roads intersections", "MapData") },
        findings(validate_map(mapem->map, ValidationProfile::CRoads)));
    EXPECT_EQ(Findings { error_at("C-Roads intersections", "MapData") },
        findings(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, croads_connection_id_missing_is_profile_only)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    GenericLane& ingress = *intersection.laneSet.list.array[0];
    connect(ingress, 1, Maneuver::Straight, 1, 1);
    Connection_t& connection = *ingress.connectsTo->list.array[0];
    remove_optional(connection.connectionID);

    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(no_issues, findings(validate_map(mapem->map, ValidationProfile::Car2Car)));
    EXPECT_EQ(Findings { error_at("C-Roads connectionID", connection_path(0, 0)) },
        findings(validate_map(mapem->map, ValidationProfile::CRoads)));
}

// =================================================================================================
// validate_spat: rules without an RS identifier
// =================================================================================================

TEST(MapSpatValidation, spat_asn1_constraint_violation_reports_only_one_issue)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_timed_movement(state, 1);
    movement.signalGroup = 256; // SignalGroupID ::= INTEGER (0..255)

    EXPECT_EQ(Findings { error_at("ASN.1", "SPAT") },
        findings(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, duplicate_signal_group_reports_warning)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    add_timed_movement(state, 1);
    add_timed_movement(state, 1);

    EXPECT_EQ(Findings { warning_at("SignalGroupID unique", movement_path(1)) }, findings(validate_spat(spatem->spat)));
}

TEST(MapSpatValidation, signal_group_zero_reports_warning)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    add_timed_movement(state, 0);

    EXPECT_EQ(Findings { warning_at("SignalGroupID value", movement_path(0)) }, findings(validate_spat(spatem->spat)));
}

TEST(MapSpatValidation, consecutive_equal_event_states_report_warning)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(20));

    EXPECT_EQ(Findings { warning_at("eventState sequence", event_path(0, 1)) }, findings(validate_spat(spatem->spat)));
}

TEST(MapSpatValidation, signal_group_255_with_stop_and_remain_reports_error_in_every_profile)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    add_timed_movement(state, 255); // its second event is stop-And-Remain

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(Findings { error_at("SignalGroupID 255", event_path(0, 1)) },
            findings(validate_spat(spatem->spat, profile)));
    }
}

TEST(MapSpatValidation, signal_group_255_with_only_allowed_events_is_not_reported)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 255);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_protected_Movement_Allowed, reference_time, std::chrono::seconds(30));

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, profile)));
    }
}

// =================================================================================================
// validate_spat: C2C-CC RS 2077 rules (Car2Car / Combined)
// =================================================================================================

TEST(MapSpatValidation, spat_rs_arsm_11_missing_region_is_profile_only)
{
    asn1::Spatem spatem;
    IntersectionState& state = add_intersection_state(spatem->spat, intersection_id, intersection_revision);
    set_timestamp(state, reference_time);
    const int bit = IntersectionStatusObject_fixedTimeOperation;
    state.status.buf[bit / 8] |= 0x80 >> (bit % 8);
    add_timed_movement(state, 1);
    // region left unset

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_11", state_path + ".id") },
        findings(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, spat_rs_arsm_12_duplicate_reference_id_reports_error)
{
    asn1::Spatem spatem;
    for (int i = 0; i < 2; ++i) {
        add_timed_movement(build_intersection_state(spatem, 9, 1, reference_time), 1);
    }

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_12", "SPAT.intersections[1].id") },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_56_unknown_min_end_time_is_profile_only_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_timed_movement(state, 1);
    movement.state_time_speed.list.array[0]->timing->minEndTime = cTimeMarkUnknown;

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_56", event_path(0, 0) + ".timing.minEndTime") },
        findings(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_57_max_end_time_with_traffic_dependent_operation_is_car2car_only)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_bare_state(spatem, intersection_id, intersection_revision, reference_time);
    const int bit = IntersectionStatusObject_trafficDependentOperation;
    state.status.buf[bit / 8] |= 0x80 >> (bit % 8);
    MovementState& movement = add_timed_movement(state, 1);
    MovementEvent& event = *movement.state_time_speed.list.array[0];
    remove_optional(event.timing->maxEndTime);

    const std::string path = event_path(0, 0) + ".timing";
    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_57", path) },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
    // under Combined the C-Roads rule makes maxEndTime mandatory regardless of the operation mode
    EXPECT_EQ(Findings { error_at("C-Roads maxEndTime", path) },
        findings(validate_spat(spatem->spat, ValidationProfile::Combined)));

    // with maxEndTime present the requirement is met
    event.timing->maxEndTime = asn1::allocate<TimeMark_t>();
    *event.timing->maxEndTime = event.timing->minEndTime;
    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_64_likely_time_with_traffic_dependent_operation_is_car2car_and_combined)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_bare_state(spatem, intersection_id, intersection_revision, reference_time);
    const int bit = IntersectionStatusObject_trafficDependentOperation;
    state.status.buf[bit / 8] |= 0x80 >> (bit % 8);
    MovementState& movement = add_timed_movement(state, 1);
    remove_optional(movement.state_time_speed.list.array[0]->timing->likelyTime);

    const Findings expected { error_at("RS_ARSM_64", event_path(0, 0) + ".timing") };
    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::CRoads)));
    EXPECT_EQ(expected, findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
    EXPECT_EQ(expected, findings(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_60_unknown_max_end_time_is_profile_only_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_timed_movement(state, 1);
    TimeChangeDetails_t& timing = *movement.state_time_speed.list.array[0]->timing;
    ASSERT_NE(nullptr, timing.maxEndTime);
    *timing.maxEndTime = cTimeMarkUnknown;

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_60", event_path(0, 0) + ".timing.maxEndTime") },
        findings(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_66_unknown_likely_time_is_profile_only_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_timed_movement(state, 1);
    TimeChangeDetails_t& timing = *movement.state_time_speed.list.array[0]->timing;
    ASSERT_NE(nullptr, timing.likelyTime);
    *timing.likelyTime = cTimeMarkUnknown;

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_66", event_path(0, 0) + ".timing.likelyTime") },
        findings(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_61_fixed_time_operation_requires_equal_times)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    MovementEvent& event = add_event(movement, MovementPhaseState_permissive_Movement_Allowed);
    MovementTiming timing;
    timing.min_end = reference_time + std::chrono::seconds(10);
    timing.max_end = reference_time + std::chrono::seconds(12); // not equal to min_end
    timing.likely = timing.min_end;
    timing.confidence = 15;
    set_timing(event, timing, reference_time);
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_61", event_path(0, 0) + ".timing") },
        findings(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_61_likely_time_differing_without_max_end_time_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100, boost::none, 110, 15);
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, cTimeMarkOutOfRange);

    // Car2Car (not Combined): the events deliberately omit maxEndTime
    EXPECT_EQ(Findings { error_at("RS_ARSM_61", event_path(0, 0) + ".timing") },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_61_max_end_time_out_of_range_is_not_reported)
{
    // a maxEndTime of 36000 (time of change not known, RS_ARSM_59) must not be mistaken for a
    // differing value
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100, cTimeMarkOutOfRange);
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, cTimeMarkOutOfRange);

    // Car2Car (not Combined): the completing event deliberately omits maxEndTime, which would
    // otherwise also trigger the unrelated "C-Roads maxEndTime" rule
    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_61_equal_times_is_not_reported)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100, 100, 100, 15);
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, cTimeMarkOutOfRange);

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_61_unknown_min_end_time_with_likely_time_does_not_crash)
{
    // minEndTime 36001 (unknown) resolves to no value; RS_ARSM_61 must short-circuit instead of
    // dereferencing it, while RS_ARSM_56 still reports the unknown minEndTime
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, cTimeMarkUnknown, boost::none, 50, 15);
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, cTimeMarkOutOfRange);

    EXPECT_EQ(Findings { error_at("RS_ARSM_56", event_path(0, 0) + ".timing.minEndTime") },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_65_likely_before_min_end_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_bare_state(spatem, intersection_id, intersection_revision, reference_time);
    const int bit = IntersectionStatusObject_trafficDependentOperation; // avoids RS_ARSM_61 interaction
    state.status.buf[bit / 8] |= 0x80 >> (bit % 8);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 200, 300, 100, 15); // likely < min
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, cTimeMarkOutOfRange, cTimeMarkOutOfRange,
        cTimeMarkOutOfRange, 15);

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_65", event_path(0, 0) + ".timing") },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_65_hour_wrap_is_not_falsely_reported)
{
    // moy refers to minute 59; minEndTime and likelyTime stay in the current hour, maxEndTime
    // (numerically smaller) resolves into the following hour and is chronologically later
    // (RS_ARSM_54); traffic dependent operation avoids an RS_ARSM_61 interaction
    const auto now = Clock::at("2026-10-04 11:59:50.000");
    asn1::Spatem spatem;
    IntersectionState& state = build_bare_state(spatem, intersection_id, intersection_revision, now);
    const int bit = IntersectionStatusObject_trafficDependentOperation;
    state.status.buf[bit / 8] |= 0x80 >> (bit % 8);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 35950, 50, 35950, 15);
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, cTimeMarkOutOfRange, cTimeMarkOutOfRange,
        cTimeMarkOutOfRange, 15);

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_69_extra_status_bit_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_bare_state(spatem, intersection_id, intersection_revision, reference_time);
    const int fixed = IntersectionStatusObject_fixedTimeOperation;
    state.status.buf[fixed / 8] |= 0x80 >> (fixed % 8);
    const int extra = IntersectionStatusObject_manualControlIsEnabled; // bit 0, outside 5..9
    state.status.buf[extra / 8] |= 0x80 >> (extra % 8);
    add_timed_movement(state, 1);

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_69", state_path + ".status") },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_70_two_mode_bits_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_bare_state(spatem, intersection_id, intersection_revision, reference_time);
    const int fixed = IntersectionStatusObject_fixedTimeOperation;
    state.status.buf[fixed / 8] |= 0x80 >> (fixed % 8);
    const int traffic = IntersectionStatusObject_trafficDependentOperation;
    state.status.buf[traffic / 8] |= 0x80 >> (traffic % 8);
    add_timed_movement(state, 1);

    EXPECT_EQ(Findings { error_at("RS_ARSM_70", state_path + ".status") },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_70_no_mode_bit_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_bare_state(spatem, intersection_id, intersection_revision, reference_time);
    add_timed_movement(state, 1); // no status bit set at all

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_70", state_path + ".status") },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_72_event_state_dark_is_car2car_only)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_dark, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(20));

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_72", event_path(0, 0) + ".eventState") },
        findings(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_78_events_out_of_chronological_order_reports_error_at_full_path)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 200);
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, 100); // earlier than the previous event

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    // Car2Car (not Combined): neither event carries a maxEndTime, which would otherwise also
    // trigger the unrelated "C-Roads maxEndTime" rule
    EXPECT_EQ(Findings { error_at("RS_ARSM_78", event_path(0, 1)) },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_78_hour_wrap_is_not_falsely_reported)
{
    // moy refers to minute 59; 35990 (59:59.0) then 10 (next hour, 00:01.0) is chronological
    const auto now = Clock::at("2026-10-04 11:59:30.000");
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, now);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 35990);
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, 10);

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_79_phase_after_unavailable_with_known_end_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100);
    add_raw_event(movement, MovementPhaseState_unavailable, 200); // unavailable is not a phase

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_79", events_path(0)) },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_79_clearance_after_green_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100);
    add_raw_event(movement, MovementPhaseState_permissive_clearance, 200); // clearance is not a phase

    EXPECT_EQ(Findings { error_at("RS_ARSM_79", events_path(0)) },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_79_red_after_clearance_is_not_reported)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100);
    add_raw_event(movement, MovementPhaseState_permissive_clearance, 200);
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, 300); // reaches a phase

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_79_extra_event_after_reaching_phase_is_allowed)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, 100);
    add_raw_event(movement, MovementPhaseState_pre_Movement, 200);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 300); // phase reached
    add_raw_event(movement, MovementPhaseState_permissive_clearance, 400); // extra event afterwards

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_79_single_event_at_time_mark_horizon_is_warning)
{
    // the event ends beyond the TimeMark horizon, so no next phase can be timed: the gap is
    // visible as a warning, not hidden and not an error
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, cTimeMarkOutOfRange); // 36000: beyond the horizon

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { warning_at("RS_ARSM_79", events_path(0)) },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_79_last_event_at_time_mark_horizon_is_warning)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100);
    add_raw_event(movement, MovementPhaseState_permissive_clearance, cTimeMarkOutOfRange); // not a phase

    EXPECT_EQ(Findings { warning_at("RS_ARSM_79", events_path(0)) },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_79_single_event_with_known_end_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100); // known end, no later event

    EXPECT_EQ(Findings { error_at("RS_ARSM_79", events_path(0)) },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_79_single_event_without_timing_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_event(movement, MovementPhaseState_permissive_Movement_Allowed); // no timing at all

    EXPECT_EQ(Findings { error_at("RS_ARSM_79", events_path(0)) },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_115_likely_time_without_confidence_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    // maxEndTime is given (100, same as minEndTime) so the unrelated "C-Roads maxEndTime" rule
    // stays silent under Combined and does not contaminate the expected single issue
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100, 100, 100); // no confidence
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    const std::string path = event_path(0, 0) + ".timing";
    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_115", path) },
        findings(validate_spat(spatem->spat, ValidationProfile::Combined)));
    // C-Roads: confidence "mandatory if likelyTime is provided" (RS 2077 Annex 7.2, C-Roads column)
    EXPECT_EQ(Findings { error_at("C-Roads confidence", path) },
        findings(validate_spat(spatem->spat, ValidationProfile::CRoads)));
}

TEST(MapSpatValidation, rs_arsm_120_missing_timing_before_phase_reports_error_at_full_path)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_event(movement, MovementPhaseState_stop_And_Remain); // event 0, no timing
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100); // event 1, reaches a phase

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    // Car2Car (not Combined): event 1 carries no maxEndTime, which would otherwise also trigger
    // the unrelated "C-Roads maxEndTime" rule
    EXPECT_EQ(Findings { error_at("RS_ARSM_120", event_path(0, 0)) },
        findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_120_missing_timing_on_last_event_is_not_reported)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100); // event 0, reaches a phase itself
    add_event(movement, MovementPhaseState_stop_And_Remain); // event 1, last event, no timing

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

// =================================================================================================
// validate_spat: C-Roads / Annex 7.2 columns
// =================================================================================================

TEST(MapSpatValidation, moy_missing_is_profile_only)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    remove_optional(state.moy);
    add_timed_movement(state, 1);

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    for (ValidationProfile profile :
            { ValidationProfile::Car2Car, ValidationProfile::CRoads, ValidationProfile::Combined }) {
        EXPECT_EQ(Findings { error_at("moy", state_path) }, findings(validate_spat(spatem->spat, profile)));
    }
}

TEST(MapSpatValidation, croads_timestamp_missing_is_profile_only)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    remove_optional(state.timeStamp);
    add_timed_movement(state, 1);

    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
    EXPECT_EQ(Findings { error_at("C-Roads timeStamp", state_path) },
        findings(validate_spat(spatem->spat, ValidationProfile::CRoads)));
    EXPECT_EQ(Findings { error_at("C-Roads timeStamp", state_path) },
        findings(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, croads_max_end_time_required_is_profile_only)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_timed_movement(state, 1);
    remove_optional(movement.state_time_speed.list.array[0]->timing->maxEndTime);

    const Findings expected { error_at("C-Roads maxEndTime", event_path(0, 0) + ".timing") };
    EXPECT_EQ(no_issues, findings(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
    EXPECT_EQ(expected, findings(validate_spat(spatem->spat, ValidationProfile::CRoads)));
    EXPECT_EQ(expected, findings(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

// =================================================================================================
// validate_map_spat
// =================================================================================================

TEST(MapSpatValidation, rs_arsm_68_spat_state_without_map_geometry)
{
    asn1::Mapem mapem; // no intersections at all
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    add_timed_movement(state, 1);

    EXPECT_EQ(Findings { warning_at("map/spat intersection", state_path + ".id") },
        findings(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_68", state_path + ".id") },
        findings(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_13_map_geometry_without_spat_state)
{
    asn1::Mapem mapem;
    build_connected_lanes(build_intersection(mapem, intersection_id, intersection_revision));
    build_connected_lanes(build_intersection(mapem, intersection_id + 1, intersection_revision));

    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    add_timed_movement(state, 1);
    // the second intersection has no matching IntersectionState

    EXPECT_EQ(Findings { warning_at("map/spat intersection", "MapData.intersections[1].id") },
        findings(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_13", "MapData.intersections[1].id") },
        findings(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, same_id_different_region_are_treated_as_different_intersections)
{
    asn1::Mapem mapem;
    build_connected_lanes(build_intersection(mapem));

    asn1::Spatem spatem;
    IntersectionState& state = add_intersection_state(spatem->spat, intersection_id, intersection_revision);
    set_region(state.id, intersection_region + 1); // same id, different region
    set_timestamp(state, reference_time);
    const int bit = IntersectionStatusObject_fixedTimeOperation;
    state.status.buf[bit / 8] |= 0x80 >> (bit % 8);
    add_timed_movement(state, 1);

    const Findings standard_expected {
        warning_at("map/spat intersection", state_path + ".id"),
        warning_at("map/spat intersection", geometry_path + ".id"),
    };
    EXPECT_EQ(standard_expected, findings(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard)));

    const Findings combined_expected {
        error_at("RS_ARSM_68", state_path + ".id"),
        error_at("RS_ARSM_13", geometry_path + ".id"),
    };
    EXPECT_EQ(combined_expected, findings(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, revision_mismatch_always_reports_error)
{
    asn1::Mapem mapem;
    build_connected_lanes(build_intersection(mapem)); // signal group 1, matches the SPAT movement below

    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision + 1,
        reference_time);
    add_timed_movement(state, 1);

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(Findings { error_at("revision", state_path + ".revision") },
            findings(validate_map_spat(mapem->map, spatem->spat, profile)));
    }
}

TEST(MapSpatValidation, rs_arsm_49_unmatched_connection_signal_group_severity_depends_on_profile)
{
    asn1::Mapem mapem;
    GenericLane& ingress = build_connected_lanes(build_intersection(mapem));
    *ingress.connectsTo->list.array[0]->signalGroup = 5; // no matching movement

    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    add_timed_movement(state, 9); // unrelated signal group, itself unmatched too

    // both directions of the pairing are unmatched here; see the RS_ARSM_75 test below for a
    // construction where only one direction is unmatched
    const std::string signal_group_path = connection_path(0, 0) + ".signalGroup";
    const Findings standard_expected {
        warning_at("map/spat signal group", signal_group_path),
        warning_at("map/spat signal group", movement_path(0)),
    };
    EXPECT_EQ(standard_expected, findings(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard)));

    const Findings combined_expected {
        error_at("RS_ARSM_49", signal_group_path),
        error_at("RS_ARSM_75", movement_path(0)),
    };
    EXPECT_EQ(combined_expected, findings(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_75_unmatched_movement_signal_group_severity_depends_on_profile)
{
    asn1::Mapem mapem;
    build_connected_lanes(build_intersection(mapem));

    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    add_timed_movement(state, 1);
    add_timed_movement(state, 7); // no connection uses this signal group

    EXPECT_EQ(Findings { warning_at("map/spat signal group", movement_path(1)) },
        findings(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(Findings { error_at("RS_ARSM_75", movement_path(1)) },
        findings(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, fragment_does_not_report_unmatched_spat_signal_group)
{
    // RS_ARSM_75 ("every MovementState signal group is used by a connection") is not checked for
    // a MapData fragment, since a signal group may be described by another fragment's geometry
    asn1::Mapem mapem;
    build_connected_lanes(build_intersection(mapem));
    set_layer_id(mapem->map, 21); // first of two fragments

    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    add_timed_movement(state, 1);
    add_timed_movement(state, 99); // described by another fragment

    EXPECT_EQ(no_issues, findings(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(no_issues, findings(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, invalid_layer_id_does_not_relax_map_spat_checks)
{
    asn1::Mapem mapem;
    build_connected_lanes(build_intersection(mapem));
    set_layer_id(mapem->map, 1); // not a fragment number, reported by validate_map

    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    add_timed_movement(state, 1);
    add_timed_movement(state, 99);

    EXPECT_EQ(Findings { error_at("RS_ARSM_75", movement_path(1)) },
        findings(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined)));
}

// =================================================================================================
// Standard profile must never leak a Car2Car (RS_ARSM_*) or C-Roads identifier
// =================================================================================================

TEST(MapSpatValidation, standard_profile_never_reports_profile_specific_identifiers)
{
    // every scenario violates a profile rule, which Combined has to report and Standard must not;
    // all of them are ASN.1-valid, otherwise the semantic checks would not even run
    std::vector<ValidationResult (*)(ValidationProfile)> scenarios {
        // validate_map, RS_ARSM_*
        [](ValidationProfile profile) {
            asn1::Mapem mapem;
            add_ingress_lane(add_intersection_geometry(mapem->map, intersection_id, intersection_revision,
                59.0 * units::degree, 30.0 * units::degree, 3.5 * meter)); // no region: RS_ARSM_11
            return validate_map(mapem->map, profile);
        },
        [](ValidationProfile profile) {
            asn1::Mapem mapem;
            add_ingress_lane(build_intersection(mapem, 9, 1));
            add_ingress_lane(build_intersection(mapem, 9, 1)); // RS_ARSM_12
            return validate_map(mapem->map, profile);
        },
        [](ValidationProfile profile) {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
            remove_optional(intersection.laneWidth); // RS_ARSM_14
            return validate_map(mapem->map, profile);
        },
        [](ValidationProfile profile) {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
            remove_optional(intersection.laneSet.list.array[0]->ingressApproach); // RS_ARSM_16
            return validate_map(mapem->map, profile);
        },
        [](ValidationProfile profile) {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_intersection(mapem);
            GenericLane& crosswalk = add_crosswalk_lane(intersection, 1, 1, 2);
            add_node(crosswalk, -5.0 * meter, 0.0 * meter);
            add_node(crosswalk, 5.0 * meter, 0.0 * meter);
            remove_optional(crosswalk.egressApproach); // RS_ARSM_17
            return validate_map(mapem->map, profile);
        },
        [](ValidationProfile profile) {
            asn1::Mapem mapem;
            GenericLane& ingress = build_connected_lanes(build_intersection(mapem));
            connect(ingress, 2, Maneuver::Straight, 1, 2); // RS_ARSM_20
            return validate_map(mapem->map, profile);
        },
        [](ValidationProfile profile) {
            asn1::Mapem mapem;
            GenericLane& ingress = build_connected_lanes(build_intersection(mapem));
            Connection_t& connection = *ingress.connectsTo->list.array[0];
            remove_owned(connection.connectingLane.maneuver, asn_DEF_AllowedManeuvers); // RS_ARSM_21
            return validate_map(mapem->map, profile);
        },
        [](ValidationProfile profile) {
            asn1::Mapem mapem;
            GenericLane& ingress = build_connected_lanes(build_intersection(mapem));
            ingress.connectsTo->list.array[0]->connectingLane.maneuver->buf[0] = 0; // RS_ARSM_22
            return validate_map(mapem->map, profile);
        },
        [](ValidationProfile profile) {
            asn1::Mapem mapem;
            GenericLane& ingress = build_connected_lanes(build_intersection(mapem));
            Connection_t& connection = *ingress.connectsTo->list.array[0];
            connection.connectingLane.maneuver->buf[0] |= 0x80 >> AllowedManeuvers_maneuverLaneChangeAllowed; // RS24
            return validate_map(mapem->map, profile);
        },
        [](ValidationProfile profile) {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_intersection(mapem);
            GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
            add_node(lane, 0.0 * meter, -20.0 * meter);
            for (int i = 0; i < 18; ++i) {
                add_node(lane, 0.0 * meter, 1.0 * meter); // RS_ARSM_35
            }
            return validate_map(mapem->map, profile);
        },
        [](ValidationProfile profile) {
            asn1::Mapem mapem;
            GenericLane& ingress = build_connected_lanes(build_intersection(mapem));
            remove_optional(ingress.connectsTo->list.array[0]->signalGroup); // RS_ARSM_48
            return validate_map(mapem->map, profile);
        },
        [](ValidationProfile profile) {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
            GenericLane& lane = *intersection.laneSet.list.array[0];
            lane.maneuvers = asn1::allocate<AllowedManeuvers_t>(); // RS_ARSM_117, BIT STRING (SIZE(12))
            lane.maneuvers->buf = static_cast<uint8_t*>(asn1::allocate(2));
            lane.maneuvers->size = 2;
            lane.maneuvers->buf[0] = 0x80 >> AllowedManeuvers_maneuverStraightAllowed;
            lane.maneuvers->bits_unused = 4;
            return validate_map(mapem->map, profile);
        },
        [](ValidationProfile profile) {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_intersection(mapem);
            GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
            lane.nodeList.present = NodeListXY_PR_computed; // RS_ARSM_118
            ComputedLane_t& computed = lane.nodeList.choice.computed;
            computed.referenceLaneId = 1;
            computed.offsetXaxis.present = ComputedLane__offsetXaxis_PR_small;
            computed.offsetYaxis.present = ComputedLane__offsetYaxis_PR_small;
            return validate_map(mapem->map, profile);
        },
        [](ValidationProfile profile) {
            asn1::Mapem mapem; // "C-Roads intersections"
            return validate_map(mapem->map, profile);
        },
        [](ValidationProfile profile) {
            asn1::Mapem mapem;
            GenericLane& ingress = build_connected_lanes(build_intersection(mapem));
            remove_optional(ingress.connectsTo->list.array[0]->connectionID); // "C-Roads connectionID"
            return validate_map(mapem->map, profile);
        },
        // validate_spat, RS_ARSM_*
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            IntersectionState& state = add_intersection_state(spatem->spat, intersection_id, intersection_revision);
            set_timestamp(state, reference_time); // no region: RS_ARSM_11
            const int bit = IntersectionStatusObject_fixedTimeOperation;
            state.status.buf[bit / 8] |= 0x80 >> (bit % 8);
            add_timed_movement(state, 1);
            return validate_spat(spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            for (int i = 0; i < 2; ++i) {
                add_timed_movement(build_intersection_state(spatem, 9, 1, reference_time), 1); // RS_ARSM_12
            }
            return validate_spat(spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_timed_movement(state, 1);
            movement.state_time_speed.list.array[0]->timing->minEndTime = cTimeMarkUnknown; // RS_ARSM_56
            return validate_spat(spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            IntersectionState& state = build_bare_state(spatem, intersection_id, intersection_revision, reference_time);
            const int bit = IntersectionStatusObject_trafficDependentOperation;
            state.status.buf[bit / 8] |= 0x80 >> (bit % 8);
            MovementState& movement = add_timed_movement(state, 1);
            TimeChangeDetails_t& timing = *movement.state_time_speed.list.array[0]->timing;
            remove_optional(timing.maxEndTime); // RS_ARSM_57 / "C-Roads maxEndTime"
            remove_optional(timing.likelyTime); // RS_ARSM_64
            return validate_spat(spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_timed_movement(state, 1);
            *movement.state_time_speed.list.array[0]->timing->maxEndTime = cTimeMarkUnknown; // RS_ARSM_60
            return validate_spat(spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_timed_movement(state, 1);
            *movement.state_time_speed.list.array[0]->timing->likelyTime = cTimeMarkUnknown; // RS_ARSM_66
            return validate_spat(spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100, 200, 100, 15); // RS_ARSM_61
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            return validate_spat(spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 200, 300, 100, 15); // RS_ARSM_65
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            return validate_spat(spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            IntersectionState& state = build_bare_state(spatem, intersection_id, intersection_revision, reference_time);
            const int fixed = IntersectionStatusObject_fixedTimeOperation;
            state.status.buf[fixed / 8] |= 0x80 >> (fixed % 8);
            const int extra = IntersectionStatusObject_manualControlIsEnabled; // RS_ARSM_69
            state.status.buf[extra / 8] |= 0x80 >> (extra % 8);
            add_timed_movement(state, 1);
            return validate_spat(spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            IntersectionState& state = build_bare_state(spatem, intersection_id, intersection_revision, reference_time);
            add_timed_movement(state, 1); // no status bit set: RS_ARSM_70
            return validate_spat(spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            add_known_event(movement, MovementPhaseState_dark, reference_time, std::chrono::seconds(10)); // RS_ARSM_72
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(20));
            return validate_spat(spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 200); // RS_ARSM_78
            add_raw_event(movement, MovementPhaseState_stop_And_Remain, 100);
            return validate_spat(spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100); // RS_ARSM_79
            return validate_spat(spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100, boost::none, 100); // RS115
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            return validate_spat(spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            add_event(movement, MovementPhaseState_stop_And_Remain); // RS_ARSM_120
            add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100);
            return validate_spat(spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            remove_optional(state.timeStamp); // "C-Roads timeStamp"
            add_timed_movement(state, 1);
            return validate_spat(spatem->spat, profile);
        },
        // validate_map_spat
        [](ValidationProfile profile) {
            asn1::Mapem mapem; // RS_ARSM_68 (no intersections at all)
            asn1::Spatem spatem;
            add_timed_movement(build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time), 1);
            return validate_map_spat(mapem->map, spatem->spat, profile);
        },
        [](ValidationProfile profile) {
            asn1::Mapem mapem;
            GenericLane& ingress = build_connected_lanes(build_intersection(mapem));
            *ingress.connectsTo->list.array[0]->signalGroup = 5; // RS_ARSM_49 / RS_ARSM_75
            asn1::Spatem spatem;
            add_timed_movement(build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time), 9);
            return validate_map_spat(mapem->map, spatem->spat, profile);
        },
    };

    for (std::size_t i = 0; i < scenarios.size(); ++i) {
        const ValidationResult standard = scenarios[i](ValidationProfile::Standard);
        const ValidationResult combined = scenarios[i](ValidationProfile::Combined);
        EXPECT_FALSE(has_rule(standard, "ASN.1")) << "scenario " << i << " is not ASN.1-valid";
        EXPECT_FALSE(has_profile_specific_identifier(standard)) << "scenario " << i << " leaked a profile identifier";
        EXPECT_TRUE(has_profile_specific_identifier(combined)) << "scenario " << i << " violates no profile rule";
    }
}

// =================================================================================================
// integration: SPaT generated by the fixed-time plan model meets the Car2Car profile
// =================================================================================================

TEST(MapSpatValidation, fixed_time_plan_output_meets_car2car_profile)
{
    using std::chrono::hours;
    using std::chrono::milliseconds;
    using std::chrono::seconds;
    const long permissive = MovementPhaseState_permissive_Movement_Allowed;
    const long clearance = MovementPhaseState_permissive_clearance;
    const long always_red_group = 4;

    ControllerConfig config;
    const auto group = [&](long id, SignalGroupKind kind) {
        config.groups.push_back({ id, kind, seconds(2), seconds(3), seconds(3), permissive, clearance });
    };
    group(1, SignalGroupKind::Traffic);
    group(2, SignalGroupKind::Traffic);
    group(3, SignalGroupKind::Pedestrian);
    group(always_red_group, SignalGroupKind::Traffic);
    config.stages = { { 1, { 1, 3 } }, { 2, { 2 } } };
    for (long main : { 1, 3 }) {
        config.intergreen[{ main, 2 }] = seconds(3);
        config.intergreen[{ 2, main }] = seconds(6);
    }
    config.plans = {
        { 1, { { 1, seconds(38) }, { 2, seconds(26) } }, seconds(45) },
        { 2, { { 1, seconds(54) }, { 2, seconds(26) } }, seconds(24) },
    };
    for (auto& day : config.week) {
        day = { { hours(0), 1 } };
    }
    config.week[0].push_back({ hours(7), 2 }); // Monday only
    config.utc_offset = Clock::duration::zero();
    const FixedTimePlan controller(config); // group 4 is in no stage, always red

    // across a cycle, the hour boundary and the plan change at Monday 07:00
    const Clock::time_point starts[] = {
        Clock::at("2026-10-04 12:58:50.000"), Clock::at("2026-10-05 06:58:50.000"),
    };
    for (const Clock::time_point& start : starts) {
        for (int step = 0; step < 160; ++step) {
            const Clock::time_point now = start + milliseconds(700 * step);
            asn1::Spatem spatem;
            controller.fill(spatem->spat, intersection_id, intersection_revision, now);
            set_region(spatem->spat.intersections.list.array[0]->id, intersection_region);

            // the always red group has one event ending beyond the TimeMark horizon, so its next
            // phase cannot be listed (RS_ARSM_79 warning); every other group meets the profile
            Findings expected;
            const IntersectionState& state = *spatem->spat.intersections.list.array[0];
            for (int j = 0; j < state.states.list.count; ++j) {
                if (state.states.list.array[j]->signalGroup == always_red_group) {
                    expected.insert(warning_at("RS_ARSM_79", events_path(j)));
                }
            }
            ASSERT_EQ(1u, expected.size()) << "always red group missing at step " << step;

            const ValidationResult result = validate_spat(spatem->spat, ValidationProfile::Car2Car);
            std::ostringstream issues;
            for (const ValidationIssue& issue : result.issues()) {
                issues << issue << "\n";
            }
            EXPECT_EQ(expected, findings(result)) << "at step " << step << "\n" << issues.str();
        }
    }
}
