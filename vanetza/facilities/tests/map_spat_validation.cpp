#include <gtest/gtest.h>
#include <vanetza/asn1/mapem.hpp>
#include <vanetza/asn1/spatem.hpp>
#include <vanetza/asn1/its/AllowedManeuvers.h>
#include <vanetza/asn1/its/ComputedLane.h>
#include <vanetza/asn1/its/Connection.h>
#include <vanetza/asn1/its/ConnectsToList.h>
#include <vanetza/asn1/its/GenericLane.h>
#include <vanetza/asn1/its/IntersectionGeometry.h>
#include <vanetza/asn1/its/IntersectionReferenceID.h>
#include <vanetza/asn1/its/IntersectionStatusObject.h>
#include <vanetza/asn1/its/LayerID.h>
#include <vanetza/asn1/its/LayerType.h>
#include <vanetza/asn1/its/MovementEvent.h>
#include <vanetza/asn1/its/MovementPhaseState.h>
#include <vanetza/asn1/its/MovementState.h>
#include <vanetza/asn1/its/NodeListXY.h>
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
#include <sstream>
#include <string>
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

bool has_issue(const ValidationResult& result, const std::string& rule, Severity severity,
        const std::string& path_substring = "")
{
    for (const ValidationIssue& issue : result.issues()) {
        if (issue.rule == rule && issue.severity == severity &&
                (path_substring.empty() || issue.path.find(path_substring) != std::string::npos)) {
            return true;
        }
    }
    return false;
}

// sorted rule identifiers of a result; used for exact-content assertions, since checking only
// that an expected issue is present ("contains") would miss extra or missing issues
std::vector<std::string> rule_list(const ValidationResult& result)
{
    std::vector<std::string> rules;
    for (const ValidationIssue& issue : result.issues()) {
        rules.push_back(issue.rule);
    }
    std::sort(rules.begin(), rules.end());
    return rules;
}

const std::vector<std::string> no_issues {};

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
        EXPECT_EQ(no_issues, rule_list(validate_map(baseline.mapem->map, profile)));
    }
}

TEST(MapSpatValidation, baseline_spat_has_no_issues_in_any_profile)
{
    Baseline baseline = build_baseline(reference_time);
    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(no_issues, rule_list(validate_spat(baseline.spatem->spat, profile)));
    }
}

TEST(MapSpatValidation, baseline_map_spat_has_no_issues_in_any_profile)
{
    Baseline baseline = build_baseline(reference_time);
    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(no_issues, rule_list(validate_map_spat(baseline.mapem->map, baseline.spatem->spat, profile)));
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
// structure: ASN.1 constraints and SEQUENCE OF size bounds skip all semantic checks, no leaks
// =================================================================================================

TEST(MapSpatValidation, map_asn1_constraint_violation_reports_only_one_issue)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -10.0 * meter);
    add_node(lane, 0.0 * meter, -5.0 * meter);
    lane.laneID = 256; // LaneID ::= INTEGER (0..255)

    ValidationResult result = validate_map(mapem->map);
    ASSERT_EQ(1u, result.issues().size());
    EXPECT_EQ(Severity::Error, result.issues()[0].severity);
    EXPECT_EQ("ASN.1", result.issues()[0].rule);
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
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    ValidationResult result = validate_spat(spatem->spat);
    ASSERT_EQ(1u, result.issues().size());
    EXPECT_EQ(Severity::Error, result.issues()[0].severity);
    EXPECT_EQ("ASN.1", result.issues()[0].rule);
}

TEST(MapSpatValidation, map_spat_returns_no_issues_when_one_message_violates_constraints)
{
    Baseline baseline = build_baseline(reference_time);
    MovementState& movement = *baseline.spatem->spat.intersections.list.array[0]->states.list.array[0];
    movement.signalGroup = 256; // SignalGroupID ::= INTEGER (0..255), MAP side stays fully valid

    ValidationResult result = validate_map_spat(baseline.mapem->map, baseline.spatem->spat);
    EXPECT_EQ(no_issues, rule_list(result));
}

TEST(MapSpatValidation, empty_lane_set_element_is_reported_as_single_asn1_issue)
{
    Baseline baseline = build_baseline(reference_time);
    IntersectionGeometry& intersection = *baseline.mapem->map.intersections->list.array[0];
    GenericLane*& slot = intersection.laneSet.list.array[2]; // crosswalk lane, not a connection target
    remove_owned(slot, asn_DEF_GenericLane);

    ValidationResult result = validate_map(baseline.mapem->map);
    ASSERT_EQ(1u, result.issues().size());
    EXPECT_EQ(Severity::Error, result.issues()[0].severity);
    EXPECT_EQ("ASN.1", result.issues()[0].rule);
    EXPECT_EQ("MapData.intersections[0].laneSet[2]", result.issues()[0].path);
}

TEST(MapSpatValidation, lane_with_single_node_violates_node_set_size_bound)
{
    // ISO TS 19091 NodeSetXY ::= SEQUENCE (SIZE(2..63)) OF NodeXY
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -10.0 * meter);

    ValidationResult result = validate_map(mapem->map);
    ASSERT_EQ(1u, result.issues().size());
    EXPECT_EQ(Severity::Error, result.issues()[0].severity);
    EXPECT_EQ("ASN.1", result.issues()[0].rule);
    EXPECT_EQ("MapData.intersections[0].laneSet[0].nodeList.nodes", result.issues()[0].path);
}

TEST(MapSpatValidation, empty_movement_event_list_violates_size_bound)
{
    // ISO TS 19091 MovementEventList ::= SEQUENCE (SIZE(1..16)) OF MovementEvent
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    add_movement(state, 1); // no events added

    ValidationResult result = validate_spat(spatem->spat);
    ASSERT_EQ(1u, result.issues().size());
    EXPECT_EQ(Severity::Error, result.issues()[0].severity);
    EXPECT_EQ("ASN.1", result.issues()[0].rule);
    EXPECT_EQ("SPAT.intersections[0].states[0].state-time-speed", result.issues()[0].path);
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
        EXPECT_EQ(std::vector<std::string>{"msgIssueRevision"}, rule_list(validate_map(mapem->map, profile)));
    }
}

TEST(MapSpatValidation, layer_type_present_reports_error_in_every_profile)
{
    asn1::Mapem mapem;
    build_single_lane_intersection(mapem);
    mapem->map.layerType = asn1::allocate<LayerType_t>();
    *mapem->map.layerType = LayerType_intersectionData;

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(std::vector<std::string>{"layerType"}, rule_list(validate_map(mapem->map, profile)));
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

    ValidationResult result = validate_map(mapem->map);
    ASSERT_EQ(1u, result.issues().size());
    EXPECT_EQ(Severity::Error, result.issues()[0].severity);
    EXPECT_EQ("LaneID unique", result.issues()[0].rule);
    EXPECT_EQ("MapData.intersections[0].laneSet[1]", result.issues()[0].path);
}

TEST(MapSpatValidation, lane_id_zero_reports_warning_only)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 0, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -10.0 * meter);
    add_node(lane, 0.0 * meter, -5.0 * meter);

    ValidationResult result = validate_map(mapem->map);
    EXPECT_EQ(std::vector<std::string>{"LaneID value"}, rule_list(result));
    EXPECT_EQ(Severity::Warning, result.issues()[0].severity);
}

TEST(MapSpatValidation, lane_id_reserved_255_reports_warning_only)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 255, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -10.0 * meter);
    add_node(lane, 0.0 * meter, -5.0 * meter);

    ValidationResult result = validate_map(mapem->map);
    EXPECT_EQ(std::vector<std::string>{"LaneID value"}, rule_list(result));
    EXPECT_EQ(Severity::Warning, result.issues()[0].severity);
}

TEST(MapSpatValidation, dangling_connecting_lane_reports_error_with_full_path)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    GenericLane& ingress = *intersection.laneSet.list.array[0];
    connect(ingress, 99, Maneuver::Straight, 1, 1); // lane 99 does not exist

    ValidationResult result = validate_map(mapem->map);
    ASSERT_EQ(1u, result.issues().size());
    EXPECT_EQ(Severity::Error, result.issues()[0].severity);
    EXPECT_EQ("connectingLane", result.issues()[0].rule);
    EXPECT_EQ("MapData.intersections[0].laneSet[0].connectsTo[0].connectingLane.lane", result.issues()[0].path);
}

TEST(MapSpatValidation, connecting_lane_target_zero_is_warning_in_every_profile)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    connect(*intersection.laneSet.list.array[0], 0, Maneuver::Straight, 1, 1);

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(std::vector<std::string>{"connectingLane"}, rule_list(validate_map(mapem->map, profile)));
        EXPECT_TRUE(has_issue(validate_map(mapem->map, profile), "connectingLane", Severity::Warning));
    }
}

TEST(MapSpatValidation, connecting_lane_target_255_is_warning_in_every_profile)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    connect(*intersection.laneSet.list.array[0], 255, Maneuver::Straight, 1, 1);

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(std::vector<std::string>{"connectingLane"}, rule_list(validate_map(mapem->map, profile)));
        EXPECT_TRUE(has_issue(validate_map(mapem->map, profile), "connectingLane", Severity::Warning));
    }
}

TEST(MapSpatValidation, connecting_lane_target_missing_is_error_without_fragment)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    connect(*intersection.laneSet.list.array[0], 9, Maneuver::Straight, 1, 1); // lane 9 does not exist

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(std::vector<std::string>{"connectingLane"}, rule_list(validate_map(mapem->map, profile)));
        EXPECT_TRUE(has_issue(validate_map(mapem->map, profile), "connectingLane", Severity::Error));
    }
}

TEST(MapSpatValidation, connecting_lane_target_missing_is_warning_for_a_fragment)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    connect(*intersection.laneSet.list.array[0], 9, Maneuver::Straight, 1, 1); // target may be in another fragment
    set_layer_id(mapem->map, 1);

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(std::vector<std::string>{"connectingLane"}, rule_list(validate_map(mapem->map, profile)));
        EXPECT_TRUE(has_issue(validate_map(mapem->map, profile), "connectingLane", Severity::Warning));
    }
}

TEST(MapSpatValidation, connection_id_reused_with_different_signal_groups_warns)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(ingress, 0.0 * meter, -10.0 * meter);
    add_node(ingress, 0.0 * meter, -5.0 * meter);
    GenericLane& egress_a = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
    add_node(egress_a, 0.0 * meter, 5.0 * meter);
    add_node(egress_a, 0.0 * meter, 10.0 * meter);
    GenericLane& egress_b = add_vehicle_lane(intersection, 3, TravelDirection::Egress, 3);
    add_node(egress_b, 5.0 * meter, 0.0 * meter);
    add_node(egress_b, 10.0 * meter, 0.0 * meter);

    connect(ingress, 2, Maneuver::Straight, 1, 1);
    connect(ingress, 3, Maneuver::Right, 2, 1); // same connectionID 1, different signal group

    ValidationResult result = validate_map(mapem->map);
    EXPECT_EQ(std::vector<std::string>{"connectionID"}, rule_list(result));
    EXPECT_EQ(Severity::Warning, result.issues()[0].severity);
}

TEST(MapSpatValidation, connection_id_reused_with_same_signal_group_is_not_reported)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(ingress, 0.0 * meter, -10.0 * meter);
    add_node(ingress, 0.0 * meter, -5.0 * meter);
    GenericLane& egress_a = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
    add_node(egress_a, 0.0 * meter, 5.0 * meter);
    add_node(egress_a, 0.0 * meter, 10.0 * meter);
    GenericLane& egress_b = add_vehicle_lane(intersection, 3, TravelDirection::Egress, 3);
    add_node(egress_b, 5.0 * meter, 0.0 * meter);
    add_node(egress_b, 10.0 * meter, 0.0 * meter);

    connect(ingress, 2, Maneuver::Straight, 1, 1);
    connect(ingress, 3, Maneuver::Right, 1, 2); // same connectionID 1, same signal group

    ValidationResult result = validate_map(mapem->map, ValidationProfile::Combined);
    EXPECT_EQ(no_issues, rule_list(result));
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
        ValidationResult result = validate_map(mapem->map, profile);
        EXPECT_TRUE(has_issue(result, "crosswalk self-loop", Severity::Warning));
        EXPECT_FALSE(has_issue(result, "crosswalk self-loop", Severity::Error));
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

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_11"}, rule_list(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_12_duplicate_reference_id_reports_error)
{
    asn1::Mapem mapem;
    add_ingress_lane(build_intersection(mapem, 9, 1));
    add_ingress_lane(build_intersection(mapem, 9, 1)); // same id and region

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_12"}, rule_list(validate_map(mapem->map, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_14_missing_lane_width_is_profile_only)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    remove_optional(intersection.laneWidth);

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_14"}, rule_list(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_16_unidirectional_lane_with_both_approaches_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem); // ingress lane, ingressApproach only
    GenericLane& lane = *intersection.laneSet.list.array[0];
    lane.egressApproach = asn1::allocate<ApproachID_t>();
    *lane.egressApproach = 2; // now both approaches set although the lane is unidirectional

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_16"}, rule_list(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_16_unidirectional_lane_with_no_approach_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    GenericLane& lane = *intersection.laneSet.list.array[0];
    remove_optional(lane.ingressApproach); // neither approach set now

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_16"}, rule_list(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_17_crosswalk_missing_approach_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& crosswalk = add_crosswalk_lane(intersection, 1, 1, 2);
    add_node(crosswalk, -5.0 * meter, 0.0 * meter);
    add_node(crosswalk, 5.0 * meter, 0.0 * meter);
    remove_optional(crosswalk.egressApproach);

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_17"}, rule_list(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_20_duplicate_connection_same_direction_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(ingress, 0.0 * meter, -10.0 * meter);
    add_node(ingress, 0.0 * meter, -5.0 * meter);
    GenericLane& egress = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
    add_node(egress, 0.0 * meter, 5.0 * meter);
    add_node(egress, 0.0 * meter, 10.0 * meter);
    connect(ingress, 2, Maneuver::Straight, 1, 1);
    connect(ingress, 2, Maneuver::Straight, 1, 2); // second connection to the same lane, same direction

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_20"}, rule_list(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_20_different_direction_to_same_lane_is_not_reported)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(ingress, 0.0 * meter, -10.0 * meter);
    add_node(ingress, 0.0 * meter, -5.0 * meter);
    GenericLane& egress = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
    add_node(egress, 0.0 * meter, 5.0 * meter);
    add_node(egress, 0.0 * meter, 10.0 * meter);
    connect(ingress, 2, Maneuver::Straight, 1, 1);
    connect(ingress, 2, Maneuver::Left, 1, 2); // different direction, same target lane

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_21_missing_maneuver_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    GenericLane& ingress = *intersection.laneSet.list.array[0];
    connect(ingress, 1, Maneuver::Straight, 1, 1); // connects to itself, irrelevant here
    Connection_t& connection = *ingress.connectsTo->list.array[0];
    remove_owned(connection.connectingLane.maneuver, asn_DEF_AllowedManeuvers);

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    ValidationResult combined = validate_map(mapem->map, ValidationProfile::Combined);
    EXPECT_TRUE(has_issue(combined, "RS_ARSM_21", Severity::Error));
    EXPECT_FALSE(has_issue(combined, "RS_ARSM_22", Severity::Error)); // no maneuver bits to miscount
}

TEST(MapSpatValidation, rs_arsm_22_connection_with_two_maneuver_bits_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(ingress, 0.0 * meter, -10.0 * meter);
    add_node(ingress, 0.0 * meter, -5.0 * meter);
    GenericLane& egress = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
    add_node(egress, 0.0 * meter, 5.0 * meter);
    add_node(egress, 0.0 * meter, 10.0 * meter);
    connect(ingress, 2, Maneuver::Straight, 1, 1);

    ASSERT_NE(nullptr, ingress.connectsTo);
    Connection_t& connection = *ingress.connectsTo->list.array[0];
    ASSERT_NE(nullptr, connection.connectingLane.maneuver);
    connection.connectingLane.maneuver->buf[0] |= 0x80 >> AllowedManeuvers_maneuverRightAllowed;

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_22"}, rule_list(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_22_connection_with_no_maneuver_bits_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(ingress, 0.0 * meter, -10.0 * meter);
    add_node(ingress, 0.0 * meter, -5.0 * meter);
    GenericLane& egress = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
    add_node(egress, 0.0 * meter, 5.0 * meter);
    add_node(egress, 0.0 * meter, 10.0 * meter);
    connect(ingress, 2, Maneuver::Straight, 1, 1);

    ASSERT_NE(nullptr, ingress.connectsTo);
    Connection_t& connection = *ingress.connectsTo->list.array[0];
    ASSERT_NE(nullptr, connection.connectingLane.maneuver);
    connection.connectingLane.maneuver->buf[0] = 0; // clear the straight bit, zero bits left

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_22"}, rule_list(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_24_turn_on_red_bit_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    GenericLane& ingress = *intersection.laneSet.list.array[0];
    connect(ingress, 1, Maneuver::Straight, 1, 1);
    Connection_t& connection = *ingress.connectsTo->list.array[0];
    connection.connectingLane.maneuver->buf[0] |= 0x80 >> AllowedManeuvers_maneuverLeftTurnOnRedAllowed;

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    ValidationResult combined = validate_map(mapem->map, ValidationProfile::Combined);
    EXPECT_TRUE(has_issue(combined, "RS_ARSM_24", Severity::Error));
    EXPECT_FALSE(has_issue(combined, "RS_ARSM_22", Severity::Error)); // straight bit still the only direction bit
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

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_35"}, rule_list(validate_map(mapem->map, ValidationProfile::Combined)));
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

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_48_connection_without_signal_group_is_profile_only_warning)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(ingress, 0.0 * meter, -10.0 * meter);
    add_node(ingress, 0.0 * meter, -5.0 * meter);
    GenericLane& egress = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
    add_node(egress, 0.0 * meter, 5.0 * meter);
    add_node(egress, 0.0 * meter, 10.0 * meter);
    connect(ingress, 2, Maneuver::Straight, 1, 1);

    ASSERT_NE(nullptr, ingress.connectsTo);
    remove_optional(ingress.connectsTo->list.array[0]->signalGroup);

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_48"}, rule_list(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_117_lane_level_maneuvers_is_profile_only_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -10.0 * meter);
    add_node(lane, 0.0 * meter, -5.0 * meter);
    // AllowedManeuvers ::= BIT STRING (SIZE(12))
    lane.maneuvers = asn1::allocate<AllowedManeuvers_t>();
    lane.maneuvers->buf = static_cast<uint8_t*>(asn1::allocate(2));
    lane.maneuvers->size = 2;
    lane.maneuvers->buf[0] = 0x80 >> AllowedManeuvers_maneuverStraightAllowed;
    lane.maneuvers->bits_unused = 4;

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_117"}, rule_list(validate_map(mapem->map, ValidationProfile::Combined)));
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

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    for (ValidationProfile profile : { ValidationProfile::Car2Car, ValidationProfile::CRoads, ValidationProfile::Combined }) {
        EXPECT_EQ(std::vector<std::string>{"RS_ARSM_118"}, rule_list(validate_map(mapem->map, profile)));
    }
}

// =================================================================================================
// validate_map: C-Roads columns of C2C-CC RS 2077 Annex 7.1
// =================================================================================================

TEST(MapSpatValidation, croads_intersections_missing_is_profile_only)
{
    asn1::Mapem mapem; // no intersections at all (optional, absent)

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Car2Car)));
    EXPECT_EQ(std::vector<std::string>{"C-Roads intersections"},
        rule_list(validate_map(mapem->map, ValidationProfile::CRoads)));
    EXPECT_EQ(std::vector<std::string>{"C-Roads intersections"},
        rule_list(validate_map(mapem->map, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, croads_connection_id_missing_is_profile_only)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
    GenericLane& ingress = *intersection.laneSet.list.array[0];
    connect(ingress, 1, Maneuver::Straight, 1, 1);
    Connection_t& connection = *ingress.connectsTo->list.array[0];
    remove_optional(connection.connectionID);

    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Standard)));
    EXPECT_EQ(no_issues, rule_list(validate_map(mapem->map, ValidationProfile::Car2Car)));
    EXPECT_EQ(std::vector<std::string>{"C-Roads connectionID"},
        rule_list(validate_map(mapem->map, ValidationProfile::CRoads)));
}

// =================================================================================================
// validate_spat: rules without an RS identifier
// =================================================================================================

TEST(MapSpatValidation, spat_asn1_constraint_violation_reports_only_one_issue)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    movement.signalGroup = 256; // SignalGroupID ::= INTEGER (0..255)

    ValidationResult result = validate_spat(spatem->spat);
    ASSERT_EQ(1u, result.issues().size());
    EXPECT_EQ(Severity::Error, result.issues()[0].severity);
    EXPECT_EQ("ASN.1", result.issues()[0].rule);
}

TEST(MapSpatValidation, duplicate_signal_group_reports_warning)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& a = add_movement(state, 1);
    add_known_event(a, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(a, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    MovementState& b = add_movement(state, 1);
    add_known_event(b, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(b, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    ValidationResult result = validate_spat(spatem->spat);
    EXPECT_EQ(std::vector<std::string>{"SignalGroupID unique"}, rule_list(result));
}

TEST(MapSpatValidation, signal_group_zero_reports_warning)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 0);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    ValidationResult result = validate_spat(spatem->spat);
    EXPECT_EQ(std::vector<std::string>{"SignalGroupID value"}, rule_list(result));
}

TEST(MapSpatValidation, consecutive_equal_event_states_report_warning)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(20));

    ValidationResult result = validate_spat(spatem->spat);
    EXPECT_EQ(std::vector<std::string>{"eventState sequence"}, rule_list(result));
}

TEST(MapSpatValidation, signal_group_255_with_stop_and_remain_reports_error_in_every_profile)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 255);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(std::vector<std::string>{"SignalGroupID 255"}, rule_list(validate_spat(spatem->spat, profile)));
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
        EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, profile)));
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
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    // region left unset

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_11"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, spat_rs_arsm_12_duplicate_reference_id_reports_error)
{
    asn1::Spatem spatem;
    for (int i = 0; i < 2; ++i) {
        IntersectionState& state = build_intersection_state(spatem, 9, 1, reference_time);
        MovementState& movement = add_movement(state, 1);
        add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
        add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    }

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_12"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_56_unknown_min_end_time_is_profile_only_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    MovementEvent& event = add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time,
        std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    event.timing->minEndTime = cTimeMarkUnknown;

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_56"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_57_max_end_time_with_traffic_dependent_operation_is_car2car_only)
{
    asn1::Spatem spatem;
    IntersectionState& state = add_intersection_state(spatem->spat, intersection_id, intersection_revision);
    set_region(state.id, intersection_region);
    set_timestamp(state, reference_time);
    const int bit = IntersectionStatusObject_trafficDependentOperation;
    state.status.buf[bit / 8] |= 0x80 >> (bit % 8);

    MovementState& movement = add_movement(state, 1);
    MovementEvent& event = add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time,
        std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    remove_optional(event.timing->maxEndTime);

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_57"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
    // under Combined the C-Roads rule makes maxEndTime mandatory regardless of the operation mode
    EXPECT_EQ(std::vector<std::string>{"C-Roads maxEndTime"},
        rule_list(validate_spat(spatem->spat, ValidationProfile::Combined)));

    // with maxEndTime present the requirement is met
    event.timing->maxEndTime = asn1::allocate<TimeMark_t>();
    *event.timing->maxEndTime = event.timing->minEndTime;
    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_64_likely_time_with_traffic_dependent_operation_is_car2car_and_combined)
{
    asn1::Spatem spatem;
    IntersectionState& state = add_intersection_state(spatem->spat, intersection_id, intersection_revision);
    set_region(state.id, intersection_region);
    set_timestamp(state, reference_time);
    const int bit = IntersectionStatusObject_trafficDependentOperation;
    state.status.buf[bit / 8] |= 0x80 >> (bit % 8);

    MovementState& movement = add_movement(state, 1);
    MovementEvent& event = add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time,
        std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    remove_optional(event.timing->likelyTime);

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::CRoads)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_64"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_64"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_60_unknown_max_end_time_is_profile_only_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    MovementEvent& event = add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time,
        std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    ASSERT_NE(nullptr, event.timing->maxEndTime);
    *event.timing->maxEndTime = cTimeMarkUnknown;

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_60"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_66_unknown_likely_time_is_profile_only_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    MovementEvent& event = add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time,
        std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    ASSERT_NE(nullptr, event.timing->likelyTime);
    *event.timing->likelyTime = cTimeMarkUnknown;

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_66"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Combined)));
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

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_61"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_61_missing_max_end_time_is_not_reported)
{
    // a maxEndTime of 36000 (out of range / unknown change time) must not be mistaken for a
    // differing value
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100, cTimeMarkOutOfRange);
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, cTimeMarkOutOfRange);

    // Car2Car (not Combined): these events deliberately omit maxEndTime on the completing event,
    // which would otherwise also trigger the unrelated "C-Roads maxEndTime" rule under CRoads
    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_61_equal_times_is_not_reported)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100, 100, 100, 15);
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, cTimeMarkOutOfRange);

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
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

    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_56"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
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

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_65"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
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

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_69_extra_status_bit_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_bare_state(spatem, intersection_id, intersection_revision, reference_time);
    const int fixed = IntersectionStatusObject_fixedTimeOperation;
    state.status.buf[fixed / 8] |= 0x80 >> (fixed % 8);
    const int extra = IntersectionStatusObject_manualControlIsEnabled; // bit 0, outside 5..9
    state.status.buf[extra / 8] |= 0x80 >> (extra % 8);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_69"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_70_two_mode_bits_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_bare_state(spatem, intersection_id, intersection_revision, reference_time);
    const int fixed = IntersectionStatusObject_fixedTimeOperation;
    state.status.buf[fixed / 8] |= 0x80 >> (fixed % 8);
    const int traffic = IntersectionStatusObject_trafficDependentOperation;
    state.status.buf[traffic / 8] |= 0x80 >> (traffic % 8);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_70"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_70_no_mode_bit_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_bare_state(spatem, intersection_id, intersection_revision, reference_time);
    // no status bit set at all
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_70"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_72_event_state_dark_is_car2car_only)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_dark, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(20));

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_72"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_78_events_out_of_chronological_order_reports_error_at_full_path)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 200);
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, 100); // earlier than the previous event

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    // Car2Car (not Combined): neither event carries a maxEndTime, which would otherwise also
    // trigger the unrelated "C-Roads maxEndTime" rule under CRoads
    ValidationResult car2car = validate_spat(spatem->spat, ValidationProfile::Car2Car);
    ASSERT_EQ(1u, car2car.issues().size());
    EXPECT_EQ(Severity::Error, car2car.issues()[0].severity);
    EXPECT_EQ("RS_ARSM_78", car2car.issues()[0].rule);
    EXPECT_EQ("SPAT.intersections[0].states[0].state-time-speed[1]", car2car.issues()[0].path);
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

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_79_phase_after_unavailable_with_known_end_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100);
    add_raw_event(movement, MovementPhaseState_unavailable, 200); // unavailable is not a phase

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_79"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_79_clearance_after_green_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100);
    add_raw_event(movement, MovementPhaseState_permissive_clearance, 200); // clearance is not a phase

    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_79"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_79_red_after_clearance_is_not_reported)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100);
    add_raw_event(movement, MovementPhaseState_permissive_clearance, 200);
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, 300); // reaches a phase

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
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

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_79_single_event_at_time_mark_horizon_is_not_reported)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_stop_And_Remain, cTimeMarkOutOfRange); // 36000: beyond the horizon

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_79_single_event_with_known_end_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100); // known end, no later event

    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_79"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

TEST(MapSpatValidation, rs_arsm_79_single_event_without_timing_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_event(movement, MovementPhaseState_permissive_Movement_Allowed); // no timing at all

    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_79"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
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

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_115"}, rule_list(validate_spat(spatem->spat, ValidationProfile::Combined)));
    // C-Roads: confidence "mandatory if likelyTime is provided" (RS 2077 Annex 7.2, C-Roads column)
    EXPECT_EQ(std::vector<std::string>{"C-Roads confidence"},
        rule_list(validate_spat(spatem->spat, ValidationProfile::CRoads)));
}

TEST(MapSpatValidation, rs_arsm_120_missing_timing_before_phase_reports_error_at_full_path)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_event(movement, MovementPhaseState_stop_And_Remain); // event 0, no timing
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100); // event 1, reaches a phase

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    // Car2Car (not Combined): event 1 carries no maxEndTime, which would otherwise also trigger
    // the unrelated "C-Roads maxEndTime" rule under CRoads
    ValidationResult car2car = validate_spat(spatem->spat, ValidationProfile::Car2Car);
    ASSERT_EQ(1u, car2car.issues().size());
    EXPECT_EQ(Severity::Error, car2car.issues()[0].severity);
    EXPECT_EQ("RS_ARSM_120", car2car.issues()[0].rule);
    EXPECT_EQ("SPAT.intersections[0].states[0].state-time-speed[0]", car2car.issues()[0].path);
}

TEST(MapSpatValidation, rs_arsm_120_missing_timing_on_last_event_is_not_reported)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100); // event 0, reaches a phase itself
    add_event(movement, MovementPhaseState_stop_And_Remain); // event 1, last event, no timing

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
}

// =================================================================================================
// validate_spat: C-Roads / Annex 7.2 columns
// =================================================================================================

TEST(MapSpatValidation, moy_missing_is_profile_only)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    remove_optional(state.moy);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    for (ValidationProfile profile : { ValidationProfile::Car2Car, ValidationProfile::CRoads, ValidationProfile::Combined }) {
        EXPECT_EQ(std::vector<std::string>{"moy"}, rule_list(validate_spat(spatem->spat, profile)));
    }
}

TEST(MapSpatValidation, croads_timestamp_missing_is_profile_only)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    remove_optional(state.timeStamp);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
    EXPECT_EQ(std::vector<std::string>{"C-Roads timeStamp"}, rule_list(validate_spat(spatem->spat, ValidationProfile::CRoads)));
    EXPECT_EQ(std::vector<std::string>{"C-Roads timeStamp"},
        rule_list(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, croads_max_end_time_required_is_profile_only)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    MovementEvent& event = add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time,
        std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    remove_optional(event.timing->maxEndTime);

    EXPECT_EQ(no_issues, rule_list(validate_spat(spatem->spat, ValidationProfile::Car2Car)));
    EXPECT_EQ(std::vector<std::string>{"C-Roads maxEndTime"}, rule_list(validate_spat(spatem->spat, ValidationProfile::CRoads)));
    EXPECT_EQ(std::vector<std::string>{"C-Roads maxEndTime"},
        rule_list(validate_spat(spatem->spat, ValidationProfile::Combined)));
}

// =================================================================================================
// validate_map_spat
// =================================================================================================

TEST(MapSpatValidation, rs_arsm_68_spat_state_without_map_geometry)
{
    // the OLD test for this exact scenario asserted "RS_ARSM_13", which the implementation never
    // produces here: a SPAT state without a matching MapData geometry is RS_ARSM_68 ("an
    // IntersectionGeometry exists for every IntersectionState"); RS_ARSM_13 is the converse case
    asn1::Mapem mapem; // no intersections at all
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    EXPECT_EQ(std::vector<std::string>{"map/spat intersection"},
        rule_list(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_68"},
        rule_list(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_13_map_geometry_without_spat_state)
{
    asn1::Mapem mapem;
    IntersectionGeometry& a = build_intersection(mapem, intersection_id, intersection_revision);
    GenericLane& a_in = add_vehicle_lane(a, 1, TravelDirection::Ingress, 1);
    add_node(a_in, 0.0 * meter, -10.0 * meter);
    add_node(a_in, 0.0 * meter, -5.0 * meter);
    GenericLane& a_out = add_vehicle_lane(a, 2, TravelDirection::Egress, 2);
    add_node(a_out, 0.0 * meter, 5.0 * meter);
    add_node(a_out, 0.0 * meter, 10.0 * meter);
    connect(a_in, 2, Maneuver::Straight, 1, 1);

    IntersectionGeometry& b = build_intersection(mapem, intersection_id + 1, intersection_revision);
    GenericLane& b_in = add_vehicle_lane(b, 1, TravelDirection::Ingress, 1);
    add_node(b_in, 0.0 * meter, -10.0 * meter);
    add_node(b_in, 0.0 * meter, -5.0 * meter);
    GenericLane& b_out = add_vehicle_lane(b, 2, TravelDirection::Egress, 2);
    add_node(b_out, 0.0 * meter, 5.0 * meter);
    add_node(b_out, 0.0 * meter, 10.0 * meter);
    connect(b_in, 2, Maneuver::Straight, 1, 2);

    asn1::Spatem spatem;
    IntersectionState& state_a = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement_a = add_movement(state_a, 1);
    add_known_event(movement_a, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement_a, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    // intersection B has no matching IntersectionState

    EXPECT_EQ(std::vector<std::string>{"map/spat intersection"},
        rule_list(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_13"},
        rule_list(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, same_id_different_region_are_treated_as_different_intersections)
{
    asn1::Mapem mapem;
    IntersectionGeometry& geometry = add_intersection_geometry(mapem->map, intersection_id, intersection_revision,
        59.0 * units::degree, 30.0 * units::degree, 3.5 * meter);
    set_region(geometry.id, intersection_region);
    GenericLane& in = add_vehicle_lane(geometry, 1, TravelDirection::Ingress, 1);
    add_node(in, 0.0 * meter, -10.0 * meter);
    add_node(in, 0.0 * meter, -5.0 * meter);
    GenericLane& out = add_vehicle_lane(geometry, 2, TravelDirection::Egress, 2);
    add_node(out, 0.0 * meter, 5.0 * meter);
    add_node(out, 0.0 * meter, 10.0 * meter);
    connect(in, 2, Maneuver::Straight, 1, 1);

    asn1::Spatem spatem;
    IntersectionState& state = add_intersection_state(spatem->spat, intersection_id, intersection_revision);
    set_region(state.id, intersection_region + 1); // same id, different region
    set_timestamp(state, reference_time);
    const int bit = IntersectionStatusObject_fixedTimeOperation;
    state.status.buf[bit / 8] |= 0x80 >> (bit % 8);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    std::vector<std::string> standard_expected { "map/spat intersection", "map/spat intersection" };
    EXPECT_EQ(standard_expected, rule_list(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard)));

    std::vector<std::string> combined_expected { "RS_ARSM_13", "RS_ARSM_68" };
    EXPECT_EQ(combined_expected, rule_list(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, revision_mismatch_always_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem); // revision == intersection_revision
    GenericLane& in = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(in, 0.0 * meter, -10.0 * meter);
    add_node(in, 0.0 * meter, -5.0 * meter);
    GenericLane& out = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
    add_node(out, 0.0 * meter, 5.0 * meter);
    add_node(out, 0.0 * meter, 10.0 * meter);
    connect(in, 2, Maneuver::Straight, 1, 1); // signal group 1, matches the SPAT movement below

    asn1::Spatem spatem;
    IntersectionState& state = add_intersection_state(spatem->spat, intersection_id, intersection_revision + 1);
    set_region(state.id, intersection_region);
    set_timestamp(state, reference_time);
    const int bit = IntersectionStatusObject_fixedTimeOperation;
    state.status.buf[bit / 8] |= 0x80 >> (bit % 8);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    EXPECT_EQ(std::vector<std::string>{"revision"},
        rule_list(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"revision"},
        rule_list(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_49_unmatched_connection_signal_group_severity_depends_on_profile)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(ingress, 0.0 * meter, -10.0 * meter);
    add_node(ingress, 0.0 * meter, -5.0 * meter);
    GenericLane& egress = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
    add_node(egress, 0.0 * meter, 5.0 * meter);
    add_node(egress, 0.0 * meter, 10.0 * meter);
    connect(ingress, 2, Maneuver::Straight, 5, 1); // signal group 5, no matching movement

    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 9); // unrelated signal group, itself unmatched too
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    // both directions of the pairing are unmatched here, so RS_ARSM_49 and RS_ARSM_75 (or their
    // Standard-profile equivalent) are both reported; see the dedicated RS_ARSM_75 test below for
    // a construction where only one direction is unmatched
    std::vector<std::string> standard_expected { "map/spat signal group", "map/spat signal group" };
    EXPECT_EQ(standard_expected, rule_list(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard)));

    std::vector<std::string> combined_expected { "RS_ARSM_49", "RS_ARSM_75" };
    EXPECT_EQ(combined_expected, rule_list(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, rs_arsm_75_unmatched_movement_signal_group_severity_depends_on_profile)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(ingress, 0.0 * meter, -10.0 * meter);
    add_node(ingress, 0.0 * meter, -5.0 * meter);
    GenericLane& egress = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
    add_node(egress, 0.0 * meter, 5.0 * meter);
    add_node(egress, 0.0 * meter, 10.0 * meter);
    connect(ingress, 2, Maneuver::Straight, 1, 1);

    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& matched = add_movement(state, 1);
    add_known_event(matched, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(matched, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    MovementState& unmatched = add_movement(state, 7); // no connection uses this signal group
    add_known_event(unmatched, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(unmatched, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    EXPECT_EQ(std::vector<std::string>{"map/spat signal group"},
        rule_list(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(std::vector<std::string>{"RS_ARSM_75"},
        rule_list(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined)));
}

TEST(MapSpatValidation, fragment_does_not_report_unmatched_spat_signal_group)
{
    // RS_ARSM_75 ("every MovementState signal group is used by a connection") is not checked for
    // a MapData fragment, since a signal group may be described by another fragment's geometry
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(ingress, 0.0 * meter, -10.0 * meter);
    add_node(ingress, 0.0 * meter, -5.0 * meter);
    GenericLane& egress = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
    add_node(egress, 0.0 * meter, 5.0 * meter);
    add_node(egress, 0.0 * meter, 10.0 * meter);
    connect(ingress, 2, Maneuver::Straight, 1, 1);
    set_layer_id(mapem->map, 1);

    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& matched = add_movement(state, 1);
    add_known_event(matched, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(matched, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    MovementState& other_fragment = add_movement(state, 99); // described by another fragment
    add_known_event(other_fragment, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(other_fragment, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    EXPECT_EQ(no_issues, rule_list(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard)));
    EXPECT_EQ(no_issues, rule_list(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined)));
}

// =================================================================================================
// Standard profile must never leak a Car2Car (RS_ARSM_*) or C-Roads identifier
// =================================================================================================

TEST(MapSpatValidation, standard_profile_never_reports_profile_specific_identifiers)
{
    std::vector<ValidationResult (*)()> scenarios {
        // validate_map, RS_ARSM_*
        [] {
            asn1::Mapem mapem;
            add_intersection_geometry(mapem->map, intersection_id, intersection_revision,
                59.0 * units::degree, 30.0 * units::degree, 3.5 * meter); // no region: RS_ARSM_11
            return validate_map(mapem->map, ValidationProfile::Standard);
        },
        [] {
            asn1::Mapem mapem;
            build_intersection(mapem, 9, 1);
            build_intersection(mapem, 9, 1); // RS_ARSM_12
            return validate_map(mapem->map, ValidationProfile::Standard);
        },
        [] {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_intersection(mapem);
            remove_optional(intersection.laneWidth); // RS_ARSM_14
            return validate_map(mapem->map, ValidationProfile::Standard);
        },
        [] {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
            remove_optional(intersection.laneSet.list.array[0]->ingressApproach); // RS_ARSM_16
            return validate_map(mapem->map, ValidationProfile::Standard);
        },
        [] {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_intersection(mapem);
            GenericLane& crosswalk = add_crosswalk_lane(intersection, 1, 1, 2);
            add_node(crosswalk, -5.0 * meter, 0.0 * meter);
            add_node(crosswalk, 5.0 * meter, 0.0 * meter);
            remove_optional(crosswalk.egressApproach); // RS_ARSM_17
            return validate_map(mapem->map, ValidationProfile::Standard);
        },
        [] {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_intersection(mapem);
            GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
            add_node(ingress, 0.0 * meter, -10.0 * meter);
            add_node(ingress, 0.0 * meter, -5.0 * meter);
            GenericLane& egress = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
            add_node(egress, 0.0 * meter, 5.0 * meter);
            add_node(egress, 0.0 * meter, 10.0 * meter);
            connect(ingress, 2, Maneuver::Straight, 1, 1);
            connect(ingress, 2, Maneuver::Straight, 1, 2); // RS_ARSM_20
            return validate_map(mapem->map, ValidationProfile::Standard);
        },
        [] {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
            GenericLane& ingress = *intersection.laneSet.list.array[0];
            connect(ingress, 1, Maneuver::Straight, 1, 1);
            Connection_t& connection = *ingress.connectsTo->list.array[0];
            remove_owned(connection.connectingLane.maneuver, asn_DEF_AllowedManeuvers); // RS_ARSM_21
            return validate_map(mapem->map, ValidationProfile::Standard);
        },
        [] {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
            GenericLane& ingress = *intersection.laneSet.list.array[0];
            connect(ingress, 1, Maneuver::Straight, 1, 1);
            ingress.connectsTo->list.array[0]->connectingLane.maneuver->buf[0] = 0; // RS_ARSM_22
            return validate_map(mapem->map, ValidationProfile::Standard);
        },
        [] {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
            GenericLane& ingress = *intersection.laneSet.list.array[0];
            connect(ingress, 1, Maneuver::Straight, 1, 1);
            Connection_t& connection = *ingress.connectsTo->list.array[0];
            connection.connectingLane.maneuver->buf[0] |= 0x80 >> AllowedManeuvers_maneuverLaneChangeAllowed; // RS24
            return validate_map(mapem->map, ValidationProfile::Standard);
        },
        [] {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_intersection(mapem);
            GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
            add_node(lane, 0.0 * meter, -20.0 * meter);
            for (int i = 0; i < 18; ++i) {
                add_node(lane, 0.0 * meter, 1.0 * meter); // RS_ARSM_35
            }
            return validate_map(mapem->map, ValidationProfile::Standard);
        },
        [] {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_intersection(mapem);
            GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
            add_node(ingress, 0.0 * meter, -10.0 * meter);
            add_node(ingress, 0.0 * meter, -5.0 * meter);
            GenericLane& egress = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
            add_node(egress, 0.0 * meter, 5.0 * meter);
            add_node(egress, 0.0 * meter, 10.0 * meter);
            connect(ingress, 2, Maneuver::Straight, 1, 1);
            remove_optional(ingress.connectsTo->list.array[0]->signalGroup); // RS_ARSM_48
            return validate_map(mapem->map, ValidationProfile::Standard);
        },
        [] {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_intersection(mapem);
            GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
            add_node(lane, 0.0 * meter, -10.0 * meter);
            add_node(lane, 0.0 * meter, -5.0 * meter);
            lane.maneuvers = asn1::allocate<AllowedManeuvers_t>(); // RS_ARSM_117
            lane.maneuvers->buf = static_cast<uint8_t*>(asn1::allocate(1));
            lane.maneuvers->size = 1;
            lane.maneuvers->buf[0] = 0x80 >> AllowedManeuvers_maneuverStraightAllowed;
            return validate_map(mapem->map, ValidationProfile::Standard);
        },
        [] {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_intersection(mapem);
            GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
            lane.nodeList.present = NodeListXY_PR_computed; // RS_ARSM_118
            ComputedLane_t& computed = lane.nodeList.choice.computed;
            computed.referenceLaneId = 1;
            computed.offsetXaxis.present = ComputedLane__offsetXaxis_PR_small;
            computed.offsetYaxis.present = ComputedLane__offsetYaxis_PR_small;
            return validate_map(mapem->map, ValidationProfile::Standard);
        },
        [] {
            asn1::Mapem mapem; // RS_ARSM_11-tolerant, but exercises "C-Roads intersections"
            return validate_map(mapem->map, ValidationProfile::Standard);
        },
        [] {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_single_lane_intersection(mapem);
            GenericLane& ingress = *intersection.laneSet.list.array[0];
            connect(ingress, 1, Maneuver::Straight, 1, 1);
            remove_optional(ingress.connectsTo->list.array[0]->connectionID); // "C-Roads connectionID"
            return validate_map(mapem->map, ValidationProfile::Standard);
        },
        // validate_spat, RS_ARSM_*
        [] {
            asn1::Spatem spatem;
            IntersectionState& state = add_intersection_state(spatem->spat, intersection_id, intersection_revision);
            set_timestamp(state, reference_time); // no region: RS_ARSM_11
            const int bit = IntersectionStatusObject_fixedTimeOperation;
            state.status.buf[bit / 8] |= 0x80 >> (bit % 8);
            MovementState& movement = add_movement(state, 1);
            add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time,
                std::chrono::seconds(10));
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Spatem spatem;
            for (int i = 0; i < 2; ++i) {
                IntersectionState& state = build_intersection_state(spatem, 9, 1, reference_time); // RS_ARSM_12
                MovementState& movement = add_movement(state, 1);
                add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time,
                    std::chrono::seconds(10));
                add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            }
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            MovementEvent& event = add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed,
                reference_time, std::chrono::seconds(10));
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            event.timing->minEndTime = cTimeMarkUnknown; // RS_ARSM_56
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Spatem spatem;
            IntersectionState& state = add_intersection_state(spatem->spat, intersection_id, intersection_revision);
            set_region(state.id, intersection_region);
            set_timestamp(state, reference_time);
            const int bit = IntersectionStatusObject_trafficDependentOperation;
            state.status.buf[bit / 8] |= 0x80 >> (bit % 8);
            MovementState& movement = add_movement(state, 1);
            MovementEvent& event = add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed,
                reference_time, std::chrono::seconds(10));
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            remove_optional(event.timing->maxEndTime); // RS_ARSM_57 / "C-Roads maxEndTime"
            remove_optional(event.timing->likelyTime); // RS_ARSM_64
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            MovementEvent& event = add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed,
                reference_time, std::chrono::seconds(10));
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            *event.timing->maxEndTime = cTimeMarkUnknown; // RS_ARSM_60
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            MovementEvent& event = add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed,
                reference_time, std::chrono::seconds(10));
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            *event.timing->likelyTime = cTimeMarkUnknown; // RS_ARSM_66
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100, 200, 100, 15); // RS_ARSM_61
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 200, 300, 100, 15); // RS_ARSM_65
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Spatem spatem;
            IntersectionState& state = build_bare_state(spatem, intersection_id, intersection_revision, reference_time);
            const int fixed = IntersectionStatusObject_fixedTimeOperation;
            state.status.buf[fixed / 8] |= 0x80 >> (fixed % 8);
            const int extra = IntersectionStatusObject_manualControlIsEnabled; // RS_ARSM_69
            state.status.buf[extra / 8] |= 0x80 >> (extra % 8);
            MovementState& movement = add_movement(state, 1);
            add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time,
                std::chrono::seconds(10));
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Spatem spatem;
            IntersectionState& state = build_bare_state(spatem, intersection_id, intersection_revision, reference_time);
            // no status bit set: RS_ARSM_70
            MovementState& movement = add_movement(state, 1);
            add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time,
                std::chrono::seconds(10));
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            add_known_event(movement, MovementPhaseState_dark, reference_time, std::chrono::seconds(10)); // RS_ARSM_72
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(20));
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 200); // RS_ARSM_78
            add_raw_event(movement, MovementPhaseState_stop_And_Remain, 100);
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100); // RS_ARSM_79
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100, boost::none, 100); // RS115
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            add_event(movement, MovementPhaseState_stop_And_Remain); // RS_ARSM_120
            add_raw_event(movement, MovementPhaseState_permissive_Movement_Allowed, 100);
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            remove_optional(state.timeStamp); // "C-Roads timeStamp"
            MovementState& movement = add_movement(state, 1);
            add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time,
                std::chrono::seconds(10));
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            return validate_spat(spatem->spat, ValidationProfile::Standard);
        },
        // validate_map_spat
        [] {
            asn1::Mapem mapem; // RS_ARSM_13 / RS_ARSM_68 (no intersections at all)
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 1);
            add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time,
                std::chrono::seconds(10));
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            return validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard);
        },
        [] {
            asn1::Mapem mapem;
            IntersectionGeometry& intersection = build_intersection(mapem);
            GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
            add_node(ingress, 0.0 * meter, -10.0 * meter);
            add_node(ingress, 0.0 * meter, -5.0 * meter);
            GenericLane& egress = add_vehicle_lane(intersection, 2, TravelDirection::Egress, 2);
            add_node(egress, 0.0 * meter, 5.0 * meter);
            add_node(egress, 0.0 * meter, 10.0 * meter);
            connect(ingress, 2, Maneuver::Straight, 5, 1); // RS_ARSM_49 / RS_ARSM_75
            asn1::Spatem spatem;
            IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision,
                reference_time);
            MovementState& movement = add_movement(state, 9);
            add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time,
                std::chrono::seconds(10));
            add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
            return validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard);
        },
    };

    for (std::size_t i = 0; i < scenarios.size(); ++i) {
        ValidationResult result = scenarios[i]();
        EXPECT_FALSE(has_profile_specific_identifier(result)) << "scenario " << i << " leaked a profile identifier";
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

    ControllerConfig config;
    const auto group = [&](long id, SignalGroupKind kind) {
        config.groups.push_back({ id, kind, seconds(2), seconds(3), seconds(3), permissive, clearance });
    };
    group(1, SignalGroupKind::Traffic);
    group(2, SignalGroupKind::Traffic);
    group(3, SignalGroupKind::Pedestrian);
    group(4, SignalGroupKind::Traffic);
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

            const ValidationResult result = validate_spat(spatem->spat, ValidationProfile::Car2Car);
            std::ostringstream issues;
            for (const ValidationIssue& issue : result.issues()) {
                issues << issue << "\n";
            }
            EXPECT_TRUE(result.issues().empty()) << "at step " << step << "\n" << issues.str();
        }
    }
}
