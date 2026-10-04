#include <gtest/gtest.h>
#include <vanetza/asn1/mapem.hpp>
#include <vanetza/asn1/spatem.hpp>
#include <vanetza/asn1/its/AllowedManeuvers.h>
#include <vanetza/asn1/its/Connection.h>
#include <vanetza/asn1/its/ConnectsToList.h>
#include <vanetza/asn1/its/GenericLane.h>
#include <vanetza/asn1/its/IntersectionGeometry.h>
#include <vanetza/asn1/its/IntersectionReferenceID.h>
#include <vanetza/asn1/its/IntersectionStatusObject.h>
#include <vanetza/asn1/its/MovementEvent.h>
#include <vanetza/asn1/its/MovementPhaseState.h>
#include <vanetza/asn1/its/MovementState.h>
#include <vanetza/asn1/its/RoadRegulatorID.h>
#include <vanetza/asn1/its/TimeChangeDetails.h>
#include <vanetza/asn1/its/TimeMark.h>
#include <vanetza/facilities/map_functions.hpp>
#include <vanetza/facilities/map_spat_validation.hpp>
#include <vanetza/facilities/spat_functions.hpp>
#include <vanetza/facilities/validation.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <array>
#include <chrono>
#include <cstdint>
#include <string>

using namespace vanetza;
using namespace vanetza::facilities;

namespace
{

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

void set_region(IntersectionReferenceID_t& id, long region)
{
    id.region = asn1::allocate<RoadRegulatorID_t>();
    *id.region = region;
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

// intersection state with region set and fixedTimeOperation flagged, no movements
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

struct Baseline
{
    asn1::Mapem mapem;
    asn1::Spatem spatem;
};

// one intersection, ingress lane 1 -> egress lane 2 via signal group 1, crosswalk lane 3,
// one SPAT intersection state with signal group 1 having two fully-timed events
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

// --- baseline and ValidationResult sanity -----------------------------------------------------

TEST(MapSpatValidation, baseline_map_has_no_issues_in_any_profile)
{
    Baseline baseline = build_baseline(reference_time);
    for (ValidationProfile profile : all_profiles) {
        ValidationResult result = validate_map(baseline.mapem->map, profile);
        EXPECT_EQ(0u, result.issues().size());
    }
}

TEST(MapSpatValidation, baseline_spat_has_no_issues_in_any_profile)
{
    Baseline baseline = build_baseline(reference_time);
    for (ValidationProfile profile : all_profiles) {
        ValidationResult result = validate_spat(baseline.spatem->spat, profile);
        EXPECT_EQ(0u, result.issues().size());
    }
}

TEST(MapSpatValidation, baseline_map_spat_has_no_issues_in_any_profile)
{
    Baseline baseline = build_baseline(reference_time);
    for (ValidationProfile profile : all_profiles) {
        ValidationResult result = validate_map_spat(baseline.mapem->map, baseline.spatem->spat, profile);
        EXPECT_EQ(0u, result.issues().size());
    }
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

// --- validate_map -------------------------------------------------------------------------------

TEST(MapSpatValidation, asn1_constraint_violation_is_reported_as_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -10.0 * meter);
    add_node(lane, 0.0 * meter, -5.0 * meter);
    lane.laneID = 256; // LaneID ::= INTEGER (0..255)

    ValidationResult result = validate_map(mapem->map);
    EXPECT_TRUE(has_issue(result, "ASN.1", Severity::Error));
}

TEST(MapSpatValidation, duplicate_lane_id_reports_error)
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
    EXPECT_TRUE(has_issue(result, "LaneID unique", Severity::Error));
}

TEST(MapSpatValidation, lane_id_zero_reports_warning)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 0, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -10.0 * meter);
    add_node(lane, 0.0 * meter, -5.0 * meter);

    ValidationResult result = validate_map(mapem->map);
    EXPECT_TRUE(has_issue(result, "LaneID value", Severity::Warning));
    EXPECT_EQ(0u, result.count(Severity::Error));
}

TEST(MapSpatValidation, lane_id_reserved_255_reports_warning)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 255, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -10.0 * meter);
    add_node(lane, 0.0 * meter, -5.0 * meter);

    ValidationResult result = validate_map(mapem->map);
    EXPECT_TRUE(has_issue(result, "LaneID value", Severity::Warning));
    EXPECT_EQ(0u, result.count(Severity::Error));
}

TEST(MapSpatValidation, dangling_connecting_lane_reports_error_with_path)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& ingress = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(ingress, 0.0 * meter, -10.0 * meter);
    add_node(ingress, 0.0 * meter, -5.0 * meter);
    connect(ingress, 99, Maneuver::Straight, 1, 1); // lane 99 does not exist

    ValidationResult result = validate_map(mapem->map);
    EXPECT_TRUE(has_issue(result, "connectingLane", Severity::Error, "laneSet[0].connectsTo[0]"));
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
    EXPECT_TRUE(has_issue(result, "connectionID", Severity::Warning));
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
    connect(ingress, 3, Maneuver::Right, 1, 1); // same connectionID 1, same signal group

    ValidationResult result = validate_map(mapem->map);
    EXPECT_FALSE(has_issue(result, "connectionID", Severity::Warning));
}

TEST(MapSpatValidation, crosswalk_self_loop_is_warning_not_error)
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

TEST(MapSpatValidation, rs_arsm_11_missing_region_is_profile_only)
{
    asn1::Mapem mapem;
    add_intersection_geometry(mapem->map, intersection_id, intersection_revision,
        59.0 * units::degree, 30.0 * units::degree, 3.5 * meter); // region left unset

    EXPECT_FALSE(has_issue(validate_map(mapem->map, ValidationProfile::Standard), "RS_ARSM_11", Severity::Error));
    EXPECT_TRUE(has_issue(validate_map(mapem->map, ValidationProfile::Combined), "RS_ARSM_11", Severity::Error));
}

TEST(MapSpatValidation, rs_arsm_12_duplicate_reference_id_reports_error)
{
    asn1::Mapem mapem;
    build_intersection(mapem, 9, 1);
    build_intersection(mapem, 9, 1); // same id and region

    EXPECT_FALSE(has_issue(validate_map(mapem->map, ValidationProfile::Standard), "RS_ARSM_12", Severity::Error));
    EXPECT_TRUE(has_issue(validate_map(mapem->map, ValidationProfile::Car2Car), "RS_ARSM_12", Severity::Error));
}

TEST(MapSpatValidation, rs_arsm_14_missing_lane_width_is_profile_only)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    intersection.laneWidth = nullptr;

    EXPECT_FALSE(has_issue(validate_map(mapem->map, ValidationProfile::Standard), "RS_ARSM_14", Severity::Error));
    EXPECT_TRUE(has_issue(validate_map(mapem->map, ValidationProfile::Combined), "RS_ARSM_14", Severity::Error));
}

TEST(MapSpatValidation, rs_arsm_17_crosswalk_missing_approach_reports_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& crosswalk = add_crosswalk_lane(intersection, 1, 1, 2);
    add_node(crosswalk, -5.0 * meter, 0.0 * meter);
    add_node(crosswalk, 5.0 * meter, 0.0 * meter);
    crosswalk.egressApproach = nullptr;

    EXPECT_FALSE(has_issue(validate_map(mapem->map, ValidationProfile::Standard), "RS_ARSM_17", Severity::Error));
    EXPECT_TRUE(has_issue(validate_map(mapem->map, ValidationProfile::Combined), "RS_ARSM_17", Severity::Error));
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

    EXPECT_FALSE(has_issue(validate_map(mapem->map, ValidationProfile::Standard), "RS_ARSM_22", Severity::Error));
    EXPECT_TRUE(has_issue(validate_map(mapem->map, ValidationProfile::Combined), "RS_ARSM_22", Severity::Error));
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

    EXPECT_FALSE(has_issue(validate_map(mapem->map, ValidationProfile::Standard), "RS_ARSM_22", Severity::Error));
    EXPECT_TRUE(has_issue(validate_map(mapem->map, ValidationProfile::Combined), "RS_ARSM_22", Severity::Error));
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
    ingress.connectsTo->list.array[0]->signalGroup = nullptr;

    EXPECT_FALSE(has_issue(validate_map(mapem->map, ValidationProfile::Standard), "RS_ARSM_48", Severity::Warning));
    EXPECT_TRUE(has_issue(validate_map(mapem->map, ValidationProfile::Combined), "RS_ARSM_48", Severity::Warning));
}

TEST(MapSpatValidation, rs_arsm_117_lane_level_maneuvers_is_profile_only_error)
{
    asn1::Mapem mapem;
    IntersectionGeometry& intersection = build_intersection(mapem);
    GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, 1);
    add_node(lane, 0.0 * meter, -10.0 * meter);
    add_node(lane, 0.0 * meter, -5.0 * meter);
    lane.maneuvers = asn1::allocate<AllowedManeuvers_t>();
    lane.maneuvers->buf = static_cast<uint8_t*>(asn1::allocate(1));
    lane.maneuvers->size = 1;
    lane.maneuvers->buf[0] = 0x80 >> AllowedManeuvers_maneuverStraightAllowed;
    lane.maneuvers->bits_unused = 0;

    EXPECT_FALSE(has_issue(validate_map(mapem->map, ValidationProfile::Standard), "RS_ARSM_117", Severity::Error));
    EXPECT_TRUE(has_issue(validate_map(mapem->map, ValidationProfile::Combined), "RS_ARSM_117", Severity::Error));
}

// --- validate_spat -------------------------------------------------------------------------------

TEST(MapSpatValidation, spat_asn1_constraint_violation_is_reported_as_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    movement.signalGroup = 256; // SignalGroupID ::= INTEGER (0..255)

    ValidationResult result = validate_spat(spatem->spat);
    EXPECT_TRUE(has_issue(result, "ASN.1", Severity::Error));
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
    EXPECT_TRUE(has_issue(result, "SignalGroupID unique", Severity::Warning));
}

TEST(MapSpatValidation, signal_group_zero_reports_warning)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 0);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    ValidationResult result = validate_spat(spatem->spat);
    EXPECT_TRUE(has_issue(result, "SignalGroupID value", Severity::Warning));
}

TEST(MapSpatValidation, consecutive_equal_event_states_report_warning)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(20));

    ValidationResult result = validate_spat(spatem->spat);
    EXPECT_TRUE(has_issue(result, "eventState sequence", Severity::Warning));
}

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

    EXPECT_FALSE(has_issue(validate_spat(spatem->spat, ValidationProfile::Standard), "RS_ARSM_11", Severity::Error));
    EXPECT_TRUE(has_issue(validate_spat(spatem->spat, ValidationProfile::Combined), "RS_ARSM_11", Severity::Error));
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

    EXPECT_FALSE(has_issue(validate_spat(spatem->spat, ValidationProfile::Standard), "RS_ARSM_12", Severity::Error));
    EXPECT_TRUE(has_issue(validate_spat(spatem->spat, ValidationProfile::Car2Car), "RS_ARSM_12", Severity::Error));
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

    EXPECT_FALSE(has_issue(validate_spat(spatem->spat, ValidationProfile::Standard), "RS_ARSM_56", Severity::Error));
    EXPECT_TRUE(has_issue(validate_spat(spatem->spat, ValidationProfile::Combined), "RS_ARSM_56", Severity::Error));
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
    ASN_STRUCT_FREE(asn_DEF_TimeMark, event.timing->maxEndTime);
    event.timing->maxEndTime = nullptr;

    EXPECT_TRUE(has_issue(validate_spat(spatem->spat, ValidationProfile::Car2Car), "RS_ARSM_57", Severity::Error));
    // under Combined the C-Roads rule makes maxEndTime mandatory regardless of the operation mode
    EXPECT_FALSE(has_issue(validate_spat(spatem->spat, ValidationProfile::Combined), "RS_ARSM_57", Severity::Error));
    EXPECT_TRUE(has_issue(validate_spat(spatem->spat, ValidationProfile::Combined), "maxEndTime", Severity::Error));
    EXPECT_FALSE(has_issue(validate_spat(spatem->spat, ValidationProfile::Standard), "RS_ARSM_57", Severity::Error));

    // with maxEndTime present the requirement is met
    event.timing->maxEndTime = asn1::allocate<TimeMark_t>();
    *event.timing->maxEndTime = event.timing->minEndTime;
    EXPECT_FALSE(has_issue(validate_spat(spatem->spat, ValidationProfile::Car2Car), "RS_ARSM_57", Severity::Error));
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

    EXPECT_FALSE(has_issue(validate_spat(spatem->spat, ValidationProfile::Standard), "RS_ARSM_60", Severity::Error));
    EXPECT_TRUE(has_issue(validate_spat(spatem->spat, ValidationProfile::Combined), "RS_ARSM_60", Severity::Error));
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

    EXPECT_FALSE(has_issue(validate_spat(spatem->spat, ValidationProfile::Standard), "RS_ARSM_66", Severity::Error));
    EXPECT_TRUE(has_issue(validate_spat(spatem->spat, ValidationProfile::Combined), "RS_ARSM_66", Severity::Error));
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

    EXPECT_FALSE(has_issue(validate_spat(spatem->spat, ValidationProfile::Standard), "RS_ARSM_61", Severity::Error));
    EXPECT_TRUE(has_issue(validate_spat(spatem->spat, ValidationProfile::Combined), "RS_ARSM_61", Severity::Error));
}

TEST(MapSpatValidation, rs_arsm_65_detects_times_out_of_order)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    MovementEvent& event = add_event(movement, MovementPhaseState_permissive_Movement_Allowed);
    MovementTiming timing;
    timing.min_end = reference_time + std::chrono::seconds(20);
    timing.max_end = reference_time + std::chrono::seconds(10); // before min_end
    timing.likely = timing.min_end;
    timing.confidence = 15;
    set_timing(event, timing, reference_time);
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    EXPECT_FALSE(has_issue(validate_spat(spatem->spat, ValidationProfile::Standard), "RS_ARSM_65", Severity::Error));
    EXPECT_TRUE(has_issue(validate_spat(spatem->spat, ValidationProfile::Combined), "RS_ARSM_65", Severity::Error));
}

TEST(MapSpatValidation, rs_arsm_65_hour_wrap_is_not_falsely_reported)
{
    // moy refers to minute 59; minEndTime stays in the current hour, maxEndTime (numerically
    // smaller) resolves into the following hour and is chronologically later (RS_ARSM_54)
    const auto now = Clock::at("2026-10-04 11:59:50.000");
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, now);
    MovementState& movement = add_movement(state, 1);
    MovementEvent& event = add_event(movement, MovementPhaseState_permissive_Movement_Allowed);
    MovementTiming timing;
    timing.min_end = Clock::at("2026-10-04 11:59:55.000");
    timing.max_end = Clock::at("2026-10-04 12:00:05.000"); // next hour, smaller TimeMark value
    timing.likely = timing.min_end;
    timing.confidence = 15;
    set_timing(event, timing, now);
    add_known_event(movement, MovementPhaseState_stop_And_Remain, now, std::chrono::seconds(30));

    ValidationResult result = validate_spat(spatem->spat, ValidationProfile::Combined);
    EXPECT_FALSE(has_issue(result, "RS_ARSM_65", Severity::Error));
}

TEST(MapSpatValidation, rs_arsm_79_missing_next_phase_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    // no further event although the end of the listed one is known

    EXPECT_FALSE(has_issue(validate_spat(spatem->spat, ValidationProfile::Standard), "RS_ARSM_79", Severity::Error));
    EXPECT_TRUE(has_issue(validate_spat(spatem->spat, ValidationProfile::Combined), "RS_ARSM_79", Severity::Error));
}

TEST(MapSpatValidation, rs_arsm_115_likely_time_without_confidence_reports_error)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    MovementEvent& event = add_event(movement, MovementPhaseState_permissive_Movement_Allowed);
    event.timing = asn1::allocate<TimeChangeDetails_t>();
    event.timing->minEndTime = 100;
    event.timing->likelyTime = asn1::allocate<TimeMark_t>();
    *event.timing->likelyTime = 100;
    // confidence intentionally left absent
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    EXPECT_FALSE(has_issue(validate_spat(spatem->spat, ValidationProfile::Standard), "RS_ARSM_115", Severity::Error));
    EXPECT_TRUE(has_issue(validate_spat(spatem->spat, ValidationProfile::Combined), "RS_ARSM_115", Severity::Error));
}

TEST(MapSpatValidation, croads_max_end_time_required_is_profile_only)
{
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    MovementEvent& event = add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time,
        std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));
    event.timing->maxEndTime = nullptr;

    EXPECT_FALSE(has_issue(validate_spat(spatem->spat, ValidationProfile::Car2Car), "maxEndTime", Severity::Error));
    EXPECT_TRUE(has_issue(validate_spat(spatem->spat, ValidationProfile::CRoads), "maxEndTime", Severity::Error));
    EXPECT_TRUE(has_issue(validate_spat(spatem->spat, ValidationProfile::Combined), "maxEndTime", Severity::Error));
}

// --- validate_map_spat ---------------------------------------------------------------------------

TEST(MapSpatValidation, rs_arsm_13_missing_geometry_severity_depends_on_profile)
{
    asn1::Mapem mapem; // no intersections at all
    asn1::Spatem spatem;
    IntersectionState& state = build_intersection_state(spatem, intersection_id, intersection_revision, reference_time);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    ValidationResult standard = validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard);
    EXPECT_TRUE(has_issue(standard, "RS_ARSM_13", Severity::Warning));
    EXPECT_FALSE(has_issue(standard, "RS_ARSM_13", Severity::Error));

    ValidationResult combined = validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined);
    EXPECT_TRUE(has_issue(combined, "RS_ARSM_13", Severity::Error));
}

TEST(MapSpatValidation, revision_mismatch_always_reports_error)
{
    asn1::Mapem mapem;
    build_intersection(mapem); // revision == intersection_revision

    asn1::Spatem spatem;
    IntersectionState& state = add_intersection_state(spatem->spat, intersection_id, intersection_revision + 1);
    set_region(state.id, intersection_region);
    set_timestamp(state, reference_time);
    const int bit = IntersectionStatusObject_fixedTimeOperation;
    state.status.buf[bit / 8] |= 0x80 >> (bit % 8);
    MovementState& movement = add_movement(state, 1);
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    EXPECT_TRUE(has_issue(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard),
        "revision", Severity::Error));
    EXPECT_TRUE(has_issue(validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined),
        "revision", Severity::Error));
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
    MovementState& movement = add_movement(state, 9); // unrelated signal group
    add_known_event(movement, MovementPhaseState_permissive_Movement_Allowed, reference_time, std::chrono::seconds(10));
    add_known_event(movement, MovementPhaseState_stop_And_Remain, reference_time, std::chrono::seconds(30));

    ValidationResult standard = validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard);
    EXPECT_TRUE(has_issue(standard, "RS_ARSM_49", Severity::Warning));
    EXPECT_FALSE(has_issue(standard, "RS_ARSM_49", Severity::Error));

    ValidationResult combined = validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined);
    EXPECT_TRUE(has_issue(combined, "RS_ARSM_49", Severity::Error));
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

    ValidationResult standard = validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Standard);
    EXPECT_TRUE(has_issue(standard, "RS_ARSM_75", Severity::Warning));
    EXPECT_FALSE(has_issue(standard, "RS_ARSM_75", Severity::Error));

    ValidationResult combined = validate_map_spat(mapem->map, spatem->spat, ValidationProfile::Combined);
    EXPECT_TRUE(has_issue(combined, "RS_ARSM_75", Severity::Error));
}
