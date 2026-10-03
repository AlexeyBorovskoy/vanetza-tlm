#include <gtest/gtest.h>
#include <vanetza/asn1/spatem.hpp>
#include <vanetza/facilities/fixed_time_plan.hpp>
#include <vanetza/facilities/spat_functions.hpp>
#include <boost/units/systems/si/length.hpp>
#include <stdexcept>

using namespace vanetza;
using namespace vanetza::facilities;
using std::chrono::seconds;
using std::chrono::hours;

namespace
{

const long permissive = MovementPhaseState_permissive_Movement_Allowed;
const long clearance = MovementPhaseState_permissive_clearance;
const long red = MovementPhaseState_stop_And_Remain;
const long red_amber = MovementPhaseState_pre_Movement;

SignalGroupConfig make_group(long id, SignalGroupKind kind)
{
    return { id, kind, seconds(2), seconds(3), seconds(3), permissive, clearance };
}

// two stages: main road with trams and side crossings, side road with main road crossing
ControllerConfig two_stage_config()
{
    ControllerConfig config;
    for (long id : { 1, 2, 3, 4 }) {
        config.groups.push_back(make_group(id, SignalGroupKind::Traffic));
    }
    for (long id : { 5, 6, 7 }) {
        config.groups.push_back(make_group(id, SignalGroupKind::Pedestrian));
    }
    for (long id : { 8, 9 }) {
        config.groups.push_back(make_group(id, SignalGroupKind::Tram));
    }
    config.stages = { { 1, { 1, 2, 6, 7, 8, 9 } }, { 2, { 3, 4, 5 } } };
    for (long main : { 1, 2, 6, 7, 8, 9 }) {
        for (long side : { 3, 4, 5 }) {
            config.intergreen[{ main, side }] = seconds(3);
            config.intergreen[{ side, main }] = seconds(6);
        }
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
    return config;
}

void expect_cycle(const std::vector<SignalInterval>& actual, const std::vector<SignalInterval>& expected)
{
    ASSERT_EQ(expected.size(), actual.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_EQ(expected[i].phase_state, actual[i].phase_state) << "interval " << i;
        EXPECT_EQ(expected[i].begin, actual[i].begin) << "interval " << i;
        EXPECT_EQ(expected[i].end, actual[i].end) << "interval " << i;
    }
}

} // namespace

TEST(FixedTimePlan, cycle_of_two_stage_plan)
{
    const FixedTimePlan controller(two_stage_config());
    const PlanConfig& plan = controller.active_plan(Clock::at("2026-10-04 12:00:00"));
    ASSERT_EQ(1, plan.id);

    // stage 1 begins at 0 s, stage 2 at 38 s; transitions take 9 s and 6 s of the next stage
    const auto s = [](int value) { return Clock::duration(seconds(value)); };
    expect_cycle(controller.cycle(plan, 1), {
        { red, s(0), s(7) }, { red_amber, s(7), s(9) }, { permissive, s(9), s(41) },
        { clearance, s(41), s(44) }, { red, s(44), s(64) } });
    expect_cycle(controller.cycle(plan, 3), {
        { permissive, s(0), s(3) }, { clearance, s(3), s(6) }, { red, s(6), s(42) },
        { red_amber, s(42), s(44) }, { permissive, s(44), s(64) } });
    // pedestrians: no red-amber and no amber, flashing green still permits movement
    expect_cycle(controller.cycle(plan, 6), {
        { red, s(0), s(9) }, { permissive, s(9), s(41) }, { red, s(41), s(64) } });
    expect_cycle(controller.cycle(plan, 5), {
        { permissive, s(0), s(3) }, { red, s(3), s(44) }, { permissive, s(44), s(64) } });
    // trams: no red-amber and no amber, green blink still allows movement
    expect_cycle(controller.cycle(plan, 8), {
        { red, s(0), s(9) }, { permissive, s(9), s(41) }, { red, s(41), s(64) } });
}

TEST(FixedTimePlan, weekly_schedule)
{
    const FixedTimePlan controller(two_stage_config());
    EXPECT_EQ(1, controller.active_plan(Clock::at("2026-10-05 06:59:59")).id); // Monday
    EXPECT_EQ(2, controller.active_plan(Clock::at("2026-10-05 07:00:00")).id);
    EXPECT_EQ(1, controller.active_plan(Clock::at("2026-10-06 00:30:00")).id); // Tuesday
}

TEST(FixedTimePlan, spat_at_given_time)
{
    const FixedTimePlan controller(two_stage_config());
    // plan 1, offset 45 s: cycle position 20 s at 00:01:05
    const auto now = Clock::at("2026-10-04 00:01:05");

    asn1::Spatem spatem;
    spatem->header.protocolVersion = 2;
    spatem->header.messageID = ItsPduHeader__messageID_spatem;
    spatem->header.stationID = 1;
    controller.fill(spatem->spat, 7, 3, now);

    std::string error;
    ASSERT_TRUE(spatem.validate(error)) << error;
    asn1::Spatem decoded;
    ASSERT_TRUE(decoded.decode(spatem.encode()));

    const IntersectionState_t* intersection = decoded->spat.intersections.list.array[0];
    EXPECT_EQ(7, intersection->id.id);
    EXPECT_EQ(3, intersection->revision);
    EXPECT_EQ(0x04, intersection->status.buf[0]); // fixedTimeOperation
    ASSERT_EQ(9, intersection->states.list.count);

    // group 1 green since 00:00:54, until 00:01:26
    const MovementEvent_t* g1 = intersection->states.list.array[0]->state_time_speed.list.array[0];
    EXPECT_EQ(permissive, g1->eventState);
    ASSERT_NE(nullptr, g1->timing);
    EXPECT_EQ(nullptr, g1->timing->startTime); // before the current minute: omitted
    EXPECT_EQ(860, g1->timing->minEndTime);
    ASSERT_NE(nullptr, g1->timing->maxEndTime);
    EXPECT_EQ(860, *g1->timing->maxEndTime);
    ASSERT_NE(nullptr, g1->timing->confidence);
    EXPECT_EQ(15, *g1->timing->confidence);
    EXPECT_EQ(nullptr, g1->timing->nextTime);

    // followed by clearance until 00:01:29 and red until 00:01:56 (next phase)
    const MovementEventList_t& g1_events = intersection->states.list.array[0]->state_time_speed;
    ASSERT_EQ(3, g1_events.list.count);
    EXPECT_EQ(clearance, g1_events.list.array[1]->eventState);
    ASSERT_NE(nullptr, g1_events.list.array[1]->timing->startTime);
    EXPECT_EQ(860, *g1_events.list.array[1]->timing->startTime);
    EXPECT_EQ(890, g1_events.list.array[1]->timing->minEndTime);
    EXPECT_EQ(red, g1_events.list.array[2]->eventState);
    EXPECT_EQ(1160, g1_events.list.array[2]->timing->minEndTime);
    EXPECT_EQ(nullptr, g1_events.list.array[2]->timing->nextTime); // only for the current state

    // group 3 red until red-amber at 00:01:27, green at 00:01:29
    const MovementEventList_t& g3_events = intersection->states.list.array[2]->state_time_speed;
    const MovementEvent_t* g3 = g3_events.list.array[0];
    EXPECT_EQ(red, g3->eventState);
    EXPECT_EQ(870, g3->timing->minEndTime);
    ASSERT_NE(nullptr, g3->timing->nextTime);
    EXPECT_EQ(890, *g3->timing->nextTime);
    ASSERT_EQ(3, g3_events.list.count);
    EXPECT_EQ(red_amber, g3_events.list.array[1]->eventState);
    EXPECT_EQ(890, g3_events.list.array[1]->timing->minEndTime);
    EXPECT_EQ(permissive, g3_events.list.array[2]->eventState);
}

TEST(FixedTimePlan, rejects_inconsistent_config)
{
    ControllerConfig config = two_stage_config();
    config.stages.push_back({ 3, { 42 } });
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument);

    config = two_stage_config();
    config.week[2].push_back({ hours(8), 99 });
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument);
}

namespace
{

PathPoint p(double east, double north)
{
    return PathPoint { east * units::si::meter, north * units::si::meter };
}

// simple cross: stage 1 serves north and south approaches, stage 2 east and west
ControllerConfig cross_config(std::vector<StageConfig> stages)
{
    ControllerConfig config;
    for (long id : { 1, 2, 3 }) {
        config.groups.push_back(make_group(id, SignalGroupKind::Traffic));
    }
    config.groups.push_back(make_group(4, SignalGroupKind::Pedestrian));
    config.stages = std::move(stages);
    // intergreen times covering the 3 s amber, so different stages never overlap
    for (long clearing : { 1, 2, 3, 4 }) {
        for (long entering : { 1, 2, 3, 4 }) {
            if (clearing != entering) {
                config.intergreen[{ clearing, entering }] = seconds(3);
            }
        }
    }
    return config;
}

// southbound through on the west half, northbound through on the east half
const MovementPath southbound { 1, { p(-1.75, 15.0), p(-1.75, -12.0), p(-1.75, -40.0) } };
const MovementPath northbound { 2, { p(1.75, -15.0), p(1.75, 12.0), p(1.75, 40.0) } };
// northbound left turn into the west arm
const MovementPath northbound_left { 2, { p(1.75, -15.0), p(-12.0, 1.75), p(-40.0, 1.75) } };
// southbound right turn into the west arm
const MovementPath southbound_right { 1, { p(-1.75, 15.0), p(-12.0, -1.75), p(-40.0, -1.75) } };
// westbound through on the north half of the east and west arms
const MovementPath westbound { 3, { p(15.0, 1.75), p(-12.0, 1.75), p(-40.0, 1.75) } };
// pedestrians crossing the west arm
const MovementPath west_crosswalk { 4, { p(-14.0, -7.0), p(-14.0, 7.0) } };

} // namespace

TEST(FixedTimePlan, opposing_through_movements_are_protected)
{
    ControllerConfig config = cross_config({ { 1, { 1, 2 } }, { 2, { 3, 4 } } });
    derive_protection(config, { southbound, northbound, westbound, west_crosswalk });
    EXPECT_EQ(MovementPhaseState_protected_Movement_Allowed, config.groups[0].movement_allowed_state);
    EXPECT_EQ(MovementPhaseState_protected_clearance, config.groups[0].clearance_state);
    EXPECT_EQ(MovementPhaseState_protected_Movement_Allowed, config.groups[1].movement_allowed_state);
    // westbound through crosses the west crosswalk served in the same stage
    EXPECT_EQ(MovementPhaseState_permissive_Movement_Allowed, config.groups[2].movement_allowed_state);
    EXPECT_EQ(MovementPhaseState_permissive_Movement_Allowed, config.groups[3].movement_allowed_state);
}

TEST(FixedTimePlan, left_turn_against_opposing_traffic_is_permissive)
{
    ControllerConfig config = cross_config({ { 1, { 1, 2 } }, { 2, { 3 } } });
    derive_protection(config, { southbound, northbound, northbound_left, westbound });
    EXPECT_EQ(MovementPhaseState_permissive_Movement_Allowed, config.groups[0].movement_allowed_state);
    EXPECT_EQ(MovementPhaseState_permissive_clearance, config.groups[0].clearance_state);
    EXPECT_EQ(MovementPhaseState_permissive_Movement_Allowed, config.groups[1].movement_allowed_state);
    EXPECT_EQ(MovementPhaseState_protected_Movement_Allowed, config.groups[2].movement_allowed_state);
}

TEST(FixedTimePlan, turn_across_parallel_crosswalk_is_permissive)
{
    ControllerConfig config = cross_config({ { 1, { 1, 4 } }, { 2, { 2, 3 } } });
    derive_protection(config, { southbound_right, west_crosswalk, northbound });
    EXPECT_EQ(MovementPhaseState_permissive_Movement_Allowed, config.groups[0].movement_allowed_state);
    EXPECT_EQ(MovementPhaseState_permissive_Movement_Allowed, config.groups[3].movement_allowed_state);
}

TEST(FixedTimePlan, merging_into_same_lane_is_permissive)
{
    // southbound right turn and northbound left turn both end in the west arm
    const MovementPath northbound_left_south_half { 2, { p(1.75, -15.0), p(-12.0, -1.75), p(-40.0, -1.75) } };
    ControllerConfig config = cross_config({ { 1, { 1, 2 } }, { 2, { 3 } } });
    derive_protection(config, { southbound_right, northbound_left_south_half });
    EXPECT_EQ(MovementPhaseState_permissive_Movement_Allowed, config.groups[0].movement_allowed_state);
    EXPECT_EQ(MovementPhaseState_permissive_Movement_Allowed, config.groups[1].movement_allowed_state);
}

TEST(FixedTimePlan, rejects_stage_shorter_than_transition)
{
    ControllerConfig config = two_stage_config();
    // entering side road groups need 3 s intergreen + 3 s green blink of the main road groups
    config.plans = { { 1, { { 1, seconds(58) }, { 2, seconds(6) } }, seconds(0) } };
    for (auto& day : config.week) {
        day = { { hours(0), 1 } };
    }
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument);
}

TEST(FixedTimePlan, rejects_cycle_not_dividing_a_day)
{
    ControllerConfig config = two_stage_config();
    config.plans[0].sequence[1].second = seconds(27); // 65 s cycle
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument);
}

TEST(FixedTimePlan, group_without_paths_keeps_its_states)
{
    ControllerConfig config = cross_config({ { 1, { 1, 2 } }, { 2, { 3, 4 } } });
    config.groups[2].movement_allowed_state = MovementPhaseState_protected_Movement_Allowed;
    derive_protection(config, { southbound, northbound });
    EXPECT_EQ(MovementPhaseState_protected_Movement_Allowed, config.groups[2].movement_allowed_state);
    EXPECT_EQ(MovementPhaseState_permissive_Movement_Allowed, config.groups[3].movement_allowed_state);
}

TEST(FixedTimePlan, prediction_follows_plan_change)
{
    const FixedTimePlan controller(two_stage_config());
    // Monday 06:59:50, plan 1 (offset 45 s): position 57 s, plan 2 starts at 07:00:00
    const auto now = Clock::at("2026-10-05 06:59:50");

    asn1::Spatem spatem;
    spatem->header.protocolVersion = 2;
    spatem->header.messageID = ItsPduHeader__messageID_spatem;
    controller.fill(spatem->spat, 7, 3, now);
    std::string error;
    ASSERT_TRUE(spatem.validate(error)) << error;

    // group 1 is red until the plan change; plan 2 (offset 24 s, 80 s cycle) is at position
    // 56 s then, where group 1 is green until 57 s, i.e. 07:00:01
    const MovementEventList_t& events = spatem->spat.intersections.list.array[0]->states.list.array[0]->state_time_speed;
    ASSERT_EQ(2, events.list.count);
    const MovementEvent_t* red_event = events.list.array[0];
    EXPECT_EQ(red, red_event->eventState);
    EXPECT_EQ(0, red_event->timing->minEndTime); // 07:00:00.0
    ASSERT_NE(nullptr, red_event->timing->maxEndTime);
    EXPECT_EQ(0, *red_event->timing->maxEndTime);
    ASSERT_NE(nullptr, red_event->timing->confidence);
    ASSERT_NE(nullptr, red_event->timing->nextTime);
    EXPECT_EQ(0, *red_event->timing->nextTime);
    const MovementEvent_t* green_event = events.list.array[1];
    EXPECT_EQ(permissive, green_event->eventState);
    ASSERT_NE(nullptr, green_event->timing->startTime);
    EXPECT_EQ(0, *green_event->timing->startTime);
    EXPECT_EQ(10, green_event->timing->minEndTime);
}

TEST(FixedTimePlan, group_never_changing_has_no_end)
{
    ControllerConfig config = two_stage_config();
    config.groups.push_back(make_group(10, SignalGroupKind::Traffic)); // in no stage, always red
    const FixedTimePlan controller(config);
    const auto now = Clock::at("2026-10-05 06:30:00");

    asn1::Spatem spatem;
    controller.fill(spatem->spat, 7, 3, now);
    const IntersectionState_t* intersection = spatem->spat.intersections.list.array[0];
    const MovementEventList_t& events = intersection->states.list.array[9]->state_time_speed;
    ASSERT_EQ(1, events.list.count);
    EXPECT_EQ(red, events.list.array[0]->eventState);
    ASSERT_NE(nullptr, events.list.array[0]->timing);
    EXPECT_EQ(cTimeMarkOutOfRange, events.list.array[0]->timing->minEndTime); // red in every plan
    EXPECT_EQ(nullptr, events.list.array[0]->timing->maxEndTime);
    EXPECT_EQ(nullptr, events.list.array[0]->timing->confidence);
}

TEST(FixedTimePlan, group_beside_group_without_path_keeps_its_states)
{
    // group 2 may cross group 1, but its path is unknown
    ControllerConfig config = cross_config({ { 1, { 1, 2 } }, { 2, { 3 } } });
    derive_protection(config, { southbound, westbound });
    EXPECT_EQ(MovementPhaseState_permissive_Movement_Allowed, config.groups[0].movement_allowed_state);
    EXPECT_EQ(MovementPhaseState_permissive_clearance, config.groups[0].clearance_state);
    EXPECT_EQ(MovementPhaseState_protected_Movement_Allowed, config.groups[2].movement_allowed_state);

    // a known conflict is permissive regardless of unknown paths
    config = cross_config({ { 1, { 1, 2 } }, { 2, { 3, 4 } } });
    config.groups[2].movement_allowed_state = MovementPhaseState_protected_Movement_Allowed;
    derive_protection(config, { westbound, west_crosswalk });
    EXPECT_EQ(MovementPhaseState_permissive_Movement_Allowed, config.groups[2].movement_allowed_state);
}

TEST(FixedTimePlan, rejects_negative_durations)
{
    ControllerConfig config = two_stage_config();
    config.groups[0].green_blink = seconds(-1);
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument);

    config = two_stage_config();
    config.intergreen[{ 1, 3 }] = seconds(-10);
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument);

    config = two_stage_config();
    config.intergreen[{ 1, 42 }] = seconds(3);
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument);
}

TEST(FixedTimePlan, plan_change_beyond_one_hour_is_out_of_range)
{
    ControllerConfig config = two_stage_config();
    config.groups.push_back(make_group(10, SignalGroupKind::Traffic)); // in no stage, always red
    const FixedTimePlan controller(config);
    // Sunday noon, the next plan change is on Monday 07:00
    const auto now = Clock::at("2026-10-04 12:00:00");

    asn1::Spatem spatem;
    controller.fill(spatem->spat, 7, 3, now);
    const IntersectionState_t* intersection = spatem->spat.intersections.list.array[0];
    const MovementEvent_t* g10 = intersection->states.list.array[9]->state_time_speed.list.array[0];
    ASSERT_NE(nullptr, g10->timing);
    EXPECT_EQ(cTimeMarkOutOfRange, g10->timing->minEndTime);
    EXPECT_EQ(nullptr, g10->timing->maxEndTime);
    EXPECT_EQ(nullptr, g10->timing->nextTime);
}

TEST(FixedTimePlan, state_continuing_across_plan_change_is_joined)
{
    ControllerConfig config = two_stage_config();
    config.week[0] = { { hours(0), 1 }, { hours(7) + seconds(4), 2 } };
    const FixedTimePlan controller(config);
    // plan 1 position 6 s: group 1 red until 7 s, exactly when plan 2 takes over at position
    // 60 s, where group 1 is red until 7 s of the next cycle (07:00:31)
    const auto now = Clock::at("2026-10-05 07:00:03");

    asn1::Spatem spatem;
    controller.fill(spatem->spat, 7, 3, now);
    const MovementEventList_t& events = spatem->spat.intersections.list.array[0]->states.list.array[0]->state_time_speed;
    ASSERT_EQ(3, events.list.count);
    EXPECT_EQ(red, events.list.array[0]->eventState);
    EXPECT_EQ(310, events.list.array[0]->timing->minEndTime);
    ASSERT_NE(nullptr, events.list.array[0]->timing->likelyTime);
    EXPECT_EQ(310, *events.list.array[0]->timing->likelyTime);
    ASSERT_NE(nullptr, events.list.array[0]->timing->nextTime);
    EXPECT_EQ(330, *events.list.array[0]->timing->nextTime);
    EXPECT_EQ(red_amber, events.list.array[1]->eventState);
    EXPECT_EQ(330, events.list.array[1]->timing->minEndTime);
    EXPECT_EQ(permissive, events.list.array[2]->eventState);
    EXPECT_EQ(810, events.list.array[2]->timing->minEndTime); // green until 57 s, 07:01:21
}

TEST(FixedTimePlan, start_before_plan_change_is_omitted)
{
    ControllerConfig config = two_stage_config();
    config.week[0] = { { hours(0), 1 }, { hours(7) + seconds(30), 2 } };
    const FixedTimePlan controller(config);
    // plan 2 (offset 24 s, 80 s cycle) took over at 07:00:30, position 6.5 s: group 1 red until 7 s
    const auto now = Clock::at("2026-10-05 07:00:30.500");

    asn1::Spatem spatem;
    controller.fill(spatem->spat, 7, 3, now);
    const MovementEvent_t* g1 = spatem->spat.intersections.list.array[0]->states.list.array[0]->state_time_speed.list.array[0];
    EXPECT_EQ(red, g1->eventState);
    EXPECT_EQ(nullptr, g1->timing->startTime); // red of plan 2 began before 07:00:30
    EXPECT_EQ(310, g1->timing->minEndTime);
}

TEST(FixedTimePlan, constant_state_without_plan_change_is_out_of_range)
{
    ControllerConfig config = two_stage_config();
    config.groups.push_back(make_group(10, SignalGroupKind::Traffic)); // in no stage, always red
    for (auto& day : config.week) {
        day = { { hours(0), 1 } };
    }
    const FixedTimePlan controller(config);

    asn1::Spatem spatem;
    controller.fill(spatem->spat, 7, 3, Clock::at("2026-10-05 06:30:00"));
    const MovementEvent_t* g10 = spatem->spat.intersections.list.array[0]->states.list.array[9]->state_time_speed.list.array[0];
    EXPECT_EQ(red, g10->eventState);
    ASSERT_NE(nullptr, g10->timing);
    EXPECT_EQ(cTimeMarkOutOfRange, g10->timing->minEndTime);
}

TEST(FixedTimePlan, rejects_times_off_the_grid_and_invalid_schedules)
{
    ControllerConfig config = two_stage_config();
    config.plans[0].offset = std::chrono::milliseconds(45025);
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument);

    config = two_stage_config();
    config.week[0].push_back({ hours(30), 2 });
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument);

    config = two_stage_config();
    config.week[0].push_back({ hours(7), 1 }); // second plan at 07:00
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument);
}

TEST(FixedTimePlan, conflicting_groups_of_different_stages_need_intergreen)
{
    // two crossing vehicle movements served in separate stages
    const MovementPath northbound_through { 1, { p(0.0, -10.0), p(0.0, 10.0) } };
    const MovementPath eastbound_through { 2, { p(-10.0, 0.0), p(10.0, 0.0) } };
    ControllerConfig config;
    config.groups = { make_group(1, SignalGroupKind::Traffic), make_group(2, SignalGroupKind::Traffic) };
    config.stages = { { 1, { 1 } }, { 2, { 2 } } };

    // without intergreen times green of one group overlaps green blink and amber of the other
    derive_protection(config, { northbound_through, eastbound_through });
    EXPECT_EQ(MovementPhaseState_permissive_Movement_Allowed, config.groups[0].movement_allowed_state);
    EXPECT_EQ(MovementPhaseState_permissive_Movement_Allowed, config.groups[1].movement_allowed_state);

    // 2 s do not cover the 3 s amber of group 2
    config.intergreen[{ 1, 2 }] = seconds(3);
    config.intergreen[{ 2, 1 }] = seconds(2);
    derive_protection(config, { northbound_through, eastbound_through });
    EXPECT_EQ(MovementPhaseState_permissive_Movement_Allowed, config.groups[0].movement_allowed_state);
    EXPECT_EQ(MovementPhaseState_permissive_Movement_Allowed, config.groups[1].movement_allowed_state);

    config.intergreen[{ 2, 1 }] = seconds(3);
    derive_protection(config, { northbound_through, eastbound_through });
    EXPECT_EQ(MovementPhaseState_protected_Movement_Allowed, config.groups[0].movement_allowed_state);
    EXPECT_EQ(MovementPhaseState_protected_clearance, config.groups[0].clearance_state);
    EXPECT_EQ(MovementPhaseState_protected_Movement_Allowed, config.groups[1].movement_allowed_state);
}

TEST(FixedTimePlan, rejects_invalid_groups_and_identifiers)
{
    ControllerConfig config = two_stage_config();
    config.groups.clear();
    config.stages = { { 1, {} } };
    config.intergreen.clear();
    config.plans = { { 1, { { 1, seconds(10) } }, seconds(0) } };
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument); // no signal group

    config = two_stage_config();
    for (long id = 10; id <= 256; ++id) {
        config.groups.push_back(make_group(id, SignalGroupKind::Traffic));
    }
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument); // more than 255 groups

    config = two_stage_config();
    config.groups.push_back(make_group(256, SignalGroupKind::Traffic));
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument); // SignalGroupID beyond 255

    config = two_stage_config();
    config.groups.push_back(make_group(1, SignalGroupKind::Pedestrian));
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument); // duplicate group

    config = two_stage_config();
    config.stages.push_back({ 1, { 3 } });
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument); // duplicate stage

    config = two_stage_config();
    config.plans.push_back(config.plans.front());
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument); // duplicate plan
}

TEST(FixedTimePlan, rejects_times_longer_than_a_day)
{
    const Clock::duration huge = std::chrono::microseconds(9223372036854700000);

    ControllerConfig config = two_stage_config();
    config.plans[0].sequence[0].second = huge;
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument);

    config = two_stage_config();
    config.intergreen[{ 1, 3 }] = huge;
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument);

    config = two_stage_config();
    config.plans[0].offset = -huge;
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument);

    config = two_stage_config();
    config.utc_offset = hours(25);
    EXPECT_THROW(FixedTimePlan { config }, std::invalid_argument);
}
