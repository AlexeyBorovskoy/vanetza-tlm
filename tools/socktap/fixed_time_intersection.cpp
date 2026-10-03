#include "fixed_time_intersection.hpp"
#include <vanetza/asn1/its/MapData.h>
#include <vanetza/asn1/its/MovementPhaseState.h>
#include <vanetza/asn1/its/SPAT.h>
#include <vanetza/facilities/map_functions.hpp>
#include <boost/units/systems/si/length.hpp>
#include <chrono>
#include <map>
#include <stdexcept>
#include <vector>

using namespace vanetza;
using namespace vanetza::facilities;
using std::chrono::hours;
using std::chrono::minutes;
using std::chrono::seconds;

namespace
{

// revision of MAPEM geometry and SPATEM state, both must be equal
const long revision = 0;

enum Arm : long { East = 1, West = 2, North = 3, South = 4 };

ControllerConfig make_config()
{
    ControllerConfig config;
    auto group = [&config](long id, SignalGroupKind kind) {
        config.groups.push_back({ id, kind, seconds(2), seconds(3), seconds(3),
            MovementPhaseState_permissive_Movement_Allowed, MovementPhaseState_permissive_clearance }); // see derive_protection()
    };
    group(1, SignalGroupKind::Traffic); // west approach
    group(2, SignalGroupKind::Traffic); // east approach
    group(3, SignalGroupKind::Traffic); // south approach
    group(4, SignalGroupKind::Traffic); // north approach
    group(5, SignalGroupKind::Pedestrian); // crossing the west arm
    group(6, SignalGroupKind::Pedestrian); // crossing the north arm
    group(7, SignalGroupKind::Pedestrian); // crossing the south arm
    group(8, SignalGroupKind::Tram); // westbound tram
    group(9, SignalGroupKind::Tram); // eastbound tram

    config.stages = { { 1, { 1, 2, 6, 7, 8, 9 } }, { 2, { 3, 4, 5 } } };
    for (long main : { 1, 2, 6, 7, 8, 9 }) {
        for (long side : { 3, 4, 5 }) {
            config.intergreen[{ main, side }] = seconds(3);
            config.intergreen[{ side, main }] = seconds(6);
        }
    }

    config.plans = {
        { 1, { { 1, seconds(38) }, { 2, seconds(26) } }, seconds(45) }, // night
        { 2, { { 1, seconds(54) }, { 2, seconds(26) } }, seconds(24) }, // off-peak
        { 3, { { 1, seconds(54) }, { 2, seconds(26) } }, seconds(4) },
        { 4, { { 1, seconds(70) }, { 2, seconds(26) } }, seconds(36) }, // peak
        { 5, { { 1, seconds(70) }, { 2, seconds(26) } }, seconds(3) },
    };
    const std::vector<ScheduleEntry> working_day = {
        { hours(7), 4 }, { hours(9) + minutes(30), 2 }, { hours(13), 3 },
        { hours(16) + minutes(30), 5 }, { hours(20), 3 }, { hours(23), 1 } };
    const std::vector<ScheduleEntry> friday = {
        { hours(7), 4 }, { hours(9) + minutes(30), 2 }, { hours(13), 3 },
        { hours(15) + minutes(30), 5 }, { hours(19), 3 }, { hours(23), 1 } };
    const std::vector<ScheduleEntry> weekend = { { hours(7), 2 }, { hours(13), 3 }, { hours(23), 1 } };
    config.week = {{ working_day, working_day, working_day, working_day, friday, weekend, weekend }};
    config.utc_offset = Clock::duration::zero();
    return config;
}

const auto meter = units::si::meter;

enum class LaneKind { Vehicle, Tram, Crosswalk };

struct LaneConfig
{
    long id;
    LaneKind kind;
    TravelDirection direction;
    long arm;
    // first node at the stop line (ingress) or the intersection box (egress), last node outwards
    double east, north, east_end, north_end;
};

const std::vector<LaneConfig> lanes = {
    // east arm: westbound lanes north of the centre line, tram tracks in the middle
    { 1, LaneKind::Vehicle, TravelDirection::Ingress, East, 12.0, 5.25, 80.0, 5.25 },
    { 2, LaneKind::Tram, TravelDirection::Ingress, East, 12.0, 1.75, 80.0, 1.75 },
    { 3, LaneKind::Vehicle, TravelDirection::Egress, East, 10.0, -5.25, 80.0, -5.25 },
    { 4, LaneKind::Tram, TravelDirection::Egress, East, 10.0, -1.75, 80.0, -1.75 },
    // west arm: eastbound lanes south of the centre line, crosswalk in front of the stop line
    { 5, LaneKind::Vehicle, TravelDirection::Ingress, West, -18.0, -8.75, -80.0, -8.75 },
    { 6, LaneKind::Vehicle, TravelDirection::Ingress, West, -18.0, -5.25, -80.0, -5.25 },
    { 7, LaneKind::Tram, TravelDirection::Ingress, West, -18.0, -1.75, -80.0, -1.75 },
    { 8, LaneKind::Vehicle, TravelDirection::Egress, West, -10.0, 8.75, -80.0, 8.75 },
    { 9, LaneKind::Vehicle, TravelDirection::Egress, West, -10.0, 5.25, -80.0, 5.25 },
    { 10, LaneKind::Tram, TravelDirection::Egress, West, -10.0, 1.75, -80.0, 1.75 },
    // north and south arms: one lane per direction, crosswalks in front of the stop lines
    { 11, LaneKind::Vehicle, TravelDirection::Ingress, North, -1.75, 18.0, -1.75, 60.0 },
    { 12, LaneKind::Vehicle, TravelDirection::Egress, North, 1.75, 12.0, 1.75, 60.0 },
    { 13, LaneKind::Vehicle, TravelDirection::Ingress, South, 1.75, -18.0, 1.75, -60.0 },
    { 14, LaneKind::Vehicle, TravelDirection::Egress, South, -1.75, -12.0, -1.75, -60.0 },
    { 15, LaneKind::Crosswalk, TravelDirection::Both, West, -14.0, -10.5, -14.0, 10.5 },
    { 16, LaneKind::Crosswalk, TravelDirection::Both, North, -3.5, 14.0, 3.5, 14.0 },
    { 17, LaneKind::Crosswalk, TravelDirection::Both, South, -3.5, -14.0, 3.5, -14.0 },
};

struct ConnectionConfig
{
    long from;
    long to;
    Maneuver maneuver;
    long signal_group;
};

const std::vector<ConnectionConfig> connections = {
    { 5, 14, Maneuver::Right, 1 }, { 6, 12, Maneuver::Left, 1 }, { 6, 3, Maneuver::Straight, 1 },
    { 1, 14, Maneuver::Left, 2 }, { 1, 12, Maneuver::Right, 2 }, { 1, 8, Maneuver::Straight, 2 },
    { 1, 9, Maneuver::Straight, 2 },
    { 13, 12, Maneuver::Straight, 3 }, { 13, 3, Maneuver::Right, 3 }, { 13, 8, Maneuver::Left, 3 },
    { 11, 14, Maneuver::Straight, 4 }, { 11, 3, Maneuver::Left, 4 }, { 11, 8, Maneuver::Right, 4 },
    { 2, 10, Maneuver::Straight, 8 }, { 7, 4, Maneuver::Straight, 9 },
    // crosswalk self-loop compatibility convention: a single crosswalk lane connects to itself
    // to carry its signal group; ISO TS 19091 neither requires nor forbids it (G.8.2.6 only
    // defines ingress/egress approaches of bidirectional lanes)
    { 15, 15, Maneuver::Straight, 5 }, { 16, 16, Maneuver::Straight, 6 }, { 17, 17, Maneuver::Straight, 7 },
};

const LaneConfig& find_lane(long id)
{
    for (const LaneConfig& lane : lanes) {
        if (lane.id == id) {
            return lane;
        }
    }
    throw std::logic_error("unknown lane");
}

// stop line, entry of the connected lane and its far end; a crosswalk path is its centre line
std::vector<MovementPath> movement_paths()
{
    std::vector<MovementPath> paths;
    for (const ConnectionConfig& connection : connections) {
        const LaneConfig& from = find_lane(connection.from);
        const LaneConfig& to = find_lane(connection.to);
        MovementPath path;
        path.signal_group = connection.signal_group;
        path.points.push_back({ from.east * meter, from.north * meter });
        if (from.kind == LaneKind::Crosswalk) {
            path.points.push_back({ from.east_end * meter, from.north_end * meter });
        } else {
            path.points.push_back({ to.east * meter, to.north * meter });
            path.points.push_back({ to.east_end * meter, to.north_end * meter });
        }
        paths.push_back(path);
    }
    return paths;
}

ControllerConfig make_protected_config()
{
    ControllerConfig config = make_config();
    derive_protection(config, movement_paths());
    return config;
}

} // namespace

FixedTimeIntersection::FixedTimeIntersection(long id) : id_(id), plan_(make_protected_config())
{
}

void FixedTimeIntersection::fill(SPAT& spat, Clock::time_point now) const
{
    plan_.fill(spat, id_, revision, now);
}

void FixedTimeIntersection::fill(MapData& map, units::GeoAngle latitude, units::GeoAngle longitude) const
{
    IntersectionGeometry& intersection = add_intersection_geometry(map, id_, revision, latitude, longitude, 3.5 * meter);

    std::map<long, GenericLane*> generic_lanes;
    for (const LaneConfig& lane : lanes) {
        GenericLane* generic = nullptr;
        switch (lane.kind) {
            case LaneKind::Vehicle:
                generic = &add_vehicle_lane(intersection, lane.id, lane.direction, lane.arm);
                break;
            case LaneKind::Tram:
                generic = &add_tram_lane(intersection, lane.id, lane.direction, lane.arm);
                break;
            case LaneKind::Crosswalk:
                generic = &add_crosswalk_lane(intersection, lane.id, lane.arm, lane.arm);
                break;
        }
        add_node(*generic, lane.east * meter, lane.north * meter);
        add_node(*generic, (lane.east_end - lane.east) * meter, (lane.north_end - lane.north) * meter);
        generic_lanes[lane.id] = generic;
    }

    long connection_id = 0;
    for (const ConnectionConfig& connection : connections) {
        connect(*generic_lanes.at(connection.from), connection.to, connection.maneuver,
            connection.signal_group, ++connection_id);
    }
}

bool FixedTimeIntersection::is_ingress_lane(long lane) const
{
    for (const LaneConfig& config : lanes) {
        if (config.id == lane) {
            return config.direction != TravelDirection::Egress;
        }
    }
    return false;
}
