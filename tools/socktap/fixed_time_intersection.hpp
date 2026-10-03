#ifndef FIXED_TIME_INTERSECTION_HPP_T4XQ7MZE
#define FIXED_TIME_INTERSECTION_HPP_T4XQ7MZE

#include <vanetza/common/clock.hpp>
#include <vanetza/facilities/fixed_time_plan.hpp>
#include <vanetza/units/angle.hpp>

struct MapData;
struct SPAT;

/**
 * Example intersection of a main road with tram tracks and a side road.
 *
 * Nine signal groups: 1-4 motor vehicles, 5-7 pedestrian crossings, 8-9 trams.
 * Two stages, five fixed-time plans (64, 80 and 96 s cycles) and a weekly schedule.
 * Main road arms point east and west, side road arms north and south; right-hand traffic.
 */
class FixedTimeIntersection
{
public:
    explicit FixedTimeIntersection(long id);

    long id() const { return id_; }

    /**
     * Add the intersection state at given time to SPAT
     * \param spat destination
     * \param now current time
     */
    void fill(SPAT& spat, vanetza::Clock::time_point now) const;

    /**
     * Add the intersection geometry to MapData
     * \param map destination
     * \param latitude reference point latitude
     * \param longitude reference point longitude
     */
    void fill(MapData& map, vanetza::units::GeoAngle latitude, vanetza::units::GeoAngle longitude) const;

    /**
     * Check whether a lane leads into the intersection, e.g. for validating priority requests
     * \param lane lane identifier as used in MAPEM
     * \return true for ingress (or bidirectional) lanes
     */
    bool is_ingress_lane(long lane) const;

private:
    long id_;
    vanetza::facilities::FixedTimePlan plan_;
};

#endif /* FIXED_TIME_INTERSECTION_HPP_T4XQ7MZE */
