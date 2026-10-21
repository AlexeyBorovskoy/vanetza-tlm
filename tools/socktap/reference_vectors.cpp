#include "reference_vectors.hpp"
#include "fixed_time_intersection.hpp"
#include <vanetza/asn1/mapem.hpp>
#include <vanetza/asn1/spatem.hpp>
#include <vanetza/asn1/srem.hpp>
#include <vanetza/asn1/ssem.hpp>
#include <vanetza/asn1/its/BasicVehicleRole.h>
#include <vanetza/asn1/its/IntersectionGeometry.h>
#include <vanetza/asn1/its/IntersectionGeometryList.h>
#include <vanetza/asn1/its/IntersectionState.h>
#include <vanetza/asn1/its/PrioritizationResponseStatus.h>
#include <vanetza/asn1/its/PriorityRequestType.h>
#include <vanetza/asn1/its/RoadRegulatorID.h>
#include <vanetza/facilities/priority_functions.hpp>
#include <vanetza/units/angle.hpp>
#include <chrono>
#include <stdexcept>

using namespace vanetza;
using namespace vanetza::facilities;

namespace
{

const long intersection_id = 1;
const long region = 1; // example RoadRegulatorID, C2C-CC RS 2077 RS_ARSM_11 requires a region
const long road_side_station = 10;
const long tram_station = 20;
// static position of socktap
const double latitude = 48.7668616;
const double longitude = 11.432068;

// priority request of the tram: ingress track 2, egress track 10 (see FixedTimeIntersection)
const long tram_ingress_lane = 2;
const long tram_egress_lane = 10;
const long request_id = 1;
const char* const request_time = "2026-10-06 10:00:00.000";
const char* const status_time = "2026-10-06 10:00:00.100";

std::string two_digits(int value)
{
    return (value < 10 ? "0" : "") + std::to_string(value);
}

void set_region(IntersectionReferenceID_t& id)
{
    id.region = asn1::allocate<RoadRegulatorID_t>();
    *id.region = region;
}

void set_header(ItsPduHeader_t& header, long message_id, long station_id)
{
    header.protocolVersion = 2;
    header.messageID = message_id;
    header.stationID = station_id;
}

template<typename MESSAGE>
ReferenceVector make_vector(const std::string& name, const std::string& message, const std::string& time,
    const std::string& description, const MESSAGE& encoded, asn_TYPE_descriptor_t& type)
{
    std::string error;
    if (!encoded.validate(error)) {
        throw std::runtime_error("invalid reference vector " + name + ": " + error);
    }
    ReferenceVector vector;
    vector.name = name;
    vector.message = message;
    vector.time = time;
    vector.description = description;
    vector.uper = encoded.encode();
    vector.xer = asn1::encode_xer(type, encoded.content());
    return vector;
}

ReferenceVector mapem_vector(const FixedTimeIntersection& intersection)
{
    asn1::Mapem mapem;
    set_header(mapem->header, ItsPduHeader__messageID_mapem, road_side_station);
    mapem->map.msgIssueRevision = 0;
    intersection.fill(mapem->map, latitude * units::degree, longitude * units::degree);
    set_region(mapem->map.intersections->list.array[0]->id);
    return make_vector("mapem", "mapem", "", "geometry of the example intersection: 17 lanes, "
        "18 connections, signal groups 1-4 vehicles, 5-7 pedestrians, 8-9 trams", mapem, asn_DEF_MAPEM);
}

ReferenceVector spatem_vector(const FixedTimeIntersection& intersection, const std::string& name,
    const std::string& time, const std::string& description)
{
    asn1::Spatem spatem;
    set_header(spatem->header, ItsPduHeader__messageID_spatem, road_side_station);
    intersection.fill(spatem->spat, Clock::at(time));
    set_region(spatem->spat.intersections.list.array[0]->id);
    return make_vector(name, "spatem", time, description, spatem, asn_DEF_SPATEM);
}

ReferenceVector srem_vector()
{
    const Clock::time_point now = Clock::at(request_time);
    asn1::Srem srem;
    set_header(srem->header, ItsPduHeader__messageID_srem, tram_station);
    set_timestamp(srem->srm, now);
    srem->srm.sequenceNumber = asn1::allocate<MsgCount_t>();
    *srem->srm.sequenceNumber = 0;
    set_requestor(srem->srm, tram_station, BasicVehicleRole_publicTransport);

    PriorityRequest request;
    request.intersection_id = intersection_id;
    request.request_id = request_id;
    request.request_type = PriorityRequestType_priorityRequest;
    request.inbound_lane = tram_ingress_lane;
    request.outbound_lane = tram_egress_lane;
    request.arrival = now + std::chrono::seconds(20);
    request.duration = std::chrono::seconds(8);
    add_request(srem->srm, request);
    return make_vector("srem_tram_request", "srem", request_time,
        "priority request of the tram on track 2 to track 10, arrival in 20 s for 8 s", srem, asn_DEF_SREM);
}

ReferenceVector ssem_vector()
{
    asn1::Ssem ssem;
    set_header(ssem->header, ItsPduHeader__messageID_ssem, road_side_station);
    set_timestamp(ssem->ssm, Clock::at(status_time));
    SignalStatus& status = add_status(ssem->ssm, intersection_id, 1); // first change of the request table

    PriorityResponse response;
    response.requester = tram_station;
    response.request_id = request_id;
    response.request_sequence = 0;
    response.role = BasicVehicleRole_publicTransport;
    response.inbound_lane = tram_ingress_lane;
    response.outbound_lane = tram_egress_lane;
    response.status = PrioritizationResponseStatus_processing;
    add_response(status, response);
    return make_vector("ssem_tram_processing", "ssem", status_time,
        "answer of the intersection to the tram request: processing", ssem, asn_DEF_SSEM);
}

} // namespace

std::vector<ReferenceVector> reference_vectors()
{
    const FixedTimeIntersection intersection(intersection_id);
    std::vector<ReferenceVector> vectors;
    vectors.push_back(mapem_vector(intersection));

    // Tuesday, plan 2 (80 s cycle) from 09:30, one cycle in steps of 10 s
    for (int second = 0; second < 80; second += 10) {
        const std::string time = "2026-10-06 10:" + two_digits(second / 60) + ":" + two_digits(second % 60) + ".000";
        vectors.push_back(spatem_vector(intersection, "spatem_cycle_" + two_digits(second) + "s", time,
            "plan 2 (80 s cycle), " + std::to_string(second) + " s after 10:00:00"));
    }
    vectors.push_back(spatem_vector(intersection, "spatem_hour_boundary", "2026-10-06 10:59:55.000",
        "plan 2, events ending in the next hour (TimeMark below the minute of moy)"));
    vectors.push_back(spatem_vector(intersection, "spatem_plan_change", "2026-10-06 06:59:50.000",
        "plan 1 (night) changing to plan 4 (peak) at 07:00"));

    vectors.push_back(srem_vector());
    vectors.push_back(ssem_vector());
    return vectors;
}
