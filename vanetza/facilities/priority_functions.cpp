#include <vanetza/asn1/its/IntersectionAccessPoint.h>
#include <vanetza/asn1/its/RequestorType.h>
#include <vanetza/asn1/its/VehicleID.h>
#include <vanetza/asn1/its/SignalRequestList.h>
#include <vanetza/asn1/its/SignalRequestMessage.h>
#include <vanetza/asn1/its/SignalRequestPackage.h>
#include <vanetza/asn1/its/SignalRequesterInfo.h>
#include <vanetza/asn1/its/SignalStatus.h>
#include <vanetza/asn1/its/SignalStatusMessage.h>
#include <vanetza/asn1/its/SignalStatusPackage.h>
#include <vanetza/facilities/priority_functions.hpp>
#include <vanetza/facilities/spat_functions.hpp>
#include <vanetza/facilities/detail/asn1_list.tpp>
#include <chrono>
#include <stdexcept>

namespace vanetza
{
namespace facilities
{

namespace
{

using detail::allocate_optional;
using detail::append;

void set_lane(IntersectionAccessPoint_t& point, long lane)
{
    point.present = IntersectionAccessPoint_PR_lane;
    point.choice.lane = lane;
}

void set_station(VehicleID_t& id, std::uint32_t station_id)
{
    id.present = VehicleID_PR_stationID;
    id.choice.stationID = station_id;
}

long to_dsecond(Clock::duration duration)
{
    // check before truncating to milliseconds
    if (duration < Clock::duration::zero() || duration > std::chrono::milliseconds(65535)) {
        throw std::out_of_range("duration exceeds DSecond");
    }
    return static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(duration).count());
}

} // namespace

void set_requestor(SignalRequestMessage& message, std::uint32_t station_id, long role,
        boost::optional<long> vehicle_type)
{
    // replace any previous identifier and type, e.g. of a decoded message
    ASN_STRUCT_RESET(asn_DEF_VehicleID, &message.requestor.id);
    set_station(message.requestor.id, station_id);
    ASN_STRUCT_FREE(asn_DEF_RequestorType, message.requestor.type);
    message.requestor.type = nullptr;
    RequestorType_t* type = allocate_optional(message.requestor.type);
    type->role = role;
    if (vehicle_type) {
        *allocate_optional(type->hpmsType) = *vehicle_type;
    }
}

void set_timestamp(SignalRequestMessage& message, const Clock::time_point& now)
{
    *allocate_optional(message.timeStamp) = minute_of_the_year(now);
    message.second = dsecond(now);
}

void add_request(SignalRequestMessage& message, const PriorityRequest& request)
{
    auto package = asn1::make_unique<SignalRequestPackage_t>(asn_DEF_SignalRequestPackage);
    SignalRequest_t& details = package->request;
    details.id.id = request.intersection_id;
    details.requestID = request.request_id;
    details.requestType = request.request_type;
    set_lane(details.inBoundLane, request.inbound_lane);
    if (request.outbound_lane) {
        set_lane(*allocate_optional(details.outBoundLane), *request.outbound_lane);
    }
    if (request.arrival) {
        *allocate_optional(package->minute) = minute_of_the_year(*request.arrival);
        *allocate_optional(package->second) = dsecond(*request.arrival);
    }
    if (request.duration) {
        *allocate_optional(package->duration) = to_dsecond(*request.duration);
    }
    append(*allocate_optional(message.requests), std::move(package));
}

void set_timestamp(SignalStatusMessage& message, const Clock::time_point& now)
{
    *allocate_optional(message.timeStamp) = minute_of_the_year(now);
    message.second = dsecond(now);
}

SignalStatus& add_status(SignalStatusMessage& message, long intersection_id, long sequence_number)
{
    auto status = asn1::make_unique<SignalStatus_t>(asn_DEF_SignalStatus);
    status->id.id = intersection_id;
    status->sequenceNumber = sequence_number;
    return append(message.status, std::move(status));
}

void add_response(SignalStatus& status, const PriorityResponse& response)
{
    auto package = asn1::make_unique<SignalStatusPackage_t>(asn_DEF_SignalStatusPackage);
    SignalRequesterInfo_t* requester = allocate_optional(package->requester);
    set_station(requester->id, response.requester);
    requester->request = response.request_id;
    requester->sequenceNumber = response.request_sequence;
    *allocate_optional(requester->role) = response.role;
    set_lane(package->inboundOn, response.inbound_lane);
    if (response.outbound_lane) {
        set_lane(*allocate_optional(package->outboundOn), *response.outbound_lane);
    }
    package->status = response.status;
    append(status.sigStatus, std::move(package));
}

} // namespace facilities
} // namespace vanetza
