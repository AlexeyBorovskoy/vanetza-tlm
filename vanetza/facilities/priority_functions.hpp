#ifndef PRIORITY_FUNCTIONS_HPP_M3RZ8WQE
#define PRIORITY_FUNCTIONS_HPP_M3RZ8WQE

#include <vanetza/common/clock.hpp>
#include <boost/optional/optional.hpp>
#include <cstdint>

// forward declaration of asn1c generated struct
struct SignalRequestMessage;
struct SignalStatus;
struct SignalStatusMessage;

namespace vanetza
{
namespace facilities
{

/**
 * Builders for signal request (SREM) and signal status (SSEM) messages
 * of the Traffic Light Control service (ETSI TS 103 301, ISO TS 19091 DSRC data elements).
 */

/**
 * Describe the requesting vehicle of a signal request message, replacing any previous requestor
 * \param message SignalRequestMessage (destination)
 * \param station_id station ID of the requesting vehicle
 * \param role BasicVehicleRole, e.g. publicTransport
 * \param vehicle_type optional VehicleType (HPMS), e.g. bus
 */
void set_requestor(SignalRequestMessage& message, std::uint32_t station_id, long role,
        boost::optional<long> vehicle_type = boost::none);

/**
 * Set timeStamp and second of a signal request message
 * \param message SignalRequestMessage (destination)
 * \param now time of the message
 */
void set_timestamp(SignalRequestMessage& message, const Clock::time_point& now);

/** One request for priority or pre-emption at an intersection */
struct PriorityRequest
{
    long intersection_id; /**< IntersectionID (0..65535) */
    long request_id; /**< RequestID unique for this requestor (0..255) */
    long request_type; /**< PriorityRequestType, e.g. priorityRequest */
    long inbound_lane; /**< ingress lane identifier (0..255) */
    boost::optional<long> outbound_lane; /**< egress lane identifier (0..255) */
    boost::optional<Clock::time_point> arrival; /**< estimated time of arrival at the stop line */
    boost::optional<Clock::duration> duration; /**< expected duration of the service */
};

/**
 * Append a request to a signal request message
 * \param message SignalRequestMessage (destination)
 * \param request request details
 * \throw std::out_of_range if duration exceeds DSecond (65.535 s)
 */
void add_request(SignalRequestMessage& message, const PriorityRequest& request);

/**
 * Set timeStamp and second of a signal status message
 * \param message SignalStatusMessage (destination)
 * \param now time of the message
 */
void set_timestamp(SignalStatusMessage& message, const Clock::time_point& now);

/**
 * Append the status of an intersection to a signal status message
 * \param message SignalStatusMessage (destination)
 * \param intersection_id IntersectionID (0..65535)
 * \param sequence_number MsgCount, incremented when the status of any request changes
 * \return appended signal status, owned by message
 */
SignalStatus& add_status(SignalStatusMessage& message, long intersection_id, long sequence_number);

/** Answer of the intersection to one request */
struct PriorityResponse
{
    std::uint32_t requester; /**< station ID of the requesting vehicle */
    long request_id; /**< RequestID of the request */
    long request_sequence; /**< sequenceNumber of the answered signal request message */
    long role; /**< BasicVehicleRole of the requestor */
    long inbound_lane; /**< ingress lane identifier (0..255) */
    boost::optional<long> outbound_lane; /**< egress lane identifier (0..255) */
    long status; /**< PrioritizationResponseStatus, e.g. processing or granted */
};

/**
 * Append a response package to a signal status
 * \param status signal status (destination)
 * \param response response details
 */
void add_response(SignalStatus& status, const PriorityResponse& response);

} // namespace facilities
} // namespace vanetza

#endif /* PRIORITY_FUNCTIONS_HPP_M3RZ8WQE */
