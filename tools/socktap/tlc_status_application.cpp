#include "tlc_status_application.hpp"
#include "infrastructure_message.hpp"
#include <vanetza/asn1/its/IntersectionAccessPoint.h>
#include <vanetza/asn1/its/RequestorType.h>
#include <vanetza/asn1/its/SignalRequestList.h>
#include <vanetza/asn1/its/SignalRequestPackage.h>
#include <vanetza/asn1/packet_visitor.hpp>
#include <vanetza/btp/ports.hpp>
#include <vanetza/common/its_aid.hpp>
#include <vanetza/facilities/priority_functions.hpp>
#include <vanetza/facilities/spat_functions.hpp>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <vector>

// This is a very simple TLC status application answering priority requests.

using namespace vanetza;
using namespace vanetza::facilities;

namespace
{

// requests not repeated within this time after their expected service are forgotten
const Clock::duration request_retention = std::chrono::seconds(30);
// bound of remembered requests, e.g. against changing pseudonyms
const std::size_t max_requests = 256;

} // namespace

TlcStatusApplication::TlcStatusApplication(const FixedTimeIntersection& intersection, Runtime& rt) :
    intersection_(intersection), runtime_(rt), requests_(max_requests)
{
}

void TlcStatusApplication::set_station_id(std::uint32_t station_id)
{
    station_id_ = station_id;
}

void TlcStatusApplication::print_generated_message(bool flag)
{
    print_tx_msg_ = flag;
}

void TlcStatusApplication::print_received_message(bool flag)
{
    print_rx_msg_ = flag;
}

TlcStatusApplication::PortType TlcStatusApplication::port()
{
    return btp::ports::SREM;
}

void TlcStatusApplication::indicate(const DataIndication&, UpPacketPtr packet)
{
    asn1::PacketVisitor<asn1::Srem> visitor;
    std::shared_ptr<const asn1::Srem> srem = boost::apply_visitor(visitor, *packet);

    std::cout << "TLC status application received a packet with " << (srem ? "decodable" : "broken") << " content" << std::endl;
    if (!srem) {
        return;
    }
    if (!is_supported_message(*srem, ItsPduHeader__messageID_srem)) {
        std::cout << "TLC status application ignores an invalid SREM or an unsupported header" << std::endl;
        return;
    }
    if (print_rx_msg_) {
        std::cout << "Received SREM contains\n";
        srem->print();
    }

    std::shared_ptr<asn1::Ssem> message = answer(*srem);
    if (message) {
        // answer outside of the indication to avoid re-entering the router
        runtime_.schedule(Clock::duration::zero(), [this, message](Clock::time_point) { send(message); }, this);
    }
}

std::shared_ptr<asn1::Ssem> TlcStatusApplication::answer(const asn1::Srem& srem)
{
    const SignalRequestMessage_t& srm = srem->srm;
    if (!srm.requests || srm.requestor.id.present != VehicleID_PR_stationID) {
        return nullptr;
    }
    const std::uint32_t requester = srm.requestor.id.choice.stationID;
    const long role = srm.requestor.type ? srm.requestor.type->role : static_cast<long>(BasicVehicleRole_none_unknown);
    const long request_sequence = srm.sequenceNumber ? *srm.sequenceNumber : 0;

    const Clock::time_point now = runtime_.now();
    requests_.expire(now);
    std::vector<PriorityResponse> responses;
    for (int i = 0; i < srm.requests->list.count; ++i) {
        const SignalRequestPackage_t& package = *srm.requests->list.array[i];
        const SignalRequest_t& request = package.request;
        if (request.id.id != intersection_.id()) {
            continue; // addressed to another intersection
        }
        if (request.requestType == PriorityRequestType_priorityCancellation) {
            requests_.cancel(requester, request.requestID);
            continue;
        }

        const bool known_lane = request.inBoundLane.present == IntersectionAccessPoint_PR_lane &&
            intersection_.is_ingress_lane(request.inBoundLane.choice.lane);
        const long result = known_lane ? PrioritizationResponseStatus_processing : PrioritizationResponseStatus_rejected;
        // a request is repeated while it is active, so keep it a while beyond its expected service
        Clock::duration retention = request_retention;
        if (package.duration) {
            retention += std::chrono::milliseconds(*package.duration);
        }
        if (!requests_.update(requester, request.requestID, result, now + retention)) {
            continue; // too many requests, ignore further ones
        }

        PriorityResponse response;
        response.requester = requester;
        response.request_id = request.requestID;
        response.request_sequence = request_sequence;
        response.role = role;
        response.inbound_lane = request.inBoundLane.present == IntersectionAccessPoint_PR_lane ?
            request.inBoundLane.choice.lane : 0;
        if (request.outBoundLane && request.outBoundLane->present == IntersectionAccessPoint_PR_lane) {
            response.outbound_lane = request.outBoundLane->choice.lane;
        }
        response.status = result;
        responses.push_back(response);
    }

    const long sequence_number = requests_.commit();
    if (responses.empty()) {
        return nullptr; // nothing to answer for this intersection
    }

    auto message = std::make_shared<asn1::Ssem>();
    ItsPduHeader_t& header = (*message)->header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_ssem;
    header.stationID = station_id_;
    set_timestamp((*message)->ssm, now);
    SignalStatus& status = add_status((*message)->ssm, intersection_.id(), sequence_number);
    for (const PriorityResponse& response : responses) {
        add_response(status, response);
    }
    return message;
}

void TlcStatusApplication::send(std::shared_ptr<asn1::Ssem> message)
{
    std::string error;
    if (!message->validate(error)) {
        throw std::runtime_error("Invalid SSEM: " + error);
    }
    if (print_tx_msg_) {
        std::cout << "Generated SSEM contains\n";
        message->print();
    }

    DownPacketPtr out { new DownPacket() };
    out->layer(OsiLayer::Application) = std::move(*message);

    DataRequest data_request;
    data_request.its_aid = aid::TLC_S;
    data_request.transport_type = geonet::TransportType::SHB;
    data_request.communication_profile = geonet::CommunicationProfile::ITS_G5;

    auto confirm = Application::request(data_request, std::move(out), btp::ports::SSEM);
    if (!confirm.accepted()) {
        throw std::runtime_error("TLC status application data request failed");
    }
}
