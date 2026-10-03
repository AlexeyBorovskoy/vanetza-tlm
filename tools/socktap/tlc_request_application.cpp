#include "tlc_request_application.hpp"
#include "infrastructure_message.hpp"
#include <vanetza/asn1/its/IntersectionAccessPoint.h>
#include <vanetza/asn1/its/SignalRequesterInfo.h>
#include <vanetza/asn1/its/SignalStatus.h>
#include <vanetza/asn1/its/SignalStatusPackage.h>
#include <vanetza/asn1/packet_visitor.hpp>
#include <vanetza/asn1/srem.hpp>
#include <vanetza/asn1/ssem.hpp>
#include <vanetza/btp/ports.hpp>
#include <vanetza/common/its_aid.hpp>
#include <vanetza/facilities/priority_functions.hpp>
#include <vanetza/facilities/spat_functions.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>

// This is a very simple TLC requesting application of a tram asking for priority.

using namespace vanetza;
using namespace vanetza::facilities;
using namespace std::chrono;

namespace
{

// westbound tram track of the example intersection and its egress track
const long tram_ingress_lane = 2;
const long tram_egress_lane = 10;
const long request_id = 1;

} // namespace

TlcRequestApplication::TlcRequestApplication(Runtime& rt, long intersection_id) :
    runtime_(rt), intersection_id_(intersection_id), interval_(seconds(1))
{
    schedule_timer();
}

void TlcRequestApplication::set_interval(Clock::duration interval)
{
    interval_ = interval;
    runtime_.cancel(this);
    schedule_timer();
}

void TlcRequestApplication::set_station_id(std::uint32_t station_id)
{
    station_id_ = station_id;
}

void TlcRequestApplication::print_generated_message(bool flag)
{
    print_tx_msg_ = flag;
}

void TlcRequestApplication::print_received_message(bool flag)
{
    print_rx_msg_ = flag;
}

TlcRequestApplication::PortType TlcRequestApplication::port()
{
    return btp::ports::SSEM;
}

void TlcRequestApplication::indicate(const DataIndication&, UpPacketPtr packet)
{
    asn1::PacketVisitor<asn1::Ssem> visitor;
    std::shared_ptr<const asn1::Ssem> ssem = boost::apply_visitor(visitor, *packet);

    std::cout << "TLC request application received a packet with " << (ssem ? "decodable" : "broken") << " content" << std::endl;
    if (!ssem) {
        return;
    }
    if (!is_supported_message(*ssem, ItsPduHeader__messageID_ssem)) {
        std::cout << "TLC request application ignores an invalid SSEM or an unsupported header" << std::endl;
        return;
    }

    for (int i = 0; i < (*ssem)->ssm.status.list.count; ++i) {
        const SignalStatus_t& status = *(*ssem)->ssm.status.list.array[i];
        for (int j = 0; j < status.sigStatus.list.count; ++j) {
            const SignalStatusPackage_t& package = *status.sigStatus.list.array[j];
            if (package.requester && package.requester->id.present == VehicleID_PR_stationID &&
                package.requester->id.choice.stationID == station_id_) {
                std::cout << "Priority request " << package.requester->request << " at intersection "
                    << status.id.id << " has status " << package.status << std::endl;
            }
        }
    }
    if (print_rx_msg_) {
        std::cout << "Received SSEM contains\n";
        ssem->print();
    }
}

void TlcRequestApplication::schedule_timer()
{
    if (interval_ <= Clock::duration::zero()) {
        return; // receive only
    }
    runtime_.schedule(interval_, std::bind(&TlcRequestApplication::on_timer, this, std::placeholders::_1), this);
}

void TlcRequestApplication::on_timer(Clock::time_point)
{
    schedule_timer();
    const Clock::time_point now = runtime_.now();

    asn1::Srem message;
    ItsPduHeader_t& header = message->header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_srem;
    header.stationID = station_id_;

    set_timestamp(message->srm, now);
    message->srm.sequenceNumber = asn1::allocate<MsgCount_t>();
    *message->srm.sequenceNumber = sequence_number_;
    set_requestor(message->srm, station_id_, BasicVehicleRole_publicTransport);

    PriorityRequest request;
    request.intersection_id = intersection_id_;
    request.request_id = request_id;
    request.request_type = first_request_ ?
        PriorityRequestType_priorityRequest : PriorityRequestType_priorityRequestUpdate;
    first_request_ = false;
    request.inbound_lane = tram_ingress_lane;
    request.outbound_lane = tram_egress_lane;
    request.arrival = now + seconds(20);
    request.duration = seconds(8);
    add_request(message->srm, request);
    sequence_number_ = next_msg_count(sequence_number_);

    std::string error;
    if (!message.validate(error)) {
        throw std::runtime_error("Invalid SREM: " + error);
    }

    if (print_tx_msg_) {
        std::cout << "Generated SREM contains\n";
        message.print();
    }

    DownPacketPtr packet { new DownPacket() };
    packet->layer(OsiLayer::Application) = std::move(message);

    DataRequest data_request;
    data_request.its_aid = aid::TLC_R;
    data_request.transport_type = geonet::TransportType::SHB;
    data_request.communication_profile = geonet::CommunicationProfile::ITS_G5;

    auto confirm = Application::request(data_request, std::move(packet), btp::ports::SREM);
    if (!confirm.accepted()) {
        throw std::runtime_error("TLC request application data request failed");
    }
}
