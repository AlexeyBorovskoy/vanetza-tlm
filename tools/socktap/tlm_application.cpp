#include "tlm_application.hpp"
#include "infrastructure_message.hpp"
#include <vanetza/asn1/packet_visitor.hpp>
#include <vanetza/asn1/spatem.hpp>
#include <vanetza/btp/ports.hpp>
#include <vanetza/common/its_aid.hpp>
#include <vanetza/facilities/spat_functions.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>

// This is a very simple TLM application sending SPATEMs of a fixed-time intersection.

using namespace vanetza;
using namespace std::chrono;

TlmApplication::TlmApplication(Runtime& rt, const FixedTimeIntersection& intersection) :
    runtime_(rt), intersection_(intersection), interval_(milliseconds(100))
{
    schedule_timer();
}

void TlmApplication::set_interval(Clock::duration interval)
{
    interval_ = interval;
    runtime_.cancel(this);
    schedule_timer();
}

void TlmApplication::set_station_id(std::uint32_t station_id)
{
    station_id_ = station_id;
}

void TlmApplication::print_generated_message(bool flag)
{
    print_tx_msg_ = flag;
}

void TlmApplication::print_received_message(bool flag)
{
    print_rx_msg_ = flag;
}

TlmApplication::PortType TlmApplication::port()
{
    return btp::ports::SPAT;
}

void TlmApplication::indicate(const DataIndication&, UpPacketPtr packet)
{
    asn1::PacketVisitor<asn1::Spatem> visitor;
    std::shared_ptr<const asn1::Spatem> spatem = boost::apply_visitor(visitor, *packet);

    std::cout << "TLM application received a packet with " << (spatem ? "decodable" : "broken") << " content" << std::endl;
    if (spatem && !is_supported_message(*spatem, ItsPduHeader__messageID_spatem)) {
        std::cout << "TLM application ignores an invalid SPATEM or an unsupported header" << std::endl;
        return;
    }
    if (spatem && print_rx_msg_) {
        std::cout << "Received SPATEM contains\n";
        spatem->print();
    }
}

void TlmApplication::schedule_timer()
{
    if (interval_ <= Clock::duration::zero()) {
        return; // receive only
    }
    runtime_.schedule(interval_, std::bind(&TlmApplication::on_timer, this, std::placeholders::_1), this);
}

void TlmApplication::on_timer(Clock::time_point)
{
    schedule_timer();
    asn1::Spatem message;

    ItsPduHeader_t& header = message->header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_spatem;
    header.stationID = station_id_;

    intersection_.fill(message->spat, runtime_.now());

    std::string error;
    if (!message.validate(error)) {
        throw std::runtime_error("Invalid SPATEM: " + error);
    }

    if (print_tx_msg_) {
        std::cout << "Generated SPATEM contains\n";
        message.print();
    }

    DownPacketPtr packet { new DownPacket() };
    packet->layer(OsiLayer::Application) = std::move(message);

    DataRequest request;
    request.its_aid = aid::TLM;
    request.transport_type = geonet::TransportType::SHB;
    request.communication_profile = geonet::CommunicationProfile::ITS_G5;

    auto confirm = Application::request(request, std::move(packet));
    if (!confirm.accepted()) {
        throw std::runtime_error("TLM application data request failed");
    }
}
