#include "rlt_application.hpp"
#include "infrastructure_message.hpp"
#include <vanetza/asn1/mapem.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <vanetza/btp/ports.hpp>
#include <vanetza/common/its_aid.hpp>
#include <vanetza/common/position_fix.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>

// This is a very simple RLT application sending MAPEMs of a fixed-time intersection.

using namespace vanetza;
using namespace std::chrono;

RltApplication::RltApplication(PositionProvider& positioning, Runtime& rt, const FixedTimeIntersection& intersection) :
    positioning_(positioning), runtime_(rt), intersection_(intersection), interval_(seconds(1))
{
    schedule_timer();
}

void RltApplication::set_interval(Clock::duration interval)
{
    interval_ = interval;
    runtime_.cancel(this);
    schedule_timer();
}

void RltApplication::set_station_id(std::uint32_t station_id)
{
    station_id_ = station_id;
}

void RltApplication::print_generated_message(bool flag)
{
    print_tx_msg_ = flag;
}

void RltApplication::print_received_message(bool flag)
{
    print_rx_msg_ = flag;
}

RltApplication::PortType RltApplication::port()
{
    return btp::ports::TOPO;
}

void RltApplication::indicate(const DataIndication&, UpPacketPtr packet)
{
    asn1::PacketVisitor<asn1::Mapem> visitor;
    std::shared_ptr<const asn1::Mapem> mapem = boost::apply_visitor(visitor, *packet);

    std::cout << "RLT application received a packet with " << (mapem ? "decodable" : "broken") << " content" << std::endl;
    if (mapem && !is_supported_message(*mapem, ItsPduHeader__messageID_mapem)) {
        std::cout << "RLT application ignores an invalid MAPEM or an unsupported header" << std::endl;
        return;
    }
    if (mapem && print_rx_msg_) {
        std::cout << "Received MAPEM contains\n";
        mapem->print();
    }
}

void RltApplication::schedule_timer()
{
    if (interval_ <= Clock::duration::zero()) {
        return; // receive only
    }
    runtime_.schedule(interval_, std::bind(&RltApplication::on_timer, this, std::placeholders::_1), this);
}

void RltApplication::on_timer(Clock::time_point)
{
    schedule_timer();

    auto position = positioning_.position_fix();
    if (!has_horizontal_position(position)) {
        std::cerr << "Skip MAPEM generation without position fix" << std::endl;
        return;
    }

    asn1::Mapem message;
    ItsPduHeader_t& header = message->header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_mapem;
    header.stationID = station_id_;

    message->map.msgIssueRevision = 0;
    intersection_.fill(message->map, position.latitude, position.longitude);

    std::string error;
    if (!message.validate(error)) {
        throw std::runtime_error("Invalid MAPEM: " + error);
    }

    if (print_tx_msg_) {
        std::cout << "Generated MAPEM contains\n";
        message.print();
    }

    DownPacketPtr packet { new DownPacket() };
    packet->layer(OsiLayer::Application) = std::move(message);

    DataRequest request;
    request.its_aid = aid::RLT;
    request.transport_type = geonet::TransportType::SHB;
    request.communication_profile = geonet::CommunicationProfile::ITS_G5;

    auto confirm = Application::request(request, std::move(packet));
    if (!confirm.accepted()) {
        throw std::runtime_error("RLT application data request failed");
    }
}
