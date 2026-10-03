#ifndef TLC_STATUS_APPLICATION_HPP_R8KW3MZV
#define TLC_STATUS_APPLICATION_HPP_R8KW3MZV

#include "application.hpp"
#include "fixed_time_intersection.hpp"
#include "priority_request_table.hpp"
#include <vanetza/asn1/srem.hpp>
#include <vanetza/asn1/ssem.hpp>
#include <vanetza/common/runtime.hpp>
#include <memory>

/**
 * Traffic Light Control status service of the example intersection: answers SREM by SSEM.
 *
 * The fixed-time example controller has no priority logic, hence valid requests are
 * answered with status processing and requests for unknown lanes with status rejected.
 */
class TlcStatusApplication : public Application
{
public:
    explicit TlcStatusApplication(const FixedTimeIntersection& intersection, vanetza::Runtime& rt);
    PortType port() override;
    void indicate(const DataIndication&, UpPacketPtr) override;
    void set_station_id(std::uint32_t station_id);
    void print_received_message(bool flag);
    void print_generated_message(bool flag);

private:
    std::shared_ptr<vanetza::asn1::Ssem> answer(const vanetza::asn1::Srem&);
    void send(std::shared_ptr<vanetza::asn1::Ssem>);

    const FixedTimeIntersection& intersection_;
    vanetza::Runtime& runtime_;
    std::uint32_t station_id_ = 1;
    PriorityRequestTable requests_;
    bool print_rx_msg_ = false;
    bool print_tx_msg_ = false;
};

#endif /* TLC_STATUS_APPLICATION_HPP_R8KW3MZV */
