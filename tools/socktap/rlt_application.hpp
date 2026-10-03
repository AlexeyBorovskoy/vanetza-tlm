#ifndef RLT_APPLICATION_HPP_J5PZK9EC
#define RLT_APPLICATION_HPP_J5PZK9EC

#include "application.hpp"
#include "fixed_time_intersection.hpp"
#include <vanetza/common/clock.hpp>
#include <vanetza/common/position_provider.hpp>
#include <vanetza/common/runtime.hpp>

/**
 * Road and Lane Topology service: sends MAPEM of an intersection and prints received MAPEMs
 *
 * The intersection reference point is the station's own position.
 */
class RltApplication : public Application
{
public:
    RltApplication(vanetza::PositionProvider& positioning, vanetza::Runtime& rt, const FixedTimeIntersection& intersection);
    PortType port() override;
    void indicate(const DataIndication&, UpPacketPtr) override;
    void set_interval(vanetza::Clock::duration); /**< zero disables sending */
    void set_station_id(std::uint32_t station_id);
    void print_received_message(bool flag);
    void print_generated_message(bool flag);

private:
    void schedule_timer();
    void on_timer(vanetza::Clock::time_point);

    vanetza::PositionProvider& positioning_;
    vanetza::Runtime& runtime_;
    const FixedTimeIntersection& intersection_;
    vanetza::Clock::duration interval_;
    std::uint32_t station_id_ = 1;
    bool print_rx_msg_ = false;
    bool print_tx_msg_ = false;
};

#endif /* RLT_APPLICATION_HPP_J5PZK9EC */
