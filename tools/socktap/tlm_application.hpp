#ifndef TLM_APPLICATION_HPP_B8DWQ2LS
#define TLM_APPLICATION_HPP_B8DWQ2LS

#include "application.hpp"
#include "fixed_time_intersection.hpp"
#include <vanetza/common/clock.hpp>
#include <vanetza/common/runtime.hpp>

/**
 * Traffic Light Maneuver service: sends SPATEM of a fixed-time intersection and prints received SPATEMs
 */
class TlmApplication : public Application
{
public:
    TlmApplication(vanetza::Runtime& rt, const FixedTimeIntersection& intersection);
    PortType port() override;
    void indicate(const DataIndication&, UpPacketPtr) override;
    void set_interval(vanetza::Clock::duration); /**< zero disables sending */
    void set_station_id(std::uint32_t station_id);
    void print_received_message(bool flag);
    void print_generated_message(bool flag);

private:
    void schedule_timer();
    void on_timer(vanetza::Clock::time_point);

    vanetza::Runtime& runtime_;
    const FixedTimeIntersection& intersection_;
    vanetza::Clock::duration interval_;
    std::uint32_t station_id_ = 1;
    bool print_rx_msg_ = false;
    bool print_tx_msg_ = false;
};

#endif /* TLM_APPLICATION_HPP_B8DWQ2LS */
