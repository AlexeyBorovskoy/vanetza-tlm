#ifndef TLC_REQUEST_APPLICATION_HPP_D5XQ2HNA
#define TLC_REQUEST_APPLICATION_HPP_D5XQ2HNA

#include "application.hpp"
#include <vanetza/common/clock.hpp>
#include <vanetza/common/runtime.hpp>

/**
 * Traffic Light Control requesting service of a tram: sends SREM asking for priority
 * at the example intersection and prints the received SSEM answers
 */
class TlcRequestApplication : public Application
{
public:
    TlcRequestApplication(vanetza::Runtime& rt, long intersection_id);
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
    long intersection_id_;
    vanetza::Clock::duration interval_;
    std::uint32_t station_id_ = 1;
    long sequence_number_ = 0;
    bool first_request_ = true;
    bool print_rx_msg_ = false;
    bool print_tx_msg_ = false;
};

#endif /* TLC_REQUEST_APPLICATION_HPP_D5XQ2HNA */
