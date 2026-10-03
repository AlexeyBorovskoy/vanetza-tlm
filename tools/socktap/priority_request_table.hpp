#ifndef PRIORITY_REQUEST_TABLE_HPP_M6RJ2KPZ
#define PRIORITY_REQUEST_TABLE_HPP_M6RJ2KPZ

#include <vanetza/common/clock.hpp>
#include <cstddef>
#include <cstdint>
#include <map>
#include <utility>

/**
 * Priority requests known to the TLC status application.
 *
 * A request is identified by the station ID of its requester and its request ID. It is
 * forgotten when it is cancelled or not repeated before it expires, and the table holds a
 * bounded number of requests. The SSEM sequence number changes whenever a request is added,
 * changes its status or is forgotten.
 */
class PriorityRequestTable
{
public:
    /**
     * \param capacity maximum number of remembered requests
     */
    explicit PriorityRequestTable(std::size_t capacity);

    /**
     * Forget requests expired before given time
     * \param now current time
     */
    void expire(vanetza::Clock::time_point now);

    /**
     * Remember a request or refresh a known one
     * \param requester station ID of the requesting vehicle
     * \param request_id RequestID
     * \param status PrioritizationResponseStatus of the answer
     * \param expiry time the request is forgotten unless repeated
     * \return false if the request is unknown and the table is full
     */
    bool update(std::uint32_t requester, long request_id, long status, vanetza::Clock::time_point expiry);

    /**
     * Forget a cancelled request
     * \param requester station ID of the requesting vehicle
     * \param request_id RequestID
     */
    void cancel(std::uint32_t requester, long request_id);

    /**
     * Complete the changes caused by one SREM
     * \return SSEM sequence number (MsgCount), the next one if anything changed since the last call
     */
    long commit();

    /**
     * \return number of remembered requests
     */
    std::size_t size() const;

private:
    struct Entry
    {
        long status;
        vanetza::Clock::time_point expiry;
    };

    std::size_t capacity_;
    std::map<std::pair<std::uint32_t, long>, Entry> entries_;
    long sequence_number_ = 0;
    bool changed_ = false;
};

#endif /* PRIORITY_REQUEST_TABLE_HPP_M6RJ2KPZ */
