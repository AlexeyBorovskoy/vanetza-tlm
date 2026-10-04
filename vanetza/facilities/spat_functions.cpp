#include <vanetza/asn1/its/SPAT.h>
#include <vanetza/asn1/its/IntersectionState.h>
#include <vanetza/asn1/its/MovementEvent.h>
#include <vanetza/asn1/its/MovementState.h>
#include <vanetza/asn1/its/TimeChangeDetails.h>
#include <vanetza/facilities/spat_functions.hpp>
#include <vanetza/facilities/detail/asn1_list.tpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <array>
#include <chrono>
#include <stdexcept>

namespace vanetza
{
namespace facilities
{

const long cMinuteOfTheYearUnknown = 527040;
const long cTimeMarkOutOfRange = 36000;
const long cTimeMarkUnknown = 36001;
const long cMsgCountMax = 127;

namespace
{

using boost::posix_time::ptime;
using boost::posix_time::time_duration;
using detail::allocate_optional;
using detail::append;

time_duration since_hour(const ptime& t)
{
    const time_duration tod = t.time_of_day();
    return tod - boost::posix_time::hours(tod.hours());
}

// round up to tenths of a second, Clock's epoch starts at a full second
Clock::time_point ceil_to_tenth(const Clock::time_point& at)
{
    const Clock::duration excess = at.time_since_epoch() % std::chrono::milliseconds(100);
    return excess == Clock::duration::zero() ? at : at - excess + std::chrono::milliseconds(100);
}

Clock::time_point minute_begin(const Clock::time_point& at)
{
    return at - std::chrono::milliseconds(dsecond(at)) - at.time_since_epoch() % std::chrono::milliseconds(1);
}

struct TaiUtcOffset
{
    const char* utc_since;
    long seconds;
};

// TAI - UTC since the 2004 epoch of Clock (IERS Bulletin C), extend when a leap second is announced
const std::array<TaiUtcOffset, 6> tai_utc_offsets {{
    { "2004-01-01 00:00:00", 32 },
    { "2006-01-01 00:00:00", 33 },
    { "2009-01-01 00:00:00", 34 },
    { "2012-07-01 00:00:00", 35 },
    { "2015-07-01 00:00:00", 36 },
    { "2017-01-01 00:00:00", 37 },
}};

} // namespace

Clock::time_point utc_from_tai(const Clock::time_point& tai)
{
    const ptime tai_time = Clock::at(tai);
    long offset = tai_utc_offsets.front().seconds;
    for (const TaiUtcOffset& entry : tai_utc_offsets) {
        const ptime switch_in_tai = boost::posix_time::time_from_string(entry.utc_since) +
            boost::posix_time::seconds(entry.seconds);
        if (tai_time >= switch_in_tai) {
            offset = entry.seconds;
        }
    }
    return tai - std::chrono::seconds(offset);
}

long minute_of_the_year(const Clock::time_point& at)
{
    const ptime t = Clock::at(at);
    const ptime year_begin { boost::gregorian::date(t.date().year(), 1, 1) };
    return static_cast<long>((t - year_begin).total_seconds() / 60);
}

long dsecond(const Clock::time_point& at)
{
    const time_duration tod = Clock::at(at).time_of_day();
    const time_duration since_minute = tod - boost::posix_time::hours(tod.hours()) - boost::posix_time::minutes(tod.minutes());
    return static_cast<long>(since_minute.total_milliseconds());
}

long time_mark(const Clock::time_point& at)
{
    return static_cast<long>(since_hour(Clock::at(at)).total_milliseconds() / 100);
}

boost::optional<Clock::time_point> time_mark_to_time_point(long mark, const Clock::time_point& reference)
{
    if (mark < 0 || mark >= cTimeMarkOutOfRange) {
        return boost::none;
    }

    const ptime ref = Clock::at(reference);
    const time_duration in_hour = since_hour(ref);
    const ptime hour_begin = ref - in_hour;
    ptime resolved = hour_begin + boost::posix_time::milliseconds(mark * 100);

    // marks before the begin of the reference minute belong to the following hour
    const long minute_begin = static_cast<long>(in_hour.minutes()) * 600;
    if (mark < minute_begin) {
        resolved += boost::posix_time::hours(1);
    }
    return Clock::at(resolved);
}

long next_msg_count(long count)
{
    return count >= cMsgCountMax || count < 0 ? 0 : count + 1;
}

IntersectionState& add_intersection_state(SPAT& spat, long id, long revision)
{
    auto intersection = asn1::make_unique<IntersectionState_t>(asn_DEF_IntersectionState);
    intersection->id.id = id;
    intersection->revision = revision;
    // IntersectionStatusObject ::= BIT STRING (SIZE(16)), no flag set
    intersection->status.size = 2;
    intersection->status.buf = static_cast<uint8_t*>(asn1::allocate(intersection->status.size));
    intersection->status.bits_unused = 0;
    return append(spat.intersections, std::move(intersection));
}

void set_timestamp(IntersectionState& intersection, const Clock::time_point& at)
{
    *allocate_optional(intersection.moy) = minute_of_the_year(at);
    *allocate_optional(intersection.timeStamp) = dsecond(at);
}

MovementState& add_movement(IntersectionState& intersection, long signal_group)
{
    auto movement = asn1::make_unique<MovementState_t>(asn_DEF_MovementState);
    movement->signalGroup = signal_group;
    return append(intersection.states, std::move(movement));
}

MovementEvent& add_event(MovementState& movement, long phase_state)
{
    auto event = asn1::make_unique<MovementEvent_t>(asn_DEF_MovementEvent);
    event->eventState = phase_state;
    return append(movement.state_time_speed, std::move(event));
}

void set_timing(MovementEvent& event, const MovementTiming& timing, const Clock::time_point& timestamp)
{
    set_timing(event, timing, timestamp, TimingOptions());
}

void set_timing(MovementEvent& event, const MovementTiming& timing, const Clock::time_point& timestamp,
        const TimingOptions& options)
{
    const Clock::time_point window_begin = minute_begin(timestamp);
    const Clock::time_point window_end = window_begin + std::chrono::hours(1);
    const auto representable = [&](const Clock::time_point& t) { return t >= window_begin && t < window_end; };
    if (timing.min_end < window_begin) {
        throw std::invalid_argument("minimum end time before the minute of the timestamp");
    }
    if (timing.likely && !timing.confidence) {
        throw std::invalid_argument("likely time without confidence");
    }

    ASN_STRUCT_FREE(asn_DEF_TimeChangeDetails, event.timing);
    event.timing = nullptr;
    TimeChangeDetails_t* details = allocate_optional(event.timing);
    details->minEndTime = timing.min_end < window_end ? time_mark(timing.min_end) : cTimeMarkOutOfRange;
    if (timing.start && representable(*timing.start)) {
        *allocate_optional(details->startTime) = time_mark(*timing.start);
    }
    if (timing.max_end && representable(ceil_to_tenth(*timing.max_end))) {
        *allocate_optional(details->maxEndTime) = time_mark(ceil_to_tenth(*timing.max_end));
    } else if (options.always_max_end) {
        *allocate_optional(details->maxEndTime) = cTimeMarkOutOfRange; // unknown or beyond the window
    }
    if (timing.likely && representable(*timing.likely)) {
        *allocate_optional(details->likelyTime) = time_mark(*timing.likely);
        if (timing.confidence) {
            *allocate_optional(details->confidence) = *timing.confidence;
        }
    }
    if (timing.next && representable(*timing.next)) {
        *allocate_optional(details->nextTime) = time_mark(*timing.next);
    }
}

} // namespace facilities
} // namespace vanetza
