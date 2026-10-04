#include <vanetza/asn1/its/IntersectionState.h>
#include <vanetza/asn1/its/IntersectionStatusObject.h>
#include <vanetza/asn1/its/MovementPhaseState.h>
#include <vanetza/asn1/its/SPAT.h>
#include <vanetza/facilities/fixed_time_plan.hpp>
#include <vanetza/facilities/spat_functions.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <algorithm>
#include <chrono>
#include <set>
#include <stdexcept>

namespace vanetza
{
namespace facilities
{

namespace
{

enum class Lamp { Red, RedAmber, Green, GreenBlink, Amber };

struct Event
{
    Clock::duration time;
    Lamp lamp;
};

using Point = std::pair<double, double>;

// TimeIntervalConfidence 15: probability 100 %, a fixed-time plan is deterministic
const long fixed_time_confidence = 15;

// MovementEventList ::= SEQUENCE (SIZE(1..16)) OF MovementEvent
const std::size_t max_movement_events = 16;

// a state outlasting this many plan changes is treated as endless
const std::size_t max_plan_changes = 64;

// all model times are tenths of a second, so known transitions encode exactly
const Clock::duration resolution = std::chrono::milliseconds(100);

const double epsilon = 1e-6;

bool contains(const StageConfig& stage, long group)
{
    return std::find(stage.groups.begin(), stage.groups.end(), group) != stage.groups.end();
}

Clock::duration modulo(Clock::duration value, Clock::duration period)
{
    return (value % period + period) % period;
}

boost::posix_time::ptime local_time(Clock::time_point at, Clock::duration utc_offset)
{
    return Clock::at(at + utc_offset);
}

Clock::duration time_of_day(const boost::posix_time::ptime& t)
{
    return std::chrono::microseconds(t.time_of_day().total_microseconds());
}

int weekday(const boost::posix_time::ptime& t)
{
    return (t.date().day_of_week().as_number() + 6) % 7; // Monday = 0
}

Point meters(const PathPoint& point)
{
    return Point { point.east / units::si::meter, point.north / units::si::meter };
}

double orientation(const Point& o, const Point& a, const Point& b)
{
    return (a.first - o.first) * (b.second - o.second) - (a.second - o.second) * (b.first - o.first);
}

bool within(const Point& p, const Point& q, const Point& r)
{
    // r lies within the bounding box of p and q
    return std::min(p.first, q.first) - epsilon <= r.first && r.first <= std::max(p.first, q.first) + epsilon &&
        std::min(p.second, q.second) - epsilon <= r.second && r.second <= std::max(p.second, q.second) + epsilon;
}

int sign(double value)
{
    return value > epsilon ? 1 : (value < -epsilon ? -1 : 0);
}

bool segments_intersect(const Point& p1, const Point& p2, const Point& p3, const Point& p4)
{
    const int d1 = sign(orientation(p3, p4, p1));
    const int d2 = sign(orientation(p3, p4, p2));
    const int d3 = sign(orientation(p1, p2, p3));
    const int d4 = sign(orientation(p1, p2, p4));
    if (d1 * d2 < 0 && d3 * d4 < 0) {
        return true;
    }
    return (d1 == 0 && within(p3, p4, p1)) || (d2 == 0 && within(p3, p4, p2)) ||
        (d3 == 0 && within(p1, p2, p3)) || (d4 == 0 && within(p1, p2, p4));
}

bool paths_conflict(const MovementPath& a, const MovementPath& b)
{
    const Point a_start = meters(a.points.front());
    const Point b_start = meters(b.points.front());
    if (sign(a_start.first - b_start.first) == 0 && sign(a_start.second - b_start.second) == 0) {
        return false; // same ingress lane
    }
    for (std::size_t i = 1; i < a.points.size(); ++i) {
        for (std::size_t j = 1; j < b.points.size(); ++j) {
            if (segments_intersect(meters(a.points[i - 1]), meters(a.points[i]),
                    meters(b.points[j - 1]), meters(b.points[j]))) {
                return true;
            }
        }
    }
    return false;
}

bool has_path(long group, const std::vector<MovementPath>& paths)
{
    for (const MovementPath& path : paths) {
        if (path.signal_group == group) {
            return true;
        }
    }
    return false;
}

bool groups_conflict(long a, long b, const std::vector<MovementPath>& paths)
{
    for (const MovementPath& pa : paths) {
        if (pa.signal_group != a) {
            continue;
        }
        for (const MovementPath& pb : paths) {
            if (pb.signal_group == b && paths_conflict(pa, pb)) {
                return true;
            }
        }
    }
    return false;
}

// times are bounded by a day, so sums of a few of them cannot overflow
void require_time(Clock::duration value, Clock::duration min, Clock::duration max)
{
    if (value < min || value > max) {
        throw std::invalid_argument("time out of range");
    }
    if (value % resolution != Clock::duration::zero()) {
        throw std::invalid_argument("times have to be multiples of 0.1 s");
    }
}

bool served(const ControllerConfig& config, long group)
{
    return std::any_of(config.stages.begin(), config.stages.end(),
        [group](const StageConfig& stage) { return contains(stage, group); });
}

bool share_stage(const ControllerConfig& config, long a, long b)
{
    return std::any_of(config.stages.begin(), config.stages.end(),
        [a, b](const StageConfig& stage) { return contains(stage, a) && contains(stage, b); });
}

// green starts after the intergreen time measured from the end of the clearing group's green
// blink, so the movements do not overlap if it covers the clearance of the clearing group
bool separated_by_intergreen(const ControllerConfig& config, const SignalGroupConfig& a, const SignalGroupConfig& b)
{
    for (const auto& transition : { std::make_pair(&a, &b), std::make_pair(&b, &a) }) {
        const SignalGroupConfig& clearing = *transition.first;
        const auto igm = config.intergreen.find(std::make_pair(clearing.id, transition.second->id));
        const Clock::duration clearance = clearing.kind == SignalGroupKind::Traffic ? clearing.amber : Clock::duration::zero();
        if (igm == config.intergreen.end() || igm->second < clearance) {
            return false;
        }
    }
    return true;
}

} // namespace

void derive_protection(ControllerConfig& config, const std::vector<MovementPath>& paths)
{
    for (const MovementPath& path : paths) {
        if (path.points.size() < 2) {
            throw std::invalid_argument("movement path needs at least two points");
        }
    }
    for (SignalGroupConfig& group : config.groups) {
        if (!has_path(group.id, paths) || !served(config, group.id)) {
            continue; // nothing known about its movements or it never moves, keep configured states
        }
        bool conflict = false;
        bool unknown = false;
        for (const SignalGroupConfig& other : config.groups) {
            if (other.id == group.id || !served(config, other.id)) {
                continue;
            }
            // groups of different stages move at the same time during insufficient transitions
            const bool overlapping = share_stage(config, group.id, other.id) ||
                !separated_by_intergreen(config, group, other);
            if (!has_path(other.id, paths)) {
                unknown |= overlapping;
            } else if (overlapping && groups_conflict(group.id, other.id, paths)) {
                conflict = true;
            }
        }
        if (unknown && !conflict) {
            continue; // a movement without path may conflict, keep configured states
        }
        group.movement_allowed_state = conflict ?
            MovementPhaseState_permissive_Movement_Allowed : MovementPhaseState_protected_Movement_Allowed;
        group.clearance_state = conflict ?
            MovementPhaseState_permissive_clearance : MovementPhaseState_protected_clearance;
    }
}

FixedTimePlan::FixedTimePlan(ControllerConfig config) : m_config(std::move(config))
{
    const Clock::duration zero = Clock::duration::zero();
    const Clock::duration day = std::chrono::hours(24);

    // MovementList ::= SEQUENCE (SIZE(1..255)) OF MovementState
    if (m_config.groups.empty() || m_config.groups.size() > 255) {
        throw std::invalid_argument("1 to 255 signal groups required");
    }
    std::set<long> ids;
    for (const SignalGroupConfig& sg : m_config.groups) {
        if (sg.id < 0 || sg.id > 255 || !ids.insert(sg.id).second) {
            throw std::invalid_argument("signal group identifiers have to be unique within 0..255");
        }
        require_time(sg.red_amber, zero, day);
        require_time(sg.green_blink, zero, day);
        require_time(sg.amber, zero, day);
    }
    ids.clear();
    for (const StageConfig& stage : m_config.stages) {
        if (!ids.insert(stage.id).second) {
            throw std::invalid_argument("duplicate stage identifier");
        }
        for (long id : stage.groups) {
            group(id);
        }
    }
    for (const auto& item : m_config.intergreen) {
        group(item.first.first);
        group(item.first.second);
        require_time(item.second, zero, day);
    }
    require_time(m_config.utc_offset, -day, day);
    ids.clear();
    for (const PlanConfig& plan : m_config.plans) {
        if (!ids.insert(plan.id).second) {
            throw std::invalid_argument("duplicate plan identifier");
        }
        if (plan.sequence.empty()) {
            throw std::invalid_argument("plan without stages");
        }
        Clock::duration length = zero;
        for (const auto& item : plan.sequence) {
            stage(item.first);
            require_time(item.second, resolution, day);
            length += item.second;
            if (length > day) {
                throw std::invalid_argument("cycle longer than a day");
            }
        }
        require_time(plan.offset, -day, day);
        if (day % length != zero) {
            throw std::invalid_argument("cycle length does not divide 24 hours");
        }
    }
    bool scheduled = false;
    for (auto& program : m_config.week) {
        std::sort(program.begin(), program.end(),
            [](const ScheduleEntry& a, const ScheduleEntry& b) { return a.time_of_day < b.time_of_day; });
        for (std::size_t k = 0; k < program.size(); ++k) {
            const ScheduleEntry& entry = program[k];
            require_time(entry.time_of_day, zero, day - resolution);
            if (k > 0 && program[k - 1].time_of_day == entry.time_of_day) {
                throw std::invalid_argument("two plans scheduled at the same time");
            }
            scheduled = true;
            plan(entry.plan);
        }
    }
    if (!scheduled) {
        throw std::invalid_argument("empty schedule");
    }
    // the cycles are computed once, every transition has to fit into its stage
    for (const PlanConfig& plan : m_config.plans) {
        for (const SignalGroupConfig& sg : m_config.groups) {
            m_cycles[std::make_pair(plan.id, sg.id)] = compute_cycle(plan, sg.id);
        }
    }
}

const SignalGroupConfig& FixedTimePlan::group(long id) const
{
    for (const SignalGroupConfig& g : m_config.groups) {
        if (g.id == id) {
            return g;
        }
    }
    throw std::invalid_argument("unknown signal group");
}

const StageConfig& FixedTimePlan::stage(long id) const
{
    for (const StageConfig& s : m_config.stages) {
        if (s.id == id) {
            return s;
        }
    }
    throw std::invalid_argument("unknown stage");
}

const PlanConfig& FixedTimePlan::plan(long id) const
{
    for (const PlanConfig& p : m_config.plans) {
        if (p.id == id) {
            return p;
        }
    }
    throw std::invalid_argument("unknown plan");
}

Clock::duration FixedTimePlan::cycle_length(const PlanConfig& plan) const
{
    Clock::duration length = Clock::duration::zero();
    for (const auto& item : plan.sequence) {
        length += item.second;
    }
    return length;
}

const PlanConfig& FixedTimePlan::active_plan(Clock::time_point at) const
{
    const boost::posix_time::ptime local = local_time(at, m_config.utc_offset);
    const Clock::duration now = time_of_day(local);
    const int today = weekday(local);

    // latest switch at or before now, looking back to previous days if necessary
    for (int back = 0; back < 8; ++back) {
        const auto& day = m_config.week[(today + 7 - back) % 7];
        for (auto it = day.rbegin(); it != day.rend(); ++it) {
            if (back > 0 || it->time_of_day <= now) {
                return plan(it->plan);
            }
        }
    }
    throw std::logic_error("no plan scheduled");
}

boost::optional<Clock::time_point> FixedTimePlan::next_plan_change(Clock::time_point now) const
{
    const long current = active_plan(now).id;
    const boost::posix_time::ptime local = local_time(now, m_config.utc_offset);
    const Clock::time_point local_midnight = now - time_of_day(local);
    const int today = weekday(local);

    for (int ahead = 0; ahead < 8; ++ahead) {
        for (const ScheduleEntry& entry : m_config.week[(today + ahead) % 7]) {
            const Clock::time_point at = local_midnight + std::chrono::hours(24 * ahead) + entry.time_of_day;
            if (at > now && entry.plan != current) {
                return at;
            }
        }
    }
    return boost::none;
}

boost::optional<Clock::time_point> FixedTimePlan::last_plan_change(Clock::time_point now) const
{
    const long current = active_plan(now).id;
    const boost::posix_time::ptime local = local_time(now, m_config.utc_offset);
    const Clock::time_point local_midnight = now - time_of_day(local);
    const int today = weekday(local);

    // walk back over the switches selecting the current plan until another plan was selected
    boost::optional<Clock::time_point> change;
    for (int back = 0; back < 8; ++back) {
        const auto& day = m_config.week[(today + 7 - back) % 7];
        for (auto it = day.rbegin(); it != day.rend(); ++it) {
            const Clock::time_point at = local_midnight - std::chrono::hours(24 * back) + it->time_of_day;
            if (at > now) {
                continue;
            }
            if (it->plan != current) {
                return change;
            }
            change = at;
        }
    }
    return boost::none; // no other plan within a week
}

const std::vector<SignalInterval>& FixedTimePlan::cycle(const PlanConfig& plan, long id) const
{
    const auto found = m_cycles.find(std::make_pair(plan.id, id));
    if (found == m_cycles.end()) {
        throw std::invalid_argument("unknown plan or signal group");
    }
    return found->second;
}

std::vector<SignalInterval> FixedTimePlan::compute_cycle(const PlanConfig& plan, long id) const
{
    const SignalGroupConfig& sg = group(id);
    const bool traffic = sg.kind == SignalGroupKind::Traffic;
    const Clock::duration red_amber = traffic ? sg.red_amber : Clock::duration::zero();
    const Clock::duration amber = traffic ? sg.amber : Clock::duration::zero();
    const Clock::duration length = cycle_length(plan);
    const std::size_t count = plan.sequence.size();

    std::vector<Event> events;
    Clock::duration boundary = Clock::duration::zero();
    for (std::size_t k = 0; k < count; ++k) {
        const StageConfig& previous = stage(plan.sequence[(k + count - 1) % count].first);
        const StageConfig& next = stage(plan.sequence[k].first);
        const Clock::duration duration = plan.sequence[k].second;

        // begin of green for every group entering the next stage, relative to the boundary
        std::map<long, Clock::duration> green_start;
        for (long entering : next.groups) {
            if (contains(previous, entering)) {
                continue;
            }
            const SignalGroupConfig& e = group(entering);
            Clock::duration start = e.kind == SignalGroupKind::Traffic ? e.red_amber : Clock::duration::zero();
            for (long clearing : previous.groups) {
                auto igm = m_config.intergreen.find(std::make_pair(clearing, entering));
                if (igm != m_config.intergreen.end() && !contains(next, clearing)) {
                    start = std::max(start, igm->second + group(clearing).green_blink);
                }
            }
            if (start >= duration) {
                throw std::invalid_argument("stage is shorter than the transition into it");
            }
            green_start[entering] = start;
        }

        if (contains(next, id) && !contains(previous, id)) {
            const Clock::duration green = boundary + green_start[id];
            if (red_amber > Clock::duration::zero()) {
                events.push_back({ green - red_amber, Lamp::RedAmber });
            }
            events.push_back({ green, Lamp::Green });
        } else if (contains(previous, id) && !contains(next, id)) {
            // keep green as long as the intergreen times to all entering groups allow
            Clock::duration blink_end = sg.green_blink;
            bool constrained = false;
            Clock::duration latest = Clock::duration::zero();
            for (const auto& entering : green_start) {
                auto igm = m_config.intergreen.find(std::make_pair(id, entering.first));
                if (igm != m_config.intergreen.end()) {
                    const Clock::duration limit = entering.second - igm->second;
                    latest = constrained ? std::min(latest, limit) : limit;
                    constrained = true;
                }
            }
            if (constrained) {
                blink_end = std::max(blink_end, latest);
            }
            if (blink_end + amber >= duration) {
                throw std::invalid_argument("stage is shorter than the transition out of the previous one");
            }
            events.push_back({ boundary + blink_end - sg.green_blink, Lamp::GreenBlink });
            if (amber > Clock::duration::zero()) {
                events.push_back({ boundary + blink_end, Lamp::Amber });
            }
            events.push_back({ boundary + blink_end + amber, Lamp::Red });
        }
        boundary += duration;
    }

    auto phase_state = [&sg](Lamp lamp) -> long {
        switch (lamp) {
            case Lamp::RedAmber:
                return MovementPhaseState_pre_Movement;
            case Lamp::Green:
                return sg.movement_allowed_state;
            case Lamp::GreenBlink:
                // flashing green still permits movement where it is used, e.g. before amber;
                // there is no flashing green MovementPhaseState, its end is announced by the timing only
                return sg.movement_allowed_state;
            case Lamp::Amber:
                return sg.clearance_state;
            case Lamp::Red:
            default:
                return MovementPhaseState_stop_And_Remain;
        }
    };

    std::vector<SignalInterval> intervals;
    if (events.empty()) {
        // group is either green or red in every stage
        const bool always_green = contains(stage(plan.sequence.front().first), id);
        intervals.push_back({ phase_state(always_green ? Lamp::Green : Lamp::Red), Clock::duration::zero(), length });
        return intervals;
    }

    for (Event& event : events) {
        event.time = modulo(event.time, length);
    }
    std::stable_sort(events.begin(), events.end(), [](const Event& a, const Event& b) { return a.time < b.time; });

    long state = phase_state(events.back().lamp); // active at cycle begin
    Clock::duration begin = Clock::duration::zero();
    for (const Event& event : events) {
        const long next_state = phase_state(event.lamp);
        if (next_state == state) {
            continue;
        }
        if (event.time > begin) {
            intervals.push_back({ state, begin, event.time });
        }
        state = next_state;
        begin = event.time;
    }
    intervals.push_back({ state, begin, length });
    return intervals;
}

FixedTimePlan::PlanState FixedTimePlan::plan_state(long group, Clock::time_point at) const
{
    const PlanConfig& plan = active_plan(at);
    const std::vector<SignalInterval>& intervals = cycle(plan, group);
    PlanState result { intervals.front().phase_state, boost::none, boost::none };
    if (intervals.size() == 1) {
        return result;
    }

    const Clock::duration length = cycle_length(plan);
    const Clock::duration position = modulo(time_of_day(local_time(at, m_config.utc_offset)) - plan.offset, length);
    std::size_t index = 0;
    while (index + 1 < intervals.size() && intervals[index].end <= position) {
        ++index;
    }
    // cycle() splits a state at the cycle boundary, join it again
    const bool joined = intervals.front().phase_state == intervals.back().phase_state;
    Clock::duration begin = intervals[index].begin;
    Clock::duration end = intervals[index].end;
    if (joined && index == 0) {
        begin = intervals.back().begin - length;
    }
    if (joined && index + 1 == intervals.size()) {
        end = length + intervals.front().end;
    }
    result.phase_state = intervals[index].phase_state;
    result.begin = at - (position - begin);
    result.end = at + (end - position);
    return result;
}

FixedTimePlan::StateSpan FixedTimePlan::state_span(long group, Clock::time_point at) const
{
    const long state = plan_state(group, at).phase_state;
    Clock::time_point cursor = at;
    for (std::size_t changes = 0; changes < max_plan_changes; ++changes) {
        const PlanState current = plan_state(group, cursor);
        if (current.phase_state != state) {
            return { state, cursor }; // ended by the plan change
        }
        const boost::optional<Clock::time_point> change = next_plan_change(cursor);
        if (current.end && (!change || *current.end < *change)) {
            return { state, current.end };
        }
        if (!change) {
            break;
        }
        cursor = *change; // the state lasts at least until the plan change
    }
    return { state, boost::none };
}

boost::optional<Clock::time_point> FixedTimePlan::next_movement(const SignalGroupConfig& sg, Clock::time_point at) const
{
    Clock::time_point cursor = at;
    for (std::size_t step = 0; step < max_movement_events; ++step) {
        const StateSpan span = state_span(sg.id, cursor);
        if (step > 0 && span.phase_state == sg.movement_allowed_state) {
            return cursor;
        }
        if (!span.end) {
            break;
        }
        cursor = *span.end;
    }
    return boost::none;
}

void FixedTimePlan::fill(SPAT& spat, long intersection_id, long revision, Clock::time_point now) const
{
    const boost::optional<Clock::time_point> plan_begin = last_plan_change(now);
    // TimeMarks are unambiguous for one hour from the begin of the current minute
    const Clock::time_point horizon = now - std::chrono::milliseconds(dsecond(now)) -
        now.time_since_epoch() % std::chrono::milliseconds(1) + std::chrono::hours(1);

    IntersectionState& intersection = add_intersection_state(spat, intersection_id, revision);
    const int status_bit = IntersectionStatusObject_fixedTimeOperation;
    intersection.status.buf[status_bit / 8] |= 0x80 >> (status_bit % 8);
    set_timestamp(intersection, now);

    for (const SignalGroupConfig& sg : m_config.groups) {
        MovementState& movement = add_movement(intersection, sg.id);
        boost::optional<Clock::time_point> begin = plan_state(sg.id, now).begin;
        if (begin && plan_begin && *begin < *plan_begin) {
            begin = boost::none; // the state began before the current plan took effect
        }

        // current state and the following states up to the next phase (C2C-CC RS 2077 RS_ARSM_79)
        Clock::time_point at = now;
        for (std::size_t events = 0; events < max_movement_events; ++events) {
            const StateSpan span = state_span(sg.id, at);
            MovementTiming timing;
            timing.start = begin;
            if (span.end) {
                timing.min_end = *span.end;
                timing.max_end = *span.end;
                timing.likely = *span.end;
                timing.confidence = fixed_time_confidence;
            } else {
                timing.min_end = horizon; // endless, sent as cTimeMarkOutOfRange
            }
            if (events == 0 && span.phase_state != sg.movement_allowed_state) {
                timing.next = next_movement(sg, now);
            }
            set_timing(add_event(movement, span.phase_state), timing, now, m_config.timing);

            const bool phase = span.phase_state == sg.movement_allowed_state ||
                span.phase_state == MovementPhaseState_stop_And_Remain;
            if (!span.end || (phase && events > 0)) {
                break;
            }
            begin = span.end;
            at = *span.end;
        }
    }
}

} // namespace facilities
} // namespace vanetza
