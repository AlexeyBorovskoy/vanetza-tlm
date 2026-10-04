#ifndef FIXED_TIME_PLAN_HPP_Q7CMV4TB
#define FIXED_TIME_PLAN_HPP_Q7CMV4TB

#include <vanetza/common/clock.hpp>
#include <vanetza/units/length.hpp>
#include <boost/optional/optional.hpp>
#include <array>
#include <map>
#include <utility>
#include <vector>

struct SPAT;

namespace vanetza
{
namespace facilities
{

/**
 * Fixed-time traffic signal controller model producing SPaT.
 *
 * A plan is a sequence of stages, each listing the signal groups with green. At the begin of
 * a stage the groups entering it show red-amber and then green; their green starts after
 * the intergreen time to every clearing group, measured from the end of the clearing group's
 * green blink. Groups leaving the stage keep green as long as the intergreen times allow and
 * then show green blink, amber and red. Groups green in both stages stay green. Transitions
 * consume the duration of the next stage, so the cycle length is the sum of stage durations.
 *
 * Plans are coordinated: the first stage starts at local midnight plus the plan offset,
 * modulo the cycle length, which therefore has to divide 24 hours. A plan change from the
 * weekly schedule takes effect immediately at its scheduled time, without a transition; this
 * simplification keeps the model deterministic, so predictions follow the schedule.
 */

/** Kind of road user served by a signal group */
enum class SignalGroupKind
{
    Traffic, /**< motor vehicles: red-amber, green, green blink, amber, red */
    Pedestrian, /**< pedestrians: green, green blink, red */
    Tram /**< trams: green, green blink, red */
};

/** Signal group of the controller */
struct SignalGroupConfig
{
    long id; /**< SignalGroupID (0..255) */
    SignalGroupKind kind; /**< served road users */
    Clock::duration red_amber; /**< red-amber before green, ignored for non-traffic groups */
    Clock::duration green_blink; /**< green blink before the end of green */
    Clock::duration amber; /**< amber after green, ignored for non-traffic groups */
    long movement_allowed_state; /**< MovementPhaseState during green and green blink, e.g. permissive (5) */
    long clearance_state; /**< MovementPhaseState during amber, e.g. permissive clearance (7) */
};

/** Stage (phase) of a plan */
struct StageConfig
{
    long id; /**< stage identifier used by plans */
    std::vector<long> groups; /**< signal groups with green during this stage */
};

/** Fixed-time plan */
struct PlanConfig
{
    long id; /**< plan identifier used by the schedule */
    std::vector<std::pair<long, Clock::duration>> sequence; /**< stage identifier and stage duration */
    Clock::duration offset; /**< begin of the first stage after local midnight */
};

/** Plan switch within a day program */
struct ScheduleEntry
{
    Clock::duration time_of_day; /**< local time of the switch */
    long plan; /**< plan identifier active from this time on */
};

/** Complete controller configuration */
struct ControllerConfig
{
    std::vector<SignalGroupConfig> groups; /**< all signal groups */
    std::vector<StageConfig> stages; /**< all stages */
    std::map<std::pair<long, long>, Clock::duration> intergreen; /**< (clearing, entering) group identifiers */
    std::vector<PlanConfig> plans; /**< all plans */
    std::array<std::vector<ScheduleEntry>, 7> week; /**< day programs, Monday first */
    Clock::duration utc_offset; /**< local time minus UTC */
};

/** One signal state of a group within a cycle */
struct SignalInterval
{
    long phase_state; /**< MovementPhaseState */
    Clock::duration begin; /**< relative to cycle start */
    Clock::duration end; /**< relative to cycle start */
};

/** Point of a movement path in a local frame */
struct PathPoint
{
    units::Length east; /**< offset towards east */
    units::Length north; /**< offset towards north */
};

/** Path of one signalled movement, e.g. from stop line via the connected lane outwards */
struct MovementPath
{
    long signal_group; /**< controlling signal group */
    std::vector<PathPoint> points; /**< polyline, at least two points */
};

/**
 * Derive whether signal groups show protected or permissive movement states.
 *
 * A group is protected if none of its paths crosses or merges into a path of a group that
 * may move at the same time, otherwise it is permissive (C2C-CC RS 2077,
 * RS_ARSM_105/106/107/110). Groups with green in a common stage move at the same time.
 * Groups of different stages are separated only if the intergreen times in both directions
 * cover the clearance (amber) of the clearing group, since the model starts green after the
 * intergreen time measured from the end of the clearing group's green blink; otherwise their
 * movements overlap during the transition.
 * Paths starting at the same point (same ingress lane) do not conflict. Groups in no stage
 * never move. Groups without any path keep their configured states, and so does a group
 * without a known conflict if a group that may move at the same time has no path.
 *
 * \param config controller configuration whose movement and clearance states are set
 * \param paths movement paths of signal groups
 */
void derive_protection(ControllerConfig& config, const std::vector<MovementPath>& paths);

class FixedTimePlan
{
public:
    /**
     * All times have to be multiples of 0.1 s, the resolution of SPaT, and at most one day.
     * \param config controller configuration
     * \throw std::invalid_argument if the configuration is inconsistent, e.g. no or more than
     * 255 signal groups, duplicate identifiers, a negative or too long time, a time off the
     * 0.1 s grid, a stage shorter than its transition, a cycle not dividing 24 hours or a
     * schedule time outside of the day
     */
    explicit FixedTimePlan(ControllerConfig config);

    /**
     * Select the plan active at given time according to the weekly schedule
     * \param at UTC based time point
     * \return active plan
     */
    const PlanConfig& active_plan(Clock::time_point at) const;

    /**
     * Get the signal states of a group over one cycle of a plan
     * \param plan plan of this controller
     * \param group signal group id
     * \return consecutive intervals covering exactly one cycle, starting at the first stage
     * \throw std::invalid_argument if plan or group is not part of this controller
     */
    const std::vector<SignalInterval>& cycle(const PlanConfig& plan, long group) const;

    /**
     * Append the intersection state at given time to SPAT
     *
     * Each signal group gets its current state followed by the states up to its next phase,
     * e.g. green, amber and red (C2C-CC RS 2077 RS_ARSM_79), following plan changes of the
     * schedule. Times beyond the TimeMark horizon of one hour are encoded by set_timing();
     * a state without end gets minimum end time cTimeMarkOutOfRange. Start times before the
     * activation of the current plan are omitted.
     *
     * \param spat destination
     * \param intersection_id IntersectionID
     * \param revision MsgCount, equal to the MAPEM revision
     * \param now UTC based current time
     */
    void fill(SPAT& spat, long intersection_id, long revision, Clock::time_point now) const;

private:
    /** State of a group within the plan active at a time point */
    struct PlanState
    {
        long phase_state; /**< MovementPhaseState */
        boost::optional<Clock::time_point> begin; /**< none if the state lasts the whole cycle */
        boost::optional<Clock::time_point> end; /**< none if the state lasts the whole cycle */
    };

    /** State of a group at a time point and its end across plan changes, none if endless */
    struct StateSpan
    {
        long phase_state; /**< MovementPhaseState */
        boost::optional<Clock::time_point> end;
    };

    const SignalGroupConfig& group(long id) const;
    const StageConfig& stage(long id) const;
    const PlanConfig& plan(long id) const;
    Clock::duration cycle_length(const PlanConfig&) const;
    std::vector<SignalInterval> compute_cycle(const PlanConfig& plan, long group) const;
    boost::optional<Clock::time_point> next_plan_change(Clock::time_point now) const;
    boost::optional<Clock::time_point> last_plan_change(Clock::time_point now) const;
    PlanState plan_state(long group, Clock::time_point at) const;
    StateSpan state_span(long group, Clock::time_point at) const;
    boost::optional<Clock::time_point> next_movement(const SignalGroupConfig&, Clock::time_point at) const;

    ControllerConfig m_config;
    std::map<std::pair<long, long>, std::vector<SignalInterval>> m_cycles; /**< by plan and group */
};

} // namespace facilities
} // namespace vanetza

#endif /* FIXED_TIME_PLAN_HPP_Q7CMV4TB */
