#ifndef MAP_SPAT_VALIDATION_HPP_H5TR9KQB
#define MAP_SPAT_VALIDATION_HPP_H5TR9KQB

#include <vanetza/facilities/validation.hpp>

// forward declaration of asn1c generated struct
struct MapData;
struct SPAT;

namespace vanetza
{
namespace facilities
{

/**
 * Semantic checks of MAPEM and SPATEM content beyond ASN.1 constraints.
 *
 * Sources: DSRC = ISO TS 19091 module with the profile comments published with ETSI TS 103 301,
 * RS = C2C-CC RS 2077 R1.6.2, Annex = profile columns of RS 2077 Annex 7 (7.1 MapData, 7.2 SPAT).
 * Rules of RS are reported only with ValidationProfile Car2Car or Combined, rules of the C-Roads
 * column only with CRoads or Combined. Checks without normative text ("project") are warnings in
 * every profile and never use an RS identifier.
 *
 * A message violating its ASN.1 constraints is reported by rule "ASN.1" only; the semantic
 * checks need a structurally valid message and are skipped. SEQUENCE OF sizes and empty list
 * elements, which asn1c constraint checking does not cover, are reported as "ASN.1" as well.
 *
 * Coverage is partial: requirements on geometry (node offsets, lane widths, distances),
 * transmission (rates, revision changes over time) and checks needing more than one message
 * besides a MapData/SPAT pair are not covered.
 *
 * MapData (validate_map)
 * - "ASN.1": constraints, error
 * - "msgIssueRevision": not 0 (DSRC MapData profile comment), error
 * - "layerType": present (DSRC MapData profile comment: shall not be used), error
 * - "LaneID unique": duplicate LaneID within an intersection (DSRC LaneID), error
 * - "LaneID value": a lane with LaneID 0 (not known) or 255 (reserved) (DSRC LaneID), warning
 * - "connectingLane": target lane of a local connection does not exist, error; warning for a
 *   MapData fragment (layerID present), whose target may be in another fragment; target LaneID 0
 *   (not known) or 255 (reserved) is a warning instead (DSRC LaneID, Connection, MapData.layerID)
 * - "connectionID": one LaneConnectionID with different signal groups (project; DSRC allows
 *   shared connection IDs), warning
 * - "crosswalk self-loop": crosswalk connected to itself, a compatibility convention neither
 *   required nor forbidden by ISO TS 19091 (project), warning
 * - Car2Car/Combined, errors unless noted:
 *   RS_ARSM_11 region present; RS_ARSM_12 IntersectionReferenceID used twice within the message
 *   (the 5 km uniqueness radius is not checkable); RS_ARSM_14 laneWidth present;
 *   RS_ARSM_16 unidirectional lane with exactly one of ingressApproach and egressApproach;
 *   RS_ARSM_17 bidirectional crosswalk or bike lane with both approaches; RS_ARSM_20 no duplicate
 *   connection to the same lane with the same direction; RS_ARSM_21 connectingLane.maneuver present;
 *   RS_ARSM_22 exactly one direction bit; RS_ARSM_24 no turn-on-red or lane change bit;
 *   RS_ARSM_35 at most 18 nodes per lane; RS_ARSM_48 signalGroup present (warning, the validator
 *   cannot know whether the connection is signalised); RS_ARSM_117 no lane level maneuvers
 * - Car2Car, CRoads, Combined: RS_ARSM_118 nodeList uses nodes, not computed
 * - CRoads/Combined: "C-Roads intersections" present, "C-Roads connectionID" present (Annex 7.1)
 *
 * SPAT (validate_spat)
 * - "ASN.1": constraints, error
 * - "SignalGroupID unique": a second MovementState for a signal group (project), warning
 * - "SignalGroupID value": signal group 0 (not known) (DSRC SignalGroupID), warning
 * - "SignalGroupID 255": signal group 255 denotes a permanent green movement state, an event of
 *   it other than permissive or protected Movement-Allowed (DSRC SignalGroupID), error
 * - "eventState sequence": consecutive events with equal eventState (project), warning
 * - Car2Car/Combined, errors: RS_ARSM_11, RS_ARSM_12 for IntersectionState ids;
 *   RS_ARSM_56 minEndTime not 36001; RS_ARSM_57 maxEndTime and RS_ARSM_64 likelyTime present for
 *   traffic dependent operation; RS_ARSM_60 maxEndTime and RS_ARSM_66 likelyTime not 36001;
 *   RS_ARSM_61 present end times equal for fixed time operation, a maxEndTime of 36000 meaning
 *   the time of change is not known; RS_ARSM_65 minEndTime <= likelyTime <= maxEndTime (an
 *   informative statement of RS, implied by the meaning of the times); RS_ARSM_69 only status
 *   bits 5 to 9; RS_ARSM_70 exactly one of them; RS_ARSM_72 eventState dark not used;
 *   RS_ARSM_78 events sorted by minEndTime; RS_ARSM_79 events reach the next phase, a phase being
 *   MovementPhaseState 2, 3, 5 or 6 (RS_ARSM_95); not reported if the last listed event ends
 *   beyond the TimeMark horizon (minEndTime 36000), because no later event can be timed then;
 *   RS_ARSM_115 confidence with likelyTime; RS_ARSM_120 timing present for every event preceding
 *   a phase event
 * - Car2Car, CRoads, Combined: "moy" present (Annex 7.2)
 * - CRoads/Combined: "C-Roads maxEndTime" and "C-Roads timeStamp" present (Annex 7.2)
 *
 * TimeMarks are compared after resolving them relative to the minute given by moy
 * (RS_ARSM_54): a TimeMark before that minute belongs to the next hour, 36000 lies beyond all
 * others, 36001 is not compared.
 *
 * MapData with SPAT (validate_map_spat)
 * - "revision": revision of IntersectionState differs from its IntersectionGeometry
 *   (DSRC IntersectionState.revision, ISO TS 19091 G.8.2.6), error
 * - Car2Car/Combined errors, otherwise the project warnings named in brackets:
 *   RS_ARSM_68 ("map/spat intersection") an IntersectionGeometry exists for every
 *   IntersectionState; RS_ARSM_13 ("map/spat intersection") an IntersectionState exists for
 *   every IntersectionGeometry; RS_ARSM_49 ("map/spat signal group") every signal group of
 *   a connection has a MovementState; RS_ARSM_75 ("map/spat signal group") every MovementState
 *   signal group is used by a connection; RS_ARSM_68 and RS_ARSM_75 are not checked for a
 *   MapData fragment (layerID present), whose other fragments may describe the rest
 *
 * Paths start with the name of the validated type, e.g. "SPAT.intersections[0].states[2]".
 */

/**
 * Check a MapData
 * \param map decoded or built MapData
 * \param profile requirements checked in addition to the standards
 * \return issues found
 */
ValidationResult validate_map(const MapData& map, ValidationProfile profile = ValidationProfile::Standard);

/**
 * Check a SPAT
 * \param spat decoded or built SPAT
 * \param profile requirements checked in addition to the standards
 * \return issues found
 */
ValidationResult validate_spat(const SPAT& spat, ValidationProfile profile = ValidationProfile::Standard);

/**
 * Check that a SPAT matches the MapData describing its intersections.
 * Only the relation is checked, use validate_map() and validate_spat() for the messages themselves;
 * nothing is checked if one of them violates its ASN.1 constraints.
 * \param map MapData of the intersections
 * \param spat SPAT of the same intersections
 * \param profile requirements checked in addition to the standards
 * \return issues found
 */
ValidationResult validate_map_spat(const MapData& map, const SPAT& spat,
        ValidationProfile profile = ValidationProfile::Standard);

} // namespace facilities
} // namespace vanetza

#endif /* MAP_SPAT_VALIDATION_HPP_H5TR9KQB */
