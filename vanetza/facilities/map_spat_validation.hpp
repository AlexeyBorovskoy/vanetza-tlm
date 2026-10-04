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
 * Rule identifiers and sources (DSRC = ISO TS 19091 module as published with ETSI TS 103 301,
 * RS = C2C-CC RS 2077 R1.6.2, CR = C-Roads column of RS 2077 Annex 7, project = consistency
 * check without normative text, always a warning):
 *
 * MapData (validate_map)
 * - "ASN.1": constraints of MapData, error
 * - "LaneID unique": LaneID unique within an intersection (DSRC LaneID), error
 * - "LaneID value": LaneID 0 (not known) or 255 (reserved) used for a lane (DSRC LaneID), warning
 * - "connectingLane": target lane of a connection without remoteIntersection exists (DSRC Connection), error
 * - "connectionID": one LaneConnectionID used with different signal groups (project; DSRC allows
 *   shared connection IDs), warning
 * - "crosswalk self-loop": a crosswalk lane connected to itself, a compatibility convention neither
 *   required nor forbidden by ISO TS 19091 (project), warning
 * - RS_ARSM_11: IntersectionReferenceID with region, Car2Car/Combined, error
 * - RS_ARSM_12: IntersectionReferenceID unique within the message, Car2Car/Combined, error
 * - RS_ARSM_14: laneWidth present, Car2Car/Combined, error
 * - RS_ARSM_17: bidirectional crosswalk or bike lane with ingress and egress approach, Car2Car/Combined, error
 * - RS_ARSM_22: exactly one direction bit in connectingLane.maneuver, Car2Car/Combined, error
 * - RS_ARSM_48: signalGroup given for a connection, Car2Car/Combined, warning (whether the
 *   connection is signalised is unknown to the validator)
 * - RS_ARSM_117: no maneuvers on lane level, Car2Car/Combined, error
 *
 * SPAT (validate_spat)
 * - "ASN.1": constraints of SPAT, error
 * - "SignalGroupID unique": one MovementState per signal group (project), warning
 * - "SignalGroupID value": signal group 0 (not known) (DSRC SignalGroupID), warning
 * - "eventState sequence": consecutive events with equal eventState (project), warning
 * - RS_ARSM_11, RS_ARSM_12 for IntersectionState ids, Car2Car/Combined, error
 * - RS_ARSM_56: minEndTime not 36001 (unknown), Car2Car/Combined, error
 * - RS_ARSM_57: maxEndTime present for traffic dependent operation, Car2Car, error
 * - RS_ARSM_60 / RS_ARSM_66: maxEndTime / likelyTime not 36001, Car2Car/Combined, error
 * - RS_ARSM_61: minEndTime, likelyTime and maxEndTime equal for fixed time operation, Car2Car/Combined, error
 * - RS_ARSM_65: minEndTime <= likelyTime <= maxEndTime (hour rule of RS_ARSM_54), Car2Car/Combined, error
 * - RS_ARSM_79: events reach the next phase if the end of the listed ones is known, Car2Car/Combined, error
 * - RS_ARSM_115: confidence present with likelyTime, Car2Car/Combined, error
 * - "maxEndTime": maxEndTime present (CR Annex 7.2), CRoads/Combined, error
 *
 * MapData with SPAT (validate_map_spat), per IntersectionState
 * - RS_ARSM_13: an IntersectionGeometry with the same IntersectionReferenceID exists,
 *   Car2Car/Combined error, otherwise warning
 * - "revision": revision of IntersectionState equals revision of IntersectionGeometry
 *   (DSRC IntersectionState.revision, ISO TS 19091 G.8.2.6), error
 * - RS_ARSM_49: every signalGroup of the geometry's connections has a MovementState,
 *   Car2Car/Combined error, otherwise warning
 * - RS_ARSM_75: every MovementState signalGroup is used by a connection, Car2Car/Combined error,
 *   otherwise warning
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
 * Only the relation is checked, use validate_map() and validate_spat() for the messages themselves.
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
