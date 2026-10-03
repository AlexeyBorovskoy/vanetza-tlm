Title: Generating SPATEM and MAPEM

# Generating SPATEM and MAPEM

Vanetza ships the ASN.1 codecs for SPATEM and MAPEM (ETSI TS 103 301 v2.1.1) as `vanetza::asn1::Spatem` and `vanetza::asn1::Mapem`.
The helper functions in `vanetza/facilities/spat_functions.hpp` and `vanetza/facilities/map_functions.hpp` fill these messages without dealing with asn1c's plain C structures directly.

## Signal phase and timing

    :::cpp
    #include <vanetza/asn1/spatem.hpp>
    #include <vanetza/facilities/spat_functions.hpp>

    using namespace vanetza;
    using namespace vanetza::facilities;

    asn1::Spatem spatem;
    spatem->header.protocolVersion = 2;
    spatem->header.messageID = ItsPduHeader__messageID_spatem;
    spatem->header.stationID = station_id;

    IntersectionState& intersection = add_intersection_state(spatem->spat, intersection_id, revision);
    set_timestamp(intersection, now);

    MovementEvent& event = add_event(add_movement(intersection, signal_group),
        MovementPhaseState_protected_Movement_Allowed);
    MovementTiming timing;
    timing.min_end = green_end_min;
    timing.max_end = green_end_max;
    timing.likely = green_end_likely;
    timing.confidence = 12; // TimeIntervalConfidence of the likely time, required with it
    set_timing(event, timing, now);

    ByteBuffer buffer = spatem.encode(); // UPER

`MovementTiming` takes absolute `Clock::time_point` values.
They are converted to *TimeMark*, i.e. tenths of a second within the current UTC hour.
Lower bounds are truncated and `max_end` is rounded up, so the encoded interval always contains the given interval.
A TimeMark is unambiguous only for one hour from the begin of the timestamp's minute: a later minimum end time is sent as `cTimeMarkOutOfRange` (36000, C2C-CC RS 2077), later optional times are omitted.
Each call of `set_timing()` replaces the previous timing of the event.
`time_mark_to_time_point()` resolves a received TimeMark following C2C-CC RS 2077 (RS_ARSM_54): the mark belongs to the hour of the reference time, or to the next hour if it lies before the reference minute, so forecasts crossing the hour boundary resolve correctly.

## Intersection topology

    :::cpp
    #include <vanetza/asn1/mapem.hpp>
    #include <vanetza/facilities/map_functions.hpp>

    asn1::Mapem mapem;
    IntersectionGeometry& intersection = add_intersection_geometry(mapem->map, intersection_id, revision,
        latitude, longitude, 3.5 * units::si::meter);
    GenericLane& lane = add_vehicle_lane(intersection, 1, TravelDirection::Ingress, approach);
    add_node(lane, 2.0 * units::si::meter, -15.0 * units::si::meter);
    add_node(lane, 0.0 * units::si::meter, -20.0 * units::si::meter);
    connect(lane, 3, Maneuver::Left, signal_group, connection_id);

`add_node()` picks the smallest *Node-XY* representation holding the offset with centimetre resolution.
The first node of an ingress lane is its stop line.
`add_tram_lane()` and `add_crosswalk_lane()` add light rail tracks and pedestrian crosswalks.
The builders follow the C2C-CC RS 2077 profile: a reference lane width per intersection, exactly one manoeuvre and a connection identifier per connection, and no manoeuvres on lane level.

## Pitfalls

- There is no flashing green MovementPhaseState. Where flashing green still permits movement (e.g. Russian traffic rules), keep the movement allowed state (5 or 6) and announce its end by the timing; never encode the dark half of a flash as `dark`.

- A crosswalk modelled as one bidirectional lane has both approach identifiers and, to carry its pedestrian signal group, a connection to itself. This is a compatibility convention used in deployed MAPs; ISO TS 19091 neither requires nor forbids it. Split crosswalks with refuge islands are better modelled as separate lanes connecting to each other.

- The builders leave the road regulator identifier (`IntersectionReferenceID.region`) empty. C2C-CC RS 2077 requires it together with the intersection identifier, so set the same value in MAPEM and SPATEM where the profile applies.
- *MsgCount* (revision counters) is limited to 0..127. Use `next_msg_count()` for wrapping, an out-of-range value makes UPER encoding throw.
- `validate()` checks value constraints but not the size constraints of `SEQUENCE OF` types. An empty movement or lane list passes validation but cannot be encoded.

See also the *tlm* and *rlt* applications of [socktap](../tools/socktap.md).
