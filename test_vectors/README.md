# Reference vectors

`intersection_01/` holds messages of the socktap example intersection (`FixedTimeIntersection`:
17 lanes, 18 connections, signal groups 1-4 motor vehicles, 5-7 pedestrians, 8-9 trams) at fixed
instants, for comparing other SPATEM/MAPEM/SREM/SSEM implementations with Vanetza:

| Vector | Content |
|---|---|
| `mapem` | intersection geometry, region 1, intersection ID 1, revision 0 |
| `spatem_cycle_00s` ... `spatem_cycle_70s` | one 80 s cycle of plan 2 in steps of 10 s, Tuesday 2026-10-06 from 10:00:00 UTC |
| `spatem_hour_boundary` | 10:59:55, events ending in the next hour (TimeMark wraps) |
| `spatem_plan_change` | 06:59:50, plan 1 changing to plan 4 at 07:00 |
| `srem_tram_request` | priority request of the tram (station 20) from track 2 to track 10 |
| `ssem_tram_processing` | answer of the intersection (station 10): request in processing |

Every vector is stored as `<name>.uper` (unaligned PER, as sent) and `<name>.xer` (basic XER,
for reading); `manifest.json` lists name, message type, UTC instant, size and description.

## How the vectors are made and checked

- `reference-vectors <directory>` (built with socktap, `tools/socktap/reference_vectors.cpp`)
  writes the files. The CI job `reference-vectors` regenerates them and fails if they differ from
  the committed ones.
- `scripts/check-vectors.py` decodes every UPER file with [pycrate](https://github.com/pycrate-org/pycrate),
  an ASN.1 implementation independent of asn1c, from the ASN.1 modules Vanetza uses
  (ETSI TS 103 301 v2.1.1, ETSI TS 102 894-2 v1.3.1, ISO TS 19091). It checks that encoding the
  decoded value gives the same bytes and compares every element with the XER written by asn1c.
- The GTest `ReferenceVectors` pins the encoding and checks the messages with the MAPEM/SPATEM
  validation: the SPATEMs meet the Standard, Car2Car, CRoads and Combined profiles alone and
  together with the MAPEM; the MAPEM has only the three crosswalk self-loop warnings.
