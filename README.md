# vanetza-tlm

Signalised intersection services for [Vanetza](https://github.com/riebl/vanetza):
SPATEM and MAPEM (ETSI Traffic Light Maneuver and Road and Lane Topology services) and
SREM/SSEM (Traffic Light Control service) on top of the existing ETSI TS 103 301 v2.1.1 codecs.

Vanetza already ships `vanetza::asn1::Spatem`, `Mapem`, `Srem` and `Ssem`, the BTP ports and
the ITS-AIDs, but no facility helpers, tests or example applications. This project adds:

- **tests:** UPER round trip, constraint violations and truncated messages for SPATEM/MAPEM,
- **facilities:** SPaT time conversions (MinuteOfTheYear, DSecond, TimeMark with hour rollover
  and "unknown"), SPATEM/MAPEM/SREM/SSEM builders following the C2C-CC RS 2077 profile,
  a fixed-time signal plan model (stages, intergreen times, coordinated plans, weekly schedule)
  and the derivation of protected or permissive movement states from conflicting paths,
- **socktap:** `tlm`, `rlt`, `tlc-request` and `tlc-status` applications around an example
  intersection with motor vehicles, pedestrians and trams,
- **fuzzing:** AFL++ and libFuzzer harnesses for the four message decoders.

Every file lives at the path it will have inside Vanetza, so the tree is a ready-made upstream
patch series.

## Build and test

```sh
git clone --recurse-submodules https://github.com/AlexeyBorovskoy/vanetza-tlm.git
cd vanetza-tlm
scripts/overlay.sh          # Vanetza v26.09 + this overlay -> work/vanetza
cmake -S work/vanetza -B build -DBUILD_TESTS=ON -DBUILD_SOCKTAP=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

`scripts/export-pr.sh` writes the upstream series to `work/pr` (asn1 tests, message helpers,
fuzzing, fixed-time plan model, socktap applications and docs) and fails if it does not
reproduce the overlay.

## Status

The series is proposed to the Vanetza maintainer and is not part of Vanetza. Further work
beyond it is in draft pull requests: semantic validation of MAPEM and SPATEM (#1) and reference
vectors of the example intersection, decoded independently by pycrate (#2).

## Continuous integration

- Vanetza's own container builds and the full test suite on Ubuntu jammy and noble,
- strict C++14 with GCC and Clang, ASan/UBSan, a libFuzzer run,
- two socktap stations exchanging SPATEM/MAPEM/SREM/SSEM over UDP, decoded independently by Wireshark,
- a daily run applying the series to the current upstream master.

## License

GNU LGPL v3, same as Vanetza (see [LICENSE.md](LICENSE.md)).
