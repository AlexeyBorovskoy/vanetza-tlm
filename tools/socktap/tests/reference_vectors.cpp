#include <gtest/gtest.h>
#include "reference_vectors.hpp"
#include <vanetza/asn1/mapem.hpp>
#include <vanetza/asn1/spatem.hpp>
#include <vanetza/asn1/srem.hpp>
#include <vanetza/asn1/ssem.hpp>
#include <vanetza/facilities/map_spat_validation.hpp>
#include <vanetza/facilities/validation.hpp>
#include <algorithm>
#include <cstdint>
#include <array>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

using namespace vanetza;
using namespace vanetza::facilities;

namespace
{

const std::array<ValidationProfile, 4> all_profiles {
    ValidationProfile::Standard, ValidationProfile::Car2Car, ValidationProfile::CRoads, ValidationProfile::Combined
};

// UPER encodings of the reference vectors: a change of the encoding has to be deliberate
const std::map<std::string, std::string> golden_uper {
    { "mapem",
        "02050000000a08000110001000100295b1244390d1b94015e1024011400000000596141a4ea42000358720001022c308"
        "000815611000040cb0980002072402140031000000b2c22bc9d4840000b0a800080e100312000000003fa0f993b58800"
        "040104800c4000001fd0d449dac4000480a480000000083e125483e440000b0e20001012406240000000041f0be641f2"
        "2000158620000812c0e000040c901c9000c40000020f875120f9100002c12000243c40208800000000818ed6412a2000"
        "10092200000000206306904a8800040288800c40000010312bc82544000481668000000009d47c20900060d04b0e8000"
        "40b581a0002062c2080010344030c800000001457e5812000cb009035100000000145787c120003e6096190000610b03"
        "20003095842000185080721000000002751350240006a01a07913000400000045107cc380060d0161f0000a20682066c"
        "00100000013516bc075e400058840003089a08a2300040000004d445101d7900016230000e24" },
    { "spatem_cycle_00s",
        "02040000000a00188000800080040061e1800000800124570005000500057a3f8002800a000a000a3d0fc0050026c026"
        "c026de00448ae000a000a000af47f00050014001400147a1f800a004d804d804dbc00c90de002800280029e005089e00"
        "14002800280028f45f00140087008700877802121bc005000500053c00a113c0028005000500051e8be0028010e010e0"
        "10ef00514378014001400147801422f800a004380438043bc018515c001400140015e87e000a014a014a014af0071457"
        "0005000500057a1f8002805280528052bc020515c001400140015e87e000a014a014a014af00914570005000500057a1"
        "f8002805280528052bc0" },
    { "spatem_cycle_10s",
        "02040000000a00188000800080040061e18271008001243f8014009b009b009b780a5227804d805280528052bd17c029"
        "4065406540655e004487f0028013601360136f014a44f009b00a500a500a57a2f805280ca80ca80cabc00c917c005002"
        "1c021c021de8fe010e012c012c012cf43f0096019a019a019a7802122f800a004380438043bd1fc021c025802580259e"
        "87e012c033403340334f005145f00140087008700877a1f804380d200d200d23c01850fe0014029402940295e02948be"
        "014a032a032a032af007143f800500a500a500a5780a522f805280ca80ca80cabc02050fe0014029402940295e02948b"
        "e014a032a032a032af009143f800500a500a500a5780a522f805280ca80ca80cabc0" },
    { "spatem_cycle_20s",
        "02040000000a00188000800080040061e184e2008001243f8014009b009b009b780a5227804d805280528052bd17c029"
        "4065406540655e004487f0028013601360136f014a44f009b00a500a500a57a2f805280ca80ca80cabc00c917c005002"
        "1c021c021de8fe010e012c012c012cf43f0096019a019a019a7802122f800a004380438043bd1fc021c025802580259e"
        "87e012c033403340334f005145f00140087008700877a1f804380d200d200d23c01850fe0014029402940295e02948be"
        "014a032a032a032af007143f800500a500a500a5780a522f805280ca80ca80cabc02050fe0014029402940295e02948b"
        "e014a032a032a032af009143f800500a500a500a5780a522f805280ca80ca80cabc0" },
    { "spatem_cycle_30s",
        "02040000000a00188000800080040061e18753008001243f8014009b009b009b780a5227804d805280528052bd17c029"
        "4065406540655e004487f0028013601360136f014a44f009b00a500a500a57a2f805280ca80ca80cabc00c90fe025806"
        "6806680669e069089e0334034803480348f45f01a40217021702177802121fc04b00cd00cd00cd3c0d2113c066806900"
        "6900691e8be0348042e042e042ef005143f808701a401a401a4781a422f80d2010b810b810bbc01850fe001402940294"
        "0295e02948be014a032a032a032af007143f800500a500a500a5780a522f805280ca80ca80cabc02050fe00140294029"
        "40295e02948be014a032a032a032af009143f800500a500a500a5780a522f805280ca80ca80cabc0" },
    { "spatem_cycle_40s",
        "02040000000a00188000800080040061e189c4008001245f00a50195019501957a3f80ca80d200d200d23d0fc069008a"
        "c08ac08ade00448be014a032a032a032af47f019501a401a401a47a1f80d2011581158115bc00c90fe02580668066806"
        "69e069089e0334034803480348f45f01a40217021702177802121fc04b00cd00cd00cd3c0d2113c0668069006900691e"
        "8be0348042e042e042ef005143f808701a401a401a4781a422f80d2010b810b810bbc018517c0294065406540655e87e"
        "032a046a046a046af007145f00a50195019501957a1f80ca811a811a811abc020517c0294065406540655e87e032a046"
        "a046a046af009145f00a50195019501957a1f80ca811a811a811abc0" },
    { "spatem_cycle_50s",
        "02040000000a00188000800080040061e18c35008001245f00a50195019501957a3f80ca80d200d200d23d0fc069008a"
        "c08ac08ade00448be014a032a032a032af47f019501a401a401a47a1f80d2011581158115bc00c90fe02580668066806"
        "69e069089e0334034803480348f45f01a40217021702177802121fc04b00cd00cd00cd3c0d2113c0668069006900691e"
        "8be0348042e042e042ef005143f808701a401a401a4781a422f80d2010b810b810bbc018517c0294065406540655e87e"
        "032a046a046a046af007145f00a50195019501957a1f80ca811a811a811abc020517c0294065406540655e87e032a046"
        "a046a046af009145f00a50195019501957a1f80ca811a811a811abc0" },
    { "spatem_cycle_60s",
        "02040000000a00188000800080040061e1900000800124570195019501957a3f80ca80d200d200d23d0fc069008ac08a"
        "c08ade00448ae032a032a032af47f019501a401a401a47a1f80d2011581158115bc00c90de066806680669e069089e03"
        "34034803480348f45f01a40217021702177802121bc0cd00cd00cd3c0d2113c0668069006900691e8be0348042e042e0"
        "42ef005143781a401a401a4781a422f80d2010b810b810bbc018515c065406540655e87e032a046a046a046af0071457"
        "0195019501957a1f80ca811a811a811abc020515c065406540655e87e032a046a046a046af00914570195019501957a1"
        "f80ca811a811a811abc0" },
    { "spatem_cycle_70s",
        "02040000000a00188000800080040061e1927100800124570195019501957a3f80ca80d200d200d23d0fc069008ac08a"
        "c08ade00448ae032a032a032af47f019501a401a401a47a1f80d2011581158115bc00c90de066806680669e069089e03"
        "34034803480348f45f01a40217021702177802121bc0cd00cd00cd3c0d2113c0668069006900691e8be0348042e042e0"
        "42ef005143781a401a401a4781a422f80d2010b810b810bbc018515c065406540655e87e032a046a046a046af0071457"
        "0195019501957a1f80ca811a811a811abc020515c065406540655e87e032a046a046a046af00914570195019501957a1"
        "f80ca811a811a811abc0" },
    { "spatem_hour_boundary",
        "02040000000a00188000800080040061e53d6d808001245f45650005000500057a3f8002800a000a000a3d0fc0050026"
        "c026c026de00448be8aca000a000a000af47f00050014001400147a1f800a004d804d804dbc00c90ff15580028002800"
        "29e005089e0014002800280028f45f00140087008700877802121fe2ab0005000500053c00a113c0028005000500051e"
        "8be0028010e010e010ef005143fc5470014001400147801422f800a004380438043bc018517d1594001400140015e87e"
        "000a014a014a014af007145f45650005000500057a1f8002805280528052bc020517d1594001400140015e87e000a014"
        "a014a014af009145f45650005000500057a1f8002805280528052bc0" },
    { "spatem_plan_change",
        "02040000000a00188000800080040061d63c35008001143fc5dd0000000000007800022f8000009880988098bc00850f"
        "f1774000000000001e00008be0000026202620262f003145f45dd0000000000007a1f8000009b009b009b3c010517d17"
        "74000000000001e87e0000026c026c026cf005145f45dd0000000000007a1f800000a000a000a03c01850ff173800000"
        "0000001e00008be0000026202620262f007143fc5ce0000000000007800022f8000009880988098bc02050ff17380000"
        "00000001e00008be0000026202620262f009143fc5ce0000000000007800022f8000009880988098bc" },
    { "srem_tram_request",
        "020900000014730f0c0000000390000404802014c3c309c403e80808000000a00080" },
    { "ssem_tram_processing",
        "020a0000000a461e18006400040002060a00000028020010040288" },
};

std::string hex(const ByteBuffer& buffer)
{
    static const char digits[] = "0123456789abcdef";
    std::string text;
    for (std::uint8_t byte : buffer) {
        text += digits[byte >> 4];
        text += digits[byte & 0x0f];
    }
    return text;
}

using Finding = std::tuple<std::string, std::string, bool>; // rule, path, error

std::multiset<Finding> findings(const ValidationResult& result)
{
    std::multiset<Finding> found;
    for (const ValidationIssue& issue : result.issues()) {
        found.emplace(issue.rule, issue.path, issue.severity == Severity::Error);
    }
    return found;
}

template<typename MESSAGE>
MESSAGE decoded(const ReferenceVector& vector)
{
    MESSAGE message;
    if (!message.decode(vector.uper)) {
        ADD_FAILURE() << vector.name << " does not decode";
    }
    return message;
}

std::string self_loop(int lane_index)
{
    return "MapData.intersections[0].laneSet[" + std::to_string(lane_index) + "].connectsTo[0].connectingLane.lane";
}

const ReferenceVector& find(const std::vector<ReferenceVector>& vectors, const std::string& name)
{
    auto found = std::find_if(vectors.begin(), vectors.end(),
        [&name](const ReferenceVector& vector) { return vector.name == name; });
    if (found == vectors.end()) {
        throw std::logic_error("no reference vector " + name);
    }
    return *found;
}

} // namespace

TEST(ReferenceVectors, names_are_unique_and_messages_known)
{
    const std::vector<ReferenceVector> vectors = reference_vectors();
    ASSERT_EQ(13u, vectors.size()); // MAPEM, 10 SPATEMs, SREM, SSEM
    std::set<std::string> names;
    for (const ReferenceVector& vector : vectors) {
        EXPECT_TRUE(names.insert(vector.name).second) << vector.name;
        EXPECT_TRUE(vector.message == "mapem" || vector.message == "spatem" || vector.message == "srem" ||
            vector.message == "ssem") << vector.name;
        EXPECT_FALSE(vector.uper.empty()) << vector.name;
        EXPECT_FALSE(vector.xer.empty()) << vector.name;
        EXPECT_EQ(std::string::npos, vector.description.find('"')) << vector.name;
    }
}

TEST(ReferenceVectors, building_is_deterministic)
{
    const std::vector<ReferenceVector> first = reference_vectors();
    const std::vector<ReferenceVector> second = reference_vectors();
    ASSERT_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        EXPECT_EQ(first[i].uper, second[i].uper) << first[i].name;
        EXPECT_EQ(first[i].xer, second[i].xer) << first[i].name;
    }
}

TEST(ReferenceVectors, encodings_are_pinned)
{
    const std::vector<ReferenceVector> vectors = reference_vectors();
    ASSERT_EQ(golden_uper.size(), vectors.size());
    for (const ReferenceVector& vector : vectors) {
        const auto golden = golden_uper.find(vector.name);
        ASSERT_NE(golden_uper.end(), golden) << vector.name;
        EXPECT_EQ(golden->second, hex(vector.uper)) << vector.name;
    }
}

TEST(ReferenceVectors, decoding_and_encoding_again_yields_the_same_bytes)
{
    for (const ReferenceVector& vector : reference_vectors()) {
        ByteBuffer again;
        if (vector.message == "mapem") {
            again = decoded<asn1::Mapem>(vector).encode();
        } else if (vector.message == "spatem") {
            again = decoded<asn1::Spatem>(vector).encode();
        } else if (vector.message == "srem") {
            again = decoded<asn1::Srem>(vector).encode();
        } else {
            again = decoded<asn1::Ssem>(vector).encode();
        }
        EXPECT_EQ(vector.uper, again) << vector.name;
    }
}

TEST(ReferenceVectors, map_has_only_the_crosswalk_convention_warnings_in_every_profile)
{
    // crosswalks 15, 16 and 17 (laneSet[14..16]) connect to themselves to carry their signal group
    const std::multiset<Finding> expected {
        Finding { "crosswalk self-loop", self_loop(14), false },
        Finding { "crosswalk self-loop", self_loop(15), false },
        Finding { "crosswalk self-loop", self_loop(16), false },
    };
    const asn1::Mapem mapem = decoded<asn1::Mapem>(find(reference_vectors(), "mapem"));
    for (ValidationProfile profile : all_profiles) {
        EXPECT_EQ(expected, findings(validate_map(mapem->map, profile)));
    }
}

TEST(ReferenceVectors, spat_meets_every_profile_alone_and_with_the_map)
{
    const std::vector<ReferenceVector> vectors = reference_vectors();
    const asn1::Mapem mapem = decoded<asn1::Mapem>(find(vectors, "mapem"));
    for (const ReferenceVector& vector : vectors) {
        if (vector.message != "spatem") {
            continue;
        }
        const asn1::Spatem spatem = decoded<asn1::Spatem>(vector);
        for (ValidationProfile profile : all_profiles) {
            EXPECT_TRUE(findings(validate_spat(spatem->spat, profile)).empty()) << vector.name;
            EXPECT_TRUE(findings(validate_map_spat(mapem->map, spatem->spat, profile)).empty()) << vector.name;
        }
    }
}

TEST(ReferenceVectors, request_and_status_meet_their_asn1_types)
{
    const std::vector<ReferenceVector> vectors = reference_vectors();
    asn1::Srem srem = decoded<asn1::Srem>(find(vectors, "srem_tram_request"));
    asn1::Ssem ssem = decoded<asn1::Ssem>(find(vectors, "ssem_tram_processing"));

    ValidationResult result;
    EXPECT_TRUE(check_asn1(asn_DEF_SREM, srem.content(), "SREM", result));
    EXPECT_TRUE(check_asn1(asn_DEF_SSEM, ssem.content(), "SSEM", result));
    EXPECT_TRUE(result.issues().empty());
}
