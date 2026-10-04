#include <gtest/gtest.h>
#include "reference_vectors.hpp"
#include <vanetza/asn1/mapem.hpp>
#include <vanetza/asn1/spatem.hpp>
#include <vanetza/asn1/srem.hpp>
#include <vanetza/asn1/ssem.hpp>
#include <vanetza/facilities/map_spat_validation.hpp>
#include <vanetza/facilities/validation.hpp>
#include <algorithm>
#include <array>
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
