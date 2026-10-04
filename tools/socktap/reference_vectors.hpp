#ifndef REFERENCE_VECTORS_HPP_R7QW2KXN
#define REFERENCE_VECTORS_HPP_R7QW2KXN

#include <vanetza/common/byte_buffer.hpp>
#include <string>
#include <vector>

/**
 * Reference messages of the example intersection (FixedTimeIntersection) at fixed instants:
 * its MAPEM, SPATEMs across a signal cycle, an hour boundary and a plan change, and a tram's
 * priority request (SREM) with the answer of the intersection (SSEM).
 *
 * The messages are built deterministically, so they serve as golden test vectors and as
 * example messages for other implementations. Intersection 1 has the example region 1; the
 * road-side station has station ID 10, the tram station ID 20.
 */
struct ReferenceVector
{
    std::string name; /**< file name stem, e.g. "spatem_cycle_10s" */
    std::string message; /**< "mapem", "spatem", "srem" or "ssem" */
    std::string time; /**< UTC instant of the message, "YYYY-MM-DD hh:mm:ss.sss" */
    std::string description; /**< what the vector shows, ASCII without quotes */
    vanetza::ByteBuffer uper; /**< unaligned PER encoding */
    vanetza::ByteBuffer xer; /**< basic XER encoding, for reading */
};

/**
 * Build all reference vectors
 * \return vectors in a stable order, MAPEM first
 */
std::vector<ReferenceVector> reference_vectors();

#endif /* REFERENCE_VECTORS_HPP_R7QW2KXN */
