#include "asn1_decoding.hpp"
#include <vanetza/asn1/mapem.hpp>
#include <vanetza/asn1/spatem.hpp>
#include <vanetza/asn1/srem.hpp>
#include <vanetza/asn1/ssem.hpp>
#include <cstdlib>

namespace vanetza
{

namespace
{

template<typename MESSAGE>
bool roundtrip(const uint8_t* data, std::size_t size)
{
    MESSAGE decoded;
    if (!decoded.decode(data, size) || !decoded.validate()) {
        return false;
    }

    // the decoder enforces PER visible constraints, so encoding must succeed;
    // an exception escaping here is reported by the fuzzer
    const ByteBuffer encoded = decoded.encode();

    MESSAGE again;
    if (!again.decode(encoded) || decoded.compare(again) != 0) {
        std::abort();
    }
    return true;
}

} // namespace

unsigned decode_infrastructure_messages(const uint8_t* data, std::size_t size)
{
    unsigned decoded = 0;
    decoded |= roundtrip<asn1::Spatem>(data, size) ? DecodedSpatem : 0;
    decoded |= roundtrip<asn1::Mapem>(data, size) ? DecodedMapem : 0;
    decoded |= roundtrip<asn1::Srem>(data, size) ? DecodedSrem : 0;
    decoded |= roundtrip<asn1::Ssem>(data, size) ? DecodedSsem : 0;
    return decoded;
}

} // namespace vanetza
