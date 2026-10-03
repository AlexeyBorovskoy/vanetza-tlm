#ifndef ASN1_DECODING_HPP_V2HKDN8W
#define ASN1_DECODING_HPP_V2HKDN8W

#include <cstddef>
#include <cstdint>

namespace vanetza
{

/** Flags of the message types an input decodes to */
enum DecodedMessage : unsigned
{
    DecodedSpatem = 1,
    DecodedMapem = 2,
    DecodedSrem = 4,
    DecodedSsem = 8
};

/**
 * Decode arbitrary input as SPATEM, MAPEM, SREM and SSEM.
 *
 * Every successfully decoded and valid message is encoded and decoded again;
 * a differing result is reported by abort().
 *
 * \return DecodedMessage flags of the types decoded into a valid message
 */
unsigned decode_infrastructure_messages(const uint8_t* data, std::size_t size);

} // namespace vanetza

#endif /* ASN1_DECODING_HPP_V2HKDN8W */
