#include "asn1_decoding.hpp"
#include <vanetza/asn1/mapem.hpp>
#include <vanetza/asn1/spatem.hpp>
#include <vanetza/asn1/srem.hpp>
#include <vanetza/asn1/ssem.hpp>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace
{

// valid message of the expected type, including its ITS PDU header (ETSI TS 103 301),
// and nothing else: the decoder ignores trailing bytes, so compare with the re-encoding
template<typename MESSAGE>
bool is_message(const std::vector<uint8_t>& buffer, long message_id)
{
    MESSAGE message;
    return message.decode(buffer.data(), buffer.size()) && message.validate() &&
        message->header.protocolVersion == 2 && message->header.messageID == message_id &&
        message.encode() == buffer;
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc != 2 && argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <filepath> [spatem|mapem|srem|ssem]" << std::endl;
        return 1;
    }

    std::ifstream file(argv[1], std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << argv[1] << std::endl;
        return 1;
    }

    const std::vector<uint8_t> buffer { std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>() };
    const unsigned decoded = vanetza::decode_infrastructure_messages(buffer.data(), buffer.size());

    std::cout << argv[1] << " decodes as:";
    std::cout << (decoded & vanetza::DecodedSpatem ? " spatem" : "");
    std::cout << (decoded & vanetza::DecodedMapem ? " mapem" : "");
    std::cout << (decoded & vanetza::DecodedSrem ? " srem" : "");
    std::cout << (decoded & vanetza::DecodedSsem ? " ssem" : "");
    std::cout << std::endl;

    if (argc == 3) {
        // fail unless the input is a valid message of the expected type
        const std::string type = argv[2];
        bool valid = false;
        if (type == "spatem") {
            valid = is_message<vanetza::asn1::Spatem>(buffer, ItsPduHeader__messageID_spatem);
        } else if (type == "mapem") {
            valid = is_message<vanetza::asn1::Mapem>(buffer, ItsPduHeader__messageID_mapem);
        } else if (type == "srem") {
            valid = is_message<vanetza::asn1::Srem>(buffer, ItsPduHeader__messageID_srem);
        } else if (type == "ssem") {
            valid = is_message<vanetza::asn1::Ssem>(buffer, ItsPduHeader__messageID_ssem);
        }
        if (!valid) {
            std::cerr << "not a valid " << type << std::endl;
            return 2;
        }
    }
    return 0;
}
