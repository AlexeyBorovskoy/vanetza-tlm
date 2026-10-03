#include "asn1_decoding.hpp"

// entry point for libFuzzer, e.g. clang++ -fsanitize=fuzzer
extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    vanetza::decode_infrastructure_messages(data, size);
    return 0;
}
