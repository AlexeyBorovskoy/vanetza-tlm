#include "reference_vectors.hpp"
#include <exception>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

// Write the reference vectors of the example intersection into a directory:
// <name>.uper (unaligned PER), <name>.xer (basic XER) and manifest.json describing them.

namespace
{

void write(const std::string& path, const vanetza::ByteBuffer& content)
{
    std::ofstream file(path, std::ios::binary);
    file.write(reinterpret_cast<const char*>(content.data()), content.size());
    if (!file) {
        throw std::runtime_error("cannot write " + path);
    }
}

std::string quoted(const std::string& text)
{
    std::string result = "\"";
    for (char c : text) {
        if (c == '"' || c == '\\') {
            result += '\\';
        }
        result += c;
    }
    return result + "\"";
}

} // namespace

int main(int argc, const char** argv)
{
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " <existing output directory>" << std::endl;
        return 1;
    }
    const std::string directory = argv[1];

    try {
        std::string manifest = "[\n";
        bool first = true;
        for (const ReferenceVector& vector : reference_vectors()) {
            write(directory + "/" + vector.name + ".uper", vector.uper);
            write(directory + "/" + vector.name + ".xer", vector.xer);
            manifest += first ? "" : ",\n";
            manifest += "  {\"name\": " + quoted(vector.name) + ", \"message\": " + quoted(vector.message) +
                ", \"time\": " + quoted(vector.time) + ", \"bytes\": " + std::to_string(vector.uper.size()) +
                ",\n   \"description\": " + quoted(vector.description) + "}";
            first = false;
        }
        manifest += "\n]\n";
        write(directory + "/manifest.json", vanetza::ByteBuffer(manifest.begin(), manifest.end()));
    } catch (const std::exception& e) {
        std::cerr << "reference vectors failed: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
