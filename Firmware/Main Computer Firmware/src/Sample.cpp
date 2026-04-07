#include "Sample.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

std::optional<Sample> Sample::parse(const std::string& line)
{
    try {
        const auto j = json::parse(line);
        const auto& d = j.at("Data");

        Sample s;
        s.timestamp_ms = d.at("Timestamp").get<uint64_t>();
        s.cycle        = d.at("Cycle #").get<uint32_t>();
        s.load_n       = d.at("Load").get<float>();
        s.distance_mm  = d.at("Distance").get<float>();
        return s;
    } catch (...) {
        return std::nullopt;
    }
}
