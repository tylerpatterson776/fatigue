#include "Sample.h"
#include <cmath>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

std::optional<Sample> Sample::parse(const std::string& line)
{
    try {
        const auto j = json::parse(line);
        const auto& d = j.at("Data");
        constexpr float VOLTAGE_TO_MM = 1;

        if (!d.at("Cycle #").is_number() || !d.at("Load").is_number() || !d.at("Distance").is_number())
            return std::nullopt;

        Sample s;
        const auto& timestamp = d.at("Timestamp");
        if (timestamp.is_number_unsigned()) {
            s.timestamp_ms = timestamp.get<uint64_t>();
        } else if (timestamp.is_number_integer()) {
            const auto value = timestamp.get<int64_t>();
            if (value < 0)
                return std::nullopt;
            s.timestamp_ms = static_cast<uint64_t>(value);
        } else if (timestamp.is_number_float()) {
            const auto value = timestamp.get<double>();
            if (!std::isfinite(value) || value < 0 || value >= 0x1p64 || std::trunc(value) != value)
                return std::nullopt;
            s.timestamp_ms = static_cast<uint64_t>(value);
        } else {
            return std::nullopt;
        }
        s.cycle = d.at("Cycle #").get<double>();
        s.load_n = d.at("Load").get<float>();
        s.distance_mm = d.at("Distance").get<float>() * VOLTAGE_TO_MM;
        if (!std::isfinite(s.cycle) || s.cycle < 0 || !std::isfinite(s.load_n) || !std::isfinite(s.distance_mm))
            return std::nullopt;
        return s;
    } catch (...) {
        return std::nullopt;
    }
}
