#pragma once

#include <cstdint>
#include <optional>
#include <string>

struct Sample
{
    uint64_t timestamp_ms = 0;
    double cycle = 0;
    float load_n = 0.f;
    float distance_mm = 0.f;

    static std::optional<Sample> parse(const std::string& line);
};
