#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

class FrequencyEstimator
{
public:
    struct Result
    {
        float frequency_hz;
        float avg_max;
        float avg_min;
    };

    explicit FrequencyEstimator(const float hysteresis = 0.002f) : m_hysteresis(std::max(0.0f, hysteresis)) {}

    void reset()
    {
        *this = FrequencyEstimator(m_hysteresis);
    }

    Result expire(const double timestamp)
    {
        const double timeout = std::max(2.0, 3.0 * m_period);
        if (m_has_crossing && timestamp - m_last_crossing_time > timeout) {
            m_frequency = 0;
            m_period = 0;
            m_period_count = 0;
            m_period_index = 0;
            m_needs_sample = true;
            m_has_crossing = false;
            m_has_cycle = false;
            m_direction = 0;
        }
        return result();
    }

    Result update(const double timestamp, const float val)
    {
        if (!std::isfinite(timestamp) || !std::isfinite(val)) {
            reset();
            return result();
        }
        if (m_has_sample && timestamp < m_last_timestamp) {
            reset();
        }
        if (m_has_sample && timestamp == m_last_timestamp) {
            return result();
        }

        expire(timestamp);
        if (!m_has_sample || m_needs_sample) {
            m_has_sample = true;
            m_needs_sample = false;
            m_last_timestamp = timestamp;
            m_mean = val;
            m_previous_delta = 0;
            m_local_max = val;
            m_local_min = val;
            return result();
        }

        const double dt = timestamp - m_last_timestamp;
        m_mean += dt / (0.5 + dt) * (val - m_mean);
        const double delta = val - m_mean;
        m_local_max = std::max(m_local_max, val);
        m_local_min = std::min(m_local_min, val);

        const bool rising = m_direction <= 0 && delta > m_hysteresis;
        const bool falling = m_direction >= 0 && delta < -m_hysteresis;
        if (rising || falling) {
            const double threshold = rising ? m_hysteresis : -m_hysteresis;
            const double fraction = std::clamp((threshold - m_previous_delta) / (delta - m_previous_delta), 0.0, 1.0);
            const double crossing_time = m_last_timestamp + fraction * dt;
            m_last_crossing_time = crossing_time;
            m_has_crossing = true;
            m_direction = rising ? 1 : -1;

            if (rising) {
                if (m_has_cycle) {
                    m_periods[m_period_index] = crossing_time - m_last_cycle_time;
                    m_period_index = (m_period_index + 1) % m_periods.size();
                    m_period_count = std::min(m_period_count + 1, m_periods.size());
                    m_period = 0;
                    for (std::size_t i = 0; i < m_period_count; ++i) {
                        m_period += m_periods[i];
                    }
                    m_period /= static_cast<double>(m_period_count);
                    m_frequency = static_cast<float>(1.0 / m_period);
                    m_max_accum += m_local_max;
                    m_min_accum += m_local_min;
                    ++m_cycle_count;
                }
                m_has_cycle = true;
                m_last_cycle_time = crossing_time;
                m_local_max = val;
                m_local_min = val;
            }
        }

        m_last_timestamp = timestamp;
        m_previous_delta = delta;
        return result();
    }

private:
    Result result() const
    {
        return {m_frequency, m_cycle_count > 0 ? static_cast<float>(m_max_accum / m_cycle_count) : 0,
                m_cycle_count > 0 ? static_cast<float>(m_min_accum / m_cycle_count) : 0};
    }

    float m_hysteresis;
    double m_mean = 0;
    double m_previous_delta = 0;
    double m_last_timestamp = 0;
    double m_last_crossing_time = 0;
    double m_last_cycle_time = 0;
    double m_period = 0;
    bool m_has_sample = false;
    bool m_needs_sample = false;
    bool m_has_crossing = false;
    bool m_has_cycle = false;
    int m_direction = 0;
    float m_frequency = 0;
    std::array<double, 4> m_periods{};
    std::size_t m_period_count = 0;
    std::size_t m_period_index = 0;

    float m_local_max = -std::numeric_limits<float>::max();
    float m_local_min = std::numeric_limits<float>::max();
    double m_max_accum = 0;
    double m_min_accum = 0;
    std::uint64_t m_cycle_count = 0;
};
