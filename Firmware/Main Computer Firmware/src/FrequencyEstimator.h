#pragma once
#include <algorithm>
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

    Result update(const float timestamp, const float val)
    {
        if (m_sample_count == 0)
        {
            m_mean = val;
            m_was_above = val >= m_mean;
        } else
        {
            constexpr float ema_factor = 0.95f;
            m_mean = (1.0f - ema_factor) * val + ema_factor * m_mean;
        }
        ++m_sample_count;

        m_local_max = std::max(m_local_max, val);
        m_local_min = std::min(m_local_min, val);

        const bool above = val >= m_mean;

        if (m_sample_count > 1 && above != m_was_above)
        {
            if (!above)
            {
                // Crossed downwards
                update_avg(m_max_accum, m_max_count, m_local_max);
                m_local_max = val;
            } else
            {
                // Crossed upwards
                update_avg(m_min_accum, m_min_count, m_local_min);
                m_local_min = val;
            }

            ++m_crossings_count;
            if (m_crossings_count % 2 == 1 && m_crossings_count >= 3)
            {
                // Let's hope we don't get a duplicate timestamp...
                const float period = timestamp - m_last_full_cycle_time;
                m_frequency = 1.0f / period;
            }
            if (m_crossings_count % 2 == 1)
            {
                m_last_full_cycle_time = timestamp;
            }

            m_was_above = above;
        }

        return {
            m_frequency,
            m_max_count > 0 ? m_max_accum / static_cast<float>(m_max_count) : 0,
            m_min_count > 0 ? m_min_accum / static_cast<float>(m_min_count) : 0
        };
    }

private:
    static void update_avg(float& accum, int &count, float value)
    {
        accum += value;
        ++count;
    }

    float m_mean = 0;
    int m_sample_count = 0;
    bool m_was_above = false;
    int m_crossings_count = 0;
    float m_last_full_cycle_time = 0;
    float m_frequency = 0;

    float m_local_max = -std::numeric_limits<float>::max();
    float m_local_min = std::numeric_limits<float>::max();

    float m_max_accum = 0;
    int m_max_count = 0;
    float m_min_accum = 0;
    int m_min_count = 0;
};
