#pragma once
#include <array>
#include <mutex>
#include <condition_variable>
#include <stdexcept>
#include <cstdint>
#include <optional>
#include <sstream>
#include <utility>
#include <string>

enum class ADC : std::uint16_t
{
    ADC1,
    ADC2,
};

struct Sample
{
    std::string packet;

    [[nodiscard]] std::string to_string() const
    {
        std::stringstream ss;
        ss << packet << "\n";
        return ss.str();
    }
};

template <typename T, size_t N> class RingBuf
{
public:
    RingBuf() : m_head(0), m_tail(0), m_size(0), m_closed(false)
    {
        static_assert(N > 0 && (N & (N - 1)) == 0, "RingBuf size must be a power of 2");
    }

    bool push(const T& value)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_closed || m_size == N)
            return false;
        m_buffer[m_head] = value;
        m_head = (m_head + 1) & (N - 1);
        ++m_size;
        m_ready.notify_one();
        return true;
    }

    std::optional<T> pop_nowait()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_size == 0)
            return std::nullopt;
        return take();
    }

    std::optional<T> wait_pop()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_ready.wait(lock, [this] { return m_closed || m_size != 0; });
        if (m_size == 0)
            return std::nullopt;
        return take();
    }

    T pop(const bool = false)
    {
        auto value = wait_pop();
        if (!value)
            throw std::runtime_error("RingBuf is closed");
        return std::move(*value);
    }

    void close()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_closed = true;
        m_ready.notify_all();
    }

    bool empty() const
    {
        return size() == 0;
    }

    bool full() const
    {
        return size() == N;
    }

    size_t size() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_size;
    }

private:
    T take()
    {
        T value = std::move(m_buffer[m_tail]);
        m_tail = (m_tail + 1) & (N - 1);
        --m_size;
        return value;
    }

    std::array<T, N> m_buffer;
    size_t m_head;
    size_t m_tail;
    size_t m_size;
    bool m_closed;
    mutable std::mutex m_mutex;
    std::condition_variable m_ready;
};

typedef RingBuf<Sample, 2048> AdcBuf;

typedef RingBuf<int, 512> Serialbuf;

typedef RingBuf<float, 4> MiniBuf;
