#pragma once
#include <array>
#include <atomic>
#include <optional>
#include <sstream>
#include <thread>

enum class ADC: std::uint16_t
{
ADC1,
ADC2,
};

struct Sample
{
ADC adc;
float value;
std::uint64_t time_milis;

[[nodiscard]] std::string to_string() const
{
std::stringstream ss;
ss << (adc == ADC::ADC1 ? "LC" : "LA") << ": " << value << (adc == ADC::ADC1 ? " N " : " mm: ");
if (adc == ADC::ADC2){ ss << time_milis << " ms\n";}
return ss.str();
}
};

template<typename T, size_t N>
class RingBuf
{
public:
RingBuf() : m_mask(N - 1), m_head(0), m_tail(0)
{
static_assert((N & (N - 1)) == 0, "RingBuf size must be a power of 2");
static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
static_assert(std::is_trivially_constructible_v<T>, "T must be trivially constructible");
}

void push(const T& value) noexcept
{
size_t head = m_head.load(std::memory_order_relaxed);
size_t tail = m_tail.load(std::memory_order_acquire);

if (head - tail >= N) {
// drop oldest by advancing tail.
m_tail.fetch_add(1, std::memory_order_release);
tail += 1;
}

const size_t index = head & m_mask;
m_buffer[index] = value; // write the element first
m_head.store(head + 1, std::memory_order_release); // then publish
}

std::optional<T> pop_nowait() noexcept
{
size_t tail = m_tail.load(std::memory_order_relaxed);
size_t head = m_head.load(std::memory_order_acquire);

if (tail == head) {
return std::nullopt; // empty
}

const size_t index = tail & m_mask;
T value = m_buffer[index];

m_tail.fetch_add(1, std::memory_order_release);

return value;
}

T pop(const bool spin = false) noexcept
{
for (;;) {
if (auto v = pop_nowait()) return *v;
if (!spin) std::this_thread::yield();
}
}

bool empty() const noexcept
{
const size_t head = m_head.load(std::memory_order_acquire);
const size_t tail = m_tail.load(std::memory_order_acquire);
return head == tail;
}

bool full() const noexcept
{
const size_t head = m_head.load(std::memory_order_acquire);
const size_t tail = m_tail.load(std::memory_order_acquire);
return (head - tail) >= N;
}

size_t size() const noexcept
{
const size_t head = m_head.load(std::memory_order_acquire);
const size_t tail = m_tail.load(std::memory_order_acquire);
return head - tail;
}

private:
std::array<T, N> m_buffer;
const size_t m_mask;

alignas(64) std::atomic<size_t> m_head;
alignas(64) std::atomic<size_t> m_tail;
};


typedef RingBuf<Sample, 64>
 AdcBuf;



/*

int main() {
AdcBuf buf;
std::atomic_bool done = false;

std::thread consumer([&]
{
while (!done || !buf.empty())
{
const auto sample = buf.pop(true);
// Write to serial or whatever.
}
});

std::thread producer([&]
{
for (int i = 0; i < 5000; ++i)
{
buf.push(Sample{
i % 2 == 0 ? ADC::ADC1 : ADC::ADC2, (uint16_t)i, (uint32_t)i
});
}
done = true;
});

consumer.join();
producer.join();

return 0;
}
*/
