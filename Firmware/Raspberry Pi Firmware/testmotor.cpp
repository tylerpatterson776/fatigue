
#include <pigpio.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <cmath>
#include <map>
#include <deque>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include "ads1115rpi.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <termios.h>
#include <byteswap.h>
#include <vector>
#include <algorithm>
#include <ctype.h>
#include <thread>
#include <cstdint>
#include <array>
#include <atomic>
#include <optional>
#include <sstream>
#include <thread>
#include "ring.h"
#include <cassert>
#include <chrono>
#include <mutex>
#include <poll.h>
#include <iomanip>
#include <locale>

#define GPIO1 27
#define GPIO2 17
#define GPIO3 22
#define GPIO4 23

const float lbsPerVolt = 10;        //change this is load cell is recalibrated
const float newtonsPerVolt = 44.48; //change this is load cell is recalibrated
const float kgPerVolt = 4.535;
const float mmPerVolt = 1;

std::uint64_t currentTimeMillis();

const int LOADCELL_ADS1115_ADDRESS = 0x48;
const int LASER_ADS1115_ADDRESS = 0x4b;
int LOADCELL_ADS1115_HANDLE = -1;
int LASER_ADS1115_HANDLE = -1;

bool doLoadcellAveraging = 1;
bool doLaserAveraging = 0;
int doCycleWriting;
std::vector<float> currentLoadCellCycle;
std::vector<float> currentLaserCycle;
float vRef = 5.0;
int gain = 0;
int serial = -1;

const int laserStopDistance = 5;
const int minFrequency = 1;
const int maxFrequency = 50;
long long sample = -1;
long long sampleStart = 0;
long long lastSampleTimestamp = 0;
float seconds = 1;
float loadcellRefVoltage;
float laserRefVoltage;

float voltsToPounds(float input, float reference, float conversionFactor = lbsPerVolt)
{
    float result = (input - reference) * conversionFactor;
    return result;
}
float voltsToNewtons(float input, float reference, float conversionFactor = newtonsPerVolt)
{
    float result = -((input - reference) * conversionFactor);
    return result;
}
float voltsToKg(float input, float reference, float conversionFactor = kgPerVolt)
{
    float result = (input - reference) * conversionFactor;
    return result;
}
float voltsToMM(float input, float reference, float conversionFactor = mmPerVolt)
{
    float result = (input - reference) * conversionFactor;
    return result;
}
float getSample(int HANDLE);

std::atomic<bool> interrupted{false};
static_assert(std::atomic<bool>::is_always_lock_free);
std::atomic<bool> stop_now{false};
std::atomic<bool> test_failed{false};
std::atomic<std::uint64_t> shutdownDeadline{0};
std::mutex cycleMutex;
std::mutex motorMutex;

float get_vector_average(const std::vector<float>& ptr)
{
    if (ptr.empty())
        return NAN;
    float sum = 0;
    for (long unsigned int i = 0; i < ptr.size(); i++) {
        sum = sum + ptr.at(i);
    }
    float average = sum / ptr.size();
    return average;
}

float get_vector_median(std::vector<float>& ptr)
{
    if (ptr.empty())
        return NAN;
    size_t n = ptr.size() / 2;
    std::nth_element(ptr.begin(), ptr.begin() + n, ptr.end());
    return ptr[n];
}

float get_vector_max(std::vector<float>& ptr)
{
    if (ptr.empty())
        return NAN;
    float max = *std::max_element(ptr.begin(), ptr.end());
    return max;
}

float get_vector_min(std::vector<float>& ptr)
{
    if (ptr.empty())
        return NAN;
    float max = *std::min_element(ptr.begin(), ptr.end());
    return max;
}

void stop_test()
{
    std::lock_guard<std::mutex> lock(motorMutex);
    gpioPWM(GPIO1, 0);
    gpioWrite(GPIO1, 0);
    gpioWrite(GPIO2, 0);
    gpioWrite(GPIO3, 0);
    gpioWrite(GPIO4, 0);
    if (!stop_now)
        shutdownDeadline = currentTimeMillis() + 5000;
    stop_now = true;
}

void fail_test(const char* message)
{
    test_failed = true;
    stop_test();
    fprintf(stderr, "%s\n", message);
}

int abort_test(void)
{
    stop_test();
    if (serial >= 0)
        close(serial);
    if (LOADCELL_ADS1115_HANDLE >= 0)
        close(LOADCELL_ADS1115_HANDLE);
    if (LASER_ADS1115_HANDLE >= 0)
        close(LASER_ADS1115_HANDLE);
    gpioTerminate();
    return test_failed ? 1 : 0;
}

static void sigint_handler(int sig)
{
    (void)sig;
    interrupted = 1;
}

bool wait_half_cycle(unsigned int half_us)
{
    auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(half_us);
    while (!stop_now && !interrupted) {
        auto remaining = deadline - std::chrono::steady_clock::now();
        if (remaining <= std::chrono::steady_clock::duration::zero())
            return true;
        std::this_thread::sleep_for(std::min(
            remaining, std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::milliseconds(1))));
    }
    return false;
}

void measure(float loadcellReference, int loadcellHandle, float laserReference, int laserHandle, AdcBuf& buf, int,
             std::vector<float>& currentLoadCellCycle, std::vector<float>& currentLaserCycle,
             std::atomic<std::uint64_t>& halfCycleCount)
{
    std::uint64_t lastPushedTime = 0;
    std::uint64_t sampleStart = currentTimeMillis();
    while (!stop_now && !interrupted) {
        std::uint64_t time2 = currentTimeMillis() - sampleStart;
        if (time2 == lastPushedTime) {
            usleep(100);
            continue;
        }
        double loadcellSample = voltsToNewtons(getSample(loadcellHandle), loadcellReference);
        double laserSample = voltsToMM(getSample(laserHandle), laserReference);
        if (!std::isfinite(loadcellSample) || !std::isfinite(laserSample)) {
            fail_test("ADC read failed. Stopping test.");
            break;
        }
        loadcellSample = std::round(loadcellSample * 1000.0) / 1000.0;
        laserSample = std::round(laserSample * 1000.0) / 1000.0;
        std::uint64_t halfCycles = halfCycleCount.load();
        std::ostringstream packet;
        packet.imbue(std::locale::classic());
        packet << "{\"Data\":{\"Timestamp\":" << time2 << ",\"Cycle #\":" << halfCycles / 2
               << (halfCycles % 2 ? ".5" : ".0") << std::fixed << std::setprecision(3) << ",\"Load\":" << loadcellSample
               << ",\"Distance\":" << laserSample << "}}";
        if (!buf.push({packet.str()})) {
            fail_test("Serial sample buffer is full. Stopping test.");
            break;
        }
        {
            std::lock_guard<std::mutex> lock(cycleMutex);
            currentLoadCellCycle.push_back(loadcellSample);
            currentLaserCycle.push_back(laserSample);
        }
        lastPushedTime = time2;
        if (std::abs(laserSample) >= laserStopDistance) {
            fail_test("Laser distance is out of bounds. Stopping test.");
            break;
        }
    }
    buf.close();
}

bool write_serial(const std::string& packet)
{
    size_t sent = 0;
    std::uint64_t deadline = currentTimeMillis() + 2000;
    while (sent < packet.size()) {
        std::uint64_t now = currentTimeMillis();
        if (now >= deadline || (stop_now && now >= shutdownDeadline.load())) {
            errno = ETIMEDOUT;
            return false;
        }
        struct pollfd fd = {serial, POLLOUT, 0};
        int ready = poll(&fd, 1, 100);
        if (ready < 0 && errno == EINTR)
            continue;
        if (ready < 0)
            return false;
        if (ready == 0)
            continue;
        if (fd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            errno = EIO;
            return false;
        }
        ssize_t count = write(serial, packet.data() + sent, packet.size() - sent);
        if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
            continue;
        if (count <= 0)
            return false;
        sent += count;
    }
    return true;
}

void send_samples(AdcBuf& buf)
{
    printf("Send thread started\n");
    while (auto sample = buf.wait_pop()) {
        if (!write_serial(sample->to_string())) {
            fail_test("Serial write failed. Stopping test; queued samples could not be sent.");
            return;
        }
    }
}

void receive(Serialbuf& receivebuffer, int& serialobj)
{
    printf("Receive thread started\n");
    while (!stop_now) {
        struct pollfd fd = {serialobj, POLLIN, 0};
        int ready = poll(&fd, 1, 100);
        if (ready < 0 && errno == EINTR)
            continue;
        if (ready < 0 || (fd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            fail_test("Serial read failed. Stopping test.");
            break;
        }
        if (ready == 0)
            continue;
        unsigned char character;
        ssize_t count = read(serialobj, &character, 1);
        if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
            continue;
        if (count <= 0) {
            fail_test("Serial connection closed. Stopping test.");
            break;
        }
        if (character != 13 && character != 10 && !receivebuffer.push(character)) {
            fail_test("Serial receive buffer is full. Stopping test.");
            break;
        }
    }
}

bool drain_serial()
{
    for (;;) {
        int pending = 0;
        if (ioctl(serial, TIOCOUTQ, &pending) < 0)
            return false;
        if (pending == 0)
            return true;
        if (currentTimeMillis() >= shutdownDeadline.load())
            return false;
        usleep(1000);
    }
}

std::uint64_t currentTimeMillis()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

std::uint64_t currentTimeMicros()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

static inline float clamp(float v, float lo, float hi)
{
    if (v > hi) {
        v = hi;
        return v;
    }
    if (v < lo) {
        v = lo;
        return v;
    }
    return v;
}

float getSample(int HANDLE)
{
    return readVoltage(HANDLE);
}

int main(int argc, char* argv[])
{
    int frequency;
    int dutyCycle;
    try {
        if (argc != 3)
            throw std::invalid_argument("arguments");
        frequency = std::stoi(argv[1]);
        dutyCycle = std::stoi(argv[2]);
        if (frequency < minFrequency || frequency > maxFrequency || dutyCycle < 0 || dutyCycle > 100)
            throw std::invalid_argument("range");
    } catch (const std::exception&) {
        fprintf(stderr, "Usage: %s frequency(1-50 Hz) duty(0-100)\n", argv[0]);
        return 1;
    }
    gpioCfgClock(1, 1, 0);
    if (gpioInitialise() < 0) {
        fprintf(stderr, "pigpio initialisation failed \n");
        return 1;
    } else {
        printf("pigpio initialisation OK\n");
    }
    setbuf(stdout, NULL);
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);
    gpioSetMode(GPIO1, PI_OUTPUT);
    gpioSetMode(GPIO2, PI_OUTPUT);
    gpioSetMode(GPIO3, PI_OUTPUT);
    gpioSetMode(GPIO4, PI_OUTPUT);

    gpioPWM(GPIO1, 0);
    gpioWrite(GPIO1, 0);
    gpioWrite(GPIO2, 0);
    gpioWrite(GPIO3, 0);
    gpioWrite(GPIO4, 0);

    if (wiringPiSetup() != 0) {
        fail_test("Cannot initialize WiringPi.");
        return abort_test();
    }

    if ((serial = serialOpen("/dev/serial0", 921600)) < 0) {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
        test_failed = true;
        return abort_test();
    }
    if (fcntl(serial, F_SETFL, O_NONBLOCK) < 0) {
        fail_test("Unable to configure serial device.");
        return abort_test();
    }
    fflush(stdout);

    AdcBuf buf;
    Serialbuf receivebuffer;

    LOADCELL_ADS1115_HANDLE = getADS1115Handle(0x48);
    usleep(100000);
    LASER_ADS1115_HANDLE = getADS1115Handle(0x4b);
    if (LOADCELL_ADS1115_HANDLE < 0 || LASER_ADS1115_HANDLE < 0) {
        fail_test("Cannot initialize ADCs.");
        return abort_test();
    }

    std::vector<float> loadcellAveraging;

    for (int i = 0; i < 100; i++) {
        float data = readVoltageSingleShot(LOADCELL_ADS1115_HANDLE, 1, 0);
        if (!std::isfinite(data)) {
            fail_test("ADC calibration failed.");
            return abort_test();
        }
        printf("%.3f \n", data);
        loadcellAveraging.push_back(data);
    }
    loadcellRefVoltage = get_vector_average(loadcellAveraging);
    //loadcellRefVoltage = 2.5;

    if (doLaserAveraging == 1) {
        std::vector<float> laserAveraging;

        for (int i = 0; i < 100; i++) {
            float data = readVoltageSingleShot(LASER_ADS1115_HANDLE, 1, 0);
            if (!std::isfinite(data)) {
                fail_test("ADC calibration failed.");
                return abort_test();
            }
            printf("%.2f\n", data);
            laserAveraging.push_back(data);
        }
        laserRefVoltage = get_vector_average(laserAveraging);
    }

    usleep(250000);
    printf("Initial voltage on Load Cell:  %.4f \n", loadcellRefVoltage);
    printf("accessing ads1115 chip on i2c address 0x%02x\n", LOADCELL_ADS1115_ADDRESS);

    printf("Initial voltage on Laser:  %.4f \n", laserRefVoltage);
    printf("accessing ads1115 chip on i2c address 0x%02x\n", LASER_ADS1115_ADDRESS);
    if (setADS1115ContinuousMode(LOADCELL_ADS1115_HANDLE, 1, 0, 7) < 0 ||
        setADS1115ContinuousMode(LASER_ADS1115_HANDLE, 1, 0, 7) < 0) {
        fail_test("Cannot configure ADCs.");
        return abort_test();
    }
    usleep(250000);
    if (!std::isfinite(readVoltage(LOADCELL_ADS1115_HANDLE)) || !std::isfinite(readVoltage(LASER_ADS1115_HANDLE))) {
        fail_test("ADC read failed before test start.");
        return abort_test();
    }

    std::atomic<std::uint64_t> halfCycleCount{0};

    double time;
    unsigned int half_us = (unsigned int)llround(500000.0 / frequency);
    gpioSetPWMfrequency(GPIO1, 20000);
    gpioSetPWMrange(GPIO1, 100);
    std::thread measureThread(measure, loadcellRefVoltage, LOADCELL_ADS1115_HANDLE, laserRefVoltage,
                              LASER_ADS1115_HANDLE, std::ref(buf), doCycleWriting, std::ref(currentLoadCellCycle),
                              std::ref(currentLaserCycle), std::ref(halfCycleCount));
    std::thread serialThread(send_samples, std::ref(buf));
    std::thread receiveThread(receive, std::ref(receivebuffer), std::ref(serial));
    sampleStart = currentTimeMillis();

    time = currentTimeMillis();

    printf("Beginning test at %.2d Hz\n", frequency);
    while (!stop_now && !interrupted) {

        if (frequency != 0) {
            int newPWM = dutyCycle;
            {
                std::lock_guard<std::mutex> lock(motorMutex);
                if (stop_now || interrupted)
                    break;
                gpioWrite(GPIO3, 1);
                gpioWrite(GPIO4, 1);
                gpioPWM(GPIO1, newPWM);
                gpioWrite(GPIO2, 0);
                ++halfCycleCount;
            }
            if (!wait_half_cycle(half_us))
                break;
            std::vector<float> loadcellCycle;
            std::vector<float> laserCycle;
            {
                std::lock_guard<std::mutex> lock(cycleMutex);
                loadcellCycle = currentLoadCellCycle;
                laserCycle = currentLaserCycle;
            }
            if (loadcellCycle.empty() || laserCycle.empty()) {
                fail_test("No ADC samples received during cycle. Stopping test.");
                break;
            }
            time = currentTimeMillis();
            printf(
                "Cycle # %.f   LOAD CELL: Median of cycle: %.2f N   Max of cycle:%.2f  Min of cycle:%.2f   PWM: %.d \n",
                halfCycleCount.load() / 2.0, get_vector_median(loadcellCycle), get_vector_max(loadcellCycle),
                get_vector_min(loadcellCycle), dutyCycle);
            printf("time: %.2f\n", time - sampleStart);

            {
                std::lock_guard<std::mutex> lock(motorMutex);
                if (stop_now || interrupted)
                    break;
                gpioWrite(GPIO1, 0);
                gpioWrite(GPIO2, 0);
                ++halfCycleCount;
            }
            if (!wait_half_cycle(half_us))
                break;
            {
                std::lock_guard<std::mutex> lock(cycleMutex);
                currentLoadCellCycle.clear();
                currentLaserCycle.clear();
            }
        }
    }

    stop_test();
    measureThread.join();
    buf.close();
    serialThread.join();
    receiveThread.join();
    if (!drain_serial()) {
        fail_test("Serial output did not finish. Some samples could not be sent.");
    }

    return abort_test();
}
