
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

#define GPIO2 27
#define GPIO1 17
#define GPIO3 22
#define GPIO4 23

const double lbsPerVolt = 10;
const double newtonsPerVolt = 44.48;
const double kgPerVolt = 4.535;
const double mmPerVolt = 1;

std::uint64_t currentTimeMillis();

const int LOADCELL_ADS1115_ADDRESS = 0x48;
const int LASER_ADS1115_ADDRESS = 0x49;
int LOADCELL_ADS1115_HANDLE = -1;
int LASER_ADS1115_HANDLE = -1;

bool doLoadcellAveraging = 1;
bool doLaserAveraging = 1;
int doCycleWriting;
std::vector<double> currentLoadCellCycle;
std::vector<double> currentLaserCycle;
double vRef = 5.0;
int gain = 0;
int serial = -1;

const double laserStopDistance = 2.0;
const int minFrequency = 1;
const int maxFrequency = 50;
long long sample = -1;
long long sampleStart = 0;
long long lastSampleTimestamp = 0;
double seconds = 1;
double loadcellRefVoltage;
double laserRefVoltage;

double getSample(int HANDLE);

std::atomic<bool> interrupted{false};
static_assert(std::atomic<bool>::is_always_lock_free);
std::atomic<bool> stop_now{false};
std::atomic<bool> test_failed{false};
std::atomic<std::uint64_t> shutdownDeadline{0};
std::mutex cycleMutex;
std::mutex motorMutex;

double get_vector_average(const std::vector<double>& ptr)
{
    if (ptr.empty())
        return NAN;
    double sum = 0;
    for (long unsigned int i = 0; i < ptr.size(); i++) {
        sum = sum + ptr.at(i);
    }
    double average = sum / ptr.size();
    return average;
}

double get_vector_median(std::vector<double>& ptr)
{
    if (ptr.empty())
        return NAN;
    size_t n = ptr.size() / 2;
    std::nth_element(ptr.begin(), ptr.begin() + n, ptr.end());
    return ptr[n];
}

double get_vector_max(std::vector<double>& ptr)
{
    if (ptr.empty())
        return NAN;
    double max = *std::max_element(ptr.begin(), ptr.end());
    return max;
}

double get_vector_min(std::vector<double>& ptr)
{
    if (ptr.empty())
        return NAN;
    double max = *std::min_element(ptr.begin(), ptr.end());
    return max;
}

void stop_test()
{
    std::lock_guard<std::mutex> lock(motorMutex);
    gpioPWM(GPIO2, 0);
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

void measure(double loadcellReference, int loadcellHandle, double laserReference, int laserHandle, AdcBuf& buf, int,
             std::vector<double>& currentLoadCellCycle, std::vector<double>& currentLaserCycle,
             std::atomic<std::uint64_t>& halfCycleCount, double loadcellOffset, double laserOffset)
{
    std::uint64_t lastPushedTime = 0;
    std::uint64_t sampleStart = currentTimeMillis();
    int laserTrips = 0;
    while (!stop_now && !interrupted) {
        std::uint64_t time2 = currentTimeMillis() - sampleStart;
        if (time2 == lastPushedTime) {
            usleep(100);
            continue;
        }
        double loadcellSample = getSample(loadcellHandle) + loadcellOffset;
        loadcellSample = ((loadcellSample * 4) - 10) * 2.5 * 4.448;
        double laserSample = getSample(laserHandle) + laserOffset;
        laserSample = (laserSample * 4) - 10;
        if (!std::isfinite(loadcellSample) || !std::isfinite(laserSample)) {
            fail_test("ADC read failed. Stopping test.");
            break;
        }
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
            ++laserTrips;
            printf("Laser tripped, distance was %.2f, trip count is %d\n", laserSample, laserTrips);
            if (laserTrips >= 3) {
                fail_test("Laser distance is out of bounds. Stopping test.");
                break;
            }
        } else {
            laserTrips = 0;
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

double clamp(double v, double lo, double hi)
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

double getSample(int HANDLE)
{
    return readVoltage(HANDLE);
}

int main(int argc, char* argv[])
{
    int frequency;
    double setpoint;
    try {
        if (argc != 3)
            throw std::invalid_argument("arguments");
        frequency = std::stoi(argv[1]);
        setpoint = std::stod(argv[2]);
        if (frequency < minFrequency || frequency > maxFrequency || !std::isfinite(setpoint) || setpoint < 10 ||
            setpoint > 100)
            throw std::invalid_argument("range");
    } catch (const std::exception&) {
        fprintf(stderr, "Usage: %s frequency(1-50 Hz) force(10-100 N)\n", argv[0]);
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

    gpioPWM(GPIO2, 0);
    gpioWrite(GPIO1, 0);
    gpioWrite(GPIO2, 0);
    gpioWrite(GPIO3, 0);
    gpioWrite(GPIO4, 0);

    if (wiringPiSetup() != 0) {
        fail_test("Cannot initialize WiringPi.");
        return abort_test();
    }
    int pwmFrequency = 20000;
    gpioSetPWMfrequency(GPIO2, pwmFrequency);
    gpioSetPWMrange(GPIO2, 1000);

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

    LOADCELL_ADS1115_HANDLE = getADS1115Handle(LOADCELL_ADS1115_ADDRESS);
    usleep(100000);
    LASER_ADS1115_HANDLE = getADS1115Handle(LASER_ADS1115_ADDRESS);
    if (LOADCELL_ADS1115_HANDLE < 0 || LASER_ADS1115_HANDLE < 0) {
        fail_test("Cannot initialize ADCs.");
        return abort_test();
    }

    std::vector<double> loadcellAveraging;

    for (int i = 0; i < 100; i++) {
        double data = readVoltageSingleShot(LOADCELL_ADS1115_HANDLE, 0, 0);
        if (!std::isfinite(data)) {
            fail_test("ADC calibration failed.");
            return abort_test();
        }
        printf("%.3f \n", data);
        loadcellAveraging.push_back(data);
    }
    loadcellRefVoltage = get_vector_median(loadcellAveraging);
    double loadcellOffset = 2.5 - loadcellRefVoltage;
    printf("Loadcell Offset: %.3f\n", loadcellOffset);

    std::vector<double> laserAveraging;

    for (int i = 0; i < 100; i++) {
        double data = readVoltageSingleShot(LASER_ADS1115_HANDLE, 0, 0);
        if (!std::isfinite(data)) {
            fail_test("ADC calibration failed.");
            return abort_test();
        }
        printf("%.2f\n", data);
        laserAveraging.push_back(data);
    }
    laserRefVoltage = get_vector_average(laserAveraging);
    double laserOffset = 2.5 - laserRefVoltage;

    usleep(250000);
    printf("Initial voltage on Load Cell:  %.4f \n", loadcellRefVoltage);
    printf("accessing ads1115 chip on i2c address 0x%02x\n", LOADCELL_ADS1115_ADDRESS);

    printf("Initial voltage on Laser:  %.4f \n", laserRefVoltage);
    printf("accessing ads1115 chip on i2c address 0x%02x\n", LASER_ADS1115_ADDRESS);
    if (setADS1115ContinuousMode(LOADCELL_ADS1115_HANDLE, 0, 0, 7) < 0 ||
        setADS1115ContinuousMode(LASER_ADS1115_HANDLE, 0, 0, 7) < 0) {
        fail_test("Cannot configure ADCs.");
        return abort_test();
    }
    usleep(250000);
    if (!std::isfinite(readVoltage(LOADCELL_ADS1115_HANDLE)) || !std::isfinite(readVoltage(LASER_ADS1115_HANDLE))) {
        fail_test("ADC read failed before test start.");
        return abort_test();
    }

    std::atomic<std::uint64_t> halfCycleCount{0};

    double Pgain = 0;
    double Dgain = 0.1;
    double Igain = 0.125;

    double P = 0;
    double D = 0;
    double I = 0;
    double prev_error = 0;
    double feedback = 0;
    double time;
    double prevtime = 0;
    unsigned int half_us = (unsigned int)llround(500000.0 / frequency);
    std::thread measureThread(measure, loadcellRefVoltage, LOADCELL_ADS1115_HANDLE, laserRefVoltage,
                              LASER_ADS1115_HANDLE, std::ref(buf), doCycleWriting, std::ref(currentLoadCellCycle),
                              std::ref(currentLaserCycle), std::ref(halfCycleCount), loadcellOffset, laserOffset);
    std::thread serialThread(send_samples, std::ref(buf));
    std::thread receiveThread(receive, std::ref(receivebuffer), std::ref(serial));
    sampleStart = currentTimeMillis(); //sampleStart is the time at which the experiment starts
    time = sampleStart;                //this time variable will change inside of the PID loop

    printf("Beginning test at %.2d Hz\n", frequency);
    while (!stop_now && !interrupted) {

        if (frequency != 0) {
            int newPWM = feedback;

            {
                std::lock_guard<std::mutex> lock(motorMutex);
                if (stop_now || interrupted)
                    break;
                gpioWrite(GPIO3, 1);
                gpioWrite(GPIO4, 1);
                gpioPWM(GPIO2, newPWM);
                gpioWrite(GPIO1, 0);
                ++halfCycleCount;
            }

            if (!wait_half_cycle(half_us))
                break;

            std::vector<double> loadcellCycle;
            std::vector<double> laserCycle;
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
            double dt = currentTimeMillis() - prevtime;
            prevtime = currentTimeMillis();

            printf(
                "Cycle # %.f   LOAD CELL: Median of cycle: %.2f N   Max of cycle:%.2f  Min of cycle:%.2f   PWM: %d \n",
                halfCycleCount.load() / 2.0, get_vector_median(loadcellCycle), get_vector_max(loadcellCycle),
                get_vector_min(loadcellCycle), newPWM);
            printf("LASER: Median of cycle: %.2f    Max of cycle:%.2f  Min of cycle:%.2f",
                   get_vector_median(laserCycle), get_vector_max(laserCycle), get_vector_min(laserCycle));
            printf("time: %.2f\n", time - sampleStart);

            double currentValue = get_vector_max(loadcellCycle);
            double error = setpoint - currentValue;

            P = error * Pgain;
            if (halfCycleCount.load() > 6) {
                D = (((error - prev_error) / dt) * Dgain);
                I += (error * dt * 0.01);
            }

            feedback = clamp(P + D + I * Igain, setpoint, 550);
            double feedbacknoclamp =
                (P + D + I * Igain); //this is what the feedback value would be without clamping. just to see
            printf("Feedback : %.3f, Feedback without clamping: %.4f, P: %.2f,  D: %.2f,  Iout:%.2f\n", feedback,
                   feedbacknoclamp, P, D, I * Igain);
            printf("new PWM: %d      error: %.2f  prev error: %.2f   setpoint: %.2f     current Value: %.2f\n", newPWM,
                   error, prev_error, setpoint, currentValue);
            prev_error = error;

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
