#include <stdio.h>
#include <signal.h>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <locale>
#include "ads1115rpi.h"
#include <wiringPi.h>

static volatile sig_atomic_t stop_now = 0;

static void sigint_handler(int sig)
{
    (void)sig;
    stop_now = 1;
}

std::uint64_t currentTimeMillis()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

int main()
{
    if (wiringPiSetup() != 0) {
        fprintf(stderr, "Cannot initialize WiringPi.\n");
        return 1;
    }
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);
    int loadcellHandle = getADS1115Handle(0x48);
    int laserHandle = getADS1115Handle(0x4b);
    if (loadcellHandle < 0 || laserHandle < 0 || setADS1115ContinuousMode(loadcellHandle, 1, 0, 7) < 0 ||
        setADS1115ContinuousMode(laserHandle, 1, 0, 7) < 0) {
        fprintf(stderr, "Cannot initialize ADCs.\n");
        if (loadcellHandle >= 0)
            close(loadcellHandle);
        if (laserHandle >= 0)
            close(laserHandle);
        return 1;
    }
    usleep(100000);
    std::uint64_t starttime = currentTimeMillis();
    std::uint64_t count = 0;
    int result = 0;
    std::cout.imbue(std::locale::classic());
    std::cout << std::fixed << std::setprecision(3);
    while (!stop_now) {
        double loadcellSample = readVoltage(loadcellHandle);
        double laserSample = readVoltage(laserHandle);
        if (!std::isfinite(loadcellSample) || !std::isfinite(laserSample)) {
            fprintf(stderr, "ADC read failed.\n");
            result = 1;
            break;
        }
        ++count;
        if (count % 100 == 1) {
            std::cout << "{\"Data\":{\"Timestamp\":" << currentTimeMillis() - starttime
                      << ",\"Cycle #\":1,\"Load\":" << loadcellSample << ",\"Distance\":" << laserSample << "}}\n";
        }
    }
    close(loadcellHandle);
    close(laserHandle);
    return result;
}
