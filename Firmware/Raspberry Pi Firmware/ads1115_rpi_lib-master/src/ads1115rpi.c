#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <time.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <byteswap.h>

#include "ads1115rpi.h"

static int debug = 0;
static struct adsConfig configuration;

static float adsMaxGain[8] = {6.144, 4.096, 2.048, 1.024, 0.512, 0.256, 0.256, 0.256};

static int observedFreq[8] = {133, 262, 507, 940, 1645, 2500, 3333, 3333};

static float adsSPS[8] = {
    // 8 SPS, 16 SPS, 32 SPS, 64 SPS, 128 SPS, 250 SPS, 475 SPS, or 860
    // 0       1       2       3        4        5        6           7
    8, 16, 32, 64, 128, 250, 475, 860};

void adsDebug(int boolean)
{
    debug = boolean;
}

int isValidSPS(int sps)
{
    if (sps < 0) {
        return 0;
    }
    if (sps > 7) {
        return 0;
    }
    return adsSPS[sps];
}

int isValidGain(int gain)
{
    if (gain < 0) {
        return 0;
    }
    if (gain > 7) {
        return 0;
    }
    return adsMaxGain[gain];
}

int getADSampleRate(int sps)
{
    return isValidSPS(sps);
}

int getObservedFreq(int sps)
{
    return observedFreq[sps];
}

float getADS1115MaxGain(int gain)
{
    if (gain < 0) {
        return -1;
    }
    if (gain > 7) {
        return -1;
    }
    return adsMaxGain[gain];
}
uint16_t config2int(struct adsConfig config)
{
    uint16_t high = 0;
    uint16_t low = 0;

    high |= (0x01 & config.status) << 7;
    high |= (0x01 & config.mux) << 6;
    high |= (0x03 & config.channel) << 4;
    high |= (0x07 & config.gain) << 1;
    high |= (0x01 & config.operationMode) << 0;

    low |= (0x07 & config.dataRate) << 5;
    low |= (0x01 & config.compareMode) << 4;
    low |= (0x01 & config.comparatorPolarity) << 3;
    low |= (0x01 & config.latchingComparator) << 2;
    low |= (0x03 & config.comparatorQueue) << 0;

    return (high << 8) | low;
}
int writeConfiguration(int handle, struct adsConfig config)
{

    uint16_t cfg = config2int(config);

    if (debug) {
        fprintf(stderr, "writing config: 0x%04x; sps=%d\n", cfg, 0x7 & (cfg >> 5));
    }

    if (handle < 0 || wiringPiI2CWriteReg16(handle, ADS1115_ConfigurationRegister, __bswap_16(cfg)) < 0)
        return -1;
    delay(1);
    return 0;
}

int setThreshold(int handle, int reg, uint16_t value)
{
    char* threshold;

    switch (reg) {
        case ADS1115_HiThresholdRegister:
            threshold = "hi";
            break;

        case ADS1115_LoThresholdRegister:
            threshold = "lo";
            break;

        default:
            fprintf(stderr, "invalid threashold register requested for update: 0x%02x\n", reg);
            return -1;
    }

    if (debug)
        fprintf(stderr, "set threshold<%s>: 0x%4.4x\n", threshold, value);
    if (handle < 0 || wiringPiI2CWriteReg16(handle, reg, __bswap_16(value)) < 0)
        return -1;
    delay(1);
    return 0;
}

// o = operation mode
// x = mux (channel)
// g = gain
// m = mode (conversation mode)

// d = data rate (default=128 sps)
// c = compare mode (default=traditional)
// p = comparator polarity (default=active-low)
// l = latching comparator (default=non-latching)
// q = comparator queue (default=disabled)
//
//                oxxx gggm dddc plqq
// default 0x8583 1000 0101 1000 0011
//                1111 0101 1000 0011
//                1111 0101 1000 0011

int setADS1115ContinuousMode(int handle, int channel, int gain, int sps)
{

    struct adsConfig config;

    if (channel < 0 || channel > 3 || gain < 0 || gain > 7 || sps < 0 || sps > 7) {
        errno = EINVAL;
        return -1;
    }

    config.status = 0; // no effect
    config.mux = 1;    // reference channel to ground
    config.channel = channel;
    config.gain = gain;
    config.operationMode = 0; // continuous conversion
    config.dataRate = sps;
    config.compareMode = 0; // traditional
    config.comparatorPolarity = 0;
    config.latchingComparator = 0;
    config.comparatorQueue = 0;

    if (debug)
        fprintf(stderr, "set continuous mode channel=%d, gain=%d, sps=%d\n", channel, gain, sps);

    if (setADS1115Config(handle, config) < 0)
        return -1;

    // set hi/lo threshold register
    if (setThreshold(handle, ADS1115_HiThresholdRegister, (uint16_t)0xFFFF) < 0)
        return -1;
    return setThreshold(handle, ADS1115_LoThresholdRegister, (uint16_t)0x0000);
}

int setADS1115Config(int handle, struct adsConfig config)
{

    if (adsReset(handle) < 0)
        return -1;
    delay(10);

    memcpy(&configuration, &config, sizeof(configuration));
    return writeConfiguration(handle, configuration);
}

int adsReset(int handle)
{
    if (debug)
        fprintf(stderr, "reset thresholds...\n");
    if (setThreshold(handle, ADS1115_HiThresholdRegister, 0x7fff) < 0)
        return -1;
    if (setThreshold(handle, ADS1115_LoThresholdRegister, 0x8000) < 0)
        return -1;

    // hi: 0x05
    configuration.status = 0;
    configuration.mux = 0;
    configuration.channel = 0;
    configuration.gain = 2;
    configuration.operationMode = 1;

    // lo: 0x83
    configuration.dataRate = 4;
    configuration.compareMode = 0;
    configuration.comparatorPolarity = 0;
    configuration.latchingComparator = 0;
    configuration.comparatorQueue = 3;

    if (writeConfiguration(handle, configuration) < 0)
        return -1;
    if (debug)
        fprintf(stderr, "ads1115 defaults set: 0x%04x\n", config2int(configuration));
    return 0;
}

int setSingeShotSingleEndedConfig(int handle, int pin, int gain)
{

    // hi: 0xC1 | mux | pin | gain
    configuration.status = 1; // start conversion
    configuration.mux = 1;
    configuration.channel = pin;
    configuration.gain = gain;
    configuration.operationMode = 1; // signle-shot

    // lo: 0x83
    configuration.dataRate = 4;
    configuration.compareMode = 0;
    configuration.comparatorPolarity = 0;
    configuration.latchingComparator = 0;
    configuration.comparatorQueue = 3;

    return writeConfiguration(handle, configuration);
}

int isDataReady(int handle)
{
    int value = wiringPiI2CReadReg16(handle, ADS1115_ConfigurationRegister);
    if (value < 0)
        return -1;
    return __bswap_16(value) & 0x8000;
}

float readVoltageSingleShot(int handle, int pin, int gain)
{
    if (handle < 0 || pin < 0 || pin > 3 || gain < 0 || gain > 7) {
        errno = EINVAL;
        return NAN;
    }
    if (setSingeShotSingleEndedConfig(handle, pin, gain) < 0)
        return NAN;

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);
    for (;;) {
        int ready = isDataReady(handle);
        if (ready < 0)
            return NAN;
        if (ready)
            return readVoltage(handle);
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if ((now.tv_sec - start.tv_sec) * 1000 + (now.tv_nsec - start.tv_nsec) / 1000000 >= 1000) {
            errno = ETIMEDOUT;
            return NAN;
        }
        delay(1);
    }
}

float readVoltage(int handle)
{
    if (handle < 0)
        return NAN;
    int value = wiringPiI2CReadReg16(handle, ADS1115_ConversionRegister);
    if (value < 0)
        return NAN;
    int16_t rslt = __bswap_16(value);

    return adsMaxGain[configuration.gain] * rslt / 32767.0;
}

int getADS1115Handle(int address)
{
    int handle = wiringPiI2CSetup(address);
    if (handle < 0)
        return -1;
    if (adsReset(handle) < 0) {
        close(handle);
        return -1;
    }
    return handle;
}
