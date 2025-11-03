//
//  pigpio_ads1115.h
//  pigpio_ads1115
//
//  Created by Diego Eguez on 30/12/22.
//

#ifndef pigpio_ads1115_h
#define pigpio_ads1115_h

#include <pigpio.h>
#include <stdexcept>

/// The default raw difference for the low threshold, when not specified in continous conversion mode (startComparatorMode_SingleEnded)
#define DEFAULT_LOW_THRESHOLD_DIFF 5

/** Enumerates the addresses available for the ADS */
enum class AdsAddress: uint8_t {

    /// Default address (use when ADR pin is connected to GND... or not connected at all)
    gnd = 0x48,

    /// Use when ADR pin is connected to VCC pin
    vcc = 0x49,

    /// Use when ADR address is connected to SDA pin
    sda = 0x4A,

    /// Use when ADR pin is connected to SCL pin
    scl = 0x4B
};

/**
 * Available gains for the PGA (programmable gain amplifier) of the Ads1115
 * The possible configurations for the PGA (bits 11:9) of the config register
 */
enum class AdsGain: uint16_t {

    /// Gain = 2/3, FSR = +/- 6.144V, Resolution / LSB size = 187.5uV
    twoThirds = 0x0,

    /// Gain = 1, FSR = +/- 4.096V, Resolution / LSB size = 125uV
    one = (uint16_t)0x1 << 9,

    /// Gain = 2, FSR = +/- 2.048, Resolution / LSB size = 62.5uV
    two = (uint16_t)0x2 << 9,

    /// Gain = 4, FSR = +/- 1.024, Resolution / LSB size = 31.25uV
    four = (uint16_t)0x3 << 9,

    /// Gain = 8, FSR = +/- 0.512, Resolution / LSB size = 15.625uV
    eight = (uint16_t)0x4 << 9,

    /// Gain = 16, FSR = +/- 0.256, Resolution / LSB size = 7.8125uV
    sixteen = (uint16_t)0x5 << 9
};

/**
 * The configuration for the data rate samples per second (bits 7:5) of the config register
 * For more details on the delays used for reading at each sample rate see: delayForChannelReading()
 */

enum class AdsSampleSpeed: uint16_t {
    sps8 = 0x0,
    sps16 = 0x1 << 5,
    sps32 = 0x2 << 5,
    sps64 = 0x3 << 5,
    sps128 = 0x4 << 5,
    sps250 = 0x5 << 5,
    sps475 = 0x6 << 5,
    sps860 = 0x7 << 5
};

/**
 * The possible configurations for the PGA (bits 11:9) of the config register ]
 * Added for compatibility with ADS1x15 Adafruit library
 */
typedef enum {

    /// Gain = 2/3, FSR = +/- 6.144V, Resolution / LSB size = 187.5uV
    GAIN_TWOTHIRDS = 0x0,

    /// Gain = 1, FSR = +/- 4.096V, Resolution / LSB size = 125uV
    GAIN_ONE = 0x0200,

    /// Gain = 2, FSR = +/- 2.048, Resolution / LSB size = 62.5uV
    GAIN_TWO = 0x0400,

    /// Gain = 4, FSR = +/- 1.024, Resolution / LSB size = 31.25uV
    GAIN_FOUR = 0x0600,

    /// Gain = 8, FSR = +/- 0.512, Resolution / LSB size = 15.625uV
    GAIN_EIGHT = 0x0800,

    /// Gain = 16, FSR = +/- 0.256, Resolution / LSB size = 7.8125uV
    GAIN_SIXTEEN = 0x0A00
} adsGain_t;

/**
 * The available configurations for the comparator mode (bit 4)
 * See Figure 28 of the datasheet for more information
 */
enum class ComparatorModeConfig : uint16_t {

    /// Asserts the ALRT pin when value goes above the high threshold or below the low threshold
    traditionalComparator = 0x0,

    /// Asserts the ALRT each time the value crosses a threshold low or high
    windowComparator = 0x1 << 4
};

/**
 * The configuration for the comparator polarity (bit 3)
 * Remember to use a pull-up resistor for this to work correctly
 */
enum class ComparatorPolarityConfig : uint16_t {

    /// The ALRT pin will be grounded when asserted
    activeLow = 0x0,

    /// The ALRT pint will be left floating when asserted (it'll be high assuming there is a pull-up resistor connected)
    activeHigh = 0x1 << 3
};

/**
 * The configuration for wheater the ALERT/RDY pin should latch after being asserted (bit 2)
 * See Figure 28 of the datasheet for more information
 */
enum class ComparatorLatchingConfig : uint16_t {

    /// ALRT pin will be asserted while above the low threshold (assuming it has first gone above the high threshold)
    nonLatching = 0x0,

    /**
     * ALRT pin will be asserted when it passes the high threshold
     * It'll be kept asserted even after it goes below the low threshold
     * Assertion will be cleared when the value is read (or a SMBus alert is sent)
     * If the assertion is cleared and the value is above the high threshold, the pin will be re-asserted
     */
    latching = 0x1 << 2
};

/**
 * These bits perform two functions (bits 1:0):
 * - When set to [disableAndSetHighImpedance] the comparator function is disabled
 * - For other values the comparator will assert (the ALERT/RDY pin) after the given number of values
 */
enum class ComparatorAssertConfig : uint16_t {

    /// Asserts the ALRT pin after one conversion
    assertAfterOne = 0x0,

    /// Asserts the ALRT pin after two conversions
    assertAfterTwo = 0x1,

    /// Asserts the ALRT pin after four conversions
    assertAfterFour = 0x2,

    /// Disables the ALRT pin asserting functionality
    disableAndSetHighImpedance = 0x3
};

/**
 * The posible configurations for the mux config (bits 14:12)
 * The mux controls the channel(s) that are read from the ADS
 * Note the ADS can read a single channel or the difference between the chanels specified
 */
enum class MuxConfig : uint16_t {

    /// Reading value = channel A0 - channel A1
    differential01 = 0x0,

    /// Reading value = channel A0 - channel A3
    differential03 = (uint16_t)0x1 << 12,

    /// Reading value = channel A1 - channel A3
    differential13 = (uint16_t)0x2 << 12,

    /// Reading value = channel A2 - channel A3
    differential23 = (uint16_t)0x3 << 12,

    /// Read channel 0
    channel0 = (uint16_t)0x4 << 12,

    /// Read channel 1
    channel1 = (uint16_t)0x5 << 12,

    /// Read channel 2
    channel2 = (uint16_t)0x6 << 12,

    /// Read channel 3
    channel3 = (uint16_t)0x7 << 12
};


/**
 * Class used to interface with the ADS1115
 * Create an instance of this class for each ADS1115 breakout / chiplet you'll read.
 * You can switch the address but it might become problematic in continous conversion mode
 * Remember you can use up to 4 ADS1115 for each i2c interface
 *
 * It is fully compatible with Adafruit_ADS1015 library (it incorporates the same interface and allows extended functionality)
 * Hence it is possible to just change the library and keep having the same code!
 * The only drawback is that this library doesn't work with the ADS1015
 *
 * This library incorporates a set opf improvements over the ADS1015
 * - Provides the addresses needed to use other boards on the same i2c interface
 * - Read from all differential channels (0-1, 0-3, 1-3 and 2-3)
 * - Get the readings in millivolts (for easier use and testing)
 * - Allows for faster reading times on single shot readings and provides info about the reading delays used
 * - Allows the configuration of the samples per second (great for time critial applications)
 * - Expands the continous conversion mode. You can now use differential channels
 * - Improves the comparator mode. You can now set Low Threshold, Assert Polarity, Comparator Window Mode, Assertion Latching and Comparator Queue
 */

class PigpioAds1115 {
private:
    
    // MARK: Private properties
    
    /** The current address for the ADS1115, defaults to GND - 0x48 */
    uint8_t address;
    
    /** The gain to be used for the readings in the adc */
    uint16_t gain;

    /** The bits for the config of the data rate */
    uint16_t sampleSpeed;

    /** The os config bit */
    uint16_t osConfig;

    /** The mux config bits */
    uint16_t muxConfig;

    /** The mode config bit */
    uint16_t adsMode;

    /** The comparator mode bit */
    uint16_t comparatorMode;

    /** The comparator polarity bit */
    uint16_t comparatorPolarity;

    /** The comparator latching bit */
    uint16_t comparatorLatching;

    /** The queue and disable bits */
    uint16_t comparatorAssertConfig;

    /** The bus of the I2C connection where this sensor is located */
    uint8_t i2cBus;

    /** The handle for the I2C connections. -1 if not set or missing */
    int i2cHandle;
    
    // MARK: Nested enums

    /** The posible configurations for the OS config bit (bit 15) */
    enum class OsConfig: uint16_t {
        noEffect = 0x0, // write
        performingConversion = 0x0, // read
        startSingleConversion = (uint16_t)0x1 << 15, // write
        notPerformingConversion = (uint16_t)0x1 << 15// read
    };

    /** The configuration for the reading mode config (bit 8) */
    enum class AdsModeConfig: uint16_t {

        /** Allows the ADS to work in continous conversion */
        continuousConversion = 0x0,

        /** Use when performing single reads from the ADS */
        singleShotConversion = 0x1 << 8
    };


    /**
     * Available address pointer values
     * The address pointer determines which registers will be read / written
     */
    enum class AddressPointerReg: uint8_t {

        /** Contains the results of the last conversion */
        conversionRegister = 0x0,

        /** Used to change the configuration of the ADS */
        configRegister = 0x1,

        /** 16-bit register used as the low threshold in comparator mode */
        lowThresholdRegister = 0x2,

        /** 16-bit register used as the high threshold in comparator mode */
        highThresholdRegister = 0x3
    };
    
    // MARK: Private methods
    
    /** Returns the config register based on the current configuration / state */
    uint16_t buildConfigRegister() {
        return
            comparatorAssertConfig | // bits 0:1
            comparatorLatching | // bit 2
            comparatorPolarity | // bit 3
            comparatorMode | // bit 4
            sampleSpeed | // bits 5:7
            adsMode | // bit 8
            gain | // bits 9:11
            muxConfig | // bits 12:14
            osConfig; // bit 15
    }
    
    /**
     * Performs a single shot reading to the ADS1115 using the current config
     * Make sure the [modeConfig] has been set to [singleShot]
     */
    int16_t currentConfigSingleShotRead() {
        writeCurrentConfig();
        auto readingDelay = delayForChannelReading();
        time_sleep(readingDelay); // TODO: Check if this works in multithreaded environments

        return readFromAds(address, (uint8_t)AddressPointerReg::conversionRegister, i2cBus, i2cHandle);
    }
    
    /** Writes the [currentConfigRegister] to the ads */ 
    void writeCurrentConfig() {
        uint16_t configRegister = buildConfigRegister();
        writeToAds(address, (uint8_t)AddressPointerReg::configRegister, configRegister, i2cBus, i2cHandle);
    }
    
    /** Returns the mux config of the given [channel] */
    uint16_t muxConfigOfSingleChannel(uint8_t channel) {
        // Change the mux config
        switch (channel) {
        case 0:
            return (uint16_t)MuxConfig::channel0;
        case 1:
            return (uint16_t)MuxConfig::channel1;
        case 2:
            return (uint16_t)MuxConfig::channel2;
        case 3:
            return (uint16_t)MuxConfig::channel3;
        }

        // Return channel 0 if none is found
        return (uint16_t)MuxConfig::channel0;
    }
    
    // MARK: Private static methods
    
    /**
     * Writes the given [value] to the Ads
     * @param i2cAddress The address of the ADS for which the value is written
     * @param reg The [AddressPointer] register to which the value will be writen
     * @param value the data to be written (note this is a 16 bit value written in batches of 8)
     */
    static void writeToAds(uint8_t i2cAddress, uint8_t reg, uint16_t value, uint8_t bus, int handle) {
        if (handle < 0) {
            throw std::invalid_argument("A connection must first be established, before sending data to the ADS");
        }
        auto data = flipWordBytes(value);
        i2cWriteWordData(handle, reg, data);
    }
    
    /**
     * Reads two bytes from the Ads (a int16), with the given [i2cAddress]
     * Note the value returned can be negative or positive
     * @param i2cAddress The address of the Ads from which the data will be read
     * @param reg The [AddressPointer] register from which data will be read
     * @param bus The I2C of the raspberry pi from which the measurement is requested
     * @return The two bytes read from the Ads as a uint16
     */
    static int16_t readFromAds(uint8_t i2cAddress, uint8_t reg, uint8_t bus, int handle) {
        if (handle < 0) {
            throw std::invalid_argument("A connection must first be established, before reading data from the ADS");
        }

        auto rawData = i2cReadWordData(handle, reg);
        return flipWordBytes(rawData);
    }

    /**
     * @brief Switches the positions of the bytes in the [word]. The least significant byte
     * becomes the most significant one and viceversa.
     * 
     * @param word The 2 byte integer whose bytes are to be flipped
     * @return uint16_t The word with the flipped bytes
     */
    static uint16_t flipWordBytes(uint16_t word) {
        auto upperByte = (word >> 8) & 0xFF;
        auto lowerByte = word & 0xFF;

        return (lowerByte << 8) | upperByte;
    }

    
public:
    
    // MARK: Constructor
    /**
     *  Creates a new PigpioAds1115 instance, with the following defaults
     * - address = AdsAddres::gnd
     * - gain = AdsGain::twoThirds +/- 6.144v
     * - dataRate = AdsDataRate::sps64 (64 samples per second, which is sufficiently fast and stable)
     * @param autoStart Initiates the I2C connection in the constructor (if true), else use begin() to start the connection.
     */
    PigpioAds1115(AdsAddress adsAddress = AdsAddress::gnd, AdsGain adsGain = AdsGain::twoThirds, AdsSampleSpeed dataRate = AdsSampleSpeed::sps64, uint8_t bus = 1, bool autoStart = true) :
    address((uint8_t)adsAddress), gain((uint16_t)adsGain), sampleSpeed((uint16_t)dataRate), i2cBus(bus), i2cHandle(-1) {
        
        // Set up the default config register values
        comparatorAssertConfig = (uint16_t) ComparatorAssertConfig::disableAndSetHighImpedance;
        comparatorLatching = (uint16_t) ComparatorLatchingConfig::nonLatching;
        comparatorPolarity = (uint16_t) ComparatorPolarityConfig::activeLow;
        comparatorMode = (uint16_t) ComparatorModeConfig::traditionalComparator;
        adsMode = (uint16_t)AdsModeConfig::singleShotConversion;
        muxConfig = (uint16_t) MuxConfig::channel0;
        osConfig = (uint16_t)OsConfig::startSingleConversion;

        if (autoStart) {
            startI2C();
        }
    }

    ~PigpioAds1115() {
        stopI2C();
    }
    
    // MARK: I2C management

    /** 
     * Tries to open the I2C communication with the device using the current state
     * Sets the [handle] value if successfull, else throws an exception
     */
    void startI2C() {
        auto handle = i2cOpen(i2cBus, address, 0);
        if (handle < 0) {
            throw std::runtime_error("Cannot connect to ADS!");
        } else {
            i2cHandle = handle;
        }
    }

    void stopI2C() {
        if (i2cHandle >= 0) {
            i2cClose(i2cHandle);
            i2cHandle = -1;
        }
    }
    
    
    // MARK: Single shot reading
    
    /** Returns the raw value (in bits) of the given [channel] */
    int16_t readADC_singleEnded(uint8_t channel) {
        if (channel > 3) {
            // TODO: Try look at exceptions config or find a different value
            return 0;
        }

        // Modify the mux config and return the reading
        muxConfig = muxConfigOfSingleChannel(channel);
        return currentConfigSingleShotRead();
    }
    
    /**
     * Reads the given [channel] from the ADS1115 with the current gain
     * @param channel The channel to be read from the ads (0 to 3)
     * @return The value of the given [channel] raw (note max value is 2^15)
     */
    int16_t readChannelRaw(uint8_t channel) {
        return readADC_singleEnded(channel);
    }

    /**
     * Reads the given [channel] from the ADS1115 with the current gain
     * @param channel The channel to be read from the ads (0 to 3)
     * @return the value of the given [channel] in millivolts
     */
    double readChannelMillivolts(uint8_t channel) {
        uint16_t channelValue = readChannelRaw(channel);
        double millivoltsPerBit = millivoltsPerRawValue();

        return channelValue * millivoltsPerBit;
    }
    
    /**
     * Reads the ADS1115 in differential mode between channels 0 and 1
     * @return (ADS channel 0 - ADS channel 1) * gain (raw value)
     */
    int16_t readDifferentialRaw01() {
        muxConfig = (uint16_t) MuxConfig::differential01;
        return currentConfigSingleShotRead();
    }
    
    /**
     * Reads the ADS1115 in differential mode between channels 0 and 3
     * @return (ADS channel 0 - ADS channel 3) * gain (raw value)
     */
    int16_t readDifferentialRaw03() {
        muxConfig = (uint16_t) MuxConfig::differential03;
        return currentConfigSingleShotRead();
    }
    
    /**
     * Reads the ADS1115 in differential mode between channels 1 and 3
     * @return (ADS channel 01- ADS channel 3) * gain (raw value)
     */
    int16_t readDifferentialRaw13() {
        muxConfig = (uint16_t) MuxConfig::differential13;
        return currentConfigSingleShotRead();
    }

    /**
     * Reads the ADS1115 in differential mode between channels 2 and 3
     * @return (ADS channel 2 - ADS channel 3) * gain (raw value)
     */
    int16_t readDifferentialRaw23() {
        muxConfig = (uint16_t) MuxConfig::differential23;
        return currentConfigSingleShotRead();
    }
    
    /**
     * Reads the ADS1115 in differential mode between channels 0 and 1
     * @return (ADS channel 0 - ADS channel 1) * gain (in millivolts)
     */
    double readDifferentialMillivolts01() {
        return millivoltsPerRawValue() * readDifferentialRaw01();
    }

    /**
     * Reads the ADS1115 in differential mode between channels 0 and 3
     * @return (ADS channel 0 - ADS channel 3) * gain (in millivolts)
     */
    double readDifferentialMillivolts03() {
        return millivoltsPerRawValue() * readDifferentialRaw03();
    }

    /**
     * Reads the ADS1115 in differential mode between channels 1 and 3
     * @return (ADS channel 1 - ADS channel 3) * gain (in millivolts)
     */
    double readDifferentialMillivolts13() {
        return millivoltsPerRawValue() * readDifferentialRaw13();
    }

    /**
     * Reads the ADS1115 in differential mode between channels 2 and 3
     * @return (ADS channel 2 - ADS channel 3) * gain (in millivolts)
     */
    double readDifferentialMillivolts23() {
        return millivoltsPerRawValue() * readDifferentialRaw23();
    }
    
    /**
     * Performs a single shot reading on the given [mux] channel
     * Note the value can only be negative when reading a differential channel
     * @param mux The channel (single or differential) to be read from the ADS
     * @return The raw value read from the ADS
     */
    int16_t readRawOnMux(MuxConfig mux) {
        muxConfig = (uint16_t)mux;
        return currentConfigSingleShotRead();
    }

    /**
     * Performs a single shot reading on the given [mux] channel
     * Note the value can only be negative when reading a differential channel
     * The current [gain] is used to tranform the raw value into millivolts
     * @param mux The channel (single or differential) to be read from the ADS
     * @return The value read from the ADS in millivolts
     */
    double readMillivoltsOnMux(MuxConfig mux) {
        return readRawOnMux(mux) * millivoltsPerRawValue();
    }
    
    // TODO: IMPLEMENT COMPARATOR MODE
    
    // MARK: Config setters and getters
    
    /**
     * Sets the gain to be used for readings
     * @param gain The PGA gain 2/3, 1, 2, 4 or 8
     * @param updateConfig if true and in continous conversion mode updates the ADS configuration when running in continous conversion mode (true by default)
     */
    void setGain(AdsGain gain, bool updateConfig = true) {
        this->gain = (uint16_t)gain;
        if (updateConfig && (AdsModeConfig)adsMode == AdsModeConfig::continuousConversion) {
            writeCurrentConfig();
        }
    }

    /** Returns the gain currently used for readings */
    AdsGain getGain() {
        return (AdsGain)gain;
    }

    /**
     * Sets the amount of samples per second the ads should perform
     * Note more samples per second results in lower single shot readings delays, but it might add noise
     * See datasheet table 1 for more information of readings noise
     * Retrieve the delays used for readings from the delayForChannelReading() method
     * @param speed The samples per second to which the ADS is configured
     * @param updateConfig If true and in continous conversion mode writes the config to the ADS  (else if might get written on a future operation)
     */
    void setSampleSpeed(AdsSampleSpeed speed, bool updateConfig = true) {
        sampleSpeed = (uint16_t)speed;
       if (updateConfig && (AdsModeConfig)adsMode == AdsModeConfig::continuousConversion) {
           writeCurrentConfig();
       }
    }

    /** Returns the sample speed currently used for readings */
    AdsSampleSpeed getSampleSpeed() {
        return (AdsSampleSpeed)sampleSpeed;
    }
    
    
    // MARK: Public utilities
    
    
    /**
     * The millivolts / bit for the current gain config (used to determine the voltage)
     * Note this is also known as the device resolution for the current gain config
     * @returns mv / rawValue factor
     */
    double millivoltsPerRawValue() {
        return millivoltsPerRawValue((AdsGain)gain);
    }
    
    /**
     * The millivolts / bit for the given [gain]
     * Note this is also known as the device resolution for the given [gain]
     * @param gain The gain for which the mv/rawValue factor is retrievd
     * @returns mv / rawValue factor
     */
    double millivoltsPerRawValue(AdsGain gain) {
        
        switch ((AdsGain)gain) {

        case AdsGain::twoThirds:
            return 0.1875;
        
        case AdsGain::one:
            return 0.125;
        
        case AdsGain::two:
            return 0.0625;
        
        case AdsGain::four:
            return 0.03215;
        
        case AdsGain::eight:
            return 0.015625;

        case AdsGain::sixteen:
            return 0.0078125;
        
        default:
            return 0;
        }
    }
    
    /** Returns the delay in seconds, for the current single shot channel reading */
    double delayForChannelReading() {
        switch ((AdsSampleSpeed)sampleSpeed) {

            case AdsSampleSpeed::sps8:
                return 0.126;
            
            case AdsSampleSpeed::sps16:
                return 0.064;
            
            case AdsSampleSpeed::sps32:
                return 0.033;

            case AdsSampleSpeed::sps64:
                return 0.017;
            
            case AdsSampleSpeed::sps128:
                return 0.009;
            
            case AdsSampleSpeed::sps250:
                return 0.005;

            case AdsSampleSpeed::sps475:
                return 0.004;

            case AdsSampleSpeed::sps860:
                return 0.003;
            
            default:
                return 0.126; // return the max delay in case the rate is not recognized
        }
    }
};


#endif /* pigpio_ads1115_h */
