/**
 * @file
 */
#include <stdint.h>

#include "scheduler_task.hpp"
#include "quadcopter.hpp"

#include "sampler.hpp"
#include "uart_dev.hpp"

#include "soft_timer.hpp"
#include "three_axis_sensor.hpp"
#include "sensor_system.hpp"



/**
 * This is the Quadcopter OS task.
 * This processes the raw sensor values through various filters, and applies
 * the flight controller inputs to fly the Quadcopter.
 *
 * @ingroup Quadcopter Tasks
 */
class quadcopter_task : public scheduler_task
{
    public:
        quadcopter_task(const uint8_t priority);
        bool init(void);
        bool regTlm(void);
        bool taskEntry(void);
        bool run(void *p);

    protected:
    private:
        quadcopter_task(); ///< Disallow default constructor (no code is defined)

        /// Detects and logs timing skew information
        void detectTimingSkew(const uint32_t millis);

        /// Updates the status LEDs of the Quadcopter
        void updateStatusLeds(void);

        /// Instance of the Quadcopter
        Quadcopter &mQuadcopter;

        /// Instance of the Sensor System
        SensorSystem mSensorSystem;

        /**
         * The trigger point of "low" battery.
         * This is saved on the disk, and set on the Quadcopter class
         */
        uint8_t mLowBatteryTriggerPercent;

        /// Tracks highest micro-seconds spent in the run() method
        uint16_t mHighestLoopTimeUs;

        /// The timestamp of last call to the run() method
        uint32_t mLastCallMs;

        /// Time at which point we will update the PID and propeller values
        uint32_t mLastPidUpdateTimeMs;

        /// Frequency at which sensor inputs will be obtained to calculate pitch, roll, and yaw
        const uint32_t mSensorPollFrequencyMs;

        /// Frequency at which propellers (ESCs) will be updated
        const uint32_t mEscUpdateFrequencyMs;

        /** @{ Counters to print number of sample outputs, these can be modified through telemetry */
        uint16_t mPrintYpr, mPrintAcc, mPrintGyro, mPrintMagno;
        /** @} */
};



/**
 * GPS task.
 * The objective of this task is to read the GPS data, parse it, and set
 * it on the Quadcopter class.
 *
 * @ingroup Quadcopter Tasks
 */
class gps_task : public scheduler_task
{
    public:
        /**
         * Constructor of the task.
         * @param [in] pGpsUart     The UART pointer connected to the GPS
         * @param [in] priority     The priority of this task
         *
         * @note pGpsUart's UART should already be initialized at the GPS baudrate
         */
        gps_task(const uint8_t priority, UartDev *pGpsUart);

        bool init(void);
        bool run(void *p);

    protected:
    private:
        gps_task(); ///< Disallow default constructor (no code is defined)

        UartDev *mpGpsUart;     ///< The UART used for the GPS
};



/**
 * This is the RC remote receiver OS task.
 * The objective is to decoded/read the input values of the RC receiver
 * and set the values to the flight controller class.
 *
 * @ingroup Quadcopter Tasks
 */
class rc_remote_task : public scheduler_task
{
    public:
        rc_remote_task(const uint8_t priority);
        bool init(void);
        bool run(void *p);

    protected:
    private:
        rc_remote_task(); ///< Disallow default constructor (no code is defined)

        /**
         * Gets the normalized value between -100 -> +100 based on the input pulse width.
         * @note mscMaxPulseWidthUs is used to calculate the range.
         *
         * @param [in] pulseWidthUs     The pulse width of the RC channel in microseconds
         */
        int32_t getNormalizedValue(const uint32_t &pulseWidthUs, const uint16_t scalingFactor, const uint16_t subtractingFactor, bool throttle);

        /// Instance of the quadcopter
        Quadcopter &mQuadcopter;

        /// The flight parameters being decoded from RC receiver
        Quadcopter::flightParams_t mRcRxFlightCmd;

        /// The maximum pulse width of a single channel in microseconds
        static const uint32_t mscMaxPulseWidthUs = 2 * 1000;


        /*
         * Scaling and Subtracting Factors determined from
         * formula = (scalingFactor * Time) / Period). Scaling
         * factor is increased until range is 200 for pitch & roll
         * and range is 100 for throttle. Subtracting factor
         * adjusts the minimum and maximum values to -100->+100
         * for pitch & roll and 0->+100 for throttle.
         * Throttle: Minimum pulseWidthUs = 1080us
         *           Maximum pulseWidthUs = 1840us
         * Throttle: Minimum pulseWidthUs = 1120us
         *           Maximum pulseWidthUs = 1840us
         * Throttle: Minimum pulseWidthUs = 1040us
         *           Maximum pulseWidthUs = 1840us
         */
        const uint16_t throttleScalingFactor = 250;//276;
        const uint16_t throttleSubtractingFactor = 130;//143;
        const uint16_t pitchScalingFactor = 170;//150;//525;
        const uint16_t pitchSubtractingFactor = 118;//108;//371;
        const uint16_t rollScalingFactor = 167;//150;//584;
        const uint16_t rollSubtractingFactor = 124;//108;//432;
        const uint16_t yawScalingFactor = 140;//150;//584;
        const uint16_t yawSubtractingFactor = 105;//108;//432;
        const uint16_t armScalingFactor = 525;
        const uint16_t armSubtractingFactor = 371;
};



/**
 * This is the battery monitor task.
 * This monitors the battery voltage and sets the value to the quadcopter task.
 *
 * @ingroup Quadcopter Tasks
 */
class battery_monitor_task : public scheduler_task
{
    public:
        battery_monitor_task(const uint8_t priority);
        bool init(void);
        bool run(void *p);

    protected:
    private:
        battery_monitor_task();         ///< Disallow default constructor (no code is defined)
        int32_t mLowestMilliVolts;      ///< Lowest battery voltage
        int32_t mHighestMilliVolts;     ///< Highest battery voltage
        int32_t mMilliVoltDeltaToLog;   ///< Data is logged if previous voltage delta is larger than this
        int32_t mPrevMilliVolts;        ///< Previous voltage sensed when data was logged

        /**
         * When the system is "learning", it needs to know a large enough delta for the
         * battery percentage to be determined correctly.  For example, we need to know
         * that a battery has been through 1.2v - 1.6v, and only then we can compute a
         * valid percentage of the battery.
         */
        int32_t mMinimumDeltaMilliVoltsForValidPercent;

        /**
         * We take multiple ADC samples before we take the average.
         * Note that we need uint32_t rather than the adequate uint16_t for 12-bit ADC
         * because when we sum and average using this class, it may overflow.
         */
        Sampler<uint32_t> mAdcSamples;

        /// The frequency at which we collect the samples in milliseconds
        static const int mscSampleFrequencyMs = 250;

        /**
         * The number of samples to take before we average it and use it to compute the voltage
         * 250 * 12 = 3 seconds
         */
        static const int mscNumAdcSamplesBeforeVoltageUpdate = 12;
};



/**
 * This is the "kill switch" task
 *
 * @ingroup Quadcopter Tasks
 */
class kill_switch_task : public scheduler_task
{
    public:
        kill_switch_task(const uint8_t priority);
        bool init(void);
        bool run(void *p);

    protected:
    private:
        kill_switch_task(); ///< Disallow default constructor (no code is defined)

        /**
         * The wireless command types (first byte of a wireless packet)
         */
        typedef enum {
            wscmd_kill    = 0,
            wscmd_arm     = 1,
            wscmd_disarm  = 2,
        } wirelessCmd_t;
};
