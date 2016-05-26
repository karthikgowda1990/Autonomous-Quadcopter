/**
 *
 */

#include "quadcopter.hpp"
#include "utilities.h"

/// The frequency of the ESC (electronic speed controller)
#define ESC_FREQUENCY_HZ    400   //TODO bhushan change needed before 100/400
float north_motor_value;
float south_motor_value;
float east_motor_value;
float west_motor_value;


Quadcopter::Quadcopter() :
    mNorthMotor(PWM::pwm1, ESC_FREQUENCY_HZ),
    mSouthMotor(PWM::pwm2, ESC_FREQUENCY_HZ),
    mEastMotor (PWM::pwm3, ESC_FREQUENCY_HZ),
    mWestMotor (PWM::pwm4, ESC_FREQUENCY_HZ)
{
    /* Nothing to do */
    mNorthMotor.set(mMinPulseWidth);
    mSouthMotor.set(mMinPulseWidth);
    mEastMotor.set(mMinPulseWidth);
    mWestMotor.set(mMinPulseWidth);
}

// Virtual method implementation
void Quadcopter::applyMotorValues(const motorValues_t& values)
{
    mNorthMotor.set(scaleMotorValues(values.north));
    mSouthMotor.set(scaleMotorValues(values.south));
    mEastMotor.set(scaleMotorValues(values.east));
    mWestMotor.set(scaleMotorValues(values.west));
    north_motor_value = scaleMotorValues(values.north);
    south_motor_value = scaleMotorValues(values.south);
    east_motor_value = scaleMotorValues(values.east);
    west_motor_value = scaleMotorValues(values.west);

}


/*float Quadcopter::scaleMotorValues(const float unScaledValue)
{
    const float scalingFactor = 12.04;
    const float minDutyCycle = 10.52;
    float scaledValue;

    scaledValue = unScaledValue/scalingFactor + minDutyCycle;

    if(scaledValue<0)
        scaledValue=minDutyCycle;

    return scaledValue;
}*/

float Quadcopter::scaleMotorValues(float percent)
{
    if (percent < mMinPulseWidth) {
        percent = mMinPulseWidth;
    }
    else if (percent > mMaxPulseWidth) {
        percent = mMaxPulseWidth;
    }
    /**
     *  Pulse Width range is 40-80 @ 400Hz
     *  0% input   = 40 pulse output
     *  100% input = 80 pulse output
     *  But because of our stronger 2400kv motors, let's use 3/4 of the strength
     *  and output only 40-70 pulses
     */
    percent = (percent * 0.3) + 40;

    return percent;
}
