/**
 *
 */

#include <string.h>
#include "flight_stabilizer.hpp"
#include "file_logger.h"
#include "ahrs.hpp"
#include <stdio.h>
#include "utilities.h"
#include "uart2.hpp"
#include "tasks.hpp"
#include "shared_handles.h"
#include "math.h"


bool once_flag = false;


FlightStabilizer::FlightStabilizer() :
    mArmed(false),
    mLogFrequencyMs(0)
{
    mFlightControllerAngles = 0;
    mCurrentAngles = 0;
    pidPitchThrottle = 0;  // for telemetry
    pidRollThrottle = 0; // for telemet
    pidYawThrottle = 0;
    ypr[0] = 0 ; ypr[1] = 0; ypr[2] = 0;
   // calibration[0]=0; calibration[1] =0; calibration[2]=0;
    //oldyaw = 0;
    min=-100.f;
    max=-100.f;
}

void FlightStabilizer::setCommonPidParameters(float minOutputValue, float maxOutputValue, uint32_t pidUpdateTimeMs)
{
    mPitchPid.setOutputLimits(minOutputValue, maxOutputValue);
    mRollPid.setOutputLimits(minOutputValue, maxOutputValue);
    mYawPid.setOutputLimits(minOutputValue, maxOutputValue);

    mPitchPid.setSampleTime(pidUpdateTimeMs);
    mRollPid.setSampleTime(pidUpdateTimeMs);
    mYawPid.setSampleTime(pidUpdateTimeMs);
}

void FlightStabilizer::setArmed(bool armed)
{
    mArmed = armed;

    /* When we are suddenly "ARMED", we don't want our PID to spike its output, so
     * we politely turn it on by using the latest value
     */
    const PID::pidMode_t pidMode = mArmed ? PID::pid_automatic : PID::pid_manual;

    mPitchPid.setMode(pidMode, mCurrentAngles.pitch);
    mRollPid.setMode(pidMode, mCurrentAngles.roll);
    mYawPid.setMode(pidMode, mCurrentAngles.yaw);
}

void FlightStabilizer::enablePidIoLogging(uint32_t frequencyMs)
{
    mLogFrequencyMs = frequencyMs;
}

void FlightStabilizer::computePitchRollYawValues(const uint32_t loopTimeMs, iMagnoIface& magno, iAcceleroIface &acc, iGyroIface &gyro)
{
    //Uart2& u2=Uart2::getInstance();
    threeAxisVector_t m = magno.getMagnoData();
    threeAxisVector_t a = acc.getAcceleroData();
    threeAxisVector_t g = gyro.getGyroAngularData();

    //static const float yaw_scale_offset = 100.0f;
    printf("%.2f %.2f\n", max, min);
    //min= max= oldval;
    AHRSupdate(g.x, g.y, g.z, a.x, a.y, a.z, m.x, m.y, m.z, loopTimeMs);

    getYawPitchRoll(ypr); //new val

    //getEuler(ypr);

    /*
    ypr[0] = ypr[0] - calibration[0];
    ypr[1] = ypr[1] - calibration[1];
    ypr[2] = ypr[2] - calibration[2];*/


    /*
    ypr[0] += ((int)(oldyaw/360))*360;
    float tmpyaw;
    (ypr[0] > oldyaw) ?tmpyaw = (ypr[0] - 360) : tmpyaw = (ypr[0] + 360);
    if(fabs(oldyaw - tmpyaw) < fabs(oldyaw - ypr[0]))
        ypr[0] = tmpyaw;

    oldyaw = ypr[0];
    */

    mCurrentAngles.yaw = ypr[0];
    /*if(mCurrentAngles.yaw>max)
    {max=mCurrentAngles.yaw;}
  //  else max = oldval;

     if(mCurrentAngles.yaw<min)
    {min = mCurrentAngles.yaw;}
    // else min = oldval;*/

    mCurrentAngles.pitch = ypr[1]; //if incorrect remove calibrate!
    mCurrentAngles.roll = ypr[2]; //if incorrect remove calibrate!
    //u2.printf("%.2f %.2f %.2f\n", mCurrentAngles.yaw, mCurrentAngles.pitch, mCurrentAngles.roll);
    //printf("%.2f %.2f %.2f\n", mCurrentAngles.yaw, mCurrentAngles.pitch, mCurrentAngles.roll);

    //oldval= ypr[0];
}

void FlightStabilizer::computeThrottleValues(const uint32_t timeNowMs)
{
    MotorControllerIface::motorValues_t values;
    //float throttle = (float) mFlightControllerAngles.throttle; // TODO: these were const variables
    uint8_t throttle = mFlightControllerAngles.throttle; // TODO: these were const variables
    /* Get the PRY values we need through the PID */
    float pitchThrottle = mPitchPid.compute(mFlightControllerAngles.angle.pitch, mCurrentAngles.pitch, timeNowMs);
    float rollThrottle = mRollPid.compute(mFlightControllerAngles.angle.roll, mCurrentAngles.roll, timeNowMs);
    float yawThrottle = mYawPid.compute(mFlightControllerAngles.angle.yaw, mCurrentAngles.yaw, timeNowMs);

    pidPitchThrottle = pitchThrottle;  // for telemetry
    pidRollThrottle = rollThrottle; // for telemet
    pidYawThrottle = yawThrottle;

    /* Do not log the data at frequency higher than mLogFrequencyMs */
    static uint32_t lastTimeMs = 0;
    if (0 != mLogFrequencyMs && (timeNowMs - lastTimeMs) >= mLogFrequencyMs)
    {
        /* Maintain frequency.  So if caller rate is every 4ms, and frequency is 10ms, then
         * we want to log the message at 12, 20, 32, 40 ms etc (about every 10ms)
         *
         * 4 -->  8 --> 12 = LOG
         *       16 --> 20 = LOG
         * 24 -> 28 --> 32 = LOG
         *       36 --> 40 = LOG
         */
        lastTimeMs = timeNowMs - (timeNowMs % mLogFrequencyMs);

/*        LOG_SIMPLE_MSG("%i,%i,%.1f,%i,%i,%.1f,%i,%i,%.1f",
                (int)mFlightControllerAngles.angle.pitch, (int)mCurrentAngles.pitch, pitchThrottle,
                (int)mFlightControllerAngles.angle.roll, (int)mCurrentAngles.roll, rollThrottle),
                (int)mFlightControllerAngles.angle.yaw, (int)mCurrentAngles.yaw, yawThrottle);*/
    }

    // Calibration
    if (throttle == 0)
    {
        values.east=0;
        values.west=0;
        values.north=0;
        values.south=0;
        pitchThrottle = 0;
        rollThrottle = 0;
        yawThrottle = 0;

    }
    else
    {
        /* Set the motor values that control the pitch
         * For example, if the desired pitch angle is 15deg (nose up), and actual is zero, then the
         * pitchThrottle value will be positive, and so we need to increase north motor and decrease
         * the south motor.
         */
        values.north = throttle + pitchThrottle + yawThrottle;
        values.south = throttle - pitchThrottle + yawThrottle;

/*        values.north = throttle + pitchThrottle;
        values.south = throttle - pitchThrottle;*/

        /* Set the motor values that control the roll
         * For example, if desired pitch angle is 15deg (to the right), and actual is zero, then the
         * rollThrottle value will be positive, and so we need to increase the west motor and decrease
         * the east motor.
         */
        values.east = throttle - rollThrottle - yawThrottle;
        values.west = throttle + rollThrottle - yawThrottle;
/*        values.east = 0;
        values.west = 0;*/

/*        values.north = throttle + pitchThrottle + rollThrottle + yawThrottle;
        values.east  = throttle + pitchThrottle - rollThrottle - yawThrottle;
        values.south = throttle - pitchThrottle - rollThrottle + yawThrottle;
        values.west  = throttle - pitchThrottle + rollThrottle - yawThrottle;*/
    }
    saveMotorValues(values);
}

void FlightStabilizer::applyThrottleValues(void)
{
    MotorControllerIface::motorValues_t values = { 0 };

    /* If we are armed, only then retrieve the values saved to the motor controller
     * interface through the saveMotorvalues()
     */

    if (mArmed) {
        values = getMotorValues();
    }

    applyMotorValues(values);
}


/*
void FlightStabilizer::zeroIMU()
{
    getYawPitchRoll(calibration);
    oldyaw = ypr[0];
    printf("calibration yaw val %f\n ", oldyaw);
}

void FlightStabilizer::zeroYaw()
{
    float vals[3];
    getYawPitchRoll(vals);
    calibration[0] = vals[0];
}
*/
