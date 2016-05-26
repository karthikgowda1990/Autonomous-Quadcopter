/**
 * @file
 */

#include <string.h>
#include <stdlib.h>
#include "pid.hpp"
#include "flight_stabilizer.hpp"

#define PID_CONTROL_LOOP_FREQUENCY      (400)   ///< Frequency of the PID loop
#define AGGRESSIVE_PID_MODE_SCALE       (2.5f)     ///< Scaling for aggressive pid mode
#define AGGRESSIVEMODE_ON               0       ///< Comment this out to disable the aggressive mode

PID::PID() :
        aggressive_pid_scale(AGGRESSIVE_PID_MODE_SCALE),
        mPidOutput(0),
        mPidSetpoint(0),
        mLastTimeMs(0),
        //mSampleTimeMs((float)1000 / PID_CONTROL_LOOP_FREQUENCY),
        mIntegralTerm(0),
        mLastInput(0),
        mPidProcessingIsOn(false),
        mPidControllerDirection(pid_direction_positive),
        mOutputMin(0),
        mOutputMax(0)
{
    mSampleTimeMs = (float)1000 / PID_CONTROL_LOOP_FREQUENCY;
    memset(&mPidParams, 0, sizeof(mPidParams));
}

float PID::compute(const float setpointValue, const float presentInputValue, const uint32_t timeNowMs)
{
    mSampleTimeMs = (float) 1000 / PID_CONTROL_LOOP_FREQUENCY;
    /* If PID processing is not ON, we shouldn't update the PID state */
    if (!mPidProcessingIsOn)
    {
        return mPidOutput;
    }

    // Check for aggressive/conservative tuning mode and set the PID params to be used for computing
    setPIDTuningMode(setpointValue, presentInputValue);

    /* Only process the PID loop if enough time has elapsed since last computation.
     * The exception is when the user forces the PID to process its loop.
     */
    if ((timeNowMs - mLastTimeMs) >= mSampleTimeMs)
    {
        mPidSetpoint = setpointValue;

        /* Compute all the working error variables */
        const float error = mPidSetpoint - presentInputValue;
#if AGGRESSIVEMODE_ON
        if (CONSERVATIVE == activePIDtuningMode) {
            mIntegralTerm += (mPidParams.ki * error);
        }
        else {
            mIntegralTerm += (mPidParams.aggrki * error);
        }
#else
        mIntegralTerm += (mPidParams.ki * error);
#endif

        /* Cap the integral term to avoid PID from using unusable values.
         * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-reset-windup/
         */
        if (mIntegralTerm > mOutputMax) {
            mIntegralTerm = mOutputMax;
        }
        else if (mIntegralTerm < mOutputMin) {
            mIntegralTerm = mOutputMin;
        }

        /* Avoid the derivative kick:
         * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
         */
        const float dInput = (presentInputValue - mLastInput);
        //const float dInput = (error - mLastInput);

        /* Compute PID Output */
#if AGGRESSIVEMODE_ON
        if (CONSERVATIVE == activePIDtuningMode) {
            mPidOutput = (mPidParams.kp * error) + mIntegralTerm - (mPidParams.kd * dInput);                  }
        else {
            mPidOutput = (mPidParams.aggrkp * error) + mIntegralTerm - (mPidParams.aggrkd * dInput);
        }
#else
        mPidOutput = (mPidParams.kp * error) + mIntegralTerm - (mPidParams.kd * dInput);

#endif

        /* Cap the PID from using unusable values */
        if (mPidOutput > mOutputMax) {
            mPidOutput = mOutputMax;
        }
        else if (mPidOutput < mOutputMin) {
            mPidOutput = mOutputMin;
        }

        /* Remember some variables for next time */
        mLastInput = presentInputValue;
        //mLastInput = error;
        mLastTimeMs = timeNowMs;
    }

    return mPidOutput;
}

void PID::setPidParameters(const pidParams_t& params)
{
    if (params.kp < 0 || params.ki < 0 || params.kd < 0) {
        return;
    }

    mSampleTimeMs = (float) 1000 / PID_CONTROL_LOOP_FREQUENCY;
    const float SampleTimeInSec = ((float)mSampleTimeMs) / 1000;
    mPidParams.kp = params.kp;
    mPidParams.ki = params.ki * SampleTimeInSec;
    mPidParams.kd = params.kd / SampleTimeInSec;

    // Set the aggressive PID parameters
    mPidParams.aggrkp = mPidParams.kp * aggressive_pid_scale;
    mPidParams.aggrki = mPidParams.ki * aggressive_pid_scale;
    mPidParams.aggrkd = mPidParams.kd * aggressive_pid_scale;

    /* Negative the PID parameters if the direction is negative
     * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/
     */
    if (pid_direction_negative == mPidControllerDirection)
    {
        mPidParams.kp = (0 - mPidParams.kp);
        mPidParams.ki = (0 - mPidParams.ki);
        mPidParams.kd = (0 - mPidParams.kd);
        mPidParams.aggrkp = (0 - mPidParams.aggrkp);
        mPidParams.aggrki = (0 - mPidParams.aggrki);
        mPidParams.aggrkd = (0 - mPidParams.aggrkd);
    }
}

void PID::setSampleTime(const uint32_t newSampleTimeMs)
{
    mSampleTimeMs = (float) 1000 / PID_CONTROL_LOOP_FREQUENCY;
    if (newSampleTimeMs > 0)
    {
        float ratio = (float) newSampleTimeMs / (float) mSampleTimeMs;
        mPidParams.ki *= ratio;
        mPidParams.aggrki *= ratio;
        mPidParams.kd /= ratio;
        mPidParams.aggrkd /= ratio;
        mSampleTimeMs = (int) newSampleTimeMs;
    }
}

void PID::setOutputLimits(float min, float max)
{
    if (min > max) {
        return;
    }

    mOutputMin = min;
    mOutputMax = max;

    if (mPidOutput > mOutputMax) {
        mPidOutput = mOutputMax;
    }
    else if (mPidOutput < mOutputMin) {
        mPidOutput = mOutputMin;
    }

    if (mIntegralTerm > mOutputMax) {
        mIntegralTerm = mOutputMax;
    }
    else if (mIntegralTerm < mOutputMin) {
        mIntegralTerm = mOutputMin;
    }
}

void PID::setMode(pidMode_t mode, float latestInput)
{
    bool newAuto = (mode == pid_automatic);

    /* If we just went from pid_manual to auto */
    if (newAuto == !mPidProcessingIsOn)
    {
        init(latestInput);
    }

    mPidProcessingIsOn = newAuto;
}

void PID::init(float latestInput)
{
    mLastInput = latestInput;
    mIntegralTerm = mPidOutput;

    if (mIntegralTerm > mOutputMax) {
        mIntegralTerm = mOutputMax;
    }
    else if (mIntegralTerm < mOutputMin) {
        mIntegralTerm = mOutputMin;
    }
}

void PID::setPidDirection(pidDirection_t pidDirection)
{
    mPidControllerDirection = pidDirection;
}

void PID::setPIDTuningMode(const float setpointValue, const float presentInputValue)
{
    if ((activePIDtuningMode == CONSERVATIVE) && (abs((int)presentInputValue) >= (abs((int)setpointValue) + mAngleLowerLimit)))
     {
        activePIDtuningMode = AGGRESSIVE;
    }
    else if ((activePIDtuningMode == AGGRESSIVE) && (abs((int)presentInputValue) < (abs((int)setpointValue) + mAngleLowerLimit)))
    {
        activePIDtuningMode = CONSERVATIVE;
    }
}
