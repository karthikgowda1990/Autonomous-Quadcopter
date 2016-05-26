/**
 * @file
 */

#ifndef QUADCOPTER_HPP_
#define QUADCOPTER_HPP_

#include "quadcopter_base.hpp"
#include "lpc_pwm.hpp"



/**
 * The quadcopter class
 *
 * This provides the interface to set the rotor PWMs
 */
class Quadcopter : public QuadcopterBase, public SingletonTemplate<Quadcopter>
{
    public:
        /**
         * @{ Pure virtual method overrides of the MotorControllerIface
         */
        void applyMotorValues(const motorValues_t& values);
        /** @} */

        /* The ESC controllers work for the values in range 10.52 to 18.82
            scaleMotorValues scales values in the proper range
        */
        float scaleMotorValues(const float unScaledValue);

    private:
        /// Private constructor for singleton class
        Quadcopter();

        PWM mNorthMotor;    ///< North motor PWM
        PWM mSouthMotor;    ///< South motor PWM
        PWM mEastMotor;     ///< East motor PWM
        PWM mWestMotor;     ///< West motor PWM

        ///< Friend class used for Singleton Template
        friend class SingletonTemplate<Quadcopter>;

        const float mMinPulseWidth  = 0;
        const float mMaxPulseWidth = 100;
};



#endif /* QUADCOPTER_HPP_ */
