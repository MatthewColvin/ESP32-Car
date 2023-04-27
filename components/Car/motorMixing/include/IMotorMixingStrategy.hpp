#pragma once
#include "mocute052.hpp"
#include "motor.hpp"
#include <memory>
#include <functional>

class IMotorMixingStrategy
{
public:
    /**
     * @brief This should be called by any child classes so that way
     *        mX and mY are updated by the Mocute052 controller.
     *
     * @param aController - mocute 052 controller
     */
    IMotorMixingStrategy(std::shared_ptr<Mocute052> aController);

    /**
     * @brief Structure to hold motor speeds vaild
     *        values from Motor::MIN_SPEED to Motor::MAX_SPEED
     */
    struct motorSpeeds
    {
        float left;
        float right;
    };

    motorSpeeds getMotorSpeeds();

    void enableTurbo() { mIsTurboEnabled = true; };
    void disableTurbo() { mIsTurboEnabled = false; };

    float getCruiseSpeed() { return mCruiseSpeed; };
    void setCruiseSpeed(float aCruiseSpeed);

protected:
    /**
     * @brief Struct to hold speed in terms of the controller.
     */
    struct speeds
    {
        float left;
        float right;
    };

    /**
     * @brief Takes speeds and validates its in the proper ranges
     *
     * @param aSpeedToValidate - input to validate
     * @return speeds - a corrected speed if a value was to high or low it sets it to max/min extents
     */
    speeds validateSpeeds(speeds aSpeedToValidate);

    /**
     * @brief Main algorithm of strategy pattern to implement.
     *        Use the given mX and mY to generate a speeds struct.
     *        These speeds should be in terms of the controller so
     *           max value: 127
     *           min value: -128
     *
     * @return motorSpeeds
     */
    virtual speeds mixAndGetSpeeds() = 0;

    /**
     * @brief Max speed of any motor while turbo is false
     */
    float mCruiseSpeed = Motor::MAX_SPEED * 0.50;
    /**
     * @brief Max speed of any motor while turbo is true
     */
    float mMaxSpeed = Motor::MAX_SPEED;
    bool mIsTurboEnabled = false;

private:
    /**
     * @brief Set the given input from the controller into protected
     *        members
     *
     * @param aX - X value between -128 and 127
     * @param aY - Y value between -128 and 127
     */
    void controllerInputHandler(uint8_t aX, uint8_t aY);
    /**
     * @brief Map value from range (aMin-aMax) into (aTargetMin-aTargetMax)
     */
    float mapValues(float value, float aMin, float aMax, float aTargetMin, float aTargetMax);

protected:
    // Current Control input from the controller in (-128 to 127 )
    float mX = 0;
    float mY = 0;
};