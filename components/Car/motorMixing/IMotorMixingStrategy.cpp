#include "IMotorMixingStrategy.hpp"
#include "motor.hpp"

#define LOG_TAG "IMotorMixingStrategy"

IMotorMixingStrategy::IMotorMixingStrategy(std::shared_ptr<Mocute052> aController)
{
    aController->onJoyStick(std::bind(&IMotorMixingStrategy::controllerInputHandler, this, std::placeholders::_1, std::placeholders::_2));
};

void IMotorMixingStrategy::setCruiseSpeed(float aCruiseSpeed)
{
    if (0 < aCruiseSpeed && aCruiseSpeed < Motor::MAX_SPEED)
    {
        mCruiseSpeed = aCruiseSpeed;
    }
    else
    {
        ESP_LOGE(LOG_TAG, "ERROR: Cannot set Curise Speed to %f range is (%f to %f)", aCruiseSpeed, 0.0, Motor::MAX_SPEED);
    }
}

float IMotorMixingStrategy::mapValues(float value, float aMin, float aMax, float aTargetMin, float aTargetMax)
{
    float span = aMax - aMin;
    float targetSpan = aTargetMax - aTargetMin;
    float scaled = (value - aMin) / span;
    return aTargetMin + (scaled * targetSpan);
}

void IMotorMixingStrategy::controllerInputHandler(uint8_t aX, uint8_t aY)
{
    // convert uint8 given by controller to floats and
    // correct ranges so 0,0 is the middle of the control range
    int refX = aX - Mocute052::MAX_XY;
    int refY = -1 * (aY - 128);

    mX = refX;
    mY = refY;
};

IMotorMixingStrategy::speeds IMotorMixingStrategy::validateSpeeds(IMotorMixingStrategy::speeds aSpeedToValidate)
{
    speeds retSpeeds = aSpeedToValidate;
    if (aSpeedToValidate.right > Mocute052::MAX_XY)
    {
        ESP_LOGE(LOG_TAG, "ERROR: Cannot set right motor speed to %f max is %f", aSpeedToValidate.right, Mocute052::MAX_XY);
        retSpeeds.right = Mocute052::MAX_XY;
    }
    else if (aSpeedToValidate.right < Mocute052::MIN_XY)
    {
        ESP_LOGE(LOG_TAG, "ERROR: Cannot set right motor speed to %f min is %f", aSpeedToValidate.right, Mocute052::MIN_XY);
        retSpeeds.right = Mocute052::MIN_XY;
    }
    else
    {
        retSpeeds.right = aSpeedToValidate.right;
    }

    if (aSpeedToValidate.left > Mocute052::MAX_XY)
    {
        ESP_LOGE(LOG_TAG, "ERROR: Cannot set left motor speed to %f max is %f", aSpeedToValidate.left, Mocute052::MAX_XY);
        retSpeeds.left = Mocute052::MAX_XY;
    }
    else if (aSpeedToValidate.left < Mocute052::MIN_XY)
    {
        ESP_LOGE(LOG_TAG, "ERROR: Cannot set left motor speed to %f min is %f", aSpeedToValidate.left, Mocute052::MIN_XY);
        retSpeeds.left = Mocute052::MIN_XY;
    }
    else
    {
        retSpeeds.left = aSpeedToValidate.left;
    }
    return retSpeeds;
}

IMotorMixingStrategy::motorSpeeds IMotorMixingStrategy::getMotorSpeeds()
{
    IMotorMixingStrategy::speeds speeds = validateSpeeds(mixAndGetSpeeds());

    motorSpeeds returnSpeeds;
    float currentFastestSpeed = mIsTurboEnabled ? mMaxSpeed : mCruiseSpeed;
    // Convert speed from controller based units to motor based units
    if (speeds.left > 0)
    {
        returnSpeeds.left = mapValues(speeds.left, 0, Mocute052::MAX_XY, 0, currentFastestSpeed);
    }
    else
    {
        returnSpeeds.left = mapValues(speeds.left, Mocute052::MIN_XY, 0, -1 * currentFastestSpeed, 0);
    }
    if (speeds.right > 0)
    {
        returnSpeeds.right = mapValues(speeds.right, 0, Mocute052::MAX_XY, 0, currentFastestSpeed);
    }
    else
    {
        returnSpeeds.right = mapValues(speeds.right, Mocute052::MIN_XY, 0, -1 * currentFastestSpeed, 0);
    }

    return returnSpeeds;
}