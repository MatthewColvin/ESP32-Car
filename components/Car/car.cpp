#include "car.hpp"
#include "TankMix.hpp"

#include "esp_log.h"

#include <cmath>

#define LOG_TAG "Car"

using namespace std;

Car::Car(std::shared_ptr<Mocute052> remote, Motor* leftMotor, Motor* rightMotor) : mRightMotor(rightMotor),
                                                                                   mLeftMotor(leftMotor),
                                                                                   mMotorMixer(std::make_unique<TankMix>(remote))
{
    xTaskCreate(this->mixerPollingImpl, "CarMixingPoll", 2048, this, 5, NULL);
}

void Car::setMotorSpeed(float aLeftMotorSpeed, float aRightMotorSpeed)
{
    if (aLeftMotorSpeed > 0)
    {
        mLeftMotor->forward();
    }
    else
    {
        mLeftMotor->reverse();
    }
    mLeftMotor->setSpeed(std::floor(std::abs(aLeftMotorSpeed)));

    if (aRightMotorSpeed > 0)
    {
        mRightMotor->forward();
    }
    else
    {
        mRightMotor->reverse();
    }
    mRightMotor->setSpeed(std::floor(std::abs(aRightMotorSpeed)));
}

void Car::mixerPollingImpl(void *_thisCar)
{
    reinterpret_cast<Car *>(_thisCar)->mixerPollingTask();
}

void Car::mixerPollingTask()
{
    while (true)
    {
        IMotorMixingStrategy::motorSpeeds currentSpeeds = mMotorMixer->getMotorSpeeds();
        setMotorSpeed(currentSpeeds.left, currentSpeeds.right);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
