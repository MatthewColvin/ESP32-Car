#include "car.hpp"
#include "TankMix.hpp"
#include "NoZeroTurnMix.hpp"

#include "esp_log.h"
#include "driver/gpio.h"

#include <cmath>

#define LOG_TAG "Car"
// TODO ADD function that enables motors since driver now has enable pin
#define MotorDriveEnablePin GPIO_NUM_17

using namespace std;

Car::Car(std::shared_ptr<Mocute052> remote, Motor *leftMotor, Motor *rightMotor) : mController(remote),
                                                                                   mRightMotor(rightMotor),
                                                                                   mLeftMotor(leftMotor),
                                                                                   mMotorMixer(std::make_unique<TankMix>(mController)),
                                                                                   mHandling(Car::Handling::Tank)
{
    xTaskCreate(this->mixerPollingImpl, "CarMixingPoll", 2048, this, 5, NULL);

    // Configure Motor Pin to allow control of motor drivers
    gpio_config_t motorSleepPinConfig;
    motorSleepPinConfig.pin_bit_mask = 1ULL << MotorDriveEnablePin;
    motorSleepPinConfig.mode = GPIO_MODE_OUTPUT;
    motorSleepPinConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    motorSleepPinConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    motorSleepPinConfig.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&motorSleepPinConfig);

    enableMotors();
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
    // ESP_LOGI(LOG_TAG,"Left:%f right:%f", aLeftMotorSpeed, aRightMotorSpeed);
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
        // ESP_LOGI(LOG_TAG, "leftspeed:%f rightspeed:%f", currentSpeeds.left, currentSpeeds.right);
        setMotorSpeed(currentSpeeds.left, currentSpeeds.right);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void Car::setHandling(Car::Handling aHandling)
{
    mHandling = aHandling;
    switch (mHandling)
    {
    case Car::Handling::Tank:
        mMotorMixer = std::make_unique<TankMix>(mController);
        break;
    case Car::Handling::Car:
        mMotorMixer = std::make_unique<NoZeroTurnMix>(mController);
        break;
    }
}

void Car::enableMotors(){
    gpio_set_level(MotorDriveEnablePin, 1);
}

void Car::disableMotors(){
    gpio_set_level(MotorDriveEnablePin, 0);
}