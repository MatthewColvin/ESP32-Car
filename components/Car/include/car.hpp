#pragma once

#include "motor.hpp"
#include "mocute052.hpp"
#include <memory>

class Car
{
public:
    Car(std::shared_ptr<Mocute052> remote, std::unique_ptr<Motor> leftMotor, std::unique_ptr<Motor> rightMotor);

private:
    void ControllerInputHandler(uint8_t x, uint8_t y);
    void enableTurbo(){mMaxSpeed = Motor::MAX_SPEED;};
    void disableTurbo(){mMaxSpeed = Motor::MAX_SPEED * 0.60;};

    std::unique_ptr<Motor> mLeftMotor;
    std::unique_ptr<Motor> mRightMotor;
    void setMotorSpeed(float aLeftMotorSpeed, float aRightMotorSpeed);

    float mMaxSpeed = Motor::MAX_SPEED * 0.60;

    float mSlopeValue = 1.0;
    float mFadingValue = 1.0;
};