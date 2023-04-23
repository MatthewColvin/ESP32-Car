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

    void IncreaseFadingValue() {mFadingValue += 1;}
    void DecreaseFadingValue() {if (mFadingValue > 1) {mFadingValue -= 1;}}
    void IncreaseSlopeValue() {mSlopeValue += 1;}
    void DecreaseSlopeValue() {if (mSlopeValue > 1) {mSlopeValue -= 1;}}

    std::unique_ptr<Motor> mLeftMotor;
    std::unique_ptr<Motor> mRightMotor;
    void setMotorSpeed(float aLeftMotorSpeed, float aRightMotorSpeed);

    // Idle will be the min motor output for a given motor to allow for better
    // control
    float mLeftMotorIdle = 6000;
    float mRightMotorIdle = 6000;

    float mSlopeValue = 1.0;
    float mFadingValue = 1.0;
};