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

    std::unique_ptr<Motor> mLeftMotor;
    std::unique_ptr<Motor> mRightMotor;
    void setMotorSpeed(float aLeftMotorSpeed, float aRightMotorSpeed);

    // Idle will be the min motor output for a given motor to allow for better
    // control
    float mLeftMotorIdle = 6000;
    float mRightMotorIdle = 6000;

};