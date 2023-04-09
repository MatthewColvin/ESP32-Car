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
};