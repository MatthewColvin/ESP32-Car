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

    void enableTurbo(){mIsTurboEnabled = true; ConvertAndUpdateSpeed();};
    void disableTurbo(){mIsTurboEnabled = false; ConvertAndUpdateSpeed();};
    bool mIsTurboEnabled = false;


    /**
     * @brief Set the speed for each motor in the ranges motors accept.
     *
     * @param aLeftMotorSpeed
     * @param aRightMotorSpeed
     */
    void setMotorSpeed(float aLeftMotorSpeed, float aRightMotorSpeed);

    /**
     * @brief Convert mRightMotorSpeed and mLeftMotorSpeed from controller
     *        ranges into a range that is suited for the motors.
     */
    void ConvertAndUpdateSpeed();

    /**
     * @brief Current Speed of motors stored in controller ranges
     */
    float mRightMotorSpeed = 0;
    float mLeftMotorSpeed = 0;
    /**
     * @brief Max Speed of any motor stored in Motor ranges
     */
    float mMaxSpeed = Motor::MAX_SPEED;
    float mCoastSpeed = Motor::MAX_SPEED * 0.50;
    void slowCoast();
    void speedUpCoast();

    std::unique_ptr<Motor> mRightMotor;
    std::unique_ptr<Motor> mLeftMotor;

};