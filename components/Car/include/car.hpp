#pragma once

#include "motor.hpp"
#include "IMotorMixingStrategy.hpp"
#include "mocute052.hpp"
#include <memory>

class Car
{
public:
    Car(std::shared_ptr<Mocute052> remote, std::unique_ptr<Motor> leftMotor, std::unique_ptr<Motor> rightMotor);

private:

    /**
     * @brief Set the speed for each motor in the ranges motors accept.
     *
     * @param aLeftMotorSpeed
     * @param aRightMotorSpeed
     */
    void setMotorSpeed(float aLeftMotorSpeed, float aRightMotorSpeed);

    std::unique_ptr<Motor> mRightMotor;
    std::unique_ptr<Motor> mLeftMotor;

    /**
     * @brief a motor mixer that takes input from the controller and
     *        provides output for the motors to drive them.
     */
    std::unique_ptr<IMotorMixingStrategy> mMotorMixer;

    /**
     * @brief Wrapper so we can Start a free RTOS task on the
     *        mixerPollingTask().
     * @param _thisCar - Pointer to car pass through freeRTOS createTask API
     */
    static void mixerPollingImpl(void* _thisCar);
    /**
     * @brief Task to check the motor mixer and
     *        update the car based on that.
     */
    void mixerPollingTask();

};