#pragma once

#include "motor.hpp"
#include "IMotorMixingStrategy.hpp"
#include "mocute052.hpp"
#include <memory>

class Car
{
public:
    /// @brief Construct a Car that has 2 speeds cruise(set through functions) and Turbo(max speed)
    /// @param remote Remote that controls the car
    /// @param leftMotor - left motor
    /// @param rightMotor - right motor
    Car(std::shared_ptr<Mocute052> remote, Motor *leftMotor, Motor *rightMotor);

    /**
     * @brief Get the Cruise Speed of the car
     *
     * @return float - Current cruise speed of the car
     */
    float getCruiseSpeed() { return mMotorMixer->getCruiseSpeed(); };
    /**
     * @brief Set the max speed of the car when turbo is not enabled
     *
     * @param aCruiseSpeed - a speed that will be max without turbo mode enabled
     */
    void setCruiseSpeed(float aCruiseSpeed) { return mMotorMixer->setCruiseSpeed(aCruiseSpeed); };

    ///@brief Allow to car to go full speed
    void enableTurbo() { mMotorMixer->enableTurbo(); };
    /// @brief Force car to use cruising speed as the fastest speed.
    void disableTurbo() { mMotorMixer->disableTurbo(); };

private:
    /**
     * @brief Set the speed for each motor in the ranges motors accept.
     *
     * @param aLeftMotorSpeed
     * @param aRightMotorSpeed
     */
    void setMotorSpeed(float aLeftMotorSpeed, float aRightMotorSpeed);

    /// @brief Refences to the right motor
    std::unique_ptr<Motor> mRightMotor;
    /// @brief Reference to Left motor
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
    static void mixerPollingImpl(void *_thisCar);
    /**
     * @brief Task to check the motor mixer and
     *        update the car based on that.
     */
    void mixerPollingTask();
};