#pragma once

#include "bdc_motor.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"

class Motor
{
public:
    /// @brief Max speed the bcd_motor library seems to handle without crashing
    static constexpr float MAX_SPEED = 20000;
    /// @brief Min speed the bcd_motor library seems to handle without crashing
    static constexpr float MIN_SPEED = -20000;

    /// @brief - Construct an instance to manage DC motor with H-bridge
    /// @param LeftPin - GPIO left pin
    /// @param rightPin - GPIO right pin
    Motor(const int LeftPin, const int rightPin);
    ~Motor();

    /// @brief Set the motor speed
    /// @param aSpeed - a speed to set the motor to
    void setSpeed(uint32_t aSpeed);

    /// @brief Set the motor to go forward
    void forward() { ESP_ERROR_CHECK(bdc_motor_forward(mHandle)); };
    /// @brief Set the motor to go backwards
    void reverse() { ESP_ERROR_CHECK(bdc_motor_reverse(mHandle)); };
    /// @brief Turn off the motion of the motor
    void disable() { ESP_ERROR_CHECK(bdc_motor_disable(mHandle)); }

private:
    /// @brief Config used to set up the bcd_motor library
    bdc_motor_config_t mConfig;
    /// @brief Config used to set up the Pwm timer and its speed
    bdc_motor_mcpwm_config_t mPwmConfig;

    /// @brief Handle to allow management of the motor via the bcd_motor library
    bdc_motor_handle_t mHandle;
};