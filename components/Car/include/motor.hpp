#pragma once

#include "bdc_motor.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"

class Motor
{
public:
    static constexpr float MAX_SPEED = 20000;
    static constexpr float MIN_SPEED = -20000;

    Motor(const int LeftPin, const int rightPin);
    ~Motor();

    void setSpeed(uint32_t aSpeed);

    void forward() { ESP_ERROR_CHECK(bdc_motor_forward(mHandle)); };
    void reverse() { ESP_ERROR_CHECK(bdc_motor_reverse(mHandle)); };
    void disable() { ESP_ERROR_CHECK(bdc_motor_disable(mHandle)); }

private:
    bdc_motor_config_t mConfig;
    bdc_motor_mcpwm_config_t mPwmConfig;

    bdc_motor_handle_t mHandle;
};