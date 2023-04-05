#pragma once

#include "bdc_motor.h"

class Motor
{
public:
    Motor(const int LeftPin, const int rightPin);

private:
    bdc_motor_config_t mConfig;
    bdc_motor_mcpwm_config_t mPwmConfig;

    bdc_motor_handle_t mHandle;
};