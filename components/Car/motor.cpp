#include "motor.hpp"
#include "bdc_motor.h"

// const byte rightMotorAPin = 15;
// const byte rightMotorBPin = 2;
// const byte leftMotorAPin = 16;
// const byte leftMotorBPin = 17;

// const byte rightMotorAChannel = 0;
// const byte rightMotorBChannel = 1;
// const byte leftMotorAChannel = 2;
// const byte leftMotorBChannel = 3;

// const byte redLED = 19;
// const byte greenLED = 18;
// const byte blueLED = 5;

// const byte servoPin = 13;

// const int freq = 500;     // Set timer to run at 500Hz
// const int resolution = 8; // Set timer to 8-bit

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000                                      // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ 25000                                                     // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

Motor::Motor(int aLeftPin, int aRightPin)
{
    mConfig.pwm_freq_hz = 500;
    mConfig.pwma_gpio_num = aLeftPin;
    mConfig.pwmb_gpio_num = aRightPin;

    mPwmConfig.group_id = 0;
    mPwmConfig.resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ;

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&mConfig, &mPwmConfig, &mHandle));
}