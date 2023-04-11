#include "motor.hpp"

// const byte redLED = 19;
// const byte greenLED = 18;
// const byte blueLED = 5;

// const byte servoPin = 13;

// const int freq = 500;     // Set timer to run at 500Hz
// const int resolution = 8; // Set timer to 8-bit

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000                                      // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ 25000                                                     // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

#define LOG_TAG "Motor"

Motor::Motor(int aLeftPin, int aRightPin)
{
    mConfig.pwm_freq_hz = 500;
    mConfig.pwma_gpio_num = aLeftPin;
    mConfig.pwmb_gpio_num = aRightPin;

    mPwmConfig.group_id = 0;
    mPwmConfig.resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ;

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&mConfig, &mPwmConfig, &mHandle));
    ESP_ERROR_CHECK(bdc_motor_enable(mHandle));
}

Motor::~Motor()
{
    ESP_ERROR_CHECK(bdc_motor_disable(mHandle));
    ESP_ERROR_CHECK(bdc_motor_del(mHandle));
}

void Motor::setSpeed(uint32_t aSpeed)
{
    if (aSpeed > Motor::MAX_SPEED)
    {
        ESP_LOGE(LOG_TAG, "CANNOT SET SPEED %lu must be > %f", aSpeed, Motor::MAX_SPEED);
        aSpeed = Motor::MAX_SPEED;
    }
    ESP_ERROR_CHECK(bdc_motor_set_speed(mHandle, aSpeed));
};