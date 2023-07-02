#include "servo.hpp"
#include "led.hpp"
#include "esp_log.h"

#define LOG_TAG "Servo"

ServoMotor::ServoMotor(gpio_num_t servoPin)
    : pin(servoPin)
{
    attach();
};

ServoMotor::~ServoMotor()
{
    LED::releaseChannel(mChannel);
    LED::releaseTimer(mTimer);
}

void ServoMotor::attach()
{
    mChannel = LED::getAvailableChannel();
    mTimer = LED::getAvailableTimer();
    if (mChannel == LEDC_CHANNEL_MAX || mTimer == LEDC_TIMER_MAX)
    {
        ESP_LOGE(LOG_TAG, "Failed to set up Servo on PIN %d due to lack of timer or channel.", pin);
    }

    ledc_timer_config_t timerConfig = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = mTimer,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timerConfig));

    ledc_channel_config_t channelConfig = {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = mChannel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = mTimer,
        .duty = 0,
        .hpoint = 0,
        .flags = {}};
    ESP_ERROR_CHECK(ledc_channel_config(&channelConfig));
}

void ServoMotor::setAngle(int angle)
{
    printf("Setting angle: %d\n", angle);
    uint32_t pulseWidth = map(angle, 0, MAX_DEGREE, MIN_PULSE, MAX_PULSE);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pulseWidth);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

uint32_t ServoMotor::map(uint32_t value, uint32_t inMin, uint32_t inMax, uint32_t outMin, uint32_t outMax)
{
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
