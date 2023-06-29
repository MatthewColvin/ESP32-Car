#include "servo.hpp"

ServoMotor::ServoMotor(gpio_num_t servoPin, uint32_t minPulse, uint32_t maxPulse, uint32_t maxDeg)
    : pin(servoPin), minPulseWidth(minPulse), maxPulseWidth(maxPulse), maxDegree(maxDeg){};

void ServoMotor::attach()
{
    // gpio_pad_select_gpio(pin);
    // gpio_set_direction(pin, GPIO_MODE_OUTPUT);

    ledc_timer_config_t timerConfig = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timerConfig));

    ledc_channel_config_t channelConfig = {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {}
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channelConfig));
}

void ServoMotor::write(int angle)
{
    printf("Setting angle: %d\n", angle);
    uint32_t pulseWidth = map(angle, 0, maxDegree, minPulseWidth, maxPulseWidth);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pulseWidth);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

uint32_t ServoMotor::map(uint32_t value, uint32_t inMin, uint32_t inMax, uint32_t outMin, uint32_t outMax)
{
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
