#pragma once

#include <stdio.h>
#include <driver/ledc.h>
#include <driver/gpio.h>

class LED {
public:
    LED(gpio_num_t pin, ledc_channel_t channel, ledc_timer_t timer);
    ~LED() = default;

    void initialize();
    void setBrightness(uint8_t brightness);

private:
    gpio_num_t pin;
    ledc_channel_t channel;
    ledc_timer_t timer;
};
