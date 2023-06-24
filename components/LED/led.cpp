#include "led.hpp"

#define PWM_FREQ 5000 // PWM frequency in Hz

LED::LED(gpio_num_t pin, ledc_channel_t channel, ledc_timer_t timer)
    : pin(pin), channel(channel), timer(timer) {}

void LED::initialize()
{
    // gpio_pad_select_gpio(pin);                 // Set the LED pin as a GPIO
    // gpio_set_direction(pin, GPIO_MODE_OUTPUT); // Set the LED pin as an output

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,  // High-speed mode
        .duty_resolution = LEDC_TIMER_8_BIT, // 8-bit duty resolution for PWM
        .timer_num = timer,                  // Timer number
        .freq_hz = PWM_FREQ,                 // PWM frequency
        .clk_cfg = LEDC_AUTO_CLK,            // Auto select the source clock
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num = pin,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0,
        .flags = {}
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void LED::setBrightness(uint8_t brightness)
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, brightness);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}