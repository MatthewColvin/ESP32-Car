#pragma once

#include "driver/gpio.h"

class buzzer
{

public:
    /**
     * @brief Construct a new buzzer on the given pin
     *
     * @param pin - pin the buzzer is on
     */
    buzzer(gpio_num_t pin)
    {
        mPin = pin;
        gpio_set_direction(mPin, GPIO_MODE_OUTPUT);
    }

    /**
     * @brief Turn the buzzer on
     */
    void on()
    {
        isBuzzing = true;
        ESP_ERROR_CHECK(gpio_set_level(mPin, 1));
    };
    /**
     * @brief Turn the buzzer off
     */
    void off()
    {
        isBuzzing = false;
        ESP_ERROR_CHECK(gpio_set_level(mPin, 0));
    };
    /**
     * @brief Check if the buzzer is on
     *
     * @return true - buzzer is on
     * @return false - buzzer is off
     */
    bool isOn() { return isBuzzing; }

private:
    /**
     * @brief Pin the Buzzer is on
     */
    gpio_num_t mPin;
    /**
     * @brief Store wether the buzzer is on or off
     */
    bool isBuzzing = false;
};