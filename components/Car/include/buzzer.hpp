#pragma once

#include "driver/gpio.h"

class buzzer{

public:
    buzzer(gpio_num_t pin){
        mPin = pin;
        gpio_set_direction(mPin,GPIO_MODE_OUTPUT);
    }

    void on(){
        isBuzzing = true;
        ESP_ERROR_CHECK(gpio_set_level(mPin,1));
    };
    void off(){
        isBuzzing = false;
        ESP_ERROR_CHECK(gpio_set_level(mPin,0));
    };

    bool isOn(){return isBuzzing;}

private:

    gpio_num_t mPin;
    bool isBuzzing = false;
};