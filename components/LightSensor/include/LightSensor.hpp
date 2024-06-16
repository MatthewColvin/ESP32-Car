#pragma once

#include <driver/gpio.h>

class LightSensor {
public:
    typedef void (*LightSensorCallback)(int64_t timestamp_ms);
    
    LightSensor(gpio_num_t aPin, LightSensorCallback aOnReadLightFunction);
    void Enable();
    void Disable();

private:
    void Initialize();
    static void LightSensorIsrHandler(void *self);
    static void CallbackWrapper(void* self);
    gpio_num_t mPin;
    LightSensorCallback mCallback;
};
