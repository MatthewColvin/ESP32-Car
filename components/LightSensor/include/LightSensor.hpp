#pragma once

#include <driver/gpio.h>
#include <functional>

class LightSensor {
public:
  typedef std::function<void(int64_t)> LightSensorCallback;

  LightSensor(gpio_num_t aPin, LightSensorCallback aOnReadLightFunction);
  void Enable();
  void Disable();

private:
  void Initialize();
  static void LightSensorIsrHandler(void *self);
  static void CallbackWrapper(void *self);
  gpio_num_t mPin;
  LightSensorCallback mCallback;
};
