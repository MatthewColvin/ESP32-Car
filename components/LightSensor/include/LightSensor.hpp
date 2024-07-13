#pragma once

#include <driver/gpio.h>
#include <functional>

class LightSensor {
public:
  typedef std::function<void(void)> LightSensorCallback;

  LightSensor(gpio_num_t aPin, LightSensorCallback aOnReadLightFunction);
  void Enable();
  void Disable();
  int64_t GetCollectionTime();

private:
  void Initialize();
  static void LightSensorIsrHandler(void *self);
  static void CallbackWrapper(void *self);
  gpio_num_t mPin;
  int64_t mStartTime;
  int64_t mEndTime;
  LightSensorCallback mCallback;
};
