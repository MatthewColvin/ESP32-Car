#pragma once

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <stdio.h>

#include <array>

class LED {
public:
  LED(gpio_num_t aPin, bool aInvert = false);
  virtual ~LED();

  static constexpr auto MAX_BRIGHTNESS = 256;

  void setBrightness(int aNewBrightness);
  int getBrightness();

  static ledc_channel_t getAvailableChannel();
  static ledc_timer_t getAvailableTimer();
  static void setupTimer(int aTimerIdx);
  static void releaseTimer(ledc_timer_t aTimer);
  static void releaseChannel(ledc_channel_t aChannel);

private:
  static std::array<bool, LEDC_CHANNEL_MAX> channelAvailability;
  static std::array<bool, LEDC_TIMER_MAX> timerInited;
  static std::array<int, LEDC_TIMER_MAX> numLedsOnTimer;

  void initialize(bool aInvert);
  gpio_num_t mPin;
  ledc_channel_t mChannel;
  ledc_timer_t mTimer;
  uint8_t mBrightness;
};
