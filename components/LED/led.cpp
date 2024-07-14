#include "led.hpp"
#include "esp_log.h"

#define PWM_FREQ 5000 // PWM frequency in Hz
#define LOG_TAG "LED"

template <unsigned size> constexpr std::array<bool, size> init_array_true() {
  std::array<bool, size> anArray;
  for (int i = 0; i < size; i++) {
    anArray[i] = true;
  }
  return anArray;
};

template <unsigned size> constexpr std::array<bool, size> init_array_false() {
  std::array<bool, size> anArray;
  for (int i = 0; i < size; i++) {
    anArray[i] = false;
  }
  return anArray;
};

template <unsigned size> constexpr std::array<int, size> init_array_zero() {
  std::array<int, size> anArray;
  for (int i = 0; i < size; i++) {
    anArray[i] = 0;
  }
  return anArray;
};

std::array<bool, LEDC_CHANNEL_MAX> LED::channelAvailability =
    init_array_true<LEDC_CHANNEL_MAX>();
std::array<bool, LEDC_TIMER_MAX> LED::timerInited =
    init_array_false<LEDC_TIMER_MAX>();
std::array<int, LEDC_TIMER_MAX> LED::numLedsOnTimer =
    init_array_zero<LEDC_TIMER_MAX>();

LED::LED(gpio_num_t aPin, bool aInvert) : mPin(aPin), mBrightness(0) {
  initialize(aInvert);
}

LED::~LED() {
  releaseChannel(mChannel);
  releaseTimer(mTimer);
}

void LED::initialize(bool aInvert) {
  mChannel = getAvailableChannel();
  mTimer = getAvailableTimer();
  if (mChannel == LEDC_CHANNEL_MAX || mTimer == LEDC_TIMER_MAX) {
    ESP_LOGE(LOG_TAG,
             "Failed to set up LED on PIN %d due to lack of timer or channel.",
             mPin);
    return;
  }
  ESP_LOGD(LOG_TAG, "LED setup on Pin: %d Timer: %d Channel %i", (int)mPin,
           (int)mTimer, (int)mChannel);

  // Prepare and then apply the LEDC PWM mChannel configuration
  ledc_channel_config_t ledc_channel = {
      .gpio_num = mPin,
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .channel = mChannel,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = mTimer,
      .duty = 0,
      .hpoint = 0,
      .flags{.output_invert = aInvert ? uint(1) : uint(0)}};
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void LED::setBrightness(int aNewBrightness) {
  if (0 > aNewBrightness || aNewBrightness > 255) {
    ESP_LOGW(LOG_TAG, "FAILED to Set Brightness to %d valid range is (0-255)",
             aNewBrightness);
    return;
  }
  ESP_LOGI(LOG_TAG, "Setting Brightness %d", aNewBrightness);

  mBrightness = aNewBrightness;
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, mChannel, mBrightness);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, mChannel);
}

int LED::getBrightness() { return mBrightness; }

ledc_channel_t LED::getAvailableChannel() {
  for (int i = 0; i < channelAvailability.size(); i++) {
    if (channelAvailability[i]) {
      channelAvailability[i] = false;
      return static_cast<ledc_channel_t>(i);
    }
  }

  ESP_LOGE(LOG_TAG, "ERROR NO LED CHANNELS LEFT!!!");
  return LEDC_CHANNEL_MAX;
}

void LED::setupTimer(int aTimerIdx) {
  auto timerToSetup = static_cast<ledc_timer_t>(aTimerIdx);
  // Setup Timer if it needs to be
  if (!timerInited[aTimerIdx]) {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,  // High-speed mode
        .duty_resolution = LEDC_TIMER_8_BIT, // 8-bit duty resolution for PWM
        .timer_num = timerToSetup,           // Timer number
        .freq_hz = PWM_FREQ,                 // PWM frequency
        .clk_cfg = LEDC_AUTO_CLK,            // Auto select the source clock
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    timerInited[aTimerIdx] = true;
    ESP_LOGD(LOG_TAG, "Inited Timer %d", aTimerIdx);
  }
}

ledc_timer_t LED::getAvailableTimer() {
  for (int i = 0; i < timerInited.size(); i++) {
    auto timerIdx = i;
    auto timerToUse = static_cast<ledc_timer_t>(i);

    setupTimer(timerIdx);
    if (numLedsOnTimer[timerIdx] < 2) {
      numLedsOnTimer[timerIdx]++;
      return timerToUse;
    }
  }
  ESP_LOGE(LOG_TAG, "ERROR NO LED TIMERS LEFT!!!");
  return LEDC_TIMER_MAX;
}

void LED::releaseTimer(ledc_timer_t aTimer) {
  numLedsOnTimer[aTimer]--;
  ESP_LOGD(LOG_TAG, "Release Timer: %d , %d LEDS Left on timer", (int)aTimer,
           (int)numLedsOnTimer[aTimer]);
}

void LED::releaseChannel(ledc_channel_t aChannel) {
  ESP_LOGD(LOG_TAG, "Releasing Channel:%d", aChannel);
  if (channelAvailability[aChannel]) {
    ESP_LOGE(LOG_TAG, "Channel %d already available!", aChannel);
  }
  channelAvailability[aChannel] = true;
}