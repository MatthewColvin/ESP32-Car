#include "LightSensor.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LOG_TAG "LightSensor"

LightSensor::LightSensor(gpio_num_t aPin,
                         LightSensorCallback aOnReadLightFunction)
    : mPin(aPin), mCallback(aOnReadLightFunction) {
  Initialize();
}

LightSensor::~LightSensor() {
  gpio_isr_handler_remove(mPin);
  gpio_uninstall_isr_service();
}

void LightSensor::Initialize() {
  gpio_config_t light_sensor_config;
  light_sensor_config.intr_type = GPIO_INTR_DISABLE;
  light_sensor_config.mode = GPIO_MODE_INPUT;
  light_sensor_config.pin_bit_mask = 1ULL << mPin;
  light_sensor_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  light_sensor_config.pull_up_en = GPIO_PULLUP_DISABLE;
  auto err = gpio_config(&light_sensor_config);
  if (err == ESP_ERR_INVALID_ARG) {
    ESP_LOGE(LOG_TAG, "Invalid Arguments in GPIO config!!!");
  }

  gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
}

void LightSensor::Enable() {
  gpio_set_intr_type(mPin, GPIO_INTR_POSEDGE);
  gpio_isr_handler_add(mPin, LightSensorIsrHandler, this);
  ESP_LOGI(LOG_TAG, "Light Sensor is enabled");
  mStartTime = 0;
  mEndTime = 0;
}

void LightSensor::Disable() {
  gpio_intr_disable(mPin);
  ESP_LOGI(LOG_TAG, "Light Sensor is disabled");
}

int64_t LightSensor::GetCollectionTime() {
  if (!mEndTime) {
    ESP_LOGW(LOG_TAG, "End measurement before getting the collection time.");
    return 0;
  }
  return (mEndTime - mStartTime) / 1000;
}

void LightSensor::LightSensorIsrHandler(void *self) {
  xTaskCreate(((LightSensor *)self)->CallbackWrapper, "led sensor", 2048, self,
              4, NULL);
}

void LightSensor::CallbackWrapper(void *self) {
  if (!((LightSensor *)self)->mStartTime) {
    ((LightSensor *)self)->mStartTime = esp_timer_get_time();
  }
  ((LightSensor *)self)->mEndTime = esp_timer_get_time();

  ((LightSensor *)self)->mCallback();
  vTaskDelete(NULL);
}