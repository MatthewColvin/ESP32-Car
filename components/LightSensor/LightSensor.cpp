#include "LightSensor.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

LightSensor::LightSensor(gpio_num_t aPin, LightSensorCallback aOnReadLightFunction)
    : mPin(aPin), mCallback(aOnReadLightFunction) {
    Initialize();
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
        ESP_LOGE("LightSensor", "Invalid Arguments!!!");
    }
}

void LightSensor::Enable() {
    gpio_set_intr_type(mPin, GPIO_INTR_POSEDGE);

    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    gpio_isr_handler_add(mPin, LightSensorIsrHandler, this);
    ESP_LOGI("LightSensor", "Light Sensor is now enabled");
}

void LightSensor::Disable() {
    gpio_intr_disable(mPin);
    ESP_LOGI("LightSensor", "Light Sensor is disabled");
}

void LightSensor::LightSensorIsrHandler(void *self)
{
    xTaskCreate(((LightSensor*)self)->CallbackWrapper, "led sensor", 2048, self, 4, NULL);
}

void LightSensor::CallbackWrapper(void* self) {
    auto current_time = esp_timer_get_time() / 1000;
    ((LightSensor *)self)->mCallback(current_time);
    vTaskDelete(NULL);
}