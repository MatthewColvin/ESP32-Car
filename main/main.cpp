#include "ble.hpp"

#include "driver/gpio.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#define LOG_TAG "Main"

extern "C" void app_main(void) {
  nvs_flash_init();
  auto *bt = Ble::getInstance();

  while (true) {
    auto devices = bt->scan(5);

    ESP_LOGI(LOG_TAG, "Found %d Devices", devices.size());
    for (auto device : devices) {
      if (device.getName() != "") {
        ESP_LOGI(LOG_TAG, "Device Name: %s", device.getName().c_str());
      }
      if (device.getName() == "VR-PARK") {
        ESP_LOGI(LOG_TAG, "FOUND THE REMOTE!!!!");
      }
      // esp_log_buffer_hex(LOG_TAG, device.bda, 6);
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}
