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

extern "C" void app_main(void)
{
    nvs_flash_init();
    auto *bt = Ble::getInstance();

    bool joystickFound = false;
    Device *joystick = nullptr;
    while (!joystickFound)
    {
        auto devices = bt->scan(5);

        ESP_LOGI(LOG_TAG, "Found %d Devices", devices.size());
        for (auto device : devices)
        {
            if (device.getName() != "")
            {
                ESP_LOGI(LOG_TAG, "Device Name: %s", device.getName().c_str());
            }
            if (device.getName() == "VR-PARK")
            {
                joystickFound = true;
                joystick = bt->connect(device);
                ESP_LOGI(LOG_TAG, "FOUND THE REMOTE!!!!");
                break;
            }
        }
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    if (joystick)
    {
        while (!joystick->isServicesSearchComplete())
        {
        };
    }

    while (true)
    {
        joystick->describeServices();
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
