#include "ble.hpp"
#include "BTClassicHID.hpp"
#include "joystick.hpp"

#include "driver/gpio.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include <memory>

#define LOG_TAG "Main"

extern "C" void app_main(void)
{
    nvs_flash_init();
    auto bt = BTClassicHID::getInstance();

    bool joystickConnected = false;
    std::shared_ptr<Joystick> joystick = nullptr;
    while (!joystickConnected)
    {
        auto devices = bt->scan(10);

        for (auto device : devices)
        {
            if (auto name = device.getName(); name != "")
            {
                ESP_LOGI(LOG_TAG, "Found: %s", name.c_str());
            }
        }

        auto joystickDevice = std::find_if(devices.begin(), devices.end(), [](auto device)
                                           { return device.getName() == "Fortune Tech Wireless"; });

        if (joystickDevice != devices.end())
        {
            ESP_LOGI(LOG_TAG, "FOUND THE REMOTE!!!!");
            // joystick = std::make_shared<Joystick>(joystickDevice->getScanResult());

            // if (bt->connect(joystick))
            // {
            //     joystickConnected = true;
            //     break;
            // }
        }
        else
        {
            ESP_LOGI(LOG_TAG, "Found %d Devices but no Joystick", devices.size());
        }

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    // if (joystick)
    // {
    //     //joystick->init();
    //     // joystick->cycleReports();
    //     // int i = 0;
    //     // while (true)
    //     // {
    //     //     joystick->nextReports(i);
    //     //     vTaskDelay(20 * 1000 / portTICK_PERIOD_MS); // delay 20 seconds
    //     //     i += 3;                                     // move to next 3 reports
    //     // }
    // }
    vTaskDelete(NULL); // Delete Main Task
}
