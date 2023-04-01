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
    std::shared_ptr<HIDDevice> classicJoystick = nullptr;
    while (!joystickConnected)
    {
        auto devices = bt->scan(10);
        esp_bd_addr_t remoteAddress{0xe0, 0xf8, 0x48, 0x05, 0x29, 0x50};

        ESP_LOGI(LOG_TAG, "found %d devices", devices.size());
        for (auto device : devices)
        {
            if (device.hasAddress(remoteAddress))
            {
                classicJoystick = std::make_shared<HIDDevice>(device);
                if (bt->connect(classicJoystick))
                {
                    ESP_LOGI(LOG_TAG, "Connected!");
                    ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(classicJoystick->getAddress()));
                    joystickConnected = true;
                }
            }
        }

        // if (joystickDevice != devices.end())
        // {
        //     ESP_LOGI(LOG_TAG, "FOUND THE REMOTE!!!!");
        //     // joystick = std::make_shared<Joystick>(joystickDevice->getScanResult());

        //     // if (bt->connect(joystick))
        //     // {
        //     //     joystickConnected = true;
        //     //     break;
        //     // }
        // }
        // else
        // {
        //     ESP_LOGI(LOG_TAG, "Found %d Devices but no Joystick", devices.size());
        // }

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
