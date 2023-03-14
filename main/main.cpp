#include "ble.hpp"
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
    auto *bt = Ble::getInstance();

    bool joystickConnected = false;
    std::shared_ptr<Joystick> joystick = nullptr;
    while (!joystickConnected)
    {
        auto devices = bt->scan(5);

        auto joystickDevice = std::find_if(devices.begin(), devices.end(), [](Device device)
                                           { return device.getName() == "VR-PARK"; });

        if (joystickDevice != devices.end())
        {
            ESP_LOGI(LOG_TAG, "FOUND THE REMOTE!!!!");
            joystick = std::make_shared<Joystick>(joystickDevice->getScanResult());

            if (bt->connect(joystick))
            {
                joystickConnected = true;
                break;
            }
        }
        else
        {
            ESP_LOGI(LOG_TAG, "Found %d Devices but no Joystick", devices.size());
        }

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    if (joystick)
    {
        while (!joystick->isServicesSearchComplete())
        {
        };

        joystick->describeServices();
        // joystick->registerCharacteristics();
        //joystick->protocolMode(); // got to boot mode i hope
        joystick->exitSuspend();
        while (true)
        {
            joystick->readAllCharacteristics();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}
