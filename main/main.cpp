#include "mocute052.hpp"
#include "motor.hpp"
#include "BTClassicHID.hpp"
#include "car.hpp"

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

    auto left = std::make_unique<Motor>(15, 2);
    auto right = std::make_unique<Motor>(16, 17);

    bool joystickConnected = false;
    std::shared_ptr<Mocute052> classicJoystick = nullptr;
    while (!joystickConnected)
    {
        auto devices = bt->scan(10);
        esp_bd_addr_t remoteAddress{0xe0, 0xf8, 0x48, 0x05, 0x29, 0x50};

        ESP_LOGI(LOG_TAG, "found %d devices", devices.size());
        for (auto device : devices)
        {
            if (device.hasAddress(remoteAddress))
            {
                classicJoystick = std::make_shared<Mocute052>(device);
                if (bt->connect(classicJoystick))
                {
                    ESP_LOGI(LOG_TAG, "Connected!");
                    ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(classicJoystick->getAddress()));
                    joystickConnected = true;
                }

                vTaskDelay(5000 / portTICK_PERIOD_MS);
            }
        }
    }

    auto car = Car(classicJoystick, std::move(left), std::move(right));

    vTaskDelay(portMAX_DELAY); // Delay main task to keep car alive
    // vTaskDelete(NULL); // Delete Main Task
}
