#include "ble.hpp"
#include "BTClassicHID.hpp"
#include "mocute052.hpp"
#include "motor.hpp"

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

// const byte rightMotorAPin = 15;
// const byte rightMotorBPin = 2;
// const byte leftMotorAPin = 16;
// const byte leftMotorBPin = 17;

// const byte rightMotorAChannel = 0;
// const byte rightMotorBChannel = 1;
// const byte leftMotorAChannel = 2;
// const byte leftMotorBChannel = 3;

extern "C" void app_main(void)
{
    nvs_flash_init();
    auto bt = BTClassicHID::getInstance();

    Motor left(15, 2);
    Motor right(16, 17);
    left.forward();
    right.forward();

    int MaxSpeed = Motor::MAX_SPEED;
    for (int i = 0; i < MaxSpeed; i += 50)
    {
        if (i == MaxSpeed - 50)
        {
            i = 0;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
        ESP_LOGI(LOG_TAG, "Speed %d", i);
        left.setSpeed(i);
        right.setSpeed(i);
    }

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
            }
        }

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL); // Delete Main Task
}
