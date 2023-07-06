#include "mocute052.hpp"
#include "motor.hpp"
#include "BTClassicHID.hpp"
#include "car.hpp"
#include "controller.hpp"
#include "buzzer.hpp"
#include "led.hpp"
#include "servo.hpp"
#include "Transceiver.hpp"

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include <memory>
#define LOG_TAG "main"

#define LOG_TAG "main"
#define NotMotorSleep GPIO_NUM_17
#define RightMotorLeftPin 16
#define RightMotorRightPin 4
#define LeftMotorLeftPin 13
#define LeftMotorRightPin 5

#define IRLED GPIO_NUM_19
#define IRDETECT GPIO_NUM_18

void onAPress() { ESP_LOGI(LOG_TAG, "A Press"); }
void onBPress() { ESP_LOGI(LOG_TAG, "B Press"); }
void onXPress() { ESP_LOGI(LOG_TAG, "X Press"); }
void onYPress() { ESP_LOGI(LOG_TAG, "Y Press"); }
void onARelease() { ESP_LOGI(LOG_TAG, "A Release"); }
void onBRelease() { ESP_LOGI(LOG_TAG, "B Release"); }
void onXRelease() { ESP_LOGI(LOG_TAG, "X Release"); }
void onYRelease() { ESP_LOGI(LOG_TAG, "Y Release"); }
void onTriggerPress() { ESP_LOGI(LOG_TAG, "Trigger Press"); };
void onTriggerRelease() { ESP_LOGI(LOG_TAG, "Trigger Release"); };
void onJoystick(uint8_t x, uint8_t y) { ESP_LOGI(LOG_TAG, "X:%d Y:%d", x, y); }
void onReceiveIRData(uint16_t address, uint16_t data, bool isRepeat)
{
    ESP_LOGI("IR RECEIVED DATA", "Address=%04X, Command=%04X\r\n\r\n", address, data);
}

void registerJoystickButtonHandlers(std::shared_ptr<Mocute052> aJoystick)
{
    aJoystick->onA(onAPress, onARelease);
    aJoystick->onB(onBPress, onBRelease);
    aJoystick->onX(onXPress, onXRelease);
    aJoystick->onY(onYPress, onYRelease);
    aJoystick->onTrigger(onTriggerPress, onTriggerRelease);
    aJoystick->onJoyStick(onJoystick);
}

void connectJoystickTask(void *nothing)
{
    auto bt = BTClassicHID::getInstance();
    esp_bd_addr_t joystickAddress{0xD0, 0x54, 0x7B, 0x00, 0x47, 0x19}; // 00 will act as "don't cares allowing it to connect to any controller."
    while (true)
    {
        auto joystick = bt->connect<controller>(joystickAddress);
        registerJoystickButtonHandlers(joystick);
        joystick->onDisconnect([]
                               { xTaskCreate(connectJoystickTask, "ConnectTojoystick", 4096, nullptr, 5, nullptr); });
        break;
    }
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
    nvs_flash_init();

    xTaskCreate(connectJoystickTask, "ConnectTojoystick", 4096, nullptr, 5, nullptr);

    vTaskDelete(NULL); // Delete Main Task
}
