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

extern "C" void app_main(void)
{
    nvs_flash_init();
    auto bt = BTClassicHID::getInstance();
    esp_bd_addr_t joystickAddress{0xe0, 0xf8, 0x48, 0x05, 0x29, 0x50};
    auto joystick = bt->connect<Mocute052>(joystickAddress);

    auto right = std::make_unique<Motor>(15, 2);
    auto left = std::make_unique<Motor>(16, 17);
    auto car = Car(joystick, std::move(left), std::move(right));

    vTaskDelay(portMAX_DELAY); // Delay main task to keep car alive
    // vTaskDelete(NULL); // Delete Main Task
}
