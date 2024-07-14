#include "BTClassicHID.hpp"
#include "LightSensor.hpp"
#include "Transceiver.hpp"
#include "buzzer.hpp"
#include "car.hpp"
#include "controller.hpp"
#include "led.hpp"
#include "mocute052.hpp"
#include "motor.hpp"
#include "servo.hpp"

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include <memory>

#define LOG_TAG "main"

/* TEST CONTROLS */
#define IR_TEST 1

#define HALF_SEC 500 / portTICK_PERIOD_MS
#define REST 2 * HALF_SEC

#define ON 0
#define OFF 1
#define STATUS_ON 1
#define STATUS_OFF 0

void onReceiveIRData(uint16_t address, uint16_t data, bool isRepeat) {
  ESP_LOGI("MAIN", "Address=%04X, Command=%04X\r\n\r\n", address, data);
  gpio_set_level(InternalGreenLedPin, ON);
}

extern "C" void app_main(void) {
  vTaskDelay(HALF_SEC);
  nvs_flash_init();

  // IR
  if (IR_TEST) {
    Transceiver *ir = new Transceiver(IRDETECT, IRLED);
    gpio_reset_pin(InternalGreenLedPin);
    gpio_set_direction(InternalGreenLedPin, GPIO_MODE_OUTPUT);
    gpio_set_level(InternalGreenLedPin, OFF);

    ir->mSetReceiveHandler(onReceiveIRData);
    ir->enableRx();
  }

  vTaskDelay(portMAX_DELAY); // delay Main Task 4 eva
}