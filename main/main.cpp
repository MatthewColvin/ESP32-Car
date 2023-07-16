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

#define LOG_TAG "MAIN"
#define RightMotorLeftPin GPIO_NUM_16
#define RightMotorRightPin GPIO_NUM_4
#define LeftMotorLeftPin GPIO_NUM_13
#define LeftMotorRightPin GPIO_NUM_5

#define IRLED GPIO_NUM_19
#define IRDETECT GPIO_NUM_18

#define ServoPin GPIO_NUM_23

#define ExternalRedLedPin GPIO_NUM_14
#define ExternalBlueLedPin GPIO_NUM_26
#define ExternalGreenLedPin GPIO_NUM_27

#define StatusRedLedPin GPIO_NUM_25
#define StatusBlueLedPin GPIO_NUM_32
#define StatusGreenLedPin GPIO_NUM_33

/* GLOBAL VARIABLES */
// Reminder: Make your variable names descriptive

// TODO: Add your additional challenge variables
// Syntax Hint: <Class name>* <variable name>;
Car *car;

/* JOYSTICK HANDLERS */
void onAPress(){};
void onARelease(){};
void onBPress(){};
void onBRelease(){};
void onXPress(){};
void onXRelease(){};
void onYPress(){};
void onYRelease(){};
void onTriggerPress(){};
void onTriggerRelease(){};

void onReceiveIRData(uint16_t address, uint16_t data, bool isRepeat)
{
    ESP_LOGI("MAIN", "Address=%04X, Command=%04X\r\n\r\n", address, data);
}

void registerJoystickButtonHandlers(std::shared_ptr<Mocute052> aJoystick)
{
    aJoystick->onA(onAPress, onARelease);
    aJoystick->onB(onBPress, onBRelease);
    aJoystick->onX(onXPress, onXRelease);
    aJoystick->onY(onYPress, onYRelease);
    aJoystick->onTrigger(onTriggerPress, onTriggerRelease);
}

extern "C" void app_main(void)
{
    nvs_flash_init();
    auto bt = BTClassicHID::getInstance();

    // TODO: Put in your MAC Address
    esp_bd_addr_t joystickAddress{0xD0, 0x54, 0x7B, 0xFF, 0xFF, 0xFF};
    auto joystick = bt->connect<controller>(joystickAddress);

    // TODO: Create the new motors for your car
    // TODO: Using the joystick and motor variables make a new car.
    // TODO: Set your LED/Servo/IR variable

    // TODO: Don't forget to tell IR how to handle incoming transmissions
    // TODO: add log to remind to enable RX

    registerJoystickButtonHandlers(joystick);
    vTaskDelete(NULL); // Delete Main Task
}