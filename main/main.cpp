#include "mocute052.hpp"
#include "motor.hpp"
#include "BTClassicHID.hpp"
#include "car.hpp"
#include "Transceiver.hpp"
#include "buzzer.hpp"

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include <memory>

#define NotMotorSleep GPIO_NUM_17
#define RightMotorLeftPin 16
#define RightMotorRightPin 4
#define LeftMotorLeftPin 13
#define LeftMotorRightPin 5
#define IRLED 19
#define IRDETECT 18
constexpr auto SpeedSetIRAddress = 0x1254;

Car *car = nullptr;
Transceiver *ir = new Transceiver(IRDETECT, IRLED);
buzzer *horn = new buzzer(GPIO_NUM_23);

void onAPress() { horn->on(); };
void onARelease() { horn->off(); };
void onBPress() { car->setCruiseSpeed(car->getCruiseSpeed() - 1000); };
void onBRelease(){};
void onXPress() { car->setCruiseSpeed(car->getCruiseSpeed() + 1000); };
void onXRelease() { ESP_LOGI("main", "x release"); };
void onYPress(){};
void onYRelease(){};
void onTriggerPress() { car->enableTurbo(); };
void onTriggerRelease() { car->disableTurbo(); };

void onReceiveIRData(uint16_t address, uint16_t data, bool isRepeat)
{
    const auto powerButton = 0xB946;
    const auto upButton = 0xB748;
    const auto downButton = 0xB24D;
    const auto leftButton = 0xB14E;
    const auto rightButton = 0xB649;

    if (address == SpeedSetIRAddress)
    {
        car->setCruiseSpeed(data);
        horn->on();
    }
    switch (data)
    {
    case powerButton:
        car->setCruiseSpeed(1);
        break;
    case upButton:
        car->setCruiseSpeed(car->getCruiseSpeed() + 1000);
        break;
    case downButton:
        car->setCruiseSpeed(car->getCruiseSpeed() - 1000);
        break;
    }

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

void transmitTask()
{
    bool isSpeedAscending = true;
    uint16_t speed = 1000;
    ir->enableTx();
    while (true)
    {
        if (speed <= 1000)
        {
            isSpeedAscending = true;
            speed = 1000;
        }
        else if (speed >= Motor::MAX_SPEED - 1000)
        {
            isSpeedAscending = false;
        }
        ESP_LOGI("main", "sending speed: %d", speed);
        ir->send(SpeedSetIRAddress, speed);
        speed += isSpeedAscending ? 1000 : -1000;
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void enableMotors()
{
    gpio_config_t motorSleepPin;
    motorSleepPin.pin_bit_mask = 1ULL << NotMotorSleep;
    motorSleepPin.mode = GPIO_MODE_OUTPUT;
    motorSleepPin.pull_up_en = GPIO_PULLUP_ENABLE;
    motorSleepPin.pull_down_en = GPIO_PULLDOWN_DISABLE;
    motorSleepPin.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&motorSleepPin);
    gpio_set_level(NotMotorSleep, 0);
}
bool isTx = false;
extern "C" void app_main(void)
{
    nvs_flash_init();
    if (isTx)
    {
        transmitTask();
    }

    auto bt = BTClassicHID::getInstance();
    esp_bd_addr_t joystickAddress{0xe0, 0xf8, 0x48, 0x05, 0x29, 0x50};
    auto joystick = bt->connect<Mocute052>(joystickAddress);

    enableMotors();
    Motor *left = new Motor(LeftMotorLeftPin, LeftMotorRightPin);
    Motor *right = new Motor(RightMotorLeftPin, RightMotorRightPin);
    car = new Car(joystick, left, right);

    ir->mSetReceiveHandler(onReceiveIRData);
    registerJoystickButtonHandlers(joystick);
    vTaskDelete(NULL); // Delete Main Task
}
