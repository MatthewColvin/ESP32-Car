#include "mocute052.hpp"
#include "motor.hpp"
#include "BTClassicHID.hpp"
#include "car.hpp"
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

#define NotMotorSleep GPIO_NUM_17
#define RightMotorLeftPin 16
#define RightMotorRightPin 4
#define LeftMotorLeftPin 13
#define LeftMotorRightPin 5

#define IRLED 19
#define IRDETECT 18

#define ServoPin GPIO_NUM_12

#define RedLedPin GPIO_NUM_25
#define BlueLedPin GPIO_NUM_32
#define GreenLedPin GPIO_NUM_33

constexpr auto SpeedSetIRAddress = 0x1254;

/* GLOBAL VARIABLES */
// Reminder: Make your variable names descriptive

// TODO: Add your additional challenge variables
// Syntax Hint: <Class name>* <variable name>;
Car *car;
LED *redLed;
LED *blueLed;
LED *greenLed;
ServoMotor *servo;
Transceiver *ir;

/* JOYSTICK CALLBACKS */
void onAPress()
{
    blueLed->setBrightness((blueLed->getBrightness() + 32) % LED::MAX_BRIGHTNESS);
    redLed->setBrightness((blueLed->getBrightness() + 20) % LED::MAX_BRIGHTNESS);
    greenLed->setBrightness((blueLed->getBrightness() + 10) % LED::MAX_BRIGHTNESS);
};
void onARelease(){};
void onBPress() { car->setCruiseSpeed(car->getCruiseSpeed() - 1000); };
void onBRelease(){};
void onXPress() { car->setCruiseSpeed(car->getCruiseSpeed() + 1000); };
void onXRelease(){};
void onYPress() { servo->setAngle(servo->getAngle() + 10 % ServoMotor::MAX_DEGREE); };
void onYRelease(){};
void onTriggerPress() { car->enableTurbo(); };
void onTriggerRelease() { car->disableTurbo(); };

void changeCarHandling()
{
    if (car->getHandling() == Car::Handling::Tank)
    {
        car->setHandling(Car::Handling::Car);
    }
    else
    {
        car->setHandling(Car::Handling::Tank);
    }
}

void onReceiveIRData(uint16_t address, uint16_t data, bool isRepeat)
{
    const auto powerButton = 0xB946;
    const auto upButton = 0xB748;
    const auto downButton = 0xB24D;
    // const auto leftButton = 0xB14E;
    // const auto rightButton = 0xB649;

    if (address == SpeedSetIRAddress)
    {
        car->setCruiseSpeed(data);
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
    gpio_set_level(NotMotorSleep, 1);
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

    // TODO: Put in your MAC Address
    // esp_bd_addr_t joystickAddress{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_bd_addr_t joystickAddress{0xD0, 0x54, 0x7B, 0x00, 0xB2, 0x33};
    auto joystick = bt->connect<Mocute052>(joystickAddress);

    // TODO: Create the new motors for your car
    Motor *left = new Motor(LeftMotorLeftPin, LeftMotorRightPin);
    Motor *right = new Motor(RightMotorLeftPin, RightMotorRightPin);

    // TODO: using the joystick and motor variables make a new car.
    car = new Car(joystick, left, right);

    // TODO: Set your LED/Servo/IR variable
    redLed = new LED(RedLedPin);
    blueLed = new LED(BlueLedPin);
    greenLed = new LED(GreenLedPin);
    servo = new ServoMotor(ServoPin);
    ir = new Transceiver(IRDETECT, IRLED);

    // Don't forget to tell IR how to handle incoming transmissions
    ir->mSetReceiveHandler(onReceiveIRData);
    registerJoystickButtonHandlers(joystick);
    vTaskDelete(NULL); // Delete Main Task
}
