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
#define NotMotorSleep GPIO_NUM_17
#define RightMotorLeftPin 16
#define RightMotorRightPin 4
#define LeftMotorLeftPin 13
#define LeftMotorRightPin 5

#define IRLED GPIO_NUM_19
#define IRDETECT GPIO_NUM_18

#define ServoPin GPIO_NUM_23

#define RedLedPin GPIO_NUM_14
#define BlueLedPin GPIO_NUM_26
#define GreenLedPin GPIO_NUM_27

constexpr auto SpeedSetIRAddress = 0x1254;

/* GLOBAL VARIABLES */
// Reminder: Make your variable names descriptive

// TODO: Add your additional challenge variables
// Syntax Hint: <Class name>* <variable name>;
Car *car;
LED *redLed;
LED *blueLed;
LED *greenLed;
int LEDState = 0;
ServoMotor *servo;
bool servoAscending = false;
Transceiver *ir = new Transceiver(IRDETECT, IRLED);

void setRGB(uint8_t aRed, uint8_t aBlue, uint8_t aGreen)
{
    redLed->setBrightness(aRed);
    greenLed->setBrightness(aGreen);
    blueLed->setBrightness(aBlue);
}

/* JOYSTICK CALLBACKS */
void onAPress()
{
    switch (LEDState)
    {
    case 0:
        setRGB(0, 0, 255);
        LEDState += 1;
        break;
    case 1:
        setRGB(0, 255, 0);
        LEDState += 1;
        break;
    case 2:
        setRGB(255, 0, 0);
        LEDState += 1;
        break;
    default:
        setRGB(0, 0, 0);
        LEDState = 0;
    }
};
void onARelease(){};
void onBPress()
{
    car->setCruiseSpeed(car->getCruiseSpeed() - 1000);
};
void onBRelease(){};
void onXPress()
{
    car->setCruiseSpeed(car->getCruiseSpeed() + 1000);
};
void onXRelease(){};
void onYPress()
{
    auto currentAngle = servo->getAngle();
    if (servoAscending)
    {
        auto nextAngle = currentAngle += 10;
        if (nextAngle >= ServoMotor::MAX_DEGREE)
        {
            servoAscending = false;
            servo->setAngle(currentAngle - 10);
        }
        servo->setAngle(nextAngle);
    }
    else
    {
        auto nextAngle = currentAngle -= 10;
        if (nextAngle <= ServoMotor::MIN_DEGREE)
        {
            servoAscending = true;
            servo->setAngle(currentAngle + 10);
        }
        servo->setAngle(nextAngle);
    }
};
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

    const auto oneButton = 0xFE01;
    const auto twoButton = 0xFD02;
    const auto threeButton = 0xFC03;
    const auto fourButton = 0xFB04;
    const auto fiveButton = 0xFA05;
    const auto sixButton = 0xF906;
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
    case oneButton:
        redLed->setBrightness(redLed->getBrightness() + 10);
        break;
    case twoButton:
        blueLed->setBrightness(blueLed->getBrightness() + 10);
        break;
    case threeButton:
        greenLed->setBrightness(greenLed->getBrightness() + 10);
        break;
    case fourButton:
        redLed->setBrightness(redLed->getBrightness() - 10);
        break;
    case fiveButton:
        blueLed->setBrightness(blueLed->getBrightness() - 10);
        break;
    case sixButton:
        greenLed->setBrightness(greenLed->getBrightness() - 10);
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
// TODO move to car
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
    esp_bd_addr_t joystickAddress{0xD0, 0x54, 0x7B, 0x00, 0x47, 0x19};
    auto joystick = bt->connect<controller>(joystickAddress);

    enableMotors();
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

    // Don't forget to tell IR how to handle incoming transmissions
    // TODO add log to remind to enable RX
    ir->mSetReceiveHandler(onReceiveIRData);
    ir->enableRx();
    registerJoystickButtonHandlers(joystick);
    vTaskDelete(NULL); // Delete Main Task
}
