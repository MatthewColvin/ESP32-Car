#include "BTClassicHID.hpp"
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
#include "nvs_flash.h"

#include <memory>

#define LOG_TAG "main"

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

void setRGB(uint8_t aRed, uint8_t aBlue, uint8_t aGreen) {
  redLed->setBrightness(aRed);
  greenLed->setBrightness(aGreen);
  blueLed->setBrightness(aBlue);
}

/* JOYSTICK CALLBACKS */
void onAPress() {
  switch (LEDState) {
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
void onBPress() { ir->send(0x00, 0x10); };
void onBRelease(){};
void onXPress() { ir->send(0x20, 0x00); };
void onXRelease(){};
void onYPress() {
  auto currentAngle = servo->getAngle();
  if (servoAscending) {
    auto nextAngle = currentAngle += 10;
    if (nextAngle >= ServoMotor::MAX_DEGREE) {
      servoAscending = false;
      servo->setAngle(currentAngle - 10);
    }
    servo->setAngle(nextAngle);
  } else {
    auto nextAngle = currentAngle -= 10;
    if (nextAngle <= ServoMotor::MIN_DEGREE) {
      servoAscending = true;
      servo->setAngle(currentAngle + 10);
    }
    servo->setAngle(nextAngle);
  }
};
void onYRelease(){};
void onTriggerPress() { car->enableTurbo(); };
void onTriggerRelease() { car->disableTurbo(); };

void onReceiveIRData(uint16_t address, uint16_t data, bool isRepeat) {
  ESP_LOGI("MAIN", "Address=%04X, Command=%04X\r\n\r\n", address, data);
}

void registerJoystickButtonHandlers(std::shared_ptr<Mocute052> aJoystick) {
  aJoystick->onA(onAPress, onARelease);
  aJoystick->onB(onBPress, onBRelease);
  aJoystick->onX(onXPress, onXRelease);
  aJoystick->onY(onYPress, onYRelease);
  aJoystick->onTrigger(onTriggerPress, onTriggerRelease);
}

extern "C" void app_main(void) {
  nvs_flash_init();
  auto bt = BTClassicHID::getInstance();

  // TODO: Put in your MAC Address
  // esp_bd_addr_t joystickAddress{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_bd_addr_t joystickAddress{0xD0, 0x54, 0x7B, 0x00, 0x00, 0x00};
  auto joystick = bt->connect<controller>(joystickAddress);

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
  ir->enableTx();
  registerJoystickButtonHandlers(joystick);
  vTaskDelete(NULL); // Delete Main Task
}