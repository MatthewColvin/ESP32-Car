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

#define UpLink true           // IR Sensor
#define TeamSyncLed true      // External LED
#define VolcanoMeasure true   // Light Frequency Sensor & IR Sensor & Status
#define CaveNavigation true   // Servo
#define SampleCollection true // Servo

/* GLOBAL VARIABLES */
// Reminder: Make your variable names descriptive

// TODO: Add your additional challenge variables
// Syntax Hint: <Class name>* <variable name>;
Car *car;
std::shared_ptr<controller> joystick = nullptr;

#if TeamSyncLed
LED *redLed = new LED(RedLedPin);
LED *blueLed = new LED(BlueLedPin);
LED *greenLed = new LED(GreenLedPin);
#endif

#if CaveNavigation || SampleCollection
ServoMotor *servo = new ServoMotor(ServoPin);
#endif

#if UpLink || VolcanoMeasure
Transceiver *ir = new Transceiver(IRDETECT, IRLED);
#endif

#if VolcanoMeasure
LightSensor *lightSensor = nullptr;
#endif

void setRGB(uint8_t aRed, uint8_t aBlue, uint8_t aGreen) {
#if TeamSyncLed
  redLed->setBrightness(aRed);
  greenLed->setBrightness(aGreen);
  blueLed->setBrightness(aBlue);
#endif
}

int LEDState = 0;
void ChangeLEDColor() {
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
}

void sendIREasy() {
#if UpLink
  ir->enableTx();
  ir->send(0, 0);
  ir->disableTx();
#endif
}

// DEV_TODO Add Address for Operation data
#define TowerAddress 0x00
void sendIRHard(uint16_t op, uint16_t aInputValue) {
// DEV_TODO Add Operations
#if (UpLink)
  uint16_t answer = 0;
  switch (op) {
  case 1:
    answer = aInputValue;
    break;
  case 2:
    answer = aInputValue;
    break;
  case 3:
    answer = aInputValue;
    break;
  case 4:
    answer = aInputValue;
    break;
  case 5:
    answer = aInputValue;
    break;
  default:
    answer = aInputValue;
  }

  ir->enableTx();
  ir->send(TowerAddress, answer);
  ir->disableTx();
#endif
}

void controlServoWithJoystick() {
#if CaveNavigation || SampleCollection
  servo->controlWith(joystick);
#endif
}

void controlCarWithJoystick() { car->controlWith(joystick); }

int numPulses = 0;
void startVolcanoCollection() {
#if VolcanoMeasure
  numPulses = 0;
  lightSensor->Enable();
#endif
}

void onPulse() { numPulses++; }

uint16_t timeBetweenPulses = 0;
void endVolcanoCollection() {
#if VolcanoMeasure
  lightSensor->Disable();
  if (numPulses > 0) {
    timeBetweenPulses = lightSensor->GetCollectionTime() / numPulses;
  }
#endif
}

// DEV_TODO Update Address for Volcano Data
#define VolcanoDataTowerAddress 0x00
void sendVolcanoTimeBetweenPulses() {
#if VolcanoMeasure
  ir->enableTx();
  ir->send(VolcanoDataTowerAddress, timeBetweenPulses);
  ir->disableTx();
  ESP_LOGI(LOG_TAG, "timeBetweenPulses: %d sent", timeBetweenPulses);
#endif
}

/* JOYSTICK CALLBACKS */
void onAPress() { ChangeLEDColor(); };
void onARelease() { sendIREasy(); };
void onBPress() { startVolcanoCollection(); };
void onBRelease() { endVolcanoCollection(); };
void onXPress() { sendVolcanoTimeBetweenPulses(); };
void onXRelease(){};
void onYPress(){};
void onYRelease(){};
void onTriggerPress() { controlServoWithJoystick(); };
void onTriggerRelease() { controlCarWithJoystick(); };

void onReceiveIRData(uint16_t address, uint16_t data, bool isRepeat) {
  ESP_LOGI("MAIN", "Address=%04X, Command=%04X\r\n\r\n", address, data);
  sendIRHard(address, data);
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
  esp_bd_addr_t joystickAddress{0xD4, 0x14, 0xA7, 0x0C, 0x14, 0x00};
  joystick = bt->connect<controller>(joystickAddress);

  // TODO: Create the new motors for your car
  Motor *left = new Motor(LeftMotorLeftPin, LeftMotorRightPin);
  Motor *right = new Motor(RightMotorLeftPin, RightMotorRightPin);

  // TODO: using the joystick and motor variables make a new car.
  car = new Car(joystick, left, right);
  car->enableTurbo();

#if VolcanoMeasure
  lightSensor = new LightSensor(LightSensorPin, onPulse);
#endif

// TODO add log to remind to enable RX
#if UpLink
  ir->mSetReceiveHandler(onReceiveIRData);
#endif

  registerJoystickButtonHandlers(joystick);
  vTaskDelete(NULL); // Delete Main Task
}