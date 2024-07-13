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
#define HALF_SEC 500 / portTICK_PERIOD_MS
#define REST 2 * HALF_SEC

constexpr auto SpeedSetIRAddress = 0x1254;

#define UpLink false           // IR Sensor
#define TeamSyncLed false      // External LED
#define VolcanoMeasure false   // Light Frequency Sensor & IR Sensor & Status
#define CaveNavigation false   // Servo
#define SampleCollection false // Servo

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
LightSensor *lightSensor = new LightSensor(LightSensorPin, led_sensor_handler);
#endif

int64_t volcano_flash_count = 0;

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

#define TowerAddress 0x00
void sendIRHard(uint16_t op, uint16_t aInputValue) {
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

/* JOYSTICK CALLBACKS */
void onAPress() { ChangeLEDColor(); };
void onARelease() { sendIREasy(); };
void onBPress(){};
void onBRelease(){};
void onXPress(){};
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

// Light Sensor Code
void led_sensor_handler() {
  ESP_LOGI("LightSensor", "------------- INTERRUPT -------------\n");
  if (gpio_get_level(InternalRedLedPin)) {
    gpio_set_level(InternalRedLedPin, 0);
  } else {
    gpio_set_level(InternalRedLedPin, 1);
  }
  volcano_flash_count += 1;
}

extern "C" void app_main(void) {
  nvs_flash_init();
  auto bt = BTClassicHID::getInstance();

  // TODO: Put in your MAC Address
  // esp_bd_addr_t joystickAddress{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_bd_addr_t joystickAddress{0xD0, 0x54, 0x7B, 0x00, 0x00, 0x00};
  joystick = bt->connect<controller>(joystickAddress);

  // TODO: Create the new motors for your car
  Motor *left = new Motor(LeftMotorLeftPin, LeftMotorRightPin);
  Motor *right = new Motor(RightMotorLeftPin, RightMotorRightPin);

  // TODO: using the joystick and motor variables make a new car.
  car = new Car(joystick, left, right);
  car->enableTurbo();

// TODO add log to remind to enable RX
#if UpLink || VolcanoMeasure
  ir->mSetReceiveHandler(onReceiveIRData);
#endif

  registerJoystickButtonHandlers(joystick);
  vTaskDelete(NULL); // Delete Main Task
}