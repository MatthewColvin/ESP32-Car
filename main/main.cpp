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

/*
This Code is for testing all components that can go on a Car and its connection
to Joystick.

It uses Modes to determine function this mode is shown by color of the servo.


*/

Car *car;
std::shared_ptr<controller> joystick = nullptr;

class ButtonHandler {
public:
  virtual void aPress(){};
  virtual void aRelease(){};
  virtual void bPress(){};
  virtual void bRelease(){};
  virtual void xRelease(){};
  virtual void xPress(){};
  virtual void yRelease(){};
  virtual void yPress(){};
};

std::unique_ptr<ButtonHandler> currentTestHandler =
    std::make_unique<ButtonHandler>();

class ServoHandler : public ButtonHandler {
  void aPress() override{};
  void aRelease() override{};
  void bPress() override{};
  void bRelease() override{};
  void xRelease() override{};
  void xPress() override{};
  void yRelease() override{};
  void yPress() override{};
};

class ExternalLedHandler : public ButtonHandler {
  void aPress() override{};
  void aRelease() override{};
  void bPress() override{};
  void bRelease() override{};
  void xRelease() override{};
  void xPress() override{};
  void yRelease() override{};
  void yPress() override{};
};

class InternalLedHandler : public ButtonHandler {
  void aPress() override{};
  void aRelease() override{};
  void bRelease() override{};
  void bPress() override{};
  void xRelease() override{};
  void xPress() override{};
  void yRelease() override{};
  void yPress() override{};
};

class LightSensorHandler : public ButtonHandler {
  void aPress() override{};
  void aRelease() override{};
  void bRelease() override{};
  void bPress() override{};
  void xRelease() override{};
  void xPress() override{};
  void yRelease() override{};
  void yPress() override{};
  // Light Sensor Code
  int64_t last_time_seen = 0;
  int64_t volcano_pulse_measurement = 0;

  void led_sensor_handler(int64_t timestamp_ms) {
    ESP_LOGI("LightSensor", "------------- INTERRUPT -------------\n");
    if (gpio_get_level(InternalRedLedPin)) {
      gpio_set_level(InternalRedLedPin, 0);
    } else {
      gpio_set_level(InternalRedLedPin, 1);
    }

    if (!last_time_seen) {
      last_time_seen = timestamp_ms;
    } else {
      volcano_pulse_measurement = timestamp_ms - last_time_seen;
      ESP_LOGI("LightSensor", "%lli - %lli = %lli", timestamp_ms,
               last_time_seen, volcano_pulse_measurement);
      last_time_seen = 0;
      volcano_pulse_measurement = 0;
    }
  }
};

class IRHandler : public ButtonHandler {
  void aPress() override{};
  void aRelease() override{};
  void bRelease() override{};
  void bPress() override{};
  void xRelease() override{};
  void xPress() override{};
  void yRelease() override{};
  void yPress() override{};
};

class DriveHandler : public ButtonHandler {
  bool mTurboEnabled = false;

  void aPress() override { // Toggle Turbo
    mTurboEnabled ? car->disableTurbo() : car->enableTurbo();
    mTurboEnabled = !mTurboEnabled;
  };

  void bPress() override { // Increase Cruise Speed

    car->setCruiseSpeed(car->getCruiseSpeed() + 1000);
  };

  void xPress() override { // decrease Cruise Speed
    car->setCruiseSpeed(car->getCruiseSpeed() - 1000);
  };

  void yPress() override { // Change Handling
    car->getHandling() == Car::Handling::Car
        ? car->setHandling(Car::Handling::Tank)
        : car->setHandling(Car::Handling::Car);
  };
};

class ButtonHandlerFactory {
public:
  enum class sensorType {
    Servo,
    ExternalLed,
    InternalLed,
    LightSensor,
    IR,
    Driving,
    COUNT
  };
  std::unique_ptr<ButtonHandler> GetHandler(sensorType aSensor) {
    switch (aSensor) {
    case sensorType::Servo:
      return std::make_unique<ServoHandler>();
    case sensorType::LightSensor:
      return std::make_unique<LightSensorHandler>();
    case sensorType::IR:
      return std::make_unique<IRHandler>();
    case sensorType::InternalLed:
      return std::make_unique<InternalLedHandler>();
    case sensorType::ExternalLed:
      return std::make_unique<ExternalLedHandler>();
    case sensorType::Driving:
      return std::make_unique<DriveHandler>();
    default:
      return std::make_unique<ButtonHandler>();
    };
  };

  std::unique_ptr<ButtonHandler> GetNextHandler() {
    sensorType nextType =
        static_cast<sensorType>((static_cast<int>(mCurrentType) + 1) %
                                static_cast<int>(sensorType::COUNT));
    mCurrentType = nextType;
    return GetHandler(nextType);
  };

  sensorType mCurrentType = sensorType::COUNT;
};

ButtonHandlerFactory handlerFactory;
void onTriggerPress() { currentTestHandler = handlerFactory.GetNextHandler(); };
void onTriggerRelease(){};

extern "C" void app_main(void) {
  nvs_flash_init();
  auto bt = BTClassicHID::getInstance();

  // Put in your MAC Address
  // Note 0x00 Acts as a don't care this can allow for connection to multiple
  // remotes. esp_bd_addr_t joystickAddress{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_bd_addr_t joystickAddress{0xD0, 0x54, 0x7B, 0x00, 0x00, 0x00};
  joystick = bt->connect<controller>(joystickAddress);

  Motor *left = new Motor(LeftMotorLeftPin, LeftMotorRightPin);
  Motor *right = new Motor(RightMotorLeftPin, RightMotorRightPin);
  car = new Car(joystick, left, right);

  joystick->onA([] { currentTestHandler->aPress(); },
                [] { currentTestHandler->aRelease(); });
  joystick->onB([] { currentTestHandler->bPress(); },
                [] { currentTestHandler->bRelease(); });
  joystick->onX([] { currentTestHandler->xPress(); },
                [] { currentTestHandler->xRelease(); });
  joystick->onY([] { currentTestHandler->yPress(); },
                [] { currentTestHandler->yRelease(); });

  joystick->onTrigger(onTriggerPress, onTriggerRelease);
  vTaskDelete(NULL); // Delete Main Task
}