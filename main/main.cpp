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
public:
  ServoHandler() : mMotor(ServoPin){};
  ~ServoHandler() { car->controlWith(joystick); }

  void aPress() override { // Control Servo With Joystick
    mMotor.controlWith(joystick);
  };

  void bPress() override { // Control Car With Joystick
    car->controlWith(joystick);
  };

  void xPress() override { mMotor.incrementAngle(10); };

  void yPress() override { mMotor.decrementAngle(10); };

  ServoMotor mMotor;
};

class RGBLedHandler : public ButtonHandler {
public:
  RGBLedHandler(gpio_num_t redPin, gpio_num_t greenPin, gpio_num_t bluePin)
      : red(redPin), green(greenPin), blue(bluePin) {
    red.setBrightness(0);
    green.setBrightness(0);
    blue.setBrightness(0);
  }
  LED red;
  LED green;
  LED blue;

  void aPress() override {
    red.setBrightness((red.getBrightness() + 10) % 255);
  };
  void bPress() override {
    green.setBrightness((green.getBrightness() + 10) % 255);
  };
  void xPress() override {
    blue.setBrightness((blue.getBrightness() + 10) % 255);
  };
  void yPress() override {
    red.setBrightness(0);
    green.setBrightness(0);
    blue.setBrightness(0);
  };
};

class LightSensorHandler : public ButtonHandler {
public:
  LightSensorHandler()
      : mLightSensor(LightSensorPin, [this](auto aTimeWhenPulse) {
          led_sensor_handler(aTimeWhenPulse);
        }) {}

  void aPress() override { mLightSensor.Enable(); };
  void bPress() override { mLightSensor.Disable(); };

  LightSensor mLightSensor;

  // Light Sensor Code
  int64_t last_time_seen = 0;
  int64_t volcano_pulse_measurement = 0;

  void led_sensor_handler(int64_t timestamp_ms) {
    ESP_LOGI("LightSensor", "------------- INTERRUPT -------------\n");
    if (gpio_get_level(StatusLedPin)) {
      gpio_set_level(StatusLedPin, 0);
    } else {
      gpio_set_level(StatusLedPin, 1);
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
public:
  IRHandler() : mIr(IRDETECT, IRLED) {
    mIr.mSetReceiveHandler([](auto addr, auto cmd, auto isRepeat) {
      ESP_LOGI(LOG_TAG, "Received Address:%i Command:%i \n", addr, cmd);
    });
    mIr.disableRx();
    mIr.disableTx();
  }
  void aPress() override { // Toggle Receive
    mRxEnabled ? mIr.disableRx() : mIr.enableRx();
    mRxEnabled = !mRxEnabled;
  };
  void bPress() override { // Toggle Send
    mTxEnabled ? mIr.disableTx() : mIr.enableTx();
    mTxEnabled = !mTxEnabled;
  };

  void xPress() override { mIr.send(0, 0); };
  void yPress() override { mIr.send(0, 135); };

  Transceiver mIr;
  bool mRxEnabled = false;
  bool mTxEnabled = false;
};

class DriveHandler : public ButtonHandler {
public:
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
      ESP_LOGI(LOG_TAG, "---Servo TEST---");
      return std::make_unique<ServoHandler>();
    case sensorType::LightSensor:
      ESP_LOGI(LOG_TAG, "---Light Sensor TEST---");
      return std::make_unique<LightSensorHandler>();
    case sensorType::IR:
      ESP_LOGI(LOG_TAG, "---IR TEST---");
      return std::make_unique<IRHandler>();
    // case sensorType::InternalLed:
    //   ESP_LOGI(LOG_TAG, "---Internal LED TEST---");
    //   return std::make_unique<RGBLedHandler>(
    //       InternalRedLedPin, InternalGreenLedPin, InternalBlueLedPin);
    case sensorType::ExternalLed:
      ESP_LOGI(LOG_TAG, "---External LED TEST---");
      return std::make_unique<RGBLedHandler>(RedLedPin, GreenLedPin,
                                             BlueLedPin);
    case sensorType::Driving:
      ESP_LOGI(LOG_TAG, "---Driving TEST---");
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
  esp_bd_addr_t joystickAddress{0xD4, 0x14, 0xA7, 0x00, 0x00, 0x00};
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