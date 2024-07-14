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
#define ALL_TEST 1
#define LED_TEST 0
#define INT_LED_TEST 0
#define STATUS_LED_TEST 0
#define SERVO_TEST 0
#define MOTOR_TEST 0
#define LIGHT_SENSOR_TEST 0
#define IR_TEST 0

#define HALF_SEC 500 / portTICK_PERIOD_MS
#define REST 2 * HALF_SEC

#define ON 0
#define OFF 1
#define STATUS_ON 1
#define STATUS_OFF 0

/* GLOBAL VARIABLES */
LED *redLed;
LED *blueLed;
LED *greenLed;

void setRGB(uint8_t aRed, uint8_t aBlue, uint8_t aGreen) {
  redLed->setBrightness(aRed);
  greenLed->setBrightness(aGreen);
  blueLed->setBrightness(aBlue);
}

void led_sensor_handler() {
  ESP_LOGI("LightSensor", "------------- INTERRUPT -------------\n");

  if (ON == gpio_get_level(InternalBlueLedPin)) {
    gpio_set_level(InternalBlueLedPin, OFF);
  } else {
    gpio_set_level(InternalBlueLedPin, ON);
  }
}

extern "C" void app_main(void) {
  vTaskDelay(HALF_SEC);
  nvs_flash_init();

  ESP_LOGI(LOG_TAG, "--- BEGIN TESTS ---");

  // LED
  if (ALL_TEST || LED_TEST) {
    redLed = new LED(RedLedPin);
    blueLed = new LED(BlueLedPin);
    greenLed = new LED(GreenLedPin);

    ESP_LOGI(LOG_TAG, "LED Test Start...");
    ESP_LOGI(LOG_TAG, "Red");
    setRGB(128, 0, 0);
    vTaskDelay(REST);
    ESP_LOGI(LOG_TAG, "Green");
    setRGB(0, 128, 0);
    vTaskDelay(REST);
    ESP_LOGI(LOG_TAG, "Blue");
    setRGB(0, 0, 128);
    vTaskDelay(REST);
    ESP_LOGI(LOG_TAG, "...LED Test Stop");
    setRGB(0, 0, 0);
    vTaskDelay(REST);
  }

  if (ALL_TEST || INT_LED_TEST) {
    gpio_reset_pin(InternalRedLedPin);
    gpio_reset_pin(InternalGreenLedPin);
    gpio_reset_pin(InternalBlueLedPin);
    gpio_set_direction(InternalRedLedPin, GPIO_MODE_OUTPUT);
    gpio_set_direction(InternalGreenLedPin, GPIO_MODE_OUTPUT);
    gpio_set_direction(InternalBlueLedPin, GPIO_MODE_OUTPUT);
    gpio_set_level(InternalRedLedPin, OFF);
    gpio_set_level(InternalGreenLedPin, OFF);
    gpio_set_level(InternalBlueLedPin, OFF);

    ESP_LOGI(LOG_TAG, "LED Internal Test Start...");
    ESP_LOGI(LOG_TAG, "Red");
    gpio_set_level(InternalRedLedPin, ON);
    vTaskDelay(REST);
    ESP_LOGI(LOG_TAG, "Green");
    gpio_set_level(InternalRedLedPin, OFF);
    gpio_set_level(InternalGreenLedPin, ON);
    vTaskDelay(REST);
    ESP_LOGI(LOG_TAG, "Blue");
    gpio_set_level(InternalGreenLedPin, OFF);
    gpio_set_level(InternalBlueLedPin, ON);
    vTaskDelay(REST);
    ESP_LOGI(LOG_TAG, "...LED Internal Test Stop");
    gpio_set_level(InternalBlueLedPin, OFF);
    vTaskDelay(REST);
  }

  if (ALL_TEST || STATUS_LED_TEST) {
    gpio_reset_pin(StatusLedPin);
    gpio_set_direction(StatusLedPin, GPIO_MODE_OUTPUT);
    gpio_set_level(StatusLedPin, STATUS_OFF);

    ESP_LOGI(LOG_TAG, "Status LED Test Start...");
    gpio_set_level(StatusLedPin, STATUS_ON);
    vTaskDelay(REST);
    ESP_LOGI(LOG_TAG, "...Status LED Test Stop");
    gpio_set_level(StatusLedPin, STATUS_OFF);
    vTaskDelay(REST);
  }

  // Servo
  if (ALL_TEST || SERVO_TEST) {
    ServoMotor *servo = new ServoMotor(ServoPin);

    ESP_LOGI(LOG_TAG, "Servo Test Start...");
    ESP_LOGI(LOG_TAG, "-90");
    servo->setAngle(-90);
    vTaskDelay(REST);
    ESP_LOGI(LOG_TAG, "0");
    servo->setAngle(0);
    vTaskDelay(REST);
    ESP_LOGI(LOG_TAG, "90");
    servo->setAngle(90);
    vTaskDelay(REST);
    ESP_LOGI(LOG_TAG, "...Servo Test Stop");
    servo->setAngle(-90);
    vTaskDelay(REST);
  }

  // Motors
  if (ALL_TEST || MOTOR_TEST) {
    Motor *leftMotor = new Motor(LeftMotorLeftPin, LeftMotorRightPin);
    Motor *rightMotor = new Motor(RightMotorLeftPin, RightMotorRightPin);

    ESP_LOGI(LOG_TAG, "Motor Test Start...");
    leftMotor->setSpeed(10000);
    rightMotor->setSpeed(0);
    ESP_LOGI(LOG_TAG, "Left Forward");
    leftMotor->forward();
    vTaskDelay(REST);
    ESP_LOGI(LOG_TAG, "Left Backward");
    leftMotor->reverse();
    vTaskDelay(REST);

    rightMotor->setSpeed(10000);
    leftMotor->setSpeed(0);
    ESP_LOGI(LOG_TAG, "Right Forward");
    rightMotor->forward();
    vTaskDelay(REST);
    ESP_LOGI(LOG_TAG, "Right Backward");
    rightMotor->reverse();
    vTaskDelay(REST);
    ESP_LOGI(LOG_TAG, "...Motor Test Stop");
    rightMotor->setSpeed(0);
    vTaskDelay(REST);
  }

  // IR
  if (ALL_TEST || IR_TEST) {
    Transceiver *ir = new Transceiver(IRDETECT, IRLED);
    ir->enableTx();
    ir->send(10, 20);
  }

  // Light Sensor
  if (ALL_TEST || LIGHT_SENSOR_TEST) {
    LightSensor *lightSensor =
        new LightSensor(LightSensorPin, led_sensor_handler);
    gpio_set_direction(InternalBlueLedPin, GPIO_MODE_INPUT_OUTPUT);

    lightSensor->Enable();
  }

  ESP_LOGI(LOG_TAG, "--- END TESTS ---");

  vTaskDelay(portMAX_DELAY); // delay Main Task 4 eva
}