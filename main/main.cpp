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
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include <memory>

#define LOG_TAG "main"
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

#define StatusLedPin GPIO_NUM_25

#define HALF_SEC 500 / portTICK_PERIOD_MS
#define REST 2 * HALF_SEC

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
// Transceiver *ir = new Transceiver(IRDETECT, IRLED);


gpio_config_t light_sensor_config;

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
void onBPress() { 
    // ir->send(0x00, 0x10); 
};
void onBRelease(){};
void onXPress() {
    // ir->send(0x20, 0x00);
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

int64_t last_time_seen = 0;
int64_t volcano_pulse_measurement = 0;

void led_sensor_handler(void * args) {
    ESP_LOGI("MAIN", "------------- INTERRUPT -------------\n");
    if (gpio_get_level(StatusLedPin)) {
        gpio_set_level(StatusLedPin, 0);
    } else {
        gpio_set_level(StatusLedPin, 1);
    }


    auto current_time = esp_timer_get_time() / 1000;
    if (!last_time_seen) {
        last_time_seen = current_time;
    } else {
        volcano_pulse_measurement = current_time - last_time_seen;
        ESP_LOGI("MAIN", "%lli - %lli = %lli", current_time, last_time_seen, volcano_pulse_measurement);
        last_time_seen = 0;
        volcano_pulse_measurement = 0;
    }

    vTaskDelete(NULL);
}

void isr_handler(void* args) {
    xTaskCreate(led_sensor_handler, "led sensor", 2048, nullptr, 4, NULL);
}

extern "C" void app_main(void)
{
    nvs_flash_init();

    gpio_reset_pin(StatusLedPin);
    gpio_set_direction(StatusLedPin, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(StatusLedPin, 1);
    vTaskDelay(REST);
    gpio_set_level(StatusLedPin, 0);
    vTaskDelay(REST);
    gpio_set_level(StatusLedPin, 1);

    ESP_LOGI("Main", "Status Pin Set");

    auto pinNumber = ServoPin;
    light_sensor_config.intr_type = GPIO_INTR_POSEDGE;
    light_sensor_config.mode = GPIO_MODE_INPUT;
    light_sensor_config.pin_bit_mask = 1ULL << pinNumber;
    light_sensor_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    light_sensor_config.pull_up_en = GPIO_PULLUP_DISABLE;
    auto err = gpio_config(&light_sensor_config);

    ESP_LOGI("Main", "Config Sensor attempt");
    if (ESP_ERR_INVALID_ARG == err) {
        vTaskDelete(NULL); // Delete Main Task
        return;
    }
    ESP_LOGI("Main", "Config Sensor success");

    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    gpio_isr_handler_add(pinNumber, isr_handler, nullptr);
    ESP_LOGI("Main", "Sensor handler set");

    // auto bt = BTClassicHID::getInstance();

    // // TODO: Put in your MAC Address
    // // esp_bd_addr_t joystickAddress{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    // esp_bd_addr_t joystickAddress{0xD0, 0x54, 0x7B, 0x39, 0x5E, 0xA1};
    // auto joystick = bt->connect<controller>(joystickAddress);

    // // TODO: Create the new motors for your car
    // Motor *left = new Motor(LeftMotorLeftPin, LeftMotorRightPin);
    // Motor *right = new Motor(RightMotorLeftPin, RightMotorRightPin);

    // // TODO: using the joystick and motor variables make a new car.
    // car = new Car(joystick, left, right);

    // // TODO: Set your LED/Servo/IR variable
    // redLed = new LED(RedLedPin);
    // blueLed = new LED(BlueLedPin);
    // greenLed = new LED(GreenLedPin);
    // servo = new ServoMotor(ServoPin);

    // // Don't forget to tell IR how to handle incoming transmissions
    // // TODO add log to remind to enable RX
    // ir->mSetReceiveHandler(onReceiveIRData);
    // ir->enableRx();
    // ir->enableTx();
    // registerJoystickButtonHandlers(joystick);
    vTaskDelete(NULL); // Delete Main Task
}