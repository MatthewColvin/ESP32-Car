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

#define LOG_TAG "MAIN"
// NOTE: To print out information while the program is running,
// use the syntax:
// ESP_LOGI(LOG_TAG, "Print information like %d \n", variableName);

/* PIN ASSIGNMENTS */
/* - These are all of the pins on your board you could be using in the challenge */
/* - The #define means these are variables where their value isn't allowed to change */
#define RightMotorLeftPin GPIO_NUM_16        // Motor
#define RightMotorRightPin GPIO_NUM_4
#define LeftMotorLeftPin GPIO_NUM_13
#define LeftMotorRightPin GPIO_NUM_5

#define StatusGreenLedPin GPIO_NUM_22        // Green Status LED on board (1 = on, 0 = off)

#define OnBoardRedLedPin GPIO_NUM_25         // LEDs on board (0 = on, 1 = off)
#define OnBoardGreenLedPin GPIO_NUM_33
#define OnBoardBlueLedPin GPIO_NUM_32

#define IRLED GPIO_NUM_19                    // IR Transceiver
#define IRDETECT GPIO_NUM_18

#define ServoPin GPIO_NUM_23                 // Servo Motor

#define ExternalRedLedPin GPIO_NUM_14        // LEDs on separate board (0 = off, 255 = full brightness)
#define ExternalGreenLedPin GPIO_NUM_26
#define ExternalBlueLedPin GPIO_NUM_27

/* GLOBAL VARIABLES */
/* - These variables can be used anywhere in this file */
Car *car;
// TODO ALL: Add your additional challenge variables
// Syntax Hint: <Class name>* <variable name>;
// Reminder: Make your variable names descriptive


/* JOYSTICK HANDLERS */
/* - These will be the functions called when you press a button on your controller */
/* - onPress will be called when you press the button down                         */
/* - onRelease will be called when you let the button go                           */
// TODO ALL: Fill in the functions if you want something to happen when you press/release any one of these buttons
// Syntax Hint: Put your line(s) of code in the {} curly brackets like other functions below
void onAPress(){};
void onARelease(){};
void onBPress(){};
void onBRelease(){};
void onXPress(){};
void onXRelease(){};
void onYPress(){};
void onYRelease(){};
void onTriggerPress(){};
void onTriggerRelease(){};

// This function tells the aJoystick object to call the above functions when specific buttons are pressed
// You can see it gets called at the bottom of the main function below
void registerJoystickButtonHandlers(std::shared_ptr<Mocute052> aJoystick)
{
    aJoystick->onA(onAPress, onARelease);
    aJoystick->onB(onBPress, onBRelease);
    aJoystick->onX(onXPress, onXRelease);
    aJoystick->onY(onYPress, onYRelease);
    aJoystick->onTrigger(onTriggerPress, onTriggerRelease);
}

/* IR Transceiver Receive Handler */
/* - This will be used for the IR challenge                             */
/* - This function is what gets called when you receive data through IR */
void onReceiveIRData(uint16_t address, uint16_t data, bool isRepeat)
{
    ESP_LOGI(LOG_TAG, "Address=%04X, Command=%04X\r\n\r\n", address, data);
}

/* MAIN C FUNCTION */
/* - Program starts here  */
extern "C" void app_main(void)
{
    nvs_flash_init();
    auto bt = BTClassicHID::getInstance();

    // TODO ALL: Put in your MAC Address (Replace 0xFF)
    esp_bd_addr_t joystickAddress{0xD0, 0x54, 0x7B, 0x00, 0xFF, 0xFF};
    auto joystick = bt->connect<controller>(joystickAddress);

    // TODO ALL: Create the new motors for your car
    // Syntax Hint: motorVariableName = new Motor(MotorLeftPin, MotorRightPin);
    
    // TODO ALL: Using the joystick and motor variables make a new car.
    // Syntax Hint: carVariableName = new Car(joystickVariable, leftMotorVariable, rightMotorVariable);
    
    // TODO (optional): If you'd like to use any of the LEDs on your board to signal
    //                  something has happened, set it up here.
    // Syntax Hint: gpio_reset_pin(ledPin);
    //              gpio_set_direction(ledPin, GPIO_MODE_OUTPUT);
    
    // TODO (challenge specific): Set your LED/Servo/IR variable
    // Syntax Hint: variableName = new Class(parameters, ...);

    // TODO (challenge specific): Tell IR how to handle incoming transmissions
    // Hint: do this with a function from the IR Transceiver class object
    
    // TODO (challenge specific): Enable IR to transmit & receive
    // Hint: do this with a function from the IR Transceiver class object

    registerJoystickButtonHandlers(joystick);
    vTaskDelete(NULL); // Delete Main Task
}
