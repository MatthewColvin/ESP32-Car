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

/*
  GLOBAL VARIABLES

  Tip: nullptr means we currently have no object stored in that variable.
*/

Car *car = nullptr;
Motor *left = nullptr;
Motor *right = nullptr;
std::shared_ptr<controller> joystick = nullptr;

ServoMotor *servo = nullptr;
LED* red = nullptr;
LED* green = nullptr;
LED* blue = nullptr;

// Use Area below to make the variable for your sensors similar to above examples
// <variableType> *<variableName> = nullptr;

///////////////////////////////////////////////////////////////////


/*
  TODO: Code your car to do things when you press buttons.

  Tip: You can use one button for more than one action
  by checking if other buttons are held down by using the
  joystick variable.See code below

  if (joystick->isTriggerDown()){
    // Do something with trigger down
  }else{
    // Do other thing when trigger is up
  }
*/

void onAPress(){
  /* This will run when you press the A button down */

}

void onARelease(){
  /*This will run when you release the A button*/
}

void onBPress(){
  // Tip: try typing <variableName>-> and then wait for editor to show you what you can do.
  // Lets say you want to make your red light come on but don't know how...

  // Try typing red->
  // The editor will show you all the things that you can do with that variable.
  // in that list you see setBrightness so click it to auto fill
  // red->setBrightness
  // Now add a parenthesis to see what the function takes
  // red->setBrightness(
  // You will notice a window pop up and see it takes an int which is just a number so fill it in
  // red->setBrightness(50)
  // Dont forget to add your semicolon
  // red->setBrightness(50);
  // This is a full working line of code that will turn on your red LED.
  // Maybe you could try to move your servo next?
}
void onBRelease(){}

void onXPress(){}
void onXRelease(){}

void onYPress(){}
void onYRelease(){}

void onTriggerPress() {};
void onTriggerRelease(){};

void postCarInitSetup(){
    servo = new ServoMotor(ServoPin);
    red = new LED(RedLedPin);
    green = new LED(GreenLedPin);
    blue = new LED(BlueLedPin);

    if(car){
      // TODO: Maybe you could update this line to make the car faster?
      car->setCruiseSpeed(Motor::MAX_SPEED - 5000);
    }
    if (joystick){
      joystick->onA(onAPress,onARelease);
      joystick->onB(onBPress,onBRelease);
      joystick->onX(onXPress,onXRelease);
      joystick->onY(onYPress,onYRelease);
      joystick->onTrigger(onTriggerPress, onTriggerRelease);
    }
}

extern "C" void app_main(void) {
  nvs_flash_init();
  auto bt = BTClassicHID::getInstance();

  // TODO: Put in your MAC Address
  esp_bd_addr_t joystickAddress{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  joystick = bt->connect<controller>(joystickAddress);

  /*
   The motor variables for your car have already been set below use
   them to remember the format of setting a variable
  */
  left = new Motor(LeftMotorLeftPin, LeftMotorRightPin);
  right = new Motor(RightMotorLeftPin, RightMotorRightPin);

  // TODO: Set your Car Variable to enable driving the car
  // Format: <VariableName> = new <VariableType>(arg1,arg2...);

  postCarInitSetup();
  vTaskDelete(NULL); // Delete Main Task
}