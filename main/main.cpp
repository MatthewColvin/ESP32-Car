#include "ble.hpp"
#include "BTClassicHID.hpp"
#include "mocute052.hpp"
#include "motor.hpp"

#include "driver/gpio.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include <cmath>
#include <memory>

#define LOG_TAG "Main"

extern "C" void app_main(void)
{
    nvs_flash_init();
    auto bt = BTClassicHID::getInstance();

    Motor left(15, 2);
    Motor right(16, 17);

    bool joystickConnected = false;
    std::shared_ptr<Mocute052> classicJoystick = nullptr;
    while (!joystickConnected)
    {
        auto devices = bt->scan(10);
        esp_bd_addr_t remoteAddress{0xe0, 0xf8, 0x48, 0x05, 0x29, 0x50};

        ESP_LOGI(LOG_TAG, "found %d devices", devices.size());
        for (auto device : devices)
        {
            if (device.hasAddress(remoteAddress))
            {
                classicJoystick = std::make_shared<Mocute052>(device);
                if (bt->connect(classicJoystick))
                {
                    ESP_LOGI(LOG_TAG, "Connected!");
                    ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(classicJoystick->getAddress()));
                    joystickConnected = true;
                    classicJoystick->setJoystickHandler([left, right](uint8_t x, uint8_t y) mutable
                                                        {
                                                            float refX = x - 128;
                                                            float refY = -1* (y - 128);
                                                            //convert xy to be like 4 quadrant grid while holding joystick in portrait.
                                                            // Use pathergy theorem to find distance from center and determine speed. 0 - 128
                                                            // Both motors will distribute this based on x bias                                                        
                                                            
                                                            constexpr float MixingZoneUpperLimit = 1;
                                                            constexpr float MixingZoneLowerLimit = .25;
                                                            // Smooth left and right turns both motors going forward or back throttle based on y value
                                                            bool isDirectionalZone = (refY > abs(refX) * MixingZoneUpperLimit) || (refY < abs(refX) * MixingZoneUpperLimit * -1);
                                                            // starting to move toward a zero turn type control need to pull power from one motor in this zone rely more on x to determine distributions
                                                            bool isMixingZone = (abs(refX) * MixingZoneUpperLimit > refY && refY > abs(refX) * MixingZoneLowerLimit) || 
                                                                                (abs(refX) * MixingZoneUpperLimit * -1 < refY && refY < abs(refX) * MixingZoneLowerLimit *-1);
                                                            // motors are moving opposite directions                                             
                                                            bool isZeroTurnZone = (0 < refY && refY < abs(refX) * MixingZoneLowerLimit) || 
                                                                                  (0 > refY && refY > abs(refX) * MixingZoneLowerLimit * -1);


                                                            float totalSpeed = sqrt(refX * refX + refY * refY);                                                            
                                                            float rightMotorSpeed = 0;
                                                            float leftMotorSpeed =  0;
                                                            float xBias = refX / 128.0;                                                                                                                                                                                    
                                                            if (isDirectionalZone){
                                                                if(xBias > 0){ // right turn
                                                                    leftMotorSpeed = xBias * totalSpeed;
                                                                    rightMotorSpeed = totalSpeed - leftMotorSpeed;
                                                                }else if (xBias < 0){ // left turn
                                                                    rightMotorSpeed = xBias * totalSpeed;
                                                                    leftMotorSpeed = totalSpeed - rightMotorSpeed;
                                                                }else{ // no bias straight forward or back
                                                                    rightMotorSpeed = totalSpeed *.5;
                                                                    leftMotorSpeed = totalSpeed *.5;
                                                                } 
                                                                ESP_LOGI(LOG_TAG,"Directional Zone");   
                                                            }
                                                            if(isMixingZone){
                                                                ESP_LOGI(LOG_TAG,"Mixing Zone");   
                                                            }if(isZeroTurnZone){
                                                                ESP_LOGI(LOG_TAG,"Zero turn Zone");   
                                                            }                                                                                                        
                                                            //right.setSpeed(rightMotorSpeed);
                                                            //left.setSpeed(leftMotorSpeed);

                                                            ESP_LOGI(LOG_TAG,"refx:%f,refy:%f xbias: %f Total Speed:%f RightSpeed: %f, LeftSpeed: %f",refX,refY, xBias, totalSpeed, rightMotorSpeed, leftMotorSpeed); });
                }
            }
        }

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL); // Delete Main Task
}
