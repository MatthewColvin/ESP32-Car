#include "mocute052.hpp"
#include "motor.hpp"
#include "BTClassicHID.hpp"
#include "car.hpp"

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include <memory>
#define LOG_TAG "main"

#define RightMotorLeftPin  15
#define RightMotorRightPin 2
#define LeftMotorLeftPin 16
#define LeftMotorRightPin 17

Car* car;

void onAPress(){ESP_LOGI(LOG_TAG,"A Press");};
void onARelease(){ESP_LOGI(LOG_TAG,"A Release");};
void onBPress(){ ESP_LOGI(LOG_TAG,"B Press");};
void onBRelease(){ESP_LOGI(LOG_TAG,"B Release");};
void onXPress(){ESP_LOGI(LOG_TAG,"X Press");};
void onXRelease(){ESP_LOGI(LOG_TAG,"X Release");};
void onYPress(){ESP_LOGI(LOG_TAG,"Y Press");};
void onYRelease(){ESP_LOGI(LOG_TAG,"Y Release");};
void onTriggerPress(){ESP_LOGI(LOG_TAG,"Trigger Press");};
void onTriggerRelease(){ESP_LOGI(LOG_TAG,"Trigger Release");};
void onJoystick(uint8_t x, uint8_t y){ ESP_LOGI(LOG_TAG,"X:%d Y:%d",x,y);}


void registerJoystickButtonHandlers(std::shared_ptr<Mocute052> aJoystick){
    aJoystick->onA(onAPress,onARelease);
    aJoystick->onB(onBPress,onBRelease);
    aJoystick->onX(onXPress,onXRelease);
    aJoystick->onY(onYPress,onYRelease);
    aJoystick->onTrigger(onTriggerPress,onTriggerRelease);
    aJoystick->onJoyStick(onJoystick);
}

extern "C" void app_main(void)
{
    nvs_flash_init();
    auto bt = BTClassicHID::getInstance();
    esp_bd_addr_t joystickAddress{0xe0, 0xf8, 0x48, 0x05, 0x29, 0x50};

    while (true){
        auto results = bt->scan(5);

        printf("\n");
        for (int i = 0; i<results.size();i++){
            auto address = results[i].getAddress();
            printf("Found HID Device #(%d)  ", i);
            for (int j = 0; j<sizeof(esp_bd_addr_t); j++){
                printf("%d:",address[j]);
            }
            printf("\n");
        }
        ESP_LOGI(LOG_TAG,"Please enter number within 5 seconds to connect.");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        auto input = getchar();
        if(input>0 && input < results.size()){
            auto joystick = bt->connect<Mocute052>(results[input].getAddress());
            registerJoystickButtonHandlers(joystick);
            break;
        }else{
            ESP_LOGI(LOG_TAG,"Rescanning...");
        }
    }

    vTaskDelete(NULL); // Delete Main Task
}
