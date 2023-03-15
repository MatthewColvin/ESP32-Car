#include "joystick.hpp"

#include "esp_log.h"

#define LOG_TAG "Joystick"

Joystick::Joystick(Device::bleScanResult aScanResult) : Device(aScanResult){};

void Joystick::init()
{

}

void Joystick::readAll()
{
    constexpr uint8_t propFilter = ESP_GATT_CHAR_PROP_BIT_READ;
    for (auto s : mServicesFound)
    {
        for (auto c : s.getCharacteristics())
        {
            c.read();
        }
    }
}

void Joystick::registerReportNotifications()
{
    constexpr uint8_t propFilter = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
    std::vector reportFilter{ESP_GATT_UUID_HID_REPORT};

    for (auto service : mServicesFound)
    {
        auto reports = service.getCharacteristics(propFilter, Characteristic::PropFilterType::Any, reportFilter);
        for (auto report : reports)
        {
            ESP_LOGI(LOG_TAG, "Service: %s", service.uuidstr().c_str());
            report.describe();
            registerforCharacteristicNotify(report,[](Device::characteristicCbParamType aParam) -> int {
                ESP_LOGI(LOG_TAG,"HOLY CRAP IT WORKED");
                return 5;});
        }
    }
    init();
}
