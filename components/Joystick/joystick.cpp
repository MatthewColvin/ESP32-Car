#include "joystick.hpp"

#include "esp_log.h"

#define LOG_TAG "Joystick"

Joystick::Joystick(Device::bleScanResult aScanResult) : Device(aScanResult){};

void Joystick::init()
{
    constexpr uint8_t propFilter = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
    std::vector reportFilter{ESP_GATT_UUID_HID_BT_MOUSE_INPUT};
    for (auto service : mServicesFound)
    {
        auto reports = service.getCharacteristics(propFilter, Characteristic::PropFilterType::Any, reportFilter);
        for (auto report : reports)
        {
            mPositionChar = report;
            break;
        }
    }
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

void Joystick::getPosition()
{
    mPositionChar.read();
}

void Joystick::registerCharacteristics()
{
    constexpr uint8_t propFilter = ESP_GATT_UUID_HID_BT_MOUSE_INPUT;
    std::vector reportFilter{ESP_GATT_UUID_HID_REPORT};

    for (auto service : mServicesFound)
    {
        auto reports = service.getCharacteristics(propFilter, Characteristic::PropFilterType::Any, reportFilter);
        for (auto report : reports)
        {
            ESP_LOGI(LOG_TAG, "Service: %s", service.uuidstr().c_str());
            report.describe();
            esp_ble_gattc_register_for_notify(mGattcIf, mRemoteAddress, report.char_handle());
            enableNotifitcation(report);
        }
    }
    init();
}
