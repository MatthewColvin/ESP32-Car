#include "joystick.hpp"

#include "esp_log.h"

#define LOG_TAG "Joystick"

Joystick::Joystick(Device::bleScanResult aScanResult) : Device(aScanResult){};

void Joystick::init()
{
    // TODO: binary semaphore in device during construction to block until its done?
    while (!isServicesSearchComplete()) // Block until we finish service discovery
    {
    };

    constexpr uint8_t propFilter = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
    std::vector reportFilter{ESP_GATT_UUID_HID_REPORT};
    for (auto service : mServicesFound)
    {
        auto reports = service.getCharacteristics(propFilter, Characteristic::PropFilterType::Any, reportFilter);
        for (auto report : reports)
        {
            mHIDReports.push_back(report);
        }
    }
}

void Joystick::nextReports(int start)
{
    // Unregister for all current characteristic's
    for (auto registration : mserviceCallbacks)
    {
        unRegisterForCharacteristicNotify(registration.first);
    }

    for (int i = start; i < start + 3; i++)
    {
        int reportIdx = i % mHIDReports.size();
        ESP_LOGI(LOG_TAG, "Registering Report #%d", reportIdx);

        registerForCharacteristicNotify(mHIDReports[reportIdx], [](Device::characteristicCbParamType aParam) -> int
                                        {
                ESP_LOGI(LOG_TAG,"Handle: %d   Data:",aParam.handle);
                ESP_LOG_BUFFER_HEX(LOG_TAG,aParam.value,aParam.value_len);
                return 5; });
    }
}

void Joystick::registerReportNotifications()
{
}
