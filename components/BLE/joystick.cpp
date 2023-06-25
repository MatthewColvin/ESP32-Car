#include "joystick.hpp"

#include "esp_log.h"

// RTOS
#include "freertos/FreeRTOS.h"

#define LOG_TAG "Joystick"

Joystick::Joystick(Device::bleScanResult aScanResult) : Device(aScanResult){};

void Joystick::init()
{
    if (!isAuthenticated())
    {
        ESP_LOGE(LOG_TAG, "could not auth!!");
    }

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
    ESP_LOGI(LOG_TAG, "successful init %d service and %d reports", mServicesFound.size(), mHIDReports.size());
}

void Joystick::cycleReports()
{
    int i = 0;
    while (true)
    {
        int reportIdx = i % mHIDReports.size();
        unRegisterAllCharacteristicNotifications();
        ESP_LOGI(LOG_TAG, "registering report at idx:%d", reportIdx);

        registerForCharacteristicNotify(mHIDReports[reportIdx], [](Device::characteristicCbParamType aParam) -> int
                                        {
                ESP_LOGI(LOG_TAG,"Handle: %d   Data:",aParam.handle);
                ESP_LOG_BUFFER_HEX(LOG_TAG,aParam.value,aParam.value_len);
                return 5; });

        vTaskDelay(5000 / portTICK_PERIOD_MS);
        i++;
    }
}

void Joystick::nextReports(int start)
{
    // Unregister for all current characteristic's
    // mserviceCallbacks likely needs to be private and
    // need an unreg from all function.

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

int Joystick::handleJoystickReport(Device::characteristicCbParamType data)
{
    ESP_LOG_BUFFER_HEX(LOG_TAG, data.value, data.value_len);
    int16_t leftRight = data.value[0];
    int16_t upDown = data.value[2];
    ESP_LOGI(LOG_TAG, "x:%hd y:%hd", leftRight, upDown);
    return 0;
}

void Joystick::registerReportNotifications()
{
    mHIDReports[3].readDescriptors();
    registerForCharacteristicNotify(mHIDReports[3], std::bind(&Joystick::handleJoystickReport, this, std::placeholders::_1));
    // registerForCharacteristicNotify(mHIDReports[4], std::bind(&Joystick::handleJoystickReport, this, std::placeholders::_1));
}

void Joystick::readReportsDescriptors()
{
    for (auto report : mHIDReports)
    {
        ESP_LOGI(LOG_TAG, "report handle: %d ", report.char_handle());
        report.readDescriptors();
    }
}

void Joystick::readHIDReport()
{
    std::vector reportFilter{ESP_GATT_UUID_HID_REPORT_MAP};
    for (auto service : mServicesFound)
    {
        auto reports = service.getCharacteristics(0xFF, Characteristic::PropFilterType::Any, reportFilter);
        for (auto report : reports)
        {
            report.read();
        }
    }
    // Parsed by http://eleccelerator.com/usbdescreqparser/

    /*
       0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
       0x09, 0x02,        // Usage (Mouse)
       0xA1, 0x01,        // Collection (Application)
       0x85, 0x02,        //   Report ID (2)
       0x09, 0x01,        //   Usage (Pointer)
       0xA1, 0x00,        //   Collection (Physical)
       0x05, 0x09,        //     Usage Page (Button)
       0x19, 0x01,        //     Usage Minimum (0x01)
       0x29, 0x05,        //     Usage Maximum (0x05)
       0x15, 0x00,        //     Logical Minimum (0)
       0x25, 0x01,        //     Logical Maximum (1)
       0x95, 0x05,        //     Report Count (5)
       0x75, 0x01,        //     Report Size (1)
       0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
       0x95, 0x01,        //     Report Count (1)
       0x75, 0x03,        //     Report Size (3)
       0x81, 0x01,        //     Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
       0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
       0x09, 0x30,        //     Usage (X)
       0x09, 0x31,        //     Usage (Y)
       0x09, 0x38,        //     Usage (Wheel)
       0x15, 0x81,        //     Logical Minimum (-127)
       0x25, 0x7F,        //     Logical Maximum (127)
       0x75, 0x08,        //     Report Size (8)
       0x95, 0x03,        //     Report Count (3)
       0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
       0xC0,              //   End Collection
       0xC0,              // End Collection
    */
    /*
       0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
       0x09, 0x06,        // Usage (Keyboard)
       0xA1, 0x01,        // Collection (Application)
       0x85, 0x01,        //   Report ID (1)
       0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
       0x75, 0x08,        //   Report Size (8)
       0x95, 0x02,        //   Report Count (2)
       0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
       0x15, 0x00,        //   Logical Minimum (0)
       0x26, 0xFF, 0x00,  //   Logical Maximum (255)
       0x75, 0x08,        //   Report Size (8)
       0x95, 0x06,        //   Report Count (6)
       0x19, 0x00,        //   Usage Minimum (0x00)
       0x29, 0xFF,        //   Usage Maximum (0xFF)
       0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
       0xC0,              // End Collection
    */
    /*
       0x05, 0x0C,        // Usage Page (Consumer)
       0x09, 0x01,        // Usage (Consumer Control)
       0xA1, 0x01,        // Collection (Application)
       0x85, 0x03,        //   Report ID (3)
       0x15, 0x00,        //   Logical Minimum (0)
       0x26, 0x80, 0x03,  //   Logical Maximum (896)
       0x19, 0x00,        //   Usage Minimum (Unassigned)
       0x2A, 0x80, 0x03,  //   Usage Maximum (0x0380)
       0x75, 0x10,        //   Report Size (16)
       0x95, 0x01,        //   Report Count (1)
       0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
       0xC0,              // End Collection
    */
    /*
       0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
       0x09, 0x05,        // Usage (Game Pad)
       0xA1, 0x01,        // Collection (Application)
       0x85, 0x04,        //   Report ID (4)
       0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
       0x26, 0xFF, 0x00,  //   Logical Maximum (255)
       0x46, 0xFF, 0x00,  //   Physical Maximum (255)
       0x09, 0x30,        //   Usage (X)
       0x09, 0x31,        //   Usage (Y)
       0x09, 0x32,        //   Usage (Z)
       0x09, 0x35,        //   Usage (Rz)
       0x75, 0x08,        //   Report Size (8)
       0x95, 0x04,        //   Report Count (4)
       0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
       0x05, 0x09,        //   Usage Page (Button)
       0x19, 0x01,        //   Usage Minimum (0x01)
       0x29, 0x10,        //   Usage Maximum (0x10)
       0x95, 0x10,        //   Report Count (16)
       0x75, 0x01,        //   Report Size (1)
       0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
       0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
       0x09, 0x39,        //   Usage (Hat switch)
       0x15, 0x01,        //   Logical Minimum (1)
       0x25, 0x08,        //   Logical Maximum (8)
       0x35, 0x00,        //   Physical Minimum (0)
       0x46, 0x3B, 0x10,  //   Physical Maximum (4155)
       0x66, 0x0E, 0x00,  //   Unit (None)
       0x75, 0x04,        //   Report Size (4)
       0x95, 0x01,        //   Report Count (1)
       0x81, 0x42,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)
       0x75, 0x0C,        //   Report Size (12)
       0x95, 0x01,        //   Report Count (1)
       0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
       0xC0,              // End Collection
    */
}