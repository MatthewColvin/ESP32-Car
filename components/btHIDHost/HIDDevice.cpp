#include "HIDDevice.hpp"

#include <cstring>

#define LOG_TAG "HIDDevice"

HIDDevice::HIDDevice(esp_hid_scan_result_t aScanResult) : mScanResult(aScanResult)
{
}

bool HIDDevice::hasAddress(const esp_bd_addr_t anAddress)
{
    if (!mScanResult.bda)
    {
        return false;
    }

    for (uint8_t i = 0; i < sizeof(esp_bd_addr_t); i++)
    {
        if (anAddress[i] != mScanResult.bda[i])
        {
            return false;
        }
    }
    return true;
}

esp_ble_addr_type_t HIDDevice::getAddressType()
{
    if (mScanResult.transport == ESP_HID_TRANSPORT_BLE)
    {
        return mScanResult.ble.addr_type;
    }
    // returning first enum since this should not matter anyway
    // because the transport is BT classic
    return BLE_ADDR_TYPE_PUBLIC;
};
