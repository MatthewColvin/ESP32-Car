#pragma once
#include "esp_hid_gap.h"
#include <string>

class HIDDevice
{
public:
    HIDDevice(esp_hid_scan_result_t aScanResult);

    uint8_t *getAddress() { return mScanResult.bda; }
    bool hasAddress(const esp_bd_addr_t anAddress);
    esp_hid_transport_t getTransport() { return mScanResult.transport; }
    esp_ble_addr_type_t getAddressType();

private:
    esp_hid_scan_result_t mScanResult;
};