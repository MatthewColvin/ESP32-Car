#pragma once
#include "esp_hid_gap.h"
#include <string>

class HIDDevice
{
public:
    HIDDevice(esp_hid_scan_result_t aScanResult);

    uint8_t* getAddress() {return mScanResult.bda; }
    bool hasAddress(esp_bd_addr_t anAddress);

private:
    esp_hid_scan_result_t mScanResult;
};