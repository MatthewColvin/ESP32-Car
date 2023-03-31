#pragma once
#include "esp_hid_gap.h"
#include <string>

class HIDDevice
{
public:
    HIDDevice(esp_hid_scan_result_t aScanResult);

    std::string getName() { return std::string(mScanResult.name); }

private:
    esp_hid_scan_result_t mScanResult;
};