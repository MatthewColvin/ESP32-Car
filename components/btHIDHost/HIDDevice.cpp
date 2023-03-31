#include "HIDDevice.hpp"

#include <cstring>

#define LOG_TAG "HIDDevice"

HIDDevice::HIDDevice(esp_hid_scan_result_t aScanResult) : mScanResult(aScanResult)
{
}

bool HIDDevice::hasAddress(esp_bd_addr_t anAddress){
    if(!mScanResult.bda){return false;}

    for(uint8_t i =0; i<sizeof(esp_bd_addr_t); i++){
        if(anAddress[i] != mScanResult.bda[i]){
            return false;
        }
    }
    return true;
}

