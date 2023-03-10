#pragma once

#include "esp_gattc_api.h"

#include <vector>

class Characteristic
{
public:
    enum class FilterType
    {
        Any,
        All
    };

    Characteristic(uint8_t aDeviceGattIf, uint8_t mServiceConnId, esp_gattc_char_elem_t anIdfCharacteristic);

    void describe();
    std::vector<esp_gattc_descr_elem_t> getDescriptors();

    void read();

private:
    uint8_t mDeviceGattIf;
    uint8_t mServiceConnId;
    esp_gattc_char_elem_t mCharacteristic;
};