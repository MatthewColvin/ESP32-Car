#pragma once

#include "esp_gattc_api.h"

#include <vector>

class Characteristic
{
public:
    enum class PropFilterType
    {
        Any,
        All
    };

    Characteristic(uint8_t aDeviceGattIf, uint8_t mServiceConnId, esp_gattc_char_elem_t anIdfCharacteristic);
    std::vector<esp_gattc_descr_elem_t> getDescriptors();

    void describe();
    void read();

    bool matchesFilters(uint8_t aFilter = 0b11111111, PropFilterType aType = PropFilterType::Any, std::vector<int> uuidFilter = {});

private:
    uint8_t mDeviceGattIf;
    uint8_t mServiceConnId;
    esp_gattc_char_elem_t mCharacteristic;
};