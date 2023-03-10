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

    void describe()const;
    void read();
    int uuid() const;

    bool matchesFilters(uint8_t aFilter, PropFilterType aType = PropFilterType::Any,std::vector<int> uuidFilter = {}) const;

    uint16_t char_handle(){ return mCharacteristic.char_handle; }

private:
    uint8_t mDeviceGattIf;
    uint8_t mServiceConnId;
    esp_gattc_char_elem_t mCharacteristic;
};