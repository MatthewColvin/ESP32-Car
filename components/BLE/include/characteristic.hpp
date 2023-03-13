#pragma once

#include "esp_gattc_api.h"

#include <vector>
#include <string>

class Characteristic
{
public:
    enum class PropFilterType
    {
        Any,
        All
    };

    Characteristic() = default; // Seems potentially bug prone
    Characteristic(uint8_t aDeviceGattIf, uint8_t mServiceConnId, esp_gattc_char_elem_t anIdfCharacteristic);
    std::vector<esp_gattc_descr_elem_t> getDescriptors() const;

    void describe() const;
    void read();
    int uuid() const;
    std::string uuidstr() const;

    bool matchesFilters(uint8_t aFilter, PropFilterType aType = PropFilterType::Any, std::vector<int> uuidFilter = {}) const;

    uint16_t char_handle() { return mCharacteristic.char_handle; }

private:
    uint8_t mDeviceGattIf;
    uint8_t mServiceConnId;
    esp_gattc_char_elem_t mCharacteristic;
};