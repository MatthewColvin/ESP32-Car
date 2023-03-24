#pragma once
// BLE Component
// ESP API
#include "esp_gattc_api.h"
// RTOS
// STD
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
    bool operator<(const Characteristic rhs) const { return mCharacteristic.char_handle < rhs.mCharacteristic.char_handle; }
    bool operator==(const Characteristic rhs) const { return char_handle() == rhs.char_handle(); }

    std::vector<esp_gattc_descr_elem_t> getDescriptors() const;

    void describe() const;
    void read();
    void readDescriptors();

    void write(uint8_t *value, uint16_t len);
    int uuid() const;
    std::string uuidstr() const;

    bool matchesFilters(uint8_t aFilter, PropFilterType aType = PropFilterType::Any, std::vector<int> uuidFilter = {}) const;

    uint16_t char_handle() const { return mCharacteristic.char_handle; }

    bool canNotify() { return mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY; }

private:
    uint8_t mDeviceGattIf;
    uint8_t mServiceConnId;
    esp_gattc_char_elem_t mCharacteristic;
};