#include "characteristic.hpp"

#include "esp_log.h"

#define LOG_TAG "Characteristic"

Characteristic::Characteristic(esp_gattc_char_elem_t anIdfCharacteristic) : mCharacteristic(anIdfCharacteristic)
{
}

void Characteristic::describe()
{
    // ESP_LOGI(LOG_TAG, "---Handle: %d , UUID: %s", mCharacteristic.char_handle, uuidToStr(mCharacteristic.uuid).c_str());
    ESP_LOGI(LOG_TAG, "------ Properties- BroadCast: %d Read: %d Write_NR: %d Write: %d Notify: %d Indicate: %d Auth: %d Ext_Prop: %d ",
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_BROADCAST) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_READ) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_WRITE) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_INDICATE) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_AUTH) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_EXT_PROP) > 0);
}

void Characteristic::read(uint8_t mDeviceGattIf, uint8_t serviceConnId)
{
    esp_ble_gattc_read_char(mDeviceGattIf, serviceConnId, mCharacteristic.char_handle, ESP_GATT_AUTH_REQ_NONE);
}