#include "characteristic.hpp"

#include "esp_log.h"

#define LOG_TAG "Characteristic"

Characteristic::Characteristic(uint8_t aDeviceGattIf, uint8_t aServiceConnId, esp_gattc_char_elem_t anIdfCharacteristic) : mDeviceGattIf(aDeviceGattIf),
                                                                                                                           mServiceConnId(aServiceConnId),
                                                                                                                           mCharacteristic(anIdfCharacteristic)
{
}

std::vector<esp_gattc_descr_elem_t> Characteristic::getDescriptors()
{
    esp_gatt_status_t status = ESP_GATT_OK;
    std::vector<esp_gattc_descr_elem_t> descriptors;
    uint16_t numDescriptions; // outside scope of loop for end read check
    do
    {
        numDescriptions = 1; // only fetch one at a time
        esp_gattc_descr_elem_t charaDescription;
        status = esp_ble_gattc_get_all_descr(mDeviceGattIf,
                                             mServiceConnId,
                                             mCharacteristic.char_handle,
                                             &charaDescription,
                                             &numDescriptions, // this will update to total number of descriptions
                                             descriptors.size());
        if (status == ESP_GATT_OK)
        {
            descriptors.push_back(charaDescription);
        }
        else if (status != ESP_GATT_NOT_FOUND)
        {
            ESP_LOGE(LOG_TAG, "FETCH DESC STATUS: %d", status);
        }
    } while (descriptors.size() < numDescriptions && // Check num descriptions
             status != ESP_GATT_NOT_FOUND);

    return descriptors;
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

void Characteristic::read()
{
    esp_ble_gattc_read_char(mDeviceGattIf, mServiceConnId, mCharacteristic.char_handle, ESP_GATT_AUTH_REQ_NONE);
}

bool Characteristic::matchesFilters(uint8_t aFilter, PropFilterType aType, std::vector<int> uuidFilter)
{
    bool isUUIDWanted = uuidFilter.empty() || std::find(uuidFilter.begin(), uuidFilter.end(), characteristic.uuid.uuid.uuid32) != uuidFilter.end();
    if (isUUIDWanted)
    {
        ESP_LOGI(LOG_TAG, "Filter: %x Prop: %x", aFilter, mCharacteristic.properties)
        if (aType == PropFilterType::Any)
        {
            return (mCharacteristic.properties & aFilter) > 0;
        }
        else // all
        {
            return (mCharacteristic.properties & aFilter) == aFilter;
        }
    }
    return false;
}
