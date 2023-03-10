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

void Characteristic::describe() const
{
    ESP_LOGI(LOG_TAG, "Handle: %d UUID: %X Properties - Ext_Prop: %d Auth: %d Indicate: %d Notify: %d Write: %d Write_NR: %d Read: %d BroadCast: %d ",
             mCharacteristic.char_handle,
             uuid(),
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_EXT_PROP) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_AUTH) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_INDICATE) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_WRITE) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_READ) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_BROADCAST) > 0
             );
}

void Characteristic::read()
{
    esp_ble_gattc_read_char(mDeviceGattIf, mServiceConnId, mCharacteristic.char_handle, ESP_GATT_AUTH_REQ_NONE);
}

bool Characteristic::matchesFilters(uint8_t aFilter, PropFilterType aType, std::vector<int> uuidFilter) const
{
    bool isUUIDWanted = uuidFilter.empty() || std::find(uuidFilter.begin(), uuidFilter.end(), uuid()) != uuidFilter.end();
    if (isUUIDWanted)
    {
        if (aType == PropFilterType::Any)
        {
            return (mCharacteristic.properties & aFilter) > 0;
        }
         // all
        return (mCharacteristic.properties & aFilter) == aFilter;
    }
    return false;
}

int Characteristic::uuid() const{
    int retval = 0;
    switch(mCharacteristic.uuid.len){
        case ESP_UUID_LEN_16:
            retval = mCharacteristic.uuid.uuid.uuid16;
            break;
        case ESP_UUID_LEN_32:
            retval = mCharacteristic.uuid.uuid.uuid32;
            break;
        case ESP_UUID_LEN_128:
            ESP_LOGE(LOG_TAG, "C++ API NOT SUPPORTING 128BIT UUIDS FOR NOW");
            break;
        default:
            ESP_LOGE(LOG_TAG, "UUID IMPROPER len");

    }
    return retval;
}
