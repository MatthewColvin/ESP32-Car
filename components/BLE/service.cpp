// Library
#include "service.hpp"

// ESP API
#include "esp_log.h"

#define LOG_TAG "Service"

Service::Service(Service::espServiceTy anEspService) : mService(anEspService)
{
}

std::vector<esp_gattc_char_elem_t> Service::getCharacteristics(uint8_t aGattIf, uint8_t propertiesFilter, Characteristic::FilterType filtertype, std::vector<int> uuidFilter)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    std::vector<esp_gattc_char_elem_t> characteristics;

    uint16_t numCharacteristics; // outside scope of loop for end read check
    uint16_t numFilterdOutCharacteristics = 0;
    do
    {
        numCharacteristics = 1; // only fetch one at a time
        esp_gattc_char_elem_t characteristic;
        status = esp_ble_gattc_get_all_char(aGattIf,
                                            mService.conn_id,
                                            mService.start_handle,
                                            mService.end_handle,
                                            &characteristic,
                                            &numCharacteristics, // this will update to total number of characteristics
                                            characteristics.size());
        if (status == ESP_GATT_OK)
        {
            bool isUUIDWanted = uuidFilter.empty() || std::find(uuidFilter.begin(), uuidFilter.end(), characteristic.uuid.uuid.uuid32) != uuidFilter.end();
            // Trust me on the bit mask :)
            bool isPropWanted = filtertype == Characteristic::FilterType::Any ? characteristic.properties & propertiesFilter : ((characteristic.properties & propertiesFilter) == propertiesFilter);
            if (isPropWanted && isUUIDWanted)
            {
                characteristics.push_back(characteristic);
            }
            else
            {
                numFilterdOutCharacteristics++;
            }
        }
        else if (status != ESP_GATT_NOT_FOUND)
        {
            ESP_LOGE(LOG_TAG, "FETCH CHAR STATUS: %d", status);
        }
    } while (characteristics.size() + numFilterdOutCharacteristics < numCharacteristics && // ensure we found all characteristics
             status != ESP_GATT_NOT_FOUND);

    return characteristics;
}