// Library
#include "service.hpp"

// ESP API
#include "esp_log.h"

#define LOG_TAG "Service"

Service::Service(uint8_t aDeviceGattIf, Service::espIdfTy anEspService) : mdeviceGattif(aDeviceGattIf),
                                                                          mService(anEspService)
{
}

std::vector<Characteristic> Service::getCharacteristics(uint8_t propertiesFilter, Characteristic::PropFilterType filtertype, std::vector<int> uuidFilter)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    std::vector<Characteristic> characteristics;

    uint16_t numCharacteristics; // outside scope of loop for end read check
    uint16_t numFilterdOutCharacteristics = 0;
    do
    {
        numCharacteristics = 1; // only fetch one at a time
        esp_gattc_char_elem_t characteristic;
        status = esp_ble_gattc_get_all_char(mdeviceGattif,
                                            mService.conn_id,
                                            mService.start_handle,
                                            mService.end_handle,
                                            &characteristic,
                                            &numCharacteristics, // this will update to total number of characteristics
                                            characteristics.size());
        if (status == ESP_GATT_OK)
        {
            Characteristic chara(mdeviceGattif, mService.conn_id, characteristic);
            if (chara.matchesFilters(propertiesFilter, filtertype, uuidFilter))
            {
                characteristics.push_back(chara);
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

void Service::describe()
{
    ESP_LOGI(LOG_TAG, "Service UUID: %d ", uuid());
    auto characteristics = getCharacteristics();
    for (auto characteristic : characteristics)
    {
        characteristic.describe();
    }
}