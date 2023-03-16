// BLE Component
#include "service.hpp"
// ESP API
#include "esp_log.h"
// RTOS
// STD

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
    uint16_t numFoundCharacteristics = 0;
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
                                            numFoundCharacteristics);
        if (status == ESP_GATT_OK)
        {
            Characteristic chara(mdeviceGattif, mService.conn_id, characteristic);
            if (chara.matchesFilters(propertiesFilter, filtertype, uuidFilter))
            {
                characteristics.push_back(chara);
            }
            numFoundCharacteristics++;
        }
        else if (status != ESP_GATT_NOT_FOUND)
        {
            ESP_LOGE(LOG_TAG, "FETCH CHAR STATUS: %d", status);
        }
    } while (numFoundCharacteristics < numCharacteristics && // ensure we found all characteristics
             status != ESP_GATT_NOT_FOUND);

    return characteristics;
}

void Service::describe()
{
    ESP_LOGI(LOG_TAG, "%s UUID: %X ", uuidstr().c_str(), uuid());
    auto characteristics = getCharacteristics();
    for (auto characteristic : characteristics)
    {
        characteristic.describe();
    }
}

int Service::uuid()
{
    int retval = 0;
    switch (mService.srvc_id.uuid.len)
    {
    case ESP_UUID_LEN_16:
        retval = mService.srvc_id.uuid.uuid.uuid16;
        break;
    case ESP_UUID_LEN_32:
        retval = mService.srvc_id.uuid.uuid.uuid32;
        break;
    case ESP_UUID_LEN_128:
        ESP_LOGE(LOG_TAG, "C++ API NOT SUPPORTING 128BIT UUIDS FOR NOW");
        break;
    default:
        ESP_LOGE(LOG_TAG, "UUID IMPROPER len");
    }
    return retval;
}

std::string Service::uuidstr()
{
    switch (uuid())
    {
    case ESP_GATT_UUID_IMMEDIATE_ALERT_SVC:
        return std::string("ESP_GATT_UUID_IMMEDIATE_ALERT_SVC");
    case ESP_GATT_UUID_LINK_LOSS_SVC:
        return std::string("ESP_GATT_UUID_LINK_LOSS_SVC");
    case ESP_GATT_UUID_TX_POWER_SVC:
        return std::string("ESP_GATT_UUID_TX_POWER_SVC");
    case ESP_GATT_UUID_CURRENT_TIME_SVC:
        return std::string("ESP_GATT_UUID_CURRENT_TIME_SVC");
    case ESP_GATT_UUID_REF_TIME_UPDATE_SVC:
        return std::string("ESP_GATT_UUID_REF_TIME_UPDATE_SVC");
    case ESP_GATT_UUID_NEXT_DST_CHANGE_SVC:
        return std::string("ESP_GATT_UUID_NEXT_DST_CHANGE_SVC");
    case ESP_GATT_UUID_GLUCOSE_SVC:
        return std::string("ESP_GATT_UUID_GLUCOSE_SVC");
    case ESP_GATT_UUID_HEALTH_THERMOM_SVC:
        return std::string("ESP_GATT_UUID_HEALTH_THERMOM_SVC");
    case ESP_GATT_UUID_DEVICE_INFO_SVC:
        return std::string("ESP_GATT_UUID_DEVICE_INFO_SVC");
    case ESP_GATT_UUID_HEART_RATE_SVC:
        return std::string("ESP_GATT_UUID_HEART_RATE_SVC");
    case ESP_GATT_UUID_PHONE_ALERT_STATUS_SVC:
        return std::string("ESP_GATT_UUID_PHONE_ALERT_STATUS_SVC");
    case ESP_GATT_UUID_BATTERY_SERVICE_SVC:
        return std::string("ESP_GATT_UUID_BATTERY_SERVICE_SVC");
    case ESP_GATT_UUID_BLOOD_PRESSURE_SVC:
        return std::string("ESP_GATT_UUID_BLOOD_PRESSURE_SVC");
    case ESP_GATT_UUID_ALERT_NTF_SVC:
        return std::string("ESP_GATT_UUID_ALERT_NTF_SVC");
    case ESP_GATT_UUID_HID_SVC:
        return std::string("ESP_GATT_UUID_HID_SVC");
    case ESP_GATT_UUID_SCAN_PARAMETERS_SVC:
        return std::string("ESP_GATT_UUID_SCAN_PARAMETERS_SVC");
    case ESP_GATT_UUID_RUNNING_SPEED_CADENCE_SVC:
        return std::string("ESP_GATT_UUID_RUNNING_SPEED_CADENCE_SVC");
    case ESP_GATT_UUID_Automation_IO_SVC:
        return std::string("ESP_GATT_UUID_Automation_IO_SVC");
    case ESP_GATT_UUID_CYCLING_SPEED_CADENCE_SVC:
        return std::string("ESP_GATT_UUID_CYCLING_SPEED_CADENCE_SVC");
    case ESP_GATT_UUID_CYCLING_POWER_SVC:
        return std::string("ESP_GATT_UUID_CYCLING_POWER_SVC");
    case ESP_GATT_UUID_LOCATION_AND_NAVIGATION_SVC:
        return std::string("ESP_GATT_UUID_LOCATION_AND_NAVIGATION_SVC");
    case ESP_GATT_UUID_ENVIRONMENTAL_SENSING_SVC:
        return std::string("ESP_GATT_UUID_ENVIRONMENTAL_SENSING_SVC");
    case ESP_GATT_UUID_BODY_COMPOSITION:
        return std::string("ESP_GATT_UUID_BODY_COMPOSITION");
    case ESP_GATT_UUID_USER_DATA_SVC:
        return std::string("ESP_GATT_UUID_USER_DATA_SVC");
    case ESP_GATT_UUID_WEIGHT_SCALE_SVC:
        return std::string("ESP_GATT_UUID_WEIGHT_SCALE_SVC");
    case ESP_GATT_UUID_BOND_MANAGEMENT_SVC:
        return std::string("ESP_GATT_UUID_BOND_MANAGEMENT_SVC");
    case ESP_GATT_UUID_CONT_GLUCOSE_MONITOR_SVC:
        return std::string("ESP_GATT_UUID_CONT_GLUCOSE_MONITOR_SVC");
    default:
        return std::string("UNKNOWN SERVICE");
    }
}
