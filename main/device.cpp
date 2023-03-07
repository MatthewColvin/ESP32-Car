#include "device.hpp"
#include "esp_gattc_api.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"

#include <cstring>

#define LOG_TAG "Device"

bool operator<(const Device::serviceUUIDType &aLeftUUID, const Device::serviceUUIDType &aRightUUID)
{
    if (aLeftUUID.len != aRightUUID.len)
    {
        ESP_LOGE(LOG_TAG, "WARNING: Comparing 2 different lengths of UUIDs.");
    }

    switch (aLeftUUID.len)
    {
    case ESP_UUID_LEN_16:
        return aLeftUUID.uuid.uuid16 < aRightUUID.uuid.uuid16;
    case ESP_UUID_LEN_32:
        return aLeftUUID.uuid.uuid32 < aRightUUID.uuid.uuid32;
    case ESP_UUID_LEN_128:
        ESP_LOGE(LOG_TAG, "128 BIT UUID compare totally guessed on implementation...");
        return aLeftUUID.uuid.uuid128[ESP_UUID_LEN_128 - 1] < aRightUUID.uuid.uuid128[ESP_UUID_LEN_128 - 1];
    }

    ESP_LOGE(LOG_TAG, "UUID Compare FAILED DUE TO Invalid len %d", aLeftUUID.len);
    return false;
}

std::string uuidToStr(Device::serviceUUIDType aUUID)
{
    std::string result;
    switch (aUUID.len)
    {
    case ESP_UUID_LEN_16:
        result = std::to_string(aUUID.uuid.uuid16);
        break;
    case ESP_UUID_LEN_32:
        result = std::to_string(aUUID.uuid.uuid32);
        break;
    case ESP_UUID_LEN_128:
        result = std::string(reinterpret_cast<const char *>(aUUID.uuid.uuid128), ESP_UUID_LEN_128);
        break;
    default:
        result = "FAILED TO MAKE UUID INTO STR";
        ESP_LOGE(LOG_TAG, "Failed to make UUID string UUID.len = %d", aUUID.len);
    }
    return result;
}
Device::Device(bleScanResult res) { mScanResult = res; }

std::string Device::getName()
{
    uint8_t *cstr_name = NULL;
    uint8_t name_len = 0;

    cstr_name = esp_ble_resolve_adv_data(mScanResult.ble_adv,
                                         ESP_BLE_AD_TYPE_NAME_CMPL, &name_len);

    std::string name;
    name.assign(reinterpret_cast<char *>(cstr_name), name_len);

    return name;
}

esp_bd_addr_t *Device::getAddress()
{
    // esp_log_buffer_hex(LOG_TAG, scan_result->scan_rst.bda, 6);
    // Decided to go with this but if they save the address of and the device
    // falls out of scope we have issues
    return &mScanResult.bda;
}

esp_ble_addr_type_t Device::getAddressType()
{
    return mScanResult.ble_addr_type;
}

void Device::openConnection(OpenEventInfo aOpenEvent)
{

    mConnected = true;

    memcpy(mRemoteAddress, aOpenEvent.remote_bda, sizeof(esp_bd_addr_t));
    mConnectionId = aOpenEvent.conn_id;
}

bool Device::isConnected() { return mConnected; }
esp_bd_addr_t *Device::getRemoteAddress() { return &mRemoteAddress; }
uint16_t Device::getConnectionId() { return mConnectionId; }

uint8_t Device::getGattcIf() { return mGattcIf; }
void Device::setGattcIf(uint8_t aGattcIf) { mGattcIf = aGattcIf; }

void Device::searchServices()
{
    if (!mConnected)
    {
        ESP_LOGE(LOG_TAG, "Cannot Search for Services without connection!");
        return;
    }
    ESP_LOGI(LOG_TAG, "Searching %s for Services", getName().c_str());
    mIsServiceSearching = true;
    auto res = esp_ble_gattc_search_service(mGattcIf, mConnectionId, nullptr);
    if (res != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "ERROR in Service Search");
    }
}

void Device::addFoundService(ServiceSearchResult aService)
{
    mServicesFound.push_back(aService);
}
void Device::serviceSearchComplete()
{
    // ESP_LOGI(LOG_TAG, "Sevices for %s", getName().c_str());
    // for (ServiceSearchResult service : mServicesFound)
    // {
    //     esp_gatt_id_t attributeId = service.srvc_id;
    //     ESP_LOGI(LOG_TAG, "UUID: %d", attributeId.uuid.uuid.uuid16);
    // }

    mIsServiceSearching = true;
}
bool Device::isServicesSearchComplete() { return mIsServiceSearching; }

void Device::registerService(characterHandleType aCharacteristicHndl, characteristicCallbackType aCallback)
{
    mserviceCallbacks.emplace(aCharacteristicHndl, std::move(aCallback));
}
Device::serviceCbRetType Device::handleService(characteristicCbParamType aParam)
{
    characterHandleType hndl = aParam.handle;
    ESP_LOGI(LOG_TAG, "Handeling Characteristic: %d for %s", hndl, getName().c_str());
    if (auto callbackPair = mserviceCallbacks.find(hndl); callbackPair != mserviceCallbacks.end())
    {
        return callbackPair->second(aParam);
    }
    else
    {
        ESP_LOGI(LOG_TAG, "got some handle service call we couldn't handle");
    }

    return 10; // TODO need to update serviceCbRetType when better understand ESPIDF api
    // Do we need to let the API know we failed to handle service???
}

std::vector<esp_gattc_char_elem_t> Device::getCharacteristics(Device::ServiceSearchResult aService)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    std::vector<esp_gattc_char_elem_t> characteristics;

    uint16_t numCharacteristics = 1;
    do
    {
        esp_gattc_char_elem_t characteristic;
        status = esp_ble_gattc_get_all_char(mGattcIf,
                                            aService.conn_id,
                                            aService.start_handle,
                                            aService.end_handle,
                                            &characteristic,
                                            &numCharacteristics,
                                            characteristics.size());
        if (status == ESP_GATT_OK)
        {
            characteristics.push_back(characteristic);
        }
        else if (status != ESP_GATT_NOT_FOUND)
        {
            ESP_LOGE(LOG_TAG, "FETCH CHAR STATUS: %d", status);
        }

    } while (characteristics.size() < numCharacteristics && status != ESP_GATT_NOT_FOUND);

    return characteristics;
}

std::vector<esp_gattc_descr_elem_t> Device::getDescriptors(esp_gattc_char_elem_t aCharacteristic)
{
    ESP_LOGE(LOG_TAG, "1Broken?");

    esp_gatt_status_t status = ESP_GATT_OK;
    std::vector<esp_gattc_descr_elem_t> descriptors;
    uint16_t numDescriptions = 1;

    do
    {
        esp_gattc_descr_elem_t charaDescription;
        status = esp_ble_gattc_get_all_descr(mGattcIf,
                                             mConnectionId,
                                             aCharacteristic.char_handle,
                                             &charaDescription,
                                             &numDescriptions,
                                             descriptors.size());
        if (status == ESP_GATT_OK)
        {
            ESP_LOGE(LOG_TAG, "Broken? size= %d", descriptors.size());
            descriptors.push_back(charaDescription);
            ESP_LOGE(LOG_TAG, "post Broken? size= %d", descriptors.size());
        }
        else if (status != ESP_GATT_NOT_FOUND)
        {
            ESP_LOGE(LOG_TAG, "not found Broken?");

            ESP_LOGE(LOG_TAG, "FETCH DESC STATUS: %d", status);
        }
    } while (descriptors.size() < numDescriptions && status != ESP_GATT_NOT_FOUND);

    ESP_LOGE(LOG_TAG, "exit Broken?");

    return descriptors;
}

void Device::describeServices()
{
    ESP_LOGE(LOG_TAG, "enter des serv Broken?");

    for (auto service : mServicesFound)
    {
        describeService(service);
        ESP_LOGE(LOG_TAG, "service desc Broken?");
    }
}

void Device::describeService(Device::ServiceSearchResult aService)
{
    ESP_LOGI(LOG_TAG, "%s Service UUID: %s ", getName().c_str(), uuidToStr(aService.srvc_id.uuid).c_str());
    ESP_LOGI(LOG_TAG, "HEAP Left: %ld", xPortGetFreeHeapSize());
    auto characteristics = getCharacteristics(aService);
    ESP_LOGI(LOG_TAG, "FOUND %d CHARAs", characteristics.size());
    int count = 0;
    for (auto characteristic : characteristics)
    {
        ESP_LOGI(LOG_TAG, "chara num %d ", count);
        ESP_LOGI(LOG_TAG, "HEAP Left: %ld", xPortGetFreeHeapSize());
        ESP_LOGI(LOG_TAG, "----Characteristic UUID: %s", uuidToStr(characteristic.uuid).c_str());
        auto descriptors = getDescriptors(characteristic);
        ESP_LOGI(LOG_TAG, "broken here?? descriptors size %d", descriptors.size());
        if (descriptors.size() != 0)
        {
            for (auto descriptor : descriptors)
            {
                ESP_LOGI(LOG_TAG, "broken here??");
                ESP_LOGI(LOG_TAG, "--------Descriptor UUID: %s", uuidToStr(descriptor.uuid).c_str());
            }
        }
        ESP_LOGI(LOG_TAG, "broken here?? HOW?? %d", descriptors.size());
        count++;
    }
    ESP_LOGI(LOG_TAG, "broken here why?????? ");
}

void Device::registerForJoystickCharactistics()
{
    for (auto service : mServicesFound)
    {
        for (auto characteristic : getCharacteristics(service))
            if (characteristic.uuid.uuid.uuid16 == ESP_GATT_UUID_HID_REPORT && characteristic.properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
            {
                describeService(service);
                return;
                // esp_ble_gattc_register_for_notify(mGattcIf, mRemoteAddress, characteristic.char_handle);
            }
    }
}