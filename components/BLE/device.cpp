#include "device.hpp"
#include "esp_gattc_api.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"

#include <cstring>
#include <algorithm>

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

void Device::registerCharacteristic(characterHandleType aCharacteristicHndl, characteristicCallbackType aCallback)
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

std::vector<esp_gattc_char_elem_t> Device::getCharacteristics(const Device::ServiceSearchResult &aService, uint8_t propertiesFilter, CharacteristicFilterType filtertype, std::vector<int> uuidFilter)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    std::vector<esp_gattc_char_elem_t> characteristics;

    uint16_t numCharacteristics; // outside scope of loop for end read check
    uint16_t numFilterdOutCharacteristics = 0;
    do
    {
        numCharacteristics = 1; // only fetch one at a time
        esp_gattc_char_elem_t characteristic;
        status = esp_ble_gattc_get_all_char(mGattcIf,
                                            aService.conn_id,
                                            aService.start_handle,
                                            aService.end_handle,
                                            &characteristic,
                                            &numCharacteristics, // this will update to total number of characteristics
                                            characteristics.size());
        if (status == ESP_GATT_OK)
        {
            bool isUUIDWanted = uuidFilter.empty() || std::find(uuidFilter.begin(), uuidFilter.end(), characteristic.uuid.uuid.uuid32) != uuidFilter.end();
            // Trust me on the bit mask :)
            bool isPropWanted = filtertype == Device::CharacteristicFilterType::Any ? characteristic.properties & propertiesFilter : ((characteristic.properties & propertiesFilter) == propertiesFilter);
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

std::vector<esp_gattc_descr_elem_t> Device::getDescriptors(const Device::ServiceSearchResult &aService, const esp_gattc_char_elem_t &aCharacteristic)
{

    esp_gatt_status_t status = ESP_GATT_OK;
    std::vector<esp_gattc_descr_elem_t> descriptors;
    uint16_t numDescriptions; // outside scope of loop for end read check
    do
    {
        numDescriptions = 1; // only fetch one at a time
        esp_gattc_descr_elem_t charaDescription;
        status = esp_ble_gattc_get_all_descr(mGattcIf,
                                             aService.conn_id,
                                             aCharacteristic.char_handle,
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

void Device::describeServices()
{
    for (auto service : mServicesFound)
    {
        describeService(service);
    }
}

void Device::describeService(const Device::ServiceSearchResult &aService)
{
    ESP_LOGI(LOG_TAG, "%s Service UUID: %s ", getName().c_str(), uuidToStr(aService.srvc_id.uuid).c_str());
    auto characteristics = getCharacteristics(aService);
    for (auto characteristic : characteristics)
    {
        describeCharacteristic(characteristic, aService);
    }
}

void Device::describeCharacteristic(const esp_gattc_char_elem_t &aCharacteristic, const Device::ServiceSearchResult &aService)
{
    ESP_LOGI(LOG_TAG, "---Handle: %d , UUID: %s", aCharacteristic.char_handle, uuidToStr(aCharacteristic.uuid).c_str());
    ESP_LOGI(LOG_TAG, "------ Properties- BroadCast: %d Read: %d Write_NR: %d Write: %d Notify: %d Indicate: %d Auth: %d Ext_Prop: %d ",
             (aCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_BROADCAST) > 0,
             (aCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_READ) > 0,
             (aCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR) > 0,
             (aCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_WRITE) > 0,
             (aCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) > 0,
             (aCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_INDICATE) > 0,
             (aCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_AUTH) > 0,
             (aCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_EXT_PROP) > 0);
}

void Device::registerForJoystickCharacteristics()
{
    auto service = std::find_if(mServicesFound.begin(), mServicesFound.end(), [](Device::ServiceSearchResult aService)
                                { return aService.srvc_id.uuid.uuid.uuid32 == ESP_GATT_UUID_HID_SVC; });

    uint8_t propFilter = ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_READ;
    std::vector<int> uuidFilter = {};
    auto characteristics = getCharacteristics(*service, propFilter, Device::CharacteristicFilterType::Any, uuidFilter);
    if (characteristics.empty())
    {
        ESP_LOGI(LOG_TAG, "empty");
    }
    for (auto characteristic : characteristics)
    {
        describeCharacteristic(characteristic, *service);
        // esp_ble_gattc_register_for_notify(mGattcIf, mRemoteAddress, characteristic.char_handle);
    }
}

void Device::handleCharacteristicRead(Device::CharacteristicReadResult aReadResult)
{

    if (aReadResult.value_len > 0)
    {
        uint8_t buff[aReadResult.value_len];
        memcpy(buff, aReadResult.value, sizeof(char) * aReadResult.value_len);
        ESP_LOG_BUFFER_HEX(LOG_TAG, buff, aReadResult.value_len);
    }
}

void Device::readCharacteristic(const Device::ServiceSearchResult aService, const esp_gattc_char_elem_t &aCharacteristic)
{
    esp_ble_gattc_read_char(mGattcIf, aService.conn_id, aCharacteristic.char_handle, ESP_GATT_AUTH_REQ_NONE);
}

void Device::logAllCharacteristicData()
{
    for (auto service : mServicesFound)
    {
        for (auto characteristic : getCharacteristics(service))
        {
            if (characteristic.properties & ESP_GATT_CHAR_PROP_BIT_READ)
            {
                readCharacteristic(service, characteristic);
            }
        }
    }
}