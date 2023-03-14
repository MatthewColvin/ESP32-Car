// Library
#include "device.hpp"

// ESP API
#include "esp_gattc_api.h"
#include "esp_log.h"

// RTOS
#include "freertos/FreeRTOS.h"

// STD
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

void Device::addFoundService(Service::espIdfTy aService)
{
    mServicesFound.push_back(Service(mGattcIf, aService));
}
void Device::serviceSearchComplete()
{
    mIsServiceSearching = true;
}
bool Device::isServicesSearchComplete() { return mIsServiceSearching; }

void Device::registerforCharacteristicNotify(characterHandleType aCharacteristicHndl, characteristicCallbackType aCallback)
{
    mserviceCallbacks.emplace(aCharacteristicHndl, std::move(aCallback));
}
Device::serviceCbRetType Device::handleCharacteristicNotify(characteristicCbParamType aParam)
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

void Device::enableNotifitcation(Characteristic aCharacteristic)
{
    // Add check to ensure that characteristic existis in the map of characteristics and throw error if not?
    auto descriptors = aCharacteristic.getDescriptors();
    auto clientConfig = std::find_if(descriptors.begin(), descriptors.end(), [](esp_gattc_descr_elem_t desc)
                                     { return desc.uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG; });

    if (clientConfig == descriptors.end())
    {
        ESP_LOGE(LOG_TAG, "%s with Char: %s Missing descriptor cannot enable notification.", getName().c_str(), aCharacteristic.uuidstr().c_str());
    }
    else
    {
        uint16_t notify_en = 1;
        ESP_LOGI(LOG_TAG, "READING: Client config descriptor for Characteristic: %s", aCharacteristic.uuidstr().c_str());
        esp_ble_gattc_read_char_descr(mGattcIf, mConnectionId, aCharacteristic.char_handle(), ESP_GATT_AUTH_REQ_NONE);

        auto status = esp_ble_gattc_write_char_descr(mGattcIf, mConnectionId, aCharacteristic.char_handle(), sizeof(notify_en),
                                                     (uint8_t *)&notify_en, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        if (status != ESP_OK)
        {
            ESP_LOGE(LOG_TAG, "%s with Char: %s Failed to write descriptor to enable notification.", getName().c_str(), aCharacteristic.uuidstr().c_str());
        }
    }
}
// void Device::disableNotifictaion(Characteristic aCharacteristic);

void Device::describeServices()
{
    for (auto service : mServicesFound)
    {
        service.describe();
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

void Device::readAllCharacteristics()
{
    uint8_t charaFilters = ESP_GATT_CHAR_PROP_BIT_READ;
    std::vector<int> uuidFilter{ESP_GATT_UUID_HID_REPORT};
    for (auto service : mServicesFound)
    {
        auto charas = service.getCharacteristics(charaFilters, Characteristic::PropFilterType::Any,uuidFilter);
        for (auto characteristic : charas)
        {
            characteristic.read();
        }
    }
}
void Device::protocolMode()
{
    uint8_t bootmode {0};

    uint8_t charaFilters = ESP_GATT_CHAR_PROP_BIT_READ;
    std::vector<int> uuidFilter{ESP_GATT_UUID_HID_PROTO_MODE};
    for (auto service : mServicesFound)
    {
        auto charas = service.getCharacteristics(charaFilters, Characteristic::PropFilterType::Any, uuidFilter);
        for (auto characteristic : charas)
        {
            characteristic.write(&bootmode,1);
        }
    }
}
void Device::exitSuspend()
{
    uint8_t exitSuspend {1};

    uint8_t charaFilters = ESP_GATT_CHAR_PROP_BIT_READ;
    std::vector<int> uuidFilter{ESP_GATT_UUID_HID_CONTROL_POINT};
    for (auto service : mServicesFound)
    {
        auto charas = service.getCharacteristics(charaFilters, Characteristic::PropFilterType::Any, uuidFilter);
        for (auto characteristic : charas)
        {
            characteristic.write(&exitSuspend,1);
        }
    }
}