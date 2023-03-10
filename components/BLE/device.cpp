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

void Device::describeServices()
{
    for (auto service : mServicesFound)
    {
        service.describe();
    }
}

void Device::registerForJoystickCharacteristics()
{

    constexpr uint8_t propFilter = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
    std::vector reportFilter{ESP_GATT_UUID_HID_REPORT};

    for (auto service: mServicesFound){
        auto reports = service.getCharacteristics(propFilter,Characteristic::PropFilterType::Any);
        for (auto report : reports){
            ESP_LOGI(LOG_TAG,"Service: %s", service.uuidstr().c_str());
            report.describe();
            //esp_ble_gattc_register_for_notify(mGattcIf, mRemoteAddress, characteristic.char_handle());
        }
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
