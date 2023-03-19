// BLE Component
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

bool operator<(const Device::UUIDType &aLeftUUID, const Device::UUIDType &aRightUUID)
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

std::string uuidToStr(Device::UUIDType aUUID)
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

void Device::registerforCharacteristicNotify(Characteristic aCharacteristic, characteristicCallbackType aCallback)
{
    ESP_ERROR_CHECK(esp_ble_gattc_register_for_notify(mGattcIf, mRemoteAddress, aCharacteristic.char_handle()));
    mserviceCallbacks.emplace(aCharacteristic, std::move(aCallback));
}
void Device::unRegisterForCharacterisitcNotify(Characteristic aCharacteristic)
{

    auto unRegCharaIt = mserviceCallbacks.find(aCharacteristic);
    if (unRegCharaIt == mserviceCallbacks.end())
    {
        ESP_LOGE(LOG_TAG, "Cannot Find Characteristic:%s Did you register it??", aCharacteristic.uuidstr().c_str());
    }
    else
    {
        ESP_ERROR_CHECK(esp_ble_gattc_unregister_for_notify(mGattcIf, mRemoteAddress, aCharacteristic.char_handle()));
    }
}

Device::serviceCbRetType Device::handleCharacteristicNotify(characteristicCbParamType aParam)
{
    auto chara = getCharacteristic(aParam.handle);
    ESP_LOGI(LOG_TAG, "Handeling Characteristic: %s for %s", chara.uuidstr().c_str(), getName().c_str());
    if (auto callbackPair = mserviceCallbacks.find(chara); callbackPair != mserviceCallbacks.end())
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

void Device::handleNotifyRegistration(NotifyRegistrationType aRegistration)
{
    auto characteristicToNotify = getCharacteristic(aRegistration.handle);

    auto descriptors = characteristicToNotify.getDescriptors();
    auto clientConfig = std::find_if(descriptors.begin(), descriptors.end(), [](esp_gattc_descr_elem_t desc)
                                     { return desc.uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG; });

    if (clientConfig == descriptors.end())
    {
        ESP_LOGE(LOG_TAG, "%s with Char: %s Missing descriptor cannot enable notification.", getName().c_str(), characteristicToNotify.uuidstr().c_str());
    }
    else
    {
        uint16_t notify_en = 1;

        auto status = esp_ble_gattc_write_char_descr(mGattcIf, mConnectionId, characteristicToNotify.char_handle(), sizeof(notify_en),
                                                     (uint8_t *)&notify_en, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NO_MITM);
        if (status != ESP_OK)
        {
            ESP_LOGE(LOG_TAG, "%s with Char: %s Failed to write descriptor to enable notification.", getName().c_str(), characteristicToNotify.uuidstr().c_str());
        }
    }
}

void Device::handleNotifyUnregistration(NotifyUnregistrationType anUnregistration)
{
    auto characteristicToNotify = getCharacteristic(anUnregistration.handle);

    auto descriptors = characteristicToNotify.getDescriptors();
    auto clientConfig = std::find_if(descriptors.begin(), descriptors.end(), [](esp_gattc_descr_elem_t desc)
                                     { return desc.uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG; });

    if (clientConfig == descriptors.end())
    {
        ESP_LOGE(LOG_TAG, "%s with Char: %s Missing descriptor cannot disable notification.", getName().c_str(), characteristicToNotify.uuidstr().c_str());
    }
    uint16_t notify_en = 0;

    auto status = esp_ble_gattc_write_char_descr(mGattcIf, mConnectionId, characteristicToNotify.char_handle(), sizeof(notify_en),
                                                 (uint8_t *)&notify_en, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NO_MITM);
    if (status != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "%s with Char: %s Failed to write descriptor to disable notification.", getName().c_str(), characteristicToNotify.uuidstr().c_str());
    }
}

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
        auto charas = service.getCharacteristics(charaFilters, Characteristic::PropFilterType::Any, uuidFilter);
        for (auto characteristic : charas)
        {
            characteristic.read();
        }
    }
}

Characteristic Device::getCharacteristic(uint16_t aSearchHandle)
{
    Characteristic characteristic;
    bool found = false;
    // Add check to ensure that characteristic existis in the map of characteristics and throw error if not?
    for (auto service : mServicesFound)
    {
        auto charas = service.getCharacteristics();
        auto charaToNotify = std::find_if(charas.begin(), charas.end(), [aSearchHandle](Characteristic aChara)
                                          { return aChara.char_handle() == aSearchHandle; });
        if (charaToNotify != charas.end())
        {
            characteristic = *charaToNotify;
            found = true;
            break;
        }
    }
    if (!found)
    {
        ESP_LOGE(LOG_TAG, "Unable To Find Characteristic with handle:%d", aSearchHandle);
    }

    return characteristic;
}
