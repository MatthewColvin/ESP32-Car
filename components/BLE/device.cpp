// BLE Component
#include "device.hpp"
// ESP API
#include "esp_gattc_api.h"
#include "esp_log.h"
// RTOS
// STD
#include <cstring>
#include <algorithm>

#define LOG_TAG "Device"

bool operator<(const esp_bt_uuid_t &aLeftUUID, const esp_bt_uuid_t &aRightUUID)
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

std::string uuidToStr(esp_bt_uuid_t aUUID)
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

Device::Device(bleScanResult res)
{
    mScanResult = res;
}

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

bool Device::isConnected()
{
    return checkEventSema(mConnectedEvent, 5000, "Could not Establish Connection!!");
}

bool Device::checkEventSema(SemaphoreHandle_t anEvent, int aTimeout, std::string anErrorMsg)
{
    bool eventDone = xSemaphoreTake(anEvent, aTimeout / portTICK_PERIOD_MS) == pdTRUE;
    if (eventDone)
    {
        xSemaphoreGive(anEvent);
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Event Timed out in %d ms ERROR: %s", aTimeout, anErrorMsg.c_str());
    }
    return eventDone;
}

void Device::openConnection(OpenEventInfo aOpenEvent)
{
    // Give the initial connection.
    xSemaphoreGive(mConnectedEvent);

    memcpy(mRemoteAddress, aOpenEvent.remote_bda, sizeof(esp_bd_addr_t));
    mConnectionId = aOpenEvent.conn_id;
}

void Device::searchServices()
{
    if (!isConnected())
    {
        ESP_LOGE(LOG_TAG, "Cannot Search for Services without connection!");
        return;
    }
    ESP_LOGI(LOG_TAG, "Searching %s for Services", getName().c_str());
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
    xSemaphoreGive(mServiceSearchCompleteEvent);
    authenticate();
}

bool Device::isServicesSearchComplete()
{
    return checkEventSema(mServiceSearchCompleteEvent, 5000, "Could not Finish Service Search!");
}

void Device::registerForCharacteristicNotify(Characteristic aCharacteristic, characteristicCallbackType aCallback)
{
    ESP_ERROR_CHECK(esp_ble_gattc_register_for_notify(mGattcIf, mRemoteAddress, aCharacteristic.char_handle()));
    mserviceCallbacks.emplace(aCharacteristic, std::move(aCallback));
}

void Device::unRegisterForCharacteristicNotify(Characteristic aCharacteristic)
{
    auto unRegCharaIt = mserviceCallbacks.find(aCharacteristic);
    if (unRegCharaIt == mserviceCallbacks.end())
    {
        ESP_LOGE(LOG_TAG, "Cannot Find Characteristic:%s Did you register it??", aCharacteristic.uuidstr().c_str());
    }
    else
    {
        ESP_ERROR_CHECK(esp_ble_gattc_unregister_for_notify(mGattcIf, mRemoteAddress, aCharacteristic.char_handle()));
        ESP_LOGI(LOG_TAG, "size %d", mserviceCallbacks.size());
        mserviceCallbacks.erase(unRegCharaIt);
    }
}

void Device::unRegisterAllCharacteristicNotifications()
{
    for (auto registration : mserviceCallbacks)
    {
        ESP_ERROR_CHECK(esp_ble_gattc_unregister_for_notify(mGattcIf, mRemoteAddress, registration.first.char_handle()));
    }
    mserviceCallbacks.clear();
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

void Device::handleAuthComplete(esp_ble_sec_t aSecurityInfo)
{
    if (aSecurityInfo.auth_cmpl.success)
    {
        ESP_LOGI(LOG_TAG, "Authentication for %s Success mode = %d", getName().c_str(), aSecurityInfo.auth_cmpl.auth_mode);
        xSemaphoreGive(mAuthenticatedEvent);
    }
    else
    {
        ESP_LOGI(LOG_TAG, "Authentication Fail: 0x%x", aSecurityInfo.auth_cmpl.fail_reason);
    }
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

bool Device::isAuthenticated()
{
    return checkEventSema(mAuthenticatedEvent, 5000, "Could not Authenticate Device!!");
}

void Device::authenticate()
{
    if (isServicesSearchComplete())
    {
        for (auto s : mServicesFound)
        {
            auto chars = s.getCharacteristics(ESP_GATT_CHAR_PROP_BIT_READ, Characteristic::PropFilterType::Any, {ESP_GATT_UUID_GAP_DEVICE_NAME});
            for (auto c : chars)
            {
                c.read();
            }
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
