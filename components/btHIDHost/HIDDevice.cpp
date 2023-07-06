#include "HIDDevice.hpp"

#include <cstring>

#define LOG_TAG "HIDDevice"

HIDDevice::HIDDevice(esp_hid_scan_result_t aScanResult) : mScanResult(aScanResult)
{
}
HIDDevice::~HIDDevice()
{
    // TODO Could Print address of device.
    // ESP_LOGI(LOG_TAG, "Successfully Destroyed Device");
}

bool HIDDevice::hasAddress(const esp_bd_addr_t anAddress)
{
    return isAddressEqualIgnoreZeros(anAddress, mScanResult.bda);
}

bool HIDDevice::hasExactAddress(const esp_bd_addr_t anAddress)
{
   return isAddressEqual(anAddress,mScanResult.bda);
}

esp_ble_addr_type_t HIDDevice::getAddressType()
{
    if (mScanResult.transport == ESP_HID_TRANSPORT_BLE)
    {
        return mScanResult.ble.addr_type;
    }
    // returning first enum since this should not matter anyway
    // because the transport is BT classic
    return BLE_ADDR_TYPE_PUBLIC;
};

void HIDDevice::handleOpenEvent(esp_hidh_event_data_t *aOpenEvent)
{
    const uint8_t *bda = esp_hidh_dev_bda_get(aOpenEvent->open.dev);
    ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(aOpenEvent->open.dev));
    esp_hidh_dev_dump(aOpenEvent->open.dev, stdout);
}
void HIDDevice::handleBatteryEvent(esp_hidh_event_data_t *aBatteryEvent)
{
    const uint8_t *bda = esp_hidh_dev_bda_get(aBatteryEvent->battery.dev);
    ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), aBatteryEvent->battery.level);
}
void HIDDevice::handleInputEvent(esp_hidh_event_data_t *anInputEvent)
{
    const uint8_t *bda = esp_hidh_dev_bda_get(anInputEvent->input.dev);
    ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(anInputEvent->input.usage), anInputEvent->input.map_index, anInputEvent->input.report_id, anInputEvent->input.length);
    ESP_LOG_BUFFER_HEX(LOG_TAG, anInputEvent->input.data, anInputEvent->input.length);
}
void HIDDevice::handleFeatureEvent(esp_hidh_event_data_t *aFeatureEvent)
{
    const uint8_t *bda = esp_hidh_dev_bda_get(aFeatureEvent->feature.dev);
    ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
             esp_hid_usage_str(aFeatureEvent->feature.usage), aFeatureEvent->feature.map_index, aFeatureEvent->feature.report_id,
             aFeatureEvent->feature.length);
    ESP_LOG_BUFFER_HEX(LOG_TAG, aFeatureEvent->feature.data, aFeatureEvent->feature.length);
}
void HIDDevice::handleCloseEvent(esp_hidh_event_data_t *aCloseEvent)
{
    const uint8_t *bda = esp_hidh_dev_bda_get(aCloseEvent->close.dev);
    ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(aCloseEvent->close.dev));
    if (mHandleDisconnect)
    {
        mHandleDisconnect();
    }
}
