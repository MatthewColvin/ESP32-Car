#include "BTClassicHID.hpp"
#include "HIDDevice.hpp"

// RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <algorithm>

std::shared_ptr<BTClassicHID> BTClassicHID::mInstance = nullptr;
std::array<std::shared_ptr<HIDDevice>, MAX_CONNECTED_DEVICES> BTClassicHID::mConnectedDevices{};

std::shared_ptr<HIDDevice> BTClassicHID::getDevice(esp_hidh_event_t anEvent, esp_hidh_event_data_t *aParams)
{
    const uint8_t *anAddress{};
    switch (anEvent)
    {
    case ESP_HIDH_OPEN_EVENT: /*!< HID device opened */
        anAddress = esp_hidh_dev_bda_get(aParams->open.dev);
        break;
    case ESP_HIDH_BATTERY_EVENT: /*!< HID device battery level changed */
        anAddress = esp_hidh_dev_bda_get(aParams->input.dev);
        break;
    case ESP_HIDH_INPUT_EVENT: /*!< Received HID device INPUT report */
        anAddress = esp_hidh_dev_bda_get(aParams->input.dev);
        break;
    case ESP_HIDH_FEATURE_EVENT: /*!< Received HID device FEATURE report */
        anAddress = esp_hidh_dev_bda_get(aParams->feature.dev);
        break;
    case ESP_HIDH_CLOSE_EVENT: /*!< HID device closed */
        anAddress = esp_hidh_dev_bda_get(aParams->close.dev);
        break;
    case ESP_HIDH_ANY_EVENT:   /*!< HID device any event */
    case ESP_HIDH_START_EVENT: /*!< HID host stack started, used only for Classic Bluetooth */
    case ESP_HIDH_STOP_EVENT:  /*!< HID host stack stopped, used only for Classic Bluetooth */
    default:
        return nullptr;
    }
    auto currentDevice = std::find_if(mConnectedDevices.begin(), mConnectedDevices.end(), [anAddress](std::shared_ptr<HIDDevice> aDevice)
                                      { return aDevice && aDevice->hasAddress(anAddress); });
    if (currentDevice != mConnectedDevices.end())
    {
        return *currentDevice;
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Could NOT Find Device!");
    }
    return nullptr;
}

void BTClassicHID::hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    auto device = getDevice(event, param);

    // Event expects a device to run it.
    if (!device && event <= ESP_HIDH_CLOSE_EVENT)
    {
        ESP_LOGE(LOG_TAG, "Cannot Preform HID Callback without Device");
        return;
    }

    switch (event)
    {
    case ESP_HIDH_OPEN_EVENT:
        device->handleOpenEvent(param);
        break;
    case ESP_HIDH_BATTERY_EVENT:
        device->handleBatteryEvent(param);
        break;
    case ESP_HIDH_INPUT_EVENT:
        device->handleInputEvent(param);
        break;
    case ESP_HIDH_FEATURE_EVENT:
        device->handleFeatureEvent(param);
        break;
    case ESP_HIDH_CLOSE_EVENT:
        device->handleCloseEvent(param);
        // TODO to handle removal from mconnecteddevices
        break;
    case ESP_HIDH_START_EVENT:
    case ESP_HIDH_STOP_EVENT:
        // only used for BT classic do nothing for now.
        break;
    default:
        ESP_LOGE(LOG_TAG, "HIDH EVENT: %d NOT HANDLED", event);
        break;
    }
}

std::shared_ptr<BTClassicHID> BTClassicHID::getInstance()
{
    if (mInstance == nullptr)
    {
        // not thread save
        mInstance = std::shared_ptr<BTClassicHID>(new BTClassicHID());
    }
    return mInstance;
}

BTClassicHID::BTClassicHID()
{
    ESP_ERROR_CHECK(esp_hid_gap_init(HID_HOST_MODE));
    esp_hidh_config_t config = {
        .callback = BTClassicHID::hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));
#endif /* CONFIG_BT_BLE_ENABLED */
    vTaskDelay(2000 / portTICK_PERIOD_MS);
}

std::vector<HIDDevice> BTClassicHID::scan(uint32_t seconds)
{
    size_t numResults = 0;
    esp_hid_scan_result_t *results = nullptr;
    esp_hid_scan(seconds, &numResults, &results);

    std::vector<HIDDevice> devices;
    for (int i = 0; i < numResults; i++)
    {
        devices.push_back(HIDDevice(results[i]));
    }
    esp_hid_scan_results_free(results);

    return devices;
}

bool BTClassicHID::connect(std::shared_ptr<HIDDevice> aDevice)
{
    for (auto &device : mConnectedDevices)
    {
        // Dont have a device here yet add one
        if (!device)
        {
            auto hidHandle = esp_hidh_dev_open(aDevice->getAddress(), aDevice->getTransport(), aDevice->getAddressType());
            if (hidHandle)
            {
                device = aDevice;
                return true;
            }
            else
            {
                ESP_LOGE(LOG_TAG, "Failed to open HID!");
                return false;
            }
        }
    }
    ESP_LOGE(LOG_TAG, "Max Devices connected sorry!");
    return false;
}
