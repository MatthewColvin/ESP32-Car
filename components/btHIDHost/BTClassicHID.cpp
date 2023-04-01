#include "BTClassicHID.hpp"
#include "HIDDevice.hpp"

#include <algorithm>

#define LOG_TAG "BTClassicHID"

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
                                      { 
                                        if (aDevice){
                                            ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(aDevice->getAddress()));
                                            ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(anAddress));
                                        }
                                        return aDevice && aDevice->hasAddress(anAddress); });
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

    switch (event)
    {
    case ESP_HIDH_OPEN_EVENT:
    {
        if (param->open.status == ESP_OK)
        {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
            esp_hidh_dev_dump(param->open.dev, stdout);
        }
        else
        {
            ESP_LOGE(LOG_TAG, " OPEN failed!");
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT:
    {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        break;
    }
    case ESP_HIDH_INPUT_EVENT:
    {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
        ESP_LOG_BUFFER_HEX(LOG_TAG, param->input.data, param->input.length);
        break;
    }
    case ESP_HIDH_FEATURE_EVENT:
    {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                 esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                 param->feature.length);
        ESP_LOG_BUFFER_HEX(LOG_TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDH_CLOSE_EVENT:
    {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        break;
    }
    default:
        ESP_LOGI(LOG_TAG, "EVENT: %d", event);
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
    xTaskCreate(&BTClassicHID::init, "hid_task", 6 * 1024, NULL, 2, NULL);
}

void BTClassicHID::init(void *params)
{
    ESP_ERROR_CHECK(esp_hid_gap_init(HID_HOST_MODE));
    esp_hidh_config_t config = {
        .callback = BTClassicHID::hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(esp_hidh_init(&config));
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
