#include "BTClassicHID.hpp"
#include "HIDDevice.hpp"

#define LOG_TAG "BTClassicHID"

std::shared_ptr<BTClassicHID> BTClassicHID::mInstance = nullptr;

void BTClassicHID::hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

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
    ESP_ERROR_CHECK(esp_hid_gap_init(ESP_BT_MODE_CLASSIC_BT));
    esp_hidh_config_t config = {
        .callback = BTClassicHID::hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(esp_hidh_init(&config));
}

std::vector<HIDDevice> BTClassicHID::scan(int seconds)
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
