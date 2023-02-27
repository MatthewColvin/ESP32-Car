#include "ble.hpp"
#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_bt_device.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_system.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "string.h"
#include <vector>

#define LOG_TAG "BLE"
#define MATTS_TAG "MATT PRINTS"
#define PROFILE_APP_ID 0
#define BT_BD_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
#define BT_BD_ADDR_HEX(addr) addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
#define ESP_GATT_SPP_SERVICE_UUID 0xABF0
#define SCAN_ALL_THE_TIME 0

bool Ble::is_connect = false;
esp_ble_gap_cb_param_t Ble::scan_rst;
esp_ble_scan_params_t Ble::ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE};

const char *Ble::device_name = "VR-PARK";
Ble *Ble::mInstance = nullptr;

std::vector<esp_ble_gap_cb_param_t::ble_scan_result_evt_param> devices;

Ble::Ble()
{
    // Release Bluetooth Classic we will not need it.
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    esp_err_t ret;
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(LOG_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(LOG_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(LOG_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(LOG_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(LOG_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
}

Ble *Ble::getInstance()
{
    if (mInstance == nullptr)
    {
        mInstance = new Ble();
    }
    return mInstance;
}

std::vector<esp_ble_gap_cb_param_t::ble_scan_result_evt_param> Ble::scan(uint32_t secondsToScan)
{
    esp_ble_gap_start_scanning(secondsToScan);
    vTaskDelay(secondsToScan * 1000 / portTICK_PERIOD_MS);
    return devices;
}

void Ble::esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    esp_err_t err;

    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    {
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
    {
        // scan start complete event to indicate scan start successfully or failed
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(LOG_TAG, "Scan start failed: %s", esp_err_to_name(err));
            break;
        }
        ESP_LOGI(LOG_TAG, "Scan start successed");
        break;
    }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
    {
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(LOG_TAG, "Scan stop failed: %s", esp_err_to_name(err));
            break;
        }
        ESP_LOGI(LOG_TAG, "Scan stop successed");
        if (is_connect == false)
        {
            ESP_LOGI(LOG_TAG, "Connect to the remote device.");
            // esp_ble_gattc_open(our profile.gattc_if, scan_rst.scan_rst.bda, scan_rst.scan_rst.ble_addr_type, true);
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;

        switch (scan_result->scan_rst.search_evt)
        {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            devices.push_back(scan_result->scan_rst);
            // esp_log_buffer_hex(LOG_TAG, scan_result->scan_rst.bda, 6);
            // ESP_LOGI(LOG_TAG, "Searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            esp_log_buffer_char(MATTS_TAG, adv_name, adv_name_len);
            if (adv_name != NULL)
            {
                if (strncmp((char *)adv_name, device_name, adv_name_len) == 0)
                {
                    memcpy(&(scan_rst), scan_result, sizeof(esp_ble_gap_cb_param_t));
                    esp_ble_gap_stop_scanning();
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
    {
        if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(LOG_TAG, "Adv stop failed: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(LOG_TAG, "Stop adv successfully");
        }
        break;
    }
    default:
        break;
    }
}

void Ble::esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    ESP_LOGI(LOG_TAG, "EVT %d, gattc if %d", event, gattc_if);

    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            // ourprofile??[param->reg.app_id].gattc_if = gattc_if;
        }
        else
        {
            ESP_LOGI(LOG_TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }
    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    // do
    // {
    //     int idx;
    //     for (idx = 0; idx < PROFILE_NUM; idx++)
    //     {
    //         if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
    //             gattc_if == gl_profile_tab[idx].gattc_if)
    //         {
    //             if (gl_profile_tab[idx].gattc_cb)
    //             {
    //                 gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
    //             }
    //         }
    //     }
    // } while (0);
}

void Ble::ble_client_appRegister(void)
{
    esp_err_t status;
    char err_msg[20];

    ESP_LOGI(LOG_TAG, "register callback");

    // register the scan callback function to the gap module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "gap register error: %s", esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
        return;
    }
    // register the callback function to the gattc module
    if ((status = esp_ble_gattc_register_callback(esp_gattc_cb)) != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "gattc register error: %s", esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
        return;
    }
    esp_ble_gattc_app_register(PROFILE_APP_ID);

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(200);
    if (local_mtu_ret)
    {
        ESP_LOGE(LOG_TAG, "set local  MTU failed: %s", esp_err_to_name_r(local_mtu_ret, err_msg, sizeof(err_msg)));
    }
}