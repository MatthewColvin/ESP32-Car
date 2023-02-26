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

#define LOG_TAG "BLE"
#define MATTS_TAG "MATT PRINTS"
#define PROFILE_NUM 1
#define PROFILE_APP_ID 0
#define BT_BD_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
#define BT_BD_ADDR_HEX(addr) addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
#define ESP_GATT_SPP_SERVICE_UUID 0xABF0
#define SCAN_ALL_THE_TIME 0

static esp_bt_uuid_t spp_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = ESP_GATT_SPP_SERVICE_UUID,
    },
};
static uint16_t count = SPP_IDX_NB;

bool Ble::is_connect = false;
esp_ble_gap_cb_param_t Ble::scan_rst;
esp_ble_scan_params_t Ble::ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE};

int Ble::notify_value_offset = 0;
int Ble::notify_value_count = 0;
esp_gattc_db_elem_t *Ble::db = NULL;
QueueHandle_t Ble::cmd_reg_queue = NULL;
uint16_t Ble::spp_conn_id = 0;
uint16_t Ble::spp_mtu_size = 23;
uint16_t Ble::cmd = 0;
uint16_t Ble::spp_srv_start_handle = 0;
uint16_t Ble::spp_srv_end_handle = 0;
uint16_t Ble::spp_gattc_if = 0xff;
char *Ble::notify_value_p = NULL;
const char *Ble::device_name = "VR-PARK";
Ble *Ble::mInstance = nullptr;
/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
struct gattc_profile_inst Ble::gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = {
        .gattc_cb = Ble::gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    }};

Ble *Ble::getInstance()
{
    if (mInstance == nullptr)
    {
        mInstance = new Ble();
    }
    return mInstance;
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
        if ((err = param->scan_param_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(LOG_TAG, "Scan param set failed: %s", esp_err_to_name(err));
            break;
        }
        // the unit of the duration is second
        uint32_t duration = 0xFFFF;
        ESP_LOGI(LOG_TAG, "Enable Ble Scan:during time %04" PRIx32 " minutes.", duration);
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        // scan start complete event to indicate scan start successfully or failed
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(LOG_TAG, "Scan start failed: %s", esp_err_to_name(err));
            break;
        }
        ESP_LOGI(LOG_TAG, "Scan start successed");
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(LOG_TAG, "Scan stop failed: %s", esp_err_to_name(err));
            break;
        }
        ESP_LOGI(LOG_TAG, "Scan stop successed");
        if (is_connect == false)
        {
            ESP_LOGI(LOG_TAG, "Connect to the remote device.");
            esp_ble_gattc_open(gl_profile_tab[PROFILE_APP_ID].gattc_if, scan_rst.scan_rst.bda, scan_rst.scan_rst.ble_addr_type, true);
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt)
        {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            // esp_log_buffer_hex(LOG_TAG, scan_result->scan_rst.bda, 6);
            //  ESP_LOGI(LOG_TAG, "Searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOGI(LOG_TAG, "Searched Device Name Len %d", adv_name_len);
            // esp_log_buffer_char(MATTS_TAG, adv_name, adv_name_len);
            // ESP_LOGI(LOG_TAG, "\n");
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
        if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(LOG_TAG, "Adv stop failed: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(LOG_TAG, "Stop adv successfully");
        }
        break;
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
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        }
        else
        {
            ESP_LOGI(LOG_TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }
    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gattc_if == gl_profile_tab[idx].gattc_if)
            {
                if (gl_profile_tab[idx].gattc_cb)
                {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

void Ble::gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(LOG_TAG, "REG EVT, set scan params");
        esp_ble_gap_set_scan_params(&ble_scan_params);
        break;
    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(LOG_TAG, "ESP_GATTC_CONNECT_EVT: conn_id=%d, gatt_if = %d", spp_conn_id, gattc_if);
        ESP_LOGI(LOG_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(LOG_TAG, gl_profile_tab[PROFILE_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        spp_gattc_if = gattc_if;
        is_connect = true;
        spp_conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gattc_search_service(spp_gattc_if, spp_conn_id, &spp_service_uuid);
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(LOG_TAG, "disconnect");
        Ble::free_gattc_srv_db();
        esp_ble_gap_start_scanning(SCAN_ALL_THE_TIME);
        break;
    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(LOG_TAG, "ESP_GATTC_SEARCH_RES_EVT: start_handle = %d, end_handle = %d, UUID:0x%04x", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.uuid.uuid.uuid16);
        spp_srv_start_handle = p_data->search_res.start_handle;
        spp_srv_end_handle = p_data->search_res.end_handle;
        break;
    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(LOG_TAG, "SEARCH_CMPL: conn_id = %x, status %d", spp_conn_id, p_data->search_cmpl.status);
        esp_ble_gattc_send_mtu_req(gattc_if, spp_conn_id);
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
    {
        ESP_LOGI(LOG_TAG, "Index = %d,status = %d,handle = %d\n", cmd, p_data->reg_for_notify.status, p_data->reg_for_notify.handle);
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
        {
            ESP_LOGE(LOG_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT, status = %d", p_data->reg_for_notify.status);
            break;
        }
        uint16_t notify_en = 1;
        esp_ble_gattc_write_char_descr(
            spp_gattc_if,
            spp_conn_id,
            (db + cmd + 1)->attribute_handle,
            sizeof(notify_en),
            (uint8_t *)&notify_en,
            ESP_GATT_WRITE_TYPE_RSP,
            ESP_GATT_AUTH_REQ_NONE);

        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(LOG_TAG, "ESP_GATTC_NOTIFY_EVT\n");
        notify_event_handler(p_data);
        break;
    case ESP_GATTC_READ_CHAR_EVT:
        ESP_LOGI(LOG_TAG, "ESP_GATTC_READ_CHAR_EVT\n");
        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        ESP_LOGI(LOG_TAG, "ESP_GATTC_WRITE_CHAR_EVT:status = %d,handle = %d", param->write.status, param->write.handle);
        if (param->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(LOG_TAG, "ESP_GATTC_WRITE_CHAR_EVT, error status = %d", p_data->write.status);
            break;
        }
        break;
    case ESP_GATTC_PREP_WRITE_EVT:
        break;
    case ESP_GATTC_EXEC_EVT:
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        ESP_LOGI(LOG_TAG, "ESP_GATTC_WRITE_DESCR_EVT: status =%d,handle = %d \n", p_data->write.status, p_data->write.handle);
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(LOG_TAG, "ESP_GATTC_WRITE_DESCR_EVT, error status = %d", p_data->write.status);
            break;
        }
        switch (cmd)
        {
        case SPP_IDX_SPP_DATA_NTY_VAL:
            cmd = SPP_IDX_SPP_STATUS_VAL;
            xQueueSend(cmd_reg_queue, &cmd, 10 / portTICK_PERIOD_MS);
            break;
        case SPP_IDX_SPP_STATUS_VAL:
            break;
        default:
            break;
        };
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (p_data->cfg_mtu.status != ESP_OK)
        {
            break;
        }
        ESP_LOGI(LOG_TAG, "+MTU:%d\n", p_data->cfg_mtu.mtu);
        spp_mtu_size = p_data->cfg_mtu.mtu;

        db = (esp_gattc_db_elem_t *)malloc(count * sizeof(esp_gattc_db_elem_t));
        if (db == NULL)
        {
            ESP_LOGE(LOG_TAG, "%s:malloc db falied\n", __func__);
            break;
        }
        if (esp_ble_gattc_get_db(spp_gattc_if, spp_conn_id, spp_srv_start_handle, spp_srv_end_handle, db, &count) != ESP_GATT_OK)
        {
            ESP_LOGE(LOG_TAG, "%s:get db falied\n", __func__);
            break;
        }
        if (count != SPP_IDX_NB)
        {
            ESP_LOGE(LOG_TAG, "%s:get db count != SPP_IDX_NB, count = %d, SPP_IDX_NB = %d\n", __func__, count, SPP_IDX_NB);
            break;
        }
        for (int i = 0; i < SPP_IDX_NB; i++)
        {
            switch ((db + i)->type)
            {
            case ESP_GATT_DB_PRIMARY_SERVICE:
                ESP_LOGI(LOG_TAG, "attr_type = PRIMARY_SERVICE,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",
                         (db + i)->attribute_handle, (db + i)->start_handle, (db + i)->end_handle, (db + i)->properties, (db + i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_SECONDARY_SERVICE:
                ESP_LOGI(LOG_TAG, "attr_type = SECONDARY_SERVICE,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",
                         (db + i)->attribute_handle, (db + i)->start_handle, (db + i)->end_handle, (db + i)->properties, (db + i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_CHARACTERISTIC:
                ESP_LOGI(LOG_TAG, "attr_type = CHARACTERISTIC,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",
                         (db + i)->attribute_handle, (db + i)->start_handle, (db + i)->end_handle, (db + i)->properties, (db + i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_DESCRIPTOR:
                ESP_LOGI(LOG_TAG, "attr_type = DESCRIPTOR,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",
                         (db + i)->attribute_handle, (db + i)->start_handle, (db + i)->end_handle, (db + i)->properties, (db + i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_INCLUDED_SERVICE:
                ESP_LOGI(LOG_TAG, "attr_type = INCLUDED_SERVICE,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",
                         (db + i)->attribute_handle, (db + i)->start_handle, (db + i)->end_handle, (db + i)->properties, (db + i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_ALL:
                ESP_LOGI(LOG_TAG, "attr_type = ESP_GATT_DB_ALL,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",
                         (db + i)->attribute_handle, (db + i)->start_handle, (db + i)->end_handle, (db + i)->properties, (db + i)->uuid.uuid.uuid16);
                break;
            default:
                break;
            }
        }
        cmd = SPP_IDX_SPP_DATA_NTY_VAL;
        xQueueSend(cmd_reg_queue, &cmd, 10 / portTICK_PERIOD_MS);
        break;
    case ESP_GATTC_SRVC_CHG_EVT:
        break;
    default:
        break;
    }
}

void Ble::spp_client_reg_task(void *arg)
{
    uint16_t cmd_id;
    for (;;)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (xQueueReceive(cmd_reg_queue, &cmd_id, portMAX_DELAY))
        {
            if (db != NULL)
            {
                if (cmd_id == SPP_IDX_SPP_DATA_NTY_VAL)
                {
                    ESP_LOGI(LOG_TAG, "Index = %d,UUID = 0x%04x, handle = %d \n", cmd_id, (db + SPP_IDX_SPP_DATA_NTY_VAL)->uuid.uuid.uuid16, (db + SPP_IDX_SPP_DATA_NTY_VAL)->attribute_handle);
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda, (db + SPP_IDX_SPP_DATA_NTY_VAL)->attribute_handle);
                }
                else if (cmd_id == SPP_IDX_SPP_STATUS_VAL)
                {
                    ESP_LOGI(LOG_TAG, "Index = %d,UUID = 0x%04x, handle = %d \n", cmd_id, (db + SPP_IDX_SPP_STATUS_VAL)->uuid.uuid.uuid16, (db + SPP_IDX_SPP_STATUS_VAL)->attribute_handle);
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda, (db + SPP_IDX_SPP_STATUS_VAL)->attribute_handle);
                }
            }
        }
    }
}

void Ble::notify_event_handler(esp_ble_gattc_cb_param_t *p_data)
{
    uint8_t handle = 0;

    if (p_data->notify.is_notify == true)
    {
        ESP_LOGI(LOG_TAG, "+NOTIFY:handle = %d,length = %d ", p_data->notify.handle, p_data->notify.value_len);
    }
    else
    {
        ESP_LOGI(LOG_TAG, "+INDICATE:handle = %d,length = %d ", p_data->notify.handle, p_data->notify.value_len);
    }
    handle = p_data->notify.handle;
    if (db == NULL)
    {
        ESP_LOGE(LOG_TAG, " %s db is NULL\n", __func__);
        return;
    }
    if (handle == db[SPP_IDX_SPP_DATA_NTY_VAL].attribute_handle)
    {
#ifdef SPP_DEBUG_MODE
        esp_log_buffer_char(LOG_TAG, (char *)p_data->notify.value, p_data->notify.value_len);
#else
        if ((p_data->notify.value[0] == '#') && (p_data->notify.value[1] == '#'))
        {
            if ((++notify_value_count) != p_data->notify.value[3])
            {
                if (notify_value_p != NULL)
                {
                    free(notify_value_p);
                }
                notify_value_count = 0;
                notify_value_p = NULL;
                notify_value_offset = 0;
                ESP_LOGE(LOG_TAG, "notify value count is not continuous,%s\n", __func__);
                return;
            }
            if (p_data->notify.value[3] == 1)
            {
                notify_value_p = (char *)malloc(((spp_mtu_size - 7) * (p_data->notify.value[2])) * sizeof(char));
                if (notify_value_p == NULL)
                {
                    ESP_LOGE(LOG_TAG, "malloc failed,%s L#%d\n", __func__, __LINE__);
                    notify_value_count = 0;
                    return;
                }
                memcpy((notify_value_p + notify_value_offset), (p_data->notify.value + 4), (p_data->notify.value_len - 4));
                if (p_data->notify.value[2] == p_data->notify.value[3])
                {
                    uart_write_bytes(UART_NUM_0, (char *)(notify_value_p), (p_data->notify.value_len - 4 + notify_value_offset));
                    free(notify_value_p);
                    notify_value_p = NULL;
                    notify_value_offset = 0;
                    return;
                }
                notify_value_offset += (p_data->notify.value_len - 4);
            }
            else if (p_data->notify.value[3] <= p_data->notify.value[2])
            {
                memcpy((notify_value_p + notify_value_offset), (p_data->notify.value + 4), (p_data->notify.value_len - 4));
                if (p_data->notify.value[3] == p_data->notify.value[2])
                {
                    uart_write_bytes(UART_NUM_0, (char *)(notify_value_p), (p_data->notify.value_len - 4 + notify_value_offset));
                    free(notify_value_p);
                    notify_value_count = 0;
                    notify_value_p = NULL;
                    notify_value_offset = 0;
                    return;
                }
                notify_value_offset += (p_data->notify.value_len - 4);
            }
        }
        else
        {
            uart_write_bytes(UART_NUM_0, (char *)(p_data->notify.value), p_data->notify.value_len);
        }
#endif
    }
    else if (handle == ((db + SPP_IDX_SPP_STATUS_VAL)->attribute_handle))
    {
        esp_log_buffer_char(LOG_TAG, (char *)p_data->notify.value, p_data->notify.value_len);
        // TODO:server notify status characteristic
    }
    else
    {
        esp_log_buffer_char(LOG_TAG, (char *)p_data->notify.value, p_data->notify.value_len);
    }
}

void Ble::free_gattc_srv_db(void)
{
    is_connect = false;
    spp_gattc_if = 0xff;
    spp_conn_id = 0;
    spp_mtu_size = 23;
    cmd = 0;
    spp_srv_start_handle = 0;
    spp_srv_end_handle = 0;
    notify_value_p = NULL;
    notify_value_offset = 0;
    notify_value_count = 0;
    if (db)
    {
        free(db);
        db = NULL;
    }
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

    cmd_reg_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(Ble::spp_client_reg_task, "spp_client_reg_task", 2048, NULL, 10, NULL);
}