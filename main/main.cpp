/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
 *
 * This file is for ble spp client demo.
 *
 ****************************************************************************/

#include "ble.hpp"

#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_system.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"

#define MATTS_TAG "MATT PRINTS"
#define LOG_TAG "Main"
#define PROFILE_NUM 1
#define PROFILE_APP_ID 0
#define BT_BD_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
#define BT_BD_ADDR_HEX(addr) addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
#define ESP_GATT_SPP_SERVICE_UUID 0xABF0
#define SCAN_ALL_THE_TIME 0

Ble *bluetooth = new Ble();

void uart_task(void *pvParameters)
{
    uart_event_t event;
    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(bluetooth->spp_uart_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            switch (event.type)
            {
            // Event of UART receving data
            case UART_DATA:
                if (event.size && (bluetooth->is_connect == true) && (bluetooth->db != NULL) && ((bluetooth->db + SPP_IDX_SPP_DATA_RECV_VAL)->properties & (ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_WRITE)))
                {
                    uint8_t *temp = NULL;
                    temp = (uint8_t *)malloc(sizeof(uint8_t) * event.size);
                    if (temp == NULL)
                    {
                        ESP_LOGE(LOG_TAG, "malloc failed,%s L#%d\n", __func__, __LINE__);
                        break;
                    }
                    memset(temp, 0x0, event.size);
                    uart_read_bytes(UART_NUM_0, temp, event.size, portMAX_DELAY);
                    esp_ble_gattc_write_char(bluetooth->spp_gattc_if,
                                             bluetooth->spp_conn_id,
                                             (bluetooth->db + SPP_IDX_SPP_DATA_RECV_VAL)->attribute_handle,
                                             event.size,
                                             temp,
                                             ESP_GATT_WRITE_TYPE_RSP,
                                             ESP_GATT_AUTH_REQ_NONE);
                    free(temp);
                }
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

static void spp_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 8192, 10, &bluetooth->spp_uart_queue, 0);
    // Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    // Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(uart_task, "uTask", 2048, (void *)UART_NUM_0, 8, NULL);
}

extern "C" void app_main(void)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    nvs_flash_init();
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

    bluetooth->ble_client_appRegister();
    spp_uart_init();
}
