#pragma once

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include <vector>

#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"

struct gattc_profile_inst
{
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

class Ble
{
public:
    Ble(Ble &other) = delete;
    void operator=(const Ble &) = delete;
    static Ble *getInstance();

    static void ble_client_appRegister();

    static std::vector<esp_ble_gap_cb_param_t::ble_scan_result_evt_param> scan(uint32_t secondsToScan);

protected:
    Ble();
    static Ble *mInstance;

private:
    static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
    static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
    static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

    static bool is_connect;
    static esp_ble_gap_cb_param_t scan_rst;
    static esp_ble_scan_params_t ble_scan_params;

    static const char *device_name;
    static gattc_profile_inst gl_profile_tab[];
};