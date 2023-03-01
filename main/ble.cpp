#include "ble.hpp"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_gatt_defs.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "string.h"
#include <optional>
#include <vector>

#define LOG_TAG "BLE"
#define MATTS_TAG "MATT PRINTS"
#define PROFILE_APP_ID 0
#define BT_BD_ADDR_STR "%02x:%02x:%02x:%02x:%02x:%02x"
#define BT_BD_ADDR_HEX(addr)                                                   \
  addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
#define ESP_GATT_SPP_SERVICE_UUID 0xABF0
#define SCAN_ALL_THE_TIME 0

bool Ble::is_connect = false;

const char *Ble::device_name = "VR-PARK";
Ble *Ble::mInstance = nullptr;

std::vector<Device> scannedDevices;
std::vector<Device> connectedDevices;

Ble::Ble() {
  // Release Bluetooth Classic we will not need it.
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

  esp_err_t ret;
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(LOG_TAG, "%s enable controller failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(LOG_TAG, "%s enable controller failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }

  ESP_LOGI(LOG_TAG, "%s init bluetooth\n", __func__);
  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(LOG_TAG, "%s init bluetooth failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }
  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(LOG_TAG, "%s enable bluetooth failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }

  ble_client_appRegister();
}

Ble *Ble::getInstance() {
  if (mInstance == nullptr) {
    mInstance = new Ble();
  }
  return mInstance;
}

std::vector<Device> Ble::scan(uint32_t secondsToScan,
                              esp_ble_scan_params_t aScanParams) {
  esp_ble_gap_set_scan_params(&aScanParams);
  esp_ble_gap_start_scanning(secondsToScan);
  vTaskDelay(secondsToScan * 1000 / portTICK_PERIOD_MS);
  esp_ble_gap_stop_scanning();
  return scannedDevices;
}

bool Ble::connect(Device aDevice) {
  // Use index to try to get reference to device will pose issue if accounting
  // for case of disconnect

  // there is a limitation on number of connected devices
  // the gattc_if is only valid for 3-8
  if (connectedDevices.size() + 2 > 8) {
    ESP_LOGE(LOG_TAG, "ERROR Can only Connect 6 devices!");
    return false;
  }

  esp_err_t ret =
      esp_ble_gattc_open(connectedDevices.size() + 3, *aDevice.getAddress(),
                         aDevice.getAddressType(), true);
  if (ret == ESP_OK) {
    connectedDevices.push_back(aDevice);
  }
  return ret == ESP_OK;
}

void Ble::esp_gap_cb(esp_gap_ble_cb_event_t event,
                     esp_ble_gap_cb_param_t *param) {
  esp_err_t err;

  switch (event) {
  case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
    break;
  }
  case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
    // scan start complete event to indicate scan start successfully or failed
    if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(LOG_TAG, "Scan start failed: %s", esp_err_to_name(err));
      break;
    }
    ESP_LOGI(LOG_TAG, "Scan start successed Clearing Devices");
    scannedDevices.clear();
    break;
  }
  case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: {
    if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(LOG_TAG, "Scan stop failed: %s", esp_err_to_name(err));
      break;
    }
    ESP_LOGI(LOG_TAG, "Scan stop successed. Found %d Devices",
             scannedDevices.size());
    break;
  }
  case ESP_GAP_BLE_SCAN_RESULT_EVT: {
    esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;

    switch (scan_result->scan_rst.search_evt) {
    case ESP_GAP_SEARCH_INQ_RES_EVT: {
      Device d(scan_result->scan_rst);
      scannedDevices.push_back(d);
    }
    case ESP_GAP_SEARCH_INQ_CMPL_EVT:
      break;
    default:
      break;
    }
    break;
  }
  case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT: {
    if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(LOG_TAG, "Adv stop failed: %s", esp_err_to_name(err));
    } else {
      ESP_LOGI(LOG_TAG, "Stop adv successfully");
    }
    break;
  }
  default:
    break;
  }
}

void Ble::esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                       esp_ble_gattc_cb_param_t *param) {
  ESP_LOGI(LOG_TAG, "EVT %d, gattc if %d", event, gattc_if);

  size_t cbDeviceIdx = gattc_if - 3;
  if (connectedDevices.size() > cbDeviceIdx) {
    ESP_LOGI(LOG_TAG, "Found Device %s",
             connectedDevices[cbDeviceIdx].getName().c_str());
  }

  switch (event) {
  case ESP_GATTC_REG_EVT: {
    if (param->reg.status == ESP_GATT_OK) {
      // May need to store something into the device???
    } else {
      ESP_LOGI(LOG_TAG, "Reg app failed, app_id %04x, status %d",
               param->reg.app_id, param->reg.status);
      return;
    }
    break;
  }
  case ESP_GATTC_OPEN_EVT: {
    connectedDevices[cbDeviceIdx].openConnection(param->open);
    break;
  }
  case ESP_GATTC_CONNECT_EVT: {
    connectedDevices[cbDeviceIdx].setGattcIf(gattc_if);
    break;
  }
  case ESP_GATTC_DIS_SRVC_CMPL_EVT: {
    connectedDevices[cbDeviceIdx].searchServices();
    break;
  }
  default: {
    break;
  }
  }
}

void Ble::ble_client_appRegister(void) {
  esp_err_t status;
  char err_msg[20];

  ESP_LOGI(LOG_TAG, "register callback");

  // register the scan callback function to the gap module
  if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
    ESP_LOGE(LOG_TAG, "gap register error: %s",
             esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
    return;
  }
  // register the callback function to the gattc module
  if ((status = esp_ble_gattc_register_callback(esp_gattc_cb)) != ESP_OK) {
    ESP_LOGE(LOG_TAG, "gattc register error: %s",
             esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
    return;
  }
  esp_ble_gattc_app_register(PROFILE_APP_ID);

  esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(200);
  if (local_mtu_ret) {
    ESP_LOGE(LOG_TAG, "set local  MTU failed: %s",
             esp_err_to_name_r(local_mtu_ret, err_msg, sizeof(err_msg)));
  }
}