// BLE Component
#include "ble.hpp"
// ESP API
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_gatt_defs.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
// RTOS
#include "freertos/FreeRTOS.h"
// STD
#include <string>
#include <optional>
#include <vector>

#define LOG_TAG "BLE"
#define PROFILE_APP_ID 0

std::shared_ptr<Ble> Ble::mInstance = nullptr;
std::vector<Device> Ble::scannedDevices;
std::vector<std::shared_ptr<Device>> Ble::connectedDevices;

bool addrEq(const esp_bd_addr_t lhs, const esp_bd_addr_t rhs)
{
  for (int i = 0; i < sizeof(esp_bd_addr_t); i++)
  {
    if (lhs[i] != rhs[i])
    {
      return false;
    }
  }
  return true;
}

std::shared_ptr<Ble> Ble::getInstance()
{
  if (mInstance == nullptr)
  {
    // not thread save
    mInstance = std::shared_ptr<Ble>(new Ble());
  }
  return mInstance;
}

std::vector<Device> Ble::scan(uint32_t secondsToScan,
                              esp_ble_scan_params_t aScanParams)
{
  esp_ble_gap_set_scan_params(&aScanParams);
  esp_ble_gap_start_scanning(secondsToScan);
  vTaskDelay(secondsToScan * 1000 / portTICK_PERIOD_MS);
  esp_ble_gap_stop_scanning();
  return scannedDevices;
}

bool Ble::connect(std::shared_ptr<Device> aDevice)
{
  // Use index to try to get reference to device will pose issue if accounting
  // for case of disconnect

  // there is a limitation on number of connected devices
  // the gattc_if is only valid for 3-8
  if (connectedDevices.size() + 2 > 8)
  {
    ESP_LOGE(LOG_TAG, "ERROR Can only Connect 6 devices!");
    return false;
  }

  esp_err_t ret =
      esp_ble_gattc_open(connectedDevices.size() + 3, *aDevice->getAddress(),
                         aDevice->getAddressType(), true);
  if (ret == ESP_OK)
  {
    connectedDevices.push_back(aDevice);
    return true;
  }
  return false;
}

Ble::Ble()
{
  // Release Bluetooth Classic we will not need it.
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

  esp_err_t ret;
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret)
  {
    ESP_LOGE(LOG_TAG, "%s enable controller failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret)
  {
    ESP_LOGE(LOG_TAG, "%s enable controller failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }

  ESP_LOGI(LOG_TAG, "%s init bluetooth\n", __func__);
  ret = esp_bluedroid_init();
  if (ret)
  {
    ESP_LOGE(LOG_TAG, "%s init bluetooth failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }
  ret = esp_bluedroid_enable();
  if (ret)
  {
    ESP_LOGE(LOG_TAG, "%s enable bluetooth failed: %s\n", __func__,
             esp_err_to_name(ret));
    return;
  }

  ble_client_appRegister();
}

// Helper for logging events
bool gap_event_handeled(esp_gap_ble_cb_event_t event)
{
  switch (event)
  {
  case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
  case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
  case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
  case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
  case ESP_GAP_BLE_SCAN_RESULT_EVT:
  case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
  case ESP_GAP_BLE_AUTH_CMPL_EVT:
    return true;
  default:
    return false;
  }
}

void Ble::esp_gap_cb(esp_gap_ble_cb_event_t event,
                     esp_ble_gap_cb_param_t *param)
{
  esp_err_t err;
  if (!gap_event_handeled(event))
  {
    ESP_LOGE(LOG_TAG, "Unhandled GAP Event: %d", event);
    return;
  }

  switch (event)
  {
  case ESP_GAP_BLE_AUTH_CMPL_EVT:
  {
    esp_bd_addr_t bd_addr;
    memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));

    auto authDevice = std::find_if(connectedDevices.begin(), connectedDevices.end(),
                                   [bd_addr](std::shared_ptr<Device> device)
                                   { return addrEq(bd_addr, *device->getAddress()); });

    if (authDevice != connectedDevices.end())
    {
      (*authDevice)->handleAuthComplete(param->ble_security);
    }
    else
    {
      ESP_LOGE(LOG_TAG, "Unable to Find Device During Authentication!");
    }

    break;
  }
  case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
  {
    ESP_LOGI(LOG_TAG, "GAP Recieved Event: %d", event);
    break;
  }
  case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
  {
    ESP_LOGI(LOG_TAG, "local privacy set %d", event);
    ESP_ERROR_CHECK(param->local_privacy_cmpl.status);
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
    ESP_LOGI(LOG_TAG, "Scan start successed Clearing Devices");
    scannedDevices.clear();
    break;
  }
  case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
  {
    if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS)
    {
      ESP_LOGE(LOG_TAG, "Scan stop failed: %s", esp_err_to_name(err));
      break;
    }
    ESP_LOGI(LOG_TAG, "Scan stop successed. Found %d Devices",
             scannedDevices.size());
    break;
  }
  case ESP_GAP_BLE_SCAN_RESULT_EVT:
  {
    esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;

    switch (scan_result->scan_rst.search_evt)
    {
    case ESP_GAP_SEARCH_INQ_RES_EVT:
    {
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

// Helper for logging events
bool gattcEventHandledByDevice(esp_gattc_cb_event_t event)
{
  switch (event)
  {
  case ESP_GATTC_OPEN_EVT:
  case ESP_GATTC_CONNECT_EVT:
  case ESP_GATTC_SEARCH_RES_EVT:
  case ESP_GATTC_SEARCH_CMPL_EVT:
  case ESP_GATTC_DIS_SRVC_CMPL_EVT:
  case ESP_GATTC_NOTIFY_EVT:
  case ESP_GATTC_REG_FOR_NOTIFY_EVT:
  case ESP_GATTC_UNREG_FOR_NOTIFY_EVT:
  case ESP_GATTC_READ_DESCR_EVT:
  case ESP_GATTC_READ_CHAR_EVT:
  // BLE Handeled
  case ESP_GATTC_WRITE_CHAR_EVT:
  case ESP_GATTC_WRITE_DESCR_EVT:
    return true;
  default:
    return false;
  }
}

void Ble::esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                       esp_ble_gattc_cb_param_t *param)
{
  if (event == ESP_GATTC_REG_EVT)
  {
    ESP_LOGI(LOG_TAG, "Registered Gattc Callback :)");
    ESP_ERROR_CHECK(param->reg.status);
    return;
  }

  size_t cbDeviceIdx = gattc_if - 3;
  std::shared_ptr<Device> cbDevice = nullptr;

  if (cbDeviceIdx < connectedDevices.size())
  {

    cbDevice = connectedDevices[cbDeviceIdx];

    if (!gattcEventHandledByDevice(event))
    {
      ESP_LOGE(LOG_TAG, "UnHandled EVT %d, gattc if %d", event, gattc_if);
      if (connectedDevices.size() > cbDeviceIdx)
      {
        ESP_LOGE(LOG_TAG, "Found Device %s", cbDevice->getName().c_str());
      }
    }
  }
  else
  {
    ESP_LOGE(LOG_TAG, "No Device to handle event %d", event);
  }

  switch (event)
  {
  case ESP_GATTC_OPEN_EVT:
  {
    cbDevice->openConnection(param->open);
    break;
  }
  case ESP_GATTC_CONNECT_EVT:
  {
    cbDevice->mGattcIf = gattc_if;
    break;
  }
  case ESP_GATTC_SEARCH_RES_EVT:
  {
    cbDevice->addFoundService(param->search_res);
    break;
  }
  case ESP_GATTC_SEARCH_CMPL_EVT:
  {
    cbDevice->serviceSearchComplete();
    break;
  }
  case ESP_GATTC_DIS_SRVC_CMPL_EVT:
  {
    cbDevice->searchServices();
    break;
  }
  case ESP_GATTC_NOTIFY_EVT:
  {
    cbDevice->handleCharacteristicNotify(param->notify);
    break;
  }
  case ESP_GATTC_REG_FOR_NOTIFY_EVT:
  {
    cbDevice->handleNotifyRegistration(param->reg_for_notify);
    break;
  }
  case ESP_GATTC_UNREG_FOR_NOTIFY_EVT:
  {
    cbDevice->handleNotifyUnregistration(param->unreg_for_notify);
    break;
  }
  case ESP_GATTC_READ_CHAR_EVT:
  case ESP_GATTC_READ_DESCR_EVT:
  {
    cbDevice->handleCharacteristicRead(param->read);
    break;
  }
  case ESP_GATTC_WRITE_CHAR_EVT:
  case ESP_GATTC_WRITE_DESCR_EVT:
  {
    if (param->write.status != ESP_OK)
    {
      ESP_LOGE(LOG_TAG, "Failed to Write to Characteristic or Descriptor.");
    }
    break;
  }

  default:
  {
    break;
  }
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
    ESP_LOGE(LOG_TAG, "gap register error: %s",
             esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
    return;
  }
  // register the callback function to the gattc module
  if ((status = esp_ble_gattc_register_callback(esp_gattc_cb)) != ESP_OK)
  {
    ESP_LOGE(LOG_TAG, "gattc register error: %s",
             esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
    return;
  }
  esp_ble_gattc_app_register(PROFILE_APP_ID);

  esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(200);
  if (local_mtu_ret)
  {
    ESP_LOGE(LOG_TAG, "set local  MTU failed: %s",
             esp_err_to_name_r(local_mtu_ret, err_msg, sizeof(err_msg)));
  }

  ESP_ERROR_CHECK(esp_ble_gap_config_local_privacy(true)); // set local privacy to true to help with bond?

  // *****steal whole block from security client example

  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND; // bonding with peer device after authentication
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));

  esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE; // set the IO capability to No output No input
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));

  uint8_t key_size = 16; // the key size should be 7~16 bytes
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));

  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));

  uint8_t oob_support = ESP_BLE_OOB_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));

  /* If your BLE device act as a Slave, the init_key means you hope which types
  of key of the master should distribute to you, and the response key means
  which key you can distribute to the Master; If your BLE device act as a
  master, the response key means you hope which types of key of the slave should
  distribute to you,
  and the init key means which key you can distribute to the slave. */
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
  /// **** end stolen block
}