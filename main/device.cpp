#include "device.hpp"
#include "esp_gattc_api.h"
#include "esp_log.h"

#include <cstring>

#define LOG_TAG "Device"

Device::Device(bleScanResult res) { mScanResult = res; }

std::string Device::getName() {
  uint8_t *cstr_name = NULL;
  uint8_t name_len = 0;

  cstr_name = esp_ble_resolve_adv_data(mScanResult.ble_adv,
                                       ESP_BLE_AD_TYPE_NAME_CMPL, &name_len);

  std::string name;
  name.assign(reinterpret_cast<char *>(cstr_name), name_len);

  return name;
}

esp_bd_addr_t *Device::getAddress() {
  // esp_log_buffer_hex(LOG_TAG, scan_result->scan_rst.bda, 6);
  // Decided to go with this but if they save the address of and the device
  // falls out of scope we have issues
  return &mScanResult.bda;
}

esp_ble_addr_type_t Device::getAddressType() {
  return mScanResult.ble_addr_type;
}

void Device::openConnection(OpenEventInfo aOpenEvent) {

  mConnected = true;

  memcpy(mRemoteAddress, aOpenEvent.remote_bda, sizeof(esp_bd_addr_t));
  mConnectionId = aOpenEvent.conn_id;
}

bool Device::isConnected() { return mConnected; }
esp_bd_addr_t *Device::getRemoteAddress() { return &mRemoteAddress; }
uint16_t Device::getConnectionId() { return mConnectionId; }

uint8_t Device::getGattcIf() { return mGattcIf; }
void Device::setGattcIf(uint8_t aGattcIf) { mGattcIf = aGattcIf; }

void Device::searchServices() {
  if (!mConnected) {
    ESP_LOGE(LOG_TAG, "Cannot Search for Services without connection!");
    return;
  }
  esp_ble_gattc_search_service(mGattcIf, mConnectionId, nullptr);
}
