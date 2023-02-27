#include "device.hpp"

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
// esp_log_buffer_hex(LOG_TAG, scan_result->scan_rst.bda, 6);
