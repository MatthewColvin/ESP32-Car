#pragma once
#include "esp_gap_ble_api.h"
#include <string>

class Device {
public:
  typedef esp_ble_gap_cb_param_t::ble_scan_result_evt_param bleScanResult;
  Device(bleScanResult res);

  std::string getName();

private:
  bleScanResult mScanResult;
};