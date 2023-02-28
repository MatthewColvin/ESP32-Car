#pragma once
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include <string>

class Device {
public:
  typedef esp_ble_gap_cb_param_t::ble_scan_result_evt_param bleScanResult;
  typedef esp_ble_gattc_cb_param_t::gattc_open_evt_param OpenEventInfo;

  Device(bleScanResult res);

  // Pre-Connetion
  std::string getName();
  esp_bd_addr_t *getAddress();
  esp_ble_addr_type_t getAddressType();

  // Post-Connection
  void openConnection(OpenEventInfo aOpenEvent);
  bool isConnected();

  esp_bd_addr_t *getRemoteAddress();
  uint16_t getConnectionId();

  void setGattcIf(uint8_t aGattcIf);
  uint8_t getGattcIf();

  void searchServices();

private:
  // Pre Connection
  bleScanResult mScanResult;

  // Post Connection
  bool mConnected = false;
  uint8_t mGattcIf;

  esp_bd_addr_t mRemoteAddress;
  uint16_t mConnectionId;
};