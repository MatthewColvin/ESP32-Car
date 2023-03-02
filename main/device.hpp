#pragma once
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <utility>

class Device
{
public:
  typedef esp_ble_gap_cb_param_t::ble_scan_result_evt_param bleScanResult;
  typedef esp_ble_gattc_cb_param_t::gattc_open_evt_param OpenEventInfo;
  typedef esp_ble_gattc_cb_param_t::gattc_search_res_evt_param ServiceSearchResult;
  typedef esp_bt_uuid_t serviceUUIDType;

  typedef int serviceCbParamType;
  typedef int serviceCbRetType;
  typedef std::function<serviceCbRetType(serviceCbParamType)> serviceCallbackType;
  typedef std::pair<uint16_t, serviceCallbackType> serviceCBPairType;

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
  void addFoundService(ServiceSearchResult aService);
  void serviceSearchComplete();
  bool isServicesSearchComplete();

  void registerService(ServiceSearchResult aService, serviceCallbackType aCallback);
  serviceCbRetType handleService(uint16_t uuid, serviceCbParamType params);

private:
  // Pre Connection
  bleScanResult mScanResult;

  // Post Connection
  bool mConnected = false;
  uint8_t mGattcIf;

  bool mIsServiceSearching = false;
  std::vector<ServiceSearchResult> mServicesFound;
  std::map<serviceUUIDType, serviceCallbackType> mserviceCallbacks;

  esp_bd_addr_t mRemoteAddress;
  uint16_t mConnectionId;
};