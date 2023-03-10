#pragma once

#include "service.hpp"

#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"

#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

class Device
{
public:
  typedef esp_ble_gap_cb_param_t::ble_scan_result_evt_param bleScanResult;
  typedef esp_ble_gattc_cb_param_t::gattc_read_char_evt_param CharacteristicReadResult;
  typedef esp_ble_gattc_cb_param_t::gattc_open_evt_param OpenEventInfo;
  typedef esp_ble_gattc_cb_param_t::gattc_search_res_evt_param
      ServiceSearchResult;
  typedef esp_bt_uuid_t serviceUUIDType;

  typedef esp_ble_gattc_cb_param_t::gattc_notify_evt_param characteristicCbParamType;
  typedef int serviceCbRetType;
  typedef std::function<serviceCbRetType(characteristicCbParamType)> characteristicCallbackType;
  // serviceCbreturnType aFunctionName(characteristicCbParamType aParam){}

  typedef uint16_t characterHandleType;
  typedef std::pair<characterHandleType, characteristicCallbackType> characteristicCBPairType;

  Device(bleScanResult res);

  bleScanResult getScanResult(){return mScanResult;}

  // Pre Connection
  std::string getName();
  esp_bd_addr_t *getAddress();
  esp_ble_addr_type_t getAddressType();
  bool isConnected();

  // Post Connection
  esp_bd_addr_t *getRemoteAddress();
  uint16_t getConnectionId();

  void searchServices();
  bool isServicesSearchComplete();
  void describeServices();

  void registerforCharacteristicNotify(characterHandleType aCharacteristicHndl, characteristicCallbackType aCallback);

  void registerForJoystickCharacteristics();

  // Interface with ESP BLE API To update Device state
  void openConnection(OpenEventInfo aOpenEvent);

  void serviceSearchComplete();
  void addFoundService(Service::espIdfTy aService);

  void setGattcIf(uint8_t aGattcIf);
  uint8_t getGattcIf();

  serviceCbRetType handleCharacteristicNotify(characteristicCbParamType params);
  void handleCharacteristicRead(Device::CharacteristicReadResult aReadResult);

protected:
  // Pre Connection
  bleScanResult mScanResult;

  // Post Connection
  bool mConnected = false;
  uint8_t mGattcIf;

  bool mIsServiceSearching = false;
  std::vector<Service> mServicesFound;
  std::map<characterHandleType, characteristicCallbackType> mserviceCallbacks;

  esp_bd_addr_t mRemoteAddress;
  uint16_t mConnectionId;
};