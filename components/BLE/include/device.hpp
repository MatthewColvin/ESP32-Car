#pragma once
// BLE Component
#include "service.hpp"
// ESP API
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
// RTOS
// STD
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

class Ble;

class Device
{
  friend Ble;

public:
  typedef esp_ble_gap_cb_param_t::ble_scan_result_evt_param bleScanResult;
  typedef esp_bt_uuid_t UUIDType;

  typedef esp_ble_gattc_cb_param_t::gattc_notify_evt_param characteristicCbParamType;
  typedef int serviceCbRetType;
  typedef std::function<serviceCbRetType(characteristicCbParamType)> characteristicCallbackType;

  typedef std::pair<Characteristic, characteristicCallbackType> characteristicCBPairType;

  Device(bleScanResult res);

  bleScanResult getScanResult() { return mScanResult; }

  // Pre Connection
  std::string getName();
  esp_bd_addr_t *getAddress();
  esp_ble_addr_type_t getAddressType();
  bool isConnected() { return mConnected; };

  // Post Connection
  esp_bd_addr_t *getRemoteAddress() { return &mRemoteAddress; };
  uint16_t getConnectionId() { return mConnectionId; };

  void searchServices();
  bool isServicesSearchComplete();
  void describeServices();

  void registerforCharacteristicNotify(Characteristic aCharacteristic, characteristicCallbackType aCallback);
  void unRegisterForCharacterisitcNotify(Characteristic aCharacteristic);

protected: // Interface with ESP BLE API To update Device state
  // Typedefs into the ESP GATTC API
  typedef esp_ble_gattc_cb_param_t::gattc_read_char_evt_param CharacteristicReadResult;
  typedef esp_ble_gattc_cb_param_t::gattc_open_evt_param OpenEventInfo;
  typedef esp_ble_gattc_cb_param_t::gattc_search_res_evt_param ServiceSearchResult;
  typedef esp_ble_gattc_cb_param_t::gattc_reg_for_notify_evt_param NotifyRegistrationType;
  typedef esp_ble_gattc_cb_param_t::gattc_unreg_for_notify_evt_param NotifyUnregistrationType;

  void openConnection(OpenEventInfo aOpenEvent);

  void serviceSearchComplete();
  void addFoundService(Service::espIdfTy aService);

  void setGattcIf(uint8_t aGattcIf) { mGattcIf = aGattcIf; };
  uint8_t getGattcIf() { return mGattcIf; };

  serviceCbRetType handleCharacteristicNotify(characteristicCbParamType params);
  void handleCharacteristicRead(Device::CharacteristicReadResult aReadResult);
  void readAllCharacteristics();

  void handleNotifyRegistration(NotifyRegistrationType aRegistration);
  void handleNotifyUnregistration(NotifyUnregistrationType anUnregistration);

  void enableNotifitcation(Characteristic aCharacteristic);
  // void disableNotifictaion(Characteristic aCharacteristic);

  // Pre Connection
  bleScanResult mScanResult;

  // Post Connection
  bool mConnected = false;
  uint8_t mGattcIf;

  bool mIsServiceSearching = false;
  std::vector<Service> mServicesFound;
  std::map<Characteristic, characteristicCallbackType> mserviceCallbacks;

  esp_bd_addr_t mRemoteAddress;
  uint16_t mConnectionId;

private:
  Characteristic getCharacteristic(uint16_t aSearchHandle);
};