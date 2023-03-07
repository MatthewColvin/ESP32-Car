#pragma once
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


  void handleCharacteristicRead(Device::CharacteristicReadResult aReadResult);
  void registerService(characterHandleType aCharacteristicHndl, characteristicCallbackType aCallback);
  serviceCbRetType handleService(characteristicCbParamType params);

  std::vector<esp_gattc_char_elem_t> getCharacteristics(const Device::ServiceSearchResult& aService);
  std::vector<esp_gattc_descr_elem_t> getDescriptors(const Device::ServiceSearchResult& aService,const esp_gattc_char_elem_t& aCharacteristic);

  void describeCharacteristic(const esp_gattc_char_elem_t& aCharacteristic , Device::ServiceSearchResult* aService = nullptr);
  void describeService(const Device::ServiceSearchResult& aService);
  void describeServices();

  void registerForJoystickCharacteristics();

private:
  // Pre Connection
  bleScanResult mScanResult;

  // Post Connection
  bool mConnected = false;
  uint8_t mGattcIf;

  bool mIsServiceSearching = false;
  std::vector<ServiceSearchResult> mServicesFound;
  std::map<characterHandleType, characteristicCallbackType> mserviceCallbacks;

  esp_bd_addr_t mRemoteAddress;
  uint16_t mConnectionId;
};