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
  Device(bleScanResult res);

  // Pre-Connection
  bleScanResult getScanResult() { return mScanResult; }
  std::string getName();

protected:
  // Pre Connection
  esp_bd_addr_t *getAddress();
  esp_ble_addr_type_t getAddressType();
  bool isConnected() { return mConnected; };

  // Post Connection
  esp_bd_addr_t *getRemoteAddress() { return &mRemoteAddress; };
  uint16_t getConnectionId() { return mConnectionId; };

  void searchServices();
  bool isServicesSearchComplete();
  void describeServices();

  // Notification Handling types and Decelerations
  typedef esp_ble_gattc_cb_param_t::gattc_notify_evt_param characteristicCbParamType;
  typedef int serviceCbRetType;
  typedef std::function<serviceCbRetType(characteristicCbParamType)> characteristicCallbackType;
  typedef std::pair<Characteristic, characteristicCallbackType> characteristicCBPairType;
  void registerForCharacteristicNotify(Characteristic aCharacteristic, characteristicCallbackType aCallback);
  void unRegisterForCharacteristicNotify(Characteristic aCharacteristic);

private: // Interface with ESP BLE API To update Device state
  // Typedefs into the ESP GATTC API
  typedef esp_ble_gattc_cb_param_t::gattc_read_char_evt_param CharacteristicReadResult;
  typedef esp_ble_gattc_cb_param_t::gattc_open_evt_param OpenEventInfo;
  typedef esp_ble_gattc_cb_param_t::gattc_search_res_evt_param ServiceSearchResult;
  typedef esp_ble_gattc_cb_param_t::gattc_reg_for_notify_evt_param NotifyRegistrationType;
  typedef esp_ble_gattc_cb_param_t::gattc_unreg_for_notify_evt_param NotifyUnregistrationType;

  /**
   * @brief Update Device state to connected and copy in
   *        remote address from the aOpenEvent
   *
   * @param aOpenEvent - GATTC Alias to the open even
   */
  void openConnection(OpenEventInfo aOpenEvent);
  /**
   * @brief Function to check if we have found all services
   *        associated with the server we are connected with
   */
  void serviceSearchComplete();
  /**
   * @brief Add Service to the @ref mFoundServices vector
   *
   * @param aService - service to add
   */
  void addFoundService(Service::espIdfTy aService);

  /**
   * @brief Gattc Callback returns read result and this function handles the
   *        read. Right now it just dumps the hex to a log.
   *  TODO:May be able to make this take a callback
   *       and call it when we receive this
   *
   * @param aReadResult - GATTC event for reading
   */
  void handleCharacteristicRead(Device::CharacteristicReadResult aReadResult);

  // Notification Handling Section
  /**
   * @brief Use ESP GATTC Api to register the notification by writing the
   *        the a descriptor called the client characteristic config
   * @param aRegistration - GATTC event to register
   */
  void handleNotifyRegistration(NotifyRegistrationType aRegistration);
  /**
   * @brief Use ESP GATTC Api to unregister the notification by writing the
   *        the a descriptor called the client characteristic config
   * @param anUnregistration - GATTC event to unregister
   */
  void handleNotifyUnregistration(NotifyUnregistrationType anUnregistration);
  /**
   * @brief Function that is called when a notification is received by the
   *        ESP GATTC API from the server.
   *
   * @param params - Data the server sent describing the event and
   * @return serviceCbRetType - TODO: need to find out how to use this maybe an error return val?
   */
  serviceCbRetType handleCharacteristicNotify(characteristicCbParamType params);

protected: // Protect data members so extened class can use them
  // Pre Connection
  bleScanResult mScanResult;

  // Post Connection
  bool mConnected = false;
  uint8_t mGattcIf; // A simple handle that GATTC uses to identify the Device

  bool mIsServiceSearching = false;
  std::vector<Service> mServicesFound;
  std::map<Characteristic, characteristicCallbackType> mserviceCallbacks;

  esp_bd_addr_t mRemoteAddress;
  uint16_t mConnectionId;

  /**
   * @brief Get the Characteristic associated with the servers handle associated with it.
   *        When a Characteristic is not found it will print an error and return a default Characteristic
   *        which will surely not be what is expected.
   *
   *        TODO: may want to look into handling error here better than just printing error std::optional return?
   *
   * @param aSearchHandle - aService Handle for a device used in many places throughout esp_gattc_api.h
   * @return Characteristic - an instance of a characteristic that this device has in one of its services
   */
  Characteristic getCharacteristic(uint16_t aSearchHandle);
};