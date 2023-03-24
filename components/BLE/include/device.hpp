#pragma once
// BLE Component
#include "service.hpp"
// ESP API
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
// RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
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
  /**
   * @brief Check if device is connected
   *
   *  TODO Support the Gatt and Gap events that signal disconnection currently
   *       device will still seem connected if it loses inital connection
   *
   *  Quickly take the semaphore and give it back to check our connection.
   *
   * @return true - Device is connected
   * @return false - Device is not connected
   */
  bool isConnected();

  // Post Connection
  esp_bd_addr_t *getRemoteAddress() { return &mRemoteAddress; };
  uint16_t getConnectionId() { return mConnectionId; };

  void searchServices();
  bool isServicesSearchComplete();
  void describeServices();

  bool isAuthenticated();
  /**
   * @brief attempt to read the Device Name in order to force authentication
   */
  void authenticate();

  // Notification Handling types and Decelerations
  typedef esp_ble_gattc_cb_param_t::gattc_notify_evt_param characteristicCbParamType;
  typedef int serviceCbRetType;
  typedef std::function<serviceCbRetType(characteristicCbParamType)> characteristicCallbackType;
  typedef std::pair<Characteristic, characteristicCallbackType> characteristicCBPairType;
  void registerForCharacteristicNotify(Characteristic aCharacteristic, characteristicCallbackType aCallback);
  void unRegisterForCharacteristicNotify(Characteristic aCharacteristic);
  void unRegisterAllCharacteristicNotifications();

private: // Interface with ESP BLE API To update Device state
  // Typedefs into the ESP GATTC API
  typedef esp_ble_gattc_cb_param_t::gattc_read_char_evt_param CharacteristicReadResult;
  typedef esp_ble_gattc_cb_param_t::gattc_open_evt_param OpenEventInfo;
  typedef esp_ble_gattc_cb_param_t::gattc_search_res_evt_param ServiceSearchResult;
  typedef esp_ble_gattc_cb_param_t::gattc_reg_for_notify_evt_param NotifyRegistrationType;
  typedef esp_ble_gattc_cb_param_t::gattc_unreg_for_notify_evt_param NotifyUnregistrationType;

  /**
   * @brief Blocking call to help find out if we completed an event.
   *
   * @param anEvent - an Event semaphore
   * @param aTimeout - Time to wait for it to be given
   * @param anErrorMsg - Message to log if we do not get the semaphore in time.

   * @return true - event has occurred
   * @return false - event has not occured and we timed out.
   */
  bool checkEventSema(SemaphoreHandle_t anEvent, int aTimeout, std::string anErrorMsg);

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

  /**
   * @brief Handler for After GAP authenticates the device
   *
   */
  void handleAuthComplete(esp_ble_sec_t aSecurityInfo);

protected: // Protect data members so extened class can use them
  // Pre Connection
  bleScanResult mScanResult;

  // Post Connection
  SemaphoreHandle_t mConnectedEvent = xSemaphoreCreateBinary();

  uint8_t mGattcIf; // A simple handle that GATTC uses to identify the Device
  SemaphoreHandle_t mAuthenticatedEvent = xSemaphoreCreateBinary();

  SemaphoreHandle_t mServiceSearchCompleteEvent = xSemaphoreCreateBinary();
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