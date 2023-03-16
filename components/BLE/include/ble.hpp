#pragma once
// BLE Component
#include "device.hpp"
// ESP API
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "driver/uart.h"
// RTOS
#include "freertos/FreeRTOS.h"
// STD
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <memory>

static constexpr esp_ble_scan_params_t default_ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE};

/**
 * Bluetooth low energy client c++ wrapper around ESP-32's Bluetooth library
 * This singelton handles configuring BLE GAP and GATT callbacks on construction and
 * while running provides updates the the devices under its control in mConnectedDevices.
 */
class Ble
{
public:
  Ble(Ble &other) = delete;
  void operator=(const Ble &) = delete;
  static std::shared_ptr<Ble> getInstance();

  /**
   * @brief Function to get a list of devices in pairing mode
   *
   * @param secondsToScan - time in seconds to scan for devices
   * @param scanParams - scan parameters to allow for better control of scan
   * @return std::vector<Device>
   */
  static std::vector<Device>
  scan(uint32_t secondsToScan,
       esp_ble_scan_params_t scanParams = default_ble_scan_params);

  /**
   * @brief Function to connect to a device
   *
   * @param aDevice - a Device to connect to
   *                          NOTE: this has the potential to change so should look at redesign here eventually.
   * @return false device failed to connect
   *         true device connected and added to connected devices
   *               continue to use your passed shared pointer to access device functionality
   */
  static bool connect(std::shared_ptr<Device> aDevice);

protected:
  /**
   * @brief Initialized Callbacks for handling GATTC and GAP in the ESP API
   *        Singleton Paradigm only private constructor
   */
  Ble();
  /** @brief // Instance Of the BleClass not thread safe but hope to do that in the future */
  static std::shared_ptr<Ble> mInstance;
  /** @brief  devices found in GAP callback loaded and returned with the @ref Ble::scan() function */
  static std::vector<Device> scannedDevices;
  /** @brief  devices connected through the Ble::connect function */
  static std::vector<std::shared_ptr<Device>> connectedDevices; // devices to update in the GATTC callback

private:
  /**
   * @brief Register the GAP and GATTC callbacks to ESP BLE API
   *        Initiate the ESP Bluedroid API
   *        Setup some security parameters
   */
  static void ble_client_appRegister();
  /**
   * @brief GAP(GENERIC ACCESS PROFILE) -
   *        The GAP layer of the BLE protocol stack is
   *        responsible for connection functionality.
   *
   *        The GAP layer handles the access modes and procedures
   *        of the device:
   *        device discovery, link establishment, link termination,
   *        initiation of security features, and device configuration
   */
  static void esp_gap_cb(esp_gap_ble_cb_event_t event,
                         esp_ble_gap_cb_param_t *param);

  /**
   * @brief GATT (Generic Attribute Profile)
   *         GATT defines the way that two Bluetooth Low Energy devices
   *         transfer data back and forth using concepts called
   *         Services and Characteristics.
   */
  static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param);
};