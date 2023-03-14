#pragma once

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <memory>

#include "device.hpp"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"

static constexpr esp_ble_scan_params_t default_ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE};

class Ble
{
public:
  Ble(Ble &other) = delete;
  void operator=(const Ble &) = delete;
  static Ble *getInstance();

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
   *                          NOTE: this has the potential to change so should look at redesign here.
   *                          Also there is a limit on the number of devices that can be connected so we should really
   *                          make it a vector anyway.
   * @return false device failed to connect
   *         true device connected and added to connected devices
   */
  static bool connect(std::shared_ptr<Device> aDevice);

protected:
  Ble();
  static Ble *mInstance;
  static int secToScan;
  static std::vector<Device> scannedDevices;
  static std::vector<std::shared_ptr<Device>> connectedDevices;

private:
  static void ble_client_appRegister();

  static void esp_gap_cb(esp_gap_ble_cb_event_t event,
                         esp_ble_gap_cb_param_t *param);
  static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param);
  static void gattc_profile_event_handler(esp_gattc_cb_event_t event,
                                          esp_gatt_if_t gattc_if,
                                          esp_ble_gattc_cb_param_t *param);

  static void gattc_hid_event_handler(esp_gattc_cb_event_t event,
                                      esp_gatt_if_t gattc_if,
                                      esp_ble_gattc_cb_param_t *param);
  static void gattc_battery_event_handler(esp_gattc_cb_event_t event,
                                          esp_gatt_if_t gattc_if,
                                          esp_ble_gattc_cb_param_t *param);
};