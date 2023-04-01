#pragma once

#include "esp_hid_gap.h"
#include "esp_hidh.h"

#include <string>

class BTClassicHID;

class HIDDevice
{
    friend BTClassicHID;

public:
    HIDDevice(esp_hid_scan_result_t aScanResult);

    uint8_t *getAddress() { return mScanResult.bda; }
    bool hasAddress(const esp_bd_addr_t anAddress);
    esp_hid_transport_t getTransport() { return mScanResult.transport; }
    esp_ble_addr_type_t getAddressType();

protected:
    virtual void handleOpenEvent(esp_hidh_event_data_t *aOpenEvent);
    virtual void handleBatteryEvent(esp_hidh_event_data_t *aBatteryEvent);
    virtual void handleInputEvent(esp_hidh_event_data_t *anInputEvent);
    virtual void handleFeatureEvent(esp_hidh_event_data_t *aFeatureEvent);
    virtual void handleCloseEvent(esp_hidh_event_data_t *aCloseEvent);

private:
    esp_hid_scan_result_t mScanResult;
};