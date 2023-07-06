#pragma once
#include "HIDDevice.hpp"

#include "esp_hidh.h"
#include "esp_hid_gap.h"

#include <memory>
#include <vector>
#include <array>

#define MAX_CONNECTED_DEVICES 5

class BTClassicHID
{
public:
    BTClassicHID(BTClassicHID &other) = delete;
    void operator=(const BTClassicHID &) = delete;
    static std::shared_ptr<BTClassicHID> getInstance();

    static std::vector<HIDDevice> scan(uint32_t seconds, esp_bd_addr_t anEarlyReturnAddress = nullptr);
    static bool connect(std::shared_ptr<HIDDevice> aDevice);

    template <typename deviceType>
    std::shared_ptr<deviceType> connect(esp_bd_addr_t aDeviceAddress, int numScans = -1, int secondsPerScan = 10);

protected:
    BTClassicHID();

    static std::shared_ptr<BTClassicHID> mInstance;
    static std::array<std::shared_ptr<HIDDevice>, MAX_CONNECTED_DEVICES> mConnectedDevices;
    static SemaphoreHandle_t initCompleteEvent;

private:
    static constexpr char LOG_TAG[] = "BTClassicHID";

    static void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
    static void onDisconnect(std::weak_ptr<HIDDevice> aDevice);
    static std::weak_ptr<HIDDevice> getDevice(esp_hidh_event_t anEvent, esp_hidh_event_data_t *aParam);
};

template <typename deviceType>
std::shared_ptr<deviceType> BTClassicHID::connect(esp_bd_addr_t aDeviceAddress, int numScans, int secondsPerScan)
{
    bool connected = false;
    if (numScans < 0)
    {
        numScans = INT_MAX;
    }

    std::shared_ptr<deviceType> retDevice = nullptr;
    for (int i = 0; i < numScans && !connected; i++)
    {
        auto devices = scan(secondsPerScan, aDeviceAddress);
        // ESP_LOGI(LOG_TAG, "Found %d Devices on Scan %d", devices.size(), i);
        for (auto device : devices)
        {
            if (device.hasAddress(aDeviceAddress))
            {
                retDevice = std::make_shared<deviceType>(device);
                if (connect(retDevice))
                {
                    // ESP_LOGI(LOG_TAG, "Connected!");
                    // ESP_LOGI(LOG_TAG, ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(retDevice->getAddress()));
                    connected = true;
                }
            }
        }
    }
    if (!retDevice)
    {
        ESP_LOGE(LOG_TAG, "Error could not find device!");
    }
    return retDevice;
}
