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

    static std::vector<HIDDevice> scan(uint32_t seconds);
    static bool connect(std::shared_ptr<HIDDevice> aDevice);

protected:
    BTClassicHID();

    static std::shared_ptr<BTClassicHID> mInstance;
    static std::array<std::shared_ptr<HIDDevice>, MAX_CONNECTED_DEVICES> mConnectedDevices;

private:
    static void init(void *params);
    static void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
    static std::shared_ptr<HIDDevice> getDevice(esp_hidh_event_t anEvent, esp_hidh_event_data_t *aParam);
};