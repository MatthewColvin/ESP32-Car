#pragma once
#include "HIDDevice.hpp"

#include "esp_hidh.h"
#include "esp_hid_gap.h"

#include <memory>
#include <vector>

class BTClassicHID
{
public:
    BTClassicHID(BTClassicHID &other) = delete;
    void operator=(const BTClassicHID &) = delete;
    static std::shared_ptr<BTClassicHID> getInstance();
    static void init(void *params);

    std::vector<HIDDevice> scan(uint32_t seconds);

protected:
    BTClassicHID();

    static std::shared_ptr<BTClassicHID> mInstance;

private:
    static void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
};