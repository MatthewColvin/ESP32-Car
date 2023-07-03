#pragma once

#include "mocute052.hpp"

class controller : public Mocute052
{
public:
    controller(HIDDevice aDevice);

protected:
    virtual void handleInputEvent(esp_hidh_event_data_t *anInputEvent) override;
};