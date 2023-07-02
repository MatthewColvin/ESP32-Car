#include "controller.hpp"

controller::controller(HIDDevice aDevice) : Mocute052(aDevice){};

void controller::handleInputEvent(esp_hidh_event_data_t *anInputEvent)
{
    HIDDevice::handleInputEvent(anInputEvent);
}