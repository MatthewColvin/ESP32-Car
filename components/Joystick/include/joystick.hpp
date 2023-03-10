#pragma once

#include "device.hpp"

class Joystick : public Device{

    public:

    Joystick(Device::bleScanResult aScanResult);

    void registerCharacteristics();

};