#pragma once

#include "device.hpp"

class Joystick : public Device{

    Joystick(Device::bleScanResult aScanResult);

    void registerCharacteristics();

};