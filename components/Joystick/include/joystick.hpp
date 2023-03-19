#pragma once

#include "device.hpp"

class Joystick : public Device
{
public:
    Joystick(Device::bleScanResult aScanResult);

    void init();

    void registerReportNotifications();
    void nextReports(int start);

private:
    std::vector<Characteristic> mHIDReports;
};