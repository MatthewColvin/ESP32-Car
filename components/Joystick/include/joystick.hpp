#pragma once

#include "device.hpp"

class Joystick : public Device
{
public:
    Joystick(Device::bleScanResult aScanResult);
    ~Joystick() = default;

    void init();

    void cycleReports();
    void registerReportNotifications();
    void nextReports(int start);

    void readHIDReport();
    void readReportsDescriptors();

protected:
    int handleJoystickReport(Device::characteristicCbParamType aParam);

private:
    std::vector<Characteristic> mHIDReports;
};