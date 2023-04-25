#pragma once
#include "HIDDevice.hpp"
#include <functional>

class Mocute052 : public HIDDevice
{
public:
    static constexpr float MAX_XY = 128;
    static constexpr float MIN_XY = -128;

    Mocute052(HIDDevice aDevice);

    void setJoystickHandler(std::function<void(uint8_t, uint8_t)> aJoyStickHandler) { mHandleJoystick = std::move(aJoyStickHandler); };


protected:
    virtual void handleInputEvent(esp_hidh_event_data_t *anInputEvent);

public:
    uint8_t mX = 128;
    uint8_t mY = 128;
    std::function<void(uint8_t, uint8_t)> mHandleJoystick;

    bool mIsAdown = false;
    std::function<void()> mHandleAPress;
    std::function<void()> mHandleARelease;

    bool mIsBdown = false;
    std::function<void()> mHandleBPress;
    std::function<void()> mHandleBRelease;

    bool mIsXdown = false;
    std::function<void()> mHandleXPress;
    std::function<void()> mHandleXRelease;

    bool mIsYdown = false;
    std::function<void()> mHandleYPress;
    std::function<void()> mHandleYRelease;

    bool mIsTriggerdown = false;
    std::function<void()> mHandleTriggerPress;
    std::function<void()> mHandleTriggerRelease;

};
