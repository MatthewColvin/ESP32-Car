#pragma once
#include "HIDDevice.hpp"
#include <functional>

class Mocute052 : public HIDDevice
{
public:
    /// @brief max value of controller used externally
    static constexpr float MAX_XY = 127;
    /// @brief min value of controller used externally
    static constexpr float MIN_XY = -128;

    /// @brief Create an instance of Mocute052 controller to manage its button presses and joystick
    /// @param aDevice - A Device found using the BTClassicHID class to scan for devices
    Mocute052(HIDDevice aDevice);

    /// @brief Set Function to be ran when joystick events change
    /// @param aJoyStickHandler - handler for joystick events
    void onJoyStick(std::function<void(uint8_t, uint8_t)> aJoyStickHandler) { mHandleJoystick = std::move(aJoyStickHandler); };
    /// @brief Set Functions to be ran for the A button
    /// @param aPressHandler - Function to run when A is pressed
    /// @param aReleaseHandler - Function to run when A is released
    void onA(std::function<void()> aPressHandler, std::function<void()> aReleaseHandler)
    {
        mHandleAPress = std::move(aPressHandler);
        mHandleARelease = std::move(aReleaseHandler);
    };
    /// @brief Set Functions to be ran for the B button
    /// @param aPressHandler - Function to run when B is pressed
    /// @param aReleaseHandler - Function to run when B is released
    void onB(std::function<void()> aPressHandler, std::function<void()> aReleaseHandler)
    {
        mHandleBPress = std::move(aPressHandler);
        mHandleBRelease = std::move(aReleaseHandler);
    };
    /// @brief Set Functions to be ran for the X button
    /// @param aPressHandler - Function to run when X is pressed
    /// @param aReleaseHandler - Function to run when X is released
    void onX(std::function<void()> aPressHandler, std::function<void()> aReleaseHandler)
    {
        mHandleXPress = std::move(aPressHandler);
        mHandleXRelease = std::move(aReleaseHandler);
    };
    /// @brief Set Functions to be ran for the Y button
    /// @param aPressHandler - Function to run when Y is pressed
    /// @param aReleaseHandler - Function to run when Y is released
    void onY(std::function<void()> aPressHandler, std::function<void()> aReleaseHandler)
    {
        mHandleYPress = std::move(aPressHandler);
        mHandleYRelease = std::move(aReleaseHandler);
    };
    /// @brief Set Functions to be ran for the Trigger button
    /// @param aPressHandler - Function to run when Trigger is pressed
    /// @param aReleaseHandler - Function to run when Trigger is released
    void onTrigger(std::function<void()> aPressHandler, std::function<void()> aReleaseHandler)
    {
        mHandleTriggerPress = std::move(aPressHandler);
        mHandleTriggerRelease = std::move(aReleaseHandler);
    };

protected:
    /// @brief Overriden from base device to handle bluetooth HID device events
    /// @param anInputEvent - input event from BT library
    virtual void handleInputEvent(esp_hidh_event_data_t *anInputEvent) override;
    bool handleButton(bool oldIsDown, int buttonMask, uint8_t buttonByte, std::function<void()> pressCb, std::function<void()> releaseCb);

    /// @brief Current Controller X (0 - 255)
    uint8_t mX = 128;
    /// @brief Current Controller Y (0 - 255)
    uint8_t mY = 128;
    /// @brief Callable to be ran when joystick value changes
    std::function<void(uint8_t, uint8_t)> mHandleJoystick;

    /// @brief State of A button
    bool mIsAdown = false;
    /// @brief Function to run when A is Pressed
    std::function<void()> mHandleAPress;
    /// @brief Function to run when A is Released
    std::function<void()> mHandleARelease;

    /// @brief State of B button
    bool mIsBdown = false;
    /// @brief Function to run when B is Pressed
    std::function<void()> mHandleBPress;
    /// @brief Function to run when B is Released
    std::function<void()> mHandleBRelease;

    /// @brief State of X button
    bool mIsXdown = false;
    /// @brief Function to run when X is Pressed
    std::function<void()> mHandleXPress;
    /// @brief Function to run when X is Released
    std::function<void()> mHandleXRelease;

    /// @brief State of Y button
    bool mIsYdown = false;
    /// @brief Function to run when Y is Pressed
    std::function<void()> mHandleYPress;
    /// @brief Function to run when Y is Released
    std::function<void()> mHandleYRelease;

    /// @brief State of the Trigger
    bool mIsTriggerdown = false;
    /// @brief Function to run when Trigger is Pressed
    std::function<void()> mHandleTriggerPress;
    /// @brief Function to run when Trigger is Released
    std::function<void()> mHandleTriggerRelease;
};
