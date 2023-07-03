#include "mocute052.hpp"

#define LOG_TAG "mocute052"

#define buttonAMask 0x08
#define buttonBMask 0x02
#define buttonXMask 0x40
#define buttonYMask 0x80
#define triggerMask 0x10

Mocute052::Mocute052(HIDDevice aDevice) : HIDDevice(aDevice.getScanResult())
{
    mHandleAPress = []()
    { ESP_LOGI(LOG_TAG, "APressed"); };

    mHandleARelease = []()
    { ESP_LOGI(LOG_TAG, "AReleased"); };

    mHandleJoystick = [](uint8_t x, uint8_t y)
    {
        ESP_LOGI(LOG_TAG, "X:%d Y:%d", x, y);
    };
};

bool Mocute052::handleButton(bool oldIsDown, int buttonMask, uint8_t buttonByte, std::function<void()> pressCb, std::function<void()> releaseCb)
{
    bool aIsDown = buttonMask & buttonByte;
    if (aIsDown != oldIsDown) // button was either pressed or released
    {
        if (aIsDown)
        {
            if (pressCb)
            {
                pressCb();
            }
        }
        else
        {
            if (releaseCb)
            {
                releaseCb();
            }
        }
    }
    return aIsDown;
}

void Mocute052::handleInputEvent(esp_hidh_event_data_t *anInputEvent)
{
    // HIDDevice::handleInputEvent(anInputEvent);
    if (anInputEvent->input.usage == ESP_HID_USAGE_GAMEPAD)
    {
        uint8_t x = anInputEvent->input.data[0];
        uint8_t y = anInputEvent->input.data[1];
        if (mHandleJoystick && (x != mX || y != mY))
        {
            mX = x;
            mY = y;
            mHandleJoystick(x, y);
        }

        uint8_t ABbuttonByte = anInputEvent->input.data[5];
        uint8_t XYbuttonByte = anInputEvent->input.data[4];
        uint8_t TriggerByte = XYbuttonByte;

        mIsAdown = handleButton(mIsAdown, buttonAMask, ABbuttonByte, mHandleAPress, mHandleARelease);
        mIsBdown = handleButton(mIsBdown, buttonBMask, ABbuttonByte, mHandleBPress, mHandleBRelease);
        mIsXdown = handleButton(mIsXdown, buttonXMask, XYbuttonByte, mHandleXPress, mHandleXRelease);
        mIsYdown = handleButton(mIsYdown, buttonYMask, XYbuttonByte, mHandleYPress, mHandleYRelease);
        mIsTriggerdown = handleButton(mIsTriggerdown, triggerMask, TriggerByte, mHandleTriggerPress, mHandleTriggerRelease);

        // ESP_LOGI(LOG_TAG, "X:%d Y:%d", x, y);
        // ESP_LOG_BUFFER_HEX(LOG_TAG, anInputEvent->input.data, anInputEvent->input.length);
    }
}