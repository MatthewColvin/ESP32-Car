#include "controller.hpp"

#define LOG_TAG "controller"

#define buttonAMask 0x01
#define buttonBMask 0x02
#define buttonXMask 0x04
#define buttonYMask 0x08
#define triggerMask 0x03

controller::controller(HIDDevice aDevice) : Mocute052(aDevice){};

void controller::handleInputEvent(esp_hidh_event_data_t *anInputEvent)
{
    // HIDDevice::handleInputEvent(anInputEvent);
    if (anInputEvent->input.usage == ESP_HID_USAGE_GAMEPAD)
    {
        uint8_t x = anInputEvent->input.data[1];
        uint8_t y = anInputEvent->input.data[3];
        if (mHandleJoystick && (x != mX || y != mY))
        {
            mX = x;
            mY = y;
            mHandleJoystick(x, y);
        }
        uint8_t ABXYbuttonByte = anInputEvent->input.data[13];
        uint8_t TriggerByte = anInputEvent->input.data[10];

        mIsAdown = handleButton(mIsAdown, buttonAMask, ABXYbuttonByte, mHandleAPress, mHandleARelease);
        mIsBdown = handleButton(mIsBdown, buttonBMask, ABXYbuttonByte, mHandleBPress, mHandleBRelease);
        mIsXdown = handleButton(mIsXdown, buttonXMask, ABXYbuttonByte, mHandleXPress, mHandleXRelease);
        mIsYdown = handleButton(mIsYdown, buttonYMask, ABXYbuttonByte, mHandleYPress, mHandleYRelease);
        mIsTriggerdown = handleButton(mIsTriggerdown, triggerMask, TriggerByte, mHandleTriggerPress, mHandleTriggerRelease);
    }
    else
    {
        ESP_LOGE(LOG_TAG, "Controller In WRONG MODE power on with y + power button!!");
    }
}