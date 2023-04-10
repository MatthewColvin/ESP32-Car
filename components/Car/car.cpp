#include "car.hpp"

#include "esp_log.h"

#include <cmath>

#define LOG_TAG "Car"

using namespace std;

Car::Car(std::shared_ptr<Mocute052> remote, std::unique_ptr<Motor> leftMotor, std::unique_ptr<Motor> rightMotor) : mLeftMotor(std::move(leftMotor)), mRightMotor(std::move(rightMotor))
{
    remote->setJoystickHandler(std::bind(&Car::ControllerInputHandler, this, std::placeholders::_1, std::placeholders::_2));
}

Car::controlInputLocation Car::getPointLocation(int x, int y)
{
    // Smooth left and right turns both motors going forward or back throttle based on y value
    bool isDirectionalZone = (y > abs(x) * mMixingZoneUpper) || (y < abs(x) * mMixingZoneUpper * -1);
    // starting to move toward a zero turn type control need to pull power from one motor in this zone rely more on x to determine distributions
    bool isMixingZone = (abs(x) * mMixingZoneUpper > y && y > abs(x) * mMixingZoneLower) ||
                        (abs(x) * mMixingZoneUpper * -1 < y && y < abs(x) * mMixingZoneLower * -1);
    // motors are moving opposite directions
    bool isZeroTurnZone = (0 < y && y < abs(x) * mMixingZoneLower) ||
                          (0 > y && y > abs(x) * mMixingZoneLower * -1);

    // Ensure that we are in a zone and that we did not overlap zones
    // BC + AC + AB + A'B'C'
    bool a = isDirectionalZone;
    bool b = isMixingZone;
    bool c = isZeroTurnZone;
    if ((b && c) || (a && c) || (a && c) || (!a && !b && !c))
    {
        ESP_LOGE(LOG_TAG, "ERROR Multiple zones Detected Resetting upper and lower bounds");
        mMixingZoneUpper = 1;
        mMixingZoneLower = 0.25;
        return getPointLocation(x, y);
    }
    if (isDirectionalZone)
    {
        return Car::controlInputLocation::Directional;
    }
    if (isMixingZone)
    {
        return Car::controlInputLocation::Mixing;
    }
    if (isZeroTurnZone)
    {
        return Car::controlInputLocation::ZeroTurn;
    }
    return Car::controlInputLocation::Error;
}

void Car::ControllerInputHandler(uint8_t x, uint8_t y)
{
    int refX = x - 128;
    int refY = -1 * (y - 128);
    // convert xy to be like 4 quadrant grid while holding joystick in portrait.
    // 0,0 is position with no input

    //  Use pathergy theorem to find distance from center and determine speed
    float totalSpeed = std::sqrt(refX * refX + refY * refY);
    float rightMotorSpeed = 0;
    float leftMotorSpeed = 0;
    switch (getPointLocation(refX, refY))
    {
    case Car::controlInputLocation::Directional:
    {
        float xBias = refX / 128.0;
        if (xBias > 0)
        { // slight right turn
            leftMotorSpeed = xBias * totalSpeed;
            rightMotorSpeed = totalSpeed - leftMotorSpeed;
        }
        else if (xBias < 0)
        { // slight left turn
            rightMotorSpeed = xBias * totalSpeed;
            leftMotorSpeed = totalSpeed - rightMotorSpeed;
        }
        else
        { // no bias straight forward or back
            rightMotorSpeed = totalSpeed * .5;
            leftMotorSpeed = totalSpeed * .5;
        }
        ESP_LOGI(LOG_TAG, "Directional Zone");
    }
    break;
    case Car::controlInputLocation::Mixing:
    {
        ESP_LOGI(LOG_TAG, "Mixing Zone");
        float upperBoundY = mMixingZoneUpper * refX;
        float lowerBoundY = mMixingZoneLower * refX;
        float yLenghtInMixingZone = upperBoundY - lowerBoundY;
        float percentToZeroTurnZone = refY / yLenghtInMixingZone;
    }
    break;
    case Car::controlInputLocation::ZeroTurn:
    {
        ESP_LOGI(LOG_TAG, "Zero turn Zone");
    }
    break;
    default:
    {
    }
    }
    // right.setSpeed(rightMotorSpeed);
    // left.setSpeed(leftMotorSpeed);

    ESP_LOGI(LOG_TAG, "refx:%d,refy:%d Total Speed:%f RightSpeed: %f, LeftSpeed: %f", refX, refY, totalSpeed, rightMotorSpeed, leftMotorSpeed);
}
