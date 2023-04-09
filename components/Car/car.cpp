#include "car.hpp"

#include <cmath>

#define LOG_TAG "Car"

using namespace std;

Car::Car(std::shared_ptr<Mocute052> remote, std::unique_ptr<Motor> leftMotor, std::unique_ptr<Motor> rightMotor) : mLeftMotor(std::move(leftMotor)), mRightMotor(std::move(rightMotor))
{
    remote->setJoystickHandler(std::bind(&Car::ControllerInputHandler, this, std::placeholders::_1, std::placeholders::_2));
}

void Car::ControllerInputHandler(uint8_t x, uint8_t y)
{
    float refX = x - 128;
    float refY = -1 * (y - 128);
    // convert xy to be like 4 quadrant grid while holding joystick in portrait.

    //  Use pathergy theorem to find distance from center and determine speed. 0 - 128
    //  Both motors will distribute this based on x bias

    constexpr float MixingZoneUpperLimit = 1;
    constexpr float MixingZoneLowerLimit = .25;
    // Smooth left and right turns both motors going forward or back throttle based on y value
    bool isDirectionalZone = (refY > abs(refX) * MixingZoneUpperLimit) || (refY < abs(refX) * MixingZoneUpperLimit * -1);
    // starting to move toward a zero turn type control need to pull power from one motor in this zone rely more on x to determine distributions
    bool isMixingZone = (abs(refX) * MixingZoneUpperLimit > refY && refY > abs(refX) * MixingZoneLowerLimit) ||
                        (abs(refX) * MixingZoneUpperLimit * -1 < refY && refY < abs(refX) * MixingZoneLowerLimit * -1);
    // motors are moving opposite directions
    bool isZeroTurnZone = (0 < refY && refY < abs(refX) * MixingZoneLowerLimit) ||
                          (0 > refY && refY > abs(refX) * MixingZoneLowerLimit * -1);

    float totalSpeed = std::sqrt(refX * refX + refY * refY);
    float rightMotorSpeed = 0;
    float leftMotorSpeed = 0;
    float xBias = refX / 128.0;
    if (isDirectionalZone)
    {
        if (xBias > 0)
        { // right turn
            leftMotorSpeed = xBias * totalSpeed;
            rightMotorSpeed = totalSpeed - leftMotorSpeed;
        }
        else if (xBias < 0)
        { // left turn
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
    if (isMixingZone)
    {
        ESP_LOGI(LOG_TAG, "Mixing Zone");
    }
    if (isZeroTurnZone)
    {
        ESP_LOGI(LOG_TAG, "Zero turn Zone");
    }
    // right.setSpeed(rightMotorSpeed);
    // left.setSpeed(leftMotorSpeed);

    ESP_LOGI(LOG_TAG, "refx:%f,refy:%f xbias: %f Total Speed:%f RightSpeed: %f, LeftSpeed: %f", refX, refY, xBias, totalSpeed, rightMotorSpeed, leftMotorSpeed);
}
