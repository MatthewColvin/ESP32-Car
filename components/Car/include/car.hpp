#pragma once

#include "motor.hpp"
#include "mocute052.hpp"
#include <memory>

class Car
{
public:
    Car(std::shared_ptr<Mocute052> remote, std::unique_ptr<Motor> leftMotor, std::unique_ptr<Motor> rightMotor);

private:
    static constexpr int maxControlSpeed = 151;

    void ControllerInputHandler(uint8_t x, uint8_t y);

    enum class controlInputLocation
    {
        Directional,
        Mixing,
        ZeroTurn,
        Error
    };

    controlInputLocation getPointLocation(int x, int y);

    std::unique_ptr<Motor> mLeftMotor;
    std::unique_ptr<Motor> mRightMotor;
    void setMotorSpeed(float aLeftMotorSpeed, float aRightMotorSpeed);

    /**
     * @brief Motor controls handed differently depending on the location of the x,y coordinate
     *        y = m|x| + b lines
     *        x and y represented by the joystick inputs where as m is represented by mixingZones
     *  Below is an ascii art graph of y = |x| to get an idea of the control zones but there are
     *  Actually 3 zone
     *      The upperZone - Where the controller will expect the car to run mostly straight
     *      The mixing Zone - The controller will need to adjust the inside motor speed to near zero
     *      The Zero turn Zone - below the mixing zone we will need to begin to run one motor backward and the other forward.
     *
     *          \    |    /
     *           \   |   /
     *            \  |  /
     *             \ | /
     *    ---------  | -----------
     *  _____________|_______________
     *
     */
    float mMixingZoneUpper = 0.75;
    float mMixingZoneLower = 0.25;
};