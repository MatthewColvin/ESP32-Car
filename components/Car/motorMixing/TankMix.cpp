#include "TankMix.hpp"
#include "motor.hpp"

TankMix::TankMix(std::shared_ptr<Mocute052> aController) : IMotorMixingStrategy(aController)
{
}

IMotorMixingStrategy::speeds TankMix::mixAndGetSpeeds()
{
    IMotorMixingStrategy::speeds returnSpeeds;

    float algX = -1 * mX;
    float algY = mY;

    float V = (Mocute052::MAX_XY - std::abs(algX)) * (algY / Mocute052::MAX_XY) + algY;
    float W = (Mocute052::MAX_XY - std::abs(algY)) * (algX / Mocute052::MAX_XY) + algX;

    float leftSpeed = (V - W) / 2;
    float rightSpeed = (V + W) / 2;

    returnSpeeds.left = leftSpeed;
    returnSpeeds.right = rightSpeed;

    return returnSpeeds;
}
