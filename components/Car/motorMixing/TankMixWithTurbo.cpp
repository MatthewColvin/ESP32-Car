#include "TankMixWithTurbo.hpp"
#include "motor.hpp"

TankMixWithTurbo::TankMixWithTurbo(std::shared_ptr<Mocute052> aController):IMotorMixingStrategy(aController)
{
    aController->onTrigger(std::bind(&TankMixWithTurbo::enableTurbo,this),std::bind(&TankMixWithTurbo::disableTurbo,this));
    aController->onX(std::bind(&TankMixWithTurbo::speedUpCoast,this),nullptr);
    aController->onB(std::bind(&TankMixWithTurbo::slowCoast,this),nullptr);
}

IMotorMixingStrategy::motorSpeeds TankMixWithTurbo::getMotorSpeeds()
{
    IMotorMixingStrategy::motorSpeeds returnSpeeds;

    float algX = -1 * mX;
    float algY = mY;

    float V = (Mocute052::MAX_XY - std::abs(algX)) * (algY/Mocute052::MAX_XY) + algY;
    float W = (Mocute052::MAX_XY - std::abs(algY)) * (algX/Mocute052::MAX_XY) + algX;

    float leftSpeed = (V-W)/2;
    float rightSpeed = (V+W)/2;

    float currentFastestSpeed = mIsTurboEnabled ? mMaxSpeed : mCoastSpeed;

    // Convert speed from controller based units to motor based units unless user input is (0,0)
    if(leftSpeed > 0){
        returnSpeeds.left = mapValues(leftSpeed,0,Mocute052::MAX_XY,0,currentFastestSpeed);
    }
    else{
        returnSpeeds.left = mapValues(leftSpeed,Mocute052::MIN_XY,0,-1 * currentFastestSpeed,0);
    }
    if(rightSpeed > 0){
        returnSpeeds.right = mapValues(rightSpeed,0,Mocute052::MAX_XY,0,currentFastestSpeed);
    }else{
        returnSpeeds.right = mapValues(rightSpeed,Mocute052::MIN_XY,0,-1 * currentFastestSpeed,0);
    }

    return returnSpeeds;
}

void TankMixWithTurbo::speedUpCoast(){
    if(mCoastSpeed >= Motor::MAX_SPEED){
        return;
    }
    mCoastSpeed += 1000;
}

void TankMixWithTurbo::slowCoast(){
    if(mCoastSpeed <= 5000){
        return;
    }
    mCoastSpeed -= 1000;
}
