#include "car.hpp"

#include "esp_log.h"

#include <cmath>

#define LOG_TAG "Car"

using namespace std;

namespace
{
    float mapValues(float value,float aMin, float aMax, float aTargetMin, float aTargetMax)
    {
        float span = aMax - aMin;
        float targetSpan = aTargetMax - aTargetMin;
        float scaled = (value - aMin) / span;
        return aTargetMin + (scaled * targetSpan);
    }
}

Car::Car(std::shared_ptr<Mocute052> remote, std::unique_ptr<Motor> leftMotor, std::unique_ptr<Motor> rightMotor) : mRightMotor(std::move(rightMotor)),mLeftMotor(std::move(leftMotor))
{
    remote->onJoyStick(std::bind(&Car::ControllerInputHandler, this, std::placeholders::_1, std::placeholders::_2));
    remote->onTrigger(std::bind(&Car::enableTurbo,this),std::bind(&Car::disableTurbo,this));
    remote->onX(std::bind(&Car::speedUpCoast,this),nullptr);
    remote->onB(std::bind(&Car::slowCoast,this),nullptr);
}

void Car::speedUpCoast(){
    if(mCoastSpeed >= Motor::MAX_SPEED){
        return;
    }
    mCoastSpeed += 1000;
}

void Car::slowCoast(){
    if(mCoastSpeed <= 5000){
        return;
    }
    mCoastSpeed -= 1000;
}

void Car::ControllerInputHandler(uint8_t x, uint8_t y)
{
    int refX = x - Mocute052::MAX_XY;
    int refY = -1 * (y - 128);

    float algX = -1 * refX;
    float algY = refY;

    float V = (Mocute052::MAX_XY - std::abs(algX)) * (algY/Mocute052::MAX_XY) + algY;
    float W = (Mocute052::MAX_XY - std::abs(algY)) * (algX/Mocute052::MAX_XY) + algX;

    mRightMotorSpeed = (V+W)/2;
    mLeftMotorSpeed = (V-W)/2;

    ConvertAndUpdateSpeed();
    //ESP_LOGI(LOG_TAG, "refx:%d,refy:%d RightSpeed: %f, LeftSpeed: %f", refX, refY, mRightMotorSpeed, mLeftMotorSpeed);
}

void Car::ConvertAndUpdateSpeed(){
    float leftMotorConvertedSpeed = 0;
    float rightMotorConvertedSpeed = 0;
    float currentFastestSpeed = mIsTurboEnabled ? mMaxSpeed : mCoastSpeed;

    // Convert speed from controller based units to motor based units unless user input is (0,0)
    if(mLeftMotorSpeed > 0){
        leftMotorConvertedSpeed = mapValues(mLeftMotorSpeed,0,Mocute052::MAX_XY,0,currentFastestSpeed);
    }
    else{
        leftMotorConvertedSpeed = mapValues(mLeftMotorSpeed,Mocute052::MIN_XY,0,-1 * currentFastestSpeed,0);
    }
    if(mRightMotorSpeed > 0){
        rightMotorConvertedSpeed = mapValues(mRightMotorSpeed,0,Mocute052::MAX_XY,0,currentFastestSpeed);
    }else{
        rightMotorConvertedSpeed = mapValues(mRightMotorSpeed,Mocute052::MIN_XY,0,-1 * currentFastestSpeed,0);
    }
    setMotorSpeed(leftMotorConvertedSpeed, rightMotorConvertedSpeed);
}


void Car::setMotorSpeed(float aLeftMotorSpeed, float aRightMotorSpeed)
{
    if (aLeftMotorSpeed > 0)
    {
        mLeftMotor->forward();
    }
    else
    {
        mLeftMotor->reverse();
    }
    mLeftMotor->setSpeed(std::floor(std::abs(aLeftMotorSpeed)));

    if (aRightMotorSpeed > 0)
    {
        mRightMotor->forward();
    }
    else
    {
        mRightMotor->reverse();
    }
    mRightMotor->setSpeed(std::floor(std::abs(aRightMotorSpeed)));
}



