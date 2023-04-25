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

Car::Car(std::shared_ptr<Mocute052> remote, std::unique_ptr<Motor> leftMotor, std::unique_ptr<Motor> rightMotor) : mLeftMotor(std::move(leftMotor)), mRightMotor(std::move(rightMotor))
{
    remote->onJoyStick(std::bind(&Car::ControllerInputHandler, this, std::placeholders::_1, std::placeholders::_2));
    remote->onTrigger(std::bind(&Car::enableTurbo,this),std::bind(&Car::disableTurbo,this));
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
}

void Car::ConvertAndUpdateSpeed(){
    float leftMotorConvertedSpeed = 0;
    float rightMotorConvertedSpeed = 0;
    // Convert speed from controller based units to motor based units unless user input is (0,0)
    if(refX != 0 || refY != 0){
        if(mLeftMotorSpeed > 0){
            leftMotorConvertedSpeed = mapValues(mLeftMotorSpeed,0,Mocute052::MAX_XY,0,mMaxSpeed);
        }
        else{
            leftMotorConvertedSpeed = mapValues(mLeftMotorSpeed,Mocute052::MIN_XY,0,-1 * mMaxSpeed,0);
        }
        if(rightMotorSpeed > 0){
            rightMotorConvertedSpeed = mapValues(mRightMotorSpeed,0,Mocute052::MAX_XY,0,mMaxSpeed);
        }else{
            rightMotorConvertedSpeed = mapValues(mRightMotorSpeed,Mocute052::MIN_XY,0,-1 * mMaxSpeed,0);
        }
    }
    setMotorSpeed(leftMotorConvertedSpeed, rightMotorConvertedSpeed);
    ESP_LOGI(LOG_TAG, "refx:%d,refy:%d RightSpeed: %f, LeftSpeed: %f", refX, refY, rightMotorConvertedSpeed, leftMotorConvertedSpeed);
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



