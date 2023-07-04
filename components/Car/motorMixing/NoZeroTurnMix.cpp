#include "NoZeroTurnMix.hpp"
#include "motor.hpp"
#include <cmath>

NoZeroTurnMix::NoZeroTurnMix(std::shared_ptr<Mocute052> aController) : IMotorMixingStrategy(aController)
{
}

float distFromOrigin(float x,float y){
    return std::sqrt((x*x) + (y*y));
}

float slopeWithOrigin(float x, float y){
    if (x!=0){
        return y/x;
    }
    return 0;
}

float mappedDistFromOrigin(float x, float y){
    float R = Mocute052::MAX_XY * sqrt(2);
    float row = distFromOrigin(x,y);

    float rowSquareM = slopeWithOrigin(x,y);
    float rowSquare = 0;
    if (rowSquareM < -1 || rowSquareM > 1){
    // Calculate rowSquare by getting x value that passes through y = Mocute052::MAX_XY
        float rowSquareX = Mocute052::MAX_XY/rowSquareM;
        rowSquare = distFromOrigin(rowSquareX,Mocute052::MAX_XY);
    }else{
    // Calculate rowSquare by getting y value that passthrough x = Mocute052::MAX_XY;
        float rowSquareY = rowSquareM * Mocute052::MAX_XY;
        rowSquare = distFromOrigin(Mocute052::MAX_XY,rowSquareY);
    }

    float mappedSpeed = rowSquare == 0 ? 0 : (R/rowSquare) * row;
    return mappedSpeed;
}

IMotorMixingStrategy::speeds NoZeroTurnMix::mixAndGetSpeeds()
{
    IMotorMixingStrategy::speeds returnSpeeds;

    // All speeds generated based on dist from origin.
    float outerSpeed = mappedDistFromOrigin(mX,mY);
    float distanceBetweenTracks = 14; // Guessing 10 cm between tracks.
    float turnRadius = abs(slopeWithOrigin(mX,mY)) * 60; // slope of 1 make 120cm circle
    float innerSpeed = turnRadius < 0 ? 0 : outerSpeed / ((turnRadius + distanceBetweenTracks)/ turnRadius);

    constexpr float maxDistFromOrigin = Mocute052::MAX_XY * sqrt(2);

    float mappedInnerSpeed = mapValues(innerSpeed,0,maxDistFromOrigin,0,Mocute052::MAX_XY);
    float mappedOuterSpeed = mapValues(outerSpeed,0,maxDistFromOrigin,0,Mocute052::MAX_XY);

    if (mX > 0){// Right turn
        returnSpeeds.right = mappedInnerSpeed;
        returnSpeeds.left = mappedOuterSpeed;
    }else if (mX < 0){// Left turn
        returnSpeeds.left = mappedInnerSpeed;
        returnSpeeds.right = mappedOuterSpeed;
    }else {
        returnSpeeds.left = mappedOuterSpeed;
        returnSpeeds.right = mappedOuterSpeed;
    }

    if (mY < 0) {// backwards
        returnSpeeds.left = returnSpeeds.left * -1;
        returnSpeeds.right = returnSpeeds.right * -1;
    }

    //ESP_LOGI("NoZeroTurn","IN(x:%f y:%f)  Out(left:%f right:%f)", mX,mY,returnSpeeds.left,returnSpeeds.right);
    return returnSpeeds;
}
