#pragma once
#include "mocute052.hpp"
#include <memory>
#include <functional>

class IMotorMixingStrategy{
    public:

    /**
     * @brief This should be called by any child classes so that way
     *        mX and mY are updated by the Mocute052 controller.
     *
     * @param aController - mocute 052 controller
     */
    IMotorMixingStrategy(std::shared_ptr<Mocute052> aController){
        aController->onJoyStick(std::bind(&IMotorMixingStrategy::controllerInputHandler,this,std::placeholders::_1,std::placeholders::_2));
    };

    struct motorSpeeds{
        float left;
        float right;
    };

    /**
     * @brief Main algorithm of strategy pattern to implement.
     *        use the given mX and mY values to create a good
     *        motorSpeeds struct.
     *
     * @return motorSpeeds
     */
    virtual motorSpeeds getMotorSpeeds() = 0;

    private:
    /**
     * @brief Set the given input from the controller into protected
     *        members
     *
     * @param aX - X value between -128 and 127
     * @param aY - Y value between -128 and 127
     */
    void controllerInputHandler(uint8_t aX, uint8_t aY) {
        // convert uint8 given by controller to floats and
        // correct ranges so 0,0 is the middle of the control range
        int refX = aX - Mocute052::MAX_XY;
        int refY = -1 * (aY - 128);

        mX = refX;
        mY = refY;
    };


    protected:

    /**
     * @brief Map value from range (aMin-aMax) into (aTargetMin-aTargetMax)
     */
    float mapValues(float value,float aMin, float aMax, float aTargetMin, float aTargetMax)
    {
        float span = aMax - aMin;
        float targetSpan = aTargetMax - aTargetMin;
        float scaled = (value - aMin) / span;
        return aTargetMin + (scaled * targetSpan);
    }

    // Current Control input from the controller in (-128 to 127 )
    float mX = 0;
    float mY = 0;

};