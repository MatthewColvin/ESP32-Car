#include "IMotorMixingStrategy.hpp"
#include "mocute052.hpp"
#include "motor.hpp"

#include <memory>

class TankMixWithTurbo : public IMotorMixingStrategy{

    public:
    TankMixWithTurbo(std::shared_ptr<Mocute052> aController);
    virtual motorSpeeds getMotorSpeeds() override;

    private:
    /**
     * @brief Max Speed of any motor stored in Motor ranges
    */
    float mMaxSpeed = Motor::MAX_SPEED;
    void enableTurbo(){mIsTurboEnabled = true;};
    void disableTurbo(){mIsTurboEnabled = false;};
    bool mIsTurboEnabled = false;


    /**
     * @brief coasting speed and adjusters
     */
    float mCoastSpeed = Motor::MAX_SPEED * 0.50;
    void slowCoast();
    void speedUpCoast();


};