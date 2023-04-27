#include "IMotorMixingStrategy.hpp"
#include "mocute052.hpp"
#include "motor.hpp"

#include <memory>

class TankMix : public IMotorMixingStrategy
{

public:
    TankMix(std::shared_ptr<Mocute052> aController);

protected:
    virtual speeds mixAndGetSpeeds() override;
};