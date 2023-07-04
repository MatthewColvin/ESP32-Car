#include "IMotorMixingStrategy.hpp"
#include "mocute052.hpp"
#include "motor.hpp"

#include <memory>

class TankMix : public IMotorMixingStrategy
{

public:
    /// @brief A Motor mixing strategy that allows the car to make 180 degree turns
    /// @param aController - reference to the controller so it can set get the joystick inputs
    TankMix(std::shared_ptr<Mocute052> aController);

protected:
    /// @brief Using Base Abstract Class to get the current speed in terms the the controller
    /// @return Current speed to set left and right motor speeds
    virtual speeds mixAndGetSpeeds() override;
};