#include "IMotorMixingStrategy.hpp"
#include "mocute052.hpp"
#include "motor.hpp"

#include <memory>

/**
 * @brief So = Si * ((r+d)/r)
 *
 * 'So' = Speed of outer track
 * 'Si' = Speed of inner track
 * 'r'  = turn radius from inner track
 * 'd'  = distance between vehicle tracks.
 *
 *
 *              ********* <---------------- Outer Track
 *          ****    |    ****
 *       **         |<--------**----------- 'd' Distance between tracks
 *      *        *******<-------*---------- Inner Track
 *     *      ***   ^   ***      *
 *    *      *      |<-----*------*-------- 'r' Radius of Turn
 *    *     *       |       *     *
 *    *     *       O       *     *
 *    *     *               *     *
 *    *      *             *      *
 *     *      ***       ***      *
 *      *        *******        *
 *       **                   **
 *          ****         ****
 *              *********
 *
 *
 *
 */


class NoZeroTurnMix : public IMotorMixingStrategy
{

public:
    NoZeroTurnMix(std::shared_ptr<Mocute052> aController);

protected:
    virtual speeds mixAndGetSpeeds() override;
};