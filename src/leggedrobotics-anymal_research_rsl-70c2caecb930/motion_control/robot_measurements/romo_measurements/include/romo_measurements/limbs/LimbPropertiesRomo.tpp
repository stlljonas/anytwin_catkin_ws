/*!
 * @file	 LimbPropertiesRomo.tpp
 * @author Gabriel Hottiger
 * @date	 Nov, 2017
 */

// romo_measurements
#include "romo_measurements/limbs/LimbPropertiesRomo.hpp"

// message_logger
#include "message_logger/message_logger.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_, typename LimbPropertiesBase_>
LimbPropertiesRomo<ConcreteDescription_,RobotState_,LimbPropertiesBase_>::LimbPropertiesRomo(const LimbEnum limbEnum,
                                                                                             const RobotModel& model)
  : LimbPropertiesBase_(),
    model_(model),
    limbEnum_(limbEnum),
    branchEnum_(RD::template mapEnums<BranchEnum>(limbEnum))
{

}

template <typename ConcreteDescription_, typename RobotState_, typename LimbPropertiesBase_>
bool LimbPropertiesRomo<ConcreteDescription_,RobotState_,LimbPropertiesBase_>::initialize(double dt) {
  this->setLimbMass(model_.getLimbMass(branchEnum_));
  return this->advance(dt);
}

template <typename ConcreteDescription_, typename RobotState_, typename LimbPropertiesBase_>
bool LimbPropertiesRomo<ConcreteDescription_,RobotState_,LimbPropertiesBase_>::advance(double /* dt */ ) {
  this->setPositionBaseToLimbComInBaseFrame( loco::Position(model_.getPositionBaseToLimbComInBaseFrame(branchEnum_)) );
  return true;
}

template <typename ConcreteDescription_, typename RobotState_, typename LimbPropertiesBase_>
double LimbPropertiesRomo<ConcreteDescription_,RobotState_,LimbPropertiesBase_>::getMaximumLimbExtension() const {
  MELO_WARN_THROTTLE(1.0, "[LimbPropertiesRomo] The maximum limb extension is not calculated. Set to -1 [m].");
  return -1.0;
}

template <typename ConcreteDescription_, typename RobotState_, typename LimbPropertiesBase_>
double LimbPropertiesRomo<ConcreteDescription_,RobotState_,LimbPropertiesBase_>::getMinimumLimbExtension() const {
  MELO_WARN_THROTTLE(1.0, "[LimbPropertiesRomo] The minimum limb extension is not calculated. Set to -1 [m].");
  return -1.0;
}

} // namespace romo_measurements
