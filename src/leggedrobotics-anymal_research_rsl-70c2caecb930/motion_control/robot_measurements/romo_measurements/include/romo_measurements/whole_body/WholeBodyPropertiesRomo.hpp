/*!
 * @file    WholeBodyPropertiesRomo.hpp
 * @author  Gabriel Hottiger
 * @date	  Nov, 2017
 */

#pragma once

// loco
#include "loco/common/WholeBodyProperties.hpp"

// romo
#include "romo/RobotModel.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_>
class WholeBodyPropertiesRomo : public loco::WholeBodyProperties {

 protected:
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = typename RobotModel::RD;

 public:
  explicit WholeBodyPropertiesRomo(const RobotModel & model)
    : loco::WholeBodyProperties(),
      model_(model)
  {

  }

  ~WholeBodyPropertiesRomo() override =  default;

  bool initialize(double /* dt */) override {
    this->setTotalMass(model_.getTotalMass());
    return true;
  }

  bool advance(double /* dt */) override {
    return true;
  }

 protected:
  const RobotModel & model_;
};

} /* namespace romo_measurements */
