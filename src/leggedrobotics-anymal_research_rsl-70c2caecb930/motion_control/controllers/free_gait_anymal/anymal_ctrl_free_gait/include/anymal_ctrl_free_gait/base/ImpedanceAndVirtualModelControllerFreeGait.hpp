/*
 * ImpedanceAndVirtualModelControllerFreeGait.hpp
 *
 *  Created on: Feb 5, 2018
 *      Author: Dario Bellicoso
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

// loco anymal
#include <loco_anymal/motion_control/ImpedanceAndVirtualModelController.hpp>

namespace loco {

class ImpedanceAndVirtualModelControllerFreeGait : public ImpedanceAndVirtualModelController {
 public:
  using AD = anymal_description::AnymalDescription;

  // Import constructors.
  using ImpedanceAndVirtualModelController::ImpedanceAndVirtualModelController;

  // Define default destructor.
  ~ImpedanceAndVirtualModelControllerFreeGait() override = default;

 protected:
  bool setControlModeForLimbs() override;
};

} /* namespace loco */
