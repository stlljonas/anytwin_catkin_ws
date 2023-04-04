//
// Created by dbellicoso on 22/02/18.
//

#pragma once

#include "loco/common/ModuleBase.hpp"
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorBase.hpp"

namespace loco {

class SwingTrajectoryGeneratorModule : public loco::ModuleBase, public loco::SwingTrajectoryGeneratorBase {
 public:
  SwingTrajectoryGeneratorModule() = default;
  ~SwingTrajectoryGeneratorModule() override = default;
};

}  // namespace loco
