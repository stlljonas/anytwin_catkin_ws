//
// Created by dbellicoso on 22/02/18.
//

#pragma once

// loco
#include <loco/swing_trajectory_generation/SwingTrajectoryGeneratorSpline.hpp>
#include <loco/swing_trajectory_generation/SwingTrajectoryGeneratorModule.hpp>
#include <loco/common/WholeBody.hpp>
#include <loco/common/TerrainModelBase.hpp>

namespace loco {

class SwingTrajectoryGeneratorSplineModule : public SwingTrajectoryGeneratorSpline,
                                             public virtual SwingTrajectoryGeneratorModule {
 public:
  //! Import base class constructor.
  SwingTrajectoryGeneratorSplineModule(WholeBody& wholeBody, TerrainModelBase& terrain)
      : SwingTrajectoryGeneratorModule(),
        SwingTrajectoryGeneratorSpline(wholeBody, terrain)
  {

  }

  bool initialize(double dt) override { return true; }
  bool advance(double dt) override { return true; }

  bool loadParameters(const TiXmlHandle& handle) override {
    return SwingTrajectoryGeneratorSpline::loadParameters(handle);
  }

  bool getDesiredFootState(
      Position& positionWorldToDesiredFootInWorldFrame,
      LinearVelocity& linearVelocityDesiredFootInWorldFrame,
      LinearAcceleration& linearAccelerationDesiredFootInWorldFrame,
      const Position& positionWorldToDesiredFootholdInControlFrame,
      LegBase* leg, double dt) override {
    return SwingTrajectoryGeneratorSpline::getDesiredFootState(
        positionWorldToDesiredFootInWorldFrame,
        linearVelocityDesiredFootInWorldFrame,
        linearAccelerationDesiredFootInWorldFrame,
        positionWorldToDesiredFootholdInControlFrame,
        leg, dt);
  }
};

}