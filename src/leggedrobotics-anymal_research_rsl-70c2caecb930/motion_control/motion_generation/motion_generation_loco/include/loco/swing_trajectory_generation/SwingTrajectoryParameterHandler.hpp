/*
 * ComSupportParameterHandler.hpp
 *
 *  Created on: Mar 27, 2018
 *      Author: Fabian Jenelten
 */
#pragma once

// motion generation
#include "motion_generation_utils/ParameterHandler.hpp"


namespace loco {
namespace swing_traj_opt {

struct SwingParams {
  explicit SwingParams() :
      zTrajectoryScaledTimes_(),
      zTrajectoryValues_(),
      zTrajectorySpeedInitial_(0.0),
      zTrajectorySpeedFinal_(0.0),
      xyTrajectorySpeedInitial_(0.0),
      xyTrajectorySpeedFinal_(0.0) {
  }

  //! Knot points for swing timings (values normalized with swing duration).
  std::vector<double> zTrajectoryScaledTimes_;

  //! Knot points for swing height (assuming flat ground).
  std::vector<double> zTrajectoryValues_;

  //! Vertical lift-off velocity (in world frame).
  double zTrajectorySpeedInitial_;

  //! Vertical touch-down velocity (in world frame).
  double zTrajectorySpeedFinal_;

  //! Horizontal lift-off velocity (in world frame).
  double xyTrajectorySpeedInitial_;

  //! Horizontal touch-down velocity (in world frame).
  double xyTrajectorySpeedFinal_;
};


class SwingTrajectoryParameterHandler : public motion_generation::ParameterHandler<SwingParams> {
public:
  SwingTrajectoryParameterHandler() : ParameterHandler() { }
  ~SwingTrajectoryParameterHandler() override = default;

  bool loadParameters(
      const TiXmlHandle& gaitHandle,
      unsigned int gaitIndex,
      const std::string& gaitPatternTypeStr) override {
    emplace_params(gaitIndex, gaitPatternTypeStr);

    // Load vertical (height, normal) trajectory initial and final velocities.
    TiXmlHandle zTrajVelocityHandle = gaitHandle;
    if (!tinyxml_tools::getChildHandle(zTrajVelocityHandle, gaitHandle, "VerticalVelocity")) { return false; }
    if (!tinyxml_tools::loadParameter(params_.back().zTrajectorySpeedInitial_, zTrajVelocityHandle, "initial")) { return false; }
    if (!tinyxml_tools::loadParameter(params_.back().zTrajectorySpeedFinal_, zTrajVelocityHandle, "final")) { return false; }

    // Load horizontal (planar) trajectory initial and final velocities.
    TiXmlHandle xyTrajVelocityHandle = gaitHandle;
    if (!tinyxml_tools::getChildHandle(xyTrajVelocityHandle, gaitHandle, "HorizontalVelocity")) { return false; }
    if (!tinyxml_tools::loadParameter(params_.back().xyTrajectorySpeedInitial_, xyTrajVelocityHandle, "initial")) { return false; }
    if (!tinyxml_tools::loadParameter(params_.back().xyTrajectorySpeedFinal_, xyTrajVelocityHandle, "final")) { return false; }

    // Load height knot points.
    TiXmlHandle heightTrajHandle = gaitHandle;
    params_.back().zTrajectoryValues_.resize(3);
    if (!tinyxml_tools::getChildHandle(heightTrajHandle, gaitHandle, "Height")) { return false; }
    if (!tinyxml_tools::loadParameter(params_.back().zTrajectoryValues_[0], heightTrajHandle, "initial")) { return false; }
    if (!tinyxml_tools::loadParameter(params_.back().zTrajectoryValues_[1], heightTrajHandle, "middle")) { return false; }
    if (!tinyxml_tools::loadParameter(params_.back().zTrajectoryValues_[2], heightTrajHandle, "final")) { return false; }

    // Knot point for timings (hard coded).
    params_.back().zTrajectoryScaledTimes_.resize(3);
    params_.back().zTrajectoryScaledTimes_[0] = 0.0;
    params_.back().zTrajectoryScaledTimes_[1] = 0.5;
    params_.back().zTrajectoryScaledTimes_[2] = 1.0;

    return true;
  }

};


} /* swing_traj_opt */
} /* namespace loco */
