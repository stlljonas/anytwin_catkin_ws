/*
 * Parameters.hpp
 *
 *  Created on: Mar 6, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "locomotion_planner/common/type_defs.hpp"

#include <free_gait_core/free_gait_core.hpp>

#include <string>

namespace locomotion_planner {

class Parameters
{
 public:
  enum class Speed
  {
    SLOW,
    FAST
  };

  Parameters();
  virtual ~Parameters() = default;

  //! Set a nominal planar stance position for a limb.
  //! @param llimb limbEnum of the limb
  //! @param position the planar position relative to the base.
  //! @return true if successful, false otherwise.
  bool setNominalPlanarStanceForLimb(const LimbEnum& limb, Position2 position);

  const PlanarStance& getNominalPlanarStance() const;
  const Position2& getNominalPlanarStanceForLimb(const LimbEnum& limb) const;

  void setDefaultSpeedFactor(const double speedFactor);
  double getDefaultSpeedFactor() const;

  bool setMaxPoseDifferenceForGaitCycle(const Speed& speed, const PlanarPose& maxPoseDifferenceForGaitCycle);
  const PlanarPose getMaxPoseDifferenceForGaitCycle(const double speedFactor = 1.0) const;

  void setMaxUnpreferredDirectionDistance(const double distance);
  double getMaxUnpreferredDirectionDistance() const;

  void setTurnAndWalkDistance(const double distance);
  double getTurnAndWalkDistance() const;

  bool setFoostepParameters(const Speed& speed, const free_gait::Footstep& footstep);
  bool populateFoostepParameters(const double speedFactor, free_gait::Footstep& footstep) const;

  double getSkipStepThreshold() const;
  void setSkipStepThreshold(const double skipStepThreshold);

  bool setBaseMotionCommonParameters(const free_gait::BaseAuto& baseAuto);
  bool setBaseMotionParameters(const Speed& speed, const free_gait::BaseAuto& baseAuto);
  bool populateBaseAutoParameters(const double speedFactor, free_gait::BaseAuto& baseAuto) const;

  void setStopOnFootTrajectoryFailure(const bool stopOnFootTrajectoryFailure);
  bool getStopOnFootTrajectoryFailure() const;

  void setUsePrimarySwingTrajectory(const bool usePrimarySwingTrajectory);
  bool getUsePrimarySwingTrajectory() const;

  void setElevationMapLayers(const std::string& elevationLayer, const std::string& footholdScoreLayer,
                             const std::string& collisionLayer);
  const std::string& getElevationLayer() const;
  const std::string& getFootholdScoreLayer() const;
  const std::string& getCollisionLayer() const;

  //! Sets the parameters for the region of the elevation map request. The region is defined
  //! by a bounding box around the a circular area around the current robot position and
  //! a circular area around the goal position, where the goal position is located at a max.
  //! distance from the start position.
  //! @param footprintRadius the radius of the circular area.
  //! @param maxDistance the max. distance between start and goal position.
  void setElevationMapRegionParameters(const double footprintRadius, const double maxDistance);
  void getElevationMapRegionParameters(double& footprintRadius, double& maxDistance) const;

  void setFootCenterHeight(const double footCenterHeight);
  double getFootCenterHeight() const;

  void setFootholdSize(const double footholdSize);
  double getFootholdSize() const;

  void setFootholdSearchAreaSize(const double size);
  double getFootholdSearchAreaSize() const;

  void setSwingTrajectoryParameters(const double swingTrajectoryMinClearance, const double swingTrajectoryMaxHeight);
  double getSwingTrajectoryMinClearance() const;
  double getSwingTrajectoryMaxHeight() const;

  void setTwistScaling(double twistScaling) { twistScaling_ = twistScaling;}
  double getTwistScaling() const { return twistScaling_;  }

  const std::string& getDefaultGoalFrameId() const;
  void setDefaultGoalFrameId(const std::string& defaultGoalFrameId);

 private:
  PlanarStance nominalStance_;
  double defaultSpeedFactor_;
  std::unordered_map<Speed, PlanarPose, EnumClassHash> maxPoseDifferenceForGaitCycle_;
  double maxUnpreferredDirectionDistance_;
  double walkAndTurnDistance_;
  std::unordered_map<Speed, free_gait::Footstep, EnumClassHash> footstepParameters_;
  double skipStepThreshold_;
  free_gait::BaseAuto baseAutoCommonParameters_;
  std::unordered_map<Speed, free_gait::BaseAuto, EnumClassHash> baseAutoParameters_;
  bool stopOnFootTrajectoryFailure_;
  bool usePrimarySwingTrajectory_;
  std::string defaultGoalFrameId_;
  std::string elevationLayer_;
  std::string footholdScoreLayer_;
  std::string collisionLayer_;
  double mapAreaFootprintRadius_;
  double mapAreaMaxDistance_;
  double footCenterHeight_; // Above ground.
  double footholdSize_; // For point foot the diameter.
  double footholdSearchAreaSize_; // For circle the diameter.
  double swingTrajectoryMinClearance_;
  double swingTrajectoryMaxHeight_;
  double twistScaling_;
};

} /* namespace locomotion_planner */
