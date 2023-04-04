/*
 * Parameters.cpp
 *
 *  Created on: Mar 6, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "locomotion_planner/common/Parameters.hpp"
#include "locomotion_planner/common/geometry.hpp"

#include <limits>

namespace locomotion_planner {

Parameters::Parameters() :
    defaultSpeedFactor_(0.0),
    maxUnpreferredDirectionDistance_(std::numeric_limits<double>::infinity()),
    skipStepThreshold_(0.0),
    mapAreaFootprintRadius_(0.0),
    mapAreaMaxDistance_(0.0),
    footCenterHeight_(0.0),
    footholdSize_(0.0),
    footholdSearchAreaSize_(0.0),
    swingTrajectoryMinClearance_(0.0),
    swingTrajectoryMaxHeight_(0.0),
    walkAndTurnDistance_(0.0),
    twistScaling_(1.0)
{

}

bool Parameters::setNominalPlanarStanceForLimb(const LimbEnum& limb, Position2 position)
{
  nominalStance_[limb] = position;
  return true;
}

const PlanarStance& Parameters::getNominalPlanarStance() const
{
  return nominalStance_;
}

const Position2& Parameters::getNominalPlanarStanceForLimb(const LimbEnum& limb) const
{
  return nominalStance_.at(limb);
}

void Parameters::setDefaultSpeedFactor(const double speedFactor)
{
  defaultSpeedFactor_ = speedFactor;
}

double Parameters::getDefaultSpeedFactor() const
{
  return defaultSpeedFactor_;
}

bool Parameters::setMaxPoseDifferenceForGaitCycle(const Speed& speed, const PlanarPose& maxPoseDifferenceForGaitCycle)
{
  maxPoseDifferenceForGaitCycle_[speed] = maxPoseDifferenceForGaitCycle;
  return true;
}

const PlanarPose Parameters::getMaxPoseDifferenceForGaitCycle(const double speedFactor) const
{
  return interpolateForSpeedFactor(speedFactor, maxPoseDifferenceForGaitCycle_.at(Speed::SLOW),
                                   maxPoseDifferenceForGaitCycle_.at(Speed::FAST));
}

void Parameters::setMaxUnpreferredDirectionDistance(const double distance)
{
  maxUnpreferredDirectionDistance_ = distance;
}

double Parameters::getMaxUnpreferredDirectionDistance() const
{
  return maxUnpreferredDirectionDistance_;
}

void Parameters::setTurnAndWalkDistance(const double distance)
{
  walkAndTurnDistance_ = distance;
}

double Parameters::getTurnAndWalkDistance() const
{
  return walkAndTurnDistance_;
}

bool Parameters::setFoostepParameters(const Speed& speed, const free_gait::Footstep& footstep)
{
  // Erase current params for a given speed and overwrite. Assignment operator is not an option.
  footstepParameters_.erase(speed);
  footstepParameters_.insert({speed, footstep});
  return true;
}

bool Parameters::populateFoostepParameters(const double speedFactor, free_gait::Footstep& footstep) const
{
  footstep.setAverageVelocity(
      interpolateForSpeedFactor(speedFactor, footstepParameters_.at(Speed::SLOW).getAverageVelocity(),
                                footstepParameters_.at(Speed::FAST).getAverageVelocity()));
  footstep.setProfileHeight(
      interpolateForSpeedFactor(speedFactor, footstepParameters_.at(Speed::SLOW).getProfileHeight(),
                                footstepParameters_.at(Speed::FAST).getProfileHeight()));
  if (speedFactor <= 0.5) {
    footstep.setProfileType(footstepParameters_.at(Speed::SLOW).getProfileType());
  } else {
    footstep.setProfileType(footstepParameters_.at(Speed::FAST).getProfileType());
  }
  return true;
}

double Parameters::getSkipStepThreshold() const
{
  return skipStepThreshold_;
}

void Parameters::setSkipStepThreshold(double skipStepThreshold)
{
  skipStepThreshold_ = skipStepThreshold;
}

bool Parameters::setBaseMotionCommonParameters(const free_gait::BaseAuto& baseAuto)
{
  baseAutoCommonParameters_.setCenterOfMassTolerance(baseAuto.getCenterOfMassTolerance());
  baseAutoCommonParameters_.setLegLengthTolerance(baseAuto.getLegLengthTolerance());
  baseAutoCommonParameters_.setMinLimbLengthScale(baseAuto.getMinLimbLengthScale());
  baseAutoCommonParameters_.setMaxLimbLengthAtClosingContactScale(baseAuto.getMaxLimbLengthAtClosingContactScale());
  baseAutoCommonParameters_.setMaxLimbLengthAtOpeningContactScale(baseAuto.getMaxLimbLengthAtOpeningContactScale());

  return true;
}

bool Parameters::setBaseMotionParameters(const Speed& speed, const free_gait::BaseAuto& baseAuto)
{
  // Erase current params for a given speed and overwrite. Assignment operator is not an option.
  baseAutoParameters_.erase(speed);
  baseAutoParameters_.insert({speed, baseAuto});
  return true;
}

bool Parameters::populateBaseAutoParameters(const double speedFactor, free_gait::BaseAuto& baseAuto) const
{
  baseAuto.setHeight(
      interpolateForSpeedFactor(speedFactor, baseAutoParameters_.at(Speed::SLOW).getHeight(),
                                baseAutoParameters_.at(Speed::FAST).getHeight()));
  baseAuto.setAverageLinearVelocity(
      interpolateForSpeedFactor(speedFactor, baseAutoParameters_.at(Speed::SLOW).getAverageLinearVelocity(),
                                baseAutoParameters_.at(Speed::FAST).getAverageLinearVelocity()));
  baseAuto.setAverageAngularVelocity(
      interpolateForSpeedFactor(speedFactor, baseAutoParameters_.at(Speed::SLOW).getAverageAngularVelocity(),
                                baseAutoParameters_.at(Speed::FAST).getAverageAngularVelocity()));
  baseAuto.setSupportMargin(
      interpolateForSpeedFactor(speedFactor, baseAutoParameters_.at(Speed::SLOW).getSupportMargin(),
                                baseAutoParameters_.at(Speed::FAST).getSupportMargin()));

  baseAuto.setCenterOfMassTolerance(baseAutoCommonParameters_.getCenterOfMassTolerance());
  baseAuto.setLegLengthTolerance(baseAutoCommonParameters_.getLegLengthTolerance());
  baseAuto.setMinLimbLengthScale(baseAutoCommonParameters_.getMinLimbLengthScale());
  baseAuto.setMaxLimbLengthAtClosingContactScale(baseAutoCommonParameters_.getMaxLimbLengthAtClosingContactScale());
  baseAuto.setMaxLimbLengthAtOpeningContactScale(baseAutoCommonParameters_.getMaxLimbLengthAtOpeningContactScale());
  baseAuto.setNominalStanceInBaseFrame(nominalStance_);

  return true;
}

void Parameters::setStopOnFootTrajectoryFailure(const bool stopOnFootTrajectoryFailure)
{
  stopOnFootTrajectoryFailure_ = stopOnFootTrajectoryFailure;
}

bool Parameters::getStopOnFootTrajectoryFailure() const
{
  return stopOnFootTrajectoryFailure_;
}

void Parameters::setUsePrimarySwingTrajectory(const bool usePrimarySwingTrajectory)
{
  usePrimarySwingTrajectory_ = usePrimarySwingTrajectory;
}

bool Parameters::getUsePrimarySwingTrajectory() const
{
  return usePrimarySwingTrajectory_;
}

void Parameters::setElevationMapLayers(const std::string& elevationLayer, const std::string& footholdScoreLayer,
                           const std::string& collisionLayer)
{
  elevationLayer_ = elevationLayer;
  footholdScoreLayer_ = footholdScoreLayer;
  collisionLayer_ = collisionLayer;
}

const std::string& Parameters::getElevationLayer() const
{
  return elevationLayer_;
}

const std::string& Parameters::getFootholdScoreLayer() const
{
  return footholdScoreLayer_;
}

const std::string& Parameters::getCollisionLayer() const
{
  return collisionLayer_;
}

void Parameters::setElevationMapRegionParameters(const double footprintRadius, const double maxDistance)
{
  mapAreaFootprintRadius_ = footprintRadius;
  mapAreaMaxDistance_ = maxDistance;
}

void Parameters::getElevationMapRegionParameters(double& footprintRadius, double& maxDistance) const
{
  footprintRadius = mapAreaFootprintRadius_;
  maxDistance = mapAreaMaxDistance_;
}

void Parameters::setFootCenterHeight(const double footCenterHeight)
{
  footCenterHeight_ = footCenterHeight;
}

double Parameters::getFootCenterHeight() const
{
  return footCenterHeight_;
}

void Parameters::setFootholdSize(const double footholdSize)
{
  footholdSize_ = footholdSize;
}

double Parameters::getFootholdSize() const
{
  return footholdSize_;
}

void Parameters::setFootholdSearchAreaSize(const double size)
{
  footholdSearchAreaSize_ = size;
}

double Parameters::getFootholdSearchAreaSize() const
{
  return footholdSearchAreaSize_;
}

void Parameters::setSwingTrajectoryParameters(const double swingTrajectoryMinClearance, const double swingTrajectoryMaxHeight)
{
  swingTrajectoryMinClearance_ = swingTrajectoryMinClearance;
  swingTrajectoryMaxHeight_ = swingTrajectoryMaxHeight;
}

double Parameters::getSwingTrajectoryMinClearance() const
{
  return swingTrajectoryMinClearance_;
}

double Parameters::getSwingTrajectoryMaxHeight() const
{
  return swingTrajectoryMaxHeight_;
}
const std::string& Parameters::getDefaultGoalFrameId() const{
 return defaultGoalFrameId_;
}
void Parameters::setDefaultGoalFrameId(const std::string& defaultGoalFrameId){
    defaultGoalFrameId_ = defaultGoalFrameId;
}

} /* namespace */
