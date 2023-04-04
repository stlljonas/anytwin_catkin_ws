/*
 * TorsoControlFreeGait.hpp
 *
 *  Created on: Jan 14, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include <free_gait_core/free_gait_core.hpp>

// Loco
#include <loco/terrain_perception/TerrainPerceptionFreePlane.hpp>
#include <loco/torso_control/TorsoControlGaitContainer.hpp>

#include <loco_anymal/common/LegsAnymal.hpp>

// Anymal model
#include <anymal_model/AnymalModel.hpp>

// Basic Filters
#include <basic_filters/FirstOrderFilter.hpp>

// Grid map
#include <grid_map_core/Polygon.hpp>

// STD
#include <map>

namespace loco {

class TorsoControlFreeGait : public TorsoControlGaitContainer {
 private:
  using AD = anymal_description::AnymalDescription;

 public:
  TorsoControlFreeGait(WholeBody& wholeBody, anymal_model::AnymalModel& anymalModelDesired, TerrainModelBase& terrain,
                       free_gait::Executor& executor, ComSupportControlBase& comControl);
  ~TorsoControlFreeGait() override = default;

  bool initialize(double dt) override;
  virtual bool reset();
  bool advance(double dt) override;

  bool loadParameters(const TiXmlHandle& handle) override;

  bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t) override;

 protected:
  Pose desiredPoseInWorld_;
  Twist desiredTwistInBase_;

  template <typename T>
  int sgn(T val);

  void setDesiredLoadFactors(double dt);
  void setDesiredPose();
  void setDesiredVelocity();

  void getDesiredBasePitchFromTerrainPitch(const double terrainPitch, double& desiredBasePitch);
  void getDesiredBaseRollFromTerrainRoll(const double terrainRoll, double& desiredBaseRoll);
  void updateLegConsiderations(free_gait::Step& step);
  bool generateFootholdLists(free_gait::Step& step);

 private:
  //! Robot model.
  anymal_model::AnymalModel& anymalModelDesired_;
  free_gait::Executor& executor_;
  std::unordered_map<AD::LimbEnum, double, free_gait::EnumClassHash> durationSinceLegInSupport_;
  double loadFactorLoadDuration_, loadFactorLowerBound_;
  bool allowEarlyTouchDown_;
};

} /* namespace loco */
