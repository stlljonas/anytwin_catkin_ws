/*
 * TerrainPerceptionPredictiveFreePlane.hpp
 *
 *  Created on: Jan 09, 2019
 *      Author: Fabian Jenelten
 */

#pragma once

// loco
#include "loco/terrain_perception/TerrainPerceptionBase.hpp"
#include "loco/common/TerrainModelFreePlane.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/heading_generation/HeadingGenerator.hpp"

// basic filters
#include "basic_filters/filters.hpp"

namespace loco {

class TerrainPerceptionPredictiveFreePlane : public TerrainPerceptionBase {
 public:
  using EndEffectorFrame = EndEffectorBase::EndEffectorFrame;

 public:

  enum ControlFrameHeading {
    Hips = 0,
    Feet
  };

  TerrainPerceptionPredictiveFreePlane(
      TerrainModelPlane& terrainModel, WholeBody& wholeBody,
      HeadingGenerator& headingGenerator,
      ControlFrameHeading referenceHeading = ControlFrameHeading::Hips,
      EndEffectorFrame referenceEndEffectorContact = EndEffectorContactEnum::Contact,
      bool updateTerrainWithDesiredFoothold = false);

  ~TerrainPerceptionPredictiveFreePlane() override = default;

  /*! Initialize saved foot measurements.
   * @param dt  time step [s]
   */
  virtual bool initialize(double dt) override;

  /*! Advance in time. Update saved foot measurements with the latest foot positions
   * if foots are grounded. Also check if a given foot was grounded at least once.
   * @param dt  time step [s]
   */
  virtual bool advance(double dt) override;

  /** Adds variables to the signal logger
   * @param ns            namespace of the variables
   * @return              true, iff successful
   */
  virtual bool addVariablesToLog(const std::string & ns = "") const;

  /** Loads parameters from an xml file
   *  @param handle tinyxml handle
   *  @return true, iff successful
   */
  virtual bool loadParameters(const TiXmlHandle& handle);

  /*! Update a custom terrain model object with a set of points in world frame
   * @param[in] pointsInWorldFrame List of points in world frame
   * @param[out] terrainModel The terrain model object that will be updated
   */
  static bool generateTerrainModelFromPointsInWorldFrame(
      const std::vector<Position>& pointsInWorldFrame, TerrainModelFreePlane& terrainModel);

  void updateControlFrameOrigin();
  void updateControlFrameAttitude();

  /*
   * Update terrain perception.
   * usePerception = false -> Local estimation of terrain plane is computed with current and previous stance legs.
   * usePerception = true ->  Local estimation of terrain plane is computed with current and desired stance legs.
   */
  void setUpdateTerrainWithDesiredFoothold(bool updateTerrainWithDesiredFoothold) noexcept;

 protected:
  /*! Estimate the free plane parameters and update the terrain model.
   */
  virtual void updatePlaneEstimation();

  /*! Update last foot positions and torso pose for a leg.
   * @param[in,out] leg The leg to measure.
   */
  virtual void updateLocalMeasuresOfLeg(const loco::LegBase& leg);
  virtual void updateDesiredMeasuresOfLeg(const loco::LegBase& leg);

  TerrainModelPlane& terrainModel_;
  WholeBody& wholeBody_;
  HeadingGenerator& headingGenerator_;
  TorsoBase& torso_;
  Legs& legs_;

  std::vector<loco::Position> positionWorldToFootOnPlaneInWorldFrame_;
  std::vector<bool> gotFirstTouchDownOfFoot_;

  //--- First order filters
  basic_filters::FirstOrderFilterKindrVector3d normalFilter_;
  basic_filters::FirstOrderFilterKindrPosition positionFilter_;
  double filterNormalTimeConstant_;
  double filterPositionTimeConstant_;
  double filterNormalGain_;
  double filterPositionGain_;

  //! Filters used during regaining.
  basic_filters::FirstOrderFilterKindrVector3d normalFilterRegaining_;
  basic_filters::FirstOrderFilterKindrPosition positionFilterRegaining_;
  double filterNormalAndPositionTimeConstantForRegaining_;

  loco::Vector normalInWorldFrameFilterInput_;
  loco::Position positionInWorldFrameFilterInput_;
  loco::Vector normalInWorldFrameFilterOutput_;
  loco::Position positionInWorldFrameFilterOutput_;
  //---

  ControlFrameHeading referenceHeading_;

  RotationQuaternion orientationWorldToControlOld_;

  double timeStep_;

  const Eigen::Vector3d unitX_{Eigen::Vector3d::UnitX()};
  const Eigen::Vector3d unitZ_{Eigen::Vector3d::UnitZ()};
  const loco::Position positionZero_{loco::Position()};

  EndEffectorFrame referenceEndEffectorContact_;

  //! If false, use previous and current stance foothold for local terrain approximation.
  //! Otherwise, use current and desired footholds.
  bool updateTerrainWithDesiredFoothold_;

  //! If true, logs some variables.
  bool enableLogging_;
};
// class

} /* namespace loco */

