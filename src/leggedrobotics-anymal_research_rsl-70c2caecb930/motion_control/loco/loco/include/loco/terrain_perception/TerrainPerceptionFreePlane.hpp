/*
 * TerrainPerceptionFreePlane.hpp
 *
 *  Created on: Aug 28, 2014
 *      Author: C. Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/TerrainModelFreePlane.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/heading_generation/HeadingGenerator.hpp"
#include "loco/terrain_perception/TerrainPerceptionBase.hpp"

// basic filters
#include "basic_filters/filters.hpp"

namespace loco {

class TerrainPerceptionFreePlane : public TerrainPerceptionBase {
 public:
  using EndEffectorFrame = EndEffectorBase::EndEffectorFrame;

 public:
  enum ControlFrameHeading { Hips = 0, Feet };

  /*! Choose whether to estimate the measuring the footsteps in world or in base frame */
  enum EstimatePlaneInFrame { World = 0, Base };

  TerrainPerceptionFreePlane(
      TerrainModelPlane& terrainModel, WholeBody& wholeBody, HeadingGenerator& headingGenerator,
      TerrainPerceptionFreePlane::EstimatePlaneInFrame estimatePlaneInFrame = TerrainPerceptionFreePlane::EstimatePlaneInFrame::World,
      ControlFrameHeading referenceHeading = ControlFrameHeading::Hips,
      EndEffectorFrame referenceEndEffectorContact = EndEffectorContactEnum::Contact, bool updateTerrainWithDesiredFoothold = false);

  ~TerrainPerceptionFreePlane() override = default;

  /*! Initialize saved foot measurements.
   * @param dt  time step [s]
   */
  bool initialize(double dt) override;

  /*! Advance in time. Update saved foot measurements with the latest foot positions
   * if foots are grounded. Also check if a given foot was grounded at least once.
   * @param dt  time step [s]
   */
  bool advance(double dt) override;

  /** Adds variables to the signal logger
   * @param ns            namespace of the variables
   * @return              true, iff successful
   */
  bool addVariablesToLog(const std::string& ns) const override;

  /** Loads parameters from an xml file
   *  @param handle tinyxml handle
   *  @return true, iff successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /*! Change the coordinate system of the foot position vector from base to world frame taking into account
   * the last saved displacement and orientation between the reference systems for the given foot
   * @params[in/out] position A position vector, expressed in base frame, that has to be expressed in world frame
   */
  void homogeneousTransformFromBaseToWorldFrame(loco::Position& position, int footID);

  /*! Update a custom terrain model object with a set of points in world frame
   * @param[in] pointsInWorldFrame List of points in world frame
   * @param[out] terrainModel The terrain model object that will be updated
   */
  static bool generateTerrainModelFromPointsInWorldFrame(const std::vector<Position>& pointsInWorldFrame,
                                                         TerrainModelFreePlane& terrainModel);

  void updateControlFrameOrigin() override;
  void updateControlFrameAttitude() override;

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
  void updateLocalMeasuresOfLeg(const loco::LegBase& leg);
  void updateDesiredMeasuresOfLeg(const loco::LegBase& leg);

  TerrainModelPlane& terrainModel_;
  WholeBody& wholeBody_;
  HeadingGenerator& headingGenerator_;
  TorsoBase& torso_;
  Legs& legs_;

  std::vector<loco::Position> mostRecentPositionOfFoot_;
  std::vector<loco::Position> lastWorldToBasePositionInWorldFrameForFoot_;
  std::vector<RotationQuaternion> lastWorldToBaseOrientationForFoot_;
  std::vector<bool> gotFirstTouchDownOfFoot_;
  TerrainPerceptionFreePlane::EstimatePlaneInFrame estimatePlaneInFrame_;

  //--- First order filters
  basic_filters::FirstOrderFilterKindrVector3d normalFilter_;
  basic_filters::FirstOrderFilterKindrPosition positionFilter_;
  double filterNormalTimeConstant_;
  double filterPositionTimeConstant_;
  double filterNormalGain_;
  double filterPositionGain_;

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

  bool updateTerrainWithDesiredFoothold_;
};
// class

} /* namespace loco */
