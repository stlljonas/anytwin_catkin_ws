/*
 * FootholdPlan.hpp
 *
 *  Created on: Oct 9, 2017
 *      Author: dbellicoso
 */

#pragma once

// loco
#include "loco/common/typedefs.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/heading_generation/HeadingGenerator.hpp"

// motion generation
#include "motion_generation/ContactScheduleZmp.hpp"

// zmp optimizer
#include "zmp_optimizer/SupportPolygon.hpp"
#include "zmp_optimizer/VirtualPlaneFrame.hpp"

// stl
#include <vector>

// robot utils
#include "robot_utils/geometry/Line.hpp"
#include "robot_utils/geometry/Polygon.hpp"

namespace loco {

namespace foothold_generator {

class FootholdPlan {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FootholdPlan()
        : numOfEndEffectors_(),
          positionsWorldToFootholdsInWorldFrame_(),
          boundPolygonsInPlaneFrame_(),
          collisionLinesInPlaneFrame_(),
          positionWorldToThighInWorldFrame_(),
          positionZPlaneToDesiredThighHeightInPlaneFrame_(),
          virtualPlaneFrame_(zmp::vpf::VirtualPlaneFrameEnum::previousFootprint),
          footStateMeasured_(0u, loco::EndEffectorStateMeasured()),
          positionWorldToPreviousRealizedSolution_(0u, Position::Zero())
  {

  }

  virtual ~FootholdPlan() = default;

  virtual bool initialize(const loco::WholeBody& wholeBody) {
    numOfEndEffectors_ = wholeBody.getLegs().size();
    positionsWorldToFootholdsInWorldFrame_.resize(numOfEndEffectors_);
    footStateMeasured_.resize(numOfEndEffectors_);

    positionWorldToThighInWorldFrame_.resize(numOfEndEffectors_);
    positionZPlaneToDesiredThighHeightInPlaneFrame_.resize(numOfEndEffectors_);
    positionWorldToPreviousRealizedSolution_.resize(numOfEndEffectors_);

    for (const auto& leg : wholeBody.getLegs()) {
      positionsWorldToFootholdsInWorldFrame_[leg->getId()] =
          leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
      positionWorldToPreviousRealizedSolution_[leg->getId()] = positionsWorldToFootholdsInWorldFrame_[leg->getId()];
    }

    boundPolygonsInPlaneFrame_.clear();
    collisionLinesInPlaneFrame_.clear();

    return true;
  }

  bool loadParameters(const TiXmlHandle& /* handle */) {
    return true;
  }

  virtual void setVirtualPlaneFrame(zmp::vpf::VirtualPlaneFrameEnum planeFrame) {
    virtualPlaneFrame_.setVirtualPlaneFrameEnum(planeFrame);
  }

  virtual void setEndEffectorStateMeasured(const loco::EndEffectorStateMeasured& state, unsigned int footId) {
    footStateMeasured_[footId] = state;
  }

  virtual const loco::EndEffectorStateMeasured& getEndEffectorStateMeasured(unsigned int footId) const {
    return footStateMeasured_[footId];
  }

  virtual unsigned int getNumLegs() const {
    return numOfEndEffectors_;
  }

  virtual void setBoundedPolygonsInPlaneFrame(const std::vector<robot_utils::geometry::Polygon>& polygons) {
    boundPolygonsInPlaneFrame_ = polygons;
  }

  virtual const std::vector<robot_utils::geometry::Polygon>& getBoundedPolygonsInPlaneFrame() const {
    return boundPolygonsInPlaneFrame_;
  }

  virtual void setCollisionLinesInPlaneFrame(const std::vector<std::vector<std::vector<robot_utils::geometry::Line>>>& collisionLines) {
    collisionLinesInPlaneFrame_ = collisionLines;
  }

  virtual const std::vector<std::vector<std::vector<robot_utils::geometry::Line>>>& getCollisionLinesInPlaneFrame() const {
    return collisionLinesInPlaneFrame_;
  }

  virtual void setPositionWorldToFootholdInWorldFrame(const loco::Position& position, unsigned int footId) {
    positionsWorldToFootholdsInWorldFrame_[footId] = position;
  }

  virtual const loco::Position& getPositionWorldToFootholdInWorldFrame(unsigned int footId) const {
    return positionsWorldToFootholdsInWorldFrame_[footId];
  }

  virtual const std::vector<Position, Eigen::aligned_allocator<Position>>& getPositionWorldToFootholdsInWorldFrame() const {
    return positionsWorldToFootholdsInWorldFrame_;
  }

  virtual const loco::Position& getPositionWorldToThighInWorldFrame(unsigned int footId) const {
    return positionWorldToThighInWorldFrame_[footId];
  }

  virtual const zmp::VirtualPlaneFrame& getVirtualPlaneFrame() const {
    return virtualPlaneFrame_;
  }

  virtual double getPositionZPlaneToDesiredThighHeightInPlaneFrame(unsigned int legId) const {
    return positionZPlaneToDesiredThighHeightInPlaneFrame_[legId];
  }

  virtual bool updateData(
      const loco::WholeBody& wholeBody,
      const loco::TerrainModelBase& terrainModel,
      const loco::ContactScheduleZmp& contactSchedule,
      const loco::HeadingGenerator& headingGenerator,
      const Position& positionPlaneToDesiredTargetHeightInPlaneFrame) {

    // Update plane frame.
    if(!virtualPlaneFrame_.computeVirtualPlaneFrame(headingGenerator, wholeBody, terrainModel, contactSchedule)) { return false; }

    // Position base to target point in world frame.
    Position positionBaseToTargetPointInWorldFrame;
    const Position& positionWorldToBaseInWorldFrame = wholeBody.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame();
    switch (wholeBody.getTorso().getDesiredState().getTargetPoint()) {
      case(TorsoStateDesired::TargetPoint::WBCOMXY_BASEZ): {
        positionBaseToTargetPointInWorldFrame = wholeBody.getWholeBodyStateMeasured().getPositionWorldToWholeBodyCenterOfMassInWorldFrame()
                                                -positionWorldToBaseInWorldFrame;
        positionBaseToTargetPointInWorldFrame.z() = 0.0;
      } break;

      case(TorsoStateDesired::TargetPoint::WBCOM): {
        positionBaseToTargetPointInWorldFrame = wholeBody.getWholeBodyStateMeasured().getPositionWorldToWholeBodyCenterOfMassInWorldFrame()
                                                -positionWorldToBaseInWorldFrame;
      } break;

      case(TorsoStateDesired::TargetPoint::BASE): {
        positionBaseToTargetPointInWorldFrame.setZero();
      } break;

      case(TorsoStateDesired::TargetPoint::TORSOCOM): {
        positionBaseToTargetPointInWorldFrame = wholeBody.getTorso().getMeasuredState().getOrientationWorldToBase().inverseRotate(
            wholeBody.getTorso().getProperties().getBaseToCenterOfMassPositionInBaseFrame());
      } break;

      default: {
        MELO_FATAL("[updateData] Undefined target point!");
      } break;
    }

    for (unsigned int legId=0u; legId<numOfEndEffectors_; ++legId) {
      const auto& leg = wholeBody.getLegs().get(legId);
      footStateMeasured_[legId] = leg.getFoot().getStateMeasured();
      const Position positionBaseToLinkBaseInWorldFrame = wholeBody.getTorso().getMeasuredState().getOrientationWorldToBase().inverseRotate(
                                                          leg.getLinks().get(1).getBaseToLinkPositionInBaseFrame());
      positionWorldToThighInWorldFrame_[legId] = positionWorldToBaseInWorldFrame + positionBaseToLinkBaseInWorldFrame;
      positionZPlaneToDesiredThighHeightInPlaneFrame_[legId] = (positionPlaneToDesiredTargetHeightInPlaneFrame +
                                                          virtualPlaneFrame_.getPosePlaneToWorld().getRotation().inverseRotate(
                                                              positionBaseToLinkBaseInWorldFrame-positionBaseToTargetPointInWorldFrame)).z();


      // Store previous realized solution.
      positionWorldToPreviousRealizedSolution_[leg.getId()] = leg.getFoot().getStateDesired().getPositionWorldToFootholdInWorldFrame();
    }

    return true;
  }

  const std::vector<Position, Eigen::aligned_allocator<Position>>& getPreviousRealizedSolution() const noexcept {
    return positionWorldToPreviousRealizedSolution_;
  }


  //! Copy foothold plan and constraints.
  virtual void copy(const FootholdPlan& plan) {
   positionsWorldToFootholdsInWorldFrame_ = plan.getPositionWorldToFootholdsInWorldFrame();
   boundPolygonsInPlaneFrame_ = plan.getBoundedPolygonsInPlaneFrame();
   collisionLinesInPlaneFrame_ = plan.getCollisionLinesInPlaneFrame();
   virtualPlaneFrame_ = plan.getVirtualPlaneFrame();
  }

 protected:

  //! Number of End-effectors that can make contact.
  unsigned int numOfEndEffectors_;

  //! Planned footholds (result of the optimization).
  std::vector<Position, Eigen::aligned_allocator<Position>> positionsWorldToFootholdsInWorldFrame_;

  //! Constraints for enforcing kinematic feasibility.
  std::vector<robot_utils::geometry::Polygon> boundPolygonsInPlaneFrame_;

  //! Constraints for avoid leg collisions.
  std::vector<std::vector<std::vector<robot_utils::geometry::Line>>> collisionLinesInPlaneFrame_;

  //! Position of robot's thigh in world frame.
  std::vector<Position, Eigen::aligned_allocator<Position>> positionWorldToThighInWorldFrame_;

  //! Desired height of the tight above ground in plane frame.
  std::vector<double> positionZPlaneToDesiredThighHeightInPlaneFrame_;

  //! The optimization is done in virtual frame.
  zmp::VirtualPlaneFrame virtualPlaneFrame_;

  //! Measured foot state.
  std::vector<EndEffectorStateMeasured> footStateMeasured_;

  //! Previous realized solution (may deviate from previous solution).
  std::vector<Position, Eigen::aligned_allocator<Position>> positionWorldToPreviousRealizedSolution_;
};

} /* namespace foothold_generator */

} /* namespace loco */
