/*
 * FootholdGeneratorOptimizedConstraint.hpp
 *
 *  Created on: Mar 15, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// loco
#include "loco/common/WholeBody.hpp"
#include "loco/foothold_generation/FootholdGeneratorOptimizedQPBase.hpp"
#include "loco/foothold_generation/foothold_generator.hpp"

// robot utils
#include "robot_utils/geometry/geometry.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

class TiXmlHandle;

namespace loco {

class FootholdGeneratorOptimizedConstraint : public FootholdGeneratorOptimizedQPBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Base = FootholdGeneratorOptimizedQPBase;
  using Weight = Base::Weight;
  using Position2d = Base::Position2d;

  FootholdGeneratorOptimizedConstraint(WholeBody& wholeBody);
  ~FootholdGeneratorOptimizedConstraint() override = default;

  bool loadParameters(const TiXmlHandle& handle) override;
  bool addVariablesToLog() override;
  virtual bool initialize(double dt);

  //! Run optimization.
  bool compute(foothold_generator::FootholdPlan& plan) override;

 protected:
  // Computations done before the optimization starts.
  virtual bool doPreComputations(const foothold_generator::FootholdPlan& plan);

  //! Initialize the optimization for leg with index legId.
  bool setupConstraintProperties(unsigned int legId, const foothold_generator::FootholdPlan& plan) override;

  //! Compute Hessian Q and linear term l of the cost function x'Qx + l'*x.
   bool setupCostFunction(unsigned int legId, const foothold_generator::FootholdPlan& plan) override;

  //! Compute Jacobian A_in and min value b_in of the linear ineqconstraints A_in*x>=b_in.
  bool setupInequalityConstraints(unsigned int legId, const foothold_generator::FootholdPlan& plan) override;

  //! Compute Jacobian A_eq and target values b_in of the linear eqconstraints A_eq*x=b_eq.
  bool setupEqualityConstraints(unsigned int legId, const foothold_generator::FootholdPlan& plan) override;

  //! Add constraints for avoiding kinematic singularity configuration.
  virtual bool addBoundedPolygonConstraints(unsigned int legId, unsigned int lineId, const foothold_generator::FootholdPlan& plan);

  //! Geometrically establish collision line constraints.
  virtual bool constructCollistionLines(const foothold_generator::FootholdPlan& plan);

  //! True if point is in collision cone.
  virtual bool isPositionInCollisionAvoidanceCone(
      unsigned int legId,
      const Position2d& positionPlaneToPointInPlaneFrame,
      unsigned int& criticalNeughborLegId,
      const foothold_generator::FootholdPlan& plan);

  //! Add constraints for avoiding leg collisions.
  virtual bool addCollisionAvoidanceConstraints(
      unsigned int legId,
      unsigned int lineId,
      unsigned int legIdNeighbor,
      const foothold_generator::FootholdPlan& plan);

  //! Read the solution.
  virtual Position getPositionWorldToDesiredFootholdInWorldFrame(const foothold_generator::FootholdPlan& plan) const;

  //! Set up optimization problem as a QP.
  bool setUpQP(unsigned int legId, const foothold_generator::FootholdPlan& plan) override;

  //! Reference to the whole body.
  WholeBody& wholeBody_;

  //! The vector of the previous solution given in world frame.
  std::vector<Position, Eigen::aligned_allocator<Position>> positionWorldToPreviousFootholdInWorldFrame_;

  //! Kinematic constraints.
  std::vector<robot_utils::geometry::Polygon> boundPolygonsInPlaneFrame_;
  //! Collision line constraints (for each foot as three neighboring legs from which each has two line constraints).
  std::vector<std::vector<std::vector<robot_utils::geometry::Line>>> collisionLinesInPlaneFrame_;

  // Defines the shortest distance from the foothold to the collision cone.
  double safetyMarginCollisionAvoidance_;

  // Circle constraints are approximated with a convex polygon having the following number of vertices.
  unsigned int numPolygonEdgesForBoundPolygon_;
  unsigned int numPolygonEdgesForCollisionCone_;

  //! Rotation matrix in 2D that rotates about a positive angle about the z-axis.
  Eigen::Matrix2d RotationMatrixBound_;
  Eigen::Matrix2d RotationMatrixCollisionCone_;

  //! If true, the collision cone is closed (otherwise its open).
  bool closeCollisionCone_;

  //! Squared of the kinematic max leg extension.
  double maxLegExtensionSquared_;

  //! Element i is true if ith inequality constraint is active.
  std::vector<bool> enableInequalityConstraints_;

  //! neighbur leg index which might be touched.
  unsigned int criticalNeughborLegId_;

  //! Currently active line constraint of collision cone.
  unsigned int activeLineId_;

  //! True if one of the collision cone constraints is violated.
  bool isCollisionConeConstraintActive_;
};

} /* namespace loco */
