/*
 * FootholdGeneratorOptimizedConstraint.cpp
 *
 *  Created on: Mar 15, 2018
 *      Author: Fabian Jenelten
 */

// loco
#include "loco/foothold_generation/FootholdGeneratorOptimizedConstraint.hpp"

namespace loco {

FootholdGeneratorOptimizedConstraint::FootholdGeneratorOptimizedConstraint(WholeBody& wholeBody)
    : Base(),
      wholeBody_(wholeBody),
      positionWorldToPreviousFootholdInWorldFrame_(wholeBody.getLegs().size(), Position::Zero()),
      boundPolygonsInPlaneFrame_(wholeBody.getLegs().size(), robot_utils::geometry::Polygon()),
      collisionLinesInPlaneFrame_(),
      safetyMarginCollisionAvoidance_(0.0),
      numPolygonEdgesForBoundPolygon_(8u),
      numPolygonEdgesForCollisionCone_(8u),
      RotationMatrixBound_(),
      RotationMatrixCollisionCone_(),
      closeCollisionCone_(false),
      maxLegExtensionSquared_(-1.0),
      enableInequalityConstraints_(foothold_generator::SIZE_OF_INEQCONSTRAINTS, false),
      criticalNeughborLegId_(0u),
      activeLineId_(0u),
      isCollisionConeConstraintActive_(false)
{

}

bool FootholdGeneratorOptimizedConstraint::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle fpsHandle = handle;
  if (!tinyxml_tools::getChildHandle(fpsHandle, handle, "FootPlacementStrategy")) { return false; }
  TiXmlHandle fgHandle = handle;
  if(!tinyxml_tools::getChildHandle(fgHandle, fpsHandle, "FootholdGenerator")) { return false; }
  TiXmlHandle fgOptimizedHandle = handle;
  if(!tinyxml_tools::getChildHandle(fgOptimizedHandle, fgHandle, "Optimized")) { return false; }
  TiXmlHandle inequalityHandle = handle;
  if(!tinyxml_tools::getChildHandle(inequalityHandle, fgOptimizedHandle, "InequalityConstraints")) { return false; }


  // Kinematic constraints.
  TiXmlHandle kinematicHandle = handle;
  if(!tinyxml_tools::getChildHandle(kinematicHandle, inequalityHandle, "KinematicConstraints")) { return false; }

  bool enable = false;
  if(!tinyxml_tools::loadParameter(enable, kinematicHandle, "enable")) { return false; }
  enableInequalityConstraints_[foothold_generator::KinematicFeasibility] = enable;
  if(!tinyxml_tools::loadParameter(maxLegExtensionSquared_, kinematicHandle, "max_leg_extension", 0.52)) { return false; }
  maxLegExtensionSquared_ *= maxLegExtensionSquared_;
  if(!tinyxml_tools::loadParameter(numPolygonEdgesForBoundPolygon_, kinematicHandle, "num_of_edges", 8u)) { return false; }

  // Collision avoidance constraints.
  TiXmlHandle collisionHandle = handle;
  if(!tinyxml_tools::getChildHandle(collisionHandle, inequalityHandle, "CollisionAvoidance")) { return false; }
  if(!tinyxml_tools::loadParameter(enable, collisionHandle, "enable")) { return false; }
  enableInequalityConstraints_[foothold_generator::CollisionAvoidance] = enable;
  if(!tinyxml_tools::loadParameter(safetyMarginCollisionAvoidance_, collisionHandle, "safety_margin", 0.01)) { return false; }
  if(!tinyxml_tools::loadParameter(numPolygonEdgesForCollisionCone_, collisionHandle, "num_of_edges", 8u)) { return false; }
  if(!tinyxml_tools::loadParameter(closeCollisionCone_, collisionHandle, "closed_cone")) { return false; }

  return true;
}

bool FootholdGeneratorOptimizedConstraint::initialize(double dt) {
  if (!Base::initialize(dt)) { return false; }

  // Footprint of previous optimization.
  for (const auto& leg : wholeBody_.getLegs()) {
    positionWorldToPreviousFootholdInWorldFrame_[leg->getId()] =
        leg->getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
  }

  // Circle constraints are turned to rectangles with a certain number of vertices.
  const double yawAngleBound = 2.0*M_PI/( static_cast<double>(numPolygonEdgesForBoundPolygon_) );
  RotationMatrixBound_ << cos(yawAngleBound), -sin(yawAngleBound),
                          sin(yawAngleBound),  cos(yawAngleBound);

  const double yawAngleCollisionCone = 2.0*M_PI/( static_cast<double>(numPolygonEdgesForCollisionCone_) );
  RotationMatrixCollisionCone_ << cos(yawAngleCollisionCone), -sin(yawAngleCollisionCone),
                                  sin(yawAngleCollisionCone),  cos(yawAngleCollisionCone);

  return true;
}

bool FootholdGeneratorOptimizedConstraint::addVariablesToLog() {
  return true;
}

bool FootholdGeneratorOptimizedConstraint::doPreComputations(const foothold_generator::FootholdPlan& plan) {
  return constructCollistionLines(plan);
}

bool FootholdGeneratorOptimizedConstraint::compute(foothold_generator::FootholdPlan& plan) {
  // Update previous solution with previous realized solution.
  positionWorldToPreviousFootholdInWorldFrame_ = plan.getPreviousRealizedSolution();

  if(!doPreComputations(plan)) {
    MELO_WARN_STREAM("[FootholdGeneratorOptimizedConstraint::computeFootprint] Failed to perform pre-computations!");
    return false;
  }

  // Optimize each leg individually.
  double cost;
  for (unsigned int legId=0u; legId<plan.getNumLegs(); ++legId) {

    // Set up optimization problem.
    isCollisionConeConstraintActive_ = false;
    if(!setUpQP(legId, plan)) { return false; }

    // Solve QP.
    solutionFootholdInPlaneFrame_.getParams().setZero(solutionDimension_);
    if (!qpSolver_->minimize(optimizationProblemQP_.get(), solutionFootholdInPlaneFrame_, cost)) {
      MELO_WARN_STREAM("[FootholdGeneratorOptimizedConstraint::computeFootprint] Failed to solve optimization at legId " << legId << ".");
      return false;
    }

    // Check if constraints are violated.
    isCollisionConeConstraintActive_ = (
        enableInequalityConstraints_[foothold_generator::CollisionAvoidance] &&
        isPositionInCollisionAvoidanceCone(legId, static_cast<Position2d>(solutionFootholdInPlaneFrame_.getParams()), criticalNeughborLegId_, plan)
    );

    // If leg is in collision cone, compute a new optimization including associated collision constraints.
    if(isCollisionConeConstraintActive_) {
      double minCost = std::numeric_limits<double>::max();
      numopt_common::ParameterizationIdentity bestSolutionFootholdInPlaneFrame;

      // For each line in the collision cone, solve a separate optimization with that line constraint active.
      for (activeLineId_=0u; activeLineId_<collisionLinesInPlaneFrame_[legId][criticalNeughborLegId_].size(); ++activeLineId_) {

        // Set up optimization problem.
        if(!setUpQP(legId, plan)) { return false; }

        if(!qpSolver_->minimize(optimizationProblemQP_.get(), bestSolutionFootholdInPlaneFrame, cost)) {
           MELO_WARN_STREAM("[FootholdGeneratorOptimizedConstraint::computeFootprint] Failed to solve optimization at legId " << legId <<
               " and lineId " << activeLineId_ << ". Collision leg was " << criticalNeughborLegId_ << ".");
           return false;
         }

        // Seek for minimum over all line constraints.
        if (cost < minCost) {
          minCost                       = cost;
          solutionFootholdInPlaneFrame_ = bestSolutionFootholdInPlaneFrame;
        }
      } // end for lineId
    } // end if isCollision

   // Store solution.
   const Position positionWorldToDesiredFootholdOnGroundInWorldFrame = getPositionWorldToDesiredFootholdInWorldFrame(plan);
    plan.setPositionWorldToFootholdInWorldFrame(positionWorldToDesiredFootholdOnGroundInWorldFrame, legId);
    positionWorldToPreviousFootholdInWorldFrame_[legId] = positionWorldToDesiredFootholdOnGroundInWorldFrame;
  } // end for legId


  // store polygon constraints for visualization
  plan.setBoundedPolygonsInPlaneFrame(boundPolygonsInPlaneFrame_);
  plan.setCollisionLinesInPlaneFrame(collisionLinesInPlaneFrame_);

  return true;
}

bool FootholdGeneratorOptimizedConstraint::setupConstraintProperties(unsigned int legId ,const foothold_generator::FootholdPlan& plan) {
  if (!isCollisionConeConstraintActive_) {
    numOfInequalityConstraints_ = 0u;

    // Add feasibility constraints.
    if (enableInequalityConstraints_[foothold_generator::KinematicFeasibility]) {
      numOfInequalityConstraints_ += numPolygonEdgesForBoundPolygon_;
    }

    // Reset
    costFunctionHessian_.setZero(solutionDimension_, solutionDimension_);
    costFunctionLinearTerm_.setZero(solutionDimension_);

    inequalityConstraintJacobian_.resize(numOfInequalityConstraints_, solutionDimension_);
    inequalityConstraintMinValues_.resize(numOfInequalityConstraints_);
  }

  else if (activeLineId_ == 0) {
    // Add collision line constraints.
    numOfInequalityConstraints_ += 1u;
    inequalityConstraintJacobian_.conservativeResize(numOfInequalityConstraints_, solutionDimension_);
    inequalityConstraintMinValues_.conservativeResize(numOfInequalityConstraints_);
  }

  return true;
}

bool FootholdGeneratorOptimizedConstraint::setupCostFunction(unsigned int legId, const foothold_generator::FootholdPlan& plan) {
  // To be specified in the derived class...
  return true;
}

bool FootholdGeneratorOptimizedConstraint::setupInequalityConstraints(unsigned int legId, const foothold_generator::FootholdPlan& plan) {
  inequalityConstraintsCounter_ = 0u;

  if(!addBoundedPolygonConstraints(legId, activeLineId_, plan)) {
    MELO_WARN_STREAM("[FootholdGeneratorOptimizedConstraint::addBoundedPolygonConstraints] Failed!");
    return false;
  }

  if (!addCollisionAvoidanceConstraints(legId, activeLineId_, criticalNeughborLegId_, plan)) {
    MELO_WARN_STREAM("[FootholdGeneratorOptimizedConstraint::addCollisionAvoidanceConstraints] Failed!");
    return false;
  }

  if (inequalityConstraintsCounter_ != numOfInequalityConstraints_) {
    MELO_WARN_STREAM("[FootholdGeneratorOptimizedConstraint::setupInequalityConstraints] Wrong number of inequality constraints.");
    return false;
  }

  return true;
}

bool FootholdGeneratorOptimizedConstraint::setupEqualityConstraints(unsigned int legId, const foothold_generator::FootholdPlan& plan) {
  // To be specified in the derived class...
  return true;
}

bool FootholdGeneratorOptimizedConstraint::addBoundedPolygonConstraints(unsigned int legId, unsigned int lineId, const foothold_generator::FootholdPlan& plan) {
  /*
   * Center of the constraint polygon is the tight. The size of the polygon defines
   * the kinematic constraints of the leg extension.
   */

  if (!enableInequalityConstraints_[foothold_generator::KinematicFeasibility]) {
    return true;
  }

  if (isCollisionConeConstraintActive_) {
    inequalityConstraintsCounter_ += numPolygonEdgesForBoundPolygon_;
    return true;
  }

  bool success = true;

  // height above ground
  const double  distanceTerrainToHipSquared = boost::math::pow<2>(plan.getPositionZPlaneToDesiredThighHeightInPlaneFrame(legId));

  // Check if the ground terrain is kinematically reachable.
  if (maxLegExtensionSquared_ <= distanceTerrainToHipSquared) {
    MELO_WARN_STREAM("[FootholdGeneratorOptimizedConstraint::setupInequalityConstraints] Max leg extension is "
        << maxLegExtensionSquared_ << " is smaller than hip height " << distanceTerrainToHipSquared << " for leg " << legId <<".");
    return false;
  }
  const double maxDistance = std::sqrt(maxLegExtensionSquared_ - distanceTerrainToHipSquared);

  // Center of the polygon constraints: thigh projected onto the virtual frame along plane normal.
  const Position2d positionPlaneToPolygonCenterInPlaneFrame = plan.getVirtualPlaneFrame().getPosePlaneToWorld().inverseTransform(
      plan.getPositionWorldToThighInWorldFrame(legId)).toImplementation().head<2>();

  // Set up constraint polygon. The polygon vertices lay on the virtual plane.
  robot_utils::geometry::Polygon boundPolygon(
      positionPlaneToPolygonCenterInPlaneFrame,
      numPolygonEdgesForBoundPolygon_,
      maxDistance,
      RotationMatrixBound_);
  success &= boundPolygon.updateLineCoefficients();

  // Store polygon.
  boundPolygonsInPlaneFrame_[legId] = boundPolygon;

  // Set up constraints.
  for (const auto& lineConstraint : boundPolygon.getLineCoefficients()) {
    inequalityConstraintJacobian_.block<1, 2>(inequalityConstraintsCounter_, 0) << lineConstraint[zmp::lineCoeffA], lineConstraint[zmp::lineCoeffB];
    inequalityConstraintMinValues_(inequalityConstraintsCounter_) = -lineConstraint[zmp::lineCoeffC];
    ++inequalityConstraintsCounter_;
  }

  return success;
}

bool FootholdGeneratorOptimizedConstraint::constructCollistionLines(const foothold_generator::FootholdPlan& plan) {

  if (!enableInequalityConstraints_[foothold_generator::CollisionAvoidance]) {
    return true;
  }

  bool success                 = true;
  const bool isOddPolygonEdges = (numPolygonEdgesForCollisionCone_%2 != 0);
  const double scaling         = 1.0 / (safetyMarginCollisionAvoidance_*safetyMarginCollisionAvoidance_);

  collisionLinesInPlaneFrame_.resize(plan.getNumLegs());
  const Pose& posePlaneToWorld = plan.getVirtualPlaneFrame().getPosePlaneToWorld();
  std::vector<std::vector<robot_utils::geometry::Line>> collisionLines(plan.getNumLegs());
  robot_utils::geometry::Polygon::VertexList vertices(2u);

  for (unsigned int legId=0u; legId<plan.getNumLegs(); ++legId) {
    // Current foothold location.
    Position positionPlaneToEndEffectorInPlaneFrame = posePlaneToWorld.inverseTransform(
        plan.getEndEffectorStateMeasured(legId).getPositionWorldToEndEffectorInWorldFrame());

    // Find collision lines for each neighbour leg.
    for (unsigned int legIdNeighbor=0u; legIdNeighbor<plan.getNumLegs(); ++legIdNeighbor) {
      collisionLines[legIdNeighbor].clear();
      if (legId == legIdNeighbor) { continue; }

      // Neighbour foothold location.
      const Position positionPlaneToNeighborEndeffectorInPlaneFrame = posePlaneToWorld.inverseTransform(
          plan.getEndEffectorStateMeasured(legIdNeighbor).getPositionWorldToEndEffectorInWorldFrame());

      // Compute vector pointing from neighbour to current foothold.
      const Eigen::Vector2d polygonCenterToVertex0 = (
            positionPlaneToEndEffectorInPlaneFrame -
            positionPlaneToNeighborEndeffectorInPlaneFrame
          ).toImplementation().head<2>().normalized() * safetyMarginCollisionAvoidance_;


      // Rotate the Cone.
      Eigen::Vector2d polygonCenterToVertex = polygonCenterToVertex0;
      if (isOddPolygonEdges) {
        const double yawAngle = -M_PI/( static_cast<double>(numPolygonEdgesForCollisionCone_) );
        Eigen::Matrix2d  RotationMatrixCollisionConeHalf;
        RotationMatrixCollisionConeHalf <<  cos(yawAngle), -sin(yawAngle),
                                            sin(yawAngle),  cos(yawAngle);
        polygonCenterToVertex = RotationMatrixCollisionConeHalf*polygonCenterToVertex;
      }

      bool lock = false;
      for (unsigned int id=0u; id<numPolygonEdgesForCollisionCone_; ++id) {
        // Unlock the line constraint if the angle between the current and the neutral axis becomes smaller than 90°
        if (!closeCollisionCone_ && lock && polygonCenterToVertex0.dot(polygonCenterToVertex)*scaling > 1e-8) {
          lock = false;
        }

        // Start at the center of the cone.
        vertices[0] = positionPlaneToNeighborEndeffectorInPlaneFrame.toImplementation().head<2>();
        vertices[1] = positionPlaneToNeighborEndeffectorInPlaneFrame.toImplementation().head<2>();

        // Add offset.
        vertices[0] += polygonCenterToVertex;
        polygonCenterToVertex = RotationMatrixCollisionCone_*polygonCenterToVertex;
        vertices[1] += polygonCenterToVertex;

        // Lock the line constraint if the angle between the current and the neutral axis becomes  larger than 90°.
        if (!closeCollisionCone_ && !lock && polygonCenterToVertex0.dot(polygonCenterToVertex)*scaling <= 1e-8) {
          lock = true;
        }

        // Define line constraint (notice that the order is counterclock wise by construction).
        if (!lock) {
          robot_utils::geometry::Line coneConstraint(vertices, robot_utils::geometry::VertexOrder::CounterClockWise);
          coneConstraint.updateLineCoefficients();

          // Only add polygon if the feasibility region becomes not zero!
          if (!enableInequalityConstraints_[foothold_generator::KinematicFeasibility] ||
              coneConstraint.isPolygonVertexOnRHSOfLine(boundPolygonsInPlaneFrame_[legId])) {
            collisionLines[legIdNeighbor].push_back(coneConstraint);
          }
        }
      }
    }

    // Store (for visualization).
    collisionLinesInPlaneFrame_[legId] = collisionLines;
  }

  return success;
}

bool FootholdGeneratorOptimizedConstraint::isPositionInCollisionAvoidanceCone(
    unsigned int legId,
    const Position2d& positionPlaneToPointInPlaneFrame,
    unsigned int& criticalNeughborLegId,
    const foothold_generator::FootholdPlan& plan) {

  for (unsigned int legIdNeighbor=0u; legIdNeighbor<plan.getNumLegs(); ++legIdNeighbor) {
    if (legId == legIdNeighbor) { continue; }
    bool isPointInCollisionCone = true;

    for (const auto& line : collisionLinesInPlaneFrame_[legId][legIdNeighbor]) {
      if(!line.isPointInLHSPlane(positionPlaneToPointInPlaneFrame)) {
        isPointInCollisionCone = false;
        break;
      }
    }

    if (isPointInCollisionCone) {
      criticalNeughborLegId = legIdNeighbor;
      return true;
    }
  }
  return false;
}

bool FootholdGeneratorOptimizedConstraint::addCollisionAvoidanceConstraints(
    unsigned int legId,
    unsigned int lineId,
    unsigned int legIdNeighbor,
    const foothold_generator::FootholdPlan& plan) {
  /*
   * Ensure that legs do not crash.
   * For each neighbor leg, we define a cone where the foothold should not be placed in. Since this constraints is
   * non convex, it cannot be solved with a convex solver and needs a workaround. We solve the optimization for each
   * line constraint of that cone and seek over the global minimum of the cost function.
   */

  if (!enableInequalityConstraints_[foothold_generator::CollisionAvoidance] || !isCollisionConeConstraintActive_) {
    return true;
  }

  // Extract line coefficients
  const auto& lineCoefficients = collisionLinesInPlaneFrame_[legId][legIdNeighbor][lineId].getLineCoefficients();

  // Add constraints (Note: foothold needs to be on the RHS of the line constraint).
  inequalityConstraintJacobian_.block<1, 2>(inequalityConstraintsCounter_, 0) << -lineCoefficients.front()[zmp::lineCoeffA], -lineCoefficients.front()[zmp::lineCoeffB];
  inequalityConstraintMinValues_(inequalityConstraintsCounter_) = lineCoefficients.front()[zmp::lineCoeffC];
  inequalityConstraintsCounter_ += 1;

  return true;
}



Position FootholdGeneratorOptimizedConstraint::getPositionWorldToDesiredFootholdInWorldFrame(const foothold_generator::FootholdPlan& plan) const {
  // Extract solution in plane frame, transform to world frame and map to the local terrain.
  return plan.getVirtualPlaneFrame().getPosePlaneToWorld().transform(
    Position(
      solutionFootholdInPlaneFrame_.getParams().x(),
      solutionFootholdInPlaneFrame_.getParams().y(),
      0.0)
    );
}

bool FootholdGeneratorOptimizedConstraint::setUpQP(unsigned int legId, const foothold_generator::FootholdPlan& plan) {
  if(!setupConstraintProperties(legId, plan)) {
    MELO_WARN_STREAM("[ FootholdGeneratorOptimizedConstraint::setUpQP] Failed to set up constraint properties!");
    return false;
  }

  if(!setupCostFunction(legId, plan)) {
    MELO_WARN_STREAM("[ FootholdGeneratorOptimizedConstraint::setUpQP] Failed to set up cost function!");
    return false;
  }

  if(!setupEqualityConstraints(legId, plan)) {
    MELO_WARN_STREAM("[ FootholdGeneratorOptimizedConstraint::setUpQP] Failed to set up equality constraints!");
    return false;
  }

  if(!setupInequalityConstraints(legId, plan)) {
    MELO_WARN_STREAM("[ FootholdGeneratorOptimizedConstraint::setUpQP] Failed to set up inequality constraints!");
    return false;
  }

  optimizationProblemQP_->setOptimizationMatrices(
      costFunctionHessian_.sparseView(),
      costFunctionLinearTerm_,
      equalityConstraintJacobian_.sparseView(),
      inequalityConstraintJacobian_.sparseView(),
      equalityConstraintTargetValues_,
      inequalityConstraintMinValues_);
  return true;
}

} /* namespace loco */
