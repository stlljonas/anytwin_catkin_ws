/*
 * MotionPlan.cpp
 *
 *  Created on: 03.26, 2017
 *      Author: Fabian Jenelten
 */

// zmp_optimizer
#include "zmp_optimizer/MotionPlan.hpp"

// loco
#include "loco/gait_pattern/contact_schedule_anymal.hpp"

// logger
#include "message_logger/message_logger.hpp"

// robot_utils
#include <robot_utils/physical_definitions.hpp>

namespace zmp {

MotionPlan::MotionPlan()
    : MotionPlanLinearAngular(),
      supportPolygonsInPlaneFrame_(),
      pathRegularizerInPlaneFrame_(),
      comStateInPlaneFrame_(),
      comFinalBox_(),
      zmpParams_(),
      isFirstSupportPolygonNew_(false),
      orientationWorldToControl_(),
      wholeBodyMass_(0.0),
      torsoInertiaTensorInBaseFrame_(),
      gravityVectorInWorldFrame_(Eigen::Vector3d(0.0, 0.0, -robot_utils::physical_definitions::getAbsoluteGravityAcceleration())),
      vectorsTargetToLimbThighInBaseFrame_(Eigen::Vector3d::Zero()),
      footholdTrackingOffsetIndicator_(0.0),
      positionPlaneToEndEffectorInPlaneFrame_(),
      nominalLegExtension_(0.45),
      gravityFactor_(1.0) {}

bool MotionPlan::initialize(double wholeBodyMass, const Eigen::Matrix3d& torsoInertiaTensorInBaseFrame,
                            const motion_generation::anymalLegsVector& vectorsTargetToLimbThighInBaseFrame, double nominalLegExtension) {
  if (!MotionPlanLinearAngular::initialize()) {
    return false;
  }
  supportPolygonsInPlaneFrame_.clear();
  if (!pathRegularizerInPlaneFrame_.initialize()) {
    return false;
  }
  if (!comStateInPlaneFrame_.initialize()) {
    return false;
  }
  zmpParams_.clear();
  wholeBodyMass_ = wholeBodyMass;
  torsoInertiaTensorInBaseFrame_ = torsoInertiaTensorInBaseFrame;
  vectorsTargetToLimbThighInBaseFrame_ = vectorsTargetToLimbThighInBaseFrame;
  nominalLegExtension_ = nominalLegExtension;
  return true;
}

bool MotionPlan::initialize(const MotionPlan& motionPlan) {
  if (!initialize(motionPlan.getWholeBodyMass(), motionPlan.getTorsoInertiaTensorInBaseFrame(),
                  motionPlan.getVectorsTargetToLimbThighInBaseFrame(), motionPlan.getNominalLegExtension())) {
    return false;
  }
  comStateInPlaneFrame_.copyLineSearchOptions(motionPlan.getComStateInPlaneFrame());
  return true;
}

void MotionPlan::setLineSearchOptions(const zmp::LineSearchOptions& lineSearchOptions) {
  comStateInPlaneFrame_.setLineSearchOptions(lineSearchOptions);
}

unsigned int MotionPlan::computeActivePolygonIdAtTime(double trajectoryTime) const {
  if (supportPolygonsInPlaneFrame_.size() == 0) {
    return 0u;
  }

  double accumulatedPolygonTime = 0.0;
  for (auto polygonId = 0u; polygonId < supportPolygonsInPlaneFrame_.size(); ++polygonId) {
    const double accumulatedNextPolygonTime = accumulatedPolygonTime + supportPolygonsInPlaneFrame_[polygonId].getDuration();
    if (trajectoryTime >= accumulatedPolygonTime && trajectoryTime < accumulatedNextPolygonTime) {
      return polygonId;
    }
    accumulatedPolygonTime = accumulatedNextPolygonTime;
  }

  return supportPolygonsInPlaneFrame_.size() - 1;
}

bool MotionPlan::getPlaneToZmpPositionInPlaneFrame(Eigen::Vector3d& positionPlaneToZmpInPlaneFrame,
                                                   const loco::Position& positionPlaneToComInPlaneFrame,
                                                   const loco::LinearAcceleration& linearAccelerationComInPlaneFrame,
                                                   const loco::EulerAnglesZyx& eulerAnglesZyxBaseToPlane,
                                                   const loco::EulerAnglesZyxDiff& eulerAnglesZyx_dot,
                                                   const loco::EulerAnglesZyxDiff& eulerAnglesZyx_ddot) const {
  /****************************************************************
   * For 2d Case we assume that:                                  *
   *  > z component of COG is constant.                           *
   *  > n is aligned with z-axis of the virtual plane frame.      *
   ****************************************************************/
  const Eigen::Vector3d gravityVectorInPlaneFrame =
      virtualPlaneFrame_.getPosePlaneToWorld().getRotation().inverseRotate(gravityVectorInWorldFrame_);
  if (std_utils::consistsOfEnums(optimizationDofs_, zmp::optimizationXYDofs)) {
    const double zBar = positionPlaneToComInPlaneFrame.z() / gravityVectorInPlaneFrame.z();
    const double gravityRatioX = gravityVectorInPlaneFrame.x() / gravityVectorInPlaneFrame.z();
    const double gravityRatioY = gravityVectorInPlaneFrame.y() / gravityVectorInPlaneFrame.z();
    positionPlaneToZmpInPlaneFrame.x() = positionPlaneToComInPlaneFrame.x() + zBar * linearAccelerationComInPlaneFrame.x() -
                                         gravityRatioX * positionPlaneToComInPlaneFrame.z();
    positionPlaneToZmpInPlaneFrame.y() = positionPlaneToComInPlaneFrame.y() + zBar * linearAccelerationComInPlaneFrame.y() -
                                         gravityRatioY * positionPlaneToComInPlaneFrame.z();
    positionPlaneToZmpInPlaneFrame.z() = 0.0;
    return true;
  }
  /****************************************************************/

  /*****************************************************************
   * 3d Case                                                       *
   *****************************************************************/
  const Eigen::Vector3d& planeNormalInPlaneFrame = virtualPlaneFrame_.getPlaneNormalInPlaneFrame().toImplementation();
  const Eigen::Vector3d forceInPlaneFrame =
      wholeBodyMass_ * (gravityVectorInPlaneFrame - linearAccelerationComInPlaneFrame.toImplementation());
  Eigen::Vector3d angularMomentInPlaneFrame = Eigen::Vector3d::Zero();
  if (zmp::containsTranslation(optimizationDofs_)) {
    angularMomentInPlaneFrame += positionPlaneToComInPlaneFrame.toImplementation().cross(forceInPlaneFrame);
  }
  /****************************************************************/

  /*****************************************************************
   * 6d Case                                                       *
   * Equation 3.13 In Thomas report                                *
   *****************************************************************/
  if (zmp::containsRotation(optimizationDofs_)) {
    // Compute transformation matrices.
    const auto rates2Vel = zmp::getMapEulerAnglesZyxToAngularVelocityInBodyFrame(eulerAnglesZyxBaseToPlane);
    const auto rates2Vel_dot =
        zmp::getMatrixAngularRatesZyxToAngularVelocityInBodyFrameTimeDerivative(eulerAnglesZyxBaseToPlane, eulerAnglesZyx_dot);

    // Compute angular velocity and acceleration in plane frame.
    const Eigen::Vector3d angularVelocityBaseInBaseFrame = rates2Vel * eulerAnglesZyx_dot.toImplementation();
    const Eigen::Vector3d angularAccelerationBaseInBaseFrame =
        (rates2Vel * eulerAnglesZyx_ddot.toImplementation() + rates2Vel_dot * eulerAnglesZyx_dot.toImplementation());

    // Compute rate of angular momentum in base frame.
    // L = I*omega -> Ldot = (L)dot + omega x L
    const Eigen::Vector3d LInBaseFrame = torsoInertiaTensorInBaseFrame_ * angularVelocityBaseInBaseFrame;
    const Eigen::Vector3d LDotInBaseFrame =
        torsoInertiaTensorInBaseFrame_ * angularAccelerationBaseInBaseFrame + angularVelocityBaseInBaseFrame.cross(LInBaseFrame);

    // Rotate back to plane frame.
    const Eigen::Vector3d LDotInPlaneFrame = eulerAnglesZyxBaseToPlane.rotate(LDotInBaseFrame);

    // Add to angular moment.
    angularMomentInPlaneFrame -= LDotInPlaneFrame;
  }
  /****************************************************************/

  /*
   * Compute zmp.
   * > forceProjection<0: pull contact
   * > forceProjection>0: push contact, zmp not defined
   * > forceProjection=0: full flight phase, zmp not defined
   */
  const double forceProjection = planeNormalInPlaneFrame.dot(forceInPlaneFrame);
  if (forceProjection < -1e-6) {
    positionPlaneToZmpInPlaneFrame = planeNormalInPlaneFrame.cross(angularMomentInPlaneFrame) / forceProjection;
  } else {
    positionPlaneToZmpInPlaneFrame = virtualPlaneFrame_.getPosePlaneToWorld()
                                         .inverseTransform(virtualPlaneFrame_.projectOntoVirtualPlaneAlongPlaneNormalInWorldFrame(
                                             virtualPlaneFrame_.getPosePlaneToWorld().transform(positionPlaneToComInPlaneFrame)))
                                         .toImplementation();
  }

  return true;
}

bool MotionPlan::getPlaneToZmpPositionInPlaneFrame(loco::Position& positionPlaneToZmpInPlaneFrame) const {
  return getPlaneToZmpPositionInPlaneFrame(
      positionPlaneToZmpInPlaneFrame.toImplementation(), comStateInPlaneFrame_.getPositionPlaneToComInPlaneFrame(),
      comStateInPlaneFrame_.getLinearAccelerationComInPlaneFrame(), comStateInPlaneFrame_.getAnglesZyxBaseToPlane(),
      comStateInPlaneFrame_.getEulerRatesZyxBaseInPlaneFrame(), comStateInPlaneFrame_.getEulerAccelerationZyxBaseInPlaneFrame());
}

bool MotionPlan::getPlaneToZmpPositionInPlaneFrameAtTime(loco::Position& positionPlaneToZmpInPlaneFrame, double tk) const {
  return getPlaneToZmpPositionInPlaneFrame(
      positionPlaneToZmpInPlaneFrame.toImplementation(), comStateInPlaneFrame_.getPositionPlaneToComInPlaneFrameAtTime(tk),
      comStateInPlaneFrame_.getLinearAccelerationComInPlaneFrameAtTime(tk), comStateInPlaneFrame_.getAnglesZyxBaseToPlaneAtTime(tk),
      comStateInPlaneFrame_.getEulerRatesZyxBaseInPlaneFrameAtTime(tk),
      comStateInPlaneFrame_.getEulerAccelerationZyxBaseInPlaneFrameAtTime(tk));
}

bool MotionPlan::getWorldToZmpPositionInWorldFrame(loco::Position& positionWorldToZmpInWorldFrame) const {
  if (!getPlaneToZmpPositionInPlaneFrame(positionWorldToZmpInWorldFrame)) {
    return false;
  }
  positionWorldToZmpInWorldFrame = virtualPlaneFrame_.getPosePlaneToWorld().transform(positionWorldToZmpInWorldFrame);
  return true;
}

bool MotionPlan::getWorldToZmpPositionInWorldFrameAtTime(loco::Position& positionWorldToZmpInWorldFrame, double tk) const {
  if (!getPlaneToZmpPositionInPlaneFrameAtTime(positionWorldToZmpInWorldFrame, tk)) {
    return false;
  }
  positionWorldToZmpInWorldFrame = virtualPlaneFrame_.getPosePlaneToWorld().transform(positionWorldToZmpInWorldFrame);
  return true;
}

const std::vector<SupportPolygon>& MotionPlan::getSupportPolygonsInPlaneFrame() const {
  return supportPolygonsInPlaneFrame_;
}

std::vector<SupportPolygon>* MotionPlan::getSupportPolygonsInPlaneFramePtr() {
  return &supportPolygonsInPlaneFrame_;
}

const PathRegularizer& MotionPlan::getPathRegularizerInPlaneFrame() const {
  return pathRegularizerInPlaneFrame_;
}

PathRegularizer* MotionPlan::getPathRegularizerInPlaneFramePtr() {
  return &pathRegularizerInPlaneFrame_;
}

const ComStateHandler& MotionPlan::getComStateInPlaneFrame() const {
  return comStateInPlaneFrame_;
}

ComStateHandler* MotionPlan::getComStateInPlaneFramePtr() {
  return &comStateInPlaneFrame_;
}

const Box& MotionPlan::getComFinalBox() const {
  return comFinalBox_;
}

loco::ZmpParameterHandler* MotionPlan::getZmpParamsPtr() {
  return &zmpParams_;
}

const loco::ZmpParameterHandler& MotionPlan::getZmpParams() const {
  return zmpParams_;
}

void MotionPlan::setVirtualPlaneFrame(const VirtualPlaneFrame& virtualPlaneFrame) {
  virtualPlaneFrame_.copy(virtualPlaneFrame);
}

void MotionPlan::setSupportPolygonsInPlaneFrame(const std::vector<SupportPolygon>& supportPolygonsInPlaneFrame,
                                                bool isFirstSupportPolygonNew) {
  supportPolygonsInPlaneFrame_ = supportPolygonsInPlaneFrame;
  optimizationHorizon_ = 0.0;
  for (const auto& supportPolygon : supportPolygonsInPlaneFrame_) {
    optimizationHorizon_ += supportPolygon.getDuration();
  }
  isFirstSupportPolygonNew_ = isFirstSupportPolygonNew;
}

bool MotionPlan::setParameters(const loco::ZmpParameterHandler& defaultZmpParams) {
  // We need only active and desired gait parameters (hence we do not copy all the others)
  return zmpParams_.copyActiveAndDesiredGaitParams(defaultZmpParams);
}

void MotionPlan::setComFinalBox(Box&& box) {
  comFinalBox_ = std::move(box);
}

void MotionPlan::copyMotionPlan(const MotionPlan& motionPlan) {
  // If not yet initialize, do it.
  if (!isInitialized_) {
    initialize(motionPlan);
  }
  terminationState_ = motionPlan.getTerminationState();

  // For obtaining the COM and ZMP state we need...
  comStateInPlaneFrame_ = motionPlan.getComStateInPlaneFrame();
  virtualPlaneFrame_ = motionPlan.getVirtualPlaneFrame();
}

bool MotionPlan::copyComState(const MotionPlan& previousMotionPlan, double dt) {
  comStateInPlaneFrame_ = previousMotionPlan.getComStateInPlaneFrame();
  posePreviousPlaneToWorld_ = previousMotionPlan.getVirtualPlaneFrame().getPosePlaneToWorld();
  if (!comStateInPlaneFrame_.advance(dt, false)) {
    MELO_WARN_STREAM("[MotionPlan::copyComState] Failed to advance Com state.")
    return false;
  }
  return true;
}

bool MotionPlan::isFirstSupportPolygonNew() const {
  return isFirstSupportPolygonNew_;
}

loco::Position MotionPlan::getPositionWorldToComInWorldFrame() const {
  return virtualPlaneFrame_.getPosePlaneToWorld().transform(comStateInPlaneFrame_.getPositionPlaneToComInPlaneFrame());
}

loco::LinearVelocity MotionPlan::getLinearVelocityComInWorldFrame() const {
  return virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(comStateInPlaneFrame_.getLinearVelocityComInPlaneFrame());
}

loco::LinearAcceleration MotionPlan::getLinearAccelerationComInWorldFrame() const {
  return virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(comStateInPlaneFrame_.getLinearAccelerationComInPlaneFrame());
}

loco::Position MotionPlan::getPositionWorldToComInWorldFrameAtTime(const double tk) const {
  return virtualPlaneFrame_.getPosePlaneToWorld().transform(comStateInPlaneFrame_.getPositionPlaneToComInPlaneFrameAtTime(tk));
}

loco::LinearVelocity MotionPlan::getLinearVelocityComInWorldFrameAtTime(const double tk) const {
  return virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(comStateInPlaneFrame_.getLinearVelocityComInPlaneFrameAtTime(tk));
}

loco::LinearAcceleration MotionPlan::getLinearAccelerationComInWorldFrameAtTime(const double tk) const {
  return virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(
      comStateInPlaneFrame_.getLinearAccelerationComInPlaneFrameAtTime(tk));
}

loco::Position MotionPlan::getPositionWorldToPathInWorldFrameAtTime(const double tk) const {
  return virtualPlaneFrame_.getPosePlaneToWorld().transform(pathRegularizerInPlaneFrame_.getPositionPlaneToComInPlaneFrameAtTime(tk));
}

loco::LinearVelocity MotionPlan::getVelocityPathInWorldFrameAtTime(const double tk) const {
  return virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(
      pathRegularizerInPlaneFrame_.getLinearVelocityComInPlaneFrameAtTime(tk));
}

loco::EulerAnglesZyx MotionPlan::getAnglesZyxWorldToBase() const {
  const auto orientationBaseToWorld =
      loco::EulerAnglesZyx(virtualPlaneFrame_.getPosePlaneToWorld().getRotation()) * comStateInPlaneFrame_.getAnglesZyxBaseToPlane();
  return orientationBaseToWorld.inverted().getUnique();
}

loco::LocalAngularVelocity MotionPlan::getAngularVelocityBaseInWorldFrame() const {
  return virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(comStateInPlaneFrame_.getAngularVelocityBaseInPlaneFrame());
}

loco::AngularAcceleration MotionPlan::getAngularAccelerationBaseInWorldFrame() const {
  return virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(comStateInPlaneFrame_.getAngularAccelerationBaseInPlaneFrame());
}

loco::EulerAnglesZyx MotionPlan::getAnglesZyxWorldToBaseAtTime(const double tk) const {
  const auto orientationBaseToWorld = loco::EulerAnglesZyx(virtualPlaneFrame_.getPosePlaneToWorld().getRotation()) *
                                      comStateInPlaneFrame_.getAnglesZyxBaseToPlaneAtTime(tk);
  return orientationBaseToWorld.inverted().getUnique();
}

loco::LocalAngularVelocity MotionPlan::getAngularVelocityBaseInWorldFrameAtTime(const double tk) const {
  return virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(comStateInPlaneFrame_.getAngularVelocityBaseInPlaneFrameAtTime(tk));
}

loco::AngularAcceleration MotionPlan::getAngularAccelerationBaseInWorldFrameAtTime(const double tk) const {
  return virtualPlaneFrame_.getPosePlaneToWorld().getRotation().rotate(
      comStateInPlaneFrame_.getAngularAccelerationBaseInPlaneFrameAtTime(tk));
}

loco::EulerAnglesZyx MotionPlan::getAnglesZyxWorldToPathAtTime(const double tk) const {
  const auto orientationPathToWorld = loco::EulerAnglesZyx(virtualPlaneFrame_.getPosePlaneToWorld().getRotation()) *
                                      pathRegularizerInPlaneFrame_.getAnglesZyxBaseToPlaneAtTime(tk);
  return orientationPathToWorld.inverted().getUnique();
}

bool MotionPlan::checkConstraints(double tol) const {
  print();
  if (!didOptimizationSucceeded()) {
    MELO_INFO_STREAM("[MotionPlan::checkConstraints] Optimization was not successful. Cannot check for constraints.")
    return false;
  }

  const double sampleTime = optimizationHorizon_ / 100.0;

  // Check initial and final constraints.
  for (const auto& dim : optimizationDofs_) {
    for (const auto derivative : std_utils::enum_iterator<zmp::Derivative>()) {
      // May skip checking.
      bool checkInitialConstraints = true;
      bool checkFinalConstraints = true;
      if (derivative == Derivative::Second) {
        checkInitialConstraints = false;
        checkFinalConstraints = false;
      }
      if (!enforceHardFinalConstraints_) {
        checkFinalConstraints = false;
      }

      const double initialError =
          initialRobotStateInPlaneFrame_[derivative][dim] - comStateInPlaneFrame_.getComStateInPlaneFrameAtTime(0.0, dim, derivative);
      const double finalError =
          finalRobotStateInPlaneFrame_[derivative][dim] - comStateInPlaneFrame_.getFinalComStateInPlaneFrame(dim, derivative);

      if (checkInitialConstraints && std::fabs(initialError) > tol) {
        MELO_INFO_STREAM("[MotionPlan::checkConstraints] Initial state tolerance not satisfied (dim = "
                         << (int)dim << ", derivative = " << (int)derivative << ", error = " << initialError << ").")
        return false;
      }

      if (checkFinalConstraints && std::fabs(finalError) > tol) {
        MELO_INFO_STREAM("[MotionPlan::checkConstraints] Final state tolerance not satisfied (dim = "
                         << (int)dim << ", derivative = " << (int)derivative << ", error = " << finalError << ").")
        return false;
      }
    }
  }

  // Check zmp constraints.
  loco::Position positionPlaneToZmpInPlaneFrame;
  for (double tk = sampleTime; tk < optimizationHorizon_; tk += sampleTime) {
    if (!getPlaneToZmpPositionInPlaneFrameAtTime(positionPlaneToZmpInPlaneFrame, tk)) {
      return false;
    }

    const unsigned int polygonId = computeActivePolygonIdAtTime(tk);
    if (!supportPolygonsInPlaneFrame_[polygonId].getPolygon().isPointInPolygon(positionPlaneToZmpInPlaneFrame.toImplementation().head<2>(),
                                                                               tol)) {
      MELO_INFO_STREAM("[MotionPlan::checkConstraints] Zmp is not in polygon at time " << tk << " and polygon " << polygonId << ".")
    }
  }

  return true;
}

bool MotionPlan::setContainerTime(double containerTime) {
  if (!comStateInPlaneFrame_.setContainerTime(containerTime) || !pathRegularizerInPlaneFrame_.setContainerTime(containerTime)) {
    MELO_WARN_STREAM("[CMotionPlan::setContainerTime] Failed to set container time. Set to zero.")
    return false;
  }
  return true;
}

void MotionPlan::print() const {
  std::stringstream msg;
  msg << "Print motion plan." << std::endl;

  // List of support polygons.
  msg << std::left << "List of support polygons:" << std::endl;
  msg << std::setw(10) << "id" << std::setw(10) << "d[s]" << std::setw(10) << "type" << std::setw(10) << "vertices" << std::endl;
  unsigned int polId = 0;
  for (const auto& polygon : supportPolygonsInPlaneFrame_) {
    msg << std::setw(10) << std::to_string(polId) << std::setw(10) << std::to_string(polygon.getDuration()) << std::setw(10)
        << zmp::PolygonTypeNamesMap[polygon.getPolygonType()] << std::setw(10) << std::to_string(polygon.getPolygon().getNumVertices())
        << std::endl;
    polId++;
  }
  msg << std::endl;

  msg << "Timing:" << std::endl;
  msg << std::setw(30) << "opt horizon [s]" << std::to_string(optimizationHorizon_) << std::endl;
  msg << std::setw(30) << "is first polygon new" << std::to_string(isFirstSupportPolygonNew_) << std::endl;
  msg << std::setw(30) << "enforce hard final" << std::to_string(enforceHardFinalConstraints_) << std::endl;
  msg << std::setw(30) << "skip hard initial accel" << std::to_string(skipHardInitialAccelConstraints_) << std::endl;
  msg << std::setw(30) << "polygon id at switch" << std::to_string(zmpParams_.getPolygonIdAtSwitch()) << std::endl;
  msg << std::setw(30) << "optimization dofs";
  for (const auto& dof : optimizationDofs_) {
    msg << std::to_string(static_cast<int>(dof)) << " ";
  }
  msg << std::endl << std::endl;

  // Initial State.
  msg << std::setw(10) << "Initial:" << std::setw(15) << "x" << std::setw(15) << "y" << std::setw(15) << "z" << std::endl;
  msg << std::setw(10) << "pos" << std::setw(15) << getPlaneToInitialPositionInPlaneFrame().x() << std::setw(15)
      << getPlaneToInitialPositionInPlaneFrame().y() << std::setw(15) << getPlaneToInitialPositionInPlaneFrame().z() << std::endl;
  msg << std::setw(10) << "vel" << std::setw(15) << getInitialVelocityInPlaneFrame().x() << std::setw(15)
      << getInitialVelocityInPlaneFrame().y() << std::setw(15) << getInitialVelocityInPlaneFrame().z() << std::endl;
  msg << std::setw(10) << "accel" << std::setw(15) << getInitialAccelerationInPlaneFrame().x() << std::setw(15)
      << getInitialAccelerationInPlaneFrame().y() << std::setw(15) << getInitialAccelerationInPlaneFrame().z() << std::endl
      << std::endl;
  // Final state.
  msg << std::setw(10) << "Final:" << std::setw(15) << "x" << std::setw(15) << "y" << std::setw(15) << "z" << std::endl;
  msg << std::setw(10) << "pos" << std::setw(15) << getPlaneToFinalPositionInPlaneFrame().x() << std::setw(15)
      << getPlaneToFinalPositionInPlaneFrame().y() << std::setw(15) << getPlaneToFinalPositionInPlaneFrame().z() << std::endl;
  msg << std::setw(10) << "vel" << std::setw(15) << getFinalVelocityInPlaneFrame().x() << std::setw(15)
      << getFinalVelocityInPlaneFrame().y() << std::setw(15) << getFinalVelocityInPlaneFrame().z() << std::endl;
  msg << std::setw(10) << "accel" << std::setw(15) << getFinalAccelerationInPlaneFrame().x() << std::setw(15)
      << getFinalAccelerationInPlaneFrame().y() << std::setw(15) << getFinalAccelerationInPlaneFrame().z() << std::endl
      << std::endl;

  MELO_INFO_STREAM(msg.str())
}

bool MotionPlan::checkMotionPlan() const {
  bool success = true;

  for (const auto& dim : optimizationDofs_) {
    for (const auto derivative : std_utils::enum_iterator<zmp::Derivative>()) {
      if (std::isnan(initialRobotStateInPlaneFrame_[derivative][dim])) {
        MELO_WARN_STREAM("[MotionPlan::checkMotionPlan] Initial robot state is NaN for dim " << zmp::cogDimMap[dim] << " and derivative "
                                                                                             << zmp::derivativeMap[derivative] << ".")
        success = false;
      }

      if (std::isnan(finalRobotStateInPlaneFrame_[derivative][dim])) {
        MELO_WARN_STREAM("[MotionPlan::checkMotionPlan] Final robot state is NaN for dim " << zmp::cogDimMap[dim] << " and derivative "
                                                                                           << zmp::derivativeMap[derivative] << ".")
        success = false;
      }
    }
  }

  if (optimizationHorizon_ <= 0.0) {
    MELO_WARN_STREAM("[MotionPlan::checkMotionPlan] Optimization horizon is negative.")
    success = false;
  }

  if (std::isnan(virtualPlaneFrame_.getPosePlaneToWorld().getPosition().squaredNorm())) {
    MELO_WARN_STREAM("[MotionPlan::checkMotionPlan] VPF origon is NaN.")
    success = false;
  }

  if (std::isnan(virtualPlaneFrame_.getPosePlaneToWorld().getRotation().toImplementation().norm())) {
    MELO_WARN_STREAM("[MotionPlan::checkMotionPlan] VPF orientation is NaN.")
    success = false;
  }

  if (std::isnan(virtualPlaneFrame_.getPlaneNormalInWorldFrame().squaredNorm())) {
    MELO_WARN_STREAM("[MotionPlan::checkMotionPlan] VPF normal is NaN.")
    success = false;
  }

  if (!pathRegularizerInPlaneFrame_.checkTrajectoryStateHandler()) {
    MELO_WARN_STREAM("[MotionPlan::checkMotionPlan] Path regulaizer appears to be invalid.")
    success = false;
  }

  if (!comStateInPlaneFrame_.checkTrajectoryStateHandler()) {
    MELO_WARN_STREAM("[MotionPlan::checkMotionPlan] Previous solution appears to be invalid.")
    success = false;
  }

  // Safety (in case that a frame jumps).
  const auto orientationControlToDesiredBase =
      (getAnglesZyxWorldToBase() * loco::EulerAnglesZyx(orientationWorldToControl_).inverted()).getUnique();
  constexpr double tol = M_PI / 10.0;
  if (std::fabs(orientationControlToDesiredBase.roll()) > tol || std::fabs(orientationControlToDesiredBase.pitch()) > tol ||
      std::fabs(orientationControlToDesiredBase.yaw()) > tol) {
    MELO_WARN_STREAM("[MotionPlan::checkMotionPlan] Angle larger than tol " << tol << "\n")
    success = false;
  }

  return success;
}

const loco::RotationQuaternion& MotionPlan::getOrientationWorldToControl() const noexcept {
  return orientationWorldToControl_;
}

void MotionPlan::setOrientations(const loco::RotationQuaternion& orientationWorldToControl) {
  orientationWorldToControl_ = orientationWorldToControl;
}

double MotionPlan::getWholeBodyMass() const noexcept {
  return wholeBodyMass_;
}

Eigen::Matrix3d MotionPlan::getTorsoInertiaTensorInBaseFrame() const noexcept {
  return torsoInertiaTensorInBaseFrame_;
}

const motion_generation::anymalLegsVector& MotionPlan::getVectorsTargetToLimbThighInBaseFrame() const {
  return vectorsTargetToLimbThighInBaseFrame_;
}

void MotionPlan::setFootholdTrackingOffsetIndicator(double footholdTrackingOffsetIndicator, loco::contact_schedule::LegEnumAnymal legEnum) {
  footholdTrackingOffsetIndicator_[legEnum] = footholdTrackingOffsetIndicator;
}

const motion_generation::anymalLegsDouble& MotionPlan::getFootholdTrackingOffsetIndicator() const {
  return footholdTrackingOffsetIndicator_;
}

void MotionPlan::setPositionPlaneToEndEffectorInPlaneFrame(const Eigen::Vector3d& positionPlaneToEndEffectorInPlaneFrame,
                                                           const loco::contact_schedule::LegEnumAnymal& legEnum) {
  positionPlaneToEndEffectorInPlaneFrame_[legEnum] = positionPlaneToEndEffectorInPlaneFrame;
}

const motion_generation::anymalLegsVector& MotionPlan::getPositionPlaneToEndEffectorInPlaneFrame() const {
  return positionPlaneToEndEffectorInPlaneFrame_;
}

double MotionPlan::getNominalLegExtension() const {
  return nominalLegExtension_;
}

void MotionPlan::setGravityFactor(double gravityFactor) {
  gravityFactor_ = gravityFactor;
}

double MotionPlan::getGravityFactor() const noexcept {
  return gravityFactor_;
}

void MotionPlan::addVariablesToLog(const std::string& ns) const {
  signal_logger::add(optimizationHorizon_, "optimizationHorizon", ns, "s");
}

} /* namespace zmp */
