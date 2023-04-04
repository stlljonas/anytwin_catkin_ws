/*!
 * @file     ComSupportControlStaticGait.cpp
 * @author   C. Dario Bellicoso
 * @date     Oct 7, 2014
 * @brief
 */

// loco
#include "loco/torso_control/ComSupportControlStaticGait.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// message logger
#include <message_logger/message_logger.hpp>

// signal logger
#include <signal_logger/signal_logger.hpp>

// eigen
#include <Eigen/LU>

// boost
#include <boost/thread.hpp>

// stl
#include <mutex>

namespace loco {

ComSupportControlStaticGait::ComSupportControlStaticGait(WholeBody& wholeBody, GaitPatternStaticGait& gaitPattern)
    : ComSupportControlBase(*wholeBody.getLegsPtr()),
      wholeBody_(wholeBody),
      torso_(*wholeBody.getTorsoPtr()),
      gaitPattern_(gaitPattern),
      didUpdateSafeTriangles_(false),
      plannedFootHolds_(legs_.size(), Position()),
      delta_(0.0),
      isInStandConfiguration_(true),
      updateMethod_(CogUpdateMethod::None),
      comTrajectoryPhase_(0.0),
      standTime_(0.0),
      didOptimizationSucceed_(true),
      nextSwingLegIdAtStance_(3),
      lastExecutedLegIdAtStance_(0) {
  // Reset Eigen variables
  comTarget_.setZero();
  comTargetPrevious_.setZero();

  feetConfigurationCurrent_.setZero();
  feetConfigurationNext_.setZero();
  feetConfigurationLastFullStance_.setZero();

  supportTriangleCurrent_.setZero();
  supportTriangleNext_.setZero();
  supportTriangleOverNext_.setZero();

  safeTriangleCurrent_.setZero();
  safeTriangleNext_.setZero();
  safeTriangleOverNext_.setZero();

  comInterpolationFunction_.clear();
  comInterpolationFunction_.addKnot(0, 0);
  comInterpolationFunction_.addKnot(0.7, 1);
}

bool ComSupportControlStaticGait::addVariablesToLog(bool updateLogger) {
  return true;
}

bool ComSupportControlStaticGait::initialize(double dt) {
  updateMethod_ = CogUpdateMethod::CogUpdateStd;

  setIsInStandConfiguration(true);

  comTarget_ = Pos2d(torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame().x(),
                     torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame().y());
  comTargetPrevious_ = comTarget_;

  // save home feet positions
  for (auto leg : legs_) {
    const Position& positionWorldToFootInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
    plannedFootHolds_[leg->getId()] = positionWorldToFootInWorldFrame;
    feetConfigurationCurrent_.col(leg->getId()) << positionWorldToFootInWorldFrame.x(), positionWorldToFootInWorldFrame.y();
    feetConfigurationNext_ = feetConfigurationCurrent_;
    feetConfigurationLastFullStance_ = feetConfigurationCurrent_;
  }

  //  updateSafeSupportTriangles();

  /*********************
   * Initialize spline *
   *********************/
  splineQuintic_.computeCoefficients(curves::SplineOptions(1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0));
  /*********************/

  positionWorldToDesiredCoMInWorldFrame_.setZero();
  for (auto leg : legs_) {
    positionWorldToDesiredCoMInWorldFrame_ += leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame() / legs_.size();
  }
  positionWorldToDesiredCoMInWorldFrameOld_ = positionWorldToDesiredCoMInWorldFrame_;
  linearVelocityDesiredBaseInWorldFrame_.setZero();

  standTime_ = 0.0;

  didOptimizationSucceed_ = true;

  return true;
}

/*! Loads the parameters from the XML object
 * @param hParameterSet   handle
 * @return  true if all parameters could be loaded
 */
bool ComSupportControlStaticGait::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle staticGaitHandle = handle;
  if (!tinyxml_tools::getChildHandle(staticGaitHandle, handle, "StaticGait")) {
    return false;
  }

  TiXmlHandle comControlHandle = handle;
  if (!tinyxml_tools::getChildHandle(comControlHandle, staticGaitHandle, "ComSupportControl")) {
    return false;
  }

  TiXmlHandle deltaHandle = handle;
  if (!tinyxml_tools::getChildHandle(deltaHandle, comControlHandle, "Delta")) {
    return false;
  }

  if (!tinyxml_tools::loadParameter(delta_, deltaHandle, "forward")) {
    return false;
  }

  return true;
}

void ComSupportControlStaticGait::setFootHold(int legId, const Position& footHold) {
  plannedFootHolds_.at(legId) = footHold;
}

const std::vector<Position>& ComSupportControlStaticGait::getFootHolds() const {
  return plannedFootHolds_;
}

const Position& ComSupportControlStaticGait::getPositionWorldToDesiredCoMInWorldFrame() const {
  return positionWorldToDesiredCoMInWorldFrame_;
}

void ComSupportControlStaticGait::updateSafeSupportTriangles() {
  // update current configuration
  for (auto leg : legs_) {
    const Position& positionWorldToFootInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
    feetConfigurationCurrent_.col(leg->getId()) << positionWorldToFootInWorldFrame.x(), positionWorldToFootInWorldFrame.y();
  }

  feetConfigurationNext_ = getNextStanceConfig(feetConfigurationCurrent_, gaitPattern_.getNextSwingLeg());

  int j;

  // Get support triangles
  j = 0;
  for (size_t k = 0; k < legs_.size(); k++) {
    if (k != gaitPattern_.getLastSwingLeg()) {
      supportTriangleCurrent_.col(j) = feetConfigurationCurrent_.col(k);
      j++;
    }
  }
  safeTriangleCurrent_ = getSafeTriangle(supportTriangleCurrent_);

  j = 0;
  for (size_t k = 0; k < legs_.size(); k++) {
    if (k != gaitPattern_.getNextSwingLeg()) {
      supportTriangleNext_.col(j) = feetConfigurationCurrent_.col(k);
      j++;
    }
  }
  safeTriangleNext_ = getSafeTriangle(supportTriangleNext_);

  j = 0;
  for (size_t k = 0; k < legs_.size(); k++) {
    if (k != gaitPattern_.getOverNextSwingLeg()) {
      supportTriangleOverNext_.col(j) = feetConfigurationNext_.col(k);
      j++;
    }
  }
  safeTriangleOverNext_ = getSafeTriangle(supportTriangleOverNext_);
}

void ComSupportControlStaticGait::updateCogPosition(bool forceShift) {
  switch (updateMethod_) {
    case (CogUpdateStd):
      updateCogPositionStd(forceShift);
      break;
    default:
      std::cout << "Undefined CoG update method!" << std::endl;
      break;
  }
}

void ComSupportControlStaticGait::updateCogPositionStd(bool forceShift) {
  const std::vector<int> diagonalSwingLegsLast = getDiagonalElements(gaitPattern_.getLastSwingLeg());
  const std::vector<int> diagonalSwingLegsNext = getDiagonalElements(gaitPattern_.getNextSwingLeg());
  const std::vector<int> diagonalSwingLegsOverNext = getDiagonalElements(gaitPattern_.getOverNextSwingLeg());

  Pos2d intersection;
  intersection.setZero();

  Line lineSafeLast, lineSafeNext, lineSafeOverNext;

  lineSafeLast << safeTriangleCurrent_.col(diagonalSwingLegsLast[0]), safeTriangleCurrent_.col(diagonalSwingLegsLast[1]);

  lineSafeNext << safeTriangleNext_.col(diagonalSwingLegsNext[0]), safeTriangleNext_.col(diagonalSwingLegsNext[1]);

  lineSafeOverNext << safeTriangleOverNext_.col(diagonalSwingLegsOverNext[0]), safeTriangleOverNext_.col(diagonalSwingLegsOverNext[1]);

  if (lineIntersect(lineSafeLast, lineSafeNext, intersection)) {
    comTargetPrevious_ = comTarget_;
    comTarget_ = intersection;
  } else if (lineIntersect(lineSafeNext, lineSafeOverNext, intersection)) {
    comTargetPrevious_ = comTarget_;
    comTarget_ = intersection;
  } else {
    MELO_WARN("ComSupportControlStaticGait::updateCogPositionStd() : No intersection between support triangles was found!");
    comTargetPrevious_ = comTarget_;
    comTarget_ = (safeTriangleNext_.col(0) + safeTriangleNext_.col(1) + safeTriangleNext_.col(2)) / 3.0;
  }
}

bool ComSupportControlStaticGait::setToInterpolated(const ComSupportControlBase& supportPolygon1,
                                                    const ComSupportControlBase& supportPolygon2, double t) {
  return false;
}

void ComSupportControlStaticGait::setIsInStandConfiguration(bool isInStandConfiguration) {
  isInStandConfiguration_ = isInStandConfiguration;

  if (!isInStandConfiguration) {
    positionWorldToDesiredCoMInWorldFrame_ = Position(torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame().x(),
                                                      torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame().y(), 0.0);
    linearVelocityDesiredBaseInWorldFrame_.setZero();
    /*********************************************************/

    updateSafeSupportTriangles();
    didUpdateSafeTriangles_ = true;
    updateCogPosition(/* forceShift = */ false);
  }
}

bool ComSupportControlStaticGait::isInStandConfiguration() const {
  return isInStandConfiguration_;
}

bool ComSupportControlStaticGait::advance(double dt) {
  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();

  if (gaitPattern_.isFullStancePhase() && !didUpdateSafeTriangles_ && !isInStandConfiguration_) {
    updateSafeSupportTriangles();

    for (auto leg : legs_) {
      const Position& positionWorldToFootInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
      feetConfigurationLastFullStance_.col(leg->getId()) << positionWorldToFootInWorldFrame.x(), positionWorldToFootInWorldFrame.y();
    }

    nextSwingLegIdAtStance_ = gaitPattern_.getIndexOfNextSwingLeg(gaitPattern_.getLastSwingLeg());
    lastExecutedLegIdAtStance_ = gaitPattern_.getLastSwingLeg();

    updateCogPosition(/* forceShit = */ false);

    // reset flag
    didUpdateSafeTriangles_ = true;
  }

  if (!gaitPattern_.isFullStancePhase() && didUpdateSafeTriangles_) {
    didUpdateSafeTriangles_ = false;
  }

  /*************************************
   * Set CoM linear interpolation time *
   *************************************/
  double stridePhase = torso_.getStridePhase();
  double comPhase = 1.0;

  constexpr double firstStancePhaseStart = 0.0;
  double firstStancePhaseEnd = gaitPattern_.getDiagonalStanceDuration();

  constexpr double secondStancePhaseStart = 0.5;
  double secondStancePhaseEnd = secondStancePhaseStart + gaitPattern_.getDiagonalStanceDuration();

  if (stridePhase >= firstStancePhaseStart && stridePhase <= firstStancePhaseEnd) {
    comPhase = robot_utils::mapTo01Range(stridePhase, firstStancePhaseStart, firstStancePhaseEnd);
  } else if (stridePhase >= secondStancePhaseStart && stridePhase <= secondStancePhaseEnd) {
    comPhase = robot_utils::mapTo01Range(stridePhase, secondStancePhaseStart, secondStancePhaseEnd);
  }

  comPhase = comInterpolationFunction_.evaluate_linear(comPhase);
  comTrajectoryPhase_ = comPhase;
  /*************************************/

  /****************************
   * Set CoM desired position *
   ****************************/
  if (isInStandConfiguration_) {
    if (standTime_ == 0.0) {
      //      comTargetOld_ << positionWorldToDesiredCoMInWorldFrame_.x(), positionWorldToDesiredCoMInWorldFrame_.y();
      comTargetPrevious_ << torso_.getDesiredState().getPositionWorldToBaseInWorldFrame().x(),
          torso_.getDesiredState().getPositionWorldToBaseInWorldFrame().y();
    }

    //--- Update target position
    Position positionWorldToCenterOfFootprintInWorldFrame;
    for (auto leg : legs_) {
      positionWorldToCenterOfFootprintInWorldFrame +=
          leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame() / legs_.size();
    }
    comTarget_ << positionWorldToCenterOfFootprintInWorldFrame(0), positionWorldToCenterOfFootprintInWorldFrame(1);
    //---

    const Position positionWorldToOldComTarget = Position(comTargetPrevious_[0], comTargetPrevious_[1], 0.0);
    const Position positionWorldToComTarget = Position(comTarget_[0], comTarget_[1], 0.0);
    standTime_ += dt;

    if (standTime_ <= 1.0) {
      positionWorldToDesiredCoMInWorldFrame_ = positionWorldToOldComTarget + splineQuintic_.getPositionAtTime(standTime_) *
                                                                                 (positionWorldToComTarget - positionWorldToOldComTarget);

      linearVelocityDesiredBaseInWorldFrame_ =
          LinearVelocity(splineQuintic_.getVelocityAtTime(standTime_) * (positionWorldToComTarget - positionWorldToOldComTarget));

      const LinearAcceleration linearAccelerationBaseInWorldFrame =
          LinearAcceleration(splineQuintic_.getAccelerationAtTime(standTime_) * (positionWorldToComTarget - positionWorldToOldComTarget));

      torso_.getDesiredStatePtr()->setLinearAccelerationTargetInControlFrame(
          orientationWorldToControl.rotate(linearAccelerationBaseInWorldFrame));
    } else {
      positionWorldToDesiredCoMInWorldFrame_ << comTarget_[0], comTarget_[1], 0.0;
      linearVelocityDesiredBaseInWorldFrame_.setZero();
      torso_.getDesiredStatePtr()->setLinearAccelerationTargetInControlFrame(LinearAcceleration());
      comTargetPrevious_ = comTarget_;
    }

  } else {
    standTime_ = 0.0;

    switch (updateMethod_) {
      /*
       * Update center of mass based on last, current and next support triangles
       */
      case (CogUpdateStd): {
        const Position positionWorldToOldComTarget = Position(comTargetPrevious_.x(), comTargetPrevious_.y(), 0.0);
        const Position positionWorldToComTarget = Position(comTarget_.x(), comTarget_.y(), 0.0);

        positionWorldToDesiredCoMInWorldFrame_ = positionWorldToOldComTarget + splineQuintic_.getPositionAtTime(comPhase) *
                                                                                   (positionWorldToComTarget - positionWorldToOldComTarget);
        linearVelocityDesiredBaseInWorldFrame_ =
            LinearVelocity(splineQuintic_.getVelocityAtTime(comPhase) * (positionWorldToComTarget - positionWorldToOldComTarget));

        const LinearAcceleration linearAccelerationBaseInWorldFrame(splineQuintic_.getAccelerationAtTime(comPhase) *
                                                                    (positionWorldToComTarget - positionWorldToOldComTarget));

        torso_.getDesiredStatePtr()->setLinearAccelerationTargetInControlFrame(
            orientationWorldToControl.rotate(linearAccelerationBaseInWorldFrame));
      } break;

      default: {
        std::cout << "Invalid update method in static com control." << std::endl;
        throw std::runtime_error("Could not update com position.");
      } break;
    }
  }
  /****************************/

  //  const double alfa = 0.1;
  //  positionWorldToDesiredCoMInWorldFrame_ = positionWorldToDesiredCoMInWorldFrame_*alfa +
  //  positionWorldToDesiredCoMInWorldFrameOld_*(1.0-alfa); positionWorldToDesiredCoMInWorldFrameOld_ =
  //  positionWorldToDesiredCoMInWorldFrame_;
  return didOptimizationSucceed_;
}

const WholeBody& ComSupportControlStaticGait::getWholeBody() const {
  return wholeBody_;
}

WholeBody* ComSupportControlStaticGait::getWholeBodyPtr() const {
  return &wholeBody_;
}

const TorsoBase& ComSupportControlStaticGait::getTorso() const {
  return torso_;
}

TorsoBase* ComSupportControlStaticGait::getTorsoPtr() const {
  return &torso_;
}

const ComSupportControlStaticGait::SupportTriangle& ComSupportControlStaticGait::getSupportTriangleCurrent() const {
  return supportTriangleCurrent_;
}

const ComSupportControlStaticGait::SupportTriangle& ComSupportControlStaticGait::getSupportTriangleNext() const {
  return supportTriangleNext_;
}

const ComSupportControlStaticGait::SupportTriangle& ComSupportControlStaticGait::getSupportTriangleOverNext() const {
  return supportTriangleOverNext_;
}

const ComSupportControlStaticGait::SupportTriangle& ComSupportControlStaticGait::getSafeTriangleCurrent() const {
  return safeTriangleCurrent_;
}

const ComSupportControlStaticGait::SupportTriangle& ComSupportControlStaticGait::getSafeTriangleNext() const {
  return safeTriangleNext_;
}

const ComSupportControlStaticGait::SupportTriangle& ComSupportControlStaticGait::getSafeTriangleOverNext() const {
  return safeTriangleOverNext_;
}

ComSupportControlStaticGait::FeetConfiguration ComSupportControlStaticGait::getNextStanceConfig(
    const FeetConfiguration& currentStanceConfig, int steppingFoot) {
  FeetConfiguration nextStanceConfig = currentStanceConfig;
  Pos2d footStep;
  footStep << plannedFootHolds_[steppingFoot].x(), plannedFootHolds_[steppingFoot].y();
  nextStanceConfig.col(steppingFoot) = footStep;

  return nextStanceConfig;
}

bool ComSupportControlStaticGait::lineIntersect(const Line& l1, const Line& l2, Pos2d& intersection) {
  const Pos2d l1_1 = l1.col(0);
  const Pos2d l1_2 = l1.col(1);

  const Pos2d l2_1 = l2.col(0);
  const Pos2d l2_2 = l2.col(1);

  // Check if line length is zero
  if ((l1_1 - l1_2).isZero() || (l2_1 - l2_2).isZero()) {
    return false;
  }

  Pos2d v1 = l1_2 - l1_1;
  v1 = v1 / v1.norm();

  Pos2d v2 = l2_2 - l2_1;
  v2 = v2 / v2.norm();

  // Check if v1 and v2 are parallel (matrix would not be invertible)
  Eigen::Matrix2d A;
  Pos2d x;
  x(0) = -1;
  x(1) = -1;
  A << -v1, v2;
  Eigen::FullPivLU<Eigen::Matrix2d> Apiv(A);
  if (Apiv.rank() == 2) {
    x = A.lu().solve(l1_1 - l2_1);
  }

  if (x(0) >= 0.0 && x(0) <= 1.0) {
    intersection << l1_1(0) + x(0) * v1(0), l1_1(1) + x(0) * v1(1);
    return true;
  } else {
    return false;
  }
}

ComSupportControlStaticGait::SupportTriangle ComSupportControlStaticGait::getSafeTriangle(const SupportTriangle& supportTriangle) {
  SupportTriangle safeTriangle;
  safeTriangle.setZero();

  Pos2d v1, v2;

  for (int k = 0; k < 3; k++) {
    int vertex1, vertex2;
    if (k == 0) {
      vertex1 = 1;
      vertex2 = 2;
    } else if (k == 1) {
      vertex1 = 2;
      vertex2 = 0;
    } else if (k == 2) {
      vertex1 = 0;
      vertex2 = 1;
    }

    v1 = supportTriangle.col(vertex1) - supportTriangle.col(k);
    v1 = v1 / v1.norm();
    v2 = supportTriangle.col(vertex2) - supportTriangle.col(k);
    v2 = v2 / v2.norm();

    double phiv1v2 = acos(v1.dot(v2));
    safeTriangle.col(k) = supportTriangle.col(k) + delta_ / sin(phiv1v2) * (v1 + v2);
  }

  return safeTriangle;
}

std::vector<int> ComSupportControlStaticGait::getDiagonalElements(int swingLeg) const {
  std::vector<int> diagonalSwingLegs(2);

  switch (swingLeg) {
    case (0):
      diagonalSwingLegs[0] = 0;
      diagonalSwingLegs[1] = 1;
      break;
    case (1):
      diagonalSwingLegs[0] = 0;
      diagonalSwingLegs[1] = 2;
      break;
    case (2):
      diagonalSwingLegs[0] = 0;
      diagonalSwingLegs[1] = 2;
      break;
    case (3):
      diagonalSwingLegs[0] = 1;
      diagonalSwingLegs[1] = 2;
      break;
    default:
      break;
  }

  return diagonalSwingLegs;
}

} /* namespace loco */
