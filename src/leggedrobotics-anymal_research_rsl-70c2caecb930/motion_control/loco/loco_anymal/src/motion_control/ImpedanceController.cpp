/*!
* @file     ImpedanceController.cpp
* @author   Péter Fankhauser, Georg Wiedebach
* @date     Mar 8, 2016
*/


// loco
#include <loco_anymal/common/LegsAnymal.hpp>
#include <loco/common/loco_common.hpp>
#include <loco/common/topology_conversions.hpp>

// loco anymal
#include <loco_anymal/motion_control/ImpedanceController.hpp>

// robot utils
#include <robot_utils/math/math.hpp>

// message logger
#include <message_logger/message_logger.hpp>

using namespace message_logger::color;

namespace loco {

ImpedanceController::ImpedanceController(
    loco_anymal::WholeBodyAnymal& wholebody,
    anymal_model::AnymalModel& anymalModel,
    anymal_model::AnymalModel& anymalModelDesired)
    : MotionControllerBase(wholebody),
      anymalModel_(anymalModel),
      anymalModelDesired_(anymalModelDesired),
      timeSinceInitialization_(0.0),
      interpolationDuration_(1.0),
      supportLegControlMode_(ControlMode::MODE_FREEZE),
      nonSupportLegControlMode_(ControlMode::MODE_FREEZE)
{

}

bool ImpedanceController::loadParameters(const TiXmlHandle& handle)
{
  TiXmlElement* pElem;

  // read the desired default control mode for support and non-support mode
  TiXmlHandle motionControlHandle(handle.FirstChild("MotionController").FirstChild("DefaultControlModeForLeg"));
  pElem = motionControlHandle.Element();
  if (pElem == nullptr) {
    MELO_WARN("[ImpedanceController::loadParameters] Could not find MotionController::DefaultControlModeForLeg");
    return false;
  } else {
    std::string supportControlMode = "";
    std::string nonSupportControlMode = "";

    if (pElem->QueryStringAttribute("supportMode", &supportControlMode)!=TIXML_SUCCESS) {
      MELO_WARN("[ImpedanceController::loadParameters] Could not find MotionController::DefaultControlModeForLeg::supportMode");
      return false;
    } else {
      supportLegControlMode_ = topology_conversions::getControlModeEnumFromControlModeString(supportControlMode);
      MELO_INFO_STREAM("[ImpedanceController::loadParameters] Support leg control mode is set to: " << supportControlMode);
    }

    if (pElem->QueryStringAttribute("nonSupportMode", &nonSupportControlMode)!=TIXML_SUCCESS) {
      MELO_WARN("[ImpedanceController::loadParameters] Could not find MotionController::DefaultControlModeForLeg::nonSupportMode");
      return false;
    } else {
      nonSupportLegControlMode_ = topology_conversions::getControlModeEnumFromControlModeString(nonSupportControlMode);
      MELO_INFO_STREAM("[ImpedanceController::loadParameters] Non support leg control mode is set to: " << nonSupportControlMode)
    }
  }

   TiXmlHandle hFPS(handle.FirstChild("ImpedanceController"));
   pElem = hFPS.Element();
   if (pElem == nullptr) {
     printf("Could not find ImpedanceController\n");
     std::cout << magenta << "[ImpedanceController/loadParameters] "
               << red << "Warning: "
               << blue << "Could not find section 'ImpedanceController'"
               << def << std::endl;
   }

   // Load RegainContact parameter
   TiXmlElement* child = hFPS.FirstChild().ToElement();
   for(; child; child=child->NextSiblingElement()) {
     // If "RegainContact" element is found, try to read values
     if (child->ValueStr().compare("Interpolation") == 0) {
       if (child->QueryDoubleAttribute("duration", &interpolationDuration_) != TIXML_SUCCESS) {
         std::cout << magenta << "[ImpedanceController/loadParameters] "
                   << red << "Warning: "
                   << blue << "Could not find parameter 'duration' in section 'Interpolation'. Setting speed to default value: "
                   << red << interpolationDuration_
                   << def << std::endl;
       }
       else {
         std::cout << magenta << "[ImpedanceController/loadParameters] "
                   << blue << "Interpolation duration is set to: "
                   << red << interpolationDuration_
                   << def << std::endl;
       }
     }
     else {
       std::cout << magenta << "[ImpedanceController/loadParameters] "
                 << red << "Warning: "
                 << blue << "Could not find section 'Interpolation'. Using default values."
                 << def << std::endl;
     }
   }
  return true;
}

bool ImpedanceController::addVariablesToLog(const std::string& /* ns */) const
{
  return true;
}

bool ImpedanceController::initialize(double /* dt */)
{
  timeSinceInitialization_ = 0.0;
  selectionMatrix_ = anymalModel_.getActuatorSelectionMatrix();

  return true;
}

bool ImpedanceController::advance(double dt)
{
  timeSinceInitialization_ += dt;
  bool success = true;

  // Prepare.
  std::map<LegBase*, bool> supportLegs;
  std::map<LegBase*, bool> legLoadActive;
  for (auto leg: *wholeBody_.getLegsPtr()) {
    // We are using the desired support legs to compute the load distribution.
    // This stops a leg from compensating for too much of the mass.
    supportLegs[leg] = leg->getContactSchedule().shouldBeGrounded();
    legLoadActive[leg] = supportLegs[leg] && leg->getLoadFactor() < 1.0;
  }

  // Torques for complete support.
  Eigen::VectorXd jointTorquesCompleteSupport(12);
  if (!computeGravityCompensation(supportLegs, jointTorquesCompleteSupport)) return false;

  // Interpolation.
  Eigen::VectorXd jointTorques(12);
  jointTorques.setZero();
  unsigned int nLegsWithLegLoadActive = 0;
  for (auto leg: *wholeBody_.getLegsPtr()) {
    if (legLoadActive[leg]) {
      ++nLegsWithLegLoadActive;
      auto othersSupportLegs = supportLegs;
      othersSupportLegs[leg] = false;
      Eigen::VectorXd othersJointTorques(12);
      if (!computeGravityCompensation(othersSupportLegs, othersJointTorques)) return false;
      Eigen::VectorXd newJointTorques(12);
      newJointTorques = (1.0 - leg->getLoadFactor()) * othersJointTorques
          + leg->getLoadFactor() * jointTorquesCompleteSupport;
      jointTorques = ((nLegsWithLegLoadActive - 1) * jointTorques + newJointTorques) / nLegsWithLegLoadActive;
    }
  }

  // No interpolation required.
  if (nLegsWithLegLoadActive == 0) jointTorques = jointTorquesCompleteSupport;

  // Write values to legs.
  for (auto leg: *wholeBody_.getLegsPtr()) {
    if ( (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
         (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant) ) {
      // Torque.
      loco::JointTorques jointTorquesForLeg(
          jointTorques.segment<AD::getNumDofLimb()>(3 * leg->getLimbUInt()));
      jointTorquesForLeg *= leg->getLoadFactor();
      leg->getLimbStateDesiredPtr()->setJointTorques(jointTorquesForLeg);

      // Position.
      setJointPositionsFromDesiredBase(leg);

      // Velocity.
      leg->getLimbStateDesiredPtr()->setJointVelocities(leg->getInitializedJointVelocities());

      // Mode.
      loco::JointControlModes desiredJointControlModes;
      desiredJointControlModes.setConstant(loco::ControlMode::MODE_JOINT_POSITION_VELOCITY_TORQUE);
      leg->getLimbStateDesiredPtr()->setJointControlModes(desiredJointControlModes);
    }
  }

  success &= setControlModeForLegs();

  return success;
}

void ImpedanceController::setJointPositionsFromDesiredBase(LegBase* leg)
{
  const Position& positionWorldToFootInWorldFrame = leg->getStateTouchDown()->getPositionWorldToFootInWorldFrame();

  // Measured state.
  const RotationQuaternion orientationWorldToMeasuredBase = anymalModel_.getState().getOrientationBaseToWorld().inverted();
  const Position& positionWorldToMeasuredBaseInWorldFrame = anymalModel_.getState().getPositionWorldToBaseInWorldFrame();
  const Position positionMeasuredBaseToFootInBaseFrame = orientationWorldToMeasuredBase.rotate(positionWorldToFootInWorldFrame-positionWorldToMeasuredBaseInWorldFrame);

  // Desired state.
  const RotationQuaternion orientationWorldToDesiredBase = anymalModelDesired_.getState().getOrientationBaseToWorld().inverted();
  const Position& positionWorldToDesiredBaseInWorldFrame = anymalModelDesired_.getState().getPositionWorldToBaseInWorldFrame();
  const Position positionDesiredBaseToFootInBaseFrame = orientationWorldToDesiredBase.rotate(positionWorldToFootInWorldFrame-positionWorldToDesiredBaseInWorldFrame);

  // Interpolation.
  const double timeSinceTouchdown = timeSinceInitialization_ - leg->getStateTouchDown()->stateChangedAtTime();
  double interpolationFactor = robot_utils::mapTo01Range(timeSinceTouchdown, 0, interpolationDuration_);
  if (timeSinceInitialization_ < interpolationDuration_) interpolationFactor = 1.0; // Stable start after initialization.
  const Position positionBaseToFootInBaseFrame = (1.0 - interpolationFactor) * positionMeasuredBaseToFootInBaseFrame
                                                  + interpolationFactor * positionDesiredBaseToFootInBaseFrame;

//  LegBase::JointPositions desiredJoints;
  Eigen::VectorXd desiredJoints;
  anymalModel_.getLimbJointPositionsFromLimbEnumIteratively(
      desiredJoints, positionBaseToFootInBaseFrame.vector(),
      AD::mapKeyIdToKeyEnum<AD::LimbEnum>(leg->getLimbUInt()));

  leg->getLimbStateDesiredPtr()->setJointPositions(desiredJoints);
}

bool ImpedanceController::computeGravityCompensation(
    const std::map<LegBase*, bool> supportLegs,
    Eigen::VectorXd& jointTorques) const
{
  // This code is adapted from the 'Whole Body Climbing' project by Georg Wiedebach.
  const unsigned int nDofs = anymalModel_.getDofCount();
  Eigen::VectorXd gravity = Eigen::VectorXd::Zero(nDofs);
  anymalModel_.getGravityTerms(gravity);

  // Build contact Jacobian.
  unsigned int nSupportPoints = 0;
  for (const auto leg : supportLegs) if (leg.second) ++nSupportPoints;
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3 * nSupportPoints, nDofs);
  int index = 0;
  for (auto leg: *wholeBody_.getLegsPtr()) {
    if (!supportLegs.at(leg)) continue;
    Eigen::MatrixXd contactJacobian = Eigen::MatrixXd::Zero(3, nDofs);
    anymalModel_.getJacobianTranslationWorldToPointOnBody(
        contactJacobian, Eigen::Vector3d::Zero(),
        AD::mapKeyIdToKeyEnum<AD::BranchEnum>(leg->getBranchUInt()),
        AD::BodyNodeEnum::FOOT,
        AD::CoordinateFrameEnum::WORLD);
    jacobian.block(3 * index, 0, 3, nDofs) = contactJacobian;
    ++index;
  }

  // Calculate null space projection matrix, such that P_F * jacobianTransposed == 0.
  const Eigen::MatrixXd jacobianTransposed = jacobian.transpose();
  unsigned int rank = jacobianTransposed.fullPivLu().rank();
  Eigen::MatrixXd Q = jacobianTransposed.householderQr().householderQ();
  Eigen::MatrixXd P_F = Q.block(0, rank, nDofs, nDofs - rank).transpose();

  // Test: Make sure that P_F * jacobianTransposed == 0.
  const Eigen::MatrixXd zero = P_F * jacobianTransposed;
  if (fabs(zero.maxCoeff()) > 1E-10) {
    MELO_WARN("ImpedanceController::computeGravityCompensation(): Computing gravity compensation failed.");
    return false;
  }

  // Calculate the joint torques such that
  // gravity + jacobianTransposed * F_S == selectionMatrix * tau.
  Eigen::MatrixXd PS = P_F * selectionMatrix_;
  Eigen::MatrixXd PS_inv;
  kindr::pseudoInverse(PS, PS_inv);
  jointTorques = PS_inv * P_F * gravity;
  return true;
}

bool ImpedanceController::setControlModeForLegs() {
  for (auto leg: *wholeBody_.getLegsPtr()) {
    setControlModeForLeg(leg, supportLegControlMode_, nonSupportLegControlMode_);
  }
  return true;
}

} /* namespace loco */
