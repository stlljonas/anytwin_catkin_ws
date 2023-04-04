/*!
 * @author  Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief   Implementation of swing trajectory generator for WBC test controller
 * @date    Jul 15, 2019
 */

// anymal_ctrl_test_whole_body_control
#include "anymal_ctrl_test_whole_body_control/swing_trajectory/SwingTrajectoryTestWholeBodyController.hpp"

// robot_utils
#include <robot_utils/schedule/ProfileRamp.hpp>
#include <robot_utils/schedule/ProfileSinusoid.hpp>
#include <robot_utils/schedule/ProfileStep.hpp>

namespace anymal_ctrl_test_whole_body_control {

SwingTrajectoryTestWholeBodyController::SwingTrajectoryTestWholeBodyController(loco_anymal::WholeBodyAnymal& wholeBody)
    : wholeBody_(wholeBody),
      time_(0.0),
      rampDuration_(std::string{"Test WBC - Swing : Ramp duration [s]"}, 3.0, 0.1, 5.0),
      sinusoidFrequency_(std::string{"Test WBC - Swing : Sinusoid freq [Hz]"}, 1.0, 0.0, 3.0),
      xAmplitude_(std::string{"Test WBC - Swing : X swing amplitude [m]"}, 0.05, 0.0, 0.5),
      yAmplitude_(std::string{"Test WBC - Swing : Y swing amplitude [m]"}, 0.05, 0.0, 0.5),
      zAmplitude_(std::string{"Test WBC - Swing : Z swing amplitude [m]"}, 0.05, 0.0, 0.5),
      sinusoidDuration_(20.0)  // clang-format off
{  // clang-format on
  mirror_[0] = loco::Position(1.0, 1.0, 1.0);
  mirror_[1] = loco::Position(1.0, -1.0, 1.0);
  mirror_[2] = loco::Position(-1.0, 1.0, 1.0);
  mirror_[3] = loco::Position(-1.0, -1.0, 1.0);

  // Add empty profile at start.
  for (auto leg : wholeBody_.getLegs()) {
    schedule_[leg->getId()].addProfile(new robot_utils::ProfileStep<loco::Position>(loco::Position(), 0.0));
  }
  addParametersToHandler("");
}

bool SwingTrajectoryTestWholeBodyController::advance(double dt) {
  for (auto leg : *wholeBody_.getLegsPtr()) {
    if (schedule_[leg->getId()].getDuration() > 0.0) {
      if (time_ < schedule_[leg->getId()].getDuration()) {
        setEEAndJointReferencesFromTrajectory(leg, time_);
      } else {
        setEEAndJointReferencesFromTrajectory(leg, schedule_[leg->getId()].getDuration());
        isScheduleFinished_ = true;
      }
    } else {
      // Set current joint values as desired ones
      leg->getLimbStateDesiredPtr()->setJointPositions(leg->getLimbStateMeasured().getJointPositions());
      leg->getLimbStateDesiredPtr()->setJointVelocities(leg->getLimbStateMeasured().getJointVelocities());
    }
  }

  time_ += dt;
  return true;
}

void SwingTrajectoryTestWholeBodyController::setEEAndJointReferencesFromTrajectory(loco::LegBase* leg, double time) {
  // Sample desired foot trajectory
  loco::Position desPositionHipToFootInBaseFrame = schedule_[leg->getId()].getValue(time);
  loco::LinearVelocity desVelocityHipToFootInBaseFrame = loco::LinearVelocity(schedule_[leg->getId()].getValueFirstDerivative(time));
  loco::LinearAcceleration desAccelerationHipToFootInBaseFrame =
      loco::LinearAcceleration(schedule_[leg->getId()].getValueSecondDerivative(time));

  // Compute and set desired joint targets
  loco::JointPositions desJointPositions = leg->getEndEffectorPtr()->getJointPositionsFromPositionBaseToEndEffectorInBaseFrame(
      desPositionHipToFootInBaseFrame + leg->getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame());
  loco::JointVelocities desJointVelocities =
      leg->getEndEffectorPtr()->getJointVelocitiesFromLinearVelocityBaseToEndEffectorInBaseFrame(desVelocityHipToFootInBaseFrame);
  // TODO(paco): implement method to get joint accelerations from EE acc in loco.
  // JointAccelerations desJointAccelerations = leg->getEndEffector().MethodToImplement
  leg->getLimbStateDesiredPtr()->setJointPositions(desJointPositions);
  leg->getLimbStateDesiredPtr()->setJointVelocities(desJointVelocities);
  // leg->getLimbStateDesiredPtr()->setJointAccelerations(desJointAccelerations);

  // Set desired end-effector targets, assuming base is at rest
  loco::Position desPositionHipToFootInWorldFrame =
      wholeBody_.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame() +
      wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase().inverseRotate(
          desPositionHipToFootInBaseFrame + leg->getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame());
  leg->getEndEffectorPtr()->getStateDesiredPtr()->setPositionWorldToEndEffectorInWorldFrame(desPositionHipToFootInWorldFrame);
  loco::LinearVelocity desVelocityHipToFootInWorldFrame =
      wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase().inverseRotate(desVelocityHipToFootInBaseFrame);
  leg->getEndEffectorPtr()->getStateDesiredPtr()->setLinearVelocityEndEffectorInWorldFrame(desVelocityHipToFootInWorldFrame);
  loco::LinearAcceleration desAccelerationHipToFootInWorldFrame =
      wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase().inverseRotate(desAccelerationHipToFootInBaseFrame);
  leg->getEndEffectorPtr()->getStateDesiredPtr()->setLinearAccelerationEndEffectorInWorldFrame(desAccelerationHipToFootInWorldFrame);
}

bool SwingTrajectoryTestWholeBodyController::loadParameters(const TiXmlHandle& handle) {
  if (!(ModuleBase::loadParameters(handle))) {
    return false;
  }
  // Get offset handle
  TiXmlHandle swingTrajHandle = handle;
  if (!tinyxml_tools::getChildHandle(swingTrajHandle, handle, "SwingTrajectoryTestWBC")) {
    return false;
  }
  // Get offset handle
  TiXmlHandle hipToFootOffsetHandle = handle;
  if (!tinyxml_tools::getChildHandle(hipToFootOffsetHandle, swingTrajHandle, "HipToFootPositionOffset")) {
    return false;
  }

  // Load offset components
  if (!tinyxml_tools::loadParameter(positionOffsetHipToFoot_.x(), hipToFootOffsetHandle, "x", 0.0)) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(positionOffsetHipToFoot_.y(), hipToFootOffsetHandle, "y", 0.0)) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(positionOffsetHipToFoot_.z(), hipToFootOffsetHandle, "z", 0.0)) {
    return false;
  }

  return true;
}

void SwingTrajectoryTestWholeBodyController::clearTrajectory() {
  time_ = 0.0;
  isScheduleFinished_ = false;

  for (loco::LegBase* leg : *wholeBody_.getLegsPtr()) {
    // Clear schedule and add null trajectory.
    schedule_[leg->getId()].clear();
    schedule_[leg->getId()].addProfile(new robot_utils::ProfileStep<loco::Position>(loco::Position(), 0.0));

    // Set leg to support strategy, default when not in swing
    leg->getLimbStrategyPtr()->setLimbStrategyEnum(loco::LimbStrategyEnum::Support);
  }
}
void SwingTrajectoryTestWholeBodyController::computeTrajectory() {
  time_ = 0.0;
  isScheduleFinished_ = false;

  for (loco::LegBase* leg : *wholeBody_.getLegsPtr()) {
    // Generate foot trajectory according to parameters
    loco::Position startPositionHipToFootInBaseFrame = leg->getEndEffector().getStateMeasured().getPositionBaseToEndEffectorInBaseFrame() -
                                                       leg->getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame();
    loco::Position amplitude = loco::Position(mirror_[leg->getId()].toImplementation().cwiseProduct(
        Eigen::Vector3d(xAmplitude_.getValue(), yAmplitude_.getValue(), zAmplitude_.getValue())));
    // The sinusoidal trajectory starts at a peak, to avoid a step input at the velocity level
    loco::Position positionOffsetHipToFoot(
        mirror_[leg->getId()].toImplementation().cwiseProduct(positionOffsetHipToFoot_.toImplementation()));
    loco::Position endPositionHipToFootInBaseFrame = positionOffsetHipToFoot + amplitude;

    schedule_[leg->getId()].addProfile(new robot_utils::ProfileRamp<loco::Position>(
        startPositionHipToFootInBaseFrame, endPositionHipToFootInBaseFrame, rampDuration_.getValue()));
    schedule_[leg->getId()].addProfile(new robot_utils::ProfileSinusoid<loco::Position>(
        amplitude, positionOffsetHipToFoot, sinusoidFrequency_.getValue(), M_PI_2, sinusoidDuration_));

    // Set leg to swing strategy
    leg->getLimbStrategyPtr()->setLimbStrategyEnum(loco::LimbStrategyEnum::Motion);
  }
}

bool SwingTrajectoryTestWholeBodyController::addParametersToHandler(const std::string& ns) {
  parameter_handler::handler->addParam(xAmplitude_);
  parameter_handler::handler->addParam(yAmplitude_);
  parameter_handler::handler->addParam(zAmplitude_);
  parameter_handler::handler->addParam(sinusoidFrequency_);
  parameter_handler::handler->addParam(rampDuration_);
  return ModuleBase::addParametersToHandler(ns);
}

bool SwingTrajectoryTestWholeBodyController::isScheduleFinished() const {
  return isScheduleFinished_;
}

}  // namespace anymal_ctrl_test_whole_body_control
