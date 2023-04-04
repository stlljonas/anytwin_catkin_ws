/*!
 * @author  Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief   Declaration of ROS interface of WBC test controller class
 * @date    Jul 17, 2019
 */

#include <eigen_conversions/eigen_msg.h>

#include <any_state_estimator_msgs/ResetStateEstimator.h>
#include <anymal_ctrl_test_whole_body_control_msgs/MotionTrackingErrors.h>

#include "anymal_ctrl_test_whole_body_control_ros/TestWholeBodyControllerRos.hpp"

namespace anymal_ctrl_test_whole_body_control_ros {

TestWholeBodyControllerRos::TestWholeBodyControllerRos() : Base(), nodeHandle_(ros::NodeHandle("~")) {}

void TestWholeBodyControllerRos::initializeRosCommunication() {
  trackingErrorPublisher_ = nodeHandle_.advertise<anymal_ctrl_test_whole_body_control_msgs::MotionTrackingErrors>(
      "/anymal_ctrl_test_whole_body_control_ros/motion_tracking_errors", 1);
  resetEstimatorClient_ = nodeHandle_.serviceClient<any_state_estimator_msgs::ResetStateEstimator>("/state_estimator/reset");
}

void TestWholeBodyControllerRos::shutdownRosCommunication() {
  trackingErrorPublisher_.shutdown();
  resetEstimatorClient_.shutdown();
}

bool TestWholeBodyControllerRos::resetStateEstimator() {
  any_state_estimator_msgs::ResetStateEstimator srv;
  if (!resetEstimatorClient_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  resetEstimatorClient_.call(srv);
  return true;
}

bool TestWholeBodyControllerRos::initialize() {
  initializeRosCommunication();
  return TestWholeBodyController::initialize();
}

bool TestWholeBodyControllerRos::preStop() {
  shutdownRosCommunication();
  return TestWholeBodyController::preStop();
}

void TestWholeBodyControllerRos::goToOperationMode(const std::string& operationMode, anymal_motion_control::OperationModeAction* action) {
  TestWholeBodyController::goToOperationMode(operationMode, action);
  if (operationMode != "freeze" && action->getResult() == anymal_motion_control::OperationModeAction::Result::SWITCHED) {
    anymal_ctrl_test_whole_body_control_msgs::MotionTrackingErrors msg{};
    msg.header.stamp = ros::Time::now();
    msg.motion_type = operationMode;
    tf::matrixEigenToMsg(this->getJointsRmsError(Base::ControlLevel::POSITION), msg.rmsJointPositionTrackingError);
    tf::matrixEigenToMsg(this->getJointsRmsError(Base::ControlLevel::VELOCITY), msg.rmsJointVelocityTrackingError);
    tf::matrixEigenToMsg(this->getJointsRmsError(Base::ControlLevel::ACCELERATION), msg.rmsJointAccelerationTrackingError);
    msg.rmsEEPositionTrackingError.reserve(AD::getNumLegs());
    msg.rmsEEVelocityTrackingError.reserve(AD::getNumLegs());
    msg.rmsEEAccelerationTrackingError.reserve(AD::getNumLegs());
    geometry_msgs::Vector3 tempVector;
    for (unsigned int endEffectorCounter = 0; endEffectorCounter < AD::getNumLegs(); endEffectorCounter++) {
      tf::vectorEigenToMsg(this->getEndEffectorRmsError(Base::ControlLevel::POSITION, endEffectorCounter), tempVector);
      msg.rmsEEPositionTrackingError.push_back(tempVector);
      tf::vectorEigenToMsg(this->getEndEffectorRmsError(Base::ControlLevel::VELOCITY, endEffectorCounter), tempVector);
      msg.rmsEEVelocityTrackingError.push_back(tempVector);
      tf::vectorEigenToMsg(this->getEndEffectorRmsError(Base::ControlLevel::ACCELERATION, endEffectorCounter), tempVector);
      msg.rmsEEAccelerationTrackingError.push_back(tempVector);
    }
    tf::vectorEigenToMsg(this->getBaseRmsError(Base::ControlLevel::POSITION), msg.rmsBasePositionTrackingError);
    tf::vectorEigenToMsg(this->getBaseRmsError(Base::ControlLevel::VELOCITY), msg.rmsBaseVelocityTrackingError);
    tf::vectorEigenToMsg(this->getBaseRmsError(Base::ControlLevel::ACCELERATION), msg.rmsBaseAccelerationTrackingError);
    trackingErrorPublisher_.publish(msg);
  }
}

}  // namespace anymal_ctrl_test_whole_body_control_ros
