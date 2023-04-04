/*!
 * @author  Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief   Implemenation of ROS interface of WBC test controller class
 * @date    Jul 17, 2019
 */

#pragma once

#include <ros/ros.h>

#include <anymal_ctrl_test_whole_body_control/TestWholeBodyController.hpp>

namespace anymal_ctrl_test_whole_body_control_ros {

class TestWholeBodyControllerRos : public anymal_ctrl_test_whole_body_control::TestWholeBodyController {
 public:
  using Base = anymal_ctrl_test_whole_body_control::TestWholeBodyController;

  TestWholeBodyControllerRos();
  ~TestWholeBodyControllerRos() override = default;

  bool initialize() override;
  bool preStop() override;

 protected:
  bool resetStateEstimator() override;

 private:
  void initializeRosCommunication();
  void shutdownRosCommunication();

 protected:
  void goToOperationMode(const std::string& operationMode, anymal_motion_control::OperationModeAction* action) override;

 private:
  ros::NodeHandle nodeHandle_;
  ros::Publisher trackingErrorPublisher_;
  ros::ServiceClient resetEstimatorClient_;
};

}  // namespace anymal_ctrl_test_whole_body_control_ros
