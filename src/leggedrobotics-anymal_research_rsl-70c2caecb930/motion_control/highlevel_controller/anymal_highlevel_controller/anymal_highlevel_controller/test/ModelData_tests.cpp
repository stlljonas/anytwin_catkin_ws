/*!
/*!
 * @file    ModelData_tests.cpp
 * @author  Dario Bellicoso
 * @date    Nov, 2017
 */

// test
#include <gtest/gtest.h>

// kindr
#include <kindr/common/gtest_eigen.hpp>
#include <kindr/Core>

// anymal msgs
#include <anymal_msgs/AnymalState.h>

// anymal description
#include <anymal_description/AnymalDescription.hpp>

// anymal model
#include <anymal_model/AnymalModel.hpp>

// anymal model ros
#include <anymal_model_ros/conversions.hpp>
#include <anymal_model_ros/initializations.hpp>

// boost
#include <boost/filesystem.hpp>

// high-level controller
#include <anymal_highlevel_controller/ModelData.hpp>


using AD = anymal_description::AnymalDescription;


TEST(ModelDataTests, setAnymalStateROS) {

  //-- Load model for testing
  boost::filesystem::path filePath(__FILE__);
  std::string path = filePath.parent_path().parent_path().generic_string() + std::string{"/test/anymal_minimal.urdf"};

  anymal_highlevel_controller::ModelData modelData;
  modelData.init(0.0025, false, path);

  anymal_msgs::AnymalStatePtr stateMsgPtr(new anymal_msgs::AnymalState());

  anymal_model_ros::initialize(*stateMsgPtr);

  Eigen::VectorXd randomJointPositions = anymal_model::JointPositions::Random().toImplementation();
  stateMsgPtr->joints.position = std::vector<double>(randomJointPositions.data(), randomJointPositions.data() + randomJointPositions.size());

  Eigen::VectorXd randomJointVelocities = anymal_model::JointVelocities::Random().toImplementation();
  stateMsgPtr->joints.velocity = std::vector<double>(randomJointVelocities.data(), randomJointVelocities.data() + randomJointVelocities.size());

  Eigen::VectorXd randomJointEfforts = anymal_model::JointTorques::Random().toImplementation();
  stateMsgPtr->joints.effort = std::vector<double>(randomJointEfforts.data(), randomJointEfforts.data() + randomJointEfforts.size());

  anymal_model::Pose randomPoseBaseToWorld;
  randomPoseBaseToWorld.getPosition().setRandom();
  randomPoseBaseToWorld.getRotation().setRandom();
  stateMsgPtr->pose.pose.position.x = randomPoseBaseToWorld.getPosition().x();
  stateMsgPtr->pose.pose.position.y = randomPoseBaseToWorld.getPosition().y();
  stateMsgPtr->pose.pose.position.z = randomPoseBaseToWorld.getPosition().z();
  stateMsgPtr->pose.pose.orientation.w = randomPoseBaseToWorld.getRotation().w();
  stateMsgPtr->pose.pose.orientation.x = randomPoseBaseToWorld.getRotation().x();
  stateMsgPtr->pose.pose.orientation.y = randomPoseBaseToWorld.getRotation().y();
  stateMsgPtr->pose.pose.orientation.z = randomPoseBaseToWorld.getRotation().z();

  anymal_model::Twist randomTwistInBaseFrame;
  randomTwistInBaseFrame.getTranslationalVelocity().setRandom();
  randomTwistInBaseFrame.getRotationalVelocity().setRandom();
  stateMsgPtr->twist.twist.linear.x = randomTwistInBaseFrame.getTranslationalVelocity().x();
  stateMsgPtr->twist.twist.linear.y = randomTwistInBaseFrame.getTranslationalVelocity().y();
  stateMsgPtr->twist.twist.linear.z = randomTwistInBaseFrame.getTranslationalVelocity().z();
  stateMsgPtr->twist.twist.angular.x = randomTwistInBaseFrame.getRotationalVelocity().x();
  stateMsgPtr->twist.twist.angular.y = randomTwistInBaseFrame.getRotationalVelocity().y();
  stateMsgPtr->twist.twist.angular.z = randomTwistInBaseFrame.getRotationalVelocity().z();

  anymal_model::ExtendedAnymalState state;
  anymal_model_ros::fromRos(*stateMsgPtr, state);
  modelData.setAnymalState(state);

  KINDR_ASSERT_DOUBLE_MX_EQ(modelData.getState()->getAnymalModel().getState().getJointPositions().toImplementation(),
                            randomJointPositions, 1.0e-10, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(modelData.getState()->getAnymalModel().getState().getJointVelocities().toImplementation(),
                            randomJointVelocities, 1.0e-10, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(modelData.getState()->getAnymalModel().getState().getJointTorques().toImplementation(),
                            randomJointEfforts, 1.0e-10, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(modelData.getState()->getAnymalModel().getState().getPositionWorldToBaseInWorldFrame().toImplementation(),
                            randomPoseBaseToWorld.getPosition().toImplementation(), 1.0e-10, "");
  anymal_model::LinearVelocity linearVelocityFromModelInBaseFrame =
      modelData.getState()->getAnymalModel().getState().getOrientationBaseToWorld().inverseRotate(
        modelData.getState()->getAnymalModel().getState().getLinearVelocityBaseInWorldFrame()
      );
  KINDR_ASSERT_DOUBLE_MX_EQ(linearVelocityFromModelInBaseFrame.toImplementation(),
                            randomTwistInBaseFrame.getTranslationalVelocity().toImplementation(), 1.0e-10, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(modelData.getState()->getAnymalModel().getState().getAngularVelocityBaseInBaseFrame().toImplementation(),
                            randomTwistInBaseFrame.getRotationalVelocity().toImplementation(), 1.0e-10, "");
}
