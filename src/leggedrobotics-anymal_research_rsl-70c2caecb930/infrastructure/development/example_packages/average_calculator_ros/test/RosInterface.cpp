/**
 * @authors     Remo Diethelm
 * @affiliation ANYbotics
 * @brief       Test ROS interface of average calculator.
 */

#include <vector>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <average_calculator_msgs/GetAverageValue.h>
#include <average_calculator_msgs/Value.h>

class RosFixture : public ::testing::Test {
  void SetUp() override {
    const std::map<std::string, std::string> remappings{};
    ros::init(remappings, "average_calculator_ros_test");
    ros::start();
  }

  void TearDown() override { ros::shutdown(); }
};

TEST_F(RosFixture, SubscriberAndServiceServer) {  // NOLINT
  // Set up ROS node handle.
  ros::NodeHandle nodeHandle("~");

  // Set up ROS communication layer.
  ros::Publisher valuePublisher = nodeHandle.advertise<average_calculator_msgs::Value>("/average_calculator_ros/value", 10);
  ros::ServiceClient getAverageValueClient =
      nodeHandle.serviceClient<average_calculator_msgs::GetAverageValue>("/average_calculator_ros/get_average_value");

  // Wait until the node is running.
  ASSERT_TRUE(getAverageValueClient.waitForExistence(ros::Duration(10.0)));

  // Check the initialization.
  average_calculator_msgs::GetAverageValue getAverageValue;
  EXPECT_TRUE(getAverageValueClient.call(getAverageValue));
  EXPECT_EQ(0.0, getAverageValue.response.average.data);

  // Publish some value messages and get the average.
  std::vector<double> values = {-25.0, 10.0, -15.0};
  double valueAverage = 0.0;
  average_calculator_msgs::Value valueMsg;
  for (const auto& value : values) {
    // Sleep to give the communication some time.
    ros::Duration(1.0).sleep();
    // Publish the message.
    valueMsg.data = value;
    valuePublisher.publish(valueMsg);
    // Compute the expected average.
    valueAverage += value;
  }
  valueAverage /= values.size();

  // Sleep to give the communication some time.
  ros::Duration(1.0).sleep();

  // Check the computed average value.
  EXPECT_TRUE(getAverageValueClient.call(getAverageValue));
  EXPECT_EQ(valueAverage, getAverageValue.response.average.data);
}
