/**
 * @authors     Remo Diethelm
 * @affiliation ANYbotics
 * @brief       ROS node for average calculations.
 */

#include <cstdlib>

#include "average_calculator_ros/AverageCalculatorRos.hpp"

int main(int argc, char** argv) {
  // Initialize ROS.
  ros::init(argc, argv, "average_calculator_ros");
  ros::NodeHandle nodeHandle("~");

  // Create a value average calculator ROS node.
  average_calculator_ros::AverageCalculatorRos averageCalculatorRos(nodeHandle);

  // Spin until shutdown is requested.
  ros::spin();
  return EXIT_SUCCESS;
}
