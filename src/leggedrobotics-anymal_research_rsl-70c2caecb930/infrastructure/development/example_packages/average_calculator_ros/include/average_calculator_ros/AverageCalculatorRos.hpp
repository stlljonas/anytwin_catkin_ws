/**
 * @authors     Remo Diethelm
 * @affiliation ANYbotics
 * @brief       ROS-wrapper for average calculator implementation.
 */

#pragma once

#include <ros/ros.h>

#include <average_calculator_msgs/GetAverageValue.h>
#include <average_calculator_msgs/Value.h>
#include <average_calculator/AverageCalculator.hpp>

namespace average_calculator_ros {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class AverageCalculatorRos {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  explicit AverageCalculatorRos(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  ~AverageCalculatorRos() = default;

 private:
  /*!
   * ROS topic callback method.
   * @param value The received value.
   */
  void valueCallback(const average_calculator_msgs::Value& value);

  /*!
   * ROS service server callback getting the average value.
   * @param request  The request of the service.
   * @param response The provided response.
   * @return True if successful, false otherwise.
   */
  bool getAverageValueCallback(average_calculator_msgs::GetAverageValue::Request& request,
                               average_calculator_msgs::GetAverageValue::Response& response);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  // Note that the ROS interfaces shut down when they go out of scope, therefore we have to keep them as members.

  //! ROS value subscriber.
  ros::Subscriber valueSubscriber_;

  //! ROS get average value service server.
  ros::ServiceServer getAverageValueServer_;

  //! Average calculator.
  average_calculator::AverageCalculator averageCalculator_;
};

}  // namespace average_calculator_ros
