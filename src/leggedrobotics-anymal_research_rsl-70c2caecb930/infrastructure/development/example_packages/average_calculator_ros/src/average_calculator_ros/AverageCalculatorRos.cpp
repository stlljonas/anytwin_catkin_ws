/**
 * @authors     Remo Diethelm
 * @affiliation ANYbotics
 * @brief       ROS-wrapper for average calculator implementation.
 */

#include "average_calculator_ros/AverageCalculatorRos.hpp"

#include <message_logger/message_logger.hpp>
#include <param_io/get_param.hpp>

namespace average_calculator_ros {

AverageCalculatorRos::AverageCalculatorRos(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {
#ifndef NDEBUG
  // Print a warning if built in debug.
  MELO_WARN("CMake Build Type is 'Debug'. Change to 'Release' for better performance.");
#endif

  // Initialize ROS interface.
  valueSubscriber_ = nodeHandle_.subscribe(param_io::param<std::string>(nodeHandle_, "subscribers/value/topic", "value"),
                                           param_io::param<uint32_t>(nodeHandle_, "subscribers/value/queue_size", 10),
                                           &AverageCalculatorRos::valueCallback, this);
  getAverageValueServer_ =
      nodeHandle_.advertiseService(param_io::param<std::string>(nodeHandle_, "servers/get_average_value/service", "get_average_value"),
                                   &AverageCalculatorRos::getAverageValueCallback, this);

  // Inform about successful launch.
  MELO_INFO("Successfully launched node.");
}

void AverageCalculatorRos::valueCallback(const average_calculator_msgs::Value& value) {
  // Add a new value.
  averageCalculator_.addValue(value.data);

  // Logging with printf syntax.
  MELO_DEBUG("Added a new value (%f).", value.data);
}

// Since the parameter "request" is not used, we comment it to prevent compiler warnings.
bool AverageCalculatorRos::getAverageValueCallback(average_calculator_msgs::GetAverageValue::Request& /*request*/,
                                                   average_calculator_msgs::GetAverageValue::Response& response) {
  // Compute and return the average value.
  response.average.data = averageCalculator_.getAverageValue();

  // Logging with ostream syntax.
  MELO_DEBUG_STREAM("Current state of average value calculator:\n" << averageCalculator_);
  return true;
}

}  // namespace average_calculator_ros
