
// series elastic actuator ros
#include "robot_utils_ros/force_calibrators/ConvertRosMessages.hpp"

namespace robot_utils_ros {

void ConvertRosMessages::writeToMessage(robot_utils_ros::ForceCalibratorCommand& message,
                                        const robot_utils::ForceCalibratorCommand& command) {
  message.cmd_start = static_cast<uint8_t>(command.cmdStart_);
  message.cmd_continue = static_cast<uint8_t>(command.cmdContinue_);
  message.cmd_calibrate = static_cast<uint8_t>(command.cmdCalibrate_);
  message.enable_outlier_detector = static_cast<uint8_t>(command.enableOutlierDetector_);
  message.numSamples = command.numSamples_;
  message.num_good_samples = command.numGoodSamples_;
}

void ConvertRosMessages::readFromMessage(robot_utils::ForceCalibratorCommand& command,
                                         const robot_utils_ros::ForceCalibratorCommand& message) {
  command.cmdStart_ = static_cast<bool>(message.cmd_start);
  command.cmdContinue_ = static_cast<bool>(message.cmd_continue);
  command.cmdCalibrate_ = static_cast<bool>(message.cmd_calibrate);
  command.enableOutlierDetector_ = static_cast<bool>(message.enable_outlier_detector);
  command.numSamples_ = message.numSamples;
  command.numGoodSamples_ = message.num_good_samples;
}

}  // namespace robot_utils_ros
