
#include "slam_loggers/PoseLoggerRos.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "slam_pose_logger");
  ros::NodeHandle nh("~");

  slam_loggers::PoseLoggerRos PoseLoggerRos(nh);

  ros::spin();

  PoseLoggerRos.saveLogs();

  return EXIT_SUCCESS;
}