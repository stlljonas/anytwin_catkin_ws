
#include "slam_loggers/GazeboGroundTruthPosePublisher.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gazebo_ground_truth_pose_publisher");
  ros::NodeHandle nh("~");

  slam_loggers::GazeboGroundTruthPosePublisher gazeboGroundTruthPosePublisher(nh);

  ros::spin();

  return EXIT_SUCCESS;
}