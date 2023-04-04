// scan to pointcloud
#include "scan_to_pointcloud/ScanToPointcloud.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_to_pointcloud");
  ros::NodeHandle nodeHandle("~");

  try {
    scan_to_pointcloud::ScanToPointcloud node(nodeHandle);
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown Exception");
    return 1;
  }

  return 0;
}
