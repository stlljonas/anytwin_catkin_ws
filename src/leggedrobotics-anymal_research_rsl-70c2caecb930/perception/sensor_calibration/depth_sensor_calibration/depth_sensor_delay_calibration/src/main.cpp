// depth sensor delay calibration
#include "depth_sensor_delay_calibration/DepthSensorDelayCalibration.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_sensor_delay_calibration");
  ros::NodeHandle nodeHandle("~");

  depth_sensor_delay_calibration::DepthSensorDelayCalibration depthSensorDelayCalibration(nodeHandle);
  ros::spin();

  return 0;
}
