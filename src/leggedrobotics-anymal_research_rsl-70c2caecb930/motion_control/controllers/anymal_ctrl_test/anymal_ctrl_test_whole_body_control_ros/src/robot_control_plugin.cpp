/*!
 * @author  Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief   Plugin declaration of ROS WBC test controller
 * @date    Jul 17, 2019
 */

#include <pluginlib/class_list_macros.hpp>

#include "anymal_ctrl_test_whole_body_control_ros/TestWholeBodyControllerRos.hpp"

// export controller plugin
PLUGINLIB_EXPORT_CLASS(anymal_ctrl_test_whole_body_control_ros::TestWholeBodyControllerRos, robot_control::ControllerInterface)
