/*!
 * @author  Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief   Plugin declaration of WBC test controller
 * @date    Jul 12, 2019
 */

#include <pluginlib/class_list_macros.hpp>

#include "anymal_ctrl_test_whole_body_control/TestWholeBodyController.hpp"

// export controller plugin
PLUGINLIB_EXPORT_CLASS(anymal_ctrl_test_whole_body_control::TestWholeBodyController, robot_control::ControllerInterface)
