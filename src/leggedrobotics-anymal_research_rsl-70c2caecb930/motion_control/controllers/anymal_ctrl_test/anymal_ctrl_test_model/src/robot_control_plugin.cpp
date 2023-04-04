/*!
 * @author  Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief   Plugin for model test controller
 */
#include <pluginlib/class_list_macros.h>

#include "anymal_ctrl_test_model/TestModel.hpp"

// export controller plugin
PLUGINLIB_EXPORT_CLASS(anymal_ctrl_test_model::TestModel, robot_control::ControllerInterface)
