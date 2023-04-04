/**
 * @authors     Aravind Vijayan
 * @affiliation ANYbotics
 * @brief       Instantiation of Whole-Body controller
 */

#include "loco_anymal/motion_control/WholeBodyController.hpp"

#include <whole_body_control_romo/motion_control/WholeBodyController.tpp>
#include <whole_body_control_romo/motion_control/WholeBodyControllerTasks.tpp>

template class whole_body_control_romo::WholeBodyController<anymal_description::ConcreteAnymalDescription, anymal_model::AnymalState>;

namespace loco_anymal {
}