/*
 * TaskLoaderRomo.hpp
 *
 *  Created on: May 2, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// whole_body_controller
#include <whole_body_control/TaskLoader.hpp>

// whole_body_control_romo
#include "whole_body_control_romo/WholeBodyStateRomo.hpp"

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
class TaskLoaderRomo : public whole_body_control::TaskLoader {
 public:
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = romo::RobotDescription<ConcreteDescription_>;
  using WholeBodyState = WholeBodyStateRomo<ConcreteDescription_, RobotState_>;
  using TaskCreationMethod = std::function<std::unique_ptr<whole_body_control::Task>(WholeBodyState&)>;

  //! Constructor
  TaskLoaderRomo(WholeBodyState& wholeBodyState);

  //! Task registration
  static bool registerTask(std::string name, TaskCreationMethod createFunction);
  static std::unique_ptr<whole_body_control::Task> createTask(const std::string& name, WholeBodyState& wholeBodyState);

  bool loadTasks(Tasks* tasks, TiXmlHandle handle) override;

 protected:
  WholeBodyState& wholeBodyState_;
  //! List of registered task creation functions
  static std::map<std::string, TaskCreationMethod> taskCreationMethods_;
};

}  // namespace whole_body_control_romo

#include "whole_body_control_romo/TaskLoaderRomo.tpp"
