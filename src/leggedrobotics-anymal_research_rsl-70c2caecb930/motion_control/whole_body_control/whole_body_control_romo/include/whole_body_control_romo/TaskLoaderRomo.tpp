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
#include <whole_body_control_romo/WholeBodyStateRomo.hpp>

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
TaskLoaderRomo<ConcreteDescription_, RobotState_>::TaskLoaderRomo(WholeBodyState& wholeBodyState) : wholeBodyState_(wholeBodyState) {}

template <typename ConcreteDescription_, typename RobotState_>
bool TaskLoaderRomo<ConcreteDescription_, RobotState_>::registerTask(const std::string name, TaskCreationMethod createFunction) {
  auto it = taskCreationMethods_.find(name);
  if (it == taskCreationMethods_.end()) {
    MELO_DEBUG("[TaskLoaderRomo:] Registered task %s", name.c_str())
    taskCreationMethods_[name] = createFunction;
  }
  MELO_DEBUG("[TaskLoaderRomo:] Task %s already registered!", name.c_str())
  return true;
}

template <typename ConcreteDescription_, typename RobotState_>
std::unique_ptr<whole_body_control::Task> TaskLoaderRomo<ConcreteDescription_, RobotState_>::createTask(const std::string& name,
                                                                                                        WholeBodyState& wholeBodyState) {
  auto it = taskCreationMethods_.find(name);
  if (it != taskCreationMethods_.end()) {
    return it->second(wholeBodyState);  // call the creator function
  }
  return nullptr;
}

template <typename ConcreteDescription_, typename RobotState_>
bool TaskLoaderRomo<ConcreteDescription_, RobotState_>::loadTasks(Tasks* tasks, TiXmlHandle handle) {
  // Clear tasks
  tasks->clear();

  // Get tasks list from xml
  std::vector<TiXmlElement*> taskElements;
  if (!tinyxml_tools::getChildElements(taskElements, handle, "Task")) {
    MELO_WARN("[TaskLoader]: No tasks added to the whole body controller.");
    return true;
  }

  for (auto element : taskElements) {
    //! Get task type and construct new task
    std::string taskType;
    if (!tinyxml_tools::loadParameter(taskType, element, "type")) {
      return false;
    }
    auto task = createTask(taskType, wholeBodyState_);

    if (task != nullptr) {
      // Load from parameters
      if (!task->loadParameters(element)) {
        MELO_WARN_STREAM("[TaskLoaderRomo]: Couldn't load parameters for task " << task->getName() << " of type " << taskType << ".");
        return false;
      }

      MELO_DEBUG_STREAM("[TaskLoaderRomo]: Sucessfully added " << taskType << " task " << task->getName() << "!");
      tasks->emplace_back(std::move(task));
    } else {
      MELO_WARN_STREAM("[TaskLoaderRomo]: Task type " << taskType << " does not exist.");
      return false;
    }
  }

  return true;
}

template <typename ConcreteDescription_, typename RobotState_>
std::map<std::string, typename TaskLoaderRomo<ConcreteDescription_, RobotState_>::TaskCreationMethod>
    TaskLoaderRomo<ConcreteDescription_, RobotState_>::taskCreationMethods_;

}  // namespace whole_body_control_romo
