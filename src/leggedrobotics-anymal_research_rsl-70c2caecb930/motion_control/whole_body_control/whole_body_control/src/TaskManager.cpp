/*
 * TaskLoader.cpp
 *
 *  Created on: May 3, 2018
 *      Author: Gabriel Hottiger
 */

// whole_body_control
#include <whole_body_control/TaskManager.hpp>

// std_utils
#include <std_utils/std_utils.hpp>

// message_logger
#include <message_logger/message_logger.hpp>

namespace whole_body_control {

TaskManager::TaskManager(std::unique_ptr<whole_body_control::TaskLoader> taskLoader)
    : taskLoader_(std::move(taskLoader)), tasks_(), taskSetups_(), currentSetup_("uninitialized") {}

bool TaskManager::loadTaskSetups(TiXmlHandle handle) {
  // Clear setups
  taskSetups_.clear();
  tasks_.clear();

  // Load all tasks
  taskLoader_->loadTasks(&tasks_, handle);

  // Get task setup list from xml
  std::vector<TiXmlElement*> taskSetupElements;
  if (!tinyxml_tools::getChildElements(taskSetupElements, handle, "TaskSetup")) {
    MELO_WARN("[TaskLoader]: No task setups added to the whole body controller.");
    return true;
  }

  for (auto element : taskSetupElements) {
    //! Get task type and construct new task
    std::string setupName;
    if (!tinyxml_tools::loadParameter(setupName, element, "name")) {
      return false;
    }

    TaskSetup taskSetup(setupName);

    // Load from parameters
    if (!taskSetup.loadTaskSetup(element, tasks_)) {
      MELO_WARN_STREAM("[TaskManager]: Couldn't load task setup " << setupName << ".");
      return false;
    }

    // Check for extend syntax
    TiXmlHandle extendHandle = element;
    if (tinyxml_tools::getChildHandle(extendHandle, element, "Extend", false)) {
      std::string extendTask;
      if (!tinyxml_tools::loadParameter(extendTask, extendHandle, "name")) {
        MELO_WARN("[TaskManager]: Could not extend with missing name to create task %s.", setupName.c_str());
        return false;
      }

      auto extendTaskIt = taskSetups_.find(extendTask);
      if (extendTaskIt != taskSetups_.end()) {
        taskSetup.appendTasksFromSetup(extendTaskIt->second);
      } else {
        MELO_WARN("[TaskManager]: Could not extend non-existing task %s to create task %s.", extendTask.c_str(), setupName.c_str());
        return false;
      }
    }

    taskSetups_.insert(std::make_pair(setupName, taskSetup));
  }

  // Set current setup
  TiXmlHandle defaultHandle = handle;
  if (tinyxml_tools::getChildHandle(defaultHandle, handle, "DefaultTaskSetup", false)) {
    std::string defaultTask;
    if (!tinyxml_tools::loadParameter(defaultTask, defaultHandle, "name")) {
      MELO_WARN("[TaskManager]: Could not set default task with missing name.");
      return false;
    }
    auto defaultTaskIt = taskSetups_.find(defaultTask);
    if (defaultTaskIt != taskSetups_.end()) {
      switchTaskSetup(defaultTaskIt->first);
    } else {
      MELO_WARN("[TaskManager]: Could not set non-existing task %s as default.", defaultTask.c_str());
      return false;
    }
  } else {
    switchTaskSetup(taskSetups_.begin()->first);
  }

  MELO_DEBUG_STREAM("[TaskManager]: Choose task setup " << currentSetup_ << " as default!")
  taskSetups_.at(currentSetup_).print();
  return true;
}

bool TaskManager::addOptimizationTasks(double dt, int solutionSpaceDimension,
                                       const hopt::HierarchicalOptimizationBasePtr& hierarchicalOptimization) {
  if (taskSetups_.find(currentSetup_) == taskSetups_.end()) {
    MELO_ERROR("[TaskManager]: Current setup %s is invalid.", currentSetup_.c_str());
    return false;
  }
  taskSetups_.at(currentSetup_).addOptimizationTasks(dt, solutionSpaceDimension, hierarchicalOptimization);
  return true;
}

bool TaskManager::initialize(double dt) {
  if (taskSetups_.find(currentSetup_) == taskSetups_.end()) {
    MELO_ERROR("[TaskManager]: Current setup %s is invalid.", currentSetup_.c_str());
    return false;
  }
  return taskSetups_.at(currentSetup_).initializeTasks(dt);
}

bool TaskManager::addVariablesToLog(const std::string& ns) const {
  if (taskSetups_.find(currentSetup_) == taskSetups_.end()) {
    MELO_ERROR("[TaskManager]: Current setup %s is invalid.", currentSetup_.c_str());
    return false;
  }
  return taskSetups_.at(currentSetup_).addVariablesToLog(ns);
}

bool TaskManager::addParametersToHandler() {
  if (taskSetups_.find(currentSetup_) == taskSetups_.end()) {
    MELO_ERROR("[TaskManager]: Current setup %s is invalid.", currentSetup_.c_str());
    return false;
  }
  return taskSetups_.at(currentSetup_).addParametersToHandler();
}

bool TaskManager::switchTaskSetup(const std::string& newSetupName) {
  // No need to switch if the requested task setup is the current task setup
  if (currentSetup_ == newSetupName) {
    // Loading another task setup could change task priorities.
    // Therefore, re-load priorities even if the current task setup has not changed.
    // This can happen while loading task setups while switching controllers.
    taskSetups_.at(currentSetup_).reloadTaskPriorities();
    return true;
  }

  if (taskSetups_.find(newSetupName) == taskSetups_.end()) {
    MELO_ERROR("[TaskManager]: Could not switch to setup %s. The setup doesn't exist.", newSetupName.c_str());
    return false;
  }
  currentSetup_ = newSetupName;
  // Reload task priorities from the current task setup.
  taskSetups_.at(currentSetup_).reloadTaskPriorities();
  MELO_DEBUG("[TaskManager]: Switched to setup %s.", currentSetup_.c_str())
  taskSetups_.at(currentSetup_).print();
  return true;
}

}  // namespace whole_body_control
