/*
 * TaskSetup.cpp
 *
 *  Created on: Aug 3, 2018
 *      Author: Gabriel Hottiger
 */

// whole_body_control
#include <whole_body_control/TaskManager.hpp>

// std_utils
#include <std_utils/std_utils.hpp>

// message_logger
#include <message_logger/message_logger.hpp>

namespace whole_body_control {

TaskSetup::TaskSetup(std::string setupName) : setupName_(std::move(setupName)), setupTasks_{} {}

bool TaskSetup::loadTaskSetup(TiXmlHandle setupHandle, const std::vector<std::unique_ptr<Task>>& tasks) {
  // Get task setup list from xml
  std::vector<TiXmlElement*> taskElements;
  if (!tinyxml_tools::getChildElements(taskElements, setupHandle, "Task")) {
    MELO_WARN("[TaskSetup]: No tasks in setup %s.", setupName_.c_str());
    return true;
  }

  for (auto element : taskElements) {
    std::string taskName;
    if (!tinyxml_tools::loadParameter(taskName, element, "name")) {
      MELO_WARN("[TaskSetup]: Task in task setup %s has no entry name!", setupName_.c_str());
      return false;
    }

    // Check if task is present in previously loaded tasks
    auto it = std::find_if(tasks.begin(), tasks.end(), [&taskName](const TaskPtr& task) { return task->getName() == taskName; });
    if (it == tasks.end()) {
      MELO_WARN("[TaskSetup]: Task %s in task setup %s does not exist!", taskName.c_str(), setupName_.c_str());
      return false;
    }

    // Initially set priority of task in setup
    int priority;
    if (!tinyxml_tools::loadParameter(priority, element, "priority")) {
      MELO_WARN("[TaskSetup]: Task in task setup %s has no entry 'priority'!", setupName_.c_str());
      return false;
    }
    (*it)->setPriority(priority);

    // Add task to setup
    setupTasks_.emplace_back(TaskSetupPair{priority, it->get()});
  }

  return true;
}

bool TaskSetup::addOptimizationTasks(double dt, int solutionSpaceDimension,
                                     const hopt::HierarchicalOptimizationBasePtr& hierarchicalOptimization) {
  for (auto& taskPair : setupTasks_) {
    if (!taskPair.second->update(dt, solutionSpaceDimension)) {
      MELO_WARN("[TaskSetup]: Could not update task %s in task setup %s.", taskPair.second->getName().c_str(), setupName_.c_str());
      return false;
    }
    hierarchicalOptimization->addOptimizationProblem(*taskPair.second);
  }

  return true;
}

bool TaskSetup::initializeTasks(double dt) {
  for (auto& taskPair : setupTasks_) {
    // Set priorities to task (task might be present in several setups)
    taskPair.second->setPriority(taskPair.first);
    if (!taskPair.second->initialize(dt)) {
      MELO_WARN("[TaskSetup]: Could not initialize task %s in task setup %s.", taskPair.second->getName().c_str(), setupName_.c_str());
      return false;
    }
  }
  return true;
}

bool TaskSetup::addParametersToHandler() {
  for (auto& taskPair : setupTasks_) {
    // Set priorities to task (task might be present in several setups)
    taskPair.second->setPriority(taskPair.first);
    if (!taskPair.second->addParametersToHandler()) {
      MELO_WARN("[TaskSetup]: Could not add parameters to handler for task %s in task setup %s.", taskPair.second->getName().c_str(), setupName_.c_str());
      return false;
    }
  }
  return true;
}

bool TaskSetup::addVariablesToLog(const std::string& ns) const {
  for (auto& taskPair : setupTasks_) {
    // Set priorities to task (task might be present in several setups)
    taskPair.second->setPriority(taskPair.first);
    if (!taskPair.second->addVariablesToLog(ns)) {
      MELO_WARN("[TaskSetup]: Could not add variables and signals to logger for task %s in task setup %s.", taskPair.second->getName().c_str(), setupName_.c_str());
      return false;
    }
  }
  return true;
}

void TaskSetup::reloadTaskPriorities() {
  for (auto& taskPair : setupTasks_) {
    // Set priorities to task (task might be present in several setups)
    taskPair.second->setPriority(taskPair.first);
  }
}

void TaskSetup::appendTasksFromSetup(const TaskSetup& otherSetup) {
  auto oldTasks = setupTasks_;
  setupTasks_.clear();
  setupTasks_.reserve( oldTasks.size() + otherSetup.setupTasks_.size() ); // preallocate memory
  setupTasks_.insert( setupTasks_.begin(), otherSetup.setupTasks_.begin(), otherSetup.setupTasks_.end() );
  setupTasks_.insert( setupTasks_.end(), oldTasks.begin(), oldTasks.end() );
}

void TaskSetup::clear() {
  //! Clear old state
  setupTasks_.clear();
}

void TaskSetup::print() {
  //! Nicely print task setup
  char buffer[2560] = "";
  auto printTask = [&buffer](const std::string& name, const std::string& type, const std::string& priority) {
    sprintf(buffer + strlen(buffer), "\n| %-30s | %-30s | %-15s |", name.c_str(), type.c_str(), priority.c_str());
  };
  printTask("Name", "Type", "Priority");
  sprintf(buffer + strlen(buffer), "%s", message_logger::color::cyan.c_str());
  for (auto& taskPair : setupTasks_) {
    printTask(taskPair.second->getName(), taskPair.second->getTaskType(), std::to_string(taskPair.second->getPriority()));
  }
  MELO_DEBUG("\n%s[WholeBodyControl::TaskSetup]%s -> %s%s%s%s", message_logger::color::magenta.c_str(), message_logger::color::green.c_str(),
            setupName_.c_str(), message_logger::color::blue.c_str(), buffer, message_logger::color::def.c_str());
}

}  // namespace whole_body_control
