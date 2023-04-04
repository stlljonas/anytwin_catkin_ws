/*
 * TaskLoader.hpp
 *
 *  Created on: May 2, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// whole_body_controller
#include <whole_body_control/TaskLoader.hpp>
#include <whole_body_control/TaskSetup.hpp>

// tinyxml_tools
#include <tinyxml_tools/tinyxml_tools.hpp>

// std_utils
#include <std_utils/std_utils.hpp>

// STL
#include <memory>
#include <vector>

namespace whole_body_control {

class TaskManager {
 public:
  using Tasks = TaskLoader::Tasks;

  //! Constructor
  explicit TaskManager(std::unique_ptr<TaskLoader> taskLoader);

  //! Destructor
  ~TaskManager() = default;

  //! Load task
  bool loadTaskSetups(TiXmlHandle handle);

  //! Add tasks to optimization
  bool addOptimizationTasks(double dt, int solutionSpaceDimension, const hopt::HierarchicalOptimizationBasePtr& hierarchicalOptimization);

  //! Initialize tasks in current setup
  bool initialize(double dt);

  //! Add variables and signals to to logger.
  bool addVariablesToLog(const std::string& ns) const;

  //! Add parameters to parameter hangler.
  bool addParametersToHandler();

  //! Switch task setup
  bool switchTaskSetup(const std::string& newSetupName);

 protected:
  //! Task loader
  std::unique_ptr<TaskLoader> taskLoader_;
  //! Tasks
  Tasks tasks_;
  //! Tasks present in setup
  std::map<std::string, TaskSetup> taskSetups_;
  //! Current setup
  std::string currentSetup_;
};

}  // namespace whole_body_control
