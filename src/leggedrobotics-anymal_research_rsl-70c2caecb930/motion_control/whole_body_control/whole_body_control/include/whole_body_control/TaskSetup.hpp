/*
 * TaskSetup.hpp
 *
 *  Created on: Aug 2, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// whole_body_controller
#include <whole_body_control/Task.hpp>

// hierarchical optimization
#include <hierarchical_optimization/HierarchicalOptimizationBase.hpp>

// tinyxml_tools
#include <tinyxml_tools/tinyxml_tools.hpp>

// std_utils
#include <std_utils/std_utils.hpp>

// STL
#include <memory>
#include <vector>

namespace whole_body_control {

class TaskSetup {
 public:
  using TaskSetupPair = std::pair<int, Task*>;
  using TaskSetupPairVector = std::vector<TaskSetupPair>;

  //! Constructor
  explicit TaskSetup(std::string setupName);

  //! Destructor
  ~TaskSetup() = default;

  //! Load task setup configuration
  bool loadTaskSetup(TiXmlHandle setupHandle, const std::vector<std::unique_ptr<Task>>& tasks);

  //! Add task setup to hierarchical optimization
  bool addOptimizationTasks(double dt, int solutionSpaceDimension, const hopt::HierarchicalOptimizationBasePtr& hierarchicalOptimization);

  //! Initialize all tasks in current setup
  bool initializeTasks(double dt);

  //! Add parameters to parameter handler for all tasks in current setup.
  bool addParametersToHandler();

  //! Add variables and signals to logger for all tasks in current setup.
  bool addVariablesToLog(const std::string& ns) const;

  //! Reload task priorities for the current setup.
  void reloadTaskPriorities();

  //! Extend from other setup
  void appendTasksFromSetup(const TaskSetup& otherSetup);

  //! Clear tasks vector
  void clear();

  //! Print the task setup in a table
  void print();

 private:
  //! Setup name
  std::string setupName_;
  //! Tasks present in setup
  TaskSetupPairVector setupTasks_;
};

}  // namespace whole_body_control
