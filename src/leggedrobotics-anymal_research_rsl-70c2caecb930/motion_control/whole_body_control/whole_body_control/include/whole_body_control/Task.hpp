/*
 * Task.hpp
 *
 *  Created on: May 2, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// hierarchical_optimization
#include <hierarchical_optimization/common/PrioritizedProblem.hpp>

// tinyxml_tools
#include <tinyxml_tools/tinyxml_tools.hpp>

// std_utils
#include <std_utils/std_utils.hpp>

// STL
#include <memory>

namespace whole_body_control {

class Task : public hopt::PrioritizedTask {
 public:
  //! Constructor.
  Task() : hopt::PrioritizedTask("undefined", std::numeric_limits<int>::max()) {}

  //! Destructor.
  ~Task() override = default;

  //! Load Task from xml.
  virtual bool loadParameters(TiXmlHandle taskHandle);

  //! Add parameters to handler.
  virtual bool addParametersToHandler() { return true; }

  //! Add logging variables to logger.
  virtual bool addVariablesToLog(const std::string& /* ns */) const { return true; }

  //! Initialize members.
  virtual bool initialize(double /* dt */) { return true; }

  //! Update task jacobians.
  virtual bool update(double dt, int solutionSpaceDimension) = 0;

  //! Getter task type.
  const std::string& getTaskType() { return taskType_; }

 protected:
  std::string taskType_;
};

using TaskPtr = std::unique_ptr<Task>;

}  // namespace whole_body_control
