/*
 * TaskLoader.hpp
 *
 *  Created on: Aug 2, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// whole_body_controller
#include <whole_body_control/Task.hpp>

// tinyxml_tools
#include <tinyxml_tools/tinyxml_tools.hpp>

// std_utils
#include <std_utils/std_utils.hpp>

// STL
#include <memory>
#include <vector>

namespace whole_body_control {

class TaskLoader {
 public:
  using Tasks = std::vector<std::unique_ptr<Task>>;

  //! Constructor
  TaskLoader() = default;

  //! Destructor
  virtual ~TaskLoader() = default;

  //! Load parameters
  virtual bool loadTasks(Tasks* /* tasks */, TiXmlHandle /* handle */) { return true; }
};

}  // namespace whole_body_control
