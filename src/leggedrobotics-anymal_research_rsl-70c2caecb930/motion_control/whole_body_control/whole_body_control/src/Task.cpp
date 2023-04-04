/*
 * Task.cpp
 *
 *  Created on: May 2, 2018
 *      Author: Gabriel Hottiger
 */

// whole_body_control
#include <whole_body_control/Task.hpp>

namespace whole_body_control {

bool Task::loadParameters(TiXmlHandle taskHandle) {
  //! Get task handle attributes
  if (!tinyxml_tools::loadParameter(taskType_, taskHandle.ToElement(), "type")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(name_, taskHandle.ToElement(), "name")) {
    return false;
  }

  return true;
}

}  // namespace whole_body_control