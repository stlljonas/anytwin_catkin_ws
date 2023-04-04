/*
 * DocumentHandleXMLRos.cpp
 *
 *  Created on: 3 Jun 2019
 *      Author: Perry Franklin
 */

#include "tinyxml_tools_ros/DocumentHandleXMLRos.hpp"

#include <ros/package.h>

namespace tinyxml_tools_ros {

std::string DocumentHandleXMLRos::convertPath(const std::string& path) {
  std::string decodedPath = path;
  const size_t commandIndex = path.find("$(");
  if (commandIndex != std::string::npos) {
    // There is a $( in the path, so we need to extract the next word (presumably a verb)
    const size_t verbIndex = path.find_first_not_of(' ', commandIndex + 2);  // Add 2 to get past the $(
    const size_t endOfVerbIndex = path.find(' ', verbIndex);

    const std::string verb = path.substr(verbIndex, endOfVerbIndex - verbIndex);

    if (verb == "find") {
      size_t packageIndex = path.find_first_not_of(' ', endOfVerbIndex);
      size_t endOfPackageIndex = path.find_first_of(" )", packageIndex);

      const std::string packageName = path.substr(packageIndex, endOfPackageIndex - packageIndex);

      const std::string packagePath = ros::package::getPath(packageName);

      if (packagePath.empty()) {
        MELO_ERROR_STREAM("ros::package::getPath was unable to find package " << packageName << ". Are you sure this package exists?");
      } else {
        const size_t endOfCommandIndex = path.find(')', endOfPackageIndex);

        if (endOfCommandIndex != std::string::npos) {
          const size_t afterCommandIndex = endOfCommandIndex + 1;  // the endOfCommandIndex is at ")"; we need to add 1 to get past it
          decodedPath = packagePath + path.substr(afterCommandIndex, path.size() - afterCommandIndex);
          MELO_DEBUG_STREAM("The path " << path << " was converted to " << decodedPath);

        } else {
          MELO_ERROR_STREAM("Your command doesn't have a closing ).");
        }
      }
    } else if (endOfVerbIndex == std::string::npos) {
      MELO_ERROR_STREAM(
          "The beginning of a command (\"$(\") was found, but parsing failed to find a valid verb. Your syntax is probably malformed. "
          "(path is "
          << path << ")");
    } else {
      MELO_ERROR_STREAM("Verb " << verb << " is unknown to this class; including this path will probably fail. (path is " << path << ")");
    }
  }
  return Base::convertPath(decodedPath);
}

}  // namespace tinyxml_tools_ros
