/*
 * DocumentHandleXMLRos.hpp
 *
 *  Created on: 3 Jun 2019
 *      Author: Perry Franklin
 */

#pragma once

#include <tinyxml_tools/DocumentHandleXML.hpp>

namespace tinyxml_tools_ros {

class DocumentHandleXMLRos : public tinyxml_tools::DocumentHandleXML {
 private:
  //! Helper alias for the base class.
  using Base = tinyxml_tools::DocumentHandleXML;

 public:
  DocumentHandleXMLRos() = default;
  ~DocumentHandleXMLRos() override = default;

 protected:
  std::string convertPath(const std::string& path) override;
};

}  // namespace tinyxml_tools_ros
