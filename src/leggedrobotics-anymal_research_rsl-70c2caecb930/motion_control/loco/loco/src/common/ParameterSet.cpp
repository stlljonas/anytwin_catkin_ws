/*
 * ParameterSet.cpp
 *
 *  Created on: Mar 6, 2014
 *      Author: Christian Gehring
 */

// loco
#include <loco/common/ParameterSet.hpp>

// std_utils
#include <std_utils/std_utils.hpp>

namespace loco {

ParameterSet::ParameterSet()
    : isDocumentLoaded_(false),
      xmlDocument_(std_utils::make_unique<tinyxml_tools::DocumentHandleXML>()),
      xmlDocumentHandle_(xmlDocument_->getDocumentHandle()) {}

ParameterSet::ParameterSet(const std::string& filename)
    : isDocumentLoaded_(false),
      xmlDocument_(std_utils::make_unique<tinyxml_tools::DocumentHandleXML>()),
      xmlDocumentHandle_(xmlDocument_->getDocumentHandle()) {
  loadXmlDocument(filename);
  if (!isDocumentLoaded_) {
    std::cout << "Could not load parameter file: " << parameterFile_ << std::endl;
  }
}

ParameterSet::ParameterSet(std::unique_ptr<tinyxml_tools::DocumentHandleXML> documentHandle)
    : isDocumentLoaded_(false), xmlDocument_(std::move(documentHandle)), xmlDocumentHandle_(xmlDocument_->getDocumentHandle()) {}

ParameterSet::ParameterSet(const std::string& filename, std::unique_ptr<tinyxml_tools::DocumentHandleXML> documentHandle)
    : isDocumentLoaded_(false), xmlDocument_(std::move(documentHandle)), xmlDocumentHandle_(xmlDocument_->getDocumentHandle()) {
  loadXmlDocument(filename);
  if (!isDocumentLoaded_) {
    std::cout << "Could not load parameter file: " << parameterFile_ << std::endl;
  }
}

bool ParameterSet::isDocumentLoaded() {
  return isDocumentLoaded_;
}

bool ParameterSet::loadXmlDocument(const std::string& filename) {
  parameterFile_ = filename;
  isDocumentLoaded_ = xmlDocument_->create(filename, tinyxml_tools::DocumentMode::READ);
  xmlDocumentHandle_ = xmlDocument_->getDocumentHandle();
  return isDocumentLoaded_;
}

TiXmlHandle& ParameterSet::getHandle() {
  return xmlDocumentHandle_;
}

} /* namespace loco */
