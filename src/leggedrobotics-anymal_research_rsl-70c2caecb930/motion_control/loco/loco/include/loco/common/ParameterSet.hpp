/*
 * ParameterSet.hpp
 *
 *  Created on: Mar 6, 2014
 *      Author: Christian Gehring
 */

#pragma once

// stl
#include <string>

// tinyxml
#include <tinyxml.h>
#include <tinyxml_tools/DocumentHandleXML.hpp>

namespace loco {

//! Parameter set
/*! Loads parameters from an XML file
 *
 */
class ParameterSet {
 public:
  //! Constructor
  ParameterSet();

  explicit ParameterSet(const std::string& filename);

  // Pass a ptr to a DocumentHandle (such as tinyxml_tools_ros::DocumentHandleXMLRos) to use additional features
  // the unique_ptr passed into the contructor will be moved (ie become null)
  explicit ParameterSet(std::unique_ptr<tinyxml_tools::DocumentHandleXML> documentHandle);

  // the unique_ptr passed into the contructor will be moved (ie become null)
  ParameterSet(const std::string& filename, std::unique_ptr<tinyxml_tools::DocumentHandleXML> documentHandle);

  //! Destructor
  virtual ~ParameterSet() = default;

  /*! @returns true if document is loaded
   */
  bool isDocumentLoaded();

  /*! Loads the document
   * @param filename  path and file name of the XML file
   * @returns true if document could be loaded
   */
  bool loadXmlDocument(const std::string& filename);

  /*! Gets the handle to access the parameters from the XML file
   *
   * @return  handle to the XML document
   */
  TiXmlHandle& getHandle();

 protected:
  //! true if xml document is loaded
  bool isDocumentLoaded_;

  //! XML document (see xmlDocumentHandle_)
  std::unique_ptr<tinyxml_tools::DocumentHandleXML> xmlDocument_;

  //! document handle to access the parameters
  TiXmlHandle xmlDocumentHandle_;

  //! path to parameter file
  std::string parameterFile_;
};

} /* namespace loco */
