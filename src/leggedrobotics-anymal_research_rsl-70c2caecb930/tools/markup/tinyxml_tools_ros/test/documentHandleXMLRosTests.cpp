/*
 * documentHandleXMLRosTests.cpp
 *
 *  Created on: 3 Jun 2019
 *      Author: Perry Franklin
 */

#include "tinyxml_tools_ros/DocumentHandleXMLRos.hpp"

#include <gtest/gtest.h>

void checkParamsXml(tinyxml_tools::DocumentHandleXML& docHandle) {
  double val;

  ASSERT_TRUE(docHandle.read("Values", val, "a"));
  ASSERT_EQ(val, 5);
  ASSERT_TRUE(docHandle.read("Values", val, "b"));
  ASSERT_EQ(val, 7);

  ASSERT_TRUE(docHandle.seekFromRoot("RosInclude"));
  std::string string;
  ASSERT_TRUE(docHandle.read("External/Data", string, "d"));
  ASSERT_EQ(string, "hello");

  ASSERT_TRUE(docHandle.seekFromRoot("RosInclude"));
  std::vector<std::string> characters;
  ASSERT_TRUE(docHandle.readSiblings("Second/Apple", characters, "p"));
  ASSERT_EQ(characters[0], "b");
  ASSERT_EQ(characters[1], "c");

  // Test included
  ASSERT_TRUE(docHandle.seekFromRoot("RosInclude"));
  ASSERT_TRUE(docHandle.seek("NewParam/File/TMP"));
  double gain;
  ASSERT_TRUE(docHandle.readAttributes(gain, "gain"));
  ASSERT_EQ(gain, 1e-3);
}

TEST(DocumentHandleXMLRos, loadParams) {  // NOLINT
  tinyxml_tools_ros::DocumentHandleXMLRos documentHandleRos;
  ASSERT_TRUE(documentHandleRos.create("xml_files/Params.xml", tinyxml_tools::DocumentMode::READ));

  checkParamsXml(documentHandleRos);
}

TEST(DocumentHandleXMLRos, loadParamsUsingInheritance) {  // NOLINT
  std::unique_ptr<tinyxml_tools::DocumentHandleXML> documentHandle(new tinyxml_tools_ros::DocumentHandleXMLRos);
  ASSERT_TRUE(documentHandle->create("xml_files/Params.xml", tinyxml_tools::DocumentMode::READ));

  checkParamsXml(*documentHandle);
}

TEST(DocumentHandleXMLRos, badRosInclude) {  // NOLINT
  std::unique_ptr<tinyxml_tools::DocumentHandleXML> documentHandle(new tinyxml_tools_ros::DocumentHandleXMLRos);
  ASSERT_FALSE(documentHandle->create("xml_files/BadRosInclude.xml", tinyxml_tools::DocumentMode::READ));
}
