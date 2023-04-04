// gtest
#include <gtest/gtest.h>

// yaml tools
#include "yaml_tools/yaml_tools.hpp"

class TestYamlNodeSequenceOfDoubles : public ::testing::Test {
 protected:
  void SetUp() override {
    yamlNode_.pushBack(true);
    yamlNode_.pushBack(2);
    yamlNode_.pushBack(1.0);
    yamlNode_.pushBack(4.5);
    yamlNode_.pushBack("hello");
  }

 protected:
  const std::string yamlString_ =
      "- true\n"
      "- 2\n"
      "- 1.0\n"
      "- 4.5\n"
      "- hello";

  yaml_tools::YamlNode yamlNode_;
};

TEST_F(TestYamlNodeSequenceOfDoubles, readFromString) {  // NOLINT
  EXPECT_EQ(yamlNode_, yaml_tools::YamlNode::fromString(yamlString_));
}

TEST_F(TestYamlNodeSequenceOfDoubles, writeToString) {  // NOLINT
  EXPECT_EQ(yamlString_, yamlNode_.toString());
}