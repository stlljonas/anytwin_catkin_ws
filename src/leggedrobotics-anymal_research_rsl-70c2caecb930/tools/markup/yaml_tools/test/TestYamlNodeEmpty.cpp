// gtest
#include <gtest/gtest.h>

// yaml tools
#include "yaml_tools/yaml_tools.hpp"

class TestYamlNodeEmpty : public ::testing::Test {
 protected:
  const std::string nonExistingKey_ = "non_existing_key";
  const size_t nonExistingId_ = 1000;

  yaml_tools::YamlNode yamlNode_;
};

TEST_F(TestYamlNodeEmpty, keyIsEmptyOnConstruction) {  // NOLINT
  ASSERT_TRUE(yamlNode_.getNestedKey().empty());
}

TEST_F(TestYamlNodeEmpty, nullTypeIsSetByDefaultOnConstruction) {  // NOLINT
  EXPECT_TRUE(yamlNode_.isDefined());
  EXPECT_TRUE(yamlNode_.isNull());
  EXPECT_FALSE(yamlNode_.isScalar());
  EXPECT_FALSE(yamlNode_.isSequence());
  EXPECT_FALSE(yamlNode_.isMap());
}

TEST_F(TestYamlNodeEmpty, scalarTypeIsSetOnConstruction) {  // NOLINT
  yaml_tools::YamlNode yamlNodeScalar(yaml_tools::YamlNode::Type::Scalar);
  EXPECT_TRUE(yamlNodeScalar.isDefined());
  EXPECT_FALSE(yamlNodeScalar.isNull());
  EXPECT_TRUE(yamlNodeScalar.isScalar());
  EXPECT_FALSE(yamlNodeScalar.isSequence());
  EXPECT_FALSE(yamlNodeScalar.isMap());
}

TEST_F(TestYamlNodeEmpty, sequenceTypeIsSetOnConstruction) {  // NOLINT
  yaml_tools::YamlNode yamlNodeSequence(yaml_tools::YamlNode::Type::Sequence);
  EXPECT_TRUE(yamlNodeSequence.isDefined());
  EXPECT_FALSE(yamlNodeSequence.isNull());
  EXPECT_FALSE(yamlNodeSequence.isScalar());
  EXPECT_TRUE(yamlNodeSequence.isSequence());
  EXPECT_FALSE(yamlNodeSequence.isMap());
}

TEST_F(TestYamlNodeEmpty, mapTypeIsSetOnConstruction) {  // NOLINT
  yaml_tools::YamlNode yamlNodeMap(yaml_tools::YamlNode::Type::Map);
  EXPECT_TRUE(yamlNodeMap.isDefined());
  EXPECT_FALSE(yamlNodeMap.isNull());
  EXPECT_FALSE(yamlNodeMap.isScalar());
  EXPECT_FALSE(yamlNodeMap.isSequence());
  EXPECT_TRUE(yamlNodeMap.isMap());
}

TEST_F(TestYamlNodeEmpty, nonConstDoesNotThrowExceptionOnBadKey) {  // NOLINT
  yaml_tools::YamlNode yamlNode;
  EXPECT_NO_THROW(yamlNode[nonExistingKey_]);  // NOLINT
}

TEST_F(TestYamlNodeEmpty, constThrowsExceptionOnBadKey) {  // NOLINT
  const yaml_tools::YamlNode yamlNodeConst;
  EXPECT_THROW(yamlNodeConst[nonExistingKey_], yaml_tools::Exception);  // NOLINT
}
