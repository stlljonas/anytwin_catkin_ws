#pragma once

// std
#include <string>
#include <tuple>

// yaml tools
#include "yaml_tools/YamlNode.hpp"

namespace yaml_tools {

//! The iterator value can hold a sequence (YAML node) or map (key + YAML node) element.
class IteratorValue : public YamlNode, public std::pair<std::string, YamlNode> {
 public:
  explicit IteratorValue(const YamlNode& element) : YamlNode(element) {}
  explicit IteratorValue(const std::pair<std::string, YamlNode>& keyAndElement) : std::pair<std::string, YamlNode>(keyAndElement) {}
};

}  // namespace yaml_tools
