#pragma once

// std
#include <memory>

// yaml tools
#include "yaml_tools/internal/IteratorBase.hpp"

namespace yaml_tools {

//! Iterator wrapper which updates the IteratorValue on dereferencing.
class Iterator : public internal::IteratorBase<Iterator> {
 public:
  //! Constructor.
  Iterator(const YamlNode* parentYamlNode, bool begin) : internal::IteratorBase<Iterator>(parentYamlNode, begin) {}

  //! Dereference IteratorValue by star operator.
  IteratorValue operator*() { return dereference(); }

  //! Dereference IteratorValue py pointer operator.
  std::shared_ptr<IteratorValue> operator->() const {
    // The dereference operator needs to return a pointer.
    // As dereference() returns an rvalue which is destroyed when going out of scope
    // a raw pointer cannot be used. Therefore, a shared_ptr is created.
    return std::make_shared<IteratorValue>(dereference());
  }
};

}  // namespace yaml_tools
