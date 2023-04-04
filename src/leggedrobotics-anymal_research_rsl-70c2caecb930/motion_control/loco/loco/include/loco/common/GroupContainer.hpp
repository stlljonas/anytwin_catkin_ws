/*
 * GroupContainer.hpp
 *
 *  Created on: Dec 13, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// stl
#include <algorithm>
#include <memory>
#include <type_traits>
#include <vector>

// std_utils
#include <std_utils/containers/SmartPointerContainer.hpp>

namespace loco {

template <typename Item_, template <typename...> class smart_pointer_type_ = std::unique_ptr>
class GroupContainer : public std_utils::smart_pointer_container<Item_, smart_pointer_type_> {
 public:
  using Base = std_utils::smart_pointer_container<Item_, smart_pointer_type_>;

  using Container = typename Base::container_type;
  using size_type = typename Container::size_type;
  using iterator = typename Container::iterator;

  using ItemSmartPtr = typename Base::smart_pointer_type;
  using Item = typename Base::element_type;
  using ItemPtr = typename Base::element_type*;

  using reference = typename Base::reference;
  using const_reference = typename Base::const_reference;

 public:
  GroupContainer() = default;

  template <typename ItemIterator_>
  GroupContainer(ItemIterator_ begin, ItemIterator_ end) : Base(begin, end) {}

  explicit GroupContainer(const Base& other) : Base(other) {}

  ~GroupContainer() override = default;

  const Item& get(size_type offset) const { return *this->items_.at(offset); }

  ItemPtr getPtr(size_type offset) { return this->items_.at(offset).get(); }

  void addItem(ItemSmartPtr&& item) { this->items_.emplace_back(std::move(item)); }

  reference back() { return this->items_.back(); }
  const_reference back() const { return this->items_.back(); }
};

} /* namespace loco */