/*
 * Arms.hpp
 *
 *  Created on: Jan 11, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// loco
#include "loco/common/GroupContainer.hpp"
#include "loco/common/arms/ArmBase.hpp"

// std utils
#include <std_utils/containers/ObserverPointer.hpp>

namespace loco {

using Arms = GroupContainer<ArmBase, std_utils::observer_ptr>;

namespace traits {
//! is_arms_iterator false type
template <typename Iter_, typename Enable_ = void>
struct is_arms_iterator : std::false_type {};

//! is_arms_iterator true type
template <typename Iter_>
struct is_arms_iterator<
    Iter_, typename std::enable_if<
               std::is_base_of<ArmBase, typename std::remove_pointer<typename std::iterator_traits<Iter_>::value_type>::type>::value>::type>
    : std::true_type {};
}  // namespace traits

}  // namespace loco
