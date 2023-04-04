/*
 * Legs.hpp
 *
 *  Created on: Jan 11, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// loco
#include "loco/common/GroupContainer.hpp"
#include "loco/common/legs/LegBase.hpp"

// std utils
#include <std_utils/containers/ObserverPointer.hpp>

// stl
#include <type_traits>

namespace loco {

using Legs = GroupContainer<LegBase, std_utils::observer_ptr>;

namespace traits {

//! is_legs_iterator false type
template <typename Iter_, typename Enable_ = void>
struct is_legs_iterator : std::false_type {};

//! is_legs_iterator true type
template <typename Iter_>
struct is_legs_iterator<
    Iter_, typename std::enable_if<
               std::is_base_of<LegBase, typename std::remove_pointer<typename std::iterator_traits<Iter_>::value_type>::type>::value>::type>
    : std::true_type {};
}  // namespace traits

}  // namespace loco
