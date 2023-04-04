/*
 * Limbs.hpp
 *
 *  Created on: Jan 11, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// loco
#include "loco/common/GroupContainer.hpp"
#include "loco/common/limbs/LimbBase.hpp"

// std utils
#include <std_utils/containers/ObserverPointer.hpp>

namespace loco {

using Limbs = GroupContainer<LimbBase, std_utils::observer_ptr>;

namespace traits {

//! is_limbs_iterator false type
template <typename Iter_, typename Enable_ = void>
struct is_limbs_iterator : std::false_type {};

//! is_limbs_iterator true type
template <typename Iter_>
struct is_limbs_iterator<Iter_,
                         typename std::enable_if<std::is_base_of<
                             LimbBase, typename std::remove_pointer<typename std::iterator_traits<Iter_>::value_type>::type>::value>::type>
    : std::true_type {};
}  // namespace traits

}  // namespace loco
