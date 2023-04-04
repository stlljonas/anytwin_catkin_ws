/*
 * LimbInformation.hpp
 *
 *  Created on: Jan 16, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once

// loco
#include "loco/common/limbs/LimbBase.hpp"

// STL
#include <algorithm>
#include <unordered_map>

// boost
#include <boost/range/adaptor/map.hpp>

namespace loco {

//! Information for limbs
struct LimbInfo {
  bool isPartOfForceDistribution_;
  bool isLoadConstraintActive_;
  unsigned int limbIndexInLimbList_;
  unsigned int limbStartIndexInSolutionVector_;
  double groundForceWeight_;
};

class LimbInfos {
 private:
  std::unordered_map<int, LimbInfo> limbInfos_;

 public:
  unsigned int getNumLimbsInForceDistribution() const {
    return static_cast<unsigned int>(std::count_if(limbInfos_.begin(), limbInfos_.end(), [](const std::pair<int, LimbInfo>& limbInfo) {
      return limbInfo.second.isPartOfForceDistribution_;
    }));
  }

  unsigned int getNumLimbsWithActiveLegLoadConstraint() const {
    return static_cast<unsigned int>(std::count_if(limbInfos_.begin(), limbInfos_.end(), [](const std::pair<int, LimbInfo>& limbInfo) {
      return limbInfo.second.isLoadConstraintActive_;
    }));
  }

  LimbInfo& operator[](int limbId) { return limbInfos_[limbId]; }

  LimbInfo& operator[](const loco::LimbBase* const limb) { return limbInfos_[limb->getLimbUInt()]; }

  const LimbInfo& operator[](int limbId) const { return limbInfos_.at(limbId); }

  const LimbInfo& operator[](const loco::LimbBase* const limb) const { return limbInfos_.at(limb->getLimbUInt()); }

  auto get() -> decltype(this->limbInfos_ | boost::adaptors::map_values) { return limbInfos_ | boost::adaptors::map_values; }

  auto get() const -> decltype(this->limbInfos_ | boost::adaptors::map_values) { return limbInfos_ | boost::adaptors::map_values; }
};

}  // namespace loco
