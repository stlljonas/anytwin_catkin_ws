/*!
 * @file     AnymalDefinitions.hpp
 * @author   Dario Bellicoso
 * @date     Nov 22, 2017
 */

#pragma once

// anymal description
#include "anymal_description/AnymalTopology.hpp"

// std_utils
#include <std_utils/std_utils.hpp>

namespace anymal_description {

struct AnymalDefinitions {
  template <AnymalTopology::LimbEnum limb, AnymalTopology::BodyEnum body>
  using limbToBodyKV = std_utils::KeyValuePair<AnymalTopology::LimbEnum, AnymalTopology::BodyEnum, limb, body>;
  using mapLimbToHip = std_utils::CompileTimeMap<AnymalTopology::LimbEnum, AnymalTopology::BodyEnum,
                                                 limbToBodyKV<AnymalTopology::LimbEnum::LF_LEG, AnymalTopology::BodyEnum::LF_HIP>,
                                                 limbToBodyKV<AnymalTopology::LimbEnum::RF_LEG, AnymalTopology::BodyEnum::RF_HIP>,
                                                 limbToBodyKV<AnymalTopology::LimbEnum::LH_LEG, AnymalTopology::BodyEnum::LH_HIP>,
                                                 limbToBodyKV<AnymalTopology::LimbEnum::RH_LEG, AnymalTopology::BodyEnum::RH_HIP> >;
  using mapLimbToThigh = std_utils::CompileTimeMap<AnymalTopology::LimbEnum, AnymalTopology::BodyEnum,
                                                   limbToBodyKV<AnymalTopology::LimbEnum::LF_LEG, AnymalTopology::BodyEnum::LF_THIGH>,
                                                   limbToBodyKV<AnymalTopology::LimbEnum::RF_LEG, AnymalTopology::BodyEnum::RF_THIGH>,
                                                   limbToBodyKV<AnymalTopology::LimbEnum::LH_LEG, AnymalTopology::BodyEnum::LH_THIGH>,
                                                   limbToBodyKV<AnymalTopology::LimbEnum::RH_LEG, AnymalTopology::BodyEnum::RH_THIGH> >;
  using mapLimbToShank = std_utils::CompileTimeMap<AnymalTopology::LimbEnum, AnymalTopology::BodyEnum,
                                                   limbToBodyKV<AnymalTopology::LimbEnum::LF_LEG, AnymalTopology::BodyEnum::LF_SHANK>,
                                                   limbToBodyKV<AnymalTopology::LimbEnum::RF_LEG, AnymalTopology::BodyEnum::RF_SHANK>,
                                                   limbToBodyKV<AnymalTopology::LimbEnum::LH_LEG, AnymalTopology::BodyEnum::LH_SHANK>,
                                                   limbToBodyKV<AnymalTopology::LimbEnum::RH_LEG, AnymalTopology::BodyEnum::RH_SHANK> >;
  using mapLimbToFoot = std_utils::CompileTimeMap<AnymalTopology::LimbEnum, AnymalTopology::BodyEnum,
                                                  limbToBodyKV<AnymalTopology::LimbEnum::LF_LEG, AnymalTopology::BodyEnum::LF_FOOT>,
                                                  limbToBodyKV<AnymalTopology::LimbEnum::RF_LEG, AnymalTopology::BodyEnum::RF_FOOT>,
                                                  limbToBodyKV<AnymalTopology::LimbEnum::LH_LEG, AnymalTopology::BodyEnum::LH_FOOT>,
                                                  limbToBodyKV<AnymalTopology::LimbEnum::RH_LEG, AnymalTopology::BodyEnum::RH_FOOT> >;

  template <AnymalTopology::LimbEnum limb, AnymalTopology::LateralEnum lateral>
  using limbToLateralKV = std_utils::KeyValuePair<AnymalTopology::LimbEnum, AnymalTopology::LateralEnum, limb, lateral>;
  using mapLimbToLateral =
      std_utils::CompileTimeMap<AnymalTopology::LimbEnum, AnymalTopology::LateralEnum,
                                limbToLateralKV<AnymalTopology::LimbEnum::LF_LEG, AnymalTopology::LateralEnum::LEFT>,
                                limbToLateralKV<AnymalTopology::LimbEnum::RF_LEG, AnymalTopology::LateralEnum::RIGHT>,
                                limbToLateralKV<AnymalTopology::LimbEnum::LH_LEG, AnymalTopology::LateralEnum::LEFT>,
                                limbToLateralKV<AnymalTopology::LimbEnum::RH_LEG, AnymalTopology::LateralEnum::RIGHT> >;

  template <AnymalTopology::LimbEnum limb, AnymalTopology::LongitudinalEnum longitudinal>
  using limbToLongitudinalKV = std_utils::KeyValuePair<AnymalTopology::LimbEnum, AnymalTopology::LongitudinalEnum, limb, longitudinal>;
  using mapLimbToLongitudinal =
      std_utils::CompileTimeMap<AnymalTopology::LimbEnum, AnymalTopology::LongitudinalEnum,
                                limbToLongitudinalKV<AnymalTopology::LimbEnum::LF_LEG, AnymalTopology::LongitudinalEnum::FORE>,
                                limbToLongitudinalKV<AnymalTopology::LimbEnum::RF_LEG, AnymalTopology::LongitudinalEnum::FORE>,
                                limbToLongitudinalKV<AnymalTopology::LimbEnum::LH_LEG, AnymalTopology::LongitudinalEnum::HIND>,
                                limbToLongitudinalKV<AnymalTopology::LimbEnum::RH_LEG, AnymalTopology::LongitudinalEnum::HIND> >;

  template <AnymalTopology::LimbEnum limb, AnymalTopology::ContactEnum contact>
  using limbToContactKV = std_utils::KeyValuePair<AnymalTopology::LimbEnum, AnymalTopology::ContactEnum, limb, contact>;
  using mapLimbToContact =
      std_utils::CompileTimeMap<AnymalTopology::LimbEnum, AnymalTopology::ContactEnum,
                                limbToContactKV<AnymalTopology::LimbEnum::LF_LEG, AnymalTopology::ContactEnum::LF_FOOT>,
                                limbToContactKV<AnymalTopology::LimbEnum::RF_LEG, AnymalTopology::ContactEnum::RF_FOOT>,
                                limbToContactKV<AnymalTopology::LimbEnum::LH_LEG, AnymalTopology::ContactEnum::LH_FOOT>,
                                limbToContactKV<AnymalTopology::LimbEnum::RH_LEG, AnymalTopology::ContactEnum::RH_FOOT> >;

  // Degrees of freedom
  inline static constexpr unsigned int getNumLegsImpl() { return 4; }
  inline static constexpr unsigned int getNumArmsImpl() { return 0; }
  inline static constexpr unsigned int getNumDofImpl() { return 18; }
  inline static constexpr unsigned int getNumDofLimbImpl(AnymalTopology::LimbEnum limb) { return AnymalTopology::mapLimbToNDof::at(limb); }
  inline static constexpr unsigned int getNumDofLimbImpl() { return 3; }

  inline static constexpr unsigned int getBranchStartIndexInUImpl(AnymalTopology::BranchEnum branch) {
    return AnymalTopology::mapBranchToStartIndexInU::at(branch);
  }

  inline static constexpr unsigned int getLimbStartIndexInJImpl(AnymalTopology::LimbEnum limb) {
    return AnymalTopology::mapLimbToStartIndexInJ::at(limb);
  }

  inline static constexpr AnymalTopology::BodyEnum getBranchStartBodyImpl(AnymalTopology::BranchEnum branch) {
    return AnymalTopology::mapBranchToStartBody::at(branch);
  }

  inline static constexpr AnymalTopology::BodyEnum getBranchEndBodyImpl(AnymalTopology::BranchEnum branch) {
    return AnymalTopology::mapBranchToEndBody::at(branch);
  }

  // Base degrees of freedom
  inline static constexpr unsigned int getBaseGeneralizedCoordinatesDimensionImpl() { return 7; }
  inline static constexpr unsigned int getBaseGeneralizedVelocitiesDimensionImpl() { return 6; }
};

}  // namespace anymal_description
