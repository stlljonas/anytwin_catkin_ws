/**
 * @authors     Dario Bellicoso, Francisco Giraldez Gamez
 * @affiliation ETH Zurich, ANYbotics
 * @brief
 */

#pragma once

#include <analytical_inverse_kinematics/AnalyticalInverseKinematics.hpp>
#include <anymal_description/AnymalDescription.hpp>
#include <anymal_description/AnymalTopology.hpp>

#include "anymal_model/AnymalParameters.hpp"

namespace anymal_model {

class AnymalInverseKinematics : public analytical_inverse_kinematics::AnalyticalInverseKinematics {
 public:
  explicit AnymalInverseKinematics(const AnymalParameters& parameters);
  ~AnymalInverseKinematics() override = default;

  bool getLimbJointPositionsFromPositionBaseToFootInBaseFrame(Eigen::Vector3d& legJoints,
                                                              const Eigen::Vector3d& positionBaseToFootInBaseFrame, AD::LimbEnum limb,
                                                              AT::LegConfigEnum legConfiguration = AT::LegConfigEnum::XConfiguration) const;

 protected:
  using Base = analytical_inverse_kinematics::AnalyticalInverseKinematics;

  //! Kinematic parameters.
  const AnymalParameters& parameters_;
};

}  // namespace anymal_model
