/**
 * @authors     Dario Bellicoso, Francisco Giraldez Gamez
 * @affiliation ETH Zurich, ANYbotics
 * @brief
 */

#include <anymal_model/AnymalInverseKinematics.hpp>
#include <anymal_model/typedefs.hpp>

#include <message_logger/message_logger.hpp>

namespace anymal_model {

AnymalInverseKinematics::AnymalInverseKinematics(const AnymalParameters& parameters) : parameters_(parameters) {}

bool AnymalInverseKinematics::getLimbJointPositionsFromPositionBaseToFootInBaseFrame(Eigen::Vector3d& legJoints,
                                                                                     const Eigen::Vector3d& positionBaseToFootInBaseFrame,
                                                                                     AD::LimbEnum limb,
                                                                                     AT::LegConfigEnum legConfiguration) const {
  bool positiveHFESolution = false;
  bool positiveKFESolution = false;
  auto limbLongitudinal = anymal_description::AnymalDefinitions::mapLimbToLongitudinal::at(limb);
  if (legConfiguration == AT::LegConfigEnum::XConfiguration) {
    if (limbLongitudinal == AT::LongitudinalEnum::FORE) {
      positiveHFESolution = true;
      positiveKFESolution = false;
    } else if (limbLongitudinal == AT::LongitudinalEnum::HIND) {
      positiveHFESolution = false;
      positiveKFESolution = true;
    }
  } else if (legConfiguration == AT::LegConfigEnum::OConfiguration) {
    if (limbLongitudinal == AT::LongitudinalEnum::FORE) {
      positiveHFESolution = true;
      positiveKFESolution = true;
    } else if (limbLongitudinal == AT::LongitudinalEnum::HIND) {
      positiveHFESolution = false;
      positiveKFESolution = false;
    }
  } else {
    MELO_ERROR_STREAM("Leg configuration " << AT::legConfigKeys.at(legConfiguration).getName()
                                           << " is not supported by ANYmal's analytical IK!")
    return false;
  }

  return Base::getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
      legJoints, positionBaseToFootInBaseFrame, parameters_.getLegKinematicParameters(limb), positiveHFESolution, positiveKFESolution);
}

}  // namespace anymal_model