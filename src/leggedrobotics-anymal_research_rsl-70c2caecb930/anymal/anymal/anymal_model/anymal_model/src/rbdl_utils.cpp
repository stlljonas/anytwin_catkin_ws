/*
 * rbdl_utils.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: Dario Bellicoso
 */

// anymal model
#include "anymal_model/rbdl_utils.hpp"

namespace anymal_model {

void setRbdlQFromState(Eigen::VectorXd& rbdlQ, const AnymalState& state) {
  rbdlQ.resize(AnymalState::getNumberOfGeneralizedCoordinates());
  rbdlQ.segment<Position::Dimension>(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::X)) =
      state.getPositionWorldToBaseInWorldFrame().toImplementation();
  auto& orientation = state.getOrientationBaseToWorld();
  rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_X)) = orientation.x();
  rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_Y)) = orientation.y();
  rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_Z)) = orientation.z();
  rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_W)) = orientation.w();
  rbdlQ.segment<AD::getJointsDimension()>(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::LF_HAA)) =
      state.getJointPositions().toImplementation();
}

void setRbdlQDotFromState(Eigen::VectorXd& rbdlQDot, const AnymalState& state) {
  rbdlQDot = state.getGeneralizedVelocities();
}

void setStateFromRbdlQ(AnymalState& state, const Eigen::VectorXd& rbdlQ) {
  state.setJointPositions(
      JointPositions(rbdlQ.segment<AD::getJointsDimension()>(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::LF_HAA))));
  state.setPositionWorldToBaseInWorldFrame(
      Position(rbdlQ.segment<Position::Dimension>(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::X))));
  state.setOrientationBaseToWorld(RotationQuaternion(rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_W)),
                                                     rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_X)),
                                                     rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_Y)),
                                                     rbdlQ(static_cast<unsigned int>(GeneralizedCoordinatesRbdlEnum::Q_Z))));
}

}  // namespace anymal_model
