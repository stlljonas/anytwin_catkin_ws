/*!
 * @file     AnymalState.cpp
 * @author   Christian Gehring, Dario Bellicoso
 * @date     Dec, 2014
 */

// anymal model
#include "anymal_model/AnymalState.hpp"

// stl
#include <iomanip>

namespace anymal_model {

void AnymalState::getPoseBaseToWorld(kindr::HomTransformQuatD& poseBaseToWorld) const {
  poseBaseToWorld = kindr::HomTransformQuatD(getPositionWorldToBaseInWorldFrame(), getOrientationBaseToWorld());
}

void AnymalState::setPoseBaseToWorld(const kindr::HomTransformQuatD& poseBaseToWorld) {
  setOrientationBaseToWorld(poseBaseToWorld.getRotation());
  setPositionWorldToBaseInWorldFrame(poseBaseToWorld.getPosition());
}

void AnymalState::setGeneralizedCoordinatesToLinearlyInterpolated(double t, const AnymalState& state0, const AnymalState& state1) {
  AnymalState res;
  if (t <= 0.0) {
    *this = state0;
    return;
  } else if (t >= 1.0) {
    *this = state1;
    return;
  }
  positionWorldToBaseInWorldFrame_ =
      state0.getPositionWorldToBaseInWorldFrame() * (1.0 - t) + state1.getPositionWorldToBaseInWorldFrame() * t;
  orientationBaseToWorld_ =
      state0.getOrientationBaseToWorld().boxPlus(state1.getOrientationBaseToWorld().boxMinus(state0.getOrientationBaseToWorld()) * t);
  jointPositions_ = state0.getJointPositions() * (1.0 - t) + state1.getJointPositions() * t;
}

AnymalState AnymalState::boxPlus(double delta, unsigned int uIndex, bool useQuaternion) const {
  AnymalState res = *this;
  switch (uIndex) {
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::L_X): {
      Position positionWorldToBaseInWorldFrame = positionWorldToBaseInWorldFrame_;
      positionWorldToBaseInWorldFrame(0) += delta;
      res.setPositionWorldToBaseInWorldFrame(positionWorldToBaseInWorldFrame);
    } break;
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::L_Y): {
      Position positionWorldToBaseInWorldFrame = positionWorldToBaseInWorldFrame_;
      positionWorldToBaseInWorldFrame(1) += delta;
      res.setPositionWorldToBaseInWorldFrame(positionWorldToBaseInWorldFrame);
    } break;
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::L_Z): {
      Position positionWorldToBaseInWorldFrame = positionWorldToBaseInWorldFrame_;
      positionWorldToBaseInWorldFrame(2) += delta;
      res.setPositionWorldToBaseInWorldFrame(positionWorldToBaseInWorldFrame);
    } break;
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::A_X): {
      if (useQuaternion) {
        RotationQuaternion orientationWorldToBase =
            orientationBaseToWorld_.inverted().boxPlus(Eigen::Vector3d(-delta, 0.0, 0.0)).inverted();
        res.setOrientationBaseToWorld(orientationWorldToBase);
      } else {
        const kindr::EulerAnglesZyxD eulerAngles(orientationBaseToWorld_);
        res.setOrientationBaseToWorld(
            kindr::RotationQuaternionD(kindr::EulerAnglesZyxD(eulerAngles.vector() + Eigen::Vector3d(delta, 0.0, 0.0))));
      }
    } break;
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::A_Y): {
      if (useQuaternion) {
        RotationQuaternion orientationWorldToBase =
            orientationBaseToWorld_.inverted().boxPlus(Eigen::Vector3d(0.0, -delta, 0.0)).inverted();
        res.setOrientationBaseToWorld(orientationWorldToBase);
      } else {
        const kindr::EulerAnglesZyxD eulerAngles(orientationBaseToWorld_);
        res.setOrientationBaseToWorld(
            kindr::RotationQuaternionD(kindr::EulerAnglesZyxD(eulerAngles.vector() + Eigen::Vector3d(0.0, delta, 0.0))));
      }
    } break;
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::A_Z): {
      if (useQuaternion) {
        RotationQuaternion orientationWorldToBase =
            orientationBaseToWorld_.inverted().boxPlus(Eigen::Vector3d(0.0, 0.0, -delta)).inverted();
        res.setOrientationBaseToWorld(orientationWorldToBase);
      } else {
        const kindr::EulerAnglesZyxD eulerAngles(orientationBaseToWorld_);
        res.setOrientationBaseToWorld(
            kindr::RotationQuaternionD(kindr::EulerAnglesZyxD(eulerAngles.vector() + Eigen::Vector3d(0.0, 0.0, delta))));
      }
    } break;
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::LF_HAA):
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::LF_HFE):
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::LF_KFE):
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::RF_HAA):
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::RF_HFE):
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::RF_KFE):
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::LH_HAA):
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::LH_HFE):
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::LH_KFE):
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::RH_HAA):
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::RH_HFE):
    case static_cast<unsigned int>(AD::GeneralizedVelocitiesEnum::RH_KFE): {
      JointPositions jointPositions = jointPositions_;
      jointPositions(uIndex - 6) += delta;
      res.setJointPositions(jointPositions);
    } break;
    default:
      throw std::runtime_error("AnymalState::boxPlus Wrong index!");
  }
  return res;
}

const Pose& AnymalState::getFrameTransform(const AT::FrameTransformEnum& transformEnum) const {
  switch (transformEnum) {
    case AT::FrameTransformEnum::FootprintToOdom:
      return poseFootprintToOdom_;
    case AT::FrameTransformEnum::FeetcenterToOdom:
      return poseFeetcenterToOdom_;
    default:
      throw std::runtime_error("AnymalState::getFrameTransform: pose does not exist!");
  }
}

void AnymalState::setFrameTransform(const AT::FrameTransformEnum& transformEnum, const Pose& pose) {
  switch (transformEnum) {
    case AT::FrameTransformEnum::FootprintToOdom: {
      poseFootprintToOdom_ = pose;
    } break;
    case AT::FrameTransformEnum::FeetcenterToOdom: {
      poseFeetcenterToOdom_ = pose;
    } break;
    default:
      throw std::runtime_error("AnymalState::setFrameTransform: pose does not exist!");
  }
}

std::ostream& operator<<(std::ostream& out, const AnymalState& state) {
  out << std::fixed << std::setw(11) << std::setprecision(6) << std::showpoint << std::right << std::showpos;
  // std::setfill( ' ' )
  out << "Position:";
  out << " x: " << state.positionWorldToBaseInWorldFrame_.x();
  out << " y: " << state.positionWorldToBaseInWorldFrame_.y();
  out << " z: " << state.positionWorldToBaseInWorldFrame_.z();
  out << std::endl;

  out << "Orientation:";
  out << std::endl;
  EulerAnglesZyx eulerAnglesZyx(state.orientationBaseToWorld_);
  eulerAnglesZyx.setUnique();
  out << " EulerZyx: ";
  out << " z: " << eulerAnglesZyx.z();
  out << " y: " << eulerAnglesZyx.y();
  out << " x: " << eulerAnglesZyx.x();
  out << std::endl;

  out << " Quaternion: ";
  out << " w: " << state.orientationBaseToWorld_.w();
  out << " x: " << state.orientationBaseToWorld_.x();
  out << " y: " << state.orientationBaseToWorld_.y();
  out << " z: " << state.orientationBaseToWorld_.z();
  out << std::endl;

  out << "Linear velocity: ";
  out << " x: " << state.linearVelocityBaseInWorldFrame_.x();
  out << " y: " << state.linearVelocityBaseInWorldFrame_.y();
  out << " z: " << state.linearVelocityBaseInWorldFrame_.z();
  out << std::endl;

  out << "Angular velocity:";
  out << " x: " << state.angularVelocityBaseInBaseFrame_.x();
  out << " y: " << state.angularVelocityBaseInBaseFrame_.y();
  out << " z: " << state.angularVelocityBaseInBaseFrame_.z();
  out << std::endl;

  out << "Joint positions:";
  //  out << " LF_HAA: " << state.jointPositions_(0);
  //  out << " LF_HFE: " << state.jointPositions_(1);
  //  out << " LF_KFE: " << state.jointPositions_(2);
  //  out << " RF_HAA: " << state.jointPositions_(3);
  //  out << " RF_HFE: " << state.jointPositions_(4);
  //  out << " RF_KFE: " << state.jointPositions_(5);
  //  out << " LH_HAA: " << state.jointPositions_(6);
  //  out << " LH_HFE: " << state.jointPositions_(7);
  //  out << " LH_KFE: " << state.jointPositions_(8);
  //  out << " RH_HAA: " << state.jointPositions_(9);
  //  out << " RH_HFE: " << state.jointPositions_(10);
  //  out << " RH_KFE: " << state.jointPositions_(11);
  out << std::endl;
  out << " LF_HAA: " << state.jointPositions_(0);
  out << " RF_HAA: " << state.jointPositions_(3);
  out << " LH_HAA: " << state.jointPositions_(6);
  out << " RH_HAA: " << state.jointPositions_(9);
  out << std::endl;

  out << " LF_HFE: " << state.jointPositions_(1);
  out << " RF_HFE: " << state.jointPositions_(4);
  out << " LH_HFE: " << state.jointPositions_(7);
  out << " RH_HFE: " << state.jointPositions_(10);
  out << std::endl;

  out << " LF_KFE: " << state.jointPositions_(2);
  out << " RF_KFE: " << state.jointPositions_(5);
  out << " LH_KFE: " << state.jointPositions_(8);
  out << " RH_KFE: " << state.jointPositions_(11);
  out << std::endl;

  out << "Joint velocities:";
  //  out << " LF_HAA: " << state.jointVelocities_(0);
  //  out << " LF_HFE: " << state.jointVelocities_(1);
  //  out << " LF_KFE: " << state.jointVelocities_(2);
  //  out << " RF_HAA: " << state.jointVelocities_(3);
  //  out << " RF_HFE: " << state.jointVelocities_(4);
  //  out << " RF_KFE: " << state.jointVelocities_(5);
  //  out << " LH_HAA: " << state.jointVelocities_(6);
  //  out << " LH_HFE: " << state.jointVelocities_(7);
  //  out << " LH_KFE: " << state.jointVelocities_(8);
  //  out << " RH_HAA: " << state.jointVelocities_(9);
  //  out << " RH_HFE: " << state.jointVelocities_(10);
  //  out << " RH_KFE: " << state.jointVelocities_(11);
  out << std::endl;
  out << " LF_HAA: " << state.jointVelocities_(0);
  out << " RF_HAA: " << state.jointVelocities_(3);
  out << " LH_HAA: " << state.jointVelocities_(6);
  out << " RH_HAA: " << state.jointVelocities_(9);
  out << std::endl;

  out << " LF_HFE: " << state.jointVelocities_(1);
  out << " RF_HFE: " << state.jointVelocities_(4);
  out << " LH_HFE: " << state.jointVelocities_(7);
  out << " RH_HFE: " << state.jointVelocities_(10);
  out << std::endl;

  out << " LF_KFE: " << state.jointVelocities_(2);
  out << " RF_KFE: " << state.jointVelocities_(5);
  out << " LH_KFE: " << state.jointVelocities_(8);
  out << " RH_KFE: " << state.jointVelocities_(11);
  out << std::endl;

  return out;
}

}  // namespace anymal_model
