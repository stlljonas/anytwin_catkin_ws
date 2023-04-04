/*
 * MissionControlSpeedFilter.cpp
 *
 *  Created on: Mar 7, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

// signal logger
#include <signal_logger/signal_logger.hpp>

// loco
#include "loco/mission_control/MissionControlSpeedFilter.hpp"

// stl
#include <limits>

// robot utils
#include "robot_utils/math/math.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

namespace loco {

MissionControlSpeedFilter::MissionControlSpeedFilter()
    : filteredBaseTwistInControlFrame_(),
      maximumBaseTwistInControlFrame_(
          LinearVelocity(std::numeric_limits<LinearVelocity::Scalar>::max(), std::numeric_limits<LinearVelocity::Scalar>::max(),
                         std::numeric_limits<LinearVelocity::Scalar>::max()),
          LocalAngularVelocity(std::numeric_limits<LinearVelocity::Scalar>::max(), std::numeric_limits<LinearVelocity::Scalar>::max(),
                               std::numeric_limits<LinearVelocity::Scalar>::max())),
      filteredVelocities_(3),
      enableRateLimiter_(false),
      filteredPositions_(3),
      filteredRotations_(3) {}

bool MissionControlSpeedFilter::initialize(double dt) {
  filteredBaseTwistInControlFrame_ = Twist();
  unfilteredBaseTwistInControlFrame_ = Twist();

  if (!enableRateLimiter_) {
    filteredVelocities_[0].reset();
    filteredVelocities_[1].reset();
    filteredVelocities_[2].reset();
  } else {
    rateLimiterVelocities_.reset();
  }

  filteredPositions_[0].reset();
  filteredPositions_[1].reset();
  filteredPositions_[2].reset();

  filteredRotations_[0].reset();
  filteredRotations_[1].reset();
  filteredRotations_[2].reset();

  return true;
}

bool MissionControlSpeedFilter::advance(double dt) {
  /*********
   * Twist *
   *********/

  if (!enableRateLimiter_) {
    filteredVelocities_[0].update(unfilteredBaseTwistInControlFrame_.getTranslationalVelocity().x());
    filteredVelocities_[1].update(unfilteredBaseTwistInControlFrame_.getTranslationalVelocity().y());
    filteredVelocities_[2].update(unfilteredBaseTwistInControlFrame_.getRotationalVelocity().z());
  } else {
    Eigen::Vector3d unfilteredBaseTwist;
    unfilteredBaseTwist.head<2>() = unfilteredBaseTwistInControlFrame_.getTranslationalVelocity().toImplementation().head<2>();
    unfilteredBaseTwist.z() = unfilteredBaseTwistInControlFrame_.getRotationalVelocity().z();
    rateLimiterVelocities_.update(unfilteredBaseTwist, dt);
  }

  double headingVel = 0.0;
  double lateralVel = 0.0;
  double turningVel = 0.0;

  if (!enableRateLimiter_) {
    headingVel = filteredVelocities_[0].val();
    lateralVel = filteredVelocities_[1].val();
    turningVel = filteredVelocities_[2].val();
  } else {
    headingVel = rateLimiterVelocities_.getValue().x();
    lateralVel = rateLimiterVelocities_.getValue().y();
    turningVel = rateLimiterVelocities_.getValue().z();
  }

  robot_utils::boundToRange(&headingVel, -maximumBaseTwistInControlFrame_.getTranslationalVelocity().x(),
                            maximumBaseTwistInControlFrame_.getTranslationalVelocity().x());
  robot_utils::boundToRange(&lateralVel, -maximumBaseTwistInControlFrame_.getTranslationalVelocity().y(),
                            maximumBaseTwistInControlFrame_.getTranslationalVelocity().y());
  robot_utils::boundToRange(&turningVel, -maximumBaseTwistInControlFrame_.getRotationalVelocity().z(),
                            maximumBaseTwistInControlFrame_.getRotationalVelocity().z());

  const LinearVelocity linearVelocity(headingVel, lateralVel, 0.0);
  const LocalAngularVelocity angularVelocity(0.0, 0.0, turningVel);

  filteredBaseTwistInControlFrame_ = Twist(linearVelocity, angularVelocity);
  /*********/

  /************
   * Position *
   ************/
  filteredPositions_[0].update(unfilteredPositionOffsetInWorldFrame_.x());
  filteredPositions_[1].update(unfilteredPositionOffsetInWorldFrame_.y());
  filteredPositions_[2].update(unfilteredPositionOffsetInWorldFrame_.z());

  double headingOffset = filteredPositions_[0].val();
  double lateralOffset = filteredPositions_[1].val();
  double heightOffset = filteredPositions_[2].val();

  robot_utils::boundToRange(&headingOffset, minimalDesiredPositionOffsetInWorldFrame_.x(), maximalDesiredPositionOffsetInWorldFrame_.x());
  robot_utils::boundToRange(&lateralOffset, minimalDesiredPositionOffsetInWorldFrame_.y(), maximalDesiredPositionOffsetInWorldFrame_.y());
  robot_utils::boundToRange(&heightOffset, minimalDesiredPositionOffsetInWorldFrame_.z(), maximalDesiredPositionOffsetInWorldFrame_.z());

  filteredPositionOffsetInWorldFrame_ = Position(headingOffset, lateralOffset, heightOffset);
  /************/

  /***************
   * Orientation *
   ***************/
  EulerAnglesZyx unfilteredDesiredEulerAnglesZyx(unfilteredDesiredOrientationControlToBase_);
  unfilteredDesiredEulerAnglesZyx.setUnique();
  EulerAnglesZyx minEulerAnglesZyx(minimalOrientationControlToBase_);
  minEulerAnglesZyx.setUnique();
  EulerAnglesZyx maxEulerAnglesZyx(maximalOrientationControlToBase_);
  maxEulerAnglesZyx.setUnique();

  filteredRotations_[0].update(unfilteredDesiredEulerAnglesZyx.roll());
  filteredRotations_[1].update(unfilteredDesiredEulerAnglesZyx.pitch());
  filteredRotations_[2].update(unfilteredDesiredEulerAnglesZyx.yaw());

  double rollOffset = filteredRotations_[0].val();
  double pitchOffset = filteredRotations_[1].val();
  double yawOffset = filteredRotations_[2].val();

  robot_utils::boundToRange(&rollOffset, minEulerAnglesZyx.roll(), maxEulerAnglesZyx.roll());
  robot_utils::boundToRange(&pitchOffset, minEulerAnglesZyx.pitch(), maxEulerAnglesZyx.pitch());
  robot_utils::boundToRange(&yawOffset, minEulerAnglesZyx.yaw(), maxEulerAnglesZyx.yaw());

  EulerAnglesZyx filteredDesiredEulerAnglesZyx(yawOffset, pitchOffset, rollOffset);
  filteredDesiredOrientationControlToBase_(filteredDesiredEulerAnglesZyx.getUnique());
  /***************/

  minimalPoseOffset_ = Pose(minimalDesiredPositionOffsetInWorldFrame_, RotationQuaternion(minimalOrientationControlToBase_));
  maximalPoseOffset_ = Pose(maximalDesiredPositionOffsetInWorldFrame_, RotationQuaternion(maximalOrientationControlToBase_));

  return true;
}

void MissionControlSpeedFilter::setUnfilteredDesiredBaseTwistInControlFrame(const Twist& twist) {
  unfilteredBaseTwistInControlFrame_ = twist;
}

void MissionControlSpeedFilter::setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(const Position& position) {
  unfilteredPositionOffsetInWorldFrame_ = position;
}

const Twist& MissionControlSpeedFilter::getDesiredBaseTwistInControlFrame() const {
  return filteredBaseTwistInControlFrame_;
}

const Twist& MissionControlSpeedFilter::getMaximumBaseTwistInControlFrame() const {
  return maximumBaseTwistInControlFrame_;
}

void MissionControlSpeedFilter::setMaximumBaseTwistInControlFrame(Twist maximumBaseTwistInControlFrame) {
  maximumBaseTwistInControlFrame_ = maximumBaseTwistInControlFrame;
}

const Pose& MissionControlSpeedFilter::getMinimalPoseOffset() const {
  return minimalPoseOffset_;
}

const Pose& MissionControlSpeedFilter::getMaximalPoseOffset() const {
  return maximalPoseOffset_;
}

const RotationQuaternion& MissionControlSpeedFilter::getFilteredDesiredOrientationOffset() const {
  return filteredDesiredOrientationControlToBase_;
}

void MissionControlSpeedFilter::setUnfilteredDesiredOrientationOffset(const RotationQuaternion& unfilteredDesiredOrientationControlToBase) {
  unfilteredDesiredOrientationControlToBase_ = unfilteredDesiredOrientationControlToBase;
}

const Position& MissionControlSpeedFilter::getFilteredPositionOffsetInWorldFrame() const {
  return filteredPositionOffsetInWorldFrame_;
}

bool MissionControlSpeedFilter::loadParameters(const TiXmlHandle& handle) {
  TiXmlElement* pElem;

  /* maximum */
  TiXmlHandle hMaxSpeed(handle.FirstChild("Mission").FirstChild("Speed").FirstChild("Maximum"));
  pElem = hMaxSpeed.Element();
  if (pElem == nullptr) {
    printf("Could not find Mission:Speed:Maximum\n");
    return false;
  } else {
    if (pElem->QueryDoubleAttribute("headingSpeed", &maximumBaseTwistInControlFrame_.getTranslationalVelocity().x()) != TIXML_SUCCESS) {
      printf("Could not find Speed:Maximum:headingSpeed\n");
      return false;
    }
    if (pElem->QueryDoubleAttribute("lateralSpeed", &maximumBaseTwistInControlFrame_.getTranslationalVelocity().y()) != TIXML_SUCCESS) {
      printf("Could not find Speed:Maximum:lateralSpeed\n");
      return false;
    }

    if (pElem->QueryDoubleAttribute("turningSpeed", &maximumBaseTwistInControlFrame_.getRotationalVelocity().z()) != TIXML_SUCCESS) {
      printf("Could not find Speed:Maximum:turningSpeed\n");
      return false;
    }
  }

  /* Filter Alpha */
  double positionAlphaX = 0.01, positionAlphaY = 0.01, positionAlphaZ = 0.01;
  double rotationAlphaX = 0.01, rotationAlphaY = 0.01, rotationAlphaZ = 0.01;
  double velocityAlphaX = 0.03, velocityAlphaY = 0.01, velocityAlphaZ = 0.01;
  Eigen::Vector3d maxAcceleration = Eigen::Vector3d::Ones();
  Eigen::Vector3d maxDeceleration = Eigen::Vector3d::Ones();

  TiXmlHandle alphaHandle = handle;
  if (tinyxml_tools::getChildHandle(alphaHandle, handle, "Mission/Speed/RateLimits", false)) {
    bool enableRateLimiter = enableRateLimiter_;
    tinyxml_tools::loadParameter(enableRateLimiter_, alphaHandle, "enable", enableRateLimiter);
    if (enableRateLimiter_) {
      TiXmlHandle rateHandle = handle;
      if (tinyxml_tools::getChildHandle(rateHandle, handle, "Mission/Speed/RateLimits/Acceleration")) {
        tinyxml_tools::loadParameter(maxAcceleration.x(), rateHandle, "x");
        tinyxml_tools::loadParameter(maxAcceleration.y(), rateHandle, "y");
        tinyxml_tools::loadParameter(maxAcceleration.z(), rateHandle, "z");
      }

      if (tinyxml_tools::getChildHandle(rateHandle, handle, "Mission/Speed/RateLimits/Deceleration")) {
        tinyxml_tools::loadParameter(maxDeceleration.x(), rateHandle, "x");
        tinyxml_tools::loadParameter(maxDeceleration.y(), rateHandle, "y");
        tinyxml_tools::loadParameter(maxDeceleration.z(), rateHandle, "z");
      }
    }
  }
  if (tinyxml_tools::getChildHandle(alphaHandle, handle, "Mission/Speed/FilterAlpha/Position")) {
    tinyxml_tools::loadParameter(positionAlphaX, alphaHandle, "x");
    tinyxml_tools::loadParameter(positionAlphaY, alphaHandle, "y");
    tinyxml_tools::loadParameter(positionAlphaZ, alphaHandle, "z");
  }
  if (tinyxml_tools::getChildHandle(alphaHandle, handle, "Mission/Speed/FilterAlpha/Rotation")) {
    tinyxml_tools::loadParameter(rotationAlphaX, alphaHandle, "x");
    tinyxml_tools::loadParameter(rotationAlphaY, alphaHandle, "y");
    tinyxml_tools::loadParameter(rotationAlphaZ, alphaHandle, "z");
  }
  if (!enableRateLimiter_) {
    if (tinyxml_tools::getChildHandle(alphaHandle, handle, "Mission/Speed/FilterAlpha/Velocity")) {
      tinyxml_tools::loadParameter(velocityAlphaX, alphaHandle, "x");
      tinyxml_tools::loadParameter(velocityAlphaY, alphaHandle, "y");
      tinyxml_tools::loadParameter(velocityAlphaZ, alphaHandle, "z");
    }
  }

  filteredPositions_[0].setAlpha(positionAlphaX);
  filteredPositions_[1].setAlpha(positionAlphaY);
  filteredPositions_[2].setAlpha(positionAlphaZ);
  filteredRotations_[0].setAlpha(rotationAlphaX);
  filteredRotations_[1].setAlpha(rotationAlphaY);
  filteredRotations_[2].setAlpha(rotationAlphaZ);

  if (!enableRateLimiter_) {
    filteredVelocities_[0].setAlpha(velocityAlphaX);
    filteredVelocities_[1].setAlpha(velocityAlphaY);
    filteredVelocities_[2].setAlpha(velocityAlphaZ);
  } else {
    rateLimiterVelocities_.setParameters(maxAcceleration, maxDeceleration);
  }

  /*  Configuration */
  TiXmlHandle hConfiguration(handle.FirstChild("Mission").FirstChild("Configuration"));
  pElem = hConfiguration.Element();
  if (pElem == nullptr) {
    printf("Could not find Mission:Configuration\n");
    unfilteredPositionOffsetInWorldFrame_.x() = 0.0;
    unfilteredPositionOffsetInWorldFrame_.y() = 0.0;
    unfilteredPositionOffsetInWorldFrame_.z() = 0.0;
  } else {
    /* ---------------------------- Position ---------------------------- */

    TiXmlHandle hPosition(hConfiguration.FirstChild("Position"));
    pElem = hPosition.FirstChild("Initial").Element();
    if (pElem == nullptr) {
      printf("Could not find Configuration:Position:Initial\n");
      unfilteredPositionOffsetInWorldFrame_.x() = 0.0;
      unfilteredPositionOffsetInWorldFrame_.y() = 0.0;
      unfilteredPositionOffsetInWorldFrame_.z() = 0.0;
      return false;
    } else {
      if (pElem->QueryDoubleAttribute("x", &unfilteredPositionOffsetInWorldFrame_.x()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Initial:x\n");
        unfilteredPositionOffsetInWorldFrame_.x() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("y", &unfilteredPositionOffsetInWorldFrame_.y()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Initial:x\n");
        unfilteredPositionOffsetInWorldFrame_.y() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("z", &unfilteredPositionOffsetInWorldFrame_.z()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Initial:z\n");
        unfilteredPositionOffsetInWorldFrame_.z() = 0.0;
      }
    }

    pElem = hPosition.FirstChild("Minimal").Element();
    if (pElem == nullptr) {
      printf("Could not find Configuration:Position:Minimal\n");
      minimalDesiredPositionOffsetInWorldFrame_.x() = 0.0;
      minimalDesiredPositionOffsetInWorldFrame_.y() = 0.0;
      minimalDesiredPositionOffsetInWorldFrame_.z() = 0.0;
      return false;
    } else {
      if (pElem->QueryDoubleAttribute("x", &minimalDesiredPositionOffsetInWorldFrame_.x()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Minimal:x\n");
        minimalDesiredPositionOffsetInWorldFrame_.x() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("y", &minimalDesiredPositionOffsetInWorldFrame_.y()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Minimal:x\n");
        minimalDesiredPositionOffsetInWorldFrame_.y() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("z", &minimalDesiredPositionOffsetInWorldFrame_.z()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Minimal:z\n");
        minimalDesiredPositionOffsetInWorldFrame_.z() = 0.0;
      }
    }

    pElem = hPosition.FirstChild("Maximal").Element();
    if (pElem == nullptr) {
      printf("Could not find Configuration:Position:Maximal\n");
      maximalDesiredPositionOffsetInWorldFrame_.x() = 0.0;
      maximalDesiredPositionOffsetInWorldFrame_.y() = 0.0;
      maximalDesiredPositionOffsetInWorldFrame_.z() = 0.0;
      return false;
    } else {
      if (pElem->QueryDoubleAttribute("x", &maximalDesiredPositionOffsetInWorldFrame_.x()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Maximal:x\n");
        maximalDesiredPositionOffsetInWorldFrame_.x() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("y", &maximalDesiredPositionOffsetInWorldFrame_.y()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Maximal:x\n");
        maximalDesiredPositionOffsetInWorldFrame_.y() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("z", &maximalDesiredPositionOffsetInWorldFrame_.z()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Maximal:z\n");
        maximalDesiredPositionOffsetInWorldFrame_.z() = 0.0;
      }
    }

    /* ---------------------------- Orientation ---------------------------- */
    TiXmlHandle hOrientation(hConfiguration.FirstChild("Orientation"));

    pElem = hOrientation.FirstChild("Initial").Element();
    EulerAnglesZyx desiredOrientationControlToBaseEulerAnglesZyx;
    if (pElem == nullptr) {
      printf("Could not find Configuration:Orientation:Initial\n");
      desiredOrientationControlToBaseEulerAnglesZyx.setX(0.0);
      desiredOrientationControlToBaseEulerAnglesZyx.setY(0.0);
      desiredOrientationControlToBaseEulerAnglesZyx.setZ(0.0);
      return false;
    } else {
      double value = 0.0;
      if (pElem->QueryDoubleAttribute("x", &value) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Orientation:Initial:x\n");
        value = 0.0;
      }
      desiredOrientationControlToBaseEulerAnglesZyx.setX(value);

      if (pElem->QueryDoubleAttribute("y", &value) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Orientation:Initial:x\n");
        value = 0.0;
      }
      desiredOrientationControlToBaseEulerAnglesZyx.setY(value);

      if (pElem->QueryDoubleAttribute("z", &value) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Orientation:Initial:z\n");
        value = 0.0;
      }
      desiredOrientationControlToBaseEulerAnglesZyx.setZ(value);
    }

    unfilteredDesiredOrientationControlToBase_(desiredOrientationControlToBaseEulerAnglesZyx);

    /* Minimal */
    pElem = hOrientation.FirstChild("Minimal").Element();
    EulerAnglesZyx minimalOrientationControlToBaseEulerAnglesZyx;
    if (pElem == nullptr) {
      printf("Could not find Configuration:Orientation:Minimal\n");
      minimalOrientationControlToBaseEulerAnglesZyx.setX(0.0);
      minimalOrientationControlToBaseEulerAnglesZyx.setY(0.0);
      minimalOrientationControlToBaseEulerAnglesZyx.setZ(0.0);
      return false;
    } else {
      double value = 0.0;
      if (pElem->QueryDoubleAttribute("x", &value) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Orientation:Minimal:x\n");
        value = 0.0;
      }
      minimalOrientationControlToBaseEulerAnglesZyx.setX(value);

      if (pElem->QueryDoubleAttribute("y", &value) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Orientation:Minimal:x\n");
        value = 0.0;
      }
      minimalOrientationControlToBaseEulerAnglesZyx.setY(value);

      if (pElem->QueryDoubleAttribute("z", &value) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Orientation:Minimal:z\n");
        value = 0.0;
      }
      minimalOrientationControlToBaseEulerAnglesZyx.setZ(value);
    }
    minimalOrientationControlToBase_(minimalOrientationControlToBaseEulerAnglesZyx);

    /* Maximal */
    pElem = hOrientation.FirstChild("Maximal").Element();
    EulerAnglesZyx maximalOrientationControlToBaseEulerAnglesZyx;
    if (pElem == nullptr) {
      printf("Could not find Configuration:Orientation:Maximal\n");
      maximalOrientationControlToBaseEulerAnglesZyx.setX(0.0);
      maximalOrientationControlToBaseEulerAnglesZyx.setY(0.0);
      maximalOrientationControlToBaseEulerAnglesZyx.setZ(0.0);
      return false;
    } else {
      double value = 0.0;
      if (pElem->QueryDoubleAttribute("x", &value) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Orientation:Maximal:x\n");
        value = 0.0;
      }
      maximalOrientationControlToBaseEulerAnglesZyx.setX(value);

      if (pElem->QueryDoubleAttribute("y", &value) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Orientation:Maximal:x\n");
        value = 0.0;
      }
      maximalOrientationControlToBaseEulerAnglesZyx.setY(value);

      if (pElem->QueryDoubleAttribute("z", &value) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Orientation:Maximal:z\n");
        value = 0.0;
      }
      maximalOrientationControlToBaseEulerAnglesZyx.setZ(value);
    }
    maximalOrientationControlToBase_(maximalOrientationControlToBaseEulerAnglesZyx);

    minimalPoseOffset_ = Pose(minimalDesiredPositionOffsetInWorldFrame_, RotationQuaternion(minimalOrientationControlToBase_));
    maximalPoseOffset_ = Pose(maximalDesiredPositionOffsetInWorldFrame_, RotationQuaternion(maximalOrientationControlToBase_));
  }
  return true;
}

bool MissionControlSpeedFilter::setToInterpolated(const MissionControlBase& missionController1,
                                                  const MissionControlBase& missionController2, double t) {
  const auto& controller1 = dynamic_cast<const MissionControlSpeedFilter&>(missionController1);
  const auto& controller2 = dynamic_cast<const MissionControlSpeedFilter&>(missionController2);
  maximumBaseTwistInControlFrame_.getTranslationalVelocity().x() =
      robot_utils::linearlyInterpolate(controller1.getMaximumBaseTwistInControlFrame().getTranslationalVelocity().x(),
                                       controller2.getMaximumBaseTwistInControlFrame().getTranslationalVelocity().x(), 0.0, 1.0, t);
  maximumBaseTwistInControlFrame_.getTranslationalVelocity().y() =
      robot_utils::linearlyInterpolate(controller1.getMaximumBaseTwistInControlFrame().getTranslationalVelocity().y(),
                                       controller2.getMaximumBaseTwistInControlFrame().getTranslationalVelocity().y(), 0.0, 1.0, t);
  maximumBaseTwistInControlFrame_.getTranslationalVelocity().z() =
      robot_utils::linearlyInterpolate(controller1.getMaximumBaseTwistInControlFrame().getTranslationalVelocity().z(),
                                       controller2.getMaximumBaseTwistInControlFrame().getTranslationalVelocity().z(), 0.0, 1.0, t);
  maximumBaseTwistInControlFrame_.getRotationalVelocity().z() =
      robot_utils::linearlyInterpolate(controller1.getMaximumBaseTwistInControlFrame().getRotationalVelocity().z(),
                                       controller2.getMaximumBaseTwistInControlFrame().getRotationalVelocity().z(), 0.0, 1.0, t);

  minimalDesiredPositionOffsetInWorldFrame_ = robot_utils::linearlyInterpolate(
      controller1.getMinimalPositionOffsetInWorldFrame(), controller2.getMinimalPositionOffsetInWorldFrame(), 0.0, 1.0, t);

  maximalDesiredPositionOffsetInWorldFrame_ = robot_utils::linearlyInterpolate(
      controller1.getMaximalPositionOffsetInWorldFrame(), controller2.getMaximalPositionOffsetInWorldFrame(), 0.0, 1.0, t);
  minimalPoseOffset_ = Pose(minimalDesiredPositionOffsetInWorldFrame_, RotationQuaternion(minimalOrientationControlToBase_));
  maximalPoseOffset_ = Pose(maximalDesiredPositionOffsetInWorldFrame_, RotationQuaternion(maximalOrientationControlToBase_));

  return true;
}

const Position& MissionControlSpeedFilter::getMinimalPositionOffsetInWorldFrame() const {
  return minimalDesiredPositionOffsetInWorldFrame_;
}

const Position& MissionControlSpeedFilter::getMaximalPositionOffsetInWorldFrame() const {
  return maximalDesiredPositionOffsetInWorldFrame_;
}

const EulerAnglesZyx& MissionControlSpeedFilter::getMinimalDesiredOrientationOffset() const {
  return minimalOrientationControlToBase_;
}

const EulerAnglesZyx& MissionControlSpeedFilter::getMaximalDesiredOrientationOffset() const {
  return maximalOrientationControlToBase_;
}

std::ostream& operator<<(std::ostream& out, const MissionControlSpeedFilter& speedFilter) {
  return out;
}

bool MissionControlSpeedFilter::addVariablesToLog(const std::string& ns) const {
  signal_logger::add(filteredBaseTwistInControlFrame_, "filtered/baseTwistInControlFrame", ns);
  signal_logger::add(filteredDesiredOrientationControlToBase_, "filtered/desiredOrientationControlToBase", ns);
  signal_logger::add(filteredPositionOffsetInWorldFrame_, "filtered/positionOffsetInWorldFrame", ns);
  signal_logger::add(maximalDesiredPositionOffsetInWorldFrame_, "bounds/maximalDesiredPositionOffsetInWorldFrame", ns);
  signal_logger::add(maximalOrientationControlToBase_, "bounds/maximalOrientationControlToBase", ns);
  signal_logger::add(maximalPoseOffset_, "bounds/maximalPoseOffset", ns);
  signal_logger::add(maximumBaseTwistInControlFrame_, "bounds/maximumBaseTwistInControlFrame", ns);
  signal_logger::add(minimalDesiredPositionOffsetInWorldFrame_, "bounds/minimalDesiredPositionOffsetInWorldFrame", ns);
  signal_logger::add(minimalOrientationControlToBase_, "bounds/minimalOrientationControlToBase", ns);
  signal_logger::add(minimalPoseOffset_, "bounds/minimalPoseOffset", ns);
  signal_logger::add(unfilteredBaseTwistInControlFrame_, "unfiltered/baseTwistInControlFrame", ns);
  signal_logger::add(unfilteredDesiredOrientationControlToBase_, "unfiltered/desiredOrientationControlToBase", ns);
  signal_logger::add(unfilteredPositionOffsetInWorldFrame_, "unfiltered/positionOffsetInWorldFrame", ns);
  return true;
}

} /* namespace loco */
