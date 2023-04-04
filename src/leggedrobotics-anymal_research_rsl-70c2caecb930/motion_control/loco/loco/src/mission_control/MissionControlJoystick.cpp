/*
 * MissionControlJoystick.cpp
 *
 *  Created on: Mar 7, 2014
 *      Author: gech
 */

// loco
#include "loco/mission_control/MissionControlJoystick.hpp"

// stl
#include <limits>

// robot utils
#include "robot_utils/math/math.hpp"

namespace loco {

MissionControlJoystick::MissionControlJoystick(robot_utils::Joystick* joyStick)
    : joyStick_(joyStick),
      baseTwistInControlFrame_(),
      maximumBaseTwistInControlFrame_(
          LinearVelocity(std::numeric_limits<LinearVelocity::Scalar>::max(), std::numeric_limits<LinearVelocity::Scalar>::max(),
                         std::numeric_limits<LinearVelocity::Scalar>::max()),
          LocalAngularVelocity(std::numeric_limits<LinearVelocity::Scalar>::max(), std::numeric_limits<LinearVelocity::Scalar>::max(),
                               std::numeric_limits<LinearVelocity::Scalar>::max())),
      desiredPositionMiddleOfFeetToBaseInWorldFrame_(0.0, 0.0, 0.42),
      minimalPositionMiddleOfFeetToBaseInWorldFrame_(0.0, 0.0, 0.24),
      maximalPositionMiddleOfFeetToBaseInWorldFrame_(0.0, 0.0, 0.44),
      desiredOrientationControlToBase_(),
      minimalOrientationControlToBase_(),
      maximalOrientationControlToBase_(),
      filteredVelocities_(3, basic_filters::FilteredDouble()) {
  filteredVelocities_[0].setAlpha(0.005);
  filteredVelocities_[1].setAlpha(0.05);
  filteredVelocities_[2].setAlpha(0.05);
  minimalPoseOffset_ = Pose(minimalPositionMiddleOfFeetToBaseInWorldFrame_, RotationQuaternion(minimalOrientationControlToBase_));
  maximalPoseOffset_ = Pose(maximalPositionMiddleOfFeetToBaseInWorldFrame_, RotationQuaternion(maximalOrientationControlToBase_));
}

bool MissionControlJoystick::initialize(double dt) {
  return true;
}

bool MissionControlJoystick::advance(double dt) {
  const double maxHeadingVel = maximumBaseTwistInControlFrame_.getTranslationalVelocity().x();
  filteredVelocities_[0].update(joyStick_->getSagittal());
  double headingVel = filteredVelocities_[0].val();
  robot_utils::boundToRange(&headingVel, -maxHeadingVel, maxHeadingVel);

  const double maxLateralVel = maximumBaseTwistInControlFrame_.getTranslationalVelocity().y();
  filteredVelocities_[1].update(joyStick_->getCoronal());

  double lateralVel = filteredVelocities_[1].val();
  robot_utils::boundToRange(&lateralVel, -maxLateralVel, maxLateralVel);
  LinearVelocity linearVelocity(headingVel, lateralVel, 0.0);

  const double maxTurningVel = maximumBaseTwistInControlFrame_.getRotationalVelocity().z();
  filteredVelocities_[2].update(-joyStick_->getYaw());
  double turningVel = filteredVelocities_[2].val();
  robot_utils::boundToRange(&turningVel, -maxTurningVel, maxTurningVel);
  LocalAngularVelocity angularVelocity(0.0, 0.0, turningVel);

  baseTwistInControlFrame_ = Twist(linearVelocity, angularVelocity);

  /* -------- Position -------- */

  //  desiredPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.5*(maxHeight-minHeight)*(joyStick->getVertical()-minHeight) + maxHeight;
  desiredPositionMiddleOfFeetToBaseInWorldFrame_.x() = interpolateJoystickAxis(
      joyStick_->getSagittal(), minimalPositionMiddleOfFeetToBaseInWorldFrame_.x(), maximalPositionMiddleOfFeetToBaseInWorldFrame_.x());
  robot_utils::boundToRange(&desiredPositionMiddleOfFeetToBaseInWorldFrame_.x(), minimalPositionMiddleOfFeetToBaseInWorldFrame_.x(),
                            maximalPositionMiddleOfFeetToBaseInWorldFrame_.x());
  desiredPositionMiddleOfFeetToBaseInWorldFrame_.y() = interpolateJoystickAxis(
      joyStick_->getCoronal(), minimalPositionMiddleOfFeetToBaseInWorldFrame_.y(), maximalPositionMiddleOfFeetToBaseInWorldFrame_.y());
  robot_utils::boundToRange(&desiredPositionMiddleOfFeetToBaseInWorldFrame_.y(), minimalPositionMiddleOfFeetToBaseInWorldFrame_.y(),
                            maximalPositionMiddleOfFeetToBaseInWorldFrame_.y());
  desiredPositionMiddleOfFeetToBaseInWorldFrame_.z() = interpolateJoystickAxis(
      joyStick_->getVertical(), minimalPositionMiddleOfFeetToBaseInWorldFrame_.z(), maximalPositionMiddleOfFeetToBaseInWorldFrame_.z());
  robot_utils::boundToRange(&desiredPositionMiddleOfFeetToBaseInWorldFrame_.z(), minimalPositionMiddleOfFeetToBaseInWorldFrame_.z(),
                            maximalPositionMiddleOfFeetToBaseInWorldFrame_.z());

  /* -------- Orientation -------- */
  //  EulerAnglesZyx desEulerAnglesZyx;
  //  desEulerAnglesZyx.setRoll(interpolateJoystickAxis(joyStick->getRoll(), minimalOrientationControlToBase_.roll(),
  //  maximalOrientationControlToBase_.roll())); desEulerAnglesZyx.setRoll(robot_utils::boundToRange(desEulerAnglesZyx.roll(),
  //  minimalOrientationControlToBase_.roll(), maximalOrientationControlToBase_.roll()));
  //  desEulerAnglesZyx.setPitch(interpolateJoystickAxis(joyStick->getPitch(), minimalOrientationControlToBase_.pitch(),
  //  maximalOrientationControlToBase_.pitch())); desEulerAnglesZyx.setPitch(robot_utils::boundToRange(desEulerAnglesZyx.pitch(),
  //  minimalOrientationControlToBase_.pitch(), maximalOrientationControlToBase_.pitch()));
  //  desEulerAnglesZyx.setYaw(interpolateJoystickAxis(joyStick->getYaw(), minimalOrientationControlToBase_.yaw(),
  //  maximalOrientationControlToBase_.yaw())); desEulerAnglesZyx.setYaw(robot_utils::boundToRange(desEulerAnglesZyx.yaw(),
  //  minimalOrientationControlToBase_.yaw(), maximalOrientationControlToBase_.yaw())); desiredOrientationControlToBase_(desEulerAnglesZyx);

  EulerAnglesZyx desEulerAnglesZyx;
  desEulerAnglesZyx.setRoll(interpolateJoystickAxis(-joyStick_->getCoronal(), minimalOrientationControlToBase_.getUnique().roll(),
                                                    maximalOrientationControlToBase_.getUnique().roll()));
  desEulerAnglesZyx.setRoll(robot_utils::boundToRange(desEulerAnglesZyx.getUnique().roll(), minimalOrientationControlToBase_.roll(),
                                                      maximalOrientationControlToBase_.getUnique().roll()));
  desEulerAnglesZyx.setPitch(interpolateJoystickAxis(-joyStick_->getSagittal(), minimalOrientationControlToBase_.getUnique().pitch(),
                                                     maximalOrientationControlToBase_.getUnique().pitch()));
  desEulerAnglesZyx.setPitch(robot_utils::boundToRange(desEulerAnglesZyx.getUnique().pitch(),
                                                       minimalOrientationControlToBase_.getUnique().pitch(),
                                                       maximalOrientationControlToBase_.getUnique().pitch()));
  desEulerAnglesZyx.setYaw(interpolateJoystickAxis(-joyStick_->getYaw(), minimalOrientationControlToBase_.getUnique().yaw(),
                                                   maximalOrientationControlToBase_.getUnique().yaw()));
  desEulerAnglesZyx.setYaw(robot_utils::boundToRange(desEulerAnglesZyx.getUnique().yaw(),
                                                     minimalOrientationControlToBase_.getUnique().yaw(),
                                                     maximalOrientationControlToBase_.getUnique().yaw()));
  desiredOrientationControlToBase_(desEulerAnglesZyx.getUnique());

  minimalPoseOffset_ = Pose(minimalPositionMiddleOfFeetToBaseInWorldFrame_, RotationQuaternion(minimalOrientationControlToBase_));
  maximalPoseOffset_ = Pose(maximalPositionMiddleOfFeetToBaseInWorldFrame_, RotationQuaternion(maximalOrientationControlToBase_));

  return true;
}

double MissionControlJoystick::interpolateJoystickAxis(double value, double minValue, double maxValue) {
  return 0.5 * (maxValue - minValue) * (value + 1.0) + minValue;
}

const Twist& MissionControlJoystick::getDesiredBaseTwistInControlFrame() const {
  return baseTwistInControlFrame_;
}
const Position& MissionControlJoystick::getDesiredPositionMiddleOfFeetToBaseInWorldFrame() const {
  return desiredPositionMiddleOfFeetToBaseInWorldFrame_;
}
const RotationQuaternion& MissionControlJoystick::getDesiredOrientationControlToBase() const {
  return desiredOrientationControlToBase_;
}

const Twist& MissionControlJoystick::getMaximumBaseTwistInControlFrame() const {
  return maximumBaseTwistInControlFrame_;
}

const Pose& MissionControlJoystick::getMinimalPoseOffset() const {
  return minimalPoseOffset_;
}

const Pose& MissionControlJoystick::getMaximalPoseOffset() const {
  return maximalPoseOffset_;
}

bool MissionControlJoystick::loadParameters(const TiXmlHandle& handle) {
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

  /*  Configuration */
  TiXmlHandle hConfiguration(handle.FirstChild("Mission").FirstChild("Configuration"));
  pElem = hConfiguration.Element();
  if (pElem == nullptr) {
    printf("Could not find Mission:Configuration\n");
    desiredPositionMiddleOfFeetToBaseInWorldFrame_.x() = 0.0;
    desiredPositionMiddleOfFeetToBaseInWorldFrame_.y() = 0.0;
    desiredPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.0;
  } else {
    /* ---------------------------- Position ---------------------------- */

    TiXmlHandle hPosition(hConfiguration.FirstChild("Position"));
    pElem = hPosition.FirstChild("Initial").Element();
    if (pElem == nullptr) {
      printf("Could not find Configuration:Position:Initial\n");
      desiredPositionMiddleOfFeetToBaseInWorldFrame_.x() = 0.0;
      desiredPositionMiddleOfFeetToBaseInWorldFrame_.y() = 0.0;
      desiredPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.0;
      return false;
    } else {
      if (pElem->QueryDoubleAttribute("x", &desiredPositionMiddleOfFeetToBaseInWorldFrame_.x()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Initial:x\n");
        desiredPositionMiddleOfFeetToBaseInWorldFrame_.x() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("y", &desiredPositionMiddleOfFeetToBaseInWorldFrame_.y()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Initial:x\n");
        desiredPositionMiddleOfFeetToBaseInWorldFrame_.y() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("z", &desiredPositionMiddleOfFeetToBaseInWorldFrame_.z()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Initial:z\n");
        desiredPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.0;
      }
    }

    pElem = hPosition.FirstChild("Minimal").Element();
    if (pElem == nullptr) {
      printf("Could not find Configuration:Position:Minimal\n");
      minimalPositionMiddleOfFeetToBaseInWorldFrame_.x() = 0.0;
      minimalPositionMiddleOfFeetToBaseInWorldFrame_.y() = 0.0;
      minimalPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.0;
      return false;
    } else {
      if (pElem->QueryDoubleAttribute("x", &minimalPositionMiddleOfFeetToBaseInWorldFrame_.x()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Minimal:x\n");
        minimalPositionMiddleOfFeetToBaseInWorldFrame_.x() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("y", &minimalPositionMiddleOfFeetToBaseInWorldFrame_.y()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Minimal:x\n");
        minimalPositionMiddleOfFeetToBaseInWorldFrame_.y() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("z", &minimalPositionMiddleOfFeetToBaseInWorldFrame_.z()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Minimal:z\n");
        minimalPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.0;
      }
    }

    pElem = hPosition.FirstChild("Maximal").Element();
    if (pElem == nullptr) {
      printf("Could not find Configuration:Position:Maximal\n");
      maximalPositionMiddleOfFeetToBaseInWorldFrame_.x() = 0.0;
      maximalPositionMiddleOfFeetToBaseInWorldFrame_.y() = 0.0;
      maximalPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.0;
      return false;
    } else {
      if (pElem->QueryDoubleAttribute("x", &maximalPositionMiddleOfFeetToBaseInWorldFrame_.x()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Maximal:x\n");
        maximalPositionMiddleOfFeetToBaseInWorldFrame_.x() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("y", &maximalPositionMiddleOfFeetToBaseInWorldFrame_.y()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Maximal:x\n");
        maximalPositionMiddleOfFeetToBaseInWorldFrame_.y() = 0.0;
      }
      if (pElem->QueryDoubleAttribute("z", &maximalPositionMiddleOfFeetToBaseInWorldFrame_.z()) != TIXML_SUCCESS) {
        printf("Could not find Configuration:Position:Maximal:z\n");
        maximalPositionMiddleOfFeetToBaseInWorldFrame_.z() = 0.0;
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
    desiredOrientationControlToBase_(desiredOrientationControlToBaseEulerAnglesZyx);

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
  }

  return true;
}

std::ostream& operator<<(std::ostream& out, const MissionControlJoystick& joystick) {
  out << "joyStick position: " << joystick.joyStick_->getSagittal() << " " << joystick.joyStick_->getCoronal() << " "
      << joystick.joyStick_->getVertical() << std::endl;
  out << "desiredPositionMiddleOfFeetToBaseInWorldFrame: " << joystick.desiredPositionMiddleOfFeetToBaseInWorldFrame_ << std::endl;
  out << "minimalPositionMiddleOfFeetToBaseInWorldFrame: " << joystick.minimalPositionMiddleOfFeetToBaseInWorldFrame_ << std::endl;
  out << "maximalPositionMiddleOfFeetToBaseInWorldFrame: " << joystick.maximalPositionMiddleOfFeetToBaseInWorldFrame_ << std::endl;
  out << "desiredOrientationControlToBase (EulerAnglesZyx): " << EulerAnglesZyx(joystick.desiredOrientationControlToBase_).getUnique()
      << std::endl;
  out << "minimalOrientationControlToBase (EulerAnglesZyx): " << EulerAnglesZyx(joystick.minimalOrientationControlToBase_).getUnique()
      << std::endl;
  out << "maximalOrientationControlToBase (EulerAnglesZyx): " << EulerAnglesZyx(joystick.maximalOrientationControlToBase_).getUnique()
      << std::endl;
  return out;
}

} /* namespace loco */
