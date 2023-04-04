/*
 * Arm.cpp
 *
 *  Created on: Sep 3, 2015
 *      Author: Christian Gehring, Peter Fankhauser
 */
#include "loco_ros/visualization/Arm.hpp"

namespace loco_ros {

Arm::Arm() {
}

Arm::~Arm() {
}

bool Arm::initialize(ros::NodeHandle& nodeHandle, const std::string& topic) {
    armPoints_.initialize(nodeHandle, topic);
    armPoints_.addPoint(loco::Position::Zero(), 0.1, Color(ColorEnum::AZURE));
    armPoints_.addPoint(loco::Position::Zero(), 0.1, Color(ColorEnum::AZURE));
    armPoints_.addPoint(loco::Position::Zero(), 0.1, Color(ColorEnum::GOLDENROD));
    armPoints_.addPoint(loco::Position::Zero(), 0.1, Color(ColorEnum::GOLDENROD));

    return true;
}

bool Arm::shutdown() {
    armPoints_.shutdown();
    return true;
}

bool Arm::update(const loco::ArmBase& arm) {
    armPoints_.updatePoint(arm.getEndEffector().getStateMeasured(loco::TimePoint::Now, loco::EndEffectorEnum::Origin).getPositionWorldToEndEffectorInWorldFrame(), 0);
    armPoints_.updatePoint(arm.getEndEffector().getStateMeasured(loco::TimePoint::Now, loco::EndEffectorContactEnum::Contact).getPositionWorldToEndEffectorInWorldFrame(), 1);
    armPoints_.updatePoint(arm.getEndEffector().getStateDesired(loco::TimePoint::Now, loco::EndEffectorEnum::Origin).getPositionWorldToEndEffectorInWorldFrame(), 2);
    armPoints_.updatePoint(arm.getEndEffector().getStateDesired(loco::TimePoint::Now, loco::EndEffectorContactEnum::Contact).getPositionWorldToEndEffectorInWorldFrame(), 3);
    return true;
}

bool Arm::publish() {
    armPoints_.visualize();
    return true;
}

} /* namespace loco_ros */
