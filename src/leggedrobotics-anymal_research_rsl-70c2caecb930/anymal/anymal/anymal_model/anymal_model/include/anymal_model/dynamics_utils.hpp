/*
 * dynamics_utils.hpp
 *
 *  Created on: Nov 22, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

// anymal model
#include <anymal_model/AnymalModel.hpp>

namespace anymal_model {

bool computeLimbEndPointReferenceAccelerationFromDesiredMotion(
    const AD::LimbEnum limb, const Eigen::Vector3d& desiredPositionWorldToFootInWorldFrame,
    const Eigen::Vector3d& desiredLinearVelocityFootInWorldFrame, const Eigen::Vector3d& desiredLinearAccelerationFootInWorldFrame,
    const Eigen::Vector3d& proportionalGains, const Eigen::Vector3d& derivativeGains, const Eigen::Vector3d& feedforwardGains,
    const AnymalModel& model, Eigen::VectorXd& jointAccelerations);

bool computeInverseDynamicsLimbTorquesFromDesiredMotion(const AD::LimbEnum limb,
                                                        const Eigen::Vector3d& desiredPositionBaseToFootInBaseFrame,
                                                        const Eigen::Vector3d& desiredLinearVelocityFootInBaseFrame,
                                                        const Eigen::Vector3d& desiredLinearAccelerationFootInBaseFrame,
                                                        const Eigen::Vector3d& proportionalGains, const Eigen::Vector3d& derivativeGains,
                                                        const Eigen::Vector3d& feedforwardGains, const AnymalModel& model,
                                                        Eigen::VectorXd& torques);

}  // namespace anymal_model
