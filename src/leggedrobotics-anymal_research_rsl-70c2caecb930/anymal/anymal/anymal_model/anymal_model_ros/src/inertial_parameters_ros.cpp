/*!
 * @file     inertial_parameters_ros.cpp
 * @author   Ioannis Mandralis
 * @date     March, 2020
 */

#include <ros/package.h>
#include <anymal_model_ros/inertial_parameters_ros.hpp>

namespace anymal_model_ros {

bool updateInertialParameters(anymal_model::AnymalModel* anymalModel, ros::NodeHandle& nodeHandle) {
  // Loop through the bodies
  for (const auto& body : anymalModel->getBodyContainer()) {
    // Skip the root body and any fixed bodies.
    if (body->getBodyId() == anymalModel->getRbdlModel().rootId || body->getIsFixedBody()) {
      continue;
    }

    // Declare the inertial parameters
    double mass = 0.0;
    Eigen::Vector3d centerOfMassInBodyFrame = Eigen::Vector3d::Zero();
    Eigen::Matrix3d inertiaAtCom = Eigen::Matrix3d::Zero();

    // Get the current branch and body name
    std::string branchName = anymal_description::AnymalDescription::mapKeyEnumToKeyName(body->getBranchEnum());
    std::string bodyName = body->getName();

    // Load the parameters from the inertial_parameters.yaml to their respective placeholders.
    param_io::getParam(nodeHandle, "inertial_parameter_estimation/" + branchName + "/" + bodyName + "/m", mass);
    param_io::getParam(nodeHandle, "inertial_parameter_estimation/" + branchName + "/" + bodyName + "/cx", centerOfMassInBodyFrame(0));
    param_io::getParam(nodeHandle, "inertial_parameter_estimation/" + branchName + "/" + bodyName + "/cy", centerOfMassInBodyFrame(1));
    param_io::getParam(nodeHandle, "inertial_parameter_estimation/" + branchName + "/" + bodyName + "/cz", centerOfMassInBodyFrame(2));
    param_io::getParam(nodeHandle, "inertial_parameter_estimation/" + branchName + "/" + bodyName + "/Ixx", inertiaAtCom(0, 0));
    param_io::getParam(nodeHandle, "inertial_parameter_estimation/" + branchName + "/" + bodyName + "/Ixy", inertiaAtCom(0, 1));
    param_io::getParam(nodeHandle, "inertial_parameter_estimation/" + branchName + "/" + bodyName + "/Ixz", inertiaAtCom(0, 2));
    param_io::getParam(nodeHandle, "inertial_parameter_estimation/" + branchName + "/" + bodyName + "/Iyy", inertiaAtCom(1, 1));
    param_io::getParam(nodeHandle, "inertial_parameter_estimation/" + branchName + "/" + bodyName + "/Iyz", inertiaAtCom(1, 2));
    param_io::getParam(nodeHandle, "inertial_parameter_estimation/" + branchName + "/" + bodyName + "/Izz", inertiaAtCom(2, 2));

    // Make the inertia tensor symmetric
    inertiaAtCom(1, 0) = inertiaAtCom(0, 1);
    inertiaAtCom(2, 0) = inertiaAtCom(0, 2);
    inertiaAtCom(2, 1) = inertiaAtCom(1, 2);

    // Information for debug
    MELO_DEBUG_STREAM("mass is: " << mass << std::endl)
    MELO_DEBUG_STREAM("center of mass is: " << centerOfMassInBodyFrame << std::endl)
    MELO_DEBUG_STREAM("inertia is: " << inertiaAtCom << std::endl)

    // Set the updated inertia properties.
    anymalModel->setBodyInertiaProperties(body->getBodyEnum(), mass, centerOfMassInBodyFrame, inertiaAtCom);
  }

  // Check if the model parameters are valid.
  return anymalModel->validateModelParameters();
}
}  // namespace anymal_model_ros
