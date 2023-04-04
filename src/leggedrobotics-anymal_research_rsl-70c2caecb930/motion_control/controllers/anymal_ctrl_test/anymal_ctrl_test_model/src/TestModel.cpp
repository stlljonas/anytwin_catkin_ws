/*!
 * @author  Dario Bellicoso, Christian Gehring, Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief   Implementation of model test controller
 */

#include "anymal_ctrl_test_model/TestModel.hpp"

#include <Eigen/Core>

namespace anymal_ctrl_test_model {

bool TestModel::create() {
  setAvailableOperationModesForReferenceType(anymal_motion_control::ReferenceType::ACTION, {"show_basic", "show_extended"});
  return true;
}

bool TestModel::reset() {
  return initialize();
}

bool TestModel::advance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) {
  boost::unique_lock<boost::shared_mutex> lock(commandMutex);
  for (auto& actuatorCommand : command.getActuatorCommands()) {
    actuatorCommand.setMode(actuatorCommand.Mode::MODE_FREEZE);
  }
  return true;
}

void TestModel::showBasicModelData() {
  showMassProperties();
  showFootDistances();
}

void TestModel::showExtendedModelData() {
  showState();
  showMassProperties();
  showFootDistances();
  showShankDistances();
  showHipDistances();
}
void TestModel::showState() {
  MELO_INFO("----------------------------------------------------------------------");
  int i = 0;
  for (auto& reading : getState().getActuatorReadings()) {
    MELO_INFO_STREAM("Actuator reading " << i++ << ": \n" << reading.getState());
  }
  MELO_INFO("----------------------------------------------------------------------");
  MELO_INFO_STREAM("Anymal state: \n" << getState().getAnymalModel().getState());
  MELO_INFO("----------------------------------------------------------------------");
}

void TestModel::showMassProperties() {
  MELO_INFO("----------------------------------------------------------------------");
  MELO_INFO_STREAM("Total mass in model: " << getState().getAnymalModel().getTotalMass());
  MELO_INFO("----------------------------------------------------------------------");
  Eigen::Vector3d positionBaseToWbComInBaseFrame =
      getState().getAnymalModel().getPositionWorldToCom(AD::CoordinateFrameEnum::BASE) -
      getState().getAnymalModel().getPositionWorldToBody(AD::BodyEnum::BASE, AD::CoordinateFrameEnum::BASE);
  MELO_INFO_STREAM("Whole-body center of mass in base frame: x: " << positionBaseToWbComInBaseFrame.x()
                                                                  << " m y: " << positionBaseToWbComInBaseFrame.y()
                                                                  << " m z: " << positionBaseToWbComInBaseFrame.z() << " m");
  Eigen::Vector3d positionBaseToTorsoComInBaseFrame =
      getState().getAnymalModel().getBody(AD::BodyEnum::BASE).getPositionWorldToBodyCom(AD::CoordinateFrameEnum::BASE) -
      getState().getAnymalModel().getPositionWorldToBody(AD::BodyEnum::BASE, AD::CoordinateFrameEnum::BASE);
  MELO_INFO_STREAM("Torso center of mass in base frame: x: " << positionBaseToTorsoComInBaseFrame.x()
                                                             << " m y: " << positionBaseToTorsoComInBaseFrame.y()
                                                             << " m z: " << positionBaseToTorsoComInBaseFrame.z() << " m");
  MELO_INFO("----------------------------------------------------------------------");
}
void TestModel::showFootDistances() {
  MELO_INFO("----------------------------------------------------------------------");
  Eigen::Vector3d posBaseToLfFootInBaseFrame =
      getState().getAnymalModel().getPositionBodyToBody(AD::BodyEnum::BASE, AD::BodyEnum::LF_FOOT, AD::CoordinateFrameEnum::BASE);

  Eigen::Vector3d posBaseToRfFootInBaseFrame =
      getState().getAnymalModel().getPositionBodyToBody(AD::BodyEnum::BASE, AD::BodyEnum::RF_FOOT, AD::CoordinateFrameEnum::BASE);

  Eigen::Vector3d posBaseToLhFootInBaseFrame =
      getState().getAnymalModel().getPositionBodyToBody(AD::BodyEnum::BASE, AD::BodyEnum::LH_FOOT, AD::CoordinateFrameEnum::BASE);

  Eigen::Vector3d posBaseToRhFootInBaseFrame =
      getState().getAnymalModel().getPositionBodyToBody(AD::BodyEnum::BASE, AD::BodyEnum::RH_FOOT, AD::CoordinateFrameEnum::BASE);

  MELO_INFO_STREAM("Dist. betw. LF_FOOT and RF_FOOT: x: "
                   << std::abs(posBaseToRfFootInBaseFrame.x() - posBaseToLfFootInBaseFrame.x())
                   << " m y: " << std::abs(posBaseToRfFootInBaseFrame.y() - posBaseToLfFootInBaseFrame.y())
                   << " m z: " << std::abs(posBaseToRfFootInBaseFrame.z() - posBaseToLfFootInBaseFrame.z()) << " m");

  MELO_INFO_STREAM("Dist. betw. LF_FOOT and LH_FOOT: x: "
                   << std::abs(posBaseToLhFootInBaseFrame.x() - posBaseToLfFootInBaseFrame.x())
                   << " m y: " << std::abs(posBaseToLhFootInBaseFrame.y() - posBaseToLfFootInBaseFrame.y())
                   << " m z: " << std::abs(posBaseToLhFootInBaseFrame.z() - posBaseToLfFootInBaseFrame.z()) << " m");

  MELO_INFO_STREAM("Dist. betw. RF_FOOT and RH_FOOT: x: "
                   << std::abs(posBaseToRhFootInBaseFrame.x() - posBaseToRfFootInBaseFrame.x())
                   << " m y: " << std::abs(posBaseToRhFootInBaseFrame.y() - posBaseToRfFootInBaseFrame.y())
                   << " m z: " << std::abs(posBaseToRhFootInBaseFrame.z() - posBaseToRfFootInBaseFrame.z()) << " m");

  MELO_INFO_STREAM("Dist. betw. LH_FOOT and RH_FOOT: x: "
                   << std::abs(posBaseToRhFootInBaseFrame.x() - posBaseToLhFootInBaseFrame.x())
                   << " m y: " << std::abs(posBaseToRhFootInBaseFrame.y() - posBaseToLhFootInBaseFrame.y())
                   << " m z: " << std::abs(posBaseToRhFootInBaseFrame.z() - posBaseToLhFootInBaseFrame.z()) << " m");

  MELO_INFO_STREAM("LF_FOOT x: " << posBaseToLfFootInBaseFrame.x() << " m y: " << posBaseToLfFootInBaseFrame.y()
                                 << " m z: " << posBaseToLfFootInBaseFrame.z() << " m");
  MELO_INFO_STREAM("RF_FOOT x: " << posBaseToRfFootInBaseFrame.x() << " m y: " << posBaseToRfFootInBaseFrame.y()
                                 << " m z: " << posBaseToRfFootInBaseFrame.z() << " m");
  MELO_INFO_STREAM("LH_FOOT x: " << posBaseToLhFootInBaseFrame.x() << " m y: " << posBaseToLhFootInBaseFrame.y()
                                 << " m z: " << posBaseToLhFootInBaseFrame.z() << " m");
  MELO_INFO_STREAM("RH_FOOT x: " << posBaseToRhFootInBaseFrame.x() << " m y: " << posBaseToRhFootInBaseFrame.y()
                                 << " m z: " << posBaseToRhFootInBaseFrame.z() << " m");
  MELO_INFO("----------------------------------------------------------------------");
}

void TestModel::showShankDistances() {
  MELO_INFO("----------------------------------------------------------------------");
  Eigen::Vector3d posBaseToLfShankInBaseFrame =
      getState().getAnymalModel().getPositionBodyToBody(AD::BodyEnum::BASE, AD::BodyEnum::LF_SHANK, AD::CoordinateFrameEnum::BASE);

  Eigen::Vector3d posBaseToRfShankInBaseFrame =
      getState().getAnymalModel().getPositionBodyToBody(AD::BodyEnum::BASE, AD::BodyEnum::RF_SHANK, AD::CoordinateFrameEnum::BASE);

  Eigen::Vector3d posBaseToLhShankInBaseFrame =
      getState().getAnymalModel().getPositionBodyToBody(AD::BodyEnum::BASE, AD::BodyEnum::LH_SHANK, AD::CoordinateFrameEnum::BASE);

  Eigen::Vector3d posBaseToRhShankInBaseFrame =
      getState().getAnymalModel().getPositionBodyToBody(AD::BodyEnum::BASE, AD::BodyEnum::RH_SHANK, AD::CoordinateFrameEnum::BASE);

  MELO_INFO_STREAM("Dist. betw. LF_SHANK and RF_SHANK: x: "
                   << std::abs(posBaseToRfShankInBaseFrame.x() - posBaseToLfShankInBaseFrame.x())
                   << " m y: " << std::abs(posBaseToRfShankInBaseFrame.y() - posBaseToLfShankInBaseFrame.y())
                   << " m z: " << std::abs(posBaseToRfShankInBaseFrame.z() - posBaseToLfShankInBaseFrame.z()) << " m");

  MELO_INFO_STREAM("Dist. betw. LF_SHANK and LH_SHANK: x: "
                   << std::abs(posBaseToLhShankInBaseFrame.x() - posBaseToLfShankInBaseFrame.x())
                   << " m y: " << std::abs(posBaseToLhShankInBaseFrame.y() - posBaseToLfShankInBaseFrame.y())
                   << " m z: " << std::abs(posBaseToLhShankInBaseFrame.z() - posBaseToLfShankInBaseFrame.z()) << " m");

  MELO_INFO_STREAM("Dist. betw. RF_SHANK and RH_SHANK: x: "
                   << std::abs(posBaseToRhShankInBaseFrame.x() - posBaseToRfShankInBaseFrame.x())
                   << " m y: " << std::abs(posBaseToRhShankInBaseFrame.y() - posBaseToRfShankInBaseFrame.y())
                   << " m z: " << std::abs(posBaseToRhShankInBaseFrame.z() - posBaseToRfShankInBaseFrame.z()) << " m");

  MELO_INFO_STREAM("Dist. betw. LH_SHANK and RH_SHANK: x: "
                   << std::abs(posBaseToRhShankInBaseFrame.x() - posBaseToLhShankInBaseFrame.x())
                   << " m y: " << std::abs(posBaseToRhShankInBaseFrame.y() - posBaseToLhShankInBaseFrame.y())
                   << " m z: " << std::abs(posBaseToRhShankInBaseFrame.z() - posBaseToLhShankInBaseFrame.z()) << " m");

  MELO_INFO_STREAM("LF_SHANK x: " << posBaseToLfShankInBaseFrame.x() << " m y: " << posBaseToLfShankInBaseFrame.y()
                                  << " m z: " << posBaseToLfShankInBaseFrame.z() << " m");
  MELO_INFO_STREAM("RF_SHANK x: " << posBaseToRfShankInBaseFrame.x() << " m y: " << posBaseToRfShankInBaseFrame.y()
                                  << " m z: " << posBaseToRfShankInBaseFrame.z() << " m");
  MELO_INFO_STREAM("LH_SHANK x: " << posBaseToLhShankInBaseFrame.x() << " m y: " << posBaseToLhShankInBaseFrame.y()
                                  << " m z: " << posBaseToLhShankInBaseFrame.z() << " m");
  MELO_INFO_STREAM("RH_SHANK x: " << posBaseToRhShankInBaseFrame.x() << " m y: " << posBaseToRhShankInBaseFrame.y()
                                  << " m z: " << posBaseToRhShankInBaseFrame.z() << " m");
  MELO_INFO("----------------------------------------------------------------------");
}

void TestModel::showHipDistances() {
  MELO_INFO("----------------------------------------------------------------------");
  Eigen::Vector3d posBaseToLfHipInBaseFrame =
      getState().getAnymalModel().getPositionBodyToBody(AD::BodyEnum::BASE, AD::BodyEnum::LF_HIP, AD::CoordinateFrameEnum::BASE);

  Eigen::Vector3d posBaseToRfHipInBaseFrame =
      getState().getAnymalModel().getPositionBodyToBody(AD::BodyEnum::BASE, AD::BodyEnum::RF_HIP, AD::CoordinateFrameEnum::BASE);

  Eigen::Vector3d posBaseToLhHipInBaseFrame =
      getState().getAnymalModel().getPositionBodyToBody(AD::BodyEnum::BASE, AD::BodyEnum::LH_HIP, AD::CoordinateFrameEnum::BASE);

  Eigen::Vector3d posBaseToRhHipInBaseFrame =
      getState().getAnymalModel().getPositionBodyToBody(AD::BodyEnum::BASE, AD::BodyEnum::RH_HIP, AD::CoordinateFrameEnum::BASE);

  MELO_INFO_STREAM("Dist. betw. LF_HIP and RF_HIP: x: "
                   << std::abs(posBaseToRfHipInBaseFrame.x() - posBaseToLfHipInBaseFrame.x())
                   << " m y: " << std::abs(posBaseToRfHipInBaseFrame.y() - posBaseToLfHipInBaseFrame.y())
                   << " m z: " << std::abs(posBaseToRfHipInBaseFrame.z() - posBaseToLfHipInBaseFrame.z()) << " m");

  MELO_INFO_STREAM("Dist. betw. LF_HIP and LH_HIP: x: "
                   << std::abs(posBaseToLhHipInBaseFrame.x() - posBaseToLfHipInBaseFrame.x())
                   << " m y: " << std::abs(posBaseToLhHipInBaseFrame.y() - posBaseToLfHipInBaseFrame.y())
                   << " m z: " << std::abs(posBaseToLhHipInBaseFrame.z() - posBaseToLfHipInBaseFrame.z()) << " m");

  MELO_INFO_STREAM("Dist. betw. RF_HIP and RH_HIP: x: "
                   << std::abs(posBaseToRhHipInBaseFrame.x() - posBaseToRfHipInBaseFrame.x())
                   << " m y: " << std::abs(posBaseToRhHipInBaseFrame.y() - posBaseToRfHipInBaseFrame.y())
                   << " m z: " << std::abs(posBaseToRhHipInBaseFrame.z() - posBaseToRfHipInBaseFrame.z()) << " m");

  MELO_INFO_STREAM("Dist. betw. LH_HIP and RH_HIP: x: "
                   << std::abs(posBaseToRhHipInBaseFrame.x() - posBaseToLhHipInBaseFrame.x())
                   << " m y: " << std::abs(posBaseToRhHipInBaseFrame.y() - posBaseToLhHipInBaseFrame.y())
                   << " m z: " << std::abs(posBaseToRhHipInBaseFrame.z() - posBaseToLhHipInBaseFrame.z()) << " m");

  MELO_INFO_STREAM("LF_HIP x: " << posBaseToLfHipInBaseFrame.x() << " m y: " << posBaseToLfHipInBaseFrame.y()
                                << " m z: " << posBaseToLfHipInBaseFrame.z() << " m");
  MELO_INFO_STREAM("RF_HIP x: " << posBaseToRfHipInBaseFrame.x() << " m y: " << posBaseToRfHipInBaseFrame.y()
                                << " m z: " << posBaseToRfHipInBaseFrame.z() << " m");
  MELO_INFO_STREAM("LH_HIP x: " << posBaseToLhHipInBaseFrame.x() << " m y: " << posBaseToLhHipInBaseFrame.y()
                                << " m z: " << posBaseToLhHipInBaseFrame.z() << " m");
  MELO_INFO_STREAM("RH_HIP x: " << posBaseToRhHipInBaseFrame.x() << " m y: " << posBaseToRhHipInBaseFrame.y()
                                << " m z: " << posBaseToRhHipInBaseFrame.z() << " m");
  MELO_INFO("----------------------------------------------------------------------");
}

anymal_motion_control::SwitchResult TestModel::goToReferenceType(anymal_motion_control::ReferenceType referenceType) {
  if (referenceType != anymal_motion_control::ReferenceType::ACTION) {
    return anymal_motion_control::SwitchResult::NOT_FOUND;
  }
  return anymal_motion_control::SwitchResult::SWITCHED;
}

void TestModel::goToOperationMode(const std::string& operationMode, anymal_motion_control::OperationModeAction* action) {
  if (operationMode == "show_basic") {
    showBasicModelData();
  } else if (operationMode == "show_extended") {
    showExtendedModelData();
  } else {
    action->setAborted(anymal_motion_control::SwitchResult::NOT_FOUND, "Could not find operation mode " + operationMode);
    return;
  }
  action->setSucceeded(anymal_motion_control::SwitchResult::SWITCHED);
}

}  // namespace anymal_ctrl_test_model
