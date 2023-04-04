/*!
 * @author  Dario Bellicoso, Christian Gehring, Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief   Declaration of model test controller
 */

#pragma once

#include <anymal_description/AnymalDescription.hpp>
#include <anymal_motion_control/AnymalController.hpp>

namespace anymal_ctrl_test_model {

class TestModel : public anymal_motion_control::AnymalController {
 public:
  using AD = anymal_description::AnymalDescription;
  using Base = anymal_motion_control::AnymalController;

  TestModel() = default;
  ~TestModel() override = default;

  bool create() override;
  bool initialize() override { return true; };
  bool reset() override;
  bool advance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) override;
  bool stop() override { return true; }
  bool preStop() override { return true; }

 private:
  anymal_motion_control::SwitchResult goToReferenceType(anymal_motion_control::ReferenceType referenceType) override;
  void goToOperationMode(const std::string& operationMode, anymal_motion_control::OperationModeAction* action) override;

  void showBasicModelData();
  void showExtendedModelData();

  void showState();
  void showMassProperties();
  void showFootDistances();
  void showShankDistances();
  void showHipDistances();
};

}  // namespace anymal_ctrl_test_model
