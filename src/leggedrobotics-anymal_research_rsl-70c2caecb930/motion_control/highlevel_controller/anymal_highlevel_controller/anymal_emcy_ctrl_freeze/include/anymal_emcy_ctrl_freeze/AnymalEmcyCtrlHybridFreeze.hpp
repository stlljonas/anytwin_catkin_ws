/*!
 * @file        AnymalEmcyCtrlHybridFreeze.hpp
 * @author      Valentin Yuryev
 * @date        Dec 06, 2019
 * @brief       Implementation of controller that disables if moving and freezes if in place
 */

#pragma once

#include <anymal_roco/anymal_roco.hpp>

#include <loco/common/ParameterSet.hpp>

#include <roco/controllers/Controller.hpp>
#include <roco/controllers/adaptees/EmergencyControllerAdapteeInterface.hpp>

namespace robot_controller {

//! This emergency controller disables the drives if moving and freezes the configuration of the robot if velocity is low.
/*!
 */
class AnymalEmcyCtrlHybridFreeze: virtual public roco::Controller<anymal_roco::RocoState, anymal_roco::RocoCommand>, public roco::EmergencyControllerAdapteeInterface {

  enum struct EmergencyState {
    MOVING = 0,
    STATIONARY
  };

 public:
  using Base = roco::Controller<anymal_roco::RocoState, anymal_roco::RocoCommand>;

  //! Contstructor
  AnymalEmcyCtrlHybridFreeze();

  //! Destructor
  virtual ~AnymalEmcyCtrlHybridFreeze() = default;

  virtual bool create(double dt) override;

  bool initialize(double dt) override {
    return true;
  }

  bool reset(double dt) override { return true; }

  bool initializeFast(double dt) override;
  bool advance(double dt) override;

  bool cleanup() override { return true; }
  bool preStop() override { return true; }
  bool stop() override { return true; }

private:
  Eigen::Matrix<double, 12, 1> previousCommand_;
  std::unique_ptr<loco::ParameterSet> parameterSet_;
  series_elastic_actuator::SeActuatorCommand::PidGains pidGains_;
  EmergencyState emergencyState_;
  double velocityThreshold_; // Empirically set. Needs to be higher than velocity noise but low enough to trigger during movement.
  double alpha_;
  double timeout_;
  double dampingDuration_;

  bool loadParameters(double dt);
  bool advanceState();
  bool isMoving();

  bool dampDrives();
  bool freezeDrives();
};

} /* namespace robotTask */
