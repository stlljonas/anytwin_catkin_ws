/*!
 * @file 	    AnymalEmcyCtrlFreeze.hpp
 * @author    Christian Gehring
 * @date		  Oct 14, 2011
 * @version 	1.0
 * @ingroup 	robotTask
 * @brief
 */

#pragma once

#include <roco/controllers/Controller.hpp>
#include <roco/controllers/adaptees/EmergencyControllerAdapteeInterface.hpp>
#include <anymal_roco/anymal_roco.hpp>

namespace robot_controller {

//! This emergency controller freezes the configuration of the robot.
/*!
 */
class AnymalEmcyCtrlFreeze: virtual public roco::Controller<anymal_roco::RocoState, anymal_roco::RocoCommand>, public roco::EmergencyControllerAdapteeInterface {
 public:
  using Base = roco::Controller<anymal_roco::RocoState, anymal_roco::RocoCommand>;
 public:

  //! Contstructor
  AnymalEmcyCtrlFreeze();

  //! Destructor
  virtual ~AnymalEmcyCtrlFreeze();

  virtual bool create(double dt) {
    return true;
  }
  virtual bool initialize(double dt) {
    this->setIsCheckingState(false);
    return true;
  }

  virtual bool reset(double dt) { return true; }

  virtual bool initializeFast(double dt);
  virtual bool advance(double dt);

  virtual bool cleanup() { return true; }
  virtual bool preStop() { return true; }
  virtual bool stop() { return true; }

};

} /* namespace robotTask */
