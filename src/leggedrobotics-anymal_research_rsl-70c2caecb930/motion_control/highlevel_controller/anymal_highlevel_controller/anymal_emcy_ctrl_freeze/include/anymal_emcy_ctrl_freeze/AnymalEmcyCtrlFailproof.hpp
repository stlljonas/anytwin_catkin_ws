/*!
 * @file 	    RocoFreeze.hpp
 * @author    Christian Gehring
 * @date		  Oct 14, 2011
 * @version 	1.0
 * @ingroup 	robotTask
 * @brief
 */

#pragma once

#include <roco/controllers/FailproofController.hpp>
#include <anymal_roco/anymal_roco.hpp>

namespace robot_controller {

//! This fail-proof emergency controller freezes the configuration of the robot.
/*!
 */
class AnymalEmcyCtrlFailproof: virtual public roco::FailproofController<anymal_roco::RocoState, anymal_roco::RocoCommand> {
 public:
  using Base = roco::FailproofController<anymal_roco::RocoState, anymal_roco::RocoCommand>;
 public:

  //! Contstructor
  AnymalEmcyCtrlFailproof();

  //! Destructor
  virtual ~AnymalEmcyCtrlFailproof();

  virtual bool create(double dt);

  virtual void advance(double dt);

  virtual bool cleanup();

};

} /* namespace robotTask */
