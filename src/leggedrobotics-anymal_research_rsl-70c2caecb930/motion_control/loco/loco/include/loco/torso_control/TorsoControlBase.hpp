/*!
 * @file     TorsoControlBase.hpp
 * @author   Christian Gehring, Dario Bellicoso
 * @date     Feb, 2014
 * @version  1.0
 * @ingroup
 * @brief
 */

#pragma once

// tinyxml
#include <tinyxml_tools/tinyxml_tools.hpp>

// kindr
#include "kindr/Core"

// signal logger
#include <signal_logger/signal_logger.hpp>

// loco
#include "loco/common/ModuleBase.hpp"
#include "loco/common/typedefs.hpp"

namespace loco {

class ComSupportControlBase;

class TorsoControlBase : public ModuleBase {
 public:
  TorsoControlBase() : ModuleBase("torso_control"){};
  ~TorsoControlBase() override = default;

  virtual const ComSupportControlBase& getComSupportControl() const = 0;
  virtual ComSupportControlBase* getComSupportControlPtr() = 0;

  /*! Computes an interpolated version of the two controllers passed in as parameters.
   *  if t is 0, the current setting is set to controller1, 1 -> controller2, and values in between
   *  correspond to interpolated parameter set.
   * @param torsoController1
   * @param torsoController2
   * @param t interpolation parameter
   * @returns true if successful
   */
  virtual bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t) = 0;

  /*! Log signals.
   *
   * @param ns Namespace prefix of all child signals.
   */
  virtual void addVariablesToLog(const std::string& /* ns */) {}
};

} /* namespace loco */
