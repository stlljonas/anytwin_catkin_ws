/*
 * ContactInvariantDamper.hpp
 *
 *  Created on: Aug 15, 2016
 *      Author: Christian Gehring
 */

#pragma once

#include <loco/motion_control/MotionControllerBase.hpp>
#include <map>
#include <parameter_handler/parameter_handler.hpp>
#include "loco/common/WholeBody.hpp"

namespace loco {

/*!
 *
 */
class ContactInvariantDamper : public MotionControllerBase {
 public:
  struct LogData {
    JointTorques dampingJointTorques_;
    Force dampingContactForceInControlFrame_;
  };
  explicit ContactInvariantDamper(WholeBody& wholeBody);
  ~ContactInvariantDamper() override = default;

  /*!
   * Load parameters.
   * @return true if successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /*!
   * Add data to logger (optional).
   * @return true if successful
   */
  bool addVariablesToLog(const std::string& ns) const override;

  bool addParametersToHandler(const std::string& ns) override;

  bool initialize(double dt) override;

  /*!
   * Computes the joint torques from the desired base pose.
   * @return true if successful
   */
  bool advance(double dt) override;

 protected:
  void computeJointTorquesFromForceAtFootInWorldFrame(JointTorques& jointTorques, LegBase* leg,
                                                      const Force& desiredContactForceAtFootInWorldFrame) const;

 protected:
  parameter_handler::Parameter<double> headingDampingGain_;
  parameter_handler::Parameter<double> lateralDampingGain_;
  std::map<const LegBase* const, LogData> logData_;
};

} /* namespace loco */
