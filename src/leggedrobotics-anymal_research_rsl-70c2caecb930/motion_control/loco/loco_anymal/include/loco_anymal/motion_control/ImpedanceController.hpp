/*!
* @file     ImpedanceController.hpp
* @author   Péter Fankhauser, Georg Wiedebach
* @date     Mar 8, 2016
*/

#pragma once

// Loco
#include "loco/motion_control/MotionControllerBase.hpp"
#include "loco/common/typedefs.hpp"

// loco anymal
#include <loco_anymal/typedefs.hpp>
#include <loco_anymal/common/WholeBodyAnymal.hpp>

// Parameter handler
#include <parameter_handler/parameter_handler.hpp>

// Eigen
#include <Eigen/Core>

// Std
#include <iostream>

// TinyXML
#include <tinyxml_tools/tinyxml_tools.hpp>

// anymal model
#include <anymal_model/AnymalModel.hpp>

namespace loco {

class ImpedanceController : public MotionControllerBase
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using AD = anymal_description::AnymalDescription;

  /*!
   * Constructor.
   */
  ImpedanceController(loco_anymal::WholeBodyAnymal& wholebody,
                      anymal_model::AnymalModel& anymalModel,
                      anymal_model::AnymalModel& anymalModelDesired);

  /*!
   * Destructor.
   */
  ~ImpedanceController() override = default;

  /*!
   * Load parameters.
   * @return true if successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /*!
   * Add variables to log (optional).
   * @return true if successful.
   */
  bool addVariablesToLog(const std::string& ns) const override;

  bool initialize(double dt) override;

  /*!
   * Computes the joint torques from the desired base pose.
   * @return true if successful.
   */
  bool advance(double dt) override;


 private:
  void setJointPositionsFromDesiredBase(LegBase* leg);
  bool computeGravityCompensation(const std::map<LegBase*, bool> supportLegs, Eigen::VectorXd& jointTorques) const;
  bool setControlModeForLegs();

  anymal_model::AnymalModel& anymalModel_;
  anymal_model::AnymalModel& anymalModelDesired_;
  Eigen::MatrixXd selectionMatrix_;
  double timeSinceInitialization_;
  double interpolationDuration_;

  ControlMode supportLegControlMode_;
  ControlMode nonSupportLegControlMode_;
};

} /* namespace loco */
