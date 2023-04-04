/*!
 * @file    JointConfigurationsController.hpp
 * @author  Alexander Reske
 * @brief   A controller that sends the robot into different joint configurations. E.g., the drives can be sent into default positions etc.
 */

#pragma once

// stl
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

// eigen
#include <Eigen/Core>

// anymal_description
#include <anymal_description/AnymalDescription.hpp>

// anymal_motion_control
#include <anymal_motion_control/AnymalController.hpp>

// tinyxml_tools
#include <tinyxml_tools/tinyxml_tools.hpp>

// anymal_ctrl_joint_configurations
#include "anymal_ctrl_joint_configurations/MotionGeneration.hpp"

namespace anymal_ctrl_joint_configurations {

class JointConfigurationsController : public anymal_motion_control::AnymalController {
 public:
  using Base = anymal_motion_control::AnymalController;
  using AD = anymal_description::AnymalDescription;
  using JointVector = Eigen::Matrix<double, AD::getJointsDimension(), 1>;

  //! Construct JointConfigurationsController.
  JointConfigurationsController();

  //! Destruct JointConfigurationsController.
  ~JointConfigurationsController() override = default;

 protected:
  /*!
   * Create controller JointConfigurationsController.
   * @returns true if successful.
   */
  bool create() override;

  /*!
   * Initialize controller JointConfigurationsController.
   * @returns true if successful.
   */
  bool initialize() override;

  /*!
   * Advance controller JointConfigurationsController.
   * @param command Command to be filled.
   * @param commandMutex Mutex associated with the command.
   * @returns true if successful.
   */
  bool advance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) override;

  /*!
   * Reset controller JointConfigurationsController.
   * @returns true if successful.
   */
  bool reset() override;

  /*!
   * Pre-stop controller JointConfigurationsController.
   * @returns true if successful.
   */
  bool preStop() override;

  /*!
   * Stop controller JointConfigurationsController.
   * @returns true if successful.
   */
  bool stop() override;

  /*!
   * Switch to a new mode of the controller.
   * @param[in] mode controller mode.
   * @returns true if successful.
   */
  bool switchControllerMode(const std::string& mode);

  /*!
   * Get the current mode of the controller.
   * @returns mode_.
   */
  const std::string getMode() const;

  /*!
   * Get all modes of the controller.
   * @returns modes_.
   */
  const std::vector<std::string> getModes() const;

  // Data types
  enum class States { Standby, Planning, Active, Success, Preempted, Error };
  struct JointConfiguration {
    bool contact_ = true;
    JointVector jointPositions_ = JointVector::Zero();
  };

  /*!
   * Get the state of the controller.
   * @returns State of the controller.
   */
  States& getControllerState();

  /*!
   * Get the custom joint configuration.
   * @returns Struct of the custom joint configuration.
   */
  JointConfiguration& getCustomJointConfiguration();

  /*!
   * Get the controller mutex.
   * @returns Mutex for the controller.
   */
  std::mutex& getControllerMutex();

 private:
  //! @copydoc anymal_motion_control::AnymalController::goToReferenceType(ReferenceType referenceType)
  anymal_motion_control::SwitchResult goToReferenceType(anymal_motion_control::ReferenceType referenceType) override;

  //! @copydoc anymal_motion_control::AnymalController::goToOperationMode(const std::string& operationMode, OperationModeAction* action)
  void goToOperationMode(const std::string& operationMode, anymal_motion_control::OperationModeAction* action) override;

  /*!
   * Advance in state Standby, Success, or Error.
   * @param command Command to be filled.
   * @param commandMutex Mutex associated with the command.
   * @returns true if successful.
   */
  bool defaultAdvance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex);

  /*!
   * Advance in state Planning.
   * @param command Command to be filled.
   * @param commandMutex Mutex associated with the command.
   * @returns true if successful.
   */
  bool planningAdvance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex);

  /*!
   * Advance in state Active.
   * @param command Command to be filled.
   * @param commandMutex Mutex associated with the command.
   * @returns true if successful.
   */
  bool activeAdvance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex);

  /*!
   * Check if a safety check, e.g. contact or tracking, has been violated.
   * @returns true if safety violation.
   */
  bool safetyCheckViolation();

  /*!
   * Check if the robot detects a contact.
   * @returns true if in contact.
   */
  bool detectedContact();

  /*!
   * Check if tracking is bad, i.e., if tracking errors are too large.
   * @returns true if bad tracking.
   */
  bool badTracking();

  /*!
   * Check if the goal joint positions have been reached.
   * @returns true if goal reached.
   */
  bool reachedGoal();

  /*!
   * Load the controller parameters from an XML file.
   * @param[in] filename path and name of the file.
   * @returns true if successful.
   */
  bool loadParams(const std::string& filename);

  /*!
   * Load gain parameters (PID gains) from drive element.
   * @param[in] driveHandle TiXmlHandle for the element that contains the gains for HAA, HFE or KFE.
   * @param[out] pidGains Struct with the PID gains.
   * @returns true if successful.
   */
  bool loadGainParam(const TiXmlHandle& driveHandle, series_elastic_actuator::SeActuatorCommand::PidGains& pidGains);

  /*!
   * Load configuration parameters from configuration element.
   * @param[in] configurationHandle TiXmlHandle for the element that contains a joint configuration.
   * @param[out] jointConfiguration Struct with of all types of safety checks that should be performed and with a vector containing all
   * joint positions for the configuration.
   * @returns true if successful.
   */
  bool loadConfigurationParam(const TiXmlHandle& configurationHandle, JointConfiguration& jointConfiguration);

  /*!
   * Load limit parameter (e.g. maximum joint velocity) from the limit element.
   * @param[in] limitHandle TiXmlHandle for the element that contains the limit.
   * @param[out] limit The queried limit.
   * @returns true if successful.
   */
  bool loadLimitParam(const TiXmlHandle& limitHandle, double& limit);

  /*
   * Member variables
   */

  //! Internal controller state
  States state_;

  //! Current operation mode
  std::string mode_;

  //~ Flag for bad parameters that physically make no sense, e.g. negative max joint velocity
  bool badParameters_;

  //! Maximum joint velocity forwarded to the motion generator
  double maxJointVelocity_;

  //! Bad tracking is detected if joint angle errors exceed this value at any time
  double maxAbsJointPositionError_;

  //! Joint angle tolerance used to check if a goal configuration is reached
  double absJointPositionErrorTolerance_;

  //! Timeout in [s] after which an operation mode is stopped and state transitions to States::Error
  double timeout_;

  //! Motion generation module to interpolate joint trajectories
  std::unique_ptr<MotionGeneration> motionGeneration_;

  //! Vector of joint angles
  JointVector currentJointPositions_;

  //! Vector holding target joint configuration
  JointVector goalJointPositions_;

  //! Configuration parameters for each joint configuration
  std::unordered_map<std::string, JointConfiguration> jointConfigurations_;

  //! Tolerance values used to decide if the current joint configuration is close to the "rest" one
  std::unordered_map<std::string, double> maxAbsJointPositionDeviationFromRest_;

  //! PID gains for HAA joint
  series_elastic_actuator::SeActuatorCommand::PidGains gainsHaa_;

  //! PID gains for HFE joint
  series_elastic_actuator::SeActuatorCommand::PidGains gainsHfe_;

  //! PID gains for KFE joint
  series_elastic_actuator::SeActuatorCommand::PidGains gainsKfe_;

  //! Mutex to ensure state consistency
  std::mutex controllerMutex_;
};

} /* namespace anymal_ctrl_joint_configurations */
