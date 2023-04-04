/** \file DynamixelNode.h
    \brief This file defines the DynamixelNode class which implements the
           Dynamixel node.
  */

#ifndef DYNAMIXEL_NODE_H
#define DYNAMIXEL_NODE_H

#include <string>
#include <memory>
#include <cstdint>

#include <ros/ros.h>

#include <diagnostic_updater/diagnostic_updater.h>

#include "dynamixel_ros_msgs/SetPidGains.h"
#include "dynamixel_ros_msgs/GetPidGains.h"
#include "dynamixel_ros_msgs/SetCompliance.h"
#include "dynamixel_ros_msgs/GetCompliance.h"
#include "dynamixel_ros_msgs/SetAngleLimits.h"
#include "dynamixel_ros_msgs/GetAngleLimits.h"
#include "dynamixel_ros_msgs/SetMaxTorque.h"
#include "dynamixel_ros_msgs/GetMaxTorque.h"
#include "dynamixel_ros_msgs/SetTorqueEnable.h"
#include "dynamixel_ros_msgs/GetTorqueEnable.h"
#include "dynamixel_ros_msgs/SetTorqueControlModeEnable.h"
#include "dynamixel_ros_msgs/GetTorqueControlModeEnable.h"
#include "dynamixel_ros_msgs/SetGoalTorque.h"
#include "dynamixel_ros_msgs/GetGoalTorque.h"
#include "dynamixel_ros_msgs/SetGoalAcceleration.h"
#include "dynamixel_ros_msgs/GetGoalAcceleration.h"
#include "dynamixel_ros_msgs/SetGoalPosition.h"
#include "dynamixel_ros_msgs/GetGoalPosition.h"
#include "dynamixel_ros_msgs/SetMovingSpeed.h"
#include "dynamixel_ros_msgs/GetMovingSpeed.h"

#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

namespace diagnostic_updater {
  class HeaderlessTopicDiagnostic;
}

namespace dynamixel {
class Controller;
}

namespace dynamixel_ros {

  /** The class DynamixelNode implements the Dynamixel node.
      \brief Dynamixel node
    */
  class DynamixelNode {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Constructor
    DynamixelNode(const ros::NodeHandle& nh);
    /// Copy constructor
    DynamixelNode(const DynamixelNode& other) = delete;
    /// Copy assignment operator
    DynamixelNode& operator = (const DynamixelNode& other) = delete;
    /// Move constructor
    DynamixelNode(DynamixelNode&& other) = delete;
    /// Move assignment operator
    DynamixelNode& operator = (DynamixelNode&& other) = delete;
    /// Destructor
    ~DynamixelNode();
    /** @}
      */

    /** \name Public Methods
      @{
      */
    /// Spin
    void spin();
    /// Shutdown
    void goToDefaultPositionAndShutdown();
    /** @}
      */

  private:
    /** \name Private methods
      @{
      */
    /// Retrieves parameters from the parameter server
    void getParameters();
    /// Diagnose the serial connection
    void diagnoseSerialConnection(diagnostic_updater::DiagnosticStatusWrapper& status);
    /// Diagnose the servo motor
    void diagnoseMotor(diagnostic_updater::DiagnosticStatusWrapper& status);
    /// Set PID gains service
    bool setPidGains(dynamixel_ros_msgs::SetPidGains::Request& request,
      dynamixel_ros_msgs::SetPidGains::Response& response);
    /// Get PID gains service
    bool getPidGains(dynamixel_ros_msgs::GetPidGains::Request& request,
      dynamixel_ros_msgs::GetPidGains::Response& response);
    /// Set compliance service
    bool setCompliance(dynamixel_ros_msgs::SetCompliance::Request& request,
      dynamixel_ros_msgs::SetCompliance::Response& response);
    /// Get compliance service
    bool getCompliance(dynamixel_ros_msgs::GetCompliance::Request& request,
      dynamixel_ros_msgs::GetCompliance::Response& response);
    /// Set angle limits service
    bool setAngleLimits(dynamixel_ros_msgs::SetAngleLimits::Request& request,
      dynamixel_ros_msgs::SetAngleLimits::Response& response);
    /// Get angle limits service
    bool getAngleLimits(dynamixel_ros_msgs::GetAngleLimits::Request& request,
      dynamixel_ros_msgs::GetAngleLimits::Response& response);
    /// Set max torque service
    bool setMaxTorque(dynamixel_ros_msgs::SetMaxTorque::Request& request,
      dynamixel_ros_msgs::SetMaxTorque::Response& response);
    /// Get max torque service
    bool getMaxTorque(dynamixel_ros_msgs::GetMaxTorque::Request& request,
      dynamixel_ros_msgs::GetMaxTorque::Response& response);
    /// Set torque enable service
    bool setTorqueEnable(dynamixel_ros_msgs::SetTorqueEnable::Request& request,
      dynamixel_ros_msgs::SetTorqueEnable::Response& response);
    /// Get torque enable service
    bool getTorqueEnable(dynamixel_ros_msgs::GetTorqueEnable::Request& request,
      dynamixel_ros_msgs::GetTorqueEnable::Response& response);
    /// Set torque control mode enable service
    bool setTorqueControlModeEnable(
      dynamixel_ros_msgs::SetTorqueControlModeEnable::Request& request,
      dynamixel_ros_msgs::SetTorqueControlModeEnable::Response& response);
    /// Get torque control mode enable service
    bool getTorqueControlModeEnable(
      dynamixel_ros_msgs::GetTorqueControlModeEnable::Request& request,
      dynamixel_ros_msgs::GetTorqueControlModeEnable::Response& response);
    /// Set goal torque service
    bool setGoalTorque(dynamixel_ros_msgs::SetGoalTorque::Request& request,
      dynamixel_ros_msgs::SetGoalTorque::Response& response);
    /// Get goal torque service
    bool getGoalTorque(dynamixel_ros_msgs::GetGoalTorque::Request& request,
      dynamixel_ros_msgs::GetGoalTorque::Response& response);
    /// Set goal position service
    bool setGoalPosition(dynamixel_ros_msgs::SetGoalPosition::Request& request,
      dynamixel_ros_msgs::SetGoalPosition::Response& response);
    /// Get goal position service
    bool getGoalPosition(dynamixel_ros_msgs::GetGoalPosition::Request& request,
      dynamixel_ros_msgs::GetGoalPosition::Response& response);
    /// Set moving speed service
    bool setMovingSpeed(dynamixel_ros_msgs::SetMovingSpeed::Request& request,
      dynamixel_ros_msgs::SetMovingSpeed::Response& response);
    /// Get moving speed service
    bool getMovingSpeed(dynamixel_ros_msgs::GetMovingSpeed::Request& request,
      dynamixel_ros_msgs::GetMovingSpeed::Response& response);
    /// Set goal acceleration service
    bool setGoalAcceleration(dynamixel_ros_msgs::SetGoalAcceleration::Request& request,
      dynamixel_ros_msgs::SetGoalAcceleration::Response& response);
    /// Get goal acceleration service
    bool getGoalAcceleration(dynamixel_ros_msgs::GetGoalAcceleration::Request& request,
      dynamixel_ros_msgs::GetGoalAcceleration::Response& response);
    /// Set goal position default service
    bool setGoalPositionDefault(std_srvs::Empty::Request& request,
        std_srvs::Empty::Response& response);
    /// Shutdown Dynamixel
    void shutdownCallback(const std_msgs::BoolConstPtr& shutdown);
    /// Init services
    void initServices();
    /// Shutdown services
    void shutdownServices();
    /// Handle signals
    void handleSignal(const int signum);


    uint16_t angleToPosition(double angle);
    double positionToAngle(uint16_t position);
    /** @}
      */

    /** \name Private members
      @{
      */
    /// ROS node handle
    ros::NodeHandle nodeHandle_;
    /// Diagnostic updater
    diagnostic_updater::Updater diagnosticUpdater_;
    /// Device name reported by diagnostic engine
    std::string deviceName_;
    /// Acquisition loop rate
    double acquisitionLoopRate_;
    /// Serial port device name
    std::string serialPortDeviceName_;
    /// Serial port baudrate
    int serialPortBaudRate_;
    /// Dynamixel controller
    std::shared_ptr<dynamixel::Controller> controller_;
    /// Retry timeout in case of failure [s]
    double retryTimeout_;
    /// Servo motor ID
    int motorId_;
    /// Time offset
    int64_t timeOffset_;
    /// ROS joint state publisher
    ros::Publisher jointStatePublisher_;
    /// ROS joint state publisher topic name
    std::string jointStatePublisherTopic_;
    /// ROS joint state publisher frame id
    std::string jointStatePublisherFrameId_;
    /// ROS joint state publisher queue size
    int jointStatePublisherQueueSize_;
    /// ROS joint state publisher frequency diagnostic
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic>
      jointStatePublisherFrequencyDiagnostic_;
    /// ROS joint state publisher frequency tolerance percentage
    double jointStatePublisherFreqTolPercentage_;
    /// Offset angle
    double offsetAngle_;
    /// Default angle
    double defaultAngle_;
    /// Current angle
    double currentAngle_;
    /// ROS joint state publisher minimum frequency
    double jointStatePublisherMinFrequency_;
    /// ROS joint state publisher maximum frequency
    double jointStatePublisherMaxFrequency_;
    /// Servo model number
    uint16_t modelNumber_;
    /// Servo model name
    std::string modelName_;
    /// Servo firmware version
    uint8_t firmwareVersion_;
    /// Return delay time
    uint8_t returnDelayTime_;
    /// Motor connected
    bool motorConnected_;
    /// Maximum number of ticks
    uint16_t maxTicks_;
    /// Range in degrees
    double rangeInDegrees_;
    /// Rpm per tick
    double rpmPerTick_;
    /// Motor baud rate
    uint8_t servoBaudRate_;
    /// Clockwise angle limit
    uint16_t cwAngleLimit_;
    /// Counterclockwise angle limit
    uint16_t ccwAngleLimit_;
    /// Highest limit temperature
    uint8_t highestLimitTemperature_;
    /// Highest limit voltage
    uint8_t highestLimitVoltage_;
    /// Lowest limit voltage
    uint8_t lowestLimitVoltage_;
    /// Maximum torque ratio
    uint16_t maxTorque_;
    /// Torque enabled
    bool torqueEnabled_;
    /// P gain
    uint8_t pGain_;
    /// I gain
    uint8_t iGain_;
    /// D gain
    uint8_t dGain_;
    /// Clockwise compliance margin
    uint8_t cwComplianceMargin_;
    /// Counterclockwise compliance margin
    uint8_t ccwComplianceMargin_;
    /// Clockwise compliance slope
    uint8_t cwComplianceSlope_;
    /// Counterclockwise compliance slope
    uint8_t ccwComplianceSlope_;
    /// Goal position
    uint16_t goalPosition_;
    /// Moving speed
    uint16_t movingSpeed_;
    /// Torque limit ratio
    uint16_t torqueLimit_;
    /// Current position
    uint16_t presentPosition_;
    /// Current speed
    uint16_t presentSpeed_;
    /// Current load
    uint16_t presentLoad_;
    /// Current voltage
    uint8_t presentVoltage_;
    /// Current temperature
    uint8_t presentTemperature_;
    /// Is an instruction registered
    bool registered_;
    /// Is the servo moving
    bool moving_;
    /// Punch
    uint16_t punch_;
    /// Consuming current
    uint16_t current_;
    /// Torque control mode enabled
    bool torqueControlModeEnabled_;
    /// Goal torque
    uint16_t goalTorque_;
    /// Goal acceleration
    uint8_t goalAcceleration_;
    /// Services are running
    bool servicesAreRunning_ = false;
    /// Set PID gains service
    ros::ServiceServer setPidGainsService_;
    /// Get PID gains service
    ros::ServiceServer getPidGainsService_;
    /// Set compliance service
    ros::ServiceServer setComplianceService_;
    /// Get compliance service
    ros::ServiceServer getComplianceService_;
    /// Set angle limits service
    ros::ServiceServer setAngleLimitsService_;
    /// Get angle limits service
    ros::ServiceServer getAngleLimitsService_;
    /// Set max torque service
    ros::ServiceServer setMaxTorqueService_;
    /// Get max torque service
    ros::ServiceServer getMaxTorqueService_;
    /// Set torque enable service
    ros::ServiceServer setTorqueEnableService_;
    /// Get torque enable service
    ros::ServiceServer getTorqueEnableService_;
    /// Set torque control mode enable service
    ros::ServiceServer setTorqueControlModeEnableService_;
    /// Get torque control mode enable service
    ros::ServiceServer getTorqueControlModeEnableService_;
    /// Set goal torque service
    ros::ServiceServer setGoalTorqueService_;
    /// Get goal torque service
    ros::ServiceServer getGoalTorqueService_;
    /// Set goal position service
    ros::ServiceServer setGoalPositionService_;
    /// Get goal position service
    ros::ServiceServer getGoalPositionService_;
    /// Set moving speed service
    ros::ServiceServer setMovingSpeedService_;
    /// Get moving speed service
    ros::ServiceServer getMovingSpeedService_;
    /// Set goal acceleration service
    ros::ServiceServer setGoalAccelerationService_;
    /// Get goal acceleration service
    ros::ServiceServer getGoalAccelerationService_;
    /// Set goal position default service
    ros::ServiceServer setGoalPositionDefaultService_;
    /// Shutdown Dynamixel
    ros::Subscriber shutdownSubscriber_;
    /** @}
      */

  };

}

#endif // DYNAMIXEL_NODE_H
