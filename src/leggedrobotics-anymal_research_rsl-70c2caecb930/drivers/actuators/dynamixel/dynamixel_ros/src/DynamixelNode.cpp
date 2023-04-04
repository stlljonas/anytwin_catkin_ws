#include "dynamixel_ros/DynamixelNode.h"

#include <chrono>
#include <thread>
#include <ratio>
#include <cstdint>
#include <cmath>

#include <boost/make_shared.hpp>

#include <ros/rate.h>

#include <diagnostic_updater/publisher.h>

#include <sensor_msgs/JointState.h>

#include <libdynamixel/sensor/Controller.h>
#include <libdynamixel/com/SerialPort.h>
#include <libdynamixel/exceptions/IOException.h>
#include <libdynamixel/exceptions/SystemException.h>
#include <libdynamixel/exceptions/BadArgumentException.h>

#include <param_io/get_param.hpp>

#include <signal_handler/SignalHandler.hpp>

namespace dynamixel_ros {

using namespace dynamixel;
using namespace dynamixel_ros_msgs;

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  DynamixelNode::DynamixelNode(const ros::NodeHandle& nh) :
      nodeHandle_(nh),
      motorConnected_(false) {
    // bind signal handler
    signal_handler::SignalHandler::bindAll(&DynamixelNode::handleSignal, this);

    // retrieve configurable parameters
    getParameters();
    
    // set current angle to default
    currentAngle_ = defaultAngle_;

    // init diagnostic engine
    diagnosticUpdater_.setHardwareID(deviceName_);
    diagnosticUpdater_.add("Serial connection", this,
      &DynamixelNode::diagnoseSerialConnection);
    diagnosticUpdater_.add("Motor", this,
      &DynamixelNode::diagnoseMotor);
    jointStatePublisherMinFrequency_ = acquisitionLoopRate_ -
      jointStatePublisherFreqTolPercentage_ * acquisitionLoopRate_;
    jointStatePublisherMaxFrequency_ = acquisitionLoopRate_ +
      jointStatePublisherFreqTolPercentage_ * acquisitionLoopRate_;
    jointStatePublisherFrequencyDiagnostic_ =
      std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
      jointStatePublisherTopic_, diagnosticUpdater_,
      diagnostic_updater::FrequencyStatusParam(
      &jointStatePublisherMinFrequency_, &jointStatePublisherMaxFrequency_, 0.1,
      10));

    // init joint state publisher
    jointStatePublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>(
      jointStatePublisherTopic_, jointStatePublisherQueueSize_);

    // init services
    initServices();

    // init subscriber
    shutdownSubscriber_ = nodeHandle_.subscribe("shutdown", 1,
      &DynamixelNode::shutdownCallback, this);
  }
  
  DynamixelNode::~DynamixelNode() {}
    

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void DynamixelNode::diagnoseSerialConnection(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.add("Serial port device", serialPortDeviceName_);
    status.add("Serial port baud rate", serialPortBaudRate_);
    if (controller_ && controller_->getSerialPort() &&
        controller_->getSerialPort()->isOpen())
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "Serial connection opened on %s.",
        controller_->getSerialPort()->getDevice().c_str());
    else
     status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Serial connection closed on %s.", serialPortDeviceName_.c_str());
  }

  void DynamixelNode::diagnoseMotor(diagnostic_updater::DiagnosticStatusWrapper&
      status) {
    if (motorConnected_) {
      status.add("Motor ID", static_cast<unsigned>(motorId_));
      status.add("Baud rate", 2000000.0 / (servoBaudRate_ + 1));
      status.add("Model number", modelNumber_);
      status.add("Model name", modelName_);
      status.add("Firmware version", static_cast<unsigned>(firmwareVersion_));
      status.add("Ticks number", maxTicks_ + 1);
      status.add("Range [deg]", rangeInDegrees_);
      status.add("Rpm per tick", rpmPerTick_);
      status.add("Return delay time [us]",
        Controller::raw2us(returnDelayTime_));
      status.add("Time offset [ns]", timeOffset_);
      status.add("Clockwise angle limit [rad]", Controller::tick2angle(
        cwAngleLimit_, Controller::deg2rad(rangeInDegrees_), maxTicks_)); // TODO: use positionToAngle here
      status.add("Counterclockwise angle limit [rad]", Controller::tick2angle(
        ccwAngleLimit_, Controller::deg2rad(rangeInDegrees_), maxTicks_));
      status.add("Highest limit temperature [C]", static_cast<unsigned>(
        highestLimitTemperature_));
      status.add("Highest limit voltage [V]", Controller::raw2volt(
        highestLimitVoltage_));
      status.add("Lowest limit voltage [V]",Controller::raw2volt(
        lowestLimitVoltage_));
      status.add("Maximum torque [%]", Controller::raw2torqueRatio(maxTorque_) *
        100.0);
      status.add("Torque enabled", torqueEnabled_);
      if (Controller::isModelMX(modelNumber_)) {
        status.add("P gain", Controller::raw2Kp(pGain_));
        status.add("I gain", Controller::raw2Ki(iGain_));
        status.add("D gain", Controller::raw2Kd(dGain_));
      }
      else {
        status.add("Clockwise compliance margin", cwComplianceMargin_);
        status.add("Counterclockwise compliance margin", ccwComplianceMargin_);
        status.add("Clockwise compliance slope", cwComplianceSlope_);
        status.add("Counterclockwise compliance slope", ccwComplianceSlope_);
      }
      status.add("Goal position [rad]", Controller::tick2angle(
        goalPosition_, Controller::deg2rad(rangeInDegrees_), maxTicks_));
      status.add("Moving speed [rad/s]", Controller::rpm2rps(
        Controller::raw2rpm(movingSpeed_, rpmPerTick_)));
      status.add("Torque limit [%]", Controller::raw2torqueRatio(torqueLimit_) *
        100.0);
      status.add("Present position [rad]", Controller::tick2angle(
        presentPosition_, Controller::deg2rad(rangeInDegrees_), maxTicks_));
      status.add("Present speed [rad/s]", Controller::rpm2rps(
        Controller::raw2rpm(presentSpeed_, rpmPerTick_)));
      status.add("Present load [%]", Controller::raw2torqueRatio(presentLoad_) *
        100.0);
      status.add("Present voltage [V]", Controller::raw2volt(presentVoltage_));
      status.add("Present temperature [C]", static_cast<unsigned>(
        presentTemperature_));
      status.add("Moving", moving_);
      status.add("Punch", punch_);
      if (Controller::isModelTorqueControl(modelNumber_)) {
        status.add("Current [A]", Controller::raw2amp(current_));
        status.add("Torque control mode enabled", torqueControlModeEnabled_);
        status.add("Goal torque [A]", Controller::rawtorque2amp(goalTorque_));
      }
      if (Controller::isModelMX(modelNumber_))
        status.add("Goal acceleration [rad/s^2]",
          Controller::raw2rps2(goalAcceleration_));
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Motor connected");
    }
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "No motor connected");
  }

  void DynamixelNode::spin() {
    controller_ = std::make_shared<Controller>(
      std::make_shared<SerialPort>(serialPortDeviceName_, serialPortBaudRate_));
    ros::Rate loopRate(acquisitionLoopRate_);
    auto jointState = boost::make_shared<sensor_msgs::JointState>();
    jointState->header.frame_id = jointStatePublisherFrameId_;
    jointState->name.resize(1);
    jointState->name[0] = jointStatePublisherFrameId_;
    jointState->position.resize(1);
    jointState->velocity.resize(1);
    jointState->effort.resize(1);
    while (nodeHandle_.ok()) {
      try {
        if (!motorConnected_) {
          modelNumber_ = controller_->getModelNumber(motorId_);
          if (!Controller::isModelSupported(modelNumber_))
            throw BadArgumentException<size_t>(modelNumber_,
              "DynamixelNode::spin(): model not supported");
          firmwareVersion_ = controller_->getFirmwareVersion(motorId_);
          returnDelayTime_ = controller_->getReturnDelayTime(motorId_);
          modelName_ = Controller::getModelInformation(modelNumber_).name;
          maxTicks_ = Controller::getModelInformation(modelNumber_).maxTicks;
          rangeInDegrees_ = Controller::getModelInformation(modelNumber_).
            rangeInDegrees;
          rpmPerTick_ = Controller::getModelInformation(modelNumber_).
            rpmPerTick;
          servoBaudRate_ = controller_->getBaudRate(motorId_);
          controller_->getAngleLimits(motorId_, cwAngleLimit_, ccwAngleLimit_);
          highestLimitTemperature_ = controller_->getHighestLimitTemperature(
            motorId_);
          highestLimitVoltage_ = controller_->getHighestLimitVoltage(motorId_);
          lowestLimitVoltage_ = controller_->getLowestLimitVoltage(motorId_);
          maxTorque_ = controller_->getMaxTorque(motorId_);
          torqueEnabled_ = controller_->isTorqueEnable(motorId_);
          punch_ = controller_->getPunch(motorId_);
          if (Controller::isModelMX(modelNumber_)) {
            controller_->getPIDGains(motorId_, pGain_, iGain_, dGain_);
            goalAcceleration_ = controller_->getGoalAcceleration(motorId_);
          }
          else
            controller_->getCompliance(motorId_, cwComplianceMargin_,
              ccwComplianceMargin_, cwComplianceSlope_, ccwComplianceSlope_);
          controller_->getGoalPositionSpeedTorque(motorId_, goalPosition_,
            movingSpeed_, torqueLimit_);
          if (Controller::isModelTorqueControl(modelNumber_)) {
            current_ = controller_->getCurrent(motorId_);
            torqueControlModeEnabled_ =
              controller_->isTorqueControlModeEnable(motorId_);
            goalTorque_ = controller_->getGoalTorque(motorId_);
          }
          controller_->setReturnDelayTime(motorId_, 0);
        }
        auto start = std::chrono::high_resolution_clock::now();
        controller_->getState(motorId_, presentPosition_, presentSpeed_,
          presentLoad_, presentVoltage_, presentTemperature_, registered_,
          moving_);
        auto end = std::chrono::high_resolution_clock::now();
        auto timestamp = ros::Time::now();
        timeOffset_ = std::round(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
          end - start).count() / 2.0);
        timestamp -= ros::Duration(0, timeOffset_);
        jointStatePublisherFrequencyDiagnostic_->tick();
        motorConnected_ = true;
        currentAngle_ = positionToAngle(presentPosition_);
        if (jointStatePublisher_.getNumSubscribers() > 0) {
          jointState->header.stamp = timestamp;
          jointState->position[0] = currentAngle_;
          jointState->velocity[0] = Controller::rpm2rps(
            Controller::raw2rpm(presentSpeed_, rpmPerTick_));
          jointState->effort[0] = Controller::raw2torqueRatio(presentLoad_);
          jointStatePublisher_.publish(jointState);
        }
      }
      catch (const IOException& e) {
        ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
        ROS_WARN_STREAM_NAMED("dynamixel_node", "Retrying in " << retryTimeout_
          << " [s]");
        motorConnected_ = false;
        controller_->getSerialPort()->close();
        std::this_thread::sleep_for(std::chrono::milliseconds(
          static_cast<int64_t>(std::round(retryTimeout_ * std::milli::den))));
      }
      catch (const SystemException& e) {
        ROS_WARN_STREAM_NAMED("dynamixel_node", "SystemException: "
          << e.what());
        ROS_WARN_STREAM_NAMED("dynamixel_node", "Retrying in "
          << retryTimeout_ << " [s]");
        motorConnected_ = false;
        controller_->getSerialPort()->close();
        std::this_thread::sleep_for(std::chrono::milliseconds(
          static_cast<int64_t>(std::round(retryTimeout_ * std::milli::den))));
      }
      catch (const BadArgumentException<size_t>& e) {
        ROS_WARN_STREAM_NAMED("dynamixel_node", "BadArgumentException: "
          << e.what());
        ROS_WARN_STREAM_NAMED("dynamixel_node", "Retrying in " << retryTimeout_
          << " [s]");
        motorConnected_ = false;
        controller_->getSerialPort()->close();
        std::this_thread::sleep_for(std::chrono::milliseconds(
          static_cast<int64_t>(std::round(retryTimeout_ * std::milli::den))));
      }
      diagnosticUpdater_.update();
      ros::spinOnce();
      loopRate.sleep();
    }
  }
  
  void DynamixelNode::goToDefaultPositionAndShutdown() {
    shutdownServices();
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    if (!setGoalPositionDefault(req, res)) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "Default goal position could not be set.");
    }
    double timeout = 3.0;
    const double timeStep = 0.1;
    while (timeout > 0.0) {
      controller_->getState(motorId_, presentPosition_, presentSpeed_,
        presentLoad_, presentVoltage_, presentTemperature_, registered_,
        moving_);
      currentAngle_ = positionToAngle(presentPosition_);
      if (std::abs(defaultAngle_ - currentAngle_) < 0.05) {
        ROS_INFO_STREAM_NAMED("dynamixel_node", "Default goal position has been reached.");
        break;
      }
      usleep(1e6 * timeStep);
      timeout -= timeStep;
    }
    if (timeout <= 0.0) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "Default goal position has not been reached.");
    }
    std_msgs::BoolPtr shutdown(new std_msgs::Bool());
    shutdown->data = true;
    shutdownCallback(shutdown);
    ROS_INFO_STREAM_NAMED("dynamixel_node", "Shutdown executed.");
  }

  void DynamixelNode::getParameters() {
    // miscellaneous parameters
    acquisitionLoopRate_ = param_io::param<double>(nodeHandle_, "ros/acquisition_loop_rate", 100.0);
    retryTimeout_ = param_io::param<double>(nodeHandle_, "ros/retry_timeout", 1.0);

    // sensor parameters
    serialPortDeviceName_ = param_io::param<std::string>(nodeHandle_, "sensor/serial_port_device_name", "/dev/ttyUSB0");
    serialPortBaudRate_ = param_io::param<int>(nodeHandle_, "sensor/serial_port_baud_rate", 1000000);
    deviceName_ = param_io::param<std::string>(nodeHandle_, "sensor/device_name", "Dynamixel controller");
    motorId_ = param_io::param<int>(nodeHandle_, "sensor/motor_id", 1);

    // joint state publisher parameters
    jointStatePublisherTopic_ = param_io::param<std::string>(nodeHandle_, "joint_state_publisher/topic", "joint_state");
    jointStatePublisherQueueSize_ = param_io::param<int>(nodeHandle_, "joint_state_publisher/queue_size", 100);
    jointStatePublisherFrameId_ = param_io::param<std::string>(nodeHandle_, "joint_state_publisher/frame_id", "dynamixel");
    jointStatePublisherFreqTolPercentage_ = param_io::param<double>(nodeHandle_, "joint_state_publisher/freq_tol_percentage", 0.1);
    
    // angles
    offsetAngle_ = param_io::param<double>(nodeHandle_, "offset_angle", 0.0);
    std::cout << "read offset: " << offsetAngle_ << std::endl;
    defaultAngle_ = param_io::param<double>(nodeHandle_, "default_angle", 0.0);
  }

  bool DynamixelNode::setPidGains(SetPidGains::Request& request,
      SetPidGains::Response& response) {
    const auto oldPGain = pGain_;
    const auto oldIGain = iGain_;
    const auto oldDGain = dGain_;
    try {
      if (!motorConnected_)
        throw IOException("DynamixelNode::setPidGains: motor not connected");
      if (!Controller::isModelMX(modelNumber_))
        throw BadArgumentException<size_t>(modelNumber_,
          "DynamixelNode::setPidGains: not applicable");
      pGain_ = Controller::Kp2raw(request.p_gain);
      iGain_ = Controller::Ki2raw(request.i_gain);
      dGain_ = Controller::Kd2raw(request.d_gain);
      controller_->setPIDGains(motorId_, pGain_, iGain_, dGain_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "DynamixelNode::setPidGains: "
        "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      pGain_ = oldPGain;
      iGain_ = oldIGain;
      dGain_ = oldDGain;
    }
    catch (const BadArgumentException<size_t>& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "BadArgumentException: "
        << e.what());
      response.response = false;
      response.message = e.what();
      pGain_ = oldPGain;
      iGain_ = oldIGain;
      dGain_ = oldDGain;
    }
    return true;
  }

  bool DynamixelNode::getPidGains(GetPidGains::Request& /*request*/,
      GetPidGains::Response& response) {
    if (motorConnected_ && Controller::isModelMX(modelNumber_)) {
      response.p_gain = Controller::raw2Kp(pGain_);
      response.i_gain = Controller::raw2Ki(iGain_);
      response.d_gain = Controller::raw2Kd(dGain_);
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected or not applicable";
    }
    return true;
  }

  bool DynamixelNode::setCompliance(SetCompliance::Request& request,
      SetCompliance::Response& response) {
    const auto oldCwComplianceMargin = cwComplianceMargin_;
    const auto oldCcwComplianceMargin = ccwComplianceMargin_;
    const auto oldCwComplianceSlope = cwComplianceSlope_;
    const auto oldCcwComplianceSlope = ccwComplianceSlope_;
    try {
      if (!motorConnected_)
        throw IOException("DynamixelNode::setCompliance: motor not connected");
      if (Controller::isModelMX(modelNumber_))
        throw BadArgumentException<size_t>(modelNumber_,
          "DynamixelNode::setCompliance: not applicable");
      cwComplianceMargin_ = request.cw_margin;
      ccwComplianceMargin_ = request.ccw_margin;
      cwComplianceSlope_ = request.cw_slope;
      ccwComplianceSlope_ = request.ccw_slope;
      controller_->setCompliance(motorId_, cwComplianceMargin_,
        ccwComplianceMargin_, cwComplianceSlope_, ccwComplianceSlope_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      cwComplianceMargin_ = oldCwComplianceMargin;
      ccwComplianceMargin_ = oldCcwComplianceMargin;
      cwComplianceSlope_ = oldCwComplianceSlope;
      ccwComplianceSlope_ = oldCcwComplianceSlope;
    }
    catch (const BadArgumentException<size_t>& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "BadArgumentException: "
        << e.what());
      response.response = false;
      response.message = e.what();
      cwComplianceMargin_ = oldCwComplianceMargin;
      ccwComplianceMargin_ = oldCcwComplianceMargin;
      cwComplianceSlope_ = oldCwComplianceSlope;
      ccwComplianceSlope_ = oldCcwComplianceSlope;
    }
    return true;
  }

  bool DynamixelNode::getCompliance(GetCompliance::Request&
      /*request*/, GetCompliance::Response& response) {
    if (motorConnected_ && !Controller::isModelMX(modelNumber_)) {
      response.cw_margin = cwComplianceMargin_;
      response.ccw_margin = ccwComplianceMargin_;
      response.cw_slope = cwComplianceSlope_;
      response.ccw_slope = ccwComplianceSlope_;
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected or not applicable";
    }
    return true;
  }

  bool DynamixelNode::setAngleLimits(SetAngleLimits::Request&
      request, SetAngleLimits::Response& response) {
    const auto oldCwAngleLimit = cwAngleLimit_;
    const auto oldCcwAngleLimit = ccwAngleLimit_;
    try {
      if (!motorConnected_)
        throw IOException("DynamixelNode::setAngleLimits: motor not connected");
      if (request.cw_angle_limit == 0.0 &&
          request.ccw_angle_limit == 0.0) { // TODO what happens if 0.0 and 2*M_PI are requested?
        // Go to wheel mode (do not use default angle).
        cwAngleLimit_ = Controller::angle2tick(0.0, Controller::deg2rad(rangeInDegrees_), maxTicks_);
        ccwAngleLimit_ = Controller::angle2tick(0.0, Controller::deg2rad(rangeInDegrees_), maxTicks_);
      } else if (request.ccw_angle_limit > 2.0*M_PI-0.001 &&
                 request.cw_angle_limit < 0.001) {
        cwAngleLimit_ = Controller::angle2tick(0.0, Controller::deg2rad(rangeInDegrees_), maxTicks_);
        ccwAngleLimit_ = Controller::angle2tick(2.0*M_PI, Controller::deg2rad(rangeInDegrees_), maxTicks_);
      } else {
        cwAngleLimit_ = angleToPosition(request.cw_angle_limit);
        ccwAngleLimit_ = angleToPosition(request.ccw_angle_limit);
      }
      controller_->setAngleLimits(motorId_, cwAngleLimit_, ccwAngleLimit_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      cwAngleLimit_ = oldCwAngleLimit;
      ccwAngleLimit_ = oldCcwAngleLimit;
    }
    return true;
  }

  bool DynamixelNode::getAngleLimits(GetAngleLimits::Request&
      /*request*/, GetAngleLimits::Response& response) {
    if (motorConnected_) {
      if (cwAngleLimit_ == 0.0 &&
          ccwAngleLimit_ == 0.0) {
        response.cw_angle_limit = Controller::tick2angle(0, Controller::deg2rad(rangeInDegrees_), maxTicks_);
        response.ccw_angle_limit = Controller::tick2angle(0, Controller::deg2rad(rangeInDegrees_), maxTicks_);
      } else {
        response.cw_angle_limit = positionToAngle(cwAngleLimit_);
        response.ccw_angle_limit = positionToAngle(ccwAngleLimit_);
      }
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected";
    }
    return true;
  }

  bool DynamixelNode::setMaxTorque(SetMaxTorque::Request& request,
      SetMaxTorque::Response& response) {
    const auto oldMaxTorque = maxTorque_;
    try {
      if (!motorConnected_)
        throw IOException("DynamixelNode::setMaxTorque: motor not connected");
      maxTorque_ = Controller::torqueRatio2raw(request.max_torque);
      controller_->setMaxTorque(motorId_, maxTorque_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      maxTorque_ = oldMaxTorque;
    }
    return true;
  }

  bool DynamixelNode::getMaxTorque(GetMaxTorque::Request&
      /*request*/, GetMaxTorque::Response& response) {
    if (motorConnected_) {
      response.max_torque = Controller::raw2torqueRatio(maxTorque_);
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected";
    }
    return true;
  }

  bool DynamixelNode::setTorqueEnable(SetTorqueEnable::Request&
      request, SetTorqueEnable::Response& response) {
    const auto oldTorqueEnabled = torqueEnabled_;
    try {
      if (!motorConnected_)
        throw IOException(
          "DynamixelNode::setTorqueEnable: motor not connected");
      torqueEnabled_ = request.enable_torque;
      controller_->setTorqueEnable(motorId_, torqueEnabled_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      torqueEnabled_ = oldTorqueEnabled;
    }
    return true;
  }

  bool DynamixelNode::getTorqueEnable(GetTorqueEnable::Request&
      /*request*/, GetTorqueEnable::Response& response) {
    if (motorConnected_) {
      response.torque_enabled = torqueEnabled_;
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected";
    }
    return true;
  }

  bool DynamixelNode::setTorqueControlModeEnable(
      SetTorqueControlModeEnable::Request& request,
      SetTorqueControlModeEnable::Response& response) {
    const auto oldTorqueControlModeEnabled = torqueControlModeEnabled_;
    try {
      if (!motorConnected_)
        throw IOException(
          "DynamixelNode::setTorqueControlModeEnable: motor not connected");
      if (!Controller::isModelTorqueControl(modelNumber_))
        throw BadArgumentException<size_t>(modelNumber_,
          "DynamixelNode::setTorqueControlModeEnable: not applicable");
      torqueControlModeEnabled_ = request.enable_torque_control_mode;
      controller_->setTorqueControlModeEnable(motorId_,
        torqueControlModeEnabled_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      torqueControlModeEnabled_ = oldTorqueControlModeEnabled;
    }
    catch (const BadArgumentException<size_t>& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "BadArgumentException: "
        << e.what());
      response.response = false;
      response.message = e.what();
      torqueControlModeEnabled_ = oldTorqueControlModeEnabled;
    }
    return true;
  }

  bool DynamixelNode::getTorqueControlModeEnable(
      GetTorqueControlModeEnable::Request& /*request*/,
      GetTorqueControlModeEnable::Response& response) {
    if (motorConnected_ & Controller::isModelTorqueControl(modelNumber_)) {
      response.torque_control_mode_enabled = torqueControlModeEnabled_;
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected or not applicable";
    }
    return true;
  }

  bool DynamixelNode::setGoalTorque(SetGoalTorque::Request& request,
      SetGoalTorque::Response& response) {
    const auto oldGoalTorque = goalTorque_;
    const auto oldTorqueLimit = torqueLimit_;
    try {
      if (!motorConnected_)
        throw IOException(
          "DynamixelNode::setGoalTorque: motor not connected");
      if (!Controller::isModelTorqueControl(modelNumber_))
        throw BadArgumentException<size_t>(modelNumber_,
          "DynamixelNode::setGoalTorque: not applicable");
      goalTorque_ = Controller::amp2rawtorque(request.goal_torque);
      torqueLimit_ = Controller::torqueRatio2raw(request.torque_limit);
      controller_->setGoalTorque(motorId_, goalTorque_);
      if (torqueLimit_ != oldTorqueLimit)
        controller_->setTorqueLimit(motorId_, torqueLimit_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      goalTorque_ = oldGoalTorque;
      torqueLimit_ = oldTorqueLimit;
    }
    catch (const BadArgumentException<size_t>& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "BadArgumentException: "
        << e.what());
      response.response = false;
      response.message = e.what();
      goalTorque_ = oldGoalTorque;
      torqueLimit_ = oldTorqueLimit;
    }
    return true;
  }

  bool DynamixelNode::getGoalTorque(GetGoalTorque::Request&
      /*request*/, GetGoalTorque::Response& response) {
    if (motorConnected_ & Controller::isModelTorqueControl(modelNumber_)) {
      response.goal_torque = Controller::rawtorque2amp(goalTorque_);
      response.torque_limit = Controller::raw2torqueRatio(torqueLimit_);
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected or not applicable";
    }
    return true;
  }

  bool DynamixelNode::setGoalPosition(SetGoalPosition::Request&
      request, SetGoalPosition::Response& response) {
    const auto oldGoalPosition = goalPosition_;
    const auto oldMovingSpeed = movingSpeed_;
    const auto oldTorqueLimit = torqueLimit_;
    try {
      if (!motorConnected_)
        throw IOException(
          "DynamixelNode::setGoalPosition: motor not connected");
      goalPosition_ = angleToPosition(request.goal_position);
      movingSpeed_ = Controller::rpm2raw(Controller::rps2rpm(
        request.moving_speed), rpmPerTick_);
      torqueLimit_ = Controller::torqueRatio2raw(request.torque_limit);
      if (movingSpeed_ == oldMovingSpeed && torqueLimit_ == oldTorqueLimit)
        controller_->setGoalPosition(motorId_, goalPosition_);
      else if (movingSpeed_ != oldMovingSpeed && torqueLimit_ == oldTorqueLimit)
        controller_->setGoalPositionSpeed(motorId_, goalPosition_,
          movingSpeed_);
      else if (movingSpeed_ != oldMovingSpeed && torqueLimit_ != oldTorqueLimit)
        controller_->setGoalPositionSpeedTorque(motorId_, goalPosition_,
          movingSpeed_, torqueLimit_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      goalPosition_ = oldGoalPosition;
      movingSpeed_ = oldMovingSpeed;
      torqueLimit_ = oldTorqueLimit;
    }
    return true;
  }

  bool DynamixelNode::getGoalPosition(GetGoalPosition::Request&
      /*request*/, GetGoalPosition::Response& response) {
    if (motorConnected_) {
      response.goal_position = positionToAngle(goalPosition_);
      response.moving_speed = Controller::rpm2rps(
        Controller::raw2rpm(movingSpeed_, rpmPerTick_));
      response.torque_limit = Controller::raw2torqueRatio(torqueLimit_);
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected";
    }
    return true;
  }

  bool DynamixelNode::setMovingSpeed(SetMovingSpeed::Request&
      request, SetMovingSpeed::Response& response) {
    const auto oldMovingSpeed = movingSpeed_;
    const auto oldTorqueLimit = torqueLimit_;
    try {
      if (!motorConnected_)
        throw IOException(
          "DynamixelNode::setMovingSpeed: motor not connected");
      movingSpeed_ = Controller::rpm2raw(Controller::rps2rpm(
        request.moving_speed), rpmPerTick_);
      torqueLimit_ = Controller::torqueRatio2raw(request.torque_limit);
      controller_->setMovingSpeed(motorId_, movingSpeed_);
      if (torqueLimit_ != oldTorqueLimit)
        controller_->setTorqueLimit(motorId_, torqueLimit_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      movingSpeed_ = oldMovingSpeed;
      torqueLimit_ = oldTorqueLimit;
    }
    return true;
  }

  bool DynamixelNode::getMovingSpeed(GetMovingSpeed::Request&
      /*request*/, GetMovingSpeed::Response& response) {
    if (motorConnected_) {
      response.moving_speed = Controller::rpm2rps(
        Controller::raw2rpm(movingSpeed_, rpmPerTick_));
      response.torque_limit = Controller::raw2torqueRatio(torqueLimit_);
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected";
    }
    return true;
  }

  bool DynamixelNode::setGoalAcceleration(
      SetGoalAcceleration::Request& request,
      SetGoalAcceleration::Response& response) {
    const auto oldGoalAcceleration = goalAcceleration_;
    try {
      if (!motorConnected_)
        throw IOException(
          "DynamixelNode::setGoalAcceleration: motor not connected");
      if (!Controller::isModelMX(modelNumber_))
        throw BadArgumentException<size_t>(modelNumber_,
          "DynamixelNode::setGoalAcceleration: not applicable");
      goalAcceleration_ = Controller::rps22raw(request.goal_acceleration);
      controller_->setGoalAcceleration(motorId_, goalAcceleration_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      goalAcceleration_ = oldGoalAcceleration;
    }
    catch (const BadArgumentException<size_t>& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "BadArgumentException: "
        << e.what());
      response.response = false;
      response.message = e.what();
      goalAcceleration_ = oldGoalAcceleration;
    }
    return true;
  }

  bool DynamixelNode::getGoalAcceleration(
      GetGoalAcceleration::Request& /*request*/,
      GetGoalAcceleration::Response& response) {
    if (motorConnected_ & Controller::isModelMX(modelNumber_)) {
      response.goal_acceleration = Controller::raw2rps2(goalAcceleration_);
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected or not applicable";
    }
    return true;
  }

  bool DynamixelNode::setGoalPositionDefault(
      std_srvs::Empty::Request& /*request*/,
      std_srvs::Empty::Response& /*response*/) {

    SetAngleLimits::Request setAngleLimitsReq;
    setAngleLimitsReq.cw_angle_limit = 0.0;
    setAngleLimitsReq.ccw_angle_limit = 2.0*M_PI;
    SetAngleLimits::Response setAngleLimitsRes;
    if (!setAngleLimits(setAngleLimitsReq, setAngleLimitsRes)) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "Angle limits could not be set.");
    }

    controller_->getState(motorId_, presentPosition_, presentSpeed_,
            presentLoad_, presentVoltage_, presentTemperature_, registered_,
            moving_);
    currentAngle_ = positionToAngle(presentPosition_);
    const double angleDiff = M_PI - Controller::wrapAngle(currentAngle_);
    SetGoalPosition::Request setGoalPositionReq;
    setGoalPositionReq.goal_position = M_PI;
    setGoalPositionReq.moving_speed = angleDiff < 0.0 ? -2.0 : 2.0;
    setGoalPositionReq.torque_limit = 0.5;
    SetGoalPosition::Response setGoalPositionRes;
    if (!setGoalPosition(setGoalPositionReq, setGoalPositionRes)) {
      ROS_ERROR_STREAM_NAMED("dynamixel_node", "Goal position could not be set.");
      return false;
    }

    return true;

/*
    if (defaultAngle_ >= 0.0 && defaultAngle_ <= 2.0 * M_PI) {
      double angleDiff = Controller::wrapAngle(defaultAngle_ - currentAngle_);
      if (fabs(angleDiff) > 0.05) {
        // go to default angle
        const auto oldGoalPosition = goalPosition_;
        const auto oldMovingSpeed = movingSpeed_;
        const auto oldTorqueLimit = torqueLimit_;
        try {
          goalPosition_ = Controller::angle2tick(defaultAngle_,
            Controller::deg2rad(rangeInDegrees_), maxTicks_);
          movingSpeed_ = Controller::rpm2raw(Controller::rps2rpm(
            (angleDiff > 0.0 ? 2.0 : -2.0)), rpmPerTick_);
          torqueLimit_ = Controller::torqueRatio2raw(0.5);
          if (movingSpeed_ == oldMovingSpeed && torqueLimit_ == oldTorqueLimit)
            controller_->setGoalPosition(motorId_, goalPosition_);
          else if (movingSpeed_ != oldMovingSpeed && torqueLimit_ == oldTorqueLimit)
            controller_->setGoalPositionSpeed(motorId_, goalPosition_,
              movingSpeed_);
          else if (movingSpeed_ != oldMovingSpeed && torqueLimit_ != oldTorqueLimit)
            controller_->setGoalPositionSpeedTorque(motorId_, goalPosition_,
              movingSpeed_, torqueLimit_);
          return true;
        }
        catch (const IOException& e) {
          ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
          goalPosition_ = oldGoalPosition;
          movingSpeed_ = oldMovingSpeed;
          torqueLimit_ = oldTorqueLimit;
        }
      }
    }
*/
    return false;
  }

  void DynamixelNode::shutdownCallback(
      const std_msgs::BoolConstPtr& shutdown) {
    if (shutdown) {
      shutdownServices();
      SetTorqueEnable::Request req;
      SetTorqueEnable::Response res;
      req.enable_torque = false;
      setTorqueEnable(req, res);
    } else {
      SetTorqueEnable::Request req;
      SetTorqueEnable::Response res;
      req.enable_torque = true;
      setTorqueEnable(req, res);
      initServices();
    }
  }

  void DynamixelNode::initServices() {
    if (servicesAreRunning_) {
      return;
    }
    setPidGainsService_ = nodeHandle_.advertiseService(
        "set_pid_gains", &DynamixelNode::setPidGains, this);
    getPidGainsService_ = nodeHandle_.advertiseService(
        "get_pid_gains", &DynamixelNode::getPidGains, this);
    setComplianceService_ = nodeHandle_.advertiseService(
        "set_compliance", &DynamixelNode::setCompliance, this);
    getComplianceService_ = nodeHandle_.advertiseService(
        "get_compliance", &DynamixelNode::getCompliance, this);
    setAngleLimitsService_ = nodeHandle_.advertiseService(
        "set_angle_limits", &DynamixelNode::setAngleLimits, this);
    getAngleLimitsService_ = nodeHandle_.advertiseService(
        "get_angle_limits", &DynamixelNode::getAngleLimits, this);
    setMaxTorqueService_ = nodeHandle_.advertiseService(
        "set_max_torque", &DynamixelNode::setMaxTorque, this);
    getMaxTorqueService_ = nodeHandle_.advertiseService(
        "get_max_torque", &DynamixelNode::getMaxTorque, this);
    setTorqueEnableService_ = nodeHandle_.advertiseService(
        "set_torque_enable", &DynamixelNode::setTorqueEnable, this);
    getTorqueEnableService_ = nodeHandle_.advertiseService(
        "get_torque_enable", &DynamixelNode::getTorqueEnable, this);
    setTorqueControlModeEnableService_ = nodeHandle_.advertiseService(
        "set_torque_control_mode_enable",
        &DynamixelNode::setTorqueControlModeEnable, this);
    getTorqueControlModeEnableService_ = nodeHandle_.advertiseService(
        "get_torque_control_mode_enable",
        &DynamixelNode::getTorqueControlModeEnable, this);
    setGoalTorqueService_ = nodeHandle_.advertiseService(
        "set_goal_torque", &DynamixelNode::setGoalTorque, this);
    getGoalTorqueService_ = nodeHandle_.advertiseService(
        "get_goal_torque", &DynamixelNode::getGoalTorque, this);
    setGoalPositionService_ = nodeHandle_.advertiseService(
        "set_goal_position", &DynamixelNode::setGoalPosition, this);
    getGoalPositionService_ = nodeHandle_.advertiseService(
        "get_goal_position", &DynamixelNode::getGoalPosition, this);
    setMovingSpeedService_ = nodeHandle_.advertiseService(
        "set_moving_speed", &DynamixelNode::setMovingSpeed, this);
    getMovingSpeedService_ = nodeHandle_.advertiseService(
        "get_moving_speed", &DynamixelNode::getMovingSpeed, this);
    setGoalAccelerationService_ = nodeHandle_.advertiseService(
        "set_goal_acceleration", &DynamixelNode::setGoalAcceleration, this);
    getGoalAccelerationService_ = nodeHandle_.advertiseService(
        "get_goal_acceleration", &DynamixelNode::getGoalAcceleration, this);
    setGoalPositionDefaultService_ = nodeHandle_.advertiseService(
        "set_goal_position_default", &DynamixelNode::setGoalPositionDefault, this);
    servicesAreRunning_ = true;
  }

  void DynamixelNode::shutdownServices() {
    if (!servicesAreRunning_) {
      return;
    }
    setPidGainsService_.shutdown();
    getPidGainsService_.shutdown();
    setComplianceService_.shutdown();
    getComplianceService_.shutdown();
    setAngleLimitsService_.shutdown();
    getAngleLimitsService_.shutdown();
    setMaxTorqueService_.shutdown();
    getMaxTorqueService_.shutdown();
    setTorqueEnableService_.shutdown();
    getTorqueEnableService_.shutdown();
    setTorqueControlModeEnableService_.shutdown();
    getTorqueControlModeEnableService_.shutdown();
    setGoalTorqueService_.shutdown();
    getGoalTorqueService_.shutdown();
    setGoalPositionService_.shutdown();
    getGoalPositionService_.shutdown();
    setMovingSpeedService_.shutdown();
    getMovingSpeedService_.shutdown();
    setGoalAccelerationService_.shutdown();
    getGoalAccelerationService_.shutdown();
    servicesAreRunning_ = false;
  }

  void DynamixelNode::handleSignal(const int signum) {
    ROS_INFO_STREAM_NAMED("dynamixel_node", "Handled received signal (" << signum << ") ...");
    goToDefaultPositionAndShutdown();
    if (signum == SIGSEGV)
    {
      signal(signum, SIG_DFL);
      kill(getpid(), signum);
    }
    ros::requestShutdown();
  }

  uint16_t DynamixelNode::angleToPosition(double angle) {
    return Controller::angle2tick(Controller::wrapAngle(angle + offsetAngle_), Controller::deg2rad(rangeInDegrees_), maxTicks_);
  }

  double DynamixelNode::positionToAngle(uint16_t position) {
    return Controller::wrapAngle(Controller::tick2angle(position, Controller::deg2rad(rangeInDegrees_), maxTicks_) - offsetAngle_);
  }

}
