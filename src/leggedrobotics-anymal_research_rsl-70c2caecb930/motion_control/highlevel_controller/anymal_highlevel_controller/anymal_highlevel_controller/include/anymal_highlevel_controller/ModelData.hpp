/*!
 * @file    Model.hpp
 * @author  Christian Gehring, Dario Bellicoso
 * @date    Oct, 2014
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <anymal_msgs/AnymalState.h>
#include <sensor_msgs/Joy.h>
#include <series_elastic_actuator_msgs/SeActuatorCommands.h>
#include <series_elastic_actuator_msgs/SeActuatorReadings.h>
#include <series_elastic_actuator_msgs/SeActuatorCommand.h>
#include <series_elastic_actuator_msgs/SeActuatorState.h>

#include <any_measurements/Imu.hpp>
#include <any_measurements/Pose.hpp>
#include <any_measurements/PoseWithCovariance.hpp>
#include <any_measurements/Twist.hpp>
#include <anymal_model/AnymalModel.hpp>
#include <anymal_model/actuator_containers.hpp>
#include <anymal_model/AnymalState.hpp>
#include <anymal_roco/anymal_roco.hpp>

#include <kindr/Core>

#include <boost/thread.hpp>
#include <array>

namespace anymal_highlevel_controller {


//! Measurements, states, commands and model of the robot.
class ModelData {
 private:
  using AD = anymal_description::AnymalDescription;

 public:
  static constexpr int numberOfJoints_ = AD::getJointsDimension();
  typedef kindr::RotationQuaternionPD RotationQuaternion;
  typedef kindr::RotationMatrixPD     RotationMatrix;
  typedef kindr::EulerAnglesZyxPD EulerAnglesZyx;
  typedef kindr::LocalAngularVelocityPD LocalAngularVelocity;
  typedef kindr::AngleAxisPD AngleAxis;
  typedef kindr::Position3D Position;
  typedef kindr::Velocity3D LinearVelocity;
  typedef kindr::VectorTypeless3D Vector;
  typedef kindr::Force3D Force;

  typedef kindr::Position<double, numberOfJoints_> JointPositions;
  typedef kindr::Velocity<double, numberOfJoints_> JointVelocities;
  typedef kindr::Torque<double, numberOfJoints_> JointTorques;

  //! The localization contains the map's pose expressed in the world frame.
  using Localization = any_measurements::PoseWithCovariance;
  using LocalizationMsg = geometry_msgs::PoseWithCovarianceStamped;

 public:
  ModelData();
  virtual ~ModelData() = default;

  void init(double dt, bool isRealRobot, const std::string& filePath);

  void addVariablesToLog();

  //! Switch to a specific leg configuration.
  void switchLegConfigurationsTo(const anymal_model::LegConfigurations& legConfigAnymal);

  void setJoystickCommands(const sensor_msgs::Joy& joy);
  void setCommandVelocity(const any_measurements::Twist& msg);
  void setCommandPose(const any_measurements::Pose& msg);

  void setActuatorReadings(const anymal_model::ActuatorReadingRobotContainer& msg);

  std::shared_ptr<anymal_roco::RocoState>& getState();
  std::shared_ptr<anymal_roco::RocoCommand>& getCommand();

  const std::shared_ptr<anymal_roco::RocoState>& getState() const;
  std::shared_ptr<boost::shared_mutex>& getStateMutex();

  const std::shared_ptr<anymal_roco::RocoCommand>& getCommand() const;
  std::shared_ptr<boost::shared_mutex>& getCommandMutex();

  //! Write actuator commands to shared memory.
  void getActuatorCommands(anymal_model::ActuatorCommandRobotContainer& commands) const;
  //! Set state from shared memory.
  void setAnymalState(const anymal_model::ExtendedAnymalState& state);

  //! Set localization.
  void setLocalization(const LocalizationMsg& localization);

  void setImuMeasurements(const any_measurements::Imu& imu);

 protected:
  //! Model with measured state.
  std::shared_ptr<anymal_model::AnymalModel> anymalModelMeasured_;
  //! Model with desired stated.
  std::shared_ptr<anymal_model::AnymalModel> anymalModelDesired_;
  //! State of the robot, which the controllers can access.
  std::shared_ptr<anymal_roco::RocoState> state_;
  std::shared_ptr<boost::shared_mutex> mutexState_;
  //! Commands of the robot, which the controllers can access.
  std::shared_ptr<anymal_roco::RocoCommand> command_;
  std::shared_ptr<boost::shared_mutex> mutexCommand_;
};

} /* namespace model */
