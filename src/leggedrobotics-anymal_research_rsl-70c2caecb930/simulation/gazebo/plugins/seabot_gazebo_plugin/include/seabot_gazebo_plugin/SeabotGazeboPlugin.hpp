/*!
 * @file    SeabotGazeboPlugin.hpp
 * @author  Markus Staeuble
 * @date    Dec, 2017
 */

#include <anybot_gazebo_plugin/AnybotGazeboPlugin.hpp>
#include <series_elastic_actuator_ros/series_elastic_actuator_ros.hpp>
// actuators
#include <series_elastic_actuator_sim/SeActuatorPerfectTorqueSource.hpp>
// romo
#include <romo_std/common/actuator_containers.hpp>

#pragma once

namespace gazebo {

template <typename ConcreteDescription_>
class SeabotGazeboPlugin : public AnybotGazeboPlugin<ConcreteDescription_> {
 public:
  using ConcreteDescription = ConcreteDescription_;

  template <typename Msg_, typename MsgRos_>
  using SeReadingConversionTrait = series_elastic_actuator_ros::ConversionTraits<ConcreteDescription, Msg_, MsgRos_>;

  template <typename Msg_, typename MsgRos_>
  using SeCommandConversionTrait = series_elastic_actuator_ros::ConversionTraits<ConcreteDescription, Msg_, MsgRos_>;

  using ActuatorContainer =
      romo_std::ActuatorPerfectTorqueSourceRobotContainer<ConcreteDescription, series_elastic_actuator_sim::SeActuatorPerfectTorqueSource>;
  using ActuatorCommandRobotContainer = romo_std::ActuatorCommandContainer<ConcreteDescription, series_elastic_actuator::SeActuatorCommand>;
  using ActuatorReadingRobotContainer = romo_std::ActuatorReadingContainer<ConcreteDescription, series_elastic_actuator::SeActuatorReading>;

  using ActuatorCommandsShm = ActuatorCommandRobotContainer;
  using ActuatorCommandsRos = series_elastic_actuator_msgs::SeActuatorCommands;

  using ActuatorReadingsShm = ActuatorReadingRobotContainer;
  using ActuatorReadingsRos = series_elastic_actuator_msgs::SeActuatorReadings;

  using ActuatorEnum = typename ConcreteDescription::ActuatorEnum;

  explicit SeabotGazeboPlugin(const std::string& robotName);

  virtual ~SeabotGazeboPlugin() = default;

  virtual void Reset() override;

 protected:
  // Inits joint structures.
  void initJointStructures() override;

  void resetJointStructures();

  // Publishes actuator readings over ROS.
  void publishActuatorReadings() override;
  // Override to publish custom actuator readings from a derived class.
  virtual void publishActuatorReadingsDerived() {}

  void updateActuatorReading(const ActuatorEnum& actuatorEnum, const double jointPosition, const double jointVelocity,
                             const double jointAcceleration, const double jointTorque) override;

  double computeActuatorCommand(const ActuatorEnum& actuatorEnum) override;

  void receiveMessages() override;

  void actuatorCommandsCallback(const ActuatorCommandsShm& msg);

  virtual void setActuatorGains();

  // ROS actuator readings publisher.
  cosmo_ros::PublisherRosPtr<ActuatorReadingsShm, ActuatorReadingsRos, SeReadingConversionTrait> actuatorReadingsPublisher_;

  // Subscriber
  cosmo_ros::SubscriberRosPtr<ActuatorCommandsShm, ActuatorCommandsRos, SeCommandConversionTrait> actuatorCommandsSubscriber_;

  ActuatorCommandRobotContainer actuatorCommands_;
  ActuatorCommandRobotContainer actuatorDefaultPositionCommands_;
  ActuatorReadingRobotContainer actuatorReadings_;
  ActuatorContainer actuatorModel_;
  const std::chrono::microseconds receiveMaxLockTime_;
};

}  // namespace gazebo

#include <seabot_gazebo_plugin/SeabotGazeboPlugin.tpp>
