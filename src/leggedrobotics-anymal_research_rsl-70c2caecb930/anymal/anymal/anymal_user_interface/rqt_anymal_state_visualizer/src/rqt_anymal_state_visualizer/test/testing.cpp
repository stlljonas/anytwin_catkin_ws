#include <algorithm>
#include <random>

// ros
#include <ros/ros.h>
#include <std_msgs/builtin_double.h>
#include <sensor_msgs/BatteryState.h>

// msgs
#ifdef ANYDRIVE1X
#include <series_elastic_actuator_msgs/SeActuatorReadingsExtended.h>
#else
#include <anydrive_msgs/ReadingsExtended.h>
#endif

// notification
#include <notification/NotificationPublisher.hpp>

bool isRosOk() {
  return ros::ok();
}

int main(int argc, char **argv) {
#ifdef ANYDRIVE1X
  using ActuatorReading = series_elastic_actuator_msgs::SeActuatorReadingExtended;
  using ActuatorReadings = series_elastic_actuator_msgs::SeActuatorReadingsExtended;
#else
  using ActuatorReading = anydrive_msgs::ReadingExtended;
  using ActuatorReadings = anydrive_msgs::ReadingsExtended;
#endif

  // Set up ROS node.
  ros::init(argc, argv, "testing");
  ros::NodeHandle nodeHandle("~");

  std::random_device randomDevice;
  std::default_random_engine defaultRandomEngine(randomDevice());
  std::uniform_real_distribution<double> uniformRealDistribution(-10.0, 10.0);

  // Set up publishers.
  notification::NotificationPublisher screenPublisher("robot_visualizer", nodeHandle, true);
  ros::Publisher pubBattery = nodeHandle.advertise<sensor_msgs::BatteryState>("/rpsm_lpc/battery_state", 1);
  ros::Publisher pubSea = nodeHandle.advertise<ActuatorReadings>(
      "/anymal_lowlevel_controller/actuator_readings_extended_throttled", 1);

  sensor_msgs::BatteryState msg;

  ActuatorReadings ex;
  for (int i = 0; i < 12; ++i) {
    ActuatorReading reading;
    ex.readings.push_back(reading);
  }

  //ros::Duration(1.5).sleep();

  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "OPTOFORCE_LF_FOOT");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "OPTOFORCE_RF_FOOT");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "OPTOFORCE_LH_FOOT");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "OPTOFORCE_RH_FOOT");

  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LF_HAA");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LF_HFE");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LF_KFE");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "RF_HAA");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "RF_HFE");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "RF_KFE");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LH_HAA");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LH_HFE");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LH_KFE");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "RH_HAA");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "RH_HFE");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "RH_KFE");

  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "ESTIMATOR");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LLC");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LMC");
  screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "IMU");

  //ros::Duration(1.0).sleep();

  double sleeping = 0.1;
  double sleepingLong = 0.5;
  double sleepingBattery = 0.5;
  ros::Rate rate(100.0);
  while (ros::ok()) {
    // test feet
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "WARN 123", "OPTOFORCE_LF_FOOT"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "WARN 123", "OPTOFORCE_RF_FOOT"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "WARN 123", "OPTOFORCE_LH_FOOT"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "WARN 123", "OPTOFORCE_RH_FOOT"); //ros::Duration(sleeping).sleep();
    ros::Duration(sleepingLong).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "ERROR 123", "OPTOFORCE_LF_FOOT"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "ERROR 123", "OPTOFORCE_RF_FOOT"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "ERROR 123", "OPTOFORCE_LH_FOOT"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "ERROR 123", "OPTOFORCE_RH_FOOT"); //ros::Duration(sleeping).sleep();
    ros::Duration(sleepingLong).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "OPTOFORCE_LF_FOOT"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "OPTOFORCE_RF_FOOT"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "OPTOFORCE_LH_FOOT"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "OPTOFORCE_RH_FOOT"); //ros::Duration(sleeping).sleep();
    ros::Duration(sleepingLong).sleep(); if (!isRosOk()) { break; }

    //ros::Duration(sleepingLong).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "WARN 123", "ESTIMATOR"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "WARN 123", "LLC"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "WARN 123", "LMC"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "WARN 123", "IMU"); //ros::Duration(sleeping).sleep();
    ros::Duration(sleepingLong).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "ERROR 123", "ESTIMATOR"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "ERROR 123", "LLC"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "ERROR 123", "LMC"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "ERROR 123", "IMU"); //ros::Duration(sleeping).sleep();
    ros::Duration(sleepingLong).sleep();
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "ESTIMATOR"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LLC"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LMC"); //ros::Duration(sleeping).sleep();
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "IMU"); //ros::Duration(sleeping).sleep();

    ros::Duration(sleepingLong).sleep(); if (!isRosOk()) { break; }

    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LF_HFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LF_HAA");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LF_KFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LH_KFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LH_HAA");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "LH_HFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "RH_HFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "RH_HAA");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "RH_KFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "RF_KFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "RF_HAA");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", "RF_HFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "", "LF_HFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "", "LF_HAA");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "", "LF_KFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "", "LH_KFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "", "LH_HAA");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "", "LH_HFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "", "RH_HFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "", "RH_HAA");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "", "RH_KFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "", "RF_KFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "", "RF_HAA");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_WARN, "", "", "RF_HFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "", "LF_HFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "", "LF_HAA");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "", "LF_KFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "", "LH_KFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "", "LH_HAA");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "", "LH_HFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "", "RH_HFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "", "RH_HAA");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "", "RH_KFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "", "RF_KFE");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "", "RF_HAA");
    ros::Duration(sleeping).sleep(); if (!isRosOk()) { break; }
    screenPublisher.notify(notification::Level::LEVEL_ERROR, "", "", "RF_HFE");

//    screenPublisher.notify(notification::Level::LEVEL_INFO, "", "", ""); //ros::Duration(sleeping).sleep();

    msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
    msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    msg.percentage = 0.8;
    msg.voltage = 50.0;
    msg.current = 4.34;
    pubBattery.publish(msg);

    ros::Duration(0.5).sleep();

    msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
    msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
    msg.percentage = 0.25;
    msg.voltage = 54.0;
    msg.current = 5.34;
    pubBattery.publish(msg);

    ros::Duration(0.5).sleep();

    msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
    msg.percentage = 0.09;
    msg.voltage = 40.45;
    msg.current = 4.23;
    pubBattery.publish(msg);

    ros::Duration(0.5).sleep();

    msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
    msg.percentage = 0.0;
    msg.voltage = 20.45;
    msg.current = 1.23;
    pubBattery.publish(msg);

    ros::Duration(0.5).sleep();

    rate.sleep();

    for (auto &item : ex.readings) {
      item.state.joint_position = uniformRealDistribution(defaultRandomEngine);
    }

    pubSea.publish(ex);
  }

  return 0;
}
