/*
 * @file  gazebo_ros_dynamixel_motor.h
 *
 * @brief A configurable plugin that controls one or more joint.
 *
 * @author  Vincenzo Comito <clynamen@gmail.com>
 */

#ifndef GAZEBO_ROS_DYNAMIXEL_MOTOR_H_
#define GAZEBO_ROS_DYNAMIXEL_MOTOR_H_

#include "motor_state.h"

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
// #include <dynamixel_controllers/SetTorque.h>
// #include <dynamixel_controllers/SetTorqueLimit.h>
// #include <dynamixel_controllers/TorqueEnable.h>
#include <dynamixel_controllers/SetSpeed.h>
// #include <dynamixel_controllers/SetThreshold.h>
// #include <dynamixel_controllers/SetCompliancePunch.h>
// #include <dynamixel_controllers/SetComplianceSlope.h>
// #include <dynamixel_controllers/SetComplianceMargin.h>

#include <dynamixel_ros_msgs/SetAngleLimits.h>
#include <dynamixel_ros_msgs/SetGoalPosition.h>
#include <dynamixel_ros_msgs/SetMovingSpeed.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboRosDynamixelMotor : public ModelPlugin {

    public:
	  GazeboRosDynamixelMotor();
    ~GazeboRosDynamixelMotor();

    void InitServices();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    MotorState ReadMotor() const;
    void UpdateMotor(const MotorState& read_motor_state);
    void OnWorldUpdate();

    physics::JointPtr GetReferencedJoint(physics::ModelPtr model, const sdf::ElementPtr& sdf, std::string sdf_element_name);

    static const std::string PLUGIN_NAME;

    protected:
      void Shutdown();

    private:
      /**
       * Create a dynamixel_msgs JointState given a @b motor_state
       */
      dynamixel_msgs::JointState createJointStateMsg(const std::string& name, const MotorState& motor_state);
      bool SetGoalPositionService(dynamixel_ros_msgs::SetGoalPosition::Request& req, dynamixel_ros_msgs::SetGoalPosition::Response& res);
      bool SetAngleLimitsService(dynamixel_ros_msgs::SetAngleLimits::Request& req, dynamixel_ros_msgs::SetAngleLimits::Response& res);
      bool SetMovingSpeedService(dynamixel_ros_msgs::SetMovingSpeed::Request& req, dynamixel_ros_msgs::SetMovingSpeed::Response& res);

      physics::WorldPtr world;
      physics::ModelPtr parent;

      ros::Publisher dynamixel_joint_state_publisher;
      ros::Publisher dynamixel_ros_joint_state_publisher;

      ros::Subscriber command_subscriber;

      ros::ServiceServer set_goal_position_service;
      ros::ServiceServer set_moving_speed_service;
      ros::ServiceServer set_angle_limits_service;

      ros::NodeHandle* rosnode;

      std::string robot_namespace;
      std::string base_topic_name;
      std::string state_topic_name;
      std::string command_topic_name;
      std::string set_angle_limits_service_name;
      std::string set_goal_position_service_name;
      std::string set_moving_speed_service_name;
      std::string joint_state_topic_name;
      std::string joint_name;

      bool alive_;
      physics::JointPtr joint;

      // Update Rate
      double update_rate_;
      double update_period_;
      double motor_allowed_error;

      common::Time last_update_time_;
      event::ConnectionPtr update_connection;
      MotorState current_motor_state;
      std::string motor_name;
  };

}


#endif

