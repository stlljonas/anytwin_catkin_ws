#include <algorithm>
#include <assert.h>
#include <functional>
#include <cmath>

#include <gazebo_plugins/gazebo_ros_dynamixel_motor.h>
#include <gazebo_plugins/motor_state.h>

#include <sdf/sdf.hh>

#include <dynamixel_msgs/JointState.h>
//#include <dynamixel_controllers/SetTorque.h>
#include <dynamixel_controllers/SetTorqueLimit.h>
#include <dynamixel_controllers/TorqueEnable.h>
#include <dynamixel_controllers/SetSpeed.h>
//#include <dynamixel_controllers/SetThreshold.h>
#include <dynamixel_controllers/SetCompliancePunch.h>
#include <dynamixel_controllers/SetComplianceSlope.h>
#include <dynamixel_controllers/SetComplianceMargin.h>

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <std_msgs/Float64.h>

using namespace std;

using MsgType = dynamixel_msgs::JointState;

namespace gazebo {

const std::string GazeboRosDynamixelMotor::PLUGIN_NAME = "GazeboRosDynamixelMotor";

GazeboRosDynamixelMotor::GazeboRosDynamixelMotor() : alive_(true) {}

// Destructor
GazeboRosDynamixelMotor::~GazeboRosDynamixelMotor() {
  ROS_INFO_STREAM(" destroying  ");
  delete rosnode;
}

physics::JointPtr GazeboRosDynamixelMotor::GetReferencedJoint(physics::ModelPtr model, const sdf::ElementPtr& sdf, std::string sdf_element_name)
{
  joint_name = sdf->GetElement(sdf_element_name)->Get<std::string>();
  //ROS_INFO_STREAM("joint name is " << joint_name);
  return model->GetJoint(joint_name);
}

// Load the controller
void GazeboRosDynamixelMotor::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
  using namespace std;
  using namespace sdf;

  this->parent = parent;
  this->world = parent->GetWorld();


  this->robot_namespace = sdf->GetElement("robotNamespace")->Get<std::string>();

  //std::cout << "Namespace: " << this-> robot_namespace << std::endl;

  joint = GetReferencedJoint(parent, sdf, "joint");

  ROS_INFO_STREAM("Starting Dynamixel Gazebo plugin for " << joint_name);

  /*std::vector<physics::JointPtr> joints = parent->GetJoints();

    for(std::vector<int>::size_type i = 0; i != joints.size(); i++) {
        std::cout << joints[i];
    }*/


  if(joint == nullptr) {
    ROS_ERROR_STREAM("No joint was found");
    return;
  }


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  rosnode = new ros::NodeHandle(this->robot_namespace);

  current_motor_state.mode = MotorStateMode::Position;
  if (sdf->HasElement("reduction_value"))
    current_motor_state.demultiply_value = sdf->GetElement("reduction_value")->Get<double>();
  else
    current_motor_state.demultiply_value = 1;
  if (sdf->HasElement("default_pos"))
    current_motor_state.current_pos_rad = sdf->GetElement("default_pos")->Get<double>();
  else
    current_motor_state.current_pos_rad = 0.0;
  if (sdf->HasElement("default_vel_limit"))
    current_motor_state.velocity_limit_rad_s = sdf->GetElement("default_vel_limit")->Get<double>();
  else
    current_motor_state.velocity_limit_rad_s = 1;
  current_motor_state.torque_enabled = true;
  if (sdf->HasElement("allowed_error"))
    motor_allowed_error = sdf->GetElement("allowed_error")->Get<double>();
  else
    motor_allowed_error = 0.001;
  if (sdf->HasElement("default_torque_limit"))
    current_motor_state.torque_limit = sdf->GetElement("default_torque_limit")->Get<double>();
  else
    current_motor_state.torque_limit = 10;
  if (sdf->HasElement("base_topic_name"))
    base_topic_name = sdf->GetElement("base_topic_name")->Get<string>();
  else
    base_topic_name = "dynamixel_motor";

  if (sdf->HasElement("state_topic_name"))
    state_topic_name = sdf->GetElement("state_topic_name")->Get<string>();
  else
    state_topic_name = "dynamixel_motor";
  if (sdf->HasElement("command_topic_name"))
    command_topic_name = sdf->GetElement("command_topic_name")->Get<string>();
  else
    command_topic_name = "dynamixel_motor";
  if (sdf->HasElement("set_goal_position_service_name"))
  {
    set_goal_position_service_name = sdf->GetElement("set_goal_position_service_name")->Get<string>();
    set_goal_position_service = rosnode->advertiseService(set_goal_position_service_name, &GazeboRosDynamixelMotor::SetGoalPositionService, this);
  }
  if (sdf->HasElement("set_moving_speed_service_name"))
  {
    set_moving_speed_service_name = sdf->GetElement("set_moving_speed_service_name")->Get<string>();
    set_moving_speed_service = rosnode->advertiseService(set_moving_speed_service_name, &GazeboRosDynamixelMotor::SetMovingSpeedService, this);
  }
  if (sdf->HasElement("set_angle_limits_service_name"))
  {
    set_angle_limits_service_name = sdf->GetElement("set_angle_limits_service_name")->Get<string>();
    set_angle_limits_service = rosnode->advertiseService(set_angle_limits_service_name, &GazeboRosDynamixelMotor::SetAngleLimitsService, this);
  }
  if (sdf->HasElement("joint_state_topic_name"))
    joint_state_topic_name = sdf->GetElement("joint_state_topic_name")->Get<string>();
  else
    joint_state_topic_name = "/dynamixel_ros/joint_state";

#if GAZEBO_MAJOR_VERSION >= 7
  joint->SetPosition(0, current_motor_state.current_pos_rad);
#else
  joint->SetAngle(0, current_motor_state.current_pos_rad);
#endif

  //ROS_INFO_STREAM("creating subscribers");

  command_subscriber = rosnode->subscribe<std_msgs::Float64>( command_topic_name, 10, [&] (const std_msgs::Float64::ConstPtr& msg) {
    //ROS_INFO_STREAM("setting new position to " << msg->data);
    current_motor_state.mode = MotorStateMode::Position;
    current_motor_state.goal_pos_rad = msg->data;
  }
  );

  //ROS_INFO_STREAM("creating publishers");
  dynamixel_joint_state_publisher = rosnode->advertise<MsgType>( state_topic_name, 10);
  dynamixel_ros_joint_state_publisher = rosnode->advertise<sensor_msgs::JointState>( joint_state_topic_name, 10);

  //string motor_name = sdf->GetElement("motor_name")->Get<string>();

  //ROS_INFO_STREAM("subscribing to world update ");
  // listen to the update event (broadcast every simulation iteration)
  update_connection =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosDynamixelMotor::OnWorldUpdate, this));
}


bool GazeboRosDynamixelMotor::SetGoalPositionService(dynamixel_ros_msgs::SetGoalPosition::Request& req, dynamixel_ros_msgs::SetGoalPosition::Response& res)
{
  if (req.goal_position > 0 && req.goal_position < 2*M_PI)
  {
  current_motor_state.mode = MotorStateMode::Position;
  current_motor_state.goal_pos_rad = req.goal_position;
  current_motor_state.velocity_limit_rad_s = req.moving_speed;
  current_motor_state.torque_limit = req.torque_limit;

  res.message = "Ok";
  res.response = true;
  }
  else
  {
    res.message = "Goal position out of range [0,2Pi]";
    res.response = false;
  }

  return true;
}

bool GazeboRosDynamixelMotor::SetMovingSpeedService(dynamixel_ros_msgs::SetMovingSpeed::Request& req, dynamixel_ros_msgs::SetMovingSpeed::Response& res)
{

  current_motor_state.mode = MotorStateMode::Velocity;
  current_motor_state.velocity_rad_s = req.moving_speed;
  current_motor_state.velocity_limit_rad_s = req.moving_speed;
  current_motor_state.torque_limit = req.torque_limit;

  res.message = "Ok";
  res.response = true;


  return true;
}

bool GazeboRosDynamixelMotor::SetAngleLimitsService(dynamixel_ros_msgs::SetAngleLimits::Request& req, dynamixel_ros_msgs::SetAngleLimits::Response& res)
{
  res.message = "Ok";
  res.response = true;
  return true;
}


// Finalize the controller
void GazeboRosDynamixelMotor::Shutdown() {
  //ROS_INFO_STREAM("shutting down");
  alive_ = false;
  rosnode->shutdown();
}

dynamixel_msgs::JointState GazeboRosDynamixelMotor::createJointStateMsg(const std::string& name, const MotorState& motor_state)
{
  dynamixel_msgs::JointState msg;
  msg.name = name;
  msg.motor_ids = std::vector<int>{ motor_state.motor_id };
  msg.motor_temps = std::vector<int>{ motor_state.motor_temp };
  msg.current_pos = motor_state.current_pos_rad;
  msg.goal_pos = motor_state.goal_pos_rad;
  msg.is_moving = motor_state.is_moving;
  msg.error = motor_state.error_rad;
  msg.velocity = motor_state.velocity_rad_s;
  msg.load = motor_state.load;

  return msg;
}


MotorState gazebo::GazeboRosDynamixelMotor::ReadMotor() const
{
  MotorState read_motor_state = current_motor_state;
#if GAZEBO_MAJOR_VERSION > 8
  read_motor_state.current_pos_rad = joint->Position(0);
#else
  read_motor_state.current_pos_rad = joint->GetAngle(0).Radian();
#endif

  if(read_motor_state.mode == MotorStateMode::Position) {
    double pos_delta_rad = (read_motor_state.goal_pos_rad - read_motor_state.current_pos_rad);
    read_motor_state.error_rad = pos_delta_rad;
  } else {
    read_motor_state.error_rad = 0;
  }

  read_motor_state.is_moving = read_motor_state.velocity_rad_s != 0 && read_motor_state.torque_enabled;
  //read_motor_state.load = joint->GetForceTorque(0).body2Torque.x; // TODO: the axis may be wrong, review this
  read_motor_state.load = 0;

  //read_motor_state.motor_temp = nextGaussian<int>(24, 2);

  return read_motor_state;
}


void GazeboRosDynamixelMotor::UpdateMotor(const MotorState& read_motor_state)
{
  double target_velocity = 0;

  if(read_motor_state.mode == MotorStateMode::Position) {
    double pos_delta_rad = (read_motor_state.goal_pos_rad - read_motor_state.current_pos_rad);
    bool goal_reached =  fabs(pos_delta_rad) < motor_allowed_error;
    if(!goal_reached) {
      target_velocity = copysign(read_motor_state.velocity_limit_rad_s,pos_delta_rad);
    } else {
      target_velocity = 0;
    }
  }
  else if (read_motor_state.mode == MotorStateMode::Velocity) {
	  target_velocity= read_motor_state.velocity_rad_s;
  }

  if(read_motor_state.torque_enabled) {
#if GAZEBO_MAJOR_VERSION >= 7
    joint->SetEffortLimit(0, read_motor_state.torque_limit);
#else
    joint->SetMaxForce(0, read_motor_state.torque_limit);
#endif
  } else {
#if GAZEBO_MAJOR_VERSION >= 7
    joint->SetEffortLimit(0, 0);
#else
    joint->SetMaxForce(0, 0);
#endif
  }

  joint->SetVelocity(0, target_velocity);
}

void GazeboRosDynamixelMotor::OnWorldUpdate()
{
  static int i = 0;

  current_motor_state = ReadMotor();

  MsgType joint_state_msg = createJointStateMsg(motor_name, current_motor_state);
  dynamixel_joint_state_publisher.publish<MsgType>(joint_state_msg);

  sensor_msgs::JointState state_msg;
  state_msg.header.seq = i++;
  state_msg.header.frame_id = "dynamixel_frame";
  state_msg.header.stamp = ros::Time::now();
  state_msg.effort.push_back(current_motor_state.load);
  state_msg.name.push_back(joint_name);
  state_msg.position.push_back(current_motor_state.current_pos_rad);
  state_msg.velocity.push_back(current_motor_state.velocity_rad_s);
  dynamixel_ros_joint_state_publisher.publish<sensor_msgs::JointState>(state_msg);

  UpdateMotor(current_motor_state);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosDynamixelMotor)
}
