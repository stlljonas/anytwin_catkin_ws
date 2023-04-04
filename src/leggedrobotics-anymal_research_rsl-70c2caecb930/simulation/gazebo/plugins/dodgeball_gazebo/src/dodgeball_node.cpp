/*
 * dodgeball_node.cpp
 *
 *  Created on: Aug 30, 2015
 *      Author: Christian Gehring
 */


// ros
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

// gazebo
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/GetLinkState.h>

// kindr
#include <kindr/Core>

// GL
#include <GL/glut.h>


typedef kindr::RotationQuaternionPD RotationQuaternion;
typedef kindr::EulerAnglesZyxPD EulerAnglesZyx;
typedef kindr::LocalAngularVelocityPD LocalAngularVelocity;
typedef kindr::AngleAxisPD AngleAxis;
typedef kindr::Position3D Position;
typedef kindr::Velocity3D LinearVelocity;
typedef kindr::VectorTypeless3D Vector;

ros::ServiceClient clientTarget;
ros::ServiceClient clientSetLink;
std::string targetRobot;
std::string targetLink;


double getClickedHeading(int x, int y, const Position& reference)
{
  //get the modelview matrix
  double modelviewMat[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, modelviewMat);

  //and get the projection matrix
  double projectionMat[16];
  glGetDoublev(GL_PROJECTION_MATRIX, projectionMat);

  //and the viewport info
  int viewport[4];
  glGetIntegerv(GL_VIEWPORT, viewport);

  Position p1, p2;
  gluUnProject((double)x, viewport[3] - y - 1, 0, modelviewMat, projectionMat, viewport, &p1.x(), &p1.y(), &p1.z());
  gluUnProject((double)x, viewport[3] - y - 1, 1, modelviewMat, projectionMat, viewport, &p2.x(), &p2.y(), &p2.z());

  //ok, these two points define the ray that we should intersect with the ground plane in order to get the new position
  Position v = p2 - p1;
    double t = -p1.z()/v.z();
  if (t > 0){
    Position p = reference;

    v.x() = p1.x() + v.x() * t - p.x();
    v.z() = p1.z() + v.z() * t - p.z();
    v.y() = p1.y() + v.y() * t - p.y();
    v.z() = 0;

    v *= v.norm();

    return -atan2(v.x(),v.y());
  }
  return 100;
}

void getBallState(Position& positionBall,
                  LinearVelocity& linearVelocityBall,
                  const Position& posititionTarget,
                  const LinearVelocity& linearVelocityTarget,
                  int windowWidth = 800,
                  int windowHeight = 600)
{
  Vector axis(0.0, 0.0, 1.0);
  Vector dir(0.0,-1.0, 0.0);
  const double dodgeBallHeight = 0.9;


  /* simulate random clicks on window */
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> distWidth(0, windowWidth);
  std::uniform_real_distribution<double> distHeight(0, windowHeight);
  double mouseX = distWidth(mt);
  double mouseY = distHeight(mt);

  dir = AngleAxis(getClickedHeading(mouseX, mouseY, posititionTarget), axis.toImplementation()).rotate(dir);

  positionBall = posititionTarget + Position(dir * -2.0) + Position(linearVelocityTarget * 0.5);
  positionBall.z() = dodgeBallHeight;

   linearVelocityBall = LinearVelocity(dir * 6.0);
}

bool throwBall(std_srvs::Empty::Request  &req,
               std_srvs::Empty::Response &res)
{
  gazebo_msgs::GetLinkState srvTarget;
  srvTarget.request.link_name = targetRobot + "::" + targetLink;
  if (clientTarget.call(srvTarget))
  {
    if (!srvTarget.response.success) {
      ROS_ERROR_STREAM(srvTarget.response.status_message);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  Position positionBall;
  LinearVelocity linearVelocityBall;
  Position posititionTarget(srvTarget.response.link_state.pose.position.x, srvTarget.response.link_state.pose.position.y, srvTarget.response.link_state.pose.position.z );
  LinearVelocity linearVelocityTarget(srvTarget.response.link_state.twist.linear.x, srvTarget.response.link_state.twist.linear.y, srvTarget.response.link_state.twist.linear.z);

  getBallState(positionBall, linearVelocityBall, posititionTarget, linearVelocityTarget);

  gazebo_msgs::SetLinkState srv;
  srv.request.link_state.link_name = "dodgeball::dodgeball";
  srv.request.link_state.pose.position.x = positionBall.x();
  srv.request.link_state.pose.position.y = positionBall.y();
  srv.request.link_state.pose.position.z = positionBall.z();
  srv.request.link_state.twist.linear.x = linearVelocityBall.x();
  srv.request.link_state.twist.linear.y = linearVelocityBall.y();
  srv.request.link_state.twist.linear.z = linearVelocityBall.z();

  if (clientSetLink.call(srv))
  {
    if (!srv.response.success) {
      ROS_ERROR_STREAM(srv.response.status_message);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dodgeball");
  ros::NodeHandle nodeHandle("~");

  nodeHandle.param<std::string>(std::string{"target_robot"}, targetRobot, std::string{"anymal"});
  nodeHandle.param<std::string>(std::string{"target_link"}, targetLink, std::string{"base"});
  double radius;
  nodeHandle.param<double>(std::string{"radius"}, radius, 0.2);


  ros::ServiceServer service = nodeHandle.advertiseService("throw", throwBall);

  clientTarget = nodeHandle.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  clientSetLink = nodeHandle.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");

  std::cout << "[dodgeball] Waiting for Gazebo service to be set up... " << std::endl;
  if (clientTarget.waitForExistence(ros::Duration(5.0))) {
    std::cout << "[dodgeball] Found service!" << std::endl;
  } else {
    std::cout << "[dodgeball] Timeout will waiting. Exiting dodgeball node..." << std::endl;
    return 0;
  }


  ros::Publisher markerPublisher = nodeHandle.advertise<visualization_msgs::Marker>("/dodgeball_marker", 10);
//  ros::Publisher posePublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("/dodgeball/pose_in_odom", 100);

  visualization_msgs::Marker markerMsg;
  markerMsg.color.a = 1.0;
  markerMsg.color.r = 1.0;
  markerMsg.action = visualization_msgs::Marker::ADD;
  markerMsg.type = visualization_msgs::Marker::SPHERE;
  markerMsg.scale.x = radius;
  markerMsg.scale.y = radius;
  markerMsg.scale.z = radius;
  markerMsg.id = 1;
  markerMsg.header.frame_id = "odom";
  markerMsg.ns = "sphere";

//  geometry_msgs::PoseStamped poseMsg;
//  poseMsg.header.frame_id = "odom";
//  poseMsg.header.stamp = ros::Time::now();

  ros::Rate rate(25);

  while (ros::ok()) {
    gazebo_msgs::GetLinkState srvTarget;
    srvTarget.request.link_name = "dodgeball::dodgeball";
//    srvTarget.request.reference_frame = "map";
    if (clientTarget.call(srvTarget))
    {
      if (!srvTarget.response.success) {
        ROS_ERROR_STREAM(srvTarget.response.status_message);
      }
    }
    else
    {
      ROS_ERROR("Failed to call service");
      return 1;
    }

    const ros::Time stamp = ros::Time::now();

    markerMsg.header.stamp = stamp;
    markerMsg.pose = srvTarget.response.link_state.pose;

//    poseMsg.header.stamp = stamp;
//    poseMsg.pose = srvTarget.response.link_state.pose;

    markerPublisher.publish(markerMsg);
//    posePublisher.publish(poseMsg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

