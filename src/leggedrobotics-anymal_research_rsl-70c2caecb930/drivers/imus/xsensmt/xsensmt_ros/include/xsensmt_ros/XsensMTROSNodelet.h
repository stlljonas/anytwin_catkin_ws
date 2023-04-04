/*!
 * @file	XsensMTROSNodelet.h
 * @author	Russell Buchanan
 * @date	February, 2019
 */

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <cstdint>
#include <chrono>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Imu.h>
#include <any_msgs/SensorTimeInfo.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <xsensmt_interface/ImuMeasurement.h>
#include <xsensmt_interface/ImuConfig.h>
#include <xsensmt_interface/ImuStatus.h>
#include <xsensmt_interface/XsensMTInterface.h>

#include <any_node/Param.hpp>
#include <message_logger/message_logger.hpp>

using namespace param_io;

namespace xsensmt
{

class XsensMTROSNodelet : public nodelet::Nodelet
{
public:
    XsensMTROSNodelet();
    ~XsensMTROSNodelet();

    virtual void onInit();

protected:
        void getParameters();
        void measurementUpdate();
        void advertiseTopic();

        ros::NodeHandle nh_;

        // Thread pointer
        std::thread* deviceThread_ = nullptr;

        /// xsensMTInterface object
        XsensMTInterface xsensMTSensor_;

        /// xsensMTInterface config
        ImuConfig configParam_;

        // ROS publishers
        ros::Publisher imuPublisher_;
        ros::Publisher imuTimeInfoPublisher_;

        // ROS params
        std::string frameId_;
        int ROSQueueDepth_;
        bool useRosTime_;
        double timeStep_;

};
} // namespace xsensmt
