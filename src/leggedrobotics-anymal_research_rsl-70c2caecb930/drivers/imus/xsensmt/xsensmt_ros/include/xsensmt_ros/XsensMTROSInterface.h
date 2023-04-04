/** \file: XsensMTROSInterfaceNode.h
    \brief: ROS interface node for driver configuration & diagnosing
  */

#ifndef XSENS_MT_ROS_INTERFACE_H
#define XSENS_MT_ROS_INTERFACE_H

#include <memory>
#include <string>
#include <utility>
#include <cstdint>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <any_msgs/SensorTimeInfo.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <xsensmt_interface/ImuMeasurement.h>
#include <xsensmt_interface/ImuConfig.h>
#include <xsensmt_interface/ImuStatus.h>
#include <xsensmt_interface/XsensMTInterface.h>

#include <any_node/Node.hpp>
#include <message_logger/message_logger.hpp>

using namespace any_node;

namespace xsensmt {

    class XsensMTROSInterface: public any_node::Node {

        public:
            /* constructor needs to take a shared_ptr to a ros::Nodehandle instance. */
            XsensMTROSInterface() = delete; 
            XsensMTROSInterface(any_node::Node::NodeHandlePtr nh): any_node::Node(nh) { }

            /* destructor */
            ~XsensMTROSInterface() override = default;

            /// initialize node
            bool init() override;
            /// safely stop node and quit
            void cleanup() override;
            /// called on every update step of the node
            virtual bool update(const any_worker::WorkerEvent& event);

        protected:
            /// get parameters from ROS param server
            virtual void getParameters();

            void getBasicParameters();

            /// xsensMTInterface object
            XsensMTInterface xsensMTSensor_;

            ImuConfig configParam_;
            std::string frameId_;
            ros::Timer timer_;
            ros::Publisher imuPublisher_;
            ros::Publisher imuCounterPublisher_;
            int ROSQueueDepth_;
            bool useRosTime_;

        private:
            virtual void advertiseTopic();
  
    };

}


#endif

