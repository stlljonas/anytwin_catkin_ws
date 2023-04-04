/*
 * xsensmt_cosmo_node.cpp
 *
 *  Created on: Jun 27, 2017
 *      Author: Christian Gehring
 */

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "any_msgs/ImuWithTrigger.h"

#include <any_node/any_node.hpp>
#include <iostream>
#include "any_node/Node.hpp"
#include "cosmo_ros/cosmo_ros.hpp"
#include "any_measurements/Imu.hpp"
#include "any_measurements/ImuWithTrigger.hpp"
#include "any_measurements_ros/any_measurements_ros.hpp"

namespace xsensmt {

class CosmoTestNode: public any_node::Node {
public:

//	using ImuShm = any_measurements::Imu;
//	using ImuRos = sensor_msgs::Imu;
	using ImuShm = any_measurements::ImuWithTrigger;
	using ImuRos = any_msgs::ImuWithTrigger;

	using Base = any_node::Node;

	// constructor/destructor
	CosmoTestNode() = delete;
	CosmoTestNode(NodeHandlePtr nh);

	virtual ~CosmoTestNode() = default;

	// virtual functions from NodeImpl
	bool init() override;
	void cleanup() override;

	void callback(const ImuShm& msg);

protected:

	cosmo_ros::SubscriberRosPtr<ImuShm, ImuRos,
			any_measurements_ros::ConversionTraits> sub_;

	ImuShm shmMessage_;
	ImuRos rosMessage_;

};

} /* namespace */

int main(int argc, char **argv) {
	any_node::Nodewrap<xsensmt::CosmoTestNode> node(argc, argv, "xsensmt_subscriber", 1);
	return static_cast<int>(!node.execute());
}

xsensmt::CosmoTestNode::CosmoTestNode(NodeHandlePtr nh) :
		Base(nh) {

}

bool xsensmt::CosmoTestNode::init() {

	using namespace cosmo_ros;

	auto subOptions = std::make_shared < cosmo_ros::SubscriberRosOptions
			< ImuShm
					>> ("/sensors/imu", std::bind(&CosmoTestNode::callback, this,
							std::placeholders::_1));
	subOptions->autoSubscribe_ = true;
	sub_ = subscribeShmRos<ImuShm, ImuRos,
			any_measurements_ros::ConversionTraits>(subOptions);

	return true;
}

void xsensmt::CosmoTestNode::cleanup() {

}

void xsensmt::CosmoTestNode::callback(const ImuShm &msg) {
	MELO_INFO("Received the imu message. ")
	std::cout << msg << std::endl << std::endl;
}
