/*
 * XsensMTROSTriggeringInterface.h
 *
 *  Created on: Sep 13, 2018
 *      Author: jelavice
 */

#ifndef XSENS_MT_ROS_TRIGGERING_INTERFACE_H
#define XSENS_MT_ROS_TRIGGERING_INTERFACE_H

#include "xsensmt_ros/XsensMTROSInterface.h"
#include <xsensmt_interface/ImuMeasurement.h>
#include <xsensmt_interface/ImuConfig.h>
#include <xsensmt_interface/ImuStatus.h>
#include <xsensmt_interface/XsensMTInterface.h>

#include <any_node/Node.hpp>

namespace xsensmt {

class XsensMTROSTriggeringInterface: public XsensMTROSInterface {

public:

	using Base = XsensMTROSInterface;

	XsensMTROSTriggeringInterface(any_node::Node::NodeHandlePtr nh) :
			Base(nh) {
	}

	virtual ~XsensMTROSTriggeringInterface() = default;

	bool update(const any_worker::WorkerEvent& event) override final; //final for speed

private:
	void advertiseTopic() override final; // final for speed
	void getParameters() override final;

};

} /* namespace */

#endif /* XSENS_MT_ROS_TRIGGERING_INTERFACE_H */
