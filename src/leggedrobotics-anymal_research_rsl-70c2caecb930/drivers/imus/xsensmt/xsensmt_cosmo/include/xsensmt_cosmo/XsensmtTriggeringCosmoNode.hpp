/*
 * XsensmtTriggeringCosmoNode.hpp
 *
 *  Created on: Sep 24, 2018
 *      Author: jelavice
 */

#pragma once

#include "xsensmt_cosmo/XsensmtCosmoNode.hpp"
#include "any_msgs/ImuWithTrigger.h"
#include "any_measurements/ImuWithTrigger.hpp"

using namespace any_node;

namespace xsensmt {

class XsensmtTriggeringCosmoNode: public XsensmtCosmoNode {

public:
	using Base = XsensmtCosmoNode;
	using ImuShm = any_measurements::ImuWithTrigger;
	using ImuRos = any_msgs::ImuWithTrigger;
	using ImuPublisherPtr = cosmo_ros::PublisherRosPtr<ImuShm, ImuRos, any_measurements_ros::ConversionTraits>;

	/* constructor needs to take a shared_ptr to a ros::Nodehandle instance. */
	XsensmtTriggeringCosmoNode() = delete;
	XsensmtTriggeringCosmoNode(any_node::Node::NodeHandlePtr nh);

	bool update(const any_worker::WorkerEvent& event) override final;

	/* destructor */
	~XsensmtTriggeringCosmoNode() override = default;

private:

	bool advertiseTopic() override final;
	void getParameters() override final;

	ImuPublisherPtr imuPublisher_;
	ImuShm shmMessage_;
	ImuRos rosMessage_;

};

} /* namespace */
