/************************************************************
 *                                                          *
 * Copyright (C) 2017 by Yi-ting Lei                        *
 *                                                          *
 ***********************************************************/

/**
 *   @file	listener.cpp
 *   @brief  	demo for ROS listener node
 *
 *   @author	Yi-ting Lei
 *   @date	2017/10/30
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "listenerClass.hpp"
#include "beginner_tutorials/talkerService.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
	bool timeoutCounter;
	ros::init(argc, argv, "listener");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	// Register client with the master
	ros::ServiceClient client = n.serviceClient
			< beginner_tutorials::talkerService > ("talkerService");

	// Wait for service
	timeoutCounter = ros::service::waitForService("talkerService", 60000);
	//timeout after 60 seconds
	if (timeoutCounter == false) {
		ROS_WARN_STREAM("Waiting for talkerService timeout!");
	} else {
		// Create request & response objects
		beginner_tutorials::talkerService::Request req;
		beginner_tutorials::talkerService::Response resp;

		req.name = "ytlei";

		// Call service
		timeoutCounter = client.call(req, resp);

		// Check service
		if (timeoutCounter == true) {
			ROS_INFO_STREAM("Set talker name " << resp.resp);
		} else {
			ROS_ERROR_STREAM("Failed to set talker name");
		}
	}

	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

	if (sub) {
		ROS_DEBUG_STREAM("Subscribing done");
	} else {
		ROS_FATAL_STREAM("Subscribing FAILED.  End listener.");
		return -1;
	}

	ros::spin();

	return 0;
}
