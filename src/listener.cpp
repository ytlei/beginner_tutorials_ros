/********************************************************************
 *   MIT License
 *
 *   Copyright (c) 2017 Yi-ting Lei
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 ********************************************************************/

/**
 *  @file listener.cpp
 *  @brief File of main function for listener node
 *
 *  This file contains the main program of listener node subscribing to chatter topic.
 *
 *  This program demonstrate the concept of subscribing topic in ROS.
 *
 *
 *  @author Yi-ting Lei
 *  @date   11/07/2017
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

	ros::NodeHandle n;

	// Register client with the master
	ros::ServiceClient client = n.serviceClient
			< beginner_tutorials::talkerService > ("talkerService");

	// Wait for service
	timeoutCounter = ros::service::waitForService("talkerService", 60000);
	// timeout after 60 seconds
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
