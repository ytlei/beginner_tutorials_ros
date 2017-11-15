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
 *   @file	talker.cpp
 *   @brief  	this is the talker for ROS tutorial
 *   
 *
 *   @author	Yi-ting Lei
 *   @date	2017/10/31
 */
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "talkerClass.hpp"
#include "beginner_tutorials/talkerService.h"
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv) {
	// default freq: 10 Hz
	int freq = 10;

	ros::init(argc, argv, "talker");
	// set freq from input argument
	if (argc > 1) {
		ROS_DEBUG_STREAM("argv is " << argv[1]);
		freq = atoi(argv[1]);
		if (freq < 0) {
			ROS_ERROR_STREAM("The Input Frequency is negative");
		}
	}

	ROS_WARN_STREAM("set freqency to " << freq << "Hz");

	ros::NodeHandle n;

	talkerClass talker;
	// Register service
	ros::ServiceServer server = n.advertiseService("talkerService",
			&talkerClass::updateTalkerName, &talker);

	// TF broadcast
	static tf::TransformBroadcaster br;
	tf::Transform transform;

	ros::Publisher chatter_pub = n.advertise < std_msgs::String
			> ("chatter", 1000);
	ros::Rate loop_rate(freq);

	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	int count = 0;
	while (ros::ok()) {
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world by: " << talker.getName() << " " << count;
		msg.data = ss.str();
		ROS_DEBUG_STREAM("Publish: " << msg.data.c_str());

		ROS_INFO("%s", msg.data.c_str());

		chatter_pub.publish(msg);

		// broadcast the TF frame via /talker
		transform.setOrigin(
				tf::Vector3(2.0 * sin(ros::Time::now().toSec()),
						2.0 * cos(ros::Time::now().toSec()), 0.0));

		transform.setRotation(tf::Quaternion(1, 0, 0, 0));
		br.sendTransform(
				tf::StampedTransform(transform, ros::Time::now(), "world",
						"talker"));

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}
