/************************************************************
 *                                                          *
 * Copyright (C) 2017 by Yi-ting Lei                        *
 *                                                          *
 ***********************************************************/

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


int main(int argc, char **argv) {
  // default freq: 10 Hz
  int freq = 10;

  ros::init(argc, argv, "talker");
  // get freq from input argument
  if (argc > 1) {
        ROS_DEBUG_STREAM("argv is " << argv[1]);
        freq = atoi(argv[1]);
  }

  ROS_INFO_STREAM("set freqency to " << freq << "Hz");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  Talker talker;
  // Register service
  ros::ServiceServer server = n.advertiseService("talkerService", &Talker::updateTalkerName, &talker);


  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world by ytlei" << count << " talk:" << talker.getName();
    msg.data = ss.str();
    ROS_DEBUG_STREAM("Publish: " << msg.data.c_str());

    ROS_INFO("%s", msg.data.c_str());

   
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
