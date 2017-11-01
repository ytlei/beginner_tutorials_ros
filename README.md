# This is a ROS Publisher/Subscriber demo

## How to build
To build and run follow the steps:
1. go to your workspace
$ cd ~/catkin_ws/
2.$ catkin_make

## How to run
Make sure that a roscore is up and running:
$ roscore
catkin specific If you are using catkin, make sure you have sourced your workspace's setup.sh file after calling catkin_make but before trying to use your applications:

In your catkin workspace

$ cd ~/catkin_ws

$ source ./devel/setup.bash

Run the publisher called "talker"

$ rosrun beginner_tutorials talker

Run the Subscriber in other terminal

$ rosrun beginner_tutorials listener 
