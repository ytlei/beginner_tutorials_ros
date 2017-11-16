# ROS Homework_11

## Overview 

This ROS program creates a means of communication by setting up two types of nodes, a publisher and subscriber node. The publisher or "talker" node will send information that will be received and viewed by the subscriber or "listener" node. Updated with a service and launch file. 

## Build

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ git clone --recursive https://github.com/ytlei/beginner_tutorials_ros/edit -b Week11_HW
$ cd ~/catkin_ws/
$ catkin_make
```

## Run Steps


To execute the program, first build the program by running 

	catkin_make
	
within the root directory. 

Next, open up a terminal and run

	roscore

Open a new terminal and run the talker with argument for frequency set to 1

	source ./devel/setup.bash
	rosrun beginner_tutorials talker 1
	
Another terminal must be opened to run the listener code

	source ./devel/setup.bash
	rosrun beginner_tutorials listener

## Run - Call Service

Make sure talker & listener nodes are already running

Open a new terminal to see if talkerService is available

```bash
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosservice list
```

Call talkerService to update name on chatter topic

```bash
rosservice call /talkerService NewName
```
	
## Run - roslaunch

Use roslaunch to run talker and listener nodes together and set publish freq on chatter topic

Open a new terminal to make sure roscore is running:

```bash
$ roscore
```
Open a new terminal to run launch file:

```bash
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ roslaunch beginner_tutorials talkerandlistener.launch freq:=1
```

## Run - Call Service

Make sure talker & listener nodes are already running

Open a new terminal to check if talkerService is available

```bash
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosservice list
```
you sould see all the service running

Call talkerService to update name on chatter topic

```bash
rosservice call /talkerService John
```

On Talker node terminal, the output should update the name
On Listener node terminal, the name should also be updated


## TF

Talker node broadcasts a TF frame /talker with reference to /world frame.  

To inspect TF frame published by /talker:

In a new terminal
```bash
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials talker
```

In another terminal

```bash
cd ~/catkin_ws
source ./devel/setup.bash
rosrun tf tf_echo /world /talker
```

you should see the Translation and Rotation showing on screen

TF tree can be viewed using

rqt_tf_tree

```bash
rosrun rqt_tf_tree rqt_tf_tree
```

## Test - rostest

Level 2 integration test implemented by gtest can be run on talker service as follow:

```bash
cd ~/catkin_ws
catkin_make tests
source ./devel/setup.bash
rostest beginner_tutorials testTalker.launch
```

## Record/Play - rosbag

rosbag is included in talkerandlistener.launch to record all topics.

To enable recording:

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch beginner_tutorials tutorial.launch enable_record:=true freq:=1
```

To play rosbag recording:

In a new terminal
```bash
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials listener
```

Open another terminal
```bash
cd ~/.ros/
rosbag play result.bag
```

Listner terminal should output messages recorded by rosbag


## Dependencies

Requires ROS kinetic and Linux OS.

## License

Copyright 2017 Yi-ting Lei

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

