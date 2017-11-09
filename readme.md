# ROS Tutorial Homework

## Overview 

This ROS program creates a means of communication by setting up two types of nodes, a publisher and subscriber node. The publisher or "talker" node will send information that will be received and viewed by the subscriber or "listener" node. Updated with a service and launch file. 

## Build/Run Steps

To execute the program, first build the program by running 

	catkin_make
	
within the root directory. 

Next, open up a terminal and run

	roscore

Open a new terminal and run the talker code

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

