cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)
add_service_files(
 FILES
 talkerService.srv
)
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

## Add talker
add_executable(talker src/talker.cpp src/talkerClass.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

## Add listener
add_executable(listener src/listener.cpp src/listenerClass.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})


add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_dependencies(listener beginner_tutorials_generate_messages_cpp)


