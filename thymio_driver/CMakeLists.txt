cmake_minimum_required(VERSION 2.8.3)
project(thymio_driver)

find_package(catkin REQUIRED COMPONENTS
	rospy
	asebaros
	std_msgs
	geometry_msgs
	sensor_msgs
	nav_msgs
	message_generation
	tf
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Led.msg
  LedGesture.msg
  Sound.msg
  SystemSound.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS rospy asebaros std_msgs geometry_msgs tf sensor_msgs nav_msgs message_runtime
)

###########
## Build ##
###########


#############
## Install ##
#############

