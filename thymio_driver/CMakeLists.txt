cmake_minimum_required(VERSION 2.8.3)
project(thymio_driver)

find_package(catkin REQUIRED COMPONENTS
	rospy
	asebaros
	asebaros_msgs
	std_msgs
	geometry_msgs
	sensor_msgs
	nav_msgs
	message_generation
	tf
	thymio_msgs
)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS rospy asebaros asebaros_msgs std_msgs geometry_msgs tf sensor_msgs nav_msgs message_runtime thymio_msgs
)

###########
## Build ##
###########


#############
## Install ##
#############

