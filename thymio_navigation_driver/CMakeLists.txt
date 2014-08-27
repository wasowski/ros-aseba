cmake_minimum_required(VERSION 2.8.3)
project(thymio_navigation_driver)

find_package(catkin REQUIRED COMPONENTS
	rospy
	asebaros
	geometry_msgs
	tf
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS rospy asebaros geometry_msgs tf
)

###########
## Build ##
###########


#############
## Install ##
#############

