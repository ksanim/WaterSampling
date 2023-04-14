cmake_minimum_required(VERSION 3.0.2)
project(watersampling_msgs)

################
# Dependencies #
################
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
  FILES
  MS5837Stamped.msg
  ConductivityStamped.msg
  PumpInfo.msg
  EKFInfo.msg
  SonarStamped.msg
  Setpoint.msg
  RTC_time.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_generation message_runtime std_msgs
)

