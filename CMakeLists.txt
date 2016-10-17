cmake_minimum_required(VERSION 2.8.3)
project(pr2_controller)

find_package(
catkin REQUIRED COMPONENTS
  roscpp
	std_msgs
  geometry_msgs
	sensor_msgs
	actionlib_msgs
	message_generation
  pr2_controller_interface
  pr2_mechanism_model
  pluginlib
)

link_directories(${catkin_LIBRARY_DIRS})

add_action_files(
  DIRECTORY action
  FILES PR2JointCommand.action
)

# add_message_files(
#   FILES
# )

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs pr2_controller_interface pr2_mechanism_model pluginlib
  INCLUDE_DIRS include
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${Eigen_INCLUDE_DIRS}
)

add_library(pr2_joint_controller src/joint_controller.cpp)
target_link_libraries(pr2_joint_controller ${catkin_LIBRARIES})# ${Eigen_LIBRARIES})
