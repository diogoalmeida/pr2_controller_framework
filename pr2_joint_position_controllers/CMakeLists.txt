cmake_minimum_required(VERSION 2.8.3)
project(pr2_joint_position_controllers)

find_package(
catkin REQUIRED COMPONENTS
  roscpp
	std_msgs
  geometry_msgs
	sensor_msgs
	message_generation
  actionlib
  actionlib_msgs
  pr2_controller_interface
  pr2_mechanism_model
	pr2_cartesian_controllers
  pluginlib
)

link_directories(${catkin_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_message_files(
  FILES
	PIDGains.msg
	PR2JointControllerFeedback.msg
)

add_action_files(
	DIRECTORY action
	FILES PR2TuneJoint.action
)

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs pr2_controller_interface pr2_mechanism_model pluginlib pr2_cartesian_controllers actionlib actionlib_msgs
  INCLUDE_DIRS include
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pr2_manipulation_joint_controller src/manipulation_joint_controller.cpp src/template_joint_controller.cpp)
target_link_libraries(pr2_manipulation_joint_controller ${catkin_LIBRARIES})
add_dependencies(pr2_manipulation_joint_controller pr2_joint_position_controllers_generate_messages_cpp)

add_library(pr2_approach_joint_controller src/approach_joint_controller.cpp src/template_joint_controller.cpp)
target_link_libraries(pr2_approach_joint_controller ${catkin_LIBRARIES})
add_dependencies(pr2_approach_joint_controller pr2_joint_position_controllers_generate_messages_cpp)

add_library(pr2_move_joint_controller src/move_joint_controller.cpp src/template_joint_controller.cpp)
target_link_libraries(pr2_move_joint_controller ${catkin_LIBRARIES})
add_dependencies(pr2_move_joint_controller pr2_joint_position_controllers_generate_messages_cpp)

add_library(pr2_tune_joint_controller src/tune_joint_controller.cpp)
target_link_libraries(pr2_tune_joint_controller ${catkin_LIBRARIES})
add_dependencies(pr2_tune_joint_controller pr2_joint_position_controllers_generate_messages_cpp)
