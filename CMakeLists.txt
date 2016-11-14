cmake_minimum_required(VERSION 2.8.3)
project(pr2_cartesian_controllers)

find_package(
catkin REQUIRED COMPONENTS
  roscpp
	std_msgs
	geometry_msgs
  visualization_msgs
	sensor_msgs
	actionlib_msgs
	actionlib
	kdl_parser
  kdl_conversions
	message_generation
)

link_directories(${catkin_LIBRARY_DIRS})

add_action_files(
	DIRECTORY action
	FILES ManipulationController.action GuardedApproach.action Move.action
)

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs actionlib_msgs visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs actionlib_msgs actionlib kdl_parser visualization_msgs
  INCLUDE_DIRS include
  LIBRARIES manipulation_controller approach_controller move_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(controller_template src/controller_template.cpp)
target_link_libraries(controller_template ${catkin_LIBRARIES})
add_dependencies(controller_template pr2_cartesian_controllers_generate_messages_cpp)

add_library(manipulation_controller src/manipulation_controller.cpp)
target_link_libraries(manipulation_controller ${catkin_LIBRARIES})
add_dependencies(manipulation_controller pr2_cartesian_controllers_generate_messages_cpp)

# add_library(approach_controller src/approach_controller.cpp)
# target_link_libraries(approach_controller ${catkin_LIBRARIES})
# add_dependencies(approach_controller pr2_cartesian_controllers_generate_messages_cpp)

add_library(move_controller src/move_controller.cpp)
target_link_libraries(move_controller ${catkin_LIBRARIES})
add_dependencies(move_controller pr2_cartesian_controllers_generate_messages_cpp)
