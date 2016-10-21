cmake_minimum_required(VERSION 2.8.3)
project(manipulation_controller)

find_package(
catkin REQUIRED COMPONENTS
  roscpp
	std_msgs
  geometry_msgs
	sensor_msgs
	actionlib_msgs
	actionlib
	message_generation
)

link_directories(${catkin_LIBRARY_DIRS})

# add_message_files(
#   FILES
# )

add_action_files(
	DIRECTORY action
	FILES ManipulationController.action
)

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs actionlib_msgs actionlib
  INCLUDE_DIRS include
  LIBRARIES manipulation_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(manipulation_controller src/manipulation_controller.cpp)
target_link_libraries(manipulation_controller ${catkin_LIBRARIES})
add_dependencies(manipulation_controller pr2_controller_generate_messages_cpp)
