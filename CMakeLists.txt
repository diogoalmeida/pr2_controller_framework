cmake_minimum_required(VERSION 2.8.3)
project(manipulation_client)

find_package(
catkin REQUIRED COMPONENTS
  roscpp
	std_msgs
	geometry_msgs
	sensor_msgs
	actionlib_msgs
	actionlib
  manipulation_controller
	message_generation
)

link_directories(${catkin_LIBRARY_DIRS})

# add_message_files(
#   FILES
# )

# add_action_files(
# 	DIRECTORY action
# 	FILES ManipulationController.action
# )

# generate_messages(
#   DEPENDENCIES std_msgs sensor_msgs actionlib_msgs
# )

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs actionlib_msgs actionlib manipulation_controller
  INCLUDE_DIRS include
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(manipulation_client src/manipulation_client.cpp)
target_link_libraries(manipulation_client ${catkin_LIBRARIES})
