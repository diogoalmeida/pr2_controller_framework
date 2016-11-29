cmake_minimum_required(VERSION 2.8.3)
project(pr2_cartesian_clients)

find_package(
catkin REQUIRED COMPONENTS
  roscpp
	std_msgs
	geometry_msgs
	sensor_msgs
	actionlib_msgs
	actionlib
  tf
  eigen_conversions
  pr2_cartesian_controllers
	pr2_mechanism_msgs
	message_generation
)

link_directories(${catkin_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# add_message_files(
#   FILES
# )

add_action_files(
	DIRECTORY action
	FILES Manipulation.action
)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs actionlib_msgs actionlib pr2_cartesian_controllers
  INCLUDE_DIRS include
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(manipulation_client src/manipulation_client.cpp)
target_link_libraries(manipulation_client ${catkin_LIBRARIES})
