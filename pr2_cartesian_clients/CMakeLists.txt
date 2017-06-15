cmake_minimum_required(VERSION 2.8.3)
project(pr2_cartesian_clients)

find_package(
catkin REQUIRED COMPONENTS
  roscpp
  rosbag
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
set(CMAKE_CXX_FLAGS "-std=c++0x -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ${CMAKE_CXX_FLAGS}")

# add_message_files(
#   FILES
# )

add_service_files(
  FILES
  LogMessages.srv
)

add_action_files(
	DIRECTORY action
	FILES Manipulation.action Folding.action Mechanism.action
)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs actionlib_msgs actionlib pr2_cartesian_controllers rosbag
  INCLUDE_DIRS include
  LIBRARIES exclusive_controller_runner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(exclusive_controller_runner src/utils/ExclusiveControllerRunner.cpp src/utils/extra.cpp)
target_link_libraries(exclusive_controller_runner ${catkin_LIBRARIES})

add_executable(manipulation_client src/clients/manipulation_client.cpp src/utils/extra.cpp)
target_link_libraries(manipulation_client exclusive_controller_runner ${catkin_LIBRARIES})

add_executable(folding_client src/clients/folding_client.cpp src/utils/extra.cpp)
target_link_libraries(folding_client exclusive_controller_runner ${catkin_LIBRARIES})

add_executable(mechanism_client src/clients/mechanism_client.cpp src/utils/extra.cpp)
target_link_libraries(mechanism_client exclusive_controller_runner ${catkin_LIBRARIES})