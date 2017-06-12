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
	moveit_msgs
	actionlib
	kdl_parser
  kdl_conversions
	message_generation
  pr2_algorithms
)

link_directories(${catkin_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "-std=c++0x -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ${CMAKE_CXX_FLAGS}")

add_action_files(
	DIRECTORY action
	FILES ManipulationController.action GuardedApproach.action Move.action FoldingController.action MechanismIdentification.action
)

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs actionlib_msgs visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs actionlib_msgs actionlib kdl_parser visualization_msgs moveit_msgs pr2_algorithms
  INCLUDE_DIRS include
  LIBRARIES manipulation_controller approach_controller move_controller folding_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(utils
    src/utils/TwistController.cpp
)

add_library(manipulation_controller src/pr2_cartesian_controllers/manipulation_controller.cpp ${utils})
target_link_libraries(manipulation_controller ${catkin_LIBRARIES})
add_dependencies(manipulation_controller pr2_cartesian_controllers_generate_messages_cpp)

add_library(approach_controller src/pr2_cartesian_controllers/approach_controller.cpp ${utils})
target_link_libraries(approach_controller ${catkin_LIBRARIES})
add_dependencies(approach_controller pr2_cartesian_controllers_generate_messages_cpp)

add_library(move_controller src/pr2_cartesian_controllers/move_controller.cpp ${utils})
target_link_libraries(move_controller ${catkin_LIBRARIES})
add_dependencies(move_controller pr2_cartesian_controllers_generate_messages_cpp)

add_library(folding_controller src/pr2_cartesian_controllers/folding_controller.cpp ${utils})
target_link_libraries(folding_controller ${catkin_LIBRARIES})
add_dependencies(folding_controller pr2_cartesian_controllers_generate_messages_cpp)
