cmake_minimum_required(VERSION 2.8.3)
project(pr2_algorithms)

find_package(
catkin REQUIRED COMPONENTS
  roscpp
	std_msgs
	message_generation
  eigen_conversions
)

link_directories(${catkin_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

add_message_files(
  FILES
	TestBedFeedback.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_generation
  INCLUDE_DIRS include
  LIBRARIES manipulation_control_algorithm manipulation_ekf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(manipulation_control_algorithm src/manipulation_control_algorithm.cpp src/algorithm_base.cpp)
target_link_libraries(manipulation_control_algorithm ${catkin_LIBRARIES})

add_library(manipulation_ekf src/manipulation_ekf.cpp src/algorithm_base.cpp)
target_link_libraries(manipulation_ekf ${catkin_LIBRARIES})

add_executable(test_bed src/test_bed.cpp)
target_link_libraries(test_bed manipulation_ekf manipulation_control_algorithm ${catkin_LIBRARIES})
add_dependencies(test_bed pr2_algorithms_generate_messages_cpp)
