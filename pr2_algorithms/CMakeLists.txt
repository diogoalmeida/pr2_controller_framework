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
  TestBedManipulationFeedback.msg
	TestBedFoldingFeedback.msg
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_generation
  INCLUDE_DIRS include
  LIBRARIES manipulation_control_algorithm manipulation_ekf folding_assembly_controller folding_assembly_estimator
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(manipulation_control_algorithm src/dexterous_manipulation/manipulation_control_algorithm.cpp src/algorithm_base.cpp)
target_link_libraries(manipulation_control_algorithm ${catkin_LIBRARIES})

add_library(manipulation_ekf src/dexterous_manipulation/manipulation_ekf.cpp src/algorithm_base.cpp)
target_link_libraries(manipulation_ekf ${catkin_LIBRARIES})

add_library(folding_assembly_controller src/folding_assembly/folding_assembly_controller.cpp src/algorithm_base.cpp)
target_link_libraries(folding_assembly_controller ${catkin_LIBRARIES})

add_library(folding_assembly_estimator src/folding_assembly/folding_assembly_estimator.cpp src/algorithm_base.cpp)
target_link_libraries(folding_assembly_estimator ${catkin_LIBRARIES})

add_library(ects_controller src/mechanism_identification/ects_controller.cpp src/algorithm_base.cpp)
target_link_libraries(ects_controller ${catkin_LIBRARIES})

add_executable(test_bed_manipulation src/test_bed_manipulation.cpp)
target_link_libraries(test_bed_manipulation manipulation_ekf manipulation_control_algorithm ${catkin_LIBRARIES})
add_dependencies(test_bed_manipulation pr2_algorithms_generate_messages_cpp)

add_executable(test_bed_folding src/test_bed_folding.cpp)
target_link_libraries(test_bed_folding folding_assembly_estimator ${catkin_LIBRARIES})
add_dependencies(test_bed_folding pr2_algorithms_generate_messages_cpp)
