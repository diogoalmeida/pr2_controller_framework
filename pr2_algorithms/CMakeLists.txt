cmake_minimum_required(VERSION 2.8.3)
project(pr2_algorithms)

find_package(
catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  eigen_conversions
  tf
  kdl_parser
  kdl_conversions
  actionlib_msgs
  visualization_msgs
  robot_state_publisher
  dynamic_reconfigure
)

link_directories(${catkin_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "-std=c++0x -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ${CMAKE_CXX_FLAGS}")
set(ROS_BUILD_TYPE Release)

generate_dynamic_reconfigure_options(
  cfg/ects.cfg
)

add_message_files(
  FILES
  TestBedManipulationFeedback.msg
  TestBedFoldingFeedback.msg
  TestBedECTSFeedback.msg
  TestKalmanFeedback.msg
)

add_action_files(
  DIRECTORY action
  FILES ECTSDebug.action
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_generation tf kdl_conversions robot_state_publisher
  INCLUDE_DIRS include
  LIBRARIES manipulation_control_algorithm manipulation_ekf folding_assembly_controller folding_assembly_estimator kalman_filter adaptive_velocity_controller ects_controller rotational_kalman
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

add_library(kalman_filter src/mechanism_identification/kalman_filter.cpp src/algorithm_base.cpp)
target_link_libraries(kalman_filter ${catkin_LIBRARIES})

add_library(rotational_kalman src/mechanism_identification/rotational_kalman.cpp src/algorithm_base.cpp)
target_link_libraries(rotational_kalman ${catkin_LIBRARIES})

add_library(adaptive_velocity_controller src/mechanism_identification/adaptive_velocity_controller.cpp src/algorithm_base.cpp)
target_link_libraries(adaptive_velocity_controller ${catkin_LIBRARIES})

add_executable(test_bed_manipulation src/test_bed_manipulation.cpp)
target_link_libraries(test_bed_manipulation manipulation_ekf manipulation_control_algorithm ${catkin_LIBRARIES})
add_dependencies(test_bed_manipulation pr2_algorithms_generate_messages_cpp)

add_executable(test_bed_folding src/test_bed_folding.cpp)
target_link_libraries(test_bed_folding folding_assembly_estimator ${catkin_LIBRARIES})
add_dependencies(test_bed_folding pr2_algorithms_generate_messages_cpp)

add_executable(test_bed_ects src/test_bed_ects.cpp src/robot_simulator.cpp)
target_link_libraries(test_bed_ects ects_controller ${catkin_LIBRARIES})
add_dependencies(test_bed_ects pr2_algorithms_generate_messages_cpp pr2_algorithms_gencfg)

add_executable(test_kalman src/test_kalman.cpp)
target_link_libraries(test_kalman kalman_filter ${catkin_LIBRARIES})
add_dependencies(test_bed_ects pr2_algorithms_generate_messages_cpp pr2_algorithms_gencfg)