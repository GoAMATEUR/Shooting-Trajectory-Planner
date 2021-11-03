cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  mav_msgs
  std_msgs
  rospy
  message_generation
)
catkin_python_setup()
add_message_files(
  FILES
  coeff_msgs.msg
  alpha_msgs.msg
  # Message2.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  mav_msgs
  nav_msgs
)


catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)



add_executable(trajectory_generator_node 
	src/trajectory_generator_node.cpp
	src/trajectory_generator_waypoint.cpp
)

add_executable(trajectory_generator_sim_node
  src/trajectory_generator_sim_node.cpp
  src/trajectory_generator_waypoint.cpp
)

add_executable(alpha_check_node
	src/alpha_check_node.cpp
)

target_link_libraries(trajectory_generator_sim_node
  ${catkin_LIBRARIES}
)

target_link_libraries(trajectory_generator_node
   ${catkin_LIBRARIES}
)

target_link_libraries(alpha_check_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
   bin/alpha_generator_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
