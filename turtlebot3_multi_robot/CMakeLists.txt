################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_multi_robot)


################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo REQUIRED)

##############################
# Animated_box gazebo plugin
##############################
add_library (animated_box SHARED src/animated_box.cpp)
target_compile_options(animated_box PUBLIC "${GAZEBO_CXX_FLAGS}")
target_include_directories(animated_box PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_directories(animated_box PUBLIC ${GAZEBO_LIBRARY_DIRS})

install(TARGETS animated_box
  DESTINATION lib/${PROJECT_NAME})

################################################################################
# Install
################################################################################
install(DIRECTORY launch models rviz urdf worlds params
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multi_robot.dsv.in")

ament_package()
