cmake_minimum_required(VERSION 3.8)
project(turtlechasehwk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# High level packages for ROS2
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Eigen Stuff
# Notes on doing this from 
# Stole this from https://github.com/PointCloudLibrary/pcl/blob/master/cmake/Modules/FindEigen.cmake
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

# Project specific
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# Utility Libraries
include_directories(include)
add_library(utils SHARED src/utils.cpp)
ament_target_dependencies(utils rclcpp)

# Create & install CPP executables
add_executable(villain_pool src/villain_pool.cpp)
target_link_libraries(villain_pool utils)
ament_target_dependencies(villain_pool rclcpp turtlesim my_robot_interfaces)

add_executable(hero_turtle src/hero_turtle.cpp)
target_link_libraries(hero_turtle utils)
ament_target_dependencies(hero_turtle rclcpp turtlesim my_robot_interfaces geometry_msgs)

install(TARGETS
  villain_pool
  hero_turtle
  DESTINATION lib/${PROJECT_NAME}
)

# Install python modules
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
#   scripts/my_executable  # Might need to link these manually...
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
