cmake_minimum_required(VERSION 3.5)
project(controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common REQUIRED)
find_package(seumath REQUIRED)
find_package(sensor_msgs REQUIRED)

set(WEBOTS_HOME /usr/local/webots)

include_directories(
  ${WEBOTS_HOME}/include/controller/cpp
  ${Qt5Widgets_INCLUDE_DIRS}
)

link_directories(
  ${WEBOTS_HOME}/lib/controller
)

add_executable(controller src/controller.cpp src/SimRobot.cpp)

add_executable(supervisor src/supervisor.cpp)

target_link_libraries(controller
  CppController
)

target_link_libraries(supervisor
  CppController
)

target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  controller
  "rclcpp"
  "sensor_msgs"
  "common"
  "seumath"
)

target_include_directories(supervisor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  supervisor
  "rclcpp"
  "sensor_msgs"
  "common"
  "seumath"
)

install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS supervisor
  DESTINATION lib/${PROJECT_NAME})

install(FILES controller_launch.py
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
