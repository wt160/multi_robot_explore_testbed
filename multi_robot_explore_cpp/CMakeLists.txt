cmake_minimum_required(VERSION 3.5)
project(multi_robot_explore_cpp)

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
find_package(rclcpp_action REQUIRED)
find_package(multi_robot_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ZLIB REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(multi_robot_explore REQUIRED)
# find_package(robot_control_interface REQUIRED)




IF(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)


add_executable(multi_explorer src/multi_explore_node.cpp src/explore_util.cpp src/robot_control_interface.cpp src/wfd_detector.cpp src/group_coordinator.cpp src/map_frontier_merger.cpp)
ament_target_dependencies(multi_explorer
rclcpp rclcpp_action multi_robot_interfaces nav2_msgs std_msgs tf2_ros ZLIB visualization_msgs)

add_executable(test_cpp src/test.cpp)


target_include_directories(multi_explorer PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(group_coordinator_node src/group_coordinator_node.cpp  src/get_map_value_node.cpp  src/explore_util.cpp src/robot_control_interface.cpp src/wfd_detector.cpp src/map_frontier_merger.cpp)
ament_target_dependencies(group_coordinator_node
  rclcpp rclcpp_action multi_robot_interfaces nav2_msgs std_msgs tf2_ros ZLIB visualization_msgs)

target_include_directories(group_coordinator_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


add_executable(wfd_action_server_node src/wfd_action_server_node.cpp src/explore_util.cpp src/wfd_detector.cpp )
  ament_target_dependencies(wfd_action_server_node
    rclcpp rclcpp_action multi_robot_interfaces nav2_msgs std_msgs tf2_ros ZLIB visualization_msgs)
  
target_include_directories(wfd_action_server_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS
  multi_explorer
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
wfd_action_server_node
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS
  test_cpp
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  group_coordinator_node
  DESTINATION lib/${PROJECT_NAME})


# install(TARGETS
#   test_cpp
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME}/
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

# ARCHIVE DESTINATION lib
# LIBRARY DESTINATION lib
# RUNTIME DESTINATION share/${PROJECT_NAME}