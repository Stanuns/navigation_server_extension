cmake_minimum_required(VERSION 3.8)
project(navigation_server_extension)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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

add_executable(navigation_mgmt_server src/navigation_mgmt_server.cpp)
ament_target_dependencies(navigation_mgmt_server
  robot_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  std_srvs
  nav_msgs
)
target_link_libraries(navigation_mgmt_server
  ${YAML_CPP_LIBRARIES}
)
add_executable(multi_goals_client_test src/multi_goals_client_test.cpp)
ament_target_dependencies(multi_goals_client_test
  robot_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  std_srvs
  nav_msgs
)
add_executable(navigation_monitor src/navigation_monitor.cpp)
ament_target_dependencies(navigation_monitor
  robot_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  std_srvs
  nav_msgs
  lifecycle_msgs
  std_msgs
)

install(TARGETS navigation_mgmt_server multi_goals_client_test navigation_monitor
    DESTINATION lib/${PROJECT_NAME}
)
# install(
#         DIRECTORY launch
#         DESTINATION share/${PROJECT_NAME}
# )

ament_package()
