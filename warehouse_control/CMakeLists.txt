cmake_minimum_required(VERSION 3.8)
project(warehouse_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(warehouse_interfaces REQUIRED)

add_executable(pure_pursuit_controller src/pure_pursuit_controller.cpp)
ament_target_dependencies(pure_pursuit_controller rclcpp rclcpp_action nav_msgs geometry_msgs tf2 tf2_ros warehouse_interfaces)

add_executable(navigation_manager src/navigation_manager.cpp)
ament_target_dependencies(navigation_manager rclcpp rclcpp_action geometry_msgs warehouse_interfaces)

add_executable(state_manager src/state_manager.cpp)
ament_target_dependencies(state_manager rclcpp warehouse_interfaces)

add_executable(path_planner src/path_planner.cpp)
ament_target_dependencies(path_planner rclcpp nav_msgs geometry_msgs warehouse_interfaces)

install(TARGETS
  pure_pursuit_controller
  navigation_manager
  state_manager
  path_planner
  DESTINATION lib/${PROJECT_NAME}
)

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
