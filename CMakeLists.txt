cmake_minimum_required(VERSION 3.8)
project(ros2_redundancy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(Python3 REQUIRED)
# find_package(ros2_redundancy REQUIRED)

install(PROGRAMS
  scripts/primary.py
  scripts/redundant.py
  scripts/primary_node.py
  scripts/redundant_node.py
  scripts/tools_lib.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch.py
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

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/RedundancyStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RedundancyStatus.msg"
  DEPENDENCIES builtin_interfaces
)


ament_package()
