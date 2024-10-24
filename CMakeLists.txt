cmake_minimum_required(VERSION 3.0.2)  # ROS1 uses 3.0.2, ROS2 uses 3.8 or higher
project(ros_redundancy)

# Detect if building in ROS2 or ROS1
if(DEFINED ENV{ROS_VERSION} AND $ENV{ROS_VERSION} EQUAL 2)
  set(ROS2 TRUE)
else()
  set(ROS1 TRUE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
if(ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(rclpy REQUIRED)
  find_package(Python3 REQUIRED)
endif()

# ROS1 dependencies
if(ROS1)
  find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
  )
endif()

# Define message files for both ROS1 and ROS2
set(msg_files
  "msg/RedundancyStatus.msg"
)

# ROS2 message generation
if(ROS2)
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES builtin_interfaces
  )
endif()

# ROS1 message generation
if(ROS1)
  add_message_files(
    FILES
      RedundancyStatus.msg
  )

  generate_messages(
    DEPENDENCIES std_msgs
  )
endif()

# Install Python scripts (common to both ROS1 and ROS2)
if(ROS2)
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
elseif(ROS1)
  catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
  )

  catkin_install_python(PROGRAMS
    scripts/primary.py
    scripts/redundant.py
    scripts/primary_node.py
    scripts/redundant_node.py
    scripts/tools_lib.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

# Tests for ROS2
if(ROS2 AND BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ROS1 build export
if(ROS1)
  catkin_package(
    CATKIN_DEPENDS message_runtime
  )
endif()

# ROS2 export
if(ROS2)
  ament_package()
endif()
