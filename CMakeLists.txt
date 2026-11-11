cmake_minimum_required(VERSION 3.8)
project(pid_tracking_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rbs_mppi_controller REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackingProgress.msg" DEPENDENCIES std_msgs builtin_interfaces geometry_msgs)
rosidl_get_typesupport_target(progress_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")

set(dependencies
  angles
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav2_msgs
  nav_msgs
  sensor_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_library(pid_tracking_controller_plugin SHARED
  src/pid_tracking_controller.cpp
  src/tracking_progress_publisher.cpp
  src/angular_velocity_feedback_filter.cpp
  src/execution_safety.cpp
  src/execution_safety_inputs.cpp
  src/execution_safety_adapter.cpp
  src/motion_observation.cpp
  src/holonomic_command_limiter.cpp
  src/holonomic_control_law.cpp
  src/fixed_heading_yaw_gate.cpp
  src/translation_tracking_guard.cpp
  src/holonomic_speed_planner.cpp
  src/holonomic_acceleration_envelope.cpp
  src/holonomic_tracking_reference.cpp
  src/path_progress_tracker.cpp
  src/prepared_path.cpp
  src/rotation_completion_tracker.cpp
  src/rotation_entry_gate.cpp
  src/rotation_velocity_planner.cpp
)
set_target_properties(pid_tracking_controller_plugin PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_include_directories(pid_tracking_controller_plugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# Header-only full-footprint geometry; do not link the MPPI controller.
target_include_directories(pid_tracking_controller_plugin SYSTEM PRIVATE
  ${rbs_mppi_controller_INCLUDE_DIRS}
)
ament_target_dependencies(pid_tracking_controller_plugin ${dependencies})
target_link_libraries(pid_tracking_controller_plugin "${progress_typesupport}")

pluginlib_export_plugin_description_file(
  nav2_core
  pid_tracking_controller.xml
)

install(TARGETS pid_tracking_controller_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rcutils REQUIRED)
  find_package(rcl REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_plugin_loading
    test/test_plugin_loading.cpp
  )
  ament_target_dependencies(test_plugin_loading
    nav2_core
    pluginlib
  )

  ament_add_gtest(test_path_progress_tracker
    test/test_path_progress_tracker.cpp
  )
  target_include_directories(test_path_progress_tracker PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  ament_target_dependencies(test_path_progress_tracker
    geometry_msgs
    nav_msgs
  )
  target_link_libraries(test_path_progress_tracker
    pid_tracking_controller_plugin
  )

  ament_add_gtest(test_prepared_path
    test/test_prepared_path.cpp
  )
  target_include_directories(test_prepared_path PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  ament_target_dependencies(test_prepared_path
    geometry_msgs
    nav_msgs
  )
  target_link_libraries(test_prepared_path
    pid_tracking_controller_plugin
  )

  ament_add_gtest(test_holonomic_tracking_reference
    test/test_holonomic_tracking_reference.cpp
  )
  target_include_directories(test_holonomic_tracking_reference PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  ament_target_dependencies(test_holonomic_tracking_reference
    geometry_msgs
    nav_msgs
  )
  target_link_libraries(test_holonomic_tracking_reference
    pid_tracking_controller_plugin
  )

  ament_add_gtest(test_rotation_velocity_planner test/test_rotation_velocity_planner.cpp)
  target_include_directories(test_rotation_velocity_planner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(test_rotation_velocity_planner pid_tracking_controller_plugin)

  ament_add_gtest(test_holonomic_control_law
    test/test_holonomic_control_law.cpp
  )
  target_include_directories(test_holonomic_control_law PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_holonomic_control_law
    pid_tracking_controller_plugin
  )

  ament_add_gtest(test_fixed_heading_yaw_gate test/test_fixed_heading_yaw_gate.cpp)
  target_include_directories(test_fixed_heading_yaw_gate PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  ament_target_dependencies(test_fixed_heading_yaw_gate geometry_msgs nav_msgs)
  target_link_libraries(test_fixed_heading_yaw_gate pid_tracking_controller_plugin)

  ament_add_gtest(test_angular_velocity_feedback_filter test/test_angular_velocity_feedback_filter.cpp)
  target_include_directories(test_angular_velocity_feedback_filter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(test_angular_velocity_feedback_filter pid_tracking_controller_plugin)

  ament_add_gtest(test_holonomic_speed_planner
    test/test_holonomic_speed_planner.cpp
  )
  target_include_directories(test_holonomic_speed_planner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  ament_target_dependencies(test_holonomic_speed_planner
    geometry_msgs
    nav_msgs
  )
  target_link_libraries(test_holonomic_speed_planner
    pid_tracking_controller_plugin
  )

  ament_add_gtest(test_translation_tracking_guard test/test_translation_tracking_guard.cpp)
  target_include_directories(test_translation_tracking_guard PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(test_translation_tracking_guard pid_tracking_controller_plugin)

  ament_add_gtest(test_execution_safety_adapter test/test_execution_safety_adapter.cpp)
  target_include_directories(test_execution_safety_adapter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  ament_target_dependencies(test_execution_safety_adapter ${dependencies} rcutils)
  target_link_libraries(test_execution_safety_adapter pid_tracking_controller_plugin)

  ament_add_gtest(test_motion_observation test/test_motion_observation.cpp)
  target_include_directories(test_motion_observation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  ament_target_dependencies(test_motion_observation geometry_msgs rclcpp)
  target_link_libraries(test_motion_observation pid_tracking_controller_plugin)

  ament_add_gtest(test_execution_safety_inputs test/test_execution_safety_inputs.cpp)
  target_include_directories(test_execution_safety_inputs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  ament_target_dependencies(test_execution_safety_inputs
    nav2_costmap_2d nav2_msgs geometry_msgs rclcpp tf2_ros tf2_geometry_msgs)
  target_link_libraries(test_execution_safety_inputs pid_tracking_controller_plugin)

  ament_add_gtest(test_execution_safety test/test_execution_safety.cpp)
  target_include_directories(test_execution_safety PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  ament_target_dependencies(test_execution_safety nav2_costmap_2d geometry_msgs)
  target_link_libraries(test_execution_safety pid_tracking_controller_plugin)

  ament_add_gtest(test_holonomic_command_limiter
    test/test_holonomic_command_limiter.cpp
  )
  target_include_directories(test_holonomic_command_limiter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_holonomic_command_limiter
    pid_tracking_controller_plugin
  )

  ament_add_gtest(test_rotation_completion_tracker
    test/test_rotation_completion_tracker.cpp
  )
  target_include_directories(test_rotation_completion_tracker PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_rotation_completion_tracker
    pid_tracking_controller_plugin
  )

  ament_add_gtest(test_rotation_entry_gate test/test_rotation_entry_gate.cpp)
  target_include_directories(test_rotation_entry_gate PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(test_rotation_entry_gate pid_tracking_controller_plugin)

  ament_add_gtest(test_path_and_control
    test/test_path_and_control.cpp
  )
  target_include_directories(test_path_and_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_target_dependencies(test_path_and_control
    rcl
    geometry_msgs
    nav2_core
    nav2_costmap_2d
    nav_msgs
    rclcpp
  )
  target_link_libraries(test_path_and_control
    pid_tracking_controller_plugin
  )

  ament_add_gtest(test_lifecycle_smoke
    test/test_lifecycle_smoke.cpp
  )
  set_tests_properties(test_lifecycle_smoke PROPERTIES
    ENVIRONMENT "ROS_LOG_DIR=/tmp;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;ROS_DOMAIN_ID=231"
  )
  target_include_directories(test_lifecycle_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_lifecycle_smoke
    rcl
    geometry_msgs
    nav2_costmap_2d
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    rcutils
    tf2_ros
  )
  target_link_libraries(test_lifecycle_smoke
    pid_tracking_controller_plugin
  )
endif()

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime std_msgs builtin_interfaces geometry_msgs nav_msgs)
ament_package()
