cmake_minimum_required(VERSION 3.8)
project(pid_tracking_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  angles
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_library(${PROJECT_NAME} SHARED
  src/pid_tracking_controller.cpp
  src/legacy_carrot_velocity_planner.cpp
  src/path_progress_tracker.cpp
  src/prepared_path.cpp
  common/src/geometry/vec2d.cpp
  common/src/math/math_helper.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/common/include
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

pluginlib_export_plugin_description_file(
  nav2_core
  pid_tracking_controller.xml
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY common/include/
  DESTINATION include
)

install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rcutils REQUIRED)
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

  ament_add_gtest(test_math_helper
    test/test_math_helper.cpp
  )
  target_include_directories(test_math_helper PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include
  )
  target_link_libraries(test_math_helper
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_legacy_carrot_velocity_planner
    test/test_legacy_carrot_velocity_planner.cpp
  )
  target_include_directories(test_legacy_carrot_velocity_planner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_legacy_carrot_velocity_planner
    ${PROJECT_NAME}
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
    ${PROJECT_NAME}
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
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_legacy_behavior
    test/test_legacy_behavior.cpp
  )
  target_include_directories(test_legacy_behavior PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_target_dependencies(test_legacy_behavior
    geometry_msgs
    nav2_costmap_2d
  )
  target_link_libraries(test_legacy_behavior
    ${PROJECT_NAME}
    Eigen3::Eigen
  )

  ament_add_gtest(test_path_and_control
    test/test_path_and_control.cpp
  )
  target_include_directories(test_path_and_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_target_dependencies(test_path_and_control
    geometry_msgs
    nav2_core
    nav2_costmap_2d
    nav_msgs
    rclcpp
  )
  target_link_libraries(test_path_and_control
    ${PROJECT_NAME}
    Eigen3::Eigen
  )

  ament_add_gtest(test_lifecycle_smoke
    test/test_lifecycle_smoke.cpp
  )
  set_tests_properties(test_lifecycle_smoke PROPERTIES
    ENVIRONMENT "ROS_LOG_DIR=/tmp;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;ROS_DOMAIN_ID=231"
  )
  target_include_directories(test_lifecycle_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common/include
  )
  ament_target_dependencies(test_lifecycle_smoke
    geometry_msgs
    nav2_costmap_2d
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    rcutils
    tf2_ros
  )
  target_link_libraries(test_lifecycle_smoke
    ${PROJECT_NAME}
    Eigen3::Eigen
  )
endif()

ament_package()
