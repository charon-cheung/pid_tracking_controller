cmake_minimum_required(VERSION 3.5)
project(common)

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

# find ROS2 dependencies
# find_package(ros2 REQUIRED)

include_directories(
  include
#   ${EIGEN3_INCLUDE_DIR}
#   /usr/include/eigen3/
    ${eigen3_INCLUDES}/eigen3
)

file(GLOB math_src_files_cpp src/math/*.cpp)
file(GLOB geometry_common_src_files src/geometry/*.cpp)
file(GLOB geometry_curve_src_files src/geometry/curve/*.cpp)

add_library(${PROJECT_NAME}
  ${math_src_files_cpp}
  ${geometry_common_src_files}
  ${geometry_curve_src_files}
)

target_link_libraries(${PROJECT_NAME}
        ros2::ros2
        idl::idl
        kdl::kdl
        quick_log::quick_log
        tinyxml2::tinyxml2
        rcmsg::rcmsg
        fastcdr::fastcdr
        spdlog::spdlog
        eigen3::eigen3
    )

install(
        DIRECTORY include/
        DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION ${INSTALL_PREFIX}/lib
    LIBRARY DESTINATION ${INSTALL_PREFIX}/lib
    RUNTIME DESTINATION ${INSTALL_PREFIX}/bin
    INCLUDES DESTINATION include
)