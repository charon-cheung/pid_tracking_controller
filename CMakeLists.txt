cmake_minimum_required(VERSION 3.5)
project(pid_tracking_controller)

set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
  ${eigen3_INCLUDES}/eigen3
)

add_library(${PROJECT_NAME} SHARED
        src/pid_tracking_controller.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ros2::ros2
        common
        tinyxml2::tinyxml2
        conversions
        idl::idl
        navigation2::navigation2
        nav2_core::nav2_core
        quick_log::quick_log
        rcmsg::rcmsg
        spdlog::spdlog
        )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${INSTALL_PREFIX}/lib
        LIBRARY DESTINATION ${INSTALL_PREFIX}/lib
        RUNTIME DESTINATION ${INSTALL_PREFIX}/bin
        )
        
install(DIRECTORY include/
        DESTINATION include/
)
      