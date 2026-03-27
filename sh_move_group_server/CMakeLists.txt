cmake_minimum_required(VERSION 3.8)
project(sh_move_group_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sh_interfaces REQUIRED)

include_directories(
  include
)

add_executable(
  ${PROJECT_NAME}
  src/main.cpp
  src/move_group_server.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${moveit_ros_planning_interface_TARGETS}
  ${sh_interfaces_TARGETS}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
