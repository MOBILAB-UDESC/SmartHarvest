cmake_minimum_required(VERSION 3.8)
project(sh_bt_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sh_behavior_tree REQUIRED)
find_package(sh_interfaces REQUIRED)

include_directories(
  include
  ${sh_behavior_tree_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}
  src/main.cpp
  src/bt_executor.cpp
  src/bt_node_loader.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
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
