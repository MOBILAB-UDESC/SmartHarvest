cmake_minimum_required(VERSION 3.8)
project(sh_perception_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sh_base_template REQUIRED)
find_package(sh_interfaces REQUIRED)

include_directories(
  include
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/perception_server.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${cv_bridge_TARGETS}
  ${OpenCV_LIBS}
  ${sh_interfaces_TARGETS}
  ${sh_base_template_TARGETS}
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
