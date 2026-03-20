cmake_minimum_required(VERSION 3.8)
project(sh_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sh_bt_base_template REQUIRED)
find_package(sh_interfaces REQUIRED)

include_directories(
  include
)

set(dependencies
)

#ACTIONS
add_library(get_odometry_action SHARED src/plugins/action/navigation/get_odometry_action.cpp)

add_library(get_camera_data_action SHARED src/plugins/action/detection/get_camera_data_action.cpp)

add_library(predict_action SHARED src/plugins/action/detection/predict_action.cpp)

# CONDITIONS
add_library(are_objects_found_condition SHARED src/plugins/condition/detection/are_objects_found_condition.cpp)

set(bt_nodes
  # ACTIONS
  get_camera_data_action
  get_odometry_action
  predict_action
  # CONDITIONS
  are_objects_found_condition)

# ADD DEPENDENCIES TO LIBRARIES
foreach(bt_node ${bt_nodes})
  target_link_libraries(${bt_node}
    behaviortree_cpp::behaviortree_cpp
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    sh_bt_base_template::sh_bt_base_template
    ${message_filters_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${sh_interfaces_TARGETS})
  target_compile_definitions(${bt_node} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

# INSTALL LIBRARIES
install(TARGETS ${bt_nodes}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
configure_file(plugins_list.hpp.in ${GENERATED_DIR}/plugins_list.hpp @ONLY)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY trees DESTINATION share/${PROJECT_NAME})
install(FILES ${GENERATED_DIR}/plugins_list.hpp DESTINATION include/${PROJECT_NAME})

ament_export_libraries(${bt_nodes})
ament_export_include_directories(include)
ament_export_dependencies(
  behaviortree_cpp
  message_filters
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  sh_bt_base_template
  sh_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
