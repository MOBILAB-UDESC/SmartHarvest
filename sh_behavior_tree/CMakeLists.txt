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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sh_bt_base_template REQUIRED)
find_package(sh_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

set(dependencies
)

#ACTIONS
add_library(get_odometry_action SHARED src/plugins/action/navigation/get_odometry_action.cpp)

add_library(get_camera_data_action SHARED src/plugins/action/detection/get_camera_data_action.cpp)

add_library(predict_action SHARED src/plugins/action/detection/predict_action.cpp)

add_library(transform_object_pose SHARED src/plugins/action/common/transform_object_pose.cpp)

add_library(compute_grasp_orientation SHARED src/plugins/action/pick_and_place/compute_grasp_orientation.cpp)

add_library(clear_planning_scene_service SHARED src/plugins/action/pick_and_place/clear_planning_scene_service.cpp)

add_library(update_scene_from_poses_service SHARED src/plugins/action/pick_and_place/update_scene_from_poses_service.cpp)

add_library(move_arm_to_named_target_action SHARED src/plugins/action/pick_and_place/move_arm_to_named_target_action.cpp)

add_library(move_arm_to_detected_object_action SHARED src/plugins/action/pick_and_place/move_arm_to_detected_object_action.cpp)

add_library(move_gripper_action SHARED src/plugins/action/pick_and_place/move_gripper_action.cpp)

add_library(check_error_ids SHARED src/plugins/action/pick_and_place/check_error_ids.cpp)

add_library(get_next_object_to_pick SHARED src/plugins/action/pick_and_place/get_next_object_to_pick.cpp)

# CONDITIONS
add_library(are_objects_found_condition SHARED src/plugins/condition/detection/are_objects_found_condition.cpp)

set(bt_nodes
  # ACTIONS
  get_camera_data_action
  get_odometry_action
  predict_action

  transform_object_pose

  compute_grasp_orientation
  clear_planning_scene_service
  update_scene_from_poses_service
  move_arm_to_named_target_action
  move_arm_to_detected_object_action
  move_gripper_action
  check_error_ids
  get_next_object_to_pick
  # CONDITIONS
  are_objects_found_condition)

# ADD DEPENDENCIES TO LIBRARIES
foreach(bt_node ${bt_nodes})
  target_link_libraries(${bt_node}
    behaviortree_cpp::behaviortree_cpp
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_lifecycle::rclcpp_lifecycle
    sh_bt_base_template::sh_bt_base_template
    ${message_filters_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${sh_interfaces_TARGETS}
    ${tf2_geometry_msgs_TARGETS})
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
  rclcpp_action
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
