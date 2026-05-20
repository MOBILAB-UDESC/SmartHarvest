cmake_minimum_required(VERSION 3.8)
project(sh_basic_moveit_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(sh_base_template REQUIRED)

include_directories(
  include
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

###########
# Plugins #
###########

set(plugins_dependencies
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp_lifecycle
  sh_base_template
)

add_library(prism_constraint_generator SHARED src/prism_constraint_generator.cpp)
ament_target_dependencies(prism_constraint_generator
  ${plugins_dependencies}
)

add_library(basic_orientation_constraint SHARED src/basic_orientation_constraint.cpp)
ament_target_dependencies(basic_orientation_constraint
  ${plugins_dependencies}
)

set(plugins
  prism_constraint_generator
  basic_orientation_constraint
)

pluginlib_export_plugin_description_file(sh_base_template plugins.xml)

install(
  TARGETS ${plugins}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(${plugins_dependencies})
ament_export_libraries(${plugins})

ament_package()
