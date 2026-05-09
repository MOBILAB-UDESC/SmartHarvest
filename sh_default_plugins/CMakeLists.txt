cmake_minimum_required(VERSION 3.8)
project(sh_default_plugins)

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
find_package(rclpy REQUIRED)
find_package(sh_base_template REQUIRED)
find_package(sh_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

install(DIRECTORY include/
  DESTINATION include/
)

###########
# Plugins #
###########

set(plugins_dependencies
  cv_bridge
  geometry_msgs
  OpenCV
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sh_base_template
  sh_interfaces
  tf2_geometry_msgs
)

add_library(yolo_detector_py SHARED src/perception_plugin/yolo_detector_py.cpp)
ament_target_dependencies(yolo_detector_py
  ${plugins_dependencies}
)

add_library(mean_localiser SHARED src/perception_plugin/mean_localiser.cpp)
ament_target_dependencies(mean_localiser
  ${plugins_dependencies}
)

add_library(default_grasp_generator SHARED src/perception_plugin/default_grasp_generator.cpp)
ament_target_dependencies(default_grasp_generator
  ${plugins_dependencies}
)

add_library(basic_grasp_generator SHARED src/perception_plugin/basic_grasp_generator.cpp)
ament_target_dependencies(basic_grasp_generator
  ${plugins_dependencies}
)

pluginlib_export_plugin_description_file(sh_base_template sh_plugins.xml)

install(
  TARGETS yolo_detector_py mean_localiser default_grasp_generator basic_grasp_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

##########
# Python #
##########

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/perception_plugin/yolo_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
