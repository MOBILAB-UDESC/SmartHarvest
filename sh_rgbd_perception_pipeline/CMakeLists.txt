cmake_minimum_required(VERSION 3.8)
project(sh_rgbd_perception_pipeline)

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
find_package(tf2_geometry_msgs REQUIRED)

find_package(sh_base_template REQUIRED)
find_package(sh_interfaces REQUIRED)
find_package(sh_utils REQUIRED)

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
  cv_bridge
  geometry_msgs
  OpenCV
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sh_base_template
  sh_interfaces
  sh_utils
  tf2_geometry_msgs
)

add_library(rgbd_perception_pipeline SHARED src/rgbd_perception_pipeline.cpp)
ament_target_dependencies(rgbd_perception_pipeline
  ${plugins_dependencies}
)

add_library(yolo_detector_py SHARED src/yolo_detector_py.cpp)
ament_target_dependencies(yolo_detector_py
  ${plugins_dependencies}
)

add_library(mean_localiser SHARED src/mean_localiser.cpp)
ament_target_dependencies(mean_localiser
  ${plugins_dependencies}
)

add_library(default_grasp_generator SHARED src/default_grasp_generator.cpp)
ament_target_dependencies(default_grasp_generator
  ${plugins_dependencies}
)

add_library(basic_grasp_generator SHARED src/basic_grasp_generator.cpp)
ament_target_dependencies(basic_grasp_generator
  ${plugins_dependencies}
)

set(plugins
  rgbd_perception_pipeline
  yolo_detector_py
  mean_localiser
  default_grasp_generator
  basic_grasp_generator
)

pluginlib_export_plugin_description_file(sh_base_template sh_plugins.xml)

install(
  TARGETS ${plugins}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

##########
# Python #
##########

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/yolo_server.py
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

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(${plugins_dependencies})
ament_export_libraries(${plugins})

ament_package()
