cmake_minimum_required(VERSION 3.8)
project(sh_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/Detection2D.msg"
  "msg/DetectedObjects.msg"
  "msg/ErrorCodes.msg"
  "msg/ObjectInstance.msg"
  "msg/PerceptionInput.msg"
  "msg/PerceptionScene.msg"
  "msg/SingleObjectInfo.msg"
  "msg/YoloConfig.msg")

set(srv_files
  "srv/DetectObjects.srv"
  "srv/ClearPlanningScene.srv"
  "srv/RemoveObjectFromScene.srv"
  "srv/RunPerception.srv"
  "srv/RunYoloDetection.srv"
  "srv/SelectNextTarget.srv"
  "srv/TickTree.srv"
  "srv/UpdatePlanningSceneFromPoses.srv"
)

set(action_files
  "action/MoveToNamedTarget.action"
  "action/MoveToObject.action"
  "action/MoveToPose.action"
  "action/Tick.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces sensor_msgs std_msgs geometry_msgs
  ADD_LINTER_TESTS)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
