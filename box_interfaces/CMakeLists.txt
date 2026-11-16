cmake_minimum_required(VERSION 3.8)
project(box_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Inventory.msg"
  "msg/ClassSpec.msg"
  "msg/PlaceZone.msg"
  "msg/FixedPoses.msg"
  "msg/LogEvent.msg"
  "srv/GetJobData.srv"
  "srv/UpdateInventory.srv"
  "srv/SetJobStatus.srv"
  "srv/EmergencyStop.srv"
  "action/Scan.action"
  "action/Judge.action"
  "action/Place.action"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
