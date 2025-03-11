cmake_minimum_required(VERSION 3.5)
project(valeport_hyperion_chlorophyll_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ...existing code...

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChlorophyllData.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ...existing code...

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()