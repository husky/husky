cmake_minimum_required(VERSION 3.5)
project(husky_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(husky_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


## COMPILE
add_library(
  husky_hardware
  SHARED
  src/husky_hardware.cpp
  src/husky_status.cpp
  src/horizon_legacy_wrapper.cpp
  src/horizon_legacy/crc.cpp
  src/horizon_legacy/Logger.cpp
  src/horizon_legacy/Message.cpp
  src/horizon_legacy/Message_data.cpp
  src/horizon_legacy/Message_request.cpp
  src/horizon_legacy/Message_cmd.cpp
  src/horizon_legacy/Transport.cpp
  src/horizon_legacy/Number.cpp
  src/horizon_legacy/linux_serial.cpp
)

target_include_directories(
  husky_hardware
  PRIVATE
  include
)

ament_target_dependencies(
  husky_hardware
  husky_msgs
  hardware_interface
  pluginlib
  rclcpp
)


pluginlib_export_plugin_description_file(hardware_interface husky_hardware.xml)

# INSTALL
install(
  TARGETS husky_hardware
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  husky_hardware
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
