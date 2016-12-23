cmake_minimum_required(VERSION 2.8.3)
project(husky_base)

find_package(catkin REQUIRED COMPONENTS controller_manager hardware_interface husky_msgs
  diagnostic_updater roslaunch roslint roscpp sensor_msgs)
find_package(Boost REQUIRED COMPONENTS chrono)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES horizon_legacy
  CATKIN_DEPENDS diagnostic_updater hardware_interface husky_msgs roscpp sensor_msgs
  DEPENDS Boost
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(horizon_legacy
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

add_executable(husky_node
  src/husky_base.cpp
  src/husky_hardware.cpp
  src/husky_diagnostics.cpp
  src/horizon_legacy_wrapper.cpp)
target_link_libraries(husky_node horizon_legacy ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(husky_node husky_msgs_gencpp)

roslint_cpp(
  src/husky_base.cpp
  src/husky_hardware.cpp
  src/husky_diagnostics.cpp
  include/husky_base/husky_diagnostics.h
  include/husky_base/husky_hardware.h
)

roslaunch_add_file_check(launch)

install(TARGETS horizon_legacy husky_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
