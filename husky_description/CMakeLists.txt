cmake_minimum_required(VERSION 2.8.3)
project(husky_description)

find_package(catkin)

catkin_package()
catkin_add_env_hooks(50.husky_description
  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(
  DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch DEPENDENCIES ${catkin_EXPORTED_TARGETS})
endif()