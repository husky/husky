cmake_minimum_required(VERSION 2.8.3)
project(husky_gazebo)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()
catkin_add_env_hooks(50.husky_gazebo
  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch worlds urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
