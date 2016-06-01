cmake_minimum_required(VERSION 2.8.3)
project(husky_control)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch)

catkin_install_python(PROGRAMS scripts/stow_ur5 scripts/set_dual_ur5_gazebo_config DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
