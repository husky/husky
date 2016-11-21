cmake_minimum_required(VERSION 2.8.3)
project(husky_control)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch)

install(PROGRAMS scripts/stow_ur5 scripts/stow_ur5_left scripts/stow_ur5_right DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
