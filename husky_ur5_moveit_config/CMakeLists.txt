cmake_minimum_required(VERSION 2.8.3)
project(husky_ur5_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

#roslaunch_add_file_check(launch)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
