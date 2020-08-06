cmake_minimum_required(VERSION 2.8.3)
project(husky_bringup)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch)

install(PROGRAMS scripts/install scripts/calibrate_compass scripts/compute_calibration
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch udev config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
