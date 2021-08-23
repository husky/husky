cmake_minimum_required(VERSION 2.8.3)
project(husky_gazebo)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch)

install(
  DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES launch/husky_empty_world.launch
        launch/husky_playpen.launch
        launch/playpen.launch
        launch/realsense.launch
        launch/spawn_husky.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
