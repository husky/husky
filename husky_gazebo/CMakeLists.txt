cmake_minimum_required(VERSION 3.0.2)
project(husky_gazebo)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()


install(
  DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES launch/empty_world.launch
        launch/husky_playpen.launch
        launch/playpen.launch
        launch/realsense.launch
        launch/spawn_husky.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/empty_world.launch)
  roslaunch_add_file_check(launch/husky_playpen.launch)
  roslaunch_add_file_check(launch/playpen.launch)
  roslaunch_add_file_check(launch/realsense.launch)
  roslaunch_add_file_check(launch/spawn_husky.launch)
endif()
