cmake_minimum_required(VERSION 3.0.2)
project(husky_description)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

install(
  DIRECTORY etc launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test_lms1xx.xml)
  roslaunch_add_file_check(tests/roslaunch_test_plain.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ust10.xml)
endif()
