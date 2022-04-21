cmake_minimum_required(VERSION 3.5)
project(husky_bringup)

find_package(ament_cmake REQUIRED)


install(DIRECTORY config launch udev
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/install 
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
