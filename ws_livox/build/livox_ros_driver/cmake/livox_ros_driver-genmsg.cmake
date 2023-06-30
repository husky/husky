# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "livox_ros_driver: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ilivox_ros_driver:/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(livox_ros_driver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg" NAME_WE)
add_custom_target(_livox_ros_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "livox_ros_driver" "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg" ""
)

get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomMsg.msg" NAME_WE)
add_custom_target(_livox_ros_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "livox_ros_driver" "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomMsg.msg" "livox_ros_driver/CustomPoint:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(livox_ros_driver
  "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/livox_ros_driver
)
_generate_msg_cpp(livox_ros_driver
  "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/livox_ros_driver
)

### Generating Services

### Generating Module File
_generate_module_cpp(livox_ros_driver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/livox_ros_driver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(livox_ros_driver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(livox_ros_driver_generate_messages livox_ros_driver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg" NAME_WE)
add_dependencies(livox_ros_driver_generate_messages_cpp _livox_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomMsg.msg" NAME_WE)
add_dependencies(livox_ros_driver_generate_messages_cpp _livox_ros_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(livox_ros_driver_gencpp)
add_dependencies(livox_ros_driver_gencpp livox_ros_driver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS livox_ros_driver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(livox_ros_driver
  "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/livox_ros_driver
)
_generate_msg_eus(livox_ros_driver
  "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/livox_ros_driver
)

### Generating Services

### Generating Module File
_generate_module_eus(livox_ros_driver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/livox_ros_driver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(livox_ros_driver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(livox_ros_driver_generate_messages livox_ros_driver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg" NAME_WE)
add_dependencies(livox_ros_driver_generate_messages_eus _livox_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomMsg.msg" NAME_WE)
add_dependencies(livox_ros_driver_generate_messages_eus _livox_ros_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(livox_ros_driver_geneus)
add_dependencies(livox_ros_driver_geneus livox_ros_driver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS livox_ros_driver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(livox_ros_driver
  "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/livox_ros_driver
)
_generate_msg_lisp(livox_ros_driver
  "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/livox_ros_driver
)

### Generating Services

### Generating Module File
_generate_module_lisp(livox_ros_driver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/livox_ros_driver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(livox_ros_driver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(livox_ros_driver_generate_messages livox_ros_driver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg" NAME_WE)
add_dependencies(livox_ros_driver_generate_messages_lisp _livox_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomMsg.msg" NAME_WE)
add_dependencies(livox_ros_driver_generate_messages_lisp _livox_ros_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(livox_ros_driver_genlisp)
add_dependencies(livox_ros_driver_genlisp livox_ros_driver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS livox_ros_driver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(livox_ros_driver
  "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/livox_ros_driver
)
_generate_msg_nodejs(livox_ros_driver
  "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/livox_ros_driver
)

### Generating Services

### Generating Module File
_generate_module_nodejs(livox_ros_driver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/livox_ros_driver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(livox_ros_driver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(livox_ros_driver_generate_messages livox_ros_driver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg" NAME_WE)
add_dependencies(livox_ros_driver_generate_messages_nodejs _livox_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomMsg.msg" NAME_WE)
add_dependencies(livox_ros_driver_generate_messages_nodejs _livox_ros_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(livox_ros_driver_gennodejs)
add_dependencies(livox_ros_driver_gennodejs livox_ros_driver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS livox_ros_driver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(livox_ros_driver
  "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_ros_driver
)
_generate_msg_py(livox_ros_driver
  "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_ros_driver
)

### Generating Services

### Generating Module File
_generate_module_py(livox_ros_driver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_ros_driver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(livox_ros_driver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(livox_ros_driver_generate_messages livox_ros_driver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomPoint.msg" NAME_WE)
add_dependencies(livox_ros_driver_generate_messages_py _livox_ros_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/ws_livox/src/livox_ros_driver/msg/CustomMsg.msg" NAME_WE)
add_dependencies(livox_ros_driver_generate_messages_py _livox_ros_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(livox_ros_driver_genpy)
add_dependencies(livox_ros_driver_genpy livox_ros_driver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS livox_ros_driver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/livox_ros_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/livox_ros_driver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(livox_ros_driver_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/livox_ros_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/livox_ros_driver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(livox_ros_driver_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/livox_ros_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/livox_ros_driver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(livox_ros_driver_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/livox_ros_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/livox_ros_driver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(livox_ros_driver_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_ros_driver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_ros_driver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_ros_driver
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(livox_ros_driver_generate_messages_py std_msgs_generate_messages_py)
endif()
