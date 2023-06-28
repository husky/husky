# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ouster_ros: 1 messages, 3 services")

set(MSG_I_FLAGS "-Iouster_ros:/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ouster_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/msg/PacketMsg.msg" NAME_WE)
add_custom_target(_ouster_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ouster_ros" "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/msg/PacketMsg.msg" ""
)

get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetConfig.srv" NAME_WE)
add_custom_target(_ouster_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ouster_ros" "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetConfig.srv" ""
)

get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/SetConfig.srv" NAME_WE)
add_custom_target(_ouster_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ouster_ros" "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/SetConfig.srv" ""
)

get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetMetadata.srv" NAME_WE)
add_custom_target(_ouster_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ouster_ros" "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetMetadata.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/msg/PacketMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ouster_ros
)

### Generating Services
_generate_srv_cpp(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ouster_ros
)
_generate_srv_cpp(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/SetConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ouster_ros
)
_generate_srv_cpp(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetMetadata.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ouster_ros
)

### Generating Module File
_generate_module_cpp(ouster_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ouster_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ouster_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ouster_ros_generate_messages ouster_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/msg/PacketMsg.msg" NAME_WE)
add_dependencies(ouster_ros_generate_messages_cpp _ouster_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetConfig.srv" NAME_WE)
add_dependencies(ouster_ros_generate_messages_cpp _ouster_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/SetConfig.srv" NAME_WE)
add_dependencies(ouster_ros_generate_messages_cpp _ouster_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetMetadata.srv" NAME_WE)
add_dependencies(ouster_ros_generate_messages_cpp _ouster_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ouster_ros_gencpp)
add_dependencies(ouster_ros_gencpp ouster_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ouster_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/msg/PacketMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ouster_ros
)

### Generating Services
_generate_srv_eus(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ouster_ros
)
_generate_srv_eus(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/SetConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ouster_ros
)
_generate_srv_eus(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetMetadata.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ouster_ros
)

### Generating Module File
_generate_module_eus(ouster_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ouster_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ouster_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ouster_ros_generate_messages ouster_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/msg/PacketMsg.msg" NAME_WE)
add_dependencies(ouster_ros_generate_messages_eus _ouster_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetConfig.srv" NAME_WE)
add_dependencies(ouster_ros_generate_messages_eus _ouster_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/SetConfig.srv" NAME_WE)
add_dependencies(ouster_ros_generate_messages_eus _ouster_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetMetadata.srv" NAME_WE)
add_dependencies(ouster_ros_generate_messages_eus _ouster_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ouster_ros_geneus)
add_dependencies(ouster_ros_geneus ouster_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ouster_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/msg/PacketMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ouster_ros
)

### Generating Services
_generate_srv_lisp(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ouster_ros
)
_generate_srv_lisp(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/SetConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ouster_ros
)
_generate_srv_lisp(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetMetadata.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ouster_ros
)

### Generating Module File
_generate_module_lisp(ouster_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ouster_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ouster_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ouster_ros_generate_messages ouster_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/msg/PacketMsg.msg" NAME_WE)
add_dependencies(ouster_ros_generate_messages_lisp _ouster_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetConfig.srv" NAME_WE)
add_dependencies(ouster_ros_generate_messages_lisp _ouster_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/SetConfig.srv" NAME_WE)
add_dependencies(ouster_ros_generate_messages_lisp _ouster_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetMetadata.srv" NAME_WE)
add_dependencies(ouster_ros_generate_messages_lisp _ouster_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ouster_ros_genlisp)
add_dependencies(ouster_ros_genlisp ouster_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ouster_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/msg/PacketMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ouster_ros
)

### Generating Services
_generate_srv_nodejs(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ouster_ros
)
_generate_srv_nodejs(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/SetConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ouster_ros
)
_generate_srv_nodejs(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetMetadata.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ouster_ros
)

### Generating Module File
_generate_module_nodejs(ouster_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ouster_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ouster_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ouster_ros_generate_messages ouster_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/msg/PacketMsg.msg" NAME_WE)
add_dependencies(ouster_ros_generate_messages_nodejs _ouster_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetConfig.srv" NAME_WE)
add_dependencies(ouster_ros_generate_messages_nodejs _ouster_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/SetConfig.srv" NAME_WE)
add_dependencies(ouster_ros_generate_messages_nodejs _ouster_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetMetadata.srv" NAME_WE)
add_dependencies(ouster_ros_generate_messages_nodejs _ouster_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ouster_ros_gennodejs)
add_dependencies(ouster_ros_gennodejs ouster_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ouster_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/msg/PacketMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ouster_ros
)

### Generating Services
_generate_srv_py(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ouster_ros
)
_generate_srv_py(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/SetConfig.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ouster_ros
)
_generate_srv_py(ouster_ros
  "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetMetadata.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ouster_ros
)

### Generating Module File
_generate_module_py(ouster_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ouster_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ouster_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ouster_ros_generate_messages ouster_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/msg/PacketMsg.msg" NAME_WE)
add_dependencies(ouster_ros_generate_messages_py _ouster_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetConfig.srv" NAME_WE)
add_dependencies(ouster_ros_generate_messages_py _ouster_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/SetConfig.srv" NAME_WE)
add_dependencies(ouster_ros_generate_messages_py _ouster_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kaijun/Documents/husky_autonomous/catkin_ws/src/ouster-ros/srv/GetMetadata.srv" NAME_WE)
add_dependencies(ouster_ros_generate_messages_py _ouster_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ouster_ros_genpy)
add_dependencies(ouster_ros_genpy ouster_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ouster_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ouster_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ouster_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ouster_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(ouster_ros_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(ouster_ros_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ouster_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ouster_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ouster_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(ouster_ros_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(ouster_ros_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ouster_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ouster_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ouster_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(ouster_ros_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(ouster_ros_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ouster_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ouster_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ouster_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(ouster_ros_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(ouster_ros_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ouster_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ouster_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ouster_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ouster_ros_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(ouster_ros_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(ouster_ros_generate_messages_py geometry_msgs_generate_messages_py)
endif()
