# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "drone_msgs: 10 messages, 0 services")

set(MSG_I_FLAGS "-Idrone_msgs:/home/leekai/drone_ws/src/drone_msgs/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(drone_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg" NAME_WE)
add_custom_target(_drone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_msgs" "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg" "drone_msgs/DronePose:geometry_msgs/Point"
)

get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg" NAME_WE)
add_custom_target(_drone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_msgs" "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Goal.msg" NAME_WE)
add_custom_target(_drone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_msgs" "/home/leekai/drone_ws/src/drone_msgs/msg/Goal.msg" "drone_msgs/DronePose:geometry_msgs/Point"
)

get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/WindowAngleDir.msg" NAME_WE)
add_custom_target(_drone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_msgs" "/home/leekai/drone_ws/src/drone_msgs/msg/WindowAngleDir.msg" ""
)

get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/LocalPlannerState.msg" NAME_WE)
add_custom_target(_drone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_msgs" "/home/leekai/drone_ws/src/drone_msgs/msg/LocalPlannerState.msg" ""
)

get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/WindowPointDir.msg" NAME_WE)
add_custom_target(_drone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_msgs" "/home/leekai/drone_ws/src/drone_msgs/msg/WindowPointDir.msg" "drone_msgs/DronePose:geometry_msgs/Point"
)

get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/RoomParams.msg" NAME_WE)
add_custom_target(_drone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_msgs" "/home/leekai/drone_ws/src/drone_msgs/msg/RoomParams.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Diagnostics.msg" NAME_WE)
add_custom_target(_drone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_msgs" "/home/leekai/drone_ws/src/drone_msgs/msg/Diagnostics.msg" "sensor_msgs/NavSatStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfoArray.msg" NAME_WE)
add_custom_target(_drone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_msgs" "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfoArray.msg" "drone_msgs/DroneInfo:drone_msgs/DronePose:geometry_msgs/Point"
)

get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Strike.msg" NAME_WE)
add_custom_target(_drone_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_msgs" "/home/leekai/drone_ws/src/drone_msgs/msg/Strike.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_msgs
)
_generate_msg_cpp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_msgs
)
_generate_msg_cpp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/Diagnostics.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_msgs
)
_generate_msg_cpp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/WindowAngleDir.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_msgs
)
_generate_msg_cpp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/LocalPlannerState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_msgs
)
_generate_msg_cpp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/WindowPointDir.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_msgs
)
_generate_msg_cpp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/RoomParams.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_msgs
)
_generate_msg_cpp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_msgs
)
_generate_msg_cpp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg;/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_msgs
)
_generate_msg_cpp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/Strike.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(drone_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(drone_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(drone_msgs_generate_messages drone_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_cpp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_cpp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Goal.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_cpp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/WindowAngleDir.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_cpp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/LocalPlannerState.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_cpp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/WindowPointDir.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_cpp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/RoomParams.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_cpp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Diagnostics.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_cpp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfoArray.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_cpp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Strike.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_cpp _drone_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_msgs_gencpp)
add_dependencies(drone_msgs_gencpp drone_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_msgs
)
_generate_msg_eus(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_msgs
)
_generate_msg_eus(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/Diagnostics.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_msgs
)
_generate_msg_eus(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/WindowAngleDir.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_msgs
)
_generate_msg_eus(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/LocalPlannerState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_msgs
)
_generate_msg_eus(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/WindowPointDir.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_msgs
)
_generate_msg_eus(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/RoomParams.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_msgs
)
_generate_msg_eus(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_msgs
)
_generate_msg_eus(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg;/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_msgs
)
_generate_msg_eus(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/Strike.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(drone_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(drone_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(drone_msgs_generate_messages drone_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_eus _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_eus _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Goal.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_eus _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/WindowAngleDir.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_eus _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/LocalPlannerState.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_eus _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/WindowPointDir.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_eus _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/RoomParams.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_eus _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Diagnostics.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_eus _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfoArray.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_eus _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Strike.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_eus _drone_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_msgs_geneus)
add_dependencies(drone_msgs_geneus drone_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_msgs
)
_generate_msg_lisp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_msgs
)
_generate_msg_lisp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/Diagnostics.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_msgs
)
_generate_msg_lisp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/WindowAngleDir.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_msgs
)
_generate_msg_lisp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/LocalPlannerState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_msgs
)
_generate_msg_lisp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/WindowPointDir.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_msgs
)
_generate_msg_lisp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/RoomParams.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_msgs
)
_generate_msg_lisp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_msgs
)
_generate_msg_lisp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg;/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_msgs
)
_generate_msg_lisp(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/Strike.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(drone_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(drone_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(drone_msgs_generate_messages drone_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_lisp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_lisp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Goal.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_lisp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/WindowAngleDir.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_lisp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/LocalPlannerState.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_lisp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/WindowPointDir.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_lisp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/RoomParams.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_lisp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Diagnostics.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_lisp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfoArray.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_lisp _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Strike.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_lisp _drone_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_msgs_genlisp)
add_dependencies(drone_msgs_genlisp drone_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_msgs
)
_generate_msg_nodejs(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_msgs
)
_generate_msg_nodejs(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/Diagnostics.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_msgs
)
_generate_msg_nodejs(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/WindowAngleDir.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_msgs
)
_generate_msg_nodejs(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/LocalPlannerState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_msgs
)
_generate_msg_nodejs(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/WindowPointDir.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_msgs
)
_generate_msg_nodejs(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/RoomParams.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_msgs
)
_generate_msg_nodejs(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_msgs
)
_generate_msg_nodejs(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg;/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_msgs
)
_generate_msg_nodejs(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/Strike.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(drone_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(drone_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(drone_msgs_generate_messages drone_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_nodejs _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_nodejs _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Goal.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_nodejs _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/WindowAngleDir.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_nodejs _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/LocalPlannerState.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_nodejs _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/WindowPointDir.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_nodejs _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/RoomParams.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_nodejs _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Diagnostics.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_nodejs _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfoArray.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_nodejs _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Strike.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_nodejs _drone_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_msgs_gennodejs)
add_dependencies(drone_msgs_gennodejs drone_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_msgs
)
_generate_msg_py(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_msgs
)
_generate_msg_py(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/Diagnostics.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_msgs
)
_generate_msg_py(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/WindowAngleDir.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_msgs
)
_generate_msg_py(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/LocalPlannerState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_msgs
)
_generate_msg_py(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/WindowPointDir.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_msgs
)
_generate_msg_py(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/RoomParams.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_msgs
)
_generate_msg_py(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_msgs
)
_generate_msg_py(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfoArray.msg"
  "${MSG_I_FLAGS}"
  "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg;/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_msgs
)
_generate_msg_py(drone_msgs
  "/home/leekai/drone_ws/src/drone_msgs/msg/Strike.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(drone_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(drone_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(drone_msgs_generate_messages drone_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfo.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_py _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DronePose.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_py _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Goal.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_py _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/WindowAngleDir.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_py _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/LocalPlannerState.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_py _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/WindowPointDir.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_py _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/RoomParams.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_py _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Diagnostics.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_py _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/DroneInfoArray.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_py _drone_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leekai/drone_ws/src/drone_msgs/msg/Strike.msg" NAME_WE)
add_dependencies(drone_msgs_generate_messages_py _drone_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_msgs_genpy)
add_dependencies(drone_msgs_genpy drone_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(drone_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(drone_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(drone_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(drone_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(drone_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(drone_msgs_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(drone_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(drone_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(drone_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(drone_msgs_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(drone_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(drone_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(drone_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(drone_msgs_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(drone_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(drone_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(drone_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(drone_msgs_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(drone_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(drone_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
