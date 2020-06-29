# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tracking_msgs: 6 messages, 0 services")

set(MSG_I_FLAGS "-Itracking_msgs:/home/jing/combine_ws/src/tracking_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tracking_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg" NAME_WE)
add_custom_target(_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tracking_msgs" "/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg" ""
)

get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg" NAME_WE)
add_custom_target(_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tracking_msgs" "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg" "std_msgs/ColorRGBA:geometry_msgs/Twist:std_msgs/Header:geometry_msgs/Quaternion:tracking_msgs/Lane:geometry_msgs/Point32:geometry_msgs/Vector3:geometry_msgs/TwistStamped:geometry_msgs/Point:geometry_msgs/PolygonStamped:geometry_msgs/PoseStamped:tracking_msgs/Waypoint:geometry_msgs/Polygon:sensor_msgs/PointField:tracking_msgs/WaypointState:tracking_msgs/LaneArray:geometry_msgs/Pose:tracking_msgs/DTLane:sensor_msgs/PointCloud2"
)

get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg" NAME_WE)
add_custom_target(_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tracking_msgs" "/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg" "geometry_msgs/PoseStamped:geometry_msgs/Twist:std_msgs/Header:geometry_msgs/Quaternion:tracking_msgs/Lane:geometry_msgs/TwistStamped:geometry_msgs/Point:tracking_msgs/DTLane:geometry_msgs/Vector3:tracking_msgs/Waypoint:tracking_msgs/WaypointState:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg" NAME_WE)
add_custom_target(_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tracking_msgs" "/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg" "geometry_msgs/PoseStamped:geometry_msgs/Twist:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/TwistStamped:geometry_msgs/Point:tracking_msgs/DTLane:geometry_msgs/Vector3:tracking_msgs/WaypointState:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg" NAME_WE)
add_custom_target(_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tracking_msgs" "/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg" "geometry_msgs/PoseStamped:geometry_msgs/Twist:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/TwistStamped:geometry_msgs/Point:tracking_msgs/DTLane:geometry_msgs/Vector3:tracking_msgs/Waypoint:tracking_msgs/WaypointState:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObjectArray.msg" NAME_WE)
add_custom_target(_tracking_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tracking_msgs" "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObjectArray.msg" "std_msgs/ColorRGBA:geometry_msgs/Twist:std_msgs/Header:tracking_msgs/DetectedObject:geometry_msgs/Quaternion:tracking_msgs/Lane:geometry_msgs/Point32:geometry_msgs/Point:geometry_msgs/TwistStamped:geometry_msgs/Vector3:geometry_msgs/PolygonStamped:geometry_msgs/PoseStamped:tracking_msgs/Waypoint:geometry_msgs/Polygon:sensor_msgs/PointField:tracking_msgs/WaypointState:tracking_msgs/LaneArray:geometry_msgs/Pose:tracking_msgs/DTLane:sensor_msgs/PointCloud2"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tracking_msgs
)
_generate_msg_cpp(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tracking_msgs
)
_generate_msg_cpp(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tracking_msgs
)
_generate_msg_cpp(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tracking_msgs
)
_generate_msg_cpp(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tracking_msgs
)
_generate_msg_cpp(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObjectArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tracking_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(tracking_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tracking_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tracking_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tracking_msgs_generate_messages tracking_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_cpp _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_cpp _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_cpp _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_cpp _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_cpp _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObjectArray.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_cpp _tracking_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tracking_msgs_gencpp)
add_dependencies(tracking_msgs_gencpp tracking_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tracking_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tracking_msgs
)
_generate_msg_eus(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tracking_msgs
)
_generate_msg_eus(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tracking_msgs
)
_generate_msg_eus(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tracking_msgs
)
_generate_msg_eus(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tracking_msgs
)
_generate_msg_eus(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObjectArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tracking_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(tracking_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tracking_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tracking_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tracking_msgs_generate_messages tracking_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_eus _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_eus _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_eus _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_eus _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_eus _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObjectArray.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_eus _tracking_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tracking_msgs_geneus)
add_dependencies(tracking_msgs_geneus tracking_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tracking_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tracking_msgs
)
_generate_msg_lisp(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tracking_msgs
)
_generate_msg_lisp(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tracking_msgs
)
_generate_msg_lisp(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tracking_msgs
)
_generate_msg_lisp(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tracking_msgs
)
_generate_msg_lisp(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObjectArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tracking_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(tracking_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tracking_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tracking_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tracking_msgs_generate_messages tracking_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_lisp _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_lisp _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_lisp _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_lisp _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_lisp _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObjectArray.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_lisp _tracking_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tracking_msgs_genlisp)
add_dependencies(tracking_msgs_genlisp tracking_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tracking_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tracking_msgs
)
_generate_msg_nodejs(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tracking_msgs
)
_generate_msg_nodejs(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tracking_msgs
)
_generate_msg_nodejs(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tracking_msgs
)
_generate_msg_nodejs(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tracking_msgs
)
_generate_msg_nodejs(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObjectArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tracking_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(tracking_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tracking_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tracking_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tracking_msgs_generate_messages tracking_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_nodejs _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_nodejs _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_nodejs _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_nodejs _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_nodejs _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObjectArray.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_nodejs _tracking_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tracking_msgs_gennodejs)
add_dependencies(tracking_msgs_gennodejs tracking_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tracking_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tracking_msgs
)
_generate_msg_py(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tracking_msgs
)
_generate_msg_py(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tracking_msgs
)
_generate_msg_py(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tracking_msgs
)
_generate_msg_py(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tracking_msgs
)
_generate_msg_py(tracking_msgs
  "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObjectArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg;/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/jing/combine_ws/src/tracking_msgs/msg/DTLane.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tracking_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(tracking_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tracking_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tracking_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tracking_msgs_generate_messages tracking_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/WaypointState.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_py _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObject.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_py _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/LaneArray.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_py _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/Waypoint.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_py _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/Lane.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_py _tracking_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jing/combine_ws/src/tracking_msgs/msg/DetectedObjectArray.msg" NAME_WE)
add_dependencies(tracking_msgs_generate_messages_py _tracking_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tracking_msgs_genpy)
add_dependencies(tracking_msgs_genpy tracking_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tracking_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tracking_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tracking_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tracking_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(tracking_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(tracking_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tracking_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tracking_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tracking_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(tracking_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(tracking_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tracking_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tracking_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tracking_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(tracking_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(tracking_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tracking_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tracking_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tracking_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(tracking_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(tracking_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tracking_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tracking_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tracking_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tracking_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(tracking_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(tracking_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
