# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robot_move: 7 messages, 0 services")

set(MSG_I_FLAGS "-Irobot_move:/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robot_move_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg" NAME_WE)
add_custom_target(_robot_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_move" "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg" "robot_move/moveFeedback:actionlib_msgs/GoalID:std_msgs/Header:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg" NAME_WE)
add_custom_target(_robot_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_move" "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg" ""
)

get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg" NAME_WE)
add_custom_target(_robot_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_move" "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg" ""
)

get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveAction.msg" NAME_WE)
add_custom_target(_robot_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_move" "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveAction.msg" "actionlib_msgs/GoalStatus:robot_move/moveResult:robot_move/moveFeedback:robot_move/moveActionResult:robot_move/moveActionGoal:robot_move/moveActionFeedback:actionlib_msgs/GoalID:robot_move/moveGoal:std_msgs/Header"
)

get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg" NAME_WE)
add_custom_target(_robot_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_move" "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg" ""
)

get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg" NAME_WE)
add_custom_target(_robot_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_move" "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg" "actionlib_msgs/GoalID:std_msgs/Header:robot_move/moveGoal"
)

get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg" NAME_WE)
add_custom_target(_robot_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_move" "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg" "actionlib_msgs/GoalID:robot_move/moveResult:std_msgs/Header:actionlib_msgs/GoalStatus"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_move
)
_generate_msg_cpp(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_move
)
_generate_msg_cpp(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_move
)
_generate_msg_cpp(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_move
)
_generate_msg_cpp(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_move
)
_generate_msg_cpp(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_move
)
_generate_msg_cpp(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_move
)

### Generating Services

### Generating Module File
_generate_module_cpp(robot_move
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_move
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robot_move_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robot_move_generate_messages robot_move_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_cpp _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_cpp _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_cpp _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveAction.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_cpp _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_cpp _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_cpp _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_cpp _robot_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_move_gencpp)
add_dependencies(robot_move_gencpp robot_move_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_move_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_move
)
_generate_msg_eus(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_move
)
_generate_msg_eus(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_move
)
_generate_msg_eus(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_move
)
_generate_msg_eus(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_move
)
_generate_msg_eus(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_move
)
_generate_msg_eus(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_move
)

### Generating Services

### Generating Module File
_generate_module_eus(robot_move
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_move
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robot_move_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robot_move_generate_messages robot_move_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_eus _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_eus _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_eus _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveAction.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_eus _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_eus _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_eus _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_eus _robot_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_move_geneus)
add_dependencies(robot_move_geneus robot_move_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_move_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_move
)
_generate_msg_lisp(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_move
)
_generate_msg_lisp(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_move
)
_generate_msg_lisp(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_move
)
_generate_msg_lisp(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_move
)
_generate_msg_lisp(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_move
)
_generate_msg_lisp(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_move
)

### Generating Services

### Generating Module File
_generate_module_lisp(robot_move
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_move
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robot_move_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robot_move_generate_messages robot_move_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_lisp _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_lisp _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_lisp _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveAction.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_lisp _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_lisp _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_lisp _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_lisp _robot_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_move_genlisp)
add_dependencies(robot_move_genlisp robot_move_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_move_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_move
)
_generate_msg_nodejs(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_move
)
_generate_msg_nodejs(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_move
)
_generate_msg_nodejs(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_move
)
_generate_msg_nodejs(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_move
)
_generate_msg_nodejs(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_move
)
_generate_msg_nodejs(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_move
)

### Generating Services

### Generating Module File
_generate_module_nodejs(robot_move
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_move
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robot_move_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robot_move_generate_messages robot_move_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_nodejs _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_nodejs _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_nodejs _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveAction.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_nodejs _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_nodejs _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_nodejs _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_nodejs _robot_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_move_gennodejs)
add_dependencies(robot_move_gennodejs robot_move_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_move_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_move
)
_generate_msg_py(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_move
)
_generate_msg_py(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_move
)
_generate_msg_py(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_move
)
_generate_msg_py(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_move
)
_generate_msg_py(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_move
)
_generate_msg_py(robot_move
  "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_move
)

### Generating Services

### Generating Module File
_generate_module_py(robot_move
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_move
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robot_move_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robot_move_generate_messages robot_move_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionFeedback.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_py _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveFeedback.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_py _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveResult.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_py _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveAction.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_py _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveGoal.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_py _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionGoal.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_py _robot_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/catkin_learn/devel_isolated/robot_move/share/robot_move/msg/moveActionResult.msg" NAME_WE)
add_dependencies(robot_move_generate_messages_py _robot_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_move_genpy)
add_dependencies(robot_move_genpy robot_move_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_move_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_move
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(robot_move_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robot_move_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_move
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(robot_move_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robot_move_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_move
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(robot_move_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robot_move_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_move
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(robot_move_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robot_move_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_move)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_move\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_move
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(robot_move_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robot_move_generate_messages_py std_msgs_generate_messages_py)
endif()
