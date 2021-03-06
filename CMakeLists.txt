cmake_minimum_required(VERSION 2.8.3)
SET(PROJECTNAME romeo_moveit_actions)
project(${PROJECTNAME})

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  moveit_ros_planning 
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_simple_grasps
  std_msgs
  geometry_msgs
  geometric_shapes
  roscpp
  cmake_modules
  object_recognition_msgs
  actionlib
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs geometric_shapes std_msgs moveit_ros_planning moveit_ros_planning_interface moveit_visual_tools moveit_simple_grasps object_recognition_msgs
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

set(SRC
  src/main.cpp
  src/simplepickplace.cpp
  src/metablock.cpp
  src/action.cpp
  src/postures.cpp
  src/objprocessing.cpp
  src/evaluation.cpp
  src/toolsForObject.cpp
  include/romeo_moveit_actions/simplepickplace.hpp
  include/romeo_moveit_actions/metablock.hpp
  include/romeo_moveit_actions/action.hpp
  include/romeo_moveit_actions/postures.hpp
  include/romeo_moveit_actions/tools.hpp
  include/romeo_moveit_actions/toolsForAction.hpp
  include/romeo_moveit_actions/toolsForObject.hpp
  include/romeo_moveit_actions/evaluation.hpp
  include/romeo_moveit_actions/objprocessing.hpp
)

# Main executable
add_executable(${PROJECTNAME} ${SRC})

target_link_libraries(${PROJECTNAME}
  ${catkin_LIBRARIES}
)
