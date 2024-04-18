cmake_minimum_required(VERSION 3.0.2)
project(brushee_local_map_creator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  # roslinrt
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES brushee_local_map_creator
#  CATKIN_DEPENDS geometry_msgs nav_msgs ros_cpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME} src/brushee_local_map_creator.cpp)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/brushee_local_map_creator_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
#     find_package(rostest REQUIRED)
#     find_package(roslint REQUIRED)
#     roslint_cpp()
#     roslint_add_test()
# endif()