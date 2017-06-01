cmake_minimum_required(VERSION 2.8.3)
project(horus_gazebo_plugins)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs dynamic_reconfigure)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


find_package(Boost REQUIRED COMPONENTS system thread)


generate_dynamic_reconfigure_options(
  config/control_dr.cfg
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES joy_ctrl
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
  DEPENDS gazebo
)

#find_package(roscpp REQUIRED)
#find_package(std_msgs REQUIRED)
include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

#find_package(geometry_msgs REQUIRED)
#include_directories(${geometry_msgs_INCLUDE_DIRS})

add_library(horus_gazebo_plugin SHARED src/horus_gazebo_plugin.cc)
add_library(MotorModel SHARED src/motor_model.cc)
add_library(PropulsionModel SHARED src/propulsion_model.cc)

target_link_libraries(horus_gazebo_plugin MotorModel PropulsionModel ${GAZEBO_LIBRARIES}  ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${roscpp_LIBRARIES})

add_dependencies(horus_gazebo_plugin ${PROJECT_NAME}_gencfg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
